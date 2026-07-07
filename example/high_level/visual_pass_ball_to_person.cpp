#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <booster/idl/b1/Kick.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/vision/vision_client.hpp>

using namespace eprosima::fastdds::dds;
using booster::robot::ChannelFactory;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;
using booster::robot::vision::DetectResults;
using booster::robot::vision::VisionClient;

// -----------------------------------------------------------------------------
// Global run flag
// -----------------------------------------------------------------------------
// This atomic flag controls the lifetime of the main loop.
// The program runs while g_run == true.
// Pressing Ctrl+C triggers the signal handler below, which sets g_run to false.
// Because it is atomic, the flag is safe to read from the loop and write from
// the signal path without introducing normal shared-state race issues.
static std::atomic<bool> g_run{true};

// -----------------------------------------------------------------------------
// Signal handler
// -----------------------------------------------------------------------------
// The only purpose of this handler is to request a clean shutdown.
// We do not perform heavy cleanup work directly inside the signal handler.
// Instead, we flip the global run flag, and the main control loop exits on the
// next iteration. Cleanup then happens in normal program flow.
static void sigint_handler(int) {
    g_run = false;
}

// -----------------------------------------------------------------------------
// State machine definition
// -----------------------------------------------------------------------------
// The robot behavior is intentionally modeled as a finite state machine.
// Each state has a single responsibility:
//
// kSearchBall   : look around until a ball is detected
// kApproachBall : walk toward the ball until it is in a kickable position
// kSearchPerson : once the ball is ready, search for a person target
// kAlignBody    : rotate the robot so the person is approximately in front
// kKick         : switch to soccer mode, trigger the kick, publish kick frames
//
// This keeps behavior readable and makes recovery easier because each state can
// decide what "failure" means locally and how to transition back safely.
enum class State {
    kSearchBall,
    kApproachBall,
    kSearchPerson,
    kAlignBody,
    kKick
};

// -----------------------------------------------------------------------------
// Timing constants
// -----------------------------------------------------------------------------
// These delays are deliberately conservative.
// The rewrite is trying to avoid early or overly aggressive use of SDK services
// while DDS discovery / internal RPC setup may still be stabilizing.
static constexpr int kPhaseInitDelayMs = 2000;      // Delay after each Init() phase.
static constexpr int kPhaseServiceDelayMs = 1500;   // Delay after StartVisionService().
static constexpr int kPhaseModeDelayMs = 1000;      // Delay after ChangeMode().

static constexpr int kStartupPollMs = 120;          // Passive validation polling delay.
static constexpr int kSearchPollMs = 120;           // Slower search-phase polling.
static constexpr int kActivePollMs = 60;            // Faster active-control polling.

// -----------------------------------------------------------------------------
// Retry / recovery policy constants
// -----------------------------------------------------------------------------
// Retry counts are bounded to avoid hanging forever when a subsystem is down.
// Backoff is linear rather than exponential to keep logic simple and readable.
static constexpr int kRpcMaxAttempts = 4;
static constexpr int kRpcBackoffBaseMs = 250;

// -----------------------------------------------------------------------------
// Vision validation and recovery thresholds
// -----------------------------------------------------------------------------
// Passive validation means "poll vision a few times without moving the robot".
// This acts as a sanity check that the service is actually responsive before we
// begin active state-machine behavior.
static constexpr int kPassiveValidationCycles = 5;
static constexpr int kPassiveValidationFailureLimit = 2;
static constexpr int kPassiveValidationRpcAttempts = 2;

// During active motion / alignment phases, repeated polling failures are taken
// more seriously because stale vision while moving is unsafe or unproductive.
static constexpr int kActivePollFailureLimit = 2;

// If a target disappears for a few consecutive loops, we stop trusting the old
// target and recover to a safe search state.
static constexpr int kTargetLossLimit = 3;

// -----------------------------------------------------------------------------
// Search / kick behavior constants
// -----------------------------------------------------------------------------
static constexpr int kSweepStepsHalf = 20;          // Number of head sweep steps per half-sweep.
static constexpr int kKickFrames = 20;              // Number of kick reference messages to publish.
static constexpr int kKickPublishIntervalMs = 33;   // About 30 Hz reference stream during kick.
static constexpr int kSearchBodyRotateMs = 300;     // Small body rotation burst when head sweep fails.

// -----------------------------------------------------------------------------
// Geometric and behavior thresholds
// -----------------------------------------------------------------------------
// Ball "ready" window: if the ball is in this forward range and nearly centered,
// the robot considers it staged for a kick/pass.
static constexpr float kBallReadyDistMin = 0.22f;
static constexpr float kBallReadyDistMax = 0.45f;
static constexpr float kBallAlignY = 0.05f;

// Person alignment threshold: if the person angle is larger than this, the body
// will keep rotating to bring the target roughly in front.
static constexpr float kAlignYawThresh = 0.35f;

// Used to avoid near-zero denominator when computing atan2-related alignment in
// cases where the target is extremely close to sideways.
static constexpr float kMinPersonFwdDist = 0.10f;

// Kick power mapping: close target -> gentle pass, farther target -> stronger pass.
static constexpr float kPowerMin = 0.30f;
static constexpr float kPowerMax = 0.90f;
static constexpr float kPowerDist = 4.0f;

// Body rotation rates used in different contexts.
static constexpr float kBallSearchBodyYawRate = 0.25f;

// Person search can rotate slightly more aggressively because by that point the
// ball is already positioned and we only need to find the receiving target.
static constexpr float kPersonSearchBodyYawRate = 0.30f;

static constexpr float kAlignBodyYawRate = 0.30f;

// -----------------------------------------------------------------------------
// Tag helpers
// -----------------------------------------------------------------------------
// The vision service may return different capitalization variants depending on
// configuration or SDK version, so we accept both common forms.
static bool IsBall(const std::string& tag) {
    return tag == "ball" || tag == "Ball";
}

static bool IsPerson(const std::string& tag) {
    return tag == "face" || tag == "Face" ||
           tag == "person" || tag == "Person";
}

// -----------------------------------------------------------------------------
// Detection validity helper
// -----------------------------------------------------------------------------
// We only trust detections that contain at least x and y coordinates in the
// robot frame. A detection without usable position is not helpful for control.
static bool HasRobotFrameXY(const DetectResults& obj) {
    return obj.position_.size() >= 2;
}

// -----------------------------------------------------------------------------
// Distance helper
// -----------------------------------------------------------------------------
// Computes 2D distance in the robot frame using x/y only.
// If the vector is malformed, return a large fallback value so the caller will
// effectively interpret it as "far / invalid" rather than crashing.
static float Dist2D(const std::vector<float>& pos) {
    if (pos.size() < 2) {
        return 9999.0f;
    }
    return std::hypot(pos[0], pos[1]);
}

// -----------------------------------------------------------------------------
// Sleep helper
// -----------------------------------------------------------------------------
// Central wrapper for millisecond sleeps. This keeps the code shorter and also
// makes it easier to adjust timing implementation later if needed.
static void SleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// -----------------------------------------------------------------------------
// Poll-rate policy
// -----------------------------------------------------------------------------
// Search states can poll more slowly because the robot is not making fine
// motion-control decisions. Approach and alignment states benefit from a faster
// refresh rate because they are actively steering on target position.
static int PollDelayMsForState(State state) {
    return (state == State::kApproachBall || state == State::kAlignBody)
               ? kActivePollMs
               : kSearchPollMs;
}

// -----------------------------------------------------------------------------
// Generic retry wrapper for RPC-like SDK calls
// -----------------------------------------------------------------------------
// Many SDK methods used here report success with return code 0 and failure with
// a non-zero error code. This helper retries a call a bounded number of times
// with linear backoff.
//
// Why this exists:
// - SDK/service startup may not be fully settled on the first call
// - transient timeouts can happen even when the next attempt succeeds
// - we want caller-side robustness without modifying SDK internals
//
// Important note:
// This helper assumes fn() returns int. If a future SDK version changes one of
// these methods to void, that method should be wrapped separately.
static int RetryRpc(const std::string& name,
                    const std::function<int(void)>& fn,
                    int max_attempts = kRpcMaxAttempts) {
    int rc = -1;

    // Keep trying until we either succeed, exhaust retries, or the program is
    // asked to shut down.
    for (int attempt = 1; attempt <= max_attempts && g_run.load(); ++attempt) {
        rc = fn();

        // Success path: return immediately.
        if (rc == 0) {
            // Optional recovery log helps indicate that the system was unstable
            // briefly but recovered without failing the whole run.
            if (attempt > 1) {
                std::cout << "[rpc] " << name << " recovered on attempt "
                          << attempt << "/" << max_attempts << "\n";
            }
            return 0;
        }

        // Failure path with remaining retries: wait, then try again.
        if (attempt < max_attempts) {
            const int backoff_ms = kRpcBackoffBaseMs * attempt;
            std::cerr << "[rpc] " << name << " failed rc=" << rc
                      << " (attempt " << attempt << "/" << max_attempts
                      << "), retrying in " << backoff_ms << " ms\n";
            SleepMs(backoff_ms);
        }
    }

    // If we get here, all retries failed.
    std::cerr << "[rpc] " << name << " failed after " << max_attempts
              << " attempts, rc=" << rc << "\n";
    return rc;
}

// -----------------------------------------------------------------------------
// Vision polling retry wrapper
// -----------------------------------------------------------------------------
// This is similar to RetryRpc(), but specialized for detection polling.
// The function clears the output vector before each attempt and only returns
// true when the SDK reports success.
//
// The ratio parameter controls how much of the image region is used:
// - search states use 1.0 for wide coverage
// - approach state uses a tighter ratio to reduce confusion / false matches
static bool RetryGetDetectionObject(VisionClient& vision,
                                    std::vector<DetectResults>& objects,
                                    float ratio,
                                    int max_attempts = 2) {
    int rc = -1;

    for (int attempt = 1; attempt <= max_attempts && g_run.load(); ++attempt) {
        objects.clear();
        rc = vision.GetDetectionObject(objects, ratio);

        if (rc == 0) {
            return true;
        }

        if (attempt < max_attempts) {
            const int backoff_ms = kRpcBackoffBaseMs * attempt;
            std::cerr << "[vision] GetDetectionObject failed rc=" << rc
                      << " (attempt " << attempt << "/" << max_attempts
                      << "), retrying in " << backoff_ms << " ms\n";
            SleepMs(backoff_ms);
        }
    }

    std::cerr << "[vision] GetDetectionObject failed after " << max_attempts
              << " attempts, rc=" << rc << "\n";
    return false;
}

// -----------------------------------------------------------------------------
// Motion utility: stop robot and center head
// -----------------------------------------------------------------------------
// This is a very common action in transitions and recovery paths.
// We intentionally ignore return values here because this is a best-effort
// stabilizing action during control transitions.
static void StopMotionAndCenterHead(B1LocoClient& loco) {
    (void)loco.Move(0.0f, 0.0f, 0.0f);
    (void)loco.RotateHead(0.0f, 0.0f);
}

// -----------------------------------------------------------------------------
// Safe-state helper
// -----------------------------------------------------------------------------
// A safe state here means:
// 1. stop walking / rotating
// 2. center the head
// 3. try to return the robot to walking mode
//
// This is used whenever something important fails and we want the robot to end
// up in a predictable, non-aggressive posture.
static void EnterSafeState(B1LocoClient& loco) {
    std::cerr << "[safe] Stop motion, centre head, and return to walking mode.\n";
    StopMotionAndCenterHead(loco);

    // Best-effort restore to walking mode.
    (void)RetryRpc("ChangeMode(kWalking)", [&]() -> int {
        return loco.ChangeMode(booster::robot::RobotMode::kWalking);
    }, 2);
}

// -----------------------------------------------------------------------------
// Recovery helper: return to SEARCH_BALL
// -----------------------------------------------------------------------------
// This is the primary non-fatal recovery path.
// When we lose confidence in the current target or vision stream during an
// active phase, we do not terminate immediately. Instead, we reset the local
// search state and start over from finding the ball again.
//
// This clears the locked person because that target is no longer trusted.
static void RecoverToSearchBall(B1LocoClient& loco,
                                State& state,
                                int& sweep_dir,
                                int& sweep_count,
                                int& ball_lost_count,
                                int& person_lost_count,
                                std::vector<float>& person_pos,
                                const std::string& reason) {
    std::cerr << "[recover] " << reason << " -> SEARCH_BALL\n";

    StopMotionAndCenterHead(loco);

    // Clear all local targeting state so the next search is clean.
    person_pos.clear();
    sweep_dir = 1;
    sweep_count = 0;
    ball_lost_count = 0;
    person_lost_count = 0;
    state = State::kSearchBall;
}

// -----------------------------------------------------------------------------
// Passive vision validation
// -----------------------------------------------------------------------------
// After startup but before active motion, we test whether the vision service is
// responsive for several cycles while the robot stays still.
//
// Why this matters:
// - startup success alone does not guarantee stable polling immediately after
// - if repeated polling already fails while standing still, active behavior is
//   likely to be unreliable too
//
// Returns true if validation succeeds, false if repeated failures occur.
static bool PassiveVisionValidation(VisionClient& vision) {
    std::cout << "[vision] Passive validation: polling without motion.\n";

    int failure_count = 0;

    for (int cycle = 0; cycle < kPassiveValidationCycles && g_run.load(); ++cycle) {
        std::vector<DetectResults> objects;

        if (RetryGetDetectionObject(vision, objects, 1.0f, kPassiveValidationRpcAttempts)) {
            // Reset consecutive failure count after any success.
            failure_count = 0;
        } else {
            ++failure_count;

            if (failure_count >= kPassiveValidationFailureLimit) {
                std::cerr << "[vision] Passive validation failed repeatedly.\n";
                return false;
            }
        }

        SleepMs(kStartupPollMs);
    }

    return g_run.load();
}

// -----------------------------------------------------------------------------
// Deferred DDS publisher for kick reference frames
// -----------------------------------------------------------------------------
// This wrapper creates the DDS participant/topic/publisher/writer only when the
// robot is actually ready to kick. That is intentional.
//
// Design reason:
// We want to avoid creating extra DDS entities early in startup because the goal
// of this rewrite is to reduce caller-side interference with SDK discovery,
// readiness, or internal transport timing.
//
// Lifetime model:
// - Create() is called only in the kick path
// - Write() sends one kick-reference sample
// - destructor automatically cleans up DDS resources
class KickReferencePublisher {
public:
    KickReferencePublisher() : type_(new brain::msg::Kick()) {}

    // Destructor is noexcept and swallows exceptions because cleanup should not
    // throw during stack unwinding or shutdown.
    ~KickReferencePublisher() noexcept {
        try {
            Cleanup();
        } catch (...) {
        }
    }

    // Create all DDS entities needed for kick-reference publishing.
    bool Create() {
        DomainParticipantQos participant_qos;
        participant_qos.name("visual_pass_ball_to_person_kick");

        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
        if (!participant_) {
            std::cerr << "[dds] Failed to create kick participant.\n";
            return false;
        }

        type_.register_type(participant_);

        topic_ = participant_->create_topic(
            booster::robot::b1::kTopicKickReference,
            type_.get_type_name(),
            TOPIC_QOS_DEFAULT);
        if (!topic_) {
            std::cerr << "[dds] Failed to create kick topic.\n";
            return false;
        }

        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
        if (!publisher_) {
            std::cerr << "[dds] Failed to create kick publisher.\n";
            return false;
        }

        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, nullptr);
        if (!writer_) {
            std::cerr << "[dds] Failed to create kick writer.\n";
            return false;
        }

        return true;
    }

    // Publish one kick reference message containing:
    // - current ball position
    // - target person position
    // - computed kick power
    bool Write(double ball_x,
               double ball_y,
               double goal_x,
               double goal_y,
               double power) {
        if (!writer_) {
            return false;
        }

        brain::msg::Kick msg;
        msg.x(ball_x);
        msg.y(ball_y);
        msg.goal_x(goal_x);
        msg.goal_y(goal_y);
        msg.power(power);

        return writer_->write(&msg) == ReturnCode_t::RETCODE_OK;
    }

private:
    // Clean up DDS resources in reverse order of creation.
    void Cleanup() {
        if (publisher_ && writer_) {
            publisher_->delete_datawriter(writer_);
            writer_ = nullptr;
        }
        if (participant_ && publisher_) {
            participant_->delete_publisher(publisher_);
            publisher_ = nullptr;
        }
        if (participant_ && topic_) {
            participant_->delete_topic(topic_);
            topic_ = nullptr;
        }
        if (participant_) {
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }
    }

    DomainParticipant* participant_{nullptr};
    Topic* topic_{nullptr};
    Publisher* publisher_{nullptr};
    DataWriter* writer_{nullptr};
    TypeSupport type_;
};

// -----------------------------------------------------------------------------
// Kick-frame publishing helper
// -----------------------------------------------------------------------------
// This function creates the DDS publisher only when needed, publishes a fixed
// burst of reference frames, and then lets the publisher go out of scope.
//
// Returns false if DDS creation or any frame write fails.
static bool PublishKickFrames(float ball_x,
                              float ball_y,
                              const std::vector<float>& person_pos,
                              float power) {
    KickReferencePublisher publisher;

    if (!publisher.Create()) {
        return false;
    }

    for (int frame = 0; frame < kKickFrames && g_run.load(); ++frame) {
        if (!publisher.Write(ball_x, ball_y, person_pos[0], person_pos[1], power)) {
            std::cerr << "[dds] Failed to publish kick frame "
                      << frame << "/" << kKickFrames << ".\n";
            return false;
        }

        SleepMs(kKickPublishIntervalMs);
    }

    return true;
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // Basic CLI validation.
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <networkInterface>\n"
                  << "Example: " << argv[0] << " eth0\n";
        return 1;
    }

    // Register clean-stop signal handler.
    std::signal(SIGINT, sigint_handler);

    const std::string network_interface = argv[1];

    // -------------------------------------------------------------------------
    // Phase 1: initialize communication/channel layer
    // -------------------------------------------------------------------------
    // This must come before higher-level SDK clients that depend on channels.
    std::cout << "[init] Phase 1: ChannelFactory::Init\n";
    ChannelFactory::Instance()->Init(0, network_interface);
    SleepMs(kPhaseInitDelayMs);

    // -------------------------------------------------------------------------
    // Phase 2: initialize locomotion client
    // -------------------------------------------------------------------------
    B1LocoClient loco;
    std::cout << "[init] Phase 2: B1LocoClient::Init\n";
    loco.Init();
    SleepMs(kPhaseInitDelayMs);

    // -------------------------------------------------------------------------
    // Phase 3: initialize vision client
    // -------------------------------------------------------------------------
    VisionClient vision;
    std::cout << "[init] Phase 3: VisionClient::Init\n";
    vision.Init();
    SleepMs(kPhaseInitDelayMs);

    // -------------------------------------------------------------------------
    // Phase 4: start vision service
    // -------------------------------------------------------------------------
    // This is retried because service startup may not be immediately ready even
    // after Init() has returned.
    std::cout << "[init] Phase 4: StartVisionService\n";
    if (RetryRpc("StartVisionService", [&]() -> int {
            return vision.StartVisionService(
                /*enable_position=*/true,
                /*enable_color=*/false,
                /*enable_face_detection=*/true);
        }) != 0) {
        EnterSafeState(loco);
        return 1;
    }
    SleepMs(kPhaseServiceDelayMs);

    // -------------------------------------------------------------------------
    // Phase 5: switch to walking mode
    // -------------------------------------------------------------------------
    // Walking mode is needed for Move() and related commands to be accepted.
    std::cout << "[init] Phase 5: ChangeMode(kWalking)\n";
    if (RetryRpc("ChangeMode(kWalking)", [&]() -> int {
            return loco.ChangeMode(booster::robot::RobotMode::kWalking);
        }) != 0) {
        EnterSafeState(loco);

        // Best-effort vision shutdown because startup partially succeeded.
        (void)RetryRpc("StopVisionService", [&]() -> int {
            return vision.StopVisionService();
        }, 2);

        return 1;
    }
    SleepMs(kPhaseModeDelayMs);

    // -------------------------------------------------------------------------
    // Phase 6: passive vision validation
    // -------------------------------------------------------------------------
    // Do not move yet. First confirm that polling works reliably enough while
    // the robot is idle.
    std::cout << "[init] Phase 6: passive vision validation\n";
    if (!PassiveVisionValidation(vision)) {
        EnterSafeState(loco);
        (void)RetryRpc("StopVisionService", [&]() -> int {
            return vision.StopVisionService();
        }, 2);
        return 1;
    }

    // -------------------------------------------------------------------------
    // Phase 7: enter state machine
    // -------------------------------------------------------------------------
    std::cout << "[init] Phase 7: state machine\n";

    State state = State::kSearchBall;

    // Locked target person position in robot frame.
    std::vector<float> person_pos;

    // Ball position defaults are just placeholders until the first real ball is seen.
    float ball_x = 0.35f;
    float ball_y = 0.0f;

    // Search sweep control.
    int sweep_dir = 1;
    int sweep_count = 0;

    // Failure / target-loss counters.
    int detection_rpc_failures = 0;
    int ball_lost_count = 0;
    int person_lost_count = 0;

    std::cout << "[state] SEARCH_BALL\n";

    // -------------------------------------------------------------------------
    // Main control loop
    // -------------------------------------------------------------------------
    // The program runs until:
    // - Ctrl+C sets g_run to false, or
    // - the KICK path completes and sets g_run to false, or
    // - a fatal failure path decides to terminate.
    while (g_run.load()) {
        // Use a narrower ratio while approaching the ball to reduce ambiguity.
        const float ratio = (state == State::kApproachBall) ? 0.33f : 1.0f;

        std::vector<DetectResults> objects;

        // ---------------------------------------------------------------------
        // Vision polling with retry
        // ---------------------------------------------------------------------
        if (!RetryGetDetectionObject(vision, objects, ratio)) {
            ++detection_rpc_failures;

            // Repeated failure in active states is treated as more serious than
            // failure during passive search because the robot may be moving
            // based on stale or missing information.
            if (state != State::kSearchBall && state != State::kKick &&
                detection_rpc_failures >= kActivePollFailureLimit) {
                RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                    ball_lost_count, person_lost_count,
                                    person_pos,
                                    "repeated detection RPC failure during active state");
            }

            SleepMs(PollDelayMsForState(state));
            continue;
        }

        // Reset consecutive failure count after any successful poll.
        detection_rpc_failures = 0;

        // ---------------------------------------------------------------------
        // Pick best visible ball and person
        // ---------------------------------------------------------------------
        // The logic chooses the highest-confidence valid detection for each type.
        const DetectResults* best_ball = nullptr;
        const DetectResults* best_person = nullptr;

        for (const auto& object : objects) {
            if (IsBall(object.tag_) && HasRobotFrameXY(object)) {
                if (!best_ball || object.conf_ > best_ball->conf_) {
                    best_ball = &object;
                }
            } else if (IsPerson(object.tag_) && HasRobotFrameXY(object)) {
                if (!best_person || object.conf_ > best_person->conf_) {
                    best_person = &object;
                }
            }
        }

        // ---------------------------------------------------------------------
        // State machine
        // ---------------------------------------------------------------------
        switch (state) {

        case State::kSearchBall: {
            // If we see a valid ball, transition immediately to approach mode.
            if (best_ball) {
                ball_x = best_ball->position_[0];
                ball_y = best_ball->position_[1];

                StopMotionAndCenterHead(loco);

                ball_lost_count = 0;
                person_lost_count = 0;
                sweep_dir = 1;
                sweep_count = 0;

                state = State::kApproachBall;
                std::cout << "[state] Ball found -> APPROACH_BALL\n";
                break;
            }

            // Otherwise sweep the head to search visually.
            (void)loco.RotateHeadWithDirection(sweep_dir, 0);
            ++sweep_count;

            // After a full sweep with no ball, rotate the body slightly to widen
            // the search area beyond the current head field of view.
            if (sweep_count >= kSweepStepsHalf * 2) {
                sweep_dir = -sweep_dir;
                sweep_count = 0;

                (void)loco.Move(0.0f, 0.0f,
                                static_cast<float>(sweep_dir) * kBallSearchBodyYawRate);
                SleepMs(kSearchBodyRotateMs);
                (void)loco.Move(0.0f, 0.0f, 0.0f);
            }
            break;
        }

        case State::kApproachBall: {
            // If the ball disappears for a few consecutive cycles, give up on
            // the current approach and recover to a fresh search.
            if (!best_ball) {
                ++ball_lost_count;

                if (ball_lost_count >= kTargetLossLimit) {
                    RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                        ball_lost_count, person_lost_count,
                                        person_pos,
                                        "ball lost while approaching");
                }
                break;
            }

            // Ball is visible again, so reset loss counter and update position.
            ball_lost_count = 0;
            ball_x = best_ball->position_[0];
            ball_y = best_ball->position_[1];

            // Simple proportional controller:
            // - ex drives forward distance toward the ready threshold
            // - ey drives lateral correction
            // - wz yaws to help center the ball
            const double ex = ball_x - kBallReadyDistMin;
            const double ey = ball_y;
            const double vx = std::clamp(0.8 * ex, -0.25, 0.25);
            const double vy = std::clamp(1.0 * ey, -0.12, 0.12);
            const double wz = std::clamp(1.2 * ey, -0.4, 0.4);

            (void)loco.Move(static_cast<float>(vx),
                            static_cast<float>(vy),
                            static_cast<float>(wz));

            // If the ball is inside the kick-ready window and nearly centered,
            // stop and start looking for a receiving person.
            if (ball_x > kBallReadyDistMin && ball_x < kBallReadyDistMax &&
                std::abs(ball_y) < kBallAlignY) {
                StopMotionAndCenterHead(loco);
                person_lost_count = 0;
                sweep_dir = 1;
                sweep_count = 0;
                state = State::kSearchPerson;
                std::cout << "[state] Ball in range -> SEARCH_PERSON\n";
            }
            break;
        }

        case State::kSearchPerson: {
            // First valid detected person becomes the current target.
            if (best_person) {
                person_pos = best_person->position_;
                person_lost_count = 0;

                StopMotionAndCenterHead(loco);

                state = State::kAlignBody;
                std::cout << "[state] Person locked -> ALIGN_BODY\n";
                break;
            }

            // If no person is found yet, sweep with the head.
            (void)loco.RotateHeadWithDirection(sweep_dir, 0);
            ++sweep_count;

            // After a full sweep, rotate the body slightly to bring new sectors
            // into view. This widens the effective search field.
            if (sweep_count >= kSweepStepsHalf * 2) {
                sweep_dir = -sweep_dir;
                sweep_count = 0;

                (void)loco.Move(0.0f, 0.0f,
                                static_cast<float>(sweep_dir) * kPersonSearchBodyYawRate);
                SleepMs(kSearchBodyRotateMs);
                (void)loco.Move(0.0f, 0.0f, 0.0f);
            }
            break;
        }

        case State::kAlignBody: {
            // Keep refreshing the locked person position if visible.
            if (best_person) {
                person_pos = best_person->position_;
                person_lost_count = 0;
            } else {
                // If the person disappears long enough, the target is no longer
                // trustworthy and we recover back to ball search.
                ++person_lost_count;

                if (person_lost_count >= kTargetLossLimit) {
                    RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                        ball_lost_count, person_lost_count,
                                        person_pos,
                                        "person lost while aligning");
                }
                break;
            }

            // Compute the angular offset of the target in robot frame.
            const float angle = std::atan2(
                person_pos[1],
                std::max(person_pos[0], kMinPersonFwdDist));

            // If the target is still significantly off-center, keep rotating.
            if (std::abs(angle) > kAlignYawThresh) {
                (void)loco.Move(0.0f, 0.0f, std::copysign(kAlignBodyYawRate, angle));
            } else {
                // Aligned well enough to attempt the pass.
                (void)loco.Move(0.0f, 0.0f, 0.0f);
                state = State::kKick;
                std::cout << "[state] Body aligned -> KICK\n";
            }
            break;
        }

        case State::kKick: {
            // Before kicking, ensure the robot is stable and the head is centered.
            StopMotionAndCenterHead(loco);

            // Defensive check: we need a valid target position to define goal_x
            // and goal_y in the kick reference message.
            if (person_pos.size() < 2) {
                RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                    ball_lost_count, person_lost_count,
                                    person_pos,
                                    "kick state reached without a valid person target");
                break;
            }

            // Convert target distance into a bounded kick power.
            const float distance = Dist2D(person_pos);
            const float power = std::clamp(
                kPowerMin + (kPowerMax - kPowerMin) * (distance / kPowerDist),
                kPowerMin,
                kPowerMax);

            // Kick requires soccer mode.
            if (RetryRpc("ChangeMode(kSoccer)", [&]() -> int {
                    return loco.ChangeMode(booster::robot::RobotMode::kSoccer);
                }) != 0) {
                EnterSafeState(loco);
                g_run = false;
                break;
            }
            SleepMs(kPhaseModeDelayMs);

            // Start visual kick.
            if (RetryRpc("VisualKick(true)", [&]() -> int {
                    return loco.VisualKick(true, VisualKickVersion::kV2);
                }) != 0) {
                EnterSafeState(loco);
                g_run = false;
                break;
            }

            // Publish kick reference frames while the kick is active.
            const bool published = PublishKickFrames(ball_x, ball_y, person_pos, power);

            // Best-effort stop call, even if publishing failed.
            (void)RetryRpc("VisualKick(false)", [&]() -> int {
                return loco.VisualKick(false, VisualKickVersion::kV2);
            }, 2);

            if (!published) {
                std::cerr << "[kick] Failed to publish kick reference frames.\n";
            } else {
                std::cout << "[kick] Pass complete.\n";
            }

            // End in a safe posture and terminate this single-pass run.
            EnterSafeState(loco);
            g_run = false;
            break;
        }
        }

        // Small pacing delay based on current state activity level.
        SleepMs(PollDelayMsForState(state));
    }

    // -------------------------------------------------------------------------
    // Final cleanup
    // -------------------------------------------------------------------------
    // Even after the loop exits, we explicitly return to a safe state and then
    // stop the vision service with bounded retries.
    EnterSafeState(loco);
    (void)RetryRpc("StopVisionService", [&]() -> int {
        return vision.StopVisionService();
    }, 2);

    return 0;
}
