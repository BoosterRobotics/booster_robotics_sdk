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

static std::atomic<bool> g_run{true};

static void sigint_handler(int) {
    g_run = false;
}

enum class State {
    kSearchBall,
    kApproachBall,
    kSearchPerson,
    kAlignBody,
    kKick
};

static constexpr int kPhaseInitDelayMs = 2000;      // conservative discovery delay after each Init()
static constexpr int kPhaseServiceDelayMs = 1500;   // conservative settle time after StartVisionService()
static constexpr int kPhaseModeDelayMs = 1000;      // conservative settle time after ChangeMode()
static constexpr int kStartupPollMs = 120;
static constexpr int kSearchPollMs = 120;
static constexpr int kActivePollMs = 60;
static constexpr int kRpcMaxAttempts = 4;
static constexpr int kRpcBackoffBaseMs = 250;
static constexpr int kPassiveValidationCycles = 5;
static constexpr int kPassiveValidationFailureLimit = 2;
static constexpr int kPassiveValidationRpcAttempts = 2;
static constexpr int kActivePollFailureLimit = 2;
static constexpr int kTargetLossLimit = 3;
static constexpr int kSweepStepsHalf = 20;
static constexpr int kKickFrames = 20;
static constexpr int kKickPublishIntervalMs = 33;   // ~30 Hz kick reference stream
static constexpr int kSearchBodyRotateMs = 300;

static constexpr float kBallReadyDistMin = 0.22f;
static constexpr float kBallReadyDistMax = 0.45f;
static constexpr float kBallAlignY = 0.05f;
static constexpr float kAlignYawThresh = 0.35f;
static constexpr float kMinPersonFwdDist = 0.10f;
static constexpr float kPowerMin = 0.30f;
static constexpr float kPowerMax = 0.90f;
static constexpr float kPowerDist = 4.0f;
static constexpr float kBallSearchBodyYawRate = 0.25f;
// Person search widens the view a bit faster because the ball is already staged.
static constexpr float kPersonSearchBodyYawRate = 0.30f;
static constexpr float kAlignBodyYawRate = 0.30f;

static bool IsBall(const std::string& tag) {
    return tag == "ball" || tag == "Ball";
}

static bool IsPerson(const std::string& tag) {
    return tag == "face" || tag == "Face" ||
           tag == "person" || tag == "Person";
}

static bool HasRobotFrameXY(const DetectResults& obj) {
    return obj.position_.size() >= 2;
}

static float Dist2D(const std::vector<float>& pos) {
    if (pos.size() < 2) {
        return 9999.0f;
    }
    return std::hypot(pos[0], pos[1]);
}

static void SleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static int PollDelayMsForState(State state) {
    return (state == State::kApproachBall || state == State::kAlignBody)
               ? kActivePollMs
               : kSearchPollMs;
}

// The Booster B1 SDK methods used here return int32_t in this repository. If a
// future SDK changes one of them to void, wrap that call separately.
static int RetryRpc(const std::string& name,
                    const std::function<int(void)>& fn,
                    int max_attempts = kRpcMaxAttempts) {
    int rc = -1;
    for (int attempt = 1; attempt <= max_attempts && g_run.load(); ++attempt) {
        rc = fn();
        if (rc == 0) {
            if (attempt > 1) {
                std::cout << "[rpc] " << name << " recovered on attempt "
                          << attempt << "/" << max_attempts << "\n";
            }
            return 0;
        }

        if (attempt < max_attempts) {
            const int backoff_ms = kRpcBackoffBaseMs * attempt;
            std::cerr << "[rpc] " << name << " failed rc=" << rc
                      << " (attempt " << attempt << "/" << max_attempts
                      << "), retrying in " << backoff_ms << " ms\n";
            SleepMs(backoff_ms);
        }
    }

    std::cerr << "[rpc] " << name << " failed after " << max_attempts
              << " attempts, rc=" << rc << "\n";
    return rc;
}

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

static void StopMotionAndCenterHead(B1LocoClient& loco) {
    (void)loco.Move(0.0f, 0.0f, 0.0f);
    (void)loco.RotateHead(0.0f, 0.0f);
}

static void EnterSafeState(B1LocoClient& loco) {
    std::cerr << "[safe] Stop motion, centre head, and return to walking mode.\n";
    StopMotionAndCenterHead(loco);
    (void)RetryRpc("ChangeMode(kWalking)", [&]() -> int {
        return loco.ChangeMode(booster::robot::RobotMode::kWalking);
    }, 2);
}

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
    person_pos.clear();
    sweep_dir = 1;
    sweep_count = 0;
    ball_lost_count = 0;
    person_lost_count = 0;
    state = State::kSearchBall;
}

static bool PassiveVisionValidation(VisionClient& vision) {
    std::cout << "[vision] Passive validation: polling without motion.\n";
    int failure_count = 0;
    for (int cycle = 0; cycle < kPassiveValidationCycles && g_run.load(); ++cycle) {
        std::vector<DetectResults> objects;
        if (RetryGetDetectionObject(vision, objects, 1.0f, kPassiveValidationRpcAttempts)) {
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

class KickReferencePublisher {
public:
    KickReferencePublisher() : type_(new brain::msg::Kick()) {}

    ~KickReferencePublisher() noexcept {
        try {
            Cleanup();
        } catch (...) {
        }
    }

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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <networkInterface>\n"
                  << "Example: " << argv[0] << " eth0\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    const std::string network_interface = argv[1];

    std::cout << "[init] Phase 1: ChannelFactory::Init\n";
    ChannelFactory::Instance()->Init(0, network_interface);
    SleepMs(kPhaseInitDelayMs);

    B1LocoClient loco;
    std::cout << "[init] Phase 2: B1LocoClient::Init\n";
    loco.Init();
    SleepMs(kPhaseInitDelayMs);

    VisionClient vision;
    std::cout << "[init] Phase 3: VisionClient::Init\n";
    vision.Init();
    SleepMs(kPhaseInitDelayMs);

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

    std::cout << "[init] Phase 5: ChangeMode(kWalking)\n";
    if (RetryRpc("ChangeMode(kWalking)", [&]() -> int {
            return loco.ChangeMode(booster::robot::RobotMode::kWalking);
        }) != 0) {
        EnterSafeState(loco);
        (void)RetryRpc("StopVisionService", [&]() -> int {
            return vision.StopVisionService();
        }, 2);
        return 1;
    }
    SleepMs(kPhaseModeDelayMs);

    std::cout << "[init] Phase 6: passive vision validation\n";
    if (!PassiveVisionValidation(vision)) {
        EnterSafeState(loco);
        (void)RetryRpc("StopVisionService", [&]() -> int {
            return vision.StopVisionService();
        }, 2);
        return 1;
    }

    std::cout << "[init] Phase 7: state machine\n";

    State state = State::kSearchBall;
    std::vector<float> person_pos;
    float ball_x = 0.35f;
    float ball_y = 0.0f;
    int sweep_dir = 1;
    int sweep_count = 0;
    int detection_rpc_failures = 0;
    int ball_lost_count = 0;
    int person_lost_count = 0;

    std::cout << "[state] SEARCH_BALL\n";

    while (g_run.load()) {
        const float ratio = (state == State::kApproachBall) ? 0.33f : 1.0f;
        std::vector<DetectResults> objects;
        if (!RetryGetDetectionObject(vision, objects, ratio)) {
            ++detection_rpc_failures;
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
        detection_rpc_failures = 0;

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

        switch (state) {
        case State::kSearchBall: {
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

            (void)loco.RotateHeadWithDirection(sweep_dir, 0);
            ++sweep_count;
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

            ball_lost_count = 0;
            ball_x = best_ball->position_[0];
            ball_y = best_ball->position_[1];

            const double ex = ball_x - kBallReadyDistMin;
            const double ey = ball_y;
            const double vx = std::clamp(0.8 * ex, -0.25, 0.25);
            const double vy = std::clamp(1.0 * ey, -0.12, 0.12);
            const double wz = std::clamp(1.2 * ey, -0.4, 0.4);
            (void)loco.Move(static_cast<float>(vx),
                            static_cast<float>(vy),
                            static_cast<float>(wz));

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
            if (best_person) {
                person_pos = best_person->position_;
                person_lost_count = 0;
                StopMotionAndCenterHead(loco);
                state = State::kAlignBody;
                std::cout << "[state] Person locked -> ALIGN_BODY\n";
                break;
            }

            (void)loco.RotateHeadWithDirection(sweep_dir, 0);
            ++sweep_count;
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
            if (best_person) {
                person_pos = best_person->position_;
                person_lost_count = 0;
            } else {
                ++person_lost_count;
                if (person_lost_count >= kTargetLossLimit) {
                    RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                        ball_lost_count, person_lost_count,
                                        person_pos,
                                        "person lost while aligning");
                }
                break;
            }

            const float angle = std::atan2(
                person_pos[1],
                std::max(person_pos[0], kMinPersonFwdDist));
            if (std::abs(angle) > kAlignYawThresh) {
                (void)loco.Move(0.0f, 0.0f, std::copysign(kAlignBodyYawRate, angle));
            } else {
                (void)loco.Move(0.0f, 0.0f, 0.0f);
                state = State::kKick;
                std::cout << "[state] Body aligned -> KICK\n";
            }
            break;
        }

        case State::kKick: {
            StopMotionAndCenterHead(loco);
            if (person_pos.size() < 2) {
                RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                    ball_lost_count, person_lost_count,
                                    person_pos,
                                    "kick state reached without a valid person target");
                break;
            }

            const float distance = Dist2D(person_pos);
            const float power = std::clamp(
                kPowerMin + (kPowerMax - kPowerMin) * (distance / kPowerDist),
                kPowerMin,
                kPowerMax);

            if (RetryRpc("ChangeMode(kSoccer)", [&]() -> int {
                    return loco.ChangeMode(booster::robot::RobotMode::kSoccer);
                }) != 0) {
                EnterSafeState(loco);
                g_run = false;
                break;
            }
            SleepMs(kPhaseModeDelayMs);

            if (RetryRpc("VisualKick(true)", [&]() -> int {
                    return loco.VisualKick(true, VisualKickVersion::kV2);
                }) != 0) {
                EnterSafeState(loco);
                g_run = false;
                break;
            }

            const bool published = PublishKickFrames(ball_x, ball_y, person_pos, power);
            (void)RetryRpc("VisualKick(false)", [&]() -> int {
                return loco.VisualKick(false, VisualKickVersion::kV2);
            }, 2);

            if (!published) {
                std::cerr << "[kick] Failed to publish kick reference frames.\n";
            } else {
                std::cout << "[kick] Pass complete.\n";
            }

            EnterSafeState(loco);
            g_run = false;
            break;
        }
        }

        SleepMs(PollDelayMsForState(state));
    }

    EnterSafeState(loco);
    (void)RetryRpc("StopVisionService", [&]() -> int {
        return vision.StopVisionService();
    }, 2);
    return 0;
}
