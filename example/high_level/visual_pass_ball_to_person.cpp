/**
 * visual_pass_ball_to_person.cpp
 *
 * Autonomous "find ball → find person → pass" behavior using the Booster B1
 * SDK VisionClient (Path 1 – on-board detection, no external camera calibration
 * or ONNX model required).
 *
 * State machine
 * ─────────────
 *   SEARCH_BALL   – Robot stands still and sweeps its head left/right with
 *                   RotateHeadWithDirection().  When the on-board vision
 *                   service reports a ball the robot centres its head and
 *                   transitions to APPROACH_BALL.  After a full head sweep
 *                   with no result the body slowly rotates to widen the search.
 *
 *   APPROACH_BALL – Robot walks toward the ball (proportional velocity
 *                   controller) until the ball is within kicking distance
 *                   (~0.22–0.45 m forward, <0.05 m lateral).  If the ball is
 *                   lost the robot returns to SEARCH_BALL.
 *
 *   SEARCH_PERSON – Robot stops, centres its head, then sweeps again looking
 *                   for a face/person.  The *first* person detected during the
 *                   sweep is locked in – subsequent people are ignored.  If no
 *                   person is found after a full head sweep the body rotates
 *                   slightly to widen the field of view.
 *
 *   ALIGN_BODY    – If the locked-in person is significantly to the side
 *                   (|angle| > kAlignYawThresh) the robot yaws its body toward
 *                   the person so the kick direction is roughly correct.
 *
 *   KICK          – Switches to kSoccer mode, triggers VisualKick(kV2), and
 *                   publishes the kick-reference message with goal coordinates
 *                   set to the person's robot-frame position.  Kick power is
 *                   scaled linearly with the person's distance (clipped to
 *                   [kPowerMin, kPowerMax]).
 *
 * Usage:
 *   ./visual_pass_ball_to_person  <networkInterface>
 * Example:
 *   ./visual_pass_ball_to_person  eth0
 */

/**
*START program

*Initialize communication, locomotion, and vision
*Start vision service
*Set robot mode to WALKING

*state = SEARCH_BALL
*person_locked = false
*run = true

*WHILE run = true

*    Get current detections from vision
*    Find best ball
*    Find best person

*    IF state = SEARCH_BALL THEN
*        IF ball is detected THEN
*            save ball position
*            center head
*            state = APPROACH_BALL
*        ELSE
*            sweep head left/right
*            IF one full sweep is completed THEN
*                rotate body slightly
*            ENDIF
*        ENDIF
*    ENDIF

*    IF state = APPROACH_BALL THEN
*        IF ball is not detected THEN
*            stop walking
*            state = SEARCH_BALL
*        ELSE
*            update ball position
*            walk toward ball

*            IF ball is within kicking range AND laterally aligned THEN
*                stop walking
*                center head
*                state = SEARCH_PERSON
*            ENDIF
*        ENDIF
*    ENDIF

*    IF state = SEARCH_PERSON THEN
*        IF person is detected THEN
*            lock this person position
*            center head
*            state = ALIGN_BODY
*        ELSE
*            sweep head left/right
*            IF one full sweep is completed THEN
*                rotate body slightly
*            ENDIF
*        ENDIF
*    ENDIF

*    IF state = ALIGN_BODY THEN
*        compute angle to locked person

*        IF person angle is large THEN
*            rotate body toward person
*            refresh person position if seen again
*        ELSE
*            stop moving
*            state = KICK
*        ENDIF
*    ENDIF

*    IF state = KICK THEN
*        stop moving
*        compute kick power from person distance
*        switch robot to SOCCER mode

*        IF mode switch fails THEN
*            run = false
*        ELSE
*            start visual kick
*            IF kick starts successfully THEN
*                publish kick reference for fixed number of frames
*                stop visual kick
*            ENDIF
*            run = false
*        ENDIF
*    ENDIF

*    wait 40 ms

*END WHILE

*Stop robot
*Stop vision service
*Clean up and exit

*END program
**/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/vision/vision_client.hpp>
#include <booster/idl/b1/Kick.h>

using namespace eprosima::fastdds::dds;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;
using booster::robot::ChannelFactory;
using booster::robot::vision::VisionClient;
using booster::robot::vision::DetectResults;

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------
static std::atomic<bool> g_run{true};
void sigint_handler(int) { g_run = false; }

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

// Approach: stop walking when the ball is within this forward window (metres).
static constexpr float kBallReadyDistMin = 0.22f;
static constexpr float kBallReadyDistMax = 0.45f;
// Maximum lateral offset at which the robot considers itself aligned to the ball.
static constexpr float kBallAlignY       = 0.05f;

// Head sweep: number of RotateHeadWithDirection steps in each direction before
// reversing.  Each step is separated by a 40 ms detection poll, so
// kSweepStepsHalf * 40 ms = time to sweep from centre to one extreme.
static constexpr int kSweepStepsHalf = 20;

// Body yaw alignment: rotate the body if the person is farther than this angle
// away from straight-ahead (radians).
static constexpr float kAlignYawThresh = 0.35f;   // ~20 deg

// Kick power: linearly mapped from kPowerMin (person very close) to kPowerMax
// (person at kPowerDist metres away).
static constexpr float kPowerMin  = 0.30f;
static constexpr float kPowerMax  = 0.90f;
static constexpr float kPowerDist = 4.0f;   // full power at this distance (m)

// Number of Kick reference frames to publish (~660 ms at 33 ms each).
static constexpr int kKickFrames = 20;

// Seconds to wait after Init() before sending RPC requests so the robot's
// services have time to become ready.
static constexpr int kRpcReadyDelaySecs = 2;

// Minimum forward distance used in the atan2 denominator when computing the
// angle to the person.  Prevents near-zero division when the person is almost
// directly to the side.
static constexpr float kMinPersonFwdDist = 0.1f;

// ---------------------------------------------------------------------------
// Detection helpers
// ---------------------------------------------------------------------------

/** True when the detected object tag represents a ball. */
static bool IsBall(const std::string &tag) {
    return tag == "ball" || tag == "Ball";
}

/** True when the detected object tag represents a person or face. */
static bool IsPerson(const std::string &tag) {
    return tag == "face"   || tag == "Face"   ||
           tag == "person" || tag == "Person";
}

/** 2-D Euclidean distance in the robot XY plane (metres). */
static float Dist2D(const std::vector<float> &pos) {
    if (pos.size() < 2) return 9999.f;
    return std::hypot(pos[0], pos[1]);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <networkInterface>\n"
                  << "Example: " << argv[0] << " eth0\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    const std::string net_if = argv[1];

    // -----------------------------------------------------------------------
    // FastDDS publisher – kick reference topic
    // -----------------------------------------------------------------------
    DomainParticipantQos pqos;
    pqos.name("visual_pass_ball_participant");
    auto *participant =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (!participant) {
        std::cerr << "[dds] Failed to create participant\n";
        return 1;
    }

    TypeSupport kick_type(new brain::msg::Kick());
    kick_type.register_type(participant);

    Topic *topic = participant->create_topic(
        booster::robot::b1::kTopicKickReference,
        kick_type.get_type_name(), TOPIC_QOS_DEFAULT);
    Publisher  *pub    = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    DataWriter *writer = pub ? pub->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr)
                             : nullptr;
    if (!topic || !pub || !writer) {
        std::cerr << "[dds] Failed to create kick publisher\n";
        return 1;
    }

    // -----------------------------------------------------------------------
    // Channel factory, locomotion client, and vision client
    // -----------------------------------------------------------------------
    ChannelFactory::Instance()->Init(0, net_if);

    B1LocoClient loco;
    loco.Init();

    VisionClient vc;
    vc.Init();

    // Allow RPC services to become ready
    std::this_thread::sleep_for(std::chrono::seconds(kRpcReadyDelaySecs));

    // Start the on-board vision service with position estimation and face
    // detection both enabled.  The service processes the head camera image
    // internally; GetDetectionObject() retrieves the latest results.
    int32_t vs_ret = vc.StartVisionService(/*enable_position=*/true,
                                           /*enable_color=*/false,
                                           /*enable_face_detection=*/true);
    if (vs_ret != 0) {
        std::cerr << "[vision] StartVisionService failed: " << vs_ret << "\n";
        return 1;
    }
    std::cout << "[vision] Vision service started (position + face detection).\n";

    // Enter walking mode so Move() and RotateHead() are accepted.
    loco.ChangeMode(booster::robot::RobotMode::kWalking);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    enum class State {
        kSearchBall,
        kApproachBall,
        kSearchPerson,
        kAlignBody,
        kKick
    };
    State state = State::kSearchBall;

    // Locked-in person position (robot frame, metres {x, y, z}).
    // Set once during kSearchPerson and never overwritten.
    std::vector<float> person_pos;

    // Last known ball position in robot frame (metres).
    float ball_x = 0.35f;
    float ball_y = 0.0f;

    // Head sweep pitch direction: +1 = left, -1 = right
    // (matches the pitch_direction convention of RotateHeadWithDirection).
    int sweep_dir   =  1;
    int sweep_count =  0;

    std::cout << "[state] SEARCH_BALL – sweeping head to find the ball …\n";

    while (g_run.load()) {
        // ---- Poll detections -----------------------------------------------
        std::vector<DetectResults> objs;
        // Use the full image width during search phases so nothing is missed;
        // tighten the ratio when approaching the ball to reduce false matches.
        const float ratio = (state == State::kApproachBall) ? 0.33f : 1.0f;
        vc.GetDetectionObject(objs, ratio);

        // Find the highest-confidence ball and person in this frame.
        const DetectResults *best_ball   = nullptr;
        const DetectResults *best_person = nullptr;
        for (const auto &o : objs) {
            if (IsBall(o.tag_)) {
                if (!best_ball || o.conf_ > best_ball->conf_)
                    best_ball = &o;
            } else if (IsPerson(o.tag_)) {
                if (!best_person || o.conf_ > best_person->conf_)
                    best_person = &o;
            }
        }

        // ---- State transitions ---------------------------------------------
        switch (state) {

        // --------------------------------------------------------------------
        case State::kSearchBall: {
            if (best_ball && !best_ball->position_.empty()) {
                ball_x = best_ball->position_[0];
                ball_y = (best_ball->position_.size() > 1)
                             ? best_ball->position_[1] : 0.f;
                std::cout << "[state] Ball found at ("
                          << ball_x << ", " << ball_y << ") m → APPROACH_BALL\n";
                // Centre the head before walking so the gait is stable.
                loco.RotateHead(0.f, 0.f);
                sweep_dir = 1; sweep_count = 0;
                state = State::kApproachBall;
                break;
            }
            // No ball yet – pan the head.
            loco.RotateHeadWithDirection(sweep_dir, /*yaw_dir=*/0);
            ++sweep_count;
            if (sweep_count >= kSweepStepsHalf * 2) {
                // Reached the opposite limit; reverse the sweep direction.
                sweep_dir   = -sweep_dir;
                sweep_count = 0;
                // After a complete head sweep with no result, rotate the body
                // slightly in the same direction to widen the field of view.
                loco.Move(0.f, 0.f, static_cast<float>(sweep_dir) * 0.25f);
                std::this_thread::sleep_for(std::chrono::milliseconds(400));
                loco.Move(0.f, 0.f, 0.f);
            }
            break;
        }

        // --------------------------------------------------------------------
        case State::kApproachBall: {
            if (best_ball && !best_ball->position_.empty()) {
                ball_x = best_ball->position_[0];
                ball_y = (best_ball->position_.size() > 1)
                             ? best_ball->position_[1] : 0.f;
            } else {
                // Ball is no longer visible – stop and re-search.
                std::cout << "[state] Ball lost → SEARCH_BALL\n";
                loco.Move(0.f, 0.f, 0.f);
                sweep_dir = 1; sweep_count = 0;
                state = State::kSearchBall;
                break;
            }

            // Proportional velocity controller: drive toward (kBallReadyDist, 0).
            const double ex = ball_x - kBallReadyDistMin;
            const double ey = ball_y;
            const double vx = std::clamp(0.8  * ex, -0.25, 0.25);
            const double vy = std::clamp(1.0  * ey, -0.12, 0.12);
            const double wz = std::clamp(1.2  * ey, -0.4,  0.4);
            loco.Move(static_cast<float>(vx),
                      static_cast<float>(vy),
                      static_cast<float>(wz));

            if (ball_x > kBallReadyDistMin && ball_x < kBallReadyDistMax &&
                std::abs(ball_y) < kBallAlignY) {
                loco.Move(0.f, 0.f, 0.f);
                std::cout << "[state] Ball in range at ("
                          << ball_x << ", " << ball_y << ") m → SEARCH_PERSON\n";
                loco.RotateHead(0.f, 0.f);
                sweep_dir = 1; sweep_count = 0;
                state = State::kSearchPerson;
            }
            break;
        }

        // --------------------------------------------------------------------
        case State::kSearchPerson: {
            if (best_person && !best_person->position_.empty()) {
                // Lock in the first person detected during this sweep.
                person_pos = best_person->position_;
                std::cout << "[state] Person locked at ("
                          << person_pos[0] << ", " << person_pos[1] << ") m"
                          << " → ALIGN_BODY\n";
                loco.RotateHead(0.f, 0.f);
                state = State::kAlignBody;
                break;
            }
            // Pan the head to search.
            loco.RotateHeadWithDirection(sweep_dir, /*yaw_dir=*/0);
            ++sweep_count;
            if (sweep_count >= kSweepStepsHalf * 2) {
                sweep_dir   = -sweep_dir;
                sweep_count = 0;
                // No person found after a full head sweep; rotate the body to
                // bring a new sector into view.
                loco.Move(0.f, 0.f, static_cast<float>(sweep_dir) * 0.3f);
                std::this_thread::sleep_for(std::chrono::milliseconds(350));
                loco.Move(0.f, 0.f, 0.f);
            }
            break;
        }

        // --------------------------------------------------------------------
        case State::kAlignBody: {
            if (person_pos.size() < 2) {
                // Degenerate case – skip alignment and kick anyway.
                state = State::kKick;
                break;
            }
            // Compute the angle from the robot's forward axis to the person.
            const float person_angle =
                std::atan2(person_pos[1], std::max(person_pos[0], kMinPersonFwdDist));

            if (std::abs(person_angle) > kAlignYawThresh) {
                // Rotate the body toward the person.
                const float wz = std::copysign(0.3f, person_angle);
                loco.Move(0.f, 0.f, wz);
                std::this_thread::sleep_for(std::chrono::milliseconds(40));

                // Refresh the person position after each body rotation step.
                std::vector<DetectResults> align_objs;
                vc.GetDetectionObject(align_objs, 1.0f);
                for (const auto &o : align_objs) {
                    if (IsPerson(o.tag_) && !o.position_.empty()) {
                        person_pos = o.position_;
                        break;
                    }
                }
            } else {
                loco.Move(0.f, 0.f, 0.f);
                std::cout << "[state] Body aligned to person → KICK\n";
                state = State::kKick;
            }
            break;
        }

        // --------------------------------------------------------------------
        case State::kKick: {
            loco.Move(0.f, 0.f, 0.f);

            // Kick power scales linearly with the person's distance so a nearby
            // person receives a gentle pass and a distant one a full kick.
            const float dist  = Dist2D(person_pos);
            const float power = std::clamp(
                kPowerMin + (kPowerMax - kPowerMin) * (dist / kPowerDist),
                kPowerMin, kPowerMax);

            std::cout << "[kick] Target person at ("
                      << person_pos[0] << ", " << person_pos[1] << ") m"
                      << "  dist=" << dist << " m  power=" << power << "\n";

            // Switch to kSoccer mode – required for VisualKick.
            int rc = loco.ChangeMode(booster::robot::RobotMode::kSoccer);
            if (rc != 0) {
                std::cerr << "[kick] ChangeMode(kSoccer) failed: " << rc << "\n";
                g_run = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // Start the visual kick.
            int ret = loco.VisualKick(true, VisualKickVersion::kV2);
            if (ret == 0) {
                // Publish kick reference frames.  goal_x/goal_y are the
                // person's robot-frame coordinates so the robot directs the
                // ball toward them.
                for (int i = 0; i < kKickFrames && g_run.load(); ++i) {
                    brain::msg::Kick msg;
                    msg.x(static_cast<double>(ball_x));
                    msg.y(static_cast<double>(ball_y));
                    msg.goal_x(static_cast<double>(person_pos[0]));
                    msg.goal_y(static_cast<double>(person_pos[1]));
                    msg.power(static_cast<double>(power));
                    writer->write(&msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
                loco.VisualKick(false, VisualKickVersion::kV2);
                std::cout << "[kick] Pass complete.\n";
            } else {
                std::cerr << "[kick] VisualKick(start) failed: " << ret << "\n";
            }

            g_run = false;  // one pass per run – exit cleanly
            break;
        }

        } // switch (state)

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    // -----------------------------------------------------------------------
    // Cleanup
    // -----------------------------------------------------------------------
    loco.Move(0.f, 0.f, 0.f);
    vc.StopVisionService();

    pub->delete_datawriter(writer);
    participant->delete_publisher(pub);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);
    return 0;
}
