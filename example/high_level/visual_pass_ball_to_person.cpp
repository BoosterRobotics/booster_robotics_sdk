#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
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

// ============================================================================
// Diagnostic configuration
// ============================================================================
// This file is intentionally instrumented for diagnosis, not optimization.
// The goal is to reveal exactly:
//   - which SDK call blocks or fails,
//   - how long it takes,
//   - which attempt fails,
//   - which startup phase was active,
//   - and which state the robot was in when the failure happened.
static constexpr bool kDebugVerbose = true;

// Heartbeat frequency for the main loop. Every N iterations a compact status
// summary is printed so we can see whether the program is making forward
// progress or stalling in one region.
static constexpr int kHeartbeatEveryLoops = 10;

// Motion logging can get noisy. These thresholds suppress logs when the command
// arguments are effectively unchanged.
static constexpr float kMoveLogThreshold = 0.01f;
static constexpr float kHeadLogThreshold = 0.01f;

// ============================================================================
// Global runtime state
// ============================================================================
static std::atomic<bool> g_run{true};
static const auto g_program_start = std::chrono::steady_clock::now();

// We keep a few global diagnostic breadcrumbs so that on exit we can summarize
// the last successful step, last failed call, total retries, etc.
static std::string g_last_successful_phase = "<none>";
static std::string g_last_successful_rpc = "<none>";
static std::string g_last_failed_rpc = "<none>";
static int g_total_retry_attempts = 0;
static bool g_vision_started_successfully = false;
static bool g_walking_mode_succeeded = false;
static bool g_any_detection_poll_succeeded = false;
static bool g_reached_kick_state = false;

// ============================================================================
// Logging framework
// ============================================================================
// We use a tiny in-file logger instead of an external logging library to keep
// this file portable and easy to drop into your workspace.
//
// Each log line includes:
//   - elapsed milliseconds since program start,
//   - log level,
//   - optional context,
//   - the message.
//
// Example:
//   [1234 ms] [INFO ] [phase] PHASE 4 START: StartVisionService
enum class LogLevel {
    kDebug,
    kInfo,
    kWarn,
    kError
};

static const char* ToString(LogLevel level) {
    switch (level) {
        case LogLevel::kDebug: return "DEBUG";
        case LogLevel::kInfo:  return "INFO ";
        case LogLevel::kWarn:  return "WARN ";
        case LogLevel::kError: return "ERROR";
        default:               return "UNKWN";
    }
}

static bool ShouldLog(LogLevel level) {
    if (kDebugVerbose) {
        return true;
    }
    return level != LogLevel::kDebug;
}

static long long ElapsedMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - g_program_start).count();
}

static void Log(LogLevel level,
                const std::string& message,
                const std::string& context = "") {
    if (!ShouldLog(level)) {
        return;
    }

    std::ostream& os = (level == LogLevel::kError || level == LogLevel::kWarn)
                           ? std::cerr
                           : std::cout;

    os << "[" << ElapsedMs() << " ms]"
       << " [" << ToString(level) << "]";

    if (!context.empty()) {
        os << " [" << context << "]";
    }

    os << " " << message << "\n";
}

// ============================================================================
// State machine definition
// ============================================================================
enum class State {
    kSearchBall,
    kApproachBall,
    kSearchPerson,
    kAlignBody,
    kKick
};

static const char* StateName(State s) {
    switch (s) {
        case State::kSearchBall:   return "SEARCH_BALL";
        case State::kApproachBall: return "APPROACH_BALL";
        case State::kSearchPerson: return "SEARCH_PERSON";
        case State::kAlignBody:    return "ALIGN_BODY";
        case State::kKick:         return "KICK";
        default:                   return "UNKNOWN_STATE";
    }
}

static void LogStateTransition(State from, State to, const std::string& reason) {
    std::ostringstream oss;
    oss << StateName(from) << " -> " << StateName(to)
        << " | reason=" << reason;
    Log(LogLevel::kInfo, oss.str(), "state");
}

// ============================================================================
// Signal handling
// ============================================================================
static void sigint_handler(int) {
    Log(LogLevel::kWarn, "SIGINT received; requesting clean shutdown.", "signal");
    g_run = false;
}

// ============================================================================
// Timing / behavior constants
// ============================================================================
static constexpr int kPhaseInitDelayMs = 2000;
static constexpr int kPhaseServiceDelayMs = 1500;
static constexpr int kPhaseModeDelayMs = 1000;
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
static constexpr int kKickPublishIntervalMs = 33;
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
static constexpr float kPersonSearchBodyYawRate = 0.30f;
static constexpr float kAlignBodyYawRate = 0.30f;

// ============================================================================
// Formatting helpers
// ============================================================================
// These helpers make log output readable and safe even when a vector is empty
// or malformed.
static std::string FormatXY(float x, float y) {
    std::ostringstream oss;
    oss << "(" << std::fixed << std::setprecision(3) << x << ", " << y << ")";
    return oss.str();
}

static std::string FormatVecXY(const std::vector<float>& v) {
    if (v.empty()) {
        return "<empty>";
    }
    if (v.size() < 2) {
        return "<invalid:size=" + std::to_string(v.size()) + ">";
    }
    return FormatXY(v[0], v[1]);
}

static std::string FormatBool(bool value) {
    return value ? "true" : "false";
}

// ============================================================================
// Tag / geometry helpers
// ============================================================================
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

// ============================================================================
// Scoped timer for phase / scope duration logs
// ============================================================================
// This is useful for phase markers such as:
//   PHASE 4 START
//   PHASE 4 END duration_ms=...
class ScopedPhaseTimer {
public:
    ScopedPhaseTimer(std::string name, std::string context = "phase")
        : name_(std::move(name)),
          context_(std::move(context)),
          start_(std::chrono::steady_clock::now()) {
        Log(LogLevel::kInfo, name_ + " START", context_);
    }

    ~ScopedPhaseTimer() {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_).count();
        Log(LogLevel::kInfo,
            name_ + " END duration_ms=" + std::to_string(elapsed),
            context_);
    }

private:
    std::string name_;
    std::string context_;
    std::chrono::steady_clock::time_point start_;
};

// ============================================================================
// Retry wrapper for SDK calls returning int
// ============================================================================
// This helper logs:
//   - ENTER of each attempt
//   - per-attempt duration
//   - rc value
//   - retry sleep
//   - final success/failure
//
// It is one of the most important diagnostic pieces, because if the timeout is
// thrown from an SDK call we want to know:
//   - which call,
//   - which attempt,
//   - and how long it spent before returning.
static int RetryRpc(const std::string& name,
                    const std::function<int(void)>& fn,
                    int max_attempts = kRpcMaxAttempts) {
    const auto total_start = std::chrono::steady_clock::now();
    int rc = -1;

    Log(LogLevel::kDebug,
        "ENTER retry wrapper for " + name + " max_attempts=" + std::to_string(max_attempts),
        "rpc");

    for (int attempt = 1; attempt <= max_attempts && g_run.load(); ++attempt) {
        ++g_total_retry_attempts;

        const auto attempt_start = std::chrono::steady_clock::now();
        Log(LogLevel::kInfo,
            "ENTER " + name + " attempt=" + std::to_string(attempt) + "/" +
                std::to_string(max_attempts),
            "rpc");

        rc = fn();

        const auto attempt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - attempt_start).count();

        Log(LogLevel::kInfo,
            "EXIT " + name + " attempt=" + std::to_string(attempt) +
                " rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(attempt_ms),
            "rpc");

        if (rc == 0) {
            g_last_successful_rpc = name;

            const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - total_start).count();

            Log(LogLevel::kInfo,
                name + " SUCCESS total_duration_ms=" + std::to_string(total_ms) +
                    " attempts_used=" + std::to_string(attempt),
                "rpc");
            return 0;
        }

        g_last_failed_rpc = name;

        if (attempt < max_attempts) {
            const int backoff_ms = kRpcBackoffBaseMs * attempt;
            Log(LogLevel::kWarn,
                name + " failed rc=" + std::to_string(rc) +
                    " attempt=" + std::to_string(attempt) +
                    " sleeping_backoff_ms=" + std::to_string(backoff_ms),
                "rpc");
            SleepMs(backoff_ms);
        }
    }

    const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - total_start).count();

    Log(LogLevel::kError,
        name + " FINAL FAILURE rc=" + std::to_string(rc) +
            " total_duration_ms=" + std::to_string(total_ms),
        "rpc");
    return rc;
}

// ============================================================================
// Vision polling retry wrapper
// ============================================================================
// This is intentionally separate from RetryRpc because GetDetectionObject()
// writes into an output vector and is called frequently. It still logs detailed
// timing and rc so we can determine whether the timeout happens during polling.
static bool RetryGetDetectionObject(VisionClient& vision,
                                    std::vector<DetectResults>& objects,
                                    float ratio,
                                    int max_attempts = 2) {
    const auto total_start = std::chrono::steady_clock::now();
    int rc = -1;

    Log(LogLevel::kDebug,
        "ENTER retry wrapper for GetDetectionObject ratio=" + std::to_string(ratio) +
            " max_attempts=" + std::to_string(max_attempts),
        "vision");

    for (int attempt = 1; attempt <= max_attempts && g_run.load(); ++attempt) {
        ++g_total_retry_attempts;

        objects.clear();

        const auto attempt_start = std::chrono::steady_clock::now();
        Log(LogLevel::kInfo,
            "ENTER vision.GetDetectionObject attempt=" + std::to_string(attempt) + "/" +
                std::to_string(max_attempts) +
                " ratio=" + std::to_string(ratio),
            "vision");

        rc = vision.GetDetectionObject(objects, ratio);

        const auto attempt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - attempt_start).count();

        Log(LogLevel::kInfo,
            "EXIT vision.GetDetectionObject attempt=" + std::to_string(attempt) +
                " rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(attempt_ms) +
                " objects=" + std::to_string(objects.size()),
            "vision");

        if (rc == 0) {
            g_any_detection_poll_succeeded = true;
            g_last_successful_rpc = "vision.GetDetectionObject";

            const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - total_start).count();

            Log(LogLevel::kDebug,
                "vision.GetDetectionObject SUCCESS total_duration_ms=" + std::to_string(total_ms),
                "vision");
            return true;
        }

        g_last_failed_rpc = "vision.GetDetectionObject";

        if (attempt < max_attempts) {
            const int backoff_ms = kRpcBackoffBaseMs * attempt;
            Log(LogLevel::kWarn,
                "vision.GetDetectionObject failed rc=" + std::to_string(rc) +
                    " attempt=" + std::to_string(attempt) +
                    " sleeping_backoff_ms=" + std::to_string(backoff_ms),
                "vision");
            SleepMs(backoff_ms);
        }
    }

    const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - total_start).count();

    Log(LogLevel::kError,
        "vision.GetDetectionObject FINAL FAILURE rc=" + std::to_string(rc) +
            " total_duration_ms=" + std::to_string(total_ms),
        "vision");
    return false;
}

// ============================================================================
// Motion wrappers with compact logging
// ============================================================================
// We do not want to flood logs with identical Move/RotateHead commands every
// loop, so these wrappers only log when the command changes meaningfully.
struct MotionLogState {
    bool move_valid = false;
    float move_x = 0.0f;
    float move_y = 0.0f;
    float move_yaw = 0.0f;

    bool head_valid = false;
    float head_pitch = 0.0f;
    float head_yaw = 0.0f;

    bool head_sweep_valid = false;
    int head_sweep_pitch_dir = 0;
    int head_sweep_yaw_dir = 0;
};

static MotionLogState g_motion_log_state;

static bool NearlyDifferent(float a, float b, float threshold) {
    return std::abs(a - b) > threshold;
}

static int LoggedMove(B1LocoClient& loco, float x, float y, float yaw, const std::string& why) {
    const bool should_log =
        !g_motion_log_state.move_valid ||
        NearlyDifferent(g_motion_log_state.move_x, x, kMoveLogThreshold) ||
        NearlyDifferent(g_motion_log_state.move_y, y, kMoveLogThreshold) ||
        NearlyDifferent(g_motion_log_state.move_yaw, yaw, kMoveLogThreshold);

    if (should_log) {
        Log(LogLevel::kDebug,
            "ENTER loco.Move x=" + std::to_string(x) +
                " y=" + std::to_string(y) +
                " yaw=" + std::to_string(yaw) +
                " why=" + why,
            "motion");
    }

    const auto start = std::chrono::steady_clock::now();
    int rc = loco.Move(x, y, yaw);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    if (should_log) {
        Log(LogLevel::kDebug,
            "EXIT loco.Move rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(ms),
            "motion");

        g_motion_log_state.move_valid = true;
        g_motion_log_state.move_x = x;
        g_motion_log_state.move_y = y;
        g_motion_log_state.move_yaw = yaw;
    }

    return rc;
}

static int LoggedRotateHead(B1LocoClient& loco, float pitch, float yaw, const std::string& why) {
    const bool should_log =
        !g_motion_log_state.head_valid ||
        NearlyDifferent(g_motion_log_state.head_pitch, pitch, kHeadLogThreshold) ||
        NearlyDifferent(g_motion_log_state.head_yaw, yaw, kHeadLogThreshold);

    if (should_log) {
        Log(LogLevel::kDebug,
            "ENTER loco.RotateHead pitch=" + std::to_string(pitch) +
                " yaw=" + std::to_string(yaw) +
                " why=" + why,
            "motion");
    }

    const auto start = std::chrono::steady_clock::now();
    int rc = loco.RotateHead(pitch, yaw);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    if (should_log) {
        Log(LogLevel::kDebug,
            "EXIT loco.RotateHead rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(ms),
            "motion");

        g_motion_log_state.head_valid = true;
        g_motion_log_state.head_pitch = pitch;
        g_motion_log_state.head_yaw = yaw;
    }

    return rc;
}

static int LoggedRotateHeadWithDirection(B1LocoClient& loco,
                                         int pitch_direction,
                                         int yaw_direction,
                                         const std::string& why) {
    const bool should_log =
        !g_motion_log_state.head_sweep_valid ||
        g_motion_log_state.head_sweep_pitch_dir != pitch_direction ||
        g_motion_log_state.head_sweep_yaw_dir != yaw_direction;

    if (should_log) {
        Log(LogLevel::kDebug,
            "ENTER loco.RotateHeadWithDirection pitch_direction=" +
                std::to_string(pitch_direction) +
                " yaw_direction=" + std::to_string(yaw_direction) +
                " why=" + why,
            "motion");
    }

    const auto start = std::chrono::steady_clock::now();
    int rc = loco.RotateHeadWithDirection(pitch_direction, yaw_direction);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    if (should_log) {
        Log(LogLevel::kDebug,
            "EXIT loco.RotateHeadWithDirection rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(ms),
            "motion");

        g_motion_log_state.head_sweep_valid = true;
        g_motion_log_state.head_sweep_pitch_dir = pitch_direction;
        g_motion_log_state.head_sweep_yaw_dir = yaw_direction;
    }

    return rc;
}

// ============================================================================
// Safety / recovery helpers
// ============================================================================
static void StopMotionAndCenterHead(B1LocoClient& loco) {
    Log(LogLevel::kInfo, "StopMotionAndCenterHead invoked", "safe");
    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop motion");
    (void)LoggedRotateHead(loco, 0.0f, 0.0f, "center head");
}

static void EnterSafeState(B1LocoClient& loco) {
    Log(LogLevel::kWarn,
        "Entering safe state: stop motion, center head, best-effort walking mode.",
        "safe");
    StopMotionAndCenterHead(loco);

    (void)RetryRpc("loco.ChangeMode(kWalking)", [&]() -> int {
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
    Log(LogLevel::kWarn, "RecoverToSearchBall reason=" + reason, "recover");

    const State old_state = state;

    StopMotionAndCenterHead(loco);
    person_pos.clear();
    sweep_dir = 1;
    sweep_count = 0;
    ball_lost_count = 0;
    person_lost_count = 0;
    state = State::kSearchBall;

    LogStateTransition(old_state, state, reason);
}

// ============================================================================
// Passive validation
// ============================================================================
static bool PassiveVisionValidation(VisionClient& vision) {
    ScopedPhaseTimer timer("PHASE 6: passive vision validation");

    Log(LogLevel::kInfo,
        "Polling vision without robot motion to determine whether the service is "
        "already stable before entering the main state machine.",
        "vision");

    int failure_count = 0;

    for (int cycle = 0; cycle < kPassiveValidationCycles && g_run.load(); ++cycle) {
        std::vector<DetectResults> objects;

        Log(LogLevel::kInfo,
            "Passive validation cycle=" + std::to_string(cycle + 1) + "/" +
                std::to_string(kPassiveValidationCycles),
            "vision");

        if (RetryGetDetectionObject(vision, objects, 1.0f, kPassiveValidationRpcAttempts)) {
            failure_count = 0;
            Log(LogLevel::kInfo,
                "Passive validation poll success object_count=" + std::to_string(objects.size()),
                "vision");
        } else {
            ++failure_count;
            Log(LogLevel::kWarn,
                "Passive validation failure_count=" + std::to_string(failure_count),
                "vision");

            if (failure_count >= kPassiveValidationFailureLimit) {
                Log(LogLevel::kError,
                    "Passive validation failed repeatedly; vision service is not "
                    "stable enough to proceed.",
                    "vision");
                return false;
            }
        }

        SleepMs(kStartupPollMs);
    }

    return g_run.load();
}

// ============================================================================
// Deferred DDS publisher for kick reference
// ============================================================================
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
        Log(LogLevel::kInfo, "ENTER KickReferencePublisher::Create", "dds");
        const auto start = std::chrono::steady_clock::now();

        DomainParticipantQos participant_qos;
        participant_qos.name("visual_pass_ball_to_person_kick");

        participant_ =
            DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
        if (!participant_) {
            Log(LogLevel::kError, "Failed to create kick participant.", "dds");
            return false;
        }

        type_.register_type(participant_);

        topic_ = participant_->create_topic(
            booster::robot::b1::kTopicKickReference,
            type_.get_type_name(),
            TOPIC_QOS_DEFAULT);
        if (!topic_) {
            Log(LogLevel::kError, "Failed to create kick topic.", "dds");
            return false;
        }

        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
        if (!publisher_) {
            Log(LogLevel::kError, "Failed to create kick publisher.", "dds");
            return false;
        }

        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, nullptr);
        if (!writer_) {
            Log(LogLevel::kError, "Failed to create kick writer.", "dds");
            return false;
        }

        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        Log(LogLevel::kInfo,
            "EXIT KickReferencePublisher::Create success duration_ms=" + std::to_string(ms),
            "dds");
        return true;
    }

    bool Write(double ball_x, double ball_y, double goal_x, double goal_y, double power) {
        if (!writer_) {
            Log(LogLevel::kError, "Write called without valid writer.", "dds");
            return false;
        }

        brain::msg::Kick msg;
        msg.x(ball_x);
        msg.y(ball_y);
        msg.goal_x(goal_x);
        msg.goal_y(goal_y);
        msg.power(power);

        const auto start = std::chrono::steady_clock::now();
        auto rc = writer_->write(&msg);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        Log(LogLevel::kDebug,
            "Kick frame publish rc=" + std::to_string(static_cast<int>(rc)) +
                " duration_ms=" + std::to_string(ms) +
                " ball=" + FormatXY(static_cast<float>(ball_x), static_cast<float>(ball_y)) +
                " goal=" + FormatXY(static_cast<float>(goal_x), static_cast<float>(goal_y)) +
                " power=" + std::to_string(power),
            "dds");

        return rc == ReturnCode_t::RETCODE_OK;
    }

private:
    void Cleanup() {
        Log(LogLevel::kDebug, "KickReferencePublisher cleanup begin", "dds");

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

        Log(LogLevel::kDebug, "KickReferencePublisher cleanup end", "dds");
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
    Log(LogLevel::kInfo,
        "PublishKickFrames begin ball=" + FormatXY(ball_x, ball_y) +
            " person=" + FormatVecXY(person_pos) +
            " power=" + std::to_string(power),
        "kick");

    KickReferencePublisher publisher;
    if (!publisher.Create()) {
        Log(LogLevel::kError, "PublishKickFrames failed during DDS creation.", "kick");
        return false;
    }

    for (int frame = 0; frame < kKickFrames && g_run.load(); ++frame) {
        if (!publisher.Write(ball_x, ball_y, person_pos[0], person_pos[1], power)) {
            Log(LogLevel::kError,
                "Failed to publish kick frame " + std::to_string(frame) + "/" +
                    std::to_string(kKickFrames),
                "kick");
            return false;
        }
        SleepMs(kKickPublishIntervalMs);
    }

    Log(LogLevel::kInfo, "PublishKickFrames completed successfully.", "kick");
    return true;
}

// ============================================================================
// Failure summary
// ============================================================================
static void PrintFailureSummary(State final_state) {
    std::ostringstream oss;
    oss << "\n========== DIAGNOSTIC SUMMARY ==========\n"
        << "total_runtime_ms          : " << ElapsedMs() << "\n"
        << "final_state               : " << StateName(final_state) << "\n"
        << "last_successful_phase     : " << g_last_successful_phase << "\n"
        << "last_successful_rpc       : " << g_last_successful_rpc << "\n"
        << "last_failed_rpc           : " << g_last_failed_rpc << "\n"
        << "total_retry_attempts      : " << g_total_retry_attempts << "\n"
        << "vision_started_successfully: " << FormatBool(g_vision_started_successfully) << "\n"
        << "walking_mode_succeeded    : " << FormatBool(g_walking_mode_succeeded) << "\n"
        << "any_detection_poll_succeeded: " << FormatBool(g_any_detection_poll_succeeded) << "\n"
        << "reached_kick_state        : " << FormatBool(g_reached_kick_state) << "\n"
        << "========================================";

    Log(LogLevel::kInfo, oss.str(), "summary");
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <networkInterface>\n"
                  << "Example: " << argv[0] << " eth0\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    const std::string network_interface = argv[1];
    Log(LogLevel::kInfo,
        "Program start. network_interface=" + network_interface,
        "main");

    // ------------------------------------------------------------------------
    // PHASE 1: ChannelFactory::Init
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 1: ChannelFactory::Init");

        Log(LogLevel::kInfo,
            "ENTER ChannelFactory::Instance()->Init domain=0 interface=" + network_interface,
            "init");
        const auto start = std::chrono::steady_clock::now();
        ChannelFactory::Instance()->Init(0, network_interface);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        Log(LogLevel::kInfo,
            "EXIT ChannelFactory::Instance()->Init duration_ms=" + std::to_string(ms),
            "init");

        g_last_successful_phase = "PHASE 1: ChannelFactory::Init";
        SleepMs(kPhaseInitDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 2: B1LocoClient::Init
    // ------------------------------------------------------------------------
    B1LocoClient loco;
    {
        ScopedPhaseTimer phase("PHASE 2: B1LocoClient::Init");

        Log(LogLevel::kInfo, "ENTER loco.Init()", "init");
        const auto start = std::chrono::steady_clock::now();
        loco.Init();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        Log(LogLevel::kInfo, "EXIT loco.Init() duration_ms=" + std::to_string(ms), "init");

        g_last_successful_phase = "PHASE 2: B1LocoClient::Init";
        SleepMs(kPhaseInitDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 3: VisionClient::Init
    // ------------------------------------------------------------------------
    VisionClient vision;
    {
        ScopedPhaseTimer phase("PHASE 3: VisionClient::Init");

        Log(LogLevel::kInfo, "ENTER vision.Init()", "init");
        const auto start = std::chrono::steady_clock::now();
        vision.Init();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        Log(LogLevel::kInfo, "EXIT vision.Init() duration_ms=" + std::to_string(ms), "init");

        g_last_successful_phase = "PHASE 3: VisionClient::Init";
        SleepMs(kPhaseInitDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 4: StartVisionService
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 4: StartVisionService");

        if (RetryRpc("vision.StartVisionService", [&]() -> int {
                return vision.StartVisionService(
                    /*enable_position=*/true,
                    /*enable_color=*/false,
                    /*enable_face_detection=*/true);
            }) != 0) {
            Log(LogLevel::kError,
                "PHASE 4 failed. This means the timeout likely happened before the "
                "vision service became usable.",
                "init");
            EnterSafeState(loco);
            PrintFailureSummary(State::kSearchBall);
            return 1;
        }

        g_vision_started_successfully = true;
        g_last_successful_phase = "PHASE 4: StartVisionService";
        SleepMs(kPhaseServiceDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 5: ChangeMode(kWalking)
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 5: ChangeMode(kWalking)");

        if (RetryRpc("loco.ChangeMode(kWalking)", [&]() -> int {
                return loco.ChangeMode(booster::robot::RobotMode::kWalking);
            }) != 0) {
            Log(LogLevel::kError,
                "PHASE 5 failed. This means locomotion mode switch is a likely timeout source.",
                "init");
            EnterSafeState(loco);
            (void)RetryRpc("vision.StopVisionService", [&]() -> int {
                return vision.StopVisionService();
            }, 2);
            PrintFailureSummary(State::kSearchBall);
            return 1;
        }

        g_walking_mode_succeeded = true;
        g_last_successful_phase = "PHASE 5: ChangeMode(kWalking)";
        SleepMs(kPhaseModeDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 6: passive vision validation
    // ------------------------------------------------------------------------
    if (!PassiveVisionValidation(vision)) {
        Log(LogLevel::kError,
            "PHASE 6 failed. If timeout appears here, the service starts but polling is unstable.",
            "init");
        EnterSafeState(loco);
        (void)RetryRpc("vision.StopVisionService", [&]() -> int {
            return vision.StopVisionService();
        }, 2);
        PrintFailureSummary(State::kSearchBall);
        return 1;
    }
    g_last_successful_phase = "PHASE 6: passive vision validation";

    // ------------------------------------------------------------------------
    // PHASE 7: state machine start
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 7: state machine start");
        g_last_successful_phase = "PHASE 7: state machine start";
    }

    State state = State::kSearchBall;
    std::vector<float> person_pos;
    float ball_x = 0.35f;
    float ball_y = 0.0f;
    int sweep_dir = 1;
    int sweep_count = 0;
    int detection_rpc_failures = 0;
    int ball_lost_count = 0;
    int person_lost_count = 0;
    int loop_count = 0;

    Log(LogLevel::kInfo, "Initial state SEARCH_BALL", "state");

    while (g_run.load()) {
        ++loop_count;

        // Heartbeat log so we can see whether the loop is progressing normally.
        if (loop_count % kHeartbeatEveryLoops == 0) {
            std::ostringstream hb;
            hb << "loop=" << loop_count
               << " state=" << StateName(state)
               << " ball=" << FormatXY(ball_x, ball_y)
               << " person=" << FormatVecXY(person_pos)
               << " detection_rpc_failures=" << detection_rpc_failures
               << " ball_lost_count=" << ball_lost_count
               << " person_lost_count=" << person_lost_count
               << " sweep_dir=" << sweep_dir
               << " sweep_count=" << sweep_count;
            Log(LogLevel::kInfo, hb.str(), "heartbeat");
        }

        const float ratio = (state == State::kApproachBall) ? 0.33f : 1.0f;
        std::vector<DetectResults> objects;

        if (!RetryGetDetectionObject(vision, objects, ratio)) {
            ++detection_rpc_failures;

            Log(LogLevel::kWarn,
                "Detection poll failed. state=" + std::string(StateName(state)) +
                    " detection_rpc_failures=" + std::to_string(detection_rpc_failures),
                "vision");

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

                    Log(LogLevel::kInfo,
                        "Best ball found at " + FormatXY(ball_x, ball_y),
                        "vision");

                    StopMotionAndCenterHead(loco);

                    ball_lost_count = 0;
                    person_lost_count = 0;
                    sweep_dir = 1;
                    sweep_count = 0;

                    const State old_state = state;
                    state = State::kApproachBall;
                    LogStateTransition(old_state, state, "ball found");
                    break;
                }

                (void)LoggedRotateHeadWithDirection(loco, sweep_dir, 0, "searching for ball");
                ++sweep_count;

                if (sweep_count >= kSweepStepsHalf * 2) {
                    sweep_dir = -sweep_dir;
                    sweep_count = 0;

                    Log(LogLevel::kDebug,
                        "Full ball-search head sweep completed; rotating body slightly.",
                        "search");

                    (void)LoggedMove(loco, 0.0f, 0.0f,
                                     static_cast<float>(sweep_dir) * kBallSearchBodyYawRate,
                                     "widen ball search field");
                    SleepMs(kSearchBodyRotateMs);
                    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop post-search rotate");
                }
                break;
            }

            case State::kApproachBall: {
                if (!best_ball) {
                    ++ball_lost_count;
                    Log(LogLevel::kWarn,
                        "Ball not visible while approaching. ball_lost_count=" +
                            std::to_string(ball_lost_count),
                        "state");

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

                Log(LogLevel::kDebug,
                    "Approach controller ball=" + FormatXY(ball_x, ball_y) +
                        " ex=" + std::to_string(ex) +
                        " ey=" + std::to_string(ey) +
                        " vx=" + std::to_string(vx) +
                        " vy=" + std::to_string(vy) +
                        " wz=" + std::to_string(wz),
                    "control");

                (void)LoggedMove(loco,
                                 static_cast<float>(vx),
                                 static_cast<float>(vy),
                                 static_cast<float>(wz),
                                 "approach ball");

                if (ball_x > kBallReadyDistMin &&
                    ball_x < kBallReadyDistMax &&
                    std::abs(ball_y) < kBallAlignY) {
                    StopMotionAndCenterHead(loco);
                    person_lost_count = 0;
                    sweep_dir = 1;
                    sweep_count = 0;

                    const State old_state = state;
                    state = State::kSearchPerson;
                    LogStateTransition(old_state, state, "ball in ready range");
                }
                break;
            }

            case State::kSearchPerson: {
                if (best_person) {
                    person_pos = best_person->position_;
                    person_lost_count = 0;

                    Log(LogLevel::kInfo,
                        "Person locked at " + FormatVecXY(person_pos),
                        "vision");

                    StopMotionAndCenterHead(loco);

                    const State old_state = state;
                    state = State::kAlignBody;
                    LogStateTransition(old_state, state, "person locked");
                    break;
                }

                (void)LoggedRotateHeadWithDirection(loco, sweep_dir, 0, "searching for person");
                ++sweep_count;

                if (sweep_count >= kSweepStepsHalf * 2) {
                    sweep_dir = -sweep_dir;
                    sweep_count = 0;

                    Log(LogLevel::kDebug,
                        "Full person-search head sweep completed; rotating body slightly.",
                        "search");

                    (void)LoggedMove(loco, 0.0f, 0.0f,
                                     static_cast<float>(sweep_dir) * kPersonSearchBodyYawRate,
                                     "widen person search field");
                    SleepMs(kSearchBodyRotateMs);
                    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop post-search rotate");
                }
                break;
            }

            case State::kAlignBody: {
                if (best_person) {
                    person_pos = best_person->position_;
                    person_lost_count = 0;
                    Log(LogLevel::kDebug,
                        "Refresh person target " + FormatVecXY(person_pos),
                        "vision");
                } else {
                    ++person_lost_count;
                    Log(LogLevel::kWarn,
                        "Person not visible while aligning. person_lost_count=" +
                            std::to_string(person_lost_count),
                        "state");

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

                Log(LogLevel::kDebug,
                    "Alignment angle=" + std::to_string(angle) +
                        " threshold=" + std::to_string(kAlignYawThresh),
                    "control");

                if (std::abs(angle) > kAlignYawThresh) {
                    (void)LoggedMove(loco, 0.0f, 0.0f,
                                     std::copysign(kAlignBodyYawRate, angle),
                                     "align body to person");
                } else {
                    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "alignment complete");
                    const State old_state = state;
                    state = State::kKick;
                    g_reached_kick_state = true;
                    LogStateTransition(old_state, state, "body aligned");
                }
                break;
            }

            case State::kKick: {
                StopMotionAndCenterHead(loco);

                if (person_pos.size() < 2) {
                    RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                        ball_lost_count, person_lost_count,
                                        person_pos,
                                        "kick state reached without valid person target");
                    break;
                }

                const float distance = Dist2D(person_pos);
                const float power = std::clamp(
                    kPowerMin + (kPowerMax - kPowerMin) * (distance / kPowerDist),
                    kPowerMin,
                    kPowerMax);

                Log(LogLevel::kInfo,
                    "Kick prep person=" + FormatVecXY(person_pos) +
                        " distance=" + std::to_string(distance) +
                        " power=" + std::to_string(power),
                    "kick");

                if (RetryRpc("loco.ChangeMode(kSoccer)", [&]() -> int {
                        return loco.ChangeMode(booster::robot::RobotMode::kSoccer);
                    }) != 0) {
                    Log(LogLevel::kError,
                        "ChangeMode(kSoccer) failed. If timeout appears here, the issue is "
                        "specific to soccer mode service readiness.",
                        "kick");
                    EnterSafeState(loco);
                    g_run = false;
                    break;
                }

                SleepMs(kPhaseModeDelayMs);

                if (RetryRpc("loco.VisualKick(true)", [&]() -> int {
                        return loco.VisualKick(true, VisualKickVersion::kV2);
                    }) != 0) {
                    Log(LogLevel::kError,
                        "VisualKick(true) failed. This identifies kick-start RPC as the likely source.",
                        "kick");
                    EnterSafeState(loco);
                    g_run = false;
                    break;
                }

                const bool published = PublishKickFrames(ball_x, ball_y, person_pos, power);

                (void)RetryRpc("loco.VisualKick(false)", [&]() -> int {
                    return loco.VisualKick(false, VisualKickVersion::kV2);
                }, 2);

                if (!published) {
                    Log(LogLevel::kError, "Failed to publish kick reference frames.", "kick");
                } else {
                    Log(LogLevel::kInfo, "Pass complete.", "kick");
                }

                EnterSafeState(loco);
                g_run = false;
                break;
            }
        }

        SleepMs(PollDelayMsForState(state));
    }

    Log(LogLevel::kInfo, "Main loop exited; beginning shutdown.", "main");
    EnterSafeState(loco);

    (void)RetryRpc("vision.StopVisionService", [&]() -> int {
        return vision.StopVisionService();
    }, 2);

    PrintFailureSummary(state);
    return 0;
}
