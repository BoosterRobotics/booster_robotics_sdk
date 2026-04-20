#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace booster {
namespace robot {
namespace audio {

enum class AudioSourceType : int32_t {
    kPcmFile = 0,
    kWavFile = 1,
    kPcmStream = 2,
    kMp3File = 3,
};

enum class PlayerState : int32_t {
    kIdle = 0,
    kReady = 1,
    kPlaying = 2,
    kPaused = 3,
    kStopped = 4,
    kCompleted = 5,
    kError = 6,
};

enum class PlayerPriority : int32_t {
    kLow = 0,
    kMedium = 1,
    kHigh = 2,
};

enum class RecorderState : int32_t {
    kIdle = 0,
    kReady = 1,
    kRecording = 2,
    kPaused = 3,
    kStopped = 4,
    kError = 5,
};

enum class AudioCaptureStreamState : int32_t {
    kIdle = 0,
    kReady = 1,
    kStreaming = 2,
    kPaused = 3,
    kStopped = 4,
    kError = 5,
};

struct PcmFormat {
    int32_t sample_rate_hz{16000};
    int32_t channels{1};
    int32_t bits_per_sample{16};
};

struct PlayerInitOptions {
    AudioSourceType source_type{AudioSourceType::kPcmFile};
    std::string source_uri;
    int32_t sample_rate_hz{16000};
    int32_t channels{1};
    int32_t bits_per_sample{16};
    PlayerPriority priority{PlayerPriority::kMedium};
};

struct RecorderInitOptions {
    std::string output_path;
    int32_t sample_rate_hz{16000};
    int32_t channels{1};
    int32_t bits_per_sample{16};
};

struct AudioCaptureStreamOptions {
    bool enable_raw_pcm{true};
    bool enable_naec_pcm{false};
    PcmFormat requested_raw_format{16000, 1, 16};
};

struct PlayerInfo {
    PlayerState state{PlayerState::kIdle};
    int64_t played_bytes{0};
    int64_t total_bytes{0};
    float volume{1.0F};
};

struct RecorderInfo {
    RecorderState state{RecorderState::kIdle};
    int64_t captured_bytes{0};
};

struct AudioCaptureFrame {
    int64_t frame_seq{0};
    int64_t timestamp_ms{0};

    bool raw_valid{false};
    PcmFormat raw_format;
    int32_t raw_frame_samples_per_channel{0};
    std::vector<int16_t> raw_pcm;

    bool naec_valid{false};
    PcmFormat naec_format;
    int32_t naec_frame_samples_per_channel{0};
    std::vector<int16_t> naec_pcm;
};

struct AudioCaptureStreamInfo {
    AudioCaptureStreamState state{AudioCaptureStreamState::kIdle};
    bool raw_enabled{false};
    bool naec_enabled{false};
    PcmFormat actual_raw_format;
    PcmFormat actual_naec_format;
    int64_t published_frames{0};
    int64_t dropped_frames{0};
};

struct AudioError {
    int32_t ret_code{0};
    std::string ret_msg;
    int32_t error_category{0};
    int32_t error_detail{0};
};

using PlayerStateCallback = std::function<void(PlayerState)>;
using PlayerProgressCallback = std::function<void(const PlayerInfo&)>;
using PlayerCompletionCallback = std::function<void(const PlayerInfo&)>;
using PlayerErrorCallback = std::function<void(const AudioError&)>;

using RecorderStateCallback = std::function<void(RecorderState)>;
using RecorderProgressCallback = std::function<void(const RecorderInfo&)>;
using RecorderErrorCallback = std::function<void(const AudioError&)>;

using AudioCaptureFrameCallback = std::function<void(const AudioCaptureFrame&)>;
using AudioCaptureStreamStateCallback = std::function<void(AudioCaptureStreamState)>;
using AudioCaptureStreamErrorCallback = std::function<void(const AudioError&)>;

} // namespace audio
} // namespace robot
} // namespace booster
