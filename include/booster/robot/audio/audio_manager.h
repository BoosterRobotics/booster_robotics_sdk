#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <booster/idl/audio/AudioErrorTopic.h>
#include <booster/idl/audio/AudioEventTopic.h>
#include <booster/idl/audio/AudioProgressTopic.h>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/rpc/rpc_client.hpp>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {
namespace audio {

class AudioPlayer;
class AudioRecorder;
class AudioLocalizer;
class AudioSystemController;
class AudioCaptureStream;

class AudioManager {
public:
    using Json = nlohmann::json;
    using ProgressCallback = std::function<void(const booster_msgs::audio::AudioProgressTopic &)>;
    using EventCallback = std::function<void(const booster_msgs::audio::AudioEventTopic &)>;
    using ErrorCallback = std::function<void(const booster_msgs::audio::AudioErrorTopic &)>;

    enum class ServiceMethod : int32_t {
        kRegisterClient = 0,
        kInitPlayer,
        kStartPlayer,
        kPausePlayer,
        kStopPlayer,
        kResetPlayer,
        kDestroyPlayer,
        kSetPlayerVolume,
        kGetPlayerInfo,
        kSendPcmData,
        kInitRecorder,
        kStartRecorder,
        kPauseRecorder,
        kStopRecorder,
        kDestroyRecorder,
        kGetRecorderInfo,
        kGetDoaAngle,
        kSetSystemVolume,
        kGetSystemVolume,
        kSetSystemMute,
        kGetSystemMute,
        kInitCaptureStream,
        kStartCaptureStream,
        kPauseCaptureStream,
        kStopCaptureStream,
        kDestroyCaptureStream,
        kGetCaptureStreamInfo,
    };

    AudioManager();
    ~AudioManager();

    int32_t Init();
    void Shutdown();

    bool IsInitialized() const;
    std::string GetClientId() const;
    std::string GenerateRequestId();

    std::shared_ptr<AudioPlayer> CreatePlayer();
    std::shared_ptr<AudioRecorder> CreateRecorder();
    std::shared_ptr<AudioLocalizer> CreateLocalizer();
    std::shared_ptr<AudioSystemController> CreateSystemController();
    std::shared_ptr<AudioCaptureStream> CreateCaptureStream();

    int32_t CallService(ServiceMethod method, const Json &request, Json *response = nullptr,
        int64_t timeout_ms = 1000);

    void SetProgressCallback(ProgressCallback callback);
    void SetEventCallback(EventCallback callback);
    void SetErrorCallback(ErrorCallback callback);

private:
    friend class AudioPlayer;
    friend class AudioRecorder;
    friend class AudioCaptureStream;

    int64_t AddProgressListener(ProgressCallback callback);
    void RemoveProgressListener(int64_t listener_id);
    int64_t AddEventListener(EventCallback callback);
    void RemoveEventListener(int64_t listener_id);
    int64_t AddErrorListener(ErrorCallback callback);
    void RemoveErrorListener(int64_t listener_id);

    int32_t RegisterClientLocked();
    void PrewarmRpcClientsLocked();
    int32_t SendRequestLocked(ServiceMethod method, Json request, Json *response, int64_t timeout_ms,
        bool inject_client_meta);
    std::shared_ptr<booster::robot::RpcClient> GetOrCreateRpcClientLocked(ServiceMethod method);
    void EnsureTopicSubscribersLocked();
    // Topic callbacks take a snapshot of the registered listeners before invoking any
    // user-provided code. This keeps listener dispatch stable even if callbacks add/remove
    // listeners, and avoids calling external code while holding mutex_.
    void HandleProgress(const void *msg);
    void HandleEvent(const void *msg);
    void HandleError(const void *msg);

    static const char *GetRpcChannelName(ServiceMethod method);

    mutable std::mutex mutex_;
    bool initialized_{false};
    std::string client_id_;
    int64_t next_request_sequence_{1};
    std::unordered_map<ServiceMethod, std::shared_ptr<booster::robot::RpcClient>> rpc_clients_;
    ProgressCallback progress_callback_;
    EventCallback event_callback_;
    ErrorCallback error_callback_;
    int64_t next_listener_id_{1};
    std::unordered_map<int64_t, ProgressCallback> progress_listeners_;
    std::unordered_map<int64_t, EventCallback> event_listeners_;
    std::unordered_map<int64_t, ErrorCallback> error_listeners_;
    std::shared_ptr<booster::robot::ChannelSubscriber<booster_msgs::audio::AudioProgressTopic>>
        progress_subscriber_;
    std::shared_ptr<booster::robot::ChannelSubscriber<booster_msgs::audio::AudioEventTopic>>
        event_subscriber_;
    std::shared_ptr<booster::robot::ChannelSubscriber<booster_msgs::audio::AudioErrorTopic>>
        error_subscriber_;
};

} // namespace audio
} // namespace robot
} // namespace booster
