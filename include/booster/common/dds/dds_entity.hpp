#ifndef __BOOSTER_DDS_ENTITY_HPP__
#define __BOOSTER_DDS_ENTITY_HPP__

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <booster/common/dds/dds_callback.hpp>

namespace booster {
namespace common {

using namespace eprosima::fastdds::dds;

// class DdsParticipant {
// public:
//     using NATIVE_TYPE = dds::domain::DomainParticipant;

// }

// class DdsReaderListener {
// public:
//     explicit

// };

using DdsParticipantPtr = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>;
using DdsWriterPtr = std::shared_ptr<eprosima::fastdds::dds::DataWriter>;
using DdsWriter = eprosima::fastdds::dds::DataWriter;
using DdsReaderPtr = std::shared_ptr<eprosima::fastdds::dds::DataReader>;
using DdsReader = eprosima::fastdds::dds::DataReader;
using DdsPublisherPtr = std::shared_ptr<eprosima::fastdds::dds::Publisher>;
using DdsSubscriberPtr = std::shared_ptr<eprosima::fastdds::dds::Subscriber>;
using DdsTopicPtr = std::shared_ptr<eprosima::fastdds::dds::Topic>;
using DdsTopic = eprosima::fastdds::dds::Topic;
using DdsTopicDataTypePtr = std::shared_ptr<eprosima::fastdds::dds::TopicDataType>;
using DdsReaderCallbackPtr = std::shared_ptr<DdsReaderCallback>;

enum class DdsExecutorOverflowPolicy {
    kDropNewest,
    kDropOldest,
    kLatestOnly,
};

enum class DdsExecutorDispatchMode {
    kShared,
    kDedicated,
};

struct DdsReaderExecutorOptions {
    // 0 means unbounded. Generic subscribers use a bounded queue by default.
    size_t queue_capacity{64};
    DdsExecutorOverflowPolicy overflow_policy{DdsExecutorOverflowPolicy::kDropOldest};
    bool enable_metrics{false};
    DdsExecutorDispatchMode dispatch_mode{DdsExecutorDispatchMode::kShared};
};

struct DdsReaderExecutorMetrics {
    uint64_t samples_received{0};
    uint64_t samples_dropped{0};
    uint64_t callbacks_executed{0};
    size_t current_queue_size{0};
    size_t queue_high_watermark{0};
    uint64_t max_queue_latency_us{0};
    uint64_t max_callback_duration_us{0};
};

class DdsCallbackExecutor {
public:
    static DdsCallbackExecutor &Instance() {
        static DdsCallbackExecutor instance;
        return instance;
    }

    void Submit(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace_back(std::move(task));
        }
        cv_.notify_one();
    }

private:
    DdsCallbackExecutor() {
        const auto thread_count = std::min<size_t>(
            4,
            std::max<size_t>(1, std::thread::hardware_concurrency()));
        executors_.reserve(thread_count);
        for (size_t i = 0; i < thread_count; ++i) {
            executors_.emplace_back([this]() { Run(); });
        }
    }

    ~DdsCallbackExecutor() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_requested_ = true;
        }
        cv_.notify_all();
        for (auto &executor : executors_) {
            if (executor.joinable()) {
                executor.join();
            }
        }
    }

    void Run() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] {
                    return stop_requested_ || !tasks_.empty();
                });

                if (tasks_.empty()) {
                    if (stop_requested_) {
                        return;
                    }
                    continue;
                }

                task = std::move(tasks_.front());
                tasks_.pop_front();
            }

            task();
        }
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> tasks_;
    std::vector<std::thread> executors_;
    bool stop_requested_{false};
};

class DdsDedicatedCallbackExecutor {
public:
    DdsDedicatedCallbackExecutor() :
        executor_([this]() { Run(); }) {
    }

    ~DdsDedicatedCallbackExecutor() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_requested_ = true;
        }
        cv_.notify_all();
        if (executor_.joinable()) {
            executor_.join();
        }
    }

    void Submit(std::function<void()> task) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace_back(std::move(task));
        }
        cv_.notify_one();
    }

private:
    void Run() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] {
                    return stop_requested_ || !tasks_.empty();
                });

                if (tasks_.empty()) {
                    if (stop_requested_) {
                        return;
                    }
                    continue;
                }

                task = std::move(tasks_.front());
                tasks_.pop_front();
            }

            task();
        }
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> tasks_;
    std::thread executor_;
    bool stop_requested_{false};
};

template <typename MSG>
class DdsReaderListener : public DataReaderListener, public std::enable_shared_from_this<DdsReaderListener<MSG>> {
public:
    DdsReaderListener() = default;
    ~DdsReaderListener() override {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
        pending_messages_.clear();
        callback_scheduled_ = false;
    }

    void SetCallback(const DdsReaderCallback &cb) {
        if (!cb.HasMessageHandler()) {
            std::cerr << "Listener Set Callback: invalid hanlder" << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        cb_ = std::make_shared<DdsReaderCallback>(cb);
    }

    void SetExecutorOptions(const DdsReaderExecutorOptions &options) {
        std::lock_guard<std::mutex> lock(mutex_);
        executor_options_ = options;
        if (executor_options_.dispatch_mode == DdsExecutorDispatchMode::kDedicated) {
            if (dedicated_executor_ == nullptr) {
                dedicated_executor_ = std::make_unique<DdsDedicatedCallbackExecutor>();
            }
        } else {
            dedicated_executor_.reset();
        }
    }

    DdsReaderExecutorMetrics GetExecutorMetrics() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto metrics = metrics_;
        metrics.current_queue_size = pending_messages_.size();
        return metrics;
    }

    void on_data_available(DataReader *reader) override {
        if (reader == nullptr || cb_ == nullptr) {
            return;
        }

        std::deque<PendingMessage> ready_messages;
        SampleInfo info;
        while (true) {
            MSG st;
            if (reader->take_next_sample(&st, &info) != ReturnCode_t::RETCODE_OK) {
                break;
            }
            if (info.valid_data) {
                ready_messages.push_back(PendingMessage{
                    std::move(st),
                    std::chrono::steady_clock::now(),
                });
            }
        }

        if (ready_messages.empty()) {
            return;
        }

        bool should_schedule = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stop_requested_) {
                return;
            }
            for (auto &message : ready_messages) {
                EnqueuePendingMessageLocked(std::move(message));
            }
            if (!callback_scheduled_ && !pending_messages_.empty()) {
                callback_scheduled_ = true;
                should_schedule = true;
            }
        }

        if (should_schedule) {
            ScheduleExecution();
        }
    }

private:
    struct PendingMessage {
        MSG message;
        std::chrono::steady_clock::time_point enqueued_at;
    };

    void EnqueuePendingMessageLocked(PendingMessage &&message) {
        if (executor_options_.enable_metrics) {
            ++metrics_.samples_received;
        }

        const auto capacity = executor_options_.queue_capacity;
        switch (executor_options_.overflow_policy) {
        case DdsExecutorOverflowPolicy::kDropNewest:
            if (capacity > 0 && pending_messages_.size() >= capacity) {
                RecordDropLocked(1);
                return;
            }
            pending_messages_.push_back(std::move(message));
            break;
        case DdsExecutorOverflowPolicy::kDropOldest:
            if (capacity > 0 && pending_messages_.size() >= capacity) {
                pending_messages_.pop_front();
                RecordDropLocked(1);
            }
            pending_messages_.push_back(std::move(message));
            break;
        case DdsExecutorOverflowPolicy::kLatestOnly:
            if (!pending_messages_.empty()) {
                RecordDropLocked(pending_messages_.size());
                pending_messages_.clear();
            }
            pending_messages_.push_back(std::move(message));
            break;
        }

        if (executor_options_.enable_metrics) {
            metrics_.queue_high_watermark = std::max(
                metrics_.queue_high_watermark,
                pending_messages_.size());
        }
    }

    void RecordDropLocked(size_t drop_count) {
        if (executor_options_.enable_metrics) {
            metrics_.samples_dropped += drop_count;
        }
    }

    void ScheduleExecution() {
        auto weak_self = this->weak_from_this();
        std::function<void()> task = [weak_self]() {
            auto self = weak_self.lock();
            if (self == nullptr) {
                return;
            }
            self->ExecutePendingCallbacks();
        };

        DdsExecutorDispatchMode dispatch_mode = DdsExecutorDispatchMode::kShared;
        DdsDedicatedCallbackExecutor *dedicated_executor = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            dispatch_mode = executor_options_.dispatch_mode;
            dedicated_executor = dedicated_executor_.get();
        }

        if (dispatch_mode == DdsExecutorDispatchMode::kDedicated && dedicated_executor != nullptr) {
            dedicated_executor->Submit(std::move(task));
            return;
        }

        DdsCallbackExecutor::Instance().Submit(std::move(task));
    }

    void ExecutePendingCallbacks() {
        while (true) {
            PendingMessage pending_message;
            DdsReaderCallbackPtr cb;
            bool metrics_enabled = false;
            size_t pending_queue_size = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (stop_requested_) {
                    callback_scheduled_ = false;
                    return;
                }
                if (pending_messages_.empty()) {
                    callback_scheduled_ = false;
                    return;
                }

                pending_message = std::move(pending_messages_.front());
                pending_messages_.pop_front();
                cb = cb_;
                metrics_enabled = executor_options_.enable_metrics;
                pending_queue_size = pending_messages_.size();
            }

            const auto callback_start = std::chrono::steady_clock::now();
            if (cb != nullptr) {
                cb->OnDataAvailable(&pending_message.message);
            }

            if (metrics_enabled) {
                const auto callback_end = std::chrono::steady_clock::now();
                const auto queue_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    callback_start - pending_message.enqueued_at).count();
                const auto callback_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    callback_end - callback_start).count();

                std::lock_guard<std::mutex> lock(mutex_);
                ++metrics_.callbacks_executed;
                metrics_.current_queue_size = pending_queue_size;
                metrics_.max_queue_latency_us = std::max<uint64_t>(
                    metrics_.max_queue_latency_us,
                    static_cast<uint64_t>(queue_latency_us));
                metrics_.max_callback_duration_us = std::max<uint64_t>(
                    metrics_.max_callback_duration_us,
                    static_cast<uint64_t>(callback_duration_us));
            }
        }
    }

private:
    std::mutex mutex_;
    std::deque<PendingMessage> pending_messages_;
    bool stop_requested_{false};
    bool callback_scheduled_{false};
    DdsReaderExecutorOptions executor_options_;
    DdsReaderExecutorMetrics metrics_;
    DdsReaderCallbackPtr cb_;
    std::unique_ptr<DdsDedicatedCallbackExecutor> dedicated_executor_;
};

template <typename MSG>
using DdsReaderListenerPtr = std::shared_ptr<DdsReaderListener<MSG>>;
}
} // namespace booster::common

#endif
