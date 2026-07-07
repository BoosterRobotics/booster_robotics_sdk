/**
 * visual_kick_ball_detect_dds_camera.cpp
 *
 * Detects a ball by subscribing to the robot's RGB camera directly over DDS
 * (no USB/V4L capture, no ROS middleware required) and then walks toward the
 * ball and performs a visual kick.
 *
 * Two detection modes are supported and tried in priority order:
 *   1. YOLO (YOLOv5/v8 ONNX) – when a valid model path is supplied
 *   2. Color-based HSV segmentation – always active as a fallback
 *
 * Camera input is received from a DDS topic that publishes either:
 *   a) sensor_msgs::msg::CompressedImage  (e.g. JPEG, default)
 *   b) sensor_msgs::msg::Image            (raw BGR/RGB, selected with --raw flag)
 *
 * Usage:
 *   ./visual_kick_ball_detect_dds_camera  <networkInterface>  <cameraTopicName>  <yolo.onnx|none>  [--raw]
 *
 * Examples:
 *   # YOLO on a compressed topic (most common for Booster B1 head camera)
 *   ./visual_kick_ball_detect_dds_camera  eth0  rt/head_camera/color/image_raw/compressed  ball.onnx
 *
 *   # Color-only detection on a raw Image topic
 *   ./visual_kick_ball_detect_dds_camera  eth0  rt/head_camera/color/image_raw  none  --raw
 *
 * Camera DDS topic conventions for Booster B1:
 *   Compressed : rt/head_camera/color/image_raw/compressed
 *   Raw        : rt/head_camera/color/image_raw
 *
 * YOLO note:
 *   Export your model with: python export.py --weights ball.pt --include onnx --imgsz 640
 *   Pass ball_class_id (default 0) as the integer class index for the ball.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/Kick.h>
#include <booster/idl/sensor_msgs/CompressedImage.h>
#include <booster/idl/sensor_msgs/Image.h>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;
using booster::robot::ChannelFactory;
using booster::robot::ChannelSubscriber;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static std::atomic<bool> g_run{true};
void sigint_handler(int) { g_run = false; }

// ---------------------------------------------------------------------------
// Kick parameters (tune to match your field and ball)
// ---------------------------------------------------------------------------
static constexpr double kGoalX    = 4.5;
static constexpr double kGoalY    = 0.0;
static constexpr double kKickPower = 0.85;

// Thread-safe latest frame shared between the DDS callback and the main loop.
static cv::Mat  g_latest_frame;
static std::mutex g_frame_mutex;

struct Det {
    bool   found{false};
    float  conf{0.f};
    cv::Rect box;
};

// ---------------------------------------------------------------------------
// Frame conversion helpers
// ---------------------------------------------------------------------------

/** Build a cv::Mat from a raw sensor_msgs::msg::Image (bgr8 / rgb8 / mono8). */
static cv::Mat ImageMsgToMat(const sensor_msgs::msg::Image* msg) {
    if (!msg || msg->data().empty()) return {};

    const int h = static_cast<int>(msg->height());
    const int w = static_cast<int>(msg->width());
    const auto& enc = msg->encoding();

    cv::Mat out;
    if (enc == "bgr8" || enc == "8UC3") {
        cv::Mat tmp(h, w, CV_8UC3, const_cast<uint8_t*>(msg->data().data()));
        out = tmp.clone();
    } else if (enc == "rgb8") {
        cv::Mat tmp(h, w, CV_8UC3, const_cast<uint8_t*>(msg->data().data()));
        cv::cvtColor(tmp, out, cv::COLOR_RGB2BGR);
    } else if (enc == "mono8" || enc == "8UC1") {
        cv::Mat tmp(h, w, CV_8UC1, const_cast<uint8_t*>(msg->data().data()));
        cv::cvtColor(tmp, out, cv::COLOR_GRAY2BGR);
    } else if (enc == "yuv422" || enc == "yuv422_yuy2") {
        cv::Mat tmp(h, w, CV_8UC2, const_cast<uint8_t*>(msg->data().data()));
        cv::cvtColor(tmp, out, cv::COLOR_YUV2BGR_YUY2);
    } else {
        // Best-effort: try treating as bgr8
        if (static_cast<int>(msg->data().size()) >= h * w * 3) {
            cv::Mat tmp(h, w, CV_8UC3, const_cast<uint8_t*>(msg->data().data()));
            out = tmp.clone();
        }
    }
    return out;
}

/** Build a cv::Mat from a sensor_msgs::msg::CompressedImage (jpeg/png). */
static cv::Mat CompressedImageMsgToMat(const sensor_msgs::msg::CompressedImage* msg) {
    if (!msg || msg->data().empty()) return {};
    cv::Mat buf(1, static_cast<int>(msg->data().size()), CV_8UC1,
                const_cast<uint8_t*>(msg->data().data()));
    return cv::imdecode(buf, cv::IMREAD_COLOR);
}

// ---------------------------------------------------------------------------
// Ball detection: YOLO
// ---------------------------------------------------------------------------

static Det DetectBallYOLO(cv::dnn::Net& net, const cv::Mat& frame, int ball_class_id = 0) {
    Det out;
    constexpr int  IN   = 640;
    constexpr float CONF = 0.35f;
    constexpr float NMS  = 0.45f;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0,
                                          cv::Size(IN, IN), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return out;

    cv::Mat pred = outputs[0];
    if (pred.dims == 3 && pred.size[0] == 1)
        pred = pred.reshape(1, pred.size[1]);  // Nx(5+C)

    std::vector<cv::Rect> boxes;
    std::vector<float>    confs;
    const int W = frame.cols, H = frame.rows;

    for (int i = 0; i < pred.rows; ++i) {
        const float* d    = pred.ptr<float>(i);
        float        obj  = d[4];
        if (obj < 1e-6f) continue;

        int   cls      = -1;
        float cls_conf = 0.f;
        for (int c = 5; c < pred.cols; ++c) {
            if (d[c] > cls_conf) { cls_conf = d[c]; cls = c - 5; }
        }
        float conf = obj * cls_conf;
        if (conf < CONF || cls != ball_class_id) continue;

        float cx = d[0], cy = d[1], w = d[2], h = d[3];
        float sx = float(W) / IN, sy = float(H) / IN;
        cv::Rect r(int((cx - 0.5f * w) * sx), int((cy - 0.5f * h) * sy),
                   int(w * sx), int(h * sy));
        r &= cv::Rect(0, 0, W, H);
        if (r.area() > 0) { boxes.push_back(r); confs.push_back(conf); }
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, confs, CONF, NMS, keep);
    if (!keep.empty()) {
        int k      = keep[0];
        out.found  = true;
        out.conf   = confs[k];
        out.box    = boxes[k];
    }
    return out;
}

// ---------------------------------------------------------------------------
// Ball detection: color-based HSV segmentation (orange/yellow ball fallback)
// ---------------------------------------------------------------------------

// HSV thresholds for a typical orange soccer ball.
// Adjust lower_hsv / upper_hsv for your specific ball colour.
static const cv::Scalar kBallHsvLower(5,  120, 80);
static const cv::Scalar kBallHsvUpper(25, 255, 255);
static constexpr double kMinBallArea = 400.0;  // px²

static Det DetectBallColor(const cv::Mat& frame) {
    Det out;
    if (frame.empty()) return out;

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, kBallHsvLower, kBallHsvUpper, mask);

    // Morphological cleanup to remove noise.
    cv::erode (mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double   best_area = 0.0;
    cv::Rect best_rect;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area > best_area && area >= kMinBallArea) {
            best_area = area;
            best_rect = cv::boundingRect(c);
        }
    }

    if (best_area >= kMinBallArea) {
        out.found = true;
        out.conf  = static_cast<float>(best_area) / (frame.cols * frame.rows);
        out.box   = best_rect;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0]
                  << " <networkInterface> <cameraTopicName> <yolo.onnx|none> [--raw]\n"
                  << "\n"
                  << "  networkInterface  : e.g. eth0\n"
                  << "  cameraTopicName   : DDS topic name for the camera stream\n"
                  << "                      Compressed example: rt/head_camera/color/image_raw/compressed\n"
                  << "                      Raw example:        rt/head_camera/color/image_raw\n"
                  << "  yolo.onnx|none    : path to YOLOv5/v8 ONNX model, or 'none' to use\n"
                  << "                      color-based detection only\n"
                  << "  --raw             : subscribe as sensor_msgs/Image (raw pixels)\n"
                  << "                      default: sensor_msgs/CompressedImage\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    const std::string net_if      = argv[1];
    const std::string cam_topic   = argv[2];
    const std::string yolo_path   = argv[3];
    const bool use_raw_image = (argc >= 5 && std::string(argv[4]) == "--raw");
    const bool use_yolo      = (yolo_path != "none" && !yolo_path.empty());

    // -----------------------------------------------------------------------
    // FastDDS publisher for the kick reference (same as original)
    // -----------------------------------------------------------------------
    DomainParticipantQos pqos;
    pqos.name("visual_kick_dds_camera_participant");
    auto* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (!participant) { std::cerr << "Failed to create DDS participant\n"; return 1; }

    TypeSupport kick_type(new brain::msg::Kick());
    kick_type.register_type(participant);

    Topic* topic = participant->create_topic(
        booster::robot::b1::kTopicKickReference,
        kick_type.get_type_name(), TOPIC_QOS_DEFAULT);
    Publisher*  pub    = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    DataWriter* writer = pub ? pub->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr) : nullptr;
    if (!topic || !pub || !writer) {
        std::cerr << "Failed to create DDS kick publisher\n";
        return 1;
    }

    // -----------------------------------------------------------------------
    // ChannelFactory + robot
    // -----------------------------------------------------------------------
    ChannelFactory::Instance()->Init(0, net_if);

    B1LocoClient loco;
    loco.Init();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // -----------------------------------------------------------------------
    // DDS camera subscriber
    // Subscribe to either CompressedImage or raw Image depending on the flag.
    // The callback stores the latest decoded frame for the main loop.
    // -----------------------------------------------------------------------
    std::unique_ptr<ChannelSubscriber<sensor_msgs::msg::CompressedImage>> compressed_sub;
    std::unique_ptr<ChannelSubscriber<sensor_msgs::msg::Image>>           raw_sub;

    if (use_raw_image) {
        raw_sub = std::make_unique<ChannelSubscriber<sensor_msgs::msg::Image>>(
            cam_topic,
            [](const void* msg_ptr) {
                const auto* img = static_cast<const sensor_msgs::msg::Image*>(msg_ptr);
                cv::Mat frame = ImageMsgToMat(img);
                if (frame.empty()) return;
                std::lock_guard<std::mutex> lk(g_frame_mutex);
                g_latest_frame = std::move(frame);
            });
        raw_sub->InitChannel();
        std::cout << "[camera] Subscribing to raw Image topic: " << cam_topic << "\n";
    } else {
        compressed_sub =
            std::make_unique<ChannelSubscriber<sensor_msgs::msg::CompressedImage>>(
                cam_topic,
                [](const void* msg_ptr) {
                    const auto* img =
                        static_cast<const sensor_msgs::msg::CompressedImage*>(msg_ptr);
                    cv::Mat frame = CompressedImageMsgToMat(img);
                    if (frame.empty()) return;
                    std::lock_guard<std::mutex> lk(g_frame_mutex);
                    g_latest_frame = std::move(frame);
                });
        compressed_sub->InitChannel();
        std::cout << "[camera] Subscribing to CompressedImage topic: " << cam_topic << "\n";
    }

    // -----------------------------------------------------------------------
    // YOLO model (optional)
    // -----------------------------------------------------------------------
    cv::dnn::Net yolo_net;
    if (use_yolo) {
        yolo_net = cv::dnn::readNetFromONNX(yolo_path);
        if (yolo_net.empty()) {
            std::cerr << "[yolo] Failed to load model: " << yolo_path
                      << " — falling back to color detection\n";
        } else {
            yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            std::cout << "[yolo] Model loaded: " << yolo_path << "\n";
        }
    } else {
        std::cout << "[detection] YOLO disabled — using color-based HSV detection only\n";
    }

    // -----------------------------------------------------------------------
    // Wait briefly for the first frame to arrive
    // -----------------------------------------------------------------------
    std::cout << "[camera] Waiting for first frame …\n";
    for (int wait = 0; wait < 50 && g_run.load(); ++wait) {
        {
            std::lock_guard<std::mutex> lk(g_frame_mutex);
            if (!g_latest_frame.empty()) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    {
        std::lock_guard<std::mutex> lk(g_frame_mutex);
        if (g_latest_frame.empty()) {
            std::cerr << "[camera] No frame received on topic '" << cam_topic
                      << "' after 5 s. Check the topic name and network interface.\n";
            return 1;
        }
    }
    std::cout << "[camera] First frame received.\n";

    // -----------------------------------------------------------------------
    // Walking mode
    // -----------------------------------------------------------------------
    loco.ChangeMode(booster::robot::RobotMode::kWalking);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    bool   ready  = false;
    double ball_x = 0.35, ball_y = 0.0;

    // -----------------------------------------------------------------------
    // Main detection + approach loop
    // -----------------------------------------------------------------------
    while (g_run.load() && !ready) {
        // Grab the latest camera frame (non-blocking).
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lk(g_frame_mutex);
            if (!g_latest_frame.empty()) frame = g_latest_frame.clone();
        }
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // ---- Detection -------------------------------------------------------
        Det det;

        // 1) Try YOLO first (if the model loaded successfully)
        if (use_yolo && !yolo_net.empty()) {
            det = DetectBallYOLO(yolo_net, frame, /*ball_class_id=*/0);
        }

        // 2) Fall back to color-based detection
        if (!det.found) {
            det = DetectBallColor(frame);
        }

        if (!det.found) {
            loco.Move(0.0, 0.0, 0.2);  // rotate slowly to search
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            continue;
        }

        // ---- Simple proportional position estimate --------------------------
        // Project bounding-box centre to approximate ground-plane distances.
        // Replace with a proper BallPoseEstimator when calibration data is available.
        const int   cx_px  = det.box.x + det.box.width  / 2;
        const float img_w  = static_cast<float>(frame.cols);

        // Normalised image-plane horizontal offset from centre  [-0.5 .. 0.5]
        const float nx = (static_cast<float>(cx_px) / img_w) - 0.5f;

        // Heuristic ground-plane estimate (tune for your robot + camera mount):
        //   - Larger ball bbox  → ball is closer  → smaller ball_x
        //   - Positive nx       → ball is to the right → positive ball_y
        // Use squared diagonal to avoid the square-root in the hot loop;
        // the constant 14400 = 120² compensates so that the formula is equivalent.
        const float bbox_diag_sq = float(det.box.width  * det.box.width +
                                         det.box.height * det.box.height);
        ball_x = std::clamp(0.35f * (14400.f / (bbox_diag_sq + 1.f)), 0.15f, 2.0f);
        ball_y = std::clamp(nx * 0.8f, -0.5f, 0.5f);

        // ---- Approach controller -------------------------------------------
        const double ex  = ball_x - 0.33;
        const double ey  = ball_y;
        const double vx  = std::clamp(0.8  * ex, -0.25, 0.25);
        const double vy  = std::clamp(1.0  * ey, -0.12, 0.12);
        const double wz  = std::clamp(1.0  * ey, -0.4,  0.4);

        loco.Move(vx, vy, wz);

        if (ball_x > 0.22 && ball_x < 0.45 && std::abs(ball_y) < 0.05) {
            loco.Move(0.0, 0.0, 0.0);
            ready = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    // -----------------------------------------------------------------------
    // Soccer mode → visual kick
    // -----------------------------------------------------------------------
    if (ready) {
        int rc_soccer = loco.ChangeMode(booster::robot::RobotMode::kSoccer);
        if (rc_soccer == 0) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            int ret = loco.VisualKick(true, VisualKickVersion::kV2);

            if (ret == 0) {
                for (int i = 0; i < 20 && g_run.load(); ++i) {
                    brain::msg::Kick msg;
                    msg.x(ball_x);
                    msg.y(ball_y);
                    msg.goal_x(kGoalX);
                    msg.goal_y(kGoalY);
                    msg.power(kKickPower);
                    writer->write(&msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
                loco.VisualKick(false, VisualKickVersion::kV2);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Cleanup
    // -----------------------------------------------------------------------
    pub->delete_datawriter(writer);
    participant->delete_publisher(pub);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);
    return 0;
}
