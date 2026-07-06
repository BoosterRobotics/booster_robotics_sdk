#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/idl/b1/Kick.h>

// Reuse the same estimator path from robocup_demo:
#include "booster_vision/base/geometry.hpp"                 // Pose, Intrinsics
#include "booster_vision/base/detection.hpp"                // DetectionRes (adjust include if your path differs)
#include "booster_vision/pose_estimator/pose_estimator.h"   // BallPoseEstimator

using namespace eprosima::fastdds::dds;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;

static std::atomic<bool> g_run{true};
void sigint_handler(int) { g_run = false; }

struct Det {
    bool found{false};
    float conf{0.f};
    cv::Rect box;
};

static Det DetectBallYOLO(cv::dnn::Net& net, const cv::Mat& frame, int ball_class_id = 0) {
    Det out;
    constexpr int IN = 640;
    constexpr float CONF = 0.35f;
    constexpr float NMS = 0.45f;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0/255.0, cv::Size(IN, IN), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return out;

    cv::Mat pred = outputs[0];
    if (pred.dims == 3 && pred.size[0] == 1) pred = pred.reshape(1, pred.size[1]); // Nx(5+C)

    std::vector<cv::Rect> boxes;
    std::vector<float> confs;

    const int W = frame.cols, H = frame.rows;
    for (int i = 0; i < pred.rows; ++i) {
        const float* d = pred.ptr<float>(i);
        float obj = d[4];
        if (obj < 1e-6f) continue;

        int cls = -1; float cls_conf = 0.f;
        for (int c = 5; c < pred.cols; ++c) {
            if (d[c] > cls_conf) { cls_conf = d[c]; cls = c - 5; }
        }
        float conf = obj * cls_conf;
        if (conf < CONF || cls != ball_class_id) continue;

        float cx = d[0], cy = d[1], w = d[2], h = d[3];
        float sx = float(W)/IN, sy = float(H)/IN;
        cv::Rect r(int((cx - 0.5f*w)*sx), int((cy - 0.5f*h)*sy), int(w*sx), int(h*sy));
        r &= cv::Rect(0, 0, W, H);
        if (r.area() > 0) { boxes.push_back(r); confs.push_back(conf); }
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, confs, CONF, NMS, keep);
    if (!keep.empty()) {
        int k = keep[0];
        out.found = true;
        out.conf = confs[k];
        out.box = boxes[k];
    }
    return out;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " networkInterface cameraIndex yolo.onnx\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    std::string net_if = argv[1];
    int cam_idx = std::stoi(argv[2]);
    std::string yolo_path = argv[3];

    // FastDDS setup (same baseline as visual_kick_no_ros.cpp)
    DomainParticipantQos pqos;
    pqos.name("visual_kick_yolo_pose_estimator_no_ros_participant");
    auto* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (!participant) return 1;

    TypeSupport kick_type(new brain::msg::Kick());
    kick_type.register_type(participant);

    Topic* topic = participant->create_topic(
        booster::robot::b1::kTopicKickReference, kick_type.get_type_name(), TOPIC_QOS_DEFAULT);
    Publisher* pub = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    DataWriter* writer = pub ? pub->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr) : nullptr;
    if (!topic || !pub || !writer) return 1;

    // Robot
    booster::robot::ChannelFactory::Instance()->Init(0, net_if);
    B1LocoClient loco; loco.Init();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Camera + YOLO
    cv::VideoCapture cap(cam_idx);
    if (!cap.isOpened()) return 1;

    cv::dnn::Net net = cv::dnn::readNetFromONNX(yolo_path);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // --- Pose estimator setup (same path as robocup_demo) ---
    booster_vision::BallPoseEstimator ball_estimator;
    // If you have YAML config, call ball_estimator.Init(node) as in vision pipeline.

    booster_vision::Intrinsics intr;
    // TODO: set intr from your camera calibration (fx, fy, cx, cy, ...)

    // p_eye2base must come from your robot/camera extrinsics
    booster_vision::Pose p_eye2base; 
    // TODO: set real transform (camera->robot base), same source as vision node

    ball_estimator.setIntrinsics(intr); // use your actual setter API name

    // 1) walking mode first
    loco.ChangeMode(booster::robot::RobotMode::kWalking);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    bool ready = false;
    double ball_x = 0.35, ball_y = 0.0;

    while (g_run.load() && !ready) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        auto det = DetectBallYOLO(net, frame, 0);
        if (!det.found) {
            loco.Move(0.0, 0.0, 0.2); // scan in place
            continue;
        }

        // Build detection for BallPoseEstimator
        booster_vision::DetectionRes d;
        d.bbox = det.box;

        // Camera -> robot frame using robocup_demo estimator
        auto pose_ball = ball_estimator.EstimateByColor(p_eye2base, d, frame);
        auto t = pose_ball.getTranslationVec();
        ball_x = t[0];
        ball_y = t[1];

        // walk approach control
        double ex = ball_x - 0.33;
        double ey = ball_y;
        double vx = std::clamp(0.8 * ex, -0.25, 0.25);
        double vy = std::clamp(1.0 * ey, -0.12, 0.12);
        double wz = std::clamp(1.0 * ey, -0.4, 0.4);

        loco.Move(vx, vy, wz);

        if (ball_x > 0.22 && ball_x < 0.45 && std::abs(ball_y) < 0.05) {
            loco.Move(0.0, 0.0, 0.0);
            ready = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    if (ready) {
        // 2) soccer mode then visual kick (same baseline flow)
        int rc_soccer = loco.ChangeMode(booster::robot::RobotMode::kSoccer);
        if (rc_soccer == 0) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            int ret = loco.VisualKick(true, VisualKickVersion::kV2);

            if (ret == 0) {
                const double goal_x = 4.5, goal_y = 0.0, power = 0.85;
                for (int i = 0; i < 20 && g_run.load(); ++i) {
                    brain::msg::Kick msg;
                    msg.x(ball_x);
                    msg.y(ball_y);
                    msg.goal_x(goal_x);
                    msg.goal_y(goal_y);
                    msg.power(power);
                    writer->write(&msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
                loco.VisualKick(false, VisualKickVersion::kV2);
            }
        }
    }

    pub->delete_datawriter(writer);
    participant->delete_publisher(pub);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);
    return 0;
}
