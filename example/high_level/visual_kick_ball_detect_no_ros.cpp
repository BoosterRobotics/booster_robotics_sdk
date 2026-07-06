// ============================================================================
// Visual Kick Ball Detection using YOLO (No ROS)
// ============================================================================
// This program detects a ball using a YOLO neural network via OpenCV,
// estimates the ball's position in robot frame, and commands a B1 robot to kick it.
// Uses FastDDS for publishing kick commands (no ROS dependency required).
// ============================================================================

// Standard library includes for time, math, signals, I/O, threading, and containers
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>

// ============================================================================
// Conditional OpenCV Inclusion
// ============================================================================
// Check if OpenCV and its DNN module are available at compile time.
// If present, compile with YOLO detection capabilities; otherwise disable it.
#if __has_include(<opencv2/opencv.hpp>) && __has_include(<opencv2/dnn.hpp>)
#define BOOSTER_HAS_OPENCV 1
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#else
#define BOOSTER_HAS_OPENCV 0
#endif

// ============================================================================
// FastDDS (Fast Data Distribution Service) Headers
// ============================================================================
// FastDDS provides DDS-based communication for publishing kick commands
// without requiring ROS infrastructure.
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

// ============================================================================
// Booster Robotics SDK Headers
// ============================================================================
// Includes for robot control (B1 locomotion client), communication channels,
// and message definitions for kick commands.
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/idl/b1/Kick.h>

// Using declarations to simplify namespace references in code
using namespace eprosima::fastdds::dds;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;

// ============================================================================
// Global State and Signal Handling
// ============================================================================
// Atomic flag to gracefully stop the main loop on SIGINT (Ctrl+C)
static std::atomic<bool> g_run{true};

/// Signal handler function: sets g_run to false when SIGINT is received
void sigint_handler(int) { g_run = false; }

// ============================================================================
// BOOSTER_HAS_OPENCV Section: Detection and Pose Estimation
// ============================================================================
#if BOOSTER_HAS_OPENCV

/// Structure to hold ball detection results from YOLO inference
struct Det {
    bool found{false};          ///< True if a ball was detected in the frame
    float conf{0.f};            ///< Confidence score [0.0, 1.0] of the detection
    cv::Rect box;               ///< Bounding box (x, y, width, height) of the detected ball
};

// ============================================================================
// YOLO Ball Detection Function
// ============================================================================
/// Detects a ball in a video frame using a pre-loaded YOLO neural network.
///
/// This function:
/// 1. Converts the input frame to a normalized blob suitable for YOLO inference
/// 2. Runs the YOLO neural network forward pass on the blob
/// 3. Parses the raw predictions to extract class IDs, confidences, and bounding boxes
/// 4. Filters predictions by confidence threshold and ball class ID
/// 5. Applies Non-Maximum Suppression (NMS) to remove overlapping detections
/// 6. Returns the best (highest confidence) detection result
///
/// @param net           Reference to a pre-loaded YOLO neural network model
/// @param frame         Input video frame (BGR image format from OpenCV)
/// @param ball_class_id Class ID assigned to the ball in the YOLO model (default: 0)
/// @return              Det structure containing detection results (found flag, confidence, bounding box)
static Det DetectBallYOLO(cv::dnn::Net& net, const cv::Mat& frame, int ball_class_id = 0) {
    Det out;
    
    // YOLO model and inference hyperparameters
    constexpr int IN = 640;             // YOLO model input resolution: 640x640 pixels
    constexpr float CONF = 0.35f;       // Confidence threshold for filtering detections (35%)
    constexpr float NMS = 0.45f;        // Non-Maximum Suppression threshold (45%)

    // Step 1: Prepare input frame as a normalized blob for the neural network
    // - Scale pixel values to [0, 1] by dividing by 255.0
    // - Resize to 640x640 (YOLO input size)
    // - Normalize with no mean subtraction (Scalar()) and swapRB=true for BGR->RGB conversion
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0/255.0, cv::Size(IN, IN), cv::Scalar(), true, false);
    net.setInput(blob);

    // Step 2: Run neural network forward pass and collect output from all detection layers
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return out;  // Return early if no outputs produced

    // Step 3: Extract and reshape predictions if needed
    cv::Mat pred = outputs[0];
    // YOLO output shape is typically (1, num_predictions, 5+num_classes)
    // Reshape to (num_predictions, 5+num_classes) for easier row-wise iteration
    if (pred.dims == 3 && pred.size[0] == 1) pred = pred.reshape(1, pred.size[1]); // Nx(5+C)

    // Vectors to accumulate valid detections before NMS
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;

    // Step 4: Parse predictions and filter by confidence and class
    const int W = frame.cols, H = frame.rows;  // Original frame dimensions
    for (int i = 0; i < pred.rows; ++i) {
        const float* d = pred.ptr<float>(i);  // Pointer to i-th prediction row
        
        // Extract objectness score (d[4]): confidence that the cell contains an object
        float obj = d[4];
        if (obj < 1e-6f) continue;  // Skip predictions with negligible objectness

        // Find the class with the highest confidence score among all classes
        int cls = -1; 
        float cls_conf = 0.f;
        for (int c = 5; c < pred.cols; ++c) {
            if (d[c] > cls_conf) { 
                cls_conf = d[c]; 
                cls = c - 5;  // Convert column index to class ID (class 0 at column 5)
            }
        }
        
        // Final confidence = objectness * class_confidence
        float conf = obj * cls_conf;
        
        // Skip if confidence is below threshold or predicted class is not the ball
        if (conf < CONF || cls != ball_class_id) continue;

        // Step 5: Extract and denormalize bounding box coordinates
        // YOLO predicts normalized values: center_x, center_y, width, height (each in [0, 1])
        float cx = d[0], cy = d[1], w = d[2], h = d[3];
        
        // Scale coordinates back to original frame dimensions
        float sx = float(W)/IN, sy = float(H)/IN;
        
        // Convert from center format (cx, cy, w, h) to top-left format (x, y, w, h) for cv::Rect
        cv::Rect r(int((cx - 0.5f*w)*sx), int((cy - 0.5f*h)*sy), int(w*sx), int(h*sy));
        
        // Clip the bounding box to stay within frame boundaries
        r &= cv::Rect(0, 0, W, H);
        
        // Only keep boxes with positive area
        if (r.area() > 0) { boxes.push_back(r); confs.push_back(conf); }
    }

    // Step 6: Apply Non-Maximum Suppression to remove overlapping detections
    // NMS keeps only the detections with highest confidence among overlapping boxes
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, confs, CONF, NMS, keep);
    
    // Extract the best detection (first in NMS result = highest confidence)
    if (!keep.empty()) {
        int k = keep[0];  // Index of the best detection
        out.found = true;
        out.conf = confs[k];
        out.box = boxes[k];
    }
    return out;
}

// ============================================================================
// Ball Pose Estimation Function
// ============================================================================
/// Estimates ball position in robot frame from image-space bounding box.
///
/// This function converts a 2D bounding box detection in the camera frame
/// to an estimated 3D ball position relative to the robot. The estimation
/// uses heuristic formulas rather than precise camera calibration, making it
/// suitable for short-range approach control (~0.2-0.9m).
///
/// Heuristics:
/// - Horizontal position in image maps to lateral (left/right) robot offset
/// - Bounding box height correlates with forward distance (larger box = closer)
/// - Results are clamped to physically realistic ranges for the robot
///
/// @param box       Bounding box from YOLO detection [x, y, width, height]
/// @param frame_size Dimensions of the video frame [width, height]
/// @param ball_x    Output: estimated forward distance in meters
/// @param ball_y    Output: estimated lateral offset in meters
/// @return          true if estimation successful; false if input is invalid
static bool EstimateBallPoseFromBox(const cv::Rect& box, const cv::Size& frame_size, double& ball_x, double& ball_y) {
    // ========================================================================
    // Pose Estimation Constants (Heuristic Parameters)
    // ========================================================================
    // These constants map image-space coordinates to robot frame distances
    constexpr double kYawGainFromPixels = 0.6;         // Horizontal pixel offset -> lateral meters gain
    constexpr double kMaxLateralMeters = 0.25;         // Maximum lateral distance the robot can move
    constexpr double kBaseForwardMeters = 0.12;        // Minimum forward distance bias
    constexpr double kForwardScaleFromHeight = 0.015;  // Inverse scaling: larger box height = closer distance
    constexpr double kMinForwardMeters = 0.18;         // Minimum forward distance (closest kick position)
    constexpr double kMaxForwardMeters = 0.90;         // Maximum forward distance (farthest kick position)

    // ========================================================================
    // Input Validation
    // ========================================================================
    // Ensure all input values are positive and valid
    if (box.width <= 0 || box.height <= 0 || frame_size.width <= 0 || frame_size.height <= 0) {
        return false;
    }

    // ========================================================================
    // Compute Normalized Image Coordinates
    // ========================================================================
    // Calculate the horizontal center position of the bounding box in pixel coordinates
    const double center_x = box.x + 0.5 * box.width;
    
    // Normalize to [-0.5, 0.5] range where 0 = image center, -0.5 = left edge, +0.5 = right edge
    const double normalized_x = center_x / static_cast<double>(frame_size.width) - 0.5;
    
    // Normalize bounding box height to [0, 1] range relative to frame height
    // Larger normalized_h indicates ball occupies more of image (closer to camera)
    const double normalized_h = box.height / static_cast<double>(frame_size.height);
    if (normalized_h <= 0.0) {
        return false;  // Invalid height
    }

    // ========================================================================
    // Estimate Lateral Position (ball_y: left/right offset)
    // ========================================================================
    // Map normalized horizontal image coordinate to lateral robot distance
    // Formula: ball_y = -kYawGainFromPixels * normalized_x
    // Negative sign convention: 
    //   - If ball is on right side of image (normalized_x > 0) -> positive y (right offset)
    //   - If ball is on left side of image (normalized_x < 0) -> negative y (left offset)
    // Clamp result to [-kMaxLateralMeters, kMaxLateralMeters] for safe robot movement
    ball_y = std::clamp(-kYawGainFromPixels * normalized_x, -kMaxLateralMeters, kMaxLateralMeters);
    
    // ========================================================================
    // Estimate Forward Position (ball_x: forward/backward distance)
    // ========================================================================
    // Inverse relationship: larger bounding box (normalized_h) -> closer distance (smaller x)
    // Formula: ball_x = kBaseForwardMeters + kForwardScaleFromHeight / normalized_h
    // The division creates an inverse mapping: as box gets larger, the denominator dominates less
    // Clamp to valid kick range [kMinForwardMeters, kMaxForwardMeters]
    ball_x = std::clamp(kBaseForwardMeters + kForwardScaleFromHeight / normalized_h, kMinForwardMeters, kMaxForwardMeters);
    return true;
}

#endif  // End of BOOSTER_HAS_OPENCV section

// ============================================================================
// Main Program Entry Point
// ============================================================================
int main(int argc, char* argv[]) {
#if !BOOSTER_HAS_OPENCV
    // ========================================================================
    // Error Path: OpenCV Not Available
    // ========================================================================
    // This program requires OpenCV with DNN module for YOLO inference.
    // If OpenCV is not available, print error and exit.
    (void)argc;  // Suppress unused variable warning
    (void)argv;  // Suppress unused variable warning
    std::cerr << "OpenCV headers are unavailable; build/install OpenCV to use visual_kick_ball_detect_no_ros.\n";
    return 1;
#else
    // ========================================================================
    // Command Line Argument Parsing
    // ========================================================================
    // Expected usage: program networkInterface cameraIndex yolo.onnx
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " networkInterface cameraIndex yolo.onnx\n";
        return 1;
    }

    // Setup signal handler to catch Ctrl+C and gracefully shutdown
    std::signal(SIGINT, sigint_handler);

    // Extract and parse command line arguments
    std::string net_if = argv[1];           // Network interface name (e.g., "eth0")
    int cam_idx = std::stoi(argv[2]);       // Camera device index (0, 1, 2, ...)
    std::string yolo_path = argv[3];        // Path to YOLO ONNX model file

    // ========================================================================
    // FastDDS Initialization
    // ========================================================================
    // Configure and create a DDS domain participant for inter-process communication
    DomainParticipantQos pqos;
    pqos.name("visual_kick_yolo_pose_estimator_no_ros_participant");
    auto* participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (!participant) return 1;  // Exit if participant creation failed

    // Register the Kick message type with the DDS participant
    TypeSupport kick_type(new brain::msg::Kick());
    kick_type.register_type(participant);

    // Create DDS topic for publishing kick commands
    // kTopicKickReference is the topic name constant from the SDK
    Topic* topic = participant->create_topic(
        booster::robot::b1::kTopicKickReference, kick_type.get_type_name(), TOPIC_QOS_DEFAULT);
    
    // Create publisher with default QoS settings
    Publisher* pub = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    
    // Create data writer to publish kick messages
    DataWriter* writer = pub ? pub->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr) : nullptr;
    
    // Verify all DDS components were created successfully
    if (!topic || !pub || !writer) return 1;

    // ========================================================================
    // Robot Client Initialization
    // ========================================================================
    // Initialize communication channel to the robot via specified network interface
    booster::robot::ChannelFactory::Instance()->Init(0, net_if);
    
    // Create and initialize the B1 locomotion client for robot control
    B1LocoClient loco; 
    loco.Init();
    
    // Wait for robot to stabilize after initialization
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ========================================================================
    // Camera and YOLO Model Initialization
    // ========================================================================
    // Open video capture from the specified camera device
    cv::VideoCapture cap(cam_idx);
    if (!cap.isOpened()) return 1;  // Exit if camera failed to open

    // Load pre-trained YOLO neural network model from ONNX file
    cv::dnn::Net net = cv::dnn::readNetFromONNX(yolo_path);
    
    // Configure network to use OpenCV backend for better portability
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    
    // Configure network to run on CPU (not GPU)
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // ========================================================================
    // Phase 1: Walking Mode - Robot Approaches the Ball
    // ========================================================================
    // Switch robot to walking mode for locomotion and ball approach
    loco.ChangeMode(booster::robot::RobotMode::kWalking);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    bool ready = false;                     // Flag: true when robot is positioned for kicking
    double ball_x = 0.35, ball_y = 0.0;    // Current estimated ball position in robot frame

    // Main detection and approach loop
    while (g_run.load() && !ready) {
        // ====================================================================
        // Capture Camera Frame
        // ====================================================================
        cv::Mat frame;
        cap >> frame;  // Read frame from camera
        if (frame.empty()) continue;  // Skip if frame capture failed

        // ====================================================================
        // Ball Detection Phase
        // ====================================================================
        // Run YOLO detection on the current frame
        auto det = DetectBallYOLO(net, frame, 0);  // 0 = ball class ID
        
        if (!det.found) {
            // Ball not detected: rotate in place to scan environment
            loco.Move(0.0, 0.0, 0.2); // vx=0, vy=0, wz=0.2 (yaw velocity)
            continue;
        }

        // ====================================================================
        // Ball Pose Estimation Phase
        // ====================================================================
        // Convert bounding box to estimated 3D position in robot frame
        if (!EstimateBallPoseFromBox(det.box, frame.size(), ball_x, ball_y)) {
            continue;  // Pose estimation failed; skip this frame
        }

        // ====================================================================
        // Walk-Approach Control
        // ====================================================================
        // Target position: robot should be 0.33 meters in front of the ball
        double ex = ball_x - 0.33;      // Position error in forward direction (meters)
        double ey = ball_y;              // Position error in lateral direction (meters)
        
        // Proportional control: map position errors to velocity commands
        // Velocity gains tuned for stable approach (0.8 for forward, 1.0 for lateral/yaw)
        double vx = std::clamp(0.8 * ex, -0.25, 0.25);      // Forward velocity [m/s]
        double vy = std::clamp(1.0 * ey, -0.12, 0.12);      // Lateral velocity [m/s]
        double wz = std::clamp(1.0 * ey, -0.4, 0.4);        // Yaw (angular) velocity [rad/s]

        // Send movement command to robot
        loco.Move(vx, vy, wz);

        // ====================================================================
        // Check if Robot is Ready for Kicking
        // ====================================================================
        // Verify robot is positioned close enough and centered relative to ball
        // Acceptable range: x in [0.22, 0.45]m and y within ±0.05m
        if (ball_x > 0.22 && ball_x < 0.45 && std::abs(ball_y) < 0.05) {
            loco.Move(0.0, 0.0, 0.0);  // Stop all movement
            ready = true;               // Signal that robot is ready for kicking phase
        }

        // Frame rate: ~25 fps (40 ms per iteration)
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    // ========================================================================
    // Phase 2: Soccer Mode and Visual Kick Execution
    // ========================================================================
    if (ready) {
        // Switch to soccer mode for specialized kicking mechanics
        int rc_soccer = loco.ChangeMode(booster::robot::RobotMode::kSoccer);
        if (rc_soccer == 0) {  // rc_soccer = 0 means mode change successful
            // Wait for robot to stabilize in soccer mode
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // Initiate visual kick algorithm (Version 2 with improved accuracy)
            // enable=true: activate visual kick; VisualKickVersion::kV2 specifies algorithm version
            int ret = loco.VisualKick(true, VisualKickVersion::kV2);

            if (ret == 0) {  // ret = 0 means visual kick initialization successful
                // ============================================================
                // Publish Kick Commands via DDS
                // ============================================================
                // Define target goal location and kicking power
                const double goal_x = 4.5;      // Target X position: 4.5 meters forward (goal)
                const double goal_y = 0.0;      // Target Y position: 0 meters lateral (straight)
                const double power = 0.85;      // Kicking power: 85% of maximum
                
                // Publish kick commands repeatedly for ~660ms (20 * 33ms per iteration)
                // Allows robot time to execute the visual kick sequence with targeting
                for (int i = 0; i < 20 && g_run.load(); ++i) {
                    // Create and populate Kick message
                    brain::msg::Kick msg;
                    msg.x(ball_x);          // Current estimated ball X position [meters]
                    msg.y(ball_y);          // Current estimated ball Y position [meters]
                    msg.goal_x(goal_x);     // Target goal X position [meters]
                    msg.goal_y(goal_y);     // Target goal Y position [meters]
                    msg.power(power);       // Kick power: normalized [0, 1] range
                    
                    // Publish message to DDS topic for other processes/robot components
                    writer->write(&msg);
                    
                    // Publish rate: 30 Hz (33 ms per message)
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
                
                // Disable visual kick mode after kicking sequence completes
                loco.VisualKick(false, VisualKickVersion::kV2);
            }
        }
    }

    // ========================================================================
    // Cleanup and Resource Deallocation
    // ========================================================================
    // Gracefully shut down FastDDS resources in correct order
    // (always delete in reverse order of creation)
    pub->delete_datawriter(writer);
    participant->delete_publisher(pub);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);
    
    return 0;
#endif
}
