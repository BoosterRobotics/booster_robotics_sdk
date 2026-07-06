#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>

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

using namespace eprosima::fastdds::dds;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;

static std::atomic<bool> g_run{true};

void sigint_handler(int) { g_run = false; }

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);
    // ---- FastDDS setup ----
    DomainParticipantQos pqos;
    pqos.name("visual_kick_no_ros_participant");
    DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (!participant) {
        std::cerr << "Failed to create participant\n";
        return 1;
    }

    TypeSupport kick_type(new brain::msg::Kick());
    kick_type.register_type(participant);

    Topic* topic = participant->create_topic(
        booster::robot::b1::kTopicKickReference, // "rt/kick_ball"
        kick_type.get_type_name(),
        TOPIC_QOS_DEFAULT);
    if (!topic) {
        std::cerr << "Failed to create topic\n";
        return 1;
    }

    Publisher* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (!publisher) {
        std::cerr << "Failed to create publisher\n";
        return 1;
    }

    DataWriter* writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr);
    if (!writer) {
        std::cerr << "Failed to create writer\n";
        return 1;
    }

    // ---- Booster loco client ----
    booster::robot::ChannelFactory::Instance()->Init(0, "lo");
    B1LocoClient loco;
    loco.Init();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    booster::robot::b1::GetModeResponse mode;
    int mode_rc = loco.GetMode(mode);
    std::cout << "[INFO] GetMode rc=" << mode_rc
              << "  mode=" << static_cast<int>(mode.ToJson()["mode"]) << "\n";

    // VisualKick is documented as a kSoccer-mode action - transition there
    // first (matches the sequence that returned rc=0 in visual_kick_version8.cpp).
    int rc_walk = loco.ChangeMode(booster::robot::RobotMode::kWalking);
    std::cout << "[INFO] ChangeMode(kWalking) rc=" << rc_walk << "\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    int rc_soccer = loco.ChangeMode(booster::robot::RobotMode::kSoccer);
    if (rc_soccer == 0) {
        std::cout << "[OK  ] Robot mode set to kSoccer successfully.\n";
    } else {
        std::cerr << "[FAIL] Could not set kSoccer mode, rc=" << rc_soccer << ". Aborting.\n";
        return rc_soccer;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    int ret = loco.VisualKick(true, VisualKickVersion::kV2); // enable policy
    if (ret != 0) {
        std::cerr << "VisualKick(true) failed: " << ret << "\n";
        return ret;
    }

    std::cout << "VisualKick enabled. Publishing rt/kick_ball ... Ctrl+C to stop.\n";

    // ---- publish loop ----
    auto t0 = std::chrono::steady_clock::now();
    const double goal_x = 4.5;
    const double goal_y = 0.0;
    double power = 0.8; // start low; calibrate gradually

    while (g_run.load()) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();

        // Example ball estimate in robot coordinates:
        double ball_x = 0.35;
        double ball_y = 0.03 * std::sin(2.0 * M_PI * 0.4 * t);

        brain::msg::Kick msg;
        msg.x(ball_x);
        msg.y(ball_y);
        msg.goal_x(goal_x);
        msg.goal_y(goal_y);
        msg.power(power);

        // Optional if needed by your controller:
        // msg.dir(0.0);
        // msg.robot_theta_to_field(0.0);

        if (writer->write(&msg) != ReturnCode_t::RETCODE_OK) {
            std::cerr << "Failed to publish kick message\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30Hz
    }

    // ---- disable policy ----
    ret = loco.VisualKick(false, VisualKickVersion::kV2);
    if (ret != 0) {
        std::cerr << "VisualKick(false) failed: " << ret << "\n";
    }

    // ---- cleanup ----
    publisher->delete_datawriter(writer);
    participant->delete_publisher(publisher);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);

    std::cout << "Stopped.\n";
    return 0;
}
