#include <booster/robot/b1/b1_loco_client.hpp>

#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    booster::robot::b1::B1LocoClient client;
    client.Init();
    float x, y, z, yaw, pitch;
    int32_t res = 0;
    std::string input;
    while (true) {
        bool need_print = false;
        std::getline(std::cin, input);
        if (!input.empty()) {
            if (input == "mp") {
                res = client.ChangeMode(booster::robot::RobotMode::kPrepare);
            } else if (input == "md") {
                res = client.ChangeMode(booster::robot::RobotMode::kDamping);
            } else if (input == "mw") {
                res = client.ChangeMode(booster::robot::RobotMode::kWalking);
            } else if (input == "mc") {
                res = client.ChangeMode(booster::robot::RobotMode::kCustom);
            } else if (input == "w") {
                x = 0.2;
                y = 0.0;
                z = 0.0;
                need_print = true;
                res = client.Move(x, y, z);
            } else if (input == "l") {
                x = 0.0;
                y = 0.0;
                z = 0.0;
                need_print = true;
                res = client.Move(x, y, z);
            } else if (input == "a") {
                x = 0.0;
                y = 0.2;
                z = 0.0;
                need_print = true;
                res = client.Move(x, y, z);
            } else if (input == "s") {
                x = -0.2;
                y = 0.0;
                z = 0.0;
                need_print = true;
                res = client.Move(x, y, z);
            }
            if (input == "d") {
                x = 0.0;
                y = -0.2;
                z = 0.0;
                need_print = true;
                res = client.Move(x, y, z);
            } else if (input == "q") {
                x = 0.0;
                y = 0.0;
                z = 1.0;
                need_print = true;
                res = client.Move(x, y, z);
            } else if (input == "e") {
                x = 0.0;
                y = 0.0;
                z = -1.0;
                need_print = true;
                res = client.Move(x, y, z);
            } else if (input == "hd") {
                yaw = 0.0;
                pitch = 1.0;

                need_print = true;
                res = client.RotateHead(pitch, yaw);
            } else if (input == "hu") {
                yaw = 0.0;
                pitch = -0.3;

                need_print = true;
                res = client.RotateHead(pitch, yaw);
            } else if (input == "hr") {
                yaw = -0.785;
                pitch = 0.0;

                need_print = true;
                res = client.RotateHead(pitch, yaw);
            } else if (input == "hl") {
                yaw = 0.785;
                pitch = 0.0;

                need_print = true;
                res = client.RotateHead(pitch, yaw);
            } else if (input == "ho") {
                yaw = 0.0;
                pitch = 0.0;

                need_print = true;
                res = client.RotateHead(pitch, yaw);
            } else if (input == "wh") {
                res = client.WaveHand(booster::robot::b1::HandAction::kHandOpen);
            } else if (input == "ch") {
                res = client.WaveHand(booster::robot::b1::HandAction::kHandClose);
            } else if (input == "ld") {
                res = client.LieDown();
            } else if (input == "gu") {
                res = client.GetUp();
            }
            
            if (need_print) {
                std::cout << "Param: " << x << " " << y << " " << z << std::endl;
                std::cout << "Head param: " << pitch << " " << yaw << std::endl;
            }
            if (res != 0) {
                std::cout << "Request failed: error = " << res << std::endl;
            }
        }
    }

    return 0;
}