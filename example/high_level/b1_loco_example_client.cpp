#include <booster/robot/b1/b1_loco_client.hpp>

#include <chrono>
#include <iostream>
#include <thread>

void HandRock(booster::robot::b1::B1LocoClient &client) {
    std::vector<booster::robot::b1::DexterousFingerParameter> finger_params;
    booster::robot::b1::DexterousFingerParameter finger0_param;
    finger0_param.seq_ = 0;
    finger0_param.angle_ = 0;
    finger0_param.force_ = 200;
    finger0_param.speed_ = 800;
    finger_params.push_back(finger0_param);

    booster::robot::b1::DexterousFingerParameter finger1_param;
    finger1_param.seq_ = 1;
    finger1_param.angle_ = 0;
    finger1_param.force_ = 200;
    finger1_param.speed_ = 800;
    finger_params.push_back(finger1_param);

    booster::robot::b1::DexterousFingerParameter finger2_param;
    finger2_param.seq_ = 2;
    finger2_param.angle_ = 0;
    finger2_param.force_ = 200;
    finger2_param.speed_ = 800;
    finger_params.push_back(finger2_param);

    booster::robot::b1::DexterousFingerParameter finger3_param;
    finger3_param.seq_ = 3;
    finger3_param.angle_ = 0;
    finger3_param.force_ = 200;
    finger3_param.speed_ = 800;
    finger_params.push_back(finger3_param);

    booster::robot::b1::DexterousFingerParameter finger4_param;
    finger4_param.seq_ = 4;
    finger4_param.angle_ = 0;
    finger4_param.force_ = 200;
    finger4_param.speed_ = 800;
    finger_params.push_back(finger4_param);

    int res = client.ControlDexterousHand(finger_params, booster::robot::b1::HandIndex::kRightHand);
    if (res != 0) {
        std::cout << "Rock hand failed: error = " << res << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    booster::robot::b1::DexterousFingerParameter finger5_param;
    finger5_param.seq_ = 5;
    finger5_param.angle_ = 0;
    finger5_param.force_ = 200;
    finger5_param.speed_ = 800;
    finger_params.push_back(finger5_param);

    res = client.ControlDexterousHand(finger_params, booster::robot::b1::HandIndex::kRightHand);
    if (res != 0) {
        std::cout << "Rock hand thumb failed: error = " << res << std::endl;
    }
}

void HandScissor(booster::robot::b1::B1LocoClient &client) {
    std::vector<booster::robot::b1::DexterousFingerParameter> finger_params;
    booster::robot::b1::DexterousFingerParameter finger0_param;
    finger0_param.seq_ = 0;
    finger0_param.angle_ = 0;
    finger0_param.force_ = 200;
    finger0_param.speed_ = 800;
    finger_params.push_back(finger0_param);

    booster::robot::b1::DexterousFingerParameter finger1_param;
    finger1_param.seq_ = 1;
    finger1_param.angle_ = 0;
    finger1_param.force_ = 200;
    finger1_param.speed_ = 800;
    finger_params.push_back(finger1_param);

    booster::robot::b1::DexterousFingerParameter finger2_param;
    finger2_param.seq_ = 2;
    finger2_param.angle_ = 1000;
    finger2_param.force_ = 200;
    finger2_param.speed_ = 800;
    finger_params.push_back(finger2_param);

    booster::robot::b1::DexterousFingerParameter finger3_param;
    finger3_param.seq_ = 3;
    finger3_param.angle_ = 1000;
    finger3_param.force_ = 200;
    finger3_param.speed_ = 800;
    finger_params.push_back(finger3_param);

    booster::robot::b1::DexterousFingerParameter finger4_param;
    finger4_param.seq_ = 4;
    finger4_param.angle_ = 0;
    finger4_param.force_ = 200;
    finger4_param.speed_ = 800;
    finger_params.push_back(finger4_param);

    booster::robot::b1::DexterousFingerParameter finger5_param;
    finger5_param.seq_ = 5;
    finger5_param.angle_ = 0;
    finger5_param.force_ = 200;
    finger5_param.speed_ = 800;
    finger_params.push_back(finger5_param);

    int res = client.ControlDexterousHand(finger_params, booster::robot::b1::HandIndex::kRightHand);
    if (res != 0) {
        std::cout << "Scissor hand failed: error = " << res << std::endl;
    }
}

void HandPaper(booster::robot::b1::B1LocoClient &client) {
    std::vector<booster::robot::b1::DexterousFingerParameter> finger_params;
    booster::robot::b1::DexterousFingerParameter finger0_param;
    finger0_param.seq_ = 0;
    finger0_param.angle_ = 1000;
    finger0_param.force_ = 200;
    finger0_param.speed_ = 800;
    finger_params.push_back(finger0_param);

    booster::robot::b1::DexterousFingerParameter finger1_param;
    finger1_param.seq_ = 1;
    finger1_param.angle_ = 1000;
    finger1_param.force_ = 200;
    finger1_param.speed_ = 800;
    finger_params.push_back(finger1_param);

    booster::robot::b1::DexterousFingerParameter finger2_param;
    finger2_param.seq_ = 2;
    finger2_param.angle_ = 1000;
    finger2_param.force_ = 200;
    finger2_param.speed_ = 800;
    finger_params.push_back(finger2_param);

    booster::robot::b1::DexterousFingerParameter finger3_param;
    finger3_param.seq_ = 3;
    finger3_param.angle_ = 1000;
    finger3_param.force_ = 200;
    finger3_param.speed_ = 800;
    finger_params.push_back(finger3_param);

    booster::robot::b1::DexterousFingerParameter finger4_param;
    finger4_param.seq_ = 4;
    finger4_param.angle_ = 1000;
    finger4_param.force_ = 200;
    finger4_param.speed_ = 800;
    finger_params.push_back(finger4_param);

    booster::robot::b1::DexterousFingerParameter finger5_param;
    finger5_param.seq_ = 5;
    finger5_param.angle_ = 1000;
    finger5_param.force_ = 200;
    finger5_param.speed_ = 800;
    finger_params.push_back(finger5_param);

    int res = client.ControlDexterousHand(finger_params, booster::robot::b1::HandIndex::kRightHand);
    if (res != 0) {
        std::cout << "Paper hand failed: error = " << res << std::endl;
    }
}

void HandGrasp(booster::robot::b1::B1LocoClient &client) {
    std::vector<booster::robot::b1::DexterousFingerParameter> finger_params;
    booster::robot::b1::DexterousFingerParameter finger0_param;
    finger0_param.seq_ = 0;
    finger0_param.angle_ = 350;
    finger0_param.force_ = 400;
    finger0_param.speed_ = 800;
    finger_params.push_back(finger0_param);

    booster::robot::b1::DexterousFingerParameter finger1_param;
    finger1_param.seq_ = 1;
    finger1_param.angle_ = 350;
    finger1_param.force_ = 400;
    finger1_param.speed_ = 800;
    finger_params.push_back(finger1_param);

    booster::robot::b1::DexterousFingerParameter finger2_param;
    finger2_param.seq_ = 2;
    finger2_param.angle_ = 350;
    finger2_param.force_ = 400;
    finger2_param.speed_ = 800;
    finger_params.push_back(finger2_param);

    booster::robot::b1::DexterousFingerParameter finger3_param;
    finger3_param.seq_ = 3;
    finger3_param.angle_ = 350;
    finger3_param.force_ = 400;
    finger3_param.speed_ = 800;
    finger_params.push_back(finger3_param);

    booster::robot::b1::DexterousFingerParameter finger4_param;
    finger4_param.seq_ = 4;
    finger4_param.angle_ = 350;
    finger4_param.force_ = 400;
    finger4_param.speed_ = 800;
    finger_params.push_back(finger4_param);

    booster::robot::b1::DexterousFingerParameter finger5_param;
    finger5_param.seq_ = 5;
    finger5_param.angle_ = 350;
    finger5_param.force_ = 400;
    finger5_param.speed_ = 800;
    finger_params.push_back(finger5_param);

    int res = client.ControlDexterousHand(finger_params, booster::robot::b1::HandIndex::kRightHand);
    if (res != 0) {
        std::cout << "Grasp hand failed: error = " << res << std::endl;
    }
}

void HandOk(booster::robot::b1::B1LocoClient &client) {
    std::vector<booster::robot::b1::DexterousFingerParameter> finger_params;
    booster::robot::b1::DexterousFingerParameter finger0_param;
    finger0_param.seq_ = 0;
    finger0_param.angle_ = 1000;
    finger0_param.force_ = 200;
    finger0_param.speed_ = 800;
    finger_params.push_back(finger0_param);

    booster::robot::b1::DexterousFingerParameter finger1_param;
    finger1_param.seq_ = 1;
    finger1_param.angle_ = 1000;
    finger1_param.force_ = 200;
    finger1_param.speed_ = 800;
    finger_params.push_back(finger1_param);

    booster::robot::b1::DexterousFingerParameter finger2_param;
    finger2_param.seq_ = 2;
    finger2_param.angle_ = 1000;
    finger2_param.force_ = 200;
    finger2_param.speed_ = 800;
    finger_params.push_back(finger2_param);

    booster::robot::b1::DexterousFingerParameter finger3_param;
    finger3_param.seq_ = 3;
    finger3_param.angle_ = 500;
    finger3_param.force_ = 200;
    finger3_param.speed_ = 800;
    finger_params.push_back(finger3_param);

    booster::robot::b1::DexterousFingerParameter finger4_param;
    finger4_param.seq_ = 4;
    finger4_param.angle_ = 400;
    finger4_param.force_ = 200;
    finger4_param.speed_ = 800;
    finger_params.push_back(finger4_param);

    booster::robot::b1::DexterousFingerParameter finger5_param;
    finger5_param.seq_ = 5;
    finger5_param.angle_ = 80;
    finger5_param.force_ = 200;
    finger5_param.speed_ = 1000;
    finger_params.push_back(finger5_param);

    int res = client.ControlDexterousHand(finger_params, booster::robot::b1::HandIndex::kRightHand);
    if (res != 0) {
        std::cout << "Ok hand failed: error = " << res << std::endl;
    }
}

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
    int32_t hand_action_count = 0;
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
            } else if (input == "mhel") {
                booster::robot::Posture tar_posture;
                tar_posture.position_ = booster::robot::Position(0.35, 0.25, 0.1);
                tar_posture.orientation_ = booster::robot::Orientation(-1.57, -1.57, 0.);

                res = client.MoveHandEndEffectorV2(
                    tar_posture, 2000, booster::robot::b1::HandIndex::kLeftHand);

                tar_posture.position_ = booster::robot::Position(0.35, -0.2, 0.1);
                tar_posture.orientation_ = booster::robot::Orientation(1.57, -1.57, 0.);
                res = client.MoveHandEndEffectorV2(
                    tar_posture, 2000, booster::robot::b1::HandIndex::kRightHand);
            } else if (input == "gopenl") {
                booster::robot::b1::GripperMotionParameter motion_param;
                motion_param.position_ = 500;
                motion_param.force_ = 100;
                motion_param.speed_ = 100;

                res = client.ControlGripper(
                    motion_param, booster::robot::b1::GripperControlMode::kPosition,
                    booster::robot::b1::HandIndex::kLeftHand);
            } else if (input == "gft") {
                booster::robot::Frame src = booster::robot::Frame::kBody;
                booster::robot::Frame dst = booster::robot::Frame::kRightHand;
                booster::robot::Transform transform;

                res = client.GetFrameTransform(src, dst, transform);
                if (res == 0) {
                    std::cout << "pos:" << transform.position_.x_ << " " << transform.position_.y_
                              << " " << transform.position_.z_ << std::endl;
                    std::cout << "ori:" << transform.orientation_.x_ << " " << transform.orientation_.y_
                              << " " << transform.orientation_.z_ << " "
                              << transform.orientation_.w_ << std::endl;
                }
            } else if (input == "hcm-start") {
                res = client.SwitchHandEndEffectorControlMode(true);
            } else if (input == "hcm-stop") {
                res = client.SwitchHandEndEffectorControlMode(false);
            } else if (input == "hand-down") {
                booster::robot::Posture tar_posture;
                tar_posture.position_ = booster::robot::Position(0.28, -0.25, 0.05);
                tar_posture.orientation_ = booster::robot::Orientation(0., 0., 0.);

                res = client.MoveHandEndEffector(
                    tar_posture, 1000, booster::robot::b1::HandIndex::kRightHand);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                hand_action_count++;
                int random = rand() % 3;
                if (random == 0) {
                    HandRock(client);
                } else if (random == 1) {
                    HandScissor(client);
                } else {
                    HandPaper(client);
                }
            } else if (input == "hand-up") {
                booster::robot::Posture tar_posture;
                tar_posture.position_ = booster::robot::Position(0.25, -0.3, 0.25);
                tar_posture.orientation_ = booster::robot::Orientation(0., -1.0, 0.);

                res = client.MoveHandEndEffector(
                    tar_posture, 1000, booster::robot::b1::HandIndex::kRightHand);

                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                HandPaper(client);
            } else if (input == "grasp") {
                HandGrasp(client);
            } else if (input == "ok") {
                HandOk(client);
            } else if (input == "paper") {
                HandPaper(client);
            } else if (input == "scissor") {
                HandScissor(client);
            } else if (input == "rock") {
                HandRock(client);
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