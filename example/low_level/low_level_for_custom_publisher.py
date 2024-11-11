import time
from booster_robotics_sdk_python import ChannelFactory, B1LowCmdPublisher, LowCmd, LowCmdType, MotorCmd, B1JointCnt, B1JointIndex

SLEEP_TIME = 0.002


def main():
    ChannelFactory.Instance().Init(0)
    channel_publisher = B1LowCmdPublisher()
    channel_publisher.InitChannel()
    motor_cmds = [MotorCmd() for _ in range(B1JointCnt)]

    # demo for PARALLEL cmd

    # while True:
    #     low_cmd = LowCmd()
    #     low_cmd.cmd_type = LowCmdType.PARALLEL
    #     low_cmd.motor_cmd = motor_cmds

    #     q_data = [
    #         0.00,  0.00,
    #         0.25, -1.40,  0.00, -0.50,
    #         0.25,  1.40,  0.00,  0.50,
    #         0.0,
    #         -0.1, 0.0, 0.0, 0.2, 0.137, 0.125,
    #         -0.1, 0.0, 0.0, 0.2, 0.137, 0.125,]
    #     kp_data = [
    #         5., 5.,
    #         40., 50., 20., 10.,
    #         40., 50., 20., 10.,
    #         100., 
    #         350., 350., 180., 350., 450., 450.,
    #         350., 350., 180., 350., 450., 450.,
    #     ]
    #     kd_data = [
    #         .1, .1,
    #         .5, 1.5, .2, .2,
    #         .5, 1.5, .2, .2,
    #         5.0,
    #         7.5, 7.5, 3., 5.5, 0.5, 0.5,
    #         7.5, 7.5, 3., 5.5, 0.5, 0.5,
    #     ]

    #     for i in range(B1JointCnt):
    #         low_cmd.motor_cmd[i].q = q_data[i]
    #         low_cmd.motor_cmd[i].dq = 0.0
    #         low_cmd.motor_cmd[i].tau = 0.0
    #         low_cmd.motor_cmd[i].kp = kp_data[i]
    #         low_cmd.motor_cmd[i].kd = kd_data[i]
    #         low_cmd.motor_cmd[i].weight = 0.0 # weight is not effective in custom mode

    #     channel_publisher.Write(low_cmd)
    #     time.sleep(SLEEP_TIME)


    # demo for SERIAL cmd

    while True:
        low_cmd = LowCmd()
        low_cmd.cmd_type = LowCmdType.SERIAL
        low_cmd.motor_cmd = motor_cmds

        q_data = [
            0.00,  0.00,
            0.25, -1.40,  0.00, -0.50,
            0.25,  1.40,  0.00,  0.50,
            0.0,
            -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
            -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,]
        kp_data = [
            5., 5.,
            40., 50., 20., 10.,
            40., 50., 20., 10.,
            100., 
            350., 350., 180., 350., 450., 450.,
            350., 350., 180., 350., 450., 450.,
        ]
        kd_data = [
            .1, .1,
            .5, 1.5, .2, .2,
            .5, 1.5, .2, .2,
            5.0,
            7.5, 7.5, 3., 5.5, 0.5, 0.5,
            7.5, 7.5, 3., 5.5, 0.5, 0.5,
        ]

        for i in range(B1JointCnt):
            low_cmd.motor_cmd[i].q = q_data[i]
            low_cmd.motor_cmd[i].dq = 0.0
            low_cmd.motor_cmd[i].tau = 0.0
            low_cmd.motor_cmd[i].kp = kp_data[i]
            low_cmd.motor_cmd[i].kd = kd_data[i]
            low_cmd.motor_cmd[i].weight = 0.0 # weight is not effective in custom mode
        channel_publisher.Write(low_cmd)
        time.sleep(SLEEP_TIME)


if __name__ == "__main__":
    main()
