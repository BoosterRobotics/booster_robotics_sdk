from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode, B1HandIndex, GripperControlMode, Position, Orientation, Posture, GripperMotionParameter, GetModeResponse, Quaternion, Frame, Transform, DexterousFingerParameter
import sys, time, random

def hand_rock(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 0
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 0
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 0
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 0
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 0
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Rock hand failed: error = {res}")

    time.sleep(0.2)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 0
    finger5_param.force = 200
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Rock hand thumb failed: error = {res}")

def hand_scissor(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 0
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 0
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 1000
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 1000
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 0
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 0
    finger5_param.force = 200
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Scissor hand failed: error = {res}")

def hand_paper(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 1000
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 1000
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 1000
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 1000
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 1000
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 1000
    finger5_param.force = 200
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Paper hand failed: error = {res}")

def hand_grasp(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 350
    finger0_param.force = 400
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 350
    finger1_param.force = 400
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 350
    finger2_param.force = 400
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 350
    finger3_param.force = 400
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 350
    finger4_param.force = 400
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 350
    finger5_param.force = 400
    finger5_param.speed = 800
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Grasp hand failed: error = {res}")

def hand_ok(client: B1LocoClient):
    # 定义一个 名为 finger_params 的数组，用于存储每个手指的参数
    finger_params = []
    # 设置每个手指的参数
    finger0_param = DexterousFingerParameter()
    finger0_param.seq = 0
    finger0_param.angle = 1000
    finger0_param.force = 200
    finger0_param.speed = 800
    finger_params.append(finger0_param)

    finger1_param = DexterousFingerParameter()
    finger1_param.seq = 1
    finger1_param.angle = 1000
    finger1_param.force = 200
    finger1_param.speed = 800
    finger_params.append(finger1_param)

    finger2_param = DexterousFingerParameter()
    finger2_param.seq = 2
    finger2_param.angle = 1000
    finger2_param.force = 200
    finger2_param.speed = 800
    finger_params.append(finger2_param)

    finger3_param = DexterousFingerParameter()
    finger3_param.seq = 3
    finger3_param.angle = 500
    finger3_param.force = 200
    finger3_param.speed = 800
    finger_params.append(finger3_param)

    finger4_param = DexterousFingerParameter()
    finger4_param.seq = 3
    finger4_param.angle = 400
    finger4_param.force = 200
    finger4_param.speed = 800
    finger_params.append(finger4_param)

    finger5_param = DexterousFingerParameter()
    finger5_param.seq = 5
    finger5_param.angle = 350
    finger5_param.force = 200
    finger5_param.speed = 1000
    finger_params.append(finger5_param)

    res = client.ControlDexterousHand(finger_params, B1HandIndex.kRightHand)
    if res != 0:
        print(f"Ok hand failed: error = {res}")

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])

    client = B1LocoClient()
    client.Init()
    x, y, z, yaw, pitch = 0.0, 0.0, 0.0, 0.0, 0.0
    res = 0
    hand_action_count = 0

    while True:
        need_print = False
        input_cmd = input().strip()
        if input_cmd:
            if input_cmd == "mp":
                res = client.ChangeMode(RobotMode.kPrepare)
            elif input_cmd == "md":
                res = client.ChangeMode(RobotMode.kDamping)
            elif input_cmd == "mw":
                res = client.ChangeMode(RobotMode.kWalking)
            elif input_cmd == 'mc':
                res = client.ChangeMode(RobotMode.kCustom)
            elif input_cmd == "stop":
                x, y, z = 0.0, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "w":
                x, y, z = 0.8, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "a":
                x, y, z = 0.0, 0.2, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "s":
                x, y, z = -0.2, 0.0, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "d":
                x, y, z = 0.0, -0.2, 0.0
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "q":
                x, y, z = 0.0, 0.0, 0.2
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "e":
                x, y, z = 0.0, 0.0, -0.2
                need_print = True
                res = client.Move(x, y, z)
            elif input_cmd == "hd":
                yaw, pitch = 0.0, 1.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "hu":
                yaw, pitch = 0.0, -0.3
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "hr":
                yaw, pitch = -0.785, 0.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "hl":
                yaw, pitch = 0.785, 0.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "ho":
                yaw, pitch = 0.0, 0.0
                need_print = True
                res = client.RotateHead(pitch, yaw)
            elif input_cmd == "mhel":
                tar_posture = Posture()
                tar_posture.position = Position(0.35, 0.25, 0.1)
                tar_posture.orientation = Orientation(-1.57, -1.57, 0.0)
                res = client.MoveHandEndEffectorV2(tar_posture, 2000, B1HandIndex.kLeftHand)
                tar_posture = Posture()
                tar_posture.position = Position(0.35, -0.25, 0.1)
                tar_posture.orientation = Orientation(1.57, -1.57, 0.0)
                res = client.MoveHandEndEffectorV2(tar_posture, 2000, B1HandIndex.kRightHand)
            elif input_cmd == "gopenl":
                motion_param = GripperMotionParameter()
                motion_param.position = 500
                motion_param.force = 100
                motion_param.speed = 100
                res = client.ControlGripper(motion_param, GripperControlMode.kPosition, B1HandIndex.kLeftHand)
            elif input_cmd == "gft":
                src = Frame.kBody
                dst = Frame.kLeftHand

                transform: Transform = Transform()
                res = client.GetFrameTransform(src, dst, transform)
                if res == 0:
                    print(f"Transform: {transform}")
            elif input_cmd == "gm":
                gm: GetModeResponse = GetModeResponse()
                res = client.GetMode(gm)
                if res == 0:
                    print(f"gm: {gm.mode}")
            elif input_cmd == "hcm-start":
                res = client.SwitchHandEndEffectorControlMode(True)
            elif input_cmd == "hcm-stop":
                res = client.SwitchHandEndEffectorControlMode(False)
            elif input_cmd == "hand-down":
                tar_posture = Posture()
                tar_posture.position = Position(0.28, -0.25, 0.05)
                tar_posture.orientation = Orientation(0.0, 0.0, 0.0)
                res = client.MoveHandEndEffector(tar_posture, 1000, B1HandIndex.kRightHand)
                time.sleep(0.3)
                hand_action_count += 1
                r_num = random.randint(0, 2)
                if r_num == 0:
                    hand_rock(client)
                elif r_num == 1:
                    hand_scissor(client)
                else:
                    hand_paper(client)
            
            elif input_cmd == "hand-up":
                tar_posture = Posture()
                tar_posture.position = Position(0.25, -0.3, 0.25)
                tar_posture.orientation = Orientation(0.0, -1.0, 0.0)
                res = client.MoveHandEndEffector(tar_posture, 1000, B1HandIndex.kRightHand)
                time.sleep(0.3)
                hand_paper(client)
            elif input_cmd == "grasp":
                hand_grasp(client)
            elif input_cmd == "ok":
                hand_ok(client)
            elif input_cmd == "paper":
                hand_paper(client)
            elif input_cmd == "scissor":
                hand_scissor(client)
            elif input_cmd == "rock":
                hand_rock(client)

            if need_print:
                print(f"Param: {x} {y} {z}")
                print(f"Head param: {pitch} {yaw}")

            if res != 0:
                print(f"Request failed: error = {res}")

if __name__ == "__main__":
    main()