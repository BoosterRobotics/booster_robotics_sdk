from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, RobotMode, B1HandIndex, GripperControlMode, Position, Orientation, Posture, GripperMotionParameter, Quaternion, Frame, Transform
import sys

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        sys.exit(-1)

    ChannelFactory.Instance().Init(0, sys.argv[1])

    client = B1LocoClient()
    client.Init()
    x, y, z, yaw, pitch = 0.0, 0.0, 0.0, 0.0, 0.0
    res = 0

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
                tar_posture.orientation = Orientation(0.0, 0.0, 0.0)
                res = client.MoveHandEndEffector(tar_posture, 2000, B1HandIndex.kLeftHand)
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

            if need_print:
                print(f"Param: {x} {y} {z}")
                print(f"Head param: {pitch} {yaw}")

            if res != 0:
                print(f"Request failed: error = {res}")

if __name__ == "__main__":
    main()