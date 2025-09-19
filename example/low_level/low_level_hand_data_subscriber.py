from booster_robotics_sdk_python import ChannelFactory, B1LowHandDataScriber, B1LowHandTouchDataScriber
import time

def handler(hand_data_msg):
    print("Received hand message:")
    for i, data in enumerate(hand_data_msg.hand_data):
        print(f" seq:{data.seq} angle{data.angle}, force:{data.force}, current:{data.current}, status:{data.status}, temp:{data.temp}, error:{data.error}")
    print(f" hand index:{hand_data_msg.hand_index} hand type:{hand_data_msg.hand_type} ")
    print("done")

def touch_handler(touch_hand_data_msg):
    print("Received touch hand message:")
        #print(f" seq:{data.seq} angle{data.angle}, force:{data.force}, current:{data.current}, status:{data.status}, temp:{data.temp}, error:{data.error}")
    print(f" finger_one:{touch_hand_data_msg.touch_data.finger_one}")
    print(f" finger_two:{touch_hand_data_msg.touch_data.finger_two}")
    print(f" finger_three:{touch_hand_data_msg.touch_data.finger_three}")
    print(f" finger_four:{touch_hand_data_msg.touch_data.finger_four}")
    print(f" finger_five:{touch_hand_data_msg.touch_data.finger_five}")
    print(f" finger_palm:{touch_hand_data_msg.touch_data.finger_palm}")
    print(f" hand index:{touch_hand_data_msg.hand_index} hand type:{touch_hand_data_msg.hand_type} ")
    print(" -------------------------------------------------------------------------- ")
    print("done")

def main():
    ChannelFactory.Instance().Init(0)

    touch_channel_subscriber = B1LowHandTouchDataScriber(touch_handler)
    touch_channel_subscriber.InitChannel()

    channel_subscriber = B1LowHandDataScriber(handler)
    channel_subscriber.InitChannel()

    print("init handler")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
