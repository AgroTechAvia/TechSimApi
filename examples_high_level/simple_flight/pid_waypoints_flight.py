from agrotechsimapi import HighLevelSimClient, PID

import time

ip = "127.0.0.1"
port = "1233"


def main():
    # Configure all PID controllers explicitly.
    pid_pos_x = PID(kp=1.85, ki=0.0, kd=1.5, max_control=0.2, i_limit=0.1)
    pid_pos_y = PID(kp=1.85, ki=0.0, kd=1.5, max_control=0.2, i_limit=0.1)

    pid_vel_pitch = PID(kp=3.15, ki=0.0, kd=3.4, max_control=2.0, i_limit=0.0033)
    pid_vel_roll = PID(kp=3.15, ki=0.0, kd=3.4, max_control=2.0, i_limit=0.0033)

    pid_yaw = PID(kp=20.0, ki=0.0, kd=0.4, max_control=1.0, i_limit=None)

    pid_height = PID(kp=100, ki=0.0, kd=0.0, i_limit=1.0, processing_func=lambda x: ((2 / (1 + (2.7 ** (-x * 12.0)))) - 1) * 1000)

    client = HighLevelSimClient(
        drone_name="AGRO",
        #pid_pos_x=pid_pos_x,
        #pid_pos_y=pid_pos_y,
        #pid_vel_pitch=pid_vel_pitch,
        #pid_vel_roll=pid_vel_roll,
        #pid_yaw=pid_yaw,
        #pid_height=pid_height,
    )

    try:
        client.connect(ip, port)
        client.armDrone()
        client.altholdOn()

        # Flight plan:
        # 1) Take off
        # 2) Set height 1.5
        # 3) Fly in odom frame to [3.0, 0.1]
        # 4) Set yaw to 0.75
        # 5) Fly in odom frame to [0.0, 0.0]
        # 6) Land
        print("Takeoff:", client.takeoff())
        print("Set height 1.5:", client.setHeight(1.5))

        # Reset odometry origin before waypoint mission.
        print("Reset odometry:", client.setZeroOdomOpticflow())
        time.sleep(0.5)

        print("Go to [3.0, 0.1] odom:", client.gotoXYodom(3.0, 0.1))
        print("Set yaw 0.75:", client.setYaw(0.75))
        print("Go to [5.0, 3.0] odom:", client.gotoXYodom(5.0, 3.0))
        print("Go to [0.0, 0.0] odom:", client.gotoXYodom(0.0, 0.0))
        print("Set yaw 0.75:", client.setYaw(0.0))

        print("Landing:", client.boarding())
        time.sleep(3.0)
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
