#!/usr/bin/env python3

import pymavlink.mavutil as mavutil
import time
import sys

def land_aircraft():
    try:
        # 连接到PX4
        print("Connecting to PX4...")
        master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

        # 等待连接
        master.wait_heartbeat()
        print(f'Connected to PX4 (System ID: {master.target_system})')

        # 发送着陆命令
        print("Sending land command...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            21,  # MAV_CMD_NAV_LAND
            0,   # confirmation
            0, 0, 0, 0, 0, 0, 0  # parameters
        )

        print("Land command sent successfully!")

        # 等待确认
        time.sleep(1)

        # 检查命令是否被接受
        while True:
            msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                if msg.command == 21:  # MAV_CMD_NAV_LAND
                    if msg.result == 0:  # MAV_RESULT_ACCEPTED
                        print("Land command accepted!")
                    else:
                        print(f"Land command rejected: {msg.result}")
                    break
            else:
                print("No response from aircraft")
                break

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    land_aircraft()
