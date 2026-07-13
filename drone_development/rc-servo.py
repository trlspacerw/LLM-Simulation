"""
    It may be desired to execute certain tasks on a system based on RC channels.
    For example, controlling on-board flight computer software task using an RC transmitter.
    Or you might want to do a certain task based on servo output channels.
    For example, warn user if servo outputs are inconsistent, too high, at certain value, range, etc.
    RC_CHANNELS message contains 18 RC input channels.
    These values can be received from RC transmitter or can be overridden with a MAVLink message.
    SERVO_OUTPUT_RAW message contains 16 output servo channels from autopilot.
    Each channel corresponds a PWM value between 1000-2000 microseconds.

    https://mavlink.io/en/messages/common.html#RC_CHANNELS
    https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW
"""

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import time


def task_1():
    for i in range(5):
        print("Task 1 executed")
        time.sleep(0.1)

def task_2():
    for i in range(3):
        print("Task 2 executed")
        time.sleep(0.1)

# connect to the vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14551")

# wait for a heartbeat
vehicle.wait_heartbeat()

# inform the user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

"""
# infite loop to receive messages
while True:

    # capture RC_CHANNELS message
    message = vehicle.recv_match(type=dialect.MAVLink_rc_channels_message.msgname, blocking=True)

    # convert message to a dictionary
    message = message.to_dict()

    # print the values of RC channels
    # print("Channel 16 PWM value:", message["chan16_raw"])

    # check 16th channel's PWM value
    if 1200 < message["chan16_raw"] < 1500:

        # call task 1
        task_1()

        break

    elif 1500 < message["chan16_raw"] < 1700:

        # call task 2
        task_2()

        break

    else:
        
        # do nothing
        print("Invalid channel 16 PWM value:", message["chan16_raw"])

"""

# infote loop 
while True:

    # capture SERVO_OUTPUT_RAW message
    message = vehicle.recv_match(type=dialect.MAVLink_servo_output_raw_message.msgname, blocking=True)

    # convert message to a dictionary
    message = message.to_dict()

    # print the values of servo output channels
    # print(message)
    print("Motor 1 :", message["servo1_raw"],
          "Motor 2 :", message["servo2_raw"],
          "Motor 3 :", message["servo3_raw"],
          "Motor 4 :", message["servo4_raw"])