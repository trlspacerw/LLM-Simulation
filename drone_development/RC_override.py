"""
RC overrides interpreted by the autopilot as manual control input from RC transmitter.
This is useful for testing the autopilot 's manual control handling.
The value 65535 means that ignore that channel.
PWM values should be between 1000 and 2000 microseconds.
This message allows you to control up to 18 channels.

Channel 1: Aileron, A > Roll left, D > Roll right
Channel 2: Elevator, W > Pitch down(Tilt forward), S > Pitch up(Tilt down)
Channel 3: Throttle, R > Increase throttle, F > Decrease throttle (Homework: Try to arm the vehicle and increase throttle to takeoff)
Channel 4: Rudder, Q > Yaw left, E > Yaw right (Homework: Try to yaw the vehicle left and right)

"""


import time 
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import getkey

def override_channels(vehicle, channels):
    """
    Send RC override message to the vehicle with the specified channel values.
    """
    # create a list of 18 channels with default value 65535 (ignore)
    temp_channels = [65535] * 18

    # update the specified channels with the provided values
    for i in range(len(temp_channels)):
        if i + 1 in channels.keys():
            temp_channels[i] = channels[i + 1]
    channels = temp_channels
    return dialect.MAVLink_rc_channels_override_message(
        vehicle.target_system,
        vehicle.target_component,
        *channels
    )

def safe_pwm(value, offset):
    value = value + offset
    if value < 1000:
        value = 1000
    if value > 2000:
        value = 2000

    return value
 
# connect to the vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")

# wait for heartbeat
vehicle.wait_heartbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# initialize channels
channels = {
    1: 1500,  # Aileron
    2: 1500,  # Elevator
    3: 1500,  # Throttle
    4: 1500   # Rudder
}

# infite loop to read keyboard input and send RC override messages
while True:

    # capture key presses
    pressed_key = getkey.getkey()

    # check pressed key
    if pressed_key == "a":
        channels[1] = safe_pwm(channels[1], -100)  # Roll left
        print("Roll left")
    elif pressed_key == "d":
        channels[1] = safe_pwm(channels[1], 100)   # Roll right
        print("Roll right")
    elif pressed_key == "w":
        channels[2] = safe_pwm(channels[2], -100)  # Pitch down (Tilt forward)
        print("Pitch down (Tilt forward)")
    elif pressed_key == "s":
        channels[2] = safe_pwm(channels[2], 100)   # Pitch up (Tilt down)
        print("Pitch up (Move backward)")
    elif pressed_key == "r":
        channels[3] = safe_pwm(channels[3], 100)   # Increase throttle
        print("Increase throttle")
    elif pressed_key == "f":
        channels[3] = safe_pwm(channels[3], -100)  # Decrease throttle
        print("Decrease throttle")
    elif pressed_key == "q":
        channels[4] = safe_pwm(channels[4], -100)  # Yaw left
        print("Yaw left")
    elif pressed_key == "e":
        channels[4] = safe_pwm(channels[4], 100)   # Yaw right
        print("Yaw right")
    elif pressed_key == "x":
        print("Exiting...")
        break

    # debug the channels
    print(channels)

    # create rc channel override message
    message = override_channels(vehicle=vehicle, channels=channels)

    # send the message to the vehicle
    vehicle.mav.send(message)

