import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# takeoff altitude definition 
TAKEOFF_ALTITUDE = 50

# connect to the vehicle
vehicle = utility.mavlink_connection(device='udpin:127.0.0.1:14550')

# wait for the heartbeat message to find the system ID
vehicle.wait_heartbeat()

# inform user
print("connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# set mode to GUIDED (mode number 4 for ArduCopter)
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    dialect.MAV_CMD_DO_SET_MODE,
    0,
    utility.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4, 0, 0, 0, 0, 0
)
ack = vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
print("GUIDED mode set")

# arm the vehicle
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    dialect.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

vehicle.motors_armed_wait()
print("Vehicle armed")

# send the takeoff command
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    dialect.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, TAKEOFF_ALTITUDE
)

# inform the user about the takeoff
print("sent takeoff command to vehicle")

# check the pre-arm
while True:

    # catch GLOBAL_POSITION_INT message
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)

    # convert the message to a dictionary
    message_dict = message.to_dict()

    # get relative altitude
    relative_altitude = message_dict['relative_alt'] * 0.001  # convert from millimeters to meters

    # print out the message 
    print("Relative Altitude:", relative_altitude, "meters")

    #check if reached the target altitude
    if TAKEOFF_ALTITUDE - relative_altitude < 1:  # if within 1 meter of the target altitude

        print("Takeoff to", TAKEOFF_ALTITUDE, "meters is successful")
        
        break


# wait 10 second
print("waiting 10 seconds")
time.sleep(10)

# land the vehicle
vehicle.mav.command_long_send(
    vehicle.target_system,
    vehicle.target_component,
    dialect.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)

# inform the user about the landing 
print("sent land command to vehicle")


# check pre-arm
while True:

    # catch GLOBAL_POSITION_INT message
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname, blocking=True)

    # convert the message to a dictionary
    message_dict = message.to_dict()

    # get relative altitude
    relative_altitude = message_dict['relative_alt'] * 0.001  # convert from millimeters to meters

    # print out the message 
    print("Relative Altitude:", relative_altitude, "meters")

    # check if landed (relative altitude less than 1 meter)
    if relative_altitude < 1:
        print("landed successfully")
        break


# wait some seconds to land
time.sleep(10)

# print out that land is successful
print("landed successfully")

