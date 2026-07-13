import pymavlink.mavutil

# connect to the vehicle
vehicle = pymavlink.mavutil.mavlink_connection(device='udpin:127.0.0.1:14550')

# wait for the heartbeat message to find the system ID
vehicle.wait_heartbeat(timeout=5)

# debuging message 
print("connected to the vehicle")
print("Target system ID:", vehicle.target_system, "Target component ID:", vehicle.target_component)