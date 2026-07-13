from pymavlink import mavutil

# Connect to vehicle
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for heartbeat
master.wait_heartbeat()

print("connected")

while True:
    # wait for a message of type 'HEARTBEAT'
    msg = master.recv_match(type='HEARTBEAT', blocking=True)

    if msg:
        mode = mavutil.mode_string_v10(msg)
        print(f"Current flight mode: {mode}")
        