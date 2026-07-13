from pymavlink import mavutil

# Connect to vehicle
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait for heartbeat
master.wait_heartbeat()

print(
    f"Connected to system {master.target_system}, "
    f"component {master.target_component}"
)

while True:
    msg = master.recv_match(blocking=True)

    if msg:
        print(msg)