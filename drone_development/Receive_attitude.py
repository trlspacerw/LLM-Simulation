from pymavlink import mavutil

# Connect to vehicle

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

master.wait_heartbeat()


while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True)

    if msg:
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")