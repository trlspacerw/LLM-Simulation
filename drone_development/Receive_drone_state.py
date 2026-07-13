from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()

while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)

    if msg:
        mode = mavutil.mode_string_v10(msg)

        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        state = "ARMED" if armed else "DISARMED"

        print(f"Current flight mode: {mode}, state: {state}")
        