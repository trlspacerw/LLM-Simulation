from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()

gps_status = {
    0: "No GPS",
    1: "No Fix",
    2: "2D Fix",
    3: "3D Fix",
    4: "DGPS",
    5: "RTK Float",
    6: "RTK Fixed"
}

while True:
    msg = master.recv_match(
        type='GPS_RAW_INT',
        blocking=True
    )

    if msg:
        status = gps_status.get(
            msg.fix_type,
            "Unknown"
        )

        print(
            f"GPS: {status} | "
            f"Satellites: {msg.satellites_visible}"
        )