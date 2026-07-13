from pymavlink import mavutil

# Connect to vehicle
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

master.wait_heartbeat()

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
