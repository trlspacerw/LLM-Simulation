"""Read telemetry from both drones over MAVLink."""
import threading
from pymavlink import mavutil

def stream(uri, tag):
    m = mavutil.mavlink_connection(uri)
    m.wait_heartbeat()
    print(f"[{tag}] connected sys={m.target_system}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)  # 4 Hz
    while True:
        msg = m.recv_match(type='ATTITUDE', blocking=True)
        print(f"[{tag}] roll={msg.roll:+.2f}  pitch={msg.pitch:+.2f}  yaw={msg.yaw:+.2f}")

for uri, tag in [("tcp:127.0.0.1:5760", "drone1"),
                 ("tcp:127.0.0.1:5770", "drone2")]:
    threading.Thread(target=stream, args=(uri, tag), daemon=True).start()

threading.Event().wait()  # block forever; Ctrl+C to quit
