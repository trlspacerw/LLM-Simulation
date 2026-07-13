from pymavlink import mavutil

# Connect to vehicle
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()

while True:
    msg = master.recv_match(type='SYS_STATUS', blocking=True)

    if msg:
        voltage = msg.voltage_battery / 1000.0 # Convert mV to V
        current = msg.current_battery / 100.0 # Convert 10mA to A
        remaining = msg.battery_remaining # Percentage

        print(f"Battery Voltage: {voltage:.2f} V, Current: {current:.2f} A, Remaining: {remaining}%")

        