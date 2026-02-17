#!/opt/homebrew/bin/python3

from controller import Robot
import json
import time
import os
from pathlib import Path

# ── Parametri fisici E-puck ──────────────────
WHEEL_RADIUS     = 0.0205   # m
WHEEL_DISTANCE   = 0.052    # m  
MAX_SPEED        = 6.28     # rad/s

robot = Robot()
timestep = int(robot.getBasicTimeStep())

SHARED_DIR  = Path.home() / "webots_ros2_bridge"
SHARED_DIR.mkdir(exist_ok=True)
SENSOR_FILE = SHARED_DIR / "sensor_data.json"
CMD_FILE    = SHARED_DIR / "cmd_vel.json"

print(f"Shared directory: {SHARED_DIR}")
print(f"Sensor file: {SENSOR_FILE}")
print(f"Command file: {CMD_FILE}")

# Motori
left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

if left_motor is None or right_motor is None:
    print("ERROR: Motors not found!")
else:
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    print("Motors initialized")

sensors = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    if sensor is not None:
        sensor.enable(timestep)
        sensors.append(sensor)
print(f"Initialized {len(sensors)} sensors")
print("Webots Controller Started!")

loop_count = 0

while robot.step(timestep) != -1:
    loop_count += 1

    if CMD_FILE.exists():
        try:
            with open(CMD_FILE, 'r') as f:
                cmd = json.load(f)

            linear  = cmd.get('linear',  0.0)
            angular = cmd.get('angular', 0.0)

            half_dist = WHEEL_DISTANCE / 2.0

            left_vel_ms  = linear - angular * half_dist
            right_vel_ms = linear + angular * half_dist

            left_vel_rads  = left_vel_ms  / WHEEL_RADIUS
            right_vel_rads = right_vel_ms / WHEEL_RADIUS

            left_vel_rads  = max(-MAX_SPEED, min(MAX_SPEED, left_vel_rads))
            right_vel_rads = max(-MAX_SPEED, min(MAX_SPEED, right_vel_rads))

            left_motor.setVelocity(left_vel_rads)
            right_motor.setVelocity(right_vel_rads)

            if loop_count % 10 == 0:
                print(f"CMD: linear={linear:.2f}, angular={angular:.2f} "
                      f"| wheels: L={left_vel_rads:.2f} R={right_vel_rads:.2f} rad/s")

        except Exception as e:
            if loop_count % 100 == 0:
                print(f"Error reading command: {e}")

    if len(sensors) > 0:
        try:
            sensor_values = [s.getValue() for s in sensors]

            data = {
                'timestamp':    time.time(),
                'distances':    sensor_values,
                'min_distance': float(min(sensor_values)),
                'max_distance': float(max(sensor_values))
            }

            temp_file = SHARED_DIR / "sensor_data_temp.json"
            with open(temp_file, 'w') as f:
                json.dump(data, f)
                f.flush()
            os.replace(str(temp_file), str(SENSOR_FILE))

            if loop_count % 50 == 0:
                vals = [f"{v:.0f}" for v in sensor_values]
                print(f"Sensors: {vals} | min={data['min_distance']:.1f} max={data['max_distance']:.1f}")

        except Exception as e:
            if loop_count % 100 == 0:
                print(f"Error writing sensors: {e}")
