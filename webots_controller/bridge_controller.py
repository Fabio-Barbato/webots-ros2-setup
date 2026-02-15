#!/usr/bin/env python3
"""
Webots Bridge Controller

Reads E-puck robot sensors and writes to JSON for ROS2.
Reads velocity commands from ROS2 and controls motors.
"""

from controller import Robot
import json
import time
from pathlib import Path

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Setup shared directory - UPDATE THIS PATH!
SHARED_DIR = Path.home() / "webots_ros2_bridge"
SHARED_DIR.mkdir(exist_ok=True)
SENSOR_FILE = SHARED_DIR / "sensor_data.json"
CMD_FILE = SHARED_DIR / "cmd_vel.json"

print(f"📁 Shared directory: {SHARED_DIR}")
print(f"📄 Sensor file: {SENSOR_FILE}")
print(f"🎮 Command file: {CMD_FILE}")

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

if left_motor is None or right_motor is None:
    print("❌ ERROR: Motors not found!")
else:
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    print("✅ Motors initialized")

# Initialize proximity sensors
sensors = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    if sensor is not None:
        sensor.enable(timestep)
        sensors.append(sensor)

print(f"✅ Initialized {len(sensors)} sensors")
print("🤖 Webots Controller Started!")

# Main control loop
loop_count = 0
while robot.step(timestep) != -1:
    loop_count += 1
    
    # READ COMMANDS FROM ROS2
    if CMD_FILE.exists():
        try:
            with open(CMD_FILE, 'r') as f:
                cmd = json.load(f)
            
            linear = cmd.get('linear', 0.0)
            angular = cmd.get('angular', 0.0)
            
            # Differential drive kinematics
            left_velocity = linear - angular
            right_velocity = linear + angular
            
            # Convert m/s to rad/s
            left_motor.setVelocity(left_velocity * 6.28)
            right_motor.setVelocity(right_velocity * 6.28)
            
            if loop_count % 10 == 0:
                print(f"🎮 CMD: linear={linear:.2f}, angular={angular:.2f}")
        except Exception as e:
            if loop_count % 100 == 0:
                print(f"⚠️  Error reading command: {e}")
    
    # WRITE SENSOR DATA FOR ROS2
    if len(sensors) > 0:
        try:
            sensor_values = [s.getValue() for s in sensors]
            
            data = {
                'timestamp': time.time(),
                'distances': sensor_values,
                'min_distance': float(min(sensor_values)),
                'max_distance': float(max(sensor_values))
            }
            
            json_str = json.dumps(data)
            
            with open(SENSOR_FILE, 'w') as f:
                f.write(json_str)
                f.flush()
            
            if loop_count % 50 == 0:
                print(f"📊 Wrote {len(json_str)} bytes, min={data['min_distance']:.1f}")
                
        except Exception as e:
            if loop_count % 100 == 0:
                print(f"⚠️  Error writing sensors: {e}")
