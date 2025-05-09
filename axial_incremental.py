"""
Title: Axial Rotation Testing Protocol
Purpose: Connects to the xArm robotic arm, activates force control, and performs axial rotation movements.
Author/Contact: Christian Voloshen - Cornell University BME M.Eng 2024-2025

Description:
This script establishes a connection with the xArm robotic arm using the XArmAPI library,
enables motion and force control, and then performs axial rotation movements.
The axial rotation is controlled using servo angle adjustments. During the movement,
the script collects force data and provides real-time visualization.

References:
xArm API Documentation - https://github.com/xArm-Developer/xArm-Python-SDK/tree/master/xarm
"""
from xarm.wrapper import XArmAPI
import math
import time
import matplotlib.pyplot as plt

# Global variable to track keypress
continue_flag = False

def on_key(event):
    global continue_flag
    if event.key == 'n':
        continue_flag = True

# Pause for force data collection and update the real-time plot.
# Waits for the user to press 'n' to continue.
def pause_for_force_data(arm, ax, line, fig, x_data, y_data):
    global continue_flag
    continue_flag = False
    fig.canvas.mpl_connect('key_press_event', on_key)
    print("Collecting force data. Press 'n' in the plot window to continue...")

    while not continue_flag:
        # Capture force data from the sensor.
        f_1 = arm.get_ft_sensor_data()[1][2]
        current_time = time.time()

        # Append new data points and maintain a rolling window of 100 points.
        x_data.append(current_time)
        y_data.append(f_1)
        x_data[:] = x_data[-100:]
        y_data[:] = y_data[-100:]

        # Update the plot with the new data.
        line.set_xdata(x_data)
        line.set_ydata(y_data)
        ax.relim()
        ax.autoscale_view()
        plt.draw()
        plt.pause(0.05)

# Simulate weight on the arm
# This function sets up the force control parameters for the arm
# Configuration of PID parameters along with force control courtesy of xArmAPI documentation and examples 
def sim_weight(arm):
    # PID parameters for force control
    Kp = 0.005
    Ki = 0.0000
    Kd = 0.05

    linear_v_max = 15.0
    rot_v_max = 0.35
    ref_frame = 1
    force_axis = [0, 0, 1, 0, 0, 0]
    # Desired applied force
    force_ref = [0, 0, 170, 0, 0, 0]
    arm.set_force_control_pid([Kp]*6, [Ki]*6, [Kd]*6, [linear_v_max]*3 + [rot_v_max]*3)
    code = arm.config_force_control(ref_frame, force_axis, force_ref, [0]*6)
    if code == 0 and arm.error_code == 0:
        arm.ft_sensor_enable(1)
        arm.ft_sensor_app_set(2)

arm = XArmAPI('192.168.1.197')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

plt.ion()
fig, ax = plt.subplots()
x_data, y_data = [], []
line, = ax.plot(x_data, y_data)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Force (N)")
ax.set_title("Live Sensor Data")

# Get the current position of the robot (assuming pos is a tuple: (code, [x, y, z, rx, ry, rz]))
pos = arm.get_position(is_radian=False)
servo = arm.get_servo_angle(servo_id=None, is_radian=False)
tolerance = 1.0 # Tolerance for error in initial position

if any(abs(s - e) > tolerance for s, e in zip(servo[1], [0.7, -37.5, -37.4, 3.3, -1.1, -3.3])):
    arm.set_state(state=4)
    print('Robot is stopped. Please reset and put in appropriate initial position.')
else:

    # Call force control in initial position 
    sim_weight(arm)
    arm.set_state(0)
    time.sleep(1)

    # Run Axial Rotation Movement
    arm.set_mode(0)
    arm.set_state(0)

    # Get the current angle of the servo responsible for rotation.
    _, current_angle = arm.get_servo_angle(servo_id=6, is_radian=False)

    # Rotate to the right by adding 5 degrees.
    arm.set_servo_angle(servo_id=6, angle=current_angle + 5, is_radian=False, wait=True)
    pause_for_force_data(arm, ax, line, fig, x_data, y_data)

    # Rotate to the left by subtracting 10 degrees (5 degrees from the initial position).
    arm.set_servo_angle(servo_id=6, angle=current_angle - 5, is_radian=False, wait=True)
    pause_for_force_data(arm, ax, line, fig, x_data, y_data)

    # Return to the original position by adding 5 degrees.
    arm.set_servo_angle(servo_id=6, angle=current_angle + 5, is_radian=False, wait=True)
