"""
Title: Lateral Bending Testing Protocol
Purpose: Connects to the xArm robotic arm, activates force control, and performs lateral bending movements (left and right).
Author/Contact: Christian Voloshen - Cornell University BME M.Eng 2024-2025

Description:
This script establishes a connection with the xArm robotic arm using the XArmAPI library,
enables motion and force control, and then performs incremental lateral bending movements (left and right).
Lateral bending movements are separated into two functions to further customization.
During each movement, the script collects force data and provides real-time visualization.

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

# Perform left lateral bending by moving the robotic arm incrementally to the left.
def move_joint_left(arm, x_i, z_i, y_i, theta_i, theta_f, pitch, yaw, steps, speed, ax, line, fig, x_data, y_data):
    """
    Moves the robotic arm to the left in a lateral bending motion.
    
    Args:
        arm: xArmAPI object controlling the robotic arm.
        x_i: Initial x-coordinate of the robotic arm's position.
        z_i: Initial z-coordinate of the robotic arm's position.
        y_i: Initial y-coordinate of the robotic arm's position.
        theta_i: Initial roll or bending angle in degrees.
        theta_f: Desired change in roll or bending angle in degrees.
        pitch: Fixed pitch angle (rotation around the y-axis).
        yaw: Fixed yaw angle (rotation around the z-axis).
        steps: Number of incremental steps for the movement.
        speed: Speed of the robotic arm movement.
        ax: Matplotlib axes object for plotting force data.
        line: Matplotlib line object for live plotting of force data.
        fig: Matplotlib figure object for the plot window.
        x_data: List to store x-axis data points (time) for the live plot.
        y_data: List to store y-axis data points (force) for the live plot.
    """ 
    # Calculate the change in angle per step for smooth lateral bending.
    delta_theta = theta_f / steps
    positions = [(x_i, z_i, theta_i)]

    for i in range(1, steps + 1):
        # Calculate the bending angle and positional offsets.
        theta = i * delta_theta
        # Calculate the offsets (point of rotation for vertebrae is 93 mm below TCP).
        x_offset = 93 * math.sin(math.radians(theta))  # Lateral offset
        z_offset = 93 * (math.cos(math.radians(theta)) - 1)  # Vertical compensation

        # Calculate the new position and orientation.
        x_s = x_i + x_offset
        z_s = z_i + z_offset
        theta_s = theta + theta_i
        positions.append((x_s, z_s, theta_s))

        # Move the arm to the calculated position.
        print(f"Moving left to step {i}: x={x_s}, y={y_i}, z={z_s}, roll={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=pitch, roll=theta_s, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)
        pause_for_force_data(arm, ax, line, fig, x_data, y_data)

    # Reverse the lateral bending motion.
    for i in range(len(positions) - 1, -1, -1):
        x_s, z_s, theta_s = positions[i]
        print(f"Reversing left to step {i}: x={x_s}, y={y_i}, z={z_s}, roll={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=pitch, roll=theta_s, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)


# Perform right lateral bending by moving the robotic arm incrementally to the right.
def move_joint_right(arm, x_i, z_i, y_i, theta_i, theta_f, pitch, yaw, steps, speed, ax, line, fig, x_data, y_data):
    """
    Moves the robotic arm in an extension motion (opposite of flexion).

    Args:
        Similar to move_joint_flexion.
    """
    delta_theta = theta_f / steps
    positions = [(x_i, z_i, theta_i)]

    for i in range(1, steps + 1):
        theta = i * delta_theta

        x_offset = -93 * math.sin(math.radians(theta))  # Negative for right bend
        z_offset = 93 * (math.cos(math.radians(theta)) - 1)

        x_s = x_i + x_offset
        z_s = z_i + z_offset
        theta_s = -theta + theta_i
        positions.append((x_s, z_s, theta_s))

        print(f"Moving right to step {i}: x={x_s}, y={y_i}, z={z_s}, roll={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=pitch, roll=theta_s, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)
        pause_for_force_data(arm, ax, line, fig, x_data, y_data)

    for i in range(len(positions) - 1, -1, -1):
        x_s, z_s, theta_s = positions[i]
        print(f"Reversing right to step {i}: x={x_s}, y={y_i}, z={z_s}, roll={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=pitch, roll=theta_s, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)


arm = XArmAPI('192.168.1.197')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# Initialize the plot for live data visualization
plt.ion()
fig, ax = plt.subplots()
x_data, y_data = [], []
line, = ax.plot(x_data, y_data)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Force (N)")
ax.set_title("Live Sensor Data")
plt.show()


# Check initial position of the arm
pos = arm.get_position(is_radian=False)
servo = arm.get_servo_angle(servo_id=None, is_radian=False)
tolerance = 1.0 # Tolerance for error in initial position

if any(abs(s - e) > tolerance for s, e in zip(servo[1], [0.7, -37.5, -37.4, 3.3, -1.1, -93.3])):
    arm.set_state(state=4)
    print('Robot is stopped. Please reset and put in appropriate initial position.')
else:
    # Extract position values from the tuple
    x_i, y_i, z_i, roll, pitch, yaw = pos[1]  # pos[1] gives the list of coordinates and angles
    #arm.ft_sensor_set_zero()
    time.sleep(0.2)
    sim_weight(arm)
    arm.set_state(0)
    time.sleep(1)

    # Perform left and right lateral bending movements
    arm.set_mode(0)
    arm.set_state(0)
    move_joint_left(arm, x_i=x_i, z_i=z_i, y_i=y_i, theta_i=roll, theta_f=5, pitch=pitch, yaw=yaw,  steps=3, speed=10, ax=ax, line=line, fig=fig, x_data=x_data, y_data=y_data)
    move_joint_right(arm, x_i=x_i, z_i=z_i, y_i=y_i, theta_i=roll, theta_f=5, pitch=pitch, yaw=yaw, steps=3, speed=10, ax=ax, line=line, fig=fig, x_data=x_data, y_data=y_data)
    plt.ioff()
    
    # Disable force control and sensor after the experiment
    arm.ft_sensor_app_set(0)
    arm.ft_sensor_enable(0)
    arm.disconnect()