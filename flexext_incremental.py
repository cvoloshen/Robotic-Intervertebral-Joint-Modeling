"""
Title: Flexion-Extension Testing Protocol
Purpose: Connects to the xArm robotic arm, activates force control, and performs flexion and extension movements.
Author/Contact: Christian Voloshen - Cornell University BME M.Eng 2024-2025

Description:
This script establishes a connection with the xArm robotic arm using the XArmAPI library,
enables motion and force control, and then incrementally steps through flexion and extension movements. Flexion and extension were kept as 
separate functions to allow for higher customization of the movements and future implementation of more complex movements.
Through each movement, the script collects force data during the movements and allows for real-time visualization of the force control data.

References:
xArm API Documentation - https://github.com/xArm-Developer/xArm-Python-SDK/tree/master/xarm
"""
from xarm.wrapper import XArmAPI
import math
import time
import matplotlib.pyplot as plt

# Global variable to track keypress
continue_flag = False

# Keypress event handler
# This allows the script to pause and wait for user input before continuing, holding each degree of movement
def on_key(event):
    global continue_flag
    if event.key == 'n':
        continue_flag = True

# Pause for force data collection
# This function collects force data from the arm and updates the plot in real-time
def pause_for_force_data(arm, ax, line, fig, x_data, y_data):
    global continue_flag
    continue_flag = False
    fig.canvas.mpl_connect('key_press_event', on_key)
    print("Collecting force data. Press 'n' in the plot window to continue...")

    while not continue_flag:
        f_1 = arm.get_ft_sensor_data()[1][2]
        current_time = time.time()

        # Maintain a rolling window of 100 points for the plot.
        x_data.append(current_time)
        y_data.append(f_1)
        x_data[:] = x_data[-100:]
        y_data[:] = y_data[-100:]

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

# Function for moving the arm in a flexion direction
def move_joint_flexion(arm, x_i, z_i, y_i, theta_i, theta_f, roll, yaw, steps, speed, ax, line, fig, x_data, y_data):
    """
    Moves the robotic arm in a flexion motion.
    Args:
        arm: xArmAPI object controlling the robotic arm.
        x_i, z_i, y_i: Initial Cartesian coordinates.
        theta_i: Initial pitch or flexion angle
        theta_f: Desired level of change in flexion or pitch angle 
        roll, yaw: Roll and yaw angles (fixed).
        steps: Number of incremental steps for the movement.
        speed: Movement speed.
        ax, line, fig: Matplotlib objects for plotting force data.
        x_data, y_data: Lists to store real-time data points.
    """
    # Calculate the change in angle per step for smooth movement.
    delta_theta = theta_f / steps
    
    # List to store intermediate positions for movement reversal.
    positions = [(x_i, z_i, theta_i)]

    for i in range(1, steps + 1):
        # Incrementally calculate the pitch angle.
        theta = i * delta_theta
        
        # Calculate the offsets based on the flexion angle (point of rotation for vertebrae is 93 mm below TCP).
        x_offset = 93 * math.sin(math.radians(theta))
        z_offset = 93 * (math.cos(math.radians(theta)) - 1)

        # Compute the new position by adding offsets to the initial coordinates.
        x_s = x_i + x_offset
        z_s = z_i + z_offset
        theta_s = theta + theta_i
        
        # Store the new position for later reversal.
        positions.append((x_s, z_s, theta_s))

        # Move the arm to the calculated position.
        print(f"Moving to step {i}: x={x_s}, y={y_i}, z={z_s}, pitch={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=theta_s, roll=roll, yaw=yaw, speed=speed, wait=True)

        # Pause briefly for smooth motion and data collection.
        time.sleep(0.2)
        pause_for_force_data(arm, ax, line, fig, x_data, y_data)

    # Revert to the initial position after completing the flexion.
    print("Reversing the movement to return to the starting position.")
    for i in range(len(positions) - 1, -1, -1):
        x_s, z_s, theta_s = positions[i]
        print(f"Reversing to step {i}: x={x_s}, y={y_i}, z={z_s}, pitch={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=theta_s, roll=roll, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)

def move_joint_extension(arm, x_i, z_i, y_i, theta_i, theta_f, roll, yaw, steps, speed, ax, line, fig, x_data, y_data):
    """
    Moves the robotic arm in an extension motion (opposite of flexion).

    Args:
        Similar to move_joint_flexion.
    """
    # Calculate the change in angle per step for smooth movement.
    delta_theta = theta_f / steps

    # Store the initial position for reversal.
    positions = [(x_i, z_i, theta_i)]

    for i in range(1, steps + 1):
        # Incrementally calculate the extension angle.
        theta = i * delta_theta

        # Calculate offsets based on the extension angle (93 mm below TCP).
        x_offset = -93 * math.sin(math.radians(theta))
        z_offset = 93 * (math.cos(math.radians(theta)) - 1)

        # Compute the new position.
        x_s = x_i + x_offset
        z_s = z_i + z_offset
        theta_s = -1 * theta + theta_i
        
        # Store the new position for later reversal.
        positions.append((x_s, z_s, theta_s))

        # Move the arm to the calculated position.
        print(f"Moving to step {i}: x={x_s}, y={y_i}, z={z_s}, pitch={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=theta_s, roll=roll, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)
        pause_for_force_data(arm, ax, line, fig, x_data, y_data)

    # Revert to the initial position after completing the extension.
    print("Reversing the movement to return to the starting position.")
    for i in range(len(positions) - 1, -1, -1):
        x_s, z_s, theta_s = positions[i]
        print(f"Reversing to step {i}: x={x_s}, y={y_i}, z={z_s}, pitch={theta_s}")
        arm.set_position(x=x_s, y=y_i, z=z_s, pitch=theta_s, roll=roll, yaw=yaw, speed=speed, wait=True)
        time.sleep(0.2)


# Initialization
arm = XArmAPI('192.168.1.197')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# Set up the plot for real-time data visualization
plt.ion()
fig, ax = plt.subplots()
x_data, y_data = [], []
line, = ax.plot(x_data, y_data)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Force (N)")
ax.set_title("Live Sensor Data")
plt.show()

# Get initial position 
pos = arm.get_position(is_radian=False)
servo = arm.get_servo_angle(servo_id=None, is_radian=False)
tolerance = 1.0

# Check if the arm is in the correct initial position
if any(abs(s - e) > tolerance for s, e in zip(servo[1], [0.7, -37.5, -37.4, 3.3, -1.1, -3.3])):
    arm.set_state(state=4)
    print('Robot is stopped. Please reset and put in appropriate initial position.')
else:
    x_i, y_i, z_i, roll, pitch, yaw = pos[1]
    # If not done already, zero the force sensor at the begining of the experiment. 
    # Only do this once at the beginning of the experiment.
    # arm.ft_sensor_set_zero()
    time.sleep(0.2)

    # Call the function to simulate weight on the arm
    sim_weight(arm)
    arm.set_state(0)
    time.sleep(1)

    # Run Flexion and Extension Code
    move_joint_flexion(arm, x_i=x_i, z_i=z_i, y_i=y_i, theta_i=pitch, theta_f=5, roll=roll, yaw=yaw,  steps=3, speed=10,ax=ax, line=line, fig=fig, x_data=x_data, y_data=y_data )
    move_joint_extension(arm, x_i=x_i, z_i=z_i, y_i=y_i, theta_i=pitch, theta_f=5, roll=roll, yaw=yaw, steps=3, speed=20, ax=ax, line=line, fig=fig, x_data=x_data, y_data=y_data)

    plt.ioff()
    # Disable force control and disconnect from the arm
    arm.ft_sensor_app_set(0)
    arm.ft_sensor_enable(0)
    arm.disconnect()