"""
Title: Initial Position for Spinal Testing Protocol
Purpose: Connects to the xArm robotic arm and sets servo angles to an initial position.
Author/Contact: Christian Voloshen - Cornell University BME M.Eng 2024-2025

Description:
This script establishes a connection with the xArm robotic arm using the XArmAPI library,
enables motion, and sets the arm to a specified angle configuration.

References:
xArm API Documentation - https://github.com/xArm-Developer/xArm-Python-SDK/tree/master/xarm
"""

from xarm.wrapper import XArmAPI
import time

# Connect to the arm
arm = XArmAPI('192.168.1.197')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# Set Servo Angles for Initial Position

# For Flextion-Extension or Axial-Rotation Testing Use Position Below
angle_fe = [0.7, -37.5, -37.4, 3.3, -1.1, -3.3]

# For Lateral Testing Use Position Below
angle_fe = [0.7, -37.5, -37.4, 3.3, -1.1, -93.3]

arm.set_servo_angle(angle=angle_fe, is_radian=False, wait=False, speed=10)

