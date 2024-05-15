import os
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# PID controller for servo control
def pid_controller(ball_pos, prev_error, integral, dt=0.01, Kp=0.1, Ki=0.01, Kd=0.05):
    error = -ball_pos
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, error, integral

# Calculate pitch and roll based on ball position using transformation matrices
def calculate_pitch_roll(ball_x, ball_y, width, height):
    # Normalize ball positions
    normalized_x = (ball_x - width / 2) / (width / 2)
    normalized_y = (ball_y - height / 2) / (height / 2)
    
    # Define the rotation matrices for pitch and roll
    pitch_rad = np.deg2rad(normalized_y * 30)  # Assuming maximum pitch angle is 30 degrees
    roll_rad = np.deg2rad(normalized_x * 30)   # Assuming maximum roll angle is 30 degrees

    # Rotation matrix for pitch
    T_pitch = np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad), np.cos(pitch_rad)]
    ])
    
    # Rotation matrix for roll
    T_roll = np.array([
        [np.cos(roll_rad), 0, np.sin(roll_rad)],
        [0, 1, 0],
        [-np.sin(roll_rad), 0, np.cos(roll_rad)]
    ])
    
    # Total transformation matrix
    T = np.dot(T_roll, T_pitch)

    # Example ball position in the camera frame (assuming z = 0)
    ball_pos_camera = np.array([ball_x, ball_y, 0])
    
    # Transform ball position to platform frame
    ball_pos_platform = np.dot(T, ball_pos_camera)

    # Extract pitch and roll from the transformation matrix
    pitch = np.rad2deg(np.arcsin(T[1, 2]))
    roll = np.rad2deg(np.arcsin(-T[0, 2]))

    return pitch, roll

# Convert pitch and roll to servo angles
def calculate_servo_angles(pitch, roll):
    # Convert pitch and roll angles to radians
    pitch_rad = np.deg2rad(pitch)
    roll_rad = np.deg2rad(roll)

    # Calculate the angle for each servo
    servo1_angle = pitch_rad * np.cos(np.deg2rad(0)) + roll_rad * np.sin(np.deg2rad(0))
    servo2_angle = pitch_rad * np.cos(np.deg2rad(120)) + roll_rad * np.sin(np.deg2rad(120))
    servo3_angle = pitch_rad * np.cos(np.deg2rad(240)) + roll_rad * np.sin(np.deg2rad(240))

    # Convert the angles back to degrees
    servo1_angle = np.rad2deg(servo1_angle)
    servo2_angle = np.rad2deg(servo2_angle)
    servo3_angle = np.rad2deg(servo3_angle)

    return servo1_angle, servo2_angle, servo3_angle

# ROS 2 node for controlling the 3-DOF platform
class ThreeDOFController(Node):
    def __init__(self):
        super().__init__('three_dof_controller')
        self.publisher_pitch = self.create_publisher(Float64, 'pitch', 1)
        self.publisher_roll = self.create_publisher(Float64, 'roll', 1)
        self.publisher_ballx = self.create_publisher(Float64, 'ballx', 1)
        self.publisher_bally = self.create_publisher(Float64, 'bally', 1)
        self.publisher_servo1 = self.create_publisher(Float64, 'servo1_angle', 1)
        self.publisher_servo2 = self.create_publisher(Float64, 'servo2_angle', 1)
        self.publisher_servo3 = self.create_publisher(Float64, 'servo3_angle', 1)
        self.timer = self.create_timer(0.1, self.control_loop)

        camera_device = '/dev/video4'
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)
        self.cap.set(3, 1020)
        self.cap.set(4, 720)
        self.myColorFinder = ColorFinder(False)
        self.color_hsv_values = {'hmin': 0, 'smin': 0, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 255}  # Set initial values, can be adjusted as needed
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

    def control_loop(self):
        ret, img = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read from camera')
            return

        imgColor, mask = self.myColorFinder.update(img, self.color_hsv_values)  # Use the color_hsv_values dictionary
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            img = cv2.circle(img, center, int(radius), (0, 255, 0), 2)

            # Calculate angles using PID controller
            angle_x, self.prev_error_x, self.integral_x = pid_controller(x, self.prev_error_x, self.integral_x)
            angle_y, self.prev_error_y, self.integral_y = pid_controller(y, self.prev_error_y, self.integral_y)
            
            # Calculate pitch and roll using transformation matrices
            pitch, roll = calculate_pitch_roll(x, y, img.shape[1], img.shape[0])
            
            # Publish pitch and roll
            self.publisher_pitch.publish(Float64(data=pitch))
            self.publisher_roll.publish(Float64(data=roll))

            # Assuming ballx and bally can be derived from x and y
            ballx = angle_x  
            bally = angle_y  

            # Publish ballx and bally
            self.publisher_ballx.publish(Float64(data=ballx))
            self.publisher_bally.publish(Float64(data=bally))

            # Calculate servo angles
            servo1_angle, servo2_angle, servo3_angle = calculate_servo_angles(pitch, roll)

            # Publish servo angles
            self.publisher_servo1.publish(Float64(data=servo1_angle))
            self.publisher_servo2.publish(Float64(data=servo2_angle))
            self.publisher_servo3.publish(Float64(data=servo3_angle))

        cv2.imshow('Image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = ThreeDOFController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()