import os
import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import numpy as np
import time
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

# Calculate pitch and roll based on ball position
def calculate_pitch_roll(ball_x, ball_y, width, height):
    # Normalize ball positions
    normalized_x = (ball_x - width / 2) / (width / 2)
    normalized_y = (ball_y - height / 2) / (height / 2)
    
    # Example conversion to pitch and roll
    pitch = normalized_y * 30  # Assuming maximum pitch angle is 30 degrees
    roll = normalized_x * 30   # Assuming maximum roll angle is 30 degrees
    
    return pitch, roll

# ROS 2 node for controlling the 3-DOF platform
class ThreeDOFController(Node):
    def __init__(self):
        super().__init__('three_dof_controller')
        self.publisher_pitch = self.create_publisher(Float64, 'pitch', 1)
        self.publisher_roll = self.create_publisher(Float64, 'roll', 1)
        self.publisher_ballx = self.create_publisher(Float64, 'ballx', 1)
        self.publisher_bally = self.create_publisher(Float64, 'bally', 1)
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
            
            # Calculate pitch and roll
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
