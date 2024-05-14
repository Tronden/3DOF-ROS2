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

# ROS 2 node for controlling the 3-DOF platform
class ThreeDOFController(Node):
    def __init__(self):
        super().__init__('three_dof_controller')
        self.publisher_joint1 = self.create_publisher(Float64, '/joint1_position_controller/command', 10)
        self.publisher_joint2 = self.create_publisher(Float64, '/joint2_position_controller/command', 10)
        self.publisher_joint3 = self.create_publisher(Float64, '/joint3_position_controller/command', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.cap.set(3, 1020)
        self.cap.set(4, 720)
        self.myColorFinder = ColorFinder(False)
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

    def control_loop(self):
        ret, img = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read from camera')
            return

        imgColor, mask = self.myColorFinder.update(img, [0, 0, 0])  # Adjust color range as needed
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            img = cv2.circle(img, center, int(radius), (0, 255, 0), 2)

            # Calculate angles using PID controller
            angle_x, self.prev_error_x, self.integral_x = pid_controller(x, self.prev_error_x, self.integral_x)
            angle_y, self.prev_error_y, self.integral_y = pid_controller(y, self.prev_error_y, self.integral_y)
            
            # Assuming the third angle can be derived from x and y
            angle_z = 0  

            self.publisher_joint1.publish(Float64(data=angle_x))
            self.publisher_joint2.publish(Float64(data=angle_y))
            self.publisher_joint3.publish(Float64(data=angle_z))

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
