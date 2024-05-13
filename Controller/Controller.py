import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import numpy as np
import time
from tkinter import *
import csv
import sys
import os

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
#variables
movement =0
# Constants

# Adjust the import path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from Controller.Pid import pid_controller  # Import the PID controller
list_of_prev_poses = []
list_of_prev_desired_pos = []

def draw_ball_trajectory(img, current_pos, prev_pos):
    if prev_pos is not None:
        list_of_prev_poses.append(current_pos)
        if len(list_of_prev_poses) > 50:
            list_of_prev_poses.pop(0)

    if len(list_of_prev_poses) > 1:
        for i in range(len(list_of_prev_poses) - 1):
            cv2.line(img, list_of_prev_poses[i], list_of_prev_poses[i + 1], (0, 255, 0), 2)

    return img

def draw_desired_pos(img, pos):
    if len(list_of_prev_desired_pos) > 50:
        list_of_prev_desired_pos.pop(0)

    for i in range(len(list_of_prev_desired_pos)-1):
        cv2.line(img, list_of_prev_desired_pos[i], list_of_prev_poses[i+1], (0,0,255), 2)
    return img

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo_angle_limit = 60

def ball_track(key1, queue, img_size):
    camera_port = 1
    cap = cv2.VideoCapture(camera_port, cv2.CAP_DSHOW)
    cap.set(3, 1020)  # 1280 -> 640
    cap.set(4, 720)  # 720, legg til 480p

    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 200, 'vmin': 162, 'hmax': 179, 'smax': 255, 'vmax': 255}

    center_point = [480, 290, 2210]  # center point of the plate, calibrated[600, 380, 2210]

    global prev_ball_pos, pr

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        if countours:
            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                round(int(countours[0]['area'] - center_point[2]) / 100)

            current_ball_pos = (int(countours[0]['center'][0]), int(countours[0]['center'][1]))

            if prev_ball_pos is not None:
                img = draw_ball_trajectory(img, current_ball_pos, prev_ball_pos)

            prev_ball_pos = current_ball_pos

            queue.put(data)

        else:
            data = 'nil'  # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([img, imgContour], 1, 1)
        #imgStack = cvzone.stackImages([img, imgColor, mask, imgContour], 2, 0.5)  # use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def servo_control(key2, queue):
    port_id = 'COM7'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    if key2:
        print('Servo controls are initiated')


    def Get_Angles(p, r):
        R = 40
        L = 225
        p = p * np.pi / 180
        r = r * np.pi / 180
        # Define the position array
        PosMatNor = np.array([
            (L / 2, L / 2 * np.sqrt(3), 0),
            (-L / 2, L / 2 * np.sqrt(3), 0),
            (0, -L / np.sqrt(3), 0)
        ])

        # Define the rotation angle in radians
        v = 0.1 # Adjust the angle as needed

        # Define the rotation matrix for a rotation around the z-axis
        rotation_matrix = np.array([
            [np.cos(v), -np.sin(v), 0],
            [np.sin(v), np.cos(v), 0],
            [0, 0, 1]
        ])

        # Apply the rotation to the position array
        PosMatRotated = np.dot(PosMatNor, rotation_matrix)

        # Getting angle for pitch and roll for PID and chaning to radians.
        Vp = p * np.pi / 180
        Vr = r * np.pi / 180

        Transform_matrix_p = np.array([(1, 0, 0),
                                       (0, np.cos(Vp), -np.sin(Vp)),
                                       (0, np.sin(Vp), np.cos(Vp))
                                       ])

        PosPitch = np.dot(PosMatRotated, Transform_matrix_p)

        Transform_matrix_r = np.array([(np.cos(Vr), 0, -np.sin(Vr)),
                                       (0, 1, 0),
                                       (np.sin(Vr), 0, np.cos(Vr))
                                       ])

        z = np.dot(PosPitch, Transform_matrix_r)

        Va = np.zeros(3)
        VaF = np.zeros(3)
        VaFF = np.zeros(3)

        Va[0] = np.rad2deg(np.arctan(z[0][2] / R))
        Va[1] = np.rad2deg(np.arctan(z[1][2] / R))
        Va[2] = np.rad2deg(np.arctan(z[2][2] / R))

        alpha = 0.44
        VaF[0] = (alpha * Va[0] + (1 - alpha) * Va[0]) * -0.8
        VaF[1] = (alpha * Va[1] + (1 - alpha) * Va[1]) * -0.8
        VaF[2] = (alpha * Va[2] + (1 - alpha) * Va[2]) * -0.8

        if (-servo_angle_limit < VaF[0] < servo_angle_limit):
            VaFF[0] = VaF[0]
        else:
            VaFF[0] = VaFF[0]
        if (-servo_angle_limit < VaF[1] < servo_angle_limit):
            VaFF[1] = VaF[1]
        else:
            VaFF[1] = VaFF[1]
        if (-servo_angle_limit < VaF[2] < servo_angle_limit):
            VaFF[2] = VaF[2]
        else:
            VaFF[2] = VaFF[2]

        return VaFF
    
    def circular_setpoint(radius, angular_speed, time_values):
        x = radius * np.cos(angular_speed * time_values)
        y = radius * np.sin(angular_speed * time_values)
        return x, y

    def writeCoord():
        corrd_info = queue.get()
        global ep, er, integralp, integralr, dt, prevtime, currenttime, pr
        try:
            ball = [float(value) for value in corrd_info]
            ball_x = ball[0]
            ball_y = ball[1]
        except ValueError:
            return

        currenttime = time.time()
        dt = currenttime - prevtime
        prevtime = currenttime

        angle = np.zeros(3)

        if movement == 0:
            pr = [0, 0]
        elif movement == 1:
            radius = 14
            angular_speed = 1.35
            pr = circular_setpoint(radius, angular_speed, currenttime)

        print(corrd_info)
        if (not (-30 < corrd_info[0] < 30) or not (-30 < corrd_info[0] < 30)):
            angle[0] = 0
            angle[1] = 0
            angle[2] = 0
            prev_error_x = 0
            prev_error_z = 0
            integral_x = 0.0
            integral_y = 0.0
        else:
            tilt_x, prev_error_x, integral_x = pid_controller(ball_x, prev_error_x, integral_x)
            tilt_y, prev_error_z, integral_z = pid_controller(ball_z, prev_error_z, integral_z)

        angles: tuple = (round(float(angle[0]), 1),
                         round(float(angle[1]), 1),
                         round(float(angle[2]), 1))

        write_arduino(str(angles))

    def write_arduino(data):
        arduino.write(bytes(data, 'utf-8'))

    while key2:
        writeCoord() 

 
if __name__ == '__main__':
    queue = Queue()  # The queue is done inorder for the communication between the two processes.
    key1 = 1  # just two dummy arguments passed for the processes
    key2 = 2
    img_size = (1920, 1080)  # Replace this with the actual dimensions of your image
    p1 = mp.Process(target=ball_track, args=(key1, queue, img_size)) #, movementQueue
    p2 = mp.Process(target=servo_control, args=(key2, queue))#, movementQueue
    p1.start()
    p2.start()
    root = Tk()
    root.mainloop()
    p1.join()
    p2.join()