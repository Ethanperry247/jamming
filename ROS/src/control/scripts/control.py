#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Int32
import serial
import time
import math

ser = None

# Starts the robot up in safe mode to prevent overheating, crashes, etc. Gives the robot time to sleep before operation.
def safe_start():
    ser.write(b'\x80')
    ser.write(b'\x84')
    time.sleep(3)

# Takes in a distance and a speed to move the robot. Distance can be positive or negative. Speed should be positive.
def move(distance, speed):
    array = bytearray()
    start_code = 145

    seconds = 0.0
    seconds = distance/speed
    seconds = abs(seconds)

    if (distance > 0):
        speed = speed
        low_byte = 0
        high_byte = speed

    if (distance < 0):
        temp1 = 0xFF
        temp2 = speed
        temp3 = temp2 ^ temp1
        temp3 = temp3 + 0x01
        low_byte = 255
        high_byte = int(temp3)

    array.append(start_code)
    array.append(low_byte)
    array.append(high_byte)
    array.append(low_byte)
    array.append(high_byte)
    ser.write(array)
    time.sleep(seconds)
    ser.write(b'\x89\x00\x00\x00\x00')

# Specified in radians. Positive will be a counterclockwise rotation. 
def turn(angle):

    angle = angle * (180 / math.pi)

    array = bytearray()
    array.append(137)

    array.append(angle)
    array.append(200 + angle)

    array.append(0)
    array.append(0)

    ser.write(array)
    time.sleep(1.1 * angle / 90)
    ser.write(b'\x89\x00\x00\x00\x00')


# Sets up robot in safe start mode and provides serial connection. 
def provide_connection():
    connection = serial.Serial(port='/dev/ttyUSB1', baudrate=57600)
    print("Serial Port Name:", ser.name)
    print("Baudrate:", ser.baudrate)
    safe_start(ser)
    
    global ser
    ser = connection

# Close serial connection. 
def close_connection():
    ser.close()

def linear_callback(distance):
        move(distance.data, 200)

def angle_callback(angle):
        turn(angle.data)
    
def control():
    rospy.init_node('control', anonymous=True)
    linear_sub = rospy.Subscriber('/create/linear', Int32, linear_callback)
    angle_sub = rospy.Subscriber('/create/angle', Int32, angle_callback)

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        close_connection()