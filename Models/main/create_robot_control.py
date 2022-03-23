import serial
import time
import math

# Starts the robot up in safe mode to prevent overheating, crashes, etc. Gives the robot time to sleep before operation.
def safe_start(ser):
    ser.write(b'\x80')
    ser.write(b'\x84')
    time.sleep(3)

# Takes in a distance and a speed to move the robot. Distance can be positive or negative. Speed should be positive.
def move(ser, distance, speed):
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
def turn(ser, angle):

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
    ser = serial.Serial(port='/dev/ttyUSB1', baudrate=57600)
    print("Serial Port Name:", ser.name)
    print("Baudrate:", ser.baudrate)
    safe_start(ser)
    return ser

# Close serial connection. 
def close_connection(ser):
    ser.close()
