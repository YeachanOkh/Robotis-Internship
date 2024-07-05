#!/usr/bin/env python3
import serial
import time

def handmotor(response):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    ser.write("Response: "+response)
    line = ser.readline().decode('utf-8').rstrip()
    print(line)

if __name__ == '__main__':
    handmotor()