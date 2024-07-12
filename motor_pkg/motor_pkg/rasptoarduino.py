#!/usr/bin/env python3
import serial
import time

def handmotor(response):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    changetobyte = response.encode(encoding="utf-8") 
    ser.write(changetobyte)
    
    # Add a delay to ensure all responses are sent by the Arduino
    time.sleep(1)
    
    while ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        ser.reset_input_buffer()

if __name__ == '__main__':
    handmotor("hello")  # Example command, replace "hello" with the desired command