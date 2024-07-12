#!/usr/bin/env python3
import serial
import time

def handmotor(response):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    changetobyte=response.encode(encoding="utf-8") 
    ser.write(changetobyte)
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    ser.reset_input_buffer()
    time.sleep(8)

if __name__ == '__main__':
    handmotor()