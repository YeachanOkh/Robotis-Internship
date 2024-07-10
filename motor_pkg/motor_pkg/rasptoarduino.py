#!/usr/bin/env python3
import serial
import time

def handmotor():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    # changetobyte=response.encode(encoding="utf-8") 
    ser.write(b"Response")
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    time.sleep(5)

if __name__ == '__main__':
    handmotor()