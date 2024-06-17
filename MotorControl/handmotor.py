import time 
import RPi.GPIO as gpio

servo1_pin = 25
gpio.setmode(gpio.BCM)
gpio.setup(servo1_pin,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)

p = gpio.PWM(servo1_pin, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0
print("servo running")

for ii in range(0,3):
    p.ChangeDutyCycle(2.0) # rotate to 0 degrees
    time.sleep(0.5)
    p.ChangeDutyCycle(12.0) # rotate to 180 degrees
    time.sleep(0.5)
    p.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(0.5)

p.ChangeDutyCycle(0) # this prevents jitter
p.stop() # stops the pwm on 13
GPIO.cleanup() # good practice when finished using a pin
# gpio.setup(16,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
# p1 = gpio.PWM(16, 50)     # Sets up pin 11 as a PWM pin
# p1.start(0)               # Starts running PWM on the pin and sets it to 0
# print("servo running")
# gpio.setup(17,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
# p2 = gpio.PWM(17, 50)     # Sets up pin 11 as a PWM pin
# p2.start(0)               # Starts running PWM on the pin and sets it to 0
# print("servo running")
# gpio.setup(27,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
# p3 = gpio.PWM(27, 50)     # Sets up pin 11 as a PWM pin
# p3.start(0)               # Starts running PWM on the pin and sets it to 0
# print("servo running")
# gpio.setup(22,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
# p4 = gpio.PWM(22, 50)     # Sets up pin 11 as a PWM pin
# p4.start(0)               # Starts running PWM on the pin and sets it to 0
# print("servo running")

# Move the servo back and forth
# Clean up everything
#p.stop()                 # At the end of the program, stop the PWM
