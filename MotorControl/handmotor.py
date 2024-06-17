import time 
import RPi.GPIO as gpio
gpio.setmode(gpio.BCM)

gpio.setup(25,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p = gpio.PWM(25, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0
print("servo running")
gpio.setup(16,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p1 = gpio.PWM(16, 50)     # Sets up pin 11 as a PWM pin
p1.start(0)               # Starts running PWM on the pin and sets it to 0
print("servo running")
gpio.setup(17,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p2 = gpio.PWM(17, 50)     # Sets up pin 11 as a PWM pin
p2.start(0)               # Starts running PWM on the pin and sets it to 0
print("servo running")
gpio.setup(27,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p3 = gpio.PWM(27, 50)     # Sets up pin 11 as a PWM pin
p3.start(0)               # Starts running PWM on the pin and sets it to 0
print("servo running")
gpio.setup(22,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p4 = gpio.PWM(22, 50)     # Sets up pin 11 as a PWM pin
p4.start(0)               # Starts running PWM on the pin and sets it to 0
print("servo running")

# Move the servo back and forth
# Clean up everything
p.stop()                 # At the end of the program, stop the PWM
