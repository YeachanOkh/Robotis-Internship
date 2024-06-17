import time 
import RPi.GPIO as GPIO

servo1_pin = 22
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo1_pin,GPIO.OUT)  # Sets up pin 11 to an output (instead of an input)

servo1 = GPIO.PWM(servo1_pin, 50)     # Sets up pin 11 as a PWM pin
servo1.start(0)               # Starts running PWM on the pin and sets it to 0

def SetAngle(angle):
	duty = angle / 18 + 2
	GPIO.output(servo1_pin, True)
	servo1.ChangeDutyCycle(duty)
	time.sleep(1)
	GPIO.output(servo1_pin, False)
	servo1.ChangeDutyCycle(0)
	
SetAngle(90)

servo1.stop()
GPIO.cleanup

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
