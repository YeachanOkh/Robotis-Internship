import RPi.GPIO as GPIO
import time

# setup the GPIO pin for the servo GPIO 12, 13, 18, 19 for PWM
servo1_pin = 12
servo2_pin = 13
servo3_pin = 18
servo4_pin = 19
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1_pin,GPIO.OUT)
GPIO.setup(servo2_pin,GPIO.OUT)
GPIO.setup(servo3_pin,GPIO.OUT)
GPIO.setup(servo4_pin,GPIO.OUT)

# setup PWM process
pwm1 = GPIO.PWM(servo1_pin,50) # 50 Hz (20 ms PWM period)
pwm2 = GPIO.PWM(servo2_pin,50) # 50 Hz (20 ms PWM period)
pwm3 = GPIO.PWM(servo3_pin,50) # 50 Hz (20 ms PWM period)
pwm4 = GPIO.PWM(servo4_pin,50) # 50 Hz (20 ms PWM period)

pwm1.start(7) # start PWM by rotating to 90 degrees
pwm2.start(7) # start PWM by rotating to 90 degrees
pwm3.start(7) # start PWM by rotating to 90 degrees
pwm4.start(7) # start PWM by rotating to 90 degrees

for ii in range(0,9):
    pwm1.ChangeDutyCycle(2.0) # rotate to 0 degrees
    time.sleep(0.5)
    pwm1.ChangeDutyCycle(12.0) # rotate to 180 degrees
    time.sleep(0.5)
    pwm1.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(0.5)

    pwm2.ChangeDutyCycle(2.0) # rotate to 0 degrees
    time.sleep(0.5)
    pwm2.ChangeDutyCycle(12.0) # rotate to 180 degrees
    time.sleep(0.5)
    pwm2.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(0.5)
    
    pwm3.ChangeDutyCycle(2.0) # rotate to 0 degrees
    time.sleep(0.5)
    pwm3.ChangeDutyCycle(12.0) # rotate to 180 degrees
    time.sleep(0.5)
    pwm3.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(0.5)

    pwm4.ChangeDutyCycle(2.0) # rotate to 0 degrees
    time.sleep(0.5)
    pwm4.ChangeDutyCycle(12.0) # rotate to 180 degrees
    time.sleep(0.5)
    pwm4.ChangeDutyCycle(7.0) # rotate to 90 degrees
    time.sleep(0.5)

pwm1.ChangeDutyCycle(0) # this prevents jitter
pwm1.stop() # stops the pwm on 13

pwm2.ChangeDutyCycle(0) # this prevents jitter
pwm2.stop() # stops the pwm on 13

pwm3.ChangeDutyCycle(0) # this prevents jitter
pwm3.stop() # stops the pwm on 13

pwm4.ChangeDutyCycle(0) # this prevents jitter
pwm4.stop() # stops the pwm on 13
GPIO.cleanup() # good practice when finished using a pin