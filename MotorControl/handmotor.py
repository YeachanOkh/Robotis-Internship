import time 
import RPi.GPIO as gpio
gpio.setup(25,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p = gpio.PWM(25, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0

gpio.setup(16,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p = gpio.PWM(16, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0

gpio.setup(17,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p = gpio.PWM(17, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0

gpio.setup(27,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p = gpio.PWM(27, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0

gpio.setup(22,gpio.OUT)  # Sets up pin 11 to an output (instead of an input)
p = gpio.PWM(22, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0


# Move the servo back and forth
# Clean up everything
p.stop()                 # At the end of the program, stop the PWM
