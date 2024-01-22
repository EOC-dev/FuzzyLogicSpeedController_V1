import RPi.GPIO as GPIO 
import time  
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# GPIO pin assignments for motor control

#Osoyoo used an l298n based motor driver which requires 3 pins per motor: a PWM(analog) signal pin to control the speed of the motor (the EN pins), and two digital logic pins to control the direction of the motor using an H-bridge circuit.
IN1Rear, IN2Rear = 16, 18
IN3Rear, IN4Rear = 13, 15 
ENA, ENB = 12, 33

#front Wheels were not used do to a hardware problem with the osoyoo motor x driver
IN1Front, IN2Front = 40, 38
IN3Front, IN4Front = 36, 32

#Setting up pins for the ultrasonic sensor
TRIGGER_PIN, ECHO_PIN = 31, 37

# Initialization - setting up GPIO pins as input/output. For more info, please read the GPIO docs: https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/
GPIO.setmode(GPIO.BOARD)
GPIO.setup([IN1Rear, IN2Rear, IN3Rear, IN4Rear, ENA, ENB, IN1Front, IN2Front, IN3Front, IN4Front, TRIGGER_PIN, ECHO_PIN], GPIO.OUT)
GPIO.output([ENA, ENB], True)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Motor control functions (e.g., go_ahead, turn_left, etc.)
# Each function corresponds to a motor action, these defs were mainly taken from the provided osoyoo tutorial series for the robot kit used in this project: https://osoyoo.com/driver/mecanum/mecanum-pi.py

#make rear right motor moving forward
def rr_ahead(speed):
    GPIO.output(IN1Rear,True)
    GPIO.output(IN2Rear,False)

    #ChangeDutyCycle(speed) function can change the motor rotation speed
    #rightSpeed.ChangeDutyCycle(speed)

#make rear left motor moving forward    
def rl_ahead(speed):  
    GPIO.output(IN3Rear,True)
    GPIO.output(IN4Rear,False)
    #leftSpeed.ChangeDutyCycle(speed)

def go_ahead(speed):
    rl_ahead(speed)
    rr_ahead(speed)
#     fl_ahead(speed)
#     fr_ahead(speed)
    GPIO.output(IN1Front,False) #Since we are just using motors that use motor pi controller, the rear wheels are disabled
    GPIO.output(IN2Front,False) #Sadly, only motors controlled using motor pi can have the pwm signal modified
    GPIO.output(IN3Front,False)
    GPIO.output(IN4Front,False)

# Setup for PWM speed control
#following code only works when using Model-Pi instead of Model X motor driver board which can give raspberry Pi USB 5V power
#Initialize Rear model Pi board ENA and ENB pins, tell OS that ENA,ENB will output analog PWM signal with 1000 frequency
rightSpeed = GPIO.PWM(ENA, 1000)
leftSpeed = GPIO.PWM(ENB, 1000)
rightSpeed.start(0)
leftSpeed.start(0)


# Ultrasonic distance measurement function. I go into more detail on the concepts behind this code on the corresponding blog entry for this project:
def DistMeasure():
    # set Trigger to HIGH to send out pulse
    GPIO.output(TRIGGER_PIN, True)
 
    # set Trigger pin to low after 0.01ms
    time.sleep(0.00001)
    GPIO.output(TRIGGER_PIN, False)
 
    TimeSent = time.time()
    TimeRecieved = time.time()
 
    # save time the pulse was sent
    while GPIO.input(ECHO_PIN) == 0:
        TimeSent = time.time()
 
    # save time the reflected pulse was recieved
    while GPIO.input(ECHO_PIN) == 1:
        TimeRecieved = time.time()
 
    # difference between timestamp sending the pulse and timestamp recieving the reflected wave back
    ElapsedTIme = TimeRecieved - TimeSent

    # Solving for distance using the classic speed = dist/time equation. Speed is speed of sound (343 m/s so 34300 cm/s), and we divide by two here to account for our time calculation being the time for the pulse to hit an object and then return -- whereas we just want the time to travel to the object.
    distance = (ElapsedTIme * 34300) / 2
 
    return distance

# Fuzzy inferencing system that takes in the distance calculated from the ultrasonic sensor, and uses it to determine what speed to set motors to.
def get_speed_value(dist):
    # Define universe variables
    distance = ctrl.Antecedent(np.arange(0, 40, 1), 'distance')
    speed = ctrl.Consequent(np.arange(0, 100, 1), 'speed')

    # Define fuzzy membership functions 
    distance['close'] = fuzz.trimf(distance.universe, [0, 0, 20])
    distance['medium'] = fuzz.trimf(distance.universe, [10, 20, 30])
    distance['far'] = fuzz.trimf(distance.universe, [20, 40, 40])

    speed['slow'] = fuzz.trimf(speed.universe, [0, 0, 40])
    speed['medium'] = fuzz.trimf(speed.universe, [30, 50, 70])
    speed['fast'] = fuzz.trimf(speed.universe, [60, 100, 100])

    # Define fuzzy rules
    rule1 = ctrl.Rule(distance['close'], speed['slow'])
    rule2 = ctrl.Rule(distance['medium'], speed['medium'])
    rule3 = ctrl.Rule(distance['far'], speed['fast'])

    # Create control system
    speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])

    # Create a control system simulator
    speedL = ctrl.ControlSystemSimulation(speed_ctrl)

    # Set inputs
    speedL.input['distance'] = dist

    # Calculate results
    speedL.compute()

    FuzzySpeed = int(speedL.output['speed'])
    
    return FuzzySpeed

# Continuous loop for processing and control
try:
    while True:
        UltraDist = DistMeasure()  # Measure distance
        print("Object distance = " + str(UltraDist) + " cm")
        FuzzSpeedOut = get_speed_value(UltraDist)  # Process distance through fuzzy logic
        print("\n Fuzzy Speed is: " + str(FuzzSpeedOut))
        go_ahead(10)  # initialize a speed for the motor
        rightSpeed.ChangeDutyCycle(FuzzSpeedOut) #USe the speed obtained 
        leftSpeed.ChangeDutyCycle(FuzzSpeedOut)
        time.sleep(0.1)

# Shutdown - cleanup on interrupt (CTRL+C)
except KeyboardInterrupt:
    print("User ended program")
    GPIO.cleanup()

GPIO.cleanup()  #This is redundant -- but I like to see the message it gives that pins have already been cleared. 

