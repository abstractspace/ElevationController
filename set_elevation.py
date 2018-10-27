# Simple demo of of the LSM303 accelerometer & magnetometer library.
# Will print the accelerometer & magnetometer X, Y, Z axis values every half
# second.
# Author: Tony DiCola
# License: Public Domain
import time
import RPi.GPIO as GPIO
# Import the LSM303 module.
import Adafruit_LSM303
import math
import sys

setpoint = sys.argv[1]
setpoint = float(setpoint)
maxdist = sys.argv[2]
maxdist = float(maxdist)

h1 = 17
h2 = 27
#set numbering mode for the program 
GPIO.setmode(GPIO.BCM)
#setup led(pin 8) as output pin
GPIO.setup(h1, GPIO.OUT)
GPIO.setup(h2, GPIO.OUT)
#turn on and off the led in intervals of 1 second

p1 = GPIO.PWM(h1, 100)
p2 = GPIO.PWM(h2, 100)

p1.start(0)
p2.start(0)

# Create a LSM303 instance.
lsm303 = Adafruit_LSM303.LSM303()

# Alternatively you can specify the I2C bus with a bus parameter:
#lsm303 = Adafruit_LSM303.LSM303(busum=2)

print('Printing accelerometer & magnetometer X, Y, Z axis values, press Ctrl-C to quit...')
integral = 0
last_error = 0
y_angle_sum = 0
n_samples = 1
tolerance = 0.1
while True:
    for read_i in range(0,n_samples):
        # Read the X, Y, Z axis acceleration values and print them.
        accel, mag = lsm303.read()
    
        # Grab the X, Y, Z components from the reading and print them out.
        accel_x, accel_y, accel_z = accel
        mag_x, mag_y, mag_z = mag
    
   
        #print('Accel X={0}, Accel Y={1}, Accel Z={2}, Mag X={3}, Mag Y={4}, Mag Z={5}'.format(
        #     accel_x, accel_y, accel_z, mag_x, mag_y, mag_z))
         
        total_accel = math.sqrt(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z)
        y_angle = math.acos(accel_y/total_accel)
        y_angle = math.degrees(y_angle)
        y_angle = 90.0-y_angle
    
        y_angle_sum += y_angle
        time.sleep(0.1)
    
    y_angle_avg = y_angle_sum/n_samples*1.0
    # print(total_accel)
    #print(y_angle)
    
    # setpoint = 45
    maxpwm = 100
    
    error = y_angle - setpoint
    if abs(error)<maxdist:
        p1.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        break
    integral = integral + error
    derivative = error - last_error
    
    kp = 5.0
    ki = 0.1
    kd = 0.0
    
    def clamp(n, minn, maxn):
        return max(min(maxn, n), minn)
    
    pwm = (kp*error) + (ki*integral) + (kd*derivative)
    pwm = clamp(pwm, -maxpwm, maxpwm)
    # pwm = -pwm
    #if (abs(pwm)<20):
     #   pwm=0.0
    print('\n')
    print('Accel X={0}, Accel Y={1}, Accel Z={2}, Mag X={3}, Mag Y={4}, Mag Z={5}'.format(accel_x, accel_y, accel_z, mag_x, mag_y, mag_z))
    print('Current Position:{0}deg, Setpoint:{1}deg, Error:{2}deg'.format(y_angle, setpoint, error))
    print('{0} = ({1}*{2}) + ({3}*{4}) + ({5}*{6})'.format(pwm, kp, error, ki, integral, kd, derivative))
    
    if (pwm>=0):
        p1.ChangeDutyCycle(pwm)
        print(pwm)
        p2.ChangeDutyCycle(1)
        #turn forward, set as HIGH or 1
        #GPIO.output(h1,GPIO.HIGH)
        #GPIO.output(h2,GPIO.LOW)
    else: 
        pwm = -1.0*pwm
        p1.ChangeDutyCycle(1)
        p2.ChangeDutyCycle(pwm)
        print(pwm)
        #turn backward, set as LOW or 0
        #GPIO.output(h1, GPIO.LOW)
        #GPIO.output(h2, GPIO.HIGH)
    last_error = error
    


