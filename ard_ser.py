#python motor control library for Iris Bot
#importing necessary packages
import serial
import time
import RPi.GPIO as GPIO

#setting up the pin configurations
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
TRIG_f = 25
ECHO_f = 14
TRIG_l = 27
ECHO_l = 4
TRIG_r = 23
ECHO_r = 24
GPIO.setup(TRIG_f, GPIO.OUT)
GPIO.setup(ECHO_f, GPIO.IN)
GPIO.setup(TRIG_l, GPIO.OUT)
GPIO.setup(ECHO_l, GPIO.IN)
GPIO.setup(TRIG_r, GPIO.OUT)
GPIO.setup(ECHO_r, GPIO.IN)
right_front = GPIO.PWM(12,50)
right_back = GPIO.PWM(18,50)
left_front = GPIO.PWM(19,50)
left_back = GPIO.PWM(13,50)


#establish communication with the Arduino UNO
Arduino_Serial = serial.Serial('/dev/ttyACM0',9600)

    
#ultrasonic sensor functions for obstacle avoidance
def US_f():
    GPIO.output(TRIG_f, False)
    time.sleep(0.00002)
    GPIO.output(TRIG_f, True)
    
    time.sleep(0.0001)
    GPIO.output(TRIG_f, False)
    time.sleep(0.00002)
 
    StartTime = time.time()
    StopTime = time.time()
 
    while GPIO.input(ECHO_f) == 0:
        StartTime = time.time()
   
    while GPIO.input(ECHO_f) == 1:
        StopTime = time.time()
    
    TotalTime=StopTime - StartTime
  
    distance=(TotalTime * 34300) / 2
    
    return round(distance)
def US_l():
    GPIO.output(TRIG_l, False)
    time.sleep(0.00002)
    GPIO.output(TRIG_l, True)
    
    time.sleep(0.0001)
    GPIO.output(TRIG_l, False)
    time.sleep(0.00002)
 
    StartTime = time.time()
    StopTime = time.time()
 
    while GPIO.input(ECHO_l) == 0:
        StartTime = time.time()
   
    while GPIO.input(ECHO_l) == 1:
        StopTime = time.time()
    
    TotalTime=StopTime - StartTime
  
    distance=(TotalTime * 34300) / 2
    
    return round(distance)
def US_r():
    GPIO.output(TRIG_r, False)
    time.sleep(0.00002)
    GPIO.output(TRIG_r, True)
    
    time.sleep(0.0001)
    GPIO.output(TRIG_r, False)
    time.sleep(0.00002)
 
    StartTime = time.time()
    StopTime = time.time()
 
    while GPIO.input(ECHO_r) == 0:
        StartTime = time.time()
   
    while GPIO.input(ECHO_r) == 1:
        StopTime = time.time()
    
    TotalTime=StopTime - StartTime
  
    distance=(TotalTime * 34300) / 2
    
    return round(distance)
    
#robo arm motor control functions
def base_l():
    Arduino_Serial.write('q'.encode())
    print ("base left")
    
def base_r():
    Arduino_Serial.write('w'.encode())
    print ("base right")
    
def shoulder_u():
    Arduino_Serial.write('e'.encode())
    print("shoulder up")
    
   
def shoulder_d():
    Arduino_Serial.write('r'.encode())
    print("shoulder down")
    
    
def elbow_u():
    Arduino_Serial.write('t'.encode())
    print("elbow up")
    
    
def elbow_d():
    Arduino_Serial.write('y'.encode())
    print("elbow down")
    
    
def gripper_o():
    Arduino_Serial.write('u'.encode())
    print("open")
    
    
def gripper_c():
    Arduino_Serial.write('i'.encode())
    print("close")
    
#Base rover motor control functions    
def bot_front():
 while (US_f()>10):
    right_front.start(25)
    left_front.start(25)
    right_front.ChangeDutyCycle(75)
    left_front.ChangeDutyCycle(75)
 while(US_f()<10):
    right_front.ChangeDutyCycle(0)
    left_front.ChangeDutyCycle(0)
    right_front.stop()
    left_front.stop()
    
def bot_back():
    right_back.start(25)
    left_back.start(25)
    right_back.ChangeDutyCycle(75)
    left_back.ChangeDutyCycle(75)
    time.sleep(2)
    right_back.ChangeDutyCycle(0)
    left_back.ChangeDutyCycle(0)
    right_back.stop()
    left_back.stop()
    
def bot_left():
  while (US_l()>10):
    left_back.start(25)
    right_front.start(25)
    left_back.ChangeDutyCycle(75)    
    right_front.ChangeDutyCycle(75)
  while(US_l()<10):  
    left_back.ChangeDutyCycle(0)
    right_front.ChangeDutyCycle(0)
    right_front.stop()
    left_back.stop()
    
     
def bot_right():
  while (US_r()>10):
    left_front.start(25)
    right_back.start(25)
    right_back.ChangeDutyCycle(75)
    left_front.ChangeDutyCycle(75)
  while(US_r()<10):  
    left_front.ChangeDutyCycle(0)
    right_back.ChangeDutyCycle(0)
    left_front.stop()
    right_back.stop()
    
    
def bot_stop():
     right_front.ChangeDutyCycle(0)
     left_front.ChangeDutyCycle(0)     
     right_back.ChangeDutyCycle(0)
     left_back.ChangeDutyCycle(0)
    
