#Iris server python script for establishing control communication for client(user)
#importing necessary packages
from __future__ import division
from flask_socketio import SocketIO, emit
from flask import Flask, render_template, url_for, copy_current_request_context,Response
import time
import sys
import os
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import struct
import _thread
import board
import busio as bus
import adafruit_mlx90614
import RPi.GPIO as GPIO
from ard_ser import *
from lora_ser import *
from serial import Serial
import paho.mqtt.client as mqtt
from threading import Thread
import eventlet
import picamera
import cv2
import socket
import io

#starting eventlet for socketio and setting up GPIO pin modes 
eventlet.monkey_patch()
GPIO.setwarnings(False)
ir=21
GPIO.setup(ir,GPIO.IN)
#initializing I2C bus
i2c = bus.I2C(board.SCL, board.SDA, frequency=100000)
mlx = adafruit_mlx90614.MLX90614(i2c)

#starting mqtt service
mqttc=mqtt.Client()
mqttc.connect("localhost",1883,60)
mqttc.loop_start()

app = Flask(__name__)

socketio = SocketIO(app, async_mode=None, logger=True, engineio_logger=True)

thread=None

ap = argparse.ArgumentParser()
ap.add_argument("-f", "--face", type=str,
    default="face_detector",
    help="path to face detector model directory")
ap.add_argument("-m", "--model", type=str,
    default="mask_detector.model",
    help="path to trained face mask detector model")
ap.add_argument("-c", "--confidence", type=float, default=0.5,
    help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

# load our serialized face detector model from disk
print("[INFO] loading face detector model...")
prototxtPath = os.path.sep.join([args["face"], "deploy.prototxt"])
weightsPath = os.path.sep.join([args["face"],
    "res10_300x300_ssd_iter_140000.caffemodel"])
faceNet = cv2.dnn.readNet(prototxtPath, weightsPath)

# load the face mask detector model from disk
print("[INFO] loading face mask detector model...")
maskNet = load_model(args["model"])

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = cv2.VideoCapture(0)

def detect_and_predict_mask(frame, faceNet, maskNet):
    # grab the dimensions of the frame and then construct a blob
    # from it
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300),
        (104.0, 177.0, 123.0))

    # pass the blob through the network and obtain the face detections
    faceNet.setInput(blob)
    detections = faceNet.forward()

    # initialize our list of faces, their corresponding locations,
    # and the list of predictions from our face mask network
    faces = []
    locs = []
    preds = []

    # loop over the detections
    for i in range(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with
        # the detection
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the confidence is
        # greater than the minimum confidence
        if confidence > args["confidence"]:
            # compute the (x, y)-coordinates of the bounding box for
            # the object
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # ensure the bounding boxes fall within the dimensions of
            # the frame
            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))

            # extract the face ROI, convert it from BGR to RGB channel
            # ordering, resize it to 224x224, and preprocess it
            face = frame[startY:endY, startX:endX]
            face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
            face = cv2.resize(face, (224, 224))
            face = img_to_array(face)
            face = preprocess_input(face)
            face = np.expand_dims(face, axis=0)

            # add the face and bounding boxes to their respective
            # lists
            faces.append(face)
            locs.append((startX, startY, endX, endY))

    # only make a predictions if at least one face was detected
    if len(faces) > 0:
        # for faster inference we'll make batch predictions on *all*
        # faces at the same time rather than one-by-one predictions
        # in the above `for` loop
        preds = maskNet.predict(faces)

    # return a 2-tuple of the face locations and their corresponding
    # locations
    return (locs, preds)



#function for temperature scanning and video streaming executed as thread
def scanner():
  while True:
    rval, frame = vs.read()
 
     # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = imutils.resize(frame, width=400)

    # detect faces in the frame and determine if they are wearing a
    # face mask or not
    (locs, preds) = detect_and_predict_mask(frame, faceNet, maskNet)

    # loop over the detected face locations and their corresponding
    # locations
    for (box, pred) in zip(locs, preds):
        # unpack the bounding box and predictions
        (startX, startY, endX, endY) = box
        (mask, withoutMask) = pred

        # determine the class label and color we'll use to draw
        # the bounding box and text
        label = "Mask" if mask > withoutMask else "No Mask"
        color = (0, 255, 0) if label == "Mask" else (0, 0, 255)

        # include the probability in the label
        label = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)

        # display the label and bounding box rectangle on the output
        # frame
        cv2.putText(frame, label, (startX, startY - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
    
    
    
    
    cv2.imwrite('t.jpg', frame)
    yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + open('t.jpg', 'rb').read() + b'\r\n')
    data=GPIO.input(ir)
    if data == 1: 
        number=mlx.object_temperature
        print(number)
        number=(number * 9/5) + 32
        number=round(number)  
        socketio.emit('temp', {'number': number}, namespace='/test')
        time.sleep(1)
    socketio.emit('hum', {'number': hum_temp()}, namespace='/test')
    socketio.emit('press', {'number': press_temp()}, namespace='/test')
    socketio.emit('temperature', {'number': temperature()}, namespace='/test')
    socketio.emit('acc_gyr', {'number': acc_gyr()}, namespace='/test')
    socketio.emit('acc', {'number': acc()}, namespace='/test')
    socketio.emit('magn', {'number': magn()}, namespace='/test')
#function to render template and thread data
@app.route('/')
def index():
    global thread
    if thread is None:
        thread = Thread(target=scanner)
        thread.daemon = True
        thread.start()
    
    return render_template('Iris_server.html')

#video streming route, src for html video streaming
@app.route('/video_feed')
def video_feed():

    return Response(scanner(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

#functions for handling soicketio emit function
@socketio.on('connect', namespace='/test')
def test_connect():
    print('Client connected')
        
@socketio.on('base left',namespace='/test')
def base_left():
  base_l()
    
@socketio.on('base right',namespace='/test')
def base_right():
  base_r()
  
@socketio.on('shoulder up',namespace='/test')
def shoulder_up():
  shoulder_u() 
  
@socketio.on('shoulder down',namespace='/test')
def shoulder_down():
  shoulder_d()
  
@socketio.on('elbow up',namespace='/test')
def elbow_up():
  elbow_u()
     
@socketio.on('elbow down',namespace='/test')
def elbow_down():
  elbow_d()  
  
@socketio.on('open',namespace='/test')
def base_left():
  gripper_o()
  
@socketio.on('close',namespace='/test')
def base_left():
  gripper_c()
  
@socketio.on('front',namespace='/test')
def front():
  bot_front()

@socketio.on('back',namespace='/test')
def back():
  bot_back()

@socketio.on('left',namespace='/test')
def left():
  bot_left()
  
@socketio.on('right',namespace='/test')
def right():
  bot_right() 
  
@socketio.on('stop',namespace='/test')
def stop():
  bot_stop()  

@socketio.on('on',namespace='/test')
def light_on():
  mqttc.publish("lights","on")

@socketio.on('off',namespace='/test')
def light_off():
  mqttc.publish("lights","off")
  
@socketio.on('on1',namespace='/test')
def light1_on():
  mqttc.publish("lights","on1")

@socketio.on('off1',namespace='/test')
def light1_off():
  mqttc.publish("lights","off1")
  
@socketio.on('on2',namespace='/test')
def light2_on():
  mqttc.publish("lights","on2")

@socketio.on('off2',namespace='/test')
def light2_off():
  mqttc.publish("lights","off2")
  
@socketio.on('on3',namespace='/test')
def fan_on():
  mqttc.publish("fans","on3")

@socketio.on('off3',namespace='/test')
def fan_off():
  mqttc.publish("fans","off3")
  
@socketio.on('on4',namespace='/test')
def fan1_on():
  mqttc.publish("fans","on4")

@socketio.on('off4',namespace='/test')
def fan1_off():
  mqttc.publish("fans","off4")
   
@socketio.on('disconnect', namespace='/test')
def test_disconnect():
    print('Client disconnected')

#starts the server and runs the server forever till user interrupts
if __name__ == '__main__':
   socketio.run(app,host='0.0.0.0',port=8000,use_reloader=False)
   
