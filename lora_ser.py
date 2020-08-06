import serial

lora_Serial = serial.Serial("/dev/ttyACM1",115200,timeout=1)


def hum_temp():    
  lora_Serial.write("humi".encode())            
  data =  lora_Serial.readline().decode('utf-8')
  return data

def press_temp():    
  lora_Serial.write("pres".encode())            
  data =  lora_Serial.readline().decode('utf-8')
  return data

def temperature():    
  lora_Serial.write("temp".encode())            
  data =  lora_Serial.readline().decode('utf-8')
  return data

def acc_gyr():    
  lora_Serial.write("gyro".encode())            
  data =  lora_Serial.readline().decode('utf-8')
  return data

def acc():    
  lora_Serial.write("acce".encode())            
  data =  lora_Serial.readline().decode('utf-8')
  return data

def magn():    
  lora_Serial.write("magn".encode())            
  data =  lora_Serial.readline().decode('utf-8')
  return data