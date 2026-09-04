""" import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)  # allow Arduino to reset if needed

commands = [b'center,0,forward,14', b'right,90,forward,0', b'center,0,forward,130', b'left,90,reverse,0', b'left,90,reverse,0', b'SNAP6_C', b'FIN']
commands = [b'center,0,forward,20', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,6', b'center,0,forward,180', b'center,0,reverse,0', b'left,90,reverse,0', b'center,0,reverse,12', b'center,0,reverse,0', b'left,90,reverse,0', b'center,0,reverse,12', b'SNAP6_C', b'FIN']
commands = [b'center,0,forward,120', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,6', b'SNAP1_C', b'FIN']
commands =[b'center,0,forward,280', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,32', b'center,0,forward,40', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,32', b'SNAP5_C', b'FIN']
commands = [b'center,0,forward,260', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,20', b'center,0,forward,180', b'SNAP3', b'center,0,reverse,40', b'center,0,reverse,0', b'left,90,reverse,0', b'center,0,reverse,12', b'center,0,forward,100', b'SNAP5', b'FIN']
commands = [b'center,0,forward,280', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,20', b'center,0,forward,40', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,20', b'SNAP5_C', b'center,0,reverse,100', b'center,0,forward,0', b'left,90,forward,0', b'center,0,forward,10', b'center,0,forward,20', b'SNAP3', b'center,0,reverse,80', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,20', b'center,0,forward,180', b'center,0,forward,0', b'left,90,forward,0', b'center,0,forward,10', b'center,0,reverse,20', b'SNAP9', b'FIN']

for cmd in commands:
    ser.write(cmd)
    time.sleep(6)  # wait for robot to finish movement
b
ser.close() """



commands = [b'center,0,forward,260', b'center,0,forward,0', b'right,90,forward,0', b'center,0,forward,20', b'center,0,forward,180', b'SNAP3', b'center,0,reverse,40', b'center,0,reverse,0', b'left,90,reverse,0', b'center,0,reverse,12', b'center,0,forward,100', b'SNAP5', b'center,0,reverse,20', b'center,0,reverse,0', b'left,90,reverse,0', b'center,0,reverse,12', b'center,0,forward,140', b'SNAP7', b'center,0,reverse,80', b'center,0,forward,0', b'left,90,forward,0', b'center,0,forward,10', b'center,0,forward,200', b'center,0,forward,0', b'left,90,forward,0', b'center,0,forward,10', b'center,0,forward,100', b'SNAP9', b'FIN']
