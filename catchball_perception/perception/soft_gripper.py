import serial
import time

comms = serial.Serial("/dev/ttyACM0", baudrate=9600)
time.sleep(0)

msg = "on\n"
comms.write(msg.encode())

comms.close()