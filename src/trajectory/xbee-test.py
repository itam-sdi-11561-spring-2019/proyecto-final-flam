import serial
import numpy as np

PORT = '/dev/ttyUSB0'
BAUD = 9600

serial = serial.Serial(PORT, BAUD)

right = np.uint8(0)

while(right >= 0):
    right = np.uint8(raw_input("Ticks/seg derecho: "))
    dir_r = np.uint8(raw_input("Direccion: "))    
    left = np.uint8(raw_input("Ticks/seg izq: "))
    dir_l = np.uint8(raw_input("Direccion: "))

    serial.write(bytearray([right, dir_r, left, dir_l]))
