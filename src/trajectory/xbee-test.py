import serial

PORT = '/dev/ttyUSB0'
BAUD = 9600

serial_port = serial.Serial(PORT, BAUD)

theta = 0

while(theta >= 0):
    theta = float(raw_input("Ingresa un ángulo: "))
    print(theta)
    serial.write(theta)