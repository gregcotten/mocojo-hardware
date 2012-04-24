import serial, time

dev = serial.Serial('/dev/tty.usbserial-A6008RQE', baudrate = 1000000, timeout=4)
time.sleep(6)

while True:
	print dev.readline().strip()
