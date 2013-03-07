import serial, time
def main():
	ser = serial.Serial("/dev/cu.usbserial-A6008RQE", baudrate = 1000000, timeout=1)
	time.sleep(5)

	while True:
		print ser.readline().rstrip('\n');

main()