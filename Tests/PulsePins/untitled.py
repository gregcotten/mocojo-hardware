import serial, time




def base256_encode(n, minwidth=0): # int/long to byte array
	if n > 0:
		arr = []
		while n:
			n, rem = divmod(n, 256)
			arr.append(rem)
		b = bytearray(reversed(arr))
	elif n == 0:
		b = bytearray(b'\x00')
	else:
		raise ValueError

	if minwidth > 0 and len(b) < minwidth: # zero padding needed?
		b = (minwidth-len(b)) * '\x00' + b
	return b
	
ser = serial.Serial('/dev/cu.usbserial-A800H22L', baudrate = 115200, timeout=1)
time.sleep(2)

while True:
	ser.write(chr(254))
	print ser.readline()
	ser.write(chr(253))
	ser.write(base256_encode(1060, 4))
	ser.write(chr(1))
	time.sleep(.1)