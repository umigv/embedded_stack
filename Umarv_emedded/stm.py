
Import serial

ser =  serial.Serial{

	port= '/dev/tty50',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	bytesize=serial.EIGHTBITS,
	timeout=1000
}

while 1:
	ser.read()
