#ifndef SerialTools_h
#define SerialTools_h

#include <WProgram.h>

namespace SerialTools {
	void writeShortToSerial(HardwareSerial &serial, int number);    
    int readShortFromSerial(HardwareSerial &serial);
	void writeLongToSerial(HardwareSerial &serial, long number);
	long readLongFromSerial(HardwareSerial &serial);
	void writeDummyBytesToSerial(HardwareSerial &serial, int numberOfBytes);
	void readDummyBytesFromSerial(HardwareSerial &serial, int numberOfBytes);
	void blockUntilBytesArrive(HardwareSerial &serial, int numberOfBytes);
	void blockUntilBytesArrive(HardwareSerial &serial, int numberOfBytes, int timeoutMillis);
}

#endif
