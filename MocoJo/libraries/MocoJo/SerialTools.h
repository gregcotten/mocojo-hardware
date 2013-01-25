#ifndef SerialTools_h
#define SerialTools_h

#include <WProgram.h>
#include <WString.h>

namespace SerialTools {
	void writeShortToSerial(HardwareSerial stream, int number);    
    int readShortFromSerial(HardwareSerial stream);
	void writeLongToSerial(HardwareSerial stream, long number);
	long readLongFromSerial(HardwareSerial stream);
	void writeDummyBytesToSerial(HardwareSerial stream, int numberOfBytes);
	void readDummyBytesFromSerial(HardwareSerial stream, int numberOfBytes);
}

#endif
