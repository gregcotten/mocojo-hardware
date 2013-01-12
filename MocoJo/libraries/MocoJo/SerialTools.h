#ifndef SerialTools_h
#define SerialTools_h

#include <WProgram.h>
#include <WString.h>

namespace SerialTools {
	void writeShortToSerial(int number);    
    int readShortFromSerial();
	void writeLongToSerial(long number);
	long readLongFromSerial();
	void writeDummyBytesToSerial(int numberOfBytes);
}

#endif
