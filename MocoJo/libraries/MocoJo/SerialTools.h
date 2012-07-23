#ifndef SerialTools_h
#define SerialTools_h

#include <WProgram.h>
#include <WString.h>

namespace SerialTools {
	void writeIntToSerial(int number);    
    int readIntFromSerial();
	void writeLongToSerial(long number);
	long readLongFromSerial();
}

#endif
