#include <SerialTools.h>
#include <WProgram.h>
#include <WString.h>

//All send/receive methods use MSB byte order

void SerialTools::writeShortToSerial(HardwareSerial &serial, int number){
    (&serial) -> write((uint8_t)((number >> 8) & 0XFF));
	(&serial) -> write((uint8_t)((number & 0XFF)));
}

int SerialTools::readShortFromSerial(HardwareSerial &serial){
    byte byte1 = (&serial) -> read();
    byte byte2 = (&serial) -> read();
    return (byte1 << 8) + (byte2);
}

void SerialTools::writeLongToSerial(HardwareSerial &serial, long number){
	(&serial) -> write((uint8_t)((number >> 24) & 0xFF));
	(&serial) -> write((uint8_t)((number >> 16) & 0xFF));
	(&serial) -> write((uint8_t)((number >> 8) & 0XFF));
	(&serial) -> write((uint8_t)((number & 0XFF)));
}

long SerialTools::readLongFromSerial(HardwareSerial &serial){
	byte byte1 = (&serial) -> read();
	byte byte2 = (&serial) -> read();
	byte byte3 = (&serial) -> read();
	byte byte4 = (&serial) -> read();
	
	return ((byte1 << 24) + (byte2 << 16) + (byte3 << 8) + (byte4));
}

void SerialTools::writeDummyBytesToSerial(HardwareSerial &serial, int numberOfDummyBytes){
	for (int i = 0; i < numberOfDummyBytes; i++){
		(&serial) -> write((uint8_t)0);
	}
}

void SerialTools::readDummyBytesFromSerial(HardwareSerial &serial, int numberOfDummyBytes){
	for (int i = 0; i < numberOfDummyBytes; i++){
		(&serial) -> read();
	}
}

void SerialTools::blockUntilBytesArrive(HardwareSerial &serial, int numberOfBytes){
	while( ((&serial)->available()) < numberOfBytes){
	}
}

bool SerialTools::blockUntilBytesArrive(HardwareSerial &serial, int numberOfBytes, int timeoutMillis){
	unsigned long startTime = millis();
	while( ((&serial)->available()) < numberOfBytes){
		if((millis() - startTime) > timeoutMillis){
			return true;
		}
	}
	return false;
}

