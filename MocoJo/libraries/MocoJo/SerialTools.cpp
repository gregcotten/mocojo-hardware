#include <SerialTools.h>
#include <WProgram.h>
#include <WString.h>

//All send/receive methods use MSB byte order

void SerialTools::writeShortToSerial(HardwareSerial stream, int number){
    stream.write((uint8_t)((number >> 8) & 0XFF));
	stream.write((uint8_t)((number & 0XFF)));
}

int SerialTools::readShortFromSerial(HardwareSerial stream){
    byte byte1 = stream.read();
    byte byte2 = stream.read();
    return (byte1 << 8) + (byte2);
}

void SerialTools::writeLongToSerial(HardwareSerial stream, long number){
	stream.write((uint8_t)((number >> 24) & 0xFF));
	stream.write((uint8_t)((number >> 16) & 0xFF));
	stream.write((uint8_t)((number >> 8) & 0XFF));
	stream.write((uint8_t)((number & 0XFF)));
}

long SerialTools::readLongFromSerial(HardwareSerial stream){
	byte byte1 = stream.read();
	byte byte2 = stream.read();
	byte byte3 = stream.read();
	byte byte4 = stream.read();
	
	return ((byte1 << 24) + (byte2 << 16) + (byte3 << 8) + (byte4));
}

void SerialTools::writeDummyBytesToSerial(HardwareSerial stream, int numberOfDummyBytes){
	for (int i = 0; i < numberOfDummyBytes; i++){
		stream.write((uint8_t)1);
	}
}

void SerialTools::readDummyBytesFromSerial(HardwareSerial stream, int numberOfDummyBytes){
	for (int i = 0; i < numberOfDummyBytes; i++){
		stream.read();
	}
}

