#include <SerialTools.h>
#include <WProgram.h>
#include <WString.h>

//All send/receive methods use MSB byte order

void SerialTools::writeShortToSerial(int number){
    Serial.write((uint8_t)((number >> 8) & 0XFF));
	Serial.write((uint8_t)((number & 0XFF)));
}

int SerialTools::readShortFromSerial(){
    byte byte1 = Serial.read();
    byte byte2 = Serial.read();
    return (byte1 << 8) + (byte2);
}

void SerialTools::writeLongToSerial(long number){
	Serial.write((uint8_t)((number >> 24) & 0xFF));
	Serial.write((uint8_t)((number >> 16) & 0xFF));
	Serial.write((uint8_t)((number >> 8) & 0XFF));
	Serial.write((uint8_t)((number & 0XFF)));
}

long SerialTools::readLongFromSerial(){
	byte byte1 = Serial.read();
	byte byte2 = Serial.read();
	byte byte3 = Serial.read();
	byte byte4 = Serial.read();
	
	return ((byte1 << 24) + (byte2 << 16) + (byte3 << 8) + (byte4));
}

