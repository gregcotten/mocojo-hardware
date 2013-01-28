#include <SerialTools.h>

void setup(){
	Serial.begin(115200);
	SerialTools::writeLongToSerial(Serial, 124024);
}

void loop(){
	delay(1000);
}