#include <SMC.h>
#include <SMCProtocolConstants.h>

#include <WProgram.h>
#include <WString.h>


SMC::SMC(){

}

void SMC::initialize(){
	Serial1.begin(SMCProtocolBaudRate);
	delay(1);
	Serial1.write(0xAA); //SMC needs to establish the baud rate
}

void SMC::exitSafeStart(){
	Serial1.write(SMCProtocolExitSafeStart);
}

void SMC::setMotorSpeed(int speed){

	if (speed >= 0){
		Serial1.write(SMCProtocolSetMotorForward);
	}
	else {
		Serial1.write(SMCProtocolSetMotorReverse);
		speed = -speed;
	}

	Serial1.write(speed % 32); //speed byte 1
	Serial1.write(speed >> 5); //speed byte 2
	
}

int SMC::getVariable(variableID){
	Serial1.write(SMCProtocolGetVariable);
	Serial1.write(variableID);
	
	//wait for the response to arrive!
	while(Serial1.available() < 2){}
	
	int lowByte = Serial1.read();
	int highByte = Serial1.read();
	return lowByte + 256*highByte;
}