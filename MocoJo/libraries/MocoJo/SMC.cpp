#include <SMC.h>
#include <SMCProtocolConstants.h>

#include <WProgram.h>
#include <WString.h>



SMC::SMC(HardwareSerial* serial){
	_serial = serial;

}

void SMC::initialize(){
	_serial.begin(SMCProtocolBaudRate);
	delay(1);
	_serial.write(0xAA); //SMC needs to establish the baud rate
}

void SMC::exitSafeStart(){
	_serial.write(SMCProtocolExitSafeStart);
}


//speed can be [-3200, 3200]
void SMC::setMotorSpeed(int speed){

	if (speed >= 0){
		_serial.write(SMCProtocolSetMotorForward);
	}
	else {
		_serial.write(SMCProtocolSetMotorReverse);
		speed = -speed;
	}

	_serial.write(speed % 32); //speed byte 1
	_serial.write(speed >> 5); //speed byte 2
	
}

int SMC::getVariable(int variableID){
	_serial.write(SMCProtocolGetVariable);
	_serial.write(variableID);
	
	//wait for the response to arrive!
	while(_serial.available() < 2){}
	
	int lowByte = _serial.read();
	int highByte = _serial.read();
	return lowByte + 256*highByte;
}