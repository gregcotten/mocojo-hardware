#include <SMC.h>
#include <SMCProtocolConstants.h>

#include <WProgram.h>
#include <WString.h>

int _deadpanSpeed = 0;


SMC::SMC(HardwareSerial &serial, int resetPin, int errorPin){
	_serial = &serial;
	_errorPin = errorPin;
	_resetPin = resetPin;
	pinMode(_errorPin, INPUT);
}

void SMC::initialize(){
	resetController();
	_serial->write(0xAA); //SMC needs to establish the baud rate
}

boolean SMC::isError(){
	return digitalRead(_errorPin) == 1;
}

void SMC::exitSafeStart(){
	_serial->write(SMCProtocolExitSafeStart);
}

void SMC::stopMotor(){
	_serial->write(SMCProtocolStopMotor);
}

void SMC::resetController(){
	pinMode(_resetPin, OUTPUT);
	digitalWrite(_resetPin, LOW);  // reset SMC
	delay(1);  // wait 1 ms
	pinMode(_resetPin, INPUT);  // let SMC run again
	delay(5);
}

void SMC::setDeadpanSpeed(int dead){
	if (dead < 0){
		dead = 0;
	}
	else if (dead > 3200){
		_deadpanSpeed = 3200;
	}
	_deadpanSpeed = dead;	
	
}

//speed can be [-3200, 3200]
void SMC::setMotorSpeed(int speed){
	
	if (speed >= 0){
		_serial->write(SMCProtocolSetMotorForward);
	}
	else {
		_serial->write(SMCProtocolSetMotorReverse);
		speed = -speed;
	}

	if(speed < 0){
		speed = 0;
	}
	else if(speed > 3200){
		speed = 3200;
	}
	
	if(_deadpanSpeed > 0){
		speed = map(speed, 0, 3200, _deadpanSpeed, 3200);	
	}
	
	
	_serial->write(speed & 0x1F); //speed byte 1
	_serial->write(speed >> 5); //speed byte 2
	
}

int SMC::getVariable(int variableID){
	_serial->write(SMCProtocolGetVariable);
	_serial->write(variableID);
	
	//wait for the response to arrive!
	while(_serial->available() < 2){}
	
	int lowByte = _serial->read();
	int highByte = _serial->read();
	return lowByte + 256*highByte;
}