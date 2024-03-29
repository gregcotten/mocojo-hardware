#include <SMC.h>
#include <SMCProtocolConstants.h>
#include <MathHelper.h>
//#include <Arduino.h>
#include <WProgram.h>

SMC::SMC(HardwareSerial &serial, int resetPin, int errorPin){
	_serial = &serial;
	_errorPin = errorPin;
	_resetPin = resetPin;
	//default min/max
	_minimumSpeed = 0;
	_maximumSpeed = 1.0;
}

void SMC::initialize(){
	resetController();
	_serial->write(0xAA); //SMC needs to establish the baud rate
}

bool SMC::didError(){
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
	delay(1);
	pinMode(_resetPin, INPUT);  // let SMC run again
	delay(5);
}

void SMC::setMinimumSpeed(float min){
	_minimumSpeed = MathHelper::clamp(min, 0.0, 1.0);	
	
}

void SMC::setMaximumSpeed(float max){
	_maximumSpeed = MathHelper::clamp(max, 0.0, 1.0);
}

//speed can be [-1.0,1.0]
void SMC::setMotorSpeed(float speed){
	if (speed >= 0){
		_serial->write(SMCProtocolSetMotorForward);
	}
	else {
		_serial->write(SMCProtocolSetMotorReverse);
	}
	speed = MathHelper::clamp(MathHelper::absvalue(speed), _minimumSpeed, _maximumSpeed);
	int speedAsInt = (int)(speed*3200.0);
	
	_serial->write(speedAsInt & 0x1F); //speed byte 1
	_serial->write(speedAsInt >> 5); //speed byte 2
	
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