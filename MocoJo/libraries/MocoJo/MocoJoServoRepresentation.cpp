#include <MocoJoServoRepresentation.h>
#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

int _servoID = 0;
int _servoTargetBufferAmountFresh = 0;
int _servoTargetBufferSize = 100;

MocoJoServoRepresentation::MocoJoServoRepresentation(HardwareSerial &serial, int ID){
	_servoID = ID;
	_serial = &serial;
}

boolean MocoJoServoRepresentation::handshake(){
	//returns true if handshake successful
	MocoJoServoCommunication::writeHandshakeRequestToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 6);
	initialize();
	return true;
	
}

void MocoJoServoRepresentation::initialize(){
	MocoJoServoCommunication::writeInitializeToServo(*_serial, _servoID);
}

void MocoJoServoRepresentation::exitSafeStart(){
	MocoJoServoCommunication::writeExitSafeStartToServo(*_serial, _servoID);
} 

long MocoJoServoRepresentation::getCurrentPosition(){
	MocoJoServoCommunication::writeGetCurrentPositionToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 2);
	return SerialTools::readLongFromSerial(*_serial);
}

long MocoJoServoRepresentation::getPositionAtLastSync(){
	MocoJoServoCommunication::writeGetPositionAtLastSyncToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	return SerialTools::readLongFromSerial(*_serial);
}

long MocoJoServoRepresentation::setTargetPosition(long targetPosition){
	MocoJoServoCommunication::writeSetTargetPositionToServo(*_serial, _servoID, targetPosition);
}

long MocoJoServoRepresentation::addTargetPositionToBuffer(long targetPosition){
	MocoJoServoCommunication::writeAddTargetPositionToBufferToServo(*_serial, _servoID, targetPosition);
}

boolean MocoJoServoRepresentation::targetPositionBufferIsFull(){
	return true;
}







