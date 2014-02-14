#include <MocoJoServoRepresentation.h>
#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

int _servoID = 0;
boolean _isInitialized = false;



MocoJoServoRepresentation::MocoJoServoRepresentation(HardwareSerial &serial, int ID){
	_servoID = ID;
	_serial = &serial;
}

//INIT
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
	_isInitialized = true;
}

boolean MocoJoServoRepresentation::isInitialized(){
	return _isInitialized;
}
//----------------

//PLAYBACK
void MocoJoServoRepresentation::startPlayback(){
	MocoJoServoCommunication::writeStartPlaybackToServo(*_serial, _servoID);
} 

void MocoJoServoRepresentation::stopPlayback(){
	MocoJoServoCommunication::writeStopPlaybackToServo(*_serial, _servoID);
}



//-----------------

//NOTIFIERS
void MocoJoServoRepresentation::proceedToHone(){
	MocoJoServoCommunication::writeProceedToHoneToServo(*_serial, _servoID);
} 
//-----------------

//GETTERS

int MocoJoServoRepresentation::getServoID(){
	return _servoID;
}


long MocoJoServoRepresentation::getCurrentPosition(){
	MocoJoServoCommunication::writeGetCurrentPositionToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 2);
	return SerialTools::readLongFromSerial(*_serial);
}

long MocoJoServoRepresentation::getMotorTargetSpeed(){
	MocoJoServoCommunication::writeGetMotorTargetSpeedToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 2);
	return SerialTools::readLongFromSerial(*_serial);
}


long MocoJoServoRepresentation::getPositionAtLastSync(){
	MocoJoServoCommunication::writeGetPositionAtLastSyncToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 2);
	return SerialTools::readLongFromSerial(*_serial);
}

boolean MocoJoServoRepresentation::isHoning(){
	MocoJoServoCommunication::writeGetIsHoningToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 2);
	return SerialTools::readLongFromSerial(*_serial) == 1;
}

//-----------

//SETTERS

long MocoJoServoRepresentation::setTargetPosition(long targetPosition){
	MocoJoServoCommunication::writeSetTargetPositionToServo(*_serial, _servoID, targetPosition);
}

void MocoJoServoRepresentation::exitSafeStart(){
	MocoJoServoCommunication::writeExitSafeStartToServo(*_serial, _servoID);
} 

void MocoJoServoRepresentation::addTargetPositionToBuffer(long targetPosition){
	MocoJoServoCommunication::writeAddTargetPositionToBufferToServo(*_serial, _servoID, targetPosition);
}
//-----------------