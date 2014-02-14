#include <MocoJoServoRepresentation.h>
#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

int _servoID = 0;
bool _isInitialized = false;



MocoJoServoRepresentation::MocoJoServoRepresentation(HardwareSerial &serial, int ID){
	_servoID = ID;
	_serial = &serial;
}

//INIT
bool MocoJoServoRepresentation::handshake(){
	//returns true if handshake successful
	MocoJoServoCommunication::writeHandshakeRequestToServo(*_serial, _servoID);
	if (SerialTools::blockUntilBytesArrive(*_serial, 6, 5)){
		SerialTools::readDummyBytesFromSerial(*_serial, 6);
		return true;	
	}
	return false;
	
}

void MocoJoServoRepresentation::initialize(){
	if(handshake()){
		MocoJoServoCommunication::writeInitializeToServo(*_serial, _servoID);	
		_isInitialized = true;
	}
}

bool MocoJoServoRepresentation::isInitialized(){
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

bool MocoJoServoRepresentation::isHoning(){
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