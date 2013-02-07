#include <MocoJoServoRepresentation.h>
#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

int _servoID = 0;
int _servoTargetBufferAmountFresh = 0;
int _servoTargetBufferSize = 100;
int _targetBufferAmountFresh = 0;


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
}

//----------------

//PLAYBACK
void MocoJoServoRepresentation::startPlayback(){
	MocoJoServoCommunication::writeStartPlaybackToServo(*_serial, _servoID);
} 

void MocoJoServoRepresentation::stopPlayback(){
	MocoJoServoCommunication::writeStopPlaybackToServo(*_serial, _servoID);
}

void MocoJoServoRepresentation::playbackShutterDidFire(){
	_targetBufferAmountFresh--;
}

boolean MocoJoServoRepresentation::targetPositionBufferIsFull(){
	if (_targetBufferAmountFresh == MocoJoServoBufferSize){
		return true;
	}
	return false;
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

long MocoJoServoRepresentation::getPositionAtLastSync(){
	MocoJoServoCommunication::writeGetPositionAtLastSyncToServo(*_serial, _servoID);
	SerialTools::blockUntilBytesArrive(*_serial, 6);
	SerialTools::readDummyBytesFromSerial(*_serial, 2);
	return SerialTools::readLongFromSerial(*_serial);
}

boolean MocoJoServoRepresentation::isHoning(){
	//implement later!
	return true;
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
	_targetBufferAmountFresh++;
}
//-----------------