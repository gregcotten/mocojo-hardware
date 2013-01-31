#include <MocoJoServoRepresentation.h>
#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

MocoJoServoRepresentation::MocoJoServoRepresentation(HardwareSerial &serial, int ID){
	servoID = ID;
	_serial = &serial;
	_serial->begin(MocoJoServoBaudRate);
}

boolean MocoJoServoRepresentation::handshake(){
	//returns true if handshake successful
	MocoJoServoCommunication::writeHandshakeRequestToServo(_serial, servoID);
	SerialTools::blockUntilBytesArrive(6, 100);

} 
long MocoJoServoRepresentation::getCurrentPosition();
long MocoJoServoRepresentation::getPositionAtLastSync();
long MocoJoServoRepresentation::setTargetPosition(long targetPosition);
long MocoJoServoRepresentation::addTargetPositionToBuffer(long targetPosition);
boolean MocoJoServoRepresentation::targetPositionBufferIsFull();