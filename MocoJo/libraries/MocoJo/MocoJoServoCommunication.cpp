#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

//SERVO to MCU
void MocoJoServoCommunication::writeHandshakeSuccessToMCU(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoHandshakeSuccessfulResponse);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}
	
void MocoJoServoCommunication::writeCurrentPositionToMCU(HardwareSerial &serial, int ID, long currentPosition){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoCurrentPosition);
	SerialTools::writeLongToSerial(serial, currentPosition);
}

void MocoJoServoCommunication::writePositionAtLastSyncToMCU(HardwareSerial &serial, int ID, long positionAtLastSync){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoPositionAtLastSync);
	SerialTools::writeLongToSerial(serial, positionAtLastSync);
}

//----------------------

//MCU to SERVO

//init
void MocoJoServoCommunication::writeHandshakeRequestToServo(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoHandshakeRequest);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}

void MocoJoServoCommunication::writeExitSafeStartToServo(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoExitSafeStart);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}



//limits
void MocoJoServoCommunication::writeSetMaxSpeedToServo(HardwareSerial &serial, int ID, long maxSpeed){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoSetMaxSpeed);
	SerialTools::writeLongToSerial(serial, maxSpeed);
}


//position

void MocoJoServoCommunication::writeGetCurrentPositionToServo(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoGetCurrentPosition);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}

void MocoJoServoCommunication::writeGetPositionAtLastSyncToServo(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoGetPositionAtLastSync);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}

void MocoJoServoCommunication::writeSetTargetPositionToServo(HardwareSerial &serial, int ID, long targetPosition){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoSetTargetPosition);
	SerialTools::writeLongToSerial(serial, targetPosition);
}

void MocoJoServoCommunication::writeAddTargetPositionToBufferToServo(HardwareSerial &serial, int ID, long targetPosition){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoAddTargetPositionToBuffer);
	SerialTools::writeLongToSerial(serial, targetPosition);
}

//playback
void MocoJoServoCommunication::writeStartPlaybackToServo(HardwareSerial &serial, int ID, long firstPosition){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoStartPlayback);
	SerialTools::writeLongToSerial(serial, targetPosition);
}

void MocoJoServoCommunication::writeStopPlaybackToServo(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoStopPlayback);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}
//-----------------------


