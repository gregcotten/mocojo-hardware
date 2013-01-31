#ifndef MocoJoServoCommunication_h
#define MocoJoServoCommunication_h

#include <WProgram.h>

namespace MocoJoServoCommunication {
	//Servo to MCU
	void writeHandshakeSuccessToMCU(HardwareSerial &serial, int ID);
	
	void writeCurrentPositionToMCU(HardwareSerial &serial, int ID, long currentPosition);
	void writePositionAtLastSyncToMCU(HardwareSerial &serial, int ID, long positionAtLastSync);

	void writeMocoJoServoDidHoneToFirstPosition(HardwareSerial &serial, int ID);
	//--------------


	//MCU to Servo
	void writeHandshakeRequestToServo(HardwareSerial &serial, int ID);
	void MocoJoServoCommunication::writeExitSafeStartToServo(HardwareSerial &serial, int ID);
	//limits
	void MocoJoServoCommunication::writeSetMaxSpeedToServo(HardwareSerial &serial, int ID, long maxSpeed);
	//position
	void MocoJoServoCommunication::writeSetTargetPositionToServo(HardwareSerial &serial, int ID, long targetPosition);
	void MocoJoServoCommunication::writeAddTargetPositionToBufferToServo(HardwareSerial &serial, int ID, long targetPosition);
	//playback
	void MocoJoServoCommunication::writeStartPlaybackToServo(HardwareSerial &serial, int ID, long firstPosition);
	void MocoJoServoCommunication::writeStopPlaybackToServo(HardwareSerial &serial, int ID);
	//--------------
}

#endif