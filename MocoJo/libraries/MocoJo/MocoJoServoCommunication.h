#ifndef MocoJoServoCommunication_h
#define MocoJoServoCommunication_h

#include <WProgram.h>

namespace MocoJoServoCommunication {
	//Servo to MCU
	void writeHandshakeSuccessToMCU(HardwareSerial &serial, int ID);
	
	void writeCurrentPositionToMCU(HardwareSerial &serial, int ID, long currentPosition);
	void writePositionAtLastSyncToMCU(HardwareSerial &serial, int ID, long positionAtLastSync);
	//--------------


	//MCU to Servo
	void writeHandshakeRequestToServo(HardwareSerial &serial, int ID);
	void writeInitializeToServo(HardwareSerial &serial, int ID);
	void writeExitSafeStartToServo(HardwareSerial &serial, int ID);
	//limits
	void writeSetMaxSpeedToServo(HardwareSerial &serial, int ID, long maxSpeed);
	//position
	void writeGetCurrentPositionToServo(HardwareSerial &serial, int ID);
	void writeGetPositionAtLastSyncToServo(HardwareSerial &serial, int ID);

	void writeSetTargetPositionToServo(HardwareSerial &serial, int ID, long targetPosition);
	void writeAddTargetPositionToBufferToServo(HardwareSerial &serial, int ID, long targetPosition);
	//playback
	void writeStartPlaybackToServo(HardwareSerial &serial, int ID);
	void writeStopPlaybackToServo(HardwareSerial &serial, int ID);
	void writeProceedToHoneToServo(HardwareSerial &serial, int ID);
	//--------------
}

#endif