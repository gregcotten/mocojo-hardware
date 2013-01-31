#ifndef MocoJoServoRepresentation_h
#define MocoJoServoRepresentation_h

#include <WProgram.h>

class MocoJoServoRepresentation{
	public:
		MocoJoServoRepresentation(HardwareSerial &serial, int ID);
		boolean handshake(); //returns true if handshake successful
		void exitSafeStart();
		long getCurrentPosition();
		long getPositionAtLastSync();
		long setTargetPosition(long targetPosition);
		long addTargetPositionToBuffer(long targetPosition);
		boolean targetPositionBufferIsFull();

	private:
		void initialize();
		int servoID;
		int servoTargetBufferAmountFresh;
		int servoTargetBufferSize;
		HardwareSerial* _serial; //pointer for what serial to use
};
#endif