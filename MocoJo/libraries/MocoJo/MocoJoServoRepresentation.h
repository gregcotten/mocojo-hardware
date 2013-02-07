#ifndef MocoJoServoRepresentation_h
#define MocoJoServoRepresentation_h

#include <WProgram.h>

class MocoJoServoRepresentation{
	public:
		MocoJoServoRepresentation(HardwareSerial &serial, int ID);
		boolean handshake(); //returns true if handshake successful
		int getServoID();
		void exitSafeStart();
		long getCurrentPosition();
		long getPositionAtLastSync();
		long setTargetPosition(long targetPosition);
		void addTargetPositionToBuffer(long targetPosition);
		boolean targetPositionBufferIsFull();
		void playbackShutterDidFire();
		void startPlayback();
		void stopPlayback();
		void proceedToHone();
		boolean isHoning();

	private:
		void initialize();
		HardwareSerial* _serial; //pointer for what serial to use
};
#endif