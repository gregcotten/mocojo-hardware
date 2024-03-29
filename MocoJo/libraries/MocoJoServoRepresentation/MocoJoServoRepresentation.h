#ifndef MocoJoServoRepresentation_h
#define MocoJoServoRepresentation_h

#include <WProgram.h>

class MocoJoServoRepresentation{
	public:
		MocoJoServoRepresentation(HardwareSerial &serial, int ID);
		bool handshake(); //returns true if handshake successful
		int getServoID();
		void exitSafeStart();
		long getCurrentPosition();
		long getPositionAtLastSync();
		long getMotorTargetSpeed();
		long setTargetPosition(long targetPosition);
		void addTargetPositionToBuffer(long targetPosition);
		void startPlayback();
		void stopPlayback();
		void proceedToHone();
		bool isHoning();
		bool isInitialized();

	private:
		void initialize();
		bool _isInitialized;
		HardwareSerial* _serial; //pointer for what serial to use
};
#endif