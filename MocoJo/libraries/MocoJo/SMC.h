#ifndef SMC_h
#define SMC_h

#include <WProgram.h>

class SMC{
	public:
		SMC(HardwareSerial &serial, int resetPin, int errorPin);
		void initialize();
		boolean isError();
		void setMotorSpeed(int speed);
		void exitSafeStart();
		void stopMotor();
		void setDeadpanSpeed(int dead);
	private:
		int getVariable(int variableID);
		void resetController();
		HardwareSerial* _serial; //pointer for what serial to use
		int _deadPanSpeed;
		int _errorPin;
		int _resetPin;
};
#endif