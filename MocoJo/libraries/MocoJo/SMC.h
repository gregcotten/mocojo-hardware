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
		void setMinimumSpeed(int min);
		void setMaximumSpeed(int max);
	private:
		int getVariable(int variableID);
		void resetController();
		HardwareSerial* _serial; //pointer for what serial to use
		int _minimumSpeed;
		int _maximumSpeed;
		int _errorPin;
		int _resetPin;
};
#endif