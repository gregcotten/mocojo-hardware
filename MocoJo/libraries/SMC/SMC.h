#ifndef SMC_h
#define SMC_h

//#include <Arduino.h>
#include <WProgram.h>
#include <HardwareSerial.h>
class SMC{
	public:
		SMC(HardwareSerial &serial, int resetPin, int errorPin);
		void initialize();
		bool didError();
		void setMotorSpeed(float speed);
		void exitSafeStart();
		void stopMotor();
		void setMinimumSpeed(float min);
		void setMaximumSpeed(float max);
	private:
		int getVariable(int variableID);
		void resetController();
		HardwareSerial* _serial;
		float _minimumSpeed;
		float _maximumSpeed;
		int _errorPin;
		int _resetPin;
};
#endif