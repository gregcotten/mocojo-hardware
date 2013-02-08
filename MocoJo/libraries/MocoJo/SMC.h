#ifndef SMC_h
#define SMC_h

#include <WProgram.h>

class SMC{
	public:
		SMC(HardwareSerial &serial);
		void initialize();
		void setMotorSpeed(int speed);
		void exitSafeStart();
		void stopMotor();
		void setDeadpanSpeed(int dead);
	private:
		int getVariable(int variableID);
		HardwareSerial* _serial; //pointer for what serial to use
		int _deadPanSpeed;
		
};
#endif