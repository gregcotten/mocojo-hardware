#ifndef SMC_h
#define SMC_h

#include <WProgram.h>
#include <WString.h>

class SMC{
	public:
		SMC();
		void initialize();
		void setMotorSpeed(int speed);
		void exitSafeStart();
	private:

		
};
#endif