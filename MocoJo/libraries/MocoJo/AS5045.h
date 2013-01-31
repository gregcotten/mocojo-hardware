#ifndef AS5045_h
#define AS5045_h

#include <WProgram.h>
#include <WString.h>

class AS5045{
	public:
		AS5045(int chipSelect, int clock, int input, float sensitivity, int debug);
		void update();
		int getRelativePosition();
		long getAbsolutePosition();
		void setAbsolutePosition(long desiredPosition);
		float getVelocity();
	private:
		void checkErrors();
		
		

		int _encoderDebug;
		//Pins
		int _encoderChipSelectPin; //output to chip select
		int _encoderClockPin; //output to clock
		int _encoderInputPin; //read AS5045
		
		//Positional Data
		int _encoderRelativePosition;
		int _encoderPreviousRelativePosition;
		long _encoderPreviousAbsolutePosition;
		long _encoderAbsolutePosition;
		long _encoderAbsolutePositionOffset;
		int _encoderRevolutionCount;
		float _encoderSensitivity;

		//Velocity Data
		long _timeInMillisecondsAtLastUpdate;
		float _encoderVelocity;

		//Backend
		int inputstream; //one bit read from pin
		long packeddata; //two bytes concatenated from inputstream
		long absPosition; //holds processed angle value
		long absPositionMask; // 0x111111111111000000: mask to obtain first 12 digits with position info
		long statusmask; // 0x000000000000111111; mask to obtain last 6 digits containing status info
		long statusbits; //holds status/error information
		int DECn; //bit holding decreasing magnet field error data
		int INCn; //bit holding increasing magnet field error data
		int OCF; //bit holding startup-valid bit
		int COF; //bit holding cordic DSP processing error data
		int LIN; //bit holding magnet field displacement error data
		int shortdelay; // this is the microseconds of delay in the data clock
		
};
#endif