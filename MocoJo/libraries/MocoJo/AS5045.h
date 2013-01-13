#ifndef AS5045_h
#define AS5045_h

#include <WProgram.h>
#include <WString.h>

class AS5045{
	public:
		AS5045(int chipSelect, int clock, int input, int debug);
		void update();
		int getRelativePosition();
		long getAbsolutePosition();

	private:
		int _encoderDebug;
		//Pins
		int _encoderChipSelectPin; //output to chip select
		int _encoderClockPin; //output to clock
		int _encoderInputPin; //read AS5045
		
		//Positional Data
		int _encoderResolution;
		int _encoderRelativePosition;
		long _encoderPreviousRelativePosition;
		long _encoderAbsolutePosition;
		int _encoderRevolutionCount;

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
		int longdelay; // this is the milliseconds between readings
		
}