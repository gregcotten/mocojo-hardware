#include "AS5045.h"

#include <WProgram.h>
#include <WString.h>

int _encoderDebug;

//Pins
int _encoderChipSelectPin; //output to chip select
int _encoderClockPin; //output to clock
int _encoderInputPin; //read AS5045

//Positional Data
int _encoderResolution = 4095;
int _encoderRelativePosition;
long _encoderPreviousRelativePosition;
long _encoderAbsolutePosition;
int _encoderRevolutionCount;

//----------MAGNETIC ENCODER GENERAL-----------
int inputstream = 0; //one bit read from pin
long packeddata = 0; //two bytes concatenated from inputstream
long absPosition = 0; //holds processed angle value
long absPositionMask = 262080; // 0x111111111111000000: mask to obtain first 12 digits with position info
long statusmask = 63; // 0x000000000000111111; mask to obtain last 6 digits containing status info
long statusbits; //holds status/error information
int DECn; //bit holding decreasing magnet field error data
int INCn; //bit holding increasing magnet field error data
int OCF; //bit holding startup-valid bit
int COF; //bit holding cordic DSP processing error data
int LIN; //bit holding magnet field displacement error data
int shortdelay = 1; // this is the microseconds of delay in the data clock
int longdelay = 1; // this is the milliseconds between readings

//----------------------------------------------

AS5045::AS5045(int chipSelect, int clock, int input, int debug){
	_encoderChipSelectPin = chipSelect;
	_encoderClockPin = clock;
	_encoderInputPin = input;
	_encoderDebug = debug;
}

int AS5045::getRelativePosition(){
	return _encoderRelativePosition;
}

long AS5045::getAbsolutePosition(){
	return _encoderAbsolutePosition;
}

void AS5045::update(){
	digitalWrite(_encoderChipSelectPin, HIGH); // CSn high
  digitalWrite(_encoderClockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  //digitalWrite(ledPin, HIGH); // signal start of transfer with LED
  digitalWrite(_encoderChipSelectPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWrite(_encoderClockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low
 
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWrite(_encoderClockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream =digitalRead(_encoderInputPin); // read one bit of data from pin
    packeddata = ((packeddata << 1) + inputstream);// left-shift summing variable, add pin value
    digitalWrite(_encoderClockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  //digitalWrite(ledPin, LOW); // signal end of transmission
  
  _encoderRelativePosition = packeddata & absPositionMask; // mask rightmost 6 digits of packeddata to zero, into angle.

  _encoderRelativePosition = (_encoderRelativePosition >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  //Serial.println(controllerPanEncoder_AbsolutePosition,DEC);
  
  //detect a revolution!
  if (_encoderPreviousRelativePosition > 3900 && _encoderRelativePosition < 100) { //it did a clockwise rev
    _encoderRevolutionCount++;
  } 
  else if (_encoderPreviousRelativePosition < 100 && _encoderRelativePosition > 3900) { //it did a counter-clockwise rev
    _encoderRevolutionCount--;
  }
  

  
  if (_encoderDebug)
  {
    statusbits = packeddata & statusmask;
    DECn = statusbits & 2; // goes high if magnet moved away from IC
    INCn = statusbits & 4; // goes high if magnet moved towards IC
    LIN = statusbits & 8; // goes high for linearity alarm
    COF = statusbits & 16; // goes high for cordic overflow: data invalid
    OCF = statusbits & 32; // this is 1 when the chip startup is finished.
    if (DECn && INCn) { 
    //Serial.println("magnet moved out of range"); 
    }
    else
    {
      if (DECn) { Serial.println("magnet moved away from chip"); }
      if (INCn) { Serial.println("magnet moved towards chip"); }
    }
    if (LIN) { Serial.println("linearity alarm: magnet misaligned? Data questionable."); }
    if (COF) { Serial.println("cordic overflow: magnet misaligned? Data invalid."); }
  }

  
  
  _encoderAbsolutePosition = _encoderRelativePosition + _encoderResolution*_encoderRevolutionCount;
  
  _encoderPreviousRelativePosition = _encoderRelativePosition;
  
  packeddata = 0; // reset both variables to zero so they don't just accumulate
}

