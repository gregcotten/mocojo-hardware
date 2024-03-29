#include "AS5045.h"

#include <WProgram.h>
#include <WString.h>

boolean _encoderDebug;


//Pins
int _encoderChipSelectPin; //output to chip select
int _encoderClockPin; //output to clock
int _encoderInputPin; //read AS5045

//Positional Data
int _encoderRelativePosition;
int _encoderPreviousRelativePosition; //so we don't detect a revolution at the beginning
int _encoderRevolutionCount;
long _encoderPreviousAbsolutePosition;
long _encoderAbsolutePosition;
long _encoderAbsolutePositionOffset;
float _encoderSensitivity;

//Velocity Data
unsigned long _timeoutVelocityUpdate;
unsigned long _timeAtLastVelocityUpdate;
float _encoderVelocity;

//----------MAGNETIC ENCODER GENERAL-----------
int inputstream = 0; //one bit read from pin
long packeddata = 0; //two bytes concatenated from inputstream
long relativePositionMask = 262080; // 0x111111111111000000: mask to obtain first 12 digits with position info
long statusmask = 63; // 0x000000000000111111; mask to obtain last 6 digits containing status info
long statusbits; //holds status/error information
int DECn; //bit holding decreasing magnet field error data
int INCn; //bit holding increasing magnet field error data
int OCF; //bit holding startup-valid bit
int COF; //bit holding cordic DSP processing error data
int LIN; //bit holding magnet field displacement error data
int shortdelay = 1; // this is the microseconds of delay in the data clock

//----------------------------------------------

AS5045::AS5045(int chipSelect, int clockpin, int input, float sensitivity, boolean debug){
	_encoderChipSelectPin = chipSelect;
	_encoderClockPin = clockpin;
	_encoderInputPin = input;
	_encoderDebug = debug;
  _encoderSensitivity = sensitivity;

  //set pins!
  pinMode(_encoderChipSelectPin, OUTPUT);
  pinMode(_encoderClockPin, OUTPUT);
  pinMode(_encoderInputPin, INPUT);

  //catch that pesky fake revolution
  update();
  _encoderRevolutionCount = 0;
  _encoderVelocity = 0; 
  //zero out so that absolute position is zero
  setAbsolutePosition(0);
}

int AS5045::getRelativePosition(){
	return _encoderRelativePosition;
}

long AS5045::getAbsolutePosition(){
	return _encoderAbsolutePosition + _encoderAbsolutePositionOffset;
}

void AS5045::setAbsolutePosition(long desiredPosition){
  update();
  _encoderAbsolutePositionOffset = desiredPosition - _encoderAbsolutePosition;
}

float AS5045::getVelocity(){
  return _encoderVelocity;
}

//Currently takes 110 microseconds to update @ chipKIT 96MHz
void AS5045::update(){
  updatePosition();
  updateVelocity();	

  if (_encoderDebug == true){
    checkErrors();
  }
  packeddata = 0;

}

void AS5045::updateVelocity(){

  unsigned long currentTimeMillis = millis();
  if (currentTimeMillis - _timeAtLastVelocityUpdate >= 20){
    _encoderVelocity = (float)(_encoderAbsolutePosition-_encoderPreviousAbsolutePosition) / (float)(currentTimeMillis - _timeAtLastVelocityUpdate);
    _timeAtLastVelocityUpdate = currentTimeMillis;
    _encoderPreviousAbsolutePosition = _encoderAbsolutePosition;
  }
  
}

void AS5045::updatePosition(){
  digitalWrite(_encoderChipSelectPin, HIGH); // CSn high
  digitalWrite(_encoderClockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  digitalWrite(_encoderChipSelectPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWrite(_encoderClockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low

  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWrite(_encoderClockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream = digitalRead(_encoderInputPin); // read one bit of data from pin
    packeddata = ((packeddata << 1) + inputstream);// left-shift summing variable, add pin value
    digitalWrite(_encoderClockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  _encoderRelativePosition = (packeddata & relativePositionMask) >> 6; // mask rightmost 6 digits of packeddata to zero, into angle. and shift 18-digit angle right 6 digits to form 12-digit value

  //Serial.println(controllerPanEncoder_AbsolutePosition,DEC);

  //detect a revolution!
  if (_encoderPreviousRelativePosition > 3896 && _encoderRelativePosition < 200) { //it did a clockwise rev
    _encoderRevolutionCount++;
  } 
  else if (_encoderPreviousRelativePosition < 200 && _encoderRelativePosition > 3896) { //it did a counter-clockwise rev
    _encoderRevolutionCount--;
  }

  if(_encoderSensitivity != 1.0){
    _encoderAbsolutePosition = (float)(_encoderRelativePosition + 4096*_encoderRevolutionCount) * _encoderSensitivity;  
  }
  else{
    _encoderAbsolutePosition = _encoderRelativePosition + 4096*_encoderRevolutionCount;   
  }


  _encoderPreviousRelativePosition = _encoderRelativePosition;
  
  
}

void AS5045::checkErrors(){
    statusbits = packeddata & statusmask;
    DECn = statusbits & 2; // goes high if magnet moved away from IC
    INCn = statusbits & 4; // goes high if magnet moved towards IC
    LIN = statusbits & 8; // goes high for linearity alarm
    COF = statusbits & 16; // goes high for cordic overflow: data invalid
    OCF = statusbits & 32; // this is 1 when the chip startup is finished.
    if (DECn == 1 && INCn == 1) { 
    Serial.println("magnet moved out of range"); 
    }
    else
    {
      if (DECn == 1) { Serial.println("magnet moved away from chip"); }
      if (INCn == 1) { Serial.println("magnet moved towards chip"); }
    }
    if (LIN == 1) { Serial.println("linearity alarm: magnet misaligned? Data questionable."); }
    if (COF == 1) { Serial.println("cordic overflow: magnet misaligned? Data invalid."); }
}