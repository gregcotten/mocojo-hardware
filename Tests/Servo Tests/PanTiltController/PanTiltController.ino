#include <digitalWriteFast.h>

//****GENERAL****
bool encoderDebug = false;
long controllerPanEncoder_Position = 0;
long controllerPanEncoder_RevolutionCount = 0;
long controllerPanEncoder_AbsolutePosition = 0;
long controllerPanEncoder_PreviousAbsolutePosition = 2047; //middle point so a rev is not counted at start

long controllerTiltEncoder_Position = 0;
long controllerTiltEncoder_RevolutionCount = 0;
long controllerTiltEncoder_AbsolutePosition = 0;
long controllerTiltEncoder_PreviousAbsolutePosition = 2047; //middle point so a rev is not counted at start

//****ENCODERS****
const int controllerPanEncoder_CSnPin = 9; //output to chip select
const int controllerPanEncoder_clockPin = 11; //output to clock
const int controllerPanEncoder_inputPin = 10; //read AS5045

const int controllerTiltEncoder_CSnPin = 11; //output to chip select
const int controllerTiltEncoder_clockPin = 12; //output to clock
const int controllerTiltEncoder_inputPin = 13; //read AS5045

//----------MAGNETIC ENCODER GENERAL-----------
int inputstream_1 = 0; //one bit read from pin
int inputstream_2 = 0; //one bit read from pin
long packeddata_1 = 0; //two bytes concatenated from inputstream
long packeddata_2 = 0; //two bytes concatenated from inputstream
long absPositionMask = 262080; // 0x111111111111000000: mask to obtain first 12 digits with position info
long statusmask = 63; // 0x000000000000111111; mask to obtain last 6 digits containing status info
long statusbits; //holds status/error information
int DECn; //bit holding decreasing magnet field error data
int INCn; //bit holding increasing magnet field error data
int OCF; //bit holding startup-valid bit
int COF; //bit holding cordic DSP processing error data
int LIN; //bit holding magnet field displacement error data
int shortdelay = 1; // this is the microseconds of delay in the data clock
int longdelay = 0; // this is the milliseconds between readings
//----------------------------------------------

void setup()
{	
	//Serial.begin(230400);
	Serial.begin(115200);
	pinModeFast(controllerPanEncoder_clockPin, OUTPUT); // SCK
	  pinModeFast(controllerPanEncoder_CSnPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
	  pinModeFast(controllerPanEncoder_inputPin, INPUT); // SDA
	
	pinModeFast(controllerTiltEncoder_clockPin, OUTPUT); // SCK
	  pinModeFast(controllerTiltEncoder_CSnPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
	  pinModeFast(controllerTiltEncoder_inputPin, INPUT); // SDA
}



void loop()
{
	Serial.print(controllerPanEncoder_AbsolutePosition);
	Serial.print(" ");
	Serial.println(controllerTiltEncoder_AbsolutePosition);
	updateControllerPanEncoder();
	updateControllerTiltEncoder();
	delay(10);
}

void updateControllerPanEncoder(){
  // CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda
  
  digitalWriteFast(controllerPanEncoder_CSnPin, HIGH); // CSn high
  digitalWriteFast(controllerPanEncoder_clockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  //digitalWriteFast(ledPin, HIGH); // signal start of transfer with LED
  digitalWriteFast(controllerPanEncoder_CSnPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWriteFast(controllerPanEncoder_clockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low
 
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWriteFast(controllerPanEncoder_clockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream_1 =digitalReadFast(controllerPanEncoder_inputPin); // read one bit of data from pin
    packeddata_1 = ((packeddata_1 << 1) + inputstream_1);// left-shift summing variable, add pin value
    digitalWriteFast(controllerPanEncoder_clockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  //digitalWriteFast(ledPin, LOW); // signal end of transmission
  
  controllerPanEncoder_AbsolutePosition = packeddata_1 & absPositionMask; // mask rightmost 6 digits of packeddata to zero, into angle.

  controllerPanEncoder_AbsolutePosition = (controllerPanEncoder_AbsolutePosition >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  //Serial.println(controllerPanEncoder_AbsolutePosition,DEC);
  
  //detect a revolution!
  if (controllerPanEncoder_PreviousAbsolutePosition > 3900 && controllerPanEncoder_AbsolutePosition < 100) { //it did a clockwise rev
    controllerPanEncoder_RevolutionCount++;
  } 
  else if (controllerPanEncoder_PreviousAbsolutePosition < 100 && controllerPanEncoder_AbsolutePosition > 3900) { //it did a counter-clockwise rev
    controllerPanEncoder_RevolutionCount--;
  }
  

  
  if (encoderDebug)
  {
    statusbits = packeddata_1 & statusmask;
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

  
  
  controllerPanEncoder_Position = controllerPanEncoder_AbsolutePosition + 4095*controllerPanEncoder_RevolutionCount;
  
  controllerPanEncoder_PreviousAbsolutePosition = controllerPanEncoder_AbsolutePosition;
 // controllerPanEncoder_AbsolutePosition = 0;
  
  packeddata_1 = 0; // reset both variables to zero so they don't just accumulate
}

void updateControllerTiltEncoder(){
  // CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda
  
  digitalWriteFast(controllerTiltEncoder_CSnPin, HIGH); // CSn high
  digitalWriteFast(controllerTiltEncoder_clockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  //digitalWriteFast(ledPin, HIGH); // signal start of transfer with LED
  digitalWriteFast(controllerTiltEncoder_CSnPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWriteFast(controllerTiltEncoder_clockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low
 
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWriteFast(controllerTiltEncoder_clockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream_2 =digitalReadFast(controllerTiltEncoder_inputPin); // read one bit of data from pin
    packeddata_2 = ((packeddata_2 << 1) + inputstream_2);// left-shift summing variable, add pin value
    digitalWriteFast(controllerTiltEncoder_clockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  //digitalWriteFast(ledPin, LOW); // signal end of transmission
  
  controllerTiltEncoder_AbsolutePosition = packeddata_2 & absPositionMask; // mask rightmost 6 digits of packeddata to zero, into angle.

  controllerTiltEncoder_AbsolutePosition = (controllerTiltEncoder_AbsolutePosition >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  //Serial.println(controllerPanEncoder_AbsolutePosition,DEC);
  
  //detect a revolution!
  if (controllerTiltEncoder_PreviousAbsolutePosition > 3900 && controllerTiltEncoder_AbsolutePosition < 100) { //it did a clockwise rev
    controllerTiltEncoder_RevolutionCount++;
  } 
  else if (controllerTiltEncoder_PreviousAbsolutePosition < 100 && controllerTiltEncoder_AbsolutePosition > 3900) { //it did a counter-clockwise rev
    controllerTiltEncoder_RevolutionCount--;
  }
  

  
  if (encoderDebug)
  {
    statusbits = packeddata_2 & statusmask;
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

  
  
  controllerTiltEncoder_Position = controllerTiltEncoder_AbsolutePosition + 4095*controllerTiltEncoder_RevolutionCount;
  
  controllerTiltEncoder_PreviousAbsolutePosition = controllerTiltEncoder_AbsolutePosition;
 // controllerTiltEncoder_AbsolutePosition = 0;
  
  packeddata_2 = 0; // reset both variables to zero so they don't just accumulate
}