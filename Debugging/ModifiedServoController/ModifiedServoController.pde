#include <digitalWriteFast.h>
#include <Servo.h>

//DECLARATIONS

//---------------GENERAL------------------
const int ledPin = 13; //LED connected to digital pin 13
const byte MOCOJO_SERVO_ID = 0x2C //for later
//----------------------------------------


//---------------DEBUG--------------------
//1 turns the debug on, 0 turns the debug off
const int servoTerminalDebug = 0; //show PID stuff
const int servoGraphDebug = 1; //show positions and start accepting input
byte receiveBuffer[4];
const int encoderDebug = 0; //look for encoder errors
const int timingDebug = 0; //for debugging control loop frequency

long start = 0; //for timing debug
long loopCount = 0; //for timing debug
//----------------------------------------

//----------------PAN SERVO---------------------

//****GENERAL****
Servo controllerPanServo; //pan servo
const int controllerPanServo_PwmPin = 9;

const long controllerPan_MotorSpeedCenter = 1500;
long controllerPan_MotorSpeed = 1500;
long controllerPanEncoder_Position = 0;
long controllerPanEncoder_RevolutionCount = 0;
long controllerPanEncoder_AbsolutePosition = 0;
long controllerPanEncoder_PreviousAbsolutePosition = 2047; //middle point so a rev is not counted at start

//****ENCODER****
const int controllerPanEncoder_CSnPin = 2; //output to chip select
const int controllerPanEncoder_dataPin = 3; //read AS5045
const int controllerPanEncoder_clockPin = 4; //output to clock


//****PID****
long controllerPan_Target;
//correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
//PID controller constants
float controllerPan_KP = 1.8; //position multiplier (gain)
float controllerPan_KI = 0; // Intergral multiplier (gain)
float controllerPan_KD = 0; // derivative multiplier (gain)
//track the previous error for the derivitive term, and the sum of the errors for the integral term
int controllerPan_lastError = 0;
int controllerPan_sumError = 0;
//Integral term min/max (random value and not yet tested/verified)
int controllerPan_iMax = 500;
int controllerPan_iMin = 0;
//-------------------------------------------------




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


void setup()
{
  Serial.begin(1000000);
  controllerPanServo.attach(controllerPanServo_PwmPin);
  start = micros();
  pinModeFast(ledPin, OUTPUT); // visual signal of I/O to chip
  pinModeFast(controllerPanEncoder_clockPin, OUTPUT); // SCK
  pinModeFast(controllerPanEncoder_CSnPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
  pinModeFast(controllerPanEncoder_dataPin, INPUT); // SDA
  
  //temp stuff
  controllerPan_Target = 0;
}


void loop()
{
  if (timingDebug) {
    if (micros() - start > 1800) {
    Serial.println(loopCount);
    start = micros();
    loopCount = 0;
    }
    loopCount++;
  }

    updateControllerPanPID();
    updateControllerPanEncoder();
    updateControllerPanPWM();
    
    
    if (servoTerminalDebug) {
      Serial.print("Target ");
      Serial.print(controllerPan_Target);
      Serial.print(" Current ");
      Serial.print(controllerPanEncoder_Position);
      Serial.print(" Delta ");
      Serial.print(controllerPan_Target-controllerPanEncoder_Position);
      Serial.print(" PWM ");
      Serial.println(controllerPan_MotorSpeed);
    }

	//for use with ServoPositionDebugger.py
	if (servoGraphDebug) {
		if (Serial.available() > 0)
		{
			byte commandByte = Serial.read();
			/*
			if (commandByte = 252) //ID challenge byte
			{
				Serial.write("MocoJoServo");
			}
			*/
			if (commandByte == 253) //253 is the command to update target for pan
			{
				int index = 0;
				while(Serial.available()<5){} //wait for 4 bytes
				receiveBuffer[0] = Serial.read();
				receiveBuffer[1] = Serial.read();
				receiveBuffer[2] = Serial.read();
				receiveBuffer[3] = Serial.read();
				controllerPan_Target = ( (receiveBuffer[0] << 24) + (receiveBuffer[1] << 16) + (receiveBuffer[2] << 8) + (receiveBuffer[3] ) );
				if (Serial.read() == 1){ //negative number!
					controllerPan_Target = controllerPan_Target*-1;
				}
			}
			
			if (commandByte == 254) //254 is the command to get positions
			{
				Serial.print(controllerPanEncoder_Position);
				Serial.print(" ");
				Serial.println(controllerPan_Target);
			}
			Serial.flush();
		}
		
	}
    
}



void updateControllerPanPWM() {
   controllerPanServo.writeMicroseconds(controllerPan_MotorSpeed);
} 

void updateControllerPanPID() {
  long error = controllerPan_Target - controllerPanEncoder_Position; // find the error term of current position - controllerPan_Target

  // generalized PID formula
  //correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
  long ms = controllerPan_KP * error + controllerPan_KD * (error - controllerPan_lastError) +controllerPan_KI * (controllerPan_sumError);// calculate a motor speed for the current conditions

  
  // set the last and sumerrors for next loop iteration
  controllerPan_lastError = error;
  controllerPan_sumError += error;
  /*
  //scale the sum for the integral term
  if(controllerPan_sumError > controllerPan_iMax){
    controllerPan_sumError = controllerPan_iMax;
  }
  else if(controllerPan_sumError < controllerPan_iMin){
    controllerPan_sumError = controllerPan_iMin;
  }
  */
  if (ms < 0){
	controllerPan_MotorSpeed = controllerPan_MotorSpeedCenter - 15 - map(abs(ms), 0, 5000, 0, 600);
}
else if (ms > 0){
	controllerPan_MotorSpeed = controllerPan_MotorSpeedCenter + 32 + (int)(((double)map(abs(ms), 0, 5000, 0, 600))*1.7);
}
  //controllerPan_MotorSpeed = map(ms,-5185,5185,900,2100);
  
  if (abs(error) <= 4) {
    controllerPan_MotorSpeed = controllerPan_MotorSpeedCenter;
  }
}



void updateControllerPanEncoder(){
  // CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda
  
  digitalWriteFast(controllerPanEncoder_CSnPin, HIGH); // CSn high
  digitalWriteFast(controllerPanEncoder_clockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  digitalWriteFast(ledPin, HIGH); // signal start of transfer with LED
  digitalWriteFast(controllerPanEncoder_CSnPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWriteFast(controllerPanEncoder_clockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low
 
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWriteFast(controllerPanEncoder_clockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream =digitalReadFast(controllerPanEncoder_dataPin); // read one bit of data from pin
    packeddata = ((packeddata << 1) + inputstream);// left-shift summing variable, add pin value
    digitalWriteFast(controllerPanEncoder_clockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  digitalWriteFast(ledPin, LOW); // signal end of transmission
  
  controllerPanEncoder_AbsolutePosition = (packeddata & absPositionMask) >> 6; // mask rightmost 6 digits of packeddata to zero, into angle.

  //controllerPanEncoder_AbsolutePosition = (controllerPanEncoder_AbsolutePosition >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  //Serial.println(controllerPanEncoder_AbsolutePosition,DEC);
  if (controllerPanEncoder_PreviousAbsolutePosition > 4065 && controllerPanEncoder_AbsolutePosition < 30) { //it did a clockwise rev
    controllerPanEncoder_RevolutionCount++;
  } 
  else if (controllerPanEncoder_PreviousAbsolutePosition < 30 && controllerPanEncoder_AbsolutePosition > 4065) { //it did a counter-clockwise rev
    controllerPanEncoder_RevolutionCount--;
  }
  

  
  if (encoderDebug)
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

  
  
  controllerPanEncoder_Position = controllerPanEncoder_AbsolutePosition + 4095*controllerPanEncoder_RevolutionCount;
  
  controllerPanEncoder_PreviousAbsolutePosition = controllerPanEncoder_AbsolutePosition;
 // controllerPanEncoder_AbsolutePosition = 0;
  
  packeddata = 0; // reset both variables to zero so they don't just accumulate
}
