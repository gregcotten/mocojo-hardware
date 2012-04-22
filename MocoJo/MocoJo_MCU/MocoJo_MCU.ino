#include <digitalWriteFast.h>
#include <Servo.h>
#include <MsTimer2.h>
#include <MocoProtocolConstants.h>
//#include <MocoProtocolJoAxis.h>

//DECLARATIONS

//---------------GENERAL------------------
const int ledPin = 13; //LED connected to digital pin 13
const String  MocoProtocolJoMCU_ID = "MocoProtocolHandshakeConfirm"; //for later
//----------------------------------------

//MCU stuff
const boolean isSlave = true;
boolean isInitialized = false;
boolean isStreaming = false;
boolean isPlayback = false;
boolean firstLoop = true;
const int framesPerSecond = 50;

long tilt_nextTarget = 0; //temp

//---------------DEBUG--------------------
//1 turns the debug on, 0 turns the debug off
const int servoTerminalDebug = 0; //show PID stuff
const int servoGraphDebug = 0; //show positions and start accepting input
byte receiveBuffer[4];
const int encoderDebug = 0; //look for encoder errors
const int timingDebug = 0; //for debugging control loop frequency

long start = 0; //for timing debug
long loopCount = 0; //for timing debug
//----------------------------------------

long controllerTiltEncoder_Offset;
long controllerTiltEncoder_Position = 0;
long controllerTiltEncoder_RevolutionCount = 0;
long controllerTiltEncoder_AbsolutePosition = 0;
long controllerTiltEncoder_PreviousAbsolutePosition = 2047; //middle point so a rev is not counted at start
int inputstream_2 = 0; //one bit read from pin
long packeddata_2 = 0; //two bytes concatenated from inputstream
const int controllerTiltEncoder_CSnPin = 5; //output to chip select
const int controllerTiltEncoder_clockPin = 6; //output to clock
const int controllerTiltEncoder_inputPin = 7; //read AS5045




//----------------TILT SERVO---------------------

//****GENERAL****
Servo tiltServo; //tilt servo
const int tiltServo_PwmPin = 11;

const long tilt_MotorSpeedCenter = 1500;
long tilt_MotorSpeed = 1500;
long tiltEncoder_Position = 0;
long tiltEncoder_Resolution = 4095;
long tiltEncoder_RevolutionCount = 0;
long tiltEncoder_AbsolutePosition = 0;
long tiltEncoder_PreviousAbsolutePosition = 2047; //middle point so a rev is not counted at start

//

//****ENCODER****
const int tiltEncoder_CSnPin = 2; //output to chip select
const int tiltEncoder_dataPin = 4; //read AS5045
const int tiltEncoder_clockPin = 3; //output to clock

//****PID****
long tilt_Target;

//correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
//PID controller constants
float tilt_KP = 1.8; //position multiplier (gain)
float tilt_KI = 0; // Intergral multiplier (gain)
float tilt_KD = 0; // derivative multiplier (gain)
//track the previous error for the derivitive term, and the sum of the errors for the integral term
int tilt_lastError = 0;
int tilt_sumError = 0;
//Integral term min/max (random value and not yet tested/verified)
int tilt_iMax = 500;
int tilt_iMin = 0;
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
	//Serial.begin(230400);
  Serial.begin(MocoProtocolBaudRate);
Serial.flush();
  tiltServo.attach(tiltServo_PwmPin);
  start = micros();
  pinModeFast(ledPin, OUTPUT); // visual signal of I/O to chip
  digitalWriteFast(ledPin, LOW);
  pinModeFast(tiltEncoder_clockPin, OUTPUT); // SCK
  pinModeFast(tiltEncoder_CSnPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
  pinModeFast(tiltEncoder_dataPin, INPUT); // SDA

	pinModeFast(controllerTiltEncoder_clockPin, OUTPUT); // SCK
	  pinModeFast(controllerTiltEncoder_CSnPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
	  pinModeFast(controllerTiltEncoder_inputPin, INPUT); // SDA
  
  //temp stuff
  tilt_Target = 0;
isInitialized = false;
//delay(2000);
}

void loop()
{
	if (isSlave){
		if(!isInitialized){
			if(Serial.read() == MocoProtocolRequestHandshakeInstruction)
			{
				initSlaveMCU();
				writeHandshakeSuccessToComputer();
				digitalWriteFast(ledPin, HIGH);
			}
		}
		else {
			if(Serial.available()){
				byte receivedByte = Serial.read();
				if (receivedByte == MocoProtocolStartSendingAxisDataInstruction){
					startLiveDataStreamToComputer();
				}
				else if (receivedByte == MocoProtocolStopSendingAxisDataInstruction){
					stopLiveDataStreamToComputer();
				}
				else if (receivedByte == MocoProtocolStartPlaybackInstruction){
					if (isStreaming){
						stopLiveDataStreamToComputer();
					}
					startPlaybackFromComputer();
				}
				else if (receivedByte == MocoProtocolStopPlaybackInstruction){
					stopPlaybackFromComputer();
				}
				else if (receivedByte == MocoProtocolRequestAxisResolutionDataInstruction){
					writeAxisResolutionsToComputer();
				}
			}
			
		}
	}
	

	
	if (!isPlayback){
		updateControllerTiltEncoder();
		if (firstLoop){
			firstLoop=false;
		//	controllerTiltEncoder_Offset = tiltEncoder_Position;
		}
		tilt_Target = controllerTiltEncoder_Position;// - controllerTiltEncoder_Offset;
	}
	

	
	
    updateTiltPID();
    updateTiltEncoder();
    updateTiltPWM();
    
	if (timingDebug) {
	    if (micros() - start > 1800) {
	    Serial.println(loopCount);
	    start = micros();
	    loopCount = 0;
	    }
	    loopCount++;
	  }
    
    if (servoTerminalDebug) {
      Serial.print("Target ");
      Serial.print(tilt_Target);
      Serial.print(" Current ");
      Serial.print(tiltEncoder_Position);
      Serial.print(" Delta ");
      Serial.print(tilt_Target-tiltEncoder_Position);
      Serial.print(" PWM ");
      Serial.println(tilt_MotorSpeed);
    }
    
}

void initSlaveMCU()
{
	isInitialized = true;
}

void startLiveDataStreamToComputer()
{
	isStreaming = true;
	MsTimer2::set(20, writeAxisPositionsToComputer);
	MsTimer2::start();
}

void stopLiveDataStreamToComputer()
{
	isStreaming = false;
	MsTimer2::stop();
}

void startPlaybackFromComputer()
{
	isPlayback = true;
	//needs to seek to first position
	MsTimer2::set(20, updateAxisPositionsFromPlayback);
	MsTimer2::start();
}

void stopPlaybackFromComputer()
{
	isPlayback = false;
	MsTimer2::stop();
}

byte byte1;
byte byte2;
byte byte3;
byte byte4;

void updateAxisPositionsFromPlayback()
{
	//EVENTUALLY WHEN WE GET ALL THE MCUs
	//1. pulse logic high to all axis servos to notify servos to change to newly received position
	// ALSO maybe get current position data from all these servos 
	//2. request next playback frame -> receive position data
	//3. distribute position data to servos
	
	
	//FOR NOW:
	tilt_Target = tilt_nextTarget; //effectively a virtual sync
	writeRequestsForNextPositionToComputer();
	readNextPositionsFromComputer();

	
	
}

void writeRequestsForNextPositionToComputer()
{
	//TEMP - eventually will interate through all online axes
	Serial.write(MocoProtocolAdvancePositionRequestType); //request next positions
	Serial.write(MocoAxisCameraTilt);
	serialWriteLong(tiltEncoder_Position);
}

void readNextPositionsFromComputer()
{
	//TEMP - eventually will interate through all online axes
	Serial.read();
	Serial.read();
	tilt_nextTarget = serialReadLong();
}

void writeAxisPositionsToComputer()
{
	//TEMP - eventually will interate through all online axes
	Serial.write(MocoProtocolAxisPositionResponseType);//we are sending axis position data
	Serial.write(MocoAxisCameraTilt);//we are saying this is the tilt
	serialWriteLong(tiltEncoder_Position);

}

void writeHandshakeSuccessToComputer()
{
	Serial.write(MocoProtocolHandshakeResponseType);
	Serial.write(1);
	serialWriteLong((long)MocoProtocolHandshakeSuccessfulResponse);
}




void writeAxisResolutionsToComputer()
{
	//TEMP:
	Serial.write(MocoProtocolAxisResolutionResponseType);
	Serial.write(MocoAxisCameraTilt);//we are saying this is the tilt
	serialWriteLong(tiltEncoder_Resolution);
	//-----
}

void serialWriteLong(long number)
{
	Serial.write((uint8_t)((number >> 24) & 0xFF));
	Serial.write((uint8_t)((number >> 16) & 0xFF));
	Serial.write((uint8_t)((number >> 8) & 0XFF));
	Serial.write((uint8_t)((number & 0XFF)));
}

long serialReadLong()
{
	byte byte1 = Serial.read();
	byte byte2 = Serial.read();
	byte byte3 = Serial.read();
	byte byte4 = Serial.read();
	
	return ((byte1 << 24) + (byte2 << 16) + (byte3 << 8) + (byte4));
}



void updateTiltPWM() {
   tiltServo.writeMicroseconds(tilt_MotorSpeed);
} 

void updateTiltPID() {
  long error = tilt_Target - tiltEncoder_Position; // find the error term of current position - tilt_Target

  // generalized PID formula
  //correction = Kp * error + Kd * (error - prevError) + kI * (sum of errors)
  long pidResult = tilt_KP * error + tilt_KD * (error - tilt_lastError) +tilt_KI * (tilt_sumError);// calculate a motor speed for the current conditions

  
  // set the last and sumerrors for next loop iteration
  tilt_lastError = error;
  tilt_sumError += error;
  /*
  //scale the sum for the integral term
  if(tilt_sumError > tilt_iMax){
    tilt_sumError = tilt_iMax;
  }
  else if(tilt_sumError < tilt_iMin){
    tilt_sumError = tilt_iMin;
  }
  */
  if (pidResult < 0){
	tilt_MotorSpeed = tilt_MotorSpeedCenter - 15 - map(abs(pidResult), 0, 5000, 0, 600);
}
else if (pidResult > 0){
	tilt_MotorSpeed = tilt_MotorSpeedCenter + 30 + (int)(((double)map(abs(pidResult), 0, 5000, 0, 600))*1.7);
}
  //tilt_MotorSpeed = map(ms,-5185,5185,900,2100);
  
  if (abs(error) <= 5) {
    tilt_MotorSpeed = tilt_MotorSpeedCenter;
  }
}



void updateTiltEncoder(){
  // CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda
  
  digitalWriteFast(tiltEncoder_CSnPin, HIGH); // CSn high
  digitalWriteFast(tiltEncoder_clockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  //digitalWriteFast(ledPin, HIGH); // signal start of transfer with LED
  digitalWriteFast(tiltEncoder_CSnPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWriteFast(tiltEncoder_clockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low
 
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWriteFast(tiltEncoder_clockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream =digitalReadFast(tiltEncoder_dataPin); // read one bit of data from pin
    packeddata = ((packeddata << 1) + inputstream);// left-shift summing variable, add pin value
    digitalWriteFast(tiltEncoder_clockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  //digitalWriteFast(ledPin, LOW); // signal end of transmission
  
  tiltEncoder_AbsolutePosition = (packeddata & absPositionMask) >> 6; // mask rightmost 6 digits of packeddata to zero, into angle.

  //tiltEncoder_AbsolutePosition = (tiltEncoder_AbsolutePosition >> 6); // shift 18-digit angle right 6 digits to form 12-digit value
  //Serial.println(tiltEncoder_AbsolutePosition,DEC);
  if (tiltEncoder_PreviousAbsolutePosition > 4065 && tiltEncoder_AbsolutePosition < 30) { //it did a clockwise rev
    tiltEncoder_RevolutionCount++;
  } 
  else if (tiltEncoder_PreviousAbsolutePosition < 30 && tiltEncoder_AbsolutePosition > 4065) { //it did a counter-clockwise rev
    tiltEncoder_RevolutionCount--;
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

  
  
  tiltEncoder_Position = tiltEncoder_AbsolutePosition + 4095*tiltEncoder_RevolutionCount;
  
  tiltEncoder_PreviousAbsolutePosition = tiltEncoder_AbsolutePosition;
 // tiltEncoder_AbsolutePosition = 0;
  
  packeddata = 0; // reset both variables to zero so they don't just accumulate
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


