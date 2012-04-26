#include <Servo.h>
#include <MocoTimer1.h>
#include <MocoProtocolConstants.h>

#define FRAMERATE 50

//DECLARATIONS

//---------------GENERAL------------------
const int ledPin = 13; //LED connected to digital pin 13
const int ledPin2 = 43; //LED connected to digital pin 13
//----------------------------------------

//---------------DEBUG--------------------
//1 turns the debug on, 0 turns the debug off
const int servoTerminalDebug = 0; //show PID stuff
const int servoGraphDebug = 0; //show positions and start accepting input
byte receiveBuffer[4];
const int encoderDebug = 0; //look for encoder errors
const int timingDebug = 0; //for debugging control loop frequency

long start = 0; //for timing debug
long loopCount = 0; //for timing debug

boolean messageDebug = false;
boolean messageDebugHighPriority = true;
//----------------------------------------


//---------------MCU LOGIC--------------------
boolean firstBoot = true;
const boolean isSlave = true;
boolean isInitialized = false;
boolean isStreaming = false;
boolean isPlayback = false;

//PLAYBACK
long finalFrame;
long frameCounter;
boolean freshRequestSentForNextAxisPositionToComputer;
//----------------------------------------------


//---------------AXIS DATA--------------------
//TODO: Write MocoAxis class
long axis_Position = 0;
long axis_Resolution = 8*4095;
long axis_Target = 0;
long axis_TargetOffset;

int bufferSize = FRAMERATE*2; //2 second buffer
int bufferLowSizeWarning = FRAMERATE/2; 
long axis_TargetBuffer[FRAMERATE*2];
volatile int axis_TargetBuffer_AmountFreshData = 0;
long axis_TargetBuffer_currentPosition = 0;
long axis_TargetBuffer_currentBufferPosition = 0;
//-------------------------------------------



//---------------WHEELS--------------------
long controllerTiltEncoder_Position = 0;
long controllerTiltEncoder_RevolutionCount = 0;
long controllerTiltEncoder_AbsolutePosition = 0;
long controllerTiltEncoder_PreviousAbsolutePosition = 2047; //middle point so a rev is not counted at start
int inputstream_2 = 0; //one bit read from pin
long packeddata_2 = 0; //two bytes concatenated from inputstream
const int controllerTiltEncoder_CSnPin = 5; //output to chip select
const int controllerTiltEncoder_clockPin = 6; //output to clock
const int controllerTiltEncoder_inputPin = 7; //read AS5045
//-------------------------------------------



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
	Serial.begin(MocoProtocolBaudRate);
	Serial.flush();
	pinMode(ledPin, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	
	pinMode(controllerTiltEncoder_clockPin, OUTPUT); // SCK
	pinMode(controllerTiltEncoder_CSnPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
	pinMode(controllerTiltEncoder_inputPin, INPUT); // SDA

}

void loop()
{
	//make offsets for each axis's controller so that we don't make the servos jump
	if (firstBoot){
		axis_TargetOffset = controllerTiltEncoder_Position - axis_Position;
		firstBoot = false;
	}
	
	doSerialDuties();
	doGeneralDuties();
}

/*
	General duties are anything that needs to happen with some sort of immediacy (updating positions, controllers, etc.).
*/
void doGeneralDuties(){
	
	//update pan/tilt wheels and any other inputs
	updateControllerTiltEncoder(); 
	
	//refresh positions
	if (!isPlayback){
		axis_Target = controllerTiltEncoder_Position - axis_TargetOffset;
	}
}

/*
	All serial communication starts here.
*/
void doSerialDuties()
{
	if (isSlave){
		if(isInitialized){
			if(Serial.available()){
				processInstructionFromComputer(Serial.read());
			}
		}
		else {
			if(Serial.read() == MocoProtocolRequestHandshakeInstruction)
			{
				initSlaveMCU();
				writeHandshakeSuccessToComputer();
			}

		}
	}
}

/*
	Used to initialize any variables associated with the MCU - called when handshake succeeds.
*/
void initSlaveMCU()
{
	isInitialized = true;
	digitalWrite(ledPin2, HIGH); //visual indication of initialization
}

/*
	Used to deinitialize anything on the MCU - turns it back to a "blank slate".
*/
void deinitSlaveMCU()
{
	isInitialized = false;
	
	//stop everything!!!
	stopPlaybackFromComputer();
	stopLiveDataStreamToComputer();
	
	digitalWrite(ledPin2, LOW); //visual indication of deinitialization
}

/*
	Processes incoming serial data based on the header instruction.
*/
void processInstructionFromComputer(byte instruction){
	if (instruction == MocoProtocolStartSendingAxisDataInstruction){
		startLiveDataStreamToComputer();
	}
	else if (instruction == MocoProtocolStopSendingAxisDataInstruction){
		stopLiveDataStreamToComputer();

	}
	else if (instruction == MocoProtocolStartPlaybackInstruction){
		if (isStreaming){
			stopLiveDataStreamToComputer();
		}
		doPlaybackFromComputer();
	}
	else if (instruction == MocoProtocolStopPlaybackInstruction){
		stopPlaybackFromComputer();
	}
	else if (instruction == MocoProtocolRequestAxisResolutionDataInstruction){
		writeAxisResolutionsToComputer();
	}
	else if (instruction ==MocoProtocolHostWillDisconnectNotificationInstruction){
		deinitSlaveMCU();
	}
	else if (instruction == MocoProtocolPlaybackLastFrameSentNotificationInstruction){
		finalFrame = axis_TargetBuffer_currentBufferPosition-1;
	}
	else if (instruction ==  MocoProtocolPlaybackFrameDataHeader){
		freshRequestSentForNextAxisPositionToComputer = false;
		while(Serial.available() < 5){}
		int axisID = Serial.read();
		addToAxisTargetBuffer(axisID, readLongFromSerial());
		writeDebugStringToComputer(String(millis() - start, DEC), true);
	}
	else{
		writeDebugStringToComputer("Unknown Message Received: " + String(instruction, DEC), true);
	}
	
}

void startLiveDataStreamToComputer()
{
	isStreaming = true;
	MocoTimer1::set(1.0/50.0, writeAxisPositionsToComputer);
	MocoTimer1::start();
}

void stopLiveDataStreamToComputer()
{
	isStreaming = false;
	MocoTimer1::stop();
}

void doPlaybackFromComputer()
{
	//reset all data
	isPlayback = true;
	frameCounter = 0;
	finalFrame = -1;
	axis_TargetBuffer_AmountFreshData = 0;
	axis_TargetBuffer_currentPosition = 0;
	axis_TargetBuffer_currentBufferPosition = 0;
	//----------------------
	
	//TODO: wait to go to first frame????
	
	//begin to fill the buffer
	writeDebugStringToComputer("Filling Buffer", true);
	
	//fill buffer until finalFrame is found or we run out of buffer space
	freshRequestSentForNextAxisPositionToComputer = false;
	
	while(axis_TargetBuffer_AmountFreshData<bufferSize && isPlayback && finalFrame == -1){
		if(!freshRequestSentForNextAxisPositionToComputer){
			writeRequestForNextAxisPositionToComputer(MocoAxisCameraTilt);
			freshRequestSentForNextAxisPositionToComputer = true;
			start = millis();
		}
		
		doSerialDuties();
		doGeneralDuties();
	}
	if(!isPlayback){
		writeDebugStringToComputer("Playback quit before buffer could fill.", true);
		return;
	}
	writeDebugStringToComputer("Buffer Filled", true);
	
	//needs to seek to first position
	MocoTimer1::set(1.0/50.0, updateAxisPositionsFromPlayback);
	MocoTimer1::start();
	writePlaybackHasStartedToComputer();
	while(isPlayback){
		if (axis_TargetBuffer_AmountFreshData < bufferSize-1 && axis_TargetBuffer_currentBufferPosition%bufferSize != axis_TargetBuffer_currentPosition%bufferSize){
			writeDebugStringToComputer("buffer " + String(axis_TargetBuffer_currentBufferPosition, DEC) + ", current " + String(axis_TargetBuffer_currentPosition, DEC), false);
			if(!freshRequestSentForNextAxisPositionToComputer){
				writeRequestForNextAxisPositionToComputer(MocoAxisCameraTilt);
				freshRequestSentForNextAxisPositionToComputer = true;
				start = millis();
			}
		}
		if(axis_TargetBuffer_AmountFreshData < bufferLowSizeWarning){
			digitalWrite(ledPin, HIGH);
		}
		doSerialDuties();
		doGeneralDuties();
		
	}
	
}

void stopPlaybackFromComputer()
{
	isPlayback = false;
	MocoTimer1::stop();
	digitalWrite(ledPin, LOW);
}


//eventually do (MocoJoAxis axis, long target)
void addToAxisTargetBuffer(int axisID, long target)
{
	axis_TargetBuffer[axis_TargetBuffer_currentBufferPosition%bufferSize] = target;
	axis_TargetBuffer_AmountFreshData++;
	axis_TargetBuffer_currentBufferPosition++;
}


void updateAxisPositionsFromPlayback()
{
	//EVENTUALLY WHEN WE GET ALL THE MCUs
	//1. pulse logic high to all axis servos to notify servos to change to newly received position
	// ALSO maybe get current position data from all these servos 
	//2. request next playback frame -> receive position data
	//3. distribute position data to servos
	if(frameCounter == finalFrame+1 && finalFrame != -1){
		stopPlaybackFromComputer();
		writePlaybackHasCompletedToComputer();
		return;
	}
	
	if(axis_TargetBuffer_currentPosition > axis_TargetBuffer_currentBufferPosition){
		//digitalWrite(ledPin, HIGH);
		//writeDebugStringToComputer("Oh Fuck", true);
	}
	
	//writeDebugStringToComputer(String(frameCounter, DEC), true);
	
	axis_Target = axis_TargetBuffer[axis_TargetBuffer_currentPosition%bufferSize]; //effectively a virtual sync
	axis_TargetBuffer_currentPosition++;
	axis_TargetBuffer_AmountFreshData--;
	
	frameCounter++;

}


void writeRequestForNextAxisPositionToComputer(int axisID)
{
	Serial.write(MocoProtocolAdvancePlaybackRequestType); 
	Serial.write(axisID);
	writeLongToSerial(axis_Position);
}

void writePlaybackHasStartedToComputer()
{
	Serial.write(MocoProtocolPlaybackStartingNotificationResponseType);
	Serial.write(1);
	Serial.write(1);
	Serial.write(1);
	Serial.write(1);
	Serial.write(1);
}

void writePlaybackHasCompletedToComputer()
{
	Serial.write(MocoProtocolPlaybackCompleteNotificationResponseType);
	Serial.write(1);
	Serial.write(1);
	Serial.write(1);
	Serial.write(1);
	Serial.write(1);
}

void writeAxisPositionsToComputer()
{
	//TEMP - eventually will interate through all online axes
	Serial.write(MocoProtocolAxisPositionResponseType);//we are sending axis position data
	Serial.write(MocoAxisCameraTilt);//we are saying this is the tilt
	writeLongToSerial(axis_Position);

}

void writeHandshakeSuccessToComputer()
{
	Serial.write(MocoProtocolHandshakeResponseType);
	Serial.write(1);
	writeLongToSerial((long)MocoProtocolHandshakeSuccessfulResponse);
}

void writeDebugStringToComputer(String str, boolean force)
{
	if(!messageDebugHighPriority){
		force = false;
	}
	if(!messageDebug && !force){
		return;
	}
	Serial.write(MocoProtocolNewlineDelimitedDebugStringResponseType);
	Serial.println(str);
}

void writeAxisResolutionsToComputer()
{
	//TEMP:
	Serial.write(MocoProtocolAxisResolutionResponseType);
	Serial.write(MocoAxisCameraTilt);//we are saying this is the tilt
	writeLongToSerial(axis_Resolution);
	//-----
}

void writeLongToSerial(long number)
{
	Serial.write((uint8_t)((number >> 24) & 0xFF));
	Serial.write((uint8_t)((number >> 16) & 0xFF));
	Serial.write((uint8_t)((number >> 8) & 0XFF));
	Serial.write((uint8_t)((number & 0XFF)));
}

long readLongFromSerial()
{
	byte byte1 = Serial.read();
	byte byte2 = Serial.read();
	byte byte3 = Serial.read();
	byte byte4 = Serial.read();
	
	return ((byte1 << 24) + (byte2 << 16) + (byte3 << 8) + (byte4));
}


void updateControllerTiltEncoder(){
  // CSn needs to cycle from high to low to initiate transfer. Then clock cycles. As it goes high
// again, data will appear on sda
  
  digitalWrite(controllerTiltEncoder_CSnPin, HIGH); // CSn high
  digitalWrite(controllerTiltEncoder_clockPin, HIGH); // CLK high
  delayMicroseconds(shortdelay);
  //digitalWrite(ledPin, HIGH); // signal start of transfer with LED
  digitalWrite(controllerTiltEncoder_CSnPin, LOW); // CSn low: start of transfer
  delayMicroseconds(shortdelay); // delay for chip initialization
  digitalWrite(controllerTiltEncoder_clockPin, LOW); // CLK goes low: start clocking
  delayMicroseconds(shortdelay*2); // hold low
 
  for (int x=0; x <18; x++) // clock signal, 18 transitions, output to clock pin
  {
    digitalWrite(controllerTiltEncoder_clockPin, HIGH); //clock goes high
    delayMicroseconds(shortdelay);
    inputstream_2 =digitalRead(controllerTiltEncoder_inputPin); // read one bit of data from pin
    packeddata_2 = ((packeddata_2 << 1) + inputstream_2);// left-shift summing variable, add pin value
    digitalWrite(controllerTiltEncoder_clockPin, LOW);
    delayMicroseconds(shortdelay); // end of one clock cycle
  }

  //digitalWrite(ledPin, LOW); // signal end of transmission
  
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



