#include <Servo.h>
#include <MocoTimer1.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoCommunication.h>
#include <Logger.h>

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
const int encoderDebug = 0; //look for encoder errors
const int timingDebug = 0; //for debugging control loop frequency

long start = 0; //for timing debug
long loopCount = 0; //for timing debug
//----------------------------------------


//---------------Moco LOGIC--------------------
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

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncOut_Pin = 10; //HIGH is shutter off cycle, LOW is shutter on cycle
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

void setup()
{
	Serial.begin(MocoProtocolBaudRate);
	
	Logger::setDebugMode(false, true);
	
	pinMode(ledPin, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	pinMode(MCU_VirtualShutter_SyncOut_Pin, OUTPUT);
	setVirtualShutter(HIGH);
}

void loop()
{	
	if (firstBoot){
		//do first boot things
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
	
	
	//send position data to MCUs
	if (!isPlayback){
		
	}
}

/*
	All serial communication is received here.
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
				MocoJoCommunication::writeHandshakeSuccessToComputer();
			}

		}
	}
}

/*
	Sets the shutter SYNC OUT logic level value.
*/
void setVirtualShutter(int value){
	digitalWrite(MCU_VirtualShutter_SyncOut_Pin, value);
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
	
	switch (instruction){
		case MocoProtocolStartSendingAxisDataInstruction:
			startLiveDataStreamToComputer();
			break;
		
		case MocoProtocolStopSendingAxisDataInstruction:
			stopLiveDataStreamToComputer();
			break;
		
		case MocoProtocolStartPlaybackInstruction:
			if (isStreaming){
				stopLiveDataStreamToComputer();
			}
			doPlaybackFromComputer();
			break;
		
		case MocoProtocolStopPlaybackInstruction:
			stopPlaybackFromComputer();
			break;
	
		case MocoProtocolRequestAxisResolutionDataInstruction:
			writeAxisResolutionsToComputer();
			break;
	
		case MocoProtocolHostWillDisconnectNotificationInstruction:
			deinitSlaveMCU();
			break;
	
		case MocoProtocolPlaybackLastFrameSentNotificationInstruction:
			finalFrame = axis_TargetBuffer_currentBufferPosition-1;
			break;
		
		case MocoProtocolPlaybackFrameDataHeader:
			freshRequestSentForNextAxisPositionToComputer = false;
			while(Serial.available() < 5){}
			//int axisID = Serial.read();
			addToAxisTargetBuffer(Serial.read(), SerialTools::readLongFromSerial(Serial));
			//Logger::writeDebugString(String(millis() - start, DEC), true);
			break;

		case MocoProtocolSeekPositionDataHeader:
			while(Serial.available() < 5){}
			if (isPlayback){
				SerialTools::readDummyBytesFromSerial(Serial, 5);
				return;
			}
			Serial.read();//temp bogus axis
			SerialTools::readLongFromSerial(Serial); //this is the position
			break;

		default:
			Logger::writeDebugString("Unknown Message Received: " + String(instruction, DEC), true);
			break;
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
	//*********** RESET DATA ***********
	isPlayback = true;
	frameCounter = 0;
	finalFrame = -1;
	axis_TargetBuffer_AmountFreshData = 0;
	axis_TargetBuffer_currentPosition = 0;
	axis_TargetBuffer_currentBufferPosition = 0;
	//**********
	
	
	//*********** BEGIN FIRST BUFFER ***********
	
	Logger::writeDebugString("Filling Buffer", true);
	
	//fill buffer until finalFrame is found or we run out of buffer space
	freshRequestSentForNextAxisPositionToComputer = false;
	
	while(axis_TargetBuffer_AmountFreshData<bufferSize && isPlayback && finalFrame == -1){
		if(!freshRequestSentForNextAxisPositionToComputer){
			MocoJoCommunication::writeRequestForNextFrameToComputer();
			freshRequestSentForNextAxisPositionToComputer = true;
			start = millis();
		}
		
		doSerialDuties();
		doGeneralDuties();
	}
	if(!isPlayback){
		Logger::writeDebugString("Playback quit before buffer could fill.", true);
		return;
	}
	
	Logger::writeDebugString("Buffer Filled", true);
	
	//*********** END FIRST BUFFER ***********



	//      TODO: HONE SLOWLY TO FIRST POSITIONS!!!!




	//*********** PLAYBACK LOOP BEGIN ***********
	MocoTimer1::set(1.0/50.0, updateAxisPositionsFromPlayback);
	MocoTimer1::start();
	MocoJoCommunication::writePlaybackHasStartedToComputer();
	
	while(isPlayback){
		if (axis_TargetBuffer_AmountFreshData < bufferSize-1 && axis_TargetBuffer_currentBufferPosition%bufferSize != axis_TargetBuffer_currentPosition%bufferSize){
			//Logger::writeDebugString("buffer " + String(axis_TargetBuffer_currentBufferPosition, DEC) + ", current " + String(axis_TargetBuffer_currentPosition, DEC), false);
			if(!freshRequestSentForNextAxisPositionToComputer){
				MocoJoCommunication::writeRequestForNextFrameToComputer();
				freshRequestSentForNextAxisPositionToComputer = true;
				//start = millis();
			}
		}
		if(axis_TargetBuffer_AmountFreshData < bufferLowSizeWarning){
			digitalWrite(ledPin, HIGH);
		}
		doSerialDuties();
		doGeneralDuties();
		
	}
	//*********** PLAYBACK LOOP END ***********
	
}

void stopPlaybackFromComputer()
{
	isPlayback = false;
	MocoTimer1::stop();
	digitalWrite(ledPin, LOW);
}



void addToAxisTargetBuffer(int axisID, long target)
{
	//eventually seperate out for each axis... duh
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
	
	/*
		If we have reached the final frame, stop!
	*/
	if(frameCounter == finalFrame+1 && finalFrame != -1){
		stopPlaybackFromComputer();
		MocoJoCommunication::writePlaybackHasCompletedToComputer();
		Logger::writeDebugString("Playback Completed at Frame "+ String(frameCounter-1, DEC), true);
		return;
	}
	
	if(axis_TargetBuffer_currentPosition > axis_TargetBuffer_currentBufferPosition){
		//THIS SHOULD NEVER HAPPEN
	}
	
	//Logger::writeDebugString(String(frameCounter, DEC), true);
	
	//axis_Target = axis_TargetBuffer[axis_TargetBuffer_currentPosition%bufferSize]; //effectively a virtual sync
	axis_TargetBuffer_currentPosition++;
	axis_TargetBuffer_AmountFreshData--;
	
	frameCounter++;

}


/*
	Eventually port to MocoJoCommunication and have writeAxisPositionsToComputer(MocoJoAxis[] axes)
*/
void writeAxisPositionsToComputer()
{
	//TEMP - eventually will interate through all online axes
	Serial.write(MocoProtocolAxisPositionResponseType);//we are sending axis position data
	Serial.write(MocoAxisCameraTilt);//we are saying this is the tilt
	SerialTools::writeLongToSerial(Serial, 0);

}


/*
	Eventually port to MocoJoCommunication and have writeAxisResolutionsToComputer(MocoJoAxis[] axes)
*/
void writeAxisResolutionsToComputer()
{
	//TEMP:
	Serial.write(MocoProtocolAxisResolutionResponseType);
	Serial.write(MocoAxisCameraTilt);//we are saying this is the tilt
	SerialTools::writeLongToSerial(Serial, 100);
	//-----
}



