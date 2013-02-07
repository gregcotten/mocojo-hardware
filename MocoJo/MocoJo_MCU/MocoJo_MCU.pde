#include <MocoTimer1.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoCommunication.h>
#include <MocoJoServoRepresentation.h>
#include <MocoJoServoProtocol.h>
#include <Logger.h>

#define FRAMERATE 50

//DECLARATIONS

//---------------GENERAL------------------
const int ledPin1 = 13; //LED connected to digital pin 13
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
boolean isInitialized = false;
boolean isStreaming = false;
boolean isPlayback = false;

//PLAYBACK
long finalFrame;
long frameCounter;
long frameBufferCounter;
long amountFreshBufferCounter;
//----------------------------------------------

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncOut_Pin = 10; //HIGH is shutter off cycle, LOW is shutter on cycle
//----------------------------------------------

//---------------AXIS DATA--------------------
MocoJoServoRepresentation servoJibLift(Serial1, MocoAxisJibLift);




void setup()
{
	Serial.begin(MocoProtocolBaudRate);
	Serial1.begin(MocoJoServoBaudRate);
	Logger::setDebugMode(false, true);
	
	pinMode(ledPin1, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin1, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	pinMode(MCU_VirtualShutter_SyncOut_Pin, OUTPUT);
	setVirtualShutter(LOW);
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
		if(Serial.available()){
			processInstructionFromComputer(Serial.read());
		}

}



/*
	Used to initialize any variables associated with the MCU - called when handshake succeeds.
*/
void initialize()
{
	isInitialized = true;
	digitalWrite(ledPin2, HIGH); //visual indication of initialization
	
	servoJibLift.handshake();
}

/*
	Used to deinitialize anything on the MCU - turns it back to a "blank slate".
*/
void deinitialize()
{
	isInitialized = false;
	
	//stop everything!!!
	stopPlaybackFromComputer();
	stopLiveDataStreamToComputer();
	
	digitalWrite(ledPin2, LOW); //visual indication of deinitialization
}

void startLiveDataStreamToComputer()
{
	isStreaming = true;
	MocoTimer1::set(1.0/50.0, writeServoPositionsToComputer);
	MocoTimer1::start();
}

void stopLiveDataStreamToComputer()
{
	isStreaming = false;
	MocoTimer1::stop();
}

long getAxisPositionFromNextFrame(int axisID){
	MocoJoCommunication::writeRequestForNextFrameToComputer(axisID);
	byte instruction;
	while(true){
		if (Serial.available()){
			instruction = Serial.read();
			if (instruction != MocoProtocolPlaybackFrameDataHeader){
				processInstructionFromComputer(instruction);
			}
			else{
				SerialTools::blockUntilBytesArrive(Serial, 5);
				SerialTools::readDummyBytesFromSerial(Serial, 1);
				return SerialTools::readLongFromSerial(Serial);
			}
		}
	}
}

void addTargetPositionToBuffer(int servoID, long position){
	switch(servoID){
		case MocoAxisJibLift:
			servoJibLift.addTargetPositionToBuffer(position);
			break;
	}
}

void fillBuffer(){
	if(!servoJibLift.targetPositionBufferIsFull()){
			servoJibLift.addTargetPositionToBuffer(getAxisPositionFromNextFrame(servoJibLift.getServoID()));	
	}
	else{
		Logger::writeDebugString("you filled when it was full bitch!", true);
	}
	frameBufferCounter++;
	amountFreshBufferCounter++;
}

void startPlaybackFromComputer()
{
	//*********** RESET DATA ***********
	isPlayback = true;
	frameCounter = 0;
	frameBufferCounter = 0;
	amountFreshBufferCounter = 0;
	//**********
	
	servoJibLift.startPlayback();
	
	//*********** BEGIN FIRST BUFFER ***********
	
	Logger::writeDebugString("Filling Initial Buffer", true);
	
	//fill buffer until finalFrame is found or we run out of buffer space
	while(isPlayback && frameBufferCounter < finalFrame && amountFreshBufferCounter < MocoJoServoBufferSize){
		fillBuffer();
		Logger::writeDebugString(String(frameBufferCounter), true);
	}
	
	Logger::writeDebugString("Initial Buffer Filled, notifying servos to hone.", true);

	servoJibLift.proceedToHone();
	
	//*********** END FIRST BUFFER ***********

	//      TODO: Wait for hone to finish

	//*********** PLAYBACK LOOP BEGIN ***********
	MocoTimer1::set(1.0/50.0, playbackShutterDidFire);
	MocoTimer1::start();
	MocoJoCommunication::writePlaybackHasStartedToComputer();
	
	runPlayback();
	//*********** PLAYBACK LOOP END ***********
	
}

void runPlayback(){
	while(isPlayback && frameCounter < finalFrame){
		doGeneralDuties();
		doSerialDuties();
		if(frameBufferCounter < finalFrame && amountFreshBufferCounter < MocoJoServoBufferSize){
			fillBuffer();	
		}
	}
	stopPlaybackFromComputer();
	MocoJoCommunication::writePlaybackHasCompletedToComputer();
	Logger::writeDebugString("Playback Completed at Frame "+ String(frameCounter, DEC), true);
}


void stopPlaybackFromComputer()
{
	isPlayback = false;
	MocoTimer1::stop();
	digitalWrite(ledPin1, LOW);

	servoJibLift.stopPlayback();
}

void playbackShutterDidFire()
{
	shutterFire();
	servoJibLift.playbackShutterDidFire();
	frameCounter++;
	amountFreshBufferCounter--;
}

/*
	Eventually port to MocoJoCommunication and have writeAxisPositionsToComputer(MocoJoAxis[] axes)
*/
void writeServoPositionsToComputer()
{
	shutterFire();

	//TEMP - eventually will interate through all online axes
	Serial.write(MocoProtocolAxisPositionResponseType);//we are sending axis position data
	Serial.write(servoJibLift.getServoID());//we are saying this is the tilt
	SerialTools::writeLongToSerial(Serial, servoJibLift.getPositionAtLastSync());

}


/*
	Eventually port to MocoJoCommunication and have writeAxisResolutionsToComputer(MocoJoAxis[] axes)
*/
void writeServoResolutionsToComputer()
{
	//TEMP:
	Serial.write(MocoProtocolAxisResolutionResponseType);
	Serial.write(MocoAxisJibLift);//we are saying this is the tilt
	SerialTools::writeLongToSerial(Serial, (long)4096*(long)4);
	//-----
}


/*
	Sets the shutter SYNC OUT logic level value.
*/
void setVirtualShutter(int value){
	digitalWrite(MCU_VirtualShutter_SyncOut_Pin, value);
}

/*
	Do the shutter!
*/

void shutterFire(){
	setVirtualShutter(LOW);
	delay(1);
	setVirtualShutter(HIGH);
}



/*
	Processes incoming serial data based on the header instruction.
*/
void processInstructionFromComputer(byte instruction){

	if(instruction == MocoProtocolRequestHandshakeInstruction)
	{
		initialize();
		MocoJoCommunication::writeHandshakeSuccessToComputer();
		return;
	}

	if (!isInitialized){
		return;
	}
	
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
			SerialTools::blockUntilBytesArrive(Serial, 4);
			finalFrame = SerialTools::readLongFromSerial(Serial);
			Logger::writeDebugString("Final Frame: "+ String(finalFrame), true);
			startPlaybackFromComputer();
			break;
		
		case MocoProtocolStopPlaybackInstruction:
			stopPlaybackFromComputer();
			break;
	
		case MocoProtocolRequestAxisResolutionDataInstruction:
			writeServoResolutionsToComputer();
			break;
	
		case MocoProtocolHostWillDisconnectNotificationInstruction:
			deinitialize();
			break;
	
		case MocoProtocolPlaybackLastFrameSentNotificationInstruction:
			//set our last frame limit thing
			break;
		
		case MocoProtocolPlaybackFrameDataHeader:
			SerialTools::blockUntilBytesArrive(Serial, 5);
			addTargetPositionToBuffer(Serial.read(), SerialTools::readLongFromSerial(Serial));
			break;

		case MocoProtocolSeekPositionDataHeader:
			SerialTools::blockUntilBytesArrive(Serial, 5);
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