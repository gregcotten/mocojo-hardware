#include <MocoTimer1.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoCommunication.h>
#include <MocoJoServoRepresentation.h>
#include <MocoJoServoProtocol.h>
#include <Logger.h>
#include <AS5045.h>

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
const int MCU_VirtualShutter_SyncOut_Pin = 2; //HIGH is shutter off cycle, LOW is shutter on cycle
//----------------------------------------------

//--------------Wheels--------------------------
AS5045 tiltEncoder(8,9,10, .5, false);
//----------------------------------------------

//---------------AXIS DATA--------------------
MocoJoServoRepresentation servoJibLift(Serial1, MocoAxisJibLift);

unsigned long timeAtLastServoManualUpdate = 0;




void setup()
{
	Serial.begin(MocoProtocolBaudRate);
	Serial1.begin(MocoJoServoBaudRate);
	Logger::setDebugMode(false, true);
	
	pinMode(ledPin1, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin1, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	
	//config virtual shutter
	pinMode(MCU_VirtualShutter_SyncOut_Pin, OUTPUT);
	digitalWrite(MCU_VirtualShutter_SyncOut_Pin, LOW);

	
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
	tiltEncoder.update();
	
	//send position data to servos
	if (!isPlayback){
		if(millis() - timeAtLastServoManualUpdate > 20){
			servoJibLift.setTargetPosition(tiltEncoder.getAbsolutePosition());
			timeAtLastServoManualUpdate = millis();
		}
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
	tiltEncoder.setAbsolutePosition(servoJibLift.getCurrentPosition());
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
	while(isPlayback){
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

void fillBuffer(){
	servoJibLift.addTargetPositionToBuffer(getAxisPositionFromNextFrame(servoJibLift.getServoID()));
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
	unsigned long then = millis();
	//fill buffer until finalFrame is found or we run out of buffer space
	while(isPlayback && frameBufferCounter < finalFrame && amountFreshBufferCounter < MocoJoServoBufferSize){
		fillBuffer();
		//Logger::writeDebugString(String(frameBufferCounter), true);
	}
	if(!isPlayback){
		return;
	}

	unsigned long duration = millis() - then;
	
	Logger::writeDebugString("Initial Buffer Filled in "+String(duration, DEC)+"ms, notifying servos to hone.", true);

	servoJibLift.proceedToHone();
	
	unsigned long lastHoneCheck = millis();
	boolean allDidHone = false;

	while(isPlayback && !allDidHone){
		if(lastHoneCheck - millis() >= 500){
			allDidHone = !servoJibLift.isHoning(); // && servo2 && ...
			
			lastHoneCheck= millis();
		}
		doGeneralDuties();
		doSerialDuties();
	}
	
	//*********** END FIRST BUFFER ***********

	//      TODO: Wait for hone to finish

	//*********** PLAYBACK LOOP BEGIN ***********
	if(!isPlayback){
		return;
	}
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
	if(isPlayback){
		stopPlaybackFromComputer();
		//notify computer that we finished normally!
		MocoJoCommunication::writePlaybackHasCompletedToComputer();
	}
	else{
		stopPlaybackFromComputer();
	}
	
	Logger::writeDebugString("Playback Completed at Frame "+ String(frameCounter, DEC), true);
}


void stopPlaybackFromComputer()
{
	isPlayback = false;
	MocoTimer1::stop();
	digitalWrite(ledPin1, LOW);

	servoJibLift.stopPlayback();
	tiltEncoder.setAbsolutePosition(servoJibLift.getCurrentPosition());
}

void playbackShutterDidFire()
{
	shutterFire();
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
	Do the shutter!
*/

void shutterFire(){
	digitalWrite(MCU_VirtualShutter_SyncOut_Pin, LOW);
	delay(1);
	digitalWrite(MCU_VirtualShutter_SyncOut_Pin, HIGH);
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