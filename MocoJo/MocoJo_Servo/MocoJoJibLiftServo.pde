#include <Servo.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoServoProtocol.h>
#include <MocoJoServoCommunication.h>
#include <PID_v1.h>
#include <LongBuffer.h>
#include <SMC.h>
#include <AS5045.h>

//---------------GENERAL------------------
const int ledPin = 13; //LED connected to digital pin 13
const int ledPin2 = 43; //LED connected to digital pin 13
//----------------------------------------

//---------------Moco LOGIC--------------------
boolean firstBoot = true;
boolean isInitialized = false;
boolean isStopped = true;
boolean isPlayback = false;

long frameCounter = 0; //local use

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncIn_Pin = 10; //HIGH is shutter off cycle, LOW is shutter on cycle
//----------------------------------------------

//--------------Servo Stuff-----------------------
const int servoID = MocoAxisJibLift;
long servoCurrentPosition = 0;
long servoPositionAtLastSync = 0;
long servoResolution = 8*4095;

long servoTargetPosition = 0;
LongBuffer servoTargetPositionBuffer(100);


int servoTargetSpeed = 0;
const int servoMaxSpeed = 3200;

PID servoPositionPID(&servoCurrentPosition, &servoTargetSpeed, &servoTargetPosition,1,0,0, DIRECT);
int servoPositionPIDSampleTimeMillis = 1;
//-----------------------------------------------


//--------------Peripherals-----------------------
AS5045 servoEncoder(4,5,6, 0, 1.0, false);
SMC motorController(&Serial2);
//-----------------------------------------------


void setup(){
	Serial.begin(MocoJoServoBaudRate);
	
	pinMode(ledPin, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	
	pinMode(MCU_VirtualShutter_SyncIn_Pin, INPUT);

	servoPositionPID.setOutputLimits(-servoMaxSpeed, servoMaxSpeed);
	servoPositionPID.setSampleTime(servoPositionPIDSampleTimeMillis);
}

void loop(){
	if (firstBoot){
		//do first boot things
		firstBoot = false;
	}

	if (isInitialized){
		doPIDDuties();	
	}
	
	doSerialDuties();
//	doGeneralDuties();
}

void initialize(){
	motorController.initialize();
	isInitialized = true;
	attachInterrupt(MCU_VirtualShutter_SyncIn_Pin, syncInterrupt, RISING);
	Serial.println("initialized!");
}

/*
	General duties are anything that needs to happen with some sort of immediacy (updating positions, controllers, etc.).
*/
void doGeneralDuties(){
	if (!isPlayback){
		
	}
}

void doPIDDuties(){
	//not hooked up to real motor and encoder so don't do this yet - we'll just emulate it!
	//servoEncoder.update();
	//servoCurrentPosition = servoEncoder.getAbsolutePosition();

	//until we're hooked up for real let's pretend the PID is doing a GREAT job.
	servoCurrentPosition = servoTargetPosition;
	
	if (!isStopped && servoPositionPID.compute()){
		motorController.setMotorSpeed(servoTargetSpeed);
	}
	
}

/*
	All serial communication is interpreted here.
*/
void doSerialDuties()
{
	if(Serial1.available() >= 6){
		processInstructionFromMCU();
	}
}

void syncInterrupt(){
	servoPositionAtLastSync = servoCurrentPosition;
	if (isPlayback){
		if(servoTargetPositionBuffer.amountFresh() > 0){
			servoTargetPosition = servoTargetPositionBuffer.nextLong();
			return;
		}
		else{
			stopPlayback();
			Serial.println("Target Buffer overrun or playback stopped @ frame " + String(frameCounter));
		}
	}
}

void exitSafeStart(){
	motorController.exitSafeStart();
	isStopped = false;
}

void stopEverything(){
	isPlayback = false;
	motorController.stopMotor();
	isStopped = true;
	resetPlaybackParameters();
}

void honeToPosition(long honePosition){
	servoTargetPosition = honePosition;

	// **servoPositionPID - make sure to change parameters for slow mode!

	while (servoCurrentPosition != servoTargetPosition){
		doPIDDuties();
		doSerialDuties();
	}
	delay(1000); //delay for a second just to be sure we've stopped.

	// **servoPositionPID - make sure to change parameters for normal mode!
}


void startPlayback(long honePosition){
	honeToPosition(honePosition);
	MocoJoServoCommunication::writeMocoJoServoDidHoneToFirstPosition(Serial1, servoID);

	Serial.println("Waiting for buffer to fill...");
	while(!servoTargetPositionBuffer.isFull()){}
	Serial.println("Buffer filled. Commencing playback by your command!");

	isPlayback = true;
}

void stopPlayback(){
	
	isPlayback = false;
	resetPlaybackParameters();
	servoTargetPositionBuffer.reset();
	frameCounter = 0;
}

void processInstructionFromMCU(){
	byte ID = Serial.read();

	if(ID != servoID){
		SerialTools::readDummyBytesFromSerial(Serial1, 5);
		return;
	}
	
	byte instruction = Serial1.read();
	
	switch(instruction){
		
		//Initialization:
		case MocoJoServoHandshakeRequest:
			MocoJoServoCommunication::writeHandshakeSuccessToMCU(Serial1, servoID);
			break;
		case MocoJoServoInitializeRequest:
			initialize();
			break;

		//Safety Precautions:
		case MocoJoServoStopEverything:
			stopEverything();
			SerialTools::readDummyBytesFromSerial(Serial1, 4);
			break;
		case MocoJoServoExitSafeStart:
			exitSafeStart();
			SerialTools::readDummyBytesFromSerial(Serial1, 4);
			break;

		//Playback:
		case MocoJoServoStartPlayback:
			startPlayback(SerialTools::readLongFromSerial(Serial1));
			break;
		case MocoJoServoStopPlayback;
			SerialTools::readDummyBytesFromSerial(Serial1, 4);
			break;

		//GETTERS:
		case MocoJoServoGetCurrentPosition:
			MocoJoServoCommunication::writeCurrentPositionToMCU(Serial1, servoID, servoCurrentPosition);
			break;
		case MocoJoServoGetPositionAtLastSync:
			MocoJoServoCommunication::writePositionAtLastSyncToMCU(Serial1, servoID, servoPositionAtLastSync);
			break;

		//SETTERS:
		case MocoJoServoSetTargetPosition:
			if(!isPlayback){
				servoTargetPosition = SerialTools::readLongFromSerial(Serial1);	
			}
			else{
				SerialTools::readDummyBytesFromSerial(Serial1, 4);
			}
			break;
		case MocoJoServoAddTargetPositionToBuffer:
			if(isPlayback){
				servoTargetPositionBuffer.addLong(SerialTools::readLongFromSerial(Serial1));	
			}
			else{
				SerialTools::readDummyBytesFromSerial(Serial1, 4);
			}
			break;
		case MocoJoServoSetMaxSpeed:
			servoMaxSpeed = SerialTools::readLongFromSerial(Serial1);
			servoPositionPID.setOutputLimits(-servoMaxSpeed, servoMaxSpeed);
			break;

		default:
			Serial.println("Instruction Invalid: " + String(instruction, DEC));
			SerialTools::readDummyBytesFromSerial(Serial1, 4);
			break;

	}
}













