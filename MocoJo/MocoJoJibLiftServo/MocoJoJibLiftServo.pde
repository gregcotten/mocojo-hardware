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


long servoTargetSpeed = 0;
int servoMaxSpeed = 3200;

PID servoPositionPID(&servoCurrentPosition, &servoTargetSpeed, &servoTargetPosition,1,0,0, DIRECT);
int servoPositionPIDSampleTimeMillis = 1;
//-----------------------------------------------


//--------------Peripherals-----------------------
AS5045 servoEncoder(4,5,6, 0, 1.0, false);
SMC motorController(Serial); //change this to Serial2
//-----------------------------------------------


void setup(){
	Serial.begin(115200);
	Serial1.begin(MocoJoServoBaudRate);
	
	pinMode(ledPin, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	
	pinMode(MCU_VirtualShutter_SyncIn_Pin, INPUT);

	servoPositionPID.SetOutputLimits(-servoMaxSpeed, servoMaxSpeed);
	servoPositionPID.SetSampleTime(servoPositionPIDSampleTimeMillis);
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
	digitalWrite(ledPin2, HIGH); //visual indication of initialization
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
	if (!isStopped && servoPositionPID.Compute()){
		motorController.setMotorSpeed(servoTargetSpeed);
	}
	
}

/*
	All serial communication is interpreted here.
*/
void doSerialDuties()
{
	if(Serial1.available() >= 6){
		Serial.println("Packet received");
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
	stopPlayback();
	motorController.stopMotor();
	isStopped = true;
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
	
	servoTargetPositionBuffer.reset();
	frameCounter = 0;
}

void processInstructionFromMCU(){
	Serial.println("read ID");
	byte ID = Serial1.read();

	if(ID != servoID){
		Serial.println("wrong ID");
		SerialTools::readDummyBytesFromSerial(Serial1, 5);
		Serial.println(Serial1.available());
		return;
	}
	
	byte instruction = Serial1.read();
	long data = SerialTools::readLongFromSerial(Serial1);
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
			break;
	
		case MocoJoServoExitSafeStart:
			exitSafeStart();
			break;

		//Playback:
		case MocoJoServoStartPlayback:
			startPlayback(data);
			break;

		case MocoJoServoStopPlayback:
			stopPlayback();
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
				servoTargetPosition = data;	
			}
			break;

		case MocoJoServoAddTargetPositionToBuffer:
			if(isPlayback){
				servoTargetPositionBuffer.addLong(data);	
			}
			break;

		case MocoJoServoSetMaxSpeed:
			servoMaxSpeed = data;
			servoPositionPID.SetOutputLimits(-servoMaxSpeed, servoMaxSpeed);
			break;
		
		default:
			Serial.println("Instruction Invalid: " + String(instruction, DEC));
		
	}
}













