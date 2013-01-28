#include <Servo.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoServoProtocol.h>
#include <MocoJoServoCommunication.h>
#include <PID_v1.h>
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

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncIn_Pin = 10; //HIGH is shutter off cycle, LOW is shutter on cycle
//----------------------------------------------

//--------------Servo Stuff-----------------------
const int servoID = MocoAxisJibLift;
long servoCurrentPosition = 0;
long servoPositionAtLastSync = 0;
long servoResolution = 8*4095;

long servoTargetPosition = 0;
long servoTargetPositionForNextSync = 0; //for playback purposes


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
	Serial.flush();
	
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

	doPIDDuties();
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
	servoEncoder.update();
	servoCurrentPosition = servoEncoder.getAbsolutePosition();
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
	if (isPlayback){
		
	}
}

void processInstructionFromMCU(){
	byte ID = Serial.read();

	if(ID != servoID){
		SerialTools::readDummyBytesFromSerial(Serial1, 5);
		return;
	}

	switch(Serial1.read()){
		
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
			isPlayback = true;
			break;
		case MocoJoServoStopPlayback;
			isPlayback = false;
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
		case MocoJoServoSetTargetPositionForNextSync:
			if(isPlayback){
				servoNextTargetPosition = SerialTools::readLongFromSerial(Serial1);	
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
			SerialTools::readDummyBytesFromSerial(Serial1, 4);
			break;

	}
}

void exitSafeStart(){
	motorController.exitSafeStart();
	isStopped = false;
}

void stopEverything(){
	motorController.stopMotor();
	isStopped = true;
}













