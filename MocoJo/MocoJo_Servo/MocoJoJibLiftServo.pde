#include <Servo.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoServoProtocol.h>
#include <PID_v1.h>
#include <SMC.h>
#include <AS5045.h>

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
boolean isInitialized = false;
boolean isStopped = true;
boolean isPlayback = false;

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncIn_Pin = 10; //HIGH is shutter off cycle, LOW is shutter on cycle
//----------------------------------------------

//--------------Servo Stuff-----------------------
const int servoID = MocoAxisJibLift;
long servoCurrentPosition = 0;
long servoResolution = 8*4095;
long servoTargetPosition = 0;


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
	if(Serial1.available()){
		processInstructionFromMCU(Serial1.read());
	}
}

void processInstructionFromMCU(byte ID){
	SerialTools::blockUntilBytesArrive(Serial1, 5);
	if(ID != servoID){
		SerialTools::readDummyBytes(Serial1, 5);
		return;
	}

	switch(Serial1.read()){
		case MocoJoServoHandshakeRequest:

			break;
		case MocoJoServoStopEverything:

			break;
		case MocoJoServoExitSafeStart:

			break;

		default:
			SerialTools::readDummyBytes(Serial1, 4);
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













