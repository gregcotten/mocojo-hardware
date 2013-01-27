#include <Servo.h>
#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoServoProtocol.h>
#include <Logger.h>

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

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncIn_Pin = 10; //HIGH is shutter off cycle, LOW is shutter on cycle
//----------------------------------------------

//--------------Servo Data-----------------------
long servoCurrentPosition = 0;
long servoResolution = 8*4095;
long servoTargetPosition = 0;


void setup(){
	Serial.begin(MocoJoServoBaudRate);
	Serial.flush();
	
	Logger::setDebugMode(false, true);
	
	pinMode(ledPin, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);
	pinMode(MCU_VirtualShutter_SyncIn_Pin, INPUT);
}

void loop(){

}