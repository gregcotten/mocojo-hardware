#include <MocoProtocolConstants.h>
#include <SerialTools.h>
#include <MocoJoServoProtocol.h>
#include <MocoJoServoCommunication.h>
#include <PID_v1.h>
#include <LongBuffer.h>
#include <SMC.h>
#include <SMCProtocolConstants.h>
#include <AS5045.h>
#include <ChangeNotification.h>
#include <MathHelper.h>

//---------------GENERAL------------------
const int ledPin1 = 13; //LED connected to digital pin 13
const int ledPin2 = 43; //LED connected to digital pin 43
//----------------------------------------

//---------------Moco LOGIC--------------------
boolean isInitialized = false;
boolean isStopped = true;
boolean isPlayback = false;
boolean isHoning = false;

long frameCounter = 0; //local use

//--------------Moco GPIO------------------------
const int MCU_VirtualShutter_SyncIn_Pin = 11; //HIGH is shutter off cycle, LOW is shutter on cycle
const cn MCU_VirtualShutter_SyncIn_CN_Pin = CN_10; //CN_10 is pin 11
//----------------------------------------------

//--------------Servo Stuff-----------------------
const int servoID = MocoAxisJibLift;

long servoCurrentPosition = 0;
long servoCurrentVelocity = 0;

volatile long servoPositionAtLastSync=0;
volatile long servoVelocityAtLastSync=0;

long servoResolution = 8*4095;

long servoTargetPosition = 0;
long servoTargetVelocity = 0;
long motorTargetSpeed = 0;

//buffer
LongBuffer servoTargetPositionBuffer(MocoJoServoBufferSize);

//commands
boolean proceedToHone = false;




//PID
PID servoPositionPID(&servoCurrentPosition, &motorTargetSpeed, &servoTargetPosition,1,1,1, DIRECT);

const int servoPIDSampleTimeMillis = 1;
//-----------------------------------------------


//--------------Peripherals-----------------------
AS5045 servoEncoder(4,5,6, 1.0, false);
SMC motorController(Serial, 2, 3); //change this to Serial2
//-----------------------------------------------


void setup(){
	Serial.begin(9600);
	Serial1.begin(MocoJoServoBaudRate);
	
	pinMode(ledPin1, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin1, LOW);
	pinMode(ledPin2, OUTPUT); // visual signal of I/O to chip
	digitalWrite(ledPin2, LOW);

	servoPositionPID.SetOutputLimits(-3200, 3200);
	servoPositionPID.SetSampleTime(servoPIDSampleTimeMillis);
	servoPositionPID.SetMode(AUTOMATIC);

	servoEncoder.setAbsolutePosition(0);

	motorController.setMinimumSpeed(.05);
	
}

void loop(){
	if (isInitialized){
		doPIDDuties();	
	}

	doSerialDuties();
}

void initialize(){
	
	isInitialized = true;
	attachInterrupt(MCU_VirtualShutter_SyncIn_CN_Pin, syncInterrupt, RISING);

	motorController.initialize();
	motorController.exitSafeStart();
	isStopped = false;
	digitalWrite(ledPin1, HIGH); //visual indication of initialization
}

void doPIDDuties(){
	
	 servoEncoder.update();
	 servoCurrentPosition = servoEncoder.getAbsolutePosition();
	 servoCurrentVelocity = servoEncoder.getVelocity();
	
	if(servoPositionPID.Compute() && !isStopped){
		motorController.setMotorSpeed(MathHelper::fromIntTo01(motorTargetSpeed, 3200));	
	}
	
}

/*
	All serial communication is interpreted here.
*/
void doSerialDuties()
{
	if(Serial1.available() >= 6){
		//Serial.println("Packet received");
		processInstructionFromMCU();
	}
}



void syncInterrupt(){
	servoPositionAtLastSync = servoCurrentPosition;
	servoVelocityAtLastSync = servoCurrentVelocity;
	
	if (isPlayback){
		servoTargetPosition = servoTargetPositionBuffer.nextLong();
		frameCounter++;
	}
}


void stopEverything(){
	stopPlayback();
	motorController.stopMotor();
	servoTargetPosition = servoCurrentPosition;
	isStopped = true;
}

void honeToPosition(long honePosition){
	isHoning = true;
	

	// **servoPositionPID - make sure to change parameters for slow mode!

	//also put servoVelocity != 0
	motorController.setMaximumSpeed(.1);
	servoTargetPosition = honePosition;
	while (isPlayback && servoCurrentPosition != servoTargetPosition){
		doPIDDuties();
		doSerialDuties();
	}
	isHoning = false;
	motorController.setMaximumSpeed(1.0);

	// **servoPositionPID - make sure to change parameters for normal mode!
}


void startPlayback(){

	//TODO: Stop moving you bastard!

	frameCounter = 0;
	proceedToHone = false;

	//Serial.println("Waiting for buffer to fill...");
	while(isPlayback && !servoTargetPositionBuffer.isFull() && !proceedToHone){
		doPIDDuties();
		doSerialDuties();
	}
	if(!isPlayback){
		return;
	}
	//Serial.println("Buffer filled.");
	
	//Serial.println("Honing to position: " + String(servoTargetPositionBuffer.peek(), DEC));
	honeToPosition(servoTargetPositionBuffer.peek());

	runPlayback();

}


void runPlayback(){
	while(servoTargetPositionBuffer.amountFresh() > 0 && isPlayback){
		doSerialDuties();
		doPIDDuties();
	}
	if(isPlayback){
	//	Serial.println("Target Buffer overrun or playback stopped @ frame " + String(frameCounter) +" and position " + String(servoTargetPosition, DEC));
		stopPlayback();	
		
	}
	
}

void stopPlayback(){
	
	isPlayback = false;
	isHoning = false;

	servoTargetPosition = servoCurrentPosition;
	
	servoTargetPositionBuffer.reset();
	//Serial.println("Playback Stopped!");
}



void processInstructionFromMCU(){
	//Serial.println("read ID");
	byte ID = Serial1.read();

	if(ID != servoID){
		//Serial.println("wrong ID: " + String(ID, DEC) + " Instruction: " + String(Serial1.read(), DEC));
		SerialTools::readDummyBytesFromSerial(Serial1, 5);
		//Serial.println(Serial1.available());
		return;
	}

	byte instruction = Serial1.read();
	long data = SerialTools::readLongFromSerial(Serial1);

	//if it's not a handshake or initialization request ignore the instruction!
	if(!isInitialized){
		if (instruction == MocoJoServoHandshakeRequest || instruction == MocoJoServoInitializeRequest){
			//do nothing
		}
		else{
			return;
		}
	}
	//Serial.println("instruction: " + String(instruction, DEC));
	//Serial.println("data: " + String(data, DEC));
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
			//exitSafeStart();
			break;

		//Playback:
		case MocoJoServoStartPlayback:
			isPlayback = true;
			startPlayback();
			break;

		case MocoJoServoStopPlayback:
			stopPlayback();
			break;
		case MocoJoServoProceedToHone:
			proceedToHone = true;
			break;
		//GETTERS:
		case MocoJoServoGetCurrentPosition:
			MocoJoServoCommunication::writeCurrentPositionToMCU(Serial1, servoID, servoCurrentPosition);
			break;

		case MocoJoServoGetMotorTargetSpeed:
			MocoJoServoCommunication::writeMotorTargetSpeedToMCU(Serial1, servoID, motorTargetSpeed);
			break;

		case MocoJoServoGetPositionAtLastSync:
			MocoJoServoCommunication::writePositionAtLastSyncToMCU(Serial1, servoID, servoPositionAtLastSync);
			break;

		case MocoJoServoGetIsHoning:
			MocoJoServoCommunication::writeIsHoningToMCU(Serial1, servoID, isHoning);
			break;

		//SETTERS:
		case MocoJoServoSetTargetPosition:
			if(!isPlayback){
				servoTargetPosition = data;	
			}
			break;

		case MocoJoServoAddTargetPositionToBuffer:
			if(!isPlayback){
			//	Serial.println("MCU tried to add to target position buffer when not in playback!");
				return;
			}
			if(servoTargetPositionBuffer.isFull()){
			//	Serial.println("MCU wrote to target position buffer when it was full!");
				stopPlayback();
			}
			servoTargetPositionBuffer.addLong(data);	
			break;

		case MocoJoServoSetMaxSpeed:
			//implement this?
			break;
		
		default:
			break;
			//Serial.println("Instruction Invalid: " + String(instruction, DEC) + " Data: " + String(data, DEC));
		
	}
}









