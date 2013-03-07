#include <SMC.h>
#include <SMCProtocolConstants.h>
#include <AS5045.h>
#include <PID_v1.h>

long servoCurrentPosition = 0;
long servoCurrentRelativePosition = 0;
long servoCurrentVelocity = 0;

volatile long servoPositionAtLastSync=0;
volatile long servoVelocityAtLastSync=0;

long servoResolution = 8*4095;

long servoTargetPosition = 0;
long servoTargetVelocity = 0;
long motorTargetSpeed = 0;

PID servoPositionPID(&servoCurrentPosition, &motorTargetSpeed, &servoTargetPosition,1,0,0, REVERSE);
const int servoPIDSampleTimeMillis = 1;

AS5045 tiltEncoder(8,9,10, .5, false);

AS5045 servoEncoder(4,5,6, 1.0, false);
SMC motorController(Serial1, 2, 3); //change this to Serial2

void setup(){
	Serial.begin(1000000);
	Serial1.begin(400000);

	servoPositionPID.SetOutputLimits(-3200, 3200);
	servoPositionPID.SetSampleTime(servoPIDSampleTimeMillis);
	servoPositionPID.SetMode(AUTOMATIC);

	motorController.setMaximumSpeed(500); //128 = 4% power
	motorController.initialize();
	motorController.exitSafeStart();
	
	
	servoEncoder.setAbsolutePosition(0);
	tiltEncoder.setAbsolutePosition(0);
}

unsigned long timeAtLastSerialUpdate = 0;

void loop(){
 	doPIDDuties();
 	updateInputEncoders();
	if(millis() - timeAtLastSerialUpdate > 1){
		Serial.println("RP "+String(servoCurrentRelativePosition, DEC) +" CP " + String(servoCurrentPosition, DEC) + " TP " + String(servoTargetPosition, DEC) + " D " + String(servoTargetPosition-servoCurrentPosition, DEC) + " TM " + String(motorTargetSpeed, DEC));

		//Serial.println(servoTargetPosition-servoCurrentPosition);
		//servoTargetPosition+=1;
		servoTargetPosition = tiltEncoder.getAbsolutePosition();
		timeAtLastSerialUpdate = millis();
	}
	
}

void updateInputEncoders(){
	tiltEncoder.update();
	 
}

void doPIDDuties(){

	 
	
	 servoEncoder.update();
	 servoCurrentPosition = servoEncoder.getAbsolutePosition();
	 servoCurrentRelativePosition = servoEncoder.getRelativePosition();
	 servoCurrentVelocity = servoEncoder.getVelocity();

	
	if(servoPositionPID.Compute()){
		motorController.setMotorSpeed(motorTargetSpeed);	
	}

	
}