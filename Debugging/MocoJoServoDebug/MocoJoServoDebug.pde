#include <MocoProtocolConstants.h>
#include <MocoJoServoRepresentation.h>
#include <MocoJoServoProtocol.h>

MocoJoServoRepresentation servoJibLift(Serial1, MocoAxisJibLift);

void setup(){
	Serial.begin(115200);
	Serial1.begin(MocoJoServoBaudRate);
	Serial.println("Started up the shield!");
	if (servoJibLift.handshake()){
		Serial.println("hands shook!");
	}


}

void loop(){
	unsigned long then = micros();
	servoJibLift.getCurrentPosition();
	unsigned long duration = micros() - then;
	Serial.println("time: " + String(duration, DEC));
	delay(100);

}