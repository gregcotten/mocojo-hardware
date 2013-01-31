#include <MocoProtocolConstants.h>
#include <MocoJoServoRepresentation.h>

MocoJoServoRepresentation servoJibLift(Serial1, MocoAxisJibLift);

void setup(){
	Serial.begin(115200);
	Serial.println("Started up the shield!");
	if (servoJibLift.handshake()){
		Serial.println("hands shook!");
	}

}

void loop(){

}