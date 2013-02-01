#include <MocoProtocolConstants.h>
#include <MocoJoServoRepresentation.h>
#include <MocoJoServoProtocol.h>

MocoJoServoRepresentation servoJibLift(Serial1, MocoAxisJibLift);

int syncOutPin = 2;

void setup(){
	pinMode(syncOutPin, OUTPUT);

	Serial.begin(115200);
	Serial1.begin(MocoJoServoBaudRate);
	Serial.println("Started up the shield!");
	if (servoJibLift.handshake()){
		Serial.println("hands shook!");
	}
	delay(100);

}
long i = 0;
void loop(){
	//unsigned long then = micros();
	digitalWrite(syncOutPin, LOW);
	delay(20);
	digitalWrite(syncOutPin, HIGH);
	
	Serial.println(servoJibLift.getCurrentPosition());
	servoJibLift.setTargetPosition(i++);

	//unsigned long duration = micros() - then;
	//Serial.println("time: " + String(duration, DEC));
	delay(100);

}