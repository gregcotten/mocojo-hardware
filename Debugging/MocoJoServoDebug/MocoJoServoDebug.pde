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
	//servoJibLift.exitSafeStart();
	servoJibLift.setTargetPosition(4096);

	delay(100);

}
void loop(){
	Serial.println(servoJibLift.getMotorTargetSpeed());
	delay(100);
}