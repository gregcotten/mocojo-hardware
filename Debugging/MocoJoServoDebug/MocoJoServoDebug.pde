#include <MocoProtocolConstants.h>
#include <MocoJoServoRepresentation.h>
#include <MocoJoServoProtocol.h>
#include <AS5045.h>

MocoJoServoRepresentation servoJibLift(Serial1, MocoAxisJibLift);
AS5045 tiltEncoder(8,9,10, .5, false);

int syncOutPin = 2;
unsigned long timeAtLastSerialUpdate = 0;

void setup(){
	pinMode(syncOutPin, OUTPUT);

	Serial.begin(115200);
	Serial1.begin(MocoJoServoBaudRate);
	Serial.println("Started up the shield!");
	if (servoJibLift.handshake()){
		Serial.println("hands shook!");
	}
	delay(5);
	//servoJibLift.exitSafeStart();
	tiltEncoder.setAbsolutePosition(0);

}
void loop(){
	updateInputEncoders();
	if(millis() - timeAtLastSerialUpdate > 20){
		long currentPosition = servoJibLift.getCurrentPosition();
		Serial.println("CP" + String(currentPosition, DEC) + " TP" + String(tiltEncoder.getAbsolutePosition(), DEC) + " D" + String(tiltEncoder.getAbsolutePosition()-currentPosition, DEC) + " TM" + String(servoJibLift.getMotorTargetSpeed(), DEC));
		//Serial.println(servoTargetPosition-servoCurrentPosition);
		//servoTargetPosition+=1;
		servoJibLift.setTargetPosition(tiltEncoder.getAbsolutePosition());
		timeAtLastSerialUpdate = millis();
	}
	
}



void updateInputEncoders(){
	tiltEncoder.update();
	 
}