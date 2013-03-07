#include <AS5045.h>
#include <SMC.h>

AS5045 encoder(4,5,6, 1.0, true);
SMC motorController(Serial1, 2, 3); //change this to Serial2


unsigned long then;

unsigned long sampleThen;

void setup(){
  Serial.begin(1000000);
  Serial1.begin(400000);

  motorController.setMaximumSpeed(500); 
  motorController.initialize();
  motorController.exitSafeStart();
  delay(1000);
  encoder.setAbsolutePosition(0);
  motorController.setMotorSpeed(800);
}

void loop(){
	//sampleThen = micros();
	encoder.update();
	//Serial.println(micros() - sampleThen);
	
	
	if(millis() - then >= 5){
		Serial.println("P " + String(encoder.getAbsolutePosition(), DEC) + " R " + String(encoder.getRelativePosition(), DEC));
		//Serial.println(encoder.getVelocity());	
		then = millis();	
	}
	
	
}


