#include <AS5045.h>
#include <MathHelper.h>
#include <SMC.h>

AS5045 encoder(4,5,6, 1.0, true);
SMC motorController(Serial1, 2, 3); //change this to Serial2


unsigned long then;

unsigned long sampleThen;

void setup(){
  Serial.begin(115200);
  Serial1.begin(9600);

  //motorController.setMaximumSpeed(500); 
  motorController.initialize();
  motorController.exitSafeStart();
  delay(1000);
  encoder.setAbsolutePosition(0);
  motorController.setMotorSpeed(.03);
}

void loop(){
	//sampleThen = micros();
	encoder.update();
	//Serial.println(micros() - sampleThen);
	
	
	if(millis() - then >= 20){
		Serial.println("P " + String(encoder.getAbsolutePosition(), DEC) + " R " + String(encoder.getRelativePosition(), DEC) + " V " + String(encoder.getVelocity(), 4));	
		then = millis();	
	}
	
	
}


