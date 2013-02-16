#include <AS5045.h>

AS5045 encoder(8,9,10, 1.0, false);
unsigned long then;

unsigned long sampleThen;

void setup(){
  Serial.begin(115200);
  then = millis();
}

void loop(){
	//sampleThen = micros();
	encoder.update();
	//Serial.println(micros() - sampleThen);
	
	
	if(millis() - then >= 10){
		Serial.println("P " + String(encoder.getAbsolutePosition(), DEC) + " R " + String(encoder.getRelativePosition(), DEC));
		//Serial.println(encoder.getVelocity());	
		then = millis();	
	}
	
	
}


