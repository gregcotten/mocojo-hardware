#include <AS5045.h>

AS5045 encoder(4,5,6, 1.0, true);
unsigned long then;

unsigned long sampleThen;

void setup(){
  Serial.begin(115200);
  then = millis();
}

void loop(){
	sampleThen = micros();
	encoder.update();
	Serial.println(micros() - sampleThen);
	
	// if(millis() - then >= 100){
	// 	Serial.println("Position: " + String(encoder.getAbsolutePosition(), DEC) + " Velocity: " + String((int)encoder.getVelocity(), DEC));
	// 	then = millis();	
	// }
	
}


