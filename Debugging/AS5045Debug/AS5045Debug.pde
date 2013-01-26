#include <AS5045.h>

AS5045 encoder(4,5,6,true);
int then;
void setup(){
  Serial.begin(115200);
  then = millis();
}
int measure;
void loop(){
	encoder.update();
	if(millis() - then >= 100){
		Serial.println(encoder.getAbsolutePosition());
		then = millis();	
	}
	
}


