#include <ChangeNotification.h>

const int interruptPin = 13;

void setup(){
	Serial.begin(115200);
	Serial.println("attaching interrupt");
	pinMode(13, INPUT);
	attachInterrupt(CN_8, interrupt, RISING);
}

volatile long var1 = 0;
volatile long var2 = 0;
volatile long interruptCount = 0;
void loop(){
	var1 = var1+1;
	Serial.println(digitalRead(13));
	Serial.println("interrupt count: " + String(interruptCount, DEC));
}

void interrupt(){
	interruptCount++;
	//var2 = var1;
}

