#include <SMC.h>

SMC smc(Serial);

void setup(){
	Serial.begin(500000);
	smc.initialize();
	smc.exitSafeStart();
	smc.setMotorSpeed(-1000);
}

void loop(){
	delay(2000);
	smc.stopMotor();
	delay(2000);
	smc.exitSafeStart();
	smc.setMotorSpeed(-1000);

}