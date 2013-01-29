#include <SMC.h>

SMC smc(Serial1*);

void setup(){
	smc.initialize();
	smc.setMotorSpeed(1200);
}

void loop(){

}