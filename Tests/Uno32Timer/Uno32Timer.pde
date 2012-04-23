#include <plib.h>

// Configuration Bit settings 
#define SYS_FREQ (80000000L) 
// Let compile time pre-processor calculate the PR1 (period) 
#define PB_DIV           1 
#define PRESCALE         256 
#define TOGGLES_PER_SEC  1 
#define T1_TICK_RATE         (SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)


long start;
volatile long counter = 0;

void setup()
{
	Serial.begin(1000000);
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE); 
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
	// STEP 2. configure Timer 1 using internal clock, 1:256 prescale 
	OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK_RATE); 
	// set up the timer interrupt with a priority of 2 
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2); 
	// enable multi-vector interrupts 
	INTEnableSystemMultiVectoredInt();
	start = millis();
}


void loop()
{
	if(millis() - start > 1000){
		Serial.println(counter, DEC);
		start = millis();
		counter =0;
	}
	
}


extern "C"
{
	void __ISR(_TIMER_1_VECTOR,ipl2) pwmOn(void)
	{
		counter++;
		mT1ClearIntFlag(); // Clear interrupt flag
	}
}