#include <plib.h>
#define SYS_FREQ	 (80000000) 

long start;
long counter = 0;

void setup()
{
	Serial.begin(1000000)
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE); 
	T1CON = 0x0; // Stop the Timer and Reset Control register
	// Set prescaler at 1:1, internal clock source
	TMR1 = 0x0; // Clear timer register
	PR1 = 0x0001; // Load period register
	IPC1SET = 0x000C; // Set priority level=3
	IPC1SET = 0x0001; // Set subpriority level=1
	IFS0CLR = 0x0010; // Clear Timer interrupt status flag
	IEC0SET = 0x0010; // Enable Timer interrupts
	T1CONSET = 0x8000; // Start Time
	
	start = millis();
}

void loop()
{
	if (millis() - start > 1000){
		Serial.println(counter);
	}
}


extern "C"
{
	void __ISR(_TIMER_1_VECTOR,ipl3) pwmOn(void)
	{
		counter++;
		mT1ClearIntFlag(); // Clear interrupt flag
	}
}