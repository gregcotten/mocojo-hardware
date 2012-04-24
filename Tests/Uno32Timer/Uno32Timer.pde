#include <plib.h>

#define SYS_FREQ 80000000

void StartTimer1(float timeInSeconds)
{
	unsigned long period = (unsigned long)((timeInSeconds*SYS_FREQ)/256) - 1;
	OpenTimer1(T1_ON | T1_PS_1_256 | T1_SOURCE_INT, period);
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);
	INTEnableSystemMultiVectoredInt();
	WriteTimer1(0);
}

void StopTimer1()
{
	CloseTimer1();
}



long start;
volatile long counter = 0;

void setup()
{
	Serial.begin(1000000);
	
}


void loop()
{
	if(millis() - start > 1000){
		Serial.println(counter, DEC);
		start = millis();
		counter =0;
		CloseTimer1();
	}
	
}


extern "C"
{
	
	void __ISR(_TIMER_1_VECTOR,ipl1) pwmOn(void)
	{
		counter++;
		mT1ClearIntFlag(); // Clear interrupt flag
	}
	
	
}

