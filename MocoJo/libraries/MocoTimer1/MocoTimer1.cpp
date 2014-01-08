/*
  MocoTimer1.h - PIC32 timer1 implementation with 1ms resolution
  Greg Cotten
*/

#include <MocoTimer1.h>
#include <plib.h>

unsigned long MocoTimer1::period;
void (*MocoTimer1::function)();

void MocoTimer1::set(float timeInSeconds, void (*func)()) {
	period = (unsigned long) (((timeInSeconds*80000000.0)/256.0) - 1.0);
	function = func;
}

void MocoTimer1::start() {
	OpenTimer1(T1_ON | T1_PS_1_256 | T1_SOURCE_INT, period);
	ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);
	INTEnableSystemMultiVectoredInt();
	WriteTimer1(0);
	
}

void MocoTimer1::stop() {
	CloseTimer1();
}

void MocoTimer1::overflow() {
	(*function)();
}

extern "C"
{
	void __ISR(_TIMER_1_VECTOR,ipl1) MocoTimer1Alarm(void)
	{
		mT1ClearIntFlag(); // Clear interrupt flag
		MocoTimer1::overflow();
	}
}

