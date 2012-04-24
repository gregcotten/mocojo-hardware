/*
  MsTimer2.h - Using timer2 with 1ms resolution
  Javier Valencia <javiervalencia80@gmail.com>
  
  History:
  	29/May/09 - V0.5 added support for Atmega1280 (thanks to Manuel Negri)
  	19/Mar/09 - V0.4 added support for ATmega328P (thanks to Jerome Despatis)
  	11/Jun/08 - V0.3 
  		changes to allow working with different CPU frequencies
  		added support for ATMega128 (using timer2)
  		compatible with ATMega48/88/168/8
	10/May/08 - V0.2 added some security tests and volatile keywords
	9/May/08 - V0.1 released working on ATMEGA168 only
	

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <MocoTimer1.h>
#include <plib.h>

unsigned long MocoTimer1::period;
void (*MocoTimer1::func)();

void MocoTimer1::set(float timeInSeconds, void (*f)()) {
	period = (unsigned long)((timeInSeconds*80000000.0)/256.0) - 1;
	func = f;
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
	(*func)();
}

extern "C"
{
	void __ISR(_TIMER_1_VECTOR,ipl1) MocoTimer1Alarm(void)
	{
		mT1ClearIntFlag(); // Clear interrupt flag
		MocoTimer1::overflow();
	}
}

