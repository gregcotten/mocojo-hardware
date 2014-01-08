#ifndef MocoTimer1_h
#define MocoTimer1_h

namespace MocoTimer1 {
	extern unsigned long period;
	extern void (*function)();

	void set(float timeInSeconds, void (*func)());
	void start();
	void stop();
	void overflow();
}

#endif
