#ifndef MocoTimer1_h
#define MocoTimer1_h

namespace MocoTimer1 {
	extern unsigned long period;
	extern void (*func)();

	void set(float timeInSeconds, void (*f)());
	void start();
	void stop();
	void overflow();
}

#endif
