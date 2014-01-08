#ifndef Logger_h
#define Logger_h

#include <wiring.h>
#include <WString.h>
#include <WProgram.h>

namespace Logger {
	static boolean useDebug = false;
	static boolean useDebugHighPriority = true;
	void setDebugMode(boolean useD, boolean useDHigh);
	void writeDebugString(String str, boolean force);
}

#endif