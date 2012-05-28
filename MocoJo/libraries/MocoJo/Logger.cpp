#include <Logger.h>

#include <wiring.h>
#include <WString.h>
#include <WProgram.h>
#include <MocoProtocolConstants.h>

static boolean useDebug;
static boolean useDebugHighPriority;

void Logger::setDebugMode(boolean useD, boolean useDHigh){
	useDebug = useD;
	useDebugHighPriority = useDHigh;
}

void Logger::writeDebugString(String str, boolean force)
{
	if(!useDebugHighPriority){
		force = false;
	}
	if(!useDebug && !force){
		return;
	}
	Serial.write(MocoProtocolNewlineDelimitedDebugStringResponseType);
	Serial.println(str);
}