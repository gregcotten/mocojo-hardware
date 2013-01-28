#ifndef MocoJoServoCommunication_h
#define MocoJoServoCommunication_h

#include <WProgram.h>

namespace MocoJoServoCommunication {
	void writeHandshakeSuccessToMCU(HardwareSerial serial);
	
	void writeCurrentPositionToMCU(HardwareSerial serial, int ID, long currentPosition);
	void writePositionAtLastSyncToMCU(HardwareSerial serial, int ID, long positionAtLastSync);
}

#endif