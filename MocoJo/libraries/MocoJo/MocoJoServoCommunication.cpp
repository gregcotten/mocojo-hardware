#include <MocoJoServoCommunication.h>
#include <MocoJoServoProtocol.h>
#include <SerialTools.h>

void MocoJoServoCommunication::writeHandshakeSuccessToMCU(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoHandshakeSuccessfulResponse);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}
	
void MocoJoServoCommunication::writeCurrentPositionToMCU(HardwareSerial &serial, int ID, long currentPosition){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoCurrentPosition);
	SerialTools::writeLongToSerial(serial, currentPosition);
}

void MocoJoServoCommunication::writePositionAtLastSyncToMCU(HardwareSerial &serial, int ID, long positionAtLastSync){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoPositionAtLastSync);
	SerialTools::writeLongToSerial(serial, positionAtLastSync);
}

void MocoJoServoCommunication::writeMocoJoServoDidHoneToFirstPosition(HardwareSerial &serial, int ID){
	(&serial) -> write(ID);
	(&serial) -> write(MocoJoServoDidHoneToFirstPosition);
	SerialTools::writeDummyBytesToSerial(serial, 4);
}
