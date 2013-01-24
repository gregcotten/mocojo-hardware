#include <MocoJoCommunication.h>
#include <SerialTools.h>
#include <wiring.h>
#include <WProgram.h>
#include <MocoProtocolConstants.h>

void MocoJoCommunication::writeHandshakeSuccessToComputer()
{
	Serial.write(MocoProtocolHandshakeResponseType);
	Serial.write(1);
	Serial.write(MocoProtocolHandshakeSuccessfulResponse);
	SerialTools.writeDummyBytesToSerial(3);
}

void MocoJoCommunication::writeRequestForNextFrameToComputer(int axisID)
{
	Serial.write(MocoProtocolAdvancePlaybackRequestType); 
	Serial.write(axisID);
	
	//TODO: Slipstream previous position of axis into request instead of sending meaningless info here
	SerialTools::writeLongToSerial(1);
}

void MocoJoCommunication::writePlaybackHasStartedToComputer()
{
	Serial.write(MocoProtocolPlaybackStartingNotificationResponseType);
	SerialTools::writeDummyBytesToSerial(5);
}

void MocoJoCommunication::writePlaybackHasCompletedToComputer()
{
	Serial.write(MocoProtocolPlaybackCompleteNotificationResponseType);
	SerialTools::writeDummyBytesToSerial(5);
}
