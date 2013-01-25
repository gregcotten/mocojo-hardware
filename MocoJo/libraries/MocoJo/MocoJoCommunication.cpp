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
	SerialTools::writeDummyBytesToSerial(Serial, 3);
}

void MocoJoCommunication::writeRequestForNextFrameToComputer()
{
	Serial.write(MocoProtocolAdvancePlaybackRequestType); 
	Serial.write(10); //bogus axis
	
	//TODO: Slipstream previous position of axis into request instead of sending meaningless info here
	SerialTools::writeLongToSerial(Serial, 1);
}

void MocoJoCommunication::writePlaybackHasStartedToComputer()
{
	Serial.write(MocoProtocolPlaybackStartingNotificationResponseType);
	SerialTools::writeDummyBytesToSerial(Serial, 5);
}

void MocoJoCommunication::writePlaybackHasCompletedToComputer()
{
	Serial.write(MocoProtocolPlaybackCompleteNotificationResponseType);
	SerialTools::writeDummyBytesToSerial(Serial, 5);
}
