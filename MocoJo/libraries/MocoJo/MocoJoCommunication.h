#ifndef MocoJoCommunication_h
#define MocoJoCommunication_h



namespace MocoJoCommunication {
	void writeHandshakeSuccessToComputer();
	void writeRequestForNextAxisPositionToComputer(int axisID);
	void writePlaybackHasStartedToComputer();
	void writePlaybackHasCompletedToComputer();
}

#endif