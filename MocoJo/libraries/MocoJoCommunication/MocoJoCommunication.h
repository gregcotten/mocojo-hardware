#ifndef MocoJoCommunication_h
#define MocoJoCommunication_h



namespace MocoJoCommunication {
	long getNextFrameFromComputer(int axisID);
	void writeHandshakeSuccessToComputer();
	void writeRequestForNextFrameToComputer(int axisID);
	void writePlaybackHasStartedToComputer();
	void writePlaybackHasCompletedToComputer();
}

#endif