#ifndef MocoJoCommunication_h
#define MocoJoCommunication_h



namespace MocoJoCommunication {
	void writeHandshakeSuccessToComputer();
	void writeRequestForNextFrameToComputer();
	void writePlaybackHasStartedToComputer();
	void writePlaybackHasCompletedToComputer();
}

#endif