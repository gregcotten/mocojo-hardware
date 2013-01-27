//Initializing
enum{
	MocoJoServoBaudRate = 1000000,
	MocoJoServoHandshakeRequest = 0,
	MocoJoServoHandshakeResponse = 1

};

//Commands

enum{
	MocoJoServoStartPlayback = 2,
	MocoJoServoStopPlayback = 3,
	MocoJoServoStopEverything = 4,
	MocoJoServoExitSafeStart = 5

};

//Getters

enum{
	MocoJoServoGetCurrentPosition = 6,
	MocoJoServoGetPositionAtLastSync = 7,
	MocoJoServoGetID = 8
}

//Setters

enum{
	MocoJoServoSetTargetPosition = 9,
	MocoJoServoSetMaxSpeed = 10
}


