//Initializing
enum{
	MocoJoServoBaudRate = 1000000,
	MocoJoServoHandshakeRequest = 0,
	MocoJoServoHandshakeSuccessfulResponse = 1,
	MocoJoServoInitializeRequest = 14

};

//Commands

enum{
	MocoJoServoStartPlayback = 2,
	MocoJoServoStopPlayback = 3,
	MocoJoServoStopEverything = 4,
	MocoJoServoExitSafeStart = 5,
	MocoJoServoStartPlayback = 20,
	MocoJoServoStopPlayback = 21,
	MocoJoServoDidHoneToFirstPosition = 22

};

//Getters

enum{
	MocoJoServoGetCurrentPosition = 6,
	MocoJoServoGetPositionAtLastSync = 7
};

//Setters

enum{
	MocoJoServoSetTargetPosition = 9,
	MocoJoServoSetTargetPositionForNextSync = 10,
	MocoJoServoSetMaxSpeed = 11
};

// Variable IDs
enum{
	MocoJoServoCurrentPosition = 12,
	MocoJoServoPositionAtLastSync = 13
};


