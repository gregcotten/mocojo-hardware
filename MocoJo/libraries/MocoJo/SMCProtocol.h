// Setup
enum{

	SMCProtocolBaudRate = 500000
};

// Commands
enum{
	SMCProtocolStopMotor = 0xE0
	SMCProtocolExitSafeStart = 0x83,
	SMCProtocolSetMotorForward = 0x85,
	SMCProtocolSetMotorReverse = 0x86,
	SMCProtocolSetMotorBrake = 0x92,
	SMCProtocolGetVariable = 0xA1,
	SMCProtocolGetFirmwareVersion = 0xC2


};

// Variables
enum{
	//Status Flag Registers
	SMCProtocolVariableErrorStatus = 0,
	SMCProtocolVariableErrorsOccured = 1,
	SMCProtocolVariableSerialErrorsOccured = 2,
	SMCProtocolVariableLimitStatus = 3,
	SMCProtocolVariableResetFlags = 127,

	//Diagnostic Variables
	SMCProtocolVariableTargetSpeed = 20,
	SMCProtocolVariableSpeed = 21,
	SMCProtocolVariableBrakeAmount = 22,
	SMCProtocolVariableInputVoltage = 23,
	SMCProtocolVariableTemperature = 24,
	SMCProtocolVariableBaudRateRegister = 27,
	SMCProtocolVariableSystemTimeLow = 28,
	SMCProtocolVariableSystemTimeHigh = 29
};