"""CAN driver package for interfacing with motor controllers."""

from drivers.can import (
    connection,
    enums,
    messages,
    myactuator_v3_messages,
    odrive_messages,
    x4_24_messages,
)

# Connection methods and classes
CANSimple = connection.CANSimple
CANSimpleListener = connection.CANSimpleListener
BusType = enums.BusType
CANInterface = enums.CANInterface

# Base message types
CanMessage = messages.CanMessage
OdriveArbitrationID = messages.OdriveArbitrationID
MyActuatorArbitrationID = messages.MyActuatorArbitrationID
X424ArbitrationID = messages.X424ArbitrationID

# ODrive message classes
ClearErrorsCommand = odrive_messages.ClearErrorsCommand
WriteParameterCommand = odrive_messages.WriteParameterCommand
ReadParameterCommand = odrive_messages.ReadParameterCommand
ParameterResponse = odrive_messages.ParameterResponse
EncoderEstimatesMessage = odrive_messages.EncoderEstimatesMessage
HeartbeatMessage = odrive_messages.HeartbeatMessage
SetVelocityMessage = odrive_messages.SetVelocityMessage
SetPositionMessage = odrive_messages.SetPositionMessage
SetTorqueMessage = odrive_messages.SetTorqueMessage
SetAxisStateMessage = odrive_messages.SetAxisStateMessage
SetControllerMode = odrive_messages.SetControllerMode
EStop = odrive_messages.EStop
Reboot = odrive_messages.Reboot

# MyActuator V3 controller message classes
ReadMotorStatus1Message = myactuator_v3_messages.MyactuatorReadMotorStatus1Message
ReadMotorStatus2Message = myactuator_v3_messages.ReadMotorStatus2Message
ReadMultiTurnAngleMessage = myactuator_v3_messages.ReadMultiTurnAngleMessage
PositionControlCommand = myactuator_v3_messages.PositionControlCommand
SystemBrakeReleaseCommand = myactuator_v3_messages.SystemBrakeReleaseCommand
SystemBrakeLockCommand = myactuator_v3_messages.SystemBrakeLockCommand
MotorShutdownCommand = myactuator_v3_messages.MotorShutdownCommand
MotorStopCommand = myactuator_v3_messages.MotorStopCommand
SystemResetCommand = myactuator_v3_messages.SystemResetCommand
CANIDCommand = myactuator_v3_messages.CANIDCommand
VersionAcquisitionCommand = myactuator_v3_messages.VersionAcquisitionCommand
FunctionControlCommand = myactuator_v3_messages.FunctionControlCommand
SpeedControlCommand = myactuator_v3_messages.SpeedControlCommand
TorqueControlCommand = myactuator_v3_messages.TorqueControlCommand

# X4-24 controller message classes
X424CanMessageSetAndQuery = x4_24_messages.X424CanMessageSetAndQuery
QueryCommunicationModeMessage = x4_24_messages.QueryCommunicationModeMessage
QueryCANCommunicationIDMessage = x4_24_messages.QueryCANCommunicationIDMessage
SetCommunicationModeMessage = x4_24_messages.SetCommunicationModeMessage
SetZeroPositionMessage = x4_24_messages.SetZeroPositionMessage
SetMotorIDMessage = x4_24_messages.SetMotorIDMessage
ResetMotorIDMessage = x4_24_messages.ResetMotorIDMessage
X424ServoPositionControlMessage = x4_24_messages.X424ServoPositionControlMessage
X424ServoSpeedControlMessage = x4_24_messages.X424ServoSpeedControlMessage
X424CurrentControlMessage = x4_24_messages.X424CurrentControlMessage
QAReturnMessage = x4_24_messages.QAReturnMessage
QAReturnMessageType1 = x4_24_messages.QAReturnMessageType1
QAReturnMessageType2 = x4_24_messages.QAReturnMessageType2
QAReturnMessageType3 = x4_24_messages.QAReturnMessageType3
QAReturnMessageType4 = x4_24_messages.QAReturnMessageType4
QAReturnMessageType5 = x4_24_messages.QAReturnMessageType5
