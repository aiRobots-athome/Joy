#include "Motor.h"

Motor::Motor()
	: motor(),
	  BAUDRATE(0),
	  Motor_ID(0),
	  ADDR_TORQUE_ENABLE(0),
	  ADDR_GOAL_POSITION(0),
	  ADDR_GOAL_VELOCITY(0),
	  ADDR_GOAL_TORQUE(0),
	  ADDR_PRESENT_POSITION(0),
	  ADDR_PRESENT_VELOCITY(0),
	  ADDR_PRESENT_TORQUE(0),
	  LEN_TORQUE_ENABLE(0),
	  LEN_GOAL_POSITION(0),
	  LEN_GOAL_VELOCITY(0),
	  LEN_GOAL_TORQUE(0),
	  LEN_PRESENT_POSITION(0),
	  LEN_PRESENT_VELOCITY(0),
	  LEN_PRESENT_TORQUE(0)
{
	connected = false;
	write_count = 0;
	read_count = 0;
}

Motor::Motor(
	const unsigned int &baudrate,
	const unsigned char &id,
	const uint16_t &addr_torque_enable,
	const uint16_t &addr_goal_position,
	const uint16_t &addr_goal_velocity,
	const uint16_t &addr_goal_torque,
	const uint16_t &addr_present_position,
	const uint16_t &addr_present_velocity,
	const uint16_t &addr_present_torque,
	const uint16_t &len_torque_enable,
	const uint16_t &len_goal_position,
	const uint16_t &len_goal_velocity,
	const uint16_t &len_goal_torque,
	const uint16_t &len_present_position,
	const uint16_t &len_present_velocity,
	const uint16_t &len_present_torque)
	: motor(),
	  BAUDRATE(baudrate),
	  Motor_ID(id),
	  ADDR_TORQUE_ENABLE(addr_torque_enable),
	  ADDR_GOAL_POSITION(addr_goal_position),
	  ADDR_GOAL_VELOCITY(addr_goal_velocity),
	  ADDR_GOAL_TORQUE(addr_goal_torque),
	  ADDR_PRESENT_POSITION(addr_present_position),
	  ADDR_PRESENT_VELOCITY(addr_present_velocity),
	  ADDR_PRESENT_TORQUE(addr_present_torque),
	  LEN_TORQUE_ENABLE(len_torque_enable),
	  LEN_GOAL_POSITION(len_goal_position),
	  LEN_GOAL_VELOCITY(len_goal_velocity),
	  LEN_GOAL_TORQUE(len_goal_torque),
	  LEN_PRESENT_POSITION(len_present_position),
	  LEN_PRESENT_VELOCITY(len_present_velocity),
	  LEN_PRESENT_TORQUE(len_present_torque)
{
	connected = false;
	write_count = 0;
	read_count = 0;
}

const unsigned char &Motor::GetMotorID() const { return Motor_ID; }
const bool &Motor::GetMotorConnected() const { return connected; }

void Motor::ConnectDynamixel(
	dynamixel::PortHandler *portHandler, 
	dynamixel::PacketHandler *packetHandler,
	dynamixel::GroupBulkRead *groupBulkRead,
	dynamixel::GroupBulkWrite *groupBulkWrite)
{
	this->portHandler = portHandler;
	this->packetHandler = packetHandler;
	this->groupBulkRead = groupBulkRead;
	this->groupBulkWrite = groupBulkWrite;
	ConnectDynamixel();
}

void Motor::ConnectDynamixel()
{
	uint8_t dxl_error = 0;
	uint16_t dxl_model_number = 0; // Dynamixel model number
	if (portHandler->openPort() & portHandler->setBaudRate(BAUDRATE))
	{
		int dxl_comm_result = packetHandler->ping(portHandler, Motor_ID, &dxl_model_number, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			portHandler->closePort();
			connected = false;
		}
		else if (dxl_error != 0)
		{
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			portHandler->closePort();
			connected = false;
		}
		else
		{
			// printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", Motor_ID, dxl_model_number);
			connected = true;
		}
	}
	else
	{
		portHandler->closePort();
		connected = false;
	}
}

void Motor::WriteData()
{
	// !!!!!! Must write torque enable first
	switch (write_count)
	{
	case 0:
		if (is_Write_TorqueEnable)
			WriteTorqueEnable();
		write_count = 1;
		break;
	case 1:
		if (is_Write_Torque)
			WriteTorque();
		write_count = 2;
		break;
	case 2:
		if (is_Write_Velocity)
			WriteVelocity();
		write_count = 3;
		break;
	case 3:
		if (is_Write_Scale)
			WriteScale();
		write_count = 0;	
		break;
	}
}

void Motor::AddParam()
{
	switch (read_count)
	{
	case 0:
		AddParamPresentAngle();
		break;
	case 1:
		AddParamPresentVelocity();
		break;
	case 2:
		AddParamPresentTorque();
		break;
	}
}

void Motor::ReadData()
{
	switch (read_count)
	{
	case 0:
		ReadPresentAngle();
		if (abs(Motor_Angle - Motor_Present_Angle) < 1.0)
			is_Arrival = true;
		else
			is_Arrival = false;
		read_count = 1;
		break;
	case 1:
		ReadPresentVelocity();
		read_count = 2;
		break;
	case 2:
		ReadPresentTorque();
		read_count = 0;
		break;
	}
}

void Motor::AddParamPresentAngle()
{
	bool dxl_addparam_result = groupBulkRead->addParam(Motor_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
}

void Motor::AddParamPresentVelocity()
{
	bool dxl_addparam_result = groupBulkRead->addParam(Motor_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
}

void Motor::AddParamPresentTorque()
{
	bool dxl_addparam_result = groupBulkRead->addParam(Motor_ID, ADDR_PRESENT_TORQUE, LEN_PRESENT_TORQUE);
}

void Motor::ReadPresentAngle()
{
	int16_t data = groupBulkRead->getData(Motor_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
	Motor_Present_Angle = (data - Motor_CenterScale) * MotorScale2Angle;
}

void Motor::ReadPresentVelocity()
{
	int16_t data = groupBulkRead->getData(Motor_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
	Motor_Present_Velocity = data;
}

void Motor::ReadPresentTorque()
{
	int16_t data = groupBulkRead->getData(Motor_ID, ADDR_PRESENT_TORQUE, LEN_PRESENT_TORQUE);
	Motor_Present_Torque = data / Max_Torque_Limit * 100;
}

void Motor::WriteScale()
{
	uint8_t param_goal_position[LEN_GOAL_POSITION];
	param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(Motor_Scale));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(Motor_Scale));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(Motor_Scale));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(Motor_Scale));

	groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position);
}

void Motor::WriteVelocity()
{
	uint8_t param_goal_velocity[LEN_GOAL_VELOCITY];
	param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(Motor_Velocity));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(Motor_Velocity));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(Motor_Velocity));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(Motor_Velocity));

	groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY, param_goal_velocity);
}

void Motor::WriteTorque()
{
	uint8_t param_goal_torque[LEN_GOAL_TORQUE];
	param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(Motor_Torque));
    param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(Motor_Torque));

	groupBulkWrite->addParam(Motor_ID, ADDR_GOAL_TORQUE, LEN_GOAL_TORQUE, param_goal_torque);
}

void Motor::WriteTorqueEnable()
{
	uint8_t param_torque_enable[LEN_TORQUE_ENABLE];
	param_torque_enable[0] = Motor_TorqueEnable;

	groupBulkWrite->addParam(Motor_ID, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE, param_torque_enable);
}