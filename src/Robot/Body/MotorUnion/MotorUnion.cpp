#include "MotorUnion.h"

MotorUnion::MotorUnion(const vector<unsigned char> &IDArray,
					   const vector<string> &MotorModelArray,
					   vector<unsigned char> &AllPortNumber)
	: waiting_frequency(50)
{
	for (unsigned char i = 0; i < IDArray.size(); i++)
	{
		if (MotorModelArray.at(i) == "Pro100" || MotorModelArray.at(i) == "Pro200" || MotorModelArray.at(i) == "Pro20")
			Motor_Union.push_back(new MotorPro(IDArray.at(i), MotorModelArray.at(i)));

		else if (MotorModelArray.at(i) == "Pro20+")
			Motor_Union.push_back(new MotorProPlus(IDArray.at(i), MotorModelArray.at(i)));

		else if (MotorModelArray.at(i) == "Mx106" || MotorModelArray.at(i) == "Mx64")
			Motor_Union.push_back(new MotorMx(IDArray.at(i), MotorModelArray.at(i)));
		else
			;
	}

	if (ConnectAllMotors(AllPortNumber))
		BGON();
}

MotorUnion::~MotorUnion()
{
	_is_deleted_thread_BG = true;
	thread_BG->join();
	delete thread_BG;

	deleteInVector(Motor_Union);
	deleteInVector(portHandler);
	deleteInVector(groupBulkRead);
	deleteInVector(groupBulkWrite);
}

template <class T>
void MotorUnion::deleteInVector(vector<T *> tmp_vector)
{
	while(!tmp_vector.empty()) {
        delete tmp_vector.back();
        tmp_vector.pop_back();
    }
}

////////////////////////////////////////////////////////////////////////////////
///  All Motors   //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
const bool MotorUnion::ConnectAllMotors(vector<unsigned char> &AllPortNumber)
{
	vector<unsigned char>::iterator it;
	for (it = AllPortNumber.begin(); it != AllPortNumber.end();)
	{
		// Set the port path
		string port_path = string("/dev/ttyUSB" + to_string(*it));
		// Initialize PortHandler & PacketHandler instance
		dynamixel::PortHandler *tmp_portHandler = dynamixel::PortHandler::getPortHandler(port_path.c_str());
		packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
		dynamixel::GroupBulkRead *tmp_groupBulkRead = new dynamixel::GroupBulkRead(tmp_portHandler,	packetHandler);
		dynamixel::GroupBulkWrite *tmp_groupBulkWrite = new dynamixel::GroupBulkWrite(tmp_portHandler, packetHandler);

		bool port_gate = false;
		for (int i = 0; i < Motor_Union.size(); i++)
		{
			if (Motor_Union.at(i)->GetMotorConnected() == false)
			{
				Motor_Union.at(i)->ConnectDynamixel(tmp_portHandler, packetHandler, tmp_groupBulkRead, tmp_groupBulkWrite);
				port_gate |= Motor_Union.at(i)->GetMotorConnected();
			}
		}
		if (port_gate == true)
		{
			it = AllPortNumber.erase(it);
			portHandler.push_back(tmp_portHandler);
			groupBulkRead.push_back(tmp_groupBulkRead);
			groupBulkWrite.push_back(tmp_groupBulkWrite);
		}
		else
		{
			++it;
			delete tmp_groupBulkWrite;
			delete tmp_groupBulkRead;
			delete tmp_portHandler;
		}
	}

	// Check every motor is connected
	bool connected = true;
	for (int i = 0; i < Motor_Union.size(); i++)
		connected &= Motor_Union.at(i)->GetMotorConnected();

	return connected;
}

const bool MotorUnion::CheckAllMotorsConnected() const
{
	bool connected = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		Motor_Union.at(i)->ConnectDynamixel();
		connected &= Motor_Union.at(i)->GetMotorConnected();
	}
	return connected;
}

const bool MotorUnion::CheckAllMotorsArrival() const
{
	bool arrival = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		arrival &= Motor_Union.at(i)->GetMotor_Arrival();
	}
	return arrival;
}

void MotorUnion::WaitAllMotorsArrival() const
{
	while (!CheckAllMotorsArrival())
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

void MotorUnion::WaitAllMotorsArrival(const int &total_waiting_time_ms) const
{
	for (int i = 0; i < total_waiting_time_ms / waiting_frequency; i++)
	{
		if (GetAllMotorsTorqueEnable() == false)
			break;
		this_thread::sleep_for(chrono::milliseconds(waiting_frequency));
	}
}

//------------------------------------------------------------------------------//
/*
	Get All Motors Data
*/
const bool MotorUnion::GetAllMotorsTorqueEnable() const
{
	bool tmp = true;
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		tmp &= GetMotor_TorqueEnable(i);
	}
	return tmp;
}

//------------------------------------------------------------------------------//
/*
	Set Motors Data
*/
void MotorUnion::SetAllMotorsAngle(const float &angle) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Angle(i, angle);
	}
}

void MotorUnion::SetAllMotorsVelocity(const int &velocity) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Velocity(i, velocity);
	}
}

void MotorUnion::SetAllMotorsVelocityWithTime(const short &ms) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Velocity_withTime(i, ms);
	}
}

void MotorUnion::SetAllMotorsTorque(const short &torque) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Torque(i, torque);
	}
}

void MotorUnion::SetAllMotorsTorqueEnable(const bool &enable) const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_TorqueEnable(i, enable);
	}
}

void MotorUnion::RecoveryState() const
{
	for (int i = 0; i < Motor_Union.size(); i++)
	{
		SetMotor_Torque(i, Motor_Union.at(i)->GetMotor_Torque());
		SetMotor_TorqueEnable(i, Motor_Union.at(i)->GetMotor_TorqueEnable());
	}
}

////////////////////////////////////////////////////////////////////////////////
///   Motor   //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
/*
	Get Motor Data
*/
const unsigned char &MotorUnion::GetMotor_ID(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotorID();
}

const bool &MotorUnion::GetMotor_Connected(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotorConnected();
}

const float &MotorUnion::GetMotor_Scale2RPM(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Scale2RPM();
}

const short &MotorUnion::GetMotor_CenterScale(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_CenterScale();
}

const float &MotorUnion::GetMotor_Angle(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Angle();
}

const int &MotorUnion::GetMotor_Scale(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Scale();
}

const int &MotorUnion::GetMotor_Velocity(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Velocity();
}

const short &MotorUnion::GetMotor_Torque(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Torque();
}

const bool &MotorUnion::GetMotor_TorqueEnable(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_TorqueEnable();
}

const float &MotorUnion::GetMotor_PresentAngle(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentAngle();
}

const float &MotorUnion::GetMotor_PresentVelocity(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentVelocity();
}

const float &MotorUnion::GetMotor_PresentTorque(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_PresentTorque();
}

const int &MotorUnion::GetMotor_Max_Position_Limit(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Max_Position_Limit();
}

const int &MotorUnion::GetMotor_Min_Position_Limit(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Min_Position_Limit();
}

const int &MotorUnion::GetMotor_Max_Velocity_Limit(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Max_Velocity_Limit();
}

const int &MotorUnion::GetMotor_Min_Velocity_Limit(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Min_Velocity_Limit();
}

const int &MotorUnion::GetMotor_Max_Torque_Limit(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Max_Torque_Limit();
}

const int &MotorUnion::GetMotor_Min_Torque_Limit(const unsigned char &idx) const
{
	return Motor_Union.at(idx)->GetMotor_Min_Torque_Limit();
}
//------------------------------------------------------------------------------//
/*
	Set Motor Data
*/
void MotorUnion::SetMotor_CenterScale(const unsigned char &idx, const short &motor_center_scale) const
{
	Motor_Union.at(idx)->SetMotor_CenterScale(motor_center_scale);
}

void MotorUnion::SetMotor_Angle(const unsigned char &idx, const float &angle) const
{
	Motor_Union.at(idx)->SetMotor_Angle(angle);
}

void MotorUnion::SetMotor_Velocity(const unsigned char &idx, const int &velocity) const
{
	Motor_Union.at(idx)->SetMotor_Velocity(velocity);
}

void MotorUnion::SetMotor_Velocity_withTime(const unsigned char &idx, const short &ms) const
{
	Motor_Union.at(idx)->SetMotor_Velocity(ms);
}

void MotorUnion::SetMotor_Torque(const unsigned char &idx, const short &torque) const
{
	Motor_Union.at(idx)->SetMotor_Torque(torque);
}

void MotorUnion::SetMotor_TorqueEnable(const unsigned char &idx, const bool &enable) const
{
	Motor_Union.at(idx)->SetMotor_TorqueEnable(enable);
}

////////////////////////////////////////////////////////////////////////////////
///   Background   /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void MotorUnion::BGON()
{
	_is_deleted_thread_BG = false;
	thread_BG = new thread(&MotorUnion::BGReadWrite, this);
}

void MotorUnion::BGReadWrite()
{
	while (!_is_deleted_thread_BG)
	{
		if (CheckAllMotorsConnected())
		{
			if (_is_recovery_state)
			{
				RecoveryState();
				_is_recovery_state = false;
			}
			this_thread::sleep_for(chrono::milliseconds(100));
			WriteData();
			ReadData();
		}
		else
			_is_recovery_state = true;
	}
}

void MotorUnion::WriteData() const
{
	// Add parameters
	bool is_Write = false;
	for (int i = 0; i < Motor_Union.size(); i++)
		is_Write |= Motor_Union.at(i)->WriteData();

	// Write to motor
	if(is_Write)
	{
		for (int i = 0; i < groupBulkWrite.size(); i++)
		{
			int dxl_comm_result = groupBulkWrite.at(i)->txPacket();
			if (dxl_comm_result != COMM_SUCCESS)
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			groupBulkWrite.at(i)->clearParam();
		}
	}
}

void MotorUnion::ReadData() const
{
	// Add parameters
	for (int i = 0; i < Motor_Union.size(); i++)
		Motor_Union.at(i)->AddParam();

	// Read Data
	for (int i = 0; i < groupBulkRead.size(); i++)
	{
		int dxl_comm_result = groupBulkRead.at(i)->txRxPacket();
		if (dxl_comm_result != COMM_SUCCESS)
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}

	// Record to Motor
	for (int i = 0; i < Motor_Union.size(); i++)
		Motor_Union.at(i)->ReadData();

	// clear parameters
	for (int i = 0; i < groupBulkRead.size(); i++)
		groupBulkRead.at(i)->clearParam();
}