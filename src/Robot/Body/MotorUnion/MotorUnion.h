#pragma once
#include "./Motor/MotorMx.h"
#include "./Motor/MotorPro.h"
#include "./Motor/MotorProPlus.h"
#include <vector>

class MotorUnion
{
public:
	/* 
	@ ID, 
	@ MotorModel, 
	@ Port
	*/
	MotorUnion(const vector<unsigned char> &IDArray,
			   const vector<string> &MotorModelArray,
			   vector<unsigned char> &AllPortNumber);
	virtual ~MotorUnion();
	template <class T>
	void deleteInVector(vector<T *> );

	////////////////////////////////////////////////////////////////////////////////
	///  All Motors   //////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
protected:
	/* Control All Motors */
	const bool ConnectAllMotors(vector<unsigned char> &AllPortNumber);
	const bool CheckAllMotorsConnected() const;
	const bool CheckAllMotorsArrival() const;
	void WaitAllMotorsArrival() const;
	void WaitAllMotorsArrival(const int &total_waiting_time_ms) const;

	/* Set All Motors Data */
	void SetAllMotorsAngle(const float &angle) const;
	void SetAllMotorsVelocity(const int &velocity) const;
	void SetAllMotorsVelocityWithTime(const short &ms) const;
	void SetAllMotorsTorque(const short &torque) const;
	void SetAllMotorsTorqueEnable(const bool &enable) const;

private:
	void RecoveryState() const;
	const bool GetAllMotorsTorqueEnable() const;
	////////////////////////////////////////////////////////////////////////////////
	///   Motor   //////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	/* Get Motor Data */
public:
	const unsigned char &GetMotor_ID(const unsigned char &idx) const;
	const bool &GetMotor_Connected(const unsigned char &idx) const;
	const float &GetMotor_PresentAngle(const unsigned char &idx) const;
	const float &GetMotor_PresentTorque(const unsigned char &idx) const;
	const int &GetMotor_Velocity(const unsigned char &idx) const;

protected:
	const float &GetMotor_Scale2RPM(const unsigned char &idx) const;
	const short &GetMotor_CenterScale(const unsigned char &idx) const;
	const int &GetMotor_Scale(const unsigned char &idx) const;	 // in motor scale
	const float &GetMotor_Angle(const unsigned char &idx) const; // in dregree
	const short &GetMotor_Torque(const unsigned char &idx) const;
	const bool &GetMotor_TorqueEnable(const unsigned char &idx) const;
	const float &GetMotor_PresentVelocity(const unsigned char &idx) const;
	const int &GetMotor_Max_Position_Limit(const unsigned char &idx) const;
	const int &GetMotor_Min_Position_Limit(const unsigned char &idx) const;
	const int &GetMotor_Max_Velocity_Limit(const unsigned char &idx) const;
	const int &GetMotor_Min_Velocity_Limit(const unsigned char &idx) const;
	const int &GetMotor_Max_Torque_Limit(const unsigned char &idx) const;
	const int &GetMotor_Min_Torque_Limit(const unsigned char &idx) const;
	/* Set Motor Data */
	void SetMotor_CenterScale(const unsigned char &idx, const short &motor_center_scale) const;
	void SetMotor_Angle(const unsigned char &idx, const float &angle) const;
	void SetMotor_Velocity(const unsigned char &idx, const int &velocity) const;
	void SetMotor_Velocity_withTime(const unsigned char &idx, const short &ms) const;
	void SetMotor_Torque(const unsigned char &idx, const short &torque) const;
	void SetMotor_TorqueEnable(const unsigned char &idx, const bool &enable) const;

private:
	////////////////////////////////////////////////////////////////////////////////
	///   Background   /////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////
	//Background is used for reading & writing data to motor
	thread *thread_BG;
	bool _is_deleted_thread_BG;
	bool _is_recovery_state;

	vector<dynamixel::PortHandler *> portHandler;
	dynamixel::PacketHandler *packetHandler;
	vector<dynamixel::GroupBulkRead *> groupBulkRead;
	vector<dynamixel::GroupBulkWrite *> groupBulkWrite;

	void BGON();
	void BGReadWrite();
	void WriteData() const;
	void ReadData() const;

private:
	vector<Motor *> Motor_Union;
	const int waiting_frequency;
};
