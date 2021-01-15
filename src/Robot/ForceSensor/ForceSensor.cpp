#include "ForceSensor.h"

ForceSensor::ForceSensor(std::string port_name) : force_resolution_(32),		// default = 32
												  torque_resolution_(1640),		// default = 1640
												  data_zero_point_(8192),
												  sampling_time_(10),
												  correction_data_number_(50)
{
	/* ros publisher */
	pubber_ = node_handler_.advertise<std_msgs::Int16MultiArray>(port_name, 1);

	/* fixed initialized parameters */
	file_descriptor_ = -1;

	is_comport_connected_ = false;
	is_comport_setup_ = false;
	is_back_ground_deleted_ = false;

	/* 
	*  once object is constructed, it will automatically start to connection comport and engage the first normalization
	*  forcesensor will return some STRANGE DATA in some specific situation (for example, be put on the flat surface)
	*  thus you may need to check whether data is well normalized
	*  if not, use DataNormalization() in your main thread whenever you think you need to
	*/
	Connect(port_name);
}

ForceSensor::~ForceSensor()
{
	is_back_ground_deleted_ = true;
	delete back_ground_runner_;
}

/* Fixed */
void ForceSensor::SetComportAttribute(int file_descriptor)
{
	int state;
	struct termios term;

	state = tcgetattr(file_descriptor, &term);
	if (state < 0)
	{
		is_comport_setup_ = false;
		return;
	}

	bzero(&term, sizeof(term));

	term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
	term.c_iflag = IGNPAR;
	term.c_oflag = 0;
	term.c_lflag = 0; /*ICANON;*/

	term.c_cc[VINTR] = 0;  /* Ctrl-c */
	term.c_cc[VQUIT] = 0;  /* Ctrl-? */
	term.c_cc[VERASE] = 0; /* del */
	term.c_cc[VKILL] = 0;  /* @ */
	term.c_cc[VEOF] = 4;   /* Ctrl-d */
	term.c_cc[VTIME] = 0;
	term.c_cc[VMIN] = 0;
	term.c_cc[VSWTC] = 0;	 /* '?0' */
	term.c_cc[VSTART] = 0;	 /* Ctrl-q */
	term.c_cc[VSTOP] = 0;	 /* Ctrl-s */
	term.c_cc[VSUSP] = 0;	 /* Ctrl-z */
	term.c_cc[VEOL] = 0;	 /* '?0' */
	term.c_cc[VREPRINT] = 0; /* Ctrl-r */
	term.c_cc[VDISCARD] = 0; /* Ctrl-u */
	term.c_cc[VWERASE] = 0;	 /* Ctrl-w */
	term.c_cc[VLNEXT] = 0;	 /* Ctrl-v */
	term.c_cc[VEOL2] = 0;	 /* '?0' */

	state = tcsetattr(file_descriptor, TCSANOW, &term);

	is_comport_setup_ = true;
	return;
}

/* 
*  the port connection, attribute setting and connection state will be done here
*  is_comport_connected_ indicates whether connection is done successfully or not
*  is_comport_setup_ indicates whether attributes setting is done successfully or not 
*  as long as they are both true, back_ground_runner_ is started
*
*  @param port_name - used to tell the specific port name (e.g. /dev/ttyUSB0)
*/
void ForceSensor::Connect(std::string port_name)
{
	/* we need const char* format to open comport, thus the string format should be converted */
	const char *dev_name = port_name.c_str();
	file_descriptor_ = open(dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	is_comport_connected_ = (file_descriptor_ < 0) ? false : true;

	SetComportAttribute(file_descriptor_);

	/* successfully connected, then initialize the clock for sampling */
	if (is_comport_connected_ && is_comport_setup_)
	{
		back_ground_runner_ = new std::thread(&ForceSensor::BGReadDataFromForceSensor, this);

		/* avoid the weird data from initial */
		this_thread::sleep_for(chrono::milliseconds(100));
		DataNormalization();
	}		

	else if (!is_comport_connected_)
	{
		std::cout << "\t\t[ERROR] Forcesensor connection Error." << std::endl;
	}
	else if (!is_comport_setup_)
	{
		std::cout << "\t\t[ERROR] Forcesenser setup Error." << std::endl;
	}
}

void ForceSensor::BGReadDataFromForceSensor(void)
{
	while (!is_back_ground_deleted_)
	{
		this_thread::sleep_for(chrono::milliseconds(sampling_time_));

		/* read raw data from port */
		write(file_descriptor_, "R", 1);
		int data_state = read(file_descriptor_, data_buffer_, 27);

		sscanf(data_buffer_, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick_, &raw_fx_, &raw_fy_, &raw_fz_, &raw_mx_, &raw_my_, &raw_mz_);

		PublishDataToTopic();
	}
}

void ForceSensor::PublishDataToTopic()
{
	msg_.data.push_back(raw_fx_ - mean_fx_);
	msg_.data.push_back(raw_fy_ - mean_fy_);
	msg_.data.push_back(raw_fz_ - mean_fz_);
	msg_.data.push_back(raw_mx_ - mean_mx_);
	msg_.data.push_back(raw_my_ - mean_my_);
	msg_.data.push_back(raw_mz_ - mean_mz_);

	this->pubber_.publish(msg_);
	ros::spinOnce();

	msg_.data.clear();
}

void ForceSensor::DataNormalization(void)
{
	int correction_counter = 0;

	mean_fx_ = 0;
	mean_fy_ = 0;
	mean_fz_ = 0;
	mean_mx_ = 0;
	mean_my_ = 0;
	mean_mz_ = 0;

	while (correction_counter < correction_data_number_)
	{
		this_thread::sleep_for(chrono::milliseconds(sampling_time_));

		/* accumulate first N data to calculate the mean value of samples in order to normalize data */
		// NULL data will not be taken
		if (raw_fx_ == 0 && raw_fy_ == 0 && raw_fz_ == 0 && raw_mx_ == 0 && raw_my_ == 0 && raw_mz_ == 0)
			continue;

		mean_fx_ += raw_fx_;
		mean_fy_ += raw_fy_;
		mean_fz_ += raw_fz_;
		mean_mx_ += raw_mx_;
		mean_my_ += raw_my_;
		mean_mz_ += raw_mz_;
		correction_counter++;

		// accumulation done, calculate mean value
		if (correction_counter == correction_data_number_)
		{
			mean_fx_ /= correction_data_number_;
			mean_fy_ /= correction_data_number_;
			mean_fz_ /= correction_data_number_;
			mean_mx_ /= correction_data_number_;
			mean_my_ /= correction_data_number_;
			mean_mz_ /= correction_data_number_;
		}
	}
	std::cout << "\t[INFO] DataNormalization finished!" << std::endl;
}

const int ForceSensor::GetNormalizedFx(void) { return raw_fx_ - mean_fx_; }
const int ForceSensor::GetNormalizedFy(void) { return raw_fy_ - mean_fy_; }
const int ForceSensor::GetNormalizedFz(void) { return raw_fz_ - mean_fz_; }
const int ForceSensor::GetNormalizedMx(void) { return raw_mx_ - mean_mx_; }
const int ForceSensor::GetNormalizedMy(void) { return raw_my_ - mean_my_; }
const int ForceSensor::GetNormalizedMz(void) { return raw_mz_ - mean_mz_; }