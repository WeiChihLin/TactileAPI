#include "SlipControl.h"

using namespace SC;

SC::SlipControl::SlipControl()
{
	_Kfp = 0.01;
	_Kfd = 0.01;
	_PressRate = 0.5;
	_PressTh = 0.01;

	for (int i = 0; i < 7; i++)
	{
		_PreAvePress.push_back(0);
	}	

	ID[0] = 3;
	ID[1] = 4;
	ID[2] = 5;
	ID[3] = 7;
	ID[4] = 8;
	ID[5] = 10;
}

SC::SlipControl::~SlipControl()
{
}
void SC::SlipControl::update_Presure(double Pressure, int ID)
{
	_PreAvePress[ID] = Pressure;
}

void SC::SlipControl::SlipRule(double AvePress, double &Pressure_ref, int ID)
{
	double Pressure_ref_tmp, PreAvePress_tmp;
	PreAvePress_tmp = _PreAvePress[ID];
	double Press_rate = AvePress - PreAvePress_tmp;
	if ((Pressure_ref - AvePress) > _PressTh)
		Pressure_ref_tmp = Pressure_ref + _Kfd*Press_rate;

	if (Press_rate < _PressRate)
		Pressure_ref_tmp = Pressure_ref - _Kfd*Press_rate;

	Pressure_ref = Pressure_ref_tmp;
}

SC::PID::PID()
{
	_Kp = 100;
	_Ki = 0.001;
	_Kd = 0;

	Intergrator = 0;
}

SC::PID::~PID()
{
}

void SC::PID::Angle_PID(double& theta_ref, double AvePress, double Pressure_ref)
{
	double Press_rate = Pressure_ref - AvePress;
	double Theta_tmp;

	Intergrator = Intergrator + _Ki*Press_rate*0.005;		//5 msec
	Theta_tmp = Intergrator + theta_ref;

	theta_ref = Theta_tmp;
}