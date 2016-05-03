#pragma once
#include <math.h>
#include <vector>

namespace SC{ // mean Slip Control
	class SlipControl{
	public:
		SlipControl();
		~SlipControl();

		double ID[6];

		void SlipRule(double, double&, int );
		void update_Presure(double, int);
	
	private:			
		std::vector<double> _PreAvePress;		//last time average presure for one array
		double _PressTh;			//the threshold for error between extral pressure and ref pressure
		double _PressRate;			//the threshold for pressure rate

		double _Kfp, _Kfd;
	};

	class PID{
	public:
		PID();
		~PID();

		void Angle_PID(double&, double, double);

		double Intergrator;

	private:
		double _Kp, _Ki, _Kd;
		
	};
}