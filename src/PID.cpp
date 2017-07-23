#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this -> Kp = Kp;
	this -> Ki = Ki;
	this -> Kd = Kd;

	p_error = 0;
	d_error = 0;
	i_error = 0;

	twiddle_it = 0;
	calc_err = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	return Kp * p_error + Kd * d_error + Ki * i_error;
}

double PID::TwiddleIteration() {
	twiddle_it +=1;
	return twiddle_it;
}

double PID::CalcError( double cte) {
	calc_err += (cte*cte);
	return calc_err;
}


