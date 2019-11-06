#include "PID.h"
// use following for cout debug
//#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// initialize variables
	this-> Kp = Kp;
	this-> Ki = Ki;
	this-> Kd = Kd;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

void PID::UpdateError(double cte) {
	// update the error
	d_error = cte - p_error; 
	p_error = cte;
	i_error += cte;
	
	//std::cout << "d_error: " << d_error << " p_error: " << p_error << "i_error: " << i_error << endl;
}

double PID::TotalError() {
	//std::cout << "TotalError: " << Kp*p_error + Ki*i_error + Kd*d_error << endl;
	// calculate the total error	
	return Kp * p_error + Kd * d_error + Ki * i_error;
}

