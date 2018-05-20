#include "PID_Controller.h"

void PID_Controller::CalculatePID(float _setpoint, float _realvalue, float deltaTime)
{
	error = _setpoint - _realvalue;

	P = error * Kp;

	I = I + I * Ki*deltaTime;
	if (I > antiwindup_I)
		I = antiwindup_I;
	else if (I < -antiwindup_I)
		I = -antiwindup_I;

	D = Kd * (error - last_error) / deltaTime;
	last_error = error;

	pid_output = P + I + D;
	if (pid_output > pid_max_output)
		pid_output = pid_max_output;
	else if (pid_output < -pid_max_output)
		pid_output = -pid_max_output;
}

PID_Controller::~PID_Controller()
{
}
