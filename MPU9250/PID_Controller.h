#pragma once
class PID_Controller
{
public:
	float error;
	float last_error;
	float setpoint_weight;
	float P;
	float I;
	float D;
	float Kp;
	float Ki;
	float Kd;
	float antiwindup_I;
	float pid_max_output;
	float pid_output;

	void CalculatePID(float _setpoint, float _realvalue, float deltaTime);
	~PID_Controller();
};

