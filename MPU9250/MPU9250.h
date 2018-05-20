/*
 Name:		MPU9250.h
 Created:	4/1/2018 6:31:14 PM
 Author:	Ejup Hoxha
			UNIVERSITY OF PRISHTINA, KOSOVO
			FACULTY OF ELECTRICAL AND COMPUTER ENGINEERING
			COMPUTERIZED AUTOMATION AND ROBOTICS
 Editor:	http://www.visualmicro.com
*/

#ifndef _MPU9250_h

#define _MPU9250_h

#include <inttypes.h>
#include <Wire.h>
#include "Arduino.h"



class MPU9250
{

public:

	MPU9250();

	float x_roll, y_pitch, z_yaw;
	float to_rad = PI / 180.0;
	float to_deg = 180.0 / PI;
	bool first_flag_angles = false;

	//Inicialization 
	void inicialization(short address, short GYRO_CONFIG, short FS_GYRO, short ACCEL_CONFIG, short AFS_ACCEL);
	int LPF, FS_GYRO_SEL, FS_ACCEL_SEL;

	//Gyroscope
	long raw_gyro_x, raw_gyro_y, raw_gyro_z;
	void readGyro(short address);

	//Temperature
	long raw_temperature, temperature;
	void readTemp(short address);

	//Accelerometer
	long raw_accel_x, raw_accel_y, raw_accel_z;
	void readAccelerometer(short address);

	//Magnetometer
	long raw_mag_x, raw_mag_y, raw_mag_z;
	void readMagnetometer(short address);

	//Gyro Calibration
	int gyroCalibrationRate = 100;
	float gyroCalibrationData[5];
	void calibrateMPU(short address);

	//Calculate Gyro Angles:
	float gyro_ang_x, gyro_ang_y, gyro_ang_z;
	void gyroAngle(short address,float period);

	//Calculate Accelerometer Angles:
	float accel_ang_x, accel_ang_y, accel_ang_z;
	void accelerometerAngle(short address);

	//Calculate, combine and filter final angles.
	float angle_x, angle_y, angle_z;
	void GetAngles(short address, float period);
	

	~MPU9250();
};

#endif

