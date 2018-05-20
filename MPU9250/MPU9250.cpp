/*
 Name:		MPU9250.cpp
 Created:	4/1/2018 6:31:14 PM
 Author:	Ejup Hoxha
			UNIVERSITY OF PRISHTINA, KOSOVO
			FACULTY OF ELECTRICAL AND COMPUTER ENGINEERING
			COMPUTERIZED AUTOMATION AND ROBOTICS
			Pitch  - x axis
			Roll   - y axis
			Yaw    - z axis
*/

#include "MPU9250.h"
#include "math.h"
#include <Wire.h>


#pragma region GENERAL AND CONFIG REGISTERS
#define MPU_ADDRESS						104//0x68
#define MPU9250_CONFIG                  26//0x1A
#define MPU9250_GYRO_CONFIG             27//0x1B
#define MPU9250_ACCEL_CONFIG            28//0x1C
#define MPU9250_ACCEL_CONFIG2           29//0x1D
#define MPU9250_LP_ACCEL_ODR            30//0x1E
#define MPU9250_WOM_THR                 31//0x1F

#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38
#pragma endregion
#pragma region ACCELEROMETER_CONFIGURATION

#define MPU9250_ACCEL_XOUT_H            59//0x3B
#define MPU9250_ACCEL_XOUT_L            60//0x3C
#define MPU9250_ACCEL_YOUT_H            61//0x3D
#define MPU9250_ACCEL_YOUT_L            62//0x3E
#define MPU9250_ACCEL_ZOUT_H            63//0x3F
#define MPU9250_ACCEL_ZOUT_L            64//0x40

#define MPU9250_FULL_SCALE_2G           0 
#define MPU9250_FULL_SCALE_4G           8
#define MPU9250_FULL_SCALE_8G           16
#define MPU9250_FULL_SCALE_16G          24

#pragma endregion
#pragma region TEMPERATURE_REGISTERS
#define MPU9250_TEMP_OUT_H          65 //   0x41
#define MPU9250_TEMP_OUT_L          66 //   0x42
#pragma endregion
#pragma region GYRO_CONFIGURATION
#define MPU9250_GYRO_XOUT_H           67 // 0x43
#define MPU9250_GYRO_XOUT_L           68 // 0x44
#define MPU9250_GYRO_YOUT_H           69 // 0x45
#define MPU9250_GYRO_YOUT_L           70 // 0x46
#define MPU9250_GYRO_ZOUT_H           71 // 0x47
#define MPU9250_GYRO_ZOUT_L           72 // 0x48

#define MPU9250_GYRO_FULL_SCALE_250DPS  0  // 0
#define MPU9250_GYRO_FULL_SCALE_500DPS  8  // 8
#define MPU9250_GYRO_FULL_SCALE_1000DPS 16  // 16
#define MPU9250_GYRO_FULL_SCALE_2000DPS 24  // 24
#pragma endregion
#pragma region MAGNETOMETER_CONFIGURATION_REGISTERS
//Magnetometer register maps
#define MPU9250_MAG_ADDRESS             0x0C
#define MPU9250_MAG_WIA                 0x00
#define MPU9250_MAG_INFO                0x01
#define MPU9250_MAG_ST1                 0x02
#define MPU9250_MAG_XOUT_L              0x03
#define MPU9250_MAG_XOUT_H              0x04
#define MPU9250_MAG_YOUT_L              0x05
#define MPU9250_MAG_YOUT_H              0x06
#define MPU9250_MAG_ZOUT_L              0x07
#define MPU9250_MAG_ZOUT_H              0x08
#define MPU9250_MAG_ST2                 0x09
#define MPU9250_MAG_CNTL                0x0A
#define MPU9250_MAG_RSV                 0x0B
#define MPU9250_MAG_ASTC                0x0C
#define MPU9250_MAG_TS1                 0x0D
#define MPU9250_MAG_TS2                 0x0E
#define MPU9250_MAG_I2CDIS              0x0F
#define MPU9250_MAG_ASAX                0x10
#define MPU9250_MAG_ASAY                0x11
#define MPU9250_MAG_ASAZ                0x12
//Magnetometer register masks
#define MPU9250_WIA_MASK 0x48
#pragma endregion

MPU9250::MPU9250()
{
}

void MPU9250::readGyro(short address = MPU_ADDRESS)
{
	Wire.beginTransmission(address);
	Wire.write(MPU9250_GYRO_XOUT_H); //Start reading from gyro_xout_h address = 0x43
	Wire.endTransmission();
	Wire.requestFrom(address, 6);
	while (Wire.available() < 6);
	raw_gyro_x = Wire.read() << 8 | Wire.read();
	raw_gyro_y = Wire.read() << 8 | Wire.read();
	raw_gyro_z = Wire.read() << 8 | Wire.read();
}

void MPU9250::readTemp(short address = MPU_ADDRESS)
{
	Wire.beginTransmission(address);
	Wire.write(MPU9250_TEMP_OUT_H);
	Wire.endTransmission();
	Wire.requestFrom(address, 2);
	while (Wire.available() < 2);
	raw_temperature = Wire.read() << 8 | Wire.read();
}

void MPU9250::readAccelerometer(short address = MPU_ADDRESS)
{
	Wire.beginTransmission(address);
	Wire.write(MPU9250_ACCEL_XOUT_H); //Start reading from gyro_xout_h address = 0x43
	Wire.endTransmission();
	Wire.requestFrom((short)address, 6);
	while (Wire.available() < 6);
	raw_accel_x = Wire.read() << 8 | Wire.read();
	raw_accel_y = Wire.read() << 8 | Wire.read();
	raw_accel_z = Wire.read() << 8 | Wire.read();
}

void MPU9250::readMagnetometer(short address = MPU_ADDRESS)
{
	Wire.beginTransmission(address);
	Wire.write(MPU9250_MAG_XOUT_H);
	Wire.endTransmission();
	Wire.requestFrom(address, 6);
	while (Wire.available() < 6);
	raw_mag_x = Wire.read() << 8 | Wire.read();
	raw_mag_y = Wire.read() << 8 | Wire.read();
	raw_mag_z = Wire.read() << 8 | Wire.read();
}

void MPU9250::calibrateMPU(short address = MPU_ADDRESS)
{
	for (int i = 0; i < gyroCalibrationRate; i++)
	{
		MPU9250::readGyro(address);
		gyroCalibrationData[0] += raw_gyro_x;
		gyroCalibrationData[1] += raw_gyro_y;
		gyroCalibrationData[2] += raw_gyro_z;
	}
	for (int i = 0; i < 3; i++)
	{
		gyroCalibrationData[i] /= gyroCalibrationRate;
	}
	//MPU9250::accelerometerAngle(address);
	//gyroCalibrationData[3] = accel_ang_x;
	//gyroCalibrationData[4] = accel_ang_y;
	//if (gyroCalibrationData[4] > 10.0 || gyroCalibrationData[5] > 10.0 || gyroCalibrationData[4] < -10.0 || gyroCalibrationData[5] <-10.0)
	//{
	//	for (int i = 3; i < 5; i++)
	//	{
	//		gyroCalibrationData[i] = -99;
	//		Serial.println("Error Calibrating Accelerometer! Please position the SENSOR in a level surface or check the mounting!");
	//	}
	//}

}

void MPU9250::inicialization(short address = MPU_ADDRESS, short GYRO_CONFIG = MPU9250_GYRO_CONFIG, short FS_GYRO = MPU9250_GYRO_FULL_SCALE_500DPS, short ACCEL_CONFIG = MPU9250_ACCEL_CONFIG, short AFS_ACCEL = MPU9250_FULL_SCALE_16G)
{
	Wire.beginTransmission(address);
	Wire.write(0x6B);
	Wire.write(0x00);
	Wire.endTransmission();

	Wire.beginTransmission(address);
	Wire.write(GYRO_CONFIG); // Regjistri per konfigurim te gyroskopit FS_SEL;
	Wire.write(FS_GYRO); // Zgjedhet modi 500deg/sekond apo 65.5 LSB/(º/s)
	Wire.endTransmission();


	//Konfigurimi i Accelerometer
	Wire.beginTransmission(address);
	Wire.write(ACCEL_CONFIG); // Regjistri per konfigurim te accelerometrit AFS_SEL;
	Wire.write(AFS_ACCEL); // Zgjedhet modi (+/- 8g) 4096 LSB/gravitet
	Wire.endTransmission();

	//Konfigurimi i LOW PASS FILTER
	Wire.beginTransmission(address);
	Wire.write(0x1A); // Regjistri qe duhet konfiguruar per te caktuar frekuencen e LPF
	Wire.write(0x01); // 5Hz - 0x06; 0x01 -184Hz; 0x00 - 260Hz;
	Wire.endTransmission();

	Wire.beginTransmission(address);
	Wire.write(0x1A);
	Wire.endTransmission();      //Leximi i FS_GYRO_SEL, FS_ACCEL_SEL DHE LPF
	Wire.requestFrom(0x68, 3);
	while (Wire.available() < 3);
	LPF = Wire.read();
	FS_GYRO_SEL = Wire.read();
	FS_ACCEL_SEL = Wire.read();
}

void MPU9250::gyroAngle(short address = MPU_ADDRESS, float period)
{
	MPU9250::readGyro(address);

	raw_gyro_x -= gyroCalibrationData[0];
	raw_gyro_y -= gyroCalibrationData[1];
	raw_gyro_z -= gyroCalibrationData[2]; 
	float _calc;
	switch (FS_GYRO_SEL)
	{
	case (int)0x00:
		//131
		gyro_ang_x += raw_gyro_x * (period / 131);
		gyro_ang_y += raw_gyro_y * (period / 131);
		gyro_ang_z += raw_gyro_z * (period / 131);
		_calc = period / 131;
		break;
	case (int)0x08:
		//65.5
		gyro_ang_x += raw_gyro_x * (period / 65.5);
		gyro_ang_y += raw_gyro_y* (period / 65.5);
		gyro_ang_z += raw_gyro_z * (period / 65.5);
		_calc = period / 65.5;
		break;
	case (int)0x10:
		//32.8
		gyro_ang_x += raw_gyro_x * (period / 32.8);
		gyro_ang_y += raw_gyro_y * (period / 32.8);
		gyro_ang_z += raw_gyro_z * (period / 32.8);		
		_calc = period / 32.8;
		break;
	case (int)0x18:
		//16.4
		gyro_ang_x += raw_gyro_x * (period / 16.4);
		gyro_ang_y += raw_gyro_y * (period / 16.4);
		gyro_ang_z += raw_gyro_z * (period / 16.4);
		_calc = period / 16.4;
		break;
	default:
		//131
		gyro_ang_x += raw_gyro_x * (period / 131);
		gyro_ang_y += raw_gyro_y * (period / 131);
		gyro_ang_z += raw_gyro_z * (period / 131);
		_calc = period / 131;
		break;
	}
	gyro_ang_x += gyro_ang_y * sin(raw_gyro_z*(_calc*to_rad));
	gyro_ang_y -= gyro_ang_x * sin(raw_gyro_z*(_calc*to_rad));
}
void MPU9250::accelerometerAngle(short address = MPU_ADDRESS)
{
	MPU9250::readAccelerometer(address);
	float magnitude = sqrt(pow(raw_accel_x, 2) + pow(raw_accel_y, 2) + pow(raw_accel_z, 2));
	accel_ang_x = asin((float)raw_accel_y / (float)magnitude)*to_deg;
	accel_ang_y = asin((float)raw_accel_x / (float)magnitude)*(-to_deg);
}

void MPU9250::GetAngles(short address = MPU_ADDRESS, float period)
{
	gyroAngle(address, period);
	accelerometerAngle(address);

	if (first_flag_angles)
	{
		gyro_ang_x = gyro_ang_x * 0.96 + accel_ang_x * 0.04; //Filtrim
		gyro_ang_y = gyro_ang_y * 0.96 + accel_ang_y * 0.04;
	}
	else
	{
		gyro_ang_x = accel_ang_x;
		gyro_ang_y = accel_ang_y;
		first_flag_angles = true;
	}

	angle_x = angle_x * 0.9 + gyro_ang_x * 0.1;
	angle_y = angle_y * 0.9 + gyro_ang_y * 0.1;
	angle_z = gyro_ang_z;
}

MPU9250::~MPU9250()
{
	delete this;
}
