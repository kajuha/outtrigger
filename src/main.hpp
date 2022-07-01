#ifndef _MAIN_HPP
#define _MAIN_HPP

#include <ros/ros.h>
#include <math.h>

typedef enum {UNSET=0, CALLBACK, FIRSTRUN} TYPE_KASET;

// recipelab launchbox AGV 사양
// unit m, g, s
// unit rad
// gearhead = 51:1
// Ç®ž® °šŒÓ±â = 1:1

typedef enum {LEFT_FRONT=0, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK} WHEEL_ORDER;
//typedef enum {_X=0, _Y, _THETA} ODOMPOS_ORDER;

extern int WHEEL_NUM;
extern int MOTOR_NUM;
extern double WHEEL_RADIUS;
extern double WHEEL_LENGTH;
extern double WHEEL_WIDTH;
extern double WHEEL_XY;
extern double GEAR_RATIO;
extern double MOTOR_TICK;
extern double WHEEL_MAX_RPM;
extern double MOTOR_MAX_RPM;
extern double MOTOR_CALIBRATION;
extern double SEC_TO_MIN;
extern double RAD_TO_REV;
extern double IK_STEP_TIME;
extern double DEG_TO_RAD;
extern double RAD_TO_DEG;
extern double TICK_TO_REV;
extern double REV_TO_TICK;
extern double TICK_TO_RAD;
extern double REV_TO_RAD;
extern double RPM_TO_RadPSec;
extern std::vector<double> inv_motor_in_arr;
extern std::vector<double> inv_encoder_out_arr;

extern int encoder_error;

#define ORIENTATION 4 // x, y, z, w
extern double _imu_orientation[ORIENTATION];

#endif
