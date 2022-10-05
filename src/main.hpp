#ifndef _MAIN_HPP
#define _MAIN_HPP

#include <ros/ros.h>
#include <math.h>

extern int MOTOR_NUM;
extern std::vector<double> inv_motor_in_arr;
extern std::vector<double> inv_encoder_out_arr;

enum class CmdMode {
    REQ_VER=0, CMD_BRAKE, REQ_INPOS, CMD_VEL, REQ_IO, REQ_MAIN, CMD_TQ_OFF, CMD_RESET_POS, CMD_SET_POS, REQ_INIT_SET_OK, CMD_INIT_SET2
};

#endif
