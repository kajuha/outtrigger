#ifndef _MAIN_HPP
#define _MAIN_HPP

#include <ros/ros.h>
#include <math.h>

#include <outtrigger/States.h>
#ifdef KIRO_MSG
#include <painting_robot_msg/OUTTRIGGER_State.h>
#else
#include <outtrigger/Infos.h>
#endif

#ifdef KIRO_MSG
extern painting_robot_msg::OUTTRIGGER_State outtriggerInfos;
#else
extern outtrigger::Infos outtriggerInfos;
#endif
extern outtrigger::States outtriggerStates;

extern int MOTOR_NUM;
extern std::vector<double> inv_motor_in_arr;
extern std::vector<double> inv_encoder_out_arr;
extern int override_homing;
extern int motor_info_hz;

extern double GEAR_RATIO;
extern double MOTOR_TICK;
extern double SCREW_LEAD;

#define MIN_TO_SEC 60.0

enum class CmdMode {
    REQ_VER=0, CMD_BRAKE, REQ_INPOS, CMD_VEL, REQ_IO, REQ_MAIN, CMD_TQ_OFF, CMD_RESET_POS, CMD_SET_POS, REQ_INIT_SET_OK, CMD_INIT_SET2
};

#endif
