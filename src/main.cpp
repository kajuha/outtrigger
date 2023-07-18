#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <std_msgs/Empty.h>

#ifdef KIRO_MSG
#include <painting_robot_msg/OUTTRIGGER_Command.h>
#include <painting_robot_msg/OUTTRIGGER_State.h>
#else
#include "outtrigger/Commands.h"
#include "outtrigger/Infos.h"
#endif

#include "outtrigger/PidRequest.h"
#include "outtrigger/PidCommand.h"
#include "outtrigger/PidVelocity.h"
#include "outtrigger/PidPosition.h"
#include "outtrigger/States.h"

#include "global.hpp"
#include "main.hpp"
#include "com.hpp"

#include <ros/ros.h>

#include "outtrigger/MotorInfo.h"

ros::Time ts_now;
void getMotorInfo(int rate, ros::Publisher pub_motor_info) {
  ros::Rate r(rate);
  outtrigger::MotorInfo msg;

  while (ros::ok())
  {
    ts_now = ros::Time::now();

    msg.fl.tx = Com.kaTxTsMain[OUTTRIGGER_FRONT_LEFT].toSec();
    msg.fl.tx_diff = Com.kaTxTsMain[OUTTRIGGER_FRONT_LEFT].toSec() - Com.kaTxTsMainOld[OUTTRIGGER_FRONT_LEFT].toSec();
    msg.fl.rx = Com.kaRxTsMain[OUTTRIGGER_FRONT_LEFT].toSec();
    msg.fl.rx_diff = Com.kaRxTsMain[OUTTRIGGER_FRONT_LEFT].toSec() - Com.kaRxTsMainOld[OUTTRIGGER_FRONT_LEFT].toSec();
    msg.fl.txrx_diff = msg.fl.rx - msg.fl.tx;

    msg.fr.tx = Com.kaTxTsMain[OUTTRIGGER_FRONT_RIGHT].toSec();
    msg.fr.tx_diff = Com.kaTxTsMain[OUTTRIGGER_FRONT_RIGHT].toSec() - Com.kaTxTsMainOld[OUTTRIGGER_FRONT_RIGHT].toSec();
    msg.fr.rx = Com.kaRxTsMain[OUTTRIGGER_FRONT_RIGHT].toSec();
    msg.fr.rx_diff = Com.kaRxTsMain[OUTTRIGGER_FRONT_RIGHT].toSec() - Com.kaRxTsMainOld[OUTTRIGGER_FRONT_RIGHT].toSec();
    msg.fr.txrx_diff = msg.fr.rx - msg.fr.tx;

    msg.bl.tx = Com.kaTxTsMain[OUTTRIGGER_BACK_LEFT].toSec();
    msg.bl.tx_diff = Com.kaTxTsMain[OUTTRIGGER_BACK_LEFT].toSec() - Com.kaTxTsMainOld[OUTTRIGGER_BACK_LEFT].toSec();
    msg.bl.rx = Com.kaRxTsMain[OUTTRIGGER_BACK_LEFT].toSec();
    msg.bl.rx_diff = Com.kaRxTsMain[OUTTRIGGER_BACK_LEFT].toSec() - Com.kaRxTsMainOld[OUTTRIGGER_BACK_LEFT].toSec();
    msg.bl.txrx_diff = msg.bl.rx - msg.bl.tx;

    msg.br.tx = Com.kaTxTsMain[OUTTRIGGER_BACK_RIGHT].toSec();
    msg.br.tx_diff = Com.kaTxTsMain[OUTTRIGGER_BACK_RIGHT].toSec() - Com.kaTxTsMainOld[OUTTRIGGER_BACK_RIGHT].toSec();
    msg.br.rx = Com.kaRxTsMain[OUTTRIGGER_BACK_RIGHT].toSec();
    msg.br.rx_diff = Com.kaRxTsMain[OUTTRIGGER_BACK_RIGHT].toSec() - Com.kaRxTsMainOld[OUTTRIGGER_BACK_RIGHT].toSec();
    msg.br.txrx_diff = msg.br.rx - msg.br.tx;

    msg.header.stamp = ros::Time::now();
    pub_motor_info.publish(msg);

    r.sleep();
  }
}

int MOTOR_NUM;
double GEAR_RATIO;
double MOTOR_TICK;
double SCREW_LEAD;
double TIMEOUT_SEC_MAIN;
double TIMEOUT_SEC_HOMING;
double MIN_MM;
double MAX_MM;
double MIN_MM_PER_SEC;
double MAX_MM_PER_SEC;
std::vector<double> inv_motor_in_arr;
std::vector<double> inv_encoder_out_arr;
int override_homing;
int motor_info_hz;

Communication Com;

outtrigger::States outtriggerStates;
#if KIRO_MSG
painting_robot_msg::OUTTRIGGER_State outtriggerInfos;
#else
outtrigger::Infos outtriggerInfos;
#endif

#include <queue>

#define LOCKFREE_QUEUE_SIZE 100
boost::lockfree::queue<ComData> qarr(LOCKFREE_QUEUE_SIZE);

void pidRequestCallback(const outtrigger::PidRequest& pidRequest) {
    static ComData comData;

    switch (pidRequest.pid) {
        case (int)CmdMode::REQ_VER:
            // VERSION 확인
            comData.type = MD_REQ_PID;
            comData.id = pidRequest.id;
            comData.nArray[0] = PID_VER;
            qarr.push(comData);
            break;
        case (int)CmdMode::REQ_INPOS:
            // PID_IN_POSITION_OK 확인
            comData.type = MD_CMD_PID;
            comData.id = pidRequest.id;
            comData.pid = PID_REQ_PID_DATA;
            comData.nArray[0] = PID_IN_POSITION_OK;
            qarr.push(comData);
            break;
        case (int)CmdMode::REQ_IO:
            // CTRL 입력확인용
            comData.type = MD_REQ_PID;
            comData.id = pidRequest.id;
            comData.nArray[0] = PID_IO_MONITOR;
            qarr.push(comData);
            break;
        case (int)CmdMode::REQ_MAIN:
            // 상태피드백 확인용
            comData.type = MD_REQ_PID;
            comData.id = pidRequest.id;
            comData.nArray[0] = PID_MAIN_DATA;
            qarr.push(comData);
            break;
        case (int)CmdMode::REQ_INIT_SET_OK:
            // 상태피드백 확인용
            comData.type = MD_REQ_PID;
            comData.id = pidRequest.id;
            comData.nArray[0] = PID_INIT_SET_OK;
            qarr.push(comData);
            break;
        default:
            printf("Unknown pidRequest.pid\n");
            break;
    }
}

void pidCommandCallback(const outtrigger::PidCommand& pidCommand) {
    static ComData comData;

    switch (pidCommand.pid) {
        case (int)CmdMode::CMD_BRAKE:
            // 전기적 브레이크 정지(전기적 브레이크 정지)
            comData.type = MD_CMD_PID;
            comData.id = pidCommand.id;
            comData.pid = PID_BRAKE;
            comData.nArray[0] = 0;
            qarr.push(comData);
            break;
        case (int)CmdMode::CMD_TQ_OFF:
            // 토크 정지(자연정지)
            comData.type = MD_CMD_PID;
            comData.id = pidCommand.id;
            comData.pid = PID_TQ_OFF;
            comData.nArray[0] = 0;
            qarr.push(comData);
            break;
        case (int)CmdMode::CMD_RESET_POS:
            // 모터 위치 리셋
            comData.type = MD_SET_POS;
            comData.id = pidCommand.id;
            comData.position = 0;
            qarr.push(comData);
            break;
        case (int)CmdMode::CMD_INIT_SET2:
            // 모터 위치 리셋
            comData.type = MD_CMD_INIT;
            comData.id = pidCommand.id;
            comData.pid = PID_COMMAND;
            comData.nArray[0] = PID_CMD_INIT_SET2;
            qarr.push(comData);
            break;
        default:
            printf("Unknown pidCommand.pid\n");
            break;
    }
}

void pidVelocityCallback(const outtrigger::PidVelocity& pidVelocity) {
    static ComData comData;

    // PID_PNT_VEL_CMD 확인
    comData.type = MD_SEND_RPM;
    comData.id = pidVelocity.id;
    comData.rpm = pidVelocity.rpm;
    qarr.push(comData);
}

void pidPositionCallback(const outtrigger::PidPosition& pidPosition) {
    static ComData comData;

    // 모터 위치 이동
    comData.type = MD_CMD_PID;
    comData.id = pidPosition.id;
    #if MDROBOT_PNT_ENABLE
    comData.pid = PID_PNT_POS_VEL_CMD;
    #else
    comData.pid = PID_POSI_VEL_CMD;
    #endif
    comData.position = pidPosition.position;
    comData.rpm = pidPosition.rpm;
    qarr.push(comData);
}

#ifdef KIRO_MSG
#else
void commandCallback(const outtrigger::Command& command) {
    static ComData comData;

    // 모터 위치 이동
    comData.type = MD_CMD_PID;
    comData.id = command.command;
    comData.pid = PID_PNT_POS_VEL_CMD;

    static double mm_in;
    mm_in = command.mm;

    if (MIN_MM > mm_in || mm_in > MAX_MM) return;
    static double screw_rev;
    screw_rev = mm_in / SCREW_LEAD;
    static double gear_rev;
    gear_rev = screw_rev * GEAR_RATIO;
    static double encoder;
    encoder = gear_rev * MOTOR_TICK;
    comData.position = (int)encoder;
    static double mm_sec_in;
    mm_sec_in = command.mm_per_sec;   // 10 mm/s = 1200 rpm

    if (MIN_MM_PER_SEC > mm_sec_in || mm_sec_in > MAX_MM_PER_SEC) return;
    static double mm_min;
    mm_min = mm_sec_in * MIN_TO_SEC;
    static double screw_rev_min;
    screw_rev_min = mm_min / SCREW_LEAD;
    static double gear_rev_min;
    gear_rev_min = screw_rev_min * GEAR_RATIO;
    comData.rpm = gear_rev_min;
    #if 0
    printf("[in ] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_in, mm_sec_in, comData.position, comData.rpm);
    #endif
    qarr.push(comData);

    static double mm_out;
    mm_out = 0.0;
    mm_out = (double)Com.kaPosition[ID_OFFSET-command.command] / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
    static double mm_sec_out;
    mm_sec_out = 0.0;
    mm_sec_out = (double)Com.kaSpeed[ID_OFFSET-command.command] / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
    #if 0
    printf("[out] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_out, mm_sec_out, Com.kaPosition[ID_OFFSET-command.command], Com.kaSpeed[ID_OFFSET-command.command]);
    #endif
}
#endif

#ifdef KIRO_MSG
void commandsCallback(const painting_robot_msg::OUTTRIGGER_Command& commands) {
    static ComData comData;

    static std::vector<painting_robot_msg::OUTTRIGGER_Command_Info> command = std::vector<painting_robot_msg::OUTTRIGGER_Command_Info>(MOTOR_NUM);
    static std::vector<painting_robot_msg::OUTTRIGGER_State_Info> info = std::vector<painting_robot_msg::OUTTRIGGER_State_Info>(MOTOR_NUM);

    static int motor_id;
    static double mm_in;
    static double screw_rev;
    static double gear_rev;
    static double encoder;
    static double mm_sec_in;
    static double mm_min;
    static double screw_rev_min;
    static double gear_rev_min;
    static double time_diff;

    command[OUTTRIGGER_FRONT_LEFT] = commands.frontLeft;
    command[OUTTRIGGER_FRONT_RIGHT] = commands.frontRight;
    command[OUTTRIGGER_BACK_LEFT] = commands.backLeft;
    command[OUTTRIGGER_BACK_RIGHT] = commands.backRight;

    info[OUTTRIGGER_FRONT_LEFT] = outtriggerInfos.frontLeft;
    info[OUTTRIGGER_FRONT_RIGHT] = outtriggerInfos.frontRight;
    info[OUTTRIGGER_BACK_LEFT] = outtriggerInfos.backLeft;
    info[OUTTRIGGER_BACK_RIGHT] = outtriggerInfos.backRight;

    for (int i=0; i<command.size(); i++) {
        motor_id = ID_OFFSET + i;
        #if 0
        printf("motor_id : %d\n", motor_id);
        #endif

#define ESTOP -2
#define STOP -1
#define NO_ACTION 0
#define VELOCITY 1
#define POSITION 2
#define HOMING 3
#define ERROR_CLEAR 4
        switch (command[i].Command) {
            case ESTOP:
                comData.type = MD_CMD_PID;
                comData.id = motor_id;
                comData.pid = PID_BRAKE;
                comData.nArray[0] = 0;
                qarr.push(comData);
                break;
            case STOP:
                comData.type = MD_CMD_PID;
                comData.id = motor_id;
                comData.pid = PID_TQ_OFF;
                comData.nArray[0] = 0;
                qarr.push(comData);
                break;
            case NO_ACTION:
                break;
            case VELOCITY:
                if (command[i].Position == 12345.67890 && command[i].Velocity < 0.0) {
                    printf("manual velocity[%d] control : %lf rpm\n", i, command[i].Velocity);
                    comData.type = MD_SEND_RPM;
                    comData.id = motor_id;
                    comData.rpm = (int)command[i].Velocity;
                    qarr.push(comData);

                    break;
                }
                
                time_diff = ros::Time::now().toSec() - info[i].Header.stamp.toSec();
                if (time_diff > TIMEOUT_SEC_HOMING) {
                    printf("velocity[%d] time invalid : timd_diff : %lf\n", i, time_diff);
                    break;
                }

                if (info[i].Homming) {
                    comData.type = MD_CMD_PID;
                    comData.id = motor_id;
                    comData.pid = PID_PNT_POS_VEL_CMD;

                    mm_sec_in = command[i].Velocity;   // 10 mm/s = 1200 rpm
                    if (mm_sec_in < 0.0) { 
                        mm_in = MIN_MM;
                        mm_sec_in *= -1.0;
                    } else {
                        mm_in = MAX_MM;
                        mm_sec_in *= 1.0;
                    }

                    screw_rev = mm_in / SCREW_LEAD;
                    gear_rev = screw_rev * GEAR_RATIO;
                    encoder = gear_rev * MOTOR_TICK;
                    comData.position = (int)encoder;

                    if (MIN_MM_PER_SEC > mm_sec_in || mm_sec_in > MAX_MM_PER_SEC) break;
                    mm_min = mm_sec_in * MIN_TO_SEC;
                    screw_rev_min = mm_min / SCREW_LEAD;
                    gear_rev_min = screw_rev_min * GEAR_RATIO;
                    comData.rpm = gear_rev_min;
                    #if 0
                    printf("[in ] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_in, mm_sec_in, comData.position, comData.rpm);
                    #endif
                    qarr.push(comData);
                } else {
                    printf("velocity command[%d] fail : no homing\n", i);
                }

                break;
            case POSITION:
                time_diff = ros::Time::now().toSec() - info[i].Header.stamp.toSec();
                if (time_diff > TIMEOUT_SEC_HOMING) {
                    printf("position[%d] time invalid : timd_diff : %lf\n", i, time_diff);
                    break;
                }

                if (info[i].Homming) {
                    comData.type = MD_CMD_PID;
                    comData.id = motor_id;
                    comData.pid = PID_PNT_POS_VEL_CMD;

                    mm_in = command[i].Position;

                    if (MIN_MM > mm_in || mm_in > MAX_MM) break;
                    screw_rev = mm_in / SCREW_LEAD;
                    gear_rev = screw_rev * GEAR_RATIO;
                    encoder = gear_rev * MOTOR_TICK;
                    comData.position = (int)encoder;
                    mm_sec_in = command[i].Velocity;   // 10 mm/s = 1200 rpm

                    if (MIN_MM_PER_SEC > mm_sec_in || mm_sec_in > MAX_MM_PER_SEC) break;
                    mm_min = mm_sec_in * MIN_TO_SEC;
                    screw_rev_min = mm_min / SCREW_LEAD;
                    gear_rev_min = screw_rev_min * GEAR_RATIO;
                    comData.rpm = gear_rev_min;
                    #if 0
                    printf("[in ] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_in, mm_sec_in, comData.position, comData.rpm);
                    #endif
                    qarr.push(comData);
                } else {
                    printf("position command[%d] fail : no homing\n", i);
                }

                break;
            case HOMING:
                comData.type = MD_CMD_INIT;
                comData.id = motor_id;
                comData.pid = PID_COMMAND;
                comData.nArray[0] = PID_CMD_INIT_SET2;
                qarr.push(comData);

                switch (i) {
                    case OUTTRIGGER_FRONT_LEFT:
                        outtriggerInfos.frontLeft.State = 0;
                        break;
                    case OUTTRIGGER_FRONT_RIGHT:
                        outtriggerInfos.frontRight.State = 0;
                        break;
                    case OUTTRIGGER_BACK_LEFT:
                        outtriggerInfos.backLeft.State = 0;
                        break;
                    case OUTTRIGGER_BACK_RIGHT:
                        outtriggerInfos.backRight.State = 0;
                        break;
                    default:
                        printf("unknown homing id: %d\n", i);
                        break;
                }

                break;
            case ERROR_CLEAR:
                comData.type = MD_CMD_INIT;
                comData.id = motor_id;
                comData.pid = PID_COMMAND;
                comData.nArray[0] = PID_CMD_ALARM_RESET;
                qarr.push(comData);

                switch (i) {
                    case OUTTRIGGER_FRONT_LEFT:
                        outtriggerInfos.frontLeft.State = 0;
                        break;
                    case OUTTRIGGER_FRONT_RIGHT:
                        outtriggerInfos.frontRight.State = 0;
                        break;
                    case OUTTRIGGER_BACK_LEFT:
                        outtriggerInfos.backLeft.State = 0;
                        break;
                    case OUTTRIGGER_BACK_RIGHT:
                        outtriggerInfos.backRight.State = 0;
                        break;
                    default:
                        printf("unknown error_clear id: %d\n", i);
                        break;
                }

                break;
            default:
                printf("unknown commands.Command : %d\n", command[i].Command);
                break;
        }
    }
}
#else
void commandsCallback(const outtrigger::Commands& commands) {
    static ComData comData;

    static std::vector<outtrigger::Command> command = std::vector<outtrigger::Command>(MOTOR_NUM);
    static std::vector<outtrigger::Info> info = std::vector<outtrigger::Info>(MOTOR_NUM);

    static int motor_id;
    static double mm_in;
    static double screw_rev;
    static double gear_rev;
    static double encoder;
    static double mm_sec_in;
    static double mm_min;
    static double screw_rev_min;
    static double gear_rev_min;
    static double time_diff;

    command[OUTTRIGGER_FRONT_LEFT] = commands.frontLeft;
    command[OUTTRIGGER_FRONT_RIGHT] = commands.frontRight;
    command[OUTTRIGGER_BACK_LEFT] = commands.backLeft;
    command[OUTTRIGGER_BACK_RIGHT] = commands.backRight;

    info[OUTTRIGGER_FRONT_LEFT] = outtriggerInfos.frontLeft;
    info[OUTTRIGGER_FRONT_RIGHT] = outtriggerInfos.frontRight;
    info[OUTTRIGGER_BACK_LEFT] = outtriggerInfos.backLeft;
    info[OUTTRIGGER_BACK_RIGHT] = outtriggerInfos.backRight;

    for (int i=0; i<command.size(); i++) {
        motor_id = ID_OFFSET + i;
        #if 0
        printf("motor_id : %d\n", motor_id);
        #endif

#define ESTOP -2
#define STOP -1
#define NO_ACTION 0
#define VELOCITY 1
#define POSITION 2
#define HOMING 3
#define ERROR_CLEAR 4
        switch (command[i].command) {
            case ESTOP:
                comData.type = MD_CMD_PID;
                comData.id = motor_id;
                comData.pid = PID_BRAKE;
                comData.nArray[0] = 0;
                qarr.push(comData);
                break;
            case STOP:
                comData.type = MD_CMD_PID;
                comData.id = motor_id;
                comData.pid = PID_TQ_OFF;
                comData.nArray[0] = 0;
                qarr.push(comData);
                break;
            case NO_ACTION:
                break;
            case VELOCITY:
                if (command[i].mm == 12345.67890 && command[i].mm_per_sec < 0.0) {
                    printf("manual velocity[%d] control : %lf rpm\n", i, command[i].mm_per_sec);
                    comData.type = MD_SEND_RPM;
                    comData.id = motor_id;
                    comData.rpm = (int)command[i].mm_per_sec;
                    qarr.push(comData);

                    break;
                }
                
                time_diff = ros::Time::now().toSec() - info[i].header.stamp.toSec();
                if (time_diff > TIMEOUT_SEC_HOMING) {
                    printf("velocity[%d] time invalid : timd_diff : %lf\n", i, time_diff);
                    break;
                }

                if (info[i].homing) {
                    comData.type = MD_CMD_PID;
                    comData.id = motor_id;
                    #if MDROBOT_PNT_ENABLE
                    comData.pid = PID_PNT_POS_VEL_CMD;
                    #else
                    comData.pid = PID_POSI_VEL_CMD;
                    #endif

                    mm_sec_in = command[i].mm_per_sec;   // 10 mm/s = 1200 rpm
                    if (mm_sec_in < 0.0) { 
                        mm_in = MIN_MM;
                        mm_sec_in *= -1.0;
                    } else {
                        mm_in = MAX_MM;
                        mm_sec_in *= 1.0;
                    }

                    screw_rev = mm_in / SCREW_LEAD;
                    gear_rev = screw_rev * GEAR_RATIO;
                    encoder = gear_rev * MOTOR_TICK;
                    comData.position = (int)encoder;

                    if (MIN_MM_PER_SEC > mm_sec_in || mm_sec_in > MAX_MM_PER_SEC) break;
                    mm_min = mm_sec_in * MIN_TO_SEC;
                    screw_rev_min = mm_min / SCREW_LEAD;
                    gear_rev_min = screw_rev_min * GEAR_RATIO;
                    comData.rpm = gear_rev_min;
                    #if 0
                    printf("[in ] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_in, mm_sec_in, comData.position, comData.rpm);
                    #endif
                    qarr.push(comData);
                } else {
                    printf("velocity command[%d] fail : no homing\n", i);
                }

                break;
            case POSITION:
                time_diff = ros::Time::now().toSec() - info[i].header.stamp.toSec();
                if (time_diff > TIMEOUT_SEC_HOMING) {
                    printf("position[%d] time invalid : timd_diff : %lf\n", i, time_diff);
                    break;
                }

                if (info[i].homing) {
                    comData.type = MD_CMD_PID;
                    comData.id = motor_id;
                    #if MDROBOT_PNT_ENABLE
                    comData.pid = PID_PNT_POS_VEL_CMD;
                    #else
                    comData.pid = PID_POSI_VEL_CMD;
                    #endif

                    mm_in = command[i].mm;

                    if (MIN_MM > mm_in || mm_in > MAX_MM) break;
                    screw_rev = mm_in / SCREW_LEAD;
                    gear_rev = screw_rev * GEAR_RATIO;
                    encoder = gear_rev * MOTOR_TICK;
                    comData.position = (int)encoder;
                    mm_sec_in = command[i].mm_per_sec;   // 10 mm/s = 1200 rpm

                    if (MIN_MM_PER_SEC > mm_sec_in || mm_sec_in > MAX_MM_PER_SEC) break;
                    mm_min = mm_sec_in * MIN_TO_SEC;
                    screw_rev_min = mm_min / SCREW_LEAD;
                    gear_rev_min = screw_rev_min * GEAR_RATIO;
                    comData.rpm = gear_rev_min;
                    #if 0
                    printf("[in ] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_in, mm_sec_in, comData.position, comData.rpm);
                    #endif
                    qarr.push(comData);
                } else {
                    printf("position command[%d] fail : no homing\n", i);
                }

                break;
            case HOMING:
                if (!override_homing) {
	                comData.type = MD_CMD_INIT;
	                comData.id = motor_id;
	                comData.pid = PID_COMMAND;
	                comData.nArray[0] = PID_CMD_INIT_SET2;
	                qarr.push(comData);

	                switch (i) {
	                    case OUTTRIGGER_FRONT_LEFT:
	                        outtriggerInfos.frontLeft.state = 0;
	                        break;
	                    case OUTTRIGGER_FRONT_RIGHT:
	                        outtriggerInfos.frontRight.state = 0;
	                        break;
	                    case OUTTRIGGER_BACK_LEFT:
	                        outtriggerInfos.backLeft.state = 0;
	                        break;
	                    case OUTTRIGGER_BACK_RIGHT:
	                        outtriggerInfos.backRight.state = 0;
	                        break;
	                    default:
	                        printf("unknown homing id: %d\n", i);
	                        break;
                    }
                } else {
                    printf("override[%d] homing mode!!!\n", i);
                }

                break;
            case ERROR_CLEAR:
                comData.type = MD_CMD_INIT;
                comData.id = motor_id;
                comData.pid = PID_COMMAND;
                comData.nArray[0] = PID_CMD_ALARM_RESET;
                qarr.push(comData);

                switch (i) {
                    case OUTTRIGGER_FRONT_LEFT:
                        outtriggerInfos.frontLeft.state = 0;
                        break;
                    case OUTTRIGGER_FRONT_RIGHT:
                        outtriggerInfos.frontRight.state = 0;
                        break;
                    case OUTTRIGGER_BACK_LEFT:
                        outtriggerInfos.backLeft.state = 0;
                        break;
                    case OUTTRIGGER_BACK_RIGHT:
                        outtriggerInfos.backRight.state = 0;
                        break;
                    default:
                        printf("unknown error_clear id: %d\n", i);
                        break;
                }

                break;
            default:
                printf("unknown commands.command : %d\n", command[i].command);
                break;
        }
    }
}
#endif

void checkOuttrigger() {
    ros::Time ts_now;
    double timeout;

    #if 0
    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        timeout = ts_now.toSec() - Com.kaRxTsMain[i].toSec();
        if (timeout > TIMEOUT_SEC_MAIN) {
            printf("invalid main data: #%d, timeout: %lf, SET_TIMEOUT: %lf\n", i, timeout, TIMEOUT_SEC_MAIN);
        }
    }
    #endif

    #if 0
    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        timeout = ts_now.toSec() - Com.kaRxTsHoming[i].toSec();
        if (timeout > TIMEOUT_SEC_HOMING) {
            printf("invalid homing data: #%d, timeout: %lf, SET_TIMEOUT: %lf\n", i, timeout, TIMEOUT_SEC_HOMING);
        }
    }
    #endif
}

void sendMsgMd1k(int* send_rate) {
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

    ros::Rate loop_rate(*send_rate);

    int nArray[5] = {0, };
    ComData comData;

    double ts_now, ts_begin, ts_diff;
    ts_begin = ts_now = ros::Time::now().toSec();

    while (ros::ok())
    {
        ts_now = ros::Time::now().toSec();

        if (qarr.pop(comData)) {
            #if 0
            ts_diff = ts_now - ts_begin;
            ts_begin = ts_now;
            ROS_INFO("[ts_diff]:%10.3lf(ms), %10.3lf(hz)", ts_diff*1000.0, 1.00/ts_diff);
            #endif

            if (Com.kaCom == "rs485") {
                if (comData.type == MD_SEND_RPM) {
                    PutMdData(PID_PNT_VEL_CMD, Com.nRMID, comData);
                } else if (comData.type == MD_SET_POS) {
                    PutMdData(PID_POSI_SET, Com.nRMID, comData);
                } else if (comData.type == MD_REQ_PID) {
                    PutMdData(PID_REQ_PID_DATA, Com.nRMID, comData);
                } else if (comData.type == MD_OFF) {
                    break;
                } else if (comData.type == MD_CMD_PID) {
                    PutMdData(comData.pid, Com.nRMID, comData);
                } else if (comData.type == MD_CMD_INIT) {
                    PutMdData(comData.pid, Com.nRMID, comData);
                } else {
                    printf("Unknown ComData.type : %d\n", comData.type);
                }
            } else {
            }
        } else {
        }

        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    string id = "outtrigger";
    ros::init(argc, argv, id.c_str());
    ros::NodeHandle nh("~");

    int main_hz;
    string com;

    printf("boost::lockfree::queue is ");
    if (!qarr.is_lock_free())
        printf("not ");
    printf("lockfree\n");

    // 파라미터 초기화
    #if 1
    ros::param::get("~MOTOR_NUM", MOTOR_NUM);
    ros::param::get("~GEAR_RATIO", GEAR_RATIO);
    ros::param::get("~MOTOR_TICK", MOTOR_TICK);
    ros::param::get("~SCREW_LEAD", SCREW_LEAD);
    ros::param::get("~TIMEOUT_SEC_MAIN", TIMEOUT_SEC_MAIN);
    ros::param::get("~TIMEOUT_SEC_HOMING", TIMEOUT_SEC_HOMING);
    ros::param::get("~MIN_MM", MIN_MM);
    ros::param::get("~MAX_MM", MAX_MM);
    ros::param::get("~MIN_MM_PER_SEC", MIN_MM_PER_SEC);
    ros::param::get("~MAX_MM_PER_SEC", MAX_MM_PER_SEC);
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~communication", com);
    ros::param::get("~inv_motor_in_arr", inv_motor_in_arr);
    ros::param::get("~inv_encoder_out_arr", inv_encoder_out_arr);
    ros::param::get("~override_homing", override_homing);
    ros::param::get("~motor_info_hz", motor_info_hz);
    #else
    nh.getParam("MOTOR_NUM", MOTOR_NUM);
    nh.getParam("GEAR_RATIO", GEAR_RATIO);
    nh.getParam("MOTOR_TICK", MOTOR_TICK);
    nh.getParam("SCREW_LEAD", SCREW_LEAD);
    nh.getParam("TIMEOUT_SEC_MAIN", TIMEOUT_SEC_MAIN);
    nh.getParam("TIMEOUT_SEC_HOMING", TIMEOUT_SEC_HOMING);
    nh.getParam("MIN_MM", MIN_MM);
    nh.getParam("MAX_MM", MAX_MM);
    nh.getParam("MIN_MM_PER_SEC", MIN_MM_PER_SEC);
    nh.getParam("MAX_MM_PER_SEC", MAX_MM_PER_SEC);
    nh.getParam("main_hz", main_hz);
    nh.getParam("communication", com);
    nh.getParam("inv_motor_in_arr", inv_motor_in_arr);
    nh.getParam("inv_encoder_out_arr", inv_encoder_out_arr);
    nh.getParam("override_homing", override_homing);
    nh.getParam("motor_info_hz", motor_info_hz);
    #endif

    #if 0
    std::vector<std::string> keys;
    nh.getParamNames(keys);
    for(string key: keys) {
        #if 0
        std::cout << "par: " << key << std::endl;
        #else
        printf("par: %s\n", key.c_str());
        #endif
    }
    #endif

    if (inv_motor_in_arr.size() != MOTOR_NUM || inv_encoder_out_arr.size() != MOTOR_NUM) {
        ROS_ERROR("wrong rosparam: PARAM element size(inv_motor(encoder)_in_arr) isn't matching PARAM size(MOTOR_NUM)");

        return -1;
    }

    Com = Communication(MOTOR_NUM);
    Com.kaCom = com;

    ROS_INFO("Com.kaCom: %s", Com.kaCom.c_str());

    if (Com.kaCom == "rs485") {
    } else {
    }

    ComData comData;

    //variable declaration
    IByte iData;
    static BYTE byCntComStep, byCntCmdVel, fgSendCmdVel, byCntInitStep;
    static BYTE byCnt2500us, byCntCase[10], byFgl, byFglReset, fgInitPosiResetAfter2s, byCntReset;
    static BYTE byCnt, byCntStartDelay;

    int nArray[5];

    fgSendCmdVel      = ON;
    byFgl             = OFF;
    Com.fgInitsetting = OFF;

    //Store the value of the parameter in the variable
    nh.getParam("PC", Com.nIDPC);
    nh.getParam("MDUI", Com.nIDMDUI);
    nh.getParam("MDT", Com.nIDMDT);
    nh.getParam("baudrate", Com.nBaudrate);
    nh.getParam("RMID", Com.nRMID);
    nh.getParam("slowstart", Com.nSlowstart);
    nh.getParam("slowdown", Com.nSlowdown);

    double Motor_rpm[4];
    double motor_in_rpm[4];

    unsigned int cntOffSec = 0;

    int send_rate = 200; // Hz

    if (Com.kaCom == "rs485") {
    } else {
    }

    // topic
    // subscriber
    #if 0
    ros::Subscriber sub_pidRequest = nh.subscribe("/outtrigger/pidRequest", 100, pidRequestCallback);
    ros::Subscriber sub_pidCommand = nh.subscribe("/outtrigger/pidCommand", 100, pidCommandCallback);
    ros::Subscriber sub_pidVelocity = nh.subscribe("/outtrigger/pidVelocity", 100, pidVelocityCallback);
    ros::Subscriber sub_pidPosition = nh.subscribe("/outtrigger/pidPosition", 100, pidPositionCallback);
    ros::Subscriber sub_command = nh.subscribe("/outtrigger/command", 100, commandCallback);
    #else
    ros::Subscriber sub_command = nh.subscribe("/outtrigger/command", 100, commandsCallback);
    #endif
    // publisher
    #ifdef KIRO_MSG
    ros::Publisher pub_info = nh.advertise<painting_robot_msg::OUTTRIGGER_State>("/outtrigger/info", 100);
    #else
    ros::Publisher pub_info = nh.advertise<outtrigger::Infos>("/outtrigger/info", 100);
    #endif
    ros::Publisher pub_state = nh.advertise<outtrigger::States>("/outtrigger/state", 100);

    boost::thread threadSendMsgMd1k(sendMsgMd1k, &send_rate);

    if (Com.kaCom == "rs485") {
        InitSerial();
    } else {
    }

    double time_cur = ros::Time::now().toSec();
    double time_pre = time_cur;
    double time_diff;

    double qmsg_time_pre = time_cur;
    double qmsg_time_diff;
    #if 0
    double QMSG_TS_PERIOD = 0.05;   // s
    #else
    double QMSG_TS_PERIOD = 1;   // s
    #endif

    double pub_time_pre = time_cur;
    double pub_time_diff;
    double PUB_TS_PERIOD = 0.1;

    if (Com.kaCom == "rs485") {
    } else {
    }

    ros::Rate r(main_hz);

    ros::Publisher pub_motor_info = nh.advertise<outtrigger::MotorInfo>("motor_info", 100);
    boost::thread threadGetMotorInfo(getMotorInfo, motor_info_hz, pub_motor_info);
	
    ros::Time ts_now;

    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        Com.kaTxTsMainOld[i] = ts_now;
        Com.kaTxTsMain[i] = ts_now;
        Com.kaRxTsMainOld[i] = ts_now;
        Com.kaRxTsMain[i] = ts_now;
        Com.kaRxTsIo[i] = ts_now;
    }

    while(ros::ok())
    {
        time_cur = ros::Time::now().toSec();

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        // 데이터파싱 및 패킷분리
        if (Com.kaCom == "rs485") {
            ReceiveDataFromController();
        } else {
        }

        pub_time_diff = time_cur - pub_time_pre;
        if (pub_time_diff > PUB_TS_PERIOD) {
            pub_time_pre = time_cur;

            pub_state.publish(outtriggerStates);
            pub_info.publish(outtriggerInfos);
        }

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        qmsg_time_diff = time_cur - qmsg_time_pre;
        if (qmsg_time_diff > QMSG_TS_PERIOD) {
            qmsg_time_pre = time_cur;
            // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());

            #if 1
            for (int i=0; i<MOTOR_NUM; i++) {
                #if 1
                // Homing 확인용
                comData.type = MD_REQ_PID;
                comData.id = i+ID_OFFSET;
                comData.nArray[0] = PID_INIT_SET_OK;
                qarr.push(comData);
                #endif

                #if 1
                // 상태피드백 확인용
                comData.type = MD_REQ_PID;
                comData.id = i+ID_OFFSET;
                comData.nArray[0] = PID_MAIN_DATA;
                qarr.push(comData);
                #endif
            }
            #endif

            #if 0
            printf("pos");
            for (int i=0; i<MOTOR_NUM; i++) {
                printf("[%+10d]", Com.kaPosition[i]);
            }
            printf("\n");
            #endif

            #if 0
            printf("spd");
            for (int i=0; i<MOTOR_NUM; i++) {
                printf("[%+10d]", Com.kaSpeed[i]);
            }
            printf("\n");
            #endif

            #if 0
            printf("cur");
            for (int i=0; i<MOTOR_NUM; i++) {
                printf("[%+10d]", Com.kaCurrent[i]);
            }
            printf("\n");
            #endif

            #if 0
            printf("hom");
            for (int i=0; i<MOTOR_NUM; i++) {
                printf("[%+10d]", Com.kaHomingDone[i]);
            }
            printf("\n");
            #endif
        }

        #if 0
        checkOuttrigger();
        #endif

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        ros::spinOnce();
        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        r.sleep();
    }

    threadGetMotorInfo.join();
    threadSendMsgMd1k.join();

    #if 1
    printf("send velocity 0 rpm\n");
    for (int i=0; i<MOTOR_NUM; i++) {
        comData.id = i+ID_OFFSET;
        comData.rpm = 0;

        if (Com.kaCom == "rs485") {
            PutMdData(PID_PNT_VEL_CMD, Com.nRMID, comData);
        } else {
        }
        ros::Duration(0.010).sleep();
    }
    #endif

    printf("program end\n");

    return 0;
}
