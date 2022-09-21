#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <std_msgs/Empty.h>

#include <painting_robot_msg/OUTTRIGGER_Command_Info.h>
#include <painting_robot_msg/OUTTRIGGER_Command.h>
#include <painting_robot_msg/OUTTRIGGER_State_Info.h>
#include <painting_robot_msg/OUTTRIGGER_State.h>

#include "outtrigger/PidRequest.h"
#include "outtrigger/PidCommand.h"
#include "outtrigger/PidVelocity.h"
#include "outtrigger/PidPosition.h"
#include "outtrigger/Command.h"

#include "outtrigger/Info.h"
#include "outtrigger/State.h"

#include "global.hpp"
#include "main.hpp"
#include "com.hpp"

#include <ros/ros.h>

int MOTOR_NUM;
double GEAR_RATIO;
double MOTOR_TICK;
double SCREW_LEAD;
double TIMEOUT_SEC_MAIN;
double TIMEOUT_SEC_IO;
std::vector<double> inv_motor_in_arr;
std::vector<double> inv_encoder_out_arr;

Communication Com;

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
    comData.pid = PID_PNT_POS_VEL_CMD;
    comData.position = pidPosition.position;
    comData.rpm = pidPosition.rpm;
    qarr.push(comData);
}

void commandCallback(const outtrigger::Command& command) {
    static ComData comData;

    // 모터 위치 이동
    comData.type = MD_CMD_PID;
    comData.id = command.command;
    comData.pid = PID_PNT_POS_VEL_CMD;
    #define MIN_TO_SEC 60.0
    static double mm_in;
    mm_in = command.mm;
    #define MIN_MM 0.0
    #define MAX_MM 200.0
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
    #define MIN_MM_PER_SEC 0.0
    #define MAX_MM_PER_SEC 20.0
    if (MIN_MM_PER_SEC > mm_sec_in || mm_sec_in > MAX_MM_PER_SEC) return;
    static double mm_min;
    mm_min = mm_sec_in * MIN_TO_SEC;
    static double screw_rev_min;
    screw_rev_min = mm_min / SCREW_LEAD;
    static double gear_rev_min;
    gear_rev_min = screw_rev_min * GEAR_RATIO;
    comData.rpm = gear_rev_min;
    printf("[in ] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_in, mm_sec_in, comData.position, comData.rpm);
    qarr.push(comData);

    static double mm_out;
    mm_out = 0.0;
    mm_out = (double)Com.kaPosition[ID_OFFSET-command.command] / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
    static double mm_sec_out;
    mm_sec_out = 0.0;
    mm_sec_out = (double)Com.kaSpeed[ID_OFFSET-command.command] / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
    printf("[out] mm: %7.3f, mm_sec: %7.3f, pos: %10d, rpm: %5d\n", mm_out, mm_sec_out, Com.kaPosition[ID_OFFSET-command.command], Com.kaSpeed[ID_OFFSET-command.command]);
}

void checkOuttrigger() {
    ros::Time ts_now;
    double timeout;

    #if 0
    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        timeout = ts_now.toSec() - Com.kaTsLast[i].toSec();
        if (timeout > TIMEOUT_SEC_MAIN) {
            printf("invalid main data: #%d, timeout: %lf, SET_TIMEOUT: %lf\n", i, timeout, TIMEOUT_SEC_MAIN);
        }
    }
    #endif

    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        timeout = ts_now.toSec() - Com.kaTsLastIo[i].toSec();
        if (timeout > TIMEOUT_SEC_IO) {
            printf("invalid io data: #%d, timeout: %lf, SET_TIMEOUT: %lf\n", i, timeout, TIMEOUT_SEC_IO);
        }
    }
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
    ros::param::get("~TIMEOUT_SEC_IO", TIMEOUT_SEC_IO);
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~communication", com);
    ros::param::get("~inv_motor_in_arr", inv_motor_in_arr);
    ros::param::get("~inv_encoder_out_arr", inv_encoder_out_arr);
    #else
    nh.getParam("MOTOR_NUM", MOTOR_NUM);
    nh.getParam("GEAR_RATIO", GEAR_RATIO);
    nh.getParam("MOTOR_TICK", MOTOR_TICK);
    nh.getParam("SCREW_LEAD", SCREW_LEAD);
    nh.getParam("TIMEOUT_SEC_MAIN", TIMEOUT_SEC_MAIN);
    nh.getParam("TIMEOUT_SEC_IO", TIMEOUT_SEC_IO);
    nh.getParam("main_hz", main_hz);
    nh.getParam("communication", com);
    nh.getParam("inv_motor_in_arr", inv_motor_in_arr);
    nh.getParam("inv_encoder_out_arr", inv_encoder_out_arr);
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
    ros::Subscriber sub_pidRequest = nh.subscribe("/outtrigger/pidRequest", 100, pidRequestCallback);
    ros::Subscriber sub_pidCommand = nh.subscribe("/outtrigger/pidCommand", 100, pidCommandCallback);
    ros::Subscriber sub_pidVelocity = nh.subscribe("/outtrigger/pidVelocity", 100, pidVelocityCallback);
    ros::Subscriber sub_pidPosition = nh.subscribe("/outtrigger/pidPosition", 100, pidPositionCallback);
    ros::Subscriber sub_command = nh.subscribe("/outtrigger/command", 100, commandCallback);
    // publisher
    ros::Publisher pub_info = nh.advertise<outtrigger::Info>("/outtrigger/info", 100);
    ros::Publisher pub_state = nh.advertise<outtrigger::State>("/outtrigger/state", 100);

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
  
    if (Com.kaCom == "rs485") {
    } else {
    }

    ros::Rate r(main_hz);

    ros::Time ts_now;
    
    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        Com.kaTsLast[i] = ts_now;
        Com.kaTsLastIo[i] = ts_now;
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

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        qmsg_time_diff = time_cur - qmsg_time_pre;
        if (qmsg_time_diff > QMSG_TS_PERIOD) {
            qmsg_time_pre = time_cur;
            // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());

            #if 1
            for (int i=0; i<MOTOR_NUM; i++) {
                #if 1
                // CTRL 입력확인용
                comData.type = MD_REQ_PID;
                comData.id = i+ID_OFFSET;
                comData.nArray[0] = PID_IO_MONITOR;
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

            #if 1
            printf("pos");
            for (int i=0; i<MOTOR_NUM; i++) {
                printf("[%+10d]", Com.kaPosition[i]);
            }
            printf("\n");
            #endif

            #if 1
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

            #if 1
            printf("ctl");
            for (int i=0; i<MOTOR_NUM; i++) {
                #define CTRL_BIT_LEN 10
                printf("[");
                for (int j=CTRL_BIT_LEN-1; j>=0; j--) {
                    printf("%d", Com.kaCtrlInput[i]&(1<<j)?1:0);
                }
                printf("]");
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
