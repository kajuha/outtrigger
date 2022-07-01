///v1.9d modifying code while PNT_TQ_OFF(PNT_BREKE) communication. it request PNT_MAIN_DATA and for 2s delay
///v1.9e adding code reset command after 2s
///v1.9f adding code reset command when RMID is MDT'
///v1.9g modifying 'Md.sCmdAngularVel calculation' in vel_cmd.cpp according to "md_node/angleresolution"

#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "global.hpp"
#include "main.hpp"
#include "com.hpp"

#include <ros/ros.h>

ros::Time ts_now;;

int WHEEL_NUM;
int MOTOR_NUM;
double WHEEL_RADIUS;
double WHEEL_LENGTH;
double WHEEL_WIDTH;
double WHEEL_XY;
double GEAR_RATIO;
double MOTOR_TICK;
double WHEEL_MAX_RPM;
double MOTOR_MAX_RPM;
double MOTOR_CALIBRATION;
double SEC_TO_MIN;
double RAD_TO_REV;
double IK_STEP_TIME;
double DEG_TO_RAD;
double RAD_TO_DEG;
double TICK_TO_REV;
double REV_TO_TICK;
double TICK_TO_RAD;
double REV_TO_RAD;
double RPM_TO_RadPSec;
int whisper_en;
std::vector<double> inv_motor_in_arr;
std::vector<double> inv_encoder_out_arr;

Communication Com;

int encoder_error;

geometry_msgs::Twist _vel;

double _imu_orientation[ORIENTATION];

#include <queue>

#if 0
// DEPRECATED
typedef struct _NARRAY {
    int nArray0;
    int nArray1;
    int nArray2;
    int nArray3;
    int nArray4;
    int type;
} NARRAY;
#endif

#define LOCKFREE_QUEUE_SIZE 100
#if 0
// DEPRECATED
boost::lockfree::queue<NARRAY> qarr(LOCKFREE_QUEUE_SIZE);
#else
boost::lockfree::queue<ComData> qarr(LOCKFREE_QUEUE_SIZE);
#endif

void sendMsgMd1k(int* send_rate) {
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

    ros::Rate loop_rate(*send_rate);

    int nArray[5] = {0, };
    #if 0
    // DEPRECATED
    NARRAY narr;
    #else
    ComData comData;
    #endif

    double ts_now, ts_begin, ts_diff;
    ts_begin = ts_now = ros::Time::now().toSec();

    while (ros::ok())
    {
        ts_now = ros::Time::now().toSec();

        #if 0
        // DEPRECATED
        if (qarr.pop(narr)) {
        #else
        if (qarr.pop(comData)) {
        #endif
            #if 0
            ts_diff = ts_now - ts_begin;
            ts_begin = ts_now;
            ROS_INFO("[ts_diff]:%10.3lf(ms), %10.3lf(hz)", ts_diff*1000.0, 1.00/ts_diff);
            #endif

            // DEPRECATED
            // nArray[0] = comData.nArray0;
            // nArray[1] = comData.nArray1;
            // nArray[2] = comData.nArray2;
            // nArray[3] = comData.nArray3;
            // nArray[4] = comData.nArray4;

            if (Com.kaCom == "rs485") {
                if (comData.type == MD_SEND_RPM) {
                    // DEPRECATED
                    // PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray, comData);
                    PutMdData(PID_PNT_VEL_CMD, Com.nRMID, comData);
                } else if (comData.type == MD_SET_POS) {
                    // DEPRECATED
                    // PutMdData(PID_POSI_SET, Com.nRMID, nArray, comData);
                    PutMdData(PID_POSI_SET, Com.nRMID, comData);
                } else {
                }
            } else if (Com.kaCom == "can") {
                if (comData.type == MD_SEND_RPM) {
                    // DEPRECATED
                    // rpmSend(nArray[2], nArray[0]);
                } else if (comData.type == MD_SET_POS) {
                    // DEPRECATED
                    // setPosition(nArray[2], nArray[0]);
                } else {
                }
            } else {
            }
        } else {
        }

        if (Com.kaCom == "rs485") {
        } else if (Com.kaCom == "can") {
        } else {
        }

        loop_rate.sleep();
    }
}

#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)
{
    string id = "outtrigger";
	// 모터드라이버 노드
    ros::init(argc, argv, id.c_str());
    ros::NodeHandle nh("~");

    double timeout_sec;
    int main_hz;
    string com;
    bool canError = false;
    
    cout << "boost::lockfree::queue is ";
    if (!qarr.is_lock_free())
        cout << "not ";
    cout << "lockfree" << endl;

    // 파라미터 초기화
    #if 1
    ros::param::get("~WHEEL_NUM", WHEEL_NUM);
    ros::param::get("~WHEEL_RADIUS", WHEEL_RADIUS);
    ros::param::get("~WHEEL_LENGTH", WHEEL_LENGTH);
    ros::param::get("~WHEEL_WIDTH", WHEEL_WIDTH);
    ros::param::get("~GEAR_RATIO", GEAR_RATIO);
    ros::param::get("~MOTOR_TICK", MOTOR_TICK);
    ros::param::get("~WHEEL_MAX_RPM", WHEEL_MAX_RPM);
    ros::param::get("~MOTOR_CALIBRATION", MOTOR_CALIBRATION);
    ros::param::get("~whisper_en", whisper_en);
    ros::param::get("~timeout_sec", timeout_sec);
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~communication", com);
    ros::param::get("~encoder_error", encoder_error);
    ros::param::get("~inv_motor_in_arr", inv_motor_in_arr);
    ros::param::get("~inv_encoder_out_arr", inv_encoder_out_arr);
    #else
    nh.getParam("WHEEL_NUM", WHEEL_NUM);
    nh.getParam("WHEEL_RADIUS", WHEEL_RADIUS);
    nh.getParam("WHEEL_LENGTH", WHEEL_LENGTH);
    nh.getParam("WHEEL_WIDTH", WHEEL_WIDTH);
    nh.getParam("GEAR_RATIO", GEAR_RATIO);
    nh.getParam("MOTOR_TICK", MOTOR_TICK);
    nh.getParam("WHEEL_MAX_RPM", WHEEL_MAX_RPM);
    nh.getParam("MOTOR_CALIBRATION", MOTOR_CALIBRATION);
    nh.getParam("whisper_en", whisper_en);
    nh.getParam("timeout_sec", timeout_sec);
    nh.getParam("main_hz", main_hz);
    nh.getParam("communication", com);
    nh.getParam("encoder_error", encoder_error);
    nh.getParam("inv_motor_in_arr", inv_motor_in_arr);
    nh.getParam("inv_encoder_out_arr", inv_encoder_out_arr);
    #endif
    // std::vector<std::string> keys;
    // nh.getParamNames(keys);
    // for(string key: keys) {
    //     #if 0
    //     std::cout << "par: " << key << std::endl;
    //     #else
    //     printf("par: %s\n", key.c_str());
    //     #endif
    // }
    if (inv_motor_in_arr.size() != WHEEL_NUM || inv_encoder_out_arr.size() != WHEEL_NUM) {
        ROS_ERROR("wrong rosparam: PARAM element size(inv_motor(encoder)_in_arr) isn't matching PARAM size(WHEEL_NUM)");
        
        return -1;
    }

    Com = Communication(WHEEL_NUM);
    //cmd_wheel_radPerSec = std::vector<double>(WHEEL_NUM);
    Com.kaCom = com;

    ROS_INFO("Com.kaCom: %s", Com.kaCom.c_str());

    Com.kaSet[0] = TYPE_KASET::FIRSTRUN;
    Com.kaSet[1] = TYPE_KASET::FIRSTRUN;
    Com.kaSet[2] = TYPE_KASET::FIRSTRUN;
    Com.kaSet[3] = TYPE_KASET::FIRSTRUN;

    MOTOR_NUM = WHEEL_NUM;
    WHEEL_XY = (WHEEL_LENGTH + WHEEL_WIDTH);
    //WHEEL_XY = ( WHEEL_WIDTH-WHEEL_LENGTH);
    MOTOR_MAX_RPM = abs(WHEEL_MAX_RPM * GEAR_RATIO);
    SEC_TO_MIN = (1.0/60.0);
    RAD_TO_REV = (1.0/(2.0*M_PI));
    if (Com.kaCom == "rs485") {
        IK_STEP_TIME = 0.04;
    } else if (Com.kaCom == "can") {
        IK_STEP_TIME = 0.01;
    } else {
    }
    DEG_TO_RAD = (M_PI/180.0);
    RAD_TO_DEG = (180.0/M_PI);
    TICK_TO_REV = (1.0/MOTOR_TICK);
    REV_TO_TICK = MOTOR_TICK;
    TICK_TO_RAD = (2.0*M_PI/MOTOR_TICK);
    REV_TO_RAD  = (2.0*M_PI);
    RPM_TO_RadPSec = (2.0*M_PI)/60.0;

    ros::Publisher lodom_pub = nh.advertise<nav_msgs::Odometry>("/outtrigger/lodom", 100);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);

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

    // donglerobot
    //20220120 LSM 변수 정리중
    double feed_wheel_rad[4]    = {0.0 , 0.0 , 0.0 , 0.0 };                // 메카넘휠 최종 휠 각도(rad)(모터드라이버에서 펄스위치값을 피드백받아서 계산)
    double cmd_wheel_vel_rpm[4] = {0.0 , 0.0 , 0.0 , 0.0 };                 // rev/min
    double cmd_wheel_vel_rps[4] = {0.0 , 0.0 , 0.0 , 0.0 };                 // rad/sec
    double Motor_rpm[4];
    double motor_in_rpm[4];

    double cmd_odom_vel_mps[3]={0.0, };                                  // 변수명을 변경하기 위한 재정의 및 저장

    double feed_wheel_pos_delta_rad[4]  = {0.0 , 0.0 , 0.0 , 0.0};
    double feed_wheel_vel_rps[4]        = {0.0 , 0.0 , 0.0 , 0.0};
    double feed_wheelMD1K_vel_rps[4]    = {0.0 , 0.0 , 0.0 , 0.0};

    double feed_cart_pos_delta_m[3]  = {0.0 , 0.0 , 0.0};
    double feed_cart_MD1K_vel_mps[3]  = {0.0 , 0.0 , 0.0};
    double feed_odom_pos_delta_m[3]  = {0.0 , 0.0 , 0.0};
    double feed_odom_delta_m[3]  = {0.0 , 0.0 , 0.0};
    double last_theta = 0.0;    // 메카넘휠 좌표계에서 odometry 절대좌표계 변환하는 회전값 초기에 주어지는 값
    double feed_odom_MD1K_vel_mps[3]  = {0.0 ,};                                                // MD1K Wheel RPM 기반 AMR 속도 오토메트리 값 ( 그나마 속도값이기에 정확)
    double feed_odom_pos_m[3]  = {0.0 ,};                                                // AMR 위치 오도메트리 값(LSM 좌표계)
    double feed_odom_m[3]  = {0.0 ,};                                                // AMR 위치 오도메트리 값(odom 좌표계)
    double feed_odom_vel_mps[3]  = {0.0 ,};                                                     // 위치 펄스값 기반 AMR 속도 오도메트리 값 ( 정확한 시간 계산이 안되어 오차가 발생)

    unsigned int count = 0;
  
    if (Com.kaCom == "rs485") {
    } else if (Com.kaCom == "can") {
    } else {
    }

    // thread test start
    int send_rate = 200; // 200 Hz
  
    if (Com.kaCom == "rs485") {
    } else if (Com.kaCom == "can") {
        send_rate = 2000;
    } else {
    }

    boost::thread threadSendMsgMd1k(sendMsgMd1k, &send_rate);
    // thread test end
  
    if (Com.kaCom == "rs485") {
        InitSerial();     //communication initialization in com.cpp
    } else if (Com.kaCom == "can") {
    } else {
    }

    double time_cur = ros::Time::now().toSec();
    double time_pre = time_cur;
    double time_diff;

    double qmsg_time_pre = time_cur;
    double qmsg_time_diff;
    double QMSG_TS_PERIOD = 0.05;   // s

    double whisper_time_pre = time_cur;
    double whisper_time_diff;
    double WHISPER_TS_PERIOD = 0.05;   // s
  
    if (Com.kaCom == "rs485") {
    } else if (Com.kaCom == "can") {
        QMSG_TS_PERIOD = 0.01;   // s
    } else {
    }

    // ros::Rate r(10000);
    //donglerobot                                                                 //Set the loop period -> 100us.
    ros::Rate r(main_hz);          // 메인 타이머 1000Hz 1msec

    ts_now = ros::Time::now();
    for (int i=0; i<WHEEL_NUM; i++) {
        Com.kaTsLast[i] = ts_now;
    }

    double Motor_rpm_sign[4] = {-1.0, 1.0, -1.0, 1.0};

    while(ros::ok())
    {
        time_cur = ros::Time::now().toSec();

        //printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        // 데이터파싱 및 패킷분리
        if (Com.kaCom == "rs485") {
            ReceiveDataFromController();
        } else if (Com.kaCom == "can") {
        } else {
        }

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        qmsg_time_diff = time_cur - qmsg_time_pre;
        if (qmsg_time_diff > QMSG_TS_PERIOD) {
            qmsg_time_pre = time_cur;
        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
            for (int i=0; i<MOTOR_NUM; i++) {
#if 1
                // DEPRECATED
                // static NARRAY narr;
                static ComData comData;

                motor_in_rpm[i] = Motor_rpm[i];
                motor_in_rpm[i] *= inv_motor_in_arr[i];

                #if 0
                // DEPRECATED
                if (Com.kaCom == "rs485") {
                    iData = Short2Byte((short)motor_in_rpm[i]);
                    narr.nArray0 = iData.byLow;
                    narr.nArray1 = iData.byHigh;
                    narr.nArray2 = i+MD1K_ID_OFFSET;
                    narr.nArray3 = 0;
                    narr.nArray4 = REQUEST_PNT_MAIN_DATA;
                    narr.type = MD_SEND_RPM;
                } else if (Com.kaCom == "can") {
                    narr.nArray0 = (short)motor_in_rpm[i];
                    narr.nArray1 = 0;
                    narr.nArray2 = i+MD1K_ID_OFFSET;
                    narr.nArray3 = 0;
                    narr.type = MD_SEND_RPM;
                } else {
                }
                qarr.push(narr);
                #else
                if (Com.kaCom == "rs485" || Com.kaCom == "can") {
                    comData.type = MD_SEND_RPM;
                    comData.id = i+MD1K_ID_OFFSET;
                    comData.rpm = (short)motor_in_rpm[i];
                } else {
                }
                qarr.push(comData);
                #endif
#else
                iData = Short2Byte((short)motor_in_rpm[i]);
                nArray[0] = iData.byLow;
                nArray[1] = iData.byHigh;
                nArray[2] = i+MD1K_ID_OFFSET;
                nArray[3] = 0;
                nArray[4] = REQUEST_PNT_MAIN_DATA;
                PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray);
                ros::Duration(0.010).sleep();
                // double start = ros::Time::now().toSec();
                ReceiveDataFromController();
                // double end = ros::Time::now().toSec();
                // printf("%d, %lf \n", __LINE__, end-start);
#endif
            }
            #if 0
            for (int i=0; i<MOTOR_NUM; i++) {
                printf("%6.3lf ", Motor_rpm[i]);
            }
            printf("send:%lf \n", ros::Time::now().toSec());

            for (int i=0; i<MOTOR_NUM; i++) {
                printf("%6.3lf ", (double)Com.kaSpeed[i]);
            }
            printf("recv:%lf \n", ros::Time::now().toSec());
            #endif
        }

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        time_diff = time_cur - time_pre;
        if ( time_diff > IK_STEP_TIME) {
            time_pre = time_cur;
        }

        whisper_time_diff = time_cur - whisper_time_pre;
        if (whisper_time_diff > WHISPER_TS_PERIOD) {
            whisper_time_pre = time_cur;
            if (whisper_en) {
            }
        }

        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        ros::spinOnce();
        // printf("%d, %lf \n", __LINE__, ros::Time::now().toSec());
        r.sleep();
    }

    threadSendMsgMd1k.join();

    #if 1
    printf("send velocity 0 rpm\n");
    ComData comData;
    for (int i=0; i<WHEEL_NUM; i++) {
        #if 0
        // DEPRECATED
        nArray[0] = 0;
        nArray[1] = 0;
        nArray[2] = i+MD1K_ID_OFFSET;
        nArray[3] = 0;
        nArray[4] = REQUEST_PNT_MAIN_DATA;
        #else
        comData.id = i+MD1K_ID_OFFSET;
        comData.rpm = 0;
        #endif
        if (Com.kaCom == "rs485") {
            // DEPRECATED
            // PutMdData(PID_PNT_VEL_CMD, Com.nRMID, nArray, comData);
            PutMdData(PID_PNT_VEL_CMD, Com.nRMID, comData);
        } else if (Com.kaCom == "can") {
        } else {
        }
        ros::Duration(0.010).sleep();
    }
    #endif

    printf("program end\n");

    if (Com.kaCom == "rs485") {
    } else if (Com.kaCom == "can") {
    } else {
    }

    return 0;
}
