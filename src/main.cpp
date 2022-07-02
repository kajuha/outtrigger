#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <std_msgs/Empty.h>

#include "global.hpp"
#include "main.hpp"
#include "com.hpp"

#include <ros/ros.h>

ros::Time ts_now;;

int MOTOR_NUM;
std::vector<double> inv_motor_in_arr;
std::vector<double> inv_encoder_out_arr;

Communication Com;

#include <queue>

#define LOCKFREE_QUEUE_SIZE 100
boost::lockfree::queue<ComData> qarr(LOCKFREE_QUEUE_SIZE);

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
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~communication", com);
    ros::param::get("~inv_motor_in_arr", inv_motor_in_arr);
    ros::param::get("~inv_encoder_out_arr", inv_encoder_out_arr);
    #else
    nh.getParam("MOTOR_NUM", MOTOR_NUM);
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

    unsigned int count = 0;

    int send_rate = 200;
  
    if (Com.kaCom == "rs485") {
    } else {
    }

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

    ts_now = ros::Time::now();
    for (int i=0; i<MOTOR_NUM; i++) {
        Com.kaTsLast[i] = ts_now;
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

            #if 0
            for (int i=0; i<MOTOR_NUM; i++) {
                motor_in_rpm[i] = Motor_rpm[i];
                motor_in_rpm[i] *= inv_motor_in_arr[i];

                if (Com.kaCom == "rs485") {
                    comData.type = MD_SEND_RPM;
                    comData.id = i+ID_OFFSET;
                    comData.rpm = (short)motor_in_rpm[i];
                } else {
                }
                qarr.push(comData);
            }
            #else
            // 시험
            #if 0
            comData.type = MD_REQ_PID;
            comData.id = 1;
            comData.nArray[0] = PID_VER;
            qarr.push(comData);
            #endif
            
            #if 1
            comData.type = MD_REQ_PID;
            comData.id = 1;
            comData.nArray[0] = PID_IO_MONITOR;
            qarr.push(comData);
            #endif
            // 시험
            #endif

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
