#ifndef _COM_HPP
#define _COM_HPP

#include "main.hpp"

#define OUTTRIGGER_FRONT_LEFT 0
#define OUTTRIGGER_FRONT_RIGHT 1
#define OUTTRIGGER_BACK_LEFT 2
#define OUTTRIGGER_BACK_RIGHT 3

#define MD_SEND_RPM         0
#define MD_SET_POS          1
#define MD_REQ_PID          2
#define MD_OFF              3
#define MD_CMD_PID          4
#define MD_CMD_INIT         5

#define ID_BLDC_CTRL        1
#define ID_MDUI             2
#define ID_ALL              0xfe

#define PID_VER             1
#define PID_REQ_PID_DATA    4
#define PID_TQ_OFF          5
#define PID_BRAKE           6
#define PID_ACK             7

#define PID_COMMAND         10
#define PID_CMD_TQ_OFF      2
#define PID_CMD_BRAKE       4
#define PID_CMD_ALARM_RESET 8
#define PID_CMD_POSI_RESET  10
#define PID_CMD_INIT_SET2   90

#define PID_POSI_RESET      13
#define PID_IN_POSITION_OK  49
#define PID_INIT_SET_OK     87
// #define PID_BAUDRATE        135  // DEPRECATED
// #define PID_VOLT_IN         143  // DEPRECATED
#define PID_SLOW_START      153
#define PID_SLOW_DOWN       154
// #define PID_PNT_TQ_OFF      174  // DEPRECATED
// #define PID_PNT_BRAKE       175  // DEPRECATED
#define PID_PNT_POS_VEL_CMD 206
#define PID_PNT_VEL_CMD     207
#define PID_POSI_VEL_CMD 219
#define PID_MAIN_DATA       193
#define PID_IO_MONITOR      194
// #define PID_PNT_MAIN_DATA   210  // DEPRECATED
#define PID_POSI_SET        217
// #define PID_ROBOT_PARAM     247  // DEPRECATED
// #define PID_ROBOT_MONITOR   253  // DEPRECATED
// #define PID_ROBOT_MONITOR2  224  // DEPRECATED
// #define PID_ROBOT_CMD       252  // DEPRECATED

#define MAX_PACKET_SIZE     26
#define MAX_DATA_SIZE       21

#define MOT_LEFT            0
#define MOT_RIGHT           1

#define REQUEST_PNT_MAIN_DATA 2

#define DURATION            0.0001

#define TIME_50MS           1
#define TIME_100MS          2
#define TIME_1S             20
#define TIME_5S             100

#define CHECK_EMERGENCY_SW  0
#define SEND_DATA_AFTER_1S  1

#define CMD_ALARM_RESET     8
class Communication {
    public:
    BYTE bySndBuf[MAX_PACKET_SIZE];
    BYTE byRcvBuf[MAX_PACKET_SIZE];
    BYTE byPacketSize;
    BYTE byPacketNum;
    BYTE byIn, byStep;
    BYTE byChkSend;
    BYTE byChkRcv;
    BYTE fgInIdleLine, fgPacketOK, fgComComple;
    BYTE byTotalRcvDataNum;
    BYTE fgChk;
    BYTE byChkSum, byMaxDataNum, byDataNum;

	// ID 변수(PC, MDUI, MDT, RMID)
    int nIDPC, nIDMDUI, nIDMDT, nRMID;
    int nBaudrate;
    short sSetDia, sSetWheelLen, sSetGear;
    int nCmdSpeed, nCmdAngSpeed;

    short sTempVoltIn, sSumVolt ,sVoltIn;
    BYTE byPlatStatus, bEmerSW, bBusy, bBumper1, bBumper2, bBumper3, bBumper4;
    BYTE byDocStatus, bDocComple, bChargeState, bCharComple, bIr1, bIr2, bIr3, bRccState;

    BYTE byUS1, byUS2, byUS3, byUS4;

    BYTE fgResetOdometry, fgControlstate, fgResetAngle;

    int fgSndOK, nCntDelaySnd;

    long lPosi[2], lTempPosi[2];
    short sTheta, sTempTheta, sExTheta;

    long lMoving[3][3];

    short sMotorRPM[2];
    long lMotorPosi[2];

    WORD sCurrent[2];

    BYTE byStatus[2];
    BYTE fgAlarm[2], fgCtrlFail[2], fgOverVolt[2], fgOverTemp[2];
    BYTE fgOverLoad[2], fgHallFail[2], fgInvVel[2], fgStall[2];

    BYTE byChkComError;
    BYTE fgComDataChk;

    BYTE byCntVoltAver;

    int nSlowstart, nSlowdown;
    BYTE fgInitsetting;

    BYTE fgResetAlarm;

    std::vector<short> kaSpeed;
    std::vector<short> kaCurrent;
    std::vector<BYTE> kaType;
    std::vector<short> kaRefSpeed;
    std::vector<WORD> kaCtrlOutput;
    std::vector<BYTE> kaStatus1;
    std::vector<int> kaPosition;
    std::vector<BYTE> kaBrake;
    std::vector<BYTE> kaTemp;
    std::vector<ros::Time> kaTxTsMainOld;
    std::vector<ros::Time> kaTxTsMain;
    std::vector<ros::Time> kaRxTsMainOld;
    std::vector<ros::Time> kaRxTsMain;
    std::vector<ros::Time> kaRxTsIo;
    std::vector<ros::Time> kaRxTsHoming;
    std::vector<int> kaSetPosition;
    std::vector<BYTE> kaStatus2;
    std::vector<BYTE> kaCtrlInput;
    std::vector<BYTE> ka8PinDip;
    std::vector<BYTE> kaHallsensor;
    std::vector<WORD> kaExtVolume;
    std::vector<BYTE> kaSwInput;
    std::vector<WORD> kaMainVol;
    std::vector<BYTE> kaSlowStart;
    std::vector<BYTE> kaSlowDown;
    std::vector<BYTE> kaIntVolume;

    std::vector<BYTE> kaHomingDone;
    string kaCom;

    public:
    Communication() {
    }

    Communication(int motor_num) : byPacketNum(0) {
        kaSpeed = std::vector<short>(motor_num);
        kaCurrent = std::vector<short>(motor_num);
        kaType = std::vector<BYTE>(motor_num);
        kaRefSpeed = std::vector<short>(motor_num);
        kaCtrlOutput = std::vector<WORD>(motor_num);
        kaStatus1 = std::vector<BYTE>(motor_num);
        kaPosition = std::vector<int>(motor_num);
        kaBrake = std::vector<BYTE>(motor_num);
        kaTemp = std::vector<BYTE>(motor_num);
        kaTxTsMainOld = std::vector<ros::Time>(motor_num);
        kaTxTsMain = std::vector<ros::Time>(motor_num);
        kaRxTsMainOld = std::vector<ros::Time>(motor_num);
        kaRxTsMain = std::vector<ros::Time>(motor_num);
        kaRxTsIo = std::vector<ros::Time>(motor_num);
        kaRxTsHoming = std::vector<ros::Time>(motor_num);
        kaSetPosition = std::vector<int>(motor_num);
        kaStatus2 = std::vector<BYTE>(motor_num);
        kaCtrlInput = std::vector<BYTE>(motor_num);
        ka8PinDip = std::vector<BYTE>(motor_num);
        kaHallsensor = std::vector<BYTE>(motor_num);
        kaExtVolume = std::vector<WORD>(motor_num);
        kaSwInput = std::vector<BYTE>(motor_num);
        kaMainVol = std::vector<WORD>(motor_num);
        kaSlowStart = std::vector<BYTE>(motor_num);
        kaSlowDown = std::vector<BYTE>(motor_num);
        kaIntVolume = std::vector<BYTE>(motor_num);
        kaHomingDone = std::vector<BYTE>(motor_num);
    }

    ~Communication() {
    }
};
extern Communication Com;

typedef struct {
    BYTE byLow;
    BYTE byHigh;
}IByte;

typedef struct _ComData {
    int type;
    int id;
    int pid;
    int rpm;
    int position;
    int nArray[5];
} ComData;

extern IByte Short2Byte(short sIn);
extern int Byte2Short(BYTE byLow, BYTE byHigh);
extern int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4);
extern int MovingAverage(void);

extern int InitSerial(void);
// extern int InitSetSlowStart(void);   // DEPRECATED
// extern int InitSetSlowDown(void);    // DEPRECATED
extern int PutMdData(BYTE byPID, BYTE byMID, ComData comData);
extern int MdReceiveProc(void);
extern int ReceiveDataFromController(void);
extern int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum);

#endif