#include "global.hpp"
#include "main.hpp"
#include "com.hpp"

#include <ros/ros.h>

serial::Serial ser;
//
// Get the low and high byte from short
IByte Short2Byte(short sIn)
{
    IByte Ret;

    Ret.byLow = sIn & 0xff;
    Ret.byHigh = sIn>>8 & 0xff;

    return Ret;
}

// Make short data from two bytes
int Byte2Short(BYTE byLow, BYTE byHigh)
{
    return (byLow | (int)byHigh<<8);
}

// Make long data from four bytes
int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
    return ((int)byData1 | (int)byData2<<8 | (int)byData3<<16 | (int)byData4<<24);
}
//Function for stabilizing position values
int MovingAverage(void)
{
    static BYTE byCnt;

    switch(byCnt){
        case 0:
            Com.lMoving[_X][0]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][0]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][0] = Com.sTempTheta;

            Com.lPosi[_X] = Com.lTempPosi[_X];
            Com.lPosi[_Y] = Com.lTempPosi[_Y];
            Com.lPosi[_THETA] = Com.lTempPosi[_THETA];

            byCnt = 1;
            break;
        case 1:
            Com.lMoving[_X][1]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][1]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][1] = Com.sTempTheta;

            Com.lPosi[_X] = Com.lTempPosi[_X];
            Com.lPosi[_Y] = Com.lTempPosi[_Y];
            Com.lPosi[_THETA] = Com.lTempPosi[_THETA];
            byCnt = 2;
            break;
        case 2:
            Com.lMoving[_X][2]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][2]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][2] = Com.sTempTheta;

            Com.lPosi[_X] = (Com.lMoving[_X][0] + Com.lMoving[_X][1] + Com.lMoving[_X][2])/3;
            Com.lPosi[_Y] = (Com.lMoving[_Y][0] + Com.lMoving[_Y][1] + Com.lMoving[_Y][2])/3;
            Com.sTheta    = (Com.lMoving[_THETA][0] + Com.lMoving[_THETA][1] + Com.lMoving[_THETA][2])/3;

            byCnt = 3;
            break;
        case 3:
            Com.lMoving[_X][0] = Com.lMoving[_X][1];
            Com.lMoving[_X][1] = Com.lMoving[_X][2];

            Com.lMoving[_Y][0] = Com.lMoving[_Y][1];
            Com.lMoving[_Y][1] = Com.lMoving[_Y][2];

            Com.lMoving[_THETA][0] = Com.lMoving[_THETA][1];
            Com.lMoving[_THETA][1] = Com.lMoving[_THETA][2];

            Com.lMoving[_X][2]     = Com.lTempPosi[_X];
            Com.lMoving[_Y][2]     = Com.lTempPosi[_Y];
            Com.lMoving[_THETA][2] = Com.sTempTheta;

            Com.lPosi[_X] = (Com.lMoving[_X][0] + Com.lMoving[_X][1] + (double)Com.lMoving[_X][2])/3;
            Com.lPosi[_Y] = (Com.lMoving[_Y][0] + Com.lMoving[_Y][1] + (double)Com.lMoving[_Y][2])/3;
            Com.sTheta    = (Com.lMoving[_THETA][0] + (double)Com.lMoving[_THETA][1] + Com.lMoving[_THETA][2])/3 ;

            break;
    }
    return SUCCESS;
}

//Initialize serial communication in ROS
int InitSerial(void)
{
    try
    {
        std::string serial_port;
        ros::param::get("~serial_port", serial_port);
        ser.setPort(serial_port.c_str());
        ser.setBaudrate(Com.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(2857); //1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                        //2857 when baud is 115200, 0.35ms
        ser.open();
        ser.flush();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ser.isOpen())
        ROS_INFO_STREAM("Serial Port initialized");
    else
        return -1;
    
    return 0;
}

int InitSetSlowStart(void)
{
    ComData comData;

    Com.fgControlstate = ON;

    comData.nArray[0] = Com.nSlowstart;

    PutMdData(PID_SLOW_START, Com.nRMID, comData);

    return SUCCESS;
}

int InitSetSlowDown(void)
{
    ComData comData;

    Com.fgControlstate = ON;

    comData.nArray[0] = Com.nSlowdown;

    PutMdData(PID_SLOW_DOWN, Com.nRMID, comData);

    return SUCCESS;
}

//for sending the data
int PutMdData(BYTE byPID, BYTE byMID, ComData comData)
{
    static IByte iData;
    static BYTE byPidDataSize, byDataSize, i, j;
    static BYTE byTempDataSum;
    static char* pPosition;
    static char* pRpm;

    for(j = 0; j <MAX_PACKET_SIZE; j++) Com.bySndBuf[j] = 0;

    Com.bySndBuf[0] = byMID;
    Com.bySndBuf[1] = Com.nIDPC;
    Com.bySndBuf[2] = comData.id;
    Com.bySndBuf[3] = byPID;

    switch(byPID){
		// 데이터 요청
        case PID_REQ_PID_DATA:          //PID_NUMBER -> 4

            byDataSize      = 1;
            byPidDataSize   = 7;
            byTempDataSum   = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)comData.nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            #if 0
            for (int i=0; i<byPidDataSize; i++) {
                printf("[%02x]", Com.bySndBuf[i]);
            }
            printf("\n");
            #endif

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 데이터 요청
        case PID_TQ_OFF:
        case PID_BRAKE:
            byDataSize      = 1;
            byPidDataSize   = 7;
            byTempDataSum   = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)comData.nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            #if 0
            for (int i=0; i<byPidDataSize; i++) {
                printf("[%02x]", Com.bySndBuf[i]);
            }
            printf("\n");
            #endif

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 명령 요청
        case PID_COMMAND:

            byDataSize      = 1;
            byPidDataSize   = 7;
            byTempDataSum   = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)comData.nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 위치값 리셋
        case PID_POSI_RESET:

            byDataSize    = 1;
            byPidDataSize = 7;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            #if 0
            // DEPRECATED
            Com.bySndBuf[5] = (BYTE)nArray[0];
            #else
            Com.bySndBuf[5] = 0;
            #endif

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
        #if 0
        // DEPRECATED
		// 위치값 셋
        case PID_POSI_SET:
            pPosition = (char*)&(nArray[0]);

            byDataSize    = 4;
            byPidDataSize = 10;
            byTempDataSum = 0;

            Com.bySndBuf[2] = (BYTE)nArray[2];

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)pPosition[0];
            Com.bySndBuf[6] = (BYTE)pPosition[1];
            Com.bySndBuf[7] = (BYTE)pPosition[2];
            Com.bySndBuf[8] = (BYTE)pPosition[3];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
        #endif
		// 슬로우 스타트 변수(속도를 증가시킬때 적용됨)를 설정
        case PID_SLOW_START:

            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            iData = Short2Byte((short)comData.nArray[0]);
            Com.bySndBuf[5] = iData.byLow;
            Com.bySndBuf[6] = iData.byHigh;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 슬로우 다운 변수(속도를 감소시킬때 적용됨)를 설정
        case PID_SLOW_DOWN:

            byDataSize    = 2;
            byPidDataSize = 8;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            iData = Short2Byte((short)comData.nArray[0]);
            Com.bySndBuf[5] = iData.byLow;
            Com.bySndBuf[6] = iData.byHigh;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 듀얼 모터드라이버의 속도 제어
        case PID_PNT_VEL_CMD:

            byDataSize    = 7;
            byPidDataSize = 13;
            byTempDataSum = 0;

            #if 0
            // DEPRECATED
            // Com.bySndBuf[4]  = byDataSize;
            // Com.bySndBuf[5]  = ENABLE;
            // Com.bySndBuf[6]  = nArray[0];
            // Com.bySndBuf[7]  = nArray[1];
            // Com.bySndBuf[8]  = ENABLE;
            // Com.bySndBuf[9]  = nArray[2];
            // Com.bySndBuf[10] = nArray[3];
            // Com.bySndBuf[11] = nArray[4];

            Com.bySndBuf[2]  = nArray[2];

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = nArray[2];
            Com.bySndBuf[6]  = nArray[0];
            Com.bySndBuf[7]  = nArray[1];
            Com.bySndBuf[8]  = 0;
            Com.bySndBuf[9]  = 0;
            Com.bySndBuf[10] = 0;
            Com.bySndBuf[11] = nArray[4];
            #else
            iData = Short2Byte((short)comData.rpm);

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = comData.id;
            Com.bySndBuf[6]  = iData.byLow;
            Com.bySndBuf[7]  = iData.byHigh;
            Com.bySndBuf[8]  = 0;
            Com.bySndBuf[9]  = 0;
            Com.bySndBuf[10] = 0;
            Com.bySndBuf[11] = REQUEST_PNT_MAIN_DATA;
            #endif

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            #if 0
            printf("send: ");
            for (int idx=0; idx<byPidDataSize; idx++) {
                printf("%02x ", Com.bySndBuf[idx]);
            }
            printf("\n");
            #endif

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 듀얼 모터드라이버의 위치 제어
        case PID_PNT_POS_VEL_CMD:

            byDataSize    = 15;
            byPidDataSize = 21;
            byTempDataSum = 0;

            iData = Short2Byte((short)comData.rpm);

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = comData.id;
            pPosition = (char*)&comData.position;
            Com.bySndBuf[6]  = pPosition[0];
            Com.bySndBuf[7]  = pPosition[1];
            Com.bySndBuf[8]  = pPosition[2];
            Com.bySndBuf[9]  = pPosition[3];
            pRpm = (char*)&comData.rpm;
            Com.bySndBuf[10]  = pRpm[0];
            Com.bySndBuf[11]  = pRpm[1];
            Com.bySndBuf[12]  = 0;
            Com.bySndBuf[13]  = 0;
            Com.bySndBuf[14] = 0;
            Com.bySndBuf[15]  = 0;
            Com.bySndBuf[16]  = 0;
            Com.bySndBuf[17] = 0;
            Com.bySndBuf[18] = 0;
            Com.bySndBuf[19] = REQUEST_PNT_MAIN_DATA;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            #if 0
            printf("send: ");
            for (int idx=0; idx<byPidDataSize; idx++) {
                printf("%02x ", Com.bySndBuf[idx]);
            }
            printf("\n");
            #endif

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 위치값 셋
        case PID_POSI_SET:
            static char* pPosition;
            pPosition = (char*)&(comData.position);

            byDataSize    = 4;
            byPidDataSize = 10;
            byTempDataSum = 0;

            Com.bySndBuf[4] = byDataSize;
            Com.bySndBuf[5] = (BYTE)pPosition[0];
            Com.bySndBuf[6] = (BYTE)pPosition[1];
            Com.bySndBuf[7] = (BYTE)pPosition[2];
            Com.bySndBuf[8] = (BYTE)pPosition[3];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
        #if 0
        // DEPRECATED
		// ?? 데이트시트상에 없음
        case PID_ROBOT_PARAM:

            byDataSize       = 6;
            byPidDataSize    = 12;
            byTempDataSum    = 0;

            Com.bySndBuf[4]  = byDataSize;

            iData = Short2Byte((short)nArray[0]);  //diameter
            Com.bySndBuf[5]  = iData.byLow;
            Com.bySndBuf[6]  = iData.byHigh;

            iData = Short2Byte((short)nArray[1]);  //wheellength
            Com.bySndBuf[7]  = iData.byLow;
            Com.bySndBuf[8]  = iData.byHigh;

            iData = Short2Byte((short)nArray[2]);  //reduction ratio
            Com.bySndBuf[9]  = iData.byLow;
            Com.bySndBuf[10] = iData.byHigh;

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// ?? 데이트시트상에 없음
        case PID_ROBOT_CMD:            //PID_NUMBER -> 247

            byDataSize       = 6;
            byPidDataSize    = 12;
            byTempDataSum    = 0;

            Com.bySndBuf[4]  = byDataSize;

            Com.bySndBuf[5]  = (BYTE)nArray[0];   //kind of control

            iData = Short2Byte((short)nArray[1]); //linear velocity(mm/s)
            Com.bySndBuf[6]  = iData.byLow;
            Com.bySndBuf[7]  = iData.byHigh;

            iData = Short2Byte((short)nArray[2]); //angular velocity(deg/s)
            Com.bySndBuf[8]  = iData.byLow;
            Com.bySndBuf[9]  = iData.byHigh;

            Com.bySndBuf[10] = (BYTE)nArray[3];   //reset encoder value(x, y, theta)

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);
            break;
		// 토크 해제
        case PID_PNT_TQ_OFF:

            byDataSize    = 3;
            byPidDataSize = 9;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];
            Com.bySndBuf[7]  = (BYTE)nArray[2];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
		// 브레이크 명령
        case PID_PNT_BRAKE:

            byDataSize    = 3;
            byPidDataSize = 9;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = (BYTE)nArray[0];
            Com.bySndBuf[6]  = (BYTE)nArray[1];
            Com.bySndBuf[7]  = (BYTE)nArray[2];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;
        #endif
        default:
            printf("Unknown send PID : %d\n", byPID);
            break;
    }

    return SUCCESS;
}

#if 0
// DEPRECATED
long *GetMdData(BYTE byPID)
{
    long *lArray = (long*)malloc(sizeof(long) * 8);

    switch(byPID) {
        case PID_ROBOT_PARAM:
            lArray[0] = (long)Com.sSetDia;
            lArray[1] = (long)Com.sSetWheelLen;
            lArray[2] = (long)Com.sSetGear;

            break;
        case PID_ROBOT_MONITOR2:
            lArray[0] = (long)Com.sVoltIn;
            lArray[1] = (long)Com.byUS1;
            lArray[2] = (long)Com.byUS2;
            lArray[3] = (long)Com.byUS3;
            lArray[4] = (long)Com.byUS4;
            lArray[5] = (long)Com.byPlatStatus;
            lArray[6] = (long)Com.byDocStatus;
            break;

        case PID_ROBOT_MONITOR:
            lArray[0] = Com.lPosi[_X];
            lArray[1] = Com.lPosi[_Y];
            lArray[2] = Com.sTheta;

            break;
    }
    return lArray;
}
#endif

// 확인완료
int MdReceiveProc(void) //save the identified serial data to defined variable according to PID NUMBER data
{
    static short kaspeed;
    static short kacurrent;
    static short karefspeed;
    static int kaposition;

    BYTE byRcvRMID, byRcvTMID, byRcvID, byRcvPID, byRcvDataSize;

    byRcvRMID     = Com.byRcvBuf[0];
    byRcvTMID     = Com.byRcvBuf[1];
    byRcvID       = Com.byRcvBuf[2];
    byRcvPID      = Com.byRcvBuf[3];
    byRcvDataSize = Com.byRcvBuf[4];

    #if 0
    printf("recv: ");
    for (int idx=0; idx<byRcvDataSize; idx++) {
        printf("%02x ", Com.byRcvBuf[idx]);
    }
    printf("\n");
    #endif

	// 파라미터 ID에 따른 처리 실시
    switch(byRcvPID){
        case PID_VER:
            #if 0
            printf("PID_VER: %d\n", Com.byRcvBuf[5]);
            #endif
            break;
        case PID_ACK:
            #if 0
            printf("PID_ACK: %d\n", Com.byRcvBuf[5]);
            #endif
            break;
        #if 0
        // DEPRECATED
        case PID_VOLT_IN: //143

            Com.sTempVoltIn = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.sSumVolt += Com.sTempVoltIn;

            if(++Com.byCntVoltAver == 10)
            {
                Com.byCntVoltAver = 0;
                Com.sVoltIn = (Com.sSumVolt/10) + 7;
                Com.sSumVolt = 0;
            }
            break;
        #endif
        // 모터 드라이버를 위한 데이터
        case PID_INIT_SET_OK: //87
            Com.kaTsLastHoming[byRcvID-ID_OFFSET] = ros::Time::now();
            Com.kaHomingDone[byRcvID-ID_OFFSET] = Com.byRcvBuf[5];

            #ifdef KIRO_MSG
            switch(byRcvID-ID_OFFSET) {
                case OUTTRIGGER_FRONT_LEFT:
                    outtriggerInfos.frontLeft.Header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.frontLeft.Homming = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                case OUTTRIGGER_FRONT_RIGHT:
                    outtriggerInfos.frontRight.Header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.frontRight.Homming = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                case OUTTRIGGER_BACK_LEFT:
                    outtriggerInfos.backLeft.Header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.backLeft.Homming = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                case OUTTRIGGER_BACK_RIGHT:
                    outtriggerInfos.backRight.Header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.backRight.Homming = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                default:
                    break;
            }
            #else
            switch(byRcvID-ID_OFFSET) {
                case OUTTRIGGER_FRONT_LEFT:
                    outtriggerInfos.frontLeft.header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.frontLeft.homing = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                case OUTTRIGGER_FRONT_RIGHT:
                    outtriggerInfos.frontRight.header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.frontRight.homing = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                case OUTTRIGGER_BACK_LEFT:
                    outtriggerInfos.backLeft.header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.backLeft.homing = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                case OUTTRIGGER_BACK_RIGHT:
                    outtriggerInfos.backRight.header.stamp = Com.kaTsLastHoming[byRcvID-ID_OFFSET];
                    outtriggerInfos.backRight.homing = Com.kaHomingDone[byRcvID-ID_OFFSET];
                    break;
                default:
                    break;
            }
            #endif

            #if 0
            printf("received PID_INIT_SET_OK : %d\n", Com.byRcvBuf[5]);
            #endif
            break;
        // 모터 드라이버를 위한 데이터
        case PID_MAIN_DATA: //193
            Com.kaTsLast[byRcvID-ID_OFFSET] = ros::Time::now();
            
            kaspeed                                     = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            kacurrent                                   = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.kaType[byRcvID-ID_OFFSET]          = Com.byRcvBuf[9];
            karefspeed                                  = Byte2Short(Com.byRcvBuf[10], Com.byRcvBuf[11]);
            Com.kaCtrlOutput[byRcvID-ID_OFFSET]    = Byte2Short(Com.byRcvBuf[12], Com.byRcvBuf[13]);
            Com.kaStatus1[byRcvID-ID_OFFSET]        = Com.byRcvBuf[14];
            kaposition                                  = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
            Com.kaBrake[byRcvID-ID_OFFSET]         = Com.byRcvBuf[19];
            Com.kaTemp[byRcvID-ID_OFFSET]          = Com.byRcvBuf[20];
            Com.kaStatus2[byRcvID-ID_OFFSET]          = Com.byRcvBuf[21];
            Com.kaSpeed[byRcvID-ID_OFFSET] = kaspeed * inv_motor_in_arr[byRcvID-ID_OFFSET];
            Com.kaCurrent[byRcvID-ID_OFFSET] = kacurrent * inv_motor_in_arr[byRcvID-ID_OFFSET];
            Com.kaRefSpeed[byRcvID-ID_OFFSET] = karefspeed * inv_motor_in_arr[byRcvID-ID_OFFSET];
            Com.kaPosition[byRcvID-ID_OFFSET] = kaposition * inv_motor_in_arr[byRcvID-ID_OFFSET];

#define STATUS1_BIT0_ALARM 0
#define STATUS1_BIT1_CTRL_FAIL 1
#define STATUS1_BIT2_OVER_VOLT 2
#define STATUS1_BIT3_OVER_TEMP 3
#define STATUS1_BIT4_OVER_LOAD 4
#define STATUS1_BIT5_HALL_FAIL 5
#define STATUS1_BIT6_INV_VEL 6
#define STATUS1_BIT7_STALL 7
#define STATUS1_BIT_LEN (STATUS1_BIT7_STALL+1)
            const static char* STATUS_ERROR[] = {"ALARM", "CTRL_FAIL", "OVER_VOLT", "OVER_TEMP", "OVER_LOAD", "HALL_FAIL", "INV_VEL", "STALL"};

            static double mm_out;
            static double mm_sec_out;
            
            #ifdef KIRO_MSG
            switch(byRcvID-ID_OFFSET) {
                case OUTTRIGGER_FRONT_LEFT:
                    outtriggerStates.frontLeft.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.id = byRcvID;
                    outtriggerStates.frontLeft.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.frontLeft.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.frontLeft.Position = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.frontLeft.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.frontLeft.Velocity = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.frontLeft.condition1) {
                        outtriggerInfos.frontLeft.State = 1;
                        printf("[%lf]outtriggerStates.frontLeft.condition1 error : 0x%02x\n", outtriggerStates.frontLeft.header.stamp.toSec(), outtriggerStates.frontLeft.condition1);
                        printf("outtriggerStates.frontLeft.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.frontLeft.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                case OUTTRIGGER_FRONT_RIGHT:
                    outtriggerStates.frontRight.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.id = byRcvID;
                    outtriggerStates.frontRight.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.frontRight.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.frontRight.Position = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.frontRight.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.frontRight.Velocity = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.frontRight.condition1) {
                        outtriggerInfos.frontRight.State = 1;
                        printf("[%lf]outtriggerStates.frontRight.condition1 error : 0x%02x\n", outtriggerStates.frontRight.header.stamp.toSec(), outtriggerStates.frontRight.condition1);
                        printf("outtriggerStates.frontRight.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.frontRight.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                case OUTTRIGGER_BACK_LEFT:
                    outtriggerStates.backLeft.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.id = byRcvID;
                    outtriggerStates.backLeft.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.backLeft.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.backLeft.Position = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.backLeft.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.backLeft.Velocity = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.backLeft.condition1) {
                        outtriggerInfos.backLeft.State = 1;
                        printf("[%lf]outtriggerStates.backLeft.condition1 error : 0x%02x\n", outtriggerStates.backLeft.header.stamp.toSec(), outtriggerStates.backLeft.condition1);
                        printf("outtriggerStates.backLeft.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.backLeft.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                case OUTTRIGGER_BACK_RIGHT:
                    outtriggerStates.backRight.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.id = byRcvID;
                    outtriggerStates.backRight.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.backRight.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.backRight.Position = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.backRight.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.backRight.Velocity = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.backRight.condition1) {
                        outtriggerInfos.backRight.State = 1;
                        printf("[%lf]outtriggerStates.backRight.condition1 error : 0x%02x\n", outtriggerStates.backRight.header.stamp.toSec(), outtriggerStates.backRight.condition1);
                        printf("outtriggerStates.backRight.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.backRight.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                default:
                    break;
            }
            #else
            switch(byRcvID-ID_OFFSET) {
                case OUTTRIGGER_FRONT_LEFT:
                    outtriggerStates.frontLeft.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.id = byRcvID;
                    outtriggerStates.frontLeft.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.frontLeft.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.frontLeft.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.frontLeft.mm = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.frontLeft.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.frontLeft.mm_per_sec = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.frontLeft.condition1) {
                        outtriggerInfos.frontLeft.state = 1;
                        printf("[%lf]outtriggerStates.frontLeft.condition1 error : 0x%02x\n", outtriggerStates.frontLeft.header.stamp.toSec(), outtriggerStates.frontLeft.condition1);
                        printf("outtriggerStates.frontLeft.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.frontLeft.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                case OUTTRIGGER_FRONT_RIGHT:
                    outtriggerStates.frontRight.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.id = byRcvID;
                    outtriggerStates.frontRight.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.frontRight.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.frontRight.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.frontRight.mm = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.frontRight.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.frontRight.mm_per_sec = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.frontRight.condition1) {
                        outtriggerInfos.frontRight.state = 1;
                        printf("[%lf]outtriggerStates.frontRight.condition1 error : 0x%02x\n", outtriggerStates.frontRight.header.stamp.toSec(), outtriggerStates.frontRight.condition1);
                        printf("outtriggerStates.frontRight.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.frontRight.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                case OUTTRIGGER_BACK_LEFT:
                    outtriggerStates.backLeft.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.id = byRcvID;
                    outtriggerStates.backLeft.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.backLeft.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.backLeft.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.backLeft.mm = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.backLeft.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.backLeft.mm_per_sec = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.backLeft.condition1) {
                        outtriggerInfos.backLeft.state = 1;
                        printf("[%lf]outtriggerStates.backLeft.condition1 error : 0x%02x\n", outtriggerStates.backLeft.header.stamp.toSec(), outtriggerStates.backLeft.condition1);
                        printf("outtriggerStates.backLeft.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.backLeft.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                case OUTTRIGGER_BACK_RIGHT:
                    outtriggerStates.backRight.header.stamp = Com.kaTsLast[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.id = byRcvID;
                    outtriggerStates.backRight.rpm = Com.kaSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.ampere = Com.kaCurrent[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.controlType = Com.kaType[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.refRpm = Com.kaRefSpeed[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.ctrlOutput = Com.kaCtrlOutput[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.condition1 = Com.kaStatus1[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.position = Com.kaPosition[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.brake = Com.kaBrake[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.temperature = Com.kaTemp[byRcvID-ID_OFFSET];
                    outtriggerStates.backRight.condition2 = Com.kaStatus2[byRcvID-ID_OFFSET];

                    mm_out = 0.0;
                    mm_out = outtriggerStates.backRight.position / MOTOR_TICK / GEAR_RATIO * SCREW_LEAD;
                    outtriggerInfos.backRight.mm = mm_out;
                    mm_sec_out = 0.0;
                    mm_sec_out = outtriggerStates.backRight.rpm / GEAR_RATIO * SCREW_LEAD / MIN_TO_SEC;
                    outtriggerInfos.backRight.mm_per_sec = mm_sec_out;
                    #if 0
                    printf("[out] mm: %7.3f, mm_sec: %7.3f\n", mm_out, mm_sec_out);
                    #endif

                    if (outtriggerStates.backRight.condition1) {
                        outtriggerInfos.backRight.state = 1;
                        printf("[%lf]outtriggerStates.backRight.condition1 error : 0x%02x\n", outtriggerStates.backRight.header.stamp.toSec(), outtriggerStates.backRight.condition1);
                        printf("outtriggerStates.backRight.condition1 error list\n");
                        for (int i=0; i<STATUS1_BIT_LEN; i++) {
                            if ((outtriggerStates.backRight.condition1 >> i) & 0x01) {
                                printf("%s, ", STATUS_ERROR[i]);
                            }
                        }
                        printf("\n");
                    } else {
                    }
                    break;
                default:
                    break;
            }
            #endif

            #if 0
            // printf("%s, %d, all ok \n", __FUNCTION__, __LINE__);
            // printf("sped: ");
            // for (int idx=0; idx<MOTOR_NUM; idx++) {
            //     printf("%+05d ", Com.kaSpeed[idx]);
            // }
            // printf("\n");

            // printf("curr: ");
            // for (int idx=0; idx<MOTOR_NUM; idx++) {
            //     printf("%+05d ", Com.kaCurrent[idx]);
            // }
            // printf("\n");

            printf("posi: ");
            for (int idx=0; idx<MOTOR_NUM; idx++) {
                printf("%+010ld ", Com.kaPosition[idx]);
            }
            printf("\n");

            // printf("temp: ");
            // for (int idx=0; idx<MOTOR_NUM; idx++) {
            //     printf("%+05d ", Com.kaTemp[idx]);
            // }
            // printf("\n");

            printf("\n");
            #endif

            #if 0
            // printf("[PID_MAIN_DATA][#%02d] kaType : %d\n", byRcvID, Com.kaType[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaCtrlOutput : %d\n", byRcvID, Com.kaCtrlOutput[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaStatus1 : %d\n", byRcvID, Com.kaStatus1[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaBrake : %d\n", byRcvID, Com.kaBrake[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaTemp : %d\n", byRcvID, Com.kaTemp[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaStatus2 : %d\n", byRcvID, Com.kaStatus2[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaSpeed : %d\n", byRcvID, Com.kaSpeed[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaCurrent : %d\n", byRcvID, Com.kaCurrent[byRcvID-ID_OFFSET]);
            // printf("[PID_MAIN_DATA][#%02d] kaRefSpeed : %d\n", byRcvID, Com.kaRefSpeed[byRcvID-ID_OFFSET]);
            printf("[PID_MAIN_DATA][#%02d] kaPosition : %d\n", byRcvID, Com.kaPosition[byRcvID-ID_OFFSET]);
            #endif

            //ROS_INFO("%d  %d", Com.sCurrent[MOT_LEFT], Com.sCurrent[MOT_RIGHT]);
            //ROS_INFO("%d, %ld, %ld, %d, %ld, %ld, %d", Com.byStatus[MOT_LEFT], (long)Com.sMotorRPM[MOT_LEFT], Com.lMotorPosi[MOT_LEFT],
            //         Com.byStatus[MOT_RIGHT], (long)Com.sMotorRPM[MOT_RIGHT], Com.lMotorPosi[MOT_RIGHT], Com.fgResetEncoder);

            break;
        // 모터 드라이버를 위한 데이터
        case PID_IO_MONITOR: //194
            Com.kaTsLastIo[byRcvID-ID_OFFSET] = ros::Time::now();
            
            kaspeed = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.kaSpeed[byRcvID-ID_OFFSET] = kaspeed * inv_motor_in_arr[byRcvID-ID_OFFSET];
            kacurrent = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.kaCurrent[byRcvID-ID_OFFSET] = kacurrent * inv_motor_in_arr[byRcvID-ID_OFFSET];
            Com.kaStatus1[byRcvID-ID_OFFSET] = Com.byRcvBuf[9];
            Com.kaCtrlInput[byRcvID-ID_OFFSET] = Com.byRcvBuf[10];
            Com.kaExtVolume[byRcvID-ID_OFFSET] = Byte2Short(Com.byRcvBuf[11], Com.byRcvBuf[12]);
            Com.ka8PinDip[byRcvID-ID_OFFSET] = Com.byRcvBuf[13];
            Com.kaHallsensor[byRcvID-ID_OFFSET] = Com.byRcvBuf[14];
            Com.kaStatus2[byRcvID-ID_OFFSET] = Com.byRcvBuf[15];
            Com.kaSwInput[byRcvID-ID_OFFSET] = Com.byRcvBuf[16];
            Com.kaMainVol[byRcvID-ID_OFFSET] = Byte2Short(Com.byRcvBuf[17], Com.byRcvBuf[18]);
            Com.kaSlowStart[byRcvID-ID_OFFSET] = Com.byRcvBuf[19];
            Com.kaSlowDown[byRcvID-ID_OFFSET] = Com.byRcvBuf[20];
            Com.kaIntVolume[byRcvID-ID_OFFSET] = Com.byRcvBuf[21];

            #if 0
            // printf("[PID_IO_MONITOR][#%02d] kaSpeed : %d\n", byRcvID, Com.kaSpeed[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaCurrent : %d\n", byRcvID, Com.kaCurrent[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaStatus1 : %d\n", byRcvID, Com.kaStatus1[byRcvID-ID_OFFSET]);
            printf("[PID_IO_MONITOR][#%02d] kaCtrlInput : %d\n", byRcvID, Com.kaCtrlInput[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaExtVolume : %d\n", byRcvID, Com.kaExtVolume[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] ka8PinDip : %d\n", byRcvID, Com.ka8PinDip[byRcvID-ID_OFFSET]);
            printf("[PID_IO_MONITOR][#%02d] kaHallsensor : %d\n", byRcvID, Com.kaHallsensor[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaStatus2 : %d\n", byRcvID, Com.kaStatus2[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaSwInput : %d\n", byRcvID, Com.kaSwInput[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaMainVol : %d\n", byRcvID, Com.kaMainVol[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaSlowStart : %d\n", byRcvID, Com.kaSlowStart[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaSlowDown : %d\n", byRcvID, Com.kaSlowDown[byRcvID-ID_OFFSET]);
            // printf("[PID_IO_MONITOR][#%02d] kaIntVolume : %d\n", byRcvID, Com.kaIntVolume[byRcvID-ID_OFFSET]);
            #endif

            #if 0
            // printf("%s, %d, all ok \n", __FUNCTION__, __LINE__);
            // printf("sped: ");
            // for (int idx=0; idx<MOTOR_NUM; idx++) {
            //     printf("%+05d ", Com.kaSpeed[idx]);
            // }
            // printf("\n");

            // printf("curr: ");
            // for (int idx=0; idx<MOTOR_NUM; idx++) {
            //     printf("%+05d ", Com.kaCurrent[idx]);
            // }
            // printf("\n");

            printf("posi: ");
            for (int idx=0; idx<MOTOR_NUM; idx++) {
                printf("%+010ld ", Com.kaPosition[idx]);
            }
            printf("\n");

            // printf("temp: ");
            // for (int idx=0; idx<MOTOR_NUM; idx++) {
            //     printf("%+05d ", Com.kaTemp[idx]);
            // }
            // printf("\n");

            printf("\n");
            #endif

            //ROS_INFO("%d  %d", Com.sCurrent[MOT_LEFT], Com.sCurrent[MOT_RIGHT]);
            //ROS_INFO("%d, %ld, %ld, %d, %ld, %ld, %d", Com.byStatus[MOT_LEFT], (long)Com.sMotorRPM[MOT_LEFT], Com.lMotorPosi[MOT_LEFT],
            //         Com.byStatus[MOT_RIGHT], (long)Com.sMotorRPM[MOT_RIGHT], Com.lMotorPosi[MOT_RIGHT], Com.fgResetEncoder);

            break;
        #if 0
        // DEPRECATED
        // 듀얼 모터 드라이버를 위한 데이터
        case PID_PNT_MAIN_DATA: //210
            Com.sMotorRPM[MOT_LEFT]  = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.sCurrent[MOT_LEFT]   = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.byStatus[MOT_LEFT]   = Com.byRcvBuf[9];
            Com.fgAlarm[MOT_LEFT]    = Com.byRcvBuf[9] & BIT0;
            Com.fgCtrlFail[MOT_LEFT] = Com.byRcvBuf[9] & BIT1;
            Com.fgOverVolt[MOT_LEFT] = Com.byRcvBuf[9] & BIT2;
            Com.fgOverTemp[MOT_LEFT] = Com.byRcvBuf[9] & BIT3;
            Com.fgOverLoad[MOT_LEFT] = Com.byRcvBuf[9] & BIT4;
            Com.fgHallFail[MOT_LEFT] = Com.byRcvBuf[9] & BIT5;
            Com.fgInvVel[MOT_LEFT]   = Com.byRcvBuf[9] & BIT6;
            Com.fgStall[MOT_LEFT]    = Com.byRcvBuf[9] & BIT7;

            Com.lMotorPosi[MOT_LEFT] = Byte2LInt(Com.byRcvBuf[10], Com.byRcvBuf[11],
                Com.byRcvBuf[12], Com.byRcvBuf[13]);

            Com.sMotorRPM[MOT_RIGHT]  = Byte2Short(Com.byRcvBuf[14], Com.byRcvBuf[15]);
            Com.sCurrent[MOT_RIGHT]   = Byte2Short(Com.byRcvBuf[16], Com.byRcvBuf[17]);
            Com.byStatus[MOT_RIGHT]   = Com.byRcvBuf[18];
            Com.fgAlarm[MOT_RIGHT]    = Com.byRcvBuf[18] & BIT0;
            Com.fgCtrlFail[MOT_RIGHT] = Com.byRcvBuf[18] & BIT1;
            Com.fgOverVolt[MOT_RIGHT] = Com.byRcvBuf[18] & BIT2;
            Com.fgOverTemp[MOT_RIGHT] = Com.byRcvBuf[18] & BIT3;
            Com.fgOverLoad[MOT_RIGHT] = Com.byRcvBuf[18] & BIT4;
            Com.fgHallFail[MOT_RIGHT] = Com.byRcvBuf[18] & BIT5;
            Com.fgInvVel[MOT_RIGHT]   = Com.byRcvBuf[18] & BIT6;
            Com.fgStall[MOT_RIGHT]    = Com.byRcvBuf[18] & BIT7;

            Com.lMotorPosi[MOT_RIGHT] = Byte2LInt(Com.byRcvBuf[19], Com.byRcvBuf[20],
                    Com.byRcvBuf[21], Com.byRcvBuf[22]);

            //ROS_INFO("%d  %d", Com.sCurrent[MOT_LEFT], Com.sCurrent[MOT_RIGHT]);
            //ROS_INFO("%d, %ld, %ld, %d, %ld, %ld, %d", Com.byStatus[MOT_LEFT], (long)Com.sMotorRPM[MOT_LEFT], Com.lMotorPosi[MOT_LEFT],
            //         Com.byStatus[MOT_RIGHT], (long)Com.sMotorRPM[MOT_RIGHT], Com.lMotorPosi[MOT_RIGHT], Com.fgResetEncoder);
            break;
		// ?? 데이터시트상에 없음
        case PID_ROBOT_PARAM:  //247
            Com.sSetDia      = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.sSetWheelLen = Byte2Short(Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.sSetGear     = Byte2Short(Com.byRcvBuf[9], Com.byRcvBuf[10]);
            break;
		// ?? 데이터시트상에 없음
        case PID_ROBOT_MONITOR2:   //224
            //Com.sVoltIn      = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Com.byUS1        = Com.byRcvBuf[7];
            Com.byUS2        = Com.byRcvBuf[8];
            Com.byUS3        = Com.byRcvBuf[9];
            Com.byUS4        = Com.byRcvBuf[10];
            Com.byPlatStatus = Com.byRcvBuf[11];
            Com.bEmerSW      = Com.byRcvBuf[11] & BIT0;
            Com.bBusy        = (Com.byRcvBuf[11] & BIT1)>>1;
            Com.bBumper1     = (Com.byRcvBuf[11] & BIT3)>>3;
            Com.bBumper2     = (Com.byRcvBuf[11] & BIT4)>>4;
            Com.bBumper3     = (Com.byRcvBuf[11] & BIT5)>>5;
            Com.bBumper4     = (Com.byRcvBuf[11] & BIT6)>>6;
            Com.byDocStatus  = Com.byRcvBuf[12];
            Com.bDocComple   = Com.byRcvBuf[12] & BIT0;
            Com.bChargeState = (Com.byRcvBuf[12] & BIT1)>>1;
            Com.bCharComple  = (Com.byRcvBuf[12] & BIT2)>>2;
            Com.bIr1         = (Com.byRcvBuf[12] & BIT4)>>4;
            Com.bIr2         = (Com.byRcvBuf[12] & BIT5)>>5;
            Com.bIr3         = (Com.byRcvBuf[12] & BIT6)>>6;
            Com.bRccState    = (Com.byRcvBuf[12] & BIT7)>>7;
            break;
		// ?? 데이터시트상에 없음
        case PID_ROBOT_MONITOR:   //253
            Com.lTempPosi[_X] = Byte2LInt(Com.byRcvBuf[5], Com.byRcvBuf[6], Com.byRcvBuf[7], Com.byRcvBuf[8]);
            Com.lTempPosi[_Y] = Byte2LInt(Com.byRcvBuf[9], Com.byRcvBuf[10], Com.byRcvBuf[11], Com.byRcvBuf[12]);
            Com.sTempTheta    = Byte2Short(Com.byRcvBuf[13], Com.byRcvBuf[14]);
            break;
        #endif
        case PID_IN_POSITION_OK:
            printf("PID_IN_POSITION_OK : %d\n", Com.byRcvBuf[5]);
            break;
        default:
            printf("Unknown recv PID: %d\n", byRcvPID);
            break;
    }
    return SUCCESS;
}

// 확인완료
int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum) //Analyze the communication data
{
    ros::NodeHandle n;
    ros::Time stamp;


    static BYTE byChkSec;
    static long lExStampSec, lExStampNsec;
    BYTE i, j;

    if(Com.byPacketNum >= MAX_PACKET_SIZE)
    {
        Com.byStep = 0;
        return FAIL;
    }

    for(j = 0; j < byBufNum; j++)
    {
    	// 상태머신으로 동작
        switch(Com.byStep){
            case 0:    //Put the reading machin id after checking the data
            	// 헤더 RMID(183으로 고정되어 있음)
                if(byArray[j] == Com.nIDPC)
                {
                    for(i = 0; i < MAX_PACKET_SIZE; i++) Com.byRcvBuf[i] = 0;
                    Com.fgPacketOK  = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError = 0;

                    Com.byChkSum = byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
					// 정상이면 다음 진행
                    Com.byStep++;
                }
                else
                {
                    Com.byTotalRcvDataNum = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;
                }
                break;
            case 1:    //Put the transmitting machin id after checking the data
            	// 헤더 TMID(MAIN_CTR(128), MMI(172), MOT_CTR(183), BRIDGE_CTR(172))
                if((byArray[j] == Com.nIDMDUI) || (byArray[j] == Com.nIDMDT))
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byChkComError = 0;
					// 정상이면 다음 진행
                    Com.byStep++;
                }
                else
                {
                    Com.byStep      = 0;
                    Com.fgPacketOK  = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;

                }
                break;
            case 2:    //Check ID
            	// 모터드라이버 ID 확인
            	// ID가 1로 미리지정되어 있음 수정해야함
                // printf("motorDriver id : %d \n", byArray[j]);
                if(byArray[j] == (0+ID_OFFSET) || byArray[j] == (1+ID_OFFSET) || byArray[j] == (2+ID_OFFSET) || byArray[j] == (3+ID_OFFSET) || byArray[j] == ID_ALL )
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byChkComError = 0;
					// 정상이면 다음 진행
                    Com.byStep++;
                }
                else
                {
                    Com.byStep = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;
                }
                break;
             case 3:    //Put the PID number into the array
             	// 파라미터 ID 저장
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
			 	// 다음 진행
                Com.byStep++;
                break;

             case 4:    //Put the DATANUM into the array
             	// 데이터의 개수
                Com.byMaxDataNum = byArray[j];
                Com.byDataNum = 0;
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
				// 다음 진행
                Com.byStep++;
                break;

             case 5:    //Put the DATA into the array
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byChkSum += byArray[j];

                if(++Com.byDataNum >= MAX_DATA_SIZE)
                {
                    Com.byStep = 0;
                    Com.byTotalRcvDataNum = 0;
                    break;
                }

				// 데이터 개수만큼 입력되면 다음 진행
                if(Com.byDataNum>= Com.byMaxDataNum) Com.byStep++;
                break;

             case 6:    //Put the check sum after Checking checksum
             	// 체크섬(byChkSum) 확인
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];

				// 체크섬이 정상일 경우 패킷 확인
                if(Com.byChkSum == 0)
                {
                    Com.fgPacketOK   = 1;
                    Com.fgComDataChk = 1;
                    Com.byDataNum    = 0;
                    Com.byMaxDataNum = 0;
                }

                Com.byStep = 0;
                Com.byTotalRcvDataNum = 0;

                break;

            default:
            printf("%s, %d, %s \n", __FUNCTION__, __LINE__, "error");
                Com.byStep = 0;
                Com.fgComComple = ON;
                break;
        }

		// 체크섬 검사가 끝나고 패킷을 확인
        if(Com.fgPacketOK)
        {
            Com.fgPacketOK   = 0;
            Com.byPacketSize = 0;
            Com.byPacketNum  = 0;


            std_msgs::String com_data_string;
            std::stringstream ss;
            stamp = ros::Time::now();

            if(byChkSec == 0)
            {
                byChkSec = 1;
                lExStampSec = stamp.sec;

            }

            stamp.sec = stamp.sec - lExStampSec;
            stamp.nsec = stamp.nsec / 100000;

            lExStampNsec = stamp.nsec;

            #if 0
            printf("recv success(checksum ok)\n");
            #endif
            MdReceiveProc();                                 //save the identified serial data to defined variable
        }

        if(Com.byChkComError == 10) //while 50ms
        {
            Com.byChkComError = 0;
            Com.byStep = 0;
            Com.byChkSum = 0;
            Com.byMaxDataNum = 0;
            Com.byDataNum = 0;
            for(i = 0; i < MAX_PACKET_SIZE; i++) Com.byRcvBuf[i] = 0;
            j = byBufNum;
        }

    }
    return SUCCESS;
}


int ReceiveDataFromController(void) //Analyze the communication data
{
    #define BUFFER_SIZE 512
    static BYTE byRcvBuf[BUFFER_SIZE];
    static BYTE byBufNumber;
    static uint32_t readBytes;

    byBufNumber = ser.available();
    // ROS_INFO("test : %s, %d", __FUNCTION__, __LINE__);

    if(byBufNumber != 0)
    {
        readBytes = ser.read(byRcvBuf, byBufNumber);

        #if 0
        printf("%d, %d, ser.read: ", readBytes, byBufNumber);
        for (int idx=0; idx<byBufNumber; idx++) {
            printf("%02x ", byRcvBuf[idx]);
        }
        printf("\n");
        #endif

        AnalyzeReceivedData(byRcvBuf, byBufNumber);
    }


    return 1;
}

