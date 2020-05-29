#ifndef RBIMUSENSOR_H
#define RBIMUSENSOR_H

#include "RBDataType.h"
#include "RBCAN.h"


class RBIMUSensor
{
public:
    RBIMUSensor();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     SENSOR_TYPE;
    int     ID_RCV_DATA1;
    int     ID_RCV_DATA2;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;

    // not from DB----
    int     ConnectionStatus;

    // sensor data----
    // Current raw data
    int     ROLL_DIGIT;
    int     PITCH_DIGIT;
    int     YAW_DIGIT;
    int     ROLL_VEL_DIGIT;
    int     PITCH_VEL_DIGIT;
    int     YAW_VEL_DIGIT;
    // Currnet data
    double   ROLL;
    double   PITCH;
    double   YAW;
    double   ROLL_VEL;
    double   PITCH_VEL;
    double   YAW_VEL;
    double   ACC_X;
    double   ACC_Y;
    double   ACC_Z;
    // Offset value for angles
    double   ROLL_OFFSET;
    double   PITCH_OFFSET;
    double   YAW_OFFSET;

    double   FOG_ROLL_OFFSET;
    double   FOG_PITCH_OFFSET;
    double   FOG_YAW_OFFSET;

    double   FOG_ROLL_NULL;
    double   FOG_PITCH_NULL;
    double   FOG_YAW_NULL;



    void    RBIMU_AddCANMailBox();
    void    RBBoard_GetDBData(DB_IMU db);

    int     RBBoard_CANCheck(int _canr);
    int     RBBoard_RequestStatus(void);
    int     RBBoard_LoadDefaultValue(void);
    int     RBIMU_RequestNulling(void);
    int     RBIMU_RequestCalibration(void);
    int     RBIMU_RequestParameter(unsigned char _prf);
    int     RBIMU_SetBoardNumber(int _newbno);
    int     RBIMU_RequestData(int _type);
    int     RBIMU_ReadData(void);
    int     RBIMU_ReadData_HIB(void);

};

#endif // RBIMUSENSOR_H
