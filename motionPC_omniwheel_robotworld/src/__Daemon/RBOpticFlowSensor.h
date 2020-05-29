#ifndef RBOPTICFLOWSENSOR_H
#define RBOPTICFLOWSENSOR_H

#include "RBDataType.h"
#include "RBCAN.h"


class RBOpticFlowSensor
{
public:
    RBOpticFlowSensor();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     ID_RCV_DATA;
    int     ID_RCV_INFO;

    // not from DB----
    int     ConnectionStatus;

    // sensor data
    int     AccumX;
    int     AccumY;
    short   DeltaX;
    short   DeltaY;


    void    RBOF_AddCANMailBox();
    void    RBBoard_GetDBData(DB_OF db);

    void    RBMC_AddCANMailBox();
    int     RBBoard_CANCheck(int _canr);
    int     RBOF_RequestValue();
    int     RBOF_ReadValue();
    int     RBOF_ResetValue();
    int     RBOF_LampOnOff(int _onoff);

};

//=====================================

class OpticalDisplacement
{
public:
    OpticalDisplacement();

    void    OD_MeasureState();
    void    OD_Zero();

private:
    double Mea_th, Mea_x, Mea_y, Mea_pit;
    double RsideOpt, LsideOpt;
    double RsideOptOld, LsideOptOld;
    double RorthoOpt, LorthoOpt;
    double RorthoOptOld, LorthoOptOld;
};

#endif // RBOPTICFLOWSENSOR_H
