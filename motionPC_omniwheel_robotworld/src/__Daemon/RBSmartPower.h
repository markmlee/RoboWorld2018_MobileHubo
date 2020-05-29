#ifndef RBSMARTPOWER_H
#define RBSMARTPOWER_H

#include "RBDataType.h"
#include "RBCAN.h"


class RBSmartPower
{
public:
    RBSmartPower();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     CAN_CHANNEL;
    int     ID_RCV_DATA;
    int     ID_RCV_INFO;
    int     ID_SEND_GENERAL;

    // not from DB----
    int     ConnectionStatus;
    float   Voltage;
    float   Current;

    void    RBSP_AddCANMailBox();
    void    RBBoard_GetDBData(DB_SP db);

    int     RBBoard_CANCheck(int _canr);
    int     RBBoard_RequestStatus(void);
    int     RBBoard_LoadDefaultValue(void);
    int     RBSP_SetSwitchFunctions(int _sfunc);
    int     RBSP_RequestAlarm(int _alrm);
    int     RBSP_RequestBeep(int _bdur);
    int     RBSP_RequestVoltageAndCurrent(void);
    int     RBSP_RequestTimeAndStatus(void);
    int     RBSP_12VOnOff(int onoff);
    int     RBSP_LCDReset(void);
    int     RBSP_ReadVoltageCurrent();

};

#endif // RBSMARTPOWER_H
