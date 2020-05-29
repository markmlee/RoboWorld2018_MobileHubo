#ifndef RBFTSENSOR_H
#define RBFTSENSOR_H

#include "RBDataType.h"
#include "RBCAN.h"


class RBFTSensor
{
public:
    RBFTSensor();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     SENSOR_ID;
    int     CAN_CHANNEL;
    int     SENSOR_TYPE;
    int     ID_RCV_DATA1;
    int     ID_RCV_DATA2;
    int     ID_RCV_ACC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;

    // not from DB----
    int     ConnectionStatus;

    // sensor data----
    double   MX;
    double   MX_FILTERED;
    double   MY;
    double   MY_FILTERED;
    double   MZ;
    double   MZ_FILTERED;
    double   FX;
    double   FX_FILTERED;
    double   FY;
    double   FY_FILTERED;
    double   FZ;
    double   FZ_FILTERED;
    double   AccRoll;
    double   AccRollOld;
    double   dAccRoll;
    double   dAccRollOld;
    double   dAccRoll_Offset;
    double   AccPitch;
    double   AccPitchOld;
    double   dAccPitch;
    double   dAccPitchOld;
    double   dAccPitch_Offset;
    double   VelRoll;
    double   VelRollOld;
    double   VelPitch;
    double   VelPitchOld;
    double   ROLL;
    double   PITCH;

    // sensor setting----
    double   CutOffFeq;
    double   SFRoll;
    double   SFPitch;



    void    RBFT_AddCANMailBox();
    void    RBBoard_GetDBData(DB_FT db);

    int     RBBoard_CANCheck(int _canr);
    int     RBBoard_RequestStatus(void);
    int     RBBoard_LoadDefaultValue(void);
    int     RBFT_Nulling(int _mode);
    int     RBFT_SetCoefficient0(int _coeff1, int _coeff2, int _coeff3);
    int     RBFT_SetCoefficient1(int _coeff1, int _coeff2, int _coeff3);
    int     RBFT_SetCoefficient2(int _coeff1, int _coeff2, int _coeff3);
    int     RBFT_SetInclinometerSF(int _sf1, int _sf2, int _sf3);
    int     RBFT_SetBoardNumberAndFilterFrequency(int _newbno, int _freq);
    int     RBFT_Initialize(void);
    int     RBFT_RequestCoefficient(int _para);
    int     RBFT_RequestData(int _mode);
    int     RBFT_ReadData(void);

};

#endif // RBFTSENSOR_H
