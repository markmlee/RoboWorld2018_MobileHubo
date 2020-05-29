#ifndef RBMOTORCONTROLLER_H
#define RBMOTORCONTROLLER_H

#include "RBDataType.h"
#include "RBCAN.h"

#include <math.h>

#define CONTROL_MODE_POS        0
#define CONTROL_MODE_CUR        2
#define CONTROL_MODE_PWM        3
#define CONTROL_MODE_POS_PWM    4


typedef struct _MOVE_JOINT_
{
    double			RefAngleCurrent;	// reference to move at this step
    double			RefAngleDelta;		// reference of the past step
    double			RefAngleToGo;		// goal position - initial position
    double			RefAngleInitial;	// initial position
    unsigned long	GoalTimeCount;		// the time at which the goal is reached
    unsigned long	CurrentTimeCount;	// current time count
    char			MoveFlag;			// move flag
} MOVE_JOINT, *pMOVE_JOINT;


class RBMotorController
{
public:
    RBMotorController();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     BOARD_TYPE;
    int     TOTAL_CHANNEL;
    int     MOTOR_CHANNEL;
    int     CAN_CHANNEL;
    int     ID_SEND_REF;
    int     ID_RCV_ENC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;
    int     ID_SEND_GENERAL;

    // not from DB----
    int     ConnectionStatus;
    float   BoardTemperature;

    RB_JOINT    Joints[MAX_JOINT];
    MOVE_JOINT  MoveJoints[MAX_JOINT];


    void    RBMC_AddCANMailBox();
    void    RBBoard_GetDBData(DB_MC db);
    void    RBMC_SetFrictionParam();

    void    RBBoard_ReferenceOutEnable(bool _refEnable);
    int     RBBoard_CANCheck(int _canr);
    int     RBBoard_RequestStatus(void);
    int     RBBoard_GetStatus(void);
    int     RBBoard_LoadDefaultValue(void);
    int     RBBoard_SetBoardNumber(int _newbno, int _canr);
    int     RBBoard_RequestEncoder(int mode);
    int     RBBoard_RequestCurrent(void);

    int     RBJoint_ResetEncoder(int ch);
    int     RBJoint_EnableFETDriver(int ch, int enable);
    int     RBJoint_EnableFeedbackControl(int ch, int enable);
    int     RBJoint_FindHome(int ch);
    int     RBJoint_EnableFrictionCompensation(int ch, int enable);
    int     RBJoint_SetMaxDuty(int ch, int duty);
    int     RBJoint_SetPositionCommandMode(int _mode, int _ch);
    int     RBJoint_ClearErrorFlag(int ch);

    int     RBBoard_RequestTemperature();
    int     RBBoard_ReadTemperature(void);
    int     RBBoard_PWMCommand2ch(int mode1, short duty1, int mode2, short duty2);
    int     RBBoard_SetControlMode(int _mode);
    int     RBBoard_SendReference2ch(int ref1, int ref2);
    int     RBBoard_SendReference(void);
    int     RBBoard_ReadEncoderData(void);
    int     RBBoard_SetSwitchingMode(char _mode);
    int     RBBoard_SetFrictionParameter(int ch, short vel, int amp, int dead);

    int     RBBoard_SetMotorPositionGain0(int _kp, int _ki, int _kd);
    int     RBBoard_SetMotorPositionGain1(int _kp, int _ki, int _kd);
    int     RBBoard_SetMotorCurrentGain0(int _kp, int _ki, int _kd);
    int     RBBoard_SetMotorCurrentGain1(int _kp, int _ki, int _kd);
    int     RBBoard_SetJamPwmSat(int _jam, int _sat, int _jamduty, int _satduty);
    int     RBBoard_SetErrorBound(int _ierror, int _berror, int _teeror);
    int     RBBoard_RequestParameter(int _ch, int _type);

    int     RBJoint_SetDeadzone(int _ch, int _deadzone);
    int     RBJoint_SetHomeSearchParameter(int _ch, int _rotlimit, int _dir, int _offset);
    int     RBJoint_SetEncoderResolution(int _ch, int _res, int _auto, int _dir);
    int     RBJoint_SetMaxAccVel(int _ch, int _acc, int _vel);
    int     RBJoint_SetLowerPosLimit(int _ch, int _limit, int _mode);
    int     RBJoint_SetUpperPosLimit(int _ch, int _limit, int _mode);
    int     RBJoint_SetMaxAccVelForHomeSearch(int _ch, int _acc, int _vel1, int _vel2, int _mode, int _duty);
    int     RBJoint_GainOverride(int ch, int logscale, short msec);


    void    RBJoint_SetMoveJoint(int ch, float angle, float timeMs, int mode);
    void    RBJoint_MoveJoint(int ch);
    void    RBBoard_MoveJoint();

    int     RBJoint_LoadParameter(int ch);


    // used only once
    int     CANRate;
    int     Version;

private:
    bool    ReferenceOutEnable;

};

#endif // RBMOTORCONTROLLER_H
