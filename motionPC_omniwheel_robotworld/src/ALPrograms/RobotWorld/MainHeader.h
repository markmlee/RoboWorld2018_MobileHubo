#ifndef MAINHEADER_H
#define MAINHEADER_H
#include <QCoreApplication>

#include <alchemy/task.h>

#include <iostream>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "BasicFiles/joint.h"
#include "BasicFiles/taskmotion.h"
#include "BasicFiles/BasicMatrix.h"
#include "Drinkstock.h"

#ifndef PI
#define PI			3.141592653589793
#endif
#ifndef D2R
#define D2R			1.745329251994330e-2
#endif
#ifndef R2D
#define R2D			5.729577951308232e1
#endif

enum{
    GRASP_NO_ACT = 0, GRASP_STOP, GRASP_START, GRASP_APPROACH, GRASP_GRASP, GRASP_LIFT, GRASP_BACK,
    GRASP_WALKREADY, GRASP_PUT, GRASP_DONE, PUT_START, PUT_READY,PUT_START_GUI, PUT_PUT, PUT_BACK, PUT_WALKREADY, PUT_DONE
};
enum{
    HAND_BOTH = 0, HAND_R, HAND_L
};
enum{
    HAND_NO_ACT = 0, HAND_STOP, HAND_OPEN, HAND_GRASP, HAND_DONE
};
enum{
    NOT_MOVE = 0, MOVE_OMNI, MOVE_JOY_LOCAL, MOVE_JOY_GLOBAL, MOVE_VISION, MOVE_TEST
};
enum{
    WST_STOP = 0, WST_ZERO, WST_BACK
};
enum{
    TimeFix = 0, XFix
};
enum{
    INIT = 0, SHORT, MIDDLE, FAR, TURN_LITTLE, MOVE_LITTLE, TURN_NORMAL, MOVE_NORMAL, TURN_FAR, MOVE_FAR
};
typedef struct _USER_INPUT_PARAMETER{
    /* Distance */
    double Distance_Display2Counter_m = 3.2;
    double Distance_DrinkSpot_m = 0.45;

    double Distance_ObjectApproach_m = 0.17;
    double Distance_GraspLiftZ_m = 0.05;
    double Distance_GraspLiftX_m = -0.17;

    /* Pos of Grasp */
    double Degree_ElbowGrasp = 60.0;
    double Degree_ElbowWalkReady = 5.0;

    double Position_PutX = 0.65;
    double Position_PutY = 0.1;
    double Position_PutZ = 0.9 - 0.485 - 0.08;

    double Position_WalkReadyX = 0.3;
    double Position_WalkReadyY = 0.245;
    double Position_WalkReadyZ = 0.25;

    double Quaternion_RGraspW = 0.5;
    double Quaternion_RGraspX = 0.5;
    double Quaternion_RGraspY = -0.5;
    double Quaternion_RGraspZ = -0.5;

    double Quaternion_LGraspW = 0.5;
    double Quaternion_LGraspX = -0.5;
    double Quaternion_LGraspY = -0.5;
    double Quaternion_LGraspZ = 0.5;

    /* Offset and SafeCheck of Vision */
    double Offset_ObjectX = 0.2;
    double Offset_ObjectY = 0;
    double Offset_ObjectZ = 0.586;
    double Offset_PutZ = 0.0;

    double Offset_SAMDASOOz = 0.01;
    double Offset_SPRITEz = -0.019;
    double Offset_VITAMINz = 0.0;//-0.005;

    double Limit_ObjectXUpper = 1.0;
    double Limit_ObjectXLower = 0.3;
    double Limit_ObjectYUpper = 0.7;
    double Limit_ObjectYLower = -0.05;
    double Limit_ObjectZUpper = 1.0;
    double Limit_ObjectZLower = -0.1;
    /* Limit Gripper */
    double Limit_GripperUpeer = -8.;
    double Limit_GripperLower = 0.;
    double Position_Encoder_SAMDASOO= -1.;//-3.;
    double Position_Encoder_SPRITE  = -1.;//-2.;
    double Position_Encoder_VITAMIN = -1.;//-4.;

    /* Time of Wheel Move*/
    double Time_MoveDisplay_sec = 8.0;//6.0;
    double Time_MoveDrinkSpot_sec = 3.0;//2.0;
    double Time_Compensation_sec = 2.0;

    double Distance_OmniTrajectoryX_m = 1.0;
    double Distance_OmniTrajectoryY_m = 1.0;
    double Distance_OmniTrajectoryR_Rad = 30.0*D2R;
    double Time_OmniTrajectory_sec = 1.5;

    /* Time of Grasp */
    double Time_MoveGraspStart_sec = 1.8;
    double Time_MoveGraspApproach_sec = 1.2;
    double Time_MoveGraspLift_sec = 1.3;
    double Time_MoveWalkReady_sec = 2.0;

    double Time_MovePutStart_sec = 2.0;
    double Time_MovePutBack_sec = 1.5;

    /* OmniWheel profile */
    double Vmax_ms = 0.55;//0.55;//0.57;//1.85;
    double Vmax_rads = 65.0*D2R;

    double Distance_TrajM_m = 0.98;
    double Distance_TrajRad = 30.0*D2R;
} USER_INPUT_PARA;

typedef struct _WHEEL_INFO{
    double InitRef_Deg;
    double WheelVel_ms;
    double MoveDistance_m;
}WheelInfos;

typedef struct _INFO_ROBOTMOVE{
    double X;
    double Y;
    double M;
    double Theta;
}Robot_move;

typedef struct _WHEEL_PARAMETER{
    double alpha1 = 60*D2R;
    double alpha2 = 180*D2R;
    double alpha3 = -60*D2R;

    double WheelRm = 0.127/2;
    double RobotRm = 0.656/2;
}WHEEL_PARAMETER;

typedef struct _IK_HAND_PARA{
    vec3 pos;
    quat ori;
    double elb;
    double sec;
}IKhand;

USER_INPUT_PARA in;
typedef struct _OMNI_PARAMETER{
    Robot_move Pin;
    Robot_move Probot;
    Robot_move Vrobot;
    double CurSec;
    double GoalSec;
    double GoalSecR;
    double TrajSec;
    double TrajSecR;
    double SatSecR;
    double SatSec;
    double TickSec = 0.005;
    double RatioTraj = 0.2;
    double move_theta = 0.;

    /* Profile by HD */
    Robot_move Pc;  //move distance during acc&dec
    double Vms = in.Vmax_ms;
    double Vrads = in.Vmax_rads;
    double Trajm = in.Distance_TrajM_m;
    double Trajrad = in.Distance_TrajRad;
    double TrajT = in.Time_OmniTrajectory_sec;
}OMNI_PARAMETER;

typedef struct _ROBOT_STATE
{
    Robot_move SLAMPos;
    Robot_move GoalPos;
    Robot_move DiffPos;
}ROBOT_STATE;

WHEEL_PARAMETER Pw;
OMNI_PARAMETER OMNIinfo;
ROBOT_STATE    ROBOTinfo;

IKhand des_RH;
IKhand des_LH;

WheelInfos RWHinfo;
WheelInfos LWHinfo;
WheelInfos BWHinfo;

DrinkStock drinkstock;
/* FLAGs */
int Command_Grasp = GRASP_NO_ACT;
int Mode_OMNIMove = NOT_MOVE;
int Mode_HANDMove = NOT_MOVE;
int Mode_PutEarly = false;
int FLAG_PUTtime = false;
int FLAG_JOYStart = false;
int FLAG_OMNI = false;
int Command_WSTrot = WST_STOP;
int FLAG_WSTtime = false;
int FLAG_OMNICompensation = false;
int FLAG_PosReset = false;
int FLAG_RotReset = false;
int FLAG_HAND = HAND_NO_ACT;
int GRASP_HAND = false;
int MOVE_HAND = false;
int FLAG_Localization = false;
int isTerminated;
int PODO_NO;
int PODO_NO_DAEMON = 0;
int PODO_NO_WALKREADY;
int fix = XFix;
int __IS_WORKING = false;
int __IS_GAZEBO = false;
int WB_FLAG = false;
int FLAG_SLAMReset = false;
int OnOff_YawCompen = true;
/* Debug data */
FILE *fp;
#define ROW 50000
#define COL 100
int     Save_Index;
double  Save_Data[COL][ROW];

/* Joystick Move Variables */
doubles RWHList(120);
doubles LWHList(120);
doubles BWHList(120);
double RWHnow, LWHnow, BWHnow;
double RWHvel, LWHvel, BWHvel;

int JOY_LJOG_RL = 0;
int JOY_LJOG_UD = 0;
int JOY_RJOG_RL = 0;
int JOY_START = 0;
int JOY_BACK = 0;

float Kspeed = 0.3;

float Move_X = 0.;
float Move_Y = 0.;
float Move_R = 0.;
//float Gain_R = 0.8;
float Cur_Th = 0.;
float MotorSpeed[3] = {0.0, };
double test = 0.0;
double EncoderGripper = 0.;


double MenuDis = 0.6;
/******************************************** Functions ***********************************************/
void CatchSignals(int _signal);
int HasAnyOwnership();

void RBTaskThread(void *);
void RBFlagThread(void *);
void save();

void ShutDownAllFlag();
int CheckMotionOwned();

int RBInitialize(void);
void InitializeSharedMemory();
void InitializeDrinkStock();
void ResetPos();
void SLAMreset();

void StartWBIKmotion(int _mode);
void InitWheelInfo();

/* Joystick Move */
void CalculateMovingEverage(void);

/* OmniWheel Move */
double Traj(double _cnt, double _vel);

void Omni_CalVel_Robot();
void Omni_CalVel_Robot5th();
void Omni_CalVel_Wheel();
void Omni_CalRef_Motor();

int Omni_CarVel(double &p, double &v, double &donet, double x_i, double x_c, double x_f, double t_c);
/* Motion Thread Functions */
void MotionState_TH();
void HandMove_TH();
void OmniMove_TH();
void Grasping_TH();
void Stock_TH();

void VisionHandMove_TH();
void GO_Grasp();

double CalculateMoveTime(double t1, double t2, double t3);
double CalculateMoveTime(double t1, double t2);
int IsPosXSafe();
int IsPosYSafe();
int IsPosZSafe();
int IsElbAngleSafe();

int IsSLAMXOK();
int IsSLAMYOK();
int IsSLAMYAWOK();

int SetGoalPos();

int IsRealPos();
int SetRotation(int _wst);
void SetOMNIpara(int _mode);
void SetWheelMovePos(double _x, double _y, double _r);
void WheelMove(double _x, double _y, double _r, double _mode);
#endif // MAINHEADER_H

