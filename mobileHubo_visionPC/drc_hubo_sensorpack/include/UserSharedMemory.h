#ifndef USERSHAREDMEMORY_H
#define USERSHAREDMEMORY_H

#define USER_SHM_NAME         "USER_SHARED_MEMORY"

#define NO_SIGNAL       0
#define SPRITE          1
#define VITAMINWATER    2
#define SAMDASOO        3

#ifndef __LAN_STRUCT_GENERAL_COMMAND_DEF__
#define __LAN_STRUCT_GENERAL_COMMAND_DEF__

enum{
    ROBOTWORLD_ROS_NO_ACT = 0,
    ROBOTWORLD_AL_WORKING,
    ROBOTWORLD_ROS_GO2DISPLAY,
    ROBOTWORLD_ROS_GO2GRASPSPOT,
    ROBOTWORLD_ROS_GO2HOME,
    ROBOTWORLD_ROS_GO2DES,
    ROBOTWORLD_ROS_GRASP,
    ROBOTWORLD_ROS_PUT,
    ROBOTWORLD_ROS_E_STOP,
    ROBOTWORLD_ROS_POS_RESET
};

enum{
    NO_ERROR = 0,
    VISION_OBJ_ERROR,
    ROS_E_STOP
};

enum{
    AL_NO_ACT = 0,
    WORKING,
    DONE
};


typedef struct __LAN_STRUCT_GENERAL_COMMAND_
{
    char    param_c[10];
    int     param_i[10];
    float   param_f[10];
    int     cmd;
} LAN_GENERAL_COMMAND, *pLAN_GENERAL_COMMAND;

#endif
typedef struct OBJECT_POS
{
    float           pos_x;
    float           pos_y;
    float           pos_z;
    float           ori_w;
    float           ori_x;
    float           ori_y;
    float           ori_z;
}ObjectPos;

typedef struct _MOTION2GUI_
{
    float   curFootR[6];
    float   curFootL[6];
    float   curZMP[3];
    float   curPEL[3];
    float   _INIT_PEL[3];
    float   _INIT_COM[3];
    float   _ADDCOM;
    float   _qPEL[4];

    float   DRILL_Data[10];

    double  pRF[3];
    double  pLF[3];
    double  pRH[3];
    double  pLH[3];
    double  qRF[4];
    double  qLF[4];
    double  qRH[4];
    double  qLH[4];
    double  qPel[4];
    double  Relb;
    double  Lelb;
    double  rWST;
    double  pCOM[3];
    double  pPelZ;

    int     valveMode;

    //Car Descending
    float ROI_max_X;
    float ROI_min_X;
    float ROI_max_Y;
    float ROI_min_Y;
    float ROI_max_Z;
    float ROI_min_Z;

    double OMNIRobotPosX;
    double OMNIRobotPosY;
    double OMNIRobotPosYaw;
} MOTION2GUI, *pMOTION2GUI;

typedef struct _GUI2MOTION_
{
    double  walkingDSP[400];

    int StepNum;
    double StepLength;
    double StepOffset;
    double StepAngle;
    double StepTime;
    int WalkingModeCommand;
    int WalkingStopModeCommand;
    int WalkingGoalGoModeCommand;
    double GoalPosX;
    double GoalPosY;
    double GoalAngle;

    LAN_GENERAL_COMMAND ros_cmd;
} GUI2MOTION, *pGUI2MOTION;

typedef struct _ROS2MOTION_
{
    int             ROS_COMMAND;
    int             SELECTED_MENU;
    ObjectPos       Objpos[20];
    ObjectPos       robot_pos;
} ROS2MOTION, *pROS2MOTION;

typedef struct _MOTION2ROS_
{
    int             MOTION_ERROR;
    int             MOTION_STATE;
} MOTION2ROS, *pMOTION2ROS;

typedef struct _USER_SHM_
{
    MOTION2GUI  M2G;
    GUI2MOTION  G2M;
    ROS2MOTION  R2M;
    MOTION2ROS  M2R;

    double          WalkReadyCOM[3];
    double          ZMPInitAnlge[50];  // ZMP init. control input
    double          terrain_variable[10];

    int             WheelDoneFlag;
    int             WalkDoneFlag;
    int             EmergencyFlag; // 0=pause 1=resume 2=stop
    int             WheelUpPosDoneFlag;

    int             Laser1Line_raw[1500];
    double          Laser1Line_angle[1500];
    double          Laser1Line_xyz[1500][3];
    int             Laser1Line_size;
    bool            Laser1Line_isUpdated;

    int             Laser2Line_raw[1500];
    double          Laser2Line_angle[1500];
    double          Laser2Line_xyz[1500][3];
    int             Laser2Line_size;
    bool            Laser2Line_isUpdated;

    float           odom_data[6];
    float           vel_cmd[2];

} USER_SHM, *pUSER_SHM;




#endif // USERSHAREDMEMORY_H
