#ifndef WMUPPERBODY_H
#define WMUPPERBODY_H
#include "JoyStick/joystickclass.h"
#include "JoyStick/joystickvariable.h"
//-----------------------------------------------------
// Basic Constant
//-----------------------------------------------------
#ifndef PI
#define PI			3.141592653589793
#endif
#ifndef D2R
#define D2R			1.745329251994330e-2
#endif
#ifndef R2D
#define R2D			5.729577951308232e1
#endif

#define RIGHT_HAND                  0
#define LEFT_HAND                   1
//-----------------------------------------------------
// Basic Constant
//-----------------------------------------------------
enum ManualMove_ALCOMMAND
{
    ManualMove_AL_NO_ACT = 100,
    ManualMove_AL_UPPER_TASK_POSE,
    ManualMove_AL_MANUAL_MODE_START,
    ManualMove_AL_MANUAL_ONE_HAND_STADING_START,
    ManualMove_AL_MANUAL_BOTH_HAND_STADING_START,
    ManualMove_AL_MANUAL_FOOT_MODE_START,
    ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START,
    ManualMove_AL_MANUAL_BOTH_HAND_MODE_START,
    ManualMove_AL_MANUAL_MODE_STOP,
    ManualMove_AL_HAND,
    ManualMove_AL_GAIN,
    ManualMove_AL_E_STOP,
    ManualMove_AL_DRIVE_MODE,
    ManualMove_AL_JOYSTICK_MODE,
    ManualMove_AL_ZERO_GAIN,
    ManualMove_AL_R_GAIN_OVERRIDE,
    ManualMove_AL_L_GAIN_OVERRIDE,
    ManualMove_AL_POSITION_LOCK
};
//-----------------------------------------------------
// Joint space arm pos functions
//-----------------------------------------------------
void Change_Pos_Task(void);
void Change_Pos_Task_BACK(void);
void Change_Pos_Move(void);
void Change_Pos_Move_BACK(void);
//-----------------------------------------------------
// WBIK variables/functions
//-----------------------------------------------------
void StartWBIKmotion(int _mode);
void PrintWBIKinfo(void);
//-----------------------------------------------------
// Save Parameter
//-----------------------------------------------------
unsigned int saveFlag=0;
unsigned int saveIndex=0;
float DataBuf[10][100000];
void SaveFile(void);
unsigned int debugFlag=0;

//-----------------------------------------------------
// Manual Mode Functions and Variables
//-----------------------------------------------------
void ManualMoveHand(void);
void ManualMoveHand_Global_Ori(void);
void ManualMoveFoot(void);
void ManualMoveFoot_Right(void);
void ManualMoveFoot_Left(void);
void ManualMoveHand_Right(void);
void ManualMoveHand_Left(void);
void ManualDriving(void);
void ShutDownManualMode(void);

bool WheelModeFlag = true;
bool ManualModeFlag = false;
bool ManualModeFlag_Foot = false;
bool ManualModeFlag_Foot_both = false;
bool ManualModeFlag_Hand_both = false;
bool Joystick_flag = false;

bool _isFirst = true;
bool _isFirst_both_R = true;
bool _isFirst_both_L = true;

bool Driving_Flag = false;

// Debug
int Iteration = 0;

// Functions
int	PushCANMessage(MANUAL_CAN MCData);
int RBJointGainOverride(unsigned int _canch, unsigned int _bno, unsigned int _override1, unsigned int _override2, unsigned int _duration);
int RBBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
int RBenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _enable1, unsigned int _enable2);
unsigned int RBsetFrictionParameter(unsigned int _canch, unsigned int _bno, int _vel_saturation1, int _amp_compen1, int _vel_saturation2, int _amp_compen2);
unsigned int RBJointPWMCommand2chHR(unsigned int _canch, unsigned int _bno, int _duty1, int _duty2, int _zeroduty);
unsigned int RBJointOLCurrentCommand2ch(unsigned int _canch, unsigned int _bno, int i1_mA, int i2_mA, int _zero);
void Enc_request(int on);

//Joystick
RBJoystick          *joystick;
char GL_JOY_LT, GL_JOY_LB;
int GL_JOY_LJOG_RL, GL_JOY_LJOG_UD;
int GL_JOY_AROW_RL, GL_JOY_AROW_UD;
char GL_JOY_LJOG_PUSH;

char GL_JOY_RT, GL_JOY_RB;
int GL_JOY_RJOG_RL, GL_JOY_RJOG_UD;
char GL_JOY_A, GL_JOY_B, GL_JOY_X, GL_JOY_Y;
char GL_JOY_RJOG_PUSH;

char GL_JOY_BACK, GL_JOY_START;


#endif // WMUPPERBODY_H
