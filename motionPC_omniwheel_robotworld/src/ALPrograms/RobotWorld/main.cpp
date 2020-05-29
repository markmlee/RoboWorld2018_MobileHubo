/*
 *	This program is generated from drc-podoal-template.
 *
 *	This AL-Process will activated by "PODO_Daemon" with appropriate "AL-Number".
 *	The AL-Number is determined at Core_config.db file at Share folder.
 *	And the AL-Process needs this AL-Number as an input parameter.
 *	So trying to open the AL-Process without any input parameter will terminate the process.
 *
 *	Please check your PODO_AL_NAME and Core_Config.db file.
 *	Actually the PODO_AL_NAME is used for recognizing the certain process and making it unique.
 *	So the same name of file with different path is allowed if the PODO_AL_NAMEs are different.
 *	But we recommend you that gather all of your build file in one folder (check the Core_Config.db file).
 *
 *	You can change the period of real-time thread by changing "RT_TIMER_PERIOD_MS" as another value in "rt_task_set_periodic".
 *	Please do not change "RT_TIMER_PERIOD_MS" value in "typedef.h".
 *	You can also change the priority of the thread by changing 4th parameter of "rt_task_create".
 *	In this function you need to care about the name of thread (the name should be unique).
 *
 *	Please do not change the "RBInitialize" function and fore code of main().
 *	You may express your idea in while loop of main & real-time thread.
 *
 *	Each AL-Process has its own command structure in Shared Memory.
 *	So, it can have its own command set.
 *	Make sure that the command set of each AL-process start from over than 100.
 *	Under the 100 is reserved for common command set.
 *
 *	Now, you can do everything what you want..!!
 *	If you have any question about PODO, feel free to contact us.
 *	Thank you.
 *
 *
 *
 *	Jungho Lee		: jungho77@rainbow.re.kr
 *	Jeongsoo Lim	: yjs0497@kaist.ac.kr
 *	Okkee Sim		: sim2040@kaist.ac.kr
 *
 *	Copy Right 2014 @ Rainbow Co., HuboLab of KAIST
 *
 */



#include "MainHeader.h"
#include "BasicFiles/ManualCAN.h"
#include "../../../share/Headers/Command.h"

#define PODO_AL_NAME       "ROBOTWORLD_Demo"

using namespace std;
inline void pushData(doubles &tar, double var){
    tar.push_back(var);
    tar.pop_front();
}

pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM       userData;

RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

JointControlClass *joint;
TaskMotion      *WBmotion;

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int podoNum = -1;
    while((opt = getopt(argc, argv, "g:p:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'p':
            podoNum = atoi(optarg);
            if(podoNum == 0){
                FILE_LOG(logERROR) << optarg;
                FILE_LOG(logERROR) << "Invalid option for AL";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }else{
                PODO_NO = podoNum;
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'p'){
                FILE_LOG(logERROR) << "Option for AL";
                FILE_LOG(logERROR) << "Valid options are \"Integer Values\"";
            }
        }
    }


    cout << endl;
    FILE_LOG(logERROR) << "===========AL Setting============";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "AL for Gazebo";
    else                FILE_LOG(logWARNING) << "AL for Robot";
    FILE_LOG(logWARNING) << "AL Number: " << PODO_NO;
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}

int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT, CatchSignals);    // Ctrl-c
    signal(SIGHUP, CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if( RBInitialize() == -1 )
        isTerminated = -1;

    InitializeSharedMemory();
    InitializeDrinkStock();
    PODO_NO_WALKREADY = 3;
    usleep(500*1000);

    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, joint);

    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
        case ROBOTWORLD_AL_OMNIWHEEL_MOVE:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)//E-Stop
            {
                FILE_LOG(logERROR) << "NEW COMMAND :: OMNIWHEEL E-STOP";
                ShutDownAllFlag();
            } else
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: OMNIWHEEL MOVE";

                double x = -sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double y = -sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double r = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]*D2R;

                WheelMove(x,y,r,false);
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        case ROBOTWORLD_AL_JOYSTICK_MOVE:
        {
            ShutDownAllFlag();
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)//manual move on
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: JOYSTICK MOVE Start";
                InitWheelInfo();
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEEL();
                Mode_OMNIMove = MOVE_JOY_LOCAL;
                FLAG_JOYStart = true;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;

            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)//manual move off
            {
                FILE_LOG(logWARNING) << "NEW COMMAND :: JOYSTICK MOVE Stop";

                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[11]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[12]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[14]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[15]=0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[16]=0;

                Mode_OMNIMove = NOT_MOVE;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            }
            break;
        }
        case ROBOTWORLD_AL_SAVE:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: DATA SAVE";
                fp = fopen("data.txt","w");
                for(int i=0;i<Save_Index;i++)
                {
                    for(int j=0;j<COL;j++)fprintf(fp,"%g\t", Save_Data[j][i]);
                    fprintf(fp,"\n");
                }
                fclose(fp);
                FILE_LOG(logSUCCESS) << "Data Save Complete";
            }else
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: DATA RESET";
                Save_Index = 0;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        case ROBOTWORLD_AL_HAND_GO:
        {
            FILE_LOG(logSUCCESS) << "NEW COMMAND :: HAND MANUAL MOVE";
            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
            StartWBIKmotion(0);
            joint->SetAllMotionOwner();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)
            {
                MOVE_HAND = HAND_R;
                for(int i=0;i<3;i++)
                {
                    des_RH.pos[i] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
                }

                des_RH.ori[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                des_RH.ori[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                des_RH.ori[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                des_RH.ori[3] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];

                des_RH.elb = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
                des_RH.sec = 2.0;
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 1)
            {
                MOVE_HAND = HAND_L;
                for(int i=0;i<3;i++)
                {
                    des_LH.pos[i] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
                }

                des_LH.ori[0] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                des_LH.ori[1] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                des_LH.ori[2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                des_LH.ori[3] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];

                des_LH.elb = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
                des_LH.sec = 2.0;
            }

            Mode_HANDMove = MOVE_TEST;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        case ROBOTWORLD_AL_GRASPING:
        {
            FILE_LOG(logSUCCESS) << "NEW COMMAND :: GRASPING";
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0)      FLAG_HAND = HAND_GRASP;
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 1) FLAG_HAND = HAND_STOP;
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 2) FLAG_HAND = HAND_OPEN;

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] == 0)      GRASP_HAND = HAND_BOTH;
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] == 1) GRASP_HAND = HAND_R;
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] == 2) GRASP_HAND = HAND_L;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        case ROBOTWORLD_AL_VISION_GRASP:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)//E-Stop
            {
                FILE_LOG(logWARNING) << "NEW COMMAND :: HAND MOVE E-STOP";
                ShutDownAllFlag();
            } else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)//Grasp
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: HAND VISION GRASP MOVE";
                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(0);
                usleep(20*1000);
                joint->SetAllMotionOwner();

//                if(userData->R2M.Objpos[0].pos_y > 0.0)
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1] > 0.03)
                {
                    MOVE_HAND = HAND_L;
//                    des_LH.pos.x = userData->R2M.Objpos[0].pos_x-in.Offset_ObjectX;
//                    des_LH.pos.y = userData->R2M.Objpos[0].pos_y;
//                    des_LH.pos.z = userData->R2M.Objpos[0].pos_z-in.Offset_ObjectZ;

                    des_LH.pos.x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]-in.Offset_ObjectX;
                    des_LH.pos.y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    des_LH.pos.z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]-in.Offset_ObjectZ;

                    des_LH.elb   = 60.0;//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                    printf("LTarget pos = %f, %f, %f\n",des_LH.pos.x,des_LH.pos.y,des_LH.pos.z);
                    if(des_LH.pos.z < -0.3 || des_LH.pos.x < 0 || des_LH.pos.y < -0.1|| des_LH.elb < 0)
                    {
                        FILE_LOG(logERROR) << "POS ERROR";
                        ShutDownAllFlag();
                        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
                        break;
                    }
                    des_LH.ori.w = 0.5;
                    des_LH.ori.x = 0.5;
                    des_LH.ori.y = -0.5;
                    des_LH.ori.z = -0.5;
                } else
                {
                    MOVE_HAND = HAND_R;
//                    des_RH.pos.x = userData->R2M.Objpos[0].pos_x-in.Offset_ObjectX;
//                    des_RH.pos.y = userData->R2M.Objpos[0].pos_y;
//                    des_RH.pos.z = userData->R2M.Objpos[0].pos_z-in.Offset_ObjectZ;


                    des_RH.pos.x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]-in.Offset_ObjectX;
                    des_RH.pos.y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    des_RH.pos.z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]-in.Offset_ObjectZ;

                    des_RH.elb   = -60.0;//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                    printf("RTarget pos = %f, %f, %f\n",des_RH.pos.x,des_RH.pos.y,des_RH.pos.z);
                    if(des_RH.pos.z < -0.3 || des_RH.pos.x < 0 || des_RH.pos.y > 0.1 || des_RH.elb > 0)
                    {
                        FILE_LOG(logERROR) << "POS ERROR";
                        ShutDownAllFlag();
                        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
                        break;
                    }
                    des_RH.ori.w = 0.5;
                    des_RH.ori.x = 0.5;
                    des_RH.ori.y = -0.5;
                    des_RH.ori.z = -0.5;
                }
                Command_Grasp = GRASP_START;
                Mode_HANDMove = MOVE_VISION;
            } else //Put
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: HAND PUT MOVE";
                ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                StartWBIKmotion(0);
                usleep(20*1000);
                joint->SetAllMotionOwner();

//                if(userData->R2M.Objpos[0].pos_y > 0.03)
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1] > 0.0)
                {
                    MOVE_HAND = HAND_L;
                    des_LH.pos.x = 0.6;//userData->R2M.Objpos[0].pos_x-0.05;
                    des_LH.pos.y = 0.2;
                    des_LH.pos.z = 0.90-0.495;

//                    des_LH.pos.x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]-0.05;

                    des_LH.elb   = 50.0;//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                    printf("LTarget pos = %f, %f, %f\n",des_LH.pos.x,des_LH.pos.y,des_LH.pos.z);
                    if(des_LH.pos.z < -0.3 || des_LH.pos.x < 0 || des_LH.pos.y < -0.1|| des_LH.elb < 0)
                    {
                        FILE_LOG(logERROR) << "POS ERROR";
                        ShutDownAllFlag();
                        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
                        break;
                    }

                    des_LH.ori.w = in.Quaternion_LGraspW;
                    des_LH.ori.x = in.Quaternion_LGraspX;
                    des_LH.ori.y = in.Quaternion_LGraspY;
                    des_LH.ori.z = in.Quaternion_LGraspZ;
                } else
                {
                    MOVE_HAND = HAND_R;
                    des_RH.pos.x = 0.6;//userData->R2M.Objpos[0].pos_x-0.05;
                    des_RH.pos.y = -0.2;
                    des_RH.pos.z = 0.90-0.495;

                    printf("posx = %f\n",userData->R2M.Objpos[0].pos_x);
//                    des_RH.pos.x = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]-0.05;
//                    des_RH.pos.y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
//                    des_RH.pos.z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]-0.495;

                    des_RH.elb   = -50.0;//sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                    printf("R1Target pos = %f, %f, %f\n",des_RH.pos.x,des_RH.pos.y,des_RH.pos.z);
                    if(des_RH.pos.z < -0.3 || des_RH.pos.x < 0 || des_RH.pos.y > 0.1|| des_RH.elb > 0)
                    {
                        FILE_LOG(logERROR) << "POS ERROR";
                        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
                        break;
                    }

                    des_RH.ori.w = in.Quaternion_RGraspW;
                    des_RH.ori.x = in.Quaternion_RGraspX;
                    des_RH.ori.y = in.Quaternion_RGraspY;
                    des_RH.ori.z = in.Quaternion_RGraspZ;
                }

                Command_Grasp = PUT_START;
                Mode_HANDMove = MOVE_VISION;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        case ROBOTWORLD_AL_OMNIMOVE:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)//Go2Display
            {
                FILE_LOG(logSUCCESS) << "NEW ROS COMMAND :: OMNIMOVE FRONT";
                InitWheelInfo();
                ShutDownAllFlag();
                OnOff_YawCompen = false;
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwnerWHEELandWST();

                if(SetRotation(WST_ZERO))
                {
                    SetWheelMovePos(in.Distance_Display2Counter_m, 0.0, 0.0);

                    Command_WSTrot = WST_BACK;
                    SetGoalPos();

                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
                }
            } else//Go2Home
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: OMNIMOVE BACK";
                InitWheelInfo();
                ShutDownAllFlag();

                SetWheelMovePos(-in.Distance_Display2Counter_m ,0.0, 0.0);
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();
                Command_WSTrot = WST_ZERO;
                SetGoalPos();

                StartWBIKmotion(0);
                printf("x = %f, y = %f, theta = %f, sec = %f\n********************************\n",OMNIinfo.Pin.X, OMNIinfo.Pin.Y, OMNIinfo.Pin.Theta, OMNIinfo.GoalSec);

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            }

            break;
        }
        case ROBOTWORLD_AL_RESET:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1)//Pos Reset
            {
                FILE_LOG(logSUCCESS) << "NEW COMMAND :: POS RESET";
                InitWheelInfo();
                ShutDownAllFlag();
                ResetPos();
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        case ROBOTWORLD_AL_TEST:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0)
            {

            } else
            {

            }
            userData->R2M.ROS_COMMAND = ROBOTWORLD_ROS_GO2HOME;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }
        default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ROBOTWORLD_AL_NO_ACT;
            break;
        }

        switch(userData->R2M.ROS_COMMAND)
        {
        case ROBOTWORLD_ROS_E_STOP:
        {
            FILE_LOG(logERROR) << "NEW ROS COMMAND :: E-STOP";
            ShutDownAllFlag();
            userData->M2R.MOTION_ERROR = ROS_E_STOP;
            userData->M2R.MOTION_STATE = WORKING;
            userData->R2M.ROS_COMMAND = ROBOTWORLD_ROS_NO_ACT;
            break;
        }
        case ROBOTWORLD_ROS_GO2DISPLAY:
        {
            FILE_LOG(logSUCCESS) << "NEW ROS COMMAND :: OMNIMOVE FRONT";
            InitWheelInfo();
            ShutDownAllFlag();
            OnOff_YawCompen = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwnerWHEELandWST();

            if(SetRotation(WST_ZERO))
            {
                SetWheelMovePos(in.Distance_Display2Counter_m, 0.0, 0.0);
                Command_WSTrot = WST_BACK;
                SetGoalPos();

                userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
            }
            userData->M2R.MOTION_STATE = WORKING;
            break;
        }
        case ROBOTWORLD_ROS_GO2GRASPSPOT:
        {
            FILE_LOG(logSUCCESS) << "NEW ROS COMMAND :: Go to Grasp Spot";
            InitWheelInfo();
            ShutDownAllFlag();
            SetOMNIpara(MIDDLE);
            OnOff_YawCompen = false;
            switch(userData->R2M.SELECTED_MENU)
            {
            case SAMDASOO:
            {
                OMNIinfo.Pin.X      = 0.0;
                OMNIinfo.Pin.Y      = -in.Distance_DrinkSpot_m;
                OMNIinfo.Pin.Theta  = 0.0;
                OMNIinfo.GoalSec    = in.Time_MoveDrinkSpot_sec;
                in.Offset_PutZ = in.Offset_SAMDASOOz;
                EncoderGripper = in.Position_Encoder_SAMDASOO;
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();
                SetGoalPos();
                printf("x = %f, y = %f, theta = %f, sec = %f\n********************************\n",OMNIinfo.Pin.X, OMNIinfo.Pin.Y, OMNIinfo.Pin.Theta, OMNIinfo.GoalSec);
                FLAG_OMNI = 1;
                Mode_OMNIMove = MOVE_OMNI;
                userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
                userData->M2R.MOTION_STATE = WORKING;
                break;
            }
            case VITAMINWATER:
            {
                OMNIinfo.Pin.X      = 0.0;
                OMNIinfo.Pin.Y      = in.Distance_DrinkSpot_m;
                OMNIinfo.Pin.Theta  = 0.0;
                OMNIinfo.GoalSec    = in.Time_MoveDrinkSpot_sec;
                in.Offset_PutZ = in.Offset_VITAMINz;
                EncoderGripper = in.Position_Encoder_VITAMIN;
                joint->RefreshToCurrentReference();
                joint->SetAllMotionOwner();
                SetGoalPos();
                printf("x = %f, y = %f, theta = %f, sec = %f\n********************************\n",OMNIinfo.Pin.X, OMNIinfo.Pin.Y, OMNIinfo.Pin.Theta, OMNIinfo.GoalSec);
                FLAG_OMNI = 1;
                Mode_OMNIMove = MOVE_OMNI;
                userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
                userData->M2R.MOTION_STATE = WORKING;
                break;
            }
            case SPRITE:
            {
                FILE_LOG(logSUCCESS) << "SPOT MOVE DONE";
                in.Offset_PutZ = in.Offset_SPRITEz;
                EncoderGripper = in.Position_Encoder_SPRITE;
                userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_NO_ACT;
                userData->M2R.MOTION_STATE = DONE;
                break;
            }
            default:
            {
                FILE_LOG(logERROR) << "NO SELECTED MENU";
                userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_NO_ACT;
                userData->M2R.MOTION_STATE = DONE;
                break;
            }
            }
            break;
        }
        case ROBOTWORLD_ROS_GO2HOME:
        {
            FILE_LOG(logSUCCESS) << "NEW ROS COMMAND :: OMNIMOVE BACK";
            InitWheelInfo();
            ShutDownAllFlag();
            OnOff_YawCompen = false;
            StartWBIKmotion(0);
            OMNIinfo.Pin.X      = -in.Distance_Display2Counter_m;
            OMNIinfo.Pin.Theta  = 0.0;
            OMNIinfo.GoalSec    = in.Time_MoveDisplay_sec;

            if(userData->R2M.SELECTED_MENU == SAMDASOO)
            {
                OMNIinfo.Pin.Y      = in.Distance_DrinkSpot_m;
            } else if(userData->R2M.SELECTED_MENU == VITAMINWATER)
            {
                OMNIinfo.Pin.Y      = -in.Distance_DrinkSpot_m;
            } else if(userData->R2M.SELECTED_MENU == SPRITE)
            {

                OMNIinfo.Pin.Y      = 0.0;
            }
            Command_WSTrot = WST_ZERO;
            SetGoalPos();
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            printf("x = %f, y = %f, theta = %f, sec = %f\n********************************\n",OMNIinfo.Pin.X, OMNIinfo.Pin.Y, OMNIinfo.Pin.Theta, OMNIinfo.GoalSec);

            FLAG_OMNI = 1;
            Mode_PutEarly = true;
            Mode_OMNIMove = MOVE_OMNI;
            userData->M2R.MOTION_STATE = WORKING;
            userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
            break;
        }
        case ROBOTWORLD_ROS_GRASP:
        {
            FILE_LOG(logSUCCESS) << "NEW ROS COMMAND :: HAND VISION GRASP MOVE";
            ShutDownAllFlag();
            joint->RefreshToCurrentReference();
            StartWBIKmotion(0);
            usleep(20*1000);
            joint->SetAllMotionOwner();

            if(userData->R2M.Objpos[0].pos_y > 0.0)
            {
                MOVE_HAND = HAND_L;
                des_LH.pos.x = userData->R2M.Objpos[0].pos_x - in.Offset_ObjectX;
                des_LH.pos.y = userData->R2M.Objpos[0].pos_y - in.Offset_ObjectY;
                des_LH.pos.z = userData->R2M.Objpos[0].pos_z - in.Offset_ObjectZ;

                des_LH.elb   = in.Degree_ElbowGrasp;

                printf("LTarget pos = %f, %f, %f\n",des_LH.pos.x,des_LH.pos.y,des_LH.pos.z);
                if(IsPosXSafe() == false  || IsPosYSafe() == false || IsPosZSafe() == false || IsElbAngleSafe() == false)
                {
                    FILE_LOG(logERROR) << "POS ERROR";
                    ShutDownAllFlag();
                    userData->M2R.MOTION_STATE = NO_ACT;
                    userData->M2R.MOTION_ERROR = VISION_OBJ_ERROR;
                    userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
                    break;
                }
                des_LH.ori.w = in.Quaternion_LGraspW;
                des_LH.ori.x = in.Quaternion_LGraspX;
                des_LH.ori.y = in.Quaternion_LGraspY;
                des_LH.ori.z = in.Quaternion_LGraspZ;
            } else
            {
                MOVE_HAND = HAND_R;
                des_RH.pos.x = userData->R2M.Objpos[0].pos_x - in.Offset_ObjectX;
                des_RH.pos.y = userData->R2M.Objpos[0].pos_y - in.Offset_ObjectY;
                des_RH.pos.z = userData->R2M.Objpos[0].pos_z - in.Offset_ObjectZ;

                des_RH.elb   = -in.Degree_ElbowGrasp;

                printf("RTarget pos = %f, %f, %f\n",des_RH.pos.x,des_RH.pos.y,des_RH.pos.z);
                if(IsPosXSafe() == false  || IsPosYSafe() == false || IsPosZSafe() == false || IsElbAngleSafe() == false)
                {
                    FILE_LOG(logERROR) << "POS ERROR";
                    ShutDownAllFlag();
                    userData->M2R.MOTION_STATE = NO_ACT;
                    userData->M2R.MOTION_ERROR = VISION_OBJ_ERROR;
                    userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
                    break;
                }
                des_RH.ori.w = in.Quaternion_RGraspW;
                des_RH.ori.x = in.Quaternion_RGraspX;
                des_RH.ori.y = in.Quaternion_RGraspY;
                des_RH.ori.z = in.Quaternion_RGraspZ;
            }

            Command_Grasp = GRASP_START;
            Mode_HANDMove = MOVE_VISION;

            userData->M2R.MOTION_STATE = WORKING;
            userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
            break;
        }
        case ROBOTWORLD_ROS_PUT:
        {
                FILE_LOG(logSUCCESS) << "NEW ROS COMMAND :: HAND PUT MOVE";
                //ShutDownAllFlag();
                joint->RefreshToCurrentReference();
                //StartWBIKmotion(0);
                usleep(20*1000);
                joint->SetAllMotionOwner();

//                if(userData->R2M.Objpos[0].pos_y > 0.0)
//                {
//                    MOVE_HAND = HAND_L;
//                    des_LH.pos.x = in.Position_PutX;
//                    des_LH.pos.y = in.Position_PutY;
//                    des_LH.pos.z = in.Position_PutZ;

//                    des_LH.elb   = in.Degree_ElbowGrasp;

//                    printf("LTarget pos = %f, %f, %f\n",des_LH.pos.x,des_LH.pos.y,des_LH.pos.z);
//                    if(IsPosXSafe() == false  || IsPosYSafe() == false || IsPosZSafe() == false || IsElbAngleSafe() == false)
//                    {
//                        FILE_LOG(logERROR) << "POS ERROR";
//                        ShutDownAllFlag();
//                        userData->M2R.MOTION_STATE = NO_ACT;
//                        userData->M2R.MOTION_ERROR = VISION_OBJ_ERROR;
//                        userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
//                        break;
//                    }
//                    des_LH.ori.w = in.Quaternion_GraspW;
//                    des_LH.ori.x = in.Quaternion_GraspX;
//                    des_LH.ori.y = in.Quaternion_GraspY;
//                    des_LH.ori.z = in.Quaternion_GraspZ;
//                } else
//                {
//                    MOVE_HAND = HAND_R;
//                    des_RH.pos.x = in.Position_PutX;
//                    des_RH.pos.y = -in.Position_PutY;
//                    des_RH.pos.z = in.Position_PutZ;

//                    des_RH.elb   = -in.Degree_ElbowGrasp;

//                    printf("RTarget pos = %f, %f, %f\n",des_RH.pos.x,des_RH.pos.y,des_RH.pos.z);
//                    if(IsPosXSafe() == false  || IsPosYSafe() == false || IsPosZSafe() == false || IsElbAngleSafe() == false)
//                    {
//                        FILE_LOG(logERROR) << "POS ERROR";
//                        ShutDownAllFlag();
//                        userData->M2R.MOTION_STATE = NO_ACT;
//                        userData->M2R.MOTION_ERROR = VISION_OBJ_ERROR;
//                        userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
//                        break;
//                    }
//                    des_RH.ori.w = in.Quaternion_GraspW;
//                    des_RH.ori.x = in.Quaternion_GraspX;
//                    des_RH.ori.y = in.Quaternion_GraspY;
//                    des_RH.ori.z = in.Quaternion_GraspZ;
//                }

//                Command_Grasp = PUT_START;
                Command_Grasp = PUT_PUT;
                Mode_HANDMove = MOVE_VISION;

                userData->M2R.MOTION_STATE = WORKING;
                userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
                break;
        }
        case ROBOTWORLD_ROS_POS_RESET:
        {
            FILE_LOG(logSUCCESS) << "NEW COMMAND :: POS RESET";
            InitWheelInfo();
//            SLAMreset();
            ResetPos();
            userData->R2M.ROS_COMMAND = ROBOTWORLD_AL_WORKING;
            break;
        }
        default:
            break;
        }
    }

    cout << ">>> Process RobotWorld is terminated..!!" << endl;
    return 0;
}

/************************************************************************************************************************/
void RBTaskThread(void *)
{
    while(isTerminated == 0)
    {
        /* change flag */
        MotionState_TH();

        /* approach hand to target pos */
        HandMove_TH();

        /* Gripper Open&Close */
        Grasping_TH();

        /* OMNI Wheel Move */
        OmniMove_TH();

        /* don't use */
//        Stock_TH();

        if(WB_FLAG == true)
        {
            // Global whole body model
            WBmotion->updateAll();
            WBmotion->WBIK_UB();

            for(int i=RHY; i<=LAR; i++)
            {
                if(i!=BWH)
                    joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);
            }
            //joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

            joint->SetJointRefAngle(RSP, WBmotion->Q_filt_34x1[idRSP]*R2D);
            joint->SetJointRefAngle(RSR, WBmotion->Q_filt_34x1[idRSR]*R2D - OFFSET_RSR);
            joint->SetJointRefAngle(RSY, WBmotion->Q_filt_34x1[idRSY]*R2D);
            joint->SetJointRefAngle(REB, WBmotion->Q_filt_34x1[idREB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(RWY, WBmotion->Q_filt_34x1[idRWY]*R2D);
            joint->SetJointRefAngle(RWP, WBmotion->Q_filt_34x1[idRWP]*R2D);
            joint->SetJointRefAngle(RWY2, WBmotion->Q_filt_34x1[idRWY2]*R2D);

            joint->SetJointRefAngle(LSP, WBmotion->Q_filt_34x1[idLSP]*R2D);
            joint->SetJointRefAngle(LSR, WBmotion->Q_filt_34x1[idLSR]*R2D - OFFSET_LSR);
            joint->SetJointRefAngle(LSY, WBmotion->Q_filt_34x1[idLSY]*R2D);
            joint->SetJointRefAngle(LEB, WBmotion->Q_filt_34x1[idLEB]*R2D - OFFSET_ELB);
            joint->SetJointRefAngle(LWY, WBmotion->Q_filt_34x1[idLWY]*R2D);
            joint->SetJointRefAngle(LWP, WBmotion->Q_filt_34x1[idLWP]*R2D);
            joint->SetJointRefAngle(LWY2, WBmotion->Q_filt_34x1[idLWY2]*R2D);

            if(!CheckMotionOwned())
                WB_FLAG = false;
        }
        //printf("state = %d\n",userData->M2R.MOTION_STATE);
        save();
        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}

void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(isTerminated == 0)
    {
        rt_task_wait_period(NULL);

        //if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        //}
    }
}

void MotionState_TH()
{
    if(Command_Grasp == GRASP_DONE)
    {
        Command_Grasp = GRASP_NO_ACT;
        userData->M2R.MOTION_STATE = DONE;
    }

    if(Command_Grasp == PUT_DONE)
    {
        Command_Grasp = GRASP_NO_ACT;
        userData->M2R.MOTION_STATE = NO_ACT;
    }

    if(FLAG_RotReset == true && FLAG_OMNI == 4)
    {
        FLAG_RotReset = false;
        FLAG_OMNI = false;
    } else if(Mode_PutEarly == false && FLAG_OMNI == 4)
    {
        userData->M2R.MOTION_STATE = DONE;
        FLAG_OMNI = false;
    } else if(Mode_PutEarly == true && FLAG_OMNI == 4 && Command_Grasp == PUT_READY)
    {
        userData->M2R.MOTION_STATE = DONE;
        FLAG_OMNI = false;
        Mode_PutEarly = false;
    } /*else if(FLAG_SLAMReset == true && FLAG_OMNI == 4)
    {
        FLAG_SLAMReset = false;
        ResetPos();
    } */
}

void HandMove_TH()
{
    if(Mode_HANDMove == MOVE_VISION)
    {
        VisionHandMove_TH();
    } else if(Mode_HANDMove == MOVE_TEST)
    {
        GO_Grasp();
    }
}
double Xa3,Xa4,Xa5,Ya3,Ya4,Ya5,Ra3,Ra4,Ra5;
void OmniMove_TH()
{
    switch(Mode_OMNIMove)
    {
    case MOVE_OMNI:
    {
        if(FLAG_OMNI != false)
        {
            if(FLAG_OMNI == 1)
            {
                OMNIinfo.CurSec = 0.;
                OMNIinfo.Probot.X = 0.;
                OMNIinfo.Probot.Y = 0.;
                OMNIinfo.Probot.Theta = 0.;
                /* Set M */
                OMNIinfo.Pin.M = sqrt(OMNIinfo.Pin.X*OMNIinfo.Pin.X + OMNIinfo.Pin.Y*OMNIinfo.Pin.Y);
                OMNIinfo.move_theta = acos(fabs(OMNIinfo.Pin.Y)/OMNIinfo.Pin.M);
                if(isnan(OMNIinfo.move_theta))
                    OMNIinfo.move_theta = 0.;

                if(OMNIinfo.Pin.M < 0.3 && OMNIinfo.Pin.M > 0.0001)
                    SetOMNIpara(MOVE_LITTLE);
                else if(OMNIinfo.Pin.M < 0.7 && OMNIinfo.Pin.M > 0.0001)
                    SetOMNIpara(MOVE_NORMAL);
                else if(OMNIinfo.Pin.M < 1.2 && OMNIinfo.Pin.M > 0.0001)
                    SetOMNIpara(MOVE_FAR);

                if(fabs(OMNIinfo.Pin.Theta) < 20.0*D2R && fabs(OMNIinfo.Pin.Theta) > 0.01*D2R)
                    SetOMNIpara(TURN_LITTLE);
                else if(fabs(OMNIinfo.Pin.Theta) < 40.0*D2R && fabs(OMNIinfo.Pin.Theta) > 0.01*D2R)
                    SetOMNIpara(TURN_NORMAL);
                else if(fabs(OMNIinfo.Pin.Theta) < 80.0*D2R && fabs(OMNIinfo.Pin.Theta) > 0.01*D2R)
                    SetOMNIpara(TURN_FAR);

                printf("Pin M = %f, theta = %f\n",OMNIinfo.Pin.M, OMNIinfo.move_theta);

                if(fix == XFix)
                {
                    /* fixed move distance during acc&dec */
                    if(fabs(OMNIinfo.Pin.M) < OMNIinfo.Trajm)
                        OMNIinfo.Pc.M = OMNIinfo.Pin.M;
                    else
                        OMNIinfo.Pc.M = OMNIinfo.Trajm;

                    if(fabs(OMNIinfo.Pin.Theta) < OMNIinfo.Trajrad)
                        OMNIinfo.Pc.Theta = OMNIinfo.Pin.Theta;
                    else
                        OMNIinfo.Pc.Theta = OMNIinfo.Trajrad;

                    /* Calculated move time during acc&dec */
                    OMNIinfo.TrajSec = (30./16.)*(OMNIinfo.Pc.M - OMNIinfo.Probot.M)/(OMNIinfo.Vms*2.);
                    OMNIinfo.TrajSecR = (30./16.)*fabs(OMNIinfo.Pc.Theta - OMNIinfo.Probot.Theta)/(OMNIinfo.Vrads*2.);

                } else if(fix == TimeFix)
                {
                    /* fixed move time during acc&dec */
                    if(fabs(OMNIinfo.Pin.M) < OMNIinfo.Trajm)
                        OMNIinfo.TrajSec = OMNIinfo.TrajT*fabs(OMNIinfo.Pin.M/(OMNIinfo.Trajm));
                    else
                        OMNIinfo.TrajSec = OMNIinfo.TrajT;

                    if(fabs(OMNIinfo.Pin.Theta) < OMNIinfo.Trajrad)
                        OMNIinfo.TrajSecR = (OMNIinfo.TrajT)*fabs(OMNIinfo.Pin.Theta/OMNIinfo.Trajrad);
                    else
                        OMNIinfo.TrajSecR = OMNIinfo.TrajT;

                    /* Calculated move distance during acc&dec */
                    OMNIinfo.Pc.M = (16./30.)*OMNIinfo.TrajSec*2*OMNIinfo.Vms + OMNIinfo.Probot.M;
                    OMNIinfo.Pc.Theta = (16./30.)*OMNIinfo.TrajSecR*2*OMNIinfo.Vrads + OMNIinfo.Probot.Theta;

                }
                OMNIinfo.Pc.X = OMNIinfo.Pin.X*(OMNIinfo.Pc.M/OMNIinfo.Pin.M);
                OMNIinfo.Pc.Y = OMNIinfo.Pin.Y*(OMNIinfo.Pc.M/OMNIinfo.Pin.M);

                if(OMNIinfo.Pin.Theta <= -OMNIinfo.Trajrad)  OMNIinfo.Pc.Theta = -OMNIinfo.Pc.Theta;
                if(isnan(OMNIinfo.Pc.X))    OMNIinfo.Pc.X = 0.;
                if(isnan(OMNIinfo.Pc.Y))    OMNIinfo.Pc.Y = 0.;
                if(isnan(OMNIinfo.Pc.Theta))OMNIinfo.Pc.Theta = 0.;

                printf("PcM = %f, PcX = %f, PcY = %f, max = %f\n",OMNIinfo.Pc.M, OMNIinfo.Pc.X, OMNIinfo.Pc.Y, OMNIinfo.Vms);
                /* Calculated move time during constant vel */
                if(OMNIinfo.Pin.M < OMNIinfo.Probot.M)
                {
                    OMNIinfo.SatSec = 0.;
                    OMNIinfo.Probot.M = OMNIinfo.Pin.M;
                } else
                {
                    OMNIinfo.SatSec = (OMNIinfo.Pin.M - OMNIinfo.Pc.M)/OMNIinfo.Vms;
                }

                if(fabs(OMNIinfo.Pin.Theta) < OMNIinfo.Probot.Theta)
                {
                    OMNIinfo.SatSecR = 0.;
                    OMNIinfo.Probot.Theta = OMNIinfo.Pin.Theta;
                } else
                {
                    OMNIinfo.SatSecR = fabs(OMNIinfo.Pin.Theta - OMNIinfo.Pc.Theta)/OMNIinfo.Vrads;
                }
                printf("pin = %f, pc = %f\n",OMNIinfo.Pin.Theta*R2D, OMNIinfo.Pc.Theta*R2D);
                /* Calculated move time */
                OMNIinfo.GoalSec = OMNIinfo.TrajSec*2. + OMNIinfo.TrajSecR*2. + OMNIinfo.SatSec + OMNIinfo.SatSecR;
                OMNIinfo.GoalSecR = OMNIinfo.TrajSecR*2. + OMNIinfo.SatSecR;

                double dx = OMNIinfo.Pc.X - OMNIinfo.Probot.X;
                double dy = OMNIinfo.Pc.Y - OMNIinfo.Probot.Y;
                double dr = OMNIinfo.Pc.Theta - OMNIinfo.Probot.Theta;
                double t = OMNIinfo.TrajSec*2.;
                double tr = OMNIinfo.TrajSecR*2.;
                printf("Probot = %f, %f, %f\n",OMNIinfo.Probot.X, OMNIinfo.Probot.Y, OMNIinfo.Probot.Theta*R2D);
                printf("dx = %f, %f, %f, t=%f, %f\n",dx,dy,dr*R2D,t,tr);
                printf("GoalSec = %f, GoalSecR = %f, T = %f, Tr = %f, S = %f, Sr = %f\n",OMNIinfo.GoalSec, OMNIinfo.GoalSecR, OMNIinfo.TrajSec, OMNIinfo.TrajSecR, OMNIinfo.SatSec, OMNIinfo.SatSecR);

                Xa3 = 10.*dx/(t*t*t);
                Xa4 = -15.*dx/(t*t*t*t);
                Xa5 = 6.*dx/(t*t*t*t*t);

                Ya3 = 10.*dy/(t*t*t);
                Ya4 = -15.*dy/(t*t*t*t);
                Ya5 = 6.*dy/(t*t*t*t*t);

                Ra3 = 10.*dr/(tr*tr*tr);
                Ra4 = -15.*dr/(tr*tr*tr*tr);
                Ra5 = 6.*dr/(tr*tr*tr*tr*tr);

                FLAG_OMNI = 2;
            }

            if(!(OMNIinfo.CurSec > OMNIinfo.GoalSec))
            {
                Omni_CalVel_Robot5th();
                Omni_CalVel_Wheel();
                Omni_CalRef_Motor();
                OMNIinfo.CurSec += OMNIinfo.TickSec;
            } else
            {
                if(IsRealPos() == true)
                {
                    FILE_LOG(logSUCCESS) << "MOVE DONE!!!";
                    FLAG_OMNI = 4;
                    Mode_OMNIMove = NOT_MOVE;
                    FLAG_Localization = false;
                    InitWheelInfo();
                } else {
                    FILE_LOG(logERROR) << "YAW COMPEN";
                    FLAG_OMNI = 1;
                }
            }
        }
        if(Command_WSTrot == WST_BACK && FLAG_WSTtime == true)
        {
            joint->SetMoveJoint(WST, 180.0, (OMNIinfo.GoalSec-OMNIinfo.GoalSecR)*1000, MOVE_ABSOLUTE);
            Command_WSTrot = WST_STOP;
        } else if(Command_WSTrot == WST_ZERO && FLAG_WSTtime == true)
        {
            joint->SetMoveJoint(WST, 0.0, (OMNIinfo.GoalSec-OMNIinfo.GoalSecR)*1000, MOVE_ABSOLUTE);

            Command_WSTrot = WST_STOP;
        } else
        {
        }
        break;
    }
    case MOVE_JOY_LOCAL:
    {
        JOY_RJOG_RL = -sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[14];
        JOY_LJOG_RL = -sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10];
        JOY_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
        JOY_BACK = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[15];
        JOY_START = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[16];

        if(FLAG_JOYStart == true)
        {
            if(JOY_RJOG_RL == 0 && JOY_LJOG_RL == 0 && JOY_LJOG_UD == 0)
            {
                FLAG_JOYStart = false;
                printf("OK JOY is zero now\n");
            }
            else
            {
                printf("JOY Not Zero!!! %d, %d, %d\n",JOY_RJOG_RL, JOY_LJOG_RL, JOY_LJOG_UD);
            }

        } else
        {
            if(JOY_START == 1)
            {
                FILE_LOG(logWARNING) << "Changed Joy Mode :: GLOBAL";
                Mode_OMNIMove = MOVE_JOY_GLOBAL;
                break;
            }

            if(fabs(JOY_RJOG_RL) >35000) JOY_RJOG_RL = 0;
            if(fabs(JOY_LJOG_RL) >35000) JOY_LJOG_RL = 0;
            if(fabs(JOY_LJOG_UD) >35000) JOY_LJOG_UD = 0;

            static int tempcnt = 0;
            if(JOY_LJOG_RL == 0 && JOY_LJOG_UD == 0 && JOY_RJOG_RL == 0 && tempcnt > 20){
                tempcnt++;
                pushData(RWHList, 0.0);
                pushData(LWHList, 0.0);
                pushData(BWHList, 0.0);
            } else
            {
                Kspeed = 2.5;
                tempcnt = 0;
                Move_X = ((float)JOY_LJOG_UD/-32767.f)*Kspeed;
                Move_Y = ((float)JOY_LJOG_RL/32767.f)*Kspeed;
                Move_R = ((float)JOY_RJOG_RL/32767.f)*Kspeed*0.8;


                MotorSpeed[0] = (0.866)*Move_X - (-0.5)*Move_Y + Move_R;
                MotorSpeed[1] = (-0.866)*Move_X - (-0.5)*Move_Y + Move_R;
                MotorSpeed[2] = -Move_Y + Move_R;

                pushData(LWHList, MotorSpeed[0]);
                pushData(RWHList, MotorSpeed[1]);
                pushData(BWHList, MotorSpeed[2]);
                if(Move_X != 0 || Move_Y != 0 || Move_R != 0)
                {
                    printf("X = %f, Y = %f, Move_R = %f\n",Move_X, Move_Y, Move_R);
                }
            }
            CalculateMovingEverage();

            RWHnow += RWHvel;
            LWHnow += LWHvel;
            BWHnow += BWHvel;

            joint->SetMoveJoint(RWH, RWHinfo.InitRef_Deg + RWHnow, 5, MOVE_ABSOLUTE);
            joint->SetMoveJoint(BWH, BWHinfo.InitRef_Deg + BWHnow, 5, MOVE_ABSOLUTE);
            joint->SetMoveJoint(LWH, LWHinfo.InitRef_Deg + LWHnow, 5, MOVE_ABSOLUTE);
        }
        break;
    }
    case MOVE_JOY_GLOBAL:
    {// GLOBAL move error
        JOY_RJOG_RL = -sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[14];
        JOY_LJOG_RL = -sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10];
        JOY_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
        JOY_BACK = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[15];
        JOY_START = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[16];

        if(JOY_BACK == 1)
        {
            FILE_LOG(logWARNING) << "Changed Joy Mode :: LOCAL";
            Mode_OMNIMove = MOVE_JOY_LOCAL;
            break;
        }

        if(fabs(JOY_RJOG_RL) >35000) JOY_RJOG_RL = 0;
        if(fabs(JOY_LJOG_RL) >35000) JOY_LJOG_RL = 0;
        if(fabs(JOY_LJOG_UD) >35000) JOY_LJOG_UD = 0;

        static int tempcnt = 0;
        if(JOY_LJOG_RL == 0 && JOY_LJOG_UD == 0 && JOY_RJOG_RL == 0 && tempcnt > 20){
            tempcnt++;
            pushData(RWHList, 0.0);
            pushData(LWHList, 0.0);
            pushData(BWHList, 0.0);
        } else
        {
            tempcnt = 0;
            Kspeed = 0.5;
            Move_X = ((float)JOY_LJOG_UD/-32767.f)*Kspeed;
            Move_Y = ((float)JOY_LJOG_RL/32767.f)*Kspeed;
            Move_R = ((float)JOY_RJOG_RL/32767.f)*Kspeed;


            Cur_Th += Move_R*0.005;

            printf("Cur_Th = %f\n",Cur_Th*R2D);

            mat3 Tr = mat3(sin(Pw.alpha1), -cos(Pw.alpha1), Pw.RobotRm,
                           sin(Pw.alpha2), -cos(Pw.alpha2), Pw.RobotRm,
                           sin(Pw.alpha3), -cos(Pw.alpha3), Pw.RobotRm);
            mat3 Tth = mat3(cos(Cur_Th), -sin(Cur_Th), 0,
                            sin(Cur_Th), cos(Cur_Th), 0,
                            0, 0, 1);
            vec3 Temp = vec3(Move_X, Move_Y, 0);
            Temp = vec3(Tth*Temp);
            Move_X = Temp[0];
            Move_Y = Temp[1];

            vec3 Vr = vec3(Move_X, Move_Y, Move_R);
            vec3 temp = vec3(Tr*Vr);

            pushData(LWHList, temp[0]/Pw.WheelRm);
            pushData(RWHList, temp[2]/Pw.WheelRm);
            pushData(BWHList, temp[1]/Pw.WheelRm);

            printf("X = %f, Y = %f, Move_R = %f\n",Move_X, Move_Y, Move_R);
            printf("LWH = %f\n",temp[0]/Pw.WheelRm);
        }
        CalculateMovingEverage();

        RWHnow += RWHvel;
        LWHnow += LWHvel;
        BWHnow += BWHvel;
        printf("move = %f\n",LWHnow);
        joint->SetMoveJoint(RWH, RWHinfo.InitRef_Deg + RWHnow, 5, MOVE_ABSOLUTE);
        joint->SetMoveJoint(BWH, BWHinfo.InitRef_Deg + BWHnow, 5, MOVE_ABSOLUTE);
        joint->SetMoveJoint(LWH, LWHinfo.InitRef_Deg + LWHnow, 5, MOVE_ABSOLUTE);
        printf("LWHwheel = %f\n",LWHinfo.InitRef_Deg + LWHnow);
        break;
    }
    default:
    {
        break;
    }
    }

    /*Robot Pos Set*/
    ROBOTinfo.SLAMPos.X = userData->R2M.robot_pos.pos_x;
    ROBOTinfo.SLAMPos.Y = userData->R2M.robot_pos.pos_y;
    ROBOTinfo.SLAMPos.Theta = userData->R2M.robot_pos.ori_w*D2R - sharedSEN->ENCODER[MC_ID_CH_Pairs[WST].ch][MC_ID_CH_Pairs[WST].id].CurrentPosition*D2R;

    if(ROBOTinfo.SLAMPos.Theta > 180.0*D2R)
    {
        ROBOTinfo.SLAMPos.Theta = ROBOTinfo.SLAMPos.Theta - 360.0*D2R;
    } else if(ROBOTinfo.SLAMPos.Theta < -180.0*D2R)
    {
        ROBOTinfo.SLAMPos.Theta = ROBOTinfo.SLAMPos.Theta + 360.0*D2R;
    }
    ROBOTinfo.DiffPos.X = OMNIinfo.Pin.X;
    ROBOTinfo.DiffPos.Y = OMNIinfo.Pin.Y;
    ROBOTinfo.DiffPos.Theta = OMNIinfo.Pin.Theta;

    userData->M2G.OMNIRobotPosX = ROBOTinfo.GoalPos.X;
    userData->M2G.OMNIRobotPosY = ROBOTinfo.GoalPos.Y;
    userData->M2G.OMNIRobotPosYaw = ROBOTinfo.GoalPos.Theta;


}

void Grasping_TH()
{
    static int cnt = 0;
    double Grasp_upperlimit = -8.;
    double Grasp_lowerlimit = 0.0;
    float EncoderRHAND = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHAND].id][MC_ID_CH_Pairs[RHAND].ch].CurrentPosition;
    float EncoderLHAND = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHAND].id][MC_ID_CH_Pairs[LHAND].ch].CurrentPosition;

    if(FLAG_HAND == HAND_STOP || FLAG_HAND == HAND_DONE)
    {
        if(GRASP_HAND != HAND_L)
        {
            joint->SetJointRefAngle(RHAND, 0);//stop
        }
        if(GRASP_HAND != HAND_R)
        {
            joint->SetJointRefAngle(LHAND, 0);//stop
        }
    } else if(FLAG_HAND == HAND_OPEN)
    {
        if(GRASP_HAND != HAND_L)
        {
            if(sharedSEN->ENCODER[MC_ID_CH_Pairs[RHAND].id][MC_ID_CH_Pairs[RHAND].ch].CurrentPosition < in.Limit_GripperUpeer)
            {
                FLAG_HAND = HAND_DONE;
                FILE_LOG(logINFO) << "Grasping Open done";
                cnt = 0;
            }
            joint->SetJointRefAngle(RHAND, -130);//open
        }
        if(GRASP_HAND != HAND_R)
        {
            if(sharedSEN->ENCODER[MC_ID_CH_Pairs[LHAND].id][MC_ID_CH_Pairs[LHAND].ch].CurrentPosition < in.Limit_GripperUpeer)
            {
                FLAG_HAND = HAND_DONE;
                FILE_LOG(logINFO) << "Grasping Open done";
                cnt = 0;
            }
            joint->SetJointRefAngle(LHAND, -125);//open


        }
        cnt++;
    } else if(FLAG_HAND == HAND_GRASP)
    {
        EncoderGripper = in.Position_Encoder_SAMDASOO;
        if(GRASP_HAND != HAND_L)
        {
            if(EncoderRHAND > in.Limit_GripperLower)
            {
                if(Command_Grasp == GRASP_LIFT)
                {
                    FILE_LOG(logERROR) << "Hubo can't grasp drink!!!";
                    userData->M2R.MOTION_ERROR = GRASP_FAIL;
                    cnt = 0;
                    FLAG_HAND = HAND_DONE;
                } else
                {
                    FLAG_HAND = HAND_DONE;
                    FILE_LOG(logINFO) << "Grasping Close done";
                    cnt = 0;
                }
            }
            joint->SetJointRefAngle(RHAND, 130);//grasp
        }
        if(GRASP_HAND != HAND_R)
        {
            if(EncoderLHAND > in.Limit_GripperLower)
            {
                if(Command_Grasp == GRASP_LIFT)
                {
                    FLAG_HAND = HAND_DONE;
                    FILE_LOG(logINFO) << "Grasping Close done";
                    cnt = 0;
                } else
                {
                    FILE_LOG(logERROR) << "Hubo can't grasp drink!!!";
                    userData->M2R.MOTION_ERROR = GRASP_FAIL;
                    cnt = 0;
                    FLAG_HAND = HAND_DONE;
                }
            }
            joint->SetJointRefAngle(LHAND, 125);//grasp
        }
        cnt++;
    }

    if(cnt >= 500)
    {
        if(Command_Grasp == GRASP_LIFT)
        {
            if(GRASP_HAND == HAND_R)
            {
                if(EncoderRHAND < EncoderGripper)
                {
                    FILE_LOG(logSUCCESS) << "Grasping OK";
                    cnt = 0;
                } else
                {
                    FILE_LOG(logERROR) << "Hubo can't grasp drink!!!";
                    userData->M2R.MOTION_ERROR = GRASP_FAIL;
                    cnt = 0;
                }
            } else if(GRASP_HAND == HAND_L)
            {
                if(EncoderLHAND < EncoderGripper)
                {
                    FILE_LOG(logSUCCESS) << "Grasping OK";
                    cnt = 0;
                } else
                {
                    FILE_LOG(logERROR) << "Hubo can't grasp drink!!!";
                    userData->M2R.MOTION_ERROR = GRASP_FAIL;
                    cnt = 0;
                }
            }
        } else
        {
            cnt = 0;
            FLAG_HAND = HAND_STOP;
            FILE_LOG(logINFO) << "Time out : Grasping done";
        }
    }

}

void Stock_TH()
{

}

void VisionHandMove_TH()
{
    switch(Command_Grasp)
    {
    case GRASP_STOP:
    {
        break;
    }
    case GRASP_START:
    {
        doubles ds_InputORI(4);
        if(MOVE_HAND == HAND_R)
        {
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_RH.ori[i];
            }

            des_RH.sec = in.Time_MoveGraspStart_sec;
            TRInfo temp;
            temp = new TrajectoryCosine(des_RH.sec, des_RH.pos.x);
            WBmotion->wbPosRH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec, des_RH.pos.y);
            WBmotion->wbPosRH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec/1.5, des_RH.pos.z);
            WBmotion->wbPosRH[2]->AddTrajInfo(temp);

            WBmotion->addRHOriInfo(ds_InputORI, des_RH.sec/1.3);
            WBmotion->addRElbPosInfo(des_RH.elb, des_RH.sec/1.5);
            Command_Grasp = GRASP_APPROACH;
            FLAG_HAND = HAND_OPEN;
            GRASP_HAND = HAND_R;
        } else
        {
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_LH.ori[i];
            }

            des_LH.sec = in.Time_MoveGraspStart_sec;
            TRInfo temp;
            temp = new TrajectoryCosine(des_LH.sec, des_LH.pos.x);
            WBmotion->wbPosLH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec, des_LH.pos.y);
            WBmotion->wbPosLH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec/1.5, des_LH.pos.z);
            WBmotion->wbPosLH[2]->AddTrajInfo(temp);

            WBmotion->addLHOriInfo(ds_InputORI, des_LH.sec/1.3);
            WBmotion->addLElbPosInfo(des_LH.elb, des_LH.sec/1.5);
            Command_Grasp = GRASP_APPROACH;
            FLAG_HAND = HAND_OPEN;
            GRASP_HAND = HAND_L;
        }
        break;
    }
    case GRASP_APPROACH:
    {
        static int cnt = 0;
        cnt++;
        if(MOVE_HAND == HAND_R)
        {
            if(cnt > des_RH.sec*200*1.5 || (FLAG_HAND == HAND_DONE && cnt > des_RH.sec*200))
            {
                FLAG_HAND = HAND_STOP;
                Command_Grasp = GRASP_GRASP;
                cnt = 0;
                FILE_LOG(logINFO) << "Approach Done";
            }
        } else
        {
            if(cnt >  des_LH.sec*200*1.5 || (FLAG_HAND == HAND_DONE && cnt > des_LH.sec*200))
            {
                FLAG_HAND = HAND_STOP;
                Command_Grasp = GRASP_GRASP;
                cnt = 0;
                FILE_LOG(logINFO) << "Approach Done";
            }
        }
        break;
    }
    case GRASP_GRASP:
    {
        static double dX = 0.0;
        double MoveX = in.Distance_ObjectApproach_m;

        if(MOVE_HAND == HAND_R)
        {
            des_RH.sec = in.Time_MoveGraspApproach_sec;
            doubles ds_InputORI(4);
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_RH.ori[i];
            }
            WBmotion->addRHOriInfo(ds_InputORI, 0.005);
            dX += MoveX*0.005/des_RH.sec;
            WBmotion->addRHPosInfo(des_RH.pos.x+dX, des_RH.pos.y, des_RH.pos.z, 0.005);
            if(dX > MoveX)
            {
                FLAG_HAND = HAND_GRASP;


                Command_Grasp = GRASP_LIFT;
                des_RH.pos.x += dX;
                dX = 0.0;
            }
        } else
        {
            des_LH.sec = in.Time_MoveGraspApproach_sec;
            doubles ds_InputORI(4);
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_LH.ori[i];
            }
            WBmotion->addLHOriInfo(ds_InputORI,0.005);
            dX += MoveX*0.005/des_LH.sec;
            WBmotion->addLHPosInfo(des_LH.pos.x+dX, des_LH.pos.y, des_LH.pos.z, 0.005);

            if(dX > MoveX)
            {
                FLAG_HAND = HAND_GRASP;
                Command_Grasp = GRASP_LIFT;
                des_LH.pos.x += dX;
                dX = 0.0;
            }
        }
        break;
    }
    case GRASP_LIFT:
    {
        static int cnt = 0;
        cnt++;
        if(MOVE_HAND == HAND_R)
        {
            if(cnt > 600-des_RH.sec*200 || (FLAG_HAND == HAND_DONE))
            {
                FLAG_HAND = HAND_STOP;
                Command_Grasp = GRASP_BACK;
                cnt = 0;
                FILE_LOG(logINFO) << "Grasp Done";
            }
        } else
        {
            if(cnt > 600-des_LH.sec*200 || (FLAG_HAND == HAND_DONE))
            {
                FLAG_HAND = HAND_STOP;
                Command_Grasp = GRASP_BACK;
                cnt = 0;
                FILE_LOG(logINFO) << "Grasp Done";
            }
        }
        break;
    }
    case GRASP_BACK:
    {
        static double dZ = 0.0;
        double MoveZ = in.Distance_GraspLiftZ_m;
        static double dX = 0.0;
        double MoveX = in.Distance_GraspLiftX_m;
        if(MOVE_HAND == HAND_R)
        {
            des_RH.sec = in.Time_MoveGraspLift_sec;
            doubles ds_InputORI(4);
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_RH.ori[i];
            }
            WBmotion->addRHOriInfo(ds_InputORI, 0.005);
            dX += MoveX*0.005/des_RH.sec;
            dZ += MoveZ*0.005/(des_RH.sec/1.5);
            WBmotion->addRHPosInfo(des_RH.pos.x+dX, des_RH.pos.y, des_RH.pos.z+dZ, 0.005);

            if(dX < MoveX && dZ > MoveZ)
            {
                Command_Grasp = GRASP_WALKREADY;
                des_RH.pos.x += dX;
                des_RH.pos.z += dZ;
                dX = 0.0;
                dZ = 0.0;
                FILE_LOG(logINFO) << "Grasp Done";
            }
        } else
        {
            des_LH.sec = in.Time_MoveGraspLift_sec;
            doubles ds_InputORI(4);
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_LH.ori[i];
            }
            WBmotion->addLHOriInfo(ds_InputORI,0.005);
            dX += MoveX*0.005/des_LH.sec;
            dZ += MoveZ*0.005/(des_LH.sec/1.5);
            WBmotion->addLHPosInfo(des_LH.pos.x+dX, des_LH.pos.y, des_LH.pos.z+dZ, 0.005);

            if(dX < MoveX && dZ > MoveZ)
            {
                Command_Grasp = GRASP_WALKREADY;
                des_LH.pos.x += dX;
                des_LH.pos.z += dZ;
                dX = 0.0;
                dZ = 0.0;
                FILE_LOG(logINFO) << "Grasp Done";
            }
        }
        break;
    }
    case GRASP_WALKREADY:
    {
        doubles ds_InputRORI(4), ds_InputLORI(4);
        for(int i=0;i<4;i++)
        {
            ds_InputRORI[i] = des_RH.ori[i];
            ds_InputLORI[i] = des_LH.ori[i];
        }

        printf("Lori = %f, %f, %f, %f\nRori = %f, %f, %f, %f\n",ds_InputLORI[0],ds_InputLORI[1],ds_InputLORI[2],ds_InputLORI[3],
                ds_InputRORI[0],ds_InputRORI[1],ds_InputRORI[2],ds_InputRORI[3]);

        if(MOVE_HAND == HAND_R)
        {
            des_RH.pos.x = in.Position_WalkReadyX;
            des_RH.pos.y = -in.Position_WalkReadyY;
            des_RH.pos.z = in.Position_WalkReadyZ;

            des_RH.elb = -in.Degree_ElbowWalkReady;
            des_RH.sec = in.Time_MoveWalkReady_sec;

            TRInfo temp;
            temp = new TrajectoryCosine(des_RH.sec/1.5, des_RH.pos.x);
            WBmotion->wbPosRH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec/2., des_RH.pos.y);
            WBmotion->wbPosRH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec, des_RH.pos.z);
            WBmotion->wbPosRH[2]->AddTrajInfo(temp);

            WBmotion->addRHOriInfo(ds_InputRORI, des_RH.sec);
            WBmotion->addRElbPosInfo(des_RH.elb, des_RH.sec);

        } else
        {
            des_LH.pos.x = in.Position_WalkReadyX;
            des_LH.pos.y = in.Position_WalkReadyY;
            des_LH.pos.z = in.Position_WalkReadyZ;

            des_LH.elb = in.Degree_ElbowWalkReady;
            des_LH.sec = in.Time_MoveWalkReady_sec;

            TRInfo temp;
            temp = new TrajectoryCosine(des_LH.sec/1.5, des_LH.pos.x);
            WBmotion->wbPosLH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec/2., des_LH.pos.y);
            WBmotion->wbPosLH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec, des_LH.pos.z);
            WBmotion->wbPosLH[2]->AddTrajInfo(temp);

            WBmotion->addLHOriInfo(ds_InputLORI, des_LH.sec);
            WBmotion->addLElbPosInfo(des_LH.elb, des_LH.sec);
        }
        Command_Grasp = GRASP_PUT;
        break;
    }
    case GRASP_PUT:
    {
        static int cnt = 0;
        cnt++;
        if(MOVE_HAND == HAND_R)
        {
            if(cnt > des_RH.sec*200)
            {
                Command_Grasp = GRASP_DONE;
                cnt = 0;
                FILE_LOG(logINFO) << "All Done";
            }
        } else
        {
            if(cnt > des_LH.sec*200)
            {
                Command_Grasp = GRASP_DONE;
                cnt = 0;
                FILE_LOG(logINFO) << "All Done";
            }
        }
        break;
    }
    case PUT_START:
    {
        doubles ds_InputORI(4);
        if(MOVE_HAND == HAND_R)
        {
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_RH.ori[i];
            }
            if(Mode_PutEarly == true)
            {
                des_RH.sec = OMNIinfo.GoalSec/2.;//in.Time_MovePutStart_sec;

            } else {
                des_RH.sec = in.Time_MovePutStart_sec;
            }
            TRInfo temp;
            temp = new TrajectoryCosine(des_RH.sec/1.5, des_RH.pos.x);
            WBmotion->wbPosRH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec, des_RH.pos.y);
            WBmotion->wbPosRH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec/2., des_RH.pos.z);
            WBmotion->wbPosRH[2]->AddTrajInfo(temp);
            WBmotion->addRHOriInfo(ds_InputORI, des_RH.sec);
            WBmotion->addRElbPosInfo(des_RH.elb, des_RH.sec/1.5);
        } else
        {
            for(int i=0;i<4;i++)
            {
                ds_InputORI[i] = des_LH.ori[i];
            }
            if(Mode_PutEarly == true)
            {
                des_LH.sec = OMNIinfo.GoalSec/2.;//in.Time_MovePutStart_sec;

            } else {
                des_LH.sec = in.Time_MovePutStart_sec;
            }
            TRInfo temp;
            temp = new TrajectoryCosine(des_LH.sec/1.5, des_LH.pos.x);
            WBmotion->wbPosLH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec, des_LH.pos.y);
            WBmotion->wbPosLH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec/2., des_LH.pos.z);
            WBmotion->wbPosLH[2]->AddTrajInfo(temp);
            WBmotion->addLHOriInfo(ds_InputORI, des_LH.sec);
            WBmotion->addLElbPosInfo(des_LH.elb, des_LH.sec/1.5);
        }

        if(Mode_PutEarly == true)
        {
            Command_Grasp = PUT_READY;

        } else{

            Command_Grasp = PUT_PUT;
        }
        break;
    }
    case PUT_PUT:
    {
        static int cnt = 0;
        if(MOVE_HAND == HAND_R)
        {
            if(cnt == 0)
            {
                FLAG_HAND = HAND_OPEN;
                GRASP_HAND = HAND_R;
            }
            cnt++;

            if(FLAG_HAND == HAND_DONE || cnt > 100)
            {
                Command_Grasp = PUT_BACK;
                cnt = 0;
                FILE_LOG(logINFO) << "Put Done";
            }
        } else
        {
            if(cnt == 0)
            {
                FLAG_HAND = HAND_OPEN;
                GRASP_HAND = HAND_L;
            }
            cnt++;

            if(FLAG_HAND == HAND_DONE || cnt > 100)
            {
                Command_Grasp = PUT_BACK;
                cnt = 0;
                FILE_LOG(logINFO) << "Put Done";
            }
        }
        break;
    }
    case PUT_BACK:
    {
        static double dX = 0.0;
        double MoveX = in.Distance_GraspLiftX_m;
        if(MOVE_HAND == HAND_R)
        {
            des_RH.sec = in.Time_MovePutBack_sec;

            dX += MoveX*0.005/des_RH.sec;
            WBmotion->addRHPosInfo(des_RH.pos.x+dX, des_RH.pos.y, des_RH.pos.z, 0.005);

            if(dX < MoveX)
            {
                FLAG_HAND = HAND_GRASP;
                Command_Grasp = PUT_WALKREADY;
                des_RH.pos.x += dX;
                dX = 0.0;
            }
        } else
        {
            des_LH.sec = in.Time_MovePutBack_sec;

            dX += MoveX*0.005/des_LH.sec;
            WBmotion->addLHPosInfo(des_LH.pos.x+dX, des_LH.pos.y, des_LH.pos.z, 0.005);

            if(dX < MoveX)
            {
                FLAG_HAND = HAND_GRASP;
                Command_Grasp = PUT_WALKREADY;
                des_LH.pos.x += dX;
                dX = 0.0;
            }
        }

        break;
    }
    case PUT_WALKREADY:
    {
        doubles ds_InputRORI(4), ds_InputLORI(4);
        printf("walkready\n");

        if(MOVE_HAND == HAND_R)
        {
            des_RH.ori[0] = 0.707;
            des_RH.ori[1] = 0.0;
            des_RH.ori[2] = -0.707;
            des_RH.ori[3] = 0.0;
            for(int i=0;i<4;i++)
            {
                ds_InputRORI[i] = des_RH.ori[i];
            }
            des_RH.pos.x = in.Position_WalkReadyX;
            des_RH.pos.y = -in.Position_WalkReadyY;
            des_RH.pos.z = in.Position_WalkReadyZ;

            des_RH.elb = -in.Degree_ElbowWalkReady;
            des_RH.sec = in.Time_MoveWalkReady_sec;
            TRInfo temp;
            temp = new TrajectoryCosine(des_RH.sec/1.5, des_RH.pos.x);
            WBmotion->wbPosRH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec/3., des_RH.pos.y);
            WBmotion->wbPosRH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_RH.sec, des_RH.pos.z);
            WBmotion->wbPosRH[2]->AddTrajInfo(temp);
            WBmotion->addRHOriInfo(ds_InputRORI, des_RH.sec);
            WBmotion->addRElbPosInfo(des_RH.elb, des_RH.sec);
        } else
        {
            des_LH.ori[0] = 0.707;
            des_LH.ori[1] = 0.0;
            des_LH.ori[2] = -0.707;
            des_LH.ori[3] = 0.0;
            for(int i=0;i<4;i++)
            {
                ds_InputLORI[i] = des_LH.ori[i];
            }
            des_LH.pos.x = in.Position_WalkReadyX;
            des_LH.pos.y = in.Position_WalkReadyY;
            des_LH.pos.z = in.Position_WalkReadyZ;

            des_LH.elb = in.Degree_ElbowWalkReady;
            des_LH.sec = in.Time_MoveWalkReady_sec;
            TRInfo temp;
            temp = new TrajectoryCosine(des_LH.sec/1.5, des_LH.pos.x);
            WBmotion->wbPosLH[0]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec/3., des_LH.pos.y);
            WBmotion->wbPosLH[1]->AddTrajInfo(temp);
            temp = new TrajectoryCosine(des_LH.sec, des_LH.pos.z);
            WBmotion->wbPosLH[2]->AddTrajInfo(temp);
            WBmotion->addLHOriInfo(ds_InputLORI, des_LH.sec);
            WBmotion->addLElbPosInfo(des_LH.elb, des_LH.sec);
        }

        Command_Grasp = PUT_DONE;
        break;
    }
    case PUT_DONE:
    {
        FLAG_HAND = HAND_GRASP;
        Command_Grasp = GRASP_NO_ACT;
        FILE_LOG(logSUCCESS) << "Put Done";
        break;
    }
    default:
    {
        break;
    }
    }

}

void ShowWBInfos()
{
    printf("======================= WBInfos =======================\n");
    printf("LHPos = (%f, %f, %f)\n",WBmotion->pLH_3x1[0],WBmotion->pLH_3x1[1],WBmotion->pLH_3x1[2]);
    printf("LHOri = (%f, %f, %f, %f)\n",WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
    printf("RHPos = (%f, %f, %f)\n",WBmotion->pRH_3x1[0],WBmotion->pRH_3x1[1],WBmotion->pRH_3x1[2]);
    printf("RHOri = (%f, %f, %f, %f)\n",WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    printf("PelPos = (%f, %f, %f)\n",WBmotion->pCOM_2x1[0],WBmotion->pCOM_2x1[1],WBmotion->pPelZ);
    printf("PelOri = (%f, %f, %f, %f)\n",WBmotion->qPEL_4x1[0],WBmotion->qPEL_4x1[1],WBmotion->qPEL_4x1[2],WBmotion->qPEL_4x1[3]);
    printf("=======================================================\n\n");
}

int SetGoalPos()
{
    ROBOTinfo.GoalPos.X = ROBOTinfo.SLAMPos.X - OMNIinfo.Pin.X;
    ROBOTinfo.GoalPos.Y = ROBOTinfo.SLAMPos.Y - OMNIinfo.Pin.Y;

    if(Command_WSTrot == WST_ZERO || FLAG_PosReset == true)
    {
        ROBOTinfo.GoalPos.Theta = 0.0;//ROBOTinfo.SLAMPos.Theta + OMNIinfo.Pin.Theta;
    } else if(Command_WSTrot == WST_BACK)
    {
        ROBOTinfo.GoalPos.Theta = 180.0*D2R;
    } else
    {
        ROBOTinfo.GoalPos.Theta = ROBOTinfo.GoalPos.Theta + OMNIinfo.Pin.Theta;
    }


    if(ROBOTinfo.GoalPos.Theta > 180.0*D2R)
    {
        ROBOTinfo.GoalPos.Theta = ROBOTinfo.GoalPos.Theta - 360.0*D2R;
    } else if(ROBOTinfo.GoalPos.Theta < -180.0*D2R)
    {
        ROBOTinfo.GoalPos.Theta = ROBOTinfo.GoalPos.Theta + 360.0*D2R;
    }


    printf("GoalX = %f, SLAMX = %f, DifX = %f\n",ROBOTinfo.GoalPos.X, ROBOTinfo.SLAMPos.X, OMNIinfo.Pin.X);
    printf("GoalY = %f, SLAMY = %f, DifY = %f\n",ROBOTinfo.GoalPos.Y, ROBOTinfo.SLAMPos.Y, OMNIinfo.Pin.Y);
    printf("GoalTheta = %f, SLAMTheta = %f, DifTheta = %f\n",ROBOTinfo.GoalPos.Theta, ROBOTinfo.SLAMPos.Theta, OMNIinfo.Pin.Theta);
    return true;
}

void ResetPos()
{
    FLAG_PosReset = true;
    if(fabs(userData->R2M.robot_pos.pos_x) > 0.02 && fabs(userData->R2M.robot_pos.pos_x) < 0.2)
        OMNIinfo.Pin.X      = userData->R2M.robot_pos.pos_x;

    if(fabs(userData->R2M.robot_pos.pos_y) > 0.02 && fabs(userData->R2M.robot_pos.pos_y) < 0.25)
        OMNIinfo.Pin.Y      = userData->R2M.robot_pos.pos_y;

    if(fabs(userData->R2M.robot_pos.ori_w) > 3.0)
        OMNIinfo.Pin.Theta  = -userData->R2M.robot_pos.ori_w*D2R;
    //OMNIinfo.GoalSec    = 2.0;

    SetGoalPos();
    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwnerWHEEL();

    printf("Robot Cur Pos : %f, %f, %f\n",OMNIinfo.Pin.X,OMNIinfo.Pin.Y,OMNIinfo.Pin.Theta);
    FLAG_OMNI = 1;
    Mode_OMNIMove = MOVE_OMNI;
    userData->M2R.MOTION_STATE = WORKING;
}

void SLAMreset()
{
    FLAG_SLAMReset = true;
    OMNIinfo.Pin.X = 0.;
    OMNIinfo.Pin.Y = 0.;
    OMNIinfo.Pin.Theta = 360.*D2R;

    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwnerWHEEL();

    FLAG_OMNI = 1;
    Mode_OMNIMove = MOVE_OMNI;
    userData->M2R.MOTION_STATE = WORKING;
}

int SetRotation(int _wst)
{
    return true;
    static int cnt = 0;
    if(cnt == 0)
    {
        FLAG_RotReset = true;
    }

    switch(_wst)
    {
    case WST_ZERO:
    {
        if(IsSLAMYAWOK() == true)
        {
            printf("yaw ok\n");
            FLAG_RotReset = false;
            break;
        }
        InitWheelInfo();
        OMNIinfo.Pin.X      = 0.0;
        OMNIinfo.Pin.Y      = 0.0;
        OMNIinfo.GoalSec    = in.Time_Compensation_sec;
        Command_WSTrot = WST_ZERO;
        FLAG_OMNI = 1;
        Mode_OMNIMove = MOVE_OMNI;
        break;
    }
    case WST_BACK:
    {
        if(IsSLAMYAWOK() == true)
        {
            FLAG_RotReset = false;
            break;
        }
        InitWheelInfo();
        OMNIinfo.Pin.X      = 0.0;
        OMNIinfo.Pin.Y      = 0.0;
        IsSLAMYAWOK();
        OMNIinfo.GoalSec    = in.Time_Compensation_sec;
        Command_WSTrot = WST_BACK;
        FLAG_OMNI = 1;
        Mode_OMNIMove = MOVE_OMNI;
        break;
    }
    default:
        break;
    }
    cnt++;

    if(FLAG_RotReset == false)
    {
        return true;
    }
    return false;
}

void GO_Grasp()
{
    ShowWBInfos();

    doubles ds_InputORI(4);
    if(MOVE_HAND == HAND_R)
    {
        for(int i=0;i<4;i++)
        {
            ds_InputORI[i] = des_RH.ori[i];
        }

        WBmotion->addRHPosInfo(des_RH.pos[0],des_RH.pos[1],des_RH.pos[2],des_RH.sec);
        WBmotion->addRHOriInfo(ds_InputORI,des_RH.sec);
        WBmotion->addRElbPosInfo(des_RH.elb,des_RH.sec);
        printf("====================Right Hand Move Test=======================\n");
        printf("Quaternion angle = %f, %f, %f, %f\n",ds_InputORI[0],ds_InputORI[1],ds_InputORI[2],ds_InputORI[3]);
        printf("Elbow angle = %f\n",des_RH.elb);
    } else
    {
        for(int i=0;i<4;i++)
        {
            ds_InputORI[i] = des_LH.ori[i];
        }

        WBmotion->addLHPosInfo(des_LH.pos[0],des_LH.pos[1],des_LH.pos[2],des_LH.sec);
        WBmotion->addLHOriInfo(ds_InputORI,des_LH.sec);
        WBmotion->addLElbPosInfo(des_LH.elb,des_LH.sec);
        printf("====================Left Hand Move Test=======================\n");
        printf("Quaternion angle = %f, %f, %f, %f\n",ds_InputORI[0],ds_InputORI[1],ds_InputORI[2],ds_InputORI[3]);
        printf("Elbow angle = %f\n",des_LH.elb);
    }

    MOVE_HAND = false;

}

void ShutDownAllFlag()
{
    FLAG_HAND = HAND_NO_ACT;
    Command_Grasp = GRASP_NO_ACT;
    Mode_OMNIMove = NOT_MOVE;
    Mode_HANDMove = NOT_MOVE;
    Command_WSTrot = WST_STOP;
    FLAG_OMNI = false;
    FLAG_WSTtime = false;
    GRASP_HAND = false;
    MOVE_HAND = false;
    WB_FLAG = false;
    OnOff_YawCompen = true;
}

void InitWheelInfo()
{
    RWHinfo.InitRef_Deg = sharedSEN->ENCODER[MC_ID_CH_Pairs[RWH].id][MC_ID_CH_Pairs[RWH].ch].CurrentReference;
    LWHinfo.InitRef_Deg = sharedSEN->ENCODER[MC_ID_CH_Pairs[LWH].id][MC_ID_CH_Pairs[LWH].ch].CurrentReference;
    BWHinfo.InitRef_Deg = sharedSEN->ENCODER[MC_ID_CH_Pairs[BWH].id][MC_ID_CH_Pairs[BWH].ch].CurrentReference;

    SetOMNIpara(INIT);


    if(FLAG_PosReset == true)
    {
        FILE_LOG(logWARNING) << "Reset Robot Global Pos";
        FLAG_PosReset = false;
    }
    for(int i=0;i<120;i++)
    {
        pushData(RWHList, 0.0);
        pushData(LWHList, 0.0);
        pushData(BWHList, 0.0);
    }
    RWHnow = 0.;
    LWHnow = 0.;
    BWHnow = 0.;
    RWHvel = 0.;
    LWHvel = 0.;
    BWHvel = 0.;

    Move_X = 0.;
    Move_Y = 0.;
    Move_R = 0.;

    JOY_LJOG_RL = 0.;
    JOY_LJOG_UD = 0.;
    JOY_RJOG_RL = 0.;


    RWHinfo.MoveDistance_m = 0.;
    RWHinfo.WheelVel_ms = 0.;

    LWHinfo.MoveDistance_m = 0.;
    LWHinfo.WheelVel_ms = 0.;

    BWHinfo.MoveDistance_m = 0.;
    BWHinfo.WheelVel_ms = 0.;

    OMNIinfo.Pin.X = 0.0;
    OMNIinfo.Pin.Y = 0.0;
    OMNIinfo.Pin.Theta = 0.0;

    OMNIinfo.Probot.X = 0.0;
    OMNIinfo.Probot.Y = 0.0;
    OMNIinfo.Probot.Theta = 0.0;

    OMNIinfo.Vrobot.X = 0.0;
    OMNIinfo.Vrobot.Y = 0.0;
    OMNIinfo.Vrobot.Theta = 0.0;

    OMNIinfo.CurSec = 0.0;
    OMNIinfo.GoalSec = 0.0;
    OMNIinfo.GoalSecR = 0.0;
    OMNIinfo.SatSec = 0.0;
    OMNIinfo.SatSecR = 0.0;
    OMNIinfo.TrajSec = 0.0;
    OMNIinfo.TrajSecR = 0.0;

}

void InitializeDrinkStock()
{
    drinkstock.setCOL(1);
    drinkstock.AddDrinkFirst(VITAMINWATER, 0);
    drinkstock.AddDrinkFirst(VITAMINWATER, 1);
    drinkstock.AddDrinkFirst(VITAMINWATER, 2);
    drinkstock.AddDrinkFirst(SPRITE, 3);
    drinkstock.AddDrinkFirst(SPRITE, 4);
    drinkstock.AddDrinkFirst(SPRITE, 5);
    drinkstock.AddDrinkFirst(SAMDASOO, 6);
    drinkstock.AddDrinkFirst(SAMDASOO, 7);
    drinkstock.AddDrinkFirst(SAMDASOO, 8);
    drinkstock.ShowStock();
}

void CalculateMovingEverage(void)
{
    double temp1 = 0.0;
    double temp2 = 0.0;
    double temp3 = 0.0;
    for(int i=0; i<120; i++){
        temp1 += RWHList[i];
        temp2 += LWHList[i];
        temp3 += BWHList[i];
    }
    RWHvel = temp1/120.0;
    LWHvel = temp2/120.0;
    BWHvel = temp3/120.0;
}

int IsPosXSafe()
{
    if(MOVE_HAND == HAND_L)
    {
        if(des_LH.pos.x > in.Limit_ObjectXLower && des_LH.pos.x < in.Limit_ObjectXUpper)
            return true;
    } else
    {
        if(des_RH.pos.x > in.Limit_ObjectXLower && des_RH.pos.x < in.Limit_ObjectXUpper)
            return true;
    }
    return false;
}

int IsPosYSafe()
{
    if(MOVE_HAND == HAND_L)
    {
        if(des_LH.pos.y > in.Limit_ObjectYLower && des_LH.pos.y < in.Limit_ObjectYUpper)
            return true;
    } else
    {
        if(des_RH.pos.y < -in.Limit_ObjectYLower && des_RH.pos.y > -in.Limit_ObjectYUpper)
            return true;
    }
    return false;
}

int IsPosZSafe()
{
    if(MOVE_HAND == HAND_L)
    {
        if(des_LH.pos.z > in.Limit_ObjectZLower && des_LH.pos.z < in.Limit_ObjectZUpper)
            return true;
    } else
    {
        if(des_RH.pos.z > in.Limit_ObjectZLower && des_RH.pos.z < in.Limit_ObjectZUpper)
            return true;
    }
    return false;
}

int IsElbAngleSafe()
{
    if(MOVE_HAND == HAND_L)
    {
        if(des_LH.elb > -0.001)
            return true;
    } else
    {
        if(des_RH.elb < 0.001)
            return true;
    }
    return false;
}

int IsSLAMXOK()
{
    double deltaX = ROBOTinfo.GoalPos.X - ROBOTinfo.SLAMPos.X;
    if(fabs(deltaX) > 0.03)
    {
        OMNIinfo.Pin.X = deltaX;
        return false;
    }
    OMNIinfo.Pin.X = 0.;
    return true;
}

int IsSLAMYOK()
{
    double deltaY = ROBOTinfo.GoalPos.Y - ROBOTinfo.SLAMPos.Y;
    if(fabs(deltaY) > 0.03)
    {
        OMNIinfo.Pin.Y = deltaY;
        return false;
    }
    OMNIinfo.Pin.Y = 0.;
    return true;
}

int IsSLAMVOK()
{
    double gx = ROBOTinfo.GoalPos.X;
    double gy = ROBOTinfo.GoalPos.Y;
    double sx = ROBOTinfo.SLAMPos.X;
    double sy = ROBOTinfo.SLAMPos.Y;
    double th = ROBOTinfo.SLAMPos.Theta;

    double gv = sqrt(gx*gx + gy*gy);
    double sv = sqrt(sx*sx + sy*sy);

    double deltaV = gv - sv;

    if(fabs(deltaV) > 0.03)
    {
        double dx = cos(th)*(gx-sx) - sin(th)*(gy-sy);
        double dy = sin(th)*(gy-sy) + cos(th)*(gy-sy);

        printf("V rotate:: orin dx = %f, dy = %f ||| rot dx = %f, dy = %f\n",gx-sx, gy-sy, dx,dy);
        OMNIinfo.Pin.X = -dx;
        OMNIinfo.Pin.Y = dy;
        return false;
    }
    OMNIinfo.Pin.X = 0.;
    OMNIinfo.Pin.Y = 0.;
    return true;
}

int IsSLAMYAWOK()
{
    if(OnOff_YawCompen == false)
        return true;

    double deltaYaw;
    usleep(500*1000);
    if(FLAG_RotReset == true)
    {
        deltaYaw = 0.0 - ROBOTinfo.SLAMPos.Theta;
    }else
    {
        deltaYaw = ROBOTinfo.GoalPos.Theta - ROBOTinfo.SLAMPos.Theta;
    }

    if(deltaYaw > 180.0*D2R)
    {
        deltaYaw = deltaYaw - 360.0*D2R;
    } else if(deltaYaw < -180.0*D2R)
    {
        deltaYaw = deltaYaw + 360.0*D2R;
    }

    printf("Yaw Goal : %f, Slam : %f, Dif : %f (%f)\n",ROBOTinfo.GoalPos.Theta, ROBOTinfo.SLAMPos.Theta, deltaYaw, 0.5*D2R);
    if(fabs(deltaYaw) > 0.5*D2R)
    {

        OMNIinfo.Pin.Theta = deltaYaw;
        return false;
    }
    OMNIinfo.Pin.Theta = 0.;
    return true;
}

int IsSLAMRotOK()
{
    double deltaYaw = ROBOTinfo.GoalPos.Theta - ROBOTinfo.SLAMPos.Theta;

    if(deltaYaw > 180.0*D2R)
    {
        deltaYaw = deltaYaw - 360.0*D2R;
    } else if(deltaYaw < -180.0*D2R)
    {
        deltaYaw = deltaYaw + 360.0*D2R;
    }

    printf("Yaw Goal : %f, Slam : %f, Dif : %f\n",ROBOTinfo.GoalPos.Theta, ROBOTinfo.SLAMPos.Theta, deltaYaw);
    if(fabs(deltaYaw) > 2.0*D2R)
    {
        OMNIinfo.Pin.Theta = deltaYaw;
        return false;
    }
    OMNIinfo.Pin.Theta = 0.;
    return true;
}

int IsRealPos()
{
    static int cnt = 0;
    if(Command_WSTrot == WST_BACK)
    {
        printf("wst_back\n");
        cnt = 0;
        return true;
    }
    printf("isrealpos %d\n",cnt);
    if(cnt == 0)
    {
        cnt++;
        int isOKyaw = IsSLAMYAWOK();
        if(isOKyaw == false)
        {
            FLAG_OMNI = 1;
            OMNIinfo.Pin.X      = 0.0;
            OMNIinfo.Pin.Y      = 0.0;
            OMNIinfo.GoalSec = 2.0;
            printf("SLAM ERROR : %f, %f, %f\n",OMNIinfo.Pin.X,OMNIinfo.Pin.Y, OMNIinfo.Pin.Theta);
            return false;
        }
        printf("yaw true\n");
        cnt = 0;
        return true;
    } else
    {
        printf("twice\n");
        cnt = 0;
        return true;
    }

}

/************************** OmniWheel Move *****************************/
void Omni_CalVel_Robot()
{
    static int cnt = 0;
    double RobotXSatvel     = (OMNIinfo.Pin.X/(OMNIinfo.TrajSec - (1/PI)*sin(PI)*OMNIinfo.TrajSec + OMNIinfo.SatSec));
    double RobotYSatvel     = (OMNIinfo.Pin.Y/(OMNIinfo.TrajSec -  (1/PI)*sin(PI)*OMNIinfo.TrajSec + OMNIinfo.SatSec));
    double RobotRSatvel     = (OMNIinfo.Pin.Theta/(OMNIinfo.TrajSec -  (1/PI)*sin(PI)*OMNIinfo.TrajSec + OMNIinfo.SatSec));

    double RobotTrajMoveX   = RobotXSatvel*0.5*(1 - (1/PI)*sin(PI))*OMNIinfo.TrajSec;
    double RobotTrajMoveY   = RobotYSatvel*0.5*(1 - (1/PI)*sin(PI))*OMNIinfo.TrajSec;
    double RobotTrajMoveR   = RobotRSatvel*0.5*(1 - (1/PI)*sin(PI))*OMNIinfo.TrajSec;

    if(OMNIinfo.CurSec <= OMNIinfo.TrajSec)
    {
        FLAG_WSTtime = true;
        double NSec = OMNIinfo.CurSec/OMNIinfo.TrajSec;
        OMNIinfo.Vrobot.X = Traj(NSec, RobotXSatvel);
        OMNIinfo.Vrobot.Y = Traj(NSec, RobotYSatvel);
        OMNIinfo.Vrobot.Theta = Traj(NSec, RobotRSatvel);

        OMNIinfo.Probot.X = RobotXSatvel*0.5*(NSec - (1/PI)*sin(PI*NSec))*OMNIinfo.TrajSec;
        OMNIinfo.Probot.Y = RobotYSatvel*0.5*(NSec - (1/PI)*sin(PI*NSec))*OMNIinfo.TrajSec;
        OMNIinfo.Probot.Theta = RobotRSatvel*0.5*(NSec - (1/PI)*sin(PI*NSec))*OMNIinfo.TrajSec;
    } else if(OMNIinfo.CurSec <= OMNIinfo.GoalSec - OMNIinfo.TrajSec)
    {
        double TempSec = OMNIinfo.CurSec-OMNIinfo.TrajSec;
        OMNIinfo.Vrobot.X = RobotXSatvel;
        OMNIinfo.Vrobot.Y = RobotYSatvel;
        OMNIinfo.Vrobot.Theta = RobotRSatvel;

        OMNIinfo.Probot.X = RobotTrajMoveX + RobotXSatvel*TempSec;
        OMNIinfo.Probot.Y = RobotTrajMoveY + RobotYSatvel*TempSec;
        OMNIinfo.Probot.Theta = RobotTrajMoveR + RobotRSatvel*TempSec;

        if(OMNIinfo.CurSec > OMNIinfo.SatSec)
        {
            if(Mode_PutEarly == true && cnt == 0)
            {
                cnt = 1;
//                Mode_PutEarly = false;
                usleep(20*1000);
                if(userData->R2M.Objpos[0].pos_y > 0.0)
                {
                    MOVE_HAND = HAND_L;
                    des_LH.pos.x = in.Position_PutX;
                    des_LH.pos.y = in.Position_PutY;
                    des_LH.pos.z = in.Position_PutZ + in.Offset_PutZ;

                    des_LH.elb   = in.Degree_ElbowGrasp;

                    printf("LTarget pos = %f, %f, %f\n",des_LH.pos.x,des_LH.pos.y,des_LH.pos.z);
                    des_LH.ori.w = in.Quaternion_LGraspW;
                    des_LH.ori.x = in.Quaternion_LGraspX;
                    des_LH.ori.y = in.Quaternion_LGraspY;
                    des_LH.ori.z = in.Quaternion_LGraspZ;
                } else
                {
                    MOVE_HAND = HAND_R;
                    des_RH.pos.x = in.Position_PutX;
                    des_RH.pos.y = -in.Position_PutY;
                    des_RH.pos.z = in.Position_PutZ + in.Offset_PutZ;

                    des_RH.elb   = -in.Degree_ElbowGrasp;

                    printf("RTarget pos = %f, %f, %f\n",des_RH.pos.x,des_RH.pos.y,des_RH.pos.z);
                    des_RH.ori.w = in.Quaternion_RGraspW;
                    des_RH.ori.x = in.Quaternion_RGraspX;
                    des_RH.ori.y = in.Quaternion_RGraspY;
                    des_RH.ori.z = in.Quaternion_RGraspZ;
                }
                Command_Grasp = PUT_START;
                Mode_HANDMove = MOVE_VISION;
            }
        }
    } else if(OMNIinfo.CurSec <= OMNIinfo.GoalSec)
    {
        cnt = 0;

        double TempSec = OMNIinfo.CurSec-(OMNIinfo.GoalSec-OMNIinfo.TrajSec);
        double NSec = TempSec/OMNIinfo.TrajSec;
        OMNIinfo.Vrobot.X = RobotXSatvel - Traj(NSec, RobotXSatvel);
        OMNIinfo.Vrobot.Y = RobotYSatvel - Traj(NSec, RobotYSatvel);
        OMNIinfo.Vrobot.Theta = RobotRSatvel - Traj(NSec, RobotRSatvel);

        OMNIinfo.Probot.X = RobotTrajMoveX + RobotXSatvel*OMNIinfo.SatSec  +  (RobotXSatvel*0.5*(NSec + (1/PI)*sin(PI*NSec)))*OMNIinfo.TrajSec;
        OMNIinfo.Probot.Y = RobotTrajMoveY + RobotYSatvel*OMNIinfo.SatSec  +  (RobotYSatvel*0.5*(NSec + (1/PI)*sin(PI*NSec)))*OMNIinfo.TrajSec;
        OMNIinfo.Probot.Theta = RobotTrajMoveR + RobotRSatvel*OMNIinfo.SatSec + (RobotRSatvel*0.5*(NSec + (1/PI)*sin(PI*NSec)))*OMNIinfo.TrajSec;
    } else
    {
        printf("CurSec is - or nan");
    }
}

void Omni_CalVel_Robot5th()
{
    static int cnt = 0;
    double vmaxX = OMNIinfo.Pin.X*(OMNIinfo.Vms/OMNIinfo.Pin.M);
    double vmaxY = OMNIinfo.Pin.Y*(OMNIinfo.Vms/OMNIinfo.Pin.M);
    double vmaxR;
    if(OMNIinfo.Pin.Theta > 0.)
        vmaxR = OMNIinfo.Vrads;
    else
        vmaxR = -OMNIinfo.Vrads;
    /* Move Rotate First !! */
    if(OMNIinfo.CurSec >= 0. && OMNIinfo.CurSec < OMNIinfo.TrajSecR)
    {
        double t = OMNIinfo.CurSec;
        FLAG_WSTtime = false;
        OMNIinfo.Probot.Theta = Ra3*t*t*t + Ra4*t*t*t*t + Ra5*t*t*t*t*t;
        OMNIinfo.Vrobot.Theta = 3*Ra3*t*t + 4*Ra4*t*t*t + 5*Ra5*t*t*t*t;
    } else if(OMNIinfo.CurSec < OMNIinfo.TrajSecR + OMNIinfo.SatSecR)
    {
        FLAG_WSTtime = false;
        double t = OMNIinfo.TrajSecR;
        double nt = OMNIinfo.CurSec - OMNIinfo.TrajSecR;

        if(nt <= OMNIinfo.SatSecR)
        {
            OMNIinfo.Probot.Theta =     Ra3*t*t*t + Ra4*t*t*t*t + Ra5*t*t*t*t*t + vmaxR*(nt);
            OMNIinfo.Vrobot.Theta =     vmaxR;
        }
    } else if(OMNIinfo.CurSec < OMNIinfo.GoalSecR)
    {
        FLAG_WSTtime = false;
        double t = OMNIinfo.CurSec - OMNIinfo.SatSecR;
        OMNIinfo.Probot.Theta = Ra3*t*t*t + Ra4*t*t*t*t + Ra5*t*t*t*t*t + vmaxR*OMNIinfo.SatSecR;
        OMNIinfo.Vrobot.Theta = 3*Ra3*t*t + 4*Ra4*t*t*t + 5*Ra5*t*t*t*t;
    } else if(OMNIinfo.CurSec < OMNIinfo.GoalSecR + OMNIinfo.TrajSec)
    {
        double t = OMNIinfo.CurSec - OMNIinfo.GoalSecR;
        FLAG_WSTtime = true;
        OMNIinfo.Probot.X =     Xa3*t*t*t + Xa4*t*t*t*t + Xa5*t*t*t*t*t;
        OMNIinfo.Probot.Y =     Ya3*t*t*t + Ya4*t*t*t*t + Ya5*t*t*t*t*t;

        OMNIinfo.Vrobot.X =     3*Xa3*t*t + 4*Xa4*t*t*t + 5*Xa5*t*t*t*t;
        OMNIinfo.Vrobot.Y =     3*Ya3*t*t + 4*Ya4*t*t*t + 5*Ya5*t*t*t*t;
    } else if(OMNIinfo.CurSec < OMNIinfo.GoalSecR + OMNIinfo.TrajSec + OMNIinfo.SatSec)
    {
//        FLAG_WSTtime = true;
        double t = OMNIinfo.TrajSec;
        double nt = OMNIinfo.CurSec - OMNIinfo.TrajSec- OMNIinfo.GoalSecR;

        if(nt <= OMNIinfo.SatSec)
        {
            OMNIinfo.Probot.X =     Xa3*t*t*t + Xa4*t*t*t*t + Xa5*t*t*t*t*t + vmaxX*(nt);
            OMNIinfo.Probot.Y =     Ya3*t*t*t + Ya4*t*t*t*t + Ya5*t*t*t*t*t + vmaxY*(nt);
            OMNIinfo.Vrobot.X =     vmaxX;
            OMNIinfo.Vrobot.Y =     vmaxY;
        }

        if(OMNIinfo.CurSec > (OMNIinfo.GoalSec-OMNIinfo.GoalSecR)/2.)
        {
            if(Mode_PutEarly == true && cnt == 0)
            {
                cnt = 1;
//                Mode_PutEarly = false;
                usleep(20*1000);
                if(userData->R2M.Objpos[0].pos_y > 0.0)
                {
                    MOVE_HAND = HAND_L;
                    des_LH.pos.x = in.Position_PutX;
                    des_LH.pos.y = in.Position_PutY;
                    des_LH.pos.z = in.Position_PutZ;

                    des_LH.elb   = in.Degree_ElbowGrasp;

                    printf("LTarget pos = %f, %f, %f\n",des_LH.pos.x,des_LH.pos.y,des_LH.pos.z);
                    des_LH.ori.w = in.Quaternion_LGraspW;
                    des_LH.ori.x = in.Quaternion_LGraspX;
                    des_LH.ori.y = in.Quaternion_LGraspY;
                    des_LH.ori.z = in.Quaternion_LGraspZ;
                } else
                {
                    MOVE_HAND = HAND_R;
                    des_RH.pos.x = in.Position_PutX;
                    des_RH.pos.y = -in.Position_PutY;
                    des_RH.pos.z = in.Position_PutZ;

                    des_RH.elb   = -in.Degree_ElbowGrasp;

                    printf("RTarget pos = %f, %f, %f\n",des_RH.pos.x,des_RH.pos.y,des_RH.pos.z);
                    des_RH.ori.w = in.Quaternion_RGraspW;
                    des_RH.ori.x = in.Quaternion_RGraspX;
                    des_RH.ori.y = in.Quaternion_RGraspY;
                    des_RH.ori.z = in.Quaternion_RGraspZ;
                }
                Command_Grasp = PUT_START;
                Mode_HANDMove = MOVE_VISION;
            }
        }
    } else if(OMNIinfo.CurSec < OMNIinfo.GoalSec)
    {
//        FLAG_WSTtime = false;
        cnt = 0;
        double t = OMNIinfo.CurSec - OMNIinfo.SatSec - OMNIinfo.GoalSecR;
        OMNIinfo.Probot.X = Xa3*t*t*t + Xa4*t*t*t*t + Xa5*t*t*t*t*t + vmaxX*OMNIinfo.SatSec;
        OMNIinfo.Probot.Y = Ya3*t*t*t + Ya4*t*t*t*t + Ya5*t*t*t*t*t + vmaxY*OMNIinfo.SatSec;

        OMNIinfo.Vrobot.X = 3*Xa3*t*t + 4*Xa4*t*t*t + 5*Xa5*t*t*t*t;
        OMNIinfo.Vrobot.Y = 3*Ya3*t*t + 4*Ya4*t*t*t + 5*Ya5*t*t*t*t;
    } else
    {
        printf("CurSec is - or nan");
    }
}

void Omni_CalVel_Wheel()
{
    mat3 Tr = mat3(sin(Pw.alpha1), -cos(Pw.alpha1), Pw.RobotRm,
                   sin(Pw.alpha2), -cos(Pw.alpha2), Pw.RobotRm,
                   sin(Pw.alpha3), -cos(Pw.alpha3), Pw.RobotRm);

    /* rotation x,y by theta */
    mat3 Tth = mat3(cos(OMNIinfo.Probot.Theta), -sin(OMNIinfo.Probot.Theta), 0,
                    sin(OMNIinfo.Probot.Theta), cos(OMNIinfo.Probot.Theta), 0,
                    0, 0, 1);
    vec3 Temp = vec3(OMNIinfo.Vrobot.X, OMNIinfo.Vrobot.Y, 0);
    Temp = vec3(Tth*Temp);
    OMNIinfo.Vrobot.X = Temp[0];
    OMNIinfo.Vrobot.Y = Temp[1];
    vec3 Vr = vec3(OMNIinfo.Vrobot.X, OMNIinfo.Vrobot.Y, OMNIinfo.Vrobot.Theta);

    vec3 temp = vec3(Tr*Vr);

    LWHinfo.WheelVel_ms = temp[0]/Pw.WheelRm;
    BWHinfo.WheelVel_ms = temp[1]/Pw.WheelRm;
    RWHinfo.WheelVel_ms = temp[2]/Pw.WheelRm;
    //printf("Vwheel = %f, %f, %f\n",LWHinfo.WheelVel_ms, BWHinfo.WheelVel_ms, RWHinfo.WheelVel_ms);
}

void Omni_CalRef_Motor()
{
    LWHinfo.MoveDistance_m += LWHinfo.WheelVel_ms*R2D;
    RWHinfo.MoveDistance_m += RWHinfo.WheelVel_ms*R2D;
    BWHinfo.MoveDistance_m += BWHinfo.WheelVel_ms*R2D;
    joint->SetMoveJoint(LWH, LWHinfo.InitRef_Deg + LWHinfo.MoveDistance_m*OMNIinfo.TickSec, OMNIinfo.TickSec*1000, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWH, RWHinfo.InitRef_Deg + RWHinfo.MoveDistance_m*OMNIinfo.TickSec, OMNIinfo.TickSec*1000, MOVE_ABSOLUTE);
    joint->SetMoveJoint(BWH, BWHinfo.InitRef_Deg + BWHinfo.MoveDistance_m*OMNIinfo.TickSec, OMNIinfo.TickSec*1000, MOVE_ABSOLUTE);

   // printf("Pwheel = %f, %f, %f\n",LWHinfo.MoveDistance_m, BWHinfo.MoveDistance_m, RWHinfo.MoveDistance_m);
}

double Traj(double _cnt, double _vel)
{
    return _vel*0.5*(1.0-cos(PI*_cnt));
}

void SetOMNIpara(int _mode)
{
    switch(_mode)
    {
    case INIT:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara INIT";
        fix = XFix;
        OMNIinfo.Vms =      in.Vmax_ms;//0.6ms
        OMNIinfo.Vrads =    in.Vmax_rads;//70.0degs
        OMNIinfo.Trajm =    in.Distance_TrajM_m;//0.98m
        OMNIinfo.Trajrad =  in.Distance_TrajRad;//20.0deg
        OMNIinfo.TrajT =    in.Time_OmniTrajectory_sec;//1.5sec
        break;
    }
    case SHORT:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara SHORT";
        //maybe short than 20cm, 10deg
        fix = TimeFix;
        OMNIinfo.Vms = 0.1;
        OMNIinfo.Vrads = 20.0*D2R;
        OMNIinfo.Trajm = 0.2;
        OMNIinfo.Trajrad = 10.0*D2R;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case TURN_LITTLE:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara TURN_LITTLE";
//        fix = TimeFix;
        OMNIinfo.Vrads = 15.0*D2R;
        OMNIinfo.Trajrad = 10.0*D2R;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case MOVE_LITTLE:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara MOVE_LITTLE";
//        fix = TimeFix;
        OMNIinfo.Vms = 0.1;
        OMNIinfo.Trajm = 0.1;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case TURN_NORMAL:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara TURN_NORMAL";
//        fix = TimeFix;
        OMNIinfo.Vrads = 30.0*D2R;
        OMNIinfo.Trajrad = 15.0*D2R;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case MOVE_NORMAL:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara MOVE_NORMAL";
//        fix = TimeFix;
        OMNIinfo.Vms = 0.35;
        OMNIinfo.Trajm = 0.3;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case TURN_FAR:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara TURN_FAR";
//        fix = TimeFix;
        OMNIinfo.Vrads = 60.0*D2R;
        OMNIinfo.Trajrad = 20.0*D2R;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case MOVE_FAR:
    {
        FILE_LOG(logWARNING) << "Set OMNIpara MOVE_FAR";
        fix = XFix;
        OMNIinfo.Vms =      in.Vmax_ms;//0.6ms
        OMNIinfo.Vrads =    in.Vmax_rads;//70.0degs
        OMNIinfo.Trajm =    in.Distance_TrajM_m;//0.98m
        OMNIinfo.Trajrad =  in.Distance_TrajRad;//20.0deg
        OMNIinfo.TrajT =    in.Time_OmniTrajectory_sec;//1.5sec
        break;
    }
    case MIDDLE:
    {
        //maybe short than 1m, 30deg
        fix = TimeFix;
        OMNIinfo.Vms = 0.4;
        OMNIinfo.Vrads = 30.0*D2R;
        OMNIinfo.Trajm = 0.4;
        OMNIinfo.Trajrad = 20.0*D2R;
        OMNIinfo.TrajT = 1.0;
        break;
    }
    case FAR:
    {
        fix = XFix;
        OMNIinfo.Vms =      in.Vmax_ms;//0.6ms
        OMNIinfo.Vrads =    in.Vmax_rads;//70.0degs
        OMNIinfo.Trajm =    in.Distance_TrajM_m;//0.98m
        OMNIinfo.Trajrad =  in.Distance_TrajRad;//20.0deg
        OMNIinfo.TrajT =    in.Time_OmniTrajectory_sec;//1.5sec
        break;
    }
    }
}

void SetWheelMovePos(double _x, double _y, double _r)
{
    OMNIinfo.Pin.X = _x;
    OMNIinfo.Pin.Y = _y;
    OMNIinfo.Pin.Theta = _r;

    FLAG_OMNI = 1;
    Mode_OMNIMove = MOVE_OMNI;
}

void WheelMove(double _x, double _y, double _r, double _mode)
{
    /*
     * Wheel Move to _x, _y and rotation _r
     * _mode : Yaw compensation with SLAM feedback
     */
    InitWheelInfo();
    OnOff_YawCompen = _mode;
    joint->RefreshToCurrentReference();
    joint->SetAllMotionOwnerWHEEL();
    SetWheelMovePos(_x, _y, _r);
    SetGoalPos();

    printf("x = %f, y = %f, theta = %f, sec = %f\n********************************\n",OMNIinfo.Pin.X, OMNIinfo.Pin.Y, OMNIinfo.Pin.Theta, OMNIinfo.GoalSec);


}

double CalculateMoveTime(double t1, double t2, double t3)
{
    if(t1 > t2)
    {
        if(t1 > t3)
            return t1;
        else
            return t3;
    } else
    {
        if(t2 > t3)
            return t2;
        else
            return t3;
    }
}

double CalculateMoveTime(double t1, double t2)
{
    if(t1 > t2)
        return t1;
    else
        return t2;
}

void save()
{
    if(Save_Index < ROW)
    {
            Save_Data[0][Save_Index] = RWHinfo.InitRef_Deg;
            Save_Data[1][Save_Index] = LWHinfo.InitRef_Deg;
            Save_Data[2][Save_Index] = BWHinfo.InitRef_Deg;

            Save_Data[21][Save_Index] = RWHinfo.WheelVel_ms;
            Save_Data[22][Save_Index] = LWHinfo.WheelVel_ms;
            Save_Data[23][Save_Index] = BWHinfo.WheelVel_ms;

            Save_Data[27][Save_Index] = RWHinfo.MoveDistance_m;
            Save_Data[28][Save_Index] = LWHinfo.MoveDistance_m;
            Save_Data[29][Save_Index] = BWHinfo.MoveDistance_m;

            Save_Data[30][Save_Index] = OMNIinfo.Pin.X;
            Save_Data[31][Save_Index] = OMNIinfo.Pin.Y;
            Save_Data[32][Save_Index] = OMNIinfo.Pin.Theta;

            Save_Data[33][Save_Index] = OMNIinfo.Probot.X;
            Save_Data[34][Save_Index] = OMNIinfo.Probot.Y;
            Save_Data[35][Save_Index] = OMNIinfo.Probot.Theta;

            Save_Data[36][Save_Index] = OMNIinfo.Vrobot.X;
            Save_Data[37][Save_Index] = OMNIinfo.Vrobot.Y;
            Save_Data[38][Save_Index] = OMNIinfo.Vrobot.Theta;

            Save_Data[39][Save_Index] = sharedSEN->FT[2].Fx;
            Save_Data[40][Save_Index] = sharedSEN->FT[2].Fy;
            Save_Data[41][Save_Index] = sharedSEN->FT[2].Fz;

            Save_Data[42][Save_Index] = sharedSEN->FT[3].Fx;
            Save_Data[43][Save_Index] = sharedSEN->FT[3].Fy;
            Save_Data[44][Save_Index] = sharedSEN->FT[3].Fz;

            Save_Data[45][Save_Index] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHAND].id][MC_ID_CH_Pairs[RHAND].ch].CurrentPosition;
            Save_Data[46][Save_Index] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LHAND].id][MC_ID_CH_Pairs[LHAND].ch].CurrentPosition;

            Save_Data[52][Save_Index] = OMNIinfo.SatSecR;

            Save_Data[53][Save_Index] = OMNIinfo.CurSec;
            Save_Data[54][Save_Index] = OMNIinfo.GoalSec;
            Save_Data[55][Save_Index] = OMNIinfo.SatSec;

            Save_Data[56][Save_Index] = userData->R2M.robot_pos.pos_x;
            Save_Data[57][Save_Index] = userData->R2M.robot_pos.pos_y;
            Save_Data[58][Save_Index] = userData->R2M.robot_pos.ori_w;


            Save_Data[59][Save_Index] = sharedSEN->ENCODER[MC_ID_CH_Pairs[RWH].id][MC_ID_CH_Pairs[RWH].ch].CurrentPosition;
            Save_Data[60][Save_Index] = sharedSEN->ENCODER[MC_ID_CH_Pairs[LWH].id][MC_ID_CH_Pairs[LWH].ch].CurrentPosition;
//            Save_Data[47][Save_Index] = WBmotion->pRH_3x1[0];
//            Save_Data[48][Save_Index] = WBmotion->pRH_3x1[1];
//            Save_Data[49][Save_Index] = WBmotion->pRH_3x1[2];

//            Save_Data[50][Save_Index] = WBmotion->pLH_3x1[0];
//            Save_Data[51][Save_Index] = WBmotion->pLH_3x1[1];
//            Save_Data[52][Save_Index] = WBmotion->pLH_3x1[2];

//            Save_Data[53][Save_Index] = WBmotion->qRH_4x1[0];
//            Save_Data[54][Save_Index] = WBmotion->qRH_4x1[1];
//            Save_Data[55][Save_Index] = WBmotion->qRH_4x1[2];
//            Save_Data[56][Save_Index] = WBmotion->qRH_4x1[3];

//            Save_Data[57][Save_Index] = WBmotion->qLH_4x1[0];
//            Save_Data[58][Save_Index] = WBmotion->qLH_4x1[1];
//            Save_Data[59][Save_Index] = WBmotion->qLH_4x1[2];
//            Save_Data[60][Save_Index] = WBmotion->qLH_4x1[3];
            Save_Data[61][Save_Index] = Command_Grasp;

            Save_Index++;
            if(Save_Index >= ROW) Save_Index = 0;
    }
}


/*************************************************************************************************************************/
void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    WBmotion->ResetGlobalCoord(_mode);

    WBmotion->StopAll();

    WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}

int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}

int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}

void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}

int RBInitialize(void)
{
    // Block program termination
    isTerminated = 0;

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME_REFERENCE, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Reference]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_REFERENCE)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Reference]";
            return false;
        }else{
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedREF == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Reference]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Reference]";
    // =========================================================================

    // Core Shared Memory Creation [Sensor]=====================================
    shmFD = shm_open(RBCORE_SHM_NAME_SENSOR, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Sensor]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_SENSOR)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Sensor]";
            return false;
        }else{
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ, MAP_SHARED, shmFD, 0);
            if(sharedSEN == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Sensor]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Sensor]";
    // =========================================================================

    // Core Shared Memory Creation [Command]====================================
    shmFD = shm_open(RBCORE_SHM_NAME_COMMAND, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Command]";
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_COMMAND)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Command]";
            return false;
        }else{
            sharedCMD = (pRBCORE_SHM_COMMAND)mmap(0, sizeof(RBCORE_SHM_COMMAND), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedCMD == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Command]";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Command]";
    // =========================================================================


    // User Shared Memory Creation ============================================
    shmFD = shm_open(USER_SHM_NAME, O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open user shared memory";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return -1;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    joint = new JointControlClass(sharedREF, sharedSEN, sharedCMD, PODO_NO);
    joint->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, "RobotWorld_FLAG", 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(3, &aCPU);
        if(rt_task_set_affinity(&rtFlagCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Flag real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtFlagCon, &RBFlagThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Flag real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Flag real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Flag real-time thread";
        return -1;
    }

    if(rt_task_create(&rtTaskCon, "RobotWorld_TASK", 0, 90, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(2, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
            FILE_LOG(logWARNING) << "Task real-time thread set affinity CPU failed..";
        }
        if(rt_task_start(&rtTaskCon, &RBTaskThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Task real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Task real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Task real-time thread";
        return -1;
    }
    // =========================================================================

    return 0;
}

void InitializeSharedMemory()
{
    printf("Init SharedMemory\n");
    userData->R2M.ROS_COMMAND = ROBOTWORLD_ROS_NO_ACT;
    userData->R2M.SELECTED_MENU = NO_SIGNAL;
    userData->M2R.MOTION_ERROR = NO_ERROR;
    userData->M2R.MOTION_STATE = AL_NO_ACT;
    ROBOTinfo.GoalPos.X = userData->R2M.robot_pos.pos_x;
    ROBOTinfo.GoalPos.Y = userData->R2M.robot_pos.pos_y;
    ROBOTinfo.GoalPos.Theta = userData->R2M.robot_pos.ori_w*D2R;
}
