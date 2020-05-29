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
//Motion check
#include <time.h>

#include "joint.h"
#include "taskmotion.h"
#include "ManualMove.h"
#include "MotionChecker.h"
#include "ManualCAN.h"


#define PODO_AL_NAME       "ManualMove_AL"

#define RIGHT               true
#define LEFT                false

using namespace std;

// Functions ===========================================
// Signal handler
void CatchSignals(int _signal);
int HasAnyOwnership();
int AskALNumber(char *alname);

// Real-time thread for control
void RBTaskThread(void *);
void RBFlagThread(void *);
void JoystickThread(void *);

// Initialization
int RBInitialize(void);
// =====================================================

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM       userData;

// RT task handler for control
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;
RT_TASK rtJoystickCon;

// Program variable
int isTerminated;
// PODO No.
int PODO_NO;
// --------------------------------------------------------------------------------------------- //


JointControlClass *joint;
// CKINE_DRC_HUBO kine_drc_hubo;
int CheckMotionOwned();
// WBIK functins-------------------------------------------------------------------------------- //
// Variables
int             WB_FLAG = false;
int             WB_FLAG_LB = false;
int             WB_FLAG_WHOLEBODY = false;
long            LimitedJoint;
long            LimitType;



TaskMotion      *WBmotion;

// MotionChecker *MC;


// *** Your PODO-AL command set *** //
int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

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

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }


//    // Get PODO No. ---------------------------------------
//    if(argc == 1){
//        cout << ">>> No input argument..!!\n";
//        //PODO_NO = 13;
//        return 0;
//    }
//    else{
//        QString argStr;
//        argStr.sprintf("%s", argv[1]);
//        PODO_NO = argStr.toInt();
//        cout << "======================================================================" << endl;
//        cout << ">>> Process ManualMove is activated..!!" << endl;
//        cout << ">>> PODO NAME: ManualMove_AL" << endl;
//        cout << ">>> PODO NO: " << PODO_NO << endl;
//    }

    // Initialize RBCore -----------------------------------
    if( RBInitialize() == -1 ) isTerminated = -1;
    cout << "======================================================================" << endl;

    // Initialize internal joint classes -------------------
    joint = new JointControlClass(sharedREF, sharedSEN, sharedCMD, PODO_NO);
    joint->RefreshToCurrentReference();
    // WBIK Initialize--------------------------------------
    WBmotion = new TaskMotion(sharedREF, sharedSEN, sharedCMD, joint);

    // Joystick
    joystick = new RBJoystick();
    joystick->ConnectJoy();


    // User command cheking --------------------------------
    while(isTerminated == 0){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND)
        {
        case ManualMove_AL_UPPER_TASK_POSE:
            printf("Changing Pos...!!\n");
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            ShutDownManualMode();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){
                // Task pos 0
                Change_Pos_Task();
                usleep(4500*1000);
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){
                // Task pos 180
                Change_Pos_Task_BACK();
                usleep(4500*1000);
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2){
                // Move pos 0
                Change_Pos_Move();
                usleep(4500*1000);
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 3){
                // Move pos 180
                Change_Pos_Move_BACK();
                usleep(4500*1000);
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 4){
                // HP movement
                double des_HP = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                joint->SetMoveJoint(RHP, des_HP, 3000, MOVE_RELATIVE);
                joint->SetMoveJoint(LHP, des_HP, 3000, MOVE_RELATIVE);
                usleep(3100*1000);
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;
        case ManualMove_AL_HAND:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)// right hand
            {
                double des_cur = 125.*((double)(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]));
                joint->SetMoveJoint(RHAND, des_cur, 10, MODE_ABSOLUTE);
            }
            else//left hand
            {
                double des_cur = 125.*((double)(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]));
                joint->SetMoveJoint(LHAND, des_cur, 10, MODE_ABSOLUTE);
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;
        case ManualMove_AL_GAIN:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==0)//right arm
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]==0)//low gain
                {
                    cout<<"Right arm gain LOW"<<endl;
                    //RBBoardSetSwitchingMode(2,15,SW_MODE_NON_COMPLEMENTARY);  //RSY REB
                    //RBBoardSetSwitchingMode(2,16,SW_MODE_NON_COMPLEMENTARY);  //RWY RWP
                    MCBoardSetSwitchingMode(2, 15, SW_MODE_NON_COMPLEMENTARY);  //RSY REB
                    MCBoardSetSwitchingMode(2, 16, SW_MODE_NON_COMPLEMENTARY);  //RWY RWP
//                    RBJointGainOverride(2,15,temp_gain,temp_gain,1000);//RSY REB
//                    RBJointGainOverride(2,16,temp_gain,temp_gain,1000);//RWY RWP
                    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch ,65,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch ,65,1000);  //REB
                    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch ,65,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch ,65,1000);  //RWP

                }
                else//high gain
                {
                    cout<<"Right arm gain HIGH"<<endl;
//                    RBBoardSetSwitchingMode(2,15,SW_MODE_COMPLEMENTARY);
//                    RBBoardSetSwitchingMode(2,16,SW_MODE_COMPLEMENTARY);
                    MCBoardSetSwitchingMode(2, 15, SW_MODE_COMPLEMENTARY);  //RSY REB
                    MCBoardSetSwitchingMode(2, 16, SW_MODE_COMPLEMENTARY);  //RWY RWP


//                    RBJointGainOverride(2,15,1000,1000,2000);//RSY REB
//                    RBJointGainOverride(2,16,1000,1000,2000);//RWY RWP
                    MCJointGainOverride(JOINT_INFO[RSY].canch, JOINT_INFO[RSY].bno, JOINT_INFO[RSY].mch ,0,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[REB].canch, JOINT_INFO[REB].bno, JOINT_INFO[REB].mch ,0,1000);  //REB
                    MCJointGainOverride(JOINT_INFO[RWY].canch, JOINT_INFO[RWY].bno, JOINT_INFO[RWY].mch ,0,1000);  //RSY
                    MCJointGainOverride(JOINT_INFO[RWP].canch, JOINT_INFO[RWP].bno, JOINT_INFO[RWP].mch ,0,1000);  //RWP
                }
            }
            else//left arm
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]==0)//low gain
                {
                    cout<<"Left arm gain LOW"<<endl;
//                    RBBoardSetSwitchingMode(3,19,1);
//                    RBBoardSetSwitchingMode(3,20,1);
                    MCBoardSetSwitchingMode(3,19, SW_MODE_NON_COMPLEMENTARY);  //LSY LEB
                    MCBoardSetSwitchingMode(3,20, SW_MODE_NON_COMPLEMENTARY);  //LWY LWP


//                    unsigned int temp_gain=1;
//                    RBJointGainOverride(3,19,temp_gain,temp_gain,1000);//LSY LEB
//                    RBJointGainOverride(3,20,temp_gain,temp_gain,1000);//LWY LWP
                    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch ,65,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch ,65,1000);  //LEB
                    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch ,65,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch ,65,1000);  //LWP

                }
                else//high gain
                {
                    cout<<"Left arm gain HIGH"<<endl;
//                    RBBoardSetSwitchingMode(3,19,0);
//                    RBBoardSetSwitchingMode(3,20,0);
                    MCBoardSetSwitchingMode(3,19, SW_MODE_COMPLEMENTARY);  //LSY LEB
                    MCBoardSetSwitchingMode(3,20, SW_MODE_COMPLEMENTARY);  //LWY LWP

//                    RBJointGainOverride(3,19,1000,1000,1000);//LSY LEB
//                    RBJointGainOverride(3,20,1000,1000,1000);//LWY LWP
                    MCJointGainOverride(JOINT_INFO[LSY].canch, JOINT_INFO[LSY].bno, JOINT_INFO[LSY].mch ,0,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LEB].canch, JOINT_INFO[LEB].bno, JOINT_INFO[LEB].mch ,0,1000);  //LEB
                    MCJointGainOverride(JOINT_INFO[LWY].canch, JOINT_INFO[LWY].bno, JOINT_INFO[LWY].mch ,0,1000);  //LSY
                    MCJointGainOverride(JOINT_INFO[LWP].canch, JOINT_INFO[LWP].bno, JOINT_INFO[LWP].mch ,0,1000);  //LWP
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_MODE_START: //one hand upper body mode
            printf("Manual Hand Start...!!\n");

            ShutDownManualMode();

            StartWBIKmotion(0); // 0 -> Local UB // Start Whole Body iteration
            usleep(20*1000);

            WB_FLAG_LB = false;


            _isFirst = true;
            ManualModeFlag = true;

            //sharedData->STATE_COMMAND = TCMD_CAR_MANUAL_ONE_HAND_START;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_MODE_STOP:
            printf("Manual Stop...!!\n");

            ShutDownManualMode();

            //sharedData->STATE_COMMAND = TCMD_CAR_MANUAL_MODE_STOP;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_BOTH_HAND_MODE_START:
            //cout<<"Manual Mode Start !!"<<endl;
            printf("Manual Both Hand Start...!!!!!\n");

            ShutDownManualMode();

            usleep(1000);

            StartWBIKmotion(0); // 0 -> Local UB // Start Whole Body iteration

            usleep(20*1000);

            _isFirst_both_R = true;
            _isFirst_both_L = true;
            ManualModeFlag_Hand_both = true;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_ONE_HAND_STADING_START:
            printf("Manual One Hand Standing mode Start...!!\n");

            ShutDownManualMode();

            usleep(10*1000);
            joint->RefreshToCurrentReference();
            usleep(10*1000);
            WBmotion->ResetGlobalCoord(0);//Coord -> 0:PC
            WBmotion->StopAll();// Trajectory making ...
            WBmotion->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WB_FLAG_WHOLEBODY = true;

            _isFirst = true;
            ManualModeFlag = true;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_BOTH_HAND_STADING_START:
            printf("Manual Two Hand Standing mode Start...!!\n");

            ShutDownManualMode();

            usleep(10*1000);
            joint->RefreshToCurrentReference();
            usleep(10*1000);
            WBmotion->ResetGlobalCoord(0);//Coord -> 0:PC
            WBmotion->StopAll();// Trajectory making ...
            WBmotion->RefreshToCurrentReference();
            joint->SetAllMotionOwner();

            WB_FLAG_WHOLEBODY = true;

            _isFirst_both_R = true;
            _isFirst_both_L = true;
            ManualModeFlag_Hand_both = true;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_FOOT_MODE_START:
            printf("Manual FOOT Start...!!\n");

            ShutDownManualMode();

            //-----------------------------------------------
            joint->RefreshToCurrentReference();

            usleep(10*1000);

            WBmotion->ResetGlobalCoord(0);//Coord

            WBmotion->StopAll();// Trajectory making ...

            WBmotion->RefreshToCurrentReferenceLB();

            joint->SetAllMotionOwner();
//            for(int i=RHY; i<=LAR; i++){
//                joint->SetMotionOwner(i);
//            }


            //------------------------------------------------            
            WB_FLAG_LB = true;
            usleep(20*1000);
            _isFirst = true;
            ManualModeFlag_Foot = true;;



            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START:
            printf("Manual Both FOOT Start...!!\n");
            ShutDownManualMode();

            usleep(10*1000);
            //-----------------------------------------------
            joint->RefreshToCurrentReference();
            usleep(10*1000);
            WBmotion->ResetGlobalCoord(0);//Coord
            WBmotion->StopAll();// Trajectory making ...
            WBmotion->RefreshToCurrentReferenceLB();
            joint->SetAllMotionOwner();
//            for(int i=RHY; i<=LAR; i++){
//                joint->SetMotionOwner(i);
//            }

            WB_FLAG_LB = true;
            usleep(20*1000);

            _isFirst_both_R = true;
            _isFirst_both_L = true;
            ManualModeFlag_Foot_both = true;

            //------------------------------------------------


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_E_STOP:
            cout<<"Emergency STOP!"<<endl;
            WB_FLAG = false;
            WB_FLAG_LB = false;
            joint->RefreshToCurrentReference();
            joint->SetAllMotionOwner();
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;
        case ManualMove_AL_DRIVE_MODE:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 0){ //start
                cout<<"Manual Car Driving Mode!!"<<endl;
                WB_FLAG = false;
                WB_FLAG_LB = false;
                Driving_Flag = false;
                joint->RefreshToCurrentReference();
                //joint->SetAllMotionOwner();
                //joint->SetMotionOwner(RAP);
                joint->SetMotionOwner(RWY2);

                //--wrist and ankle , ready to pwm control
                //---RAP
//                RBsetFrictionParameter(0,4, 60, 70, 10,0);
//                RBBoardSetSwitchingMode(0,4, SW_MODE_NON_COMPLEMENTARY);
//                RBenableFrictionCompensation(0,4,ENABLE,DISABLE);
//                RBJointGainOverride(0,4,0,1000,1);
//                MCsetFrictionParameter(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno,JOINT_INFO[RAP].mch, 60,70,0);
//                MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno,SW_MODE_NON_COMPLEMENTARY);
//                MCenableFrictionCompensation(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno,JOINT_INFO[RAP].mch,ENABLE);
//                MCJointGainOverride(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno,JOINT_INFO[RAP].mch,100,10);

                //RF1
//                RBJointGainOverride(2, 36,0,1000,1);
                MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 100, 10);
                usleep(5000);

                Enc_request(1);
                usleep(1000*1000);
                joint->RefreshToCurrentEncoder_RAP_RF1();

                Driving_Flag = true;
            }
            else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] == 1){ //finish
                cout<<"Manual Driving mode stop!!"<<endl;
                WB_FLAG = false;
                WB_FLAG_LB = false;
                Driving_Flag = false;
                Enc_request(1);
                usleep(1000*1000);
                if(joint->RefreshToCurrentEncoder_RAP_RF1() == 0){
                    cout<<"Encoder is off!!"<<endl;
                    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
                    break;

                }
                else{
                    joint->SetMotionOwner(RWY2);
//                    RBenableFrictionCompensation(2,36,DISABLE,DISABLE);
                    //joint->SetMotionOwner(RAP);
//                    RBenableFrictionCompensation(0,4,DISABLE,DISABLE);
//                    RBBoardSetSwitchingMode(0,4, SW_MODE_COMPLEMENTARY);
                    //MCenableFrictionCompensation(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno,JOINT_INFO[RAP].mch,DISABLE);
                    //MCBoardSetSwitchingMode(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno,SW_MODE_COMPLEMENTARY);

//                    RBJointGainOverride(2,36,1000,1000,1000); //---RWY2
//                    RBJointGainOverride(0,4,1000,50,1000);
                    //MCJointGainOverride(JOINT_INFO[RAP].canch,JOINT_INFO[RAP].bno,JOINT_INFO[RAP].mch,0,1000);
                    MCJointGainOverride(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno, JOINT_INFO[RWY2].mch, 0, 10);
                    usleep(1010*1000);
                }
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;

        case ManualMove_AL_JOYSTICK_MODE:
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0){
                joystick->ConnectJoy();
                Joystick_flag = true;
            }
            else{
                Joystick_flag = false;
            }



        default:
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_NO_ACT;
            break;
        }

    }




    cout << ">>> Process ManualMove is terminated..!!" << endl;
    return 0;
}
// --------------------------------------------------------------------------------------------- //



// --------------------------------------------------------------------------------------------- //
void RBTaskThread(void *)
{

    while(isTerminated == 0){

        if(ManualModeFlag == true) ManualMoveHand();//ManualMoveHand_Global_Ori();

        else if(ManualModeFlag_Foot == true) ManualMoveFoot();

        else if(ManualModeFlag_Foot_both == true){
            ManualMoveFoot_Right();
            ManualMoveFoot_Left();
        }

        else if(ManualModeFlag_Hand_both == true){
            ManualMoveHand_Right();
            ManualMoveHand_Left();
        }
        else if(Driving_Flag == true) ManualDriving();



        if(WB_FLAG == true)
        {
            WBmotion->updateAll();
            WBmotion->WBIK_UB();

            for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

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

        if(WB_FLAG_LB == true)
        {
            WBmotion->updateAll();
            WBmotion->WBIK_LB();



            for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

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
                WB_FLAG_LB = false;
        }

        if(WB_FLAG_WHOLEBODY == true)
        {
            WBmotion->updateAll();
            WBmotion->WBIK();



            for(int i=RHY; i<=LAR; i++) joint->SetJointRefAngle(i, WBmotion->Q_filt_34x1[idRHY+i-RHY]*R2D);

            joint->SetJointRefAngle(WST, WBmotion->Q_filt_34x1[idWST]*R2D);

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
                WB_FLAG_WHOLEBODY = false;
        }

//        Iteration++;
//        if(Iteration % 100 == 0){
//            cout<<"Joy Valeu : "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]<<endl;
//        }
        joint->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}
// --------------------------------------------------------------------------------------------- //
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 300*1000);        // 300 usec

    while(isTerminated == 0)
    {
        rt_task_wait_period(NULL);

        //if(HasAnyOwnership()){
            {
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                joint->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}
void JoystickThread(void *){
    while(1){
        static unsigned int Joy_counter = 0;
        if(Joystick_flag == true){
            if(joystick->isConnected() == false){
                printf("JoyStick connection failure...!!!\n");
                Joystick_flag = false;
            }

            // Button Data
            GL_JOY_X = joystick->JoyButton[2];
            GL_JOY_A = joystick->JoyButton[0];
            GL_JOY_B = joystick->JoyButton[1];
            GL_JOY_Y = joystick->JoyButton[3];
            GL_JOY_LB = joystick->JoyButton[4];
            GL_JOY_RB = joystick->JoyButton[5];
            GL_JOY_LT = joystick->JoyAxis[2];
            GL_JOY_RT = joystick->JoyAxis[5];

            if((int)(GL_JOY_LT) == -1){
                GL_JOY_LT = 1;
            }
            else{
                GL_JOY_LT = 0;
            }
            if((int)(GL_JOY_RT) == -1){
                GL_JOY_RT = 1;
            }
            else{
                GL_JOY_RT = 0;
            }

            GL_JOY_BACK = joystick->JoyButton[6];
            GL_JOY_START = joystick->JoyButton[7];
            GL_JOY_LJOG_PUSH = joystick->JoyButton[9];
            GL_JOY_RJOG_PUSH = joystick->JoyButton[10];

            // AXIS Data
            GL_JOY_LJOG_RL = joystick->JoyAxis[0];
            if(GL_JOY_LJOG_RL > 30000) GL_JOY_LJOG_RL = 32767;
            else if(GL_JOY_LJOG_RL < -30000) GL_JOY_LJOG_RL = -32767;
            else GL_JOY_LJOG_RL = 0;

            GL_JOY_LJOG_UD = -joystick->JoyAxis[1];
            if(GL_JOY_LJOG_UD > 30000) GL_JOY_LJOG_UD = 32767;
            else if(GL_JOY_LJOG_UD < -30000) GL_JOY_LJOG_UD = -32767;
            else GL_JOY_LJOG_UD = 0;


            GL_JOY_RJOG_RL = joystick->JoyAxis[3];
            if(GL_JOY_RJOG_RL > 30000) GL_JOY_RJOG_RL = 32767;
            else if(GL_JOY_RJOG_RL < -30000) GL_JOY_RJOG_RL = -32767;
            else GL_JOY_RJOG_RL = 0;

            GL_JOY_RJOG_UD = -joystick->JoyAxis[4];
            if(GL_JOY_RJOG_UD > 30000) GL_JOY_RJOG_UD = 32767;
            else if(GL_JOY_RJOG_UD < -30000) GL_JOY_RJOG_UD = -32767;
            else GL_JOY_RJOG_UD = 0;

            GL_JOY_AROW_RL = joystick->JoyAxis[6];
            GL_JOY_AROW_UD = -joystick->JoyAxis[7];

            //for XBox controller
//            // Button Data
//            GL_JOY_X = joystick->JoyButton[2];
//            GL_JOY_A = joystick->JoyButton[0];
//            GL_JOY_B = joystick->JoyButton[1];
//            GL_JOY_Y = joystick->JoyButton[3];
//            GL_JOY_LB = joystick->JoyButton[4];
//            GL_JOY_RB = joystick->JoyButton[5];
//            GL_JOY_LT = joystick->JoyAxis[5];
//            GL_JOY_RT = joystick->JoyAxis[4];

//            if((int)(GL_JOY_LT) == -1){
//                GL_JOY_LT = 1;
//            }
//            else{
//                GL_JOY_LT = 0;
//            }
//            if((int)(GL_JOY_RT) == -1){
//                GL_JOY_RT = 1;
//            }
//            else{
//                GL_JOY_RT = 0;
//            }

//            GL_JOY_BACK = joystick->JoyButton[6];
//            GL_JOY_START = joystick->JoyButton[7];
//            GL_JOY_LJOG_PUSH = joystick->JoyButton[9];
//            GL_JOY_RJOG_PUSH = joystick->JoyButton[10];

//            // AXIS Data
//            GL_JOY_LJOG_RL = joystick->JoyAxis[0];
//            if(GL_JOY_LJOG_RL > 30000) GL_JOY_LJOG_RL = 32767;
//            else if(GL_JOY_LJOG_RL < -30000) GL_JOY_LJOG_RL = -32767;
//            else GL_JOY_LJOG_RL = 0;

//            GL_JOY_LJOG_UD = -joystick->JoyAxis[1];
//            if(GL_JOY_LJOG_UD > 30000) GL_JOY_LJOG_UD = 32767;
//            else if(GL_JOY_LJOG_UD < -30000) GL_JOY_LJOG_UD = -32767;
//            else GL_JOY_LJOG_UD = 0;


//            GL_JOY_RJOG_RL = joystick->JoyAxis[2];
//            if(GL_JOY_RJOG_RL > 30000) GL_JOY_RJOG_RL = 32767;
//            else if(GL_JOY_RJOG_RL < -30000) GL_JOY_RJOG_RL = -32767;
//            else GL_JOY_RJOG_RL = 0;

//            GL_JOY_RJOG_UD = -joystick->JoyAxis[3];
//            if(GL_JOY_RJOG_UD > 30000) GL_JOY_RJOG_UD = 32767;
//            else if(GL_JOY_RJOG_UD < -30000) GL_JOY_RJOG_UD = -32767;
//            else GL_JOY_RJOG_UD = 0;

//            GL_JOY_AROW_RL = joystick->JoyAxis[6];
//            GL_JOY_AROW_UD = -joystick->JoyAxis[7];
            //for Xbox Controller



            if(Joy_counter == 30){
                if(GL_JOY_X==0 && GL_JOY_A==0 && GL_JOY_B==0 && GL_JOY_Y==0 && GL_JOY_LB==0 &&
                        GL_JOY_RB==0 && GL_JOY_LT==0 && GL_JOY_RT==0 && GL_JOY_BACK==0 &&
                        GL_JOY_START==0 && GL_JOY_LJOG_PUSH==0 && GL_JOY_RJOG_PUSH==0 &&
                        GL_JOY_LJOG_RL==0 && GL_JOY_LJOG_UD==0 && GL_JOY_RJOG_RL==0 &&
                        GL_JOY_RJOG_UD==0 && GL_JOY_AROW_RL==0 && GL_JOY_AROW_UD==0){
                    //sharedData->STATE_COMMAND = TCMD_CAR_MANUAL_MODE_READY;
                }
                else{
                    //sharedData->STATE_COMMAND = TCMD_CAR_MANUAL_MODE_NOT_READY;
                    cout<<"Manual mode not Ready!"<<endl;
                }
                Joy_counter = 0;
            }
        }
        else{
            GL_JOY_LT=0;
            GL_JOY_LB=0;
            GL_JOY_LJOG_RL=0;
            GL_JOY_LJOG_UD=0;
            GL_JOY_AROW_RL=0;
            GL_JOY_AROW_UD=0;
            GL_JOY_LJOG_PUSH=0;

            GL_JOY_RT=0;
            GL_JOY_RB=0;
            GL_JOY_RJOG_RL=0;
            GL_JOY_RJOG_UD=0;
            GL_JOY_A=0;
            GL_JOY_B=0;
            GL_JOY_X=0;
            GL_JOY_Y=0;
            GL_JOY_RJOG_PUSH=0;

            GL_JOY_BACK=0;
            GL_JOY_START=0;

            if(Joy_counter == 30){
                //sharedData->STATE_COMMAND = TCMD_CAR_JOYSTICK_OFF;
                Joy_counter = 0;
            }
        }
        Joy_counter++;
        usleep(30*1000);
    }
}

int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch] == PODO_NO)
            return true;
    }
    return false;
}
// --------------------------------------------------------------------------------------------- //
void StartWBIKmotion(int _mode)
{
    WB_FLAG = false;
    usleep(10*1000);

    joint->RefreshToCurrentReference();

    usleep(10*1000);

    WBmotion->ResetGlobalCoord(_mode);//Coord

    WBmotion->StopAll();// Trajectory making ...

    if(_mode == 0)
        WBmotion->RefreshToCurrentReferenceUB();
    else
        WBmotion->RefreshToCurrentReference();

    joint->SetAllMotionOwner();

    WB_FLAG = true;
}

void ShutDownManualMode(void){
    // shut down other mode-----------
    WB_FLAG_LB = false;
    WB_FLAG = false;
    WB_FLAG_WHOLEBODY = false;
    ManualModeFlag = false;
    ManualModeFlag_Hand_both = false;
    ManualModeFlag_Foot = false;
    ManualModeFlag_Foot_both = false;
    //---------------------------------
    usleep(1000);

    for(int i=0;i<10;i++){
        sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[i] = 0;
    }
    for(int i=0;i<4;i++){
        sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[i] = 0;
    }
}

void PrintWBIKinfo(void)
{
    printf("--------------------POS INFO------------------------------\n");
    printf("RH pos: %f %f %f\n",WBmotion->pRH_3x1[0], WBmotion->pRH_3x1[1], WBmotion->pRH_3x1[2]);
    printf("LH pos: %f %f %f\n",WBmotion->pLH_3x1[0], WBmotion->pLH_3x1[1], WBmotion->pLH_3x1[2]);
    printf("--------------------ROT INFO--------------------\n");
    printf("RH rot: %f %f %f %f\n",WBmotion->qRH_4x1[0], WBmotion->qRH_4x1[1], WBmotion->qRH_4x1[2], WBmotion->qRH_4x1[3]);
    printf("LH rot: %f %f %f %f\n",WBmotion->qLH_4x1[0], WBmotion->qLH_4x1[1], WBmotion->qLH_4x1[2], WBmotion->qLH_4x1[3]);
    printf("--------------------POS  FOOT-------------------\n");
    printf("Foot R: %f %f %f\n",WBmotion->pRF_3x1[0], WBmotion->pRF_3x1[1], WBmotion->pRF_3x1[2]);
    printf("Foot L: %f %f %f\n",WBmotion->pLF_3x1[0], WBmotion->pLF_3x1[1], WBmotion->pLF_3x1[2]);
    printf("--------------------RELB INFO-------------------\n");
    printf("RI ELB: %f\n",WBmotion->RElb_ang*R2D);
    printf("LE ELB: %f\n",WBmotion->LElb_ang*R2D);
    printf("--------------------COMP INFO-------------------\n");
    printf("COM X : %f\n",WBmotion->pCOM_2x1[0]);
    printf("COM Y : %f\n",WBmotion->pCOM_2x1[1]);
    printf("--------------------PEL XY POS------------------\n");
    printf("PEL Z : %f\n",WBmotion->pPelZ);
}

// --------------------------------------------------------------------------------------------- //
void Change_Pos_Task(void)
{
    double postime=4000.;
    joint->SetMoveJoint(RSP, -30.50, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -4.26, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 16.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -113.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 4.88, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 72.14, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -30.50, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 4.26, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -16.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -113.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, -4.88, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 72.14, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -26.24, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 26.24, postime, MOVE_ABSOLUTE);
}
void Change_Pos_Task_BACK(void)
{
    double postime=4000.;
    joint->SetMoveJoint(RSP, -30.50, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, -15, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 16.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -113.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 4.88, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 72.14, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, -30.50, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, 15, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, -16.54, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -113.39, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, -4.88, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 72.14, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 180.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, -26.24, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 26.24, postime, MOVE_ABSOLUTE);
}
void Change_Pos_Move(void)
{
    double postime=4000.;
    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 0.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
}
void Change_Pos_Move_BACK(void)
{
    double postime=4000.;
    joint->SetMoveJoint(RSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSR, 15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(REB, -130, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(RWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(LSP, 40.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSR, -15.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LSY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LEB, -130.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWP, 40.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(WST, 180.0, postime, MOVE_ABSOLUTE);

    joint->SetMoveJoint(RWY2, 0.0, postime, MOVE_ABSOLUTE);
    joint->SetMoveJoint(LWY2, 0.0, postime, MOVE_ABSOLUTE);
}

// --------------------------------------------------------------------------------------------- //
int CheckMotionOwned()
{
    for(int i=0;i<NO_OF_JOINTS-2;i++)
    {
        if(sharedCMD->MotionOwner[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch]!=PODO_NO)	return 0;
    }
    return 1;
}
// --------------------------------------------------------------------------------------------- //
void CatchSignals(int _signal)
{
    switch(_signal){
    case SIGHUP:
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
        isTerminated = -1;
        break;
    }
    usleep(1000*500);
}
// --------------------------------------------------------------------------------------------- //

// --------------------------------------------------------------------------------------------- //
int RBInitialize(void)
{
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
    if(rt_task_create(&rtFlagCon, "MANUAL_FLAG", 0, 95, 0) == 0){
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

    if(rt_task_create(&rtTaskCon, "MANUAL_TASK", 0, 90, 0) == 0){
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
    //----JoyStick Thread------------------
    if(rt_task_create(&rtJoystickCon, "JOYSTICK_THREAD", 0, 90, 0) == 0){
        if(rt_task_start(&rtJoystickCon, &JoystickThread, NULL) == 0 ){
            FILE_LOG(logSUCCESS) << "Joystick real-time thread start = OK";
        }else{
            FILE_LOG(logERROR) << "Joystick real-time thread start = FAIL";
            return -1;
        }
    }else{
        FILE_LOG(logERROR) << "Fail to create Task real-time thread";
        return -1;
    }
    // =========================================================================

    return 0;
}

void Enc_request(int on)
{
    while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND != NO_ACT);

    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;
//    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = on;

    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
    cout<<"Enc_request On"<<endl;
}
// --------------------------------------------------------------------------------------------- //

void ManualMoveHand(void){
    // Variables
    static int counter = 0;
    double pVel[3] = {0,0,0};
//    static double RH_SpeedUP_V0[6] = {0,0,0};
    static double SpeedDW_V0[8] = {0,0,0,0,0,0,0,0};
    static int SpeedUp_time[8] = {0,0,0,0,0,0,0,0};
    static int SlowDown_time[8] = {0,0,0,0,0,0,0,0};
    static int plus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    static int minus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    double Acc = 0.5;
    double Dcc = 0.5;
    double MaxVel = 0.07;
    static bool MoveFlag[8] = {false,false,false,false,false,false,false,false};
 //   double ShoulderToHand;
    double ShoulderToWrist;
    static bool Manual_OK = true;
    static double manual_pos[3];
    static double manual_pos_before[3];
    double Accq = 0.05;
    double Dccq = 0.02;
    double MaxVel_ang = 0.006;
    double AngSpeed[8];
    static double RELB;
    static double RELB_before;
    static double WST_ang;
    static double WST_ang_before;
    int Grasping_value = 0;
    static int switching_counter = 0;
    static bool RL_mode = RIGHT;
    static bool RIGHT_isFirst = true;
    static bool LEFT_isFirst = false;
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_AROW_UD_before;
    static int Joy_B_before;
    static int Joy_LB_before;
    static int Joy_LJOG_RL_before;
    static int Joy_LJOG_UD_before;
    static int Joy_LT_before;
    static int Joy_RB_before;
    static int Joy_RJOG_RL_before;
    static int Joy_RJOG_UD_before;
    static int Joy_RT_before;
    static int Joy_X_before;
    static int Joy_Y_before;
//    static bool Release_Flag = false;
    static unsigned int same_counter = 0;
    static quat manual_quat;
    static quat manual_quat_before;
    quat qVel;

    // Joystick data
//    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
//    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
//    int Joy_X = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
//    int Joy_B = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];
//    int Joy_RT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
//    int Joy_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
//    int Joy_LT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6];
//    int Joy_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7];
//    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
//    int Joy_AROW_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
//    int Joy_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
//    int Joy_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
//    int Joy_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
//    int Joy_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
//    int Joy_BACK = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10];
//    int Joy_START = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[11];
//    int Joy_RJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[12];
//    int Joy_LJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
    int Joy_Y = GL_JOY_Y;
    int Joy_A = GL_JOY_A;
    int Joy_X = GL_JOY_X;
    int Joy_B = GL_JOY_B;
    int Joy_RT = GL_JOY_RT;
    int Joy_RB = GL_JOY_RB;
    int Joy_LT = GL_JOY_LT;
    int Joy_LB = GL_JOY_LB;
    int Joy_AROW_RL = GL_JOY_AROW_RL;
    int Joy_AROW_UD = GL_JOY_AROW_UD;
    int Joy_RJOG_RL = GL_JOY_RJOG_RL;
    int Joy_LJOG_RL = GL_JOY_LJOG_RL;
    int Joy_RJOG_UD = GL_JOY_RJOG_UD;
    int Joy_LJOG_UD = GL_JOY_LJOG_UD;
    int Joy_BACK = GL_JOY_BACK;
    int Joy_START = GL_JOY_START;
    int Joy_RJOG_PUSH = GL_JOY_RJOG_PUSH;
    int Joy_LJOG_PUSH = GL_JOY_LJOG_PUSH;


    if(_isFirst == true){
        for(int i=0;i<9;i++){
            SpeedDW_V0[i] = 0;
            SpeedUp_time[i] = 0;
            SlowDown_time[i] = 0;
            plus_Dcc_time[i] = 0;
            minus_Dcc_time[i] = 0;
            MoveFlag[i] = false;
        }

        RIGHT_isFirst = false;
        LEFT_isFirst = false;
        Joy_A_before = 0;
        Joy_AROW_RL_before = 0;
        Joy_AROW_UD_before = 0;
        Joy_B_before = 0;
        Joy_LB_before = 0;
        Joy_LJOG_RL_before = 0;
        Joy_LJOG_UD_before = 0;
        Joy_LT_before = 0;
        Joy_RB_before = 0;
        Joy_RJOG_RL_before = 0;
        Joy_RJOG_UD_before = 0;
        Joy_RT_before = 0;
        Joy_X_before = 0;
        Joy_Y_before = 0;


        if(RL_mode == RIGHT){
            for(int i=0 ;i<3 ; i++){
                manual_pos[i] = WBmotion->pRH_3x1[i];
                manual_pos_before[i] = WBmotion->pRH_3x1[i];
            }
            manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
            manual_quat_before = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);

            RELB = WBmotion->RElb_ang*R2D;
            RELB_before = WBmotion->RElb_ang*R2D;

            WST_ang = joint->GetJointRefAngle(WST);
            WST_ang_before = joint->GetJointRefAngle(WST);
        }
        else{
            for(int i=0 ;i<3 ; i++){
                manual_pos[i] = WBmotion->pLH_3x1[i];
                manual_pos_before[i] = WBmotion->pLH_3x1[i];
            }
            manual_quat = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
            manual_quat_before = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);

            RELB = WBmotion->LElb_ang*R2D;
            RELB_before =  WBmotion->LElb_ang*R2D;

            WST_ang = joint->GetJointRefAngle(WST);
            WST_ang_before = joint->GetJointRefAngle(WST);
        }
        _isFirst = false;
    }


    if(RL_mode == LEFT && switching_counter == 10){
        RL_mode = RIGHT;
        _isFirst = true;
        RIGHT_isFirst = true;
        LEFT_isFirst = false;
        switching_counter = 0;
    }
    if(RL_mode == RIGHT && switching_counter == 10){
        RL_mode = LEFT;
        _isFirst = true;
        LEFT_isFirst = true;
        RIGHT_isFirst = false;
        switching_counter = 0;
    }

    if(RIGHT_isFirst == true){
        for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pRH_3x1[i];
            manual_pos_before[i] = WBmotion->pRH_3x1[i];
        }
        manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        manual_quat_before = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);

        RELB = WBmotion->RElb_ang*R2D;
        RELB_before = WBmotion->RElb_ang*R2D;

        WST_ang = joint->GetJointRefAngle(WST);
        WST_ang_before = joint->GetJointRefAngle(WST);
        RIGHT_isFirst = false;
        cout<<"RIGHT HAND is Activated !!"<<endl;
    }

    else if(LEFT_isFirst == true){
        for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pLH_3x1[i];
            manual_pos_before[i] = WBmotion->pLH_3x1[i];
        }
        manual_quat = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
        manual_quat_before = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);

        RELB = WBmotion->LElb_ang*R2D;
        RELB_before = WBmotion->LElb_ang*R2D;
        WST_ang = joint->GetJointRefAngle(WST);
        WST_ang_before = joint->GetJointRefAngle(WST);
        LEFT_isFirst = false;
        cout<<"LEFT HAND is Activated !!"<<endl;

    }

    if(RL_mode == RIGHT){
        manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    }
    else{
        manual_quat = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
    }



    // Generate Position Trajectory
    for(int i=0;i<3;i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            pVel[i] = Acc*SpeedUp_time[i]*0.005;
            if(pVel[i] >= MaxVel) pVel[i] = MaxVel;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = pVel[i];
        }
        else{
            SlowDown_time[i]++;
            pVel[i] = SpeedDW_V0[i] - Dcc*SlowDown_time[i]*0.005;
            if(pVel[i] <= 0) pVel[i] = 0;
            SpeedUp_time[i] = 0;
//            RH_SpeedUP_V0[i] = pVel[i];
        }
    }

    // Generatie Angualr speed Trajectory
    for(int i=3 ; i<8 ; i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            AngSpeed[i] = Accq*SpeedUp_time[i]*0.005;
            if(AngSpeed[i] >= MaxVel_ang) AngSpeed[i] = MaxVel_ang;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = AngSpeed[i];
        }
        else{
            SlowDown_time[i]++;
            AngSpeed[i] = SpeedDW_V0[i] - Dccq*SlowDown_time[i]*0.005;
            if(AngSpeed[i] <= 0) AngSpeed[i] = 0;
            SpeedUp_time[i] = 0;
 //         RH_SpeedUP_V0[i] = AngSpeed[i];
        }
    }



    counter++;

 //   if(counter%100 == 0)
 //       cout<<"Joy Value: "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]<<" time: "<<SpeedUp_time<<" pVel: "<<pVel<<" MoveFlag: "<<MoveFlag<<endl;


    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2] && Joy_X_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]
            && Joy_B_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] && Joy_RT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] && Joy_RB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]
            && Joy_LT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6] && Joy_LB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7] && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]
            && Joy_AROW_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9] && Joy_RJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] && Joy_LJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]
            && Joy_RJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] && Joy_LJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_AROW_UD_before = Joy_AROW_UD;
    Joy_B_before = Joy_B;
    Joy_LB_before = Joy_LB;
    Joy_LJOG_RL_before = Joy_LJOG_RL;
    Joy_LJOG_UD_before = Joy_LJOG_UD;
    Joy_LT_before = Joy_LT;
    Joy_RB_before = Joy_RB;
    Joy_RJOG_RL_before = Joy_RJOG_RL;
    Joy_RJOG_UD_before = Joy_RJOG_UD;
    Joy_RT_before = Joy_RT;
    Joy_X_before = Joy_X;
    Joy_Y_before = Joy_Y;

    if(same_counter >= 400){
        Joy_Y = 0; Joy_A = 0; Joy_B = 0; Joy_X = 0; Joy_RT = 0; Joy_RB = 0; Joy_LT = 0;
        Joy_LB = 0; Joy_AROW_RL = 0; Joy_AROW_UD = 0; Joy_RJOG_RL = 0; Joy_LJOG_RL = 0;
        Joy_RJOG_UD = 0; Joy_LJOG_UD = 0;
    }


    // RH pos +X
    if(Joy_Y == 1 && Joy_A == 0){
        MoveFlag[0] = true;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && plus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] -= 1;
    }

    // RH pos -X
    if(Joy_Y == 0 && Joy_A == 1){
        MoveFlag[0] = true;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && minus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] -= 1;
    }

    // RH pos -Y
    if(Joy_B == 1 && Joy_X == 0){
        MoveFlag[1] = true;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && plus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] -= 1;
    }

    // RH pos +Y
    if(Joy_B == 0 && Joy_X == 1){
        MoveFlag[1] = true;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && minus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] -= 1;
    }

    // RH pos +Z
    if(Joy_RB == 1 && Joy_RT == 0){
        MoveFlag[2] = true;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && plus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] -= 1;
    }

    // RH pos -Z
    if(Joy_RB == 0 && Joy_RT == 1){
        MoveFlag[2] = true;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && minus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] -= 1;
    }

    // RH ori +X
    if(Joy_AROW_RL == -32767){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && plus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] -=1;
    }

    // RH ori -X
    if(Joy_AROW_RL == +32767){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && minus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] -=1;
    }

    // RH ori +Y
    if(Joy_AROW_UD == +32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && plus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] -=1;
    }

    // RH ori -Y
    if(Joy_AROW_UD == -32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && minus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] -=1;
    }

    // RH ori +Z
    if(Joy_LB == 1 && Joy_LT == 0){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && plus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] -=1;
    }

    // RH ori -Z
    if(Joy_LB == 0 && Joy_LT == 1){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && minus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] -=1;
    }


    // ELB +
    if(Joy_RJOG_UD == -32767 && Joy_RJOG_PUSH == 0){
        MoveFlag[6] = true;
        if(RL_mode == RIGHT)
            RELB += AngSpeed[6]*R2D*0.2;
        else
            RELB -= AngSpeed[6]*R2D*0.2;
        plus_Dcc_time[6] = AngSpeed[6]/Dccq*200;
    }
    if(Joy_RJOG_UD > -32767 && Joy_RJOG_UD <= 0 && plus_Dcc_time[6] > 0 && Joy_RJOG_PUSH == 0){
        MoveFlag[6] = false;
        if(RL_mode == RIGHT)
            RELB += AngSpeed[6]*R2D*0.2;
        else
            RELB -= AngSpeed[6]*R2D*0.2;
        plus_Dcc_time[6] -=1;
    }
    // ELB -
    if(Joy_RJOG_UD == +32767 && Joy_RJOG_PUSH == 0){
        MoveFlag[6] = true;
        if(RL_mode == RIGHT)
            RELB -= AngSpeed[6]*R2D*0.2;
        else
            RELB += AngSpeed[6]*R2D*0.2;
        minus_Dcc_time[6] = AngSpeed[6]/Dccq*200;
    }
    if(Joy_RJOG_UD < 32767 && Joy_RJOG_UD >=0 && minus_Dcc_time[6] > 0 && Joy_RJOG_PUSH == 0){
        MoveFlag[6] = false;
        if(RL_mode == RIGHT)
            RELB -= AngSpeed[6]*R2D*0.2;
        else
            RELB += AngSpeed[6]*R2D*0.2;
        minus_Dcc_time[6] -=1;
    }

    // WST +
    if(Joy_RJOG_RL == -32767 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = true;
        WST_ang += AngSpeed[7]*R2D*0.2;
        plus_Dcc_time[7] = AngSpeed[7]/Dccq*200;
    }
    if(Joy_RJOG_RL > -32767 && Joy_RJOG_RL <=0 && plus_Dcc_time[7] > 0 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = false;
        WST_ang += AngSpeed[7]*R2D*0.2;
        plus_Dcc_time[7] -=1;
    }
    // WST -
    if(Joy_RJOG_RL == +32767 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = true;
        WST_ang -= AngSpeed[7]*R2D*0.2;
        minus_Dcc_time[7] = AngSpeed[7]/Dccq*200;
    }
    if(Joy_RJOG_RL < 32767 && Joy_RJOG_RL >= 0 && minus_Dcc_time[7] > 0 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = false;
        WST_ang -= AngSpeed[7]*R2D*0.2;
        minus_Dcc_time[7] -=1;
    }

    // Finger Grasping
    if(Joy_LJOG_UD == 32767 && Joy_LJOG_PUSH == 0){
        Grasping_value = 125/2;
    }
    else if(Joy_LJOG_UD == -32767 && Joy_LJOG_PUSH == 0){
        Grasping_value = -125/2;
    }
    else{
        Grasping_value = 0;
    }

    // Left-Right switching
    if(RL_mode == RIGHT && Joy_BACK == 1 && Joy_START == 0){
        switching_counter++;
    }
    else if(RL_mode == LEFT && Joy_BACK == 0 && Joy_START == 1){
        switching_counter++;
    }
    else{
        switching_counter = 0;
    }

    // Mode Switching
    //static unsigned int One_Hand_switching_counter = 0;
    static unsigned int Two_Hand_switching_counter = 0;
    static unsigned int One_Foot_switching_counter = 0;
    static unsigned int Two_Foot_switching_counter = 0;

    //if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 32767 && Joy_LJOG_UD == 32767){
    if(Joy_BACK == 1 && Joy_START == 1){
        Two_Hand_switching_counter++;
        cout<<Two_Hand_switching_counter<<endl;
    }
    else if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == -32767 && Joy_LJOG_UD == -32767){
        Two_Foot_switching_counter++;
    }
//    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == 32767){
//        One_Hand_switching_counter++;
//    }
    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == -32767){
        One_Foot_switching_counter++;
    }
    else{
        Two_Hand_switching_counter = 0;
        Two_Foot_switching_counter = 0;
 //       One_Hand_switching_counter = 0;
        One_Foot_switching_counter = 0;
    }
    if(Two_Hand_switching_counter >= 50){
        if(WB_FLAG_WHOLEBODY == true){
            cout<<"OK>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_STADING_START;
            //Two_Hand_switching_counter = 0;
        }
        else{
            cout<<"OK>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;
            //Two_Hand_switching_counter = 0;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_MODE_START;

        }
    }
    else if(Two_Foot_switching_counter >= 20){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START;
        Two_Foot_switching_counter = 0;
    }
    else if(One_Foot_switching_counter >= 50){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_FOOT_MODE_START;
        One_Foot_switching_counter = 0;
    }
//    else if(One_Hand_switching_counter >= 50){
//        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_MODE_START;
//        One_Hand_switching_counter = 0;
//    }


    // Add POS info
    if(RL_mode == RIGHT){
        // boundary of work-space
        //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        mat3 Hand_Ori_mat = mat3(manual_quat);
        vec3 Hand_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
        vec3 WristToHand_HandFrame = vec3(0,0,-0.16);
        mat3 Hand_Ori_mat_inv = Hand_Ori_mat.inverse();
        vec3 WristToHand_GlobalFrame = WristToHand_HandFrame*Hand_Ori_mat_inv;
        vec3 Wrist_pos = Hand_pos - WristToHand_GlobalFrame;

        ShoulderToWrist = (Wrist_pos.x - 0)*(Wrist_pos.x - 0) + (Wrist_pos.y + 0.206)*(Wrist_pos.y + 0.206) + (Wrist_pos.z - 0.456)*(Wrist_pos.z - 0.456);
        ShoulderToWrist = sqrt(ShoulderToWrist);

        if(ShoulderToWrist < 0.68 && ShoulderToWrist > 0.15 && Wrist_pos.x > -0.55 && Wrist_pos.y < 0.15)
            Manual_OK = true;

        else
            Manual_OK = false;


        if(Manual_OK == true){
            //--------------------------pos--------------------------------------------
            WBmotion->addRHPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
            for(int i=0 ; i<3 ; i++)
                manual_pos_before[i] = manual_pos[i];

            //-------------------------Ori------------------------------------------------
            doubles RH_manual_ori(4);
            for(int i=0 ; i<4 ; i++)
                RH_manual_ori[i] = manual_quat[i];

            WBmotion->addRHOriInfo(RH_manual_ori, 0.005);

            manual_quat_before = manual_quat;
            //manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);


        }
        else{
            for(int i=0 ; i<3 ; i++)
                manual_pos[i] = manual_pos_before[i];

            manual_quat = manual_quat_before;
        }




        //------------------------ELB--------------------------------------------------
       // if(RELB < 0 || RELB > -90){
            WBmotion->addRElbPosInfo(RELB,0.005);
            WBmotion->des_RElb_ang = RELB;
            RELB_before = RELB;

        //}
//        else{
//            RELB = RELB_before;
//        }
        //-------------------------------------------------------------------------------



        //------------------------Finger-----------------------------------------------
        joint->SetJointRefAngle(RHAND, Grasping_value);
        if(sharedSEN->EXF_R_Enabled){
            joint->SetJointRefAngle(RF1, Grasping_value);
            joint->SetJointRefAngle(RF2, Grasping_value);
            joint->SetJointRefAngle(RF3, Grasping_value);
            joint->SetJointRefAngle(RF4, Grasping_value);
        }
        //----------------------------------------------------------------------------
    }
    else{
        // boundary of work-space

        mat3 Hand_Ori_mat = mat3(manual_quat);
        vec3 Hand_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
        vec3 WristToHand_HandFrame = vec3(0,0,-0.16);
        mat3 Hand_Ori_mat_inv = Hand_Ori_mat.inverse();
        vec3 WristToHand_GlobalFrame = WristToHand_HandFrame*Hand_Ori_mat_inv;
        vec3 Wrist_pos = Hand_pos - WristToHand_GlobalFrame;

        ShoulderToWrist = (Wrist_pos.x - 0)*(Wrist_pos.x - 0) + (Wrist_pos.y - 0.206)*(Wrist_pos.y - 0.206) + (Wrist_pos.z - 0.456)*(Wrist_pos.z - 0.456);
        ShoulderToWrist = sqrt(ShoulderToWrist);

        if(ShoulderToWrist < 0.68 && ShoulderToWrist > 0.15 && Wrist_pos.x > -0.55 && Wrist_pos.y > -0.15)
            Manual_OK = true;

        else
            Manual_OK = false;


        if(Manual_OK == true){
            //--------------------------pos--------------------------------------------
            WBmotion->addLHPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
            for(int i=0 ; i<3 ; i++)
                manual_pos_before[i] = manual_pos[i];


            //-------------------------Ori------------------------------------------------
            doubles RH_manual_ori(4);
            for(int i=0 ; i<4 ; i++)
                RH_manual_ori[i] = manual_quat[i];

            WBmotion->addLHOriInfo(RH_manual_ori, 0.005);

            manual_quat_before = manual_quat;


        }
        else{
            for(int i=0 ; i<3 ; i++)
                manual_pos[i] = manual_pos_before[i];

            manual_quat = manual_quat_before;
        }
        //---------------------------------------------------------------------------

        //------------------------ELB--------------------------------------------------
//        if(RELB < 0 || RELB > -90){
            WBmotion->addLElbPosInfo(RELB,0.005);
            RELB_before = RELB;
//        }
//        else{
//            RELB = RELB_before;
//        }
        //-------------------------------------------------------------------------------
        //------------------------Finger-----------------------------------------------
        joint->SetJointRefAngle(LHAND, Grasping_value);
        if(sharedSEN->EXF_L_Enabled){
            joint->SetJointRefAngle(LF1, Grasping_value);
            joint->SetJointRefAngle(LF2, Grasping_value);
            joint->SetJointRefAngle(LF3, Grasping_value);
            joint->SetJointRefAngle(LF4, Grasping_value);
        }
        //----------------------------------------------------------------------------
    }
    //--------------------------WST-------------------------------------------------
    if(WST_ang > -540 && WST_ang < 540){
        WBmotion->addWSTPosInfo(WST_ang,0.005);
        WST_ang_before = WST_ang;
    }
    else{
        WST_ang = WST_ang_before;
    }
    //-------------------------------------------------------------------------------
    if(counter >= 100){
        //
        counter = 0;
    }

}

void ManualMoveHand_Right(void){
    // Variables
    static int counter = 0;
    double pVel[3] = {0,0,0};
//    static double RH_SpeedUP_V0[6] = {0,0,0};
    static double SpeedDW_V0[8] = {0,0,0,0,0,0,0,0};
    static int SpeedUp_time[8] = {0,0,0,0,0,0,0,0};
    static int SlowDown_time[8] = {0,0,0,0,0,0,0,0};
    static int plus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    static int minus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    double Acc = 0.5;
    double Dcc = 0.5;
    double MaxVel = 0.07;
    static bool MoveFlag[8] = {false,false,false,false,false,false,false,false};
 //   double ShoulderToHand;
    double ShoulderToWrist;
    static bool Manual_OK = true;
    static double manual_pos[3];
    static double manual_pos_before[3];
    double Accq = 0.05;
    double Dccq = 0.02;
    double MaxVel_ang = 0.006;
    double AngSpeed[8];
    int Grasping_value = 0;
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_AROW_UD_before;
    static int Joy_B_before;
    static int Joy_LB_before;
    static int Joy_LJOG_RL_before;
    static int Joy_LJOG_UD_before;
    static int Joy_LT_before;
    static int Joy_RB_before;
    static int Joy_RJOG_RL_before;
    static int Joy_RJOG_UD_before;
    static int Joy_RT_before;
    static int Joy_X_before;
    static int Joy_Y_before;
    static int Joy_BACK_before;
    static int Joy_START_before;
//    static bool Release_Flag = false;
    static unsigned int same_counter = 0;
    static quat manual_quat;
    static quat manual_quat_before;
    quat qVel;
    static double WST_ang;
    static double WST_ang_before;

    if(_isFirst_both_R == true){
        for(int i=0;i<9;i++){
            SpeedDW_V0[i] = 0;
            SpeedUp_time[i] = 0;
            SlowDown_time[i] = 0;
            plus_Dcc_time[i] = 0;
            minus_Dcc_time[i] = 0;
            MoveFlag[i] = false;
        }

        Joy_A_before = 0;
        Joy_AROW_RL_before = 0;
        Joy_AROW_UD_before = 0;
        Joy_B_before = 0;
        Joy_LB_before = 0;
        Joy_LJOG_RL_before = 0;
        Joy_LJOG_UD_before = 0;
        Joy_LT_before = 0;
        Joy_RB_before = 0;
        Joy_RJOG_RL_before = 0;
        Joy_RJOG_UD_before = 0;
        Joy_RT_before = 0;
        Joy_X_before = 0;
        Joy_Y_before = 0;
        Joy_START_before = 0;
        Joy_BACK_before = 0;

        for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pRH_3x1[i];
            manual_pos_before[i] = WBmotion->pRH_3x1[i];
        }
        manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        manual_quat_before = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);

        WST_ang = joint->GetJointRefAngle(WST);
        WST_ang_before = joint->GetJointRefAngle(WST);

        _isFirst_both_R = false;
    }


    manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);


    // Generate Position Trajectory
    for(int i=0;i<3;i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            pVel[i] = Acc*SpeedUp_time[i]*0.005;
            if(pVel[i] >= MaxVel) pVel[i] = MaxVel;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = pVel[i];
        }
        else{
            SlowDown_time[i]++;
            pVel[i] = SpeedDW_V0[i] - Dcc*SlowDown_time[i]*0.005;
            if(pVel[i] <= 0) pVel[i] = 0;
            SpeedUp_time[i] = 0;
//            RH_SpeedUP_V0[i] = pVel[i];
        }
    }

    // Generatie Angualr speed Trajectory
    for(int i=3 ; i<8 ; i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            AngSpeed[i] = Accq*SpeedUp_time[i]*0.005;
            if(AngSpeed[i] >= MaxVel_ang) AngSpeed[i] = MaxVel_ang;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = AngSpeed[i];
        }
        else{
            SlowDown_time[i]++;
            AngSpeed[i] = SpeedDW_V0[i] - Dccq*SlowDown_time[i]*0.005;
            if(AngSpeed[i] <= 0) AngSpeed[i] = 0;
            SpeedUp_time[i] = 0;
 //         RH_SpeedUP_V0[i] = AngSpeed[i];
        }
    }



    counter++;

 //   if(counter%100 == 0)
 //       cout<<"Joy Value: "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]<<" time: "<<SpeedUp_time<<" pVel: "<<pVel<<" MoveFlag: "<<MoveFlag<<endl;
//    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
//    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
//    int Joy_X = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
//    int Joy_B = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];
//    int Joy_RT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
//    int Joy_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
//    int Joy_LT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6];
//    int Joy_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7];
//    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
//    int Joy_AROW_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
//    int Joy_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
//    int Joy_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
//    int Joy_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
//    int Joy_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
//    int Joy_RJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[12];
//    int Joy_LJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
//    int Joy_BACK = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10];
//    int Joy_START = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[11];
    int Joy_Y = GL_JOY_Y;
    int Joy_A = GL_JOY_A;
    int Joy_X = GL_JOY_X;
    int Joy_B = GL_JOY_B;
    int Joy_RT = GL_JOY_RT;
    int Joy_RB = GL_JOY_RB;
    int Joy_LT = GL_JOY_LT;
    int Joy_LB = GL_JOY_LB;
    int Joy_AROW_RL = GL_JOY_AROW_RL;
    int Joy_AROW_UD = GL_JOY_AROW_UD;
    int Joy_RJOG_RL = GL_JOY_RJOG_RL;
    int Joy_LJOG_RL = GL_JOY_LJOG_RL;
    int Joy_RJOG_UD = GL_JOY_RJOG_UD;
    int Joy_LJOG_UD = GL_JOY_LJOG_UD;
    int Joy_BACK = GL_JOY_BACK;
    int Joy_START = GL_JOY_START;
    int Joy_RJOG_PUSH = GL_JOY_RJOG_PUSH;
    int Joy_LJOG_PUSH = GL_JOY_LJOG_PUSH;



    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2] && Joy_X_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]
            && Joy_B_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] && Joy_RT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] && Joy_RB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]
            && Joy_LT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6] && Joy_LB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7] && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]
            && Joy_AROW_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9] && Joy_RJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] && Joy_LJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]
            && Joy_RJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] && Joy_LJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_AROW_UD_before = Joy_AROW_UD;
    Joy_B_before = Joy_B;
    Joy_LB_before = Joy_LB;
    Joy_LJOG_RL_before = Joy_LJOG_RL;
    Joy_LJOG_UD_before = Joy_LJOG_UD;
    Joy_LT_before = Joy_LT;
    Joy_RB_before = Joy_RB;
    Joy_RJOG_RL_before = Joy_RJOG_RL;
    Joy_RJOG_UD_before = Joy_RJOG_UD;
    Joy_RT_before = Joy_RT;
    Joy_X_before = Joy_X;
    Joy_Y_before = Joy_Y;

    if(same_counter >= 400){
        Joy_Y = 0;
        Joy_A = 0;
        Joy_B = 0;
        Joy_X = 0;
        Joy_RT = 0;
        Joy_RB = 0;
        Joy_LT = 0;
        Joy_LB = 0;
        Joy_AROW_RL = 0;
        Joy_AROW_UD = 0;
        Joy_RJOG_RL = 0;
        Joy_LJOG_RL = 0;
        Joy_RJOG_UD = 0;
        Joy_LJOG_UD = 0;
    }


    // RH pos +X
    if(Joy_Y == 1 && Joy_A == 0){
        MoveFlag[0] = true;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && plus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] -= 1;
    }

    // RH pos -X
    if(Joy_Y == 0 && Joy_A == 1){
        MoveFlag[0] = true;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && minus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] -= 1;
    }

    // RH pos -Y
    if(Joy_B == 1 && Joy_X == 0){
        MoveFlag[1] = true;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && plus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] -= 1;
    }

    // RH pos +Y
    if(Joy_B == 0 && Joy_X == 1){
        MoveFlag[1] = true;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && minus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] -= 1;
    }

    // RH pos +Z
    if(Joy_RB == 1 && Joy_RT == 0){
        MoveFlag[2] = true;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && plus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] -= 1;
    }

    // RH pos -Z
    if(Joy_RB == 0 && Joy_RT == 1){
        MoveFlag[2] = true;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && minus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] -= 1;
    }

    // RH ori +X
    if(Joy_LB == 1 && Joy_LT == 0){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && plus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[3] -=1;
    }

    // RH ori -X
    if(Joy_LB == 0 && Joy_LT == 1){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && minus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[3] -=1;
    }

    // RH ori +Y
    if(Joy_AROW_UD == +32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD ==0 && plus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[4] -=1;
    }

    // RH ori -Y
    if(Joy_AROW_UD == -32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD ==0 && minus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[4] -=1;
    }

    // RH ori +Z
    if(Joy_AROW_RL == -32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && plus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[5] -=1;
    }

    // RH ori -Z
    if(Joy_AROW_RL == +32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && minus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[5] -=1;
    }


    // WST +
    if(Joy_RJOG_RL == -32767 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = true;
        WST_ang += AngSpeed[7]*R2D*0.2;
        plus_Dcc_time[7] = AngSpeed[7]/Dccq*200;
    }
    if(Joy_RJOG_RL > -32767 && Joy_RJOG_RL <=0 && plus_Dcc_time[7] > 0 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = false;
        WST_ang += AngSpeed[7]*R2D*0.2;
        plus_Dcc_time[7] -=1;
    }
    // WST -
    if(Joy_RJOG_RL == +32767 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = true;
        WST_ang -= AngSpeed[7]*R2D*0.2;
        minus_Dcc_time[7] = AngSpeed[7]/Dccq*200;
    }
    if(Joy_RJOG_RL < 32767 && Joy_RJOG_RL >= 0 && minus_Dcc_time[7] > 0 && Joy_RJOG_PUSH == 0){
        MoveFlag[7] = false;
        WST_ang -= AngSpeed[7]*R2D*0.2;
        minus_Dcc_time[7] -=1;
    }



    // Finger Grasping
    if(Joy_LJOG_UD == 32767 && Joy_LJOG_PUSH == 0){
        Grasping_value = 75/2;
    }
    else if(Joy_LJOG_UD == -32767 && Joy_LJOG_PUSH == 0){
        Grasping_value = -75/2;
    }
    else{
        Grasping_value = 0;
    }

    // Mode Switching
    static unsigned int One_Hand_switching_counter = 0;
    //static unsigned int Two_Hand_switching_counter = 0;
    static unsigned int One_Foot_switching_counter = 0;
    static unsigned int Two_Foot_switching_counter = 0;

//    if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 32767 && Joy_LJOG_UD == 32767){
//        Two_Hand_switching_counter++;
//    }
    if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == -32767 && Joy_LJOG_UD == -32767){
        Two_Foot_switching_counter++;
    }
   // else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == 32767){
    else if((Joy_BACK == 1 && Joy_START == 0)||(Joy_BACK == 0 && Joy_START == 1)){
        cout<<One_Hand_switching_counter<<endl;
        One_Hand_switching_counter++;
    }
    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == -32767){
        One_Foot_switching_counter++;
    }
    else{
 //       Two_Hand_switching_counter = 0;
        Two_Foot_switching_counter = 0;
        One_Hand_switching_counter = 0;
        One_Foot_switching_counter = 0;
    }
//    if(Two_Hand_switching_counter >= 20){
//        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_MODE_START;
//        Two_Hand_switching_counter = 0;
//    }
    if(Two_Foot_switching_counter >= 20){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START;
        Two_Foot_switching_counter = 0;
    }
    else if(One_Foot_switching_counter >= 50){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_FOOT_MODE_START;
        One_Foot_switching_counter = 0;
    }
    else if(One_Hand_switching_counter >= 50){
        if(WB_FLAG_WHOLEBODY == true){
            cout<<"OK<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_ONE_HAND_STADING_START;
            //One_Hand_switching_counter = 0;
        }
        else{
            cout<<"OK<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_MODE_START;
            //One_Hand_switching_counter = 0;
        }
    }

    // Add POS info

    // boundary of work-space
    //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    mat3 Hand_Ori_mat = mat3(manual_quat);
    vec3 Hand_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
    vec3 WristToHand_HandFrame = vec3(0,0,-0.16);
    mat3 Hand_Ori_mat_inv = Hand_Ori_mat.inverse();
    vec3 WristToHand_GlobalFrame = WristToHand_HandFrame*Hand_Ori_mat_inv;
    vec3 Wrist_pos = Hand_pos - WristToHand_GlobalFrame;

    ShoulderToWrist = (Wrist_pos.x - 0)*(Wrist_pos.x - 0) + (Wrist_pos.y + 0.206)*(Wrist_pos.y + 0.206) + (Wrist_pos.z - 0.456)*(Wrist_pos.z - 0.456);
    ShoulderToWrist = sqrt(ShoulderToWrist);

    if(ShoulderToWrist < 0.68 && ShoulderToWrist > 0.15 && Wrist_pos.x > -0.5 && Wrist_pos.y < 0.05)
        Manual_OK = true;

    else
        Manual_OK = false;


    if(Manual_OK == true){
        //--------------------------pos--------------------------------------------
        WBmotion->addRHPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
        for(int i=0 ; i<3 ; i++){
            manual_pos_before[i] = manual_pos[i];
        }


        //-------------------------Ori------------------------------------------------
        doubles RH_manual_ori(4);
        for(int i=0 ; i<4 ; i++)
            RH_manual_ori[i] = manual_quat[i];

        WBmotion->addRHOriInfo(RH_manual_ori, 0.005);

        manual_quat_before = manual_quat;
        //manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);


    }
    else{
        for(int i=0 ; i<3 ; i++)
            manual_pos[i] = manual_pos_before[i];

        manual_quat = manual_quat_before;
    }



    //------------------------Finger-----------------------------------------------
    joint->SetJointRefAngle(RHAND, Grasping_value);
    if(sharedSEN->EXF_R_Enabled){
        joint->SetJointRefAngle(RF1, Grasping_value);
        joint->SetJointRefAngle(RF2, Grasping_value);
        joint->SetJointRefAngle(RF3, Grasping_value);
        joint->SetJointRefAngle(RF4, Grasping_value);
    }
    //----------------------------------------------------------------------------
    //------------------------Finger-----------------------------------------------
    joint->SetJointRefAngle(LHAND, Grasping_value);
    if(sharedSEN->EXF_L_Enabled){
        joint->SetJointRefAngle(LF1, Grasping_value);
        joint->SetJointRefAngle(LF2, Grasping_value);
        joint->SetJointRefAngle(LF3, Grasping_value);
        joint->SetJointRefAngle(LF4, Grasping_value);
    }
    //----------------------------------------------------------------------------

    //--------------------------WST-------------------------------------------------
    if(WST_ang > -540 && WST_ang < 540){
        WBmotion->addWSTPosInfo(WST_ang,0.005);
        WST_ang_before = WST_ang;
    }
    else{
        WST_ang = WST_ang_before;
    }
    //-------------------------------------------------------------------------------



}

void ManualMoveHand_Left(void){
    // Variables
    static int counter = 0;
    double pVel[3] = {0,0,0};
//    static double RH_SpeedUP_V0[6] = {0,0,0};
    static double SpeedDW_V0[8] = {0,0,0,0,0,0,0,0};
    static int SpeedUp_time[8] = {0,0,0,0,0,0,0,0};
    static int SlowDown_time[8] = {0,0,0,0,0,0,0,0};
    static int plus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    static int minus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    double Acc = 0.5;
    double Dcc = 0.5;
    double MaxVel = 0.07;
    static bool MoveFlag[8] = {false,false,false,false,false,false,false,false};
 //   double ShoulderToHand;
    double ShoulderToWrist;
    static bool Manual_OK = true;
    static double manual_pos[3];
    static double manual_pos_before[3];
    double Accq = 0.05;
    double Dccq = 0.02;
    double MaxVel_ang = 0.006;
    double AngSpeed[8];
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_AROW_UD_before;
    static int Joy_B_before;
    static int Joy_LB_before;
    static int Joy_LJOG_RL_before;
    static int Joy_LJOG_UD_before;
    static int Joy_LT_before;
    static int Joy_RB_before;
    static int Joy_RJOG_RL_before;
    static int Joy_RJOG_UD_before;
    static int Joy_RT_before;
    static int Joy_X_before;
    static int Joy_Y_before;
//    static bool Release_Flag = false;
    static unsigned int same_counter = 0;
    static quat manual_quat;
    static quat manual_quat_before;
    quat qVel;

    if(_isFirst_both_L == true){
        for(int i=0;i<9;i++){
            SpeedDW_V0[i] = 0;
            SpeedUp_time[i] = 0;
            SlowDown_time[i] = 0;
            plus_Dcc_time[i] = 0;
            minus_Dcc_time[i] = 0;
            MoveFlag[i] = false;
        }
        Joy_A_before = 0;
        Joy_AROW_RL_before = 0;
        Joy_AROW_UD_before = 0;
        Joy_B_before = 0;
        Joy_LB_before = 0;
        Joy_LJOG_RL_before = 0;
        Joy_LJOG_UD_before = 0;
        Joy_LT_before = 0;
        Joy_RB_before = 0;
        Joy_RJOG_RL_before = 0;
        Joy_RJOG_UD_before = 0;
        Joy_RT_before = 0;
        Joy_X_before = 0;
        Joy_Y_before = 0;

        for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pLH_3x1[i];
            manual_pos_before[i] = WBmotion->pLH_3x1[i];
        }
        manual_quat = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);
        manual_quat_before = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);

        _isFirst_both_L = false;
    }


    manual_quat = quat(WBmotion->qLH_4x1[0],WBmotion->qLH_4x1[1],WBmotion->qLH_4x1[2],WBmotion->qLH_4x1[3]);


    // Generate Position Trajectory
    for(int i=0;i<3;i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            pVel[i] = Acc*SpeedUp_time[i]*0.005;
            if(pVel[i] >= MaxVel) pVel[i] = MaxVel;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = pVel[i];
        }
        else{
            SlowDown_time[i]++;
            pVel[i] = SpeedDW_V0[i] - Dcc*SlowDown_time[i]*0.005;
            if(pVel[i] <= 0) pVel[i] = 0;
            SpeedUp_time[i] = 0;
//            RH_SpeedUP_V0[i] = pVel[i];
        }
    }

    // Generatie Angualr speed Trajectory
    for(int i=3 ; i<8 ; i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            AngSpeed[i] = Accq*SpeedUp_time[i]*0.005;
            if(AngSpeed[i] >= MaxVel_ang) AngSpeed[i] = MaxVel_ang;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = AngSpeed[i];
        }
        else{
            SlowDown_time[i]++;
            AngSpeed[i] = SpeedDW_V0[i] - Dccq*SlowDown_time[i]*0.005;
            if(AngSpeed[i] <= 0) AngSpeed[i] = 0;
            SpeedUp_time[i] = 0;
 //         RH_SpeedUP_V0[i] = AngSpeed[i];
        }
    }



    counter++;

 //   if(counter%100 == 0)
 //       cout<<"Joy Value: "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]<<" time: "<<SpeedUp_time<<" pVel: "<<pVel<<" MoveFlag: "<<MoveFlag<<endl;
//    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
//    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
//    int Joy_X = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
//    int Joy_B = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];
//    int Joy_RT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
//    int Joy_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
//    int Joy_LT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6];
//    int Joy_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7];
//    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
//    int Joy_AROW_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
//    int Joy_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
//    int Joy_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
//    int Joy_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
//    int Joy_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
//    int Joy_RJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[12];
//    int Joy_LJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
    int Joy_Y = GL_JOY_Y;
    int Joy_A = GL_JOY_A;
    int Joy_X = GL_JOY_X;
    int Joy_B = GL_JOY_B;
    int Joy_RT = GL_JOY_RT;
    int Joy_RB = GL_JOY_RB;
    int Joy_LT = GL_JOY_LT;
    int Joy_LB = GL_JOY_LB;
    int Joy_AROW_RL = GL_JOY_AROW_RL;
    int Joy_AROW_UD = GL_JOY_AROW_UD;
    int Joy_RJOG_RL = GL_JOY_RJOG_RL;
    int Joy_LJOG_RL = GL_JOY_LJOG_RL;
    int Joy_RJOG_UD = GL_JOY_RJOG_UD;
    int Joy_LJOG_UD = GL_JOY_LJOG_UD;




    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2] && Joy_X_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]
            && Joy_B_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] && Joy_RT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] && Joy_RB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]
            && Joy_LT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6] && Joy_LB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7] && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]
            && Joy_AROW_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9] && Joy_RJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] && Joy_LJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]
            && Joy_RJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] && Joy_LJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_AROW_UD_before = Joy_AROW_UD;
    Joy_B_before = Joy_B;
    Joy_LB_before = Joy_LB;
    Joy_LJOG_RL_before = Joy_LJOG_RL;
    Joy_LJOG_UD_before = Joy_LJOG_UD;
    Joy_LT_before = Joy_LT;
    Joy_RB_before = Joy_RB;
    Joy_RJOG_RL_before = Joy_RJOG_RL;
    Joy_RJOG_UD_before = Joy_RJOG_UD;
    Joy_RT_before = Joy_RT;
    Joy_X_before = Joy_X;
    Joy_Y_before = Joy_Y;

    if(same_counter >= 400){
        Joy_Y = 0;
        Joy_A = 0;
        Joy_B = 0;
        Joy_X = 0;
        Joy_RT = 0;
        Joy_RB = 0;
        Joy_LT = 0;
        Joy_LB = 0;
        Joy_AROW_RL = 0;
        Joy_AROW_UD = 0;
        Joy_RJOG_RL = 0;
        Joy_LJOG_RL = 0;
        Joy_RJOG_UD = 0;
        Joy_LJOG_UD = 0;
    }


    // RH pos +X
    if(Joy_Y == 1 && Joy_A == 0){
        MoveFlag[0] = true;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && plus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] -= 1;
    }

    // RH pos -X
    if(Joy_Y == 0 && Joy_A == 1){
        MoveFlag[0] = true;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && minus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] -= 1;
    }

    // RH pos -Y
    if(Joy_B == 1 && Joy_X == 0){
        MoveFlag[1] = true;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && plus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] -= 1;
    }

    // RH pos +Y
    if(Joy_B == 0 && Joy_X == 1){
        MoveFlag[1] = true;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && minus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] -= 1;
    }

    // RH pos +Z
    if(Joy_RB == 1 && Joy_RT == 0){
        MoveFlag[2] = true;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && plus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] -= 1;
    }

    // RH pos -Z
    if(Joy_RB == 0 && Joy_RT == 1){
        MoveFlag[2] = true;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && minus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] -= 1;
    }

    // RH ori +X
    if(Joy_LB == 1 && Joy_LT == 0){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && plus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[3] -=1;
    }

    // RH ori -X
    if(Joy_LB == 0 && Joy_LT == 1){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && minus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[3] -=1;
    }

    // RH ori +Y
    if(Joy_AROW_UD == +32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD ==0 && plus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[4] -=1;
    }

    // RH ori -Y
    if(Joy_AROW_UD == -32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD ==0 && minus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[4] -=1;
    }

    // RH ori +Z
    if(Joy_AROW_RL == -32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && plus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        plus_Dcc_time[5] -=1;
    }

    // RH ori -Z
    if(Joy_AROW_RL == +32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && minus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = qVel*manual_quat;
        minus_Dcc_time[5] -=1;
    }



    // Add POS info

    // boundary of work-space
    //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    mat3 Hand_Ori_mat = mat3(manual_quat);
    vec3 Hand_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
    vec3 WristToHand_HandFrame = vec3(0,0,-0.16);
    mat3 Hand_Ori_mat_inv = Hand_Ori_mat.inverse();
    vec3 WristToHand_GlobalFrame = WristToHand_HandFrame*Hand_Ori_mat_inv;
    vec3 Wrist_pos = Hand_pos - WristToHand_GlobalFrame;

    ShoulderToWrist = (Wrist_pos.x - 0)*(Wrist_pos.x - 0) + (Wrist_pos.y - 0.206)*(Wrist_pos.y - 0.206) + (Wrist_pos.z - 0.456)*(Wrist_pos.z - 0.456);
    ShoulderToWrist = sqrt(ShoulderToWrist);

    if(ShoulderToWrist < 0.68 && ShoulderToWrist > 0.15 && Wrist_pos.x > -0.5 && Wrist_pos.y > -0.05)
        Manual_OK = true;

    else
        Manual_OK = false;


    if(Manual_OK == true){
        //--------------------------pos--------------------------------------------
        WBmotion->addLHPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
        for(int i=0 ; i<3 ; i++)
            manual_pos_before[i] = manual_pos[i];

        //-------------------------Ori------------------------------------------------
        doubles manual_ori(4);
        for(int i=0 ; i<4 ; i++)
            manual_ori[i] = manual_quat[i];

        WBmotion->addLHOriInfo(manual_ori, 0.005);

        manual_quat_before = manual_quat;
        //manual_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    }
    else{
        for(int i=0 ; i<3 ; i++)
            manual_pos[i] = manual_pos_before[i];

        manual_quat = manual_quat_before;
    }

}

void ManualMoveFoot(void){
    // Variables
    static int counter = 0;
    double pVel[3] = {0,0,0};
    static double SpeedDW_V0[8] = {0,0,0,0,0,0,0,0};
    static int SpeedUp_time[8] = {0,0,0,0,0,0,0,0};
    static int SlowDown_time[8] = {0,0,0,0,0,0,0,0};
    static int plus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    static int minus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    double Acc = 0.5;
    double Dcc = 0.3;
    double MaxVel = 0.07;
    static bool MoveFlag[8] = {false,false,false,false,false,false,false,false};
    double HipToAnkle;
    static bool Manual_OK = true;
    static double manual_pos[3];
    static double manual_pos_before[3];
    double Accq = 0.03;
    double Dccq = 0.02;
    double MaxVel_ang = 0.0025;
    double AngSpeed[8];
    static int switching_counter = 0;
    static bool RL_mode = RIGHT;
    static bool RIGHT_isFirst = true;
    static bool LEFT_isFirst = false;
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_AROW_UD_before;
    static int Joy_B_before;
    static int Joy_LB_before;
    static int Joy_LJOG_RL_before;
    static int Joy_LJOG_UD_before;
    static int Joy_LT_before;
    static int Joy_RB_before;
    static int Joy_RJOG_RL_before;
    static int Joy_RJOG_UD_before;
    static int Joy_RT_before;
    static int Joy_X_before;
    static int Joy_Y_before;
    static unsigned int same_counter = 0;
    quat manual_quat;
    static quat manual_quat_before;
    quat qVel;

    if(_isFirst == true){
        if(RL_mode == RIGHT){
            for(int i=0 ;i<3 ; i++){
                manual_pos[i] = WBmotion->pRF_3x1[i];
                manual_pos_before[i] = WBmotion->pRF_3x1[i];
            }
            manual_quat = quat(WBmotion->qRF_4x1[0],WBmotion->qRF_4x1[1],WBmotion->qRF_4x1[2],WBmotion->qRF_4x1[3]);
            manual_quat_before = quat(WBmotion->qRF_4x1[0],WBmotion->qRF_4x1[1],WBmotion->qRF_4x1[2],WBmotion->qRF_4x1[3]);

        }
        else{
            for(int i=0 ;i<3 ; i++){
                manual_pos[i] = WBmotion->pLF_3x1[i];
                manual_pos_before[i] = WBmotion->pLF_3x1[i];
            }
            manual_quat = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);
            manual_quat_before = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);
        }
        _isFirst = false;
    }


    if(RL_mode == LEFT && switching_counter == 10){
        RL_mode = RIGHT;
        RIGHT_isFirst = true;
        switching_counter = 0;
    }
    if(RL_mode == RIGHT && switching_counter == 10){
        RL_mode = LEFT;
        LEFT_isFirst = true;
        switching_counter = 0;
    }

    if(RIGHT_isFirst == true){
        for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pRF_3x1[i];
            manual_pos_before[i] = WBmotion->pRF_3x1[i];
        }
        manual_quat = quat(WBmotion->pRF_3x1[0],WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2],WBmotion->pRF_3x1[3]);
        manual_quat_before = quat(WBmotion->pRF_3x1[0],WBmotion->pRF_3x1[1],WBmotion->pRF_3x1[2],WBmotion->pRF_3x1[3]);
        RIGHT_isFirst = false;
        cout<<"RIGHT Foot is Activated !!"<<endl;
    }

    if(LEFT_isFirst == true){
        for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pLF_3x1[i];
            manual_pos_before[i] = WBmotion->pLF_3x1[i];
        }
        manual_quat = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);
        manual_quat_before = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);

        LEFT_isFirst = false;
        cout<<"LEFT Foot is Activated !!"<<endl;

    }

    if(RL_mode == RIGHT){
        manual_quat = quat(WBmotion->qRF_4x1[0],WBmotion->qRF_4x1[1],WBmotion->qRF_4x1[2],WBmotion->qRF_4x1[3]);
    }
    else{
        manual_quat = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);
    }



    // Generate Position Trajectory
    for(int i=0;i<3;i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            pVel[i] = Acc*SpeedUp_time[i]*0.005;
            if(pVel[i] >= MaxVel) pVel[i] = MaxVel;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = pVel[i];
        }
        else{
            SlowDown_time[i]++;
            pVel[i] = SpeedDW_V0[i] - Dcc*SlowDown_time[i]*0.005;
            if(pVel[i] <= 0) pVel[i] = 0;
            SpeedUp_time[i] = 0;
//            RH_SpeedUP_V0[i] = pVel[i];
        }
    }

    // Generatie Angualr speed Trajectory
    for(int i=3 ; i<8 ; i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            AngSpeed[i] = Accq*SpeedUp_time[i]*0.005;
            if(AngSpeed[i] >= MaxVel_ang) AngSpeed[i] = MaxVel_ang;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = AngSpeed[i];
        }
        else{
            SlowDown_time[i]++;
            AngSpeed[i] = SpeedDW_V0[i] - Dccq*SlowDown_time[i]*0.005;
            if(AngSpeed[i] <= 0) AngSpeed[i] = 0;
            SpeedUp_time[i] = 0;
 //         RH_SpeedUP_V0[i] = AngSpeed[i];
        }
    }



    counter++;

 //   if(counter%100 == 0)
 //       cout<<"Joy Value: "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]<<" time: "<<SpeedUp_time<<" pVel: "<<pVel<<" MoveFlag: "<<MoveFlag<<endl;
//    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
//    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
//    int Joy_X = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
//    int Joy_B = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];
//    int Joy_RT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
//    int Joy_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
//    int Joy_LT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6];
//    int Joy_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7];
//    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
//    int Joy_AROW_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
//    int Joy_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
//    int Joy_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
//    int Joy_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
//    int Joy_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
//    int Joy_BACK = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[10];
//    int Joy_START = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[11];
//    int Joy_RJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[12];
//    int Joy_LJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
    int Joy_Y = GL_JOY_Y;
    int Joy_A = GL_JOY_A;
    int Joy_X = GL_JOY_X;
    int Joy_B = GL_JOY_B;
    int Joy_RT = GL_JOY_RT;
    int Joy_RB = GL_JOY_RB;
    int Joy_LT = GL_JOY_LT;
    int Joy_LB = GL_JOY_LB;
    int Joy_AROW_RL = GL_JOY_AROW_RL;
    int Joy_AROW_UD = GL_JOY_AROW_UD;
    int Joy_RJOG_RL = GL_JOY_RJOG_RL;
    int Joy_LJOG_RL = GL_JOY_LJOG_RL;
    int Joy_RJOG_UD = GL_JOY_RJOG_UD;
    int Joy_LJOG_UD = GL_JOY_LJOG_UD;
    int Joy_BACK = GL_JOY_BACK;
    int Joy_START = GL_JOY_START;
    int Joy_RJOG_PUSH = GL_JOY_RJOG_PUSH;
    int Joy_LJOG_PUSH = GL_JOY_LJOG_PUSH;


    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2] && Joy_X_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]
            && Joy_B_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] && Joy_RT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] && Joy_RB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]
            && Joy_LT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6] && Joy_LB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7] && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]
            && Joy_AROW_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9] && Joy_RJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] && Joy_LJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]
            && Joy_RJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] && Joy_LJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_AROW_UD_before = Joy_AROW_UD;
    Joy_B_before = Joy_B;
    Joy_LB_before = Joy_LB;
    Joy_LJOG_RL_before = Joy_LJOG_RL;
    Joy_LJOG_UD_before = Joy_LJOG_UD;
    Joy_LT_before = Joy_LT;
    Joy_RB_before = Joy_RB;
    Joy_RJOG_RL_before = Joy_RJOG_RL;
    Joy_RJOG_UD_before = Joy_RJOG_UD;
    Joy_RT_before = Joy_RT;
    Joy_X_before = Joy_X;
    Joy_Y_before = Joy_Y;

    if(same_counter >= 400){
        Joy_Y = 0;
        Joy_A = 0;
        Joy_B = 0;
        Joy_X = 0;
        Joy_RT = 0;
        Joy_RB = 0;
        Joy_LT = 0;
        Joy_LB = 0;
        Joy_AROW_RL = 0;
        Joy_AROW_UD = 0;
        Joy_RJOG_RL = 0;
        Joy_LJOG_RL = 0;
        Joy_RJOG_UD = 0;
        Joy_LJOG_UD = 0;
    }


    // Foot pos +X
    if(Joy_Y == 1 && Joy_A == 0){
        MoveFlag[0] = true;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && plus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] -= 1;
    }

    // Foot pos -X
    if(Joy_Y == 0 && Joy_A == 1){
        MoveFlag[0] = true;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && minus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] -= 1;
    }

    // Foot pos -Y
    if(Joy_B == 1 && Joy_X == 0){
        MoveFlag[1] = true;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && plus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] -= 1;
    }

    // Foot pos +Y
    if(Joy_B == 0 && Joy_X == 1){
        MoveFlag[1] = true;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && minus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] -= 1;
    }

    // Foot pos +Z
    if(Joy_RB == 1 && Joy_RT == 0){
        MoveFlag[2] = true;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && plus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] -= 1;
    }

    // Foot pos -Z
    if(Joy_RB == 0 && Joy_RT == 1){
        MoveFlag[2] = true;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && minus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] -= 1;
    }

    // Foot ori +Z
    if(Joy_LB == 1 && Joy_LT == 0){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && plus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] -=1;
    }

    // Foot ori -Z
    if(Joy_LB == 0 && Joy_LT == 1){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && minus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] -=1;
    }

    // Foot ori +Y
    if(Joy_AROW_UD == +32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && plus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] -=1;
    }

    // Foot ori -Y
    if(Joy_AROW_UD == -32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && minus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] -=1;
    }

    // Foot ori +X
    if(Joy_AROW_RL == -32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && plus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] -=1;
    }

    // Foot ori -X
    if(Joy_AROW_RL == +32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && minus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] -=1;
    }

    // Left-Right switching
    if(RL_mode == RIGHT && Joy_BACK == 1 && Joy_START == 0){
        switching_counter++;
    }
    else if(RL_mode == LEFT && Joy_BACK == 0 && Joy_START == 1){
        switching_counter++;
    }
    else{
        switching_counter = 0;
    }

    // Mode Switching
    static unsigned int One_Hand_switching_counter = 0;
    static unsigned int Two_Hand_switching_counter = 0;
    //static unsigned int One_Foot_switching_counter = 0;
    static unsigned int Two_Foot_switching_counter = 0;

    if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 32767 && Joy_LJOG_UD == 32767){
        Two_Hand_switching_counter++;
    }
    else if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == -32767 && Joy_LJOG_UD == -32767){
        Two_Foot_switching_counter++;
    }
    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == 32767){
        One_Hand_switching_counter++;
    }
//    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == -32767){
//        One_Foot_switching_counter++;
//    }
    else{
        Two_Hand_switching_counter = 0;
        Two_Foot_switching_counter = 0;
        One_Hand_switching_counter = 0;
 //       One_Foot_switching_counter = 0;
    }
    if(Two_Hand_switching_counter >= 20){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_MODE_START;
        Two_Hand_switching_counter = 0;
    }
    else if(Two_Foot_switching_counter >= 20){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START;
        Two_Foot_switching_counter = 0;
    }
//    else if(One_Foot_switching_counter >= 50){
//        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_FOOT_MODE_START;
//        One_Foot_switching_counter = 0;
//    }
    else if(One_Hand_switching_counter >= 50){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_MODE_START;
        One_Hand_switching_counter = 0;
    }


    // Add POS info
    if(RL_mode == RIGHT){
        // boundary of work-space
        //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        mat3 Foot_Ori_mat = mat3(manual_quat);
        vec3 Foot_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
        vec3 AnkleToFoot_FootFrame = vec3(0,0,-0.1);
        mat3 Foot_Ori_mat_inv = Foot_Ori_mat.inverse();
        vec3 AnkleToFoot_GlobalFrame = AnkleToFoot_FootFrame*Foot_Ori_mat_inv;
        vec3 Ankle_pos = Foot_pos - AnkleToFoot_GlobalFrame;

        HipToAnkle = (Ankle_pos.x - 0)*(Ankle_pos.x - 0) + (Ankle_pos.y + 0.105)*(Ankle_pos.y + 0.105) + (Ankle_pos.z - 0.0)*(Ankle_pos.z - 0.0);
        HipToAnkle = sqrt(HipToAnkle);

        if(HipToAnkle < 0.80 && HipToAnkle > 0.30)
            Manual_OK = true;

        else
            Manual_OK = false;


        if(Manual_OK == true){
            //--------------------------pos--------------------------------------------
            WBmotion->addRFPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
            for(int i=0 ; i<3 ; i++)
                manual_pos_before[i] = manual_pos[i];

            //-------------------------Ori------------------------------------------------
            doubles RH_manual_ori(4);
            for(int i=0 ; i<4 ; i++)
                RH_manual_ori[i] = manual_quat[i];

            WBmotion->addRFOriInfo(RH_manual_ori, 0.005);

            manual_quat_before = manual_quat;

        }
        else{
            for(int i=0 ; i<3 ; i++)
                manual_pos[i] = manual_pos_before[i];

            manual_quat = manual_quat_before;
        }


    }
    else if(RL_mode == LEFT){ // LEFT Foot
        // Checking boundary of work-space

        //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
        mat3 Foot_Ori_mat = mat3(manual_quat);
        vec3 Foot_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
        vec3 AnkleToFoot_FootFrame = vec3(0,0,-0.1);
        mat3 Foot_Ori_mat_inv = Foot_Ori_mat.inverse();
        vec3 AnkleToFoot_GlobalFrame = AnkleToFoot_FootFrame*Foot_Ori_mat_inv;
        vec3 Ankle_pos = Foot_pos - AnkleToFoot_GlobalFrame;

        HipToAnkle = (Ankle_pos.x - 0)*(Ankle_pos.x - 0) + (Ankle_pos.y - 0.105)*(Ankle_pos.y - 0.105) + (Ankle_pos.z - 0.0)*(Ankle_pos.z - 0.0);
        HipToAnkle = sqrt(HipToAnkle);

        if(HipToAnkle < 0.80 && HipToAnkle > 0.30)
            Manual_OK = true;

        else
            Manual_OK = false;


        if(Manual_OK == true){
            //--------------------------pos--------------------------------------------
            WBmotion->addLFPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
            for(int i=0 ; i<3 ; i++)
                manual_pos_before[i] = manual_pos[i];

            //-------------------------Ori------------------------------------------------
            doubles RH_manual_ori(4);
            for(int i=0 ; i<4 ; i++)
                RH_manual_ori[i] = manual_quat[i];

            WBmotion->addLFOriInfo(RH_manual_ori, 0.005);

            manual_quat_before = manual_quat;

        }
        else{
            for(int i=0 ; i<3 ; i++)
                manual_pos[i] = manual_pos_before[i];

            manual_quat = manual_quat_before;
        }


    }

}

void ManualMoveFoot_Right(void){
    // Variables
    static int counter = 0;
    double pVel[3] = {0,0,0};
    static double SpeedDW_V0[8] = {0,0,0,0,0,0,0,0};
    static int SpeedUp_time[8] = {0,0,0,0,0,0,0,0};
    static int SlowDown_time[8] = {0,0,0,0,0,0,0,0};
    static int plus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    static int minus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    double Acc = 0.5;
    double Dcc = 0.3;
    double MaxVel = 0.07;
    static bool MoveFlag[8] = {false,false,false,false,false,false,false,false};
    double HipToAnkle;
    static bool Manual_OK = true;
    static double manual_pos[3];
    static double manual_pos_before[3];
    double Accq = 0.03;
    double Dccq = 0.02;
    double MaxVel_ang = 0.0025;
    double AngSpeed[8];
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_AROW_UD_before;
    static int Joy_B_before;
    static int Joy_LB_before;
    static int Joy_LJOG_RL_before;
    static int Joy_LJOG_UD_before;
    static int Joy_LT_before;
    static int Joy_RB_before;
    static int Joy_RJOG_RL_before;
    static int Joy_RJOG_UD_before;
    static int Joy_RT_before;
    static int Joy_X_before;
    static int Joy_Y_before;
    static unsigned int same_counter = 0;
    quat manual_quat;
    static quat manual_quat_before;
    quat qVel;

    if(_isFirst_both_R == true){
       for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pRF_3x1[i];
            manual_pos_before[i] = WBmotion->pRF_3x1[i];
        }
        manual_quat = quat(WBmotion->qRF_4x1[0],WBmotion->qRF_4x1[1],WBmotion->qRF_4x1[2],WBmotion->qRF_4x1[3]);
        manual_quat_before = quat(WBmotion->qRF_4x1[0],WBmotion->qRF_4x1[1],WBmotion->qRF_4x1[2],WBmotion->qRF_4x1[3]);

       _isFirst_both_R = false;
    }



    manual_quat = quat(WBmotion->qRF_4x1[0],WBmotion->qRF_4x1[1],WBmotion->qRF_4x1[2],WBmotion->qRF_4x1[3]);




    // Generate Position Trajectory
    for(int i=0;i<3;i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            pVel[i] = Acc*SpeedUp_time[i]*0.005;
            if(pVel[i] >= MaxVel) pVel[i] = MaxVel;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = pVel[i];
        }
        else{
            SlowDown_time[i]++;
            pVel[i] = SpeedDW_V0[i] - Dcc*SlowDown_time[i]*0.005;
            if(pVel[i] <= 0) pVel[i] = 0;
            SpeedUp_time[i] = 0;
//            RH_SpeedUP_V0[i] = pVel[i];
        }
    }

    // Generatie Angualr speed Trajectory
    for(int i=3 ; i<8 ; i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            AngSpeed[i] = Accq*SpeedUp_time[i]*0.005;
            if(AngSpeed[i] >= MaxVel_ang) AngSpeed[i] = MaxVel_ang;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = AngSpeed[i];
        }
        else{
            SlowDown_time[i]++;
            AngSpeed[i] = SpeedDW_V0[i] - Dccq*SlowDown_time[i]*0.005;
            if(AngSpeed[i] <= 0) AngSpeed[i] = 0;
            SpeedUp_time[i] = 0;
 //         RH_SpeedUP_V0[i] = AngSpeed[i];
        }
    }



    counter++;

 //   if(counter%100 == 0)
 //       cout<<"Joy Value: "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]<<" time: "<<SpeedUp_time<<" pVel: "<<pVel<<" MoveFlag: "<<MoveFlag<<endl;
//    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
//    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
//    int Joy_X = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
//    int Joy_B = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];
//    int Joy_RT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
//    int Joy_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
//    int Joy_LT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6];
//    int Joy_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7];
//    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
//    int Joy_AROW_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
//    int Joy_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
//    int Joy_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
//    int Joy_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
//    int Joy_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];
//    int Joy_RJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[12];
//    int Joy_LJOG_PUSH = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[13];
    int Joy_Y = GL_JOY_Y;
    int Joy_A = GL_JOY_A;
    int Joy_X = GL_JOY_X;
    int Joy_B = GL_JOY_B;
    int Joy_RT = GL_JOY_RT;
    int Joy_RB = GL_JOY_RB;
    int Joy_LT = GL_JOY_LT;
    int Joy_LB = GL_JOY_LB;
    int Joy_AROW_RL = GL_JOY_AROW_RL;
    int Joy_AROW_UD = GL_JOY_AROW_UD;
    int Joy_RJOG_RL = GL_JOY_RJOG_RL;
    int Joy_LJOG_RL = GL_JOY_LJOG_RL;
    int Joy_RJOG_UD = GL_JOY_RJOG_UD;
    int Joy_LJOG_UD = GL_JOY_LJOG_UD;
    int Joy_BACK = GL_JOY_BACK;
    int Joy_START = GL_JOY_START;
    int Joy_RJOG_PUSH = GL_JOY_RJOG_PUSH;
    int Joy_LJOG_PUSH = GL_JOY_LJOG_PUSH;



    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2] && Joy_X_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]
            && Joy_B_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] && Joy_RT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] && Joy_RB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]
            && Joy_LT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6] && Joy_LB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7] && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]
            && Joy_AROW_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9] && Joy_RJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] && Joy_LJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]
            && Joy_RJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] && Joy_LJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_AROW_UD_before = Joy_AROW_UD;
    Joy_B_before = Joy_B;
    Joy_LB_before = Joy_LB;
    Joy_LJOG_RL_before = Joy_LJOG_RL;
    Joy_LJOG_UD_before = Joy_LJOG_UD;
    Joy_LT_before = Joy_LT;
    Joy_RB_before = Joy_RB;
    Joy_RJOG_RL_before = Joy_RJOG_RL;
    Joy_RJOG_UD_before = Joy_RJOG_UD;
    Joy_RT_before = Joy_RT;
    Joy_X_before = Joy_X;
    Joy_Y_before = Joy_Y;

    if(same_counter >= 400){
        Joy_Y = 0;
        Joy_A = 0;
        Joy_B = 0;
        Joy_X = 0;
        Joy_RT = 0;
        Joy_RB = 0;
        Joy_LT = 0;
        Joy_LB = 0;
        Joy_AROW_RL = 0;
        Joy_AROW_UD = 0;
        Joy_RJOG_RL = 0;
        Joy_LJOG_RL = 0;
        Joy_RJOG_UD = 0;
        Joy_LJOG_UD = 0;
    }


    // Foot pos +X
    if(Joy_Y == 1 && Joy_A == 0){
        MoveFlag[0] = true;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && plus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] -= 1;
    }

    // Foot pos -X
    if(Joy_Y == 0 && Joy_A == 1){
        MoveFlag[0] = true;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && minus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] -= 1;
    }

    // Foot pos -Y
    if(Joy_B == 1 && Joy_X == 0){
        MoveFlag[1] = true;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && plus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] -= 1;
    }

    // Foot pos +Y
    if(Joy_B == 0 && Joy_X == 1){
        MoveFlag[1] = true;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && minus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] -= 1;
    }

    // Foot pos +Z
    if(Joy_RB == 1 && Joy_RT == 0){
        MoveFlag[2] = true;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && plus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] -= 1;
    }

    // Foot pos -Z
    if(Joy_RB == 0 && Joy_RT == 1){
        MoveFlag[2] = true;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && minus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] -= 1;
    }

    // Foot ori +Z
    if(Joy_LB == 1 && Joy_LT == 0){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && plus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] -=1;
    }

    // Foot ori -Z
    if(Joy_LB == 0 && Joy_LT == 1){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && minus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] -=1;
    }

    // Foot ori +Y
    if(Joy_AROW_UD == +32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && plus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] -=1;
    }

    // Foot ori -Y
    if(Joy_AROW_UD == -32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && minus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] -=1;
    }

    // Foot ori +X
    if(Joy_AROW_RL == -32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && plus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] -=1;
    }

    // Foot ori -X
    if(Joy_AROW_RL == +32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && minus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] -=1;
    }

    // Mode Switching
    static unsigned int One_Hand_switching_counter = 0;
    static unsigned int Two_Hand_switching_counter = 0;
    static unsigned int One_Foot_switching_counter = 0;
    //static unsigned int Two_Foot_switching_counter = 0;

    if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 32767 && Joy_LJOG_UD == 32767){
        Two_Hand_switching_counter++;
    }
//    else if(Joy_RJOG_PUSH == 1 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == -32767 && Joy_LJOG_UD == -32767){
//        Two_Foot_switching_counter++;
//    }
    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == 32767){
        One_Hand_switching_counter++;
    }
    else if(Joy_RJOG_PUSH == 0 && Joy_LJOG_PUSH == 1 && Joy_RJOG_UD == 0 && Joy_LJOG_UD == -32767){
        One_Foot_switching_counter++;
    }
    else{
        Two_Hand_switching_counter = 0;
 //       Two_Foot_switching_counter = 0;
        One_Hand_switching_counter = 0;
        One_Foot_switching_counter = 0;
    }
    if(Two_Hand_switching_counter >= 20){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_HAND_MODE_START;
        Two_Hand_switching_counter = 0;
    }
//    else if(Two_Foot_switching_counter >= 20){
//        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_BOTH_FOOT_MODE_START;
//        Two_Foot_switching_counter = 0;
//    }
    else if(One_Foot_switching_counter >= 50){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_FOOT_MODE_START;
        One_Foot_switching_counter = 0;
    }
    else if(One_Hand_switching_counter >= 50){
        sharedCMD->COMMAND[PODO_NO].USER_COMMAND = ManualMove_AL_MANUAL_MODE_START;
        One_Hand_switching_counter = 0;
    }

    // Add POS info
        // boundary of work-space
        //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    mat3 Foot_Ori_mat = mat3(manual_quat);
    vec3 Foot_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
    vec3 AnkleToFoot_FootFrame = vec3(0,0,-0.1);
    mat3 Foot_Ori_mat_inv = Foot_Ori_mat.inverse();
    vec3 AnkleToFoot_GlobalFrame = AnkleToFoot_FootFrame*Foot_Ori_mat_inv;
    vec3 Ankle_pos = Foot_pos - AnkleToFoot_GlobalFrame;

    HipToAnkle = (Ankle_pos.x - 0)*(Ankle_pos.x - 0) + (Ankle_pos.y + 0.105)*(Ankle_pos.y + 0.105) + (Ankle_pos.z - 0.0)*(Ankle_pos.z - 0.0);
    HipToAnkle = sqrt(HipToAnkle);

    if(HipToAnkle < 0.80 && HipToAnkle > 0.20)
        Manual_OK = true;

    else
        Manual_OK = false;


    if(Manual_OK == true){
        //--------------------------pos--------------------------------------------
        WBmotion->addRFPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
        for(int i=0 ; i<3 ; i++)
            manual_pos_before[i] = manual_pos[i];

        //-------------------------Ori------------------------------------------------
        doubles RH_manual_ori(4);
        for(int i=0 ; i<4 ; i++)
            RH_manual_ori[i] = manual_quat[i];

        WBmotion->addRFOriInfo(RH_manual_ori, 0.005);

        manual_quat_before = manual_quat;

    }
    else{
        for(int i=0 ; i<3 ; i++)
            manual_pos[i] = manual_pos_before[i];

        manual_quat = manual_quat_before;
    }

}

void ManualMoveFoot_Left(void){
    // Variables
    static int counter = 0;
    double pVel[3] = {0,0,0};
    static double SpeedDW_V0[8] = {0,0,0,0,0,0,0,0};
    static int SpeedUp_time[8] = {0,0,0,0,0,0,0,0};
    static int SlowDown_time[8] = {0,0,0,0,0,0,0,0};
    static int plus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    static int minus_Dcc_time[8] = {0,0,0,0,0,0,0,0};
    double Acc = 0.5;
    double Dcc = 0.3;
    double MaxVel = 0.07;
    static bool MoveFlag[8] = {false,false,false,false,false,false,false,false};
    double HipToAnkle;
    static bool Manual_OK = true;
    static double manual_pos[3];
    static double manual_pos_before[3];
    double Accq = 0.03;
    double Dccq = 0.02;
    double MaxVel_ang = 0.0025;
    double AngSpeed[8];
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_AROW_UD_before;
    static int Joy_B_before;
    static int Joy_LB_before;
    static int Joy_LJOG_RL_before;
    static int Joy_LJOG_UD_before;
    static int Joy_LT_before;
    static int Joy_RB_before;
    static int Joy_RJOG_RL_before;
    static int Joy_RJOG_UD_before;
    static int Joy_RT_before;
    static int Joy_X_before;
    static int Joy_Y_before;
    static unsigned int same_counter = 0;
    quat manual_quat;
    static quat manual_quat_before;
    quat qVel;

    if(_isFirst_both_L == true){
       for(int i=0 ;i<3 ; i++){
            manual_pos[i] = WBmotion->pLF_3x1[i];
            manual_pos_before[i] = WBmotion->pLF_3x1[i];
        }
        manual_quat = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);
        manual_quat_before = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);

       _isFirst_both_L = false;
    }



    manual_quat = quat(WBmotion->qLF_4x1[0],WBmotion->qLF_4x1[1],WBmotion->qLF_4x1[2],WBmotion->qLF_4x1[3]);




    // Generate Position Trajectory
    for(int i=0;i<3;i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            pVel[i] = Acc*SpeedUp_time[i]*0.005;
            if(pVel[i] >= MaxVel) pVel[i] = MaxVel;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = pVel[i];
        }
        else{
            SlowDown_time[i]++;
            pVel[i] = SpeedDW_V0[i] - Dcc*SlowDown_time[i]*0.005;
            if(pVel[i] <= 0) pVel[i] = 0;
            SpeedUp_time[i] = 0;
//            RH_SpeedUP_V0[i] = pVel[i];
        }
    }

    // Generatie Angualr speed Trajectory
    for(int i=3 ; i<8 ; i++){
        if(MoveFlag[i] == true){
            SpeedUp_time[i]++;
            AngSpeed[i] = Accq*SpeedUp_time[i]*0.005;
            if(AngSpeed[i] >= MaxVel_ang) AngSpeed[i] = MaxVel_ang;
            SlowDown_time[i] = 0;
            SpeedDW_V0[i] = AngSpeed[i];
        }
        else{
            SlowDown_time[i]++;
            AngSpeed[i] = SpeedDW_V0[i] - Dccq*SlowDown_time[i]*0.005;
            if(AngSpeed[i] <= 0) AngSpeed[i] = 0;
            SpeedUp_time[i] = 0;
 //         RH_SpeedUP_V0[i] = AngSpeed[i];
        }
    }



    counter++;

 //   if(counter%100 == 0)
 //       cout<<"Joy Value: "<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]<<" time: "<<SpeedUp_time<<" pVel: "<<pVel<<" MoveFlag: "<<MoveFlag<<endl;
    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
    int Joy_X = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
    int Joy_B = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];
    int Joy_RT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4];
    int Joy_RB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5];
    int Joy_LT = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6];
    int Joy_LB = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7];
    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
    int Joy_AROW_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9];
    int Joy_RJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
    int Joy_LJOG_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
    int Joy_RJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2];
    int Joy_LJOG_UD = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3];



    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2] && Joy_X_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]
            && Joy_B_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3] && Joy_RT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[4] && Joy_RB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[5]
            && Joy_LT_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[6] && Joy_LB_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[7] && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]
            && Joy_AROW_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[9] && Joy_RJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] && Joy_LJOG_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1]
            && Joy_RJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] && Joy_LJOG_UD_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_AROW_UD_before = Joy_AROW_UD;
    Joy_B_before = Joy_B;
    Joy_LB_before = Joy_LB;
    Joy_LJOG_RL_before = Joy_LJOG_RL;
    Joy_LJOG_UD_before = Joy_LJOG_UD;
    Joy_LT_before = Joy_LT;
    Joy_RB_before = Joy_RB;
    Joy_RJOG_RL_before = Joy_RJOG_RL;
    Joy_RJOG_UD_before = Joy_RJOG_UD;
    Joy_RT_before = Joy_RT;
    Joy_X_before = Joy_X;
    Joy_Y_before = Joy_Y;

    if(same_counter >= 400){
        Joy_Y = 0;
        Joy_A = 0;
        Joy_B = 0;
        Joy_X = 0;
        Joy_RT = 0;
        Joy_RB = 0;
        Joy_LT = 0;
        Joy_LB = 0;
        Joy_AROW_RL = 0;
        Joy_AROW_UD = 0;
        Joy_RJOG_RL = 0;
        Joy_LJOG_RL = 0;
        Joy_RJOG_UD = 0;
        Joy_LJOG_UD = 0;
    }


    // Foot pos +X
    if(Joy_Y == 1 && Joy_A == 0){
        MoveFlag[0] = true;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && plus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] += pVel[0]*0.005;
        plus_Dcc_time[0] -= 1;
    }

    // Foot pos -X
    if(Joy_Y == 0 && Joy_A == 1){
        MoveFlag[0] = true;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] = pVel[0]/Dcc*200;
    }
    if(Joy_Y == 0 && Joy_A == 0 && minus_Dcc_time[0] > 0){
        MoveFlag[0] = false;
        manual_pos[0] -= pVel[0]*0.005;
        minus_Dcc_time[0] -= 1;
    }

    // Foot pos -Y
    if(Joy_B == 1 && Joy_X == 0){
        MoveFlag[1] = true;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && plus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] -= pVel[1]*0.005;
        plus_Dcc_time[1] -= 1;
    }

    // Foot pos +Y
    if(Joy_B == 0 && Joy_X == 1){
        MoveFlag[1] = true;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] = pVel[1]/Dcc*200;
    }
    if(Joy_B == 0 && Joy_X == 0 && minus_Dcc_time[1] > 0){
        MoveFlag[1] = false;
        manual_pos[1] += pVel[1]*0.005;
        minus_Dcc_time[1] -= 1;
    }

    // Foot pos +Z
    if(Joy_RB == 1 && Joy_RT == 0){
        MoveFlag[2] = true;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && plus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] += pVel[2]*0.005;
        plus_Dcc_time[2] -= 1;
    }

    // Foot pos -Z
    if(Joy_RB == 0 && Joy_RT == 1){
        MoveFlag[2] = true;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] = pVel[2]/Dcc*200;
    }
    if(Joy_RB == 0 && Joy_RT == 0 && minus_Dcc_time[2] > 0){
        MoveFlag[2] = false;
        manual_pos[2] -= pVel[2]*0.005;
        minus_Dcc_time[2] -= 1;
    }

    // Foot ori +Z
    if(Joy_LB == 1 && Joy_LT == 0){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && plus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[3] -=1;
    }

    // Foot ori -Z
    if(Joy_LB == 0 && Joy_LT == 1){
        MoveFlag[3] = true;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] = AngSpeed[3]/Dccq*200;
    }
    if(Joy_LB == 0 && Joy_LT == 0 && minus_Dcc_time[3] > 0){
        MoveFlag[3] = false;
        qVel  = qVel.rotateZ(-AngSpeed[3]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[3] -=1;
    }

    // Foot ori +Y
    if(Joy_AROW_UD == +32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && plus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[4] -=1;
    }

    // Foot ori -Y
    if(Joy_AROW_UD == -32767){
        MoveFlag[4] = true;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] = AngSpeed[4]/Dccq*200;
    }
    if(Joy_AROW_UD == 0 && minus_Dcc_time[4] > 0){
        MoveFlag[4] = false;
        qVel  = qVel.rotateY(-AngSpeed[4]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[4] -=1;
    }

    // Foot ori +X
    if(Joy_AROW_RL == -32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && plus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        plus_Dcc_time[5] -=1;
    }

    // Foot ori -X
    if(Joy_AROW_RL == +32767){
        MoveFlag[5] = true;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] = AngSpeed[5]/Dccq*200;
    }
    if(Joy_AROW_RL == 0 && minus_Dcc_time[5] > 0){
        MoveFlag[5] = false;
        qVel  = qVel.rotateX(-AngSpeed[5]);
        manual_quat = manual_quat*qVel;
        minus_Dcc_time[5] -=1;
    }

    // Add POS info
        // boundary of work-space
        //quat Hand_quat = quat(WBmotion->qRH_4x1[0],WBmotion->qRH_4x1[1],WBmotion->qRH_4x1[2],WBmotion->qRH_4x1[3]);
    mat3 Foot_Ori_mat = mat3(manual_quat);
    vec3 Foot_pos = vec3(manual_pos[0],manual_pos[1],manual_pos[2]);
    vec3 AnkleToFoot_FootFrame = vec3(0,0,-0.1);
    mat3 Foot_Ori_mat_inv = Foot_Ori_mat.inverse();
    vec3 AnkleToFoot_GlobalFrame = AnkleToFoot_FootFrame*Foot_Ori_mat_inv;
    vec3 Ankle_pos = Foot_pos - AnkleToFoot_GlobalFrame;

    HipToAnkle = (Ankle_pos.x - 0)*(Ankle_pos.x - 0) + (Ankle_pos.y - 0.105)*(Ankle_pos.y - 0.105) + (Ankle_pos.z - 0.0)*(Ankle_pos.z - 0.0);
    HipToAnkle = sqrt(HipToAnkle);

    if(HipToAnkle < 0.80 && HipToAnkle > 0.20)
        Manual_OK = true;

    else
        Manual_OK = false;


    if(Manual_OK == true){
        //--------------------------pos--------------------------------------------
        WBmotion->addLFPosInfo(manual_pos[0], manual_pos[1],manual_pos[2], 0.005);
        for(int i=0 ; i<3 ; i++)
            manual_pos_before[i] = manual_pos[i];

        //-------------------------Ori------------------------------------------------
        doubles RH_manual_ori(4);
        for(int i=0 ; i<4 ; i++)
            RH_manual_ori[i] = manual_quat[i];

        WBmotion->addLFOriInfo(RH_manual_ori, 0.005);

        manual_quat_before = manual_quat;

    }
    else{
        for(int i=0 ; i<3 ; i++)
            manual_pos[i] = manual_pos_before[i];

        manual_quat = manual_quat_before;
    }


    if(counter == 200){

//        printf("%d %d\n", Joy_RJOG_UD, Joy_LJOG_RL);
//        cout<<"both left is good"<<endl;
        counter = 0;
    }

}

void ManualDriving(void){
    static int Joy_A_before;
    static int Joy_AROW_RL_before;
    static int Joy_Y_before;
    static unsigned int same_counter=0;


//    int Joy_AROW_RL = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8];
//    int Joy_Y = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
//    int Joy_A = (int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
    int Joy_Y = GL_JOY_Y;
    int Joy_A = GL_JOY_A;
    int Joy_X = GL_JOY_X;
    int Joy_B = GL_JOY_B;
    int Joy_RT = GL_JOY_RT;
    int Joy_RB = GL_JOY_RB;
    int Joy_LT = GL_JOY_LT;
    int Joy_LB = GL_JOY_LB;
    int Joy_AROW_RL = GL_JOY_AROW_RL;
    int Joy_AROW_UD = GL_JOY_AROW_UD;
    int Joy_RJOG_RL = GL_JOY_RJOG_RL;
    int Joy_LJOG_RL = GL_JOY_LJOG_RL;
    int Joy_RJOG_UD = GL_JOY_RJOG_UD;
    int Joy_LJOG_UD = GL_JOY_LJOG_UD;
    int Joy_BACK = GL_JOY_BACK;
    int Joy_START = GL_JOY_START;
    int Joy_RJOG_PUSH = GL_JOY_RJOG_PUSH;
    int Joy_LJOG_PUSH = GL_JOY_LJOG_PUSH;

    if(Joy_Y_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1] && Joy_A_before==(int)sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2]
            && Joy_AROW_RL_before==sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8])
    {
        same_counter++;
    }
    else{
        same_counter = 0;
    }

    Joy_A_before = Joy_A;
    Joy_AROW_RL_before = Joy_AROW_RL;
    Joy_Y_before = Joy_Y;

//    if(same_counter >= 2000){
//        Joy_Y = 0;
//        Joy_A = 0;
//        Joy_AROW_RL = 0;
//    }

    if(Joy_AROW_RL == 32767){
        //RF1 Open loop Duty control
//        RBJointPWMCommand2chHR(2, 36, 1000, 0, 0x04);
        MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno,2,1000,0,0);
    }
    if(Joy_AROW_RL == -32767){
        //RF1 Open loop Duty control
//        RBJointPWMCommand2chHR(2, 36, -1000, 0, 0x04);
         MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno,2,-1000,0,0);
    }
    if(Joy_AROW_RL == 0){
//        RBJointPWMCommand2chHR(2, 36, 0, 0, 0x04);
        MCJointPWMCommand2chHR(JOINT_INFO[RWY2].canch, JOINT_INFO[RWY2].bno,2,0,0,0);
    }
    if(Joy_Y == 1){
        //---RAP
//        RBJointOLCurrentCommand2ch(0, 4, 30, 0, 0x05);
        //MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno,4,30,0,0);
    }
    if(Joy_Y == 0){
        //if(sharedSEN->ENCODER[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch].CurrentPosition < 18 ){
//           RBJointOLCurrentCommand2ch(0, 4, -50, 0, 0x05);
            //MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno,4,-50,0,0);
        }
        else{
//           RBJointOLCurrentCommand2ch(0, 4,0, 0, 0x05);
            //MCJointPWMCommand2chHR(JOINT_INFO[RAP].canch, JOINT_INFO[RAP].bno,4,0,0,0);
        }
}
//    static int counter = 0;
//    if(counter%50 == 0){
//        cout<<sharedData->CurrentPosition[MC_ID_CH_Pairs[RAP].id][MC_ID_CH_Pairs[RAP].ch]<<endl;
//        cout<<sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[8]<<endl;
//        cout<<Joy_AROW_RL<<endl;

//    }
//    counter ++;


