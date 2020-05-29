#ifndef BASICSETTING_H
#define BASICSETTING_H


#include "BasicJoint.h"
#include "UserSharedMemory.h"
//#include "RBSharedMemory.h"
//#include "RBLog.h"
//#include "JointInformation.h"


#include <iostream>
#include <libpcan.h>
#include <iostream>
#include <sys/mman.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <alchemy/task.h>

// Basic --------
extern pRBCORE_SHM_COMMAND      sharedCMD;
extern pRBCORE_SHM_REFERENCE    sharedREF;
extern pRBCORE_SHM_SENSOR       sharedSEN;
extern pUSER_SHM                userData;
extern JointControlClass        *jCon;

extern int     __IS_WORKING;
extern int     __IS_GAZEBO;
extern int     PODO_NO;

char __AL_NAME[30];

// RT task handler for control
RT_TASK rtTaskCon;
RT_TASK rtFlagCon;

// Real-time thread for control
void RBTaskThread(void *);
void RBFlagThread(void *);


using namespace std;

void CatchSignals(int _signal)
{
    switch(_signal)
    {
    case SIGHUP:     // shell termination
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        __IS_WORKING = false;
        break;
    }
    usleep(500*1000);
}

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


int RBInitialize(void){
    __IS_WORKING = true;

    char task_thread_name[30];
    char flag_thread_name[30];
    sprintf(task_thread_name, "%s_TASK", __AL_NAME);
    sprintf(flag_thread_name, "%s_FLAG", __AL_NAME);

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
        return false;
    }else{
        if(ftruncate(shmFD, sizeof(USER_SHM)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate user shared memory";
            return false;
        }else{
            userData = (pUSER_SHM)mmap(0, sizeof(USER_SHM), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(userData == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping user shared memory";
                return false;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "User shared memory creation = OK";
    // =========================================================================


    // Initialize internal joint classes =======================================
    jCon = new JointControlClass(sharedREF, sharedSEN, sharedCMD, PODO_NO);
    jCon->RefreshToCurrentReference();
    // =========================================================================


    // Create and start real-time thread =======================================
    if(rt_task_create(&rtFlagCon, flag_thread_name, 0, 95, 0) == 0){
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
            return false;
        }
    }else{
        FILE_LOG(logERROR) << cout << "Fail to create Flag real-time thread";
        return false;
    }

    if(rt_task_create(&rtTaskCon, task_thread_name, 0, 90, 0) == 0){
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
            return false;
        }
    }else{
        FILE_LOG(logERROR) << cout << "Fail to create Task real-time thread";
        return false;
    }
    // =========================================================================

    __IS_WORKING = true;
    return true;
}

int HasAnyOwnership(){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(sharedCMD->MotionOwner[MC_GetID(i)][MC_GetCH(i)] == PODO_NO)
            return true;
    }
    return false;
}

#endif // BASICSETTING_H
