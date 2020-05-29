#include "MotionChecker.h"
#include <time.h>
#include <istream>

using namespace std;

MotionChecker::MotionChecker(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, JointControlClass *_joint){

    TESTMotion = new TaskMotion(_shm_ref,_shm_sen, _shm_com, _joint);
    Motion_sTime_COM = 0;
    Motion_sTime_pLH = 0;
    Motion_sTime_qLH = 0;
    Motion_sTime_pRH = 0;
    Motion_sTime_qRH = 0;
    Motion_sTime_pLF = 0;
    Motion_sTime_qLF = 0;
    Motion_sTime_pRF = 0;
    Motion_sTime_qRF = 0;
    Motion_sTime_WST = 0;
    Motion_sTime_pPEL = 0;
    Motion_sTime_qPEL = 0;
    Motion_sTime_RElb = 0;
    Motion_sTime_LElb = 0;
    Motion_sTime = 0;


}

MotionChecker::~MotionChecker(){

}
errorInfo MotionChecker::MotionCheck(){
    struct timespec my_time1, my_time2;
    clock_gettime(CLOCK_REALTIME, &my_time1);

    TESTMotion->RefreshToCurrentReference();

    Motion_sTime = longestTime14(Motion_sTime_COM, Motion_sTime_pLH,
                                 Motion_sTime_qLH, Motion_sTime_pRH,
                                 Motion_sTime_qRH, Motion_sTime_pLF,
                                 Motion_sTime_qLF, Motion_sTime_pRF,
                                 Motion_sTime_qRF, Motion_sTime_WST,
                                 Motion_sTime_pPEL, Motion_sTime_qPEL,
                                 Motion_sTime_RElb, Motion_sTime_LElb);

    // Initialize -------------------------------------------
    errorInfo eInfo;
    eInfo.Init();

    double pLH_Error_Norm = 0;
    double pRH_Error_Norm = 0;
    unsigned int MotionTest_Iteration = 0;
    //--------------------------------------------------------

    while(1){
        TESTMotion->updateAll();
        TESTMotion->WBIK();

        //Position Error Norm Hand
        for(int i=0; i<3 ; i++){
            pLH_Error_Norm += (TESTMotion->des_pLH_3x1[i] - TESTMotion->pLH_3x1[i])*(TESTMotion->des_pLH_3x1[i] - TESTMotion->pLH_3x1[i]);
            pRH_Error_Norm += (TESTMotion->des_pRH_3x1[i] - TESTMotion->pRH_3x1[i])*(TESTMotion->des_pRH_3x1[i] - TESTMotion->pRH_3x1[i]);
        }
        pLH_Error_Norm = sqrt(pLH_Error_Norm);
        pRH_Error_Norm = sqrt(pRH_Error_Norm);

        // Orientation rpy of Hand
        quat des_qRH = quat(TESTMotion->des_qRH_4x1[0],TESTMotion->des_qRH_4x1[1],TESTMotion->des_qRH_4x1[2],TESTMotion->des_qRH_4x1[3]);
        quat des_qLH = quat(TESTMotion->des_qLH_4x1[0],TESTMotion->des_qLH_4x1[1],TESTMotion->des_qLH_4x1[2],TESTMotion->des_qLH_4x1[3]);
        quat qRH = quat(TESTMotion->qRH_4x1[0],TESTMotion->qRH_4x1[1],TESTMotion->qRH_4x1[2],TESTMotion->qRH_4x1[3]);
        quat qLH = quat(TESTMotion->qLH_4x1[0],TESTMotion->qLH_4x1[1],TESTMotion->qLH_4x1[2],TESTMotion->qLH_4x1[3]);
        rpy des_rpyRH = rpy(des_qRH);
        rpy des_rpyLH = rpy(des_qLH);
        rpy rpyRH = rpy(qRH);
        rpy rpyLH = rpy(qLH);


        MotionTest_Iteration++;

        double pError= 0.01;
        double qError = 0.01;

        if(pRH_Error_Norm > pError){
            eInfo.pRH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }
        if(pLH_Error_Norm > pError){
            eInfo.pLH_ErorrFlag = true;
            eInfo.Motion_OK_flag = false;
        }
        pLH_Error_Norm = 0;
        pRH_Error_Norm = 0;

        if(des_rpyRH.r - rpyRH.r > qError || des_rpyRH.p - rpyRH.p > qError || des_rpyRH.y - rpyRH.y > qError){
            eInfo.qRH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }
        if(des_rpyLH.r - rpyLH.r > qError || des_rpyLH.p - rpyLH.p > qError || des_rpyLH.y - rpyLH.y > qError){
            eInfo.qLH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }

        if(MotionTest_Iteration >= (unsigned int)(Motion_sTime/0.005)){
            break;
        }
    }
    MotionTest_Iteration = 0;
    TESTMotion->StopAll();

    Motion_sTime_COM = 0;
    Motion_sTime_pLH = 0;
    Motion_sTime_qLH = 0;
    Motion_sTime_pRH = 0;
    Motion_sTime_qRH = 0;
    Motion_sTime_pLF = 0;
    Motion_sTime_qLF = 0;
    Motion_sTime_pRF = 0;
    Motion_sTime_qRF = 0;
    Motion_sTime_WST = 0;
    Motion_sTime_pPEL = 0;
    Motion_sTime_qPEL = 0;
    Motion_sTime_RElb = 0;
    Motion_sTime_LElb = 0;
    Motion_sTime = 0;

    clock_gettime(CLOCK_REALTIME, &my_time2);
    if(my_time2.tv_nsec > my_time1.tv_nsec){
        eInfo.calcTime = (double)(my_time2.tv_nsec - my_time1.tv_nsec)/1e6;
    }

    return eInfo;


}






errorInfo MotionChecker::MotionCheck_UB(){
    struct timespec my_time1, my_time2;
    clock_gettime(CLOCK_REALTIME, &my_time1);

    TESTMotion->RefreshToCurrentReferenceUB();

    Motion_sTime = longestTime4(Motion_sTime_pLH, Motion_sTime_qLH, Motion_sTime_pRH, Motion_sTime_qRH);


    // Initialize----------------------------------------------
    errorInfo eInfo;
    eInfo.Init();

    double pLH_Error_Norm = 0;
    double pRH_Error_Norm = 0;
    unsigned int MotionTest_Iteration = 0;
    //----------------------------------------------------------

    while(1){
        TESTMotion->updateAll();
        TESTMotion->WBIK_UB();

        //Position Error Norm
        for(int i=0; i<3 ; i++){
            pLH_Error_Norm += (TESTMotion->des_pLH_3x1[i] - TESTMotion->pLH_3x1[i])*(TESTMotion->des_pLH_3x1[i] - TESTMotion->pLH_3x1[i]);
            pRH_Error_Norm += (TESTMotion->des_pRH_3x1[i] - TESTMotion->pRH_3x1[i])*(TESTMotion->des_pRH_3x1[i] - TESTMotion->pRH_3x1[i]);
        }
        pLH_Error_Norm = sqrt(pLH_Error_Norm);
        pRH_Error_Norm = sqrt(pRH_Error_Norm);

        quat des_qRH = quat(TESTMotion->des_qRH_4x1[0],TESTMotion->des_qRH_4x1[1],TESTMotion->des_qRH_4x1[2],TESTMotion->des_qRH_4x1[3]);
        quat des_qLH = quat(TESTMotion->des_qLH_4x1[0],TESTMotion->des_qLH_4x1[1],TESTMotion->des_qLH_4x1[2],TESTMotion->des_qLH_4x1[3]);
        quat qRH = quat(TESTMotion->qRH_4x1[0],TESTMotion->qRH_4x1[1],TESTMotion->qRH_4x1[2],TESTMotion->qRH_4x1[3]);
        quat qLH = quat(TESTMotion->qLH_4x1[0],TESTMotion->qLH_4x1[1],TESTMotion->qLH_4x1[2],TESTMotion->qLH_4x1[3]);


        rpy des_rpyRH = rpy(des_qRH);
        rpy des_rpyLH = rpy(des_qLH);
        rpy rpyRH = rpy(qRH);
        rpy rpyLH = rpy(qLH);

        double pError= 0.0001;
        double qError = 0.01;

        if(des_rpyRH.r - rpyRH.r > qError || des_rpyRH.p - rpyRH.p > qError || des_rpyRH.y - rpyRH.y > qError){
            eInfo.qRH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }
        if(des_rpyLH.r - rpyLH.r > qError || des_rpyLH.p - rpyLH.p > qError || des_rpyLH.y - rpyLH.y > qError){
            eInfo.qLH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }


        MotionTest_Iteration++;

        if(pRH_Error_Norm > pError){
            eInfo.pRH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }
        if(pLH_Error_Norm > pError){
            eInfo.qLH_ErrorFlag = true;
            eInfo.Motion_OK_flag = false;
        }



        if(MotionTest_Iteration >= (unsigned int)(Motion_sTime/0.005)){
            break;
        }
        pLH_Error_Norm = 0;
        pRH_Error_Norm = 0;

    }

    MotionTest_Iteration = 0;
    TESTMotion->StopAll();

    Motion_sTime_COM = 0;
    Motion_sTime_pLH = 0;
    Motion_sTime_qLH = 0;
    Motion_sTime_pRH = 0;
    Motion_sTime_qRH = 0;
    Motion_sTime_pLF = 0;
    Motion_sTime_qLF = 0;
    Motion_sTime_pRF = 0;
    Motion_sTime_qRF = 0;
    Motion_sTime_WST = 0;
    Motion_sTime_pPEL = 0;
    Motion_sTime_qPEL = 0;
    Motion_sTime_RElb = 0;
    Motion_sTime_LElb = 0;
    Motion_sTime = 0;


    clock_gettime(CLOCK_REALTIME, &my_time2);
    if(my_time2.tv_nsec > my_time1.tv_nsec){
        eInfo.calcTime = (double)(my_time2.tv_nsec - my_time1.tv_nsec)/1e6;
    }

    return eInfo;

}

double MotionChecker::longestTime4(double a, double b, double c, double d){
    double temp;

    if(a >= b) temp = a;
    else temp = b;

    if(temp >= c);
    else temp = c;

    if(temp >= d);
    else temp = d;

    return temp;
}

double MotionChecker::longestTime14(double t1, double t2, double t3, double t4, double t5, double t6, double t7, double t8, double t9, double t10, double t11, double t12, double t13, double t14){
    double temp;
    if(t1 >= t2) temp = t1;
    else temp = t2;
    if(temp >= t3);
    else temp = t3;
    if(temp >= t4);
    else temp = t4;
    if(temp >= t5);
    else temp = t5;
    if(temp >= t6);
    else temp = t6;
    if(temp >= t7);
    else temp = t7;
    if(temp >= t8);
    else temp = t8;
    if(temp >= t9);
    else temp = t9;
    if(temp >= t10);
    else temp = t10;
    if(temp >= t11);
    else temp = t11;
    if(temp >= t12);
    else temp = t12;
    if(temp >= t13);
    else temp = t13;
    if(temp >= t14);
    else temp = t14;
    return temp;
}


void MotionChecker::ResetGlobalCoord(int RF_OR_LF_OR_PC){
    TESTMotion->ResetGlobalCoord(RF_OR_LF_OR_PC);
}

void MotionChecker::addCOMInfo(double _xCOM, double _yCOM, double _sTime){
    TESTMotion->addCOMInfo(_xCOM, _yCOM, _sTime);
    Motion_sTime_COM += _sTime;
}
void MotionChecker::addCOMInfo(double _sTime){
    TESTMotion->addCOMInfo(_sTime);
    Motion_sTime_COM += _sTime;
}

void MotionChecker::addRFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime){
    TESTMotion->addRFPosInfo(_xLeg, _yLeg, _zLeg, _sTime);
    Motion_sTime_pRF += _sTime;
}
void MotionChecker::addRFPosInfo(double _sTime){
    TESTMotion->addRFPosInfo(_sTime);
    Motion_sTime_pRF += _sTime;
}

void MotionChecker::addLFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime){
    TESTMotion->addLFPosInfo(_xLeg, _yLeg, _zLeg, _sTime);
    Motion_sTime_pLF += _sTime;
}
void MotionChecker::addLFPosInfo(double _sTime){
    TESTMotion->addLFPosInfo(_sTime);
    Motion_sTime_pLF += _sTime;
}

void MotionChecker::addRHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime){
    TESTMotion->addRHPosInfo(_xArm, _yArm, _zArm, _sTime);
    Motion_sTime_pRH += _sTime;
}
void MotionChecker::addRHPosInfo(double _sTime){
    TESTMotion->addRHPosInfo(_sTime);
    Motion_sTime_pRH += _sTime;
}

void MotionChecker::addLHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime){
    TESTMotion->addLHPosInfo(_xArm, _yArm, _zArm, _sTime);
    Motion_sTime_pLH += _sTime;
}
void MotionChecker::addLHPosInfo(double _sTime){
    TESTMotion->addLHPosInfo(_sTime);
    Motion_sTime_pLH += _sTime;
}

void MotionChecker::addWSTPosInfo(double _wst, double _sTime){
    TESTMotion->addWSTPosInfo(_wst, _sTime);
    Motion_sTime_WST += _sTime;
}
void MotionChecker::addWSTPosInfo(double _sTime){
    TESTMotion->addWSTPosInfo(_sTime);
    Motion_sTime_WST += _sTime;
}

void MotionChecker::addPELPosInfo(double _pelz, double _sTime){
    TESTMotion->addPELPosInfo(_pelz, _sTime);
    Motion_sTime_pPEL += _sTime;
}
void MotionChecker::addPELPosInfo(double _sTime){
    TESTMotion->addPELPosInfo(_sTime);
    Motion_sTime_pPEL += _sTime;
}

void MotionChecker::addPELOriInfo(doubles _quat, double _sTime){
    TESTMotion->addPELOriInfo(_quat,_sTime);
    Motion_sTime_qPEL += _sTime;
}
void MotionChecker::addPELOriInfo(double _sTime){
    TESTMotion->addPELOriInfo(_sTime);
    Motion_sTime_qPEL += _sTime;
}

void MotionChecker::addRHOriInfo(doubles _quat, double _sTime){
    TESTMotion->addRHOriInfo(_quat, _sTime);
    Motion_sTime_qRH += _sTime;
}
void MotionChecker::addRHOriInfo(double _sTime){
    TESTMotion->addRHOriInfo(_sTime);
    Motion_sTime_qRH += _sTime;
}

void MotionChecker::addLHOriInfo(doubles _quat, double _sTime){
    TESTMotion->addLHOriInfo(_quat, _sTime);
    Motion_sTime_qLH += _sTime;
}
void MotionChecker::addLHOriInfo(double _sTime){
    TESTMotion->addLHOriInfo(_sTime);
    Motion_sTime_qLH += _sTime;
}
void MotionChecker::addRFOriInfo(doubles _quat, double _sTime){
    TESTMotion->addRFOriInfo(_quat, _sTime);
    Motion_sTime_qRF += _sTime;
}
void MotionChecker::addRFOriInfo(double _sTime){
    TESTMotion->addRFOriInfo(_sTime);
    Motion_sTime_qRF += _sTime;
}
void MotionChecker::addLFOriInfo(doubles _quat, double _sTime){
    TESTMotion->addLFOriInfo(_quat, _sTime);
    Motion_sTime_qLF += _sTime;
}
void MotionChecker::addLFOriInfo(double _sTime){
    TESTMotion->addLFOriInfo(_sTime);
    Motion_sTime_qLF += _sTime;
}

void MotionChecker::addRElbPosInfo(double _angle, double _sTime){
    TESTMotion->addRElbPosInfo(_angle, _sTime);
    Motion_sTime_RElb += _sTime;
}
void MotionChecker::addRElbPosInfo(double _sTime){
    TESTMotion->addRElbPosInfo(_sTime);
    Motion_sTime_RElb += _sTime;
}

void MotionChecker::addLElbPosInfo(double _angle, double _sTime){
    TESTMotion->addLElbPosInfo(_angle, _sTime);
    Motion_sTime_LElb += _sTime;
}
void MotionChecker::addLElbPosInfo(double _sTime){
    TESTMotion->addLElbPosInfo(_sTime);
    Motion_sTime_LElb += _sTime;
}

