#ifndef MOTIONCHECKER_H
#define MOTIONCHECKER_H

#include "taskmotion.h"

struct errorInfo
{
    bool pRH_ErrorFlag;
    bool qRH_ErrorFlag;
    bool pLH_ErorrFlag;
    bool qLH_ErrorFlag;
    bool Motion_OK_flag;
    double calcTime;

    void Init(){
        pRH_ErrorFlag = false;
        qRH_ErrorFlag = false;
        pLH_ErorrFlag = false;
        qLH_ErrorFlag = false;
        Motion_OK_flag = true;
        calcTime = 0;
    }
};

class MotionChecker
{

public:
    explicit MotionChecker(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, JointControlClass *_joint);
    ~MotionChecker();

    TaskMotion *TESTMotion;

    errorInfo MotionCheck_UB();
    errorInfo MotionCheck();

//private:
public:
    double Motion_sTime_COM;
    double Motion_sTime_pLH;
    double Motion_sTime_qLH;
    double Motion_sTime_pRH;
    double Motion_sTime_qRH;
    double Motion_sTime_pLF;
    double Motion_sTime_qLF;
    double Motion_sTime_pRF;
    double Motion_sTime_qRF;
    double Motion_sTime_WST;
    double Motion_sTime_pPEL;
    double Motion_sTime_qPEL;
    double Motion_sTime_RElb;
    double Motion_sTime_LElb;
    double Motion_sTime;

private:
    double longestTime4(double a, double b, double c, double d);
    double longestTime14(double t1, double t2, double t3, double t4, double t5, double t6, double t7, double t8, double t9, double t10, double t11, double t12, double t13, double t14);

public:

    void    ResetGlobalCoord(int RF_OR_LF_OR_PC);

    void addCOMInfo(double _xCOM, double _yCOM, double _sTime);
    void addCOMInfo(double _sTime);

    void addRFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime);
    void addRFPosInfo(double _sTime);

    void addLFPosInfo(double _xLeg, double _yLeg, double _zLeg, double _sTime);
    void addLFPosInfo(double _sTime);

    void addRHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime);
    void addRHPosInfo(double _sTime);

    void addLHPosInfo(double _xArm, double _yArm, double _zArm, double _sTime);
    void addLHPosInfo(double _sTime);

    void addWSTPosInfo(double _wst, double _sTime);
    void addWSTPosInfo(double _sTime);

    void addPELPosInfo(double _pelz, double _sTime);
    void addPELPosInfo(double _sTime);

    void addPELOriInfo(doubles _quat, double _sTime);
    void addPELOriInfo(double _sTime);
    void addRFOriInfo(doubles _quat, double _sTime);
    void addRFOriInfo(double _sTime);
    void addLFOriInfo(doubles _quat, double _sTime);
    void addLFOriInfo(double _sTime);
    void addRHOriInfo(doubles _quat, double _sTime);
    void addRHOriInfo(double _sTime);
    void addLHOriInfo(doubles _quat, double _sTime);
    void addLHOriInfo(double _sTime);

    void addRElbPosInfo(double _angle, double _sTime);
    void addRElbPosInfo(double _sTime);
    void addLElbPosInfo(double _angle, double _sTime);
    void addLElbPosInfo(double _sTime);



};

#endif // MOTIONCHECKER_H
