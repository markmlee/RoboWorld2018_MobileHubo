#include "joint.h"


unsigned char JointClass::SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode){
	if(_msTime <= 0) { std::cout << ">>> Goal time must be grater than zero..!!(setMoveJointAngle)" << std::endl; return ERR_GOAL_TIME; }

	MoveFlag = DISABLE;
	switch(_mode)
	{
	case MOVE_RELATIVE:	// relative mode
		RefAngleToGo = RefAngleCurrent + _angle;
		break;
	case MOVE_ABSOLUTE:	// absolute mode
		RefAngleToGo = _angle;
		break;
	default:
		std::cout << ">>> Wrong reference mode(RBsetMoveJointAngle)..!!" << std::endl;
		return ERR_WRONG_MODE;
		break;
	}
	RefAngleInitial = RefAngleCurrent;
	RefAngleDelta = RefAngleToGo - RefAngleCurrent;
	CurrentTimeCount = 0;

	GoalTimeCount = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
	MoveFlag = ENABLE;
	return ERR_OK;
}

unsigned char JointClass::MoveJoint(){
	// for reference generator
	if(MoveFlag == ENABLE){
		CurrentTimeCount++;
		if(GoalTimeCount <= CurrentTimeCount){
			GoalTimeCount = CurrentTimeCount = 0;
			RefAngleCurrent = RefAngleToGo;
			MoveFlag = DISABLE;
			return MOVE_DONE;
		}else{
			RefAngleCurrent = RefAngleInitial+RefAngleDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount*(double)CurrentTimeCount));
		}
	}
	return STILL_MOVING;
}


JointControlClass::JointControlClass(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, int _podoNum){
    Shm_ref = _shm_ref;
    Shm_sen = _shm_sen;
    Shm_com = _shm_com;
	PODONum = _podoNum;
	Joints = JointVector(NO_OF_JOINTS);
	for(int i=0; i<NO_OF_JOINTS; i++)
		Joints[i] = new JointClass(MC_ID_CH_Pairs[i].id, MC_ID_CH_Pairs[i].ch);
}

void JointControlClass::RefreshToCurrentReference(){
	int mcId, mcCh;
    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = Joints[i]->GetId();
        mcCh = Joints[i]->GetCh();
        Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentReference);
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
    }
}

void JointControlClass::SetMotionOwner(const int _jointNum){
	int mcId = Joints[_jointNum]->GetId();
	int mcCh = Joints[_jointNum]->GetCh();
    Shm_com->MotionOwner[mcId][mcCh] = PODONum;
}

void JointControlClass::SetAllMotionOwner(){
//    for(int i=0; i<(NO_OF_JOINTS-2); i++)
//		SetMotionOwner(i);

//    for(int i=0; i<=LHAND; i++)
//        SetMotionOwner(i);
//    for(int i=RF1; i<=LF4; i++)
//        SetMotionOwner(i);

    for(int i=0; i<NO_OF_JOINTS; i++)
        SetMotionOwner(i);
}

unsigned char JointControlClass::SetMoveJoint(const int _jointNum, const double _angle, const double _msTime, const unsigned int _mode){
	return Joints[_jointNum]->SetMoveJoint(_angle, _msTime, _mode);
}
unsigned char JointControlClass::MoveJoint(const int _jointNum){
	return Joints[_jointNum]->MoveJoint();
}
void JointControlClass::MoveAllJoint(){
    int mcId, mcCh;
    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = Joints[i]->GetId();
        mcCh = Joints[i]->GetCh();
        MoveJoint(i);
    }
}

void JointControlClass::JointUpdate(){
    int mcId, mcCh;
    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = Joints[i]->GetId();
        mcCh = Joints[i]->GetCh();
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
        if(Shm_com->MotionOwner[mcId][mcCh] == PODONum)
            Shm_com->ACK_SIGNAL[mcId][mcCh] = true;
    }
    Shm_com->SYNC_SIGNAL[PODONum] = false;
}


int JointControlClass::GetJointNumber(const int _mcId, const int _mcCh){
	int res = -1;
	for(int i=0; i<NO_OF_JOINTS; i++){
		if(Joints[i]->GetId() == _mcId && Joints[i]->GetCh() == _mcCh)
			return i;
	}
	return res;
}

int JointControlClass::RefreshToCurrentEncoder(){
    if(Shm_sen->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition < 0.0001 && Shm_sen->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition > -0.0001){
        return 0; //Encoder is off
    }
    else{
        int mcId, mcCh;

        //--Left ARM
        for(int i=LSP; i<=LWP; i++){
            mcId = Joints[i]->GetId();
            mcCh = Joints[i]->GetCh();

            Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentPosition);
            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
        }

        mcId = Joints[LWY2]->GetId();
        mcCh = Joints[LWY2]->GetCh();

        Joints[LWY2]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentPosition);
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[LWY2]->GetRefAngleCurrent();

        //--Right ARM
        for(int i=RSP; i<=RWP; i++){
            mcId = Joints[i]->GetId();
            mcCh = Joints[i]->GetCh();

            Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentPosition);
            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
        }

        mcId = Joints[RWY2]->GetId();
        mcCh = Joints[RWY2]->GetCh();

        Joints[RWY2]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentPosition);
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[RWY2]->GetRefAngleCurrent();



    }
    return 1; // refresh done!
}

int JointControlClass::RefreshToCurrentEncoder_RAP_RF1(){
    if(fabs(Shm_sen->ENCODER[MC_ID_CH_Pairs[RKN].id][MC_ID_CH_Pairs[RKN].ch].CurrentPosition) < 0.00001){
        return 0; //Encoder is off
    }
    else{
        int mcId, mcCh;
        //--RAP
       // mcId = Joints[RAP]->GetId();
        //mcCh = Joints[RAP]->GetCh();
        //Joints[RAP]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentPosition);
       // Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[RAP]->GetRefAngleCurrent();

        //--RF1
        mcId = Joints[RWY2]->GetId();
        mcCh = Joints[RWY2]->GetCh();

        Joints[RWY2]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentPosition);
        Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[RWY2]->GetRefAngleCurrent();
    }
}

