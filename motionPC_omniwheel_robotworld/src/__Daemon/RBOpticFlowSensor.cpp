#include "RBOpticFlowSensor.h"

using namespace std;
RBOpticFlowSensor::RBOpticFlowSensor()
{
}

void RBOpticFlowSensor::RBOF_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
}

void RBOpticFlowSensor::RBBoard_GetDBData(DB_OF db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    SENSOR_ID       = db.SENSOR_ID;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    ID_RCV_DATA     = db.ID_RCV_DATA;
    ID_RCV_INFO     = db.ID_RCV_INFO;
}

int RBOpticFlowSensor::RBBoard_CANCheck(int _canr){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x01;		// command
    mb.data[2] = _canr;	// CAN communication rate(msec)
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(15*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_INFO;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMOF: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMOF: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int RBOpticFlowSensor::RBOF_RequestValue(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = SENSOR_ID;
    mb.dlc = 1;
    mb.id = 0x03;
    return canHandler->RBCAN_WriteData(mb);
}

int RBOpticFlowSensor::RBOF_ReadValue(){
    int ret = true;
    RBCAN_MB mb;

    // Read X, Y
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_DATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        //FILE_LOG(logINFO) << "Accum Read..";
        AccumX = (int)((mb.data[3]<<24) | (mb.data[2]<<16) | (mb.data[1]<<8) | (mb.data[0]));
        AccumY = (int)((mb.data[7]<<24) | (mb.data[6]<<16) | (mb.data[5]<<8) | (mb.data[4]));
    }else ret = false;
    return ret;
}

int RBOpticFlowSensor::RBOF_ResetValue(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;
    mb.data[1] = 0x81;
    mb.dlc = 2;
    mb.id = 0x01;
    return canHandler->RBCAN_WriteData(mb);
}

int RBOpticFlowSensor::RBOF_LampOnOff(int _onoff){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;
    mb.data[1] = 0x83;
    mb.data[2] = _onoff;     //on:1, off:0
    mb.dlc = 3;
    mb.id = 0x01;
    return canHandler->RBCAN_WriteData(mb);
}


//=====================================================


OpticalDisplacement::OpticalDisplacement(){

}

void OpticalDisplacement::OD_MeasureState(){

    // ================================================== //
    // Gyro Sensor...
    // ================================================== //
    //Mea_th = Mea_th + sharedData->IMURollVel[0]*0.005*3.141592/180;
    Mea_th = Mea_th + sharedSEN->IMU[0].YawVel*0.005*3.141592/180;
    Mea_pit = Mea_pit + sharedSEN->IMU[0].PitchVel*0.005*3.141592/180;

    // ================================================== //
    // Optical Flow Sensor...
    // ================================================== //
    int Opt1Xval = sharedSEN->OF.AccumX[0];
    int Opt1Yval = sharedSEN->OF.AccumY[0]; // right leg
    int Opt2Xval = sharedSEN->OF.AccumX[1]; // left leg
    int Opt2Yval = sharedSEN->OF.AccumY[1];


    RsideOptOld = RsideOpt; // left leg
    LsideOptOld = LsideOpt; // right leg
    RsideOpt = Opt2Yval ; // mm
    LsideOpt = Opt1Yval ;

    //printf("RR %.1f    LL   %.1f \n", RsideOpt, LsideOpt);

    double deltaRside, deltaLside;
    deltaRside = (RsideOpt - RsideOptOld)* 1000.0 / (-3360.6)/1000.0;
    deltaLside = (LsideOpt - LsideOptOld)* 1000.0 / (-3275.5)/1000.0;

    double widthOpt = 220.0/2.0/1000.0;
    double offsetOpt = -16.0/1000.0;

    double bigRadius=0;
    double bigAlpha=0;

    double Ropt_next_localX=0;
    double Ropt_next_localY=0;

    double Lopt_next_localX=0;
    double Lopt_next_localY=0;

    double Robot_next_localX=0;
    double Robot_next_localY=0;

    if(deltaRside*deltaLside >= 0){
        if(deltaRside == deltaLside){
            Ropt_next_localX = deltaRside;
            Lopt_next_localX = deltaLside;
        }
        else{
            if(fabs(deltaRside) < fabs(deltaLside)){
                bigRadius = (-2*widthOpt*deltaRside)/(deltaRside - deltaLside);
                bigAlpha = (deltaRside-deltaLside)/(-2*widthOpt);

                Ropt_next_localX = bigRadius*sin(bigAlpha);
                Ropt_next_localY = -widthOpt - bigRadius*(1-cos(bigAlpha));

                Lopt_next_localX = (bigRadius + 2*widthOpt)*sin(bigAlpha);
                Lopt_next_localY = widthOpt - (bigRadius+2*widthOpt)*(1-cos(bigAlpha));
            }

            else{
                bigRadius = (-2*widthOpt*deltaLside)/(deltaLside - deltaRside);
                bigAlpha = (deltaLside-deltaRside)/(-2*widthOpt);

                Ropt_next_localX = (bigRadius + 2*widthOpt)*sin(bigAlpha);
                Ropt_next_localY = -widthOpt + (bigRadius+2*widthOpt)*(1-cos(bigAlpha));

                Lopt_next_localX = bigRadius*sin(bigAlpha);
                Lopt_next_localY = widthOpt + bigRadius*(1-cos(bigAlpha));
            }
        }


    }
    else{
        double smallAlpha;
        double smallRadiusR;
        double smallRadiusL;

        smallRadiusR = 2*widthOpt*fabs(deltaRside)/(fabs(deltaRside)+fabs(deltaLside));
        smallRadiusL = 2*widthOpt*fabs(deltaLside)/(fabs(deltaRside)+fabs(deltaLside));

        smallAlpha = fabs(acos((2*smallRadiusR*smallRadiusR - deltaRside*deltaRside)/(2*smallRadiusR*smallRadiusR)));


        if(deltaRside > 0){
            Ropt_next_localX = smallRadiusR*sin(smallAlpha);
            Ropt_next_localY = smallRadiusR*(1-cos(smallAlpha));

            Lopt_next_localX = -smallRadiusL*sin(smallAlpha);
            Lopt_next_localY = smallRadiusL*(cos(smallAlpha)-1);
        }
        else{
            Ropt_next_localX = -smallRadiusR*sin(smallAlpha);
            Ropt_next_localY = smallRadiusR*(cos(smallAlpha)-1);

            Lopt_next_localX = smallRadiusL*sin(smallAlpha);
            Lopt_next_localY = smallRadiusL*(1-cos(smallAlpha));
        }
    }

    Robot_next_localX = (Ropt_next_localX + Lopt_next_localX)/2.0 ;
    Robot_next_localY = (Ropt_next_localY + Lopt_next_localY)/2.0 ;


//        Mea_x = Mea_x + cos(Mea_th)*Robot_next_localX - sin(Mea_th)*Robot_next_localY;
//        Mea_y = Mea_y + sin(Mea_th)*Robot_next_localX + cos(Mea_th)*Robot_next_localY;





    double Robot_next_localX_ortho=0;
    double Robot_next_localY_ortho=0;
    {
        // Y val
        RorthoOptOld = RorthoOpt; // left leg
        LorthoOptOld = LorthoOpt; // right leg
        RorthoOpt = Opt2Xval ; // mm
        LorthoOpt = Opt1Xval ;

        double deltaRortho, deltaLortho;
        deltaRortho = (RorthoOpt - RorthoOptOld)* 1000.0 / (3360.6)/1000.0;
        deltaLortho = (LorthoOpt - LorthoOptOld)* 1000.0 / (3275.5)/1000.0;



        Robot_next_localX_ortho = 0;
        Robot_next_localY_ortho = (deltaRortho + deltaLortho)/2.0;
    }

    Mea_x = Mea_x + cos(Mea_th)*Robot_next_localX - sin(Mea_th)*Robot_next_localY + cos(Mea_th)*Robot_next_localX_ortho - sin(Mea_th)*Robot_next_localY_ortho;
    Mea_y = Mea_y + sin(Mea_th)*Robot_next_localX + cos(Mea_th)*Robot_next_localY + sin(Mea_th)*Robot_next_localX_ortho + cos(Mea_th)*Robot_next_localY_ortho;


    if(isnan(Mea_x)){
        Mea_x = 0;
    }
    if(isnan(Mea_y)){
        Mea_y = 0;
    }
    if(isnan(Mea_th)){
        Mea_th = 0;
    }
    if(isnan(Mea_pit)){
        Mea_pit = 0;
    }

    sharedSEN->OF.Xmea = (float)(Mea_x);
    sharedSEN->OF.Ymea = (float)(Mea_y);
    sharedSEN->OF.thmea = (float)(Mea_th);
    sharedSEN->OF.Pitmea = (float)(Mea_pit);
}

void OpticalDisplacement::OD_Zero(){
    Mea_x = 0;
    Mea_y = 0;
    Mea_th = 0;
    Mea_pit = 0;

    int Opt1Yval = sharedSEN->OF.AccumX[0];
    int Opt2Yval = sharedSEN->OF.AccumX[1]; // left leg

    sharedSEN->OF.Xmea = 0;
    sharedSEN->OF.Ymea = 0;
    sharedSEN->OF.thmea = 0;
    sharedSEN->OF.Pitmea = 0;

    RsideOpt = Opt2Yval ; // mm
    LsideOpt = Opt1Yval ;

    RsideOptOld = RsideOpt+1; // left leg
    LsideOptOld = LsideOpt+1; // right leg

    RorthoOptOld = 0; // left leg
    LorthoOptOld = 0; // right leg
    RorthoOpt = 0; // mm
    LorthoOpt = 0;

    OD_MeasureState();
}

