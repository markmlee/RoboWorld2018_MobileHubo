#include "RBIMUSensor.h"

RBIMUSensor::RBIMUSensor()
{
}

void RBIMUSensor::RBIMU_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA1);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA2);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
    canHandler->RBCAN_AddMailBox(ID_RCV_PARA);
    //canHandler->RBCAN_AddMailBox(ID_RCV_STAT);
}

void RBIMUSensor::RBBoard_GetDBData(DB_IMU db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    SENSOR_ID       = db.SENSOR_ID;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    SENSOR_TYPE     = db.SENSOR_TYPE;
    ID_RCV_DATA1    = db.ID_RCV_DATA1;
    ID_RCV_DATA2    = db.ID_RCV_DATA2;
    ID_RCV_STAT     = db.ID_RCV_STAT;
    ID_RCV_INFO     = db.ID_RCV_INFO;
    ID_RCV_PARA     = db.ID_RCV_PARA;
}

int RBIMUSensor::RBBoard_CANCheck(int _canr){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x01;		// command
    mb.data[2] = _canr;	// CAN communication rate(msec)
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    if( canHandler->RBCAN_WriteData(mb) == true ){
        usleep(15*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_INFO;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int RBIMUSensor::RBBoard_RequestStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x02;		// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBBoard_LoadDefaultValue(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xFA;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestNulling(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x81;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestCalibration(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x82;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestParameter(unsigned char _prf){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x24;			// command
    mb.data[2] = _prf;		    // parameter request
    // _prf = 0x01 : ACC_X_GAIN  ACC_Y_GAIN  ACC_Z_GAIN
    // _prf = 0x02 : ACC_X_BIAS  ACC_Y_BIAS  ACC_Z_BIAS
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_SetBoardNumber(int _newbno){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA8;						// command
    mb.data[2] = _newbno;					// new board number
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestData(int _type){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = SENSOR_ID;				// Sensor board no. if _sbno = 0xFF : all sensor boards ??
    mb.data[1] = _type;              // Request Angle and Rate data
    mb.data[2] = 0x01;              // Extra value to indicate an IMU sensor, otherwise this CAN message would be
                                    // the same as one of the FT request data messages.
    mb.dlc = 3;
    mb.id = SENSOR_REQUEST_CANID;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBIMUSensor::RBIMU_ReadData(void){
    RBCAN_MB mb;
    RBCAN_MB mb2;

    // Read IMU Data1
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_DATA1;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA)
    {
//        ROLL = (double)((short)((mb.data[1]<<8)|mb.data[0]))/100.0f + ROLL_OFFSET;
//        PITCH = (double)((short)((mb.data[3]<<8)|mb.data[2]))/100.0f + -1.0;//PITCH_OFFSET;
//        ROLL_VEL = (double)((short)((mb.data[5]<<8)|mb.data[4]))/100.0f;
//        PITCH_VEL = (double)((short)((mb.data[7]<<8)|mb.data[6]))/100.0f;
        //return true;
        ACC_X = (double)((short)((mb.data[1]<<8)|mb.data[0]))/100.0f;
        ACC_Y = (double)((short)((mb.data[3]<<8)|mb.data[2]))/100.0f;
        ACC_Z = (double)((short)((mb.data[5]<<8)|mb.data[4]))/100.0f;
        //TEMP = (double)((short)((mb.data[7]<<8)|mb.data[6]))/100.0f;

        mb.status = RBCAN_NODATA;
    }

//    // Read IMU Data2
//    mb2.channel = CAN_CHANNEL;
//    mb2.id = ID_RCV_DATA2;
//    canHandler->RBCAN_ReadData(&mb2);
//    if(mb2.status != RBCAN_NODATA){
//        ACC_X = (double)((short)((mb2.data[1]<<8)|mb2.data[0]))/100.0f + ROLL_OFFSET;//FOG Edit
//        ACC_Y = (double)((short)((mb2.data[3]<<8)|mb2.data[2]))/100.0f + -1.0;//PITCH_OFFSET;//FOG Edit
//        ACC_Z = (double)((short)((mb2.data[5]<<8)|mb2.data[4]))/100.0f;
//        YAW_VEL = (double)((short)((mb2.data[7]<<8)|mb2.data[6]))/100.0f;
//        //return true;//FOG Edit
//        mb.status = RBCAN_NODATA;
//    }
    return true;
}
