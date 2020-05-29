#include "RBFTSensor.h"

RBFTSensor::RBFTSensor()
{
    CutOffFeq = 3.0f;
    SFRoll = 1.0f/0.0255f;
    SFPitch = 1.0f/0.0255f;
}

void RBFTSensor::RBFT_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA1);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA2);
    canHandler->RBCAN_AddMailBox(ID_RCV_ACC);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
    canHandler->RBCAN_AddMailBox(ID_RCV_PARA);
    //canHandler->RBCAN_AddMailBox(ID_RCV_STAT);
}

void RBFTSensor::RBBoard_GetDBData(DB_FT db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    SENSOR_ID       = db.SENSOR_ID;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    SENSOR_TYPE     = db.SENSOR_TYPE;
    ID_RCV_DATA1    = db.ID_RCV_DATA1;
    ID_RCV_DATA2    = db.ID_RCV_DATA2;
    ID_RCV_ACC      = db.ID_RCV_ACC;
    ID_RCV_STAT     = db.ID_RCV_STAT;
    ID_RCV_INFO     = db.ID_RCV_INFO;
    ID_RCV_PARA     = db.ID_RCV_PARA;
}

int RBFTSensor::RBBoard_CANCheck(int _canr){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x01;		// and
    mb.data[2] = _canr;	// CAN unication rate(msec)
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(15*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_INFO;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMFT: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMFT: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int RBFTSensor::RBBoard_RequestStatus(void){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x02;		// and
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBBoard_LoadDefaultValue(void){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xFA;			// and
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_Nulling(int _mode){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x81;			// and
    mb.data[2] = _mode;		// NULL mode
    // _mode = 0x00 : FT sensor
    // _mode = 0x04 : Inclinometers in FT sensor
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetCoefficient0(int _coeff1, int _coeff2, int _coeff3){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA0;						// and
    mb.data[2] = (_coeff1 & 0xFF);			// scale factor 00
    mb.data[3] = ((_coeff1>>8) & (0xFF));	// scale factor 00
    mb.data[4] = (_coeff2 & 0xFF);			// scale factor 01
    mb.data[5] = ((_coeff2>>8) & (0xFF));	// scale factor 01
    mb.data[6] = (_coeff3 & 0xFF);			// scale factor 02
    mb.data[7] = ((_coeff3>>8) & (0xFF));	// scale factor 02
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetCoefficient1(int _coeff1, int _coeff2, int _coeff3){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA1;						// and
    mb.data[2] = (_coeff1 & 0xFF);			// scale factor 10
    mb.data[3] = ((_coeff1>>8) & (0xFF));	// scale factor 10
    mb.data[4] = (_coeff2 & 0xFF);			// scale factor 11
    mb.data[5] = ((_coeff2>>8) & (0xFF));	// scale factor 11
    mb.data[6] = (_coeff3 & 0xFF);			// scale factor 12
    mb.data[7] = ((_coeff3>>8) & (0xFF));	// scale factor 12
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetCoefficient2(int _coeff1, int _coeff2, int _coeff3){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA2;						// and
    mb.data[2] = (_coeff1 & 0xFF);			// scale factor 20
    mb.data[3] = ((_coeff1>>8) & (0xFF));	// scale factor 20
    mb.data[4] = (_coeff2 & 0xFF);			// scale factor 21
    mb.data[5] = ((_coeff2>>8) & (0xFF));	// scale factor 21
    mb.data[6] = (_coeff3 & 0xFF);			// scale factor 22
    mb.data[7] = ((_coeff3>>8) & (0xFF));	// scale factor 22
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetInclinometerSF(int _sf1, int _sf2, int _sf3){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA5;						// and
    mb.data[2] = (_sf1 & 0xFF);			// scale factor 20
    mb.data[3] = ((_sf1>>8) & (0xFF));		// scale factor 20
    mb.data[4] = (_sf2 & 0xFF);			// scale factor 21
    mb.data[5] = ((_sf2>>8) & (0xFF));		// scale factor 21
    mb.data[6] = (_sf3 & 0xFF);			// scale factor 22
    mb.data[7] = ((_sf3>>8) & (0xFF));		// scale factor 22
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_SetBoardNumberAndFilterFrequency(int _newbno, int _freq){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA8;						// and
    mb.data[2] = _newbno;					// new board number
    mb.data[4] = (_freq & 0xFF);	    	// low-pass filter(1st order) cut-off frequency
    mb.data[5] = ((_freq>>8) & (0xFF));	// low-pass filter(1st order) cut-off frequency
    // cut-off frequency = _freq/10
    mb.dlc = 8;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_Initialize(void){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xFA;						// and
    mb.data[2] = 0xAA;						// and
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_RequestCoefficient(int _para){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0x24;						// and
    mb.data[2] = _para;					// parameter request
    // _para = 0x01 : SF00 SF01 SF02 FREQ
    // _para = 0x02 : SF10 SF11 SF12
    // _para = 0x03 : SF20 SF21 SF22
    // _para = 0x04 : SFI0 SFI1 SFI2
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBFTSensor::RBFT_RequestData(int _mode){
    if(CAN_CHANNEL < 0)
        return false;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = SENSOR_ID;					// sensor board no. if _sbno = 0xFF : all sensor boards
    mb.data[1] = _mode;					// requested data mode
    // _mode = 0x00 : request FT and tilt in digit
    // _mode = 0x02 : request FT and tilt with scale
    // _mode = 0x03 : request FT with scale and tilt in digit
    // _mode = 0x04 : request FT in digit and tilt with scale
    // _mode = 0x11 : request FT in digit
    // _mode = 0x12 : request FT with scale
    // _mode = 0x21 : request tilt in digit
    // _mode = 0x22 : request tilt with scale
    // _mode = 0x13 : request gyro and temperature
    mb.dlc = 2;
    mb.id = SENSOR_REQUEST_CANID;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}


int RBFTSensor::RBFT_ReadData(void){
    if(CAN_CHANNEL < 0)
        return false;

    int ret = true;
    RBCAN_MB mb;

    // Read MX MY FZ
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_DATA1;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        //printf("FT sensor data in!!!");
        MX = -(double)((short)((mb.data[2]<<8)|mb.data[1]))/100.0f;
        MY = -(double)((short)((mb.data[4]<<8)|mb.data[3]))/100.0f;
        FZ = -(double)((short)((mb.data[6]<<8)|mb.data[5]))/10.0f;

        MX_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*MX_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*MX;
        MY_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*MY_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*MY;
        FZ_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*FZ_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*FZ;
        mb.status = RBCAN_NODATA;
    }else ret = false;

    mb.id = ID_RCV_DATA2;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        FX = -(double)((short)((mb.data[2]<<8)|mb.data[1]))/10.0f;
        FY = -(double)((short)((mb.data[4]<<8)|mb.data[3]))/10.0f;
        MZ = -(double)((short)((mb.data[6]<<8)|mb.data[5]))/100.0f;

        FX_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*FX_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*FX;
        FY_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*FY_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*FY;
        MZ_FILTERED = (1.f-2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f)*MZ_FILTERED + 2.f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001f*MZ;
        mb.status = RBCAN_NODATA;
    }else ret = false;

    if(SENSOR_TYPE == 0){
        // Read ACC
        mb.id = ID_RCV_ACC;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            dAccRoll =  (double)((short)((mb.data[1] | (mb.data[2]<<8))))*0.1;//(double)((short)((mb.data[0] | (mb.data[1]<<8))));
            dAccPitch = (double)((short)((mb.data[3] | (mb.data[4]<<8))))*0.1;//(double)((short)((mb.data[2] | (mb.data[3]<<8))));
            AccRoll = (double)((1.0f - 2.0f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS/1000.0f)*AccRollOld + 2.0f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS/1000.0f*(double)dAccRoll);
            AccPitch = (double)((1.0f - 2.0f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS/1000.0f)*AccPitchOld + 2.0f*RBCORE_PI*CutOffFeq*(double)RT_TIMER_PERIOD_MS/1000.0f*(double)dAccPitch);
            AccRollOld = AccRoll;
            AccPitchOld = AccPitch;
            PITCH = AccPitch/SFPitch;
            ROLL = AccRoll/SFRoll;
            VelRoll = dAccRoll;//(double)((1.0f - 2.0f*RBCORE_PI*CutOffFeq*RT_TIMER_PERIOD_MS/1000.0f)*VelRollOld + RT_TIMER_PERIOD_MS/1000.0f*AccRollOld);
            VelRollOld = VelRoll;
            VelPitch = dAccPitch;//(double)((1.0f - 2.0f*RBCORE_PI*CutOffFeq*RT_TIMER_PERIOD_MS/1000.0f)*VelPitchOld + RT_TIMER_PERIOD_MS/1000.0f*AccPitchOld);
            VelPitchOld = VelPitch;
            mb.status = RBCAN_NODATA;
        }
        ret = true;
    }
    return ret;
}
