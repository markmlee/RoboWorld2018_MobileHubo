#include "RBSmartPower.h"

RBSmartPower::RBSmartPower()
{
}

void RBSmartPower::RBSP_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
}

void RBSmartPower::RBBoard_GetDBData(DB_SP db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    ID_RCV_DATA     = db.ID_RCV_DATA;
    ID_RCV_INFO     = db.ID_RCV_INFO;
    ID_SEND_GENERAL = db.ID_SEND_GENERAL;
}

int RBSmartPower::RBBoard_CANCheck(int _canr){
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
            std::cout << ">>> RMSP: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMSP: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int RBSmartPower::RBBoard_RequestStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x02;		// command
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBBoard_LoadDefaultValue(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xFA;			// command
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBSP_SetSwitchFunctions(int _sfunc){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x81;			// command
    mb.data[2] = _sfunc;       // _sfunc =
    //        0x00 :  SEC_TIME reset to zero
    //        0x01 :  Set 12VSEN and LEDR (turn on 12V for sensor)
    //        0x02 :  Clear 12VSEN and LEDR (turn off 12V for sensor)
    //        0x04 :  Set M_BTN_ON (execute power on sequenece to turn on 48V)
    //        0x05 :  Set M_BTN_OFF (execute power off sequenece to turn off 48V)
    //        0x07 :  Set BEEPF (beeper enable)
    //        0x08 :  Clear BEEPF and turn off beep (beeper disable)
    //        0x0A :  Set TGL_PC1 (turn on pc1)
    //        0x0B :  Clear TRL_PC1 (turn off pc1)
    //        0x0C :  Set TGL_PC2 (turn on pc2)
    //        0x0D :  Clear TRL_PC2 (turn off pc2)
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBSP_RequestAlarm(int _alrm){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x82;			// command
    mb.data[2] = _alrm;        // _alrm =
    //      0x00 :  Alarm OFF
    //      0x01 :  Alarm sound 1, beeper enable
    //      0x02 :  Alarm sound 2, beeper enable
    //      0x03 :  Alarm sound 3, beeper enable
    //      0x04 :  Alarm sound 4, beeper enable
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBSP_RequestBeep(int _bdur){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x83;			// command
    mb.data[2] = _bdur;        // beep duration in 0.1 sec units
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBSP_RequestVoltageAndCurrent(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xE0;			// command
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBSP_RequestTimeAndStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xE1;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBSmartPower::RBSP_ReadVoltageCurrent(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_DATA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        Voltage = (float)((int)((mb.data[0]) | (mb.data[1]<<8)))/100.f;
        Current = (float)((int)((mb.data[2]) | (mb.data[3]<<8)))/100.f;
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}

int RBSmartPower::RBSP_12VOnOff(int onoff){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;
    mb.data[1] = 0x81;
    if(onoff == true){
        mb.data[2] = 0x01;
    }else{
        mb.data[2] = 0x02;
    }
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBSmartPower::RBSP_LCDReset(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;
    mb.data[1] = 0x81;
    mb.data[2] = 0x00;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}
