#include "RBCAN.h"

// --------------------------------------------------------------------------------------------- //
void RBCAN::RBCAN_ReadThread(void *_arg){
    RBCAN *rbCAN = (RBCAN *)_arg;
    TPCANRdMsg m;
    int index;

    while(rbCAN->isWorking == true){
        if(rbCAN->isSuspend == true){
            usleep(10);
            continue;
        }
        for(int i=0; i<rbCAN->chNum; i++){
            while(LINUX_CAN_Read_Timeout(rbCAN->canHandler[i], &m, 0) == 0){
                index = rbCAN->RBCAN_GetMailBoxIndex(m.Msg.ID);

                if(index < rbCAN->canMBCounter){
                    rbCAN->canReadMB[index].id = m.Msg.ID;
                    rbCAN->canReadMB[index].dlc = m.Msg.LEN;
                    memcpy(rbCAN->canReadMB[index].data, m.Msg.DATA, m.Msg.LEN);
                    rbCAN->canReadMB[index].channel = i;
                    if(rbCAN->canReadMB[index].status == RBCAN_NEWDATA)
                        rbCAN->canReadMB[index].status = RBCAN_OVERWRITE;
                    else rbCAN->canReadMB[index].status = RBCAN_NEWDATA;
                }
            }
        }
        usleep(10);
    }
}
// --------------------------------------------------------------------------------------------- //
void *RBCAN::RBCAN_WriteThread(void *_arg){
    RBCAN *rbCAN = (RBCAN *)_arg;
    TPCANMsg m;
    int index = 0;

    while(rbCAN->isWorking){
        if(rbCAN->isSuspend == true){
            usleep(3);
            continue;
        }
        if(rbCAN->canSendSuspend == true){
            if(rbCAN->canHeadIndex != rbCAN->canTailIndex){
                index = rbCAN->canTailIndex;
                m.ID = rbCAN->canWriteMB[index].id;
                m.LEN = rbCAN->canWriteMB[index].dlc;
                memcpy(m.DATA, rbCAN->canWriteMB[index].data, rbCAN->canWriteMB[index].dlc);

                LINUX_CAN_Write_Timeout(rbCAN->canHandler[rbCAN->canWriteMB[index].channel], &m, 0);
                rbCAN->canTailIndex = (rbCAN->canTailIndex + 1) % RBCAN_MAX_MB;
            }
        }
        usleep(5);
    }
    return NULL;
}
// --------------------------------------------------------------------------------------------- //


// --------------------------------------------------------------------------------------------- //
RBCAN::RBCAN(int _ChNum){
    if(_ChNum > RBCAN_MAX_CAN_CHANNEL){
        FILE_LOG(logERROR) << "Over the maximum CAN channel [" << RBCAN_MAX_CAN_CHANNEL << " ]";
        chNum = 0;
        return;
    }else{
        chNum = _ChNum;
    }

    int oknum = 0;
    int okflag = false;
    for(int i=0; i<MAX_SEARCH_CHANNEL; i++){
        char filePath[30];
        sprintf(filePath, "/dev/pcanusb%d", i);
        canHandler[oknum] = NULL;
        canHandler[oknum] = LINUX_CAN_Open(filePath, O_RDWR);

        if(canHandler[oknum] != NULL){
            if(CAN_Init(canHandler[oknum], CAN_BAUD_1M, CAN_INIT_TYPE_ST) !=  CAN_ERR_OK){
                FILE_LOG(logWARNING) << "Fail to setting CAN device (" << i << ")";
            }else{
                oknum++;
                if(oknum >= chNum){
                    okflag = true;
                    break;
                }
            }
        }else{
            FILE_LOG(logWARNING) << "Fail to open CAN device (" << i << ")";
        }
    }

    if(okflag == true){
        isWorking = true;
        canSendSuspend = true;
        isSuspend = false;
        canMBCounter = 0;
        canHeadIndex = 0;
        canTailIndex = 0;

        if(RBCAN_StartThread() == false){
            isWorking = false;
            FILE_LOG(logERROR) << "CAN hardware initialize = FAIL";
        }else{
            FILE_LOG(logSUCCESS) << "CAN hardware initialize = OK";
        }
    }else{
        isWorking = false;
        FILE_LOG(logERROR) << "CAN hardware initialize = FAIL";
    }
}
// --------------------------------------------------------------------------------------------- //
RBCAN::~RBCAN(){
    isWorking = false;
    usleep(100*1000);

    for(int i=0; i<chNum; i++)
        CAN_Close(canHandler[i]);
}
// --------------------------------------------------------------------------------------------- //
void RBCAN::Finish(){
    isWorking = false;
    usleep(100*1000);

//    for(int i=0; i<chNum; i++)
//        CAN_Close(canHandler[i]);
}

// --------------------------------------------------------------------------------------------- //
void RBCAN::RBResetCAN(){

    isSuspend = true;
    usleep(10);
    for(int i=0; i<chNum; i++)
        CAN_Close(canHandler[i]);

    int oknum = 0;
    for(int i=0; i<MAX_SEARCH_CHANNEL; i++){
        char filePath[30];
        sprintf(filePath, "/dev/pcanusb%d", i);
        canHandler[oknum] = NULL;
        canHandler[oknum] = LINUX_CAN_Open(filePath, O_RDWR);

        if(canHandler[oknum] != NULL){
            if(CAN_Init(canHandler[oknum], CAN_BAUD_1M, CAN_INIT_TYPE_ST) !=  CAN_ERR_OK){
                FILE_LOG(logWARNING) << "Fail to setting CAN device (" << i << ")";
            }else{
                oknum++;
                if(oknum >= chNum){
                    break;
                }
            }
        }else{
            FILE_LOG(logWARNING) << "Fail to open CAN device (" << i << ")";
        }
    }

    isSuspend = false;
}

// --------------------------------------------------------------------------------------------- //
bool RBCAN::IsWorking(){
    return isWorking;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_StartThread(void)
{
    if(rt_task_create(&canReadThreadHandler, "RBCAN_READ_TASK", 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(1, &aCPU);
        if(rt_task_set_affinity(&canReadThreadHandler, &aCPU) != 0){
            FILE_LOG(logWARNING) << "RBCAN: Read thread set affinity CPU failed..";
        }
        if(rt_task_start(&canReadThreadHandler, &RBCAN_ReadThread, this) == 0){

        }else{
            FILE_LOG(logERROR) << "RBCAN: Read thread Creation Error";
            return false;
        }
    }else{
        FILE_LOG(logERROR) << "RBCAN: Read thread Creation Error";
        return false;
    }

    int threadID = pthread_create(&canWriteThreadHandler, NULL, &RBCAN_WriteThread, this);
    if(threadID < 0){
        FILE_LOG(logERROR) << "RBCAN: Write thread Creation Error";
        return false;
    }
    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteData(RBCAN_MB _mb)
{
    canWriteMB[canHeadIndex] = _mb;
    canHeadIndex = (canHeadIndex+1) % RBCAN_MAX_MB;
    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteDataDirectly(RBCAN_MB _mb)
{
    TPCANMsg m;
    m.ID = _mb.id;
    m.LEN = _mb.dlc;
    m.MSGTYPE = MSGTYPE_STANDARD;
    memcpy(m.DATA, _mb.data, m.LEN);

    if(LINUX_CAN_Write_Timeout(canHandler[_mb.channel], &m, 0)){
        return false;
    }
    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_ReadData(pRBCAN_MB _mb)
{
    unsigned int index = RBCAN_GetMailBoxIndex(_mb->id);

    _mb->dlc = canReadMB[index].dlc;
    memcpy(_mb->data, canReadMB[index].data, canReadMB[index].dlc);
    _mb->status = canReadMB[index].status;

    canReadMB[index].status = RBCAN_NODATA;
    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_AddMailBox(unsigned int _id)
{
    unsigned char i;

    if(canMBCounter >= RBCAN_MAX_MB){
        FILE_LOG(logWARNING) << "Over the CAN mail box";
        return false;
    }else{
        if(RBCAN_GetMailBoxIndex(_id) == canMBCounter){
            canReadMB[canMBCounter].id = _id;                           // CAN id assign
            canReadMB[canMBCounter].dlc = 0x00;                         // Data Length Code
            for(i=0; i<8; i++) canReadMB[canMBCounter].data[i] = 0x00;	// Data init.
            canReadMB[canMBCounter].status = RBCAN_NODATA;          // Initial MB status = No Data
            canMBCounter++;
            return true;
        }else{
            if(_id != 0x00){
                FILE_LOG(logWARNING) << "CAN ID(" << _id << ") is already assigned";
            }
            return false;
        }
    }
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_GetMailBoxIndex(unsigned int _id)
{
    for(int i=0 ; i<canMBCounter; i++) if(canReadMB[i].id == _id) return i;
    return canMBCounter;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteEnable(int _suspend)
{
    canSendSuspend = _suspend;
    return canSendSuspend;
}
// --------------------------------------------------------------------------------------------- //
