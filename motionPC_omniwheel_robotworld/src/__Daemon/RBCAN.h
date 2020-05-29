#ifndef RBCAN_H
#define RBCAN_H

#include "RBLog.h"
#include "RBDataType.h"

#include <libpcan.h>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>


#include <alchemy/task.h>


#define 	RBCAN_MAX_CAN_CHANNEL   4
#define 	RBCAN_MAX_MB			150
#define     MAX_SEARCH_CHANNEL      10

typedef enum{
    RBCAN_NODATA = 0,
    RBCAN_NEWDATA,
    RBCAN_OVERWRITE,
    RBCAN_SENTDATA
} RBCAN_STATUS;

typedef struct _RBCAN_MB_
{
    uint    id;         // Identifier
    uchar   data[8];    // Data
    uchar   dlc;        // Data Length Code
    uchar   status;     // MB status
    uchar   channel;    // CAN channel (0, 1, 2, ...)
} RBCAN_MB, *pRBCAN_MB;



class RBCAN
{
public:
    RBCAN(int _ChNum = 2);
    ~RBCAN();

    void    Finish();
    void    RBResetCAN();

    // Loading mail box configuration
    int                 RBCAN_WriteData(RBCAN_MB _mb);
    int                 RBCAN_WriteDataDirectly(RBCAN_MB _mb);
    int                 RBCAN_ReadData(pRBCAN_MB _mb);
    int                 RBCAN_AddMailBox(unsigned int _id);
    int                 RBCAN_WriteEnable(int _suspend);

    bool                IsWorking();
    void				*canHandler[RBCAN_MAX_CAN_CHANNEL];


private:
    // member functions
    int                 RBCAN_StartThread(void);
    static void         RBCAN_ReadThread(void *_arg);
    static void         *RBCAN_WriteThread(void *_arg);
    int					RBCAN_GetMailBoxIndex(unsigned int _id);
    // member variables
    int					isWorking;
    //ulong				canReadThreadHandler;
    RT_TASK             canReadThreadHandler;
    ulong				canWriteThreadHandler;
    RBCAN_MB		canReadMB[RBCAN_MAX_MB];
    RBCAN_MB		canWriteMB[RBCAN_MAX_MB];

    int					chNum;
    int					canMBCounter;
    int					canHeadIndex;
    int					canTailIndex;
    int					canSendSuspend;

    int                 isSuspend;
};



#endif // RBCAN_H
