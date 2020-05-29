#ifndef RBRAWLAN_H
#define RBRAWLAN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <pthread.h>

#include "RBLog.h"
#include "RBDataType.h"


enum{
    NEW_CONNECTION = 0,
    DATA_IN_BUFFER,
    CONNECTION_CLOSED,
    LAN_NEW_DATA,
    LAN_OLD_DATA
};

#define RX_DATA_SIZE    4000

class RBRawLAN
{
public:
    RBRawLAN();

    int ConnectionStatus;
    int RXDataReadIndex;
    int RXDataStoreIndex;
    char RXData[RX_DATA_SIZE];


    int LAN_SERVER_FD;
    int LAN_CLIENT_FD;

    int RBLanOpen(int _port);
    int RBLanClose();
    int RBLanWriteData(void *_txData, int _size);
    int RBLanReadData(char *_rxData, int _size, char _mode);
    int RBLanReadDataByte(char *_rxData);
    int RBLanStoredDataSize();
    int RBLanClearStoredData();

    int LanTCPReady(int _port = 8000);
    int LanTCPAccept();
    int LanTCPClientClose();
    int LanTCPServerClose();

private:
    int					isWorking;

    struct sockaddr_in  ServerAddr;
    struct sockaddr_in  ClientAddr;

    static void     *RBLAN_ReadThread(void *_arg);
    ulong           lanReadThreadHandler;
};

#endif // RBRAWLAN_H
