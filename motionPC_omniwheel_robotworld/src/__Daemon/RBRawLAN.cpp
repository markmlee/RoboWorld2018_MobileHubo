#include "RBRawLAN.h"

RBRawLAN::RBRawLAN()
{
    RXDataStoreIndex = RXDataReadIndex = 0;
    LAN_SERVER_FD = NULL;
    LAN_CLIENT_FD = NULL;
    ConnectionStatus = false;
}


void *RBRawLAN::RBLAN_ReadThread(void *_arg){
    RBRawLAN *rbLAN = (RBRawLAN *)_arg;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    char tempData[RX_DATA_SIZE];

    while(rbLAN->isWorking){
        if(tcp_status == 0x00){     // if client was not connected
            if(rbLAN->LanTCPAccept() == true){
                tcp_status = 0x01;  // LanTCPAccept() will wait until client is connected
                rbLAN->RXDataReadIndex = rbLAN->RXDataStoreIndex = 0;
                rbLAN->ConnectionStatus = true;
                FILE_LOG(logSUCCESS) << "Client is connected..!!";
            }else{
                FILE_LOG(logERROR) << "Can't accept client connection..!!";
            }
        }

        if(tcp_status == 0x01){     // if client was connected
            // tcp_size is bytes of received data.
            // If 0 was returned, it means the connect is closed
            // tcp_size = read(rbLAN->LAN_CLIENT_FD, &rbLAN->RXData[Owner->RXDataStoreIndex], RX_DATA_SIZE);
            tcp_size = read(rbLAN->LAN_CLIENT_FD, tempData, RX_DATA_SIZE);

            if(tcp_size == 0){
                rbLAN->LanTCPClientClose();
                rbLAN->LAN_CLIENT_FD = NULL;
                tcp_status = 0x00;
                rbLAN->RXDataReadIndex = rbLAN->RXDataStoreIndex = 0;
                rbLAN->ConnectionStatus = false;
                FILE_LOG(logERROR) << "Client is disconnected..!!";
            }else if(tcp_size > 0){
                for(int i=0; i<tcp_size; i++){
                    rbLAN->RXDataStoreIndex %= RX_DATA_SIZE;
                    rbLAN->RXData[rbLAN->RXDataStoreIndex] = tempData[i];
                    rbLAN->RXDataStoreIndex++;
                }
            }
        }
    }
}


int RBRawLAN::RBLanOpen(int _port){
    if(LanTCPReady(_port) == true){
        isWorking = true;
        int threadID = pthread_create(&lanReadThreadHandler, NULL, &RBLAN_ReadThread, this);
        if(threadID < 0){
            isWorking = false;
            FILE_LOG(logERROR) << "RBRawLAN: Read thread Creation Error";
            return false;
        }
    }else{
        FILE_LOG(logERROR) << "RBRawLAN: TCP Ready Error";
        return false;
    }
    return true;
}


int RBRawLAN::RBLanClose(){
    isWorking = false;
    usleep(100*1000);

    if(LAN_CLIENT_FD == NULL){
        // do nothing
    }else{
        if(shutdown(LAN_SERVER_FD, SHUT_RDWR) == -1){
            FILE_LOG(logERROR) << "LanTCPServerShutdown is Failed..!!";
        }
        usleep(100*1000);
        if(LanTCPClientClose() == -1){
            FILE_LOG(logERROR) << "LanTCPClientClose is Failed..!!";
        }
        usleep(100*1000);
    }
    usleep(200*1000);

    memset(RXData, 0, RX_DATA_SIZE);
    return true;
}


int RBRawLAN::RBLanWriteData(void *_txData, int _size){
    // send data when data is updated
    int n;
    if( (n=write(LAN_CLIENT_FD, _txData, _size)) == _size ){
        return true;
    }
    return false;
}


int RBRawLAN::RBLanReadData(char *_rxData, int _size, char _mode){
    int tempSize = 0;
    int timeOutCount = 0;

    if(_mode == 0x00){   // check one time
        tempSize = RBLanStoredDataSize();
        if(tempSize < _size)
            return LAN_OLD_DATA;
    }else{               // check until receiving wanted size of data
        while(tempSize < _size){
            timeOutCount++;
            tempSize = RBLanStoredDataSize();
            usleep(500);

            if(timeOutCount >= 10)
                return LAN_OLD_DATA;
        }
    }

    for(tempSize=0 ; tempSize<_size ; tempSize++){
        RXDataReadIndex %= RX_DATA_SIZE;
        memcpy(&_rxData[tempSize], &RXData[RXDataReadIndex], 1);
        RXDataReadIndex++;
    }

    return LAN_NEW_DATA;
}

int RBRawLAN::RBLanReadDataByte(char *_rxData){
    if(RXDataStoreIndex != RXDataReadIndex){
        RXDataReadIndex %= RX_DATA_SIZE;
        *_rxData = RXData[RXDataReadIndex];
        RXDataReadIndex++;
        return LAN_NEW_DATA;
    }
    return LAN_OLD_DATA;
}

int RBRawLAN::RBLanStoredDataSize(void){
    int tempSize;

    if(RXDataStoreIndex < RXDataReadIndex){
        tempSize = RXDataStoreIndex - RXDataReadIndex + RX_DATA_SIZE;
    }else{
        tempSize = RXDataStoreIndex - RXDataReadIndex;
    }

    return tempSize+1;
}

int RBRawLAN::RBLanClearStoredData(){
    char tempData = 0;
    int tempCount = 0;
    int status = RBLanReadDataByte(&tempData);

    while(status == LAN_NEW_DATA){
        status = RBLanReadDataByte(&tempData);

        if(tempCount >= RX_DATA_SIZE) return false;
        else tempCount++;
    }

    RXDataStoreIndex = RXDataReadIndex = 0;
    return true;
}

int RBRawLAN::LanTCPReady(int _port){
    LAN_SERVER_FD = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_SERVER_FD == -1)
        return false;

    memset(&ServerAddr, 0, sizeof(ServerAddr));
    ServerAddr.sin_family     = AF_INET;
    ServerAddr.sin_port       = htons(_port);
    ServerAddr.sin_addr.s_addr= htonl(INADDR_ANY);

    int optval = 1;
    setsockopt(LAN_SERVER_FD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(LAN_SERVER_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    struct linger optlinger;
    optlinger.l_onoff = 1;
    optlinger.l_linger = 3600;	// 3600sec
    setsockopt(LAN_SERVER_FD, SOL_SOCKET, SO_LINGER, (char*)&optlinger, sizeof(optlinger));

    if(bind(LAN_SERVER_FD, (struct sockaddr*)&ServerAddr, sizeof(ServerAddr)) == -1)
        return false;
    return true;
}


int RBRawLAN::LanTCPAccept(void){
    unsigned int client_addr_size;

    FILE_LOG(logINFO) << "Server waits for client connection..!!";

    if(listen(LAN_SERVER_FD, 5) == -1)
        return false;

    client_addr_size = sizeof(ClientAddr);
    LAN_CLIENT_FD = accept(LAN_SERVER_FD, (struct sockaddr*)&ClientAddr, &client_addr_size);

    if(LAN_CLIENT_FD == -1)
        return false;

    return true;
}


int RBRawLAN::LanTCPClientClose(void)
{
    return close(LAN_CLIENT_FD);
}
int RBRawLAN::LanTCPServerClose(void)
{
    return close(LAN_SERVER_FD);
}
