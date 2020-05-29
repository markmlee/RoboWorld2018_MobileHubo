#ifndef RBFOGSENSOR_H
#define RBFOGSENSOR_H

#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>

#include "RBDataType.h"
#include "RBLog.h"

#include <alchemy/task.h>


#define RS232_RECEIVE_DATA_BUFFER_SIZE	2000

enum _FOG_PORT_OPEN_STAT_{
    FOG_PORT_SUCCESS = 0,
    FOG_PORT_OPEN_ERROR,
    FOG_BAUD_SET_ERROR,
    FOG_OTHER_SET_ERROR,
    FOG_THREAD_CREATE_ERROR,
    FOG_CHECK_OVER_ERROR
};


class RBFOGSensor
{
public:
    RBFOGSensor();
    ~RBFOGSensor();

    // RS-232 ----
    uint    StoredDataIndex;
    uint    ReadDataIndex;
    char    NewDataAvailable;
    char    ReceivedData[RS232_RECEIVE_DATA_BUFFER_SIZE];
    char    ReceivedByte;
    char    WantedByte;
    int     RS232DeviceHandler;

    // FOG -----
    int     FOGNullFlag;
    float   FOGRoll;
    float   FOGPitch;
    float   FOGYaw;

    float   FOGQ0;
    float   FOGQ1;
    float   FOGQ2;
    float   FOGQ3;

    int     FOGStatus;
    int     FOGReading;


    // functions for serial communication
    int     RBOpenPort(int baud);
    int     RBClosePort(void);
    int     RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode);
    int     RBWritePort(const char *_uart_frame, uchar _bytes, uchar _mode);
    int     RBGetReceivedDataByte(char *_data);

private:
    int         isWorking;
    RT_TASK     ReceiveThreadHandler;
    static void RBFOG_ReadThread(void *_arg);

    // for CRC
    int     order;
    uint    polynom;
    int     direct;
    uint    crcinit;
    uint    crcxor;
    int     refin;
    int     refout;

    uint    crcmask;
    uint    crchighbit;
    uint    crcinit_direct;
    uint    crcinit_nondirect;
    uint    crctab[256];

    uint    reflect (uint crc, int bitnum);
    uint    crcbitbybit(uchar* p, int len);
    uint    crcbitbybitfast(uchar* p, uint len);
    int     clearBuf(void);
};

#endif // RBFOGSENSOR_H
