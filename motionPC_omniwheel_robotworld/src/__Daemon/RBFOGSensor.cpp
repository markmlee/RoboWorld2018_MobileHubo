#include "RBFOGSensor.h"
#include "RBIMUSensor.h"

extern RBIMUSensor         _DEV_IMU[MAX_IMU];

using namespace std;


float p[4] = {1.0,0.,0.,0.};
float q[4] = {1.0,0.,0.,0.};
float Q[4] = {1.0,0.,0.,0.};


RBFOGSensor::RBFOGSensor()
{
    RS232DeviceHandler = 0;
    isWorking = false;
    FOGNullFlag = false;

    FOGRoll = FOGPitch = FOGYaw = 0.0;
    FOGReading = false;
    FOGStatus = -1;
}

RBFOGSensor::~RBFOGSensor(){
    isWorking = false;
    usleep(500*1000);
}

int RBFOGSensor::RBOpenPort(int baud){
    int port_fd, ret;//, ret1, ret2;
    struct termios settings;

    char port_name[50];
    int i;
    for(i=0; i<5; i++){
        sprintf(port_name, "/dev/ttyUSB%d",i);
        port_fd = open(port_name, O_RDWR|O_NONBLOCK|O_NOCTTY);
        if(port_fd == -1){
            FOGStatus = FOG_PORT_OPEN_ERROR;
            continue;
        }else{
            fcntl(port_fd, F_SETFL, 0);
            RS232DeviceHandler = port_fd;

            // get current port setting
            tcgetattr(RS232DeviceHandler, &settings);

            //settng the baud rate add error checking
            //ret1 = cfsetispeed(&settings, baud);
            //ret2 = cfsetospeed(&settings, baud);
            //if(ret1==-1 && ret2==-1) return -2;

            //set local mode & enable receiver
            settings.c_cflag        = baud | CS8 | CLOCAL | CREAD;
            settings.c_iflag        = IGNPAR;
            settings.c_oflag        = 0;
            settings.c_lflag        = 0;
            settings.c_cc[VMIN]     = 0;
            settings.c_cc[VTIME]    = 0;

            ret = tcsetattr(RS232DeviceHandler, TCSANOW, &settings);
            if(ret == -1){
                FOGStatus = FOG_OTHER_SET_ERROR;
                return FOG_OTHER_SET_ERROR;
            }

            if(rt_task_create(&ReceiveThreadHandler, "RBFOG_READ_TASK", 0, 94, 0) == 0){
                cpu_set_t aCPU;
                CPU_ZERO(&aCPU);
                CPU_SET(1, &aCPU);
                if(rt_task_set_affinity(&ReceiveThreadHandler, &aCPU) != 0){
                    FILE_LOG(logWARNING) << "RBFOG: Read thread set affinity CPU failed..";
                }
                if(rt_task_start(&ReceiveThreadHandler, &RBFOG_ReadThread, this) == 0){

                }else{
                    FILE_LOG(logERROR) << "RBFOG: Read thread Creation Error";
                    FOGStatus = FOG_THREAD_CREATE_ERROR;
                    return FOG_THREAD_CREATE_ERROR;
                }
            }else{
                FILE_LOG(logERROR) << "RBFOG: Read thread Creation Error";
                FOGStatus = FOG_THREAD_CREATE_ERROR;
                return FOG_THREAD_CREATE_ERROR;
            }
            break;
        }
    }

    if(i == 5){
        FILE_LOG(logERROR) << "FOG checking port over error";
        FOGStatus = FOG_CHECK_OVER_ERROR;
        return FOG_CHECK_OVER_ERROR;
    }

    FILE_LOG(logSUCCESS) << "FOG Port Open Success [ttyUSB" << i << "]";
    FOGStatus = FOG_PORT_SUCCESS;
    return FOG_PORT_SUCCESS;
}

int RBFOGSensor::RBClosePort(void){
    isWorking = false;
    usleep(100*1000);

    rt_task_delete(&ReceiveThreadHandler);
    return true;
}

int RBFOGSensor::RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode){
    uchar receivedByte = 0;
    uchar index = 0;
    uint loopTime = 0;
    char receivedData[20];
    uchar i;

    if(_mode == 0x00){
        usleep(10000);
        return read(RS232DeviceHandler, _uart_frame, _bytes);
    }else{
        while(receivedByte < _bytes){
            usleep(10000);

            index = read(RS232DeviceHandler, &receivedData[index], 20);
            if(index > 0){
                for(i=0 ; i<index ; i++) _uart_frame[receivedByte+i] = receivedData[i];
                receivedByte += index;
            }

            if(loopTime > 50) return -1;
            else loopTime++;
        }
        return receivedByte;
    }
}

int RBFOGSensor::RBWritePort(const char *_uart_frame, uchar _bytes, uchar _mode){
    char temp[20];
    if(_mode == 0x00) while((RBReadPort(temp, 20, 0x00) != 0));

    return write(RS232DeviceHandler, _uart_frame, _bytes);
}

int RBFOGSensor::clearBuf(void){
    char temp[20];

    while(RBReadPort(temp, 20, 0x00) > 0);
    StoredDataIndex = ReadDataIndex = 0;

    return 1;
}

int RBFOGSensor::RBGetReceivedDataByte(char *_data){
    if(StoredDataIndex%RS232_RECEIVE_DATA_BUFFER_SIZE != ReadDataIndex){
        *_data = ReceivedData[ReadDataIndex];
        ReadDataIndex++;

        ReadDataIndex %= RS232_RECEIVE_DATA_BUFFER_SIZE;
        return 1;
    }
    else return -1;
}


void RBFOGSensor::RBFOG_ReadThread(void *_arg)
{
    RBFOGSensor *fog = (RBFOGSensor *)_arg;
    fog->isWorking = true;

    uchar index = 0;
    char tempData;

    char state=0;
    char rcv;
    int cnt = 0;
    int tempCnt = 1000;

    //char tempC[28];
    uchar tempC[32];
    uchar crcBuff[32];
    float   tempXnull = 0.0;
    float   tempYnull = 0.0;
    float   tempZnull = 0.0;

    fog->FOGQ0 = 1.0;
    fog->FOGQ1 = 0.0;
    fog->FOGQ2 = 0.0;
    fog->FOGQ3 = 0.0;

    int oldSeq = -1;

    union{
        struct{float x,y,z,dum1,dum2,dum3; uchar disc; u_int8_t seq; short temp; uint crc;};
        uchar cData[32];
    }UnionData;


    // for CRC
    fog->order = 32;
    fog->polynom = 0x4c11db7;
    fog->direct = 1;
    fog->crcinit = 0xffffffff;
    fog->crcxor = 0;
    fog->refin = 0;
    fog->refout = 0;

    uint bit, crc;

    fog->crcmask = ((((uint)1<<(fog->order-1))-1)<<1)|1;
    fog->crchighbit = (uint)1<<(fog->order-1);

    if (!(fog->direct)){
        fog->crcinit_nondirect = fog->crcinit;
        crc = fog->crcinit;
        for(int i=0; i<fog->order; i++) {

            bit = crc & (fog->crchighbit);
            crc<<= 1;
            if (bit) crc^= fog->polynom;
        }
        crc&= (fog->crcmask);
        fog->crcinit_direct = crc;
    }else{
        fog->crcinit_direct = fog->crcinit;
        crc = fog->crcinit;
        for(int i=0; i<fog->order; i++) {

            bit = crc & 1;
            if (bit) crc^= (fog->polynom);
            crc >>= 1;
            if (bit) crc|= (fog->crchighbit);
        }
        fog->crcinit_nondirect = crc;
    }


    rt_task_set_periodic(NULL, TM_NOW, 2*1000000);

    while(fog->isWorking)
    {
        rt_task_wait_period(NULL);

        while(true){
            index = read(fog->RS232DeviceHandler, &tempData, 1);

            if(index == 1){
                fog->StoredDataIndex %= RS232_RECEIVE_DATA_BUFFER_SIZE;
                fog->ReceivedData[fog->StoredDataIndex] = tempData;
                fog->StoredDataIndex += index;
            }else{
                break;
            }

            while(fog->RBGetReceivedDataByte(&rcv) == 1){

                switch(state){
                case 0:
                    if((u_int8_t)rcv == 0xFE)
                        state = 1;
                    break;
                case 1:
                    if((u_int8_t)rcv == 0x81){
                        state = 2;
                    }else if((u_int8_t)rcv == 0xFE){
                        state = 1;
                    }else{
                        state = 0;
                    }
                    break;
                case 2:
                    if((u_int8_t)rcv == 0xFF){
                        state = 3;
                    }else if((u_int8_t)rcv == 0xFE){
                        state = 1;
                    }else{
                        state = 0;
                    }
                    break;
                case 3:
                    if((u_int8_t)rcv == 0x55){
                        state =4;
                    }else if((u_int8_t)rcv == 0xFE){
                        state = 1;
                    }else{
                        state = 0;
                    }
                    break;
                case 4:
                    tempC[cnt] = (u_int8_t)rcv;
                    cnt++;
                    if(cnt == 32){
                        fog->FOGReading = true;
                        cnt = 0;
                        state = 0;
                        // original FOG X
                        UnionData.cData[0] = tempC[3];
                        UnionData.cData[1] = tempC[2];
                        UnionData.cData[2] = tempC[1];
                        UnionData.cData[3] = tempC[0];

                        // original FOG Y
                        UnionData.cData[4] = tempC[7];
                        UnionData.cData[5] = tempC[6];
                        UnionData.cData[6] = tempC[5];
                        UnionData.cData[7] = tempC[4];

                        // original FOG Z
                        UnionData.cData[8] = tempC[11];
                        UnionData.cData[9] = tempC[10];
                        UnionData.cData[10] = tempC[9];
                        UnionData.cData[11] = tempC[8];

                        UnionData.cData[24] = tempC[24];
                        UnionData.cData[25] = tempC[25];

                        // CRC
                        UnionData.cData[28] = tempC[31];
                        UnionData.cData[29] = tempC[30];
                        UnionData.cData[30] = tempC[29];
                        UnionData.cData[31] = tempC[28];

                        u_int8_t seqNum = UnionData.seq;

                        // CRC Checking
                        crcBuff[0] = 0xFE;
                        crcBuff[1] = 0x81;
                        crcBuff[2] = 0xFF;
                        crcBuff[3] = 0x55;
                        memcpy(&crcBuff[4],  tempC, 28);
                        uint retCRC = fog->crcbitbybit(crcBuff, 32);
                        if(retCRC != UnionData.crc){
                            FILE_LOG(logERROR) << "CRC ERROR";
                            break;
                        }

                        int invalid = true;
                        if((UnionData.disc) && 0x01 == 0x00){
                            invalid = false;
                            FILE_LOG(logWARNING) << "x invalid";
                        }
                        if((UnionData.disc>>1) && 0x01 == 0x00){
                            invalid = false;
                            FILE_LOG(logWARNING) << "y invalid";
                        }
                        if((UnionData.disc>>2) && 0x01 == 0x00){
                            invalid = false;
                            FILE_LOG(logWARNING) << "z invalid";
                        }
                        if(invalid == false){
                            break;
                        }

                        float R2D = 57.2957802f;
                        float D2R = 0.0174533f;
                        float velX = (float)(UnionData.z);  // original FOG X
                        float velY = -(float)(UnionData.y);  // original FOG Y
                        float velZ = (float)(UnionData.x);  // original FOG Z

                        const float maxVel = 200.0*D2R;
                        if(fabs(velX) > maxVel || fabs(velY) > maxVel || fabs(velZ) > maxVel){
                            FILE_LOG(logWARNING) << "FOG over 200deg/sec";
                            sharedSEN->FOG.Warning = true;
                        }

//                        float delX = velX + velY*sin((fog->FOGRoll)*D2R)*tan(fog->FOGPitch*D2R) + velZ*cos(fog->FOGRoll*D2R)*tan(fog->FOGPitch*D2R);
//                        float delY = velY*cos(fog->FOGRoll*D2R) - velZ*sin(fog->FOGRoll*D2R);
//                        float delZ = velY*sin(fog->FOGRoll*D2R)/(cos(fog->FOGPitch*D2R)) + velZ*cos(fog->FOGRoll*D2R)/(cos(fog->FOGPitch*D2R));



                        float delX = velX + velY*sin((fog->FOGRoll*D2R))*tan(fog->FOGPitch*D2R) + velZ*cos(fog->FOGRoll*D2R)*tan(fog->FOGPitch*D2R);
                        float delY = velY*cos(fog->FOGRoll*D2R) - velZ*sin(fog->FOGRoll*D2R);
                        float delZ = velY*sin(fog->FOGRoll*D2R)/(cos(fog->FOGPitch*D2R)) + velZ*cos(fog->FOGRoll*D2R)/(cos(fog->FOGPitch*D2R));

                        // 17.1.27
                        // quaternion
                        float l = sqrt(velX*velX + velY*velY + velZ*velZ);
                        float p[4] = {cos(0.002*l/2.0),velX*sin(0.002*l/2.0)/l,velY*sin(0.002*l/2.0)/l,velZ*sin(0.002*l/2.0)/l};

                        Q[0] = fog->FOGQ0*p[0] - fog->FOGQ1*p[1] - fog->FOGQ2*p[2] - fog->FOGQ3*p[3];
                        Q[1] = fog->FOGQ0*p[1] + fog->FOGQ1*p[0] + fog->FOGQ2*p[3] - fog->FOGQ3*p[2];
                        Q[2] = fog->FOGQ0*p[2] - fog->FOGQ1*p[3] + fog->FOGQ2*p[0] + fog->FOGQ3*p[1];
                        Q[3] = fog->FOGQ0*p[3] + fog->FOGQ1*p[2] - fog->FOGQ2*p[1] + fog->FOGQ3*p[0];

                        fog->FOGQ0 = Q[0];
                        fog->FOGQ1 = Q[1];
                        fog->FOGQ2 = Q[2];
                        fog->FOGQ3 = Q[3];


                        if(isnan(delX) == true){
                            FILE_LOG(logWARNING) << "FOG nan value";
                            delX = 0.;
                            fog->FOGQ0 = 1.0;
                            fog->FOGQ1 = 0.;//Q[0]Q[0];
                            fog->FOGQ2 = 0.;//Q[0]Q[0];

                            fog->FOGQ3 = 0.;//Q[0];
                        }
                        if(isnan(delY) == true){
                            FILE_LOG(logWARNING) << "FOG nan value";
                            delY = 0.;

                            fog->FOGQ0 = 1.0;
                            fog->FOGQ1 = 0.;//Q[0]Q[0];
                            fog->FOGQ2 = 0.;//Q[0]Q[0];

                            fog->FOGQ3 = 0.;//Q[0];
                        }
                        if(isnan(delZ) == true){
                            FILE_LOG(logWARNING) << "FOG nan value";
                            delZ = 0.;

                            fog->FOGQ0 = 1.0;
                            fog->FOGQ1 = 0.;//Q[0]Q[0];
                            fog->FOGQ2 = 0.;//Q[0]Q[0];

                            fog->FOGQ3 = 0.;//Q[0];
                        }
                        sharedSEN->FOG.RollVel  = delX;
                        sharedSEN->FOG.PitchVel = delY;
                        sharedSEN->FOG.YawVel   = delZ;


                        if(fog->FOGNullFlag == true){
                            fog->FOGNullFlag = false;
                            tempXnull = 0.0;
                            tempYnull = 0.0;
                            tempZnull = 0.0;
                            tempCnt = 0;
                            FILE_LOG(logINFO) << "FOG nulling...";
                        }
                        if(tempCnt < 1000){
                            tempXnull += delX;
                            tempYnull += delY;
                            tempZnull += delZ;
                            tempCnt++;
                            if(tempCnt == 1000){
                                FILE_LOG(logINFO) << "FOG nulling done";
                            }
                        }

//                        fog->FOGRoll += delX*R2D*0.002;//0.001;
//                        fog->FOGPitch += delY*R2D*0.002;//0.001;
//                        fog->FOGYaw += delZ*R2D*0.002;//0.001;

                        fog->FOGYaw = R2D*atan2(2.0*(Q[1]*Q[2] + Q[0]*Q[3]),Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3]);//Q[1];//delX*0.002;//0.001;
                        fog->FOGPitch = R2D*asin(-2.0*(Q[1]*Q[3] - Q[0]*Q[2]));//Q[2];//delY*0.002;//0.001;
                        fog->FOGRoll = R2D*atan2(2.0*(Q[2]*Q[3] + Q[0]*Q[1]) , Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3]);//Q[3];//delZ*0.002;//0.001;



                        if((u_int8_t)(seqNum-oldSeq) != 1 && (int)(seqNum-oldSeq) != -127 && oldSeq != -1){
                            FILE_LOG(logWARNING) << "FOG sequence number error";
                        }

                        _DEV_IMU[0].FOG_ROLL_NULL = fog->FOGRoll;
                        _DEV_IMU[0].FOG_PITCH_NULL = fog->FOGPitch;
                        _DEV_IMU[0].FOG_YAW_NULL = fog->FOGYaw;

                        if(isnan(fog->FOGYaw) == true){
                            _DEV_IMU[0].FOG_ROLL_OFFSET  = -_DEV_IMU[0].FOG_ROLL_NULL;//-sharedData->FOGRoll;
                            _DEV_IMU[0].FOG_PITCH_OFFSET = -_DEV_IMU[0].FOG_PITCH_NULL;//-sharedData->FOGPitch;
                            _DEV_IMU[0].FOG_YAW_OFFSET = -_DEV_IMU[0].FOG_YAW_NULL;
                            fog->FOGYaw = 0.;
                        }
                        if(isnan(fog->FOGPitch) == true){
                            FILE_LOG(logWARNING) << "FOG nan value";
                            fog->FOGPitch = 0.;
                        }
                        if(isnan(fog->FOGRoll) == true){
                            FILE_LOG(logWARNING) << "FOG nan value";
                            fog->FOGRoll = 0.;
                        }

//                        if(fog->FOGPitch >= 0.5235)   fog->FOGPitch = 0.5235;
//                        if(fog->FOGPitch <= -0.5235)  fog->FOGPitch = -0.5235;
//                        if(fog->FOGRoll >= 0.5235)   fog->FOGRoll = 0.5235;
//                        if(fog->FOGRoll <= -0.5235)  fog->FOGRoll = -0.5235;

                        if(fog->FOGPitch >= 30.0)   fog->FOGPitch = 30.0;
                        if(fog->FOGPitch <= -30.0)  fog->FOGPitch = -30.0;
                        if(fog->FOGRoll >= 30.0)   fog->FOGRoll = 30.0;
                        if(fog->FOGRoll <= -30.0)  fog->FOGRoll = -30.0;



                        sharedSEN->FOG.Roll = fog->FOGRoll;
                        sharedSEN->FOG.Pitch = fog->FOGPitch;
                        sharedSEN->FOG.Yaw = fog->FOGYaw;

                        oldSeq = seqNum;
                    }
                    break;
                default:
                    break;
                }
            }
        }
    }
}


uint RBFOGSensor::reflect (uint crc, int bitnum){
    // reflects the lower 'bitnum' bits of 'crc'
    uint i, j=1, crcout=0;

    for (i=(uint)1<<(bitnum-1); i; i>>=1) {
        if (crc & i) crcout|=j;
        j<<= 1;
    }
    return (crcout);
}

uint RBFOGSensor::crcbitbybit(uchar* p, int len){
    // bit by bit algorithm with augmented zero bytes.
    // does not use lookup table, suited for polynom orders between 1...32.

    uint j, c, bit;
    uint crc = crcinit_nondirect;

    for(int i=0; i<len; i++){
        c = (uint)*p++;
        if (refin) c = reflect(c, 8);

        for(j=0x80; j; j>>=1){
            bit = crc & crchighbit;
            crc<<= 1;
            if (c & j) crc|= 1;
            if (bit) crc^= polynom;
        }
    }

    for(int i=0; i<order; i++){
        bit = crc & crchighbit;
        crc<<= 1;
        if (bit) crc^= polynom;
    }

    if (refout) crc=reflect(crc, order);
    crc^= crcxor;
    crc&= crcmask;

    return(crc);
}

uint RBFOGSensor::crcbitbybitfast(uchar* p, uint len){
    // fast bit by bit algorithm without augmented zero bytes.
    // does not use lookup table, suited for polynom orders between 1...32.
    uint i, j, c, bit;
    uint crc = crcinit_direct;

    for (i=0; i<len; i++){
        c = (uint)*p++;
        if (refin) c = reflect(c, 8);

        for (j=0x80; j; j>>=1){
            bit = crc & crchighbit;
            crc<<= 1;
            if (c & j) bit^= crchighbit;
            if (bit) crc^= polynom;
        }
    }

    if (refout) crc=reflect(crc, order);
    crc^= crcxor;
    crc&= crcmask;

    return(crc);
}
