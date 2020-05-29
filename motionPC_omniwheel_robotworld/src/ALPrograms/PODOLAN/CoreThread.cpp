#include "CoreThread.h"


extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern pUSER_SHM               sharedUSER;

CoreThread::CoreThread()
{

}

void CoreThread::run(){
    QTimer timerPODO2GUI, timerGUI2PODO, timerVISION, timerROS;
    CoreWorker worker;

    connect(&timerPODO2GUI, SIGNAL(timeout()), &worker, SLOT(onPODO2GUI()));
    connect(&timerGUI2PODO, SIGNAL(timeout()), &worker, SLOT(onGUI2PODO()));

    /*RX from VisionPC*/
    connect(&timerVISION, SIGNAL(timeout()), &worker, SLOT(onVISION2PODO()));
    connect(&timerVISION, SIGNAL(timeout()), &worker, SLOT(onPODO2VISION()));
    connect(&timerVISION, SIGNAL(timeout()), &worker, SLOT(onROS2PODO()));
    connect(&timerVISION, SIGNAL(timeout()), &worker, SLOT(onPODO2ROS()));

    timerPODO2GUI.start(50);
    timerGUI2PODO.start(50);
    timerVISION.start(50);
    timerROS.start(50);
    exec();
}



CoreWorker::CoreWorker(){
    serverPODOGUI = new PODO_GUI_Server();
    serverPODOGUI->RBServerOpen(QHostAddress::AnyIPv4, 4000);

    serverPODOVISION = new PODO_VISION_Server();
    serverPODOVISION->RBServerOpen(QHostAddress::AnyIPv4, 5500);

    serverROSCMD = new ROS_CMD_Server();
    serverROSCMD->RBServerOpen(QHostAddress::AnyIPv4, 6000);


}


void CoreWorker::SendtoGUI(){
    memcpy(&(DATA_PODO.CoreREF), sharedREF, sizeof(RBCORE_SHM_REFERENCE));
    memcpy(&(DATA_PODO.CoreSEN), sharedSEN, sizeof(RBCORE_SHM_SENSOR));
    memcpy(&(DATA_PODO.CoreCMD), sharedCMD, sizeof(RBCORE_SHM_COMMAND));
    memcpy(&(DATA_PODO.UserM2G), &(sharedUSER->M2G), sizeof(MOTION2GUI));
    memcpy(&(DATA_PODO.UserR2M), &(sharedUSER->R2M), sizeof(ROS2MOTION));



    QByteArray SendData = QByteArray::fromRawData((char*)&DATA_PODO, sizeof(LAN_PODO2GUI));
    serverPODOGUI->RBSendData(SendData);

    sharedCMD->ErrorInform2GUI = 0;
}

void CoreWorker::ReadfromGUI(){
    QByteArray tempData = serverPODOGUI->dataReceived[0];
    serverPODOGUI->dataReceived.pop_front();

//    USER_COMMAND cmd;
//    memcpy(&cmd, tempData, sizeof(USER_COMMAND));

//    int target = cmd.COMMAND_TARGET;
//    for(int i=0; i<MAX_COMMAND_DATA; i++){
//        sharedCMD->COMMAND[target].USER_PARA_CHAR[i]    = cmd.COMMAND_DATA.USER_PARA_CHAR[i];
//        sharedCMD->COMMAND[target].USER_PARA_INT[i]     = cmd.COMMAND_DATA.USER_PARA_INT[i];
//        sharedCMD->COMMAND[target].USER_PARA_FLOAT[i]   = cmd.COMMAND_DATA.USER_PARA_FLOAT[i];
//        sharedCMD->COMMAND[target].USER_PARA_DOUBLE[i]  = cmd.COMMAND_DATA.USER_PARA_DOUBLE[i];
//    }
//    sharedCMD->COMMAND[target].USER_COMMAND = cmd.COMMAND_DATA.USER_COMMAND;

    LAN_GUI2PODO tempDATA;
    memcpy(&tempDATA, tempData, sizeof(LAN_GUI2PODO));
    memcpy(&(sharedUSER->G2M), &(tempDATA.UserG2M), sizeof(GUI2MOTION));

    int target = tempDATA.UserCMD.COMMAND_TARGET;
    for(int i=0; i<MAX_COMMAND_DATA; i++){
        sharedCMD->COMMAND[target].USER_PARA_CHAR[i]    = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_CHAR[i];
        sharedCMD->COMMAND[target].USER_PARA_INT[i]     = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_INT[i];
        sharedCMD->COMMAND[target].USER_PARA_FLOAT[i]   = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_FLOAT[i];
        sharedCMD->COMMAND[target].USER_PARA_DOUBLE[i]  = tempDATA.UserCMD.COMMAND_DATA.USER_PARA_DOUBLE[i];
    }
    sharedCMD->CommandAccept[target] = false;
    sharedCMD->COMMAND[target].USER_COMMAND = tempDATA.UserCMD.COMMAND_DATA.USER_COMMAND;
}

void CoreWorker::onPODO2GUI(){
    if(serverPODOGUI->RBConnectionState == RBLAN_CS_CONNECTED){
        SendtoGUI();
    }
}

void CoreWorker::onGUI2PODO(){
    if(serverPODOGUI->dataReceived.size() > 0){
        ReadfromGUI();
    }
}

//====================================
void CoreWorker::SendtoVISION()
{
    memcpy(&(DATA_VISION.CoreSEN), sharedSEN,sizeof(RBCORE_SHM_SENSOR));
    QByteArray SendData = QByteArray::fromRawData((char*)&DATA_VISION, sizeof(LAN_PODO2VISION));
    serverPODOVISION->RBSendData(SendData);
}

void CoreWorker::ReadfromVISION(){
    QByteArray tempData = serverPODOVISION->dataReceived[0];
    serverPODOVISION->dataReceived.pop_front();


    LAN_VISION2PODO receivedData;
    memcpy(&receivedData, tempData, sizeof(LAN_VISION2PODO));


    sharedUSER->R2M.Objpos[0].pos_x = receivedData.object_pose.pos_x;
    sharedUSER->R2M.Objpos[0].pos_z = receivedData.object_pose.pos_z;
    sharedUSER->R2M.Objpos[0].pos_y = receivedData.object_pose.pos_y;
    sharedUSER->R2M.Objpos[0].ori_w = receivedData.object_pose.ori_w;
    sharedUSER->R2M.Objpos[0].ori_x = receivedData.object_pose.ori_x;
    sharedUSER->R2M.Objpos[0].ori_y = receivedData.object_pose.ori_y;
    sharedUSER->R2M.Objpos[0].ori_z = receivedData.object_pose.ori_z;

    sharedUSER->R2M.robot_pos.pos_x = receivedData.robot_current_pose.pos_x;
    sharedUSER->R2M.robot_pos.pos_y = receivedData.robot_current_pose.pos_y;
    sharedUSER->R2M.robot_pos.pos_z = receivedData.robot_current_pose.pos_z;
    sharedUSER->R2M.robot_pos.ori_w = receivedData.robot_current_pose.ori_w;
}

void CoreWorker::onVISION2PODO(){
    if(serverPODOVISION->dataReceived.size() > 0){
        ReadfromVISION();
    }
}

void CoreWorker::onPODO2VISION()
{
    if(serverPODOVISION->RBConnectionState == RBLAN_CS_CONNECTED)
    {
        SendtoVISION();
    }
}

//==========================================
void CoreWorker::SendtoROS()
{
    memcpy(&(DATA_ROS.M2R), &(sharedUSER->M2R), sizeof(MOTION2ROS));
    QByteArray SendData = QByteArray::fromRawData((char*)&DATA_ROS, sizeof(LAN_M2R));
    serverROSCMD->RBSendData(SendData);
}

void CoreWorker::ReadfromROS()
{
    QByteArray tempData = serverROSCMD->dataReceived[0];
    serverROSCMD->dataReceived.pop_front();

    LAN_R2M_CMD receivedData;
    memcpy(&receivedData, tempData, sizeof(LAN_R2M_CMD));

    sharedUSER->R2M.ROS_COMMAND = receivedData.ros_cmd;
    sharedUSER->R2M.SELECTED_MENU = receivedData.object_menu;

    FILE_LOG(logWARNING) << "ROS CMD :: " << receivedData.ros_cmd << " MENU :: " << receivedData.object_menu;

}

void CoreWorker::onROS2PODO(){
    if(serverROSCMD->dataReceived.size() > 0)
    {
        ReadfromROS();
    }
}

void CoreWorker::onPODO2ROS()
{
    if(serverROSCMD->RBConnectionState == RBLAN_CS_CONNECTED)
    {
        SendtoROS();
    }
}
