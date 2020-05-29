#ifndef CORETHREAD_H
#define CORETHREAD_H

#include <sys/mman.h>
#include <fcntl.h>

#include <QThread>
#include <QByteArray>
#include <QTimer>

#include "PODOServer.h"


class CoreThread : public QThread
{
    Q_OBJECT
public:
    CoreThread();

protected:
    void run();
};


class CoreWorker : public QObject
{
    Q_OBJECT
public:
    CoreWorker();

private slots:
    void onPODO2GUI();
    void onGUI2PODO();

    void onVISION2PODO();
    void onPODO2VISION();

    void onROS2PODO();
    void onPODO2ROS();

private:
    PODO_GUI_Server         *serverPODOGUI;
    PODO_VISION_Server      *serverPODOVISION;
    ROS_CMD_Server          *serverROSCMD;

    LAN_PODO2GUI            DATA_PODO;
    LAN_M2R                 DATA_ROS;
    LAN_PODO2VISION         DATA_VISION;

    void    SendtoGUI();
    void    ReadfromGUI();

    void    SendtoVISION();
    void    ReadfromVISION();

    void    SendtoROS();
    void    ReadfromROS();

};



#endif // CORETHREAD_H
