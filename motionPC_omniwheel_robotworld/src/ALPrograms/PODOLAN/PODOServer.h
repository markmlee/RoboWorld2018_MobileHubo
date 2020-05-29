#ifndef PODOSERVER_H
#define PODOSERVER_H

#include <LAN/RBTCPServer.h>
#include <QDataStream>
#include "../../../share/Headers/LANData/RBLANData.h"
#include "../../../share/Headers/LANData/ROSLANData.h"
#include "../../../share/Headers/LANData/VisionLANData.h"


class PODO_GUI_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_GUI_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};


class PODO_VISION_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_VISION_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};

class ROS_CMD_Server : public RBTCPServer
{
    Q_OBJECT
public:
    ROS_CMD_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};


class PODO_ROS_Server : public RBTCPServer
{
    Q_OBJECT
public:
    PODO_ROS_Server();

    QByteArrays dataReceived;
    QByteArray  RBData;

protected slots:
    virtual void    RBReadData();

private:
    int         dataSize;
};

#endif // PODOSERVER_H
