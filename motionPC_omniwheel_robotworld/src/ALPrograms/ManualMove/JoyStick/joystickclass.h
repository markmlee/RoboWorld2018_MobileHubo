#ifndef JOYSTICKCLASS_H
#define JOYSTICKCLASS_H

#include <iostream>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <pthread.h>

#include <QVector>
#include <QString>

// typedef =============================================
//typedef QVector<double>         doubles;
//typedef QVector<int>			ints;
typedef QVector<char>			chars;
typedef unsigned int            uint;

//const int RB_FAIL = 0;
//const int RB_SUCCESS = 1;


class RBJoystick
{
public:
    RBJoystick(const QString _devName = "/dev/input/js0");
    ~RBJoystick();

    int		ConnectJoy();
    int		isConnected()			{return connection;}
    int*    GetAxis()               {return JoyAxis;}
    char*   GetButton()             {return JoyButton;}

    int		JoyAxis[8];
    char	JoyButton[12];

private:
    int		isTerminated;
    int		connection;
    int		fdJoy;


    QString	devName;


//	ints	JoyAxis;
//	chars	JoyButton;

    struct js_event JoyEvent;

    unsigned long	JoyThraedHandler;
    static void* RBJoyThread(void *_arg);
};

#endif // JOYSTICKCLASS_H
