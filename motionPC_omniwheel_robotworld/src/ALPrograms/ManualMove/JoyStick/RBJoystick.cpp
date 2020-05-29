
#include "joystickclass.h"
#include <stdio.h>
//#include "CommonHeader.h"
using namespace std;


int		numAxis;
int		numButton;
char	nameJoy[80];

RBJoystick::RBJoystick(const QString _devName){
    isTerminated = true;
    connection = false;
    devName = _devName;

    int threadID = pthread_create(&JoyThraedHandler, NULL, &RBJoyThread, this);
    if(threadID < 0){
        cout << ">>> Fail to create JoyThread..!!" << endl;
    }
}

RBJoystick::~RBJoystick(){
    isTerminated = 0; // RB_FAIL;
    usleep(500*1000);
}

int RBJoystick::ConnectJoy(){
    if((fdJoy = open(devName.toStdString().c_str(), O_RDONLY)) == -1){
        //cout << ">>> Fail to open the joystick device..!!" << endl;
        connection = false;
        return 0; //RB_FAIL;
    }else{
        //cout << ">>> Success to open the joystick device..!!" << endl;

        ioctl(fdJoy, JSIOCGAXES, &numAxis);
        ioctl(fdJoy, JSIOCGBUTTONS, &numButton);
        ioctl(fdJoy, JSIOCGNAME(80), &nameJoy);

        printf("%d, %d\n", numAxis, numButton);
        printf("%s\n", nameJoy);
//		numAxis = 6;
//		numButton = 12;
//		JoyAxis = ints(numAxis);
//		JoyButton = chars(numButton);

        fcntl(fdJoy, F_SETFL, O_NONBLOCK);	// use non-blocking methods
        connection = true;
        return 1; //RB_SUCCESS;
    }
}

void *RBJoystick::RBJoyThread(void *_arg){
    RBJoystick *rbJoy = (RBJoystick*)_arg;

    while(rbJoy->isTerminated ==  1){
        if(rbJoy->connection == true){
            // read the joystick
            if(sizeof(struct js_event) == read(rbJoy->fdJoy, &(rbJoy->JoyEvent), sizeof(struct js_event))){
                switch(rbJoy->JoyEvent.type & ~JS_EVENT_INIT){
                case JS_EVENT_AXIS:
                    if(rbJoy->JoyEvent.number < 8)
                        (rbJoy->JoyAxis)[rbJoy->JoyEvent.number] = rbJoy->JoyEvent.value;
                    break;
                case JS_EVENT_BUTTON:
                    if(rbJoy->JoyEvent.number < 12)
                        (rbJoy->JoyButton)[rbJoy->JoyEvent.number] = rbJoy->JoyEvent.value;
                    break;
                }
            }
        }
        usleep(10*1000);
    }
    close(rbJoy->fdJoy);
}

