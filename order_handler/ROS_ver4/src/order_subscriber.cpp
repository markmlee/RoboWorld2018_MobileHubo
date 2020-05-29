#include "ros/ros.h"
#include "android_ros_pkg/orders.h"
#include "android_ros_pkg/workDone.h"

#include <iostream>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio_ext.h>

using namespace std;

int getch(void);
int kbhit(void);
bool msgRecieved = false;


void msgCallback(const android_ros_pkg::orders::ConstPtr& msg)
{
	ROS_INFO("recieved msg = %d", msg->data);
	if(msg->data == 0) msgRecieved = false;
	else msgRecieved = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "order_subscriber");
	ros::NodeHandle nh;

	ros::Subscriber order_subscriber = nh.subscribe("order_msg", 1, msgCallback);
	ros::Publisher order_sub_pub = nh.advertise<android_ros_pkg::workDone>("done_msg", 1);
	
	ros::Rate loop_rate(10);//루프 주기 설정, 10Hz(0.1초 간격)
	android_ros_pkg::workDone doneMsg;
	
	
	while(true){
		cout << endl << "*****************************************" << endl;
		cout << "waiting for the msg from publisher" << endl;
		while(!msgRecieved){
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		
		cout << endl << "waiting for the key input" << endl;
		
		while(true)
		{
			if(kbhit())
			{
				cout <<"keyboard detected: ";
				doneMsg.done = true;
				cout << "Publishing Done message..." << endl;
				order_sub_pub.publish(doneMsg); //메시지 발행
				break;
			}
			
		}
		
		//empty keyboard buffer
		__fpurge(stdin);
		tcflush(0, TCIFLUSH); 		
		
		loop_rate.sleep();
		doneMsg.done = false;
		msgRecieved = false;
		cout << "*****************************************" << endl;
	}
	return 0;
}





//getch 함수
int getch(void){
    int ch;
    struct termios buf, save;
    tcgetattr(0,&save);
    buf = save;
    buf.c_lflag &= ~(ICANON|ECHO);
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    return ch;
}

//kbhit() 함수
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}
