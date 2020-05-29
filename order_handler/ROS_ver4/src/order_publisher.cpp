//ROS headers
#include "ros/ros.h" //ROS 기본 헤더 파일
#include "android_ros_pkg/orders.h" //메시지 파일 헤더(빌드 후 자동 생성됨)
#include "android_ros_pkg/workDone.h"

//SOCKET headers
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio_ext.h>
#include "std_msgs/String.h"

#include <typeinfo>

using namespace std;

//SOCKET variables
const int MAXCONNECTIONS = 5; 		// 최대 접속대기 클라이언트 수
const int MAXWAITBUFSIZE = 4096;	// 대기 버퍼 크기
const int MAXSENDBUFSIZE = 1024;	// SEND용 버퍼의 크기
const int MAXRECEIVEBUFSIZE = 1024;	// RECV용 버퍼의 크기

// 메세지 타입
#define ODSP_COMMAND 1
#define ODSP_REQUEST 2
#define ODSP_ACK 3
#define ODSP_ECHO 4
#define ODSP_END 5

// 메세지 헤더
struct ODSP_HDR
{
    unsigned int msgType;
    unsigned int msgLen;
};

//***** 새로 추가한 변수/함수 *****//
//각 버튼의 click 수
int btn1 = 0, btn2 = 0, btn3 = 0;
//int CLICK_MAX = 10;
int CLICK_MAX = 3;

//esc버튼 입력
char keyboard_in;
int escOut();
int getch();
int kbhit();

bool msgRecieved = false;


//ROS callback 함수
void msgCallback(const android_ros_pkg::workDone::ConstPtr& msg)
{
	//ROS_INFO("recieved msg = %b", msg->done);
	if(msg->done) {msgRecieved = true; ROS_INFO("recieved msg = true");}
	else {msgRecieved = false; ROS_INFO("recieved msg = false");}
}

//ROS callback 함수
void msgCallback_str(const std_msgs::StringConstPtr& str)
{
    //ROS_INFO("recieved msg = %b", msg->done);
    {msgRecieved = true;}
}

int main(int argc, char **argv)
{
	
	//Settings

	//ROS SETTING********************************************************************
	ros::init(argc, argv, "order_publisher"); //노드명 초기화
	ros::NodeHandle nh;   // ROS 시스템과 통신을 위한 노드 핸들 선언
   
	//ROS publisher 선언
	ros::Publisher order_publisher = nh.advertise<android_ros_pkg::orders>("order_msg", 1);
	ros::Publisher order_publisher_str = nh.advertise<std_msgs::String>("/order_msg_str", 10);

	ros::Subscriber order_pub_sub = nh.subscribe("done_msg", 1, msgCallback);
    ros::Subscriber order_pub_sub_str = nh.subscribe("/done_msg_str", 1, msgCallback_str);
	ros::Rate loop_rate(10);//루프 주기 설정, 10Hz(0.1초 간격)
	android_ros_pkg::orders msg;
	std_msgs::String msg_str;

	//*******************************************************************************
	
   
	//SOCKET SETTING*****************************************************************
    
    bool isHeader = true;    	// 버퍼에서 헤더를 분석할 순서 인지를 판별하기위한 변수 
    unsigned int sendByteLen;	// 보낸 데이터 길이
    unsigned int byteLen;		// 받은 데이터, 보낸 데이터 바이트 길이를 받는 변수

    // 데이터를 처리할 수 있는 길이가 되었는지 확인용 변수
    unsigned int curLen; 			// 현재 대기 버퍼안에 있는 데이터의 길이를 저장
    int listenSockFD;   			// listen socket file discriptor    
    int clientSockFD; 				// client socket file discriptor
    char *ptrRecvBufIdx = 0;		// 버퍼의 인덱스 포인터 
    char *ptrDataSortingIdx = 0;	// data 정리용 포인터    
    char *ptrSendBufIdx = 0;		// send buf 용 인덱스 포인터    
    char buf[MAXRECEIVEBUFSIZE];	// RECV용 버퍼 선언    
    char bufSend[MAXSENDBUFSIZE];	// SEND용 버퍼 선언 (헤더 + 메세지)    
    char bufSendMsg[MAXSENDBUFSIZE];// message SEND용 버퍼 선언    
    char bufWait[MAXWAITBUFSIZE];	// 시스템용 대기 버퍼 선언    
    char bufHdr[MAXWAITBUFSIZE];	// 헤더 처리 버퍼    
    char bufMsg[MAXWAITBUFSIZE];	// 메세지 처리 버퍼    
    char bufTemp[MAXWAITBUFSIZE]; 	// 대기 버퍼 스왑용 버퍼
    string Msg = "";    
    sockaddr_in server_addr, client_addr; 	// 소켓 주소 구조체 선언    
    ODSP_HDR hdr;					// 송신용 메세지 헤더 구조체 선언    
    ODSP_HDR *recv_hdr;				// 메세지 헤더 구조체 포인터 선언    
    listenSockFD = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); 	// listen socket 생성
    //*******************************************************************************
	

    if(listenSockFD < 0)
    {
        cout << endl << "socket create error" << endl;
        return 0;
    }

    // 소켓 초기화
    int on = 1;
    if(setsockopt(listenSockFD, SOL_SOCKET, SO_REUSEADDR, (const char*) &on, sizeof(on)) < 0)
    {
        cout << endl  << "set option curLen = 0; error!!" << endl;
        return 0;
    }

    // 서버에서 연결을 수락할 IP 주소 설정
    server_addr.sin_addr.s_addr = INADDR_ANY;   // INADDR_ANY -> 모든 주소 허용
    server_addr.sin_family = AF_INET;    		// AF_INET ->  ipv4
    server_addr.sin_port = htons(5000);    		// 포트 셋팅

    // listen socket에 주소 할당
    if(bind(listenSockFD, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
    {
        cout << endl << "bind error" << endl;
        return 0;
    }

    cout << "server running. waiting client..." << endl;

    // passive 대기 상태로 client의 접속을 대기
    // client 에서 connect를 통해 접속 시도 시 3-way handshake 가 일어남
    if(listen(listenSockFD, MAXCONNECTIONS) < 0)
    {
        cout << endl << "listen error" << endl;
        return 0;
    }

    int clientAddrSize =  sizeof(client_addr);
	
    while(true)
    {

        // 대기버퍼에 있는 데이터 길이 초기
        curLen = 0;

        // 대기 버퍼 초기화
        memset(bufWait, 0, MAXWAITBUFSIZE);

        // 버퍼 포인터 초기화
        // 대기 버퍼의 시작 주소를 가진다.
        ptrRecvBufIdx = bufWait;

        // client에 대해 허용
        clientSockFD = accept(listenSockFD, (struct sockaddr *) &client_addr, (socklen_t *) &clientAddrSize);

        if(clientSockFD < 0)
        {
            cout << endl << "accept error" << endl;
            return 0;
        }

		if(escOut()) break;
        cout << endl << "***********************";
        cout << endl << "client accepted" << endl;
        cout << "address : " << inet_ntoa(client_addr.sin_addr) << endl;
        cout << "port : " << ntohs(client_addr.sin_port) << endl;
		
		char buffer[10] = "ready \n";
				
        // data 수신 처리
        while(true)
        {            
            // receive 버퍼 초기화
            memset(buf, 0, MAXRECEIVEBUFSIZE);

            // data를 가져옴
               // 최대 버퍼 사이즈 만큼의 데이터를 가져와서 buf에 저장
            // recv 함수는 가져온 byte의 길이를 return
            byteLen = recv(clientSockFD, buf, MAXRECEIVEBUFSIZE, 0);
			
			//empty keyboard buffer
			__fpurge(stdin);
			tcflush(0, TCIFLUSH); 
			
			char buf_first_char[10];
			strncpy(buf_first_char,buf,1);
            
            int temp = atoi(buf);
            
            //switch(atoi(buf))
            switch(atoi(buf_first_char))
            {
                case 1:
                    if(btn1 < CLICK_MAX){
                        btn1++;
                        cout << endl << "DRINK 1 was ordered " << btn1 << " time(s)" << endl;
                        msg.data = 1;

                        std::stringstream ss;
						ss << "1";
						msg_str.data = ss.str();
    
                        ROS_INFO("send msg = %d", msg.data);
						order_publisher.publish(msg); //메시지 발행
						order_publisher_str.publish(msg_str);
						
						cout << endl << "Waiting for DONE message..." << endl;
						while(!msgRecieved){
							ros::spinOnce();
							if(escOut()) break;
						}
						msgRecieved = false;
						//buffer = "ready \n";;					
						strcpy(buffer,"ready \n");
						if(btn1 == CLICK_MAX) strcpy(buffer,"soldout \n");
						write(clientSockFD, buffer, strlen(buffer));									
                    }
                    else{
                        cout << endl << "Drink 1 is SOLD OUT" << endl; 			
						strcpy(buffer,"soldout \n");				
						write(clientSockFD, buffer, strlen(buffer));
                    }
                    break;

                case 2:
                    if(btn2 < CLICK_MAX){
                        btn2++;
                        cout << endl << "DRINK 2 was ordered " << btn2 << " time(s)" << endl;
                        msg.data = 2;
                        
                        std::stringstream ss2;
						ss2 << "2";
						msg_str.data = ss2.str();
    
                        ROS_INFO("send msg = %d", msg.data);
						order_publisher.publish(msg); //메시지 발행
						order_publisher_str.publish(msg_str);
						                   			
						cout << endl << "Waiting for DONE message..." << endl;
						while(!msgRecieved){
							ros::spinOnce();
							if(kbhit()) break;
						}
						msgRecieved = false;			
						strcpy(buffer,"ready \n");
						if(btn2 == CLICK_MAX) strcpy(buffer,"soldout \n");					
						write(clientSockFD, buffer, strlen(buffer));	
                    }
                    else{
                        cout << endl << "Drink 2 is SOLD OUT" << endl; 			
					strcpy(buffer,"soldout \n");					
						write(clientSockFD, buffer, strlen(buffer));
					}
                    break;

                case 3:
                    if(btn3 < CLICK_MAX){
                        btn3++;
                        cout << endl << "DRINK 3 was ordered " << btn3 << " time(s)" << endl;
                        msg.data = 3;
                        
                        std::stringstream ss3;
						ss3 << "3";
						msg_str.data = ss3.str();
    
                        ROS_INFO("send msg = %d", msg.data);
						order_publisher.publish(msg); //메시지 발행
						order_publisher_str.publish(msg_str);
						
						cout << endl << "Waiting for DONE message..." << endl;
						while(!msgRecieved){
							ros::spinOnce();
							if(kbhit()) break;
						}
						msgRecieved = false;			
						strcpy(buffer,"ready \n");
						if(btn3 == CLICK_MAX) strcpy(buffer,"soldout \n");					
						write(clientSockFD, buffer, strlen(buffer));
                    }
                    else{
                        cout << endl << "Drink 3 is SOLD OUT" << endl; 			
						strcpy(buffer,"soldout \n");									
						write(clientSockFD, buffer, strlen(buffer));
					}
                    break;
                case 4:
					btn1 = 0; btn2 = 0; btn3 = 0;
					cout << endl << "RESET" << endl;			
					strcpy(buffer,"ready \n");					
					write(clientSockFD, buffer, strlen(buffer));
					break;
				case 5:
					close(listenSockFD);
					cout << "Server End" << endl;
					return 0;
					break;
				case 7:
					break;
				case 8:
					strcpy(buffer," ");
					char s[2];
					sprintf(s,"%d",CLICK_MAX-btn1); strcpy(buffer, s);
					sprintf(s,"%d",CLICK_MAX-btn2); strcat(buffer, s);
					sprintf(s,"%d",CLICK_MAX-btn3); strcat(buffer, s);
					strcat(buffer," \n");
					cout<< "buffer to Client: " << buffer << endl;			
					write(clientSockFD, buffer, strlen(buffer));
					break;
				case 9:
					//cout << endl << "Case 9" << endl;
					char temp[2];
					strncpy(temp,buf+1,1); btn1 = CLICK_MAX - atoi(temp);
					strncpy(temp,buf+2,1); btn2 = CLICK_MAX - atoi(temp);
					strncpy(temp,buf+3,1); btn3 = CLICK_MAX - atoi(temp);
					cout<< "btn1 = " << btn1 << endl;
					cout<< "btn2 = " << btn2 << endl;
					cout<< "btn3 = " << btn3 << endl;
					break;                    
                default:
					//cout << endl << "Default Case" << endl;
                    break;

            }
            
            
            
            if(byteLen == 0)
            {
                cout << endl << "client " << inet_ntoa(client_addr.sin_addr) << " closed." << endl;
                cout << "***********************" << endl;
                close(clientSockFD);
                break;
            }

            if(byteLen > MAXRECEIVEBUFSIZE)
            {
                cout << endl << "client " << inet_ntoa(client_addr.sin_addr) << " closed." << endl;
                close(clientSockFD);
                break;
            }

            if(byteLen > 0)
            {
                memcpy(ptrRecvBufIdx, buf, byteLen);
                curLen += byteLen;
                ptrRecvBufIdx += byteLen;
                byteLen = 0;

                // 헤더를 분석할 순서
                if(isHeader)
                {
                    // 헤더 길이 이상의 data가 있는지 확인
                    if(curLen >= sizeof(recv_hdr))
                    {
                        memset(bufHdr, 0, sizeof(bufHdr));
                        memcpy(bufHdr, bufWait, sizeof(recv_hdr));
                        bufHdr[sizeof(recv_hdr)] = '\0';
                        recv_hdr = (ODSP_HDR *)bufHdr;

                        // 버퍼의 data를 정리
                        ptrDataSortingIdx = bufWait + sizeof(recv_hdr);
                        memset(bufTemp, 0, sizeof(bufTemp));
                        memcpy(bufTemp, ptrDataSortingIdx, sizeof(bufWait) - sizeof(recv_hdr));
                        memset(bufWait, 0, sizeof(bufWait));
                        memcpy(bufWait, bufTemp, sizeof(bufTemp));

                        curLen -= sizeof(recv_hdr);
                        ptrRecvBufIdx -= sizeof(recv_hdr);
                        isHeader = false;
                    }
                }

                if(!isHeader)
                {
                    // 분석된 헤더의 메세지 길이 정보를 통해 현재 버퍼에 메세지 길이 이상의 data가 있는지 확인
                    if(curLen >= recv_hdr->msgLen)
                    {
                        memset(bufMsg, 0, sizeof(bufMsg));
                        memcpy(bufMsg, bufWait, recv_hdr->msgLen);
                        bufMsg[recv_hdr->msgLen] = '\0';

                        // 버퍼의 data를 정리
                        ptrDataSortingIdx = bufWait + recv_hdr->msgLen;
                        memset(bufTemp, 0, sizeof(bufTemp));
                        memcpy(bufTemp, ptrDataSortingIdx, sizeof(bufWait) - recv_hdr->msgLen);
                        memset(bufWait, 0, sizeof(bufWait));
                        memcpy(bufWait, bufTemp, sizeof(bufTemp));

                        curLen -= recv_hdr->msgLen;
                        ptrRecvBufIdx -= recv_hdr->msgLen;
                        isHeader = true;

                        if(recv_hdr->msgType == ODSP_COMMAND)
                        {
                            cout << endl << "received command message" << endl;
                            cout << "message length is " << recv_hdr->msgLen << "byte" << endl;
                            cout << "receive message : " << bufMsg << endl;

                            Msg = "received command message";
                            strcpy(bufSendMsg, Msg.c_str());

                            memset(&hdr, 0, sizeof(hdr));
                            hdr.msgType = ODSP_ACK;
                            hdr.msgLen = Msg.length();

                            ptrSendBufIdx = bufSend;
                            memcpy(ptrSendBufIdx, &hdr, sizeof(hdr));
                            ptrSendBufIdx = ptrSendBufIdx + sizeof(hdr);
                            memcpy(ptrSendBufIdx, bufSendMsg, hdr.msgLen);

                            sendByteLen = send(clientSockFD, bufSend, sizeof(hdr) + hdr.msgLen, 0);

                            if(sendByteLen < 0)
                            {
                                cout << endl << "send error" << endl;
                                return 0;
                            }
                        }
                        else if(recv_hdr->msgType == ODSP_REQUEST)
                        {
                            cout << endl << "received request message" << endl;
                            cout << "message length is " << recv_hdr->msgLen << "byte" << endl;
                            cout << "receive message : " << bufMsg << endl;
                        }
                        else if(recv_hdr->msgType == ODSP_ACK)
                        {
                            cout << endl << "received ack message" << endl;
                            cout << "message length is " << recv_hdr->msgLen << "byte" << endl;
                            cout << "receive message : " << bufMsg << endl;
                        }
                        else if(recv_hdr->msgType == ODSP_ECHO)
                        {
                            cout << endl << "received echo message" << endl;
                            cout << "message length is " << recv_hdr->msgLen << "byte" << endl;
                            cout << "receive message : " << bufMsg << endl;

                            Msg = bufMsg;
                            strcpy(bufSendMsg, Msg.c_str());

                            memset(&hdr, 0, sizeof(hdr));
                            hdr.msgType = ODSP_ECHO;
                            hdr.msgLen = recv_hdr->msgLen;

                            ptrSendBufIdx = bufSend;
                            memcpy(ptrSendBufIdx, &hdr, sizeof(hdr));
                            ptrSendBufIdx = ptrSendBufIdx + sizeof(hdr);
                            memcpy(ptrSendBufIdx, bufSendMsg, hdr.msgLen);

                            sendByteLen = send(clientSockFD, bufSend, sizeof(hdr) + hdr.msgLen, 0);

                            if(sendByteLen < 0)
                            {
                                cout << endl << "send error" << endl;
                                return 0;
                            }
                        }
                        else if(recv_hdr->msgType == ODSP_END)
                        {
                            cout << endl << "received end message" << endl;
                            cout << "message length is " << recv_hdr->msgLen << "byte" << endl;
                            cout << "receive message : " << bufMsg << endl;
                        }
                        else
                        {
                            cout << endl << "received unknown message" << endl;
                            cout << "exit server" << endl;
                            close(clientSockFD);
                            break;
                        }
                    }
                }
            } //byteLen > 0	
			
		}
		__fpurge(stdin);
		tcflush(0, TCIFLUSH); 
		if(escOut()) break;
		
   }


   close(listenSockFD);

   cout << "server end" << endl;

   return 0;

}

int escOut()
{
	//ESC 버튼 입력 받으면 1 return
	if(kbhit())
	{
		if(!(keyboard_in = getch()))
		{
			keyboard_in	= getch();
			cout << endl << "keyboard_in = " << keyboard_in	<<endl;
		}
		if(keyboard_in == 27)
			return 1;
	}
	return 0;

}

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


