/* =============================================================
 *
 * This Node was developed to communicate ROS and PODO
 *
 * input: subscribe to /bbox3d for object pose
 *        subscribe to /TF for robot localization pose
 * output: LAN TX to podo 
 *
 * E-mail : ml634@kaist.ac.kr (Moonyoung Lee)
 *
 * Versions :
 * v0.2.0 (18.09.06) Connect PODO by TCP/IP
 * =============================================================
 */


/*includes for basic usage*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <math.h> 


/*includes for TCP/IP connetion*/
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/*includes for image and pointcloud topic time sync*/
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

/*includes for path and TF*/
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include  <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>

/*include for RX robot state sensor data type*/
#include "../include/VisionLANData.h"
/*include for TX robot vision data type*/
#include "drc_hubo_sensorpack/Planning_data.h"
#include "drc_hubo_sensorpack/Planning_command.h"

/*include for vision package*/
#include "object_detector/bbox3d.h"

/*TCP/IP for RX robot sensor*/
#define PODO_ADDR "10.12.3.30"
#define PODO_VISION_PORT 5500
#define PODO_CMD_PORT 6000

//receive SHM from motion
LAN_PODO2VISION 	RXdata;
LAN_M2R 			RXcommand;


int tcp_size = 0;
int RXDataSize = 0;
int RXcommandSize = 0;
int sock = 0;
int sock_cmd = 0;
struct sockaddr_in  server;
struct sockaddr_in  server_cmd;
pthread_t LANTHREAD_t;
pthread_t LANTHREAD_t_cmd;
int TXDataSize;
int TXcommandSize;
void* TXBuffer;
void* RXBuffer;
void* TXcommandBuffer;
void* RXcommandBuffer;

int delaycount = 0;
int counterMotion = 0;
int robot_motion_state_counter = 0;


//podo enum command
#define NO_ACT  		0
#define AL_WORKING  	1
#define GOTODISPLAY 	2
#define GOTOGRASPSPOT 	3
#define GOTOHOME  		4
#define GOTODES  		5
#define GRASP  			6
#define PUT 			7
#define ESTOP  			8


int     CreateSocket(const char *addr, int port);
int     Connect2Server();
int     Connect2Server_cmd();
void*   LANThread(void *);
void*   LANThread_cmd(void *);

//global var for motion flag
int robot_motion_state_flag = 0;


/*TX data to PODO_data*/
drc_hubo_sensorpack::Planning_data msg;
/*TX data to PODO_cmd*/
drc_hubo_sensorpack::Planning_command msg_cmd;

/*TX data to podo for localization*/
geometry_msgs::Pose tfPose;

/*publisher for FSM podo status flag */
ros::Publisher motion_status_pub;
ros::Time currentTime, beginTime;

/*callback function from object detection to TX to podo*/
/*input: /bbox3d msg*/
/*output: TX object pose podo*/
void boundingbox_cb (object_detector::bbox3dConstPtr const& input)
{

	std::cout << "@@@ STARTING BBOX CALLBACK @@@" << std::endl;
	beginTime = ros::Time::now();

	//couldnt detect any objects
	if(input->length == 0)
	{
		std::cout << "COULDNT DETECT. NOTHING DONE" << std::endl;

	}

	//non emtpy objects
	else
	{
		//std::cout << "=== handling bounding BOX === " << std::endl;
		/*link parameters to transform camera to robot base TF*/
		float l0 = 77.19/1000;
		float l1 = (31.+133.+1075.+80)/1000;
		float l2 = 5.5 / 1000;
		float l3 = 20./1000.;
		float thetaDegree = -30.;
		float thetaRad = (thetaDegree/180.0)*3.14159;

		float x_c, y_c, z_c, x_c_min, y_c_min, z_c_min, norm, norm_min;
		
		
		if(input->names[0].compare("display") == 0)	
		{

			x_c_min = input->centers[3];
			y_c_min = input->centers[4];
			z_c_min = input->centers[5];

			norm_min = sqrt( pow(input->centers[3],2) + pow(input->centers[5],2) );


		/*copy bbox center parameters to local*/
		//find requested center from user input
		for (int i = 1; i < input->length; i ++)
		{
			//find index of the desired
			//std::cout << "name: " <<  input->names[i] << std::endl;

			if( (input->names[i].compare("sprite") == 0) || (input->names[i].compare("vitamin_water_orange") == 0) || (input->names[i].compare("samdasu") == 0) )
			{
				//calculate norm sqrt(x2+z2)
				norm = sqrt( pow(input->centers[i*3+0],2) + pow(input->centers[i*3+2],2) );

				//return object closest to robot
				if(norm < norm_min)
				{
					x_c_min = input->centers[i*3+0];
			 		y_c_min = input->centers[i*3+1];
			 		z_c_min = input->centers[i*3+2];
			 		norm_min = norm;
				}
			}
		}

		}

		//no display detected in list
		else
		{

			x_c_min = input->centers[0];
			y_c_min = input->centers[1];
			z_c_min = input->centers[2];

			norm_min = sqrt( pow(x_c_min,2) + pow(z_c_min,2) );

		/*copy bbox center parameters to local*/
		//find requested center from user input
		for (int i = 0; i < input->length; i ++)
		{
			//find index of the desired
			//std::cout << "name: " <<  input->names[i] << std::endl;

			if( (input->names[i].compare("sprite") == 0) || (input->names[i].compare("vitamin_water_orange") == 0) || (input->names[i].compare("samdasu") == 0) )
			{
				
				//calculate norm sqrt(x2+z2)
				norm = sqrt( pow(input->centers[i*3+0],2) + pow(input->centers[i*3+2],2) );

				//return object closest to robot (minimum z_c)
				if(norm < norm_min)
				{
					x_c_min = input->centers[i*3+0];
			 		y_c_min = input->centers[i*3+1];
			 		z_c_min = input->centers[i*3+2];
			 		norm_min = norm;
				}

			}
		}

		}

		
		//transform closest object to robot coordinate
		x_c = x_c_min;
		y_c = y_c_min;
		z_c = z_c_min;
		 
		msg.object_pos_x = l0 + (l2 * sin(thetaRad) ) + (l3 * cos(thetaRad)) + (y_c*sin(thetaRad)) + (z_c*cos(thetaRad));
	    msg.object_pos_y = -1*x_c;
	    msg.object_pos_z = l1 + (l2*cos(thetaRad)) - (l3*sin(thetaRad)) - (y_c*cos(thetaRad)) + (z_c*sin(thetaRad));
	    
	    //tuning numbers for better grasp
	    msg.object_pos_y = msg.object_pos_y + 0.02;
		msg.object_pos_z = msg.object_pos_z -0.00;
	    delaycount++;

	    //if samdasu, offset x tuning
	    if(msg_cmd.object_menu == 3)
	    {
	    	msg.object_pos_x = msg.object_pos_x + 0.02;
	    }

	    //if sprite, offset z tuning
	    if(msg_cmd.object_menu == 1)
	    {
	    	msg.object_pos_z = msg.object_pos_z + 0.02;
	    }

	    currentTime = ros::Time::now();
		std::cout << "processing time for bbox callback: " <<  currentTime - beginTime << std::endl;

	    //send at intervals 

		ROS_INFO("base_camera: (%.4f, %.4f. %.4f) -----> base_link: (%.4f, %.4f, %.4f)",
        x_c, y_c, z_c,
        msg.object_pos_x, msg.object_pos_y, msg.object_pos_z);
        write(sock, &msg, TXDataSize);

        //pause to ensure data update before command
        usleep(500000); //500ms

        msg_cmd.robot_command = 6; //grasp hard coded datacommand
		std::cout << "command: " << msg_cmd.robot_command  <<  " menu: " << msg_cmd.object_menu << std::endl;
		write(sock_cmd, &msg_cmd, TXcommandSize);

		
	}//else non-empty bbox 
	
	
}

//update TX msg for object menu
void update_order_cb (const std_msgs::StringConstPtr& str)
{
	//std::cout << "=== handling fsm object input === " << std::endl;
	//std::cout << "data: " << str->data << std::endl;

	if(str->data  == "1")
		msg_cmd.object_menu = 1; 
	else if(str->data  == "2")
		msg_cmd.object_menu = 2; 
	else if(str->data  == "3") 
		msg_cmd.object_menu = 3;
	else
		std::cout<< "" << std::endl;
		//msg_cmd.object_menu = 0;

	// if(str->data  == "sprite")
	// 	msg.object_menu = 1; 
	// else if(str->data  == "vitamin_water_orange")
	// 	msg.object_menu = 2; 
	// else if(str->data  == "samdasoo") 
	// 	msg.object_menu = 3;
	// else
	// 	 msg.object_menu = 0;

}

//update TX msg for command 
void update_command_cb (const std_msgs::Int32ConstPtr& input)
{
	//if grasp command, wait until updated pose flag
	std::cout << "input->data: "  << input->data << std::endl;
	if(input->data == 6)
	{
		std::cout << "GRASP CMD...SKIP: "  << std::endl;
	}

	else
	{

		msg_cmd.robot_command = input->data;
		std::cout << "command: " << msg_cmd.robot_command  <<  "menu: " << msg_cmd.object_menu << std::endl;
		write(sock_cmd, &msg_cmd, TXcommandSize);
	}
	

}

void init_msg(){

	msg_cmd.object_menu = 0;
	msg_cmd.robot_command = 0;

}


int main(int argc, char **argv)
{

    std::cout << "\033[1;32m=======================================" << std::endl << std::endl;
    std::cout << "  Node name   : ROS_PODO_Communication_node" << std::endl << std::endl;
    std::cout << "  version     : 2.0" << std::endl;
    std::cout << "  Author      : MoonYoung Lee (ml634@kaist.ac.kr)" << std::endl;
    std::cout << "=======================================\033[0m" << std::endl;


    init_msg();


    /* Initialize The Node name */
    ros::init(argc, argv, "communication_node");
    /* Define the NodeHandle to communication with ROS system */
	ros::NodeHandle nh;
	/* Loop Cycle = 10Hz = 0.1s */
	ros::Rate loop_rate(10);
	/*TF listener for robot localization*/
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::Quaternion q;
	float yaw;
	float yawDegree;
	int counter = 0;

	// Create a ROS subscriber for the bounding box vision
    ros::Subscriber sub_box = nh.subscribe ("/hubo_bboxes3d", 10, boundingbox_cb, ros::TransportHints().tcpNoDelay());
    // Create a ROS subscriber for updating object input
    ros::Subscriber sub_object = nh.subscribe ("/fsm_order_podo", 1, update_order_cb);
    // Create a ROS subscriber for updating object input
    ros::Subscriber sub_command = nh.subscribe ("/fsm_command", 1, update_command_cb);

    // Create a ROS Publisher for robot motion status rx from podo
    motion_status_pub = nh.advertise<std_msgs::Int32>("/robot_motion_state",1);
    

	
	/*=========== Create Socket for data server ============================================*/

    if(CreateSocket(PODO_ADDR, PODO_VISION_PORT))
    {
        ROS_INFO("Creating Socket for VISION DATA..");

        /*Init buffer to RX robot Data from Motion PC*/
        RXDataSize = sizeof(LAN_PODO2VISION);
        RXBuffer = (void*)malloc(RXDataSize);

        /*Init buffer to TX Planning Data to Motion PC*/
        TXDataSize = sizeof(drc_hubo_sensorpack::Planning_data);
        TXBuffer = (void*)malloc(TXDataSize);
                
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0)
        {
            ROS_ERROR("Create Thread VISION data Error...");
            return 0;
        }
    } else {
        ROS_ERROR("Create VISION Socket Error..");
        ROS_ERROR("Terminate Node...");
        return 0;
    }

    /*=========== Create Socket for command server ============================================*/

    if(CreateSocket(PODO_ADDR, PODO_CMD_PORT))
    {
        ROS_INFO("Creating Socket for CMD..");

        /*Init buffer to RX robot Data from Motion PC*/
        RXcommandSize = sizeof(LAN_M2R);
        RXcommandBuffer = (void*)malloc(RXcommandSize);

        /*Init buffer to TX Planning Data to Motion PC*/
        TXcommandSize = sizeof(drc_hubo_sensorpack::Planning_command);
        TXcommandBuffer = (void*)malloc(TXcommandSize);
        
        
        int threadID_cmd = pthread_create(&LANTHREAD_t_cmd, NULL, &LANThread_cmd, NULL);
        if(threadID_cmd < 0)
        {
            ROS_ERROR("Create Thread VISION CMD Error...");
            return 0;
        }
    } else {
        ROS_ERROR("Create CMD Socket Error..");
        ROS_ERROR("Terminate Node...");
        return 0;
    }


    /*==================================================================*/


    //handle ros communication events
    while(ros::ok())
    {
    	
    	counter++;
    	if(counter > 10){
    		counter = 0;
    		
    		// std::cout << "motion_state_counter: " << robot_motion_state_counter  <<  std::endl;
    		// motion_status_pub.publish(robot_motion_state_counter);
    		// robot_motion_state_counter = robot_motion_state_counter + 1;
    		std::cout << "motion_state_flag: " << robot_motion_state_flag  <<  std::endl;
    		motion_status_pub.publish(robot_motion_state_flag);
    	}


    	try
	    	{
			
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);


		  	}
		  	catch(tf::TransformException& ex)
		  	{
				//ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
			}

			//msg.robot_command = AL_WORKING;
			tfPose.position.x=transform.getOrigin().x();
			tfPose.position.y=transform.getOrigin().y();
			tfPose.position.z=transform.getOrigin().z();
			q = transform.getRotation();
			yaw = tf::getYaw(q);
			yawDegree = yaw*180/3.14;

			// ROS_INFO("x: %.3f, y: %.3f, z: %.3f yaw_robot: %.2f\n",tfPose.position.x,tfPose.position.y,tfPose.position.z, yawDegree);
			msg.robot_pos_x = tfPose.position.x;
			msg.robot_pos_y = tfPose.position.y;
			msg.robot_pos_z = tfPose.position.z;
			msg.robot_ori_w = yawDegree;

			//ROS_INFO("yaw: %f\n",msg.robot_ori_w);
			write(sock, &msg, TXDataSize);

    	
		

		ros::spinOnce();
    	loop_rate.sleep(); // Go to sleep according to the loop period defined above.
        

    }

    
    return 0;
}




int CreateSocket(const char *addr, int port){
    
     if(port == PODO_VISION_PORT) {
    	sock = socket(AF_INET, SOCK_STREAM, 0);
    	if(sock == -1){
        	return false;
    	}

    	server.sin_addr.s_addr = inet_addr(addr);
	    server.sin_family = AF_INET;
	    server.sin_port = htons(port);

	    int optval = 1;
	    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
	    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
	
     }

    if(port == PODO_CMD_PORT) {
    	sock_cmd = socket(AF_INET, SOCK_STREAM, 0);
    	if(sock_cmd == -1){
        	return false;
    	}
    	server_cmd.sin_addr.s_addr = inet_addr(addr);
	    server_cmd.sin_family = AF_INET;
	    server_cmd.sin_port = htons(port);

	    int optval = 1;
	    setsockopt(sock_cmd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
	    setsockopt(sock_cmd, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));	
    }
    

    
    return true;
}

int Connect2Server(){
    /*
     * Connect to PODO server
     */
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (ROS_PODO)" << std::endl;
    return true;
}

int Connect2Server_cmd(){
    /*
     * Connect to PODO server
     */
    if(connect(sock_cmd, (struct sockaddr*)&server_cmd, sizeof(server_cmd)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (ROS_PODO)" << std::endl;
    return true;
}


void* LANThread(void *){

    unsigned int tcp_status = 0x00;
    int connectCnt = 0;

    while(1){
        usleep(100);
        if(tcp_status == 0x00){ // If client was not connected
            if(sock == 0){
                CreateSocket(PODO_ADDR, PODO_VISION_PORT);
            }
            if(Connect2Server()){
                tcp_status = 0x01;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        


        if (tcp_status == 0x01) //connected
        {

            tcp_size = read(sock, RXBuffer, RXDataSize);

            if( tcp_size == RXDataSize)
            {
            	/*copy received rx buffer data*/
                memcpy(&(RXdata), RXBuffer, RXDataSize);
                //std::cout << "FT[LeftHand] data: " << RXdata.CoreSEN.FT[3].Fx << std::endl;


            }
        }
        
    }
    return NULL;
}

void* LANThread_cmd(void *){

    unsigned int tcp_status = 0x00;
    int connectCnt = 0;

    while(1){
        usleep(100);
        if(tcp_status == 0x00){ // If client was not connected
            if(sock_cmd == 0){
                CreateSocket(PODO_ADDR, PODO_CMD_PORT);
            }
            if(Connect2Server_cmd()){
                tcp_status = 0x01;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        


        if (tcp_status == 0x01) //connected
        {

            tcp_size = read(sock_cmd, RXcommandBuffer, RXcommandSize);

            if( tcp_size == RXcommandSize)
            {
            	/*copy received rx buffer data*/
                memcpy(&(RXcommand), RXcommandBuffer, RXcommandSize);
                
                //publish rx podo flag to FSM
                //std::cout << "received RX CMD: " << RXcommand.M2R.MOTION_STATE  << std::endl;
                robot_motion_state_flag = RXcommand.M2R.MOTION_STATE;
                //motion_status_pub.publish(msg_cmd.robot_command); //publishing here causes much delay



            }
        }
        
    }
    return NULL;
}
