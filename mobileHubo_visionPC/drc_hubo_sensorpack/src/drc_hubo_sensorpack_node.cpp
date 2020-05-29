/* =============================================================
 *
 * This node receives vision & robot sensor information and time syncs
 * Vision data from subscribing to topic
 * Robot data from TCP/IP with Motion PC
 *
 * Output : pointcloud2, RGB, robot imu (time synced)
 *        : Path TX to Motion PC
 * Input  : pointcloud2, RGB, robot imu
 *        : cmd RX from Operating PC
 * 
 * E-mail : ml634@kaist.ac.kr (Moonyoung Lee)
 *
 * Versions :
 * v1.0.2018.07
 * =============================================================
 */

/*includes for basic usage*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h> 


/*includes for TCP/IP connetion*/
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>

/*includes for image and pointcloud topic time sync*/
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"

/*includes for communicating operating PC*/
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include  <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>

/*include for robot sensor data type*/
#include "../include/RBLANData.h"

/*include for Planning data TX to PODO(Motion PC)*/
//#include "drc_hubo_sensorpack/Planning_data.h"
//#include "drc_hubo_sensorpack/Planning_dataArray.h"

/*TCP/IP for RX robot sensor*/
#define PODO_ADDR "10.12.3.30"
#define PODO_PORT 5500
#define SEND_DATA_FALSE 0
#define SEND_DATA_TRUE 1
#define FINISHED_PATH_SIZE 2
#define NEXT_PATH_STEPSIZE 1

/*For path plan feature detect*/
#define PEAK_INDEX 0.02

int sock = 0;
struct sockaddr_in  server;
pthread_t LANTHREAD_t;

int RXDataSize;
int TXDataSize;
void* TXBuffer;
void* RXBuffer;

LAN_PODO2VISION RXdata;

/*imu topics*/
sensor_msgs::Imu imu;
geometry_msgs::Quaternion imu_orientation;
geometry_msgs::Vector3 imu_angular_velocity;
geometry_msgs::Vector3 imu_linear_acceleration;

/*lidar and camera topics*/
sensor_msgs::PointCloud2 lidar_pcloud;
sensor_msgs::PointCloud2 realsense_pcloud;
sensor_msgs::Image zed_rgb;

/*lidar and camera topics time synced*/
sensor_msgs::PointCloud2 lidar_pcloud_sync;
sensor_msgs::PointCloud2 realsense_pcloud_sync;
sensor_msgs::Image realsense_rgb_sync;
sensor_msgs::PointCloud2 zed_pcloud_sync;
sensor_msgs::Image zed_rgb_sync;

/*planning msg TX to PODO*/
//drc_hubo_sensorpack::Planning_data planning_msg;
//drc_hubo_sensorpack::Planning_dataArray planning_msg_array;


int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void *);
void    RXpodoData();
void    set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg); // set start pose, which is current robot pose
void	globalPathtoMotion();


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;
ros::Publisher imu_pub;
ros::Publisher realsense_pub;
ros::Publisher zed_rgb_pub;
ros::Publisher lidar_pub;

/*time synced publisher*/
ros::Publisher realsenseRGBD_sync_pub;
ros::Publisher realsenseRGB_sync_pub;
ros::Publisher  zedRGBD_sync_pub;
ros::Publisher  zedRGB_sync_pub;
ros::Publisher lidar_sync_pub;
ros::Publisher pubArray;


/*constants*/
float D2R = 0.0174533f;
float R2D = 57.2957802f;

/*global var*/
ros::Time currentTime, beginTime;
int FPScount = 0;
int isFirstPath = 0;
int feature_pose_index = 0;
int startupCounter = 0;
int motionDelayCounter = 900;

geometry_msgs::Pose robot_current_pose;
nav_msgs::Path feature_path;
		
/*callback to set robot current pose*/
void set_start_pose(const tf2_msgs::TFMessage::ConstPtr& _msg)
{
	startupCounter++;
	
    // Convert tf2_msgs to geometry_msgs::Pose
    for(int i = 0; i < _msg->transforms.size(); i++){
        if(std::string("base_link").compare(_msg->transforms.at(i).child_frame_id) == 0){
            robot_current_pose.position.x = _msg->transforms.at(i).transform.translation.x;
            robot_current_pose.position.y = _msg->transforms.at(i).transform.translation.y;
            robot_current_pose.position.z = 0.0;

            robot_current_pose.orientation.w = _msg->transforms.at(i).transform.rotation.w;
            robot_current_pose.orientation.x = _msg->transforms.at(i).transform.rotation.x;
            robot_current_pose.orientation.y = _msg->transforms.at(i).transform.rotation.y;
            robot_current_pose.orientation.z = _msg->transforms.at(i).transform.rotation.z;

            break;
        }
    }
/*
    planning_msg.robot__pos_x = robot_current_pose.position.x;
    planning_msg.robot__pos_y = robot_current_pose.position.y;
    planning_msg.robot__pos_z = robot_current_pose.position.z;
    planning_msg.robot__ori_w = robot_current_pose.orientation.w;
    planning_msg.robot__ori_x = robot_current_pose.orientation.x;
    planning_msg.robot__ori_y = robot_current_pose.orientation.y;
    planning_msg.robot__ori_z = robot_current_pose.orientation.z;
*/

	//write msg TX
	//write(sock, &planning_msg, TXDataSize);


//    std::cout << "Set start pose" << std::endl;

	//go to path
	//wait for other values 
	if(startupCounter > 100) {
		
		motionDelayCounter++;
		
		if(motionDelayCounter%100 == 0) {printf("motionDelayCounter: %i\n", motionDelayCounter);}
		
		//TX motion call only periodically
		if(motionDelayCounter > 300) {
			//globalPathtoMotion();
			motionDelayCounter = 0;
		}
	}
}


/*callback for camera pointcloud data*/
void callbackCameraPointCloud(const sensor_msgs::PointCloud2ConstPtr& pCloudIn)
{
    realsense_pcloud.header.stamp = ros::Time::now();

    /* =============================================================
     *
     * add realsense callback code here
     * =============================================================
     */

    realsense_pcloud = *pCloudIn;

    realsense_pub.publish(realsense_pcloud);

}

/*callback for lidar pointcloud data*/
void callbackLidarPointCloud(const sensor_msgs::PointCloud2ConstPtr& pCloudIn)
{

    lidar_pcloud.header.stamp = ros::Time::now();
    /* =============================================================
     *
     * add lidar callback code here
     * =============================================================
     */
    lidar_pcloud = *pCloudIn;

    lidar_pub.publish(lidar_pcloud);
    
   
    
}

/*callback for time synced data {camera, lidar, imu} but low {2-10} FPS */
void callback(const sensor_msgs::PointCloud2ConstPtr& realsenseRGBDIn, const sensor_msgs::Image::ConstPtr& realsenseRGBIn, const sensor_msgs::PointCloud2ConstPtr& zedRGBDIn, const sensor_msgs::Image::ConstPtr& zedRGBIn, const sensor_msgs::PointCloud2ConstPtr& lidarIn)
{
	

    //determine FPS
    currentTime = ros::Time::now();
    ros::Duration durationTime = currentTime - beginTime;
    FPScount++;
    if (durationTime.toSec() > 1.0) {
        ROS_INFO("FPS of lidar,camera,imu sync: %d\n", FPScount);
        FPScount = 0;
        beginTime = currentTime;

    }


	//publish subscribed input according to time sync
	realsense_pcloud_sync = *realsenseRGBDIn;
    realsenseRGBD_sync_pub.publish(realsense_pcloud_sync);
    realsense_rgb_sync = *realsenseRGBIn;
    realsenseRGB_sync_pub.publish(realsense_rgb_sync);
    
    zed_pcloud_sync = *zedRGBDIn;
    zedRGBD_sync_pub.publish(zed_pcloud_sync);
    zed_rgb_sync = *zedRGBIn;
    zedRGB_sync_pub.publish(zed_rgb_sync);
    
    lidar_pcloud_sync = *lidarIn;
    lidar_sync_pub.publish(lidar_pcloud_sync);
    

}


void globalPathtoMotion() 
{
	
	
	//goal path based on global origin
	float currentXworld, nextXworld, currentYworld, nextYworld, currentYaw, nextYaw;
	float deltaXworld, deltaYworld, deltaYawworld;
	//robot move command based on current robot origin 
	float deltaXrobot, deltaYrobot, deltaYawrobot;
	
	for (int i = 0; i <feature_path.poses.size()-1; i++) 
	{
		//if iterating thru one last time to compensate for yaw error
		//if(feature_pose_index == feature_path.poses.size()) feature_pose_index=feature_path.poses.size()-1;
		
		//get current pose
		currentXworld = feature_path.poses.at(i).pose.position.x;
		currentYworld = feature_path.poses.at(i).pose.position.y;
		
		tf::Quaternion qCurrent(feature_path.poses.at(i).pose.orientation.x, feature_path.poses.at(i).pose.orientation.y, feature_path.poses.at(i).pose.orientation.z, feature_path.poses.at(i).pose.orientation.w);
		tf::Matrix3x3 mCurrent(qCurrent);
		double  rollCurrent, pitchCurrent, yawCurrent;
		mCurrent.getRPY(rollCurrent, pitchCurrent, yawCurrent);
		
		currentYaw = yawCurrent ; //in degrees 2 rad
		
		//get difference between current pose and featurePose[index]
		//convert quaternion to RPY
		
		tf::Quaternion q(feature_path.poses.at(i+1).pose.orientation.x, feature_path.poses.at(i+1).pose.orientation.y, feature_path.poses.at(i+1).pose.orientation.z, feature_path.poses.at(i+1).pose.orientation.w);
		tf::Matrix3x3 m(q);
		double  roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		
		nextXworld = feature_path.poses.at(i+1).pose.position.x;
		nextYworld = feature_path.poses.at(i+1).pose.position.y;
		
		//nextXworld = feature_path.poses.at(feature_pose_index).pose.position.x;
		//nextYworld = feature_path.poses.at(feature_pose_index).pose.position.y;
		nextYaw = yaw;
		
		//determine difference in X,Y,Yaw
		deltaXworld = nextXworld - currentXworld;
		deltaYworld = nextYworld - currentYworld;
		deltaYawworld =  nextYaw - currentYaw;
		
	/*
		//if difference small --> index++			
		if( abs(deltaXworld) <= 0.1 && abs(deltaXworld) <= 0.1) 
		{
			 if(feature_pose_index < feature_path.poses.size()-1){
				feature_pose_index++;
				} 
			}
		*/
		
		//TX robot motion (every 5 sec)
		//Transform global delta X,Y to robot X,Y origin
		deltaXrobot = (cos(currentYaw) * deltaXworld) + (sin(currentYaw) * deltaYworld); 
		deltaYrobot = (-sin(currentYaw) * deltaXworld) + (cos(currentYaw) * deltaYworld); 
		deltaYawrobot = deltaYawworld*R2D;

		//TX to PODO motion PC
				
				/*
		planning_msg.path_plan_Xrobot = deltaXrobot;
		planning_msg.path_plan_Yrobot = deltaYrobot;
		planning_msg.path_plan_Yawrobot = deltaYawrobot;
		*/
		ROS_INFO("Current Pose (X,Y): (%f, %f) FeaturePose[X,Y,index] :  [%f, %f, %i]\n", currentXworld, currentYworld, nextXworld, nextYworld, feature_pose_index);
				
		//ROS_INFO("==> robotX Y Yaw : %f, %f, %f, %i\n", planning_msg.path_plan_Xrobot, planning_msg.path_plan_Yrobot, planning_msg.path_plan_Yawrobot, planning_msg.flag_done  );
	}
	
	
	
	
	if(feature_pose_index < feature_path.poses.size()) { 1; }//1
	else {
		/*
		//finished last pose
		if(feature_pose_index == feature_path.poses.size()) 
		{
			//make only orientation exact as goal
			planning_msg.path_plan_Xrobot = 0;
			planning_msg.path_plan_Yrobot = 0;
			planning_msg.path_plan_Yawrobot = deltaYawrobot;
			planning_msg.flag_done = SEND_DATA_TRUE; //1 one last pivot
			
		}
			
		else 
		{
			planning_msg.path_plan_Xrobot = 0;
			planning_msg.path_plan_Yrobot = 0;
			planning_msg.flag_done = SEND_DATA_FALSE;
		}*/
		
	}
	
			
	

	//ROS_INFO("===========================\n");
	//write msg TX
	//write(sock, &planning_msg, TXDataSize);
	feature_pose_index++;

			
}


/*callback for operating pc published msg to TX to PODO motion PC*/
void opcCallback(const nav_msgs::Path::ConstPtr& msgOPC)
{
				
	//done condition: path array size is significantly small
    if ( (msgOPC->poses.size() < FINISHED_PATH_SIZE) ) {
				
			//planning_msg.flag_done = SEND_DATA_FALSE; //0
			ROS_INFO("Arrived at goal pose \n");
	}
		
	//send path to motion PC
	else {
		
		if(isFirstPath < 1) {
			
		//create new variables every time new path is given
		float xdot[msgOPC->poses.size()-1];
		float ydot[msgOPC->poses.size()-1];
		float vdot[msgOPC->poses.size()-1];
		float theta[msgOPC->poses.size()];
		float thetadot[msgOPC->poses.size()-1];
		float vdd[msgOPC->poses.size()-2];
		int point[msgOPC->poses.size()-2]; //only partially used
		int indexCounter =0;
		
		
		geometry_msgs::PoseStamped path_node;
		
		//create theta array
		for(int i = 0; i < msgOPC->poses.size(); i++){
				//convert quaternion to RPY
				tf::Quaternion q( msgOPC->poses.at(i).pose.orientation.x,  msgOPC->poses.at(i).pose.orientation.y,  msgOPC->poses.at(i).pose.orientation.z,  msgOPC->poses.at(i).pose.orientation.w);
				tf::Matrix3x3 m(q);
				double  roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);
				
				theta[i] = yaw*180/3.14  ;
			}
		
		//get xdot, ydot, vdot
		for(int i = 0; i < msgOPC->poses.size()-1; i++){
			
			xdot[i] = msgOPC->poses.at(i+1).pose.position.x - msgOPC->poses.at(i).pose.position.x;
			ydot[i] = msgOPC->poses.at(i+1).pose.position.y - msgOPC->poses.at(i).pose.position.y;
			vdot[i] = sqrt( pow(xdot[i],2)+ pow(ydot[i],2) );
			
			thetadot[i] = theta[i+1] - theta[i];
		}
		
		
		//print test
		for(int i = 0; i < sizeof(thetadot)/sizeof(*thetadot); i++){
			
			//printf("thetadot: %f, i: %i, size: %i\n",thetadot[i], i, sizeof(thetadot)/sizeof(*thetadot));
		}
		
		
		
		
		//get vdd
		for(int i = 0; i < msgOPC->poses.size()-2; i++){
			
			vdd[i] = vdot[i+1] - vdot[i];		
			
			if(vdd[i] > PEAK_INDEX || vdd[i] < -PEAK_INDEX || abs(thetadot[i]) >=45 ) {point[indexCounter] = i+1; indexCounter++;}
	
		
		}
		
		//print test
		for(int i = 0; i < sizeof(vdd)/sizeof(*vdd); i++){
			
			//printf("vdd: %f, i: %i, size: %i\n",vdd[i], i, sizeof(vdd)/sizeof(*vdd));
		}
		
		//new pose array containing only features using point index
		for(int i = 0; i < indexCounter; i++){
			
			//add original starting pose to new array
			if(i == 0) {
				path_node.pose.position.x = msgOPC->poses.at(0).pose.position.x;
				path_node.pose.position.y = msgOPC->poses.at(0).pose.position.y;
				path_node.pose.position.z = msgOPC->poses.at(0).pose.position.z;			
				path_node.pose.orientation.x = msgOPC->poses.at(0).pose.orientation.x;
				path_node.pose.orientation.y = msgOPC->poses.at(0).pose.orientation.y;
				path_node.pose.orientation.z = msgOPC->poses.at(0).pose.orientation.z;
				path_node.pose.orientation.w = msgOPC->poses.at(0).pose.orientation.w;
				
				feature_path.poses.push_back(path_node);
				}
				
			path_node.pose.position.x = msgOPC->poses.at(point[i]).pose.position.x;
			path_node.pose.position.y = msgOPC->poses.at(point[i]).pose.position.y;
			path_node.pose.position.z = msgOPC->poses.at(point[i]).pose.position.z;
			path_node.pose.orientation.x = msgOPC->poses.at(point[i]).pose.orientation.x;
			path_node.pose.orientation.y = msgOPC->poses.at(point[i]).pose.orientation.y;
			path_node.pose.orientation.z = msgOPC->poses.at(point[i]).pose.orientation.z;
			path_node.pose.orientation.w = msgOPC->poses.at(point[i]).pose.orientation.w;
			
			feature_path.poses.push_back(path_node);
			
			//add original end pose to new array
			if(i == indexCounter-1) {
				path_node.pose.position.x = msgOPC->poses.at(msgOPC->poses.size()-1).pose.position.x;
				path_node.pose.position.y = msgOPC->poses.at(msgOPC->poses.size()-1).pose.position.y;
				path_node.pose.position.z = msgOPC->poses.at(msgOPC->poses.size()-1).pose.position.z;			
				path_node.pose.orientation.x = msgOPC->poses.at(msgOPC->poses.size()-1).pose.orientation.x;
				path_node.pose.orientation.y = msgOPC->poses.at(msgOPC->poses.size()-1).pose.orientation.y;
				path_node.pose.orientation.z = msgOPC->poses.at(msgOPC->poses.size()-1).pose.orientation.z;
				path_node.pose.orientation.w = msgOPC->poses.at(msgOPC->poses.size()-1).pose.orientation.w;
				
				feature_path.poses.push_back(path_node);
				}
				
		}
		
		// print original array. 
		/*
			for(int i = 0; i < msgOPC->poses.size(); i++){
				//convert quaternion to RPY
				tf::Quaternion q( msgOPC->poses.at(i).pose.orientation.x,  msgOPC->poses.at(i).pose.orientation.y,  msgOPC->poses.at(i).pose.orientation.z,  msgOPC->poses.at(i).pose.orientation.w);
				tf::Matrix3x3 m(q);
				double  roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);
				
				//ROS_INFO("=== original == ith X Y Yaw: %i, %f, %f, %f\n", i, msgOPC->poses.at(i).pose.position.x, msgOPC->poses.at(i).pose.position.y, yaw*180/3.14  );
			}
			*/
			
			
			for(int i = 0; i < feature_path.poses.size(); i++){
		
				//convert quaternion to RPY
				tf::Quaternion q(feature_path.poses.at(i).pose.orientation.x, feature_path.poses.at(i).pose.orientation.y, feature_path.poses.at(i).pose.orientation.z, feature_path.poses.at(i).pose.orientation.w);
				tf::Matrix3x3 m(q);
				double  roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);
							
				ROS_INFO("Feat == ith X Y Yaw: %i, %f, %f, %f\n", i, feature_path.poses.at(i).pose.position.x, feature_path.poses.at(i).pose.position.y, yaw*180/3.14  );
				
			
			}
		
			isFirstPath++;
			
		} //only once

	
	} //else path small
	
	
	
	
     
}



int CreateSocket(const char *addr, int port){
    /* Create Socket with TCP
     * AF_INET = Internet Protocol (TCP/IP)
     * SOCK_STREAM(TCP), SOCK_DGRAM(UDP), SOCK_RAW(RAW) */
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

    return true;
}

int Connect2Server(){
    /*Connect to PODO server */
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (data_grab)" << std::endl;
    return true;
}

void RXpodoData(){

    imu.header.stamp = ros::Time::now();

    /*assign RPY to quaternion*/
    double cy = cos((RXdata.CoreData.IMU[0].Yaw)*D2R * 0.5);
    double sy = sin((RXdata.CoreData.IMU[0].Yaw)*D2R * 0.5);
    double cr = cos((RXdata.CoreData.IMU[0].Roll)*D2R * 0.5);
    double sr = sin((RXdata.CoreData.IMU[0].Roll)*D2R * 0.5);
    double cp = cos((RXdata.CoreData.IMU[0].Pitch)*D2R * 0.5);
    double sp = sin((RXdata.CoreData.IMU[0].Pitch)*D2R * 0.5);

    imu_orientation.w = cy * cr * cp + sy * sr * sp;
    imu_orientation.x = cy * sr * cp - sy * cr * sp;
    imu_orientation.y = cy * cr * sp + sy * sr * cp;
    imu_orientation.z = sy * cr * cp - cy * sr * sp;

    /*assign velocity*/
    imu_angular_velocity.x =  RXdata.CoreData.IMU[0].RollVel  * D2R;
    imu_angular_velocity.y =  RXdata.CoreData.IMU[0].PitchVel * D2R;
    imu_angular_velocity.z =  RXdata.CoreData.IMU[0].YawVel   * D2R;

    /*assign acceleration*/
    imu_linear_acceleration.x =  RXdata.CoreData.IMU[0].AccX  * D2R;
    imu_linear_acceleration.y =  RXdata.CoreData.IMU[0].AccY  * D2R;
    imu_linear_acceleration.z =  RXdata.CoreData.IMU[0].AccZ  * D2R;

    /*assign as sensorMsg IMU format*/
    imu.orientation = imu_orientation;
    imu.angular_velocity = imu_angular_velocity;
    imu.linear_acceleration = imu_linear_acceleration;

    imu_pub.publish(imu);

}

void* LANThread(void *){

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    int connectCnt = 0;

    while(1){
        usleep(100);
        if(tcp_status == 0x00){ // If client was not connected
            if(sock == 0){
                CreateSocket(PODO_ADDR, PODO_PORT);
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

            if(tcp_size == RXDataSize)
            {
                /*copy received rx buffer data*/
                memcpy(&(RXdata), RXBuffer, RXDataSize);
                RXpodoData();
            }
        }
    }
    return NULL;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "data_grab_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10); //10Hz 

    std::cout << "\033[1;32m=======================================" << std::endl << std::endl;
    std::cout << "  Node name   : Data Grab" << std::endl << std::endl;
    std::cout << "  version     : 0.1.201806" << std::endl;
    std::cout << "  Developer   : Jeongsoo Lim (yjs0497@kaist.ac.kr)" << std::endl;
    std::cout << "  Modifier    : MoonYoung Lee (ml634@kaist.ac.kr)" << std::endl;
    std::cout << "=======================================\033[0m" << std::endl;


    /*Create Socket*/
    if(CreateSocket(PODO_ADDR, PODO_PORT))
    {
        ROS_INFO("Created Socket..");

        /*Init buffer to RX robot Data from Motion PC*/
        RXDataSize = sizeof(LAN_PODO2VISION);
        RXBuffer = (void*)malloc(RXDataSize);
        
        /*Init buffer to TX Planning Data to Motion PC*/
        TXDataSize = sizeof(float_t);
        //TXDataSize = sizeof(drc_hubo_sensorpack::Planning_data);
        TXBuffer = (void*)malloc(TXDataSize);

        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0)
        {
            ROS_ERROR("Create Thread Error...");
            return 0;
        }
    } else {
        ROS_ERROR("Create Socket Error..");
        ROS_ERROR("Terminate Node...");
        return 0;
    }
    
    

	std::string realsenseRGBDTopic("/camera/depth_registered/points");
    std::string realsenseRGBTopic("/camera/color/image_raw");
    std::string zedRGBDTopic("/zed/point_cloud/cloud_registered");
    std::string zedRGBTopic("/zed/rgb/image_raw_color");
    std::string lidarTopic("/velodyne_points");
    std::string imuTopic("/imu_output");
    int queueSize = 10;


    /*time sync approximately sensor inputs */
    message_filters::Subscriber<PointCloud2> realsenseRGBD_sub(nh, realsenseRGBDTopic, queueSize);
    message_filters::Subscriber<Image> realsenseRGB_sub(nh, realsenseRGBTopic, queueSize);
    message_filters::Subscriber<PointCloud2> zedRGBD_sub(nh, zedRGBDTopic, queueSize);
    message_filters::Subscriber<Image> zedRGB_sub(nh, zedRGBTopic, queueSize);
    message_filters::Subscriber<PointCloud2> lidar_sub(nh, lidarTopic, queueSize);
    //message_filters::Subscriber<Imu> imu_sub(nh, imuTopic, queueSize);


    typedef sync_policies::ApproximateTime<PointCloud2, Image, PointCloud2, Image, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(queueSize),realsenseRGBD_sub,realsenseRGB_sub, zedRGBD_sub, zedRGB_sub, lidar_sub );
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

    /*individual callback for sensor inputs*/
    /*
    ros::Subscriber subCamera = nh.subscribe<sensor_msgs::PointCloud2> (realsenseRGBDTopic, 1, callbackCameraPointCloud);
    ros::Subscriber subLidar = nh.subscribe<sensor_msgs::PointCloud2> (lidarTopic, 1, callbackLidarPointCloud);
    */
    
    /*callback for operating pc */
    ros::Subscriber subOPC = nh.subscribe("/mobile_hubo/navigation_path",1,opcCallback);
    ros::Subscriber start_pose_subscriber = nh.subscribe("/tf", 1, set_start_pose);

    // Create a ROS publisher for the output
    /*
    realsense_pub = nh.advertise<sensor_msgs::PointCloud2> ("realsense_output", 1);
    lidar_pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_output", 1);
    */imu_pub = nh.advertise<sensor_msgs::Imu> ("imu_output",1);
    
    
    /*time synced publisher*/
    realsenseRGBD_sync_pub = nh.advertise<sensor_msgs::PointCloud2> ("hubo_realsenseRGBD_output", 10);
    realsenseRGB_sync_pub = nh.advertise<sensor_msgs::Image> ("hubo_realsenseRGB_output", 10);
    zedRGBD_sync_pub = nh.advertise<sensor_msgs::PointCloud2> ("hubo_zedRGBD_output", 10);
    zedRGB_sync_pub = nh.advertise<sensor_msgs::Image> ("hubo_zedRGB_output", 10);
    lidar_sync_pub = nh.advertise<sensor_msgs::PointCloud2> ("hubo_lidar_output", 10);
    
    
	//pubArray = nh.advertise<drc_hubo_sensorpack::Planning_dataArray>("testArray", 1000);


    ros::spin();

    return 0;
}
