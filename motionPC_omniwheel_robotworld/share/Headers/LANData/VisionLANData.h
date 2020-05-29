#ifndef VISIONLANDATA_H
#define VISIONLANDATA_H

#include "../RBSharedMemory.h"
#include "../UserSharedMemory.h"

typedef struct __LAN_STRUCT_VISION2PODO__
{
    ObjectPos object_pose;
    ObjectPos robot_current_pose;

} LAN_VISION2PODO,*pLAN_VISION2PODO;



typedef struct __LAN_STRUCT_PODO2VISION__
{
    RBCORE_SHM_SENSOR       CoreSEN;

} LAN_PODO2VISION, *pLAN_PODO2VISION;



typedef struct __LAN_STRUCT_ROS2MOTION_CMD_
{
    int ros_cmd;
    int object_menu;
} LAN_R2M_CMD, *pLAN_R2M_CMD;

typedef struct __LAN_STRUCT_MTION2ROS
{
    MOTION2ROS              M2R;
} LAN_M2R, *pLAN_M2R;

#endif // RBLANDATA_H
