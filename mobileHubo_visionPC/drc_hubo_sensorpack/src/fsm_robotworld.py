#!/usr/bin/env python

import rospy
import numpy as np
import smach
import smach_ros
from std_msgs.msg import String
from std_msgs.msg import Int32
from object_detector.msg import bbox2d, bbox3d # custom messages
import time


#podo enum command
NO_ACT = 0
AL_WORKING = 1
GOTODISPLAY = 2
GOTOGRASPSPOT = 3
GOTOHOME = 4
GOTODES = 5
GRASP = 6
PUT = 7
ESTOP = 8
RESET = 9

#podo_motion_state enum command
# AL_NO_ACT = 0
# WORKING = 1
# DONE = 2


#global variables to update
receivedInputFlag = 0
receivedInput = ''

gotoPose = np.zeros([3,1], dtype = float)
timecounter = 0
starttime = 0
motion_counter = 0
robot_motion_state = 0
object_detectedflag = 0
object_detect_fail_flag = 0


#publish order to object detector
pub = rospy.Publisher('/fsm_detect_object', String, queue_size=10)
#publish order to ros_podo
pub_order = rospy.Publisher('/fsm_order_podo', String, queue_size=10)
#publish robot command
pub_command = rospy.Publisher('/fsm_command', Int32, queue_size=10)
#publish done to android publisher
pub_done_android = rospy.Publisher('/done_msg_str', String, queue_size=10)

#=================================#
def input_callback(data):
    #rospy.loginfo("user_input: %s", data.data)

    global receivedInputFlag, receivedInput
    receivedInputFlag = 1
    receivedInput = data.data

def motion_state_callback(data):
    global robot_motion_state, motion_counter

    motion_counter = motion_counter +1

    if (motion_counter > 10000):
        motion_counter = 0
        
        robot_motion_state = data.data

        rospy.loginfo("motion_state_input: %d\n", robot_motion_state)


def objectdetection_cb(data):
    global object_detectedflag, object_detect_fail_flag
    #rospy.loginfo("object length @@@@@@@@@@: %d", data.length)
    
    time.sleep(2)
    object_detectedflag = 1

    #if bounding box length is 0 --> detect failed
    if(data.length == 0):
        object_detect_fail_flag = 1
#=================================#





#=================================#
# define state IDLE
class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
 

    def execute(self, userdata):
        global receivedInputFlag, starttime
        #rospy.loginfo('Executing state IDLE')
        rospy.Subscriber("/order_msg_str",String, input_callback)

        #=================================#
        if (receivedInputFlag == 1):
            
            #publish user input
            pub_order.publish(receivedInput)
            starttime = time.time()
            #publish robot command
            pub_command.publish(GOTODISPLAY)

            #transition to next state
            return 'outcome1' #go to display
        
        else:
            
            return 'outcome2' #remain in idle


# define state GOTO_DISPLAY
class GOTO_DISPLAY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        global starttime, robot_motion_state
        #rospy.loginfo('Executing state GOTO_DISPLAY')

        currenttime = time.time()
        #rospy.loginfo("currentime: %f, %f\n", currenttime,starttime )
        #check for motion_state
        rospy.Subscriber("/robot_motion_state",Int32, motion_state_callback, queue_size=1, buff_size=1000)

        #print test for determining motion_state
        # print "motion_state: ", (robot_motion_state), "DONE: ", DONE #DONE returns class NOT int value
        # print "cond0: ", (robot_motion_state == 0)
        # print "cond1: ", (robot_motion_state == 1)
        # print "cond2: ", (robot_motion_state == 2)
        # print "cond2: ", (robot_motion_state == DONE)

        
        if ( ((currenttime - starttime) > 5) and (robot_motion_state == 2)):
            starttime = time.time()
            
            #publish robot command
            pub_command.publish(GOTOGRASPSPOT)
            

            return 'outcome1' #detect object
        
        else:
            
            return 'outcome2' #remain in gotodisplay
        

# define state GOTO_GRASPSPOT
class GOTO_GRASPSPOT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state GOTO_GRASPSPOT')

        global starttime, robot_motion_state

        currenttime = time.time()
        #check for motion_state
        rospy.Subscriber("/robot_motion_state",Int32, motion_state_callback, queue_size=1, buff_size=1000)

        #if sprite, dont wait time
        if (receivedInput == "1"):
            if  ( ((currenttime - starttime) > 0.5) and (robot_motion_state == 2)):
                
                starttime = time.time()
                order_str = ""

                if(receivedInput == "1"):
                    order_str = "sprite"
                elif(receivedInput == "2"):
                    order_str = "vitamin_water_orange"
                elif(receivedInput == "3"):
                    order_str = "samdasu"
                else:
                    order_str = ""

                pub.publish(order_str)
                return 'outcome1' #grab object
            
            else:
                
                return 'outcome2' #remain 

        #if vitamin or water, wait time
        else:
            if  ( ((currenttime - starttime) > 3) and (robot_motion_state == 2)):
                
                starttime = time.time()
                order_str = ""

                if(receivedInput == "1"):
                    order_str = "sprite"
                elif(receivedInput == "2"):
                    order_str = "vitamin_water_orange"
                elif(receivedInput == "3"):
                    order_str = "samdasu"
                else:
                    order_str = ""

                pub.publish(order_str)
                return 'outcome1' #grab object
            
            else:
                
                return 'outcome2' #remain 

# define state DETECT_OBJECT
class DETECT_OBJECT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state DETECT_OBJECT')

        global starttime

        currenttime = time.time()

        #detect bbox3d subscribe
        rospy.Subscriber("/hubo_bboxes3d", bbox3d, objectdetection_cb, queue_size=10)


        if ( (currenttime - starttime > 1) and (object_detectedflag==1) ):
            #publish robot command
            #pub_command.publish(GRASP)
            starttime = time.time()
            return 'outcome1' #grab object
        
        else:
            
            return 'outcome2' #remain 


# define state GRAB_OBJECT
class GRAB_OBJECT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):

        global starttime, robot_motion_state
        #rospy.loginfo('Executing state GRAB_OBJECT')

        currenttime = time.time()
        #check for motion_state
        rospy.Subscriber("/robot_motion_state",Int32, motion_state_callback, queue_size=1, buff_size=1000)
        
        
        #until rx done grab
        if  ( ((currenttime - starttime) > 9) and (robot_motion_state == 2)):
            pub_command.publish(GOTOHOME)
            starttime = time.time()

            return 'outcome1' #grab object
        
        else:
            
            return 'outcome2' #remain 



# define state GOTO_HOME
class GOTO_HOME(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        global starttime, robot_motion_state
        #rospy.loginfo('Executing state GRAB_OBJECT')

        currenttime = time.time()
        #check for motion_state
        rospy.Subscriber("/robot_motion_state",Int32, motion_state_callback, queue_size=1, buff_size=1000)
        

        #until rx done grab
        if  ( ((currenttime - starttime) > 5) and (robot_motion_state == 2)):
            pub_command.publish(PUT)
            starttime = time.time()

            return 'outcome1' #grab object
        
        else:
            
            return 'outcome2' #remain 


# define state PUT_OBJECT
class PUT_OBJECT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        global receivedInputFlag, robot_motion_state, starttime

        #rospy.loginfo('Executing state PUT_OBJECT')

        currenttime = time.time()
        #check for motion_state
        rospy.Subscriber("/robot_motion_state",Int32, motion_state_callback, queue_size=1, buff_size=1000)
        
        

        if  ( ((currenttime - starttime) > 5) and (robot_motion_state == 0)):

            pub_command.publish(RESET)
            starttime = time.time()
            
            
            return 'outcome1' 

        else:
            return 'outcome2' #remain 

# define state RESET_POSE
class RESET_POSE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        global receivedInputFlag, robot_motion_state, starttime

        #rospy.loginfo('Executing state RESET_POSE')

        currenttime = time.time()
        #check for motion_state
        rospy.Subscriber("/robot_motion_state",Int32, motion_state_callback, queue_size=1, buff_size=1000)
        
        

        if  ( ((currenttime - starttime) > 2) and (robot_motion_state == 2)):
            
            starttime = time.time()
            pub_done_android.publish("done")
            receivedInputFlag = 0
            object_detectedflag = 0
            
            return 'outcome1' 

        else:
            return 'outcome2' #remain 




        



# define state DONE
class DONE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DONE')


        return 'outcome1' #grab object
        

#=================================#


def main():
    rospy.init_node('smach_example_state_machine')
    
   
    #publish first blank object detect to load files 
    pub.publish("")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('IDLE', IDLE(), 
                               transitions={'outcome1':'GOTO_DISPLAY', 'outcome2':'IDLE'})

        smach.StateMachine.add('GOTO_DISPLAY', GOTO_DISPLAY(), 
                               transitions={'outcome1':'GOTO_GRASPSPOT', 'outcome2':'GOTO_DISPLAY'})

        smach.StateMachine.add('GOTO_GRASPSPOT', GOTO_GRASPSPOT(), 
                               transitions={'outcome1':'DETECT_OBJECT', 'outcome2':'GOTO_GRASPSPOT'})

        smach.StateMachine.add('DETECT_OBJECT', DETECT_OBJECT(), 
                               transitions={'outcome1':'GRAB_OBJECT', 'outcome2':'DETECT_OBJECT'})

        smach.StateMachine.add('GRAB_OBJECT', GRAB_OBJECT(), 
                               transitions={'outcome1':'GOTO_HOME', 'outcome2':'GRAB_OBJECT'})

        smach.StateMachine.add('GOTO_HOME', GOTO_HOME(), 
                               transitions={'outcome1':'PUT_OBJECT', 'outcome2':'GOTO_HOME'})
        smach.StateMachine.add('PUT_OBJECT', PUT_OBJECT(), 
                               transitions={'outcome1':'RESET_POSE', 'outcome2':'PUT_OBJECT'})
        smach.StateMachine.add('RESET_POSE', RESET_POSE(), 
                               transitions={'outcome1':'IDLE', 'outcome2':'RESET_POSE'})

        smach.StateMachine.add('DONE', DONE(), 
                               transitions={'outcome1':'DONE'})



    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
   
    
    #rospy.spin()

    sis.stop()
   



if __name__ == '__main__':
    main()