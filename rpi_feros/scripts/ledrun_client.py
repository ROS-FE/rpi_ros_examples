#!/usr/bin/env python3

import rospy
# import actionlib
import actionlib
# import action msgs
from rpi_msgs.msg import runningLedAction, runningLedGoal, runningLedResult

# define action server status
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

class runled_client():
    
    def __init__(self):

        # define simple action client 
        # actionlib.SimpleActionClient('action:name', actionType)
        self.client = actionlib.SimpleActionClient('run_led', runningLedAction)
        # wait, until server isnt active
        self.client.wait_for_server()
        rospy.loginfo("Server is active.")

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def run_led_client(self, goalNum):
              
        # define goal
        goal = runningLedGoal()
        goal.numberOfRuns = goalNum

        # send goal 
        self.client.send_goal(goal)

        '''
        ###################################
        # FOR PREEMT TESTING
        # after 3 s send new goal
        rospy.sleep(3)
        goal.numberOfRuns = 2
        self.client.send_goal(goal) 
        print('New goal was sent.')
        ###################################
        '''
        
        # OPTION A - wait, until server doesn finish (similar to service)
        self.client.wait_for_result()

        '''
        # OPTION B - do something else while you are waiting    

        ## read current server state
        current_state = self.client.get_state()

        ## define loop frequency 1 Hz
        r2 = rospy.Rate(1)

        # until server status isnt DONE, do something else
        while current_state < DONE:
            # action is running, so do somethin productive
            
            # check state
            current_state = self.client.get_state()
            # 1 Hz
            r2.sleep()

        # is the server state is WARN
        if current_state == WARN:
            rospy.logwarn("Warning on the action server side.")

        # if the server state is ERROR
        if current_state == ERROR:
            rospy.logerr("Error on the action server side.")
        '''

        # return result
        return self.client.get_result() 


    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c

        self.ctrl_c = True
            
if __name__ == '__main__':
    # initialise node
    rospy.init_node('run_led_client')
    # initialise class
    runled = runled_client()

    try:
        # poslji goal         
        result = runled.run_led_client(goalNum = 10)
        # izpisi rezultat
        print("Result: %i" % result.finalRun)
    except rospy.ROSInterruptException:
        # v primeru napake
        print("Program interrupted before completion.")
