#!/usr/bin/env python3

import rospy

class hello_world():
    
    def __init__(self):
        # init variables
        self.delay = 5
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c
        rospy.loginfo('This node has been terminated.')
        self.ctrl_c = True

    def test_node(self):
        
        rospy.loginfo('This node has been started.')
        rospy.sleep(self.delay)
        print('Exit now')
            
if __name__ == '__main__':
    # initialise node
    rospy.init_node('my_first_python_node')
    # initialise class
    first_node = hello_world()
    try:
        first_node.test_node()
    except rospy.ROSInterruptException:
        pass
