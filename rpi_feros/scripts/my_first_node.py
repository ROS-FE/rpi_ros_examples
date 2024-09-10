#!/usr/bin/env python3

import rospy

if __name__ == '__main__':
  rospy.init_node('my_first_python_node')
  rospy.loginfo('This node has been started.')
  rospy.sleep(1)
  print('Exit now')
