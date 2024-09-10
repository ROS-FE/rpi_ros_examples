#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
# include actionlib
import actionlib
# include action msgs
from rpi_msgs.msg import runningLedFeedback, runningLedResult, runningLedAction


class runled_server():
    
    def __init__(self):
        # init variables
       
        # set GPIO kot BCM
        GPIO.setmode(GPIO.BCM)
        # set all leds
        for ii in range(2,8):
            # set IO as outputs
            GPIO.setup(ii,GPIO.OUT)
        
        # define simple action server
        # actionlib.SimpleActionServer('action_name', actionType, callback, autostart)
        self.ACserver = actionlib.SimpleActionServer('run_led', runningLedAction, self.goal_callback, False)
        # run server
        self.ACserver.start()
        print('Server pripravljen')

        # define loop frequency (6 Hz)
        self.r = rospy.Rate(6)
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def goal_callback(self, goal):

        # Do lots of awesome groundbreaking robot stuff here
        self.resetLed()

        # print number of iterations 
        print("Stevilo iteracij: %i" % goal.numberOfRuns)

        # define feedback variable
        feedback1 = runningLedFeedback()
        # define result variable
        result1 = runningLedResult()

        success = True
        doPreemt = False
        # do number of iterations
        for kk in range(1,goal.numberOfRuns+1):
            # turn on individual LED (GPIO 2 - GPIO 7)
            for ii in range(2,8):
                # check if there was a preemt request
                if self.ACserver.is_preempt_requested():
                    # skip other interations
                    print('Goal preempted.')
                    # definie result
                    result1.finalRun = kk
                    # in case of preemt send result and text
                    self.ACserver.set_preempted(result=result1,text='Goal preemted.')
                    success = False 
                    doPreemt = True
                    # break inside loop - turning on individual LEDs
                    break
                ###############################
                # ACTIONS
                # clear all LED
                self.resetLed()
                # turn on LED i
                GPIO.output(ii,True)
                # 6 Hz
                self.r.sleep()
                ###############################	
            # in case of preemt, break outside loop - iterations
            if doPreemt:
                break
            # send feedback after each interation
            feedback1.currentRun = kk
            self.ACserver.publish_feedback(feedback1)

        # send result after all iterations
        if success:
            # define results
            result1.finalRun = feedback1.currentRun
            # log
            rospy.loginfo('Zakljuceno - Succeeded') 
            # poslji rezultat
            self.ACserver.set_succeeded(result1)

    def resetLed(self):
        # reset all leds
        for ii in range(2,8):
            # turn off all leds
            GPIO.output(ii,False)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c

        # clear all settings
        GPIO.cleanup()
        self.ctrl_c = True
            
if __name__ == '__main__':
    # initialise node
    rospy.init_node('runled_server')
    # initialise class
    runled = runled_server()
    # reset leds
    runled.resetLed()
    try:
        # loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    
