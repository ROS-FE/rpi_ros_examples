#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO


# GPIO za LED:
#   Green 1 - GPIO 2
#   Green 2 - GPIO 3
#   Yellow 1 - GPIO 4
#   Yellow 2 - GPIO 5
#   Red 1 - GPIO 6
#   Red 2 - GPIO 7


class rpi_led():
    
    def __init__(self):
        # init variables
        self.LED_GPIO = 7
        # set GPIO kot BCM
        GPIO.setmode(GPIO.BCM)
        # set all leds
        for ii in range(2,8):
            # set IO as outputs
            GPIO.setup(ii,GPIO.OUT)
        
        # define subscriber
        # rospy.Subscriber('topic_name', varType, callback)
        self.sub = rospy.Subscriber('/button_state', Bool, self.button_state_callback)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def button_state_callback(self, msg):
        # the code that is executed when data is received
        # turn on LED
        GPIO.output(self.LED_GPIO, msg.data)

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
    rospy.init_node('led_actuator')
    # initialise class
    led_act = rpi_led()
    # reset leds
    led_act.resetLed()
    try:
        # loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    
