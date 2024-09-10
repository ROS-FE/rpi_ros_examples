#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool
import RPi.GPIO as GPIO

class buttonClient():
    
    def __init__(self):
        # init variables
        self.BUTTON_GPIO = 11
        self.LED_STATE = False
        # set GPIO kot BCM
        GPIO.setmode(GPIO.BCM)
        # set button GPIO as input
        GPIO.setup(self.BUTTON_GPIO, GPIO.IN)
        # set interrupt
        # GPIO.add_event_detect(gpio_num, edge, callback, bouncetime)
        GPIO.add_event_detect(self.BUTTON_GPIO, GPIO.RISING, callback=self.button_callback, bouncetime=500)

        # define service proxy
        # rospy.ServiceProxy('service_name', varType)
        # wait for service
        rospy.wait_for_service('/set_led_state')
        # define proxy 
        self.set_led_state = rospy.ServiceProxy('/set_led_state', SetBool)
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def button_callback(self, channel):
        # code that is called from interrupt
        # get the button state
        power_on_led = GPIO.input(self.BUTTON_GPIO)

        # change LED state
        self.LED_STATE = not self.LED_STATE

        try:
            # send request, get response
            resp = self.set_led_state(self.LED_STATE)
            # print response
            print(resp)
        except rospy.ServiceException as e:
            # in case of error
            rospy.logwarn(e)
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c

        # clear all settings
        GPIO.cleanup()
        self.ctrl_c = True
            
if __name__ == '__main__':
    # initialise node
    rospy.init_node('button_monitor')
    # initialise class
    button_client = buttonClient()
    try:
        # loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

















#############################################33333
# definicija GPIO za gumb
BUTTON_GPIO = 11
# definicija globalne spremenljivke
LED_STATE = False

def button_callback(channel):
    # funkcija, ki se izvede ob prekinitvi
    global LED_STATE
    # preberi stanje gumba
    power_on_led = GPIO.input(BUTTON_GPIO)

    # zamenjaj stanje LED
    LED_STATE = not LED_STATE

    # cakaj, dokler ni na voljo zeljen service
    rospy.wait_for_service('set_led_state')
    try:
        # definicija proxya 
        # rospy.ServiceProxy('service_name', varType)
        set_led_state = rospy.ServiceProxy('set_led_state', SetBool)
        # poslji request, dobi response
        resp = set_led_state(LED_STATE)
        # izpis odgovora
        print(resp)
    except rospy.ServiceException as e:
        # v primeru napake jo zapisi v log
        rospy.logwarn(e)

if __name__=='__main__':
    # inicializacija noda
    rospy.init_node('button_monitor')
    # nastavitve GPIO
    GPIO.setmode(GPIO.BCM)
    # nastavi GPIO za gumb kot vhod
    GPIO.setup(BUTTON_GPIO, GPIO.IN)
    # definiranje prekinitve
    # GPIO.add_event_detect(gpio_num, fronta, callback, bouncetime)
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.RISING, callback=button_callback, bouncetime=500)

    # vrtenje zanke
    rospy.spin()

    # pobrisi GPIO nastavitve
    GPIO.cleanup()
