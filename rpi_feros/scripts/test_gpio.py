#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO

# define button GPIO pins
    # button 1 - GPIO 11
    # button 2 - GPIO 12

# define LED GPIO pins
    # Green 1 - GPIO 2
    # Green 2 - GPIO 3
    # Yellow 1 - GPIO 4
    # Yellow 2 - GPIO 5
    # Red 1 - GPIO 6
    # Red 2 - GPIO 7


def resetLed():
     # nastavi in resetiraj vse LED
    for ii in range(2,8):
        # nastavi IO kot izhode
        GPIO.setup(ii,GPIO.OUT)
        # postavi izhode na nizek nivo
        GPIO.output(ii,False)

if __name__ == '__main__':
    # node init
    rospy.init_node('test_gpio_rpi')
    # set GPIO as BCM
    GPIO.setmode(GPIO.BCM)
    # reset LED
    resetLed()
    # set button1 as input
    GPIO.setup(11, GPIO.IN)
    # set loop to 10 Hz
    rate = rospy.Rate(10)

    gpio_state = False
    # izvajaj program dokler ni node terminiran
    while not rospy.is_shutdown():
        # preberi GPIO pin/gumb
        #gpio_state = GPIO.input(11)
        gpio_state = not gpio_state
        """
        interrupt definition
        GPIO.add_event_detect(gpio_num, detected_edge, callback, bouncetime)
        GPIO.add_event_detect(BUTTON_GPIO, GPIO.RISING, callback=button_callback, bouncetime=500)
        """
        GPIO.output(2, gpio_state)
        #if gpio_state:
        #    GPIO.output(2, True)
        rate.sleep()

    # ko se zakljuci izvajanje node, pobrisi nastavitve GPIO pinov
    GPIO.cleanup()
