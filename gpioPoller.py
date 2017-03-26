#!/usr/bin/env python
#
# program periodically checks RasPi GPIO lines and sets some other line on a raspberry pi
#
from time import sleep     # Allows us to call the sleep function to slow down our loop
import RPi.GPIO as GPIO    # Allows us to call our GPIO pins and names it just GPIO


# This pin goes high if you want an 'alert' for some conditions
alertPin  = 24

# Put gpio pin numbers in this array and they are re-checked on the loop interval
#inputPins = [31, 32, 33, 34]
inputPins = [35]

GPIO.setmode(GPIO.BCM)     # Set's GPIO pins to Pi GPIO numbering.  Use GPIO.BCM for direct BCM pins

GPIO.setup(alertPin, GPIO.OUT)

GPIO.setup(inputPins, GPIO.IN)    # Set all input pins to be an input

loopIdx = 1

# Start a loop that never ends
while True:
    print('Loop number: ' + str(loopIdx))
    for pin in inputPins:
        if (GPIO.input(pin) == True): # Physically read the pin now
            print(str(pin) + ' HIGH')
            GPIO.output(alertPin, GPIO.HIGH)
        else:
            print(str(pin) + ' LOW')
            GPIO.output(alertPin, GPIO.LOW)
    sleep(1);           # Sleep for a full second before restarting our loop
    loopIdx += 1
