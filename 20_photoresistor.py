#!/usr/bin/env python
import PCF8591_3 as ADC
import RPi.GPIO as GPIO
import time

DO = 17
GPIO.setmode(GPIO.BCM)

def setup():
    ADC.setup(0x48)
    GPIO.setup(DO, GPIO.IN)


def loop():
  
    status = 1
    while True:
        conv = ADC.read(0) * 5 #our values ranged from 0 ~ 123 from ADC
                            #online equation requires 0 ~ 512
        resistance = ((1024 + (2 * conv)) * 10000) /(512 - conv) #human resistance measured in ohms
        '''reference: wiki.seeedstudio.com/Grove-GSR_Sensor'''
        conductivity = 1 / resistance * 1000000
        print ('Value: ', round(resistance, 3), round(conductivity, 3), ADC.read(0))
		
        time.sleep(0.2)
if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt: 
        pass	
