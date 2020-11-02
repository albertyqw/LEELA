### TEST PLAN

#OLED Libraries 
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
import time
from time import sleep, time
from random import randrange

#Audio PLayer Libraries
import pygame

#Pin Control - Vibrator
import RPi.GPIO as GPIO

#Pulse Sensor Libraries
import smbus
from filters import DCFilter, MeanDiffFilter, ButterworthFilter
import math
from max30100_regs import *

#GSR Sensor Libraries
import PCF8591_3 as ADC

#Pin Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(20, GPIO.OUT)

#GSR
DO = 17
GPIO.setmode(GPIO.BCM)

#OLED Setup
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, rotate=0)

with canvas(device) as draw:
    draw.text((20, 20), "Hi! I'm LEELA.", fill="white")

#Basal bpm assumed to be 75

def normal():
    print("Includes readings that are within normal range")
    bpm_list = [75,74,76,73,79,80,78,79]
    amplitude_list = [0,0.5,0.1,0.2,0.3,0,0.2,0.1] 
    bpm_sum = 0
    amplitude_sum = 0
    for value in range(8):
        bpm_sum += bpm_list[value]
        amplitude_sum += amplitude_list[value]
    avg_bpm = round((bpm_sum / 8), 2)
    avg_amplitude = round((amplitude_sum / 8), 2)
    print("Average BPM: ", avg_bpm)
    print("Average amplitude: ", avg_amplitude)

def vibration_only():
    print("Includes readings that would allow for vibration only")
    bpm_list = [80,83,86,88,87,89,88,90]
    amplitude_list = [1,1.1,1.2,1,0.8,0.9,1.2,1.4]
    bpm_sum = 0
    amplitude_sum = 0
    for value in range(8):
        bpm_sum += bpm_list[value]
        amplitude_sum += amplitude_list[value]
    avg_bpm = round((bpm_sum / 8), 2)
    avg_amplitude = round((amplitude_sum / 8), 2)
    print("Average BPM: ", avg_bpm)
    print("Average amplitude: ", avg_amplitude)
    print("Average BPM greater than basal level + 10 BPM (= 85 BPM)")
    print("Average amplitude greater than 1.0")

    print("Vibrator ON")
    GPIO.output(20, GPIO.HIGH) #Turns vibration motor on
    sleep(5)
    print("Vibrator OFF")
    GPIO.output(20, GPIO.LOW) #Turns vibration motor off after 5 seconds

    with canvas(device) as draw:
        draw.text((5, 2), "You're doing great!", fill="white")
        draw.rectangle((54,20, 55,28), fill="white")
        draw.rectangle((64,20, 65,28), fill="white")
        #draw.rectangle((50,34, 70,34), fill="white")
        draw.arc((50,40,70,49),0,180,fill="white")
        '''
        OLED displays encouraging text and draws a smiling face. Smiling
        face represents low stress levels for the user.
        '''

def with_music():
    print("Includes readings that would allow for vibration and music")
    bpm_list = [90,92,89,93,88,91,90,91]
    amplitude_list = [1.5,1.6,1.4,1.3,1.4,1.6,1.7,1.7]
    bpm_sum = 0
    amplitude_sum = 0
    for value in range(8):
        bpm_sum += bpm_list[value]
        amplitude_sum += amplitude_list[value]
    avg_bpm = round((bpm_sum / 8), 2)
    avg_amplitude = round((amplitude_sum / 8), 2)
    print("Average BPM: ", avg_bpm)
    print("Average amplitude: ", avg_amplitude)
    print("Average BPM greater than basal level + 15 BPM (= 90 BPM)")
    print("Average GSR amplitude greater than 1.5")

    print("Vibrator ON")
    GPIO.output(20, GPIO.HIGH) #Turns vibration motor on
    
    with canvas(device) as draw:
        draw.rectangle((54,20, 55,28), fill="white")
        draw.rectangle((64,20, 65,28), fill="white")
        draw.rectangle((50,34, 70,34), fill="white")
        #draw.arc((50,40,70,49),180,0,fill="white")
        '''
        Draws a straight face on the OLED screen. Straight face
        represents moderate stress levels of the user.
        '''
    pygame.mixer.music.load('/home/pi/Downloads/be_happy.mp3')
    pygame.mixer.music.play()
    sleep(5)
    pygame.mixer.music.fadeout(2000)
    GPIO.output(20, GPIO.LOW)
    #uplifting music plays for 5 seconds; vibration motor turns off

def with_voice():
    print("Includes readings that would allow for vibration and voice audio")
    bpm_list = [95,96,97,95,94,98,97,94]
    amplitude_list = [2,1.9,1.8,2,2.2,2.3,2.2,2.4]
    bpm_sum = 0
    amplitude_sum = 0
    for value in range(8):
        bpm_sum += bpm_list[value]
        amplitude_sum += amplitude_list[value]
    avg_bpm = round((bpm_sum / 8), 2)
    avg_amplitude = round((amplitude_sum / 8), 2)
    print("Average BPM: ", avg_bpm)
    print("Average amplitude: ", avg_amplitude)
    print("Average BPM greate than basal level + 20 BPM (= 95 BPM)")
    
    print("Vibrator ON")
    GPIO.output(20, GPIO.HIGH) #Vibration motor turns on

    with canvas(device) as draw:
        draw.rectangle((54,20, 55,28), fill="white")
        draw.rectangle((64,20, 65,28), fill="white")
        #draw.rectangle((50,34, 70,34), fill="white")
        draw.arc((50,40,70,49),180,0,fill="white")
        '''
        Draws a frowning face on OLED screen. Frowning face
        represents high stress levels of the user.
        '''

    pygame.mixer.music.load('/home/pi/Downloads/message.mp3')
    pygame.mixer.music.play()
    sleep(7)
    #supportive message audio is played

    print("Vibrator OFF")
    GPIO.output(20, GPIO.LOW) #Vibration motor turns off
    
def main():
    print("1. Normal conditions")
    print("2. Vibration only")
    print("3. Vibration with music")
    print("4. Vibration with voice audio")
    print("Basal heart rate set at 75 bpm")
    print("Basal GSR amplitude set at 0")
    while True:
        choice = int(input("Enter choice: "))
        if choice == 1:
            normal()
        elif choice == 2:
            vibration_only()
        elif choice == 3:
            with_music()
        elif choice == 4:
            with_voice()

main()
