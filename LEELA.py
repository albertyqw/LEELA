#OLED Libraries 
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
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

#OLED Setup
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial, rotate=0)


pygame.init()
pygame.mixer.init()

print("==========================================================")
print("Booting LEELA: the Life-Emulating Emotion-Linked Assistant")
print("==========================================================")

with canvas(device) as draw:
    draw.text((20, 20), "Hi! I'm LEELA.", fill="white")

def setup():
    ADC.setup(0x48)
    GPIO.setup(DO, GPIO.IN)
    GPIO.setup(DO, GPIO.OUT)

#Pulse
''' SOME ADJUSTABLE PARAMETERS '''
MAX_THRESHOLD   = 45.0
MIN_THRESHOLD   = 20.0
CUT_OFF         = 200.0
MAX_BPM         = 220.0
QUEUE_SIZE      = 10

class Timer:
    ''' A class that performs a stopwatch functionality '''
    def __init__(self):
        self.reset()

    def reset(self):
        ''' Resets the timer to current time (zero)'''
        self.start = time()

    def get_time(self):
        ''' Returns the time from the last reset'''
        return time()-self.start

class MAX30100:
    ''' MAX30100 Sensor Class '''
    def __init__(self):
        # Create the I2C bus
        self.bus = smbus.SMBus(1)
        self.reset()

        # Set the mode to SPO2
        self.set_mode(MODE_SPO2)

        # Set the IR LED Current
        self.set_current_ir(LED_CURRENT_27_1_MA)

        # Set the RED LED Current
        self.set_current_red(LED_CURRENT_27_1_MA)

        # Enable ADC High Resolution Mode (16 bit)
        self.set_spo2_hi_res()

        # Set the Sample Rate (16 bit at most 100 sps)
        self.set_sample_rate(SAMPLE_RATE_100_HZ)

        # Set the LED Pulse Width (16 bit needs 1.6ms)
        self.set_pulse_width(PULSE_WIDTH_1_6_MS)

        # IR Filters
        self.ir_filter = DCFilter()     
        self.ir_mean_filter = MeanDiffFilter()
        self.ir_lpb = ButterworthFilter()

        # RED Filters
        self.red_filter = DCFilter()

        # Create a Timer
        self.beat_timer = Timer()
        self.beat_timer.reset()

        # Beat detector
        self.found_beat = False
        self.bpm = 0.0

        # IR LED attributes 
        self.ir_filtered = 0.0
        self.ir_ac = 0.0
        self.prev_ir_filtered = 0.0

        # Red LED attribures
        self.red_filtered = 0.0
        self.red_ac = 0.0

        # BPM queue for averaging
        self.bpm_queue = []

        # SPO2 attributes 
        self.ir_ac2_sum = 0.0
        self.red_ac2_sum = 0.0
        self.beats_detected = 0
        self.samples_recorded = 0

    def reset(self):
        ''' Resets the sensor on initialization '''
        # Issue a reset command
        self.set_mode(MODE_RESET)
        sleep(1)
        # Reset FIFO Registers
        self.bus.write_byte_data(MAX30100_ADDR, FIFO_WRITE_REG, 0x00)
        self.bus.write_byte_data(MAX30100_ADDR, FIFO_OVF_REG, 0x00)
        self.bus.write_byte_data(MAX30100_ADDR, FIFO_READ_REG, 0x00)

        
    def set_mode(self, mode):
        ''' Configures the mode '''
        self.bus.write_byte_data(MAX30100_ADDR, MODE_CONFIG_REG, mode)
        
    def set_current_ir(self, ir_current):
        ''' Sets the IR LED Current'''
        reg = self.bus.read_byte_data(MAX30100_ADDR, LED_CONFIG_REG)
        # Preserve the 4 bits of RED LED current
        self.bus.write_byte_data(MAX30100_ADDR, LED_CONFIG_REG, (reg & 0xF0) | ir_current)
    
    def set_current_red(self, red_current):
        ''' Set the Red LED Current '''
        reg = self.bus.read_byte_data(MAX30100_ADDR, LED_CONFIG_REG)
        # Preserve the 4 bits of IR LED current
        self.bus.write_byte_data(MAX30100_ADDR, LED_CONFIG_REG, (reg & 0x0F) | (red_current<<4))

    def set_sample_rate(self, sample_rate):
        ''' Sets the ADC sample rate '''
        reg = self.bus.read_byte_data(MAX30100_ADDR, SPO2_CONFIG_REG)
        # Preserve the other bits 
        self.bus.write_byte_data(MAX30100_ADDR, SPO2_CONFIG_REG, (reg & 0xE3) | (sample_rate<<2))

    def set_pulse_width(self, pulse_width):
        ''' Sets the pulse width of both the IR and Red LEDs'''
        reg = self.bus.read_byte_data(MAX30100_ADDR, SPO2_CONFIG_REG)
        # Preserve the other bits
        self.bus.write_byte_data(MAX30100_ADDR, SPO2_CONFIG_REG, (reg & 0xFC) | pulse_width)
    
    def set_spo2_hi_res(self, enabled=1):
        ''' Enables/Disables the SPO2 High Resolution Mode'''
        reg = self.bus.read_byte_data(MAX30100_ADDR, SPO2_CONFIG_REG)
        # Preserve the other bits
        self.bus.write_byte_data(MAX30100_ADDR, SPO2_CONFIG_REG, (reg & 0xBF) | (enabled<<6))

    def __get_raw(self):
        ''' Reads raw IR, RED values'''
        raw_data = self.bus.read_i2c_block_data(MAX30100_ADDR, FIFO_DATA_REG, 4)
        self.raw_ir = (raw_data[0] << 8) | raw_data[1]
        self.raw_red = (raw_data[2] << 8) | raw_data[3] 
        return self.raw_ir, self.raw_red
    
    def update(self):
        ''' Updates the IR and Red LED raw and filtered values'''
        self.prev_ir_filtered = self.ir_filtered
        raw_reading = self.__get_raw()
        ir_ac = self.ir_filter.dc_removal(raw_reading[0])
        ir_mean = self.ir_mean_filter.mean_diff(ir_ac)
        self.ir_ac = ir_ac
        self.ir_filtered = self.ir_lpb.lpb(ir_mean)
        self.red_ac = self.red_filter.dc_removal(raw_reading[1])
        self.red_filtered = self.red_ac 
    
    def get_filtered(self):
        ''' Returns the filtered IR and Red LED values'''
        return self.ir_filtered, self.red_filtered
    
    def detect_beat(self):
        ''' Detects a heart beat '''
        if self.ir_filtered > MAX_THRESHOLD and not self.found_beat:
            # if a value above threshold is detected
            if self.ir_filtered < self.prev_ir_filtered:
                self.found_beat = True   # found a peak, record time
                self.beats_detected += 1 
                return True
        elif self.found_beat:
            if self.red_filtered < MIN_THRESHOLD:
                self.found_beat = False
        return False
        
    
    def get_bpm(self):
        ''' Calculates the value of heart rate in BPM '''
        if self.ir_filtered > CUT_OFF:
            return 0
        elif self.detect_beat():
            bpm = 60/(self.beat_timer.get_time()) 
            if bpm < MAX_BPM:
                self.bpm = bpm
            else:
                return 0
            if len(self.bpm_queue) > QUEUE_SIZE:
                self.bpm_queue = self.bpm_queue[1:]
            self.bpm_queue.append(self.bpm)
            self.beat_timer.reset()
        return self.bpm            

    def get_avg_bpm(self):
        ''' Calculates the average heart rate in BPM'''
        if len(self.bpm_queue) == 0: return 0
        return sum(self.bpm_queue)/len(self.bpm_queue)

def calc_avg(value, values_list):
    '''
        Calculates the average of the 4 most recent values and keeps a record
        of the 10 most recent values
    '''
    
    values_list.append(value)
    if len(values_list) == 1:
        average = value
    elif len(values_list) == 2:
        average = (values_list[-1] + values_list[-2]) / 2
    elif len(values_list) == 3:
        average = (values_list[-1] + values_list[-2] + values_list[-3]) / 3
    elif len(values_list) >= 4:
        average = (values_list[-1] + values_list[-2] +
                values_list[-3] + values_list[-4]) / 4
    else:
        print("Error in calculating average")
    if len(values_list) > 10:
        values_list.pop(0)
    return average

def main():
    status = 1
    pulseox = MAX30100()
    cond_list = []
    counter = 0
    while True:
        conv = ADC.read(0) * 512 / 123 #our values ranged from 0 ~ 123 from ADC, but the online equation requires 0 ~ 512.
        resistance = ((1024 + (2 * conv)) * 10000) /(512 - conv) #human resistance measured in ohms
        '''reference: wiki.seeedstudio.com/Grove-GSR_Sensor'''
        conductivity = 1 / resistance * 1000000
        
        pulseox.update() # updates the sensor readings
        bpm = pulseox.get_bpm() # Also updates the values as well
        avg_bpm = pulseox.get_avg_bpm()
        ir_filtered, red_filtered = pulseox.get_filtered()
        avg_cond = calc_avg(conductivity, cond_list)
        '''Calculates average of conductivity values from GSR sensor'''
        amplitude = conductivity - avg_cond

        '''
            Calculates the amplitude of the indivitual conductivity values to
            the average. Information can be used to create a graph to show the
            trend over time.
        '''
        
        if bpm != None:
            print("BPM:", bpm)
        else:
            print("BPM: NO BEAT DETECTED")
        print("AVG BPM:", avg_bpm)
        print ('GSR: ', round(conductivity, 3), "uS")
        print('AVP Conductivity:', round(avg_cond, 3), "uS")
        print('Conductivity Amplitude', round(amplitude, 3), "uS")
        print("==============================")
        sleep(0.01) # we only get 100 sps so update every 1/100 secs

        if bpm > avg_bpm + 20 or amplitude > 2.0:
            #Tests for further increased heart rate or GSR
            print("Vibrator ON")
            GPIO.output(20, GPIO.HIGH)

            with canvas(device) as draw:
                draw.rectangle((54,20, 55,28), fill="white")
                draw.rectangle((64,20, 65,28), fill="white")
                draw.arc((50,40,70,49),180,0,fill="white")
                '''
                Draws a frowning face on OLED screen. Frowning face
                Represents high stress levels of the user.
                '''

            pygame.mixer.music.load('/home/pi/Downloads/message.mp3')
            pygame.mixer.music.play()
            sleep(5)
            #supportive mesage

            print("Vibrator OFF")
            GPIO.output(20, GPIO.LOW)

        elif bpm > avg_bpm + 15 or amplitude > 1.5:
            '''Tests for continued increase in heart rate or GSR'''
            print("Vibrator ON")
            GPIO.output(20, GPIO.HIGH)
                
            with canvas(device) as draw:
                draw.rectangle((54,20, 55,28), fill="white")
                draw.rectangle((64,20, 65,28), fill="white")
                draw.rectangle((50,34, 70,34), fill="white")
                #OLED = so-so face

            pygame.mixer.music.load('/home/pi/Downloads/be_happy.mp3')
            pygame.mixer.music.play()
            sleep(5)
            pygame.mixer.music.fadeout(2000)
            GPIO.output(20, GPIO.LOW)
            #uplifting music

        
        elif bpm > avg_bpm + 10 or amplitude > 1.0:
            '''Tests for increased heart rate or galvanic skin response (GSR)'''
            print("Vibrator ON")
            GPIO.output(20, GPIO.HIGH)
            sleep(1.5)
            print("Vibrator OFF")
            GPIO.output(20, GPIO.LOW)

            with canvas(device) as draw:
                draw.text((5, 2), "You're doing great!", fill="white")
                draw.rectangle((54,20, 55,28), fill="white")
                draw.rectangle((64,20, 65,28), fill="white")
                draw.arc((50,40,70,49),0,180,fill="white")
                #OLED = happy face
                

        #If the user's heart rate exceeds 110 bpm or drops below 30, LEELA
        #will deem these to be unhealthy values indcitive of some heart abnormality.
        #LEELA will then ask the user if they are fine. LEELA will pause to allow the
        #user to potentially readjust the sensor. If the problem continues, they can
        #reset the program. Otherwise, Leela will alert first responders.
                
        elif bpm < 30 or bpm > 110:
            '''Tests for extremely abnormal resting heart rate values'''
            counter +=1
            '''The counter increases by one every time it goes through the loop
            and if, after the program has run through 40 times, and the user
            still has not reset it, LEELA will alert first responders'''
            if counter %400 == 0:
                with canvas(device) as draw:
                    draw.text((5, 2), "How are you feeling?", fill="white")
                sleep(3)
                with canvas(device) as draw:    
                    draw.text((5, 20), "Reset me if", fill="white")
                    draw.text((5, 38), "you're feeling okay.", fill="white")
                sleep(3)
                with canvas(device) as draw:
                    draw.text((5, 2), "Alerting", fill="white")
                    draw.text((5, 20), "first responders.", fill="white")
                sleep (3)

            '''Not enough functionality for LEELA to actually alert first responders, but
            the logic is demonstrated here.'''

        else:
            with canvas(device) as draw:
                draw.text((5, 2), "Have a great day!", fill="white")
                draw.rectangle((54,20, 55,28), fill="white")
                draw.rectangle((64,20, 65,28), fill="white")
                draw.arc((50,40,70,49),0,180,fill="white")
                #OLED = happy face
                    

                      
if __name__ == "__main__" :
    try:
        setup()
        main()
    except KeyboardInterrupt: 
        pass
