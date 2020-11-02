''' PLEASE DO NOT UPLOAD THIS ANYWHERE '''
import smbus
from time import sleep, time
from filters import DCFilter, MeanDiffFilter, ButterworthFilter
import math
from max30100_regs import *

''' SOME ADJUSTABLE PARAMETERS '''
MAX_THRESHOLD   = 45.0
MIN_THRESHOLD   = 20.0
CUT_OFF         = 200.0
MAX_BPM         = 220.0
SPO2_N_BEATS    = 10
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

# SPO2 Lookup Table
spo2_lut = [100,100,100,100,99,99,99,99,99,99,98,98,98,98,98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,95,95,95,95,94,94,94,94,94,93,93,93,93,93,92,92,92,92,92,91,91,91,91,91]


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
        self.spo2 = 0.0

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
            return None
        elif self.detect_beat():
            bpm = 60/(self.beat_timer.get_time()) 
            if bpm < MAX_BPM:
                self.bpm = bpm
            else:
                return None
            if len(self.bpm_queue) > QUEUE_SIZE:
                self.bpm_queue = self.bpm_queue[1:]
            self.bpm_queue.append(self.bpm)
            self.beat_timer.reset()
        return self.bpm            

    def get_avg_bpm(self):
        ''' Calculates the average heart rate in BPM'''
        if len(self.bpm_queue) == 0: return None
        return sum(self.bpm_queue)/len(self.bpm_queue)
   
    def calculate_spo2(self):
        ''' 
            Calculates the SPO2 value (Not sure how reliable this is)
            Source: https://github.com/oxullo/Arduino-MAX30100 
        '''
        self.ir_ac2_sum += self.ir_ac ** 2
        self.red_ac2_sum += self.red_ac ** 2
        self.samples_recorded += 1
        if self.beats_detected == SPO2_N_BEATS:
            self.ac_sq_ratio = 100.0 * math.log(self.red_ac2_sum/self.samples_recorded) / math.log(self.ir_ac2_sum/self.samples_recorded)
            index = 0
            if self.ac_sq_ratio > 66:
                index = self.ac_sq_ratio - 66
            elif self.ac_sq_ratio > 50:
                index = self.ac_sq_ratio - 50
            self.samples_recorded = 0
            self.ir_ac2_sum = 0.0
            self.red_ac2_sum = 0.0
            self.beats_detected = 0.0
            self.spo2 = 0.0
            self.spo2 = spo2_lut[int(index)]
        return self.spo2

def main():
    pulseox = MAX30100()
    while True:
        pulseox.update() # updates the sensor readings
        bpm = pulseox.get_bpm() # Also Updates the values as well
        avg_bpm = pulseox.get_avg_bpm()
        ir_filtered, red_filtered = pulseox.get_filtered()
        spo2 = pulseox.calculate_spo2()
        if bpm != None:
            print("BPM:", bpm)
        else:
            print("BPM: NO BEAT DETECTED")
        print("AVG BPM:", avg_bpm)
        print("SPO2:", spo2)
        print("IR:", ir_filtered)
        print("RED:", red_filtered)
        print("====================")
        sleep(0.1) # we only get 100 sps so update every 1/100 secs

if __name__ == "__main__" :
    main()
    
