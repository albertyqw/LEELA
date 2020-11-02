''' PLEASE DO NOT UPLOAD THIS ANYWHERE '''

''' MAX30100 I2C ADDRESS '''
MAX30100_ADDR   = 0x57

''' FIFO '''
FIFO_WRITE_REG  = 0x02
FIFO_OVF_REG    = 0x03
FIFO_READ_REG   = 0x04
FIFO_DATA_REG   = 0x05

''' CONFIG '''
MODE_CONFIG_REG = 0x06
SPO2_CONFIG_REG = 0x07
LED_CONFIG_REG  = 0x09

''' MODES '''
MODE_RESET      = 0x40
MODE_HR_ONLY    = 0x02
MODE_SPO2       = 0x03

''' LED CURRENT '''
LED_CURRENT_0_0_MA = 0x00
LED_CURRENT_4_4_MA = 0x01
LED_CURRENT_7_6_MA = 0x02
LED_CURRENT_11_0_MA = 0x03
LED_CURRENT_14_2_MA = 0x04
LED_CURRENT_17_4_MA = 0x05
LED_CURRENT_20_8_MA = 0x06
LED_CURRENT_24_0_MA = 0x07
LED_CURRENT_27_1_MA = 0x08
LED_CURRENT_30_6_MA = 0x09
LED_CURRENT_33_8_MA = 0x0A
LED_CURRENT_37_0_MA = 0x0B
LED_CURRENT_40_2_MA = 0x0C
LED_CURRENT_43_6_MA = 0x0D
LED_CURRENT_46_8_MA = 0x0E
LED_CURRENT_50_0_MA = 0x0F

''' SPO2 SAMPLE RATE '''
SAMPLE_RATE_50_HZ   = 0x00
SAMPLE_RATE_100_HZ  = 0x01
SAMPLE_RATE_167_HZ  = 0x02
SAMPLE_RATE_200_HZ  = 0x03
SAMPLE_RATE_400_HZ  = 0x04
SAMPLE_RATE_600_HZ  = 0x05
SAMPLE_RATE_800_HZ  = 0x06
SAMPLE_RATE_1000_HZ = 0x07


''' PULSE WIDTH '''
PULSE_WIDTH_200_US  = 0x00 # 200us 13 bits
PULSE_WIDTH_400_US  = 0x01 # 400us 14 bits
PULSE_WIDTH_800_US  = 0x02 # 800us 15 bits
PULSE_WIDTH_1_6_MS  = 0x03 # 1.6ms 16 bits 
