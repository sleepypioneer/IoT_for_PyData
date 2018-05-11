# APDS9960 i2c address
I2C_ADDR = const(0x39)

# APDS9960 gesture parameters
GESTURE_THRESHOLD_OUT = const(10)
GESTURE_SENSITIVITY_1 = const(50)
GESTURE_SENSITIVITY_2 = const(20)

# APDS9960 device IDs
DEV_ID = [0xab, 0x9c, 0xa8]

# APDS9960 times
TIME_FIFO_PAUSE = 0.03

# APDS9960 register addresses
REG_ENABLE = const(0x80)
REG_ATIME = const(0x81)
REG_WTIME = const(0x83)
REG_AILTL = const(0x84)
REG_AILTH = const(0x85)
REG_AIHTL = const(0x86)
REG_AIHTH = const(0x87)
REG_PILT = const(0x89)
REG_PIHT = const(0x8b)
REG_PERS = const(0x8c)
REG_CONFIG1 = const(0x8d)
REG_PPULSE = const(0x8e)
REG_CONTROL = const(0x8f)
REG_CONFIG2 = const(0x90)
REG_ID = const(0x92)
REG_STATUS = const(0x93)
REG_CDATAL = const(0x94)
REG_CDATAH = const(0x95)
REG_RDATAL = const(0x96)
REG_RDATAH = const(0x97)
REG_GDATAL = const(0x98)
REG_GDATAH = const(0x99)
REG_BDATAL = const(0x9a)
REG_BDATAH = const(0x9b)
REG_PDATA = const(0x9c)
REG_POFFSET_UR = const(0x9d)
REG_POFFSET_DL = const(0x9e)
REG_CONFIG3 = const(0x9f)
REG_GPENTH = const(0xa0)
REG_GEXTH = const(0xa1)
REG_GCONF1 = const(0xa2)
REG_GCONF2 = const(0xa3)
REG_GOFFSET_U = const(0xa4)
REG_GOFFSET_D = const(0xa5)
REG_GOFFSET_L = const(0xa7)
REG_GOFFSET_R = const(0xa9)
REG_GPULSE = const(0xa6)
REG_GCONF3 = const(0xaA)
REG_GCONF4 = const(0xaB)
REG_GFLVL = const(0xae)
REG_GSTATUS = const(0xaf)
REG_IFORCE = const(0xe4)
REG_PICLEAR = const(0xe5)
REG_CICLEAR = const(0xe6)
REG_AICLEAR = const(0xe7)
REG_GFIFO_U = const(0xfc)
REG_GFIFO_D = const(0xfd)
REG_GFIFO_L = const(0xfe)
REG_GFIFO_R = const(0xff)

# APDS9960 bit fields
BIT_PON = const(0b00000001)
BIT_AEN = const(0b00000010)
BIT_PEN = const(0b00000100)
BIT_WEN = const(0b00001000)
APSD9960_BIT_AIEN = const(0b00010000)
BIT_PIEN = const(0b00100000)
BIT_GEN = const(0b01000000)
BIT_GVALID = const(0b00000001)

# APDS9960 modes
MODE_POWER = const(0)
MODE_AMBIENT_LIGHT = const(1)
MODE_PROXIMITY = const(2)
MODE_WAIT = const(3)
MODE_AMBIENT_LIGHT_INT = const(4)
MODE_PROXIMITY_INT = const(5)
MODE_GESTURE = const(6)
MODE_ALL = const(7)

# LED Drive values
LED_DRIVE_100MA = const(0)
LED_DRIVE_50MA = const(1)
LED_DRIVE_25MA = const(2)
LED_DRIVE_12_5MA = const(3)

# Proximity Gain (PGAIN) values
PGAIN_1X = const(0)
PGAIN_2X = const(1)
PGAIN_4X = const(2)
PGAIN_8X = const(3)

# ALS Gain (AGAIN) values
AGAIN_1X = const(0)
AGAIN_4X = const(1)
AGAIN_16X = const(2)
AGAIN_64X = const(3)

# Gesture Gain (GGAIN) values
GGAIN_1X = const(0)
GGAIN_2X = const(1)
GGAIN_4X = const(2)
GGAIN_8X = const(3)

# LED Boost values
LED_BOOST_100 = const(0)
LED_BOOST_150 = const(1)
LED_BOOST_200 = const(2)
LED_BOOST_300 = const(3)    

# Gesture wait time values
GWTIME_0MS = const(0)
GWTIME_2_8MS = const(1)
GWTIME_5_6MS = const(2)
GWTIME_8_4MS = const(3)
GWTIME_14_0MS = const(4)
GWTIME_22_4MS = const(5)
GWTIME_30_8MS = const(6)
GWTIME_39_2MS = const(7)

# Default values
DEFAULT_ATIME = const(219)                            # 103ms
DEFAULT_WTIME = const(246)                            # 27ms
DEFAULT_PROX_PPULSE = const(0x87)                     # 16us, 8 pulses
DEFAULT_GESTURE_PPULSE = const(0x89)                  # 16us, 10 pulses
DEFAULT_POFFSET_UR = const(0)                         # 0 offset
DEFAULT_POFFSET_DL = const(0)                         # 0 offset
DEFAULT_CONFIG1 = const(0x60)                         # No 12x wait (WTIME) factor
DEFAULT_LDRIVE = const(LED_DRIVE_100MA)
DEFAULT_PGAIN = const(PGAIN_4X)
DEFAULT_AGAIN = const(AGAIN_4X)
DEFAULT_PILT = const(0)                               # Low proximity threshold
DEFAULT_PIHT = const(50)                              # High proximity threshold
DEFAULT_AILT = const(0xffff)                          # Force interrupt for calibration
DEFAULT_AIHT = const(0)
DEFAULT_PERS = const(0x11)                           # 2 consecutive prox or ALS for int.
DEFAULT_CONFIG2 = const(0x01)                        # No saturation interrupts or LED boost  
DEFAULT_CONFIG3 = const(0)                           # Enable all photodiodes, no SAI
DEFAULT_GPENTH = const(40)                           # Threshold for entering gesture mode
DEFAULT_GEXTH = const(30)                            # Threshold for exiting gesture mode    
DEFAULT_GCONF1 = const(0x40)                         # 4 gesture events for int., 1 for exit
DEFAULT_GGAIN = const(GGAIN_4X)
DEFAULT_GLDRIVE = const(LED_DRIVE_100MA)
DEFAULT_GWTIME = const(GWTIME_2_8MS)
DEFAULT_GOFFSET = const(0)                           # No offset scaling for gesture mode
DEFAULT_GPULSE = const(0xc9)                         # 32us, 10 pulses
DEFAULT_GCONF3 = const(0)                            # All photodiodes active during gesture
DEFAULT_GIEN = const(0)                              # Disable gesture interrupts

# gesture directions
DIR_NONE = const(0)
DIR_LEFT = const(1)
DIR_RIGHT = const(2)
DIR_UP = const(3)
DIR_DOWN = const(4)
DIR_NEAR = const(5)
DIR_FAR = const(6)
DIR_ALL = const(7)

# state definitions
STATE_NA = const(0)
STATE_NEAR = const(1)
STATE_FAR = const(2)
STATE_ALL = const(3)
