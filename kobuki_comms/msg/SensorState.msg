# Kobuki Sensor Data Messages
#

# Buttons bitmasks to decode byte "buttons"
uint8 F0 = 2  # 0x02
uint8 F1 = 1  # 0x01
uint8 F2 = 4  # 0x04

# Byte "charger" format:
# - first four bits distinguish between adapter or docking base charging
uint8 ADAPTER     = 16 # bitmask 0x10
# - last 4 bits specified the charging status
uint8 DISCHARGING = 0
uint8 CHARGED     = 2
uint8 CHARGING    = 6


Header header

###################
# Core Packet
###################
uint16 time_stamp
uint8  bumper
uint8  wheel_drop
uint8  cliff
uint16 left_encoder
uint16 right_encoder
int8  left_pwm
int8  right_pwm
uint8  buttons
uint8  charger
uint8  battery

###################
# Cliff Packet
###################
uint16[] bottom

###################
# Current Packet
###################
uint8[] current

###################
# GP Input Packet
###################
uint16 digital_input
uint16[] analog_input
