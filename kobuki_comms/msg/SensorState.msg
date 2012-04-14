Header header

###################
# Core Packet
###################
uint16 time_stamp
uint8  bump
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
uint16 gp_input
uint16[] gp_adc
