# Kobuki Sensor Data Messages
#
uint8 F0 = 2  # bitmask 0x02
uint8 F1 = 1  # bitmask 0x01
uint8 F2 = 4  # bitmask 0x04

Header header
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
