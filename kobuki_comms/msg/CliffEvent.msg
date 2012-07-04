# Provides a cliff sensor event
# This message is generated whenever the robot approaches or moves away from a cliff

uint8 LEFT   = 0
uint8 CENTER = 1
uint8 RIGHT  = 2

uint8 FLOOR = 0
uint8 CLIFF = 1

uint8  state
uint8  sensor
uint16 bottom