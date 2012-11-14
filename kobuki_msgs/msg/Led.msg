# Sends a command for controlling the a LED.
# 
# Typically the first LED is always reserved to denote
# the state - the remainder will be controllable. 

uint8 BLACK   = 0
uint8 GREEN   = 1
uint8 ORANGE  = 2
uint8 RED     = 3

# For kobuki there are only two controllable LED's.
uint8 value
