# Sends a command for controlling the led array.
# 
# Note that this command will fail if send an array 
# which does not match the number of controllable LED's 
# on the machine.
#
# Typically the first led is always reserved to denote
# the state - the remainder will be controllable. 

uint8 BLACK   = 0
uint8 GREEN   = 1
uint8 ORANGE  = 2
uint8 RED     = 3

# For kobuki there are only two controllable led's.
uint8[] values
