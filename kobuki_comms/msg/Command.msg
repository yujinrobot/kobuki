# Kobuki Header IDs
#
uint8 commandBaseControl    = 1
uint8 commandSound          = 3
uint8 commandSoundSequence  = 4
uint8 commandRequestExtra   = 9
uint8 commandChangeFrame    = 10
uint8 commandRequestEeprom  = 11
uint8 commandSetDigitalOut  = 12

##
uint8 command

# commandBaseControl    
uint16 speed
uint16 radius

# commandSound          
uint16 note
uint8 duration

# commandSoundSequence  
uint8 segment_name

# commandRequestExtra   
uint16 request_flags

# commandChangeFrame & commandRequestEeprom  
uint8 frame_id

# commandSetDigitalOut  
uint16 gp_out
