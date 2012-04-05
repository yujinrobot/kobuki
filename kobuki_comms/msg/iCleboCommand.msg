# iClebo Header IDs
#
uint8 commandBaseControl    = 1
uint8 commandCleaningMotor  = 2
uint8 commandSound          = 3
uint8 commandSoundSequence  = 4
uint8 commandAnimation      = 5
uint8 commandDisplay        = 6
uint8 commandSetTime        = 7
uint8 commandSetPower       = 8
uint8 commandRequestExtra   = 9
uint8 commandChangeFrame    = 10
uint8 commandRequestEeprom  = 11
uint8 commandSetDigitalOut  = 12

##
uint8 command

# commandBaseControl    
uint16 speed
uint16 radius

# commandCleaningMotor  
uint8 brush_power
uint8 vacuum_power
uint8 side_brush_power

# commandSound          
uint16 note
uint8 duration

# commandSoundSequence  
uint8 segment_name

# commandAnimation      
uint8 animation_code
uint8 options

# commandDisplay    
uint8[] digits
uint8[] tmp_intensity
    
# commandSetTime        
uint8 hh
uint8 mm

# commandSetPower       
uint16 power_control_flags

# commandRequestExtra   
uint16 request_flags

# commandChangeFrame & commandRequestEeprom  
uint8 frame_id

# commandSetDigitalOut  
uint16 gp_out
