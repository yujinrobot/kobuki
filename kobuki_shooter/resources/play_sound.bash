#!/bin/bash

# Set the volume to max
amixer -c 0 sset PCM,0 100% >/dev/null
amixer -c 0 sset Master,0 100% >/dev/null

# Then play whatever
aplay $1 >/dev/null &
