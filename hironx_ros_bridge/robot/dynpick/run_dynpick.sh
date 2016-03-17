#!/bin/sh

# Written by TORK

# slay devc-serusb
# sleep 1
## devc-serusb needs to be that of QNX6.5.0 SP1, to recognize Dynpick WDF-6M200-3, which is USB-Serial with 921.6kbps.
## And looks like we don't need to specifically call it as long as right version of /sbin/devc-serusb is placed.
# devc-serusb -E -F -b 921600 -d busno=0,devno=1,vid=0x10c4,did=0xea60
# sleep 1
# stty < /dev/serusb1
./dynpick_driver &
# stty < /dev/serusb1
