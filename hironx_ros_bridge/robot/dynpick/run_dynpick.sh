#!/bin/sh

# Written by TORK

slay devc-serusb
sleep 1
devc-serusb -E -F -b 921600 -d busno=0,devno=1,vid=0x10c4,did=0xea60
sleep 1
# stty < /dev/serusb1
./dynpick_driver
# stty < /dev/serusb1
