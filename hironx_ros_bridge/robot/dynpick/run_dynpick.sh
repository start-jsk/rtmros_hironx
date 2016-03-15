#!/bin/sh

slay devc-serusb_qnx650sp1
sleep 1
./devc-serusb_qnx650sp1  -E -F -b 921600 -d module=ftdi,busno=0,devno=1,vid=0x0403,did=0x6001
sleep 1
stty < /dev/serusb1
make
./dynpick_driver
stty < /dev/serusb1


