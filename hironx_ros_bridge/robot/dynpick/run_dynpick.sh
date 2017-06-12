#!/bin/sh

# This program is for Dynpick F/T sensor for HiroNXO robot on QNX.
# Copyright (c) 2016, TORK (Tokyo Opensource Robotics Kyokai Association)
# All rights reserved.
# * Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# * * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
# * Neither the name of TOKYO. nor the names of its contributors may be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Written by TORK

# 201706 We found that "slay -f devc-serusb" occasionally fails, which results in failing this entire 
#        script. Instead of slaying the process started during OS initialization steps, we found that
#        not starting devc-serusb for the f/t sensor device works. To do so, we manually edited
#        /etc/system/enum/devices/usb/char file.
#        NOTE that this is a hack and still being experimented. How to modify the aforementioned file
#        is only documented in a private ticket https://github.com/tork-a/delivery/issues/436#issuecomment-306789518

#echo "Before slay devc-serusb"
#ps -ef | grep devc-serusb
#slay -f devc-serusb
sleep 3
#echo "After slay devc-serusb"
ps -ef | grep devc-serusb
## devc-serusb needs to be that of QNX6.5.0 SP1, to recognize Dynpick WDF-6M200-3, which is USB-Serial with 921.6kbps.

#devc-serusb -E -F -b 921600 -d busno=4,devno=2,unit=1,vid=0x10c4,did=0xea60
# Sensors' USB directly connected to qnx, busno might be 3
#devc-serusb -E -F -b 921600 -d busno=3,devno=1,unit=1,vid=0x10c4,did=0xea60
#devc-serusb -E -F -b 921600 -d busno=3,devno=2,unit=2,vid=0x10c4,did=0xea60

# Sensors' USB connected via USB hub, busno might become 4. 
# {r, t}x_urbs seem to be necessary, to avoid error "serusb_reader: URB not hooked for port /dev/serusb1 - sts = Unknown error, pflags =0x4a7"
devc-serusb -E -F -b 921600 -d busno=4,devno=2,unit=1,vid=0x10c4,did=0xea60,rx_urbs=50,tx_urbs=50
sleep 3
devc-serusb -E -F -b 921600 -d busno=4,devno=3,unit=2,vid=0x10c4,did=0xea60,rx_urbs=50,tx_urbs=50
sleep 3
echo "After calling devc-serusb"
ps -ef | grep devc-serusb
# stty < /dev/serusb1
./dynpick_driver &
# stty < /dev/serusb1
sleep 3
