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

# slay devc-serusb
# sleep 1
## devc-serusb needs to be that of QNX6.5.0 SP1, to recognize Dynpick WDF-6M200-3, which is USB-Serial with 921.6kbps.
## And looks like we don't need to specifically call it as long as right version of /sbin/devc-serusb is placed.
# devc-serusb -E -F -b 921600 -d busno=0,devno=1,vid=0x10c4,did=0xea60
# sleep 1
# stty < /dev/serusb1
./dynpick_driver &
# stty < /dev/serusb1
