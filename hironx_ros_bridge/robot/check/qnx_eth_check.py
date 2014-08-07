#!/usr/bin/env python

import subprocess, re, sys
def qnx_eth_check():

    info = subprocess.Popen(["ifconfig"], stdout=subprocess.PIPE).communicate()[0]
    print re.sub('(^|\n)', '\n  -', info)

    return "wm0: " in info


