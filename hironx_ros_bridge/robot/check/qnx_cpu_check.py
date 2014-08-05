#!/usr/bin/env python

import subprocess, re, sys

def check_pinfo(info, search_key, expected_str):
    global ret
    tag =  re.findall(search_key, info)
    if tag :
        tag = tag[-1]
    else:
        tag = "Could nof find " + search_key

    print "%s\t\t\t\t\t\t" % (tag),
    if tag == expected_str :
        print "Ok"
    else:
        print "False"
        ret = False

def qnx_cpu_check() :
    global ret

    info = subprocess.Popen(["pidin", "info"], stdout=subprocess.PIPE).communicate()[0]

    # CPU:X86 Release:6.5.0  FreeMem:833Mb/1023Mb BootTime:Apr 27 22:29:28 UTC 2014
    # Processes: 37, Threads: 98
    # Processor1: 131758 Intel 686 F6M15S2 1969MHz FPU

    ret = True
    
    print "  Check CPU Type .. ",
    check_pinfo(info, 'CPU:\S+', "CPU:X86")

    print "  Check Memory Size .. ",
    check_pinfo(info, '(?<=Mb\/)\S+', "3318Mb")  # Positive-lookbehind assertion

    print "  Check OS Release .. ",
    check_pinfo(info, 'Release:\S+', "Release:6.5.0")

    print "  Check Number of CPU  .. ",
    check_pinfo(info, 'Processor\S+:', "Processor2:")

    print re.sub('(^|\n)', '\n  -', info)

    return ret



