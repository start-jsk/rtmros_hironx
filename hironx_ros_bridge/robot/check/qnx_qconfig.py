#!/usr/bin/env python

import subprocess, re, sys

def check_qconfig(config, search_key, expected_str):
    global ret

    tag =  re.findall(search_key, config)
    if tag :
        tag = tag[-1]
    else:
        tag = "Could nof find " + search_key

    print "%s\t\t\t\t" % (tag),
    if tag == expected_str :
        print "Ok"
    else:
        print "False"
        ret = False

def qnx_qconfig() :
    global ret

    config = subprocess.Popen(["qconfig", "-a"], stdout=subprocess.PIPE).communicate()[0]

    # installation-name: QNX Software Development Platform 6.5.0
    # installation-base: /usr/qnx650/
    # installation-host: /usr/qnx650/host/qnx6/x86/
    # installation-target: /usr/qnx650/target/qnx6/

    ret = True

    print "  Check Installation Name .. ",
    check_qconfig(config, "installation-name: (.+)", "QNX Software Development Platform 6.5.0")

    print "  Check Installation Base .. ",
    check_qconfig(config, "installation-base: (.+)", "/usr/qnx650/")

    print "  Check Installation Host .. ",
    check_qconfig(config, "installation-host: (.+)", "/usr/qnx650/host/qnx6/x86/")

    print "  Check Installation Target .. ",
    check_qconfig(config, "installation-target: (.+)", "/usr/qnx650/target/qnx6/")

    print re.sub('(^|\n)', '\n  -', config)
    return ret
