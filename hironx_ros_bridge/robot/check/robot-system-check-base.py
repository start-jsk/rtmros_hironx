#!/usr/bin/env python

import sys, pickle
from qnx_cpu_check import *
from qnx_qconfig import *
from md5sum_check import *
#from md5sum_check_files import *


# run cpu check
print "* Check CPU Info"
ret = qnx_cpu_check()

# run qnx config
print "* Check QNX Info"
ret = qnx_qconfig() and ret

print "* Check /usr/local directories"
ret = md5sum_check_dir(['/usr/local',      0x1848d9debc509a0f04620f499d14ceb2L]) and ret

print "* Check /usr/pkg directories"
ret = md5sum_check_dir(['/usr/pkg',         0x91f33b46b798da7d176e55367144903L]) and ret
ret = md5sum_check_dir(['/usr/pkg/bin',    0x3d8eff565b05b7532b3107b20589cbe6L]) and ret
ret = md5sum_check_dir(['/usr/pkg/etc',    0xbeef94e03b158101a77526be67d3b9ecL]) and ret
ret = md5sum_check_dir(['/usr/pkg/include',0xe23c6c662a7acf846d4dd25b63fbe6f4L]) and ret
ret = md5sum_check_dir(['/usr/pkg/info',   0x3e0519bf78f3258cc72f3d095e942ed8L]) and ret
ret = md5sum_check_dir(['/usr/pkg/lib',    0x36d574207bdbb13bc5b898c9870735a6L]) and ret
ret = md5sum_check_dir(['/usr/pkg/man',    0xb97c0f124a4170abd7c170fc882b7425L]) and ret
ret = md5sum_check_dir(['/usr/pkg/sbin',   0x58a3b20eafea4170215f820182f68483L]) and ret
ret = md5sum_check_dir(['/usr/pkg/share',  0x814de4e7c790d11dc4764139b5fb3625L]) and ret

print "* Check /opt/nextage-open directories"
ret = md5sum_check_dir(['/opt/nextage-open',    0x5644cb667d7241ebec88d79b610b4558L]) and ret

print "* Check /usr/local files"
#ret = md5sum_check_files() and ret

print "* Check /usr/pkg files"

print ""
print "---"
print ""
print "Done all test, Result is ... ", ret
print "Saving results...."


sys.exit(ret)






