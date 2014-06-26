#!/usr/bin/env python

import sys
from md5sum_check import *

if len(sys.argv) < 1:
  print "usaage create-md5sum.py [dir]"

dir_name = sys.argv[1]

md5sum_check_dir([dir_name,    None])

print "Writing result to ...", md5sum_check_filename(dir_name)



