#!/usr/bin/env python

import pickle, sys
from md5sum_check import *

if len(sys.argv) < 1:
  print "usaage create-md5sum.py [dir]"

dir_name = sys.argv[1]

md5sum_check_dir([dir_name,    None])

filename = full_dir.replace("/","-") + ".py"
if filename[0] == "-":
  filenaem = filename[1:]
filename = "md5sum-" + filename

print "Writing result to ...", filename



