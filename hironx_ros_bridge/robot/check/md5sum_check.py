#!/usr/bin/env python

import hashlib
import os
from operator import add
from pprint import pprint


def md5sum_check_dir_to_name(full_dir):
  name = full_dir.replace("/", "_")
  name = name.replace("-", "_")
  if name[0] == "_":
    name = name[1:]
  return name


def md5sum_check_filename(full_dir):
  return "md5sum_" + md5sum_check_dir_to_name(full_dir) + ".py"


def md5sum_file(filename):
  if os.path.exists(filename):
    file_md5 = hashlib.md5(open(filename, 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ", filename, "\t\t", file_md5
  return file_md5


def md5sum_check_dir(dir_info, output_dir=''):
  ret = True
  info = {}

  full_dir = dir_info[0]
  dir_md5 = 0
  print "  Check ", full_dir,
  for root, dirs, files in os.walk(full_dir):
    for file in files:
      filename = os.path.join(root, file)
      file_md5 = md5sum_file(filename)
      info[filename] = file_md5
      dir_md5 = dir_md5 ^ int(file_md5, 16)
  print "   \t(", hex(dir_md5), ")\t",
  if dir_md5 != dir_info[1]:
    print "False"
    ret = False
  else:
    print "Ok"

  filename = md5sum_check_filename(full_dir)

  print "  Writingg results to ... ", os.path.join(output_dir, filename)
  f = open(os.path.join(output_dir, filename), 'wb')
  print >> f, 'info=',
  pprint(info, f, width=240)
  f.close()

  return ret


def md5sum_check_files(full_dir, info):
  ret = True
  for root, dirs, files in os.walk(full_dir):
    for file in files:
      filename = os.path.join(root, file)
      file_md5 = md5sum_file(filename)
      if filename in info.keys():
        if info[filename] != file_md5:
          print " **", filename, "\t\t has changed"
          ret = False
        del info[filename]
      else:
        print " **", filename, "\t\t is possibly newly added (not found on database)."
        ret = False
  for fname in info.keys():
    print " **", fname, "\t\t is missing."
  return ret

# for file in files
#   print [hashlib.md5(open(fname, 'rb').read()).hexdigest() for fname in reduce(add, [[os.path.join(root, file) for file in files] for root, dirs, files in os.walk('$dir')])]"
# done
