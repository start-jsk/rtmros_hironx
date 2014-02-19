#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk/etc/HIRONX model files"

f = open('/tmp/check-opt-jsk-etc-hironx-md5.txt', 'w')
for dir in [['/opt/jsk/etc/HIRONX',	   0xabcbf0c9b8bd8e8bef01be7b9e4f1641L]]:
  full_dir = dir[0]
  dir_md5 = 0
  for root, dirs, files in os.walk(full_dir):
    for file in files:
      if os.path.exists(os.path.join(root,file)):
        file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
      else:
        file_md5 = 'No such file or directory'
        print "  ** ",os.path.join(root,file),"\t\t",file_md5
      print >>f, os.path.join(root,file),"\t\t",file_md5
      dir_md5 = dir_md5 ^ int(file_md5, 16)
  print "  Check ", full_dir, "   \t(", hex(dir_md5), ")\t", dir_md5==dir[1]

EOF


