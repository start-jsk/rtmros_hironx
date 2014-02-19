#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk/etc/HIRONX model files"

f = open('/tmp/check-opt-jsk-etc-hironx-md5.txt', 'w')
for dir in [['/opt/jsk/etc/HIRONX',	   '2b2e658a22a6587094aa17ee775dd04f']]:
  full_dir = dir[0]
  dir_md5 = hashlib.md5()
  for root, dirs, files in os.walk(full_dir):
    for file in files:
      if os.path.exists(os.path.join(root,file)):
        file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
      else:
        file_md5 = 'No such file or directory'
        print "  ** ",os.path.join(root,file),"\t\t",file_md5
      print >>f, os.path.join(root,file),"\t\t",file_md5
      dir_md5.update(file_md5)
  print "  Check ", full_dir, "   \t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()==dir[1]

EOF


