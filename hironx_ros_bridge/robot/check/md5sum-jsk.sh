#!/bin/sh

exit 0 # disabled becouse this include hrpsys file that may change when updated
python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files"

f = open('/tmp/check-opt-jsk-md5.txt', 'w')
for dir in [['/opt/jsk/bin',    '57b3069206df621f2c40ac2c1d74e9c8'],
            ['/opt/jsk/etc',	'25dd2d8728a4937aa34b7f3037ff5e09'],
            ['/opt/jsk/include','6986e05543c7a0b8be186fe6ac68440a'],
            ['/opt/jsk/lib',	'c8c60efa2c9d8d4e670c0897ddac4aaf'],
            ['/opt/jsk/share',	'9775299f6bfc32c691eecb5d00f0932c']]
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
f.close

EOF





