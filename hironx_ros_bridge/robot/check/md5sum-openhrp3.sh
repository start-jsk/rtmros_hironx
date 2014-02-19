#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files (OpenHRP3)"

f = open('/tmp/check-opt-jsk-oepnhrp3-md5.txt', 'w')
for dir in [['/opt/jsk/include/OpenHRP-3.1',    '34ee2a6c7eab15c42a4b2d0b0caa53c7'],
            ['/opt/jsk/share/OpenHRP-3.1',      '25a0851d9fec96db6f67711726190337']]:
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

root = '/opt/jsk/lib'
dir_md5 = hashlib.md5()
for file in ['pkgconfig/openhrp3.1.pc','libhrpCorbaStubSkel-3.1.a','libhrpUtil-3.1.so.0.0.0','libhrpUtil-3.1.so.0','libhrpUtil-3.1.so','libhrpCollision-3.1.so.0.0.0','libhrpCollision-3.1.so.0','libhrpCollision-3.1.so','libhrpModel-3.1.so.0.0.0','libhrpModel-3.1.so.0','libhrpModel-3.1.so','SimulationEC.so.0.0.0','SimulationEC.so.0','SimulationEC.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='bd864d88cef39b9bca6c97f7e5f725f1'

root = '/opt/jsk/bin'
dir_md5 = hashlib.md5()
for file in ['openhrp-model-loader','export-vrml','openhrp-collision-detector','openhrp-shutdown-servers','openhrp-jython-prompt']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='ead09b2eebe3b6c2da80282367036584'


EOF


