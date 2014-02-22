#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files (OpenHRP3)"

f = open('/tmp/check-opt-jsk-oepnhrp3-md5.txt', 'w')
for dir in [['/opt/jsk/include/OpenHRP-3.1',    0xa20be86660da52f7eec582fbcefecbbdL],
            ['/opt/jsk/share/OpenHRP-3.1',      0xf4312237b7075c23dcf0000761c7ee1dL]]:
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

root = '/opt/jsk/lib'
dir_md5 = 0
for file in ['pkgconfig/openhrp3.1.pc','libhrpCorbaStubSkel-3.1.a','libhrpUtil-3.1.so.0.0.0','libhrpUtil-3.1.so.0','libhrpUtil-3.1.so','libhrpCollision-3.1.so.0.0.0','libhrpCollision-3.1.so.0','libhrpCollision-3.1.so','libhrpModel-3.1.so.0.0.0','libhrpModel-3.1.so.0','libhrpModel-3.1.so','SimulationEC.so.0.0.0','SimulationEC.so.0','SimulationEC.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0x101bbeeb3a8e2357942115caca7e7f9aL

root = '/opt/jsk/bin'
dir_md5 = 0
for file in ['openhrp-model-loader','export-vrml','openhrp-collision-detector','openhrp-shutdown-servers','openhrp-jython-prompt']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0x2c67d749a0185e7854ce828469be2432L

EOF


