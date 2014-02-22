#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/hiro files"

f = open('/tmp/check-opt-hiro-md5.txt', 'w')
root = '/opt/hiro/lib'
dir_md5 = 0
for file in ['libhrpIo.so.315.1.7']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0xe2e07b593dbbf6bbe18328af441af645L

root = '/opt/hiro/bin'
dir_md5 = 0
for file in ['10param.csv','11offsets.csv','12gain.csv','13limit.csv','StatusPluginParam','calibCurrent.csv','pci7020bios020X1141fix','shared','shm_iob_hiro']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0x96421843a98240ec39b15cc46ec782d7L

EOF


