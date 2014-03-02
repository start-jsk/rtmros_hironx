#!/bin/sh

echo "* Create MD5 sum files"

python <<EOF

### BEGIN System Check Variables

dirs_to_check = ['/opt/hiro/bin', '/usr/pkg/bin', '/usr/pkg/include', '/usr/pkg/lib', '/usr/pkg/sbin', '/usr/pkg/etc', '/usr/pkg/info', '/usr/pkg/man', '/usr/pkg/share']

### END

import hashlib
import re
import os
from operator import add


dir_md5 = 0
for dir in dirs_to_check:
  file_name = '/tmp/check'+re.sub('/','-',dir)+'.txt'
  f = open(file_name, 'w')
  print "* Check",dir,"files"
  for root, dirs, files in os.walk(dir):
    for file in files:
      file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
      #print os.path.join(root,file),"\t\t",file_md5
      dir_md5 = dir_md5 ^ int(file_md5, 16)
      print >>f, os.path.join(root,file),"\t",file_md5
  print "  Check ", root, "   \t(", hex(dir_md5), ")"
  f.close()
  file_rename = '/tmp/check'+re.sub('/','-',dir)+'-'+hex(dir_md5)+'.txt'
  os.rename(file_name, file_rename)
  print "* Check",dir,"files and write to", file_rename
EOF
