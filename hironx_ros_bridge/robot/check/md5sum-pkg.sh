#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /usr/pkg files"

f = open('/tmp/check-usr-pkg-md5.txt', 'w')
for dir in [['/usr/pkg/bin',    0x3d8eff565b05b7532b3107b20589cbe6L],
            ['/usr/pkg/include',0xcf89f4f17f1c569afc8ec3ad8a5ec10aL],
            ['/usr/pkg/lib',    0x5facb204c74dce823ebe7fef06d80cffL],
            ['/usr/pkg/sbin',   0x58a3b20eafea4170215f820182f68483L],
            ['/usr/pkg/etc',    0xbeef94e03b158101a77526be67d3b9ecL],
            ['/usr/pkg/info',   0x3e0519bf78f3258cc72f3d095e942ed8L],
            ['/usr/pkg/man',    0xb97c0f124a4170abd7c170fc882b7425L],
            ['/usr/pkg/share',  0x814de4e7c790d11dc4764139b5fb3625L]]:
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
f.close
#  for file in files
# print [hashlib.md5(open(fname, 'rb').read()).hexdigest() for fname in reduce(add, [[os.path.join(root, file) for file in files] for root, dirs, files in os.walk('$dir')])]"
#done

EOF




