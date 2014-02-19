#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /usr/pkg files"

f = open('/tmp/check-usr-pkg-md5.txt', 'w')
for dir in [['/usr/pkg/bin',    '91139a9c30f34dffc77c5c2f503490b9'],
            ['/usr/pkg/include','571b30ce3d36b39462bddc8594e2d1a9'],
            ['/usr/pkg/lib',    '8df0558a7a5cc363edda1faa0f51ea2c'],
            ['/usr/pkg/sbin',   'd26912c9261a982f5314062c768db1c6'],
            ['/usr/pkg/etc',    'e3368de29e1af11f6abfe18e1429a71a'],
            ['/usr/pkg/info',   '5d4bf3fef0568d72f8d42199e46d6a72'],
            ['/usr/pkg/man',    'cb127126cfb1cffde510f0d9cc98ef27'],
            ['/usr/pkg/share',  '81d02f473ac239886f36a2498d73d5be']]:
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
#  for file in files
# print [hashlib.md5(open(fname, 'rb').read()).hexdigest() for fname in reduce(add, [[os.path.join(root, file) for file in files] for root, dirs, files in os.walk('$dir')])]"
#done

EOF




