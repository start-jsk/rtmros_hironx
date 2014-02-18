#!/bin/sh


python <<EOF

import hashlib
import os
from operator import add

print "* Check /usr/pkg files"

f = open('/tmp/check-usr-pkg-md5.txt', 'w')
for dir in [['/usr/pkg/bin',    'e846c663623045c0846353f8d1187c0c'],
            ['/usr/pkg/include','461ea43f9de05ef519c487f5e84bd988'],
            ['/usr/pkg/lib',    'cc0f135dc109309132292fe2834afbda'],
            ['/usr/pkg/sbin',   'c85c04e21d100b9a9d40da813ff2ec04'],
            ['/usr/pkg/etc',    '6fa3a22c96ae700e95c3007f2da3cc11'],
            ['/usr/pkg/info',   '56e7ec0e27be98c0e51f17cd21b652b8'],
            ['/usr/pkg/man',    '9b9a2be866fd83d13d826657fba8367f'],
            ['/usr/pkg/share',  '1aff998cad709c0a8d8f95138b01e5a5']]:
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




