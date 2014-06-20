#!/usr/bin/env python

import hashlib, pickle, os
from operator import add

def md5sum_check_dir(dir_info):
  ret = True
  info = []
  
  full_dir = dir_info[0]
  dir_md5 = 0
  print "  Check ", full_dir, 
  for root, dirs, files in os.walk(full_dir):
    for file in files:
      if os.path.exists(os.path.join(root,file)):
        file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
      else:
        file_md5 = 'No such file or directory'
        print "  ** ",os.path.join(root,file),"\t\t",file_md5
      info.append([os.path.join(root,file),file_md5])
      dir_md5 = dir_md5 ^ int(file_md5, 16)
  print "   \t(", hex(dir_md5), ")\t", 
  if dir_md5 != dir_info[1]:
    print "False"
    ret = False
  else:
    print "Ok"

  filename = full_dir.replace("/","-") + ".py"
  if filename[0] == "-":
    filenaem = filename[1:]
  filename = "md5sum-" + filename

  print filename
  f = open(filename, 'wb')
  pickle.dump(info, f)
  f.close()

  return ret

# for file in files
#   print [hashlib.md5(open(fname, 'rb').read()).hexdigest() for fname in reduce(add, [[os.path.join(root, file) for file in files] for root, dirs, files in os.walk('$dir')])]"
# done






