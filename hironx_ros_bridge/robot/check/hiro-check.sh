#!/bin/sh

echo "* Check Hiro"

### BEGIN test-hiroio test program

cat <<EOF > /tmp/test-hrpio.cpp
#include <stdio.h>
#include <sys/types.h>
#include <io/iob.h>
int main() {
  open_iob();
  close_iob();
}
EOF

### END

python <<EOF

### BEGIN System Check Variables

opt_hiro_bin_files = ['10param.csv','11offsets.csv','12gain.csv','13limit.csv','StatusPluginParam','calibCurrent.csv','pci7020bios020X1141fix','shared','shm_iob_hiro']
opt_hiro_bin_md5sum = 0x96421843a98240ec39b15cc46ec782d7L
libhrpio_md5sum     = 0xe2e07b593dbbf6bbe18328af441af645L
libhrpio_size       = 1907174

### END

print "  Check md5sum of /opt/hiro/bin\t\t\t\t"

import hashlib
import os
from operator import add

f = open('/tmp/check-opt-hiro-bin-md5.txt', 'w')
dir_md5 = 0
for root, dirs, files in os.walk('/opt/hiro/bin'):
  for file in files:
    file_md5 = '1'
    if not os.path.exists(os.path.join(root,file)):
      print "  ** ",os.path.join(root,file),"\t\tNo such file or directory"
    elif not file in opt_hiro_bin_files:
      print os.path.join(root,file),"\t\t\t\t\t\t\t",file_md5, " *ERROR* Untracked file is found"
    else:
      file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
      print os.path.join(root,file),"\t\t",file_md5
    dir_md5 = dir_md5 ^ int(file_md5, 16)
    print >>f, os.path.join(root,file),"\t\t",file_md5
  print "  Check ", root, "   \t(", hex(dir_md5), ")\t", dir_md5==opt_hiro_bin_md5sum
f.close()

print "* Check /opt/hiro/lib/libhrpIo.so"
hiro_libhrpio_md5sum = hashlib.md5(open('/opt/hiro/lib/libhrpIo.so','rb').read()).hexdigest()
hiro_libhrpio_size   = os.path.getsize('/opt/hiro/lib/libhrpIo.so')
print "\t   size $size\t\t\t\t\t",  hiro_libhrpio_size, "\t",hiro_libhrpio_size == libhrpio_size
print "\t md5sum $md5sum\t\t", hiro_libhrpio_md5sum, "\t", int(hiro_libhrpio_md5sum,16) == libhrpio_md5sum



EOF

echo -n "  Check /opt/hiro/lib/libhrpIo.so is compilable\t\t\t\t"

gcc -o /tmp/test-hrpio /tmp/test-hrpio.cpp -I/opt/jsk/include/hrpsys -L/usr/pkg/lib -L/opt/hiro/lib -lhrpIo -lboost_thread -lboost_signals -lboost_filesystem -lboost_system -lboost_regex -lf2c
ret=$?

if [ $ret == 0 ] ;then
  echo "OK"
else
  echo "False"
  ret=1
fi 


# ldd /opt/hiro/lib/libhrpIo.so  2>&1  | grep ldd
echo -n "  Check ldd of /opt/hiro/lib/libhrpIo.so\t\t\t\t"
LD_LIBRARY_PATH=/usr/pkg/lib:/opt/jsk/lib ldd /opt/hiro/lib/libhrpIo.so  2>&1  | tee | grep ldd
ret=$?

if [ $ret == 1 ] ;then
  echo "OK"
else
  echo "False"
  ret=1
fi 

