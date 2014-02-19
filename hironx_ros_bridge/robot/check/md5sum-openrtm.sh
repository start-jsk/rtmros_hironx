#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files (openrtm and base libraries)"

f = open('/tmp/check-opt-jsk-md5.txt', 'w')
for dir in [['/opt/jsk/include/openrtm-1.1',	0xe3a414e7aa8d737dc0539928e1a46d87L],
            ['/opt/jsk/include/coil-1.1',	0x615ea973d8fe760103c2321e0bf5a6ceL],
            ['/opt/jsk/include/eigen3',         0x4c6b8ba8abaf09f93c72f1e837b10421L],
            ['/opt/jsk/include/omniORB4',	0x52e7fef1b9a95550154dbef554c2169fL],
            ['/opt/jsk/include/omnithread',	0x46be0a324d3f28430fa5ae58593e0448L],
            ['/opt/jsk/include/COS',            0x8b12f85f9e8e28618b81f98af9245588L],
            ['/opt/jsk/include/uuid',           0x300b89d8dbcb69b11bc970cfd7fc6a42L],
            ['/opt/jsk/share/openrtm-1.1',	0x4160c166397aea61798f2313f5948b10L]]:
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
for file in ['libCOS4.a','libCOS4.so','libCOS4.so.1','libCOS4.so.1.6','libCOSDynamic4.a','libCOSDynamic4.so','libCOSDynamic4.so.1','libCOSDynamic4.so.1.6','libRTC-1.1.0.so','libRTC.a','libRTC.la','libRTC.so','libblas.a','libcoil-1.1.0.so','libcoil.a','libcoil.la','libcoil.so','libdynamicedt3d.a','libdynamicedt3d.so','libf2c.a','libgsl.a','libgsl.la','libgsl.so','libgsl.so.16','libgslcblas.a','libgslcblas.la','libgslcblas.so','libgslcblas.so.0','liblapack.a','libomniCodeSets4.a','libomniCodeSets4.so','libomniCodeSets4.so.1','libomniCodeSets4.so.1.6','libomniConnectionMgmt4.a','libomniConnectionMgmt4.so','libomniConnectionMgmt4.so.1','libomniConnectionMgmt4.so.1.6','libomniDynamic4.a','libomniDynamic4.so','libomniDynamic4.so.1','libomniDynamic4.so.1.6','libomniORB4.a','libomniORB4.so','libomniORB4.so.1','libomniORB4.so.1.6','libomnithread.a','libomnithread.so','libomnithread.so.3','libomnithread.so.3.4','libqhull.so','libqhull.so.6.3.1','libqhull_p.so','libqhull_p.so.6.3.1','libqhullcpp.a','libqhullstatic.a','libqhullstatic_p.a','libuuid.a','libuuid.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0x94f1db9fe286a296a12d7d783d048d58L

root = '/opt/jsk/bin'
dir_md5 = 0
for file in ['binvox2bt','bt2vrml','catior','coil-config','compare_octrees','convert_octree','convertior','edit_octree','eval_octree_accuracy','export-vrml','genior','graph2tree','gsl-config','gsl-histogram','gsl-randist','iobtest_lower','iobtest_upper','log2graph','nameclt','omkdepend','omniMapper','omniNames','omnicpp','omniidl','omniidlrun.py','qconvex','qdelaunay','qhalf','qhull','qvoronoi','rbox','rtc-template','rtcd','rtcprof','rtm-config','rtm-naming','rtm-skelwrapper','unlock_iob']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0xe20b1b18caa0e7ea491eba56037f7001L


EOF


