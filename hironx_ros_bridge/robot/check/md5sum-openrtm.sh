#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files (openrtm and base libraries)"

f = open('/tmp/check-opt-jsk-md5.txt', 'w')
for dir in [['/opt/jsk/include/openrtm-1.1',	'96a33e7deca37432b6dbdb7aa0900186'],
            ['/opt/jsk/include/coil-1.1',	'ab43833e33fe339db2ead70a4875b440'],
            ['/opt/jsk/include/eigen3',         '83f8ea807b80a89c3759fe5df593e7cd'],
            ['/opt/jsk/include/omniORB4',	'8f4686575b01569665003db9c9e67852'],
            ['/opt/jsk/include/omnithread',	'feca337362737d975d04b7416adf63cb'],
            ['/opt/jsk/include/COS',            '9fb8ad76f3dfbdaa5abed599c11a5677'],
            ['/opt/jsk/include/uuid',           '574494a5883c16d7b001ea5488e7c45f'],
            ['/opt/jsk/share/openrtm-1.1',	'52739f8295c6ce57bea78fc6bde79645']]:
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
for file in ['libCOS4.a','libCOS4.so','libCOS4.so.1','libCOS4.so.1.6','libCOSDynamic4.a','libCOSDynamic4.so','libCOSDynamic4.so.1','libCOSDynamic4.so.1.6','libRTC-1.1.0.so','libRTC.a','libRTC.la','libRTC.so','libblas.a','libcoil-1.1.0.so','libcoil.a','libcoil.la','libcoil.so','libdynamicedt3d.a','libdynamicedt3d.so','libf2c.a','libgsl.a','libgsl.la','libgsl.so','libgsl.so.16','libgslcblas.a','libgslcblas.la','libgslcblas.so','libgslcblas.so.0','liblapack.a','libomniCodeSets4.a','libomniCodeSets4.so','libomniCodeSets4.so.1','libomniCodeSets4.so.1.6','libomniConnectionMgmt4.a','libomniConnectionMgmt4.so','libomniConnectionMgmt4.so.1','libomniConnectionMgmt4.so.1.6','libomniDynamic4.a','libomniDynamic4.so','libomniDynamic4.so.1','libomniDynamic4.so.1.6','libomniORB4.a','libomniORB4.so','libomniORB4.so.1','libomniORB4.so.1.6','libomnithread.a','libomnithread.so','libomnithread.so.3','libomnithread.so.3.4','libqhull.so','libqhull.so.6.3.1','libqhull_p.so','libqhull_p.so.6.3.1','libqhullcpp.a','libqhullstatic.a','libqhullstatic_p.a','libuuid.a','libuuid.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='798626a0198233aa8d56b8e8d1f4afdc'

root = '/opt/jsk/bin'
dir_md5 = hashlib.md5()
for file in ['binvox2bt','bt2vrml','catior','coil-config','compare_octrees','convert_octree','convertior','edit_octree','eval_octree_accuracy','export-vrml','genior','graph2tree','gsl-config','gsl-histogram','gsl-randist','iobtest_lower','iobtest_upper','log2graph','nameclt','omkdepend','omniMapper','omniNames','omnicpp','omniidl','omniidlrun.py','qconvex','qdelaunay','qhalf','qhull','qvoronoi','rbox','rtc-template','rtcd','rtcprof','rtm-config','rtm-naming','rtm-skelwrapper','unlock_iob']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='16e03e840307fe3ac945ab726244dd84'


EOF


