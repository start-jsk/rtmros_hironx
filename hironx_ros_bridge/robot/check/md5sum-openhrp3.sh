#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files"

f = open('/tmp/check-opt-jsk-md5.txt', 'w')
for dir in [['/opt/jsk/include/OpenHRP-3.1',    '288667b981f2ab487f735098c294a19a'],
            ['/opt/jsk/include/openrtm-1.1',	'c019758d211d840f5fa99e5cb394be0b'],
            ['/opt/jsk/include/coil-1.1',	'8acf735856619034e5e2a3ce0567a081'],
            ['/opt/jsk/include/eigen3',         'acb45fb4b8c943ff18a02bd2282225de'],
            ['/opt/jsk/include/omniORB4',	'89914ae9209e870e8564b964bd5d7c57'],
            ['/opt/jsk/include/omnithread',	'5684c8a508d830485a794c93741ada30'],
            ['/opt/jsk/include/COS',            'b9628b427098b08f29c6f7fec39fcd25'],
            ['/opt/jsk/include/uuid',           '574494a5883c16d7b001ea5488e7c45f'],
            ['/opt/jsk/share/OpenHRP-3.1',      'd99ba387cb643a9ad01cbd63799feab9'],
            ['/opt/jsk/share/openrtm-1.1',	'f671e0d3f45f5d284354660aab7309cd']]:
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
for file in ['libCOS4.a','libCOS4.so','libCOS4.so.1','libCOS4.so.1.6','libCOSDynamic4.a','libCOSDynamic4.so','libCOSDynamic4.so.1','libCOSDynamic4.so.1.6','libRTC-1.1.0.so','libRTC.a','libRTC.la','libRTC.so','libblas.a','libcoil-1.1.0.so','libcoil.a','libcoil.la','libcoil.so','libdynamicedt3d.a','libdynamicedt3d.so','libf2c.a','libf2c.so','libf2c.so.0','libhrpCollision-3.1.so','libhrpCollision-3.1.so.0','libhrpCollision-3.1.so.0.0.0','libhrpCorbaStubSkel-3.1.a','libhrpModel-3.1.so','libhrpModel-3.1.so.0','libhrpModel-3.1.so.0.0.0','libhrpUtil-3.1.so','libhrpUtil-3.1.so.0','libhrpUtil-3.1.so.0.0.0','liblapack.a','libomniCodeSets4.a','libomniCodeSets4.so','libomniCodeSets4.so.1','libomniCodeSets4.so.1.6','libomniConnectionMgmt4.a','libomniConnectionMgmt4.so','libomniConnectionMgmt4.so.1','libomniConnectionMgmt4.so.1.6','libomniDynamic4.a','libomniDynamic4.so','libomniDynamic4.so.1','libomniDynamic4.so.1.6','libomniORB4.a','libomniORB4.so','libomniORB4.so.1','libomniORB4.so.1.6','libomnithread.a','libomnithread.so','libomnithread.so.3','libomnithread.so.3.4','libuuid.a','libuuid.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='dae1ad6ccb37a42a7b30bc86022df56f'

root = '/opt/jsk/bin'
dir_md5 = hashlib.md5()
for file in ['ModelLoader.sh','NameServer.sh','coil-config','export-vrml','omniMapper','omniNames','omnicpp','omniidl','omniidlrun.py','openhrp-aist-dynamics-simulator','openhrp-collision-detector','openhrp-jython-prompt','openhrp-model-loader','openhrp-shutdown-servers','rtc-template','rtcd','rtcd.sh','rtcprof','rtm-config','rtm-naming','rtm-skelwrapper','startJSK.sh','unlock_iob']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='b6d00470e134e22ccd40275ba3f580ce'


EOF


