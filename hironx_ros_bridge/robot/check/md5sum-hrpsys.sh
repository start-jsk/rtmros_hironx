#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files (hrpsys)"

f = open('/tmp/check-opt-jsk-hrpsys-md5.txt', 'w')
for dir in [['/opt/jsk/include/hrpsys',    'c074dc6787518f1f73f9c52a49142433'],
            ['/opt/jsk/share/hrpsys',      'c92792acf2ca018de8c0f2500d88a583']]:
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
for file in ['libhrpsysBaseStub.so','hrpEC.so','NullComponent.so','RobotHardware.so','StateHolder.so','WavPlayer.so','SequencePlayer.so','DataLogger.so','ForwardKinematics.so','HGcontroller.so','Range2PointCloud.so','ImpedanceController.so','AutoBalancer.so','SoftErrorLimiter.so','VirtualForceSensor.so','GraspController.so','TorqueFilter.so','KalmanFilter.so','Stabilizer.so','AbsoluteForceSensor.so','ServoController.so','ThermoEstimator.so','ThermoLimiter.so','TorqueController.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='c204ca459a4f0ba262513dadde1b2f9f'

root = '/opt/jsk/bin'
dir_md5 = hashlib.md5()
for file in ['rtcd.sh','NullComponentComp','RobotHardwareComp','StateHolderComp','WavPlayerComp','SequencePlayerComp','DataLoggerComp','ForwardKinematicsComp','HGcontrollerComp','Range2PointCloudComp','ImpedanceControllerComp','AutoBalancerComp','testPreviewController','testGaitGenerator','SoftErrorLimiterComp','VirtualForceSensorComp','GraspControllerComp','TorqueFilterComp','KalmanFilterComp','StabilizerComp','AbsoluteForceSensorComp','ServoControllerComp','ThermoEstimatorComp','ThermoLimiterComp','TorqueControllerComp','testMotorTorqueController','NameServer.sh','ModelLoader.sh','startJSK.sh','hrpsyspy']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5.update(file_md5)
print "  Check ", root, "\t\t\t(", dir_md5.hexdigest(), ")\t", dir_md5.hexdigest()=='99a1c4ec0044d45352f8f40550a9581e'


EOF


