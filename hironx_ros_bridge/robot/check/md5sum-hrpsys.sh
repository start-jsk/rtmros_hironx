#!/bin/sh

python <<EOF

import hashlib
import os
from operator import add

print "* Check /opt/jsk files (hrpsys)"

f = open('/tmp/check-opt-jsk-hrpsys-md5.txt', 'w')
for dir in [['/opt/jsk/include/hrpsys',    0x3421e0aa99b0db4037b535d9964da43fL],
            ['/opt/jsk/share/hrpsys',      0xedf57864dddeba2caedf019adc9d8d37L]]:
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
for file in ['libhrpsysBaseStub.so','hrpEC.so','NullComponent.so','RobotHardware.so','StateHolder.so','WavPlayer.so','SequencePlayer.so','DataLogger.so','ForwardKinematics.so','HGcontroller.so','Range2PointCloud.so','ImpedanceController.so','AutoBalancer.so','SoftErrorLimiter.so','VirtualForceSensor.so','GraspController.so','TorqueFilter.so','KalmanFilter.so','Stabilizer.so','AbsoluteForceSensor.so','ServoController.so','ThermoEstimator.so','ThermoLimiter.so','TorqueController.so']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0xbb9aa52b4cc6588f6c8cfc4fa0c6eb66L

root = '/opt/jsk/bin'
dir_md5 = 0
for file in ['rtcd.sh','NullComponentComp','RobotHardwareComp','StateHolderComp','WavPlayerComp','SequencePlayerComp','DataLoggerComp','ForwardKinematicsComp','HGcontrollerComp','Range2PointCloudComp','ImpedanceControllerComp','AutoBalancerComp','testPreviewController','testGaitGenerator','SoftErrorLimiterComp','VirtualForceSensorComp','GraspControllerComp','TorqueFilterComp','KalmanFilterComp','StabilizerComp','AbsoluteForceSensorComp','ServoControllerComp','ThermoEstimatorComp','ThermoLimiterComp','TorqueControllerComp','testMotorTorqueController','NameServer.sh','ModelLoader.sh','startJSK.sh','hrpsyspy']:
  if os.path.exists(os.path.join(root,file)):
    file_md5 = hashlib.md5(open(os.path.join(root,file), 'rb').read()).hexdigest()
  else:
    file_md5 = 'No such file or directory'
    print "  ** ",os.path.join(root,file),"\t\t",file_md5
  print >>f, os.path.join(root,file),"\t\t",file_md5
  dir_md5 = dir_md5 ^ int(file_md5, 16)
print "  Check ", root, "\t\t\t(", hex(dir_md5), ")\t", dir_md5==0x9c96f180c35635b15e8f757dcfa27226L

EOF


