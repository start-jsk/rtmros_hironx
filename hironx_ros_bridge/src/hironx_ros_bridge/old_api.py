# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Felix von Drigalski
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of JSK Lab, University of Tokyo. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This module extends the HIRONX client class with some functions
# from the original HIRO API, so that old code can be ported more easily.
# Usage in an old script:
# 1) Add these at the start of your file
# from hironx_ros_bridge import old_api
# robot = hironx_client.HIRONX()
#
# 2) Go through the code and replace functions like this:
# moveRelativeL(dz=0.05)              ==>      robot.moveRelativeL(dz=0.05)
# getCurrentConfiguration(armL_svc)   ==>      robot.getCurrentConfiguration('LARM_JOINT5')


def setTargetAngular(self, group_name, x, y, z, r, p, w, rate=10.0, wait=True):
    ''' 
    This is only an approximation of the original function. It can be
    considerably slower for small movements that contain rotation.
    '''
    if group_name == 'larm':
        joint_name = 'LARM_JOINT5'
    elif group_name == 'rarm':
        joint_name = 'RARM_JOINT5'

    scaling_factor_translation = 50
    scaling_factor_rotation = 5
    x_now, y_now, z_now, r_now, p_now, w_now = self.getCurrentConfiguration(joint_name)
    translation_distances = map(abs, [x-x_now * scaling_factor_translation,
                                      y-y_now * scaling_factor_translation,
                                      z-z_now * scaling_factor_translation])
    rotation_distances = map(abs, [r-r_now * scaling_factor_rotation,
                                   p-p_now * scaling_factor_rotation,
                                   w-w_now * scaling_factor_rotation])
    max_dist = max(translation_distances + rotation_distances)  # This is just concatenated
    mvmt_time = max_dist / rate

    pos = [x, y, z]
    rpw = [r, p, w]
    ret = self.setTargetPose(group_name, pos, rpw, mvmt_time)
    if ret and wait:
        self.waitInterpolationOfGroup(group_name)

    return ret

def move(self, group_name, x, y, z, r, p, w, rate, wait = True):
    return self.setTargetAngular(group_name, x, y, z, r, p, w, rate, wait)

def moveR(self, x, y, z, r, p, w, rate = 10, wait = True):
    return self.move('rarm', x, y, z, r, p, w, rate, wait)

def moveL(self, x, y, z, r, p, w, rate = 10, wait = True):
    return self.move('larm', x, y, z, r, p, w, rate, wait)

def moveRelative(self, group_name, joint_name, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate = 8, wait=True):
    pos = self.getCurrentPosition(joint_name)
    rpw = self.getCurrentRPY(joint_name)
    return self.move(group_name, pos[0] + dx, pos[1] + dy , pos[2] + dz , rpw[0] + dr, rpw[1] + dp, rpw[2] + dw, rate, wait)

def moveRelativeR(self, dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate = 8, wait = True):
    return self.moveRelative('rarm', 'RARM_JOINT5', dx, dy, dz, dr, dp, dw, rate, wait)

def moveRelativeL(self,dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate = 8, wait = True):
    return self.moveRelative('larm', 'LARM_JOINT5', dx, dy, dz, dr, dp, dw, rate, wait)

def getCurrentConfiguration(self, joint_name):
    xyz = self.getCurrentPosition(joint_name)
    rpw = self.getCurrentRPY(joint_name)
    return xyz[0], xyz[1], xyz[2], rpw[0], rpw[1], rpw[2]

from hironx_ros_bridge import hironx_client

# https://stackoverflow.com/questions/9455111/python-define-method-outside-of-class-definition
hironx_client.HIRONX.setTargetAngular = setTargetAngular
hironx_client.HIRONX.move = move
hironx_client.HIRONX.moveR = moveR
hironx_client.HIRONX.moveL = moveL
hironx_client.HIRONX.moveRelative = moveRelative
hironx_client.HIRONX.moveRelativeR = moveRelativeR
hironx_client.HIRONX.moveRelativeL = moveRelativeL
hironx_client.HIRONX.getCurrentConfiguration = getCurrentConfiguration
