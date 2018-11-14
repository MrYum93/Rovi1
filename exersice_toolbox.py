#!/usr/bin/python
# #/***************************************************************************
# RoVi1 scrip for all exersice Robotcs part
# Copyright (c) 2018, Mathias W. Madsen <matam14@student.sdu.dk> <mwittenm@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
This is a script containing all functions needed to pass RoVi1

NOT YET: Rest of lectures...

Revision
YYYY-MM-DD
2018-11-07 MW First version
2018-11-09 MW Added RobWork functionality, run in terminal
'''

#import
import sys
# sys.path.append('/home/mathias/RobWork/RobWork/libs/release')
# sys.path.append('/home/mathias/RobWork/RobWorkStudio/libs/release')
# sys.path.append('/home/mathias/RobWork/RobWorkSim/libs/release')
# import rw, rws, rwsim
import math
import numpy as np

class gcs_node:
    def __init__(self):
        pass

    def point_a_to_b(self, point_a, point_b):
        new_point_a = np.array(np.zeros(3))
        a = point_a
        b = point_b
        new_point_a[0] = (a[0] * b[0]) * b[0] + (a[0] * b[1]) * b[1] + (a[0] * b[2]) * b[2]
        new_point_a[1] = (a[1] * b[0]) * b[0] + (a[1] * b[1]) * b[1] + (a[1] * b[2]) * b[2]
        new_point_a[2] = (a[2] * b[0]) * b[0] + (a[2] * b[1]) * b[1] + (a[2] * b[2]) * b[2]
        return new_point_a

    def point_from_point(self):
        pass

    # Write code which calculates the 3 × 3 rotation matrix given:
    # • A specification of which axis to rotate around.
    # • A specification of whether to use fixed frame or Euler rotation.
    # • The 3 parameters corresponding to the rotation around the individual axes.
    def rot_mat_from_two_points(self, point_a, point_b):
        # makes a rotation matrix from two points. Namely mat a -> mat b || R_A^B
        a = point_a
        b = point_b
        # rot_mat = np.matrix(((a[0]*b[0], a[0]*b[1], a[0]*b[2]),
        #                      (a[1]*b[0], a[1]*b[1], a[1]*b[2]),
        #                      (a[2]*b[0], a[2]*b[1], a[2]*b[2])))
        # print("rotation matrix\n", rot_mat)
        # Can also be done more simple
        a = a[np.newaxis]
        rot_mat = np.mat(a.T) * np.mat(b)
        # print("rotation matrix\n", other_mat)
        return rot_mat

    def rot_mat_from_euler_rpy(self, R, P, Y):
        rot_mat = np.matrix(((math.cos(R) * math.cos(), math.cos() * math.cos(), math.cos() * math.cos()),
                             (math.),
                             (math)))

    def eaa_calc(self, rot_mat):
        # This func returns the EAA rot matrix based on parameters from page 47 in robotics notes.
        # First find x than find if [R_11 + R_22 + R_33 - 1] is smaller or larger than 0
        # x = 1/2*||R_32 - R_23, R_13 - R_31, R_21 - R_12||
        x = 1 / 2 * math.sqrt((rot_mat[3][2] - rot_mat[2][3]) ** 2 +
                              (rot_mat[1][3] - rot_mat[3][1]) ** 2 +
                              (rot_mat[2][1] - rot_mat[1][2]) ** 2)
        req = rot_mat[1][1] + rot_mat[2][2] + rot_mat[3][3] - 1
        if x >= 10 ** (-6):
            # Eq. 4.18
            pass
        else:
            if req > 0:
                # Eq. 4.19
                pass
            else:
                # Eq. 4.20
                pass
        return  # result
        pass

    def parabolic_blend(self, time_before_blend_time, blend_point, vel_one, vel_two, blend_size):
        # this func returns a parabola between two linear path with different velocities and averages the velocities.
        parabola = (vel_two - vel_one) / (4 * blend_size) * (time_before_blend_time + blend_size)**2 + \
                   vel_one * time_before_blend_time + blend_point
        return parabola

    def main(self):
        point_a = np.array([1, 2, 3])
        point_b = np.array((3, 2, 1))
        new_point_a = self.rot_mat_from_two_points(point_a, point_b)
        pass

if __name__ == '__main__':
    gcs = gcs_node()
    gcs.main()