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
# import sys
# sys.path.append('/home/mathias/RobWork/RobWork/libs/release')
# sys.path.append('/home/mathias/RobWork/RobWorkStudio/libs/release')
# sys.path.append('/home/mathias/RobWork/RobWorkSim/libs/release')
# import rw, rws, rwsim
import math
import numpy as np

class gcs_node:
    def __init__(self):
        pass

    # def rw_studio(self):
    #     rwstudio = rws.getRobWorkStudio()
    #     # now load a workcell
    #     workcell = '/home/mathias/Dropbox/_SDU/9.Semester/RoVi/Robotics/workcells/Kr16WallWorkCell/Scene.wc.xml'
    #     # rwstudio.openWorkCell(workcell)
    #     # lets get the workcell
    #     wc = rwstudio.getWorkCell()
    #     print(wc.getName())
    #     pass

    def point_a_to_b(self, point_a, point_b):
        # Makes a rotation matrix between two vectors
        new_point_a = np.array(np.zeros(3))
        a = point_a
        b = point_b
        new_point_a[0] = (a[0] * b[0]) * b[0] + (a[0] * b[1]) * b[1] + (a[0] * b[2]) * b[2]
        new_point_a[1] = (a[1] * b[0]) * b[0] + (a[1] * b[1]) * b[1] + (a[1] * b[2]) * b[2]
        new_point_a[2] = (a[2] * b[0]) * b[0] + (a[2] * b[1]) * b[1] + (a[2] * b[2]) * b[2]
        return new_point_a

    def rot_mat_from_two_points(self, point_a, point_b):
        # makes a rotation matrix from two points. Namely mat a -> mat b || R_A^B
        a = point_a
        b = point_b
        # rot_mat = np.matrix(((a[0]*b[0], a[0]*b[1], a[0]*b[2]),
        #                      (a[1]*b[0], a[1]*b[1], a[1]*b[2]),
        #                      (a[2]*b[0], a[2]*b[1], a[2]*b[2])))
        # print("rotation matrix\n", rot_mat)
        # # Can also be done more simple
        a = a[np.newaxis]
        rot_mat = np.mat(a.T) * np.mat(b)
        # print("rotation matrix\n", other_mat)
        return rot_mat

    def rpy_to_rot(self, r, p, y):
        # this class creates a rot matrix about the RPY angles, this can be used to rotate a
        # frame about the RPY/XYZ axis
        def c(axis):
            return  math.cos(axis)

        def s(axis):
            return math.sin(axis)

        rot_mat = np.matrix(((c(p)*c(y), s(r)*s(p)*c(y)-c(r)*s(y), c(r)*s(p)*c(y)+s(r)*s(y)),
                             (c(p)*s(y), s(r)*s(p)*s(y)+c(r)*c(y), c(r)*s(p)*s(y)-s(r)*c(y)),
                             (-s(p), s(r)*c(p), c(r)*c(p))))
        return rot_mat

    def rot_mat_to_rpy_angles(self, rot_mat):
        # this method takes any rotation matrix and returns what axis has been rotated around.
        y_ang = p = math.asin(-rot_mat.item(2, 0))
        if abs(p) != 1:
            x_ang = r = math.atan2(math.cos(p)*rot_mat.item(2, 1), math.cos(p)*rot_mat.item(2, 2))
            z_ang = y = math.atan2(math.cos(p)*rot_mat.item(1, 0), math.cos(p)*rot_mat.item(0, 0))
        else:
            return "singularity with RPY, use quarternions"
        return r, p, y

    def trans_mat_hom(self, rot_mat, trans_mat):
        # print(rot_mat, '\n\n', trans_mat, '\n')
        trans_mat = np.insert(rot_mat, 3, trans_mat, axis=1)
        trans_mat = np.insert(trans_mat, 3, [0, 0, 0, 1], axis=0)
        # print(trans_mat)
        return trans_mat

    def eaa_calc(self, rot_mat):
        # This func returns the EAA rot matrix based on parameters from page 47 in robotics notes.
        # First find x than find if [R_11 + R_22 + R_33 - 1] is smaller or larger than 0
        # x = 1/2*||R_32 - R_23, R_13 - R_31, R_21 - R_12||
        abs_of_rot = math.sqrt((rot_mat[2, 1] - rot_mat[1, 2]) ** 2 +
                               (rot_mat[0, 2] - rot_mat[2, 0]) ** 2 +
                               (rot_mat[1, 0] - rot_mat[0, 1]) ** 2)

        x = 1 / 2 * abs_of_rot
        cos_part = math.acos((rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2] - 1)/2)
        rot_part = np.array([rot_mat[2,1]-rot_mat[1,2], rot_mat[0,2]-rot_mat[2,1], rot_mat[1,0]-rot_mat[0,1]])
        if x >= 10**(-6):
            # Eq. 4.18
            w_rot = cos_part / abs_of_rot * rot_part
        else:
            if x > 0:
                # Eq. 4.19
                w_rot = 1/2 * rot_part
                pass
            else:
                # Eq. 4.20
                w_rot = ((math.pi - 1/2 * abs_of_rot) / math.sqrt(2)) * 1 #TODO missing part of 4.20
                pass
        # return  # result
        # pass

    def parabolic_blend(self, time_before_blend_time, blend_point, vel_one, vel_two, blend_size):
        # this func returns a parabola between two linear path with different velocities and averages the velocities.
        parabola = (vel_two - vel_one) / (4 * blend_size) * (time_before_blend_time + blend_size)**2 + \
                   vel_one * time_before_blend_time + blend_point
        return parabola

    def t_base_tcp_test(self):
        # This method is an example of a concrete exersice from robotics notes, and is generalized in self.t_base_tcp
        tbase1 = self.trans_mat_hom(self.rpy_to_rot(0, 0, math.pi/2), np.array([1.2, 2.3, -3.5])) # revolute
        t12 = self.trans_mat_hom(self.rpy_to_rot(-math.pi/2, 0, 0), np.array([0, 0, 4])) # prismatic
        t2tool = self.trans_mat_hom(self.rpy_to_rot(0, 0, 0), np.array([0, 0, 2]))
        state_vector = np.array([math.pi/3, 2.8]) # Uniquely determines the pos and orientation of the robot
        qbase = self.trans_mat_hom(self.rpy_to_rot(0, 0, state_vector[0]), np.array([0, 0, 0]))
        q1 = self.trans_mat_hom(self.rpy_to_rot(0, 0, 0), np.array([0, 0, state_vector[1]]))
        return  np.dot(tbase1, qbase) * np.dot(t12, q1) * t2tool

    def t_base_tcp(self, trans_array, trans_array_size, state_vector, state_vecor_size):
        # This method calculates a trajectory from base to tool given
        # any number of transformation matrix and state vectors
        dot_list = []
        for i in range(trans_array_size):
            t_new = trans_array[i]
            if i < state_vecor_size:
                q_new = state_vector[i]
                dot_list.append(np.dot(t_new, q_new))
                # print(dot_list[i], '\n')
                if i == 0:
                    result = dot_list[i]
                else:
                    result *= dot_list[i]
                print(dot_list[i], '\n\n')
                continue

            dot_list.append(t_new)
            result *= t_new
        print("Result\n", result)
        return result

    def main(self):
        point_a = np.array([1, 0, 0])
        point_b = np.array((3, 2, 1))
        rot_mat = self.rpy_to_rot(math.pi, 0, 0)
        hom_mat = self.trans_mat_hom(rot_mat, point_a)

        ''' 
        TODO EAA, have parts of the script but not finished
        '''
        self.eaa_calc(rot_mat)

        ''' 
        Rot matrix conventional
        '''
        # rpy_rot_mat = self.rpy_to_rot(math.pi, 0, 0)
        # new_point_a_b = self.rot_mat_from_two_points(point_a, point_b)
        # rpy_ang = self.rot_mat_to_rpy_angles(rpy_rot_mat)
        # print(rpy_rot_mat, '\n\n', rpy_ang)
        # print(t_base_tcp)

        ''' 
        A concrete example of going from base to tool and also how a homogeneous matrix is created.
        First line is a method for doing it with specific values, below is a general case. 
        '''
        # t_base_tcp = self.t_base_tcp_test()
        # tbase1 = self.trans_mat_hom(self.rpy_to_rot(0, 0, math.pi / 2), np.array([1.2, 2.3, -3.5]))  # revolute
        # t12 = self.trans_mat_hom(self.rpy_to_rot(-math.pi / 2, 0, 0), np.array([0, 0, 4]))  # prismatic
        # t2tool = self.trans_mat_hom(self.rpy_to_rot(0, 0, 0), np.array([0, 0, 2]))
        # state_vector = np.array([math.pi / 3, 2.8])  # Uniquely determines the pos and orientation of the robot.
        #                                              # In this case it is for a revolute and prismatic respectively
        # qbase = self.trans_mat_hom(self.rpy_to_rot(0, 0, state_vector[0]), np.array([0, 0, 0]))
        # q1 = self.trans_mat_hom(self.rpy_to_rot(0, 0, 0), np.array([0, 0, state_vector[1]]))
        # trans_mat_array = [tbase1, t12, t2tool]
        # state_vector_array = [qbase, q1]
        # t_base_tcp = self.t_base_tcp(trans_mat_array, len(trans_mat_array), state_vector_array, len(state_vector))
        # print("t_base_tcp: ", t_base_tcp)


if __name__ == '__main__':
    gcs = gcs_node()
    gcs.main()