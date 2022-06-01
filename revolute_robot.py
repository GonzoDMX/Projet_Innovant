#!/usr/bin/env python3

import numpy as np
import math, random

from geometry import Point3D, Target3D

"""
    Revolute Robot
    Created by: Andrew O'Shei

    Description:
        This program is used for creating a kinematics model for a revolute
        jointed robot arm. It uses a pseudo-jacobian method for calculating
        the joint deltas / angles. There are also some basic facilities for
        self-collision avoidance.
        Robot positions can be plotted and selected using the plot.
"""

''' Class for defining the characteristics of your Revolute Robot '''
class RevoluteRobot():
    def __init__(self, x=0, y=0, z=0, floor=0, padding=0):
        # ORIGIN point for the base of the robot
        self.origin = Point3D(x, y, z)
        # LIMITS set movement limits to prevent out of bounds targets
        self.limits = {'reach': 0, 'floor': floor}
        # PADDING Sets the size of the hit box around the robot
        self.padding = padding
        # EFFECTOR POSITION holds current and desired position of end effector
        #self.end_position = {'current': self.origin, 'target': self.origin}
        self.end_position = {'current': self.origin, 'target': Point3D(0, -10, 10)}
        #JOINTS holds current transformation position of joints
        self.joints = np.array(
                            [[self.origin.x, self.origin.y, self.origin.z, 1]],
                            dtype=np.float).T
        self.joint_count = 0
        # LENGTHS holds the length of the link between each joint
        self.lengths = []
        # REACH Maximum reach of robot with all joints extended
        self.reach = 0
        # THETA stores current theta (angle) and min max limits for joint angles
        self.theta = {  'theta': np.array([[]], dtype=np.float),
                        'min':   np.array([[]], dtype=np.float),
                        'max':   np.array([[]], dtype=np.float)}

    ''' Append a Joint with Link to the robot '''
    def add_joint_link(self, length=0, min_theta=0, max_theta=180, theta=None):
        self.joints = np.append(
            self.joints, np.array([[0, 0, 0, 1]]).T, axis=1)
        self.lengths.append(length)
        self.reach = sum(self.lengths)
        self.theta['min'] = np.append(self.theta['min'], math.radians(min_theta))
        self.theta['max'] = np.append(self.theta['max'], math.radians(max_theta))
        if theta is None:
            self.theta['theta'] = np.append(self.theta['theta'],
                                math.radians((min_theta + max_theta)/2))
        else:
            self.theta['theta'] = np.append(self.theta['theta'],
                                math.radians(theta))
        self.joint_count += 1
        self.update_posture()
        self.end_position['target'] = Point3D(self.joints[0, -1], self.joints[1, -1], self.lengths[0])

    def update_posture(self):
        for i in range(self.joint_count):
            if i == 0:
                T = self.get_base_homogen(self.theta['theta'][i], self.lengths[i])
                self.joints[:, [i+1]] = T.dot(np.array([[0, 0, 0, 1]]).T)
            else:
                T_mat = self.get_joint_homogen(self.theta['theta'][i], self.lengths[i])
                T = T.dot(T_mat)
                self.joints[:, [i+1]] = T.dot(np.array([[0, 0, 0, 1]]).T)
        self.end_position["current"].x = self.joints[0, -1].item()
        self.end_position["current"].y = self.joints[1, -1].item()
        self.end_position["current"].z = self.joints[2, -1].item()

    ''' Returns 4x4 homogenoeous matrix for Base rotation and translation '''
    def get_base_homogen(self, theta, alpha):
        t_mat = np.array([
            [math.cos(theta), 0,  math.sin(theta), 0],
            [math.sin(theta), 0, -math.cos(theta), 0],
            [              0, 1,                0, alpha],
            [              0, 0,                0, 1]])
        return t_mat

    ''' Returns 4x4 homogenoeous matrix for Joint rotation and translation '''
    def get_joint_homogen(self, theta, alpha):
        t_mat = np.array([
            [math.cos(theta), -math.sin(theta),  0,  alpha*math.cos(theta)],
            [math.sin(theta),  math.cos(theta),  0,  alpha*math.sin(theta)],
            [              0,                0,  1,  0],
            [              0,                0,  0,  1]])
        return t_mat

    ''' Calculate and then return the IK Jacobian Matrix '''
    def get_jacobian_matrix(self):
        # Define unit vector "k-hat" pointing along the Z axis.
        k_unit_vector = np.array([[0, 0, 1]], dtype=np.float)
        jacobian_matrix = np.zeros((3, self.joint_count), dtype=np.float)
        end_position = self.joints[:3, [-1]]
        # Utilize cross product to compute each row of the Jacobian matrix.
        for i in range(self.joint_count):
            if i == 0:
                k_unit_vector = np.array([[0, 0, 1]], dtype=np.float)
            else:
                k_unit_vector = np.array([[1, 0, 0]], dtype=np.float)
            joint_position = self.joints[:3, [i]]
            jacobian_matrix[:, i] = np.cross(
                k_unit_vector, (end_position - joint_position).reshape(3,))
        return jacobian_matrix

    ''' Run Jacobian inverse routine to move end effector toward target '''
    def get_joint_delta(self, target):
        # Set distance to move end effector toward target per algorithm iteration.
        step_size = 0.02 * self.reach
        target_vector = (target.t - self.joints[:, [-1]])[:3]
        t_unit_vector = target_vector / np.linalg.norm(target_vector)
        delta_r = step_size * t_unit_vector
        jacobian_matrix = self.get_jacobian_matrix()
        jacobian_inverse = np.linalg.pinv(jacobian_matrix)
        # Delta theta is the proposed angle displacement
        delta_theta = jacobian_inverse.dot(delta_r)
        return delta_theta

    ''' Updates the robot's joint angles '''
    def update_theta(self, delta_theta):
        self.theta['theta'] += delta_theta.flatten()
        # Restrict theta to min and max bounds
        for i in range(self.joint_count):
            if self.theta['theta'][i] < self.theta['min'][i]:
                self.theta['theta'][i] = self.theta['min'][i]
            if self.theta['theta'][i] > self.theta['max'][i]:
                self.theta['theta'][i] = self.theta['max'][i]
            #print("Joint {}: {}".format(i, self.theta['theta'][i]))

    ''' Step robot towards target '''
    def move_to_target(self):
        delta_theta = self.get_joint_delta(self.end_position['target'])
        self.update_theta(delta_theta)
        self.update_posture()
        #self.draw_hit_box()

    ''' Test if the End Effecter is on the target position '''
    def on_target(self):
        if np.linalg.norm(self.end_position['target'].t - self.joints[:, [-1]]) > 0.02 * self.reach:
            return False
        return True

    def set_target(self, x, y, z):
        self.end_position['target'] = Point3D(x, y, z)
