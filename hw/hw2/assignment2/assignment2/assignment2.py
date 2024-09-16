
''' ####################
    EN605.613 - Introduction to Robotics
    Assignment 2
    Coordinate Transforms
    -------
    For this assignment you must implement the all the functions in this file
    ==================================
    Copyright 2022,
    The Johns Hopkins University Applied Physics Laboratory LLC (JHU/APL).
    All Rights Reserved.
    #################### '''

from typing import Tuple
import numpy as np


def euler_rotation_matrix(alpha: float,beta: float,gamma:float) -> np.ndarray:
    """
    15 pts
    Creates a 3x3 rotation matrix in 3D space from euler angles

    Input
    :param alpha: The roll angle (radians)
    :param beta: The pitch angle (radians)
    :param gamma: The yaw angle (radians)

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """
    
    # It may seem redundant but you can substantially increase the speed of computing
    # the rotation by precomputing the sine and cosine values


    #This is an example of how to define a 2d matrix using NUMPY
    R = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]])
    # Rotation around z-axis (yaw component)
    Rz = np.array([[np.cos(gamma), -np.sin(gamma), 0], 
                   [np.sin(gamma), np.cos(gamma), 0], 
                   [0, 0, 1]])
    
    # Rotation around y-axis (pitch component)
    Ry = np.array([[np.cos(beta), 0, np.sin(beta)], 
                   [0, 1, 0], 
                   [-np.sin(beta), 0, np.cos(beta)]])

    # Rotation around x-axis (roll component)
    Rx = np.array([[1, 0, 0], 
                   [0, np.cos(alpha), -np.sin(alpha)], 
                   [0, np.sin(alpha), np.cos(alpha)]])
    return Rz * Ry * Rx

def quaternion_rotation_matrix(Q: np.ndarray) -> np.ndarray:
    """
    15 pts
    Creates a 3x3 rotation matrix in 3D space from a quaternion.

    Input
    :param q: A 4 element array containing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """

    Rq = np.array([[2*(Q[0]**2 + Q[1]**2) - 1), 2*(Q[1] * Q[2] - Q[0] * Q[3]), 2*(Q[1] * Q[3] + Q[0] * Q[2])],
                   [2*(Q[1] * Q[2] + Q[0] * Q[3]), 2*(Q[0]**2 + Q[2]**2) - 1), 2*(Q[2] * Q[3] - Q[0] * Q[1])],
                   [2*(Q[1] * Q[3] - Q[0] * Q[2]), 2*(Q[2] * Q[3] + Q[0] * Q[1]), 2*(Q[0]**2 + Q[3]**2) - 1)]])


    return Rq

def quaternion_multiply(Q0: np.ndarray,Q1: np.ndarray) -> np.ndarray:
    """
    15 pts
    Multiplies two quaternions.

    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 

    """

    q03 = Q0[0]*Q1[0] - Q0[1]*Q1[1] - Q0[2]*Q1[2] - Q0[3]*Q1[3]
    q13 = Q0[0]*Q1[1] + Q0[1]*Q1[0] + Q0[2]*Q1[3] - Q0[3]*Q1[2]
    q23 = Q0[0]*Q1[2] + Q0[2]*Q1[0] + Q0[3]*Q1[1] - Q0[1]*Q1[3]
    q23 = Q0[0]*Q1[3] + Q0[3]*Q1[0] + Q0[1]*Q1[2] - Q0[2]*Q1[1]

    return np.ndarray([q03, q13, q23, q33])


def quaternion_to_euler(Q: np.ndarray) -> np.ndarray:
    """
    15 pts
    Takes a quaternion and returns the roll, pitch yaw array.

    Input
    :param Q0: A 4 element array containing the quaternion (q01,q11,q21,q31) 

    Output
    :return: A 3 element array containing the roll,pitch, and yaw (alpha,beta,gamma) 

    """

    alpha = np.arctan2(2*(Q[0] * Q[1] + Q[2]*Q[3]), 1 - 2*(Q[1]**2 + Q[2]**2))
    beta = 2*(Q[0]*Q[2] - Q[3]*Q[1])
    gamma = np.arctan2(2*(Q[0]*Q[3] + Q[1]*Q[2]), 1 - 2*(Q[2]**2 + Q[3]*)**2)
    return np.ndarray([alpha, beta, gamma])


def rotate(p1: np.ndarray,alpha: float,beta: float,gamma: float) -> np.ndarray:
    """
    15 pts
    Rotates a point p1 in 3D space to a new coordinate system.

    Input
    :param p1: A 3 element array containing the original (x1,y2,z1) position]
    :param alpha: The roll angle (radians)
    :param beta: The pitch angle (radians)
    :param gamma: The yaw angle (radians)

    Output
    :return: A 3 element array containing the new rotated position (x2,y2,z2)

    """
    R = euler_rotation_matrix(alpha, beta, gamma)
    p2 = R*p1
    
    return p2


def inverse_rotation(p2: np.ndarray,alpha: float,beta: float,gamma: float) -> np.ndarray:
    """
    15 pts
    Inverse rotation from a point p2 in 3D space to the original coordinate system.

    Input
    :param p: A 3 element array containing the new rotated position (x2,y2,z2)
    :param alpha: The roll angle (radians)
    :param beta: The pitch angle (radians)
    :param gamma: The yaw angle (radians)

    Output
    :return: A 3 element array containing the original (x1,y1,z1) position]

    """
    R = euler_rotation_matrix(alpha, beta, gamma)
    p3 = R.T * p1
    return p3

def transform_pose(P: np.ndarray,Q: np.ndarray,T: np.ndarray,R: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    10 pts
    Takes a position and orientation in the original frame along with a translation and
    rotation. 

    Then converts the original point into the new coordinate system.

    Hints: 
    - Compute the quaternion that represents the new orientation by multiplying the
      old quaternion by the new quaternion (order matters!)
    - When transforming the point rotation is applied before translation

    Input
    :param P: A 3 element array containing the position (x0,y0,z0) in the original frame
    :param Q: A 4 element array containing the quaternion (q0,q1,q2,q3) 
    :param T: A 3 element array containing the vector between the origins in the two coordinate systems (dx,dy,dz) 
    :param R: A 4 element array containing the rotation in the form of a quaternion (q0,q1,q2,q3) 

    Output
    :return: New Pose, A 3 element array (x1,y2,z1) containing the position in the new coordinate frame
    :return: New Quaternion, A 4 element array containing the orientation (q0,q1,q2,q3) in the new coordinate frame

    """

    return None




