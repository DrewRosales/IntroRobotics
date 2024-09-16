
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

    return R

def quaternion_rotation_matrix(Q: np.ndarray) -> np.ndarray:
    """
    15 pts
    Creates a 3x3 rotation matrix in 3D space from a quaternion.

    Input
    :param q: A 4 element array containing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matix containing the rotation matrix

    """

    return None

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
    return None


def quaternion_to_euler(Q: np.ndarray) -> np.ndarray:
    """
    15 pts
    Takes a quaternion and returns the roll, pitch yaw array.

    Input
    :param Q0: A 4 element array containing the quaternion (q01,q11,q21,q31) 

    Output
    :return: A 3 element array containing the roll,pitch, and yaw (alpha,beta,gamma) 

    """
    return None


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
    return None


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
    return None

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




