''' ####################
EN605.613 - Introduction to Robotics
Assignment 6
Extended Kalman Filter
-------
For this assignment you must implement the LandmarkDetector class and the EKF function.

Use the definition of the landmark sensor given in Lecture 7.

==================================
Copyright 2020,
The Johns Hopkins University Applied Physics Laboratory LLC (JHU/APL).
All Rights Reserved.

#################### '''

import sys
import numpy as np
import scipy.linalg as la


class LandmarkDetector(object):
  """
  Landmark sensor for the differential drive system.
  """

  def __init__(self,landmark_list):
    """
    Computes sensor measurements for a landmark sensor.

    Input
      :param landmark_list: a 2d list of landmarks [L1,L2,L3,...,LN], where each row is a 2D landmark defined by Li =[l_i_x,l_i_y]

    """

    self.landmarks = np.array(landmark_list)
    self.N_landmarks = self.landmarks.shape[0]
    self.W = None

  def get_W(self):
    '''
    This function provides the covariance matrix W which describes the noise that can be applied to the measurements.

    Feel free to experiment with different levels of noise.
    
      Output
        :return: V: input cost matrix
    '''
    if self.W is None:
      self.W = 0.0095*np.eye(2*self.N_landmarks)
    return self.W


  def measure(self,x,w=None):
    """
    Computes the measurement for the system. This will be a list of range and relative angles for each landmark.

    Input
      :param x: The state [x,y,theta,speed] of the system -> np.array with shape (3,)
      :param w: The noise (assumed Gaussian) applied to the measurment  -> np.array with shape (2*N_Landmarks,)

    Output
      :return: y: The resulting shape (2*N_Landmarks,) measurement with y = [h1_1,h1_2,h2_1,h2_2,...,hN_2] where [hi_1,hi_2]=[range_i,angle_i]

    """

    x_k, y_k, theta = x
    num_landmarks = self.landmarks.shape[0]

    z = np.zeros(2 * num_landmarks)

    for i, landmark in enumerate(self.landmarks):
        x_li, y_li = landmark

        z[2*i] = np.sqrt((x_li - x_k)**2 + (y_li - y_k)**2)
        z[2*i + 1] = np.arctan2((x_li - x_k), (y_li - y_k))

    if w is not None:
        z += w

    return z

  def jacobian(self,x):
    """
    Computes the first order jacobian around the state x

    Input
      :param x: The starting state (position) of the system -> np.array with shape (3,)

    Output
      :return: H: The resulting Jacobian matrix H for the sensor with shape (2*N_Landmarks,3)
      
    """
    x_k, y_k, theta = x
    H = []
    for landmark in self.landmarks:
        x_li, y_li = landmark

        dist = np.sqrt((x_k - x_li)**2 + (y_k - y_li)**2)
        H_i = np.array([[(x_k - x_li)/dist, (y_k - y_li)/dist, 0],
                        [(x_li - x_k)/dist**2, (y_li - y_k)/dist**2, -1 ]])

        H.append(H_i)

    return np.vstack(H)

def predict(DiffDrive, x_hat, sigma, u, dt, V):
    """
        :param y: measurement made at time t
        :param x_hat: initial estimate of state at time t-1
        :param sigma: initial covariance at time t-1
        :param u: input to the system at time t-1
        :param V: measurement noise covariance matrix
        :param dt: timestep
    """

    x_pred = DiffDrive.forward(x=x_hat, u=u, v=None, dt=dt)

    A, B = DiffDrive.linearize(x=x_hat, u=u, dt=dt)

    V = DiffDrive.get_V()

    sigma_pred = A @ sigma @ A.T + V

    return x_pred, sigma_pred

def update(Sensor, y, x, sigma_pred, W):
    y_pred = Sensor.measure(x)
    H = Sensor.jacobian(x)

    W = Sensor.get_W() if W is None else W

    residual = y - y_pred
    S = H @ sigma_pred @ H.T + W
    K = sigma_pred @ H.T @ np.linalg.pinv(S)
    x_posterior = x + K @ residual
    sigma_posterior = (np.eye(sigma_pred.shape[0]) - K@H) @ sigma_pred

    return x_posterior, sigma_posterior


def EKF(DiffDrive,Sensor,y,x_hat,sigma,u,dt,V=None,W=None):
    """
    Implementation of an Extended Kalman Filter for a nonlinear process and nonlinear sensor. 


    Some of these matrixes will be non-square or singular. Utilize the pseudo-inverse function
    la.pinv instead of inv to avoid these errors.

    Input
      :param DiffDrive: The DifferentialDrive object defined in kinematics.py
      :param Sensor: The Landmark Sensor object defined in this class
      :param y: The measurement made at time t -> np.array with shape (2*N_Landmarks,)
      :param x_hat: The starting estimate of the state at time t-1 -> np.array with shape (3,)
      :param sigma: The state covariance matrix at time t-1 -> np.array with shape (3,3)
      :param u: The input to the system at time t-1 -> np.array with shape (2,)
      :param V: The state noise covariance matrix -> np.array with shape (3,3)
      :param W: The measurment noise covariance matrix -> np.array with shape (2*N_Landmarks, 2*N_Landmarks)

      :param dt: timestep size delta t

    Output
      :return: x_hat_1: The estimate of the state at time t
      :return: sigma_1: The covariance of the state estimate at time t

    """

    x_pred, sigma_pred = predict(DiffDrive, x_hat, sigma, u, dt, V)
    x_posterior, sigma_posterior = update(Sensor, y, x_pred, sigma_pred, W)

    return x_posterior, sigma_posterior
