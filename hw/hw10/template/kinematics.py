''' ####################
EN605.613 - Introduction to Robotics
Assignment 5
-------
For this assignment you must implement the Linearize function of the DifferentialDrive class.
The forward and inverse functions are provided

Use the vehicle kinematics defined in Lecture 3. 

#################### '''

import sys
import numpy as np

class DifferentialDrive(object):
  """
  Implementation of Differential Drive kinematics.
  This represents a two-wheeled vehicle defined by the following states
  state = [x,y,theta]
  and accepts the following control inputs
  input = [left_wheel_rotation_rate,right_wheel_rotation_rate]

  """
  def __init__(self,L=1,R=1,K_P=1):
    """
    Initializes the class
    Input
      :param L: The distance between the wheels
      :param R: The radius of the wheels
    """

    self.L = L
    self.R = R
    self.u_limit = 1 / R
    self.V = None
    self.K_P = 1

  def get_state_size(self):
    return 3

  def get_input_size(self):
    return 2

  def get_V(self):
    '''
    This function provides the covariance matrix V which describes the noise that can be applied to the forward kinematics.
    
    Feel free to experiment with different levels of noise.

      Output
        :return: V: input cost matrix
    '''
    if self.V is None:
        self.V = np.eye(3)
        self.V[0,0] = 0.01
        self.V[1,1] = 0.01
        self.V[2,2] = 0.1
    return 1e-5*self.V

  def forward(self,x,u,v,dt):
    """
    Computes the forward kinematics for the system.

    Input
      :param x0: The starting state (position) of the system (units:[m,m,rad])
      :param u: The control input to the system (e.g. wheel rotation rates) (units: [rad/s,rad/s])
      :param v: The noise applied to the system (units:[m/s, m/s])

    Output
      :return: x1: The new state of the system

    """
    u0 = np.clip(u,-self.u_limit,self.u_limit)

    if v is None:
      v = 0
      
    th = x[2]

    vl = u0[0]
    vr = u0[1]

    v1 = self.R*(vl/2+vr/2)

    omega = self.R/self.L*(vr-vl)

    dX = np.array([v1*np.cos(th),
          v1*np.sin(th),
          omega])

    x1 = x + (v+dX) * dt

    return x1
  
  def inverse(self,x, v):
    """
    Computes the inverse kinematics for the DiffDrive system (vehicle motion --> wheel motion).

    Input
      :param x: The starting state (position) of the system. This is [x,y,theta].
      :param v: The desired velocity vector for the system (in robots frame of reference).
                This is [speed, turn_rate] where Vtheta is the yaw rate (rotation rate around the z-axis)

    Output
      :return: u: The necessary control inputs to achieve the desired velocity vector. This is the rotation rates for each wheel [psi_dot_left,psi_dot_right] (e.g. radians per second)

    """

    psi_left  = 1 / self.R * (v[0] - self.L/2 * v[1])
    psi_right =  1 / self.R * (v[0] + self.L/2 * v[1])

    return np.array([psi_left, psi_right])
  
<<<<<<< HEAD
  def PID(self,x, x_goal,dt=1.0):
=======
  def PID(self,x, x_goal,dt=1):
>>>>>>> ef1e312e4d85d060950acdcd33e2ebc771769b93
    """
    Computes the Proportional control for the diff drive system given
    the current state and the desired state. Do not worry about the derivative
    or integral term for this controller. 

    Input
      :param x: The state of the system 
      :param x_goal: The control input to the system (e.g. wheel rotation rates)
      :param dt: The timestep of the simulation
      
    Output
      :return u: The control input to the system (e.g. wheel rotation rates)
    """
    
    x_r = x[0]
    y_r = x[1]
    th_r = x[2]
    pid_length_offset = 0.1
    
    x_p  =  x_r + pid_length_offset * np.cos(th_r)
    y_p  =  y_r + pid_length_offset * np.sin(th_r)

    x_g = x_goal[0]
    y_g = x_goal[1]

    delta_x = x_p - x_g 
    delta_y = y_p - y_g 

    v_x = 0
    v_y = 0 

    vf =  - self.K_P * ( np.cos(th_r) * delta_x + np.sin(th_r) * delta_y) - self.K_P * ( np.cos(th_r) * v_x + np.sin(th_r) * v_y)

    wl = - self.K_P * ( - np.sin(th_r) * delta_x + np.cos(th_r) * delta_y) / self.L - self.K_P * ( - np.sin(th_r) * v_x + np.cos(th_r) * v_y) / self.L
 
    v_new = np.array([vf,wl])
    
    return self.inverse(x, v_new)
  
  def linearize(self,x,u,dt=1):
    """
    Computes the first order jacobian for A and B around the state x and input u

    Worth 25 points

    Input
      :param x: The state of the system which has shape (3,)
      :param u: The control input to the system (e.g. wheel rotation rates) which has shape (2,)

    Output
      :return: A: The Jacobian of the kinematics with respect to x which has shape (3,3)
      :return: B: The Jacobian of the kinematics with respect to u which has shape (3,2)

<<<<<<< HEAD
    """ 
    x_k, y_k, theta = x
    v, omega = u

    A = np.array([[1, 0, -v*np.sin(theta) * dt],
                  [0, 1, v*np.cos(theta) * dt],
                  [0, 0, 1]])

    B = np.array([
        [np.cos(theta) * dt, 0],
        [np.sin(theta) * dt, 0],
        [0, dt]
    ])

    return A, B
=======
    """
    
    #25 points: Implement this function
    
    raise NotImplementedError
>>>>>>> ef1e312e4d85d060950acdcd33e2ebc771769b93
