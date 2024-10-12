
''' ####################
    EN605.613 - Introduction to Robotics
    Assignment 4
    Vehicle Kinematics
    -------
    For this assignment you must implement the forward and inverse kinematics functions for the Mechanum class.

    ==================================
    Copyright 2024,
    The Johns Hopkins University Whiting School of Engineering
    All Rights Reserved.
    #################### '''

import numpy as np

class Mecanum:
    """
    Kinematic implementation of a robot with 4 Mecanum wheels.
    Assume all wheels have identical sizes and pitch angles.
    Assume the body is symmetrical with the center of mass in the center.
    """

    def __init__(self,length,width,wheel_radius,roller_angle,roller_radius):
        """
        Constructor for the Mecanum kinematics class. Sets the vehicle properties.

        Input
        :param length: The length of the vehicle
        :param width: The width of the vehicle
        :param wheel_radius: The radius of the wheels
        :param wheel_radius: The radius of the wheels
        :param roller_angle: The pitch angle of the rollers on each wheel
        :param roller_radius: The radius of the rollers on each wheel

        """
        self.L = length
        self.W = width
        self.R = wheel_radius
        self.alpha = roller_angle
        self.r = roller_radius

    def forward(self,x,u):
        """
        Computes the forward kinematics for the Mecanum system.

        Input
        :param x: The starting state (position) of the system. This is [x,y,theta].
        :param u: The control input to the system. This is the wheel rates for each wheel [psi_1,psi_2,psi_3,psi_4]

        Output
        :return: v: The resulting velocity vector for the system. This is [Vx,Vy,Vtheta]

        """

        A = self.R * np.array([[1, (1/np.tan(self.alpha)), -1*(self.W/2 + self.L/2 * (1/np.tan(self.alpha)))],
                      [1, (-1/np.tan(self.alpha)), self.W/2 + self.L/2 * (1/np.tan(self.alpha))],
                      [1, (-1/np.tan(self.alpha)), -1*(self.W/2 + self.L * (1/np.tan(self.alpha)))],
                      [1, (1/np.tan(self.alpha)), self.W/2 + self.L/2 * (1/np.tan(self.alpha))]])
        A_inv = np.linalg.pinv(A)
        return A_inv @ u
        

    def inverse(self,x,v):

        """
        Computes the inverse kinematics for the Mecanum system.

        Input
        :param x: The starting state (position) of the system.This is [x,y,theta].
        :param v: The desired velocity vector for the system. This is [Vx,Vy,Vtheta]

        Output
        :return: u: The necessary control inputs to achieve the desired velocity vector.This is the wheel rates for each wheel [psi_1,psi_2,psi_3,psi_4]
        """

        A = self.R * np.array([[1, (1/np.tan(self.alpha)), -1*(self.W/2 + self.L/2 * (1/np.tan(self.alpha)))],
                      [1, (-1/np.tan(self.alpha)), self.W/2 + self.L/2 * (1/np.tan(self.alpha))],
                      [1, (-1/np.tan(self.alpha)), -1*(self.W/2 + self.L/2 * (1/np.tan(self.alpha)))],
                      [1, (1/np.tan(self.alpha)), self.W/2 + self.L/2 * (1/np.tan(self.alpha))]])

        return A @ v





def main():

    x0 = np.array([0,0,0])

    length = 0.3
    width =  0.15
    wheel_radius = 0.05
    roller_radius = 0.01
    roller_angle = 45/360*np.pi

    mecanum = Mecanum(length,width,wheel_radius,roller_radius,roller_angle)

    u0 = np.array([2,2,2,2])

    print(f'Simulating wheel inputs of {u0} for 3 seconds')
    dt = 0.1
    t = 0
    x = x0
    while t<3:
        v = mecanum.forward(x,u0)
        print("V: ",v)
        x = x + v*dt
        t+=dt
        print(f'{t}:{x}')

    v_desired = np.array([0,0,45/360*np.pi])
    print(f'Rotating in place with {v_desired} for 1 seconds')
    t=0
    while t<1:
        u = mecanum.inverse(x,v_desired)
        v = mecanum.forward(x,u)
        x = x + v*dt
        np.add(x, v*dt, out=x, casting="unsafe")
        t+=dt
        print(f'{t}:{x}')

    v_desired = np.array([0.5,0.5,0])
    print(f'Translating with  {v_desired} for 2 seconds')
    t=0
    while t<2:
        u = mecanum.inverse(x,v_desired)
        v = mecanum.forward(x,u)
        x = x + v*dt
        np.add(x, v*dt, out=x, casting="unsafe")
        t+=dt
        print(f'{t}:{x}')

if __name__ == '__main__':
    main()

