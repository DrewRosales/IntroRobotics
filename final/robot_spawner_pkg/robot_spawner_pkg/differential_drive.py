import numpy as np

class DifferentialDrive:
    def __init__(self, radius=0.1, length=0.5):
        self.r = radius
        self.L = length
    def forward(self, u):
        '''
        Input:
        :param u: The control input to the system as left and right wheel velocities [VL, VR]

        Output:
        :param v: The resulting velocity vector [Vx, Vy, theta]
        '''

        A = self.r * np.array([[0.5, 0.5],
                          [-1/self.L, 1/self.L]])

        return A @ u


    def inverse(self, v):

        A_inv = 1/(2*self.r) * np.array([[2, -self.L],
                                         [2, self.L]])

        return A_inv @ v

#def main():
#
#    x0 = np.array([0,0,0])
#
#    length = 0.3
#    radius = 0.05
#
#    differential_drive = DifferentialDrive(radius=radius, length=length)
#
#    u0 = np.array([2,2])
#
#    print(f'Simulating wheel inputs of {u0} for 3 seconds')
#    dt = 0.1
#    t = 0
#    x = x0
#    while t<3:
#        v = differential_drive.forward(u0, x)
#        print("V: ",v)
#        x = x + v*dt
#        print("theta: ", x[2])
#        t+=dt
#        print(f'{t}:{x}')
#
#    v_desired = np.array([0,0,45/360*np.pi])
#    x = x0
#    print(f'Rotating in place with {v_desired} for 1 seconds')
#    t=0
#    while t<1:
#        u = differential_drive.inverse(v_desired, x)
#        v = differential_drive.forward(u, x)
#        x = x + v*dt
#        print("theta: ", x[2])
#        t+=dt
#        print(f'{t}:{x}')
#
#    v_desired = np.array([0.5,0,0])
#    x = x0
#    print(f'Moving forward for {v_desired} for 3 seconds')
#    t=0
#    while t<3:
#        u = differential_drive.inverse(v_desired, x)
#        v = differential_drive.forward(u, x)
#        x = x + v*dt
#        t+=dt
#        print(f'{t}:{x}')
#if __name__ == '__main__':
#    main()
