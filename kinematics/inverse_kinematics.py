'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''



from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from numpy import matrix, linalg
from math import atan2
import numpy as np
from scipy.optimize import fmin, fmin_slsqp, fmin_l_bfgs_b, fmin_tnc


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        self.ranges = {'HeadYaw': (-2.0857, 2.0857),
                       'HeadPitch': (-0.6720, 0.5149),
                       'LShoulderPitch': (-2.0857, 2.0857),
                       'RShoulderPitch': (-2.0857, 2.0857),
                       'LShoulderRoll': (-0.3142, 1.3265),
                       'RShoulderRoll': (-0.3142, 1.3265),
                       'LElbowYaw': (-2.0857, 2.0857),
                       'RElbowYaw': (-2.0857, 2.0857),
                       'LElbowRoll': (-1.5446, 0.0349),
                       'RElbowRoll': (-1.5446, 0.0349),
                       'LHipYawPitch': (-1.145303, 0.740810),
                       'RHipYawPitch': (-1.145303, 0.740810),
                       'LHipRoll': (-0.379472, 0.790477),
                       'RHipRoll': (-0.379472, 0.790477),
                       'LHipPitch': (-1.535889, 0.484090),
                       'RHipPitch': (-1.535889, 0.484090),
                       'LKneePitch': (-0.092346, 2.112528),
                       'RKneePitch': (-0.092346, 2.112528),
                       'LAnklePitch': (-1.189516, 0.922747),
                       'RAnklePitch': (-1.189516, 0.922747),
                       'LAnkleRoll': (-0.397880, 0.769001),
                       'RAnkleRoll': (-0.397880, 0.769001),}        
        # optimization approach
        joint_angles = []
        joint_names = self.chains[effector_name]
        joint_angles = [0 for j in range(len(joint_names))]
        bounds = [self.ranges[j] for j in joint_names]
        
        func = lambda joint_values: self.error_func(transform, joint_values, joint_names)

            
        return fmin_tnc(func, joint_angles, bounds = bounds, approx_grad = True, maxfun=10000)[0]

    
    
    def error_func(self, target, joint_values, joint_names):
        Ts = [identity(4)]
        for joint in range(len(joint_values)):
            angle = joint_values[joint]
            Tl = self.local_trans(joint_names[joint], angle)
            T = np.dot(Ts[-1], Tl)
            Ts.append(T)
        Te = Ts[-1]
        e = target - Te
        return linalg.norm(e)
    
    
    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])
        self.start = self.perception.time
        joint_angles = self.inverse_kinematics(effector_name, transform)
        for i in range(len(joint_angles)):
            self.keyframes[0].append(self.chains[effector_name][i])
            self.keyframes[1].append([5, 10])
            self.keyframes[2].append([[joint_angles[i], [3, 0, 0], [3, 0, 0]],
                                    [0, [3, 0, 0], [3, 0, 0]]])


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    T[-1, 0] = 0.2
    agent.set_transforms('LLeg', T)
    agent.run()
