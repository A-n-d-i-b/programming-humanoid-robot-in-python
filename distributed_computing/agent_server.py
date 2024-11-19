'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

import threading
from inverse_kinematics import InverseKinematicsAgent
from numpy.matlib import matrix, identity
from xmlrpc.server import SimpleXMLRPCServer
import time

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, ip, port):
        super(ServerAgent, self).__init__()
        self.server_thread(ip, port)

    def server_thread(self, ip, port):
        self.server = SimpleXMLRPCServer((ip, port), allow_none=True)
        self.server.register_instance(self)
        threading.Thread(target=self.server.serve_forever).start()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.start = self.perception.time
        self.keyframes = keyframes
        while self.keyframes != ([], [], []):
            time.sleep(0.1)
        return True

    def execute_keyframes_nonblock(self, keyframes):
        '''excute keyframes, note this function is non-blocking call
        '''
        # YOUR CODE HERE
        t1 = threading.Thread(target=self.execute_keyframes, args=[keyframes], daemon=True)
        t1.start()

    def set_transform_nonblock(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        t2 = threading.Thread(target=self.set_transform, args=[effector_name, transform], daemon=True)
        t2.start()

    
    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name].tolist()

    
    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, matrix(transform))
        while self.keyframes != ([], [], []):
            time.sleep(0.1)
        return True

if __name__ == '__main__':
    agent = ServerAgent('localhost', 8000)
    agent.run()

