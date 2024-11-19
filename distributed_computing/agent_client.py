'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import threading
import xmlrpc.client as xmlrpclib
from numpy.matlib import identity, matrix
from keyframes import *
import time
class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        agent.execute_keyframes_nonblock(keyframes)

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        agent.set_transform_nonblock(effector_name, transform)


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    proxy = None
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.proxy = xmlrpclib.ServerProxy("http://localhost:8000/")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.proxy.execute_keyframes(keyframes)

    def execute_keyframes_nonblock(self, keyframes):
        '''excute keyframes, note this function is non-blocking call
        '''
        # YOUR CODE HERE
        self.proxy.execute_keyframes_nonblock(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return matrix(self.proxy.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.proxy.set_transform(effector_name, transform)
    
    def set_transform_nonblock(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.proxy.set_transform_nonblock(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    
    agent.post.execute_keyframes(hello())
    time.sleep(1)
    print(agent.get_angle('HeadYaw'))
    print(agent.get_angle('HeadYaw'))
    time.sleep(5)
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    T[-1, 0] = 0.2
    agent.post.set_transform('RLeg', T.tolist())
    time.sleep(5)
    agent.set_angle('HeadYaw', -1)
    print(agent.get_angle('HeadYaw'))
    print(agent.get_transform('LHipRoll'))
    print(agent.get_posture())
    agent.set_transform('LLeg', T.tolist())
    agent.execute_keyframes(leftBackToStand())
    agent.execute_keyframes(hello())
    