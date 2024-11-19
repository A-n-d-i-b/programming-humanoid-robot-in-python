'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
import os

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        # LOAD YOUR CLASSIFIER
        ROBOT_POSE_CLF = '../joint_control/robot_pose.pkl'
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF, 'rb'))

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        joint_list = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        angles_data = []
        for i in joint_list:
            angles_data.append(perception.joint[i])
        angles_data.append(perception.imu[0])
        angles_data.append(perception.imu[1])
        angles_data = [angles_data]
        pose_predicted = self.posture_classifier.predict(angles_data)
        ROBOT_POSE_DATA_DIR = '../joint_control/robot_pose_data' 
        classes = os.listdir(ROBOT_POSE_DATA_DIR)
        posture = classes[pose_predicted[0]]
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
