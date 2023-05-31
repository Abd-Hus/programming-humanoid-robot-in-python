'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import numpy as np
timeref = -1
bezier_data = [[],[],[]]

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        target_joints = perception.joint
        
        global timeref
        global bezier_data
        if timeref == -1:
            timeref = perception.time
        timediff = round(perception.time - timeref, 4)
        if bezier_data == [[],[],[]]:
            bezier_data[0] = keyframes[0]
            t = np.linspace(0,1,101)
            for joint in keyframes[0]:
                index = keyframes[0].index(joint)
                timesteps = len(keyframes[1][index]) - 1
                angle_values = []
                timeframe = []
                for i in range(timesteps):
                    dt = keyframes[1][index][i+1] - keyframes[1][index][i]
                    P0 = keyframes[2][index][i][0]
                    P3 = keyframes[2][index][i+1][0]
                    P1 = P0 + keyframes[2][index][i][2][2]
                    P2 = P3 + keyframes[2][index][i+1][1][2]
                    for j in t:
                        if i != 0 and j == 0:
                            continue
                        timeframe.append(keyframes[1][index][i] + j*dt)
                        angle = (1-j)**3 * P0 + 3 * (1-j)**2 * j * P1 + 3 * (1-j) * j**2 * P2 + j**3 * P3
                        angle_values.append(angle)
                bezier_data[1].append(timeframe)
                bezier_data[2].append(angle_values)
        for i in range(len(bezier_data[0])):
            if timediff in bezier_data[1][i]:
                target_joints[bezier_data[0][i]] = bezier_data[2][i][bezier_data[1][i].index(timediff)]
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
    
