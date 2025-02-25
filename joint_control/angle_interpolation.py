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

time_ref = -1

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

        global time_ref
        if time_ref == -1:
            time_ref = perception.time
        t = round(perception.time - time_ref, 4)
        for joint in keyframes[0]:
            index_joint = keyframes[0].index(joint)
            index_time = 0
            n = len(keyframes[1][index_joint])
            while (t > keyframes[1][index_joint][index_time] and index_time < n-1):
                index_time += 1
            if (index_time > 0 and index_time < n-1):
                dt = keyframes[1][index_joint][index_time+1] - keyframes[1][index_joint][index_time]
                norm_time = (t - keyframes[1][index_joint][index_time]) / dt
                P0 = keyframes[2][index_joint][index_time][0]
                P3 = keyframes[2][index_joint][index_time+1][0]
                P1 = P0 + keyframes[2][index_joint][index_time][2][2]
                P2 = P3 + keyframes[2][index_joint][index_time+1][1][2]
                angle = (1-norm_time)**3 * P0 + 3 * (1-norm_time)**2 * norm_time * P1 + 3 * (1-norm_time) * norm_time**2 * P2 + norm_time**3 * P3
                target_joints[joint] = angle

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
    
