import gym
import numpy as np
import sys
sys.path.append('./rl_games/envs/humanoid')
import webots_client

class GankenKunEnv(gym.Env):
    DIRECTION = np.array([1,1,-1,-1,1,1,1,1,1,1,1,-1,-1,1,1,1,1,1,0])
    OFFSET = np.array([0,0,-0.2,0.2,0,0,0,0,0,0,0,-0.2,0.2,0,0,0,0,0,0])
    def __init__(self, **kwargs):
        gym.Env.__init__(self)
        self.webots = webots_client.Webots()
        self.webots.initializeSensors()
        self.walking_phase = 0
        self.dt = 0.008
        self.walking_period = 0.68
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(19, ), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-5, high=5, shape=(32, ), dtype=np.float32)
        self.gravity_vec = [0, 0, -1]
        self.robot_pos = [0, 0, 0]
        self.local_ball_pos = [0, 0, 0]
        self.forward_vec = [1, 0, 0]
    
    def rotate(self, vector, rotation):
        axis = rotation[0:3]
        angle = rotation[3]
        axis = axis / np.linalg.norm(axis)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
        return np.dot(R, vector)

    def step(self, action):
        #action[:] = 0
        self.walking_phase = (self.walking_phase+self.dt/self.walking_period)%1.0
        self.walking_phase += action[18]*self.dt/self.walking_period*0.5
        right = 1-abs(2*((self.walking_phase*2)%1)*(self.walking_phase%1<0.5)-1)
        left = 1-abs(2*((self.walking_phase*2)%1)*(self.walking_phase%1>0.5)-1)
        action[2] += right
        action[3] -= right
        action[11] += left
        action[12] -= left
        angles = [0.2 * self.DIRECTION[i] * action[i] + self.OFFSET[i] for i in range(len(action))]

        self.webots.setAngle(angles)
        self.webots.measureSensors()
        state = []
        obj = self.webots.sensorMeasurements.object_positions
        if len(obj) > 2:
            pos = self.webots.sensorMeasurements.object_positions[1]
            rot = pos.rotation
            rot = np.array([rot.X, rot.Y, rot.Z, rot.W])
            gravity_vec = np.array([0, 0, -1])
            self.gravity_vec = self.rotate(gravity_vec, rot).tolist()
            forward_vec = np.array([1, 0, 0])
            self.forward_vec = self.rotate(forward_vec, rot).tolist()
            self.robot_pos = [pos.position.X, pos.position.Y, pos.position.Z]
            ball = self.webots.sensorMeasurements.object_positions[0]
            ball_pos = [ball.position.X, ball.position.Y, ball.position.Z]
            local_ball_pos = np.array([ball_pos[i] - self.robot_pos[i] for i in range(3)])
            rot[3] = -rot[3]
            self.local_ball_pos = self.rotate(local_ball_pos, rot).tolist()

        state += self.gravity_vec
        state += action.tolist()
        self.robot_pos[2] = 0
        state += self.robot_pos
        self.local_ball_pos[2] = 0
        state += self.local_ball_pos
        state += self.forward_vec
        state += [self.walking_phase]
        state = np.array(state, dtype=np.float32)

        reward = 0
        done = False
        return state, reward, done, {}

    def reset(self):
        obses = np.zeros(32, dtype=np.float32)
        return obses
    
    def display(self):
        print(self.webots.sensorMeasurements.time)
        print(self.webots.sensorMeasurements.accelerometers)
        print(self.webots.sensorMeasurements.gyros)
        print(self.webots.sensorMeasurements.object_positions)

    def get_number_of_agents(self):
        return 1

    def has_action_mask(self):
        return False

    def get_action_mask(self):
        pass

if __name__ == '__main__':
    gankenkun_env = GankenKunEnv()
    angle = [0.0]*19
    while True:
        obs, reward, done, _ = gankenkun_env.step(angle)
        #print(obs)
