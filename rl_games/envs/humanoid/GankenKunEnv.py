import gym
import numpy as np
import sys
sys.path.append('./rl_games/envs/humanoid')
import webots_client

class GankenKunEnv(gym.Env):
    DIRECTION = np.array([1,1,-1,-1,1,1,1,1,-1,1,1,-1,-1,1,1,1,1,-1,1])
    OFFSET = np.array([0,0,-0.2,0.2,0,0,0,0,0,0,0,-0.2,0.2,0,0,0,0,0])
    def __init__(self, **kwargs):
        gym.Env.__init__(self)
        self.webots = webots_client.Webots()
        self.webots.initializeSensors()
        self.walking_phase = 0
        self.dt = 0.008
        self.walking_period = 0.5
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(19, ), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-6, high=6, shape=(32, ), dtype=np.float32)
    
    def step(self, action):
        self.walking_phase = (self.walking_phase+self.dt/self.walking_period)%1.0
        self.walking_phase += action[18]*self.dt/self.walking_period*0.5
        #print("phase: "+str(self.walking_phase))
        right = 1-abs(2*((self.walking_phase*2)%1)*(self.walking_phase%1<0.5)-1)
        left = 1-abs(2*((self.walking_phase*2)%1)*(self.walking_phase%1>0.5)-1)
        #print("right: "+str(right)+", left: "+str(left))
        action = self.DIRECTION * action
        action[2] += right
        action[3] -= right
        action[11] += left
        action[12] -= left
        #print(action)
        self.webots.setAngle(0.2 * action[0:18] + self.OFFSET)
        self.webots.measureSensors()
        state = []
        gravity_vec = [0, 0, -1]
        pos = [0, 0, 0]
        local_ball_pos = [0, 0, 0]
        forward_vec = [1, 0, 0]
        projected_foward = [1, 0, 0]
        obj = self.webots.sensorMeasurements.object_positions
        if len(obj) > 1:
            blue1 = self.webots.sensorMeasurements.object_positions[1]
            rot = blue1.rotation
            #gravity_vec = quat_rotate(rot, gravity_vec)
            pos = [blue1.position.X, blue1.position.Y, blue1.position.Z] 
            ball = self.webots.sensorMeasurements.object_positions[0]
            local_ball_pos = [ball.position.X - pos[0], ball.position.Y - pos[1], ball.position.Z - pos[2]]
            print(ball.position)

        state += gravity_vec
        state += action.tolist()
        state += pos
        state += local_ball_pos
        state += projected_foward
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
