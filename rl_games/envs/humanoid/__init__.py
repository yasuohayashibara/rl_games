import gym

gym.envs.register(
     id='GankenKunEnv-v0',
     entry_point='rl_games.envs.humanoid.GankenKunEnv:GankenKunEnv',
)
