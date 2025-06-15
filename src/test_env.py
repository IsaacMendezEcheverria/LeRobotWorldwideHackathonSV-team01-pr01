from two_robot_env import TwoRobotEnv

env = TwoRobotEnv()
obs = env.reset()

for i in range(1000):
    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)
    print(f"Step {i}: reward = {reward}")
    # env.render()  ← desactivado por ahora
    if done:
        print("Episode finished")
        break