from two_robot_env import TwoRobotEnv

env = TwoRobotEnv()
obs = env.reset()
assert len(obs) == 11

for i in range(10):
    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)
    assert len(obs) == 11
    print(f"Step {i}: reward = {reward}")
    # env.render()  ← desactivado por ahora
    if done:
        print("Episode finished")
        break