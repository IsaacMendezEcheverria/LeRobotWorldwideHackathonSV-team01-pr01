import mujoco
import mujoco.viewer
import numpy as np

# Load the XML
model = mujoco.MjModel.from_xml_path("/home/isaacmendez/projects/git/LeRobotWorldwideHackathonSV-team01-pr01/mujoco_models/two_robot_scene.xml")
data = mujoco.MjData(model)

# Viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0
    while viewer.is_running():
        step += 1

        # Phase 1: Robot A moves object to platform
        if step < 300:
            data.ctrl[0] = 1.0  # robotA_joint1
            data.ctrl[1] = 0.2  # robotA_joint2
        # Phase 2: Robot B moves object from platform to hole
        elif 300 <= step < 600:
            data.ctrl[2] = -1.0  # robotB_joint1
            data.ctrl[3] = -0.2  # robotB_joint2
        else:
            data.ctrl[:] = 0.0

        mujoco.mj_step(model, data)
        viewer.sync()
