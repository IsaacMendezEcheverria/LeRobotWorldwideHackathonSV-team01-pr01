import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_string("<mujoco><worldbody></worldbody></mujoco>")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    step = 0
    while viewer.is_running():
        step += 1
        data.ctrl[:] = 0.0  # No control input
        mujoco.mj_step(model, data)
        viewer.sync()
        viewer.render()
