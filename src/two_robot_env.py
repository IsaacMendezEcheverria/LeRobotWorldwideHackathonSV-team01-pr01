import gym
from gym import spaces
import numpy as np
import mujoco
import os

class TwoRobotEnv(gym.Env):
    def __init__(self):
        super(TwoRobotEnv, self).__init__()

        # Ruta al archivo XML (ajustá si estás en otro path)
        xml_path = os.path.join(os.path.dirname(__file__), "../mujoco_models/two_robot_scene.xml")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Viewer (para renderizado opcional)
        self.viewer = None

        # Espacios de acción y observación (4 motores de control)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

        # Observación simplificada: posiciones de juntas y del objeto
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)

    def reset(self):
        self.data = mujoco.MjData(self.model)  # reiniciar estado
        return self._get_obs()

    def step(self, action):
        # Aplicar acción
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        # Calcular recompensa (ejemplo básico)
        reward = -np.linalg.norm(self.data.qpos[:2])  # distancia al origen
        done = False  # por ahora nunca termina
        return self._get_obs(), reward, done, {}

    def _get_obs(self): # Obtener ID del cuerpo llamado 'object'
        object_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'object')
        object_pos = self.data.xpos[object_id].copy()  # posición del cubo
        
        return np.concatenate([
            self.data.qpos[:4],   # 4 articulaciones
            self.data.qvel[:4],   # velocidades
            object_pos            # posición del cubo (x, y, z)
            ])


    def render(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
