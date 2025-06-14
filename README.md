# Two-Robot MuJoCo Simulation

This project simulates a two-robot collaborative task using the MuJoCo physics engine. Robot A detects and relocates an object to a platform. Robot B picks it from the platform and places it in a designated hole.

## 📁 Project Structure

- `mujoco_models/`: Contains the MJCF XML model.
- `src/`: Python control logic using MuJoCo API.
- `docs/`: Project description and technical documentation.

## 🚀 Run the Simulation

```bash
cd src
python simulate_two_robots.py
