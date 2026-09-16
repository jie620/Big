#!/usr/bin/env python3
from pathlib import Path
import mujoco
import numpy as np
model=mujoco.MjModel.from_xml_path(str(Path(__file__).parent/"dofbot_pro/dofbot_pro.xml"))
data=mujoco.MjData(model)
assert model.nu==6
for i,name in enumerate([f"Arm{j}_Joint" for j in range(1,6)]+["grip_joint"]):
    joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
    assert joint>=0 and model.actuator_trnid[i,0]==joint
assert mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"route_obstacle_geom")>=0
for _ in range(100):mujoco.mj_step(model,data)
assert np.isfinite(data.qpos).all()
print("six actuator mappings, obstacle geometry and 100 physics steps passed")
