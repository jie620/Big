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
def geom(name):
    ident=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,name)
    assert ident>=0, f"missing geometry: {name}"
    return ident

target=geom("target_cube_geom")
pick=geom("obstacle_pick_geom")
place=geom("obstacle_place_geom")
zone=geom("place_zone_geom")
np.testing.assert_allclose(model.geom_size[target],[0.015,0.015,0.015])
np.testing.assert_allclose(model.geom_size[pick],[0.015,0.015,0.03])
np.testing.assert_allclose(model.geom_size[place],[0.015,0.015,0.03])
np.testing.assert_allclose(model.geom_size[zone],[0.025,0.025,0.0005])
assert mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,"route_obstacle_geom")<0
for _ in range(100):mujoco.mj_step(model,data)
assert np.isfinite(data.qpos).all()
print("six actuator mappings, 3 cm cube, two 3x3x6 cm blockers, place zone and 100 physics steps passed")
