"""Collect simple, portable VLA demonstrations from the DOFBOT MuJoCo model.

The collector is simulation-only.  It deliberately uses the same six actuator
values as the model (radians), and never opens an I2C/ROS connection.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

# Must be set before importing mujoco on headless Jetson/CI sessions.
os.environ.setdefault("MUJOCO_GL", "egl")
import cv2
import mujoco
import numpy as np


MODEL = Path(__file__).parent / "dofbot_pro" / "dofbot_pro.xml"
JOINTS = ("Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint")
INITIAL = np.array([0.0, 0.4, -1.3, -1.35, 0.0, 0.0], dtype=np.float32)
ARM1_LIMIT = np.deg2rad(90.0)
SEARCH_MARGIN = np.deg2rad(5.0)
FPS = 20
DT = 1.0 / FPS
IMAGE_SIZE = (320, 240)
STAGES = ("searching", "approach", "lower", "close", "align", "lift", "move_left", "move_right", "release", "restore", "done", "failed")
TASKS = {
    "grasp": "抓取橘子",
    "left": "抓取橘子并放到左侧",
    "right": "抓取橘子并放到右侧",
}
ATTACH_GRIPPER_CLOSED = -0.5
ATTACH_DISTANCE = 0.05


def ids(model):
    return [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]


def qpos(model, data, joint_ids):
    return np.array([data.qpos[model.jnt_qposadr[j]] for j in joint_ids], dtype=np.float32)


def set_pose(model, data, joint_ids, pose):
    data.ctrl[:] = pose
    for j, value in zip(joint_ids, pose):
        data.qpos[model.jnt_qposadr[j]] = value
    mujoco.mj_forward(model, data)


def solve_ik(model, data, target, iterations=250, seed=None):
    trial = mujoco.MjData(model)
    trial.qpos[:] = data.qpos
    if seed is not None:
        for name, value in zip(JOINTS[:5], seed):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            trial.qpos[model.jnt_qposadr[joint]] = value
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_point")
    arm_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS[:5]]
    dofs = [model.jnt_dofadr[j] for j in arm_ids]
    for _ in range(iterations):
        mujoco.mj_forward(model, trial)
        error = np.asarray(target) - trial.site_xpos[site]
        if np.linalg.norm(error) < 1e-4:
            break
        jac = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, trial, jac, None, site)
        j = jac[:, dofs]
        step = j.T @ np.linalg.solve(j @ j.T + 1e-4 * np.eye(3), error)
        step *= min(1.0, 0.05 / max(np.linalg.norm(step), 1e-9))
        for joint, delta in zip(arm_ids, step):
            address = model.jnt_qposadr[joint]
            trial.qpos[address] = np.clip(trial.qpos[address] + delta, *model.jnt_range[joint])
    mujoco.mj_forward(model, trial)
    result = np.array([trial.qpos[model.jnt_qposadr[j]] for j in arm_ids], dtype=np.float32)
    return result, float(np.linalg.norm(np.asarray(target) - trial.site_xpos[site]))


def orange_visible(model, data, renderer, orange_id):
    renderer.update_scene(data, camera="yolo")
    rgb = renderer.render()
    # Synthetic orange has a stable color; this is the simulated detector.
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (5, 100, 80), (35, 255, 255))
    area = int(cv2.countNonZero(mask))
    return area >= 8, rgb


def carry_orange(model, data, orange_id, site_id, offset):
    rotation = data.site_xmat[site_id].reshape(3, 3)
    address = model.jnt_qposadr[model.body_jntadr[orange_id]]
    data.qpos[address:address + 3] = data.site_xpos[site_id] + rotation @ offset
    data.qvel[model.jnt_dofadr[model.body_jntadr[orange_id]]:][:6] = 0.0
    mujoco.mj_forward(model, data)


def run_episode(model, rng, task, renderer, fps):
    joint_ids = ids(model)
    data = mujoco.MjData(model)
    # Keep the orange in the true ±90° base-yaw workspace and away from edges.
    orange_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "orange")
    set_pose(model, data, joint_ids, INITIAL)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_point")
    # Rejection-sample within the physical ±90° workspace until IK confirms
    # the point is reachable; unreachable candidates are never written.
    for _ in range(100):
        theta = rng.uniform(np.deg2rad(-80), np.deg2rad(80))
        radius = rng.uniform(0.22, 0.32)
        orange_pos = np.array([radius * np.sin(theta), radius * np.cos(theta), 0.03])
        target = orange_pos + [0, 0, rng.uniform(0.09, 0.12)]
        data.qpos[model.jnt_qposadr[model.body_jntadr[orange_id]]:][:3] = orange_pos
        mujoco.mj_forward(model, data)
        approach, err = solve_ik(model, data, target)
        if err <= 0.01 and abs(float(approach[0])) <= ARM1_LIMIT + 1e-5:
            break
    else:
        return {"task": TASKS[task], "success": False, "failure": "workspace_sampling"}

    # Search is an actual bounded base scan, not a label invented after the fact.
    scan = np.linspace(INITIAL[0], np.sign(approach[0]) * (ARM1_LIMIT - SEARCH_MARGIN), 36)
    rows, frames = [], []
    stage = "searching"
    attached = False
    orange_offset = None
    locked = False
    scan_seconds = rng.uniform(0.08, 0.12)
    for arm1 in scan:
        pose = INITIAL.copy(); pose[0] = np.clip(arm1, -ARM1_LIMIT, ARM1_LIMIT)
        data.ctrl[:] = pose
        for _ in range(max(1, round(scan_seconds / model.opt.timestep))):
            mujoco.mj_step(model, data)
        visible, rgb = orange_visible(model, data, renderer, orange_id)
        frame = cv2.resize(rgb, IMAGE_SIZE)
        rows.append((qpos(model, data, joint_ids), pose.copy(), stage, visible))
        frames.append(frame)
        if visible:
            locked = True
            break
    if not locked:
        rows.append((qpos(model, data, joint_ids), data.ctrl.copy(), "failed", False))
        return {"task": TASKS[task], "success": False, "failure": "search_timeout", "rows": rows, "frames": frames}

    def move(goal, name, seconds=1.0):
        nonlocal stage
        stage = name
        start = qpos(model, data, joint_ids)
        for alpha in np.linspace(0.0, 1.0, max(2, round(seconds * fps))):
            pose = start * (1 - alpha) + np.r_[goal, data.ctrl[5]] * alpha
            pose[0] = np.clip(pose[0], -ARM1_LIMIT, ARM1_LIMIT)
            data.ctrl[:] = pose
            for _ in range(max(1, round(DT / model.opt.timestep))):
                mujoco.mj_step(model, data)
                if attached:
                    carry_orange(model, data, orange_id, site_id, orange_offset)
            visible, rgb = orange_visible(model, data, renderer, orange_id)
            rows.append((qpos(model, data, joint_ids), pose.copy(), stage, visible))
            frames.append(cv2.resize(rgb, IMAGE_SIZE))

    move(approach, "approach", rng.uniform(0.8, 1.2))
    grasp, err = solve_ik(model, data, orange_pos + [0, 0, 0.035])
    if err > 0.012:
        return {"task": TASKS[task], "success": False, "failure": "grasp_ik", "rows": rows, "frames": frames}
    move(grasp, "lower", rng.uniform(0.8, 1.2))
    stage = "close"
    for _ in range(fps // 2):
        pose = data.ctrl.copy(); pose[5] = -0.8; data.ctrl[:] = pose
        for _ in range(max(1, round(DT / model.opt.timestep))): mujoco.mj_step(model, data)
        visible, rgb = orange_visible(model, data, renderer, orange_id)
        rows.append((qpos(model, data, joint_ids), pose.copy(), stage, visible)); frames.append(cv2.resize(rgb, IMAGE_SIZE))
    gripper_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "grip_joint")
    gripper_address = model.jnt_qposadr[gripper_joint]

    def attach_if_valid():
        if data.qpos[gripper_address] > ATTACH_GRIPPER_CLOSED:
            return False, None
        distance = np.linalg.norm(data.xpos[orange_id] - data.site_xpos[site_id])
        if distance > ATTACH_DISTANCE:
            return False, None
        rotation = data.site_xmat[site_id].reshape(3, 3)
        return True, rotation.T @ (data.xpos[orange_id] - data.site_xpos[site_id])

    attached, orange_offset = attach_if_valid()
    if not attached:
        # Keep only demonstrations that satisfy the runner's grasp rule.
        align, err = solve_ik(model, data, data.xpos[orange_id] + [0, 0, 0.035])
        if err > 0.005:
            return {"task": TASKS[task], "success": False, "failure": "align_ik", "rows": rows, "frames": frames}
        move(align, "align", rng.uniform(0.3, 0.5))
        attached, orange_offset = attach_if_valid()
        if not attached:
            return {"task": TASKS[task], "success": False, "failure": "grasp_contact", "rows": rows, "frames": frames}
    move(approach, "lift", rng.uniform(0.8, 1.2))
    if task != "grasp":
        # Solve for the gripper site just above the ground.  The carried
        # orange center then rests on the ground before the fingers open.
        place = np.array([
            rng.uniform(-0.15, -0.09) if task == "left" else rng.uniform(0.09, 0.15),
            rng.uniform(0.22, 0.26),
            0.06,
        ])
        place_q, err = solve_ik(model, data, place, seed=INITIAL[:5])
        if err > 0.015 or abs(float(place_q[0])) > ARM1_LIMIT:
            return {"task": TASKS[task], "success": False, "failure": "place_ik", "rows": rows, "frames": frames}
        move(place_q, "move_left" if task == "left" else "move_right", rng.uniform(0.8, 1.2))
        stage = "release"
        attached = False
        for _ in range(fps // 2):
            pose = data.ctrl.copy(); pose[5] = 0.0; data.ctrl[:] = pose
            for _ in range(max(1, round(DT / model.opt.timestep))): mujoco.mj_step(model, data)
            visible, rgb = orange_visible(model, data, renderer, orange_id)
            rows.append((qpos(model, data, joint_ids), pose.copy(), stage, visible)); frames.append(cv2.resize(rgb, IMAGE_SIZE))
        # Restore the arm with the gripper open after the orange is released.
        move(INITIAL[:5], "restore", rng.uniform(0.8, 1.2))
    rows.append((qpos(model, data, joint_ids), data.ctrl.copy(), "done", False)); frames.append(frames[-1])
    return {"task": TASKS[task], "success": True, "rows": rows, "frames": frames}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=2000, help="successful-target episodes per task")
    ap.add_argument("--output", type=Path, default=Path("mujoco_dataset_v3"))
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite existing dataset: {args.output}")
    args.output.mkdir(parents=True)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    assert np.allclose(model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Arm1_Joint")], [-ARM1_LIMIT, ARM1_LIMIT], atol=1e-4)
    renderer = mujoco.Renderer(model, height=IMAGE_SIZE[1], width=IMAGE_SIZE[0])
    rng = np.random.default_rng(args.seed)
    manifest = {"format": "mujoco-vla-raw-v1", "fps": FPS, "image_size": IMAGE_SIZE, "arm1_range_deg": [-90, 90], "tasks": list(TASKS.values()), "episodes": []}
    try:
        for task in TASKS:
            successes = 0
            attempts = 0
            while successes < args.episodes:
                attempts += 1
                if attempts > args.episodes * 2 + 100:
                    raise RuntimeError(f"Could not collect {args.episodes} successful {task} episodes")
                result = run_episode(model, rng, task, renderer, FPS)
                episode_id = len(manifest["episodes"])
                ep_dir = args.output / f"episode_{episode_id:06d}"
                ep_dir.mkdir()
                rows = result.get("rows", [])
                if rows:
                    states, actions, stages, visible = zip(*rows)
                    np.savez_compressed(ep_dir / "trajectory.npz", state=np.stack(states), action=np.stack(actions), stage=np.array(stages), orange_visible=np.array(visible))
                    writer = cv2.VideoWriter(str(ep_dir / "observation.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), FPS, IMAGE_SIZE)
                    for frame in result["frames"]: writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    writer.release()
                meta = {"episode_index": episode_id, "task": result["task"], "success": bool(result["success"]), "failure": result.get("failure"), "frames": len(rows)}
                (ep_dir / "metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
                manifest["episodes"].append(meta)
                successes += int(result["success"])
                print(json.dumps(meta, ensure_ascii=False))
    finally:
        renderer.close()
    (args.output / "dataset_info.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
