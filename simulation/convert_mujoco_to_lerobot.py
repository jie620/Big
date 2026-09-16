#!/usr/bin/env python3
"""Convert successful MuJoCo raw episodes to a local LeRobot v2.1 dataset."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LEROBOT_SRC = ROOT / "vendor" / "lerobot-smolvla-jetson" / "src"
if str(LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(LEROBOT_SRC))

from lerobot.datasets.lerobot_dataset import LeRobotDataset


TASKS = ("抓取橘子", "抓取橘子并放到左侧", "抓取橘子并放到右侧")


def read_frames(video_path: Path, expected: int):
    capture = cv2.VideoCapture(str(video_path))
    try:
        for index in range(expected):
            ok, bgr = capture.read()
            if not ok:
                raise RuntimeError(f"{video_path}: expected {expected} frames, stopped at {index}")
            yield cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        ok, _ = capture.read()
        if ok:
            raise RuntimeError(f"{video_path}: contains more than {expected} frames")
    finally:
        capture.release()


def convert(source: Path, output: Path, limit_per_task: int | None) -> dict:
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    manifest = json.loads((source / "dataset_info.json").read_text(encoding="utf-8"))
    by_task = {
        task: [episode for episode in manifest["episodes"] if episode.get("success") and episode.get("task") == task]
        for task in TASKS
    }
    selected = [
        episode
        for task in TASKS
        for episode in (by_task[task] if limit_per_task is None else by_task[task][:limit_per_task])
    ]
    if not selected:
        raise ValueError("No successful episodes selected")

    features = {
        "observation.image": {
            "dtype": "video",
            "shape": (240, 320, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"],
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["Arm1_Joint", "Arm2_Joint", "Arm3_Joint", "Arm4_Joint", "Arm5_Joint", "grip_joint"],
        },
    }
    dataset = LeRobotDataset.create(
        repo_id="local/orange_v21",
        fps=int(manifest["fps"]),
        features=features,
        root=output,
        robot_type="dofbot_mujoco",
        use_videos=True,
        image_writer_threads=2,
        batch_encoding_size=8,
    )
    source_map = []
    try:
        for new_index, episode in enumerate(selected):
            source_index = int(episode["episode_index"])
            episode_dir = source / f"episode_{source_index:06d}"
            trajectory = np.load(episode_dir / "trajectory.npz")
            states = np.asarray(trajectory["state"], dtype=np.float32)
            actions = np.asarray(trajectory["action"], dtype=np.float32)
            if states.shape != actions.shape or states.ndim != 2 or states.shape[1] != 6:
                raise ValueError(f"episode {source_index}: invalid state/action shape {states.shape}/{actions.shape}")
            if len(states) != int(episode["frames"]):
                raise ValueError(f"episode {source_index}: metadata and trajectory frame counts differ")
            for frame_index, image in enumerate(read_frames(episode_dir / "observation.mp4", len(states))):
                dataset.add_frame(
                    {
                        "observation.image": image,
                        "observation.state": states[frame_index],
                        "action": actions[frame_index],
                    },
                    task=episode["task"],
                    timestamp=frame_index / dataset.fps,
                )
            dataset.save_episode()
            source_map.append({"new_episode_index": new_index, "source_episode_index": source_index, "task": episode["task"]})
            print(f"converted {new_index + 1}/{len(selected)} source_episode={source_index}")
        if dataset.episodes_since_last_encoding:
            start = dataset.num_episodes - dataset.episodes_since_last_encoding
            dataset.batch_encode_videos(start, dataset.num_episodes)
    except Exception:
        dataset.clear_episode_buffer()
        raise
    result = {
        "source": str(source.resolve()),
        "output": str(output.resolve()),
        "fps": int(manifest["fps"]),
        "image": "observation.image",
        "episodes": len(source_map),
        "tasks": dict(Counter(item["task"] for item in source_map)),
        "source_episode_map": source_map,
    }
    (output / "conversion_manifest.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=ROOT / "mujoco_dataset_v3")
    parser.add_argument("--output", type=Path, default=ROOT / "lerobot_orange_v3")
    parser.add_argument("--limit-per-task", type=int, default=None, help="Convert only N successful episodes per task")
    args = parser.parse_args()
    print(json.dumps(convert(args.source, args.output, args.limit_per_task), ensure_ascii=False))


if __name__ == "__main__":
    main()
