#!/usr/bin/env python
"""Print one HEI MuJoCo dataset sample and export its three camera frames."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from lerobot.datasets import LeRobotDataset


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Inspect one HEI MuJoCo dataset episode.")
    parser.add_argument("--repo-id", default="HGM/hei_rebot_lift_mujoco")
    parser.add_argument("--root", type=Path, default=None, help="Local dataset root.")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--frame-index", type=int, default=0, help="Frame within the selected episode.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mujoco_dataset_sample"))
    return parser.parse_args()


def tensor_image_to_rgb(image) -> np.ndarray:
    array = image.detach().cpu().numpy() if hasattr(image, "detach") else np.asarray(image)
    if array.ndim == 3 and array.shape[0] in (1, 3, 4):
        array = np.moveaxis(array, 0, -1)
    if np.issubdtype(array.dtype, np.floating):
        array = np.clip(array * 255.0, 0.0, 255.0).astype(np.uint8)
    return array[..., :3]


def main() -> None:
    args = parse_args()
    dataset = LeRobotDataset(
        repo_id=args.repo_id,
        root=args.root,
        episodes=[args.episode_index],
        return_uint8=True,
    )
    if not 0 <= args.frame_index < len(dataset):
        raise IndexError(f"frame index {args.frame_index} is outside [0, {len(dataset) - 1}]")
    sample = dataset[args.frame_index]
    print(
        f"[HEI Sim Inspect] root={dataset.root}, episode={args.episode_index}, "
        f"frames={len(dataset)}, fps={dataset.fps}",
        flush=True,
    )
    print(f"[HEI Sim Inspect] observation.state={sample['observation.state']}", flush=True)
    print(f"[HEI Sim Inspect] action={sample['action']}", flush=True)
    print(f"[HEI Sim Inspect] task={sample.get('task', '')}", flush=True)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name in ("front", "left_wrist", "right_wrist"):
        key = f"observation.images.{name}"
        rgb = tensor_image_to_rgb(sample[key])
        path = args.output_dir / f"episode_{args.episode_index:03d}_frame_{args.frame_index:04d}_{name}.png"
        if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError(f"Failed to save {path}")
        print(f"[HEI Sim Inspect] {name} -> {path}", flush=True)


if __name__ == "__main__":
    main()
