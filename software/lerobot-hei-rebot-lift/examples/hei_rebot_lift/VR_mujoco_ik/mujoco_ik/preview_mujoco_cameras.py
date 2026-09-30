#!/usr/bin/env python
"""Render one PNG from each named HEI Robot MuJoCo camera."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import mujoco

from hei_robot_mujoco_scene import ROBOT_CAMERAS, build_mujoco_model


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_URDF = BASE_DIR / "model" / "HEI_robot_urdf" / "urdf" / "HEI_robot_urdf.urdf"
DEFAULT_OUTPUT_DIR = BASE_DIR / "outputs" / "camera_preview"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render the three HEI Robot named cameras.")
    parser.add_argument("--model", type=Path, default=DEFAULT_URDF, help="HEI Robot URDF path.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="PNG output directory.")
    parser.add_argument("--width", type=int, default=640, help="Rendered image width.")
    parser.add_argument("--height", type=int, default=480, help="Rendered image height.")
    parser.add_argument("--plain-scene", action="store_true", help="Render the robot without the debug environment.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.width <= 0 or args.height <= 0:
        raise ValueError("Render width and height must be positive.")

    model_path = args.model.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    model = build_mujoco_model(model_path, add_environment=not args.plain_scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Renderer dimensions may exceed the URDF compiler's default offscreen buffer.
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), args.width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), args.height)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    try:
        for camera_spec in ROBOT_CAMERAS:
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_spec.name)
            if camera_id < 0:
                raise ValueError(f"MuJoCo model is missing named camera: {camera_spec.name}")

            renderer.update_scene(data, camera=camera_spec.name)
            rgb = renderer.render()
            output_path = output_dir / f"{camera_spec.name}.png"
            if not cv2.imwrite(str(output_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError(f"Failed to save camera image: {output_path}")
            print(
                f"[HEI camera preview] {camera_spec.name}: "
                f"{args.width}x{args.height}, fovy={model.cam_fovy[camera_id]:.2f} deg -> {output_path}"
            )
    finally:
        renderer.close()


if __name__ == "__main__":
    main()
