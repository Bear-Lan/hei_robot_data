#!/usr/bin/env python

import argparse
import time

from lerobot.robots.hei_rebot_lift import HeiRebotLiftClient, HeiRebotLiftClientConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from vr_control import VRActionReceiver

FPS = 30
REMOTE_IP = "192.168.31.130"
ROBOT_ID = "hei_rebot_lift"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate HEI ReBot Lift with VR controllers.")
    parser.add_argument("--remote-ip", type=str, default=REMOTE_IP, help="HEI ReBot Lift host IP address.")
    parser.add_argument("--robot-id", type=str, default=ROBOT_ID, help="Robot identifier.")
    parser.add_argument("--fps", type=int, default=FPS, help="Teleoperation control frequency.")
    parser.add_argument(
        "--no-rerun",
        action="store_true",
        help="Disable the Rerun viewer and logging (useful on WSL systems without a compatible GPU).",
    )
    parser.add_argument(
        "--rerun-ip",
        type=str,
        default=None,
        help="Connect to an existing Rerun Viewer instead of spawning one locally.",
    )
    parser.add_argument("--rerun-port", type=int, default=9876, help="Existing Rerun Viewer's gRPC port.")
    parser.add_argument(
        "--swap-wrist-cameras",
        action="store_true",
        help="Swap left_wrist/right_wrist observation images before Rerun display.",
    )
    return parser.parse_args()


def maybe_swap_wrist_cameras(observation, enabled: bool):
    if not enabled or "left_wrist" not in observation or "right_wrist" not in observation:
        return observation
    corrected = dict(observation)
    corrected["left_wrist"], corrected["right_wrist"] = (
        observation["right_wrist"],
        observation["left_wrist"],
    )
    return corrected


def main():
    args = parse_args()
    robot_config = HeiRebotLiftClientConfig(remote_ip=args.remote_ip, id=args.robot_id)
    robot = HeiRebotLiftClient(robot_config)
    vr_receiver = VRActionReceiver()
    vr_receiver.start()

    print(f"[HEI Teleoperate] Connecting to robot host={args.remote_ip}, robot_id={args.robot_id}")
    robot.connect()
    if not args.no_rerun:
        init_rerun(
            session_name="hei_rebot_lift_teleop",
            ip=args.rerun_ip,
            port=args.rerun_port if args.rerun_ip else None,
        )

    try:
        if not robot.is_connected:
            raise ValueError("Robot is not connected!")

        observation = maybe_swap_wrist_cameras(robot.get_observation(), args.swap_wrist_cameras)
        vr_receiver.set_height_from_observation(observation)

        print(f"Starting HEI ReBot Lift VR teleop loop at {args.fps} fps")
        vr_receiver.wait_for_arm_action()
        while True:
            t0 = time.perf_counter()

            observation = maybe_swap_wrist_cameras(robot.get_observation(), args.swap_wrist_cameras)
            action = vr_receiver.get_action(observation)

            if action:
                action_sent = robot.send_action(action)
                if not args.no_rerun:
                    log_rerun_data(observation=observation, action=action_sent)

            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    finally:
        vr_receiver.stop()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
