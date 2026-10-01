#!/usr/bin/env python

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path

from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility
from lerobot.datasets import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.robots.hei_rebot_lift import HeiRebotLiftClient, HeiRebotLiftClientConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

from vr_control import VRActionReceiver

NUM_EPISODES = 5
FPS = 30
EPISODE_TIME_SEC = 120
RESET_TIME_SEC = 30
TASK_DESCRIPTION = "Pick up the yellow block from the floor and put it on the table in front"
# TASK_DESCRIPTION = "My task description"
HF_REPO_ID = "HGM/hei_rebot_lift"
REMOTE_IP = "192.168.31.127"
ROBOT_ID = "hei_rebot_lift"
PROGRESS_INTERVAL_SEC = 5.0
PROJECT_ROOT = Path(__file__).resolve().parents[4]
REAL_DATA_ROOT = PROJECT_ROOT / "outputs" / "data"


@dataclass(frozen=True)
class TaskAssistProfile:
    """Low-risk assistance settings for a repeatable collection workflow."""

    description: str
    active_arm: str
    phases: tuple[str, ...]


TASK_ASSIST_PROFILES = {
    "stack_blocks_two": TaskAssistProfile(
        description="Move the green block to the red block and stack it.",
        active_arm="right",
        phases=(
            "将绿色方块移到抓取位置",
            "抓住绿色方块并抬起",
            "移动到红色方块上方并放下",
            "松开夹爪并确认叠放成功",
        ),
    ),
    "adjust_bottle": TaskAssistProfile(
        description="Pick up the fallen bottle, make it upright, and release it.",
        active_arm="right",
        phases=(
            "接近倒地瓶子的瓶身",
            "夹住瓶身并抬起",
            "旋转瓶子至竖直并移动到底座位置",
            "放下瓶子并确认站稳",
        ),
    ),
    "grab_roller": TaskAssistProfile(
        description="Use both arms to grasp and lift the wooden roller.",
        active_arm="both",
        phases=(
            "双手分别对准木棍两端",
            "同时夹住木棍",
            "双手同步抬起",
            "保持片刻后放下并确认成功",
        ),
    ),
}


class TaskAssistController:
    """Apply task-specific constraints while leaving pose control to VR.

    The first version intentionally does not infer object geometry. It reduces
    operator load by holding unused arms and disabling mobile axes, while the
    operator advances semantic task phases with ``N``.
    """

    def __init__(self, task_name: str, active_arm: str | None = None):
        self.task_name = task_name
        self.profile = TASK_ASSIST_PROFILES.get(task_name)
        self.active_arm = active_arm or (self.profile.active_arm if self.profile else "both")
        if self.active_arm not in ("left", "right", "both"):
            raise ValueError("active_arm must be left, right, or both")
        self.phase_index = 0

    @property
    def enabled(self) -> bool:
        return self.profile is not None

    @property
    def phases(self) -> tuple[str, ...]:
        return self.profile.phases if self.profile else ("VR continuous control",)

    @property
    def phase_text(self) -> str:
        return f"{self.phase_index + 1}/{len(self.phases)}: {self.phases[self.phase_index]}"

    def reset_episode(self) -> None:
        self.phase_index = 0

    def advance_phase(self) -> bool:
        if self.phase_index >= len(self.phases) - 1:
            return False
        self.phase_index += 1
        return True

    def apply(self, action: dict, observation: dict) -> dict:
        """Mask non-task controls and hold the inactive arm at its measured pose."""
        if not self.enabled:
            return action
        assisted = dict(action)
        for key in ("x.vel", "y.vel", "theta.vel"):
            assisted[key] = 0.0
        assisted["height.pos"] = float(observation.get("height.pos", 0.0))
        for side in ("left", "right"):
            if self.active_arm == "both" or side == self.active_arm:
                continue
            for joint in ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"):
                key = f"{side}_{joint}.pos"
                if key in observation:
                    assisted[key] = float(observation[key])
        return assisted


def validate_real_data_root(root):
    """Keep real recordings in this project's data tree, separate from ACT simulation."""
    target = Path(root).expanduser().resolve()
    if not target.is_relative_to(REAL_DATA_ROOT.resolve()):
        raise ValueError(f"Real recordings must be stored under {REAL_DATA_ROOT}; got {target}")
    return str(target)


def print_status(message: str) -> None:
    print(f"[HEI Record] {message}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="Record a HEI ReBot Lift dataset from VR actions.")
    parser.add_argument(
        "--task",
        choices=("custom", *TASK_ASSIST_PROFILES.keys()),
        default="custom",
        help="Task assistance profile. Profiles hold unused arms, lock base/lift, and show phase prompts.",
    )
    parser.add_argument(
        "--active-arm",
        choices=("left", "right", "both"),
        default=None,
        help="Override the profile arm selection; default is right for stack/bottle and both for roller.",
    )
    parser.add_argument("--num-episodes", type=int, default=NUM_EPISODES, help="Number of new episodes.")
    parser.add_argument("--episode-time-sec", type=float, default=EPISODE_TIME_SEC, help="Seconds per episode.")
    parser.add_argument(
        "--reset-time-sec",
        type=float,
        default=RESET_TIME_SEC,
        help="Deprecated compatibility option; manual Space-trigger mode controls reset duration.",
    )
    parser.add_argument("--task-description", type=str, default=None, help="Task text per frame.")
    parser.add_argument("--repo-id", type=str, default=HF_REPO_ID, help="Hugging Face dataset repo id.")
    parser.add_argument("--root", type=str, default=str(REAL_DATA_ROOT / "hei_vr_real"), help="Local real dataset root under this project's outputs/data.")
    parser.add_argument("--resume", action="store_true", help="Continue recording into an existing dataset.")
    parser.add_argument("--remote-ip", type=str, default=REMOTE_IP, help="HEI ReBot Lift host IP address.")
    parser.add_argument("--robot-id", type=str, default=ROBOT_ID, help="HEI ReBot Lift robot id.")
    parser.add_argument("--image-writer-threads", type=int, default=4, help="Image writer threads per camera.")
    parser.add_argument(
        "--progress-interval-sec",
        type=float,
        default=PROGRESS_INTERVAL_SEC,
        help="Seconds between recording progress logs.",
    )
    # 默认只保存到本地，避免实机录制结束后因为网络/代理问题影响数据落盘。
    parser.add_argument("--push-to-hub", dest="push_to_hub", action="store_true", default=False)
    parser.add_argument("--no-push-to-hub", dest="push_to_hub", action="store_false")
    parser.add_argument("--private", action="store_true", help="Push dataset as private.")
    parser.add_argument("--no-rerun", action="store_true", help="Disable Rerun visualization.")
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
        help="Swap left_wrist/right_wrist observation images before display and dataset storage.",
    )
    return parser.parse_args()


def complete_action_for_dataset(action, dataset):
    action_names = dataset.features[ACTION]["names"]
    return {name: float(action.get(name, 0.0)) for name in action_names}


def maybe_swap_wrist_cameras(observation, enabled: bool):
    if not enabled or "left_wrist" not in observation or "right_wrist" not in observation:
        return observation
    corrected = dict(observation)
    corrected["left_wrist"], corrected["right_wrist"] = (
        observation["right_wrist"],
        observation["left_wrist"],
    )
    return corrected


class TerminalRecordingListener:
    """Read recording controls directly from a Linux/WSL terminal."""

    def __init__(self, events: dict):
        if not sys.stdin.isatty():
            raise RuntimeError(
                "record.py needs an interactive terminal for Space/R/Esc controls; "
                "do not run it with redirected stdin."
            )
        self.events = events
        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="hei-record-terminal-input", daemon=True)
        tty.setcbreak(self.fd)
        self.thread.start()

    def _run(self) -> None:
        try:
            while not self.stop_event.is_set():
                readable, _, _ = select.select([self.fd], [], [], 0.1)
                if not readable:
                    continue
                key = os.read(self.fd, 1)
                phase = self.events.get("phase", "waiting")

                if key == b" ":
                    if phase == "waiting":
                        self.events["start_recording"] = True
                        print_status("Space pressed: start current episode")
                    elif phase == "recording":
                        self.events["exit_early"] = True
                        print_status("Space pressed: finish and save current episode")
                    else:
                        print_status(f"Space ignored while phase={phase}")
                elif key in (b"r", b"R"):
                    if phase == "recording":
                        self.events["rerecord_episode"] = True
                        self.events["exit_early"] = True
                        print_status("R pressed: discard current episode")
                    else:
                        print_status("R ignored: no episode is currently recording")
                elif key in (b"n", b"N"):
                    if phase == "recording":
                        self.events["advance_phase"] = True
                        print_status("N pressed: advance task phase")
                    else:
                        print_status("N ignored: no episode is currently recording")
                elif key == b"\x1b":
                    # Arrow/function keys also begin with ESC. Consume their suffix and ignore them;
                    # a standalone Esc has no suffix and stops the complete recording session.
                    readable, _, _ = select.select([self.fd], [], [], 0.05)
                    if readable:
                        suffix = os.read(self.fd, 8)
                        if suffix.startswith(b"["):
                            print_status("Arrow keys are not used; Space=start/finish, N=next phase, R=discard, Esc=stop")
                            continue
                    self.events["stop_recording"] = True
                    self.events["exit_early"] = True
                    print_status("Esc pressed: stop recording session")
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)


def init_terminal_recording_listener():
    events = {
        "phase": "waiting",
        "start_recording": False,
        "exit_early": False,
        "rerecord_episode": False,
        "advance_phase": False,
        "stop_recording": False,
    }
    return TerminalRecordingListener(events), events


def wait_for_manual_start(
    robot,
    events,
    fps,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
    vr_receiver,
    task_assist=None,
    display_data=False,
    swap_wrist_cameras=False,
    progress_interval_s=PROGRESS_INTERVAL_SEC,
):
    """Keep VR control/feedback active without adding dataset frames until Space is pressed."""

    events["phase"] = "waiting"
    events["start_recording"] = False
    events["exit_early"] = False
    events["rerecord_episode"] = False
    events["advance_phase"] = False
    last_progress_t = time.perf_counter()
    print_status("READY: control/reset the robot now; press Space to start recording")

    while not events["start_recording"] and not events["stop_recording"]:
        start_loop_t = time.perf_counter()
        obs = maybe_swap_wrist_cameras(robot.get_observation(), swap_wrist_cameras)
        action = vr_receiver.get_action(obs)
        if action:
            action_values = teleop_action_processor((action, obs))
            if task_assist is not None:
                action_values = task_assist.apply(action_values, obs)
            robot_action_to_send = robot_action_processor((action_values, obs))
            robot.send_action(robot_action_to_send)
            if display_data:
                obs_processed = robot_observation_processor(obs)
                log_rerun_data(observation=obs_processed, action=action_values)

        now = time.perf_counter()
        if progress_interval_s > 0 and now - last_progress_t >= progress_interval_s:
            print_status("READY: not recording; press Space to start, R discards only during recording, Esc stops")
            last_progress_t = now
        precise_sleep(max(1.0 / fps - (time.perf_counter() - start_loop_t), 0.0))

    if events["start_recording"]:
        events["start_recording"] = False
        return True
    return False


def record_vr_loop(
    robot,
    events,
    fps,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
    vr_receiver,
    task_assist=None,
    dataset=None,
    control_time_s=None,
    single_task=None,
    display_data=False,
    swap_wrist_cameras=False,
    phase_name="record",
    progress_interval_s=PROGRESS_INTERVAL_SEC,
):
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    timestamp = 0
    saved_frames = 0
    start_episode_t = time.perf_counter()
    last_progress_t = start_episode_t
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        obs = maybe_swap_wrist_cameras(robot.get_observation(), swap_wrist_cameras)
        obs_processed = robot_observation_processor(obs)
        observation_frame = (
            build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR) if dataset is not None else None
        )

        # 升降目标需要基于机器人真实 height.pos 计算，所以这里必须把 obs 传给 VRActionReceiver。
        action = vr_receiver.get_action(obs)
        if not action:
            precise_sleep(max(1.0 / fps - (time.perf_counter() - start_loop_t), 0.0))
            timestamp = time.perf_counter() - start_episode_t
            continue

        action_values = teleop_action_processor((action, obs))
        if task_assist is not None:
            if events.get("advance_phase"):
                events["advance_phase"] = False
                if task_assist.advance_phase():
                    print_status(f"Task phase: {task_assist.phase_text}")
                else:
                    print_status("Task phase: already at final phase")
            action_values = task_assist.apply(action_values, obs)
        robot_action_to_send = robot_action_processor((action_values, obs))
        _ = robot.send_action(robot_action_to_send)

        if dataset is not None:
            dataset_action = complete_action_for_dataset(action_values, dataset)
            action_frame = build_dataset_frame(dataset.features, dataset_action, prefix=ACTION)
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)
            saved_frames += 1

        if display_data:
            log_rerun_data(observation=obs_processed, action=action_values)

        now = time.perf_counter()
        if progress_interval_s > 0 and now - last_progress_t >= progress_interval_s:
            # 实机录制时最怕“不知道程序在干嘛”，这里定期打印本阶段进度。
            print_status(
                f"{phase_name}: {timestamp:.1f}/{control_time_s:.1f}s, "
                f"saved_frames={saved_frames if dataset is not None else 0}"
            )
            last_progress_t = now

        precise_sleep(max(1.0 / fps - (time.perf_counter() - start_loop_t), 0.0))
        timestamp = time.perf_counter() - start_episode_t

    print_status(
        f"{phase_name} finished: {min(timestamp, control_time_s):.1f}/{control_time_s:.1f}s, "
        f"saved_frames={saved_frames if dataset is not None else 0}"
    )
    return saved_frames


def main():
    args = parse_args()
    args.root = validate_real_data_root(args.root)
    task_assist = TaskAssistController(args.task, args.active_arm)
    if args.task != "custom" and args.task_description is None:
        args.task_description = task_assist.profile.description
    if args.task_description is None:
        args.task_description = TASK_DESCRIPTION
    print_status(
        f"Starting with task={args.task}, active_arm={task_assist.active_arm}, repo_id={args.repo_id}, episodes={args.num_episodes}, "
        f"episode_time={args.episode_time_sec}s, manual_space_trigger=True, "
        f"remote_ip={args.remote_ip}, robot_id={args.robot_id}, "
        f"push_to_hub={args.push_to_hub}, swap_wrist_cameras={args.swap_wrist_cameras}"
    )

    robot_config = HeiRebotLiftClientConfig(remote_ip=args.remote_ip, id=args.robot_id)
    robot = HeiRebotLiftClient(robot_config)
    vr_receiver = VRActionReceiver()

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    if args.resume:
        if args.root is None:
            raise ValueError("--resume requires --root with the latest LeRobot dataset writer.")
        print_status(f"Resuming local dataset at root={args.root}")
        dataset = LeRobotDataset.resume(
            args.repo_id,
            root=args.root,
            image_writer_threads=args.image_writer_threads * len(robot.config.cameras),
        )
        sanity_check_dataset_robot_compatibility(dataset, robot, FPS, dataset_features)
    else:
        print_status(f"Creating local dataset; root={args.root or 'default HF_LEROBOT_HOME/repo_id'}")
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=FPS,
            root=args.root,
            features=dataset_features,
            robot_type=robot.name,
            use_videos=True,
            image_writer_threads=args.image_writer_threads * len(robot.config.cameras),
        )
    print_status(f"Dataset ready at {dataset.root}")

    print_status("Starting VR ZMQ receiver")
    vr_receiver.start()
    print_status(f"Connecting robot client to {args.remote_ip}")
    robot.connect()
    print_status("Robot connected")

    listener, events = init_terminal_recording_listener()
    if not args.no_rerun:
        init_rerun(
            session_name="hei_rebot_lift_record",
            ip=args.rerun_ip,
            port=args.rerun_port if args.rerun_ip else None,
        )

    try:
        if not robot.is_connected:
            raise ValueError("Robot is not connected!")

        first_obs = robot.get_observation()
        first_obs = maybe_swap_wrist_cameras(first_obs, args.swap_wrist_cameras)
        vr_receiver.set_height_from_observation(first_obs)
        print_status(f"Initial lift height: {float(first_obs.get('height.pos', 0.0)):.1f} mm")

        print_status("Waiting for first VR arm action from MuJoCo/VR")
        vr_receiver.wait_for_arm_action()
        print_status("VR arm action received")
        print_status("Controls: Space=start/finish and save, N=next task phase, R=discard current episode, Esc=stop all")
        recorded_episodes = 0
        while recorded_episodes < args.num_episodes and not events["stop_recording"]:
            should_start = wait_for_manual_start(
                robot=robot,
                events=events,
                fps=FPS,
                vr_receiver=vr_receiver,
                display_data=not args.no_rerun,
                swap_wrist_cameras=args.swap_wrist_cameras,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                task_assist=task_assist,
                progress_interval_s=args.progress_interval_sec,
            )
            if not should_start:
                break

            episode_index = dataset.num_episodes
            events["phase"] = "recording"
            task_assist.reset_episode()
            events["exit_early"] = False
            events["rerecord_episode"] = False
            log_say(f"Recording episode {episode_index}")
            print_status(
                f"RECORDING episode {recorded_episodes + 1}/{args.num_episodes} "
                f"(dataset episode {episode_index}); press Space to save early or R to discard"
            )
            if task_assist.enabled:
                print_status(f"Task phase: {task_assist.phase_text}; press N after each phase")

            saved_frames = record_vr_loop(
                robot=robot,
                events=events,
                fps=FPS,
                dataset=dataset,
                vr_receiver=vr_receiver,
                control_time_s=args.episode_time_sec,
                single_task=args.task_description,
                display_data=not args.no_rerun,
                swap_wrist_cameras=args.swap_wrist_cameras,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                task_assist=task_assist,
                phase_name=f"episode {recorded_episodes + 1}/{args.num_episodes}",
                progress_interval_s=args.progress_interval_sec,
            )
            print_status(f"Episode {recorded_episodes + 1}/{args.num_episodes} control loop done, frames={saved_frames}")

            events["phase"] = "processing"

            if events["stop_recording"]:
                print_status(f"Stopping session; discarding unfinished dataset episode {episode_index}")
                dataset.clear_episode_buffer()
                break

            if events["rerecord_episode"]:
                print_status(f"Discarding episode buffer for dataset episode {episode_index}")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                print_status("Episode discarded; reset the scene and press Space when ready to try again")
                continue

            if saved_frames <= 0:
                # 没有收到可保存的 VR action 时，LeRobot 不允许保存空 episode。
                # 这里跳过本集，避免整次录制因为一次空包/误触提前退出而中断。
                print_status(
                    f"Episode {episode_index} has no saved frames; skipped. "
                    "Check MuJoCo/VR ZMQ if this happens repeatedly."
                )
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            print_status(f"Saving episode {episode_index} to disk (images/video/data)")
            dataset.save_episode()
            print_status(f"Episode {episode_index} saved")
            recorded_episodes += 1
            if recorded_episodes < args.num_episodes:
                print_status("Reset/rearrange the scene now; the next episode will not start until Space is pressed")
    finally:
        log_say("Stop recording")
        print_status("Stopping VR receiver")
        vr_receiver.stop()
        if robot.is_connected:
            print_status("Disconnecting robot")
            robot.disconnect()
            print_status("Robot disconnected")
        print_status("Stopping keyboard listener")
        listener.stop()

        print_status("Finalizing dataset (waiting image writer/video encoder/metadata)")
        dataset.finalize()
        print_status(f"Dataset finalized at {dataset.root}")
        if args.push_to_hub:
            print_status("Pushing dataset to Hugging Face Hub")
            dataset.push_to_hub(private=args.private)
            print_status("Push to Hub finished")
        else:
            print_status("Push to Hub skipped")


if __name__ == "__main__":
    main()
