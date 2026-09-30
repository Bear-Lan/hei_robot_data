# VR MuJoCo 仿真数据采集流程

这份流程只用于“Quest/Telegrip → MuJoCo 仿真 → LeRobotDataset”训练数据，**不要运行实机
`record.py`**。采集器已经接在 `mujoco_ik/hei_robot_vr_mujoco_sim.py` 的同一个仿真循环中，
因此保存的观测和 VR 控制后的仿真状态是同步的。

## 1. 环境和目录

在 WSL2 Ubuntu 中执行。先确认环境：

```bash
conda activate hei-rebot-vr
cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift/examples/hei_rebot_lift/VR_mujoco_ik
python -c "import mujoco, zmq, pinocchio; print(mujoco.__version__)"
python -c "from lerobot.common.datasets.lerobot_dataset import LeRobotDataset; print('LeRobotDataset OK')"
```

如果第二条失败，需在当前环境安装与项目训练环境相同版本的 `lerobot`；不要把实机
相机/机器人 SDK 当成仿真采集依赖。

如果 `hei-rebot-vr` 因 Python 3.10 无法安装当前 LeRobot，程序仍可采集：它会自动把每条
轨迹保存为 `raw_episodes/episode_*.npz`。采集完成后，在 ACT/训练环境执行文末的转换命令。

## 2. 启动 VR 和仿真

终端 A：

```bash
cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift/examples/hei_rebot_lift/VR_mujoco_ik
sed -i 's/\r$//' run_telegrip.sh run_hei_robot_vr_sim.sh
conda activate hei-rebot-vr
./run_telegrip.sh
```

在 Quest 浏览器打开 Telegrip 页面，点击 `Start VR`，确认 Telegrip 终端出现
`VR client connected` 和控制器数据日志。

终端 B：

```bash
cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift/examples/hei_rebot_lift/VR_mujoco_ik
conda activate hei-rebot-vr
env -u LD_LIBRARY_PATH python -u mujoco_ik/hei_robot_vr_mujoco_sim.py \
  --record \
  --record-root /mnt/e/code_product/ACT/outputs/data/hei_vr_sim \
  --record-repo-id local/hei_vr_sim \
  --record-task "VR control HEI ReBot in MuJoCo" \
  --record-episodes 20 \
  --record-fps 20
```

`--record-root` 可以改成训练机器上的任意目录。默认每帧保存：双臂 12 个关节、双夹爪、
升降、底盘 x/y/yaw，共 17 维 `observation.state`，一张 MuJoCo `front` 相机图像，以及
下一帧状态作为 `action`。当前没有真实相机，也不会向真实机器人发布指令。
如果目录中已经有 raw episode，程序会自动从现有最大编号之后继续保存，不会覆盖旧文件；
`--record-episodes 37` 表示本次新录制 37 条。

## 3. 每条 episode 的操作

先在窗口里用 VR 做一次完整示范，准备好后点击 MuJoCo 窗口：

- `Space`：开始录制；再次按 `Space`：结束并保存当前 episode，保存后程序会自动复位仿真环境。
- `Backspace`：丢弃当前 episode，不写入数据集。
- `R`：手动复位机器人和场景，并丢弃未完成录制（保存后通常不需要再按）。
- `F`：显示/隐藏 MuJoCo 坐标系。
- `Esc`：关闭窗口并退出。

建议每个任务先录 3 条短轨迹确认动作方向和图像，再录目标数量。不要在 VR 断流、仿真窗口
没有更新或夹爪状态明显异常时保存；这种 episode 用 `Backspace` 丢弃。

## 4. 检查数据

```bash
conda activate hei-rebot-vr
python - <<'PY'
from pathlib import Path
import json
root = Path('/mnt/e/code_product/ACT/outputs/data/hei_vr_sim')
info = json.loads((root / 'meta' / 'info.json').read_text())
print('episodes =', info['total_episodes'])
print('fps =', info['fps'])
print('root =', root)
PY
```

至少确认 `total_episodes > 0`，并且每条轨迹都有 `observation.state`、
`observation.images.front`、`action` 和 `task`。采集完成后再复制到 ACT 训练环境，或直接
在已安装训练依赖的环境中运行训练命令。

如果目录里是 `raw_episodes` 而不是 `meta/info.json`，先转换：

```bash
cd /mnt/e/code_product/ACT
conda activate lerobot-mujoco
python -m act_exp.convert_vr_dataset \
  --input-root /mnt/e/code_product/ACT/outputs/data/hei_vr_sim \
  --output-root /mnt/e/code_product/ACT/outputs/data/hei_vr_sim \
  --repo-id local/hei_vr_sim
```

## 5. 与 ACT 训练的关系

该采集器输出的是标准 LeRobotDataset，不需要再运行实机 `record.py`。如果训练脚本要求
数据集名，将 `repo_id`/数据根目录改成脚本使用的名字；例如：

```bash
conda activate lerobot-mujoco
python -m act_exp.train \
  --task hei_vr_sim \
  --seed 0 \
  --steps 20000 \
  --output-dir /mnt/e/code_product/act_hei_robot/outputs/train/hei_vr_sim_seed0
```

如果现有训练脚本没有 `hei_vr_sim` 任务配置，需要先把任务名、数据根目录和 17 维状态配置
加入训练入口；不要把它伪装成 14 维 ALOHA 数据。视觉 ACT 训练使用 `front` 图像，纯状态
策略验证则只使用 `observation.state`。

## 6. 常见问题

- **窗口显示 VR waiting**：先确认 Telegrip 在终端 A 运行、Quest 已点击 `Start VR`，并且
  仿真使用 `tcp://localhost:5567`；如果 Telegrip 在另一台主机，改 `--vr-endpoint`。
- **`LeRobotDataset` 导入失败**：当前 `hei-rebot-vr` 只有 VR/IK 依赖，安装项目的 lerobot
  数据集依赖，或改用已安装 `lerobot` 的训练环境运行仿真程序。
- **数据目录已有旧数据**：程序会追加到已有数据集；要重新开始，请先备份后删除该任务的
  `meta/data` 目录，再重新运行。不要删除整个项目目录。
- **图像黑屏/全黑**：确认使用了本版本 `hei_robot_mujoco_scene.py`（已加入 `front` 相机和
  场景灯光），并按文档用 `env -u LD_LIBRARY_PATH` 启动。
