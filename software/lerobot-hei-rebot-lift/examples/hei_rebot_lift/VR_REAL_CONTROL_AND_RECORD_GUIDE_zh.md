# HEI ReBot Lift：VR 真机控制、三相机预览与数据录制完整流程

本文适用于当前设备和目录：

- 机器人 Jetson IP：`10.163.141.128`
- Windows/WSL IP：`10.163.141.53`
- 电脑项目：`/mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift`
- 机器人项目：`/home/jetson/hei-rebot-lift-main/software/lerobot-hei-rebot-lift`
- WSL Conda：`/mnt/e/DevSoft/miniconda3-wsl`
- WSL VR 环境：`hei-rebot-vr`
- WSL 客户端环境：`hei-rebot-client`
- Windows Rerun 环境：`E:\DevSoft\anaconda\envs\lerobot-mujoco`

## 1. 系统组成

```text
Quest 手柄
  -> Telegrip (8443/8442 -> 5567)
  -> MuJoCo IK 真机桥
  -> 动作端口 6558
  -> teleoperate.py 或 record.py
  -> 机器人 host (6555)

机器人电机状态和三路相机
  -> 机器人 host (6556)
  -> teleoperate.py 或 record.py
  -> 状态反馈 6559 -> MuJoCo IK 真机桥
  -> Windows Rerun 9876（仅显示）
```

`teleoperate.py` 和 `record.py` 二选一，严禁同时运行。它们都会控制机器人并绑定反馈端口 `6559`。

## 2. 启动前安全检查

1. 清空机器人工作范围，第一次测试不要拿物体。
2. 急停按钮放在操作者随手可按的位置。
3. 启动机器人 host、执行升降归零时，需要按设备要求释放急停。
4. VR 两侧 grip 先全部松开，摇杆回中。
5. 同一时间只允许一个 `6558` 动作发布器和一个机器人客户端。

## 3. 第一个终端：机器人 Jetson host

必须先进入仓库根目录。不要在 `examples/.../VR_mujoco_ik` 中设置 `PYTHONPATH=src`，因为那里没有 `src` 目录。

```bash
cd /home/jetson/hei-rebot-lift-main/software/lerobot-hei-rebot-lift
export PYTHONPATH="$PWD/src"
conda run --no-capture-output -n lerobot hei-rebot-lift-host
```

也可以写成一条命令：

```bash
cd /home/jetson/hei-rebot-lift-main/software/lerobot-hei-rebot-lift
PYTHONPATH="$PWD/src" conda run --no-capture-output -n lerobot hei-rebot-lift-host
```

截图中的错误：

```text
ModuleNotFoundError: No module named 'lerobot'
```

就是因为在错误目录使用了相对的 `PYTHONPATH=src`。正确执行上述两条命令即可，不需要重建机器人 Conda 环境。

host 正常后保持终端运行。它负责电机、升降归零、三路相机以及端口 `6555/6556`。

电脑 Windows PowerShell 可检查：

```powershell
Test-NetConnection 10.163.141.128 -Port 6555
Test-NetConnection 10.163.141.128 -Port 6556
```

两项的 `TcpTestSucceeded` 都应为 `True`。

## 4. Windows：启动原生 Rerun Viewer

WSL 内的 Mesa 驱动不支持 Rerun 需要的 `R32Float`，因此 Viewer 在 Windows 原生运行，机器人客户端仍留在 WSL。

在 Windows PowerShell 执行：

```powershell
E:\DevSoft\anaconda\envs\lerobot-mujoco\Scripts\rerun.exe `
  --bind 0.0.0.0 `
  --port 9876 `
  --memory-limit 4GB
```

保持 Rerun 窗口打开。检查监听端口：

```powershell
Get-NetTCPConnection -LocalPort 9876 -State Listen
```

## 5. 第二个 WSL 终端：启动 Telegrip

```bash
source /mnt/e/DevSoft/miniconda3-wsl/etc/profile.d/conda.sh
conda activate hei-rebot-vr

cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift/examples/hei_rebot_lift/VR_mujoco_ik
sed -i 's/\r$//' run_telegrip.sh run_hei_robot_vr_real.sh
chmod +x run_telegrip.sh run_hei_robot_vr_real.sh
./run_telegrip.sh
```

Quest 浏览器打开：

```text
https://10.163.141.53:8443
```

接受证书提示并点击 `Start VR`。终端应出现：

```text
VR client connected
VR button ... pressed/released
```

Telegrip 负责端口 `8443`、`8442` 和 `5567`。

## 6. 第三个 WSL 终端：启动控制客户端

先启动客户端，让真机桥能够从 `6559` 获取真实机器人状态。

```bash
source /mnt/e/DevSoft/miniconda3-wsl/etc/profile.d/conda.sh
conda activate hei-rebot-client

cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift
export PYTHONPATH="$PWD/src"

python -u examples/hei_rebot_lift/teleoperate.py \
  --remote-ip 10.163.141.128 \
  --rerun-ip 127.0.0.1 \
  --rerun-port 9876
```

正常日志：

```text
[HEI VR] robot-state feedback publishing on tcp://*:6559
[HEI Teleoperate] Connecting to robot host=10.163.141.128
Starting HEI ReBot Lift VR teleop loop at 30 fps
Waiting for VR arm qpos messages on tcp://localhost:6558...
```

此时出现 `Waiting` 是正常的，因为第四个终端还没有发布 `6558` 动作。

如果只需要控制、不需要画面，可改用：

```bash
python -u examples/hei_rebot_lift/teleoperate.py \
  --remote-ip 10.163.141.128 \
  --no-rerun
```

## 7. 第四个 WSL 终端：启动 MuJoCo IK 真机桥

```bash
source /mnt/e/DevSoft/miniconda3-wsl/etc/profile.d/conda.sh
conda activate hei-rebot-vr

cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift/examples/hei_rebot_lift/VR_mujoco_ik
./run_hei_robot_vr_real.sh --enable-real-publish
```

两侧 grip 松开后，应看到：

```text
[HEI VR Real] command bridge ARMED; real pose synchronized
[HEI VR Real] VR=online ... bridge=armed feedback=0.xx ...
```

随后第三个终端的 `Waiting for VR arm qpos` 会停止。

状态解释：

- `VR=online`：Quest 数据正常到达。
- `bridge=armed`：真机动作发布已解锁。
- `feedback=0.xx`：真实机器人反馈新鲜。
- `bridge=locked`：VR 或机器人反馈中断；恢复后松开两侧 grip 重新解锁。

不要使用 `--allow-no-feedback` 控制真机。

## 8. 首次控制方法

1. 日志必须已经显示 `bridge=armed`。
2. 只按住一侧 grip，缓慢移动对应手柄 1～2 cm。
3. grip 按住期间机械臂跟随；松开 grip 后机械臂保持。
4. grip 按住期间，trigger 按下为夹爪张开，松开 trigger 为夹爪闭合。
5. 左 grip + 左摇杆上下控制升降。
6. 右 grip + 右摇杆控制底盘平移；旋转按钮按项目映射使用。
7. 第一次不要测试底盘和升降；先只测试一条机械臂。

异常运动时立即松开 grip，必要时按物理急停。

## 9. 在 Rerun 查看三路相机

Windows Rerun 收到 `teleoperate.py` 数据后，查找以下实体：

```text
observation.images.front
observation.images.left_wrist
observation.images.right_wrist
```

分别对应前置、左腕和右腕相机。若状态有数据但缺少某一路图像，检查机器人 host 的相机日志。反复出现 `Corrupt JPEG data` 说明对应 MJPG 帧存在损坏，应在正式录制前处理相机 USB、线缆、供电或采集参数问题。

### Rerun 是否会保存过程

Rerun 默认实时显示数据，但只保存在 Viewer 内存中。需要保存可回看的 Rerun 记录时，在 Windows PowerShell 启动 Viewer 时增加 `--save`：

```powershell
New-Item -ItemType Directory -Force E:\code_product\act_hei_robot\outputs\rerun | Out-Null

E:\DevSoft\anaconda\envs\lerobot-mujoco\Scripts\rerun.exe `
  --bind 0.0.0.0 `
  --port 9876 `
  --memory-limit 4GB `
  --save E:\code_product\act_hei_robot\outputs\rerun\hei_teleop_test.rrd
```

回放保存的 `.rrd` 文件：

```powershell
E:\DevSoft\anaconda\envs\lerobot-mujoco\Scripts\rerun.exe `
  E:\code_product\act_hei_robot\outputs\rerun\hei_teleop_test.rrd
```

`.rrd` 适合检查相机、动作和状态时间线，不能直接替代 ACT 训练所需的 LeRobotDataset。训练数据必须由 `record.py` 保存。

## 10. 从遥操作切换到数据录制

必须先关闭 `teleoperate.py`，但保留以下程序：

- 机器人 Jetson host：保留；
- Telegrip：保留；
- Windows Rerun：保留；
- MuJoCo IK 真机桥：保留；
- `teleoperate.py`：关闭；
- `record.py`：随后启动。

在第三个终端按 `Ctrl+C`。如果卡住，在另一个 WSL 终端只结束指定程序：

```bash
pkill -f 'examples/hei_rebot_lift/teleoperate.py'
```

确认已经退出：

```bash
pgrep -af 'teleoperate.py'
```

然后启动录制。下面示例录制 5 条，每条最多 120 秒，保存后等待手动复位和空格开始：

```bash
source /mnt/e/DevSoft/miniconda3-wsl/etc/profile.d/conda.sh
conda activate hei-rebot-client

cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift
export PYTHONPATH="$PWD/src"

python -u examples/hei_rebot_lift/record.py \
  --remote-ip 10.163.141.128 \
  --root /mnt/e/code_product/act_hei_robot/outputs/data/hei_vr_real \
  --repo-id local/hei_vr_real \
  --num-episodes 5 \
  --episode-time-sec 120 \
  --task-description "用双臂完成指定抓取任务" \
  --swap-wrist-cameras \
  --rerun-ip 127.0.0.1 \
  --rerun-port 9876 \
  --no-push-to-hub
```

### 三个任务的简化采集模式（无新增硬件）

`record.py` 现在提供三个任务辅助档案。档案不会自动猜测物体位置，而是利用现有 VR 控制器完成以下安全约束：

- 叠方块、扶瓶子：默认只让右臂跟随 VR，左臂保持当前姿态；底盘和升降锁定。
- 抓棍子：允许双臂跟随 VR，底盘和升降仍锁定，避免操作员误碰移动底盘。
- 每个任务显示 4 个语义阶段；完成一个阶段后按 `N` 进入下一个阶段。`Space`、`R`、`Esc` 含义不变。

叠方块：

```bash
python -u examples/hei_rebot_lift/record.py \
  --task stack_blocks_two \
  --remote-ip 10.163.141.128 \
  --root /mnt/e/code_product/act_hei_robot/outputs/data/stack_blocks_two \
  --repo-id local/stack_blocks_two \
  --num-episodes 5 --episode-time-sec 90 --no-push-to-hub
```

扶瓶子把 `--task` 换成 `adjust_bottle`，双手抓棍子把它换成 `grab_roller`。如果物体在另一侧，可增加 `--active-arm left`；抓棍子使用 `--active-arm both`（任务默认就是双臂）。如果需要恢复旧的完全连续 VR 行为，使用 `--task custom`。

建议操作顺序：先在 `READY` 状态摆好物体并用 VR 调整机器人初始姿态，按 `Space` 开始；按终端提示完成每一阶段并按 `N`；成功后按 `Space` 保存。发现失败按 `R` 丢弃，重新摆放后再录制。首次真机测试仍需低速、空载并保留急停监护。

录制键盘控制：

- 等待状态按一次 `Space`：开始当前 episode。
- 录制状态再按一次 `Space`：提前结束并保存当前 episode。
- 录制状态按 `N`：进入任务辅助档案的下一阶段。
- 录制状态按 `R`：丢弃当前 episode，回到等待状态；复位后再按 `Space` 重录。
- `Esc`：停止全部录制，丢弃尚未完成的当前 episode，并完成数据集收尾。

每条保存后不会自动开始下一条。程序会进入 `READY` 状态，此时可以自由控制机器人和重新摆放物体，但不会写入训练数据；准备好后再按 `Space` 开始下一条。`--episode-time-sec` 仍是单条数据的最长时间，超过后会自动保存。

继续向同一个本地数据集追加时，保持相同 `--root` 和 `--repo-id`，并增加：

```text
--resume
```

录制时如果不需要 Rerun，可以用 `--no-rerun` 代替 `--rerun-ip/--rerun-port`。

如果检查画面确认左右腕相机字段反了，增加 `--swap-wrist-cameras`。该参数会在显示和写入数据集前交换 `left_wrist` 与 `right_wrist`，不影响前置相机和机器人动作。永久修复应在机器人 Jetson 上重新运行相机绑定向导，正确分配 `/dev/hei_left_wrist_camera` 与 `/dev/hei_right_wrist_camera`。

录制期间，Rerun 显示当前过程；LeRobotDataset 同时把每帧动作、状态和三路相机图像写入 `--root`。录制结束后，终端出现 `Dataset finalized at ...` 才表示数据集完成写盘。

检查数据集是否已产生文件：

```bash
find /mnt/e/code_product/act_hei_robot/outputs/data/hei_vr_real -maxdepth 2 -type f | head -30
```

不要只根据 Rerun 窗口判断录制成功，应确认每个 episode 出现 `saved`，并最终出现 `Dataset finalized`。

## 11. 正确停止顺序

1. 松开两个 grip，摇杆回中。
2. 停止 `teleoperate.py` 或 `record.py`。
3. 停止 MuJoCo IK 真机桥。
4. 停止 Telegrip。
5. 最后停止机器人 host。
6. 按设备规程处理急停和机器人电源。

## 12. 常见故障快速检查

### `No module named lerobot`

进入仓库根目录再设置绝对 `PYTHONPATH`：

```bash
cd /home/jetson/hei-rebot-lift-main/software/lerobot-hei-rebot-lift
export PYTHONPATH="$PWD/src"
```

电脑 WSL 对应路径：

```bash
cd /mnt/e/code_product/act_hei_robot/software/lerobot-hei-rebot-lift
export PYTHONPATH="$PWD/src"
```

### `bash\r: No such file or directory`

```bash
sed -i 's/\r$//' run_telegrip.sh run_hei_robot_vr_real.sh
chmod +x run_telegrip.sh run_hei_robot_vr_real.sh
```

### 一直等待 `6558`

```bash
ss -ltnp | grep ':6558'
```

没有输出表示真机桥没有运行。重新启动第四个终端。

### 真机桥一直 `feedback=waiting`

确认 `teleoperate.py` 或 `record.py` 正在运行，并检查：

```bash
ss -ltnp | grep ':6559'
```

### 真机桥一直 `VR=waiting`

确认 Quest 已点击 `Start VR`，并检查 Telegrip 是否持续收到手柄事件。

### Rerun 报 `R32Float`

不要在 WSL 内生成 Viewer。先在 Windows 启动 Rerun，再给客户端传：

```text
--rerun-ip 127.0.0.1 --rerun-port 9876
```

### 查看当前程序和端口

```bash
ps -ef | grep -E 'telegrip|teleoperate.py|record.py|hei_robot_vr_mujoco_real' | grep -v grep
ss -ltnp | grep -E ':(8442|8443|5567|6558|6559)\b'
```

不要使用 `pkill -f python`，它会误杀其他控制程序。
