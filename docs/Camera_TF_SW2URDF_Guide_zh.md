# HEI ReBot Lift 相机 TF 与 SW2URDF 建模指南

本文用于在 SolidWorks / SW2URDF 中为 HEI ReBot Lift 增加三台相机的坐标系：

- `front`：机器人前视相机，建议使用 Intel RealSense D435。
- `left_wrist`：左机械臂腕部相机。
- `right_wrist`：右机械臂腕部相机。

当前阶段只建立相机的机械安装关系和 TF。相机内参、图像采集以及 MuJoCo 渲染相机将在 URDF 导出并验证后接入程序。

## 1. 推荐的 TF 树

建议保留现有机械臂的 link 和 joint 名称，只新增以下相机节点：

```text
lift_carriage_link
└── front_camera_link
    └── front_camera_optical_frame

a_right_link6
└── right_wrist_camera_link
    └── right_wrist_camera_optical_frame

b_left_link6
└── left_wrist_camera_link
    └── left_wrist_camera_optical_frame
```

推荐父链接如下：

| 相机 | 父链接 | 原因 |
| --- | --- | --- |
| 前视相机 | `lift_carriage_link` | 相机随升降平台一起上下运动 |
| 右腕相机 | `a_right_link6` | 相机随右腕运动，但不随夹爪手指开合 |
| 左腕相机 | `b_left_link6` | 相机随左腕运动，但不随夹爪手指开合 |

如果前视相机实际固定在底盘而不是升降平台，应将它的父链接改成对应的底盘固定 link。不要仅根据外观看起来的位置选择父链接，应根据相机实际随哪个部件运动来确定。

## 2. 坐标系如何建立

### 2.1 相机机械坐标系

每台相机先建立一个 `camera_link`，原点放在 RGB 镜头的光心位置，而不是外壳中心或安装孔中心。

建议采用以下右手坐标系：

- `+X`：从镜头向外，指向相机正在观察的方向。
- `+Y`：相机左侧。
- `+Z`：相机上方。

从相机后方向镜头观察方向看，`+Y` 指向左，`+Z` 指向上。可用右手定则检查：`X × Y = Z`。

### 2.2 标准光学坐标系

视觉算法和 ROS 通常使用 optical frame：

- `+Z`：镜头朝向，即图像前方。
- `+X`：图像右侧。
- `+Y`：图像下方。

因此，每个 `camera_link` 下再建立一个同原点的 `camera_optical_frame`。从上述机械坐标系到 optical frame 的固定旋转为：

```xml
<origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
```

注意：MuJoCo 相机默认沿局部 `-Z` 方向观察。不要为了适配 MuJoCo 而改变 URDF 的标准相机坐标系；后续应在 MuJoCo 场景或转换代码中处理该差异。

### 2.3 当前 `HEI_robot_urdf_2` 的实际坐标约定

本次 SolidWorks 导出的三个 `camera_link` 使用另一套同样合法的右手坐标系：

- `+Z`：镜头前方。
- `+X`：画面左侧。
- `+Y`：画面上方。

因此，合入项目时采用以下旋转建立标准 optical frame：

```xml
<origin xyz="0 0 0" rpy="0 0 3.14159265358979"/>
```

不要把第 2.2 节适用于“`+X` 朝镜头”的旋转直接套到本次导出的模型上。后续若重新建立相机坐标系，应先确认镜头方向对应 camera link 的哪根轴，再选择转换关系。

## 3. SolidWorks 中的操作建议

1. 在机器人总装中加入简化的相机零件或装配体。
2. 按真实安装支架和方向完成配合，确保相机跟随正确的父部件。
3. 在 RGB 镜头光心创建坐标系。
4. 令坐标系 `+X` 穿过镜头朝外，`+Z` 指向画面上方，再按右手定则确定 `+Y`。
5. 在 SW2URDF 中将相机作为独立 child link，通过 `fixed` joint 连接到父链接。
6. 相机外壳网格可以相对 `camera_link` 偏移，但 `camera_link` 原点仍应位于镜头光心。
7. 如果 SW2URDF 不方便创建没有网格的 optical frame，可先导出机械相机 link，再手动向 URDF 添加 optical frame。

三个相机的安装 joint 都必须使用 `fixed`，不要使用 revolute、continuous 或 prismatic。

## 4. 名称规范

建议使用以下名称，不使用空格或中文：

| 部位 | Link | Joint | Optical frame | Optical joint |
| --- | --- | --- | --- | --- |
| 前视 | `front_camera_link` | `front_camera_joint` | `front_camera_optical_frame` | `front_camera_optical_joint` |
| 左腕 | `left_wrist_camera_link` | `left_wrist_camera_joint` | `left_wrist_camera_optical_frame` | `left_wrist_camera_optical_joint` |
| 右腕 | `right_wrist_camera_link` | `right_wrist_camera_joint` | `right_wrist_camera_optical_frame` | `right_wrist_camera_optical_joint` |

这些名称与项目中的数据键 `front`、`left_wrist`、`right_wrist` 一一对应，后续接入仿真采集时更容易核对。

## 5. URDF 模板

前视相机示例：

```xml
<link name="front_camera_link"/>

<joint name="front_camera_joint" type="fixed">
  <!-- X Y Z 和 R P Y 由 SolidWorks 实际装配位置决定 -->
  <origin xyz="X Y Z" rpy="R P Y"/>
  <parent link="lift_carriage_link"/>
  <child link="front_camera_link"/>
</joint>

<link name="front_camera_optical_frame"/>

<joint name="front_camera_optical_joint" type="fixed">
  <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  <parent link="front_camera_link"/>
  <child link="front_camera_optical_frame"/>
</joint>
```

左腕相机把父链接改为 `b_left_link6`，右腕相机把父链接改为 `a_right_link6`，并使用第 4 节规定的名称。

腕部相机不要挂在左右手指 link 上，否则夹爪开合时相机也会错误移动。

## 6. D435 的坐标系

第一阶段建议把 `front_camera_link` 的原点放在 D435 RGB 彩色相机的光心，以它作为 `front` 图像的 TF 来源。

后续如果使用 RGB-D 点云，可以继续增加：

```text
front_camera_link
├── front_color_frame
│   └── front_color_optical_frame
└── front_depth_frame
    └── front_depth_optical_frame
```

彩色与深度光心之间的平移和旋转应读取 D435 的出厂标定外参，不要根据外壳尺寸估算。如果深度图已经通过 RealSense SDK 对齐到彩色图，数据采集可以继续以彩色 optical frame 为参考。

## 7. 单位换算

SW2URDF 导出前后重点检查单位：

- SolidWorks 常用毫米，URDF 的 `xyz` 使用米：毫米数值除以 `1000`。
- SolidWorks 常用角度，URDF 的 `rpy` 使用弧度：角度乘以 `π / 180`。
- `rpy` 的顺序固定为 roll、pitch、yaw，不要交换顺序。

建议记录原始测量值和换算值，避免后续无法追溯：

| 相机 | xyz（mm） | rpy（deg） | xyz（m） | rpy（rad） |
| --- | --- | --- | --- | --- |
| front |  |  |  |  |
| left_wrist |  |  |  |  |
| right_wrist |  |  |  |  |

## 8. 建模信息记录表

完成模型时请填写以下内容。之后把导出的 URDF 和这张表交给程序侧，即可继续制作 MuJoCo 三相机渲染、仿真数据采集和训练链路。

```text
前视相机 front
  型号：D435 / 其他：
  父链接：lift_carriage_link / 其他：
  原点：RGB 光心 / 其他：
  相对父链接 xyz（mm）：
  相对父链接 rpy（deg）：
  镜头朝向对应父坐标系：+X / -X / +Y / -Y / +Z / -Z
  画面上方对应父坐标系：+X / -X / +Y / -Y / +Z / -Z
  采集内容：RGB / RGB-D

左腕相机 left_wrist
  型号：
  父链接：b_left_link6
  原点：RGB 光心
  相对父链接 xyz（mm）：
  相对父链接 rpy（deg）：
  镜头朝向对应父坐标系：
  画面上方对应父坐标系：

右腕相机 right_wrist
  型号：
  父链接：a_right_link6
  原点：RGB 光心
  相对父链接 xyz（mm）：
  相对父链接 rpy（deg）：
  镜头朝向对应父坐标系：
  画面上方对应父坐标系：
```

如果不方便准确填写 `rpy`，只需给出相机相对父链接的 `xyz`、镜头朝向和画面上方方向，程序侧也可以据此计算旋转。

## 9. 导出与检查清单

建议先导出为新文件，例如 `HEI_robot_urdf_with_cameras.urdf`，验证通过后再替换正式模型。

- 保留现有机械臂 link 和 joint 名称，避免破坏 IK 和控制程序。
- 三个安装 joint 均为 `fixed`。
- `front_camera_link` 会随升降平台移动。
- 左右腕相机分别跟随对应的 `link6`，不会随夹爪手指开合。
- 镜头朝向、画面上方和左右方向符合第 2 节约定。
- 所有 link 和 joint 名称唯一。
- 相机 TF link 通常不需要 collision；若添加外壳 collision，应使用简化几何体。
- 若相机 link 添加惯性参数，应使用合理质量和惯量，不能全部为零。
- 使用 `check_urdf` 检查树结构，再使用 MuJoCo 编译并分别预览三路相机画面。

URDF 中的 camera frame 负责描述安装位姿；MuJoCo 的具名 `<camera>`、分辨率、视场角和离屏渲染仍需要在仿真场景或加载程序中配置。完成新 URDF 后，再进行这部分接入。
