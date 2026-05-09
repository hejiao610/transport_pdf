
# SensorsCalibration / OpenCalib 详细技术文档

本文档基于当前目录 `E:\WorkFile\SensorsCalibration-master` 下的源码、README、CMake 配置和模块入口整理，面向工程使用、算法理解、运行部署和问题排查。项目原始名称为 **SensorsCalibration toolbox**，论文和部分模块中也称 **OpenCalib**。

> 说明：该工程大量模块采用独立子目录编译方式，且 README 示例多为 Linux/Docker 命令。Windows 本地环境建议通过 Docker、WSL 或 Linux 主机运行；若直接在 Windows 上运行，需要自行处理 OpenCV、PCL、Pangolin、Ceres、Boost、jsoncpp 等依赖。

---

## 1. 项目总体定位

SensorsCalibration 是面向自动驾驶场景的多传感器标定工具箱，覆盖：

- 相机内参标定。
- 相机畸变评估。
- IMU/GNSS heading 标定。
- LiDAR 与 IMU、Camera、LiDAR 之间的外参标定。
- Radar 与 Camera、LiDAR、车体系之间的外参标定。
- 环视相机标定。
- 工厂产线标定。
- Sensor-to-Car 标定。
- 在线标定。

项目围绕自动驾驶常见传感器组合展开：

| 传感器 | 在工程中的典型角色 |
|---|---|
| Camera | 提供图像、车道线、消失点、地平线、标定板角点 |
| LiDAR | 提供三维点云、强度、地面、线/面特征 |
| IMU/GNSS / Pose sensor | 提供姿态、速度、轨迹、heading |
| Radar | 提供目标点、径向速度、静态目标观测 |

工程中每类标定一般都在求一个刚体变换：

```text
T_ab = [ R_ab  t_ab ]
       [  0      1  ]
```

其中：

- `R_ab`：从坐标系 b 到坐标系 a 的旋转。
- `t_ab`：从坐标系 b 到坐标系 a 的平移。
- `T_ab`：齐次变换矩阵。

实际使用时必须特别注意变换方向。例如 `lidar-to-camera`、`camera-to-car`、`gnss-to-lidar` 等命名有时来自数据文件名，有时来自代码变量名，使用前应通过投影效果或矩阵定义确认方向。

---

## 2. 顶层目录说明

| 顶层目录 | 功能 |
|---|---|
| `camera_intrinsic` | 相机内参标定与畸变评估 |
| `imu_heading` | IMU/GNSS heading 标定与评估 |
| `lidar2imu` | LiDAR 到 IMU / pose sensor 外参标定，包含手动和自动 |
| `lidar2camera` | LiDAR 到 Camera 外参标定，包含手动、自动、SAM、联合内外参 |
| `lidar2lidar` | 多 LiDAR 之间外参标定，包含手动和自动 |
| `surround-camera` | 环视相机标定，包含手动、无靶标、鱼眼、靶标 |
| `radar2camera` | Radar 到 Camera 手动外参标定 |
| `radar2lidar` | Radar 到 LiDAR 手动外参标定 |
| `factory_calib` | 工厂产线标定工具，支持多类标定板 |
| `SensorX2car` | Camera/LiDAR/Radar/PoseSensor 到车体系标定 |
| `online_calib` | 在线标定：camera2imu、lidar2imu、radar2carcenter |
| `cmake_script` | CMake 相关脚本 |
| `run_docker.sh` | Docker 启动脚本 |

---

## 3. 环境准备

### 3.1 推荐 Docker 环境

项目顶层 README 推荐：

```bash
sudo docker pull scllovewkf/opencalib:v1
docker run -it -v /home/sz3/ailab/:/share scllovewkf/opencalib:v1 /bin/bash
```

或：

```bash
sudo ./run_docker.sh
```

部分 SensorX2car 模块 README 中使用另一个镜像：

```bash
docker pull xiaokyan/opencalib:v1
```

建议优先使用项目 README 中当前模块推荐的镜像，因为不同模块依赖版本可能不同。

### 3.2 常见 C++ 依赖

| 依赖 | 常见用途 |
|---|---|
| CMake | 构建工程 |
| OpenCV | 图像读写、角点检测、PnP、Homography、畸变矫正、显示 |
| Eigen3 | 矩阵运算、旋转表示、优化变量 |
| PCL | PCD 读取、点云滤波、KDTree、ICP |
| Pangolin | 手动标定 GUI |
| Boost | 文件、系统、程序选项等 |
| jsoncpp | 读取内外参 JSON |
| Ceres | 非线性优化 |
| g2o | 部分优化模块依赖 |
| matplotlib-cpp | 部分 SensorX2car 模块绘图 |
| splinter | 部分轨迹/曲线拟合 |

### 3.3 常见 Python 依赖

| 模块 | Python 依赖 |
|---|---|
| `lidar2camera\auto_calib\tool` | TensorFlow 1.15、OpenCV、pydensecrf |
| `lidar2camera\auto_calib_v2.0` | Segment Anything、PyTorch、OpenCV、pycocotools、matplotlib |
| `SensorX2car\camera2car\auto_calib` | PyTorch、torchvision、CTRL-C 相关依赖 |
| `factory_calib\tool\factory_solution` | Python GUI、绘图、几何计算依赖 |

### 3.4 通用编译方式

大多数 C++ 子模块采用如下模式：

```bash
cd <module_or_submodule>
mkdir -p build && cd build
cmake ..
make
```

生成的可执行文件通常在当前子模块的 `bin` 目录下，例如：

```bash
./bin/run_lidar2camera
./bin/run_lidar2imu
./bin/run_AVM_Calibration
```

注意：不要默认在仓库根目录一次性编译全部模块。该仓库大量子模块独立维护，依赖和 CMake 配置并不统一。

---

## 4. 通用数据格式与结果检查

### 4.1 外参 JSON

很多模块使用 JSON 文件保存初始外参，最终输出通常也给出 4x4 形式：

```text
[r11, r12, r13, tx],
[r21, r22, r23, ty],
[r31, r32, r33, tz],
[0,   0,   0,   1 ]
```

或者文本格式：

```text
Extrinsic:
R:
r11 r12 r13
r21 r22 r23
r31 r32 r33
t: tx ty tz
```

### 4.2 相机内参 JSON

常见形式：

```text
Intrinsic:
[fx, 0,  cx],
[0,  fy, cy],
[0,  0,  1 ]

Distortion:
[k1, k2, p1, p2]
```

### 4.3 PCD 点云

LiDAR 相关模块一般使用 PCL 读取 `.pcd` 文件。部分算法需要：

- `x, y, z`。
- intensity / reflectivity。
- 连续帧时间顺序。
- 多 LiDAR 同步帧。

### 4.4 Radar CSV

Radar 模块支持不同雷达类型，代码中对 CSV 列有固定假设。例如：

- Conti 类型通常直接读取目标坐标。
- Delphi 类型可能使用 `rho`、`phi` 转为笛卡尔坐标。

如果 CSV 列顺序与代码不一致，会导致点云解析错误，标定结果无意义。

### 4.5 结果检查方法

| 模块 | 建议检查方式 |
|---|---|
| Camera 内参 | 去畸变后直线是否变直、重投影误差是否合理 |
| LiDAR-Camera | 点云是否准确压到图像中车道线、路沿、杆、车辆边缘 |
| LiDAR-IMU | 使用标定外参拼接点云，墙面/地面/车道线是否清晰重合 |
| LiDAR-LiDAR | 多雷达点云拼接后重叠区域是否无双影 |
| Radar-Camera | 鸟瞰图中 Radar 点是否沿道路/车道线方向分布 |
| Radar-LiDAR | Radar 目标是否落在 LiDAR 中对应车辆/路沿/静态物体位置 |
| SensorX2car | 输出轨迹、yaw 对比图是否平滑且偏差小 |
| 环视 | 拼接缝是否连续，车道线/路沿是否不断裂 |

---

## 5. `camera_intrinsic`：相机内参标定与畸变评估

路径：

```text
camera_intrinsic
```

### 5.1 子模块结构

```text
camera_intrinsic
├─ intrinsic_calib
│  ├─ include
│  └─ src
├─ calib_verification
│  ├─ include
│  └─ src
├─ data
└─ images
```

| 子模块 | 功能 |
|---|---|
| `intrinsic_calib` | 使用棋盘格图像求相机内参和畸变 |
| `calib_verification` | 使用 calibration harp 图像评估畸变 |

### 5.2 编译流程

#### 5.2.1 内参标定

```bash
cd camera_intrinsic/intrinsic_calib
mkdir -p build && cd build
cmake ..
make
```

#### 5.2.2 畸变评估

```bash
cd camera_intrinsic/calib_verification
mkdir -p build && cd build
cmake ..
make
```

### 5.3 内参标定运行流程

命令：

```bash
./bin/run_intrinsic_calibration <calibration_image_dir>
```

示例：

```bash
cd camera_intrinsic/intrinsic_calib
./bin/run_intrinsic_calibration ./data/
```

输入目录要求：

1. 目录中只放参与标定的棋盘格图片。
2. 图片分辨率一致。
3. 棋盘格清晰可见。
4. 图片数量建议不少于 15 张。
5. 棋盘格应覆盖图像中心、四角、边缘区域，姿态应有变化。

输出：

1. 相机内参矩阵 `K`。
2. 畸变参数 `D`。
3. 每张输入图像的去畸变图。
4. 去畸变图保存到：

```text
<calibration_image_dir>/undistort/
```

### 5.4 内参标定算法原理

该模块采用标准张正友棋盘格标定流程，可概括为：

#### 第一步：建立世界坐标点

棋盘格在自身平面上，通常令 `Z=0`：

```text
P_ij = [ i * square_size, j * square_size, 0 ]
```

其中：

- `i, j` 为棋盘角点索引。
- `square_size` 为棋盘格实际尺寸。

#### 第二步：检测图像角点

对每张图像执行：

```text
findChessboardCorners(image)
cornerSubPix(image, corners)
```

得到图像平面上的亚像素角点：

```text
p_ij = [ u_ij, v_ij ]
```

#### 第三步：相机投影模型

理想针孔模型：

```text
s * [u, v, 1]^T = K * [R | t] * [X, Y, Z, 1]^T
```

其中：

```text
K = [ fx  0  cx ]
    [  0 fy  cy ]
    [  0  0   1 ]
```

#### 第四步：畸变模型

OpenCV 常用径向 + 切向畸变：

```text
x' = x / z
y' = y / z
r2 = x'^2 + y'^2

x_dist = x' * (1 + k1*r2 + k2*r2^2 + k3*r2^3)
         + 2*p1*x'*y' + p2*(r2 + 2*x'^2)

y_dist = y' * (1 + k1*r2 + k2*r2^2 + k3*r2^3)
         + p1*(r2 + 2*y'^2) + 2*p2*x'*y'
```

然后：

```text
u = fx * x_dist + cx
v = fy * y_dist + cy
```

#### 第五步：最小化重投影误差

目标函数：

```text
min_{K,D,R_i,t_i} Σ_i Σ_j || p_ij - project(K, D, R_i, t_i, P_j) ||^2
```

其中：

- `i`：第 i 张图。
- `j`：第 j 个棋盘格角点。
- `R_i, t_i`：第 i 张图对应的棋盘格到相机外参。

#### 第六步：去畸变

标定后调用映射表完成批量去畸变：

```text
initUndistortRectifyMap(...)
remap(...)
```

### 5.5 畸变评估运行流程

命令：

```bash
./bin/run_distortion_measure <distortion_image_path>
```

示例：

```bash
cd camera_intrinsic/calib_verification
./bin/run_distortion_measure data/test.png
```

输入：

- calibration harp 图像，要求有大量清晰直线。

输出：

- 原图采样点显示。
- 去畸变后结果图。
- 畸变误差统计。

### 5.6 畸变评估算法原理

该部分基于 calibration harp 论文思想：真实世界中的直线经相机成像后，如果内参/畸变模型准确，去畸变图中的线应保持直线。

流程：

1. 使用 LSD 检测图像中的线段。
2. 使用 Canny 提取边缘点。
3. 将边缘点归属到对应线段附近。
4. 对线段上的点做插值和重采样。
5. 对点集做平滑/降采样，降低噪声。
6. 对每条线做直线拟合。
7. 计算点到拟合直线的距离。
8. 统计平均畸变误差和最大畸变误差。

直线残差：

```text
line: ax + by + c = 0
d_i = |a*x_i + b*y_i + c| / sqrt(a^2 + b^2)
```

总体指标：

```text
d_mean = mean(d_i)
d_max  = max(d_i)
```

### 5.7 常见问题

| 问题 | 原因 | 解决建议 |
|---|---|---|
| 角点检测失败 | 模糊、反光、棋盘太小、遮挡 | 重新采集清晰棋盘图 |
| 标定结果不稳定 | 图片太少或角度单一 | 增加不同视角图片 |
| 去畸变图拉伸严重 | 棋盘尺寸或角点数配置错误 | 检查棋盘规格 |
| 边缘畸变仍明显 | 边缘区域样本不足 | 采集棋盘覆盖图像边缘 |
| 畸变评估误差大 | harp 图像直线不清晰或内参错误 | 重新拍摄或重新标定内参 |

---

## 6. `imu_heading`：IMU / GNSS heading 标定

路径：

```text
imu_heading
```

### 6.1 功能说明

该模块用于标定 IMU heading 与车辆实际行驶方向之间的 yaw 偏差。车辆在道路上直行采集 GNSS/IMU 数据，利用 GPS 轨迹方向和 IMU heading 的差异计算 heading offset。

### 6.2 编译流程

```bash
cd imu_heading/auto_calib
mkdir -p build && cd build
cmake ..
make
```

### 6.3 运行流程

命令：

```bash
./bin/run_imu_heading method_id <data_dir>
```

输入：

- `method_id`：方法编号。
- `<data_dir>`：IMU/GNSS 数据目录，通常包含 `novatel-utm.csv`。

输出：

- IMU heading offset。
- 速度投影验证结果。
- heading 对比信息。

### 6.4 数据采集流程

推荐采集方式：

1. 选择开阔、直线道路。
2. 保证 GNSS 信号稳定。
3. 车辆沿直线行驶一段距离。
4. 速度不要过低，避免轨迹方向噪声过大。
5. 记录 IMU heading、GNSS 位置、时间戳。

### 6.5 算法原理

车辆在直线运动时，车体 x 轴方向与轨迹切线方向一致。设连续两个 GNSS 位置为：

```text
P_i     = [x_i,     y_i]
P_{i+1} = [x_{i+1}, y_{i+1}]
```

轨迹方向：

```text
yaw_traj_i = atan2(y_{i+1} - y_i, x_{i+1} - x_i)
```

IMU 输出 heading：

```text
yaw_imu_i
```

待估计偏差：

```text
offset_i = normalize_angle(yaw_traj_i - yaw_imu_i)
```

多帧融合：

```text
offset = robust_mean(offset_i)
```

或通过最小二乘拟合：

```text
min_offset Σ || normalize_angle(yaw_traj_i - yaw_imu_i - offset) ||^2
```

### 6.6 结果验证

标定后，可将 IMU heading 修正为：

```text
yaw_corrected = yaw_imu + offset
```

再与轨迹方向比较。如果直线段上两者一致，说明标定有效。

### 6.7 注意事项

- 直线采集比任意路线更稳定。
- GNSS 漂移会影响轨迹方向。
- 低速或停车数据应剔除。
- 转弯段会使 heading 与短时轨迹方向关系复杂。
- heading 角度要统一弧度/角度单位。
- 角度差必须做归一化，避免 `π` 和 `-π` 跳变。

---

## 7. `lidar2imu`：LiDAR 到 IMU / Pose Sensor 标定

路径：

```text
lidar2imu
```

### 7.1 子模块

| 子目录 | 功能 |
|---|---|
| `manual_calib` | 手动调整 LiDAR-IMU 外参 |
| `auto_calib` | 自动优化 LiDAR-IMU 外参 |

### 7.2 手动标定操作流程

#### 7.2.1 编译

```bash
cd lidar2imu/manual_calib
mkdir -p build && cd build
cmake ..
make
```

#### 7.2.2 运行

命令格式：

```bash
./bin/run_lidar2imu <lidar_pcds_dir> <lidar_pose_file> <extrinsic_json>
```

示例：

```bash
./bin/run_lidar2imu data/top_center_lidar data/top_center_lidar-pose.txt data/gnss-to-top_center_lidar-extrinsic.json
```

输入说明：

| 参数 | 说明 |
|---|---|
| `lidar_pcds_dir` | LiDAR PCD 序列目录 |
| `lidar_pose_file` | IMU/GNSS pose 文件 |
| `extrinsic_json` | 初始外参 |

#### 7.2.3 GUI 操作

窗口左侧为控制面板，右侧为点云显示区域。

操作目标：

1. 根据当前外参将 LiDAR 序列用 pose 拼接。
2. 观察拼接后的墙、地面、车道线、杆是否重合。
3. 使用按钮或键盘调整旋转和平移。
4. 调整到点云序列无明显重影。
5. 点击 `Save Result` 保存结果。

常用按键：

| 参数 | 增加 | 减少 |
|---|---|---|
| x 轴旋转 | `q` | `a` |
| y 轴旋转 | `w` | `s` |
| z 轴旋转 | `e` | `d` |
| x 平移 | `r` | `f` |
| y 平移 | `t` | `g` |
| z 平移 | `y` | `h` |

### 7.3 自动标定操作流程

#### 7.3.1 编译

```bash
cd lidar2imu/auto_calib
mkdir -p build && cd build
cmake ..
make
```

#### 7.3.2 运行

```bash
./bin/run_lidar2imu data/top_center_lidar/ data/NovAtel-pose-lidar-time.txt data/gnss-to-top_center_lidar-extrinsic.json
```

输入：

| 参数 | 说明 |
|---|---|
| LiDAR PCD 目录 | 连续点云帧 |
| pose 文件 | NovAtel/IMU/GNSS 位姿，且与 LiDAR 时间对齐 |
| 初始外参 JSON | GNSS/IMU 到 LiDAR 或 LiDAR 到 IMU 初始估计 |

输出：

- refined 外参文本。
- 点云拼接验证结果。

### 7.4 自动标定算法原理

目标：找到一个外参 `T_imu_lidar`，使得使用 IMU/GNSS 位姿拼接后的 LiDAR 点云在世界坐标系中最一致。

#### 7.4.1 坐标关系

设：

- `P_lidar`：LiDAR 坐标系中的点。
- `T_world_imu_i`：第 i 帧 IMU 在世界坐标系下位姿。
- `T_imu_lidar`：待标定外参。

则第 i 帧点云转到世界坐标系：

```text
P_world_i = T_world_imu_i * T_imu_lidar * P_lidar_i
```

如果 `T_imu_lidar` 正确，多帧点云会拼成清晰地图；如果错误，会出现墙面重影、地面分层、杆状物发散。

#### 7.4.2 特征提取

代码中自动流程会构建 LiDAR 特征，常见包括：

- 平面特征：地面、墙面。
- 边缘特征：杆、边、角。
- 体素地图特征：局部点云分布。

#### 7.4.3 优化变量

通常使用 6DoF 增量：

```text
δ = [δroll, δpitch, δyaw, δtx, δty, δtz]
```

将初始外参更新为：

```text
T_new = exp(δ^) * T_initial
```

#### 7.4.4 残差设计

平面约束示例：

```text
r_plane = n^T * (P_world - P_plane)
```

其中：

- `n`：局部平面法向。
- `P_plane`：平面上一点。

线约束示例：

```text
r_line = distance(P_world, line)
```

总体优化：

```text
min_δ Σ r_plane^2 + Σ r_line^2
```

#### 7.4.5 Ceres 优化

`auto_calib` 依赖 Ceres，通常采用迭代最小二乘：

1. 用当前外参计算世界点。
2. 查找邻域特征。
3. 构建残差和雅可比。
4. 求解增量。
5. 更新外参。
6. 直到收敛。

### 7.5 数据采集要求

README 中建议：

1. 地面足够平。
2. 周围有足够结构特征，如墙、车道线、杆、静止车辆。
3. 车辆按推荐路线绕行三圈。
4. 速度约 10 km/h。
5. 尽量避免动态物体。

### 7.6 常见问题

| 问题 | 原因 | 建议 |
|---|---|---|
| 优化发散 | 初始外参太差 | 先用手动工具粗调 |
| 地面分层 | roll/pitch 或高度错误 | 检查地面是否平整，检查 pose |
| 墙面双影 | yaw/平移错误或时间不同步 | 检查时间戳，重新优化 |
| 特征不足 | 场景空旷 | 选择墙、杆、车道线丰富区域 |
| 局部地图模糊 | 动态物体多 | 剔除动态场景数据 |

---

## 8. `lidar2camera`：LiDAR 到 Camera 标定

路径：

```text
lidar2camera
```

### 8.1 子模块

| 子目录 | 功能 |
|---|---|
| `manual_calib` | 手动点云投影标定 |
| `auto_calib` | 基于线特征的 CRLF 自动标定 |
| `auto_calib/tool` | 图像线特征/mask 提取 |
| `auto_calib_v2.0` | CalibAnything，基于 Segment Anything |
| `joint_calib` | 相机内参与 LiDAR-Camera 外参联合标定 |
| `joint_calib/tool/LidarCircleDetect` | LiDAR 圆形靶标检测 |

### 8.2 手动标定

#### 8.2.1 编译

```bash
cd lidar2camera/manual_calib
mkdir -p build && cd build
cmake ..
make
```

#### 8.2.2 运行

```bash
./bin/run_lidar2camera <image_path> <pcd_path> <intrinsic_json> <extrinsic_json>
```

示例：

```bash
./bin/run_lidar2camera data/0.png data/0.pcd data/center_camera-intrinsic.json data/top_center_lidar-to-center_camera-extrinsic.json
```

输入：

| 参数 | 说明 |
|---|---|
| `image_path` | 相机图像 |
| `pcd_path` | LiDAR 点云 |
| `intrinsic_json` | 相机内参 |
| `extrinsic_json` | 初始 LiDAR-Camera 外参 |

#### 8.2.3 操作方式

程序将 LiDAR 点投影到图像上：

```text
p_img = project(K, D, T_camera_lidar * P_lidar)
```

用户通过 GUI 微调外参，使点云投影与图像结构对齐。

重点观察：

- 车道线。
- 路沿。
- 杆状物。
- 墙面边缘。
- 车辆轮廓。
- 地面反射线。

支持功能：

- `Intensity Color`：按 LiDAR intensity 显示。
- `Overlap Filter`：去除近深度重叠点。
- `deg step`：旋转步长。
- `t step`：平移步长。
- `fxfy scale`：内参焦距调整步长。
- `point size`：投影点大小。
- `Save Image`：保存结果。

#### 8.2.4 输出

保存内容包括：

- 标定后投影图。
- 外参矩阵。
- 内参矩阵。
- 畸变参数。

### 8.3 CRLF 自动标定

#### 8.3.1 编译

```bash
cd lidar2camera/auto_calib
mkdir -p build && cd build
cmake ..
make
```

#### 8.3.2 运行

```bash
./bin/run_lidar2camera <mask_path> <pcd_path> <intrinsic_json> <extrinsic_json>
```

示例：

```bash
./bin/run_lidar2camera data/mask.jpg data/calib.pcd data/center_camera-intrinsic.json data/top_center_lidar-to-center_camera-extrinsic.json
```

输入：

| 参数 | 说明 |
|---|---|
| `mask_path` | 图像线特征 mask |
| `pcd_path` | LiDAR 点云 |
| `intrinsic_json` | 相机内参 |
| `extrinsic_json` | 初始外参 |

### 8.4 图像 mask 生成

路径：

```text
lidar2camera/auto_calib/tool
```

#### 8.4.1 环境

```bash
conda env create -f lidar2camera.yaml
```

下载 DeepLabV3 Cityscapes 模型：

```bash
wget http://download.tensorflow.org/models/deeplabv3_cityscapes_train_2018_02_06.tar.gz
```

#### 8.4.2 运行

```bash
python merge_mask.py calib.png mask.png
```

该工具的目标是从图像中提取车道线和柱状物等结构 mask，供 C++ 自动标定使用。

### 8.5 CRLF 算法原理

CRLF 的核心思想：道路场景中，Camera 图像和 LiDAR 点云都能观察到某些静态直线结构，例如车道线、杆、灯杆、路沿。若外参正确，LiDAR 中提取的线特征投影到图像后，应与图像线特征重合。

#### 8.5.1 Camera 侧特征

通过语义分割或图像处理得到 mask：

```text
M(u,v) = 1 表示该像素属于线特征
M(u,v) = 0 表示背景
```

为了优化方便，通常可将 mask 转换为距离场：

```text
D(u,v) = distance((u,v), nearest feature pixel)
```

#### 8.5.2 LiDAR 侧特征

从点云中提取线状或边缘特征：

```text
P_lidar_feature = {P_1, P_2, ..., P_N}
```

#### 8.5.3 投影模型

对每个 LiDAR 特征点：

```text
P_camera = T_camera_lidar * P_lidar
```

再投影：

```text
u = fx * X/Z + cx
v = fy * Y/Z + cy
```

带畸变时需先应用畸变模型。

#### 8.5.4 损失函数

如果点投影到图像线特征附近，距离场值小；否则大：

```text
loss(T) = Σ_i D( project(K, D_cam, T * P_i) )
```

也可以使用 mask 命中率：

```text
score(T) = count( M(project(T * P_i)) == 1 )
```

最终求：

```text
T* = argmin loss(T)
```

或：

```text
T* = argmax score(T)
```

#### 8.5.5 优化策略

实际工程中常使用：

1. 初始外参粗投影。
2. 旋转/平移扰动搜索。
3. 计算投影和 mask 的匹配分数。
4. 选择更优外参。
5. 迭代细化。

### 8.6 CalibAnything / `auto_calib_v2.0`

路径：

```text
lidar2camera/auto_calib_v2.0
```

#### 8.6.1 编译

README 中示例：

```bash
mkdir -p build && cd build
cmake ..
make
```

#### 8.6.2 运行示例

```bash
./run_lidar2camera ./data/st/1
./run_lidar2camera ./data/kitti/1
```

运行自定义数据：

```bash
./run_lidar2camera <data_folder>
```

#### 8.6.3 数据目录要求

目录中需要包含：

1. image file。
2. lidar file，且需要 reflectivity。
3. calib file。
4. `masks` 文件夹，存放 Segment Anything 生成的 mask。

calib file 格式示例：

```text
K: 2152.8 0 971.3 0 2155.5 605.9 0 0 1
D: -0.1192 0.162 0.00073985 0.0014
T: 0.0188623 -0.999822 -9.36529e-05 -0.0323222 ...
```

#### 8.6.4 生成 SAM masks

安装 Segment Anything：

```bash
git clone git@github.com:facebookresearch/segment-anything.git
cd segment-anything
pip install -e .
pip install opencv-python pycocotools matplotlib onnxruntime onnx
```

下载模型，例如 `vit_l`：

```bash
wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_l_0b3195.pth
```

生成 mask：

```bash
python scripts/amg.py \
  --checkpoint sam_vit_l_0b3195.pth \
  --model-type vit_l \
  --input 1.png \
  --output ./output/ \
  --stability-score-thresh 0.9 \
  --box-nms-thresh 0.4 \
  --stability-score-offset 0.9
```

#### 8.6.5 输出

| 文件 | 说明 |
|---|---|
| `init_proj.png` | 初始外参投影 |
| `init_proj_seg.png` | 初始投影与分割结果 |
| `error_proj.png` | 加入人为误差后的投影 |
| `error_proj_seg.png` | 误差投影与分割结果 |
| `refined_proj.png` | 优化后投影 |
| `refined_proj_seg.png` | 优化后投影与分割 |
| `extrinsic.txt` | refined 外参 |

### 8.7 CalibAnything 算法原理

CalibAnything 的核心思想是利用 Segment Anything 产生的图像区域边界和 LiDAR 反射强度/几何边界进行对齐。

流程：

1. 给定图像、点云、初始外参。
2. 使用 SAM 生成大量候选 mask。
3. 从 mask 中提取区域边界或分割结构。
4. 从 LiDAR 点云中利用 reflectivity 和几何变化提取可见结构。
5. 将 LiDAR 点投影到图像。
6. 计算投影结构与 mask 结构的一致性。
7. 优化外参。

与 CRLF 相比：

| 项目 | CRLF | CalibAnything |
|---|---|---|
| 图像特征 | 语义线特征 | SAM 通用 mask |
| 是否训练特定模型 | 需要语义分割模型 | 不需要针对标定训练 |
| 数据要求 | 道路线特征明显 | 图像区域结构明显，LiDAR 需要 reflectivity |
| 优点 | 道路直线场景效果好 | 泛化性更强 |
| 风险 | mask 错误影响大 | SAM mask 过多或不稳定会影响优化 |

### 8.8 联合内外参标定 `joint_calib`

#### 8.8.1 编译

```bash
cd lidar2camera/joint_calib
mkdir -p build && cd build
cmake ..
make
```

#### 8.8.2 运行

```bash
./bin/lidar2camera data/intrinsic/ data/circle.csv
```

输入：

| 参数 | 说明 |
|---|---|
| `camera_dir` | 相机标定板图像目录 |
| `csv_file` | 图像对应的圆心点或 LiDAR 检测点 |

#### 8.8.3 LiDAR 圆检测工具

路径：

```text
lidar2camera/joint_calib/tool/LidarCircleDetect
```

编译：

```bash
mkdir -p build && cd build
cmake ..
make
```

运行：

```bash
./bin/run_lidardetection data/
```

作用：从 PCD 文件中提取圆形靶标中心坐标，为联合标定提供 3D 特征点。

### 8.9 联合优化算法原理

联合标定同时估计：

1. Camera intrinsic `K`。
2. Camera distortion `D`。
3. 每张图的标定板外参 `T_camera_board_i`。
4. LiDAR-Camera 外参 `T_camera_lidar`。

观测：

- 图像中的圆心/角点 `p_image`。
- LiDAR 中对应的圆心/靶标点 `P_lidar`。

重投影：

```text
p_pred = project(K, D, T_camera_lidar * P_lidar)
```

目标函数：

```text
min Σ || p_image - p_pred ||^2
```

如果同时使用棋盘格/圆板图像特征，还会有：

```text
min Σ || p_board_image - project(K, D, T_camera_board_i * P_board) ||^2
```

联合优化的好处：

- 相机内参和外参一致性更好。
- 靶标几何约束更强。
- 比单帧手动投影更稳定。

风险：

- 特征对应关系必须正确。
- 初值不能太差。
- 标定板尺寸必须准确。

### 8.10 LiDAR-Camera 常见问题

| 问题 | 原因 | 建议 |
|---|---|---|
| 点云整体偏移 | 外参平移错误 | 手动粗调或修正初始 JSON |
| 点云旋转错位 | roll/pitch/yaw 错误 | 分别观察地面、杆、远处结构 |
| 图像边缘对不上 | 内参/畸变错误 | 重新做 camera_intrinsic |
| 自动优化失败 | mask 不准或线特征不足 | 重新生成 mask，选择道路线特征明显场景 |
| SAM 结果不稳定 | mask 太碎或无关区域多 | 调整 SAM 参数或筛选 mask |
| 联合标定发散 | 2D/3D 对应错误 | 检查 circle.csv 和图像顺序 |

---

## 9. `lidar2lidar`：LiDAR 到 LiDAR 标定

路径：

```text
lidar2lidar
```

### 9.1 子模块

| 子目录 | 功能 |
|---|---|
| `manual_calib` | 手动标定两台 LiDAR |
| `auto_calib` | 自动多 LiDAR 标定 |

### 9.2 手动标定流程

#### 9.2.1 编译

```bash
cd lidar2lidar/manual_calib
mkdir -p build && cd build
cmake ..
make
```

#### 9.2.2 运行

```bash
./bin/run_lidar2lidar <target_pcd_path> <source_pcd_path> <extrinsic_json>
```

示例：

```bash
./bin/run_lidar2lidar data/qt.pcd data/p64.pcd data/p64-to-qt-extrinsic.json
```

输入：

| 参数 | 说明 |
|---|---|
| `target_pcd_path` | 目标 LiDAR 点云 |
| `source_pcd_path` | 源 LiDAR 点云 |
| `extrinsic_json` | source 到 target 的初始外参 |

操作目标：

1. 显示 target 和 source 点云。
2. 用外参把 source 转到 target 坐标系。
3. 手动调节 6DoF。
4. 让两份点云在重叠区域重合。
5. 保存结果。

### 9.3 自动标定流程

#### 9.3.1 编译

```bash
cd lidar2lidar/auto_calib
mkdir -p build && cd build
cmake ..
make
```

#### 9.3.2 运行

```bash
./bin/run_lidar2lidar data/0001/lidar_cloud_path.txt data/0001/initial_extrinsic.txt
```

输入：

| 参数 | 说明 |
|---|---|
| `lidar_cloud_path.txt` | 多 LiDAR 点云路径列表 |
| `initial_extrinsic.txt` | 初始外参 |

输出：

- refined 外参。
- 拼接点云，例如 `stitching.pcd`。

### 9.4 自动标定算法原理

自动 LiDAR-LiDAR 标定基于道路场景多 LiDAR 重叠区域的一致性，可概括为地面对齐 + yaw 搜索 + ICP 精修。

#### 9.4.1 初始对齐

设：

- `P_source`：源 LiDAR 点。
- `P_target`：目标 LiDAR 点。
- `T_target_source`：待优化外参。

初始投影：

```text
P_source_in_target = T_target_source_initial * P_source
```

#### 9.4.2 地面提取

从 target 和 source 中提取地面点，估计地面平面：

```text
n^T x + d = 0
```

其中 `n` 是地面法向。

#### 9.4.3 roll/pitch 对齐

如果两个 LiDAR 的地面法向不一致，则说明相对 roll/pitch 有误。

求旋转 `R_align`，使：

```text
R_align * n_source ≈ n_target
```

#### 9.4.4 yaw 粗搜索

地面法向主要约束 roll/pitch，对 yaw 约束弱。因此需要在 yaw 方向搜索，使非地面结构最匹配。

评价方式：

1. 对 source 非地面点应用候选 yaw。
2. 使用 KDTree 查找 target 最近邻。
3. 计算最近邻距离和。
4. 选取距离最小的 yaw。

目标：

```text
min_yaw Σ_i || nearest(P_target, R_yaw * P_source_i) - R_yaw * P_source_i ||^2
```

#### 9.4.5 ICP 精修

在较好初值基础上使用 ICP：

```text
min_T Σ_i || q_i - T * p_i ||^2
```

其中：

- `p_i`：source 点。
- `q_i`：target 中最近邻点。

带法向 ICP 进一步考虑局部法向：

```text
min_T Σ_i point_to_plane_error(T * p_i, q_i, n_i)
```

点到平面误差：

```text
r_i = n_i^T * (T * p_i - q_i)
```

### 9.5 数据采集建议

1. 多 LiDAR 应同步采集。
2. 选择重叠区域丰富的场景。
3. 场景中应有地面、墙面、路沿、杆、静止车辆。
4. 避免大量动态车辆。
5. 保证初始外参大致正确。

### 9.6 常见问题

| 问题 | 原因 | 建议 |
|---|---|---|
| ICP 收敛到错误位置 | 初始外参太差 | 先手动粗调 |
| roll/pitch 错误 | 地面提取失败 | 选择平坦道路 |
| yaw 不稳定 | 结构特征少 | 增加墙、杆、路沿场景 |
| 多雷达双影 | 时间不同步或外参错误 | 检查同步和点云顺序 |
| 拼接边缘错位 | 重叠区域不足 | 更换采集场景 |

---

## 10. `surround-camera`：环视相机标定

路径：

```text
surround-camera
```

### 10.1 子模块

| 子目录 | 功能 |
|---|---|
| `manual_calib` | 手动环视/鱼眼标定 |
| `auto_calib` | 无靶标自动环视标定 |
| `auto_calib_fisheye` | 鱼眼自动标定 |
| `auto_calib_target` | 靶标自动标定 |

### 10.2 手动标定流程

#### 10.2.1 编译

```bash
cd surround-camera/manual_calib
mkdir -p build && cd build
cmake ..
make
```

#### 10.2.2 运行

```bash
./bin/run_avm
```

或：

```bash
./bin/run_avm pinhole_samples/imgs pinhole_samples/intrinsic pinhole_samples/extrinsic 2
./bin/run_avm ocam_samples/imgs ocam_samples/intrinsic ocam_samples/extrinsic 1
```

输入：

- 四路相机图像。
- 四路相机内参。
- 四路相机初始外参。
- 相机模型类型。

### 10.3 手动标定算法原理

环视系统通常包含前、后、左、右四个相机。目标是求每个相机到车体/地面的外参，使图像投影到地面后能够拼接成连续鸟瞰图。

手动流程一般为：

1. 读取每个相机图像。
2. 读取内参和畸变。
3. 选择或加载地面控制点。
4. 使用 `solvePnP` 求相机外参：

```text
s * p_image = K * [R | t] * P_ground
```

5. 将图像反投影到地面。
6. 拼接四路 BEV。
7. 人工调整外参使拼接缝连续。

### 10.4 无靶标自动标定

#### 10.4.1 编译

```bash
cd surround-camera/auto_calib
mkdir -p build && cd build
cmake ..
make
```

#### 10.4.2 运行

```bash
./bin/run_AVM_Calibration
```

### 10.5 无靶标自动标定算法原理

核心假设：相邻相机在地面上的重叠区域观察到同一片路面，因此正确外参下，投影到 BEV 的纹理应一致。

流程：

1. 使用初始外参将四路图像投影到地面。
2. 生成四路 BEV 图：

```text
BEV_front, BEV_left, BEV_back, BEV_right
```

3. 找到相邻相机的重叠区域：

```text
front-left
front-right
back-left
back-right
```

4. 计算纹理差异：

```text
loss = Σ | I_i(x,y) - I_j(x,y) |
```

5. 对每路相机外参做粗搜索。
6. 对候选外参做细搜索。
7. 选择重叠区域纹理最一致的外参。
8. 输出拼接结果。

### 10.6 鱼眼自动标定

#### 10.6.1 编译

```bash
cd surround-camera/auto_calib_fisheye
mkdir -p build && cd build
cmake ..
make
```

#### 10.6.2 运行

```bash
./bin/run_AVM_Calibration
```

### 10.7 鱼眼模型原理

普通针孔模型：

```text
u = fx * X/Z + cx
v = fy * Y/Z + cy
```

鱼眼镜头视场更大，畸变更强，常使用不同的畸变模型。例如 OpenCV fisheye 模型基于角度：

```text
theta = atan(r)
theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
```

然后映射到图像坐标。

环视鱼眼标定中的地面投影流程：

1. 对 BEV 上每个地面点 `P_ground`。
2. 根据相机外参转到相机坐标系：

```text
P_camera = T_camera_ground * P_ground
```

3. 使用鱼眼模型投影到图像。
4. 采样图像颜色。
5. 生成 BEV。

### 10.8 靶标自动标定

#### 10.8.1 编译

```bash
cd surround-camera/auto_calib_target
mkdir -p build && cd build
cmake ..
make
```

#### 10.8.2 运行

```bash
./bin/run_AVM_Calibration
./bin/run_stitching
```

### 10.9 靶标法算法原理

靶标法适合产线或固定标定场：

1. 在车辆周围摆放已知尺寸和位置的标定板。
2. 每个相机拍摄标定板。
3. 检测标定板角点或特征点。
4. 根据 3D 靶标点和 2D 图像点求相机外参。
5. 将四路图像投影到地面。
6. 进行融合和拼接。

PnP 目标：

```text
min_{R,t} Σ || p_i - project(K, D, R * P_i + t) ||^2
```

环视融合通常使用分区权重：

```text
I_bev(x,y) = w_i(x,y) * I_i(x,y) + w_j(x,y) * I_j(x,y)
```

其中重叠区域权重平滑过渡，以减少拼接缝。

### 10.10 环视标定注意事项

| 问题 | 原因 | 建议 |
|---|---|---|
| 拼接缝断裂 | 外参错误 | 重新优化或手动微调 |
| 左右亮度不一致 | 曝光不同 | 固定曝光/白平衡 |
| 无靶标优化失败 | 地面纹理不足 | 选择车道线、纹理丰富地面 |
| 鱼眼边缘扭曲 | 模型错误 | 确认 fisheye/ocam 参数 |
| 靶标结果偏移 | 靶标尺寸或位置错误 | 重新测量靶标几何 |

---

## 11. `radar2camera`：Radar 到 Camera 手动标定

路径：

```text
radar2camera
```

### 11.1 编译

```bash
cd radar2camera/manual_calib
mkdir -p build && cd build
cmake ..
make
```

### 11.2 运行

```bash
./bin/run_radar2camera <image_path> <radar_file_path> <intrinsic_json> <homo_json> <extrinsic_json>
```

示例：

```bash
./bin/run_radar2camera data/0.jpg data/front_radar.csv data/center_camera-intrinsic.json data/center_camera-homography.json data/radar-to-center_camera-extrinsic.json
```

输入：

| 参数 | 说明 |
|---|---|
| `image_path` | 相机图片 |
| `radar_file_path` | Radar CSV 文件 |
| `intrinsic_json` | 相机内参 |
| `homo_json` | 相机到地面的 homography |
| `extrinsic_json` | 初始 Radar-Camera 外参 |

输出：

- refined Radar-Camera 外参。
- 投影结果图。
- 标定文本。

### 11.3 操作流程

1. 启动程序。
2. 程序读取相机图像、Radar 点、内参、Homography、初始外参。
3. 在图像中选择两条车道线作为一组平行线。
4. 程序通过 Homography 将道路区域转换为鸟瞰图。
5. GUI 显示 Radar 点和道路结构。
6. 用户调整外参。
7. 目标是让 Radar 点在鸟瞰图中与道路方向、车道线方向平行或重合。
8. 保存结果。

### 11.4 算法原理

Radar-Camera 手动标定依赖两个约束：

#### 11.4.1 Camera 到地面 Homography

地面是平面，图像点和地面点之间可用单应矩阵描述：

```text
s * [u, v, 1]^T = H * [X_ground, Y_ground, 1]^T
```

反向可将图像车道线映射到鸟瞰地面。

#### 11.4.2 Radar 点到 Camera/地面坐标

Radar 检测点可表示为：

```text
P_radar = [x, y, z, 1]^T
```

通过外参：

```text
P_camera = T_camera_radar * P_radar
```

或投影到地面鸟瞰图比较。

#### 11.4.3 人工优化目标

用户实际上在手动最小化：

```text
alignment_error = distance(radar_points_projected, lane_direction_or_lane_region)
```

因为 Radar 点稀疏、噪声大、语义不明确，该模块没有采用全自动优化，而是人工交互。

### 11.5 注意事项

- 必须有清晰车道线或道路边界。
- Homography 必须准确。
- Radar CSV 格式必须正确。
- Radar 点通常稀疏，不要期待与图像像素级对齐。
- 推荐选择直路场景。
- 动态车辆会造成 Radar 点偏离道路静态结构。

---

## 12. `radar2lidar`：Radar 到 LiDAR 手动标定

路径：

```text
radar2lidar
```

### 12.1 编译

```bash
cd radar2lidar/manual_calib
mkdir -p build && cd build
cmake ..
make
```

### 12.2 运行

```bash
./bin/run_radar2lidar <pcd_path> <radar_file_path> <extrinsic_json>
```

示例：

```bash
./bin/run_radar2lidar data/lidar.pcd data/front_radar.csv data/front_radar-to-top_center_lidar-extrinsic.json
```

输入：

| 参数 | 说明 |
|---|---|
| `pcd_path` | LiDAR 点云 |
| `radar_file_path` | Radar CSV |
| `extrinsic_json` | 初始 Radar-LiDAR 外参 |

输出：

- refined Radar-LiDAR 外参。

### 12.3 操作流程

1. 读取 LiDAR PCD。
2. 读取 Radar CSV。
3. 将 Radar 点转换为点云。
4. 根据初始外参把 Radar 点变换到 LiDAR 坐标系。
5. 在 GUI 中同时显示 LiDAR 点云和 Radar 点。
6. 手动调整外参。
7. 让 Radar 点落到 LiDAR 中对应静态目标位置。
8. 保存结果。

### 12.4 算法原理

Radar-LiDAR 手动标定本质是三维点集人工配准。

Radar 点：

```text
P_radar_i
```

LiDAR 点云：

```text
Q_lidar = {Q_j}
```

待求：

```text
T_lidar_radar
```

人工观察目标：

```text
T_lidar_radar * P_radar_i ≈ corresponding object in Q_lidar
```

常用对齐对象：

- 静止车辆。
- 路沿。
- 护栏。
- 墙面。
- 反射强的道路结构。

### 12.5 注意事项

- Radar 和 LiDAR 必须时间同步。
- Radar 点少且噪声大，要选择目标明显场景。
- LiDAR intensity 颜色有助于识别车道线。
- 初始外参太差会难以判断。
- 不要用运动目标做对齐基准。

---

## 13. `factory_calib`：工厂产线标定

路径：

```text
factory_calib
```

### 13.1 支持标定板

| 标定板 | 传感器 | 特点 |
|---|---|---|
| chessboard | Camera | 棋盘角点稳定，经典内外参标定 |
| circle board | Camera | 圆心检测，亚像素精度好 |
| vertical board | Camera | 垂直线/角点几何约束 |
| apriltag board | Camera | tag ID 提供强匹配 |
| aruco marker board | Camera | marker ID + 角点 |
| round hole board | Camera + LiDAR | 可用于相机和 LiDAR 联合标定 |

### 13.2 编译

```bash
cd factory_calib
mkdir -p build && cd build
cmake ..
make
```

### 13.3 运行命令

#### 标定板检测

```bash
./bin/run_board_detect image board_type
```

#### 圆孔板 LiDAR 检测

```bash
./bin/run_lidar_detect pcds
```

#### 外参标定示例

```bash
./bin/run_extrinsic_calib
```

#### Homography / Vanishing Point 标定

```bash
./bin/run_homo_vp_calib image board_type output_dir
```

### 13.4 操作流程

1. 确定使用的标定板类型。
2. 准备标定板真实尺寸和点位定义。
3. 在产线固定位置放置标定板。
4. 采集相机图像或 LiDAR 点云。
5. 运行对应检测程序。
6. 检查检测结果图。
7. 运行外参求解程序。
8. 保存标定结果。
9. 用投影或重投影误差复核。

### 13.5 标定板识别原理

#### 13.5.1 棋盘格

1. 检测黑白格交点。
2. 按网格顺序排序。
3. 使用角点和棋盘真实坐标做 PnP 或内参标定。

#### 13.5.2 圆点板

1. 阈值/边缘检测。
2. 连通域或轮廓检测圆。
3. 椭圆拟合求圆心。
4. 根据排列关系排序圆心。

#### 13.5.3 AprilTag / Aruco

1. 检测 marker 边框。
2. 解码 ID。
3. 得到四个角点。
4. 根据 ID 建立 2D-3D 对应。

#### 13.5.4 圆孔板

1. 在 LiDAR 点云中分割板面。
2. 检测圆孔边缘。
3. 拟合圆心。
4. 与板上真实圆孔坐标匹配。

### 13.6 外参求解原理

相机外参常用 PnP：

```text
min_{R,t} Σ || p_i - project(K, D, R * P_i + t) ||^2
```

平面标定板可使用 Homography：

```text
s * p_i = H * P_i_plane
```

Homography 与外参关系：

```text
H = K * [r1 r2 t]
```

在已知 `K` 的情况下可恢复 `R,t`。

### 13.7 产线方案工具

路径：

```text
factory_calib/tool/factory_solution
```

该工具用于根据：

- 传感器安装位置。
- 传感器视场角。
- 标定板尺寸。
- 车辆尺寸。
- 标定间距。

计算：

- 标定板可见范围。
- 最小/最大距离。
- 推荐标定板位置。
- 不同传感器可覆盖区域。

### 13.8 注意事项

| 问题 | 原因 | 建议 |
|---|---|---|
| 检测失败 | 板不清晰、遮挡、反光 | 调整光照和曝光 |
| 重投影误差大 | 板尺寸错误或点排序错误 | 检查配置和检测结果 |
| LiDAR 圆孔检测失败 | 点云稀疏或角度不合适 | 调整距离和板面角度 |
| 产线结果不稳定 | 板位重复性差 | 使用固定工装 |
| Homography 偏差 | 地面/板面不平 | 保证平面假设成立 |

---

## 14. `SensorX2car`：传感器到车体系标定

路径：

```text
SensorX2car
```

### 14.1 总体说明

SensorX2car 用于将常见传感器标定到车体坐标系：

| 子模块 | 标定内容 |
|---|---|
| `camera2car` | Camera 到 car 的旋转 |
| `lidar2car` | LiDAR 到 car 的旋转和高度 |
| `pose_sensor2car` | GNSS/INS/PoseSensor 到 car 的 yaw |
| `radar2car` | Radar 到 car 的 yaw |

### 14.2 `camera2car`

路径：

```text
SensorX2car/camera2car
```

#### 14.2.1 自动方法环境

```bash
conda create -n ctrlc
conda activate ctrlc
conda install -c pytorch torchvision
pip install -r requirements.txt
```

#### 14.2.2 训练

```bash
python train.py --config-file config-files/ctrlc.yaml
```

多 GPU：

```bash
python -m torch.distributed.launch --nproc_per_node=4 --use_env train.py --config-file config-files/ctrlc.yaml
```

#### 14.2.3 测试

KITTI：

```bash
python test_kitti.py --config-file config-files/ctrlc.yaml --opts MODE test
```

单张图：

```bash
python test_img.py --config-file config-files/ctrlc.yaml --opts MODE test DATASET_DIR ./pic/ OUTPUT_DIR ./output/
```

直接标定：

```bash
python test_img_calib.py --config-file config-files/ctrlc.yaml --opts MODE test DATASET_DIR ./pic/
```

#### 14.2.4 自动算法原理

Camera-to-car 的旋转可由道路图像中的两个几何元素推断：

1. 消失点 vanishing point。
2. 地平线 horizon line。

对于直行道路，平行车道线在图像中交于消失点。消失点方向对应车体前进方向在相机坐标系中的投影。

设消失点像素为：

```text
v = [u_v, v_v, 1]^T
```

归一化到相机坐标：

```text
d_camera = K^{-1} * v
```

`d_camera` 表示车体前向在相机坐标系中的方向。地平线可进一步约束 roll。最终由 vanishing point 和 horizon angle 恢复 camera-to-car 旋转矩阵。

`vphl2R.py` 实现了从 vanishing point + horizon line 到旋转矩阵的转换。

#### 14.2.5 手动方法编译

```bash
cd SensorX2car/camera2car/manual_calib
mkdir -p build && cd build
cmake ..
make
```

#### 14.2.6 手动方法运行

```bash
./bin/run_camera2car <image_path> <intrinsic_json>
```

示例：

```bash
./bin/run_camera2car data/0.png data/center_camera-intrinsic.json
```

手动目标：

1. 调整 x/y/z 旋转。
2. 让消失点位于参考中心。
3. 让地平线与参考线平行。
4. 保存旋转矩阵。

#### 14.2.7 注意事项

- 自动方法需要直行道路图像。
- 相机内参必须准确。
- 如果换数据集，CTRL-C 可能需要重新训练。
- 雨天、夜晚、车道线不可见会影响结果。

### 14.3 `lidar2car`

路径：

```text
SensorX2car/lidar2car
```

#### 14.3.1 编译

```bash
cd SensorX2car/lidar2car
mkdir -p build && cd build
cmake ..
make
```

#### 14.3.2 运行

```bash
./bin/run_lidar2car ./data/example/ ./output/
```

指定帧范围：

```bash
./bin/run_lidar2car ./data/example/ ./output/ <start_frame> <end_frame>
```

输入：

- LiDAR 连续帧目录。
- 输出目录。

输出：

| 文件 | 说明 |
|---|---|
| `calib_result.txt` | 标定结果 |
| `pose.txt` | LiDAR 轨迹 |
| `trajectory.png` | 轨迹图 |
| `compared_yaw.png` | yaw 对比图 |

#### 14.3.3 算法原理

该模块估计 LiDAR 到车体的：

- roll。
- pitch。
- yaw。
- LiDAR 高度。

流程：

1. 使用 LiDAR 里程计估计连续帧相对位姿。
2. 根据轨迹恢复 LiDAR 坐标系下的运动方向。
3. 用地面约束估计 roll/pitch/height。
4. 用轨迹前进方向估计 yaw。
5. 输出 LiDAR-to-car 外参。

车体坐标系通常假设：

- x：车辆前方。
- y：车辆左方或右方，需看具体定义。
- z：上方。

LiDAR 运动轨迹方向应与车体 x 轴一致，因此：

```text
yaw_offset = yaw_trajectory - yaw_lidar_pose
```

地面约束：

如果地面在车体系应接近：

```text
z = 0
```

则 LiDAR 点云地面经外参变换后应平坦且高度一致。

#### 14.3.4 注意事项

- 推荐车辆直线行驶。
- 地面越平，roll/pitch 越准。
- 点云连续帧质量影响里程计。
- 若远程无显示，README 提到可将 matplotlib 后端设为 `Agg`。

### 14.4 `pose_sensor2car`

路径：

```text
SensorX2car/pose_sensor2car
```

#### 14.4.1 编译

```bash
cd SensorX2car/pose_sensor2car
mkdir -p build && cd build
cmake ..
make
```

#### 14.4.2 运行

```bash
./bin/run_imu2car ./data/example/ ./output/
```

指定帧范围：

```bash
./bin/run_imu2car ./data/example/ ./output/ 0 20000
```

输入：

- GNSS/INS pose 文件，包含 `x, y, yaw`。

输出：

- `trajectory.png`
- `compared_yaw.png`

#### 14.4.3 算法原理

pose sensor 输出的 yaw 与车辆真实行驶方向之间存在安装角偏差。利用轨迹方向估计车体 yaw，再与 pose sensor yaw 比较：

```text
yaw_car = atan2(y_{i+1} - y_i, x_{i+1} - x_i)
offset = yaw_car - yaw_pose_sensor
```

对多帧 offset 做平滑或拟合得到最终标定值。

### 14.5 `radar2car`

路径：

```text
SensorX2car/radar2car
```

#### 14.5.1 编译

```bash
cd SensorX2car/radar2car
mkdir -p build && cd build
cmake ..
make
```

#### 14.5.2 运行

```bash
./bin/run_radar2car conti data/conti/front_radar/ data/conti/novatel_enu.csv
./bin/run_radar2car delphi data/delphi/front_radar data/delphi/novatel_enu.csv 20 500
```

命令格式：

```bash
./bin/run_radar2car <radar_type> <radar_dir_path> <inspva_file> [start_file_num max_file_num]
```

#### 14.5.3 算法原理

对静态目标，Radar 测到的径向速度来自车辆自身运动投影。

设：

- 车辆速度为 `v_ego`。
- Radar 目标方位角为 `theta`。
- Radar 相对车体 yaw 偏差为 `yaw_offset`。
- Radar 测得径向速度为 `v_r`。

理想关系：

```text
v_r = -v_ego * cos(theta + yaw_offset)
```

因此可通过多个静态目标拟合：

```text
min_yaw Σ || v_r_i + v_ego_i * cos(theta_i + yaw) ||^2
```

流程：

1. 读取 Radar 目标。
2. 读取车辆速度/pose。
3. 根据速度阈值过滤无效帧。
4. 根据运动一致性筛选静态目标。
5. 拟合 yaw offset。
6. 输出 Radar-to-car yaw。

#### 14.5.4 注意事项

- 车辆速度不能太低。
- 静态目标数量要足够。
- 动态车辆会污染拟合。
- Radar 时间戳必须与车辆速度同步。

---

## 15. `online_calib`：在线标定

路径：

```text
online_calib
```

### 15.1 子模块

| 子目录 | 功能 |
|---|---|
| `vins_camera2imu` | Camera-IMU 在线标定 |
| `lidar2imu` | LiDAR-IMU / LiDAR-Car 在线标定 |
| `radar2carcenter` | Radar 到车体中心标定 |

注意：当前 `online_calib/vins_camera2imu/README.md` 中存在 merge conflict 标记，文档本身未清理，使用时应以源码和顶层 `online_calib/README.md` 为准。

### 15.2 Camera2IMU

命令格式：

```bash
./main yaml_car_config_path raw_imu_file video_file video_timestamp_file output_path
```

输入：

| 参数 | 说明 |
|---|---|
| `yaml_car_config_path` | 车辆/相机/IMU 配置 |
| `raw_imu_file` | IMU 原始数据 |
| `video_file` | 视频文件 |
| `video_timestamp_file` | 视频时间戳 |
| `output_path` | 输出路径 |

算法思想：

1. 从视频中提取视觉特征。
2. 读取 IMU 数据。
3. 对 IMU 做预积分。
4. 构造视觉重投影约束和 IMU 运动约束。
5. 联合估计相机位姿、IMU 状态、Camera-IMU 外参。

典型视觉惯性优化目标：

```text
min Σ reprojection_error^2 + Σ imu_preintegration_error^2
```

### 15.3 LiDAR2IMU 在线标定

命令格式：

```bash
./main lidar_dir novatel_pose_path config_path output_dir
./main lidar_dir novatel_pose_path config_path output_dir start_second max_seconds
```

输入：

| 参数 | 说明 |
|---|---|
| `lidar_dir` | LiDAR PCD 文件目录 |
| `novatel_pose_path` | NovAtel pose |
| `config_path` | 标定配置 |
| `output_dir` | 输出目录 |
| `start_second` | 可选起始时间 |
| `max_seconds` | 可选最大时长 |

算法思想：

1. 使用 LiDAR 前端估计 LiDAR 轨迹。
2. 使用 NovAtel pose 作为 IMU/车体轨迹。
3. 取相邻时刻运动：

```text
A_i = T_imu_i^{-1} * T_imu_{i+1}
B_i = T_lidar_i^{-1} * T_lidar_{i+1}
```

4. 构造手眼标定：

```text
A_i * X = X * B_i
```

其中 `X` 是 LiDAR 到 IMU 的外参。

5. 初值求解后进行非线性优化。

优化形式：

```text
min_X Σ || log( A_i * X * B_i^{-1} * X^{-1} ) ||^2
```

### 15.4 Radar2CarCenter

命令格式：

```bash
./main radar_type radar_dir_path inspva_file output_dir
```

输入：

| 参数 | 说明 |
|---|---|
| `radar_type` | 雷达类型 |
| `radar_dir_path` | Radar 数据目录 |
| `inspva_file` | 车辆 INS/PVA 文件 |
| `output_dir` | 输出目录 |

算法与 `SensorX2car/radar2car` 类似：

1. 读取 Radar 目标。
2. 读取车辆速度和姿态。
3. 筛选静态目标。
4. 使用径向速度余弦模型拟合 yaw。
5. 输出 Radar 到车体中心的标定结果。

---

## 16. `radar2camera`、`radar2lidar` 与 `radar2car` 的区别

| 模块 | 标定目标 | 主要观测 | 自动化程度 |
|---|---|---|---|
| `radar2camera` | Radar 到 Camera | 图像车道线 + Radar 点 | 手动 |
| `radar2lidar` | Radar 到 LiDAR | LiDAR 点云 + Radar 点 | 手动 |
| `SensorX2car/radar2car` | Radar 到车体 | Radar 径向速度 + 车速 | 自动 |
| `online_calib/radar2carcenter` | Radar 到车体中心 | Radar 径向速度 + INSPVA | 自动 |

选择建议：

- 如果需要 Radar 与图像对齐，用 `radar2camera`。
- 如果需要 Radar 与点云对齐，用 `radar2lidar`。
- 如果只需要 Radar 安装 yaw 到车体，用 `SensorX2car/radar2car`。
- 如果需要在线处理或持续更新，用 `online_calib/radar2carcenter`。

---

## 17. 推荐完整标定流程

对于一辆自动驾驶车辆，建议顺序如下。

### 17.1 先标定单传感器内部参数

1. Camera intrinsic：

```bash
camera_intrinsic/intrinsic_calib
```

2. Camera distortion verification：

```bash
camera_intrinsic/calib_verification
```

### 17.2 再标定传感器到车体系

1. Pose sensor / IMU heading：

```bash
imu_heading
SensorX2car/pose_sensor2car
```

2. Camera-to-car：

```bash
SensorX2car/camera2car
```

3. LiDAR-to-car：

```bash
SensorX2car/lidar2car
```

4. Radar-to-car：

```bash
SensorX2car/radar2car
```

### 17.3 再标定传感器之间外参

1. LiDAR-Camera：

```bash
lidar2camera
```

2. LiDAR-IMU：

```bash
lidar2imu
```

3. LiDAR-LiDAR：

```bash
lidar2lidar
```

4. Radar-Camera：

```bash
radar2camera
```

5. Radar-LiDAR：

```bash
radar2lidar
```

### 17.4 环视系统

```bash
surround-camera
```

### 17.5 产线标定

```bash
factory_calib
```

---

## 18. 数据采集通用建议

### 18.1 Camera

- 固定曝光和白平衡。
- 保持图像清晰，避免运动模糊。
- 棋盘格应覆盖全视场。
- 道路场景应包含车道线、地平线、路沿。

### 18.2 LiDAR

- 使用静态结构丰富场景。
- 保证点云无明显时间错位。
- 多 LiDAR 应同步采集。
- 尽量减少雨雾、强反射异常。

### 18.3 IMU/GNSS

- 保证 GNSS 信号质量。
- 记录准确时间戳。
- 直线段用于 yaw/heading。
- 转弯/绕行段用于激励 LiDAR-IMU 外参。

### 18.4 Radar

- 保证车辆有一定速度。
- 选择静态目标丰富场景。
- 避免拥堵和大量动态车辆。
- Radar CSV 格式必须和代码一致。

### 18.5 环视

- 四路相机同步。
- 光照均匀。
- 重叠区域有纹理。
- 靶标摆放准确。

---

## 19. 常见故障总表

| 现象 | 可能模块 | 典型原因 | 排查建议 |
|---|---|---|---|
| 图像投影整体偏左/右 | LiDAR-Camera、Radar-Camera | yaw 或 lateral translation 错 | 手动调 yaw/y 平移 |
| 投影近处准远处不准 | Camera 相关 | 内参或畸变错误 | 重标相机内参 |
| 点云拼接双影 | LiDAR-IMU、LiDAR-LiDAR | 外参或时间同步错误 | 检查时间戳和初值 |
| 地面分层 | LiDAR-IMU、LiDAR2Car | roll/pitch/height 错 | 使用平地数据重跑 |
| ICP 发散 | LiDAR-LiDAR | 初值太差或重叠不足 | 先手动粗调 |
| Radar yaw 不稳定 | Radar2Car | 静态目标少、速度低 | 换直路中速数据 |
| 环视拼接缝断裂 | surround-camera | 外参错误或曝光差 | 固定曝光、重做标定 |
| 棋盘角点检测失败 | camera_intrinsic、factory | 模糊、反光、遮挡 | 重新采集 |
| 自动 LiDAR-Camera 失败 | lidar2camera auto | mask 差或线特征少 | 重新生成 mask |
| VINS 类标定失败 | online camera2imu | 视频/IMU 不同步 | 检查时间戳 |

---

## 20. 工程维护建议

1. 每个模块独立编译，不建议全仓统一编译。
2. 保存每次标定使用的数据、初值、输出、日志。
3. 外参文件名应明确方向，例如：

```text
top_center_lidar-to-center_camera-extrinsic.json
gnss-to-top_center_lidar-extrinsic.json
front_radar-to-top_center_lidar-extrinsic.json
```

4. 所有自动标定结果必须用可视化复核。
5. 不同模块可能使用不同坐标系约定，接入系统前必须统一。
6. 当前 `online_calib/vins_camera2imu/README.md` 有冲突标记，应清理后再对外发布。
7. 对产线使用，建议固定 Docker 镜像和依赖版本。
8. 对论文复现实验，建议保留原始数据和初始外参。

---

## 21. 快速索引

| 任务 | 推荐模块 | 命令入口 |
|---|---|---|
| 相机内参 | `camera_intrinsic/intrinsic_calib` | `run_intrinsic_calibration` |
| 相机畸变评估 | `camera_intrinsic/calib_verification` | `run_distortion_measure` |
| IMU heading | `imu_heading/auto_calib` | `run_imu_heading` |
| LiDAR-IMU 手动 | `lidar2imu/manual_calib` | `run_lidar2imu` |
| LiDAR-IMU 自动 | `lidar2imu/auto_calib` | `run_lidar2imu` |
| LiDAR-Camera 手动 | `lidar2camera/manual_calib` | `run_lidar2camera` |
| LiDAR-Camera 线特征自动 | `lidar2camera/auto_calib` | `run_lidar2camera` |
| LiDAR-Camera SAM | `lidar2camera/auto_calib_v2.0` | `run_lidar2camera` |
| LiDAR-Camera 联合标定 | `lidar2camera/joint_calib` | `lidar2camera` |
| LiDAR-LiDAR 手动 | `lidar2lidar/manual_calib` | `run_lidar2lidar` |
| LiDAR-LiDAR 自动 | `lidar2lidar/auto_calib` | `run_lidar2lidar` |
| 环视手动 | `surround-camera/manual_calib` | `run_avm` |
| 环视自动 | `surround-camera/auto_calib` | `run_AVM_Calibration` |
| 鱼眼环视 | `surround-camera/auto_calib_fisheye` | `run_AVM_Calibration` |
| 靶标环视 | `surround-camera/auto_calib_target` | `run_AVM_Calibration`, `run_stitching` |
| Radar-Camera | `radar2camera/manual_calib` | `run_radar2camera` |
| Radar-LiDAR | `radar2lidar/manual_calib` | `run_radar2lidar` |
| 工厂标定 | `factory_calib` | `run_board_detect`, `run_lidar_detect`, `run_extrinsic_calib` |
| Camera-to-Car | `SensorX2car/camera2car` | `run_camera2car`, Python CTRL-C |
| LiDAR-to-Car | `SensorX2car/lidar2car` | `run_lidar2car` |
| PoseSensor-to-Car | `SensorX2car/pose_sensor2car` | `run_imu2car` |
| Radar-to-Car | `SensorX2car/radar2car` | `run_radar2car` |
| 在线 LiDAR-IMU | `online_calib/lidar2imu` | `main` |
| 在线 Radar-CarCenter | `online_calib/radar2carcenter` | `main` |

---

## 22. 结论

SensorsCalibration 是一个覆盖面很广的自动驾驶标定工具箱。它不是单一工程，而是一组相对独立的标定程序集合。使用时应按标定对象选择子模块，并特别关注：

1. 坐标系方向。
2. 初始外参质量。
3. 时间同步。
4. 相机内参准确性。
5. 数据采集场景是否满足算法假设。
6. 输出结果的可视化验证。

对于实际车辆标定，推荐采用“内参 → 单传感器到车体系 → 传感器间外参 → 环视/产线复核”的顺序逐步完成，并保留每一步的输入、输出和验证图。
