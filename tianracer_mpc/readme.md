

## Tianracer MPC controller


This package provides a launch file `demo_tianracer_mpc.launch` for the basic configuration that:

- Sets up a Gazebo simulation environment with an empty world
- Spawns the robot model in the Gazebo world at a specified position
- Includes the control.launch file to set up ROS controllers for the robot

```bash
roslaunch tianracer_mpc demo_tianracer_mpc.launch
```

# Tianracer MPC 控制器
这个包实现了基于模型预测控制(MPC)的路径跟踪算法，用于天锐赛车(Tianracer)机器人的自主导航。MPC控制器能够同时优化速度和转向控制，提供平滑且高效的路径跟踪性能。
## 功能特点

- 基于迭代线性MPC的速度和转向控制
- 支持多机器人协同仿真
- 与ROS导航栈集成
- 三次样条曲线路径平滑
- 动态参数配置

# 依赖项

- ROS Noetic
- Gazebo
- Python 3
- cvxpy (用于求解MPC优化问题)
- scipy
- numpy

## 安装

```bash
# 安装依赖项
pip3 install cvxpy scipy numpy

# 克隆仓库到工作空间
cd ~/catkin_ws/src
git clone https://github.com/uwmyuan/tianracer.git

# 编译并运行
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch tianracer_mpc demo_tianracer_mpc_nav.launch
```

```bash
# 开另一个terminal
# 发布目标位置
rostopic pub /tianracer/move_base_simple/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose: 
  position: 
    x: -0.005710601806640625
    y: 3.0816142559051514
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.7104248547260369
    w: 0.7037730641247144" --once

```

## author
@uwmyuan Yun Yuan

# tianracer_gazebo 代码结构与调用关系分析

## 文件夹结构
```plaintext
tianracer_gazebo/
├── CMakeLists.txt          # 编译配置文件
├── package.xml             # 包信息文件
├── config/                 # 配置文件目录
│   ├── keyboard_teleop.yaml
│   ├── spawn_pose.yaml
│   ├── teb/                # TEB路径规划相关配置
│   └── tianracer_control.yaml  # 控制器配置
├── launch/                 # 启动文件目录
│   ├── includes/           # 被包含的子启动文件
│   ├── tianracer_control.launch
│   ├── tianracer_on_racetrack.launch
│   ├── spawn_with_rviz.launch
│   └── ...
├── maps/                   # 地图文件
├── rviz/                   # RViz配置文件
├── scripts/                # Python脚本
├── urdf/                   # 机器人模型定义
└── worlds/                 # Gazebo世界文件
 ```

## 核心组件及调用关系
### 1. 机器人模型定义 (URDF)
机器人模型定义在 urdf/ 目录下，主要文件包括：

- tianracer_run.urdf.xacro / tianracer_run.xacro : 入口文件，包含其他URDF文件
- tianracer.urdf.xacro / tianracer.xacro : 定义机器人的主体结构
- racecar.urdf.gazebo / racecar.gazebo : 定义Gazebo相关的物理属性和插件
调用关系：

```plaintext
tianracer_run.urdf.xacro
  └── tianracer.urdf.xacro
       ├── macros_tf.xacro (定义变量和宏)
       └── racecar.urdf.gazebo (Gazebo插件和物理属性)
 ```

### 2. 启动文件 (Launch)
启动文件在 launch/ 目录下，主要的调用关系如下：
1. tianracer_on_racetrack.launch : 在赛道世界中启动Gazebo环境   
   - 加载世界文件
   - 在命名空间内生成机器人模型

2. tianracer_control.launch : 启动机器人控制器   
   - 加载控制器配置 ( config/tianracer_control.yaml )
   - 启动控制器管理器和各个控制器

3. spawn_with_rviz.launch : 生成机器人并启动RViz可视化   
   - 加载地图
   - 生成机器人模型
   - 启动控制器
   - 启动导航组件 (TEB路径规划)
   - 启动AMCL定位
   - 启动RViz

4. demo_tianracer_teb_nav.launch : 演示导航功能
      - 根据robot_name参数决定是否使用命名空间
   - 加载地图
   - 启动仿真环境
   - 启动控制器
   - 启动导航和定位组件

### 3. 控制系统
控制系统主要通过 tianracer_control.launch 和 config/tianracer_control.yaml 配置：
- 使用 gazebo_ros_control 插件连接Gazebo和ROS控制器
- 定义了多个控制器：
  - joint_state_controller : 发布关节状态
  - 轮子速度控制器: 控制后轮速度
  - 转向位置控制器: 控制前轮转向

### 4. 导航系统
导航系统主要通过 includes/teb_base.launch.xml 和相关配置文件实现：
- 使用TEB (Timed Elastic Band) 局部路径规划器
- 配置全局和局部代价地图
- 使用AMCL进行定位 ( includes/amcl.launch.xml )

### 5. Python脚本
scripts/ 目录下的Python脚本提供了额外功能：
- tianracer_tf.py : 处理TF变换和跟随行为
- f1tenth_racer.py : 实现比赛状态机，处理路点导航
- upload.py : 提供文件打包上传功能的GUI工具
## 典型启动流程
以 spawn_with_rviz.launch 为例，启动流程：
1. 加载地图服务器
2. 加载机器人模型描述
3. 在Gazebo中生成机器人模型
4. 启动控制器
5. 启动导航组件
6. 启动AMCL定位
7. 启动RViz可视化

demo_tianracer_teb_nav.launch 代码节点通信关系
## 核心节点及其通信关系
### 1. 地图服务器 (map_server)
- 发布 ：
  - $(arg robot_name)/map - 地图数据
  - $(arg robot_name)/map_metadata - 地图元数据
- 服务 ：
  - $(arg robot_name)/static_map - 提供静态地图服务
### 2. Gazebo 仿真环境
- 节点 ：
  - gazebo - 仿真引擎
  - gazebo_gui - 仿真可视化界面
- 通信 ：
  - /gazebo/model_states - 模型状态
  - /gazebo/link_states - 链接状态
  - /clock - 仿真时钟
### 3. 机器人控制系统

- 节点 ：
  - controller_spawner - 控制器生成器
  - robot_state_publisher - 机器人状态发布器
- 通信 ：
  - 订阅 ：
    - $(arg robot_name)/cmd_vel - 速度命令
    - $(arg robot_name)/joint_states - 关节状态
  - 发布 ：
    - $(arg robot_name)/odom - 里程计数据
    - $(arg robot_name)/tf - 坐标变换
### 4. 导航系统 (TEB 局部规划器)

- 节点 ：
  - move_base - 导航核心节点
  - 全局规划器
  - TEB 局部规划器
- 通信 ：
  - 订阅 ：
    - $(arg robot_name)/scan - 激光雷达数据
    - $(arg robot_name)/odom - 里程计数据
    - $(arg robot_name)/map - 地图数据
    - $(arg robot_name)/move_base_simple/goal - 简单目标点
  - 发布 ：
    - $(arg robot_name)/cmd_vel - 速度命令
    - $(arg robot_name)/global_costmap/costmap - 全局代价地图
    - $(arg robot_name)/local_costmap/costmap - 局部代价地图
    - $(arg robot_name)/global_plan - 全局路径
    - $(arg robot_name)/local_plan - 局部路径
### 5. 定位系统 (AMCL)

- 节点 ：
  - amcl - 自适应蒙特卡洛定位
- 通信 ：
  - 订阅 ：
    - $(arg robot_name)/scan - 激光雷达数据
    - $(arg robot_name)/tf - 坐标变换
    - $(arg robot_name)/map - 地图数据
    - $(arg robot_name)/initialpose - 初始位置
  - 发布 ：
    - $(arg robot_name)/amcl_pose - 定位结果
    - $(arg robot_name)/particlecloud - 粒子云
    - $(arg robot_name)/tf - 更新的坐标变换 (map->odom)
### 6. 可视化 (RViz)

- 节点 ：
  - rviz - 可视化工具
- 通信 ：
  - 订阅 ：多个话题用于可视化，包括地图、激光雷达数据、路径规划、机器人模型等
## 数据流通路径
### 1. 传感器数据流
- 激光雷达数据流 ：
  
  - Gazebo 插件生成激光雷达数据
  - 发布到 $(arg robot_name)/scan 话题
  - AMCL 和 move_base 订阅此话题进行定位和避障
- 里程计数据流 ：
  
  - 控制器计算里程计数据
  - 发布到 $(arg robot_name)/odom 话题
  - move_base 和 AMCL 订阅此话题进行定位和路径规划
### 2. 导航控制流
- 目标点设置 ：
  
  - RViz 或其他节点发布目标点到 $(arg robot_name)/move_base_simple/goal
  - move_base 订阅此话题并开始规划
- 路径规划 ：
  
  - move_base 计算全局路径和局部路径
  - 发布到 $(arg robot_name)/global_plan 和 $(arg robot_name)/local_plan
- 运动控制 ：
  
  - move_base 发布速度命令到 $(arg robot_name)/cmd_vel
  - 控制器订阅此话题并控制机器人运动
### 3. 定位与地图
- 地图服务 ：
  
  - map_server 加载地图并发布到 $(arg robot_name)/map
  - move_base 和 AMCL 订阅此话题
- 定位更新 ：
  
  - AMCL 根据激光雷达数据和地图进行定位
  - 发布定位结果到 $(arg robot_name)/amcl_pose
  - 更新 map->odom 的 TF 变换
## 坐标系统
TianRacer 使用的主要坐标系包括：

- map - 全局地图坐标系
- odom - 里程计坐标系
- base_link - 机器人基座坐标系
- 各个关节和传感器的坐标系
这些坐标系通过 TF 树连接起来，形成完整的坐标变换关系。

## 命名空间管理
demo_tianracer_teb_nav.launch 文件中使用条件分支来处理不同的命名空间情况：

```xml
<group if="$(eval arg('robot_name') == '/')">
    <!-- 不使用命名空间的节点配置 -->
</group>

<group unless="$(eval arg('robot_name') == '/')">
    <!-- 使用命名空间的节点配置 -->
</group>
 ```

这种设计是为了在单机器人和多机器人场景下灵活使用，通过命名空间隔离不同机器人的话题和服务。


## TEB 算法的调用关系分析
通过分析 tianracer_gazebo 项目的代码，梳理 TEB (Timed Elastic Band) 局部路径规划算法的调用关系，以及它与 scripts 目录下脚本的交互。TEB 算法主要通过以下方式在项目中被配置和调用：

1. TEB 配置文件
TEB 算法的配置主要位于 config/teb 目录下：
config/teb/
├── base_global_planner_params.yaml
├── costmap_common_params.yaml
├── global_costmap_params.yaml
├── local_costmap_params.yaml
├── move_base_params.yaml
└── teb_local_planner_params.yaml
其中 teb_local_planner_params.yaml 包含了 TEB 算法的核心参数。 
2. TEB 启动文件
TEB 算法通过 launch/includes/teb_base.launch.xml 文件加载。
3. 在主启动文件中的调用
在主启动文件如 demo_tianracer_teb_nav.launch 中，TEB 被调用。scripts 目录下的脚本与 TEB 算法有以下几种交互关系：

1. 消息转换 (transform.py)
transform.py 脚本负责将 cmd_vel (Twist 消息) 转换为 ackermann_cmd_stamped (AckermannDriveStamped 消息)。这个脚本订阅了 cmd_vel 话题，该话题是 TEB 规划器输出的控制命令，然后将其转换为阿克曼转向模型所需的消息格式。

2. 路点导航 (f1tenth_racer.py)
f1tenth_racer.py 脚本实现了基于路点的导航功能，它通过 ROS Action 接口与 move_base 节点（其中包含 TEB 规划器）交互。这个脚本创建了一个 SimpleActionClient 来与 move_base 交互，发送导航目标点，而 move_base 内部使用 TEB 算法进行路径规划。

3. 里程计数据处理 (gazebo_odometry.py)
gazebo_odometry.py 脚本处理 Gazebo 模拟器提供的位置信息，并将其转换为标准的 ROS 里程计消息。这个脚本发布的里程计数据被 TEB 算法用于定位和路径规划。在 teb_local_planner_params.yaml 中，odom_topic 参数指定了 TEB 应该订阅的里程计话题。

4. 键盘遥控 (keyboard_teleop.py)
keyboard_teleop.py 脚本提供了键盘控制机器人的功能,发布的控制命令会覆盖 TEB 规划器的输出，允许用户手动控制机器人。

TEB 算法与 scripts 目录下脚本的数据流向可以概括为：

输入到 TEB：
里程计数据 (gazebo_odometry.py → odom 话题 → TEB)
激光雷达数据 (Gazebo 插件 → scan 话题 → TEB)
导航目标 (f1tenth_racer.py → move_base_simple/goal 或 Action → TEB)

TEB 的输出：
速度命令 (TEB → cmd_vel 话题 → transform.py)
路径可视化 (TEB → global_plan, local_plan 话题 → RViz)

控制流：
TEB 生成的 cmd_vel → transform.py 转换 → ackermann_cmd_stamped → 机器人控制器
或者 keyboard_teleop.py 直接发布 ackermann_cmd_stamped → 机器人控制器

