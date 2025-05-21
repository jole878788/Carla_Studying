## CARLA-SB3-RL-Training-Environment
该项目只把核心模块代码上传，整个项目基于 CARLA 0.9.14 模拟器和 Stable Baselines3 强化学习库

车辆所处的environment定义都在./CARLA-SB3-RL-Training-Environment/carla_env,所以下面讲解一下Carla_env部分

### 1. `carla_env/envs/`：环境封装 & 数据采集模块

这个目录主要负责两个方面：一是定义训练用的环境类，二是采集图像数据用于 VAE 或监督训练。

#### `carla_route_env.py` —— 主强化学习环境

这是项目中最重要的一个类，`CarlaRouteEnv` 继承自 Gym 的 `Env` 类。它主要负责：

- 初始化地图、车辆与传感器
- 设置观测空间和动作空间（支持离散或连续动作）
- 接入用户自定义的奖励函数和状态提取函数
- 通过 `planner.compute_route_waypoints()` 来规划车辆的行驶路径


这个环境可以灵活适配不同状态组合，比如只用速度，也可以加上相机图像、航向角等。


#### `birdview_env.py` —— 鸟瞰图环境

这个脚本提供一个额外的类 `CarlaBirdView`，它的作用是：

- 从上帝视角获取车辆的鸟瞰图
- 可以用来绘制路径点，进行直观可视化
- 设置CARLA中的 FPS 和渲染质量

这个环境将被用于模仿学习(IL)中的视觉状态输入。


#### `collect_data_manual_env.py` —— 手动采集图像

这个脚本允许你用键盘控制车辆，同时记录：

- RGB 图像
- 分割图
- 控制信号（方向盘、油门）

采集的数据可用于后续训练 VAE 或行为克隆模型。


#### `collect_data_rl_env.py` —— 自动采集图像

跟上一个脚本类似，不过这里是由一个已训练好的 RL 策略来开车，自动采集图像数据。适合训练初期使用，用来快速收集大量图像样本。


### 2. `carla_env/navigation/`：路径规划 & 控制模块

这一部分负责路径的规划、控制器设计和智能体行为逻辑。

#### `planner.py` —— 路径动作与路径点生成

- 定义了 `RoadOption`，比如直行、左转、右转等动作类型
- 提供 `compute_route_waypoints(start, end)` 函数，用于计算从 A 点到 B 点的路径点列表


#### `global_route_planner.py` + `global_route_planner_dao.py`

- 这两个脚本共同实现全局路径规划器（类似 A* 或 Dijkstra）
- `DAO` 类用于获取地图结构数据
- 支持构建路网拓扑图并计算最优路径


#### `local_planner.py` —— 局部路径追踪器

- 用于根据全局路径点进行局部规划
- 结合 PID 控制器输出控制信号（steer, throttle）
- 常用于车辆紧贴车道线、平滑转弯等任务


#### `controller.py` —— PID 控制器实现

- 实现了横向控制（方向盘）和纵向控制（速度）
- 输出控制量用于精细操控车辆行为
- 可以单独用于规则策略或模仿学习 agent


#### `agent.py` —— 智能体接口定义

- 定义了智能体的基类，包括常用接口 like `run_step()` 和 `set_destination()`
- 所有规则或学习策略都继承自这个基类


#### `basic_agent.py` —— 规则驾驶智能体

- 基于规划 + 控制器实现的自动驾驶 agent
- 会使用全局规划器和局部控制器保持在车道中行驶
- 可绕开静态障碍，行为较为保守

#### `roaming_agent.py` —— 随机漫游 agent

- agent 会在地图中随机选取起止点行驶
- 常用于探索和多样化轨迹采集

---

配置文件都在 config.py 中，有以下参数：

- algorithm: 要使用的 RL 算法。支持 Stable Baselines 3 中的所有算法。
- algoritm_params: 算法的参数。有关更多信息，请参阅 Stable Baselines 3 文档。
- state: 用作 state 的列表。例如, steer, throttle, speed, angle_next_waypoint, maneuver, waypoints, rgb_camera, seg_camera, end_wp_vector, end_wp_fixed, distance_goal 有关更多信息，请参阅 “carla_env/state_commons.py” 文件。
- vae_model: 要使用的 VAE 模型。此 repo 包含两个预训练模型：vae_64 和 vae_64_augmentation。如果为 None，则不使用 VAE。
- action_smoothing: 是否使用动作平滑。
- reward_fn: 要使用的奖励函数。更多信息请参阅 carla_env/reward_functions.py 文件。
- reward_params: 奖励函数的参数。
- obs_res: 观察的分辨率。建议使用 (160, 80)
- seed: 要使用的随机种子。
- wrappers: 要使用的包装器列表。目前已实现两个：HistoryWrapperObsDict 和 FrameSkip。有关更多信息，请参阅 carla_env/wrappers.py 文件。
