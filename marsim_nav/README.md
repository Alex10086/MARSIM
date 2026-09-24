# marsim_nav — 让 MARSIM 的无人机用 Nav2 做导航与避障

本包把 **MARSIM**（点云级 LiDAR 无人机仿真）接到 **Nav2** 上：MARSIM 提供世界与传感器，
Nav2 提供全局规划与局部避障，`quad_pid` 的 **Twist 模式**把 Nav2 的速度指令变成电机 RPM。

本包只做两件 MARSIM 缺失的基础设施：**TF 树**与**静态占据栅格**。桥接层**不存在**——
`quad_pid` 已内置速度模式，直接订阅 Nav2 的话题。

---

## 1. 依赖与前置

- ROS 2 **Jazzy**、nav2、`quad_pid`（含 Twist 模式）、MARSIM（`map_generator` / `local_sensing` / `mars_drone_sim`）
- ⚠️ **修改 `config/*` 后必须 `colcon build`**：`setup.py` 会把 `config/` 装到 `share/`，launch 读的是已安装副本。改完直接跑会毫无效果（已踩过一次）。

---

## 2. 架构与数据流

```
[一次性建图]
  map_generator ─ /map_generator/global_cloud ─┬─► local_sensing（渲染用）
                                               └─► occupancy_grid_node ─► maps/forest.pgm/.yaml

[运行时]
  map_server ─ /map ────────────────────────────► global_costmap.static_layer
  local_sensing ─ /cloud (world, ~3.6-8.5Hz) ───► costmap.obstacle_layer ─► inflation
  quadrotor_dynamics ─ /odom ─┬─► marsim_tf_broadcaster ─► odom→base_link
                              ├─► quad_pid（当前位姿）
                              └─► Nav2（TF 定位）
  RViz "Nav2 Goal" ─► navigate_to_pose 动作 ─► bt_navigator
                                              └─► planner_server (Smac-2D) ─► /plan
  controller_server (MPPI-Omni) ─ /cmd_vel_nav (body 系) ─► quad_pid[control_mode=velocity]
  quad_pid: SetpointResolver 积分+leash → L1(含速度前馈) → L2 → L3 → L4
  quad_pid ─ /cmd_RPM ─► quadrotor_dynamics ─► 新位姿（闭环）
            └─► /quad_pid/setpoint（可视）
```

**TF 树**（MARSIM 自己不发布任何 TF，只发 `/odom` 消息）：

```
map ──静态恒等──► odom ──动态(来自 /odom)──► base_link ──静态──► lidar_link
 └──静态恒等──► world        （world 帧的点云靠这条边才可变换）
```

仿真中 `map ≡ odom ≡ world`（里程计为真值），所以 **不使用 AMCL**。

---

## 3. 一次性建图

```bash
cd ~/pi-cwd-20260922/marsim_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

MAPS=$PWD/src/marsim_nav/maps
PCD=$(ros2 pkg prefix map_generator)/share/map_generator/resource/small_forest01cutoff.pcd

ros2 run map_generator map_pub "$PCD" &      # 等待打印 "Map point size = ..."
sleep 5
ros2 run marsim_nav occupancy_grid_node --ros-args \
  -p output_dir:=$MAPS -p output_name:=forest \
  -p z_min:=0.4 -p z_max:=1.6 -p inflation_radius:=0.0
```

**`z_min/z_max` 必须等于无人机的飞行高度带**（见 §7）。`inflation_radius` 保持 **0.0**：
Nav2 的 `inflation_layer` 会在运行时自己膨胀，建图时再膨胀就是**双重膨胀**。

---

## 4. 启动

```bash
cd ~/pi-cwd-20260922/marsim_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch marsim_nav sim_navigation.launch.py
```

约 **45 秒**就绪（先起仿真约 8s，再起 Nav2：`map_server` 先激活，延迟 8s 才拉 10 个导航节点）。

**发起导航**：RViz 工具栏选 **「Nav2 Goal」**，在地图上点目标并**拖动定朝向**。

> ⚠️ **不要用「2D Goal Pose」**。它发到 `/goal_pose`，而 **Nav2 不订阅该话题**（`nav2_bt_navigator`
> 的编译产物里根本不出现 `goal_pose`），点了完全没反应，看起来像"导航坏了"。
> 真正的工具是 `nav2_rviz_plugins/GoalTool`，它发送 `navigate_to_pose` **动作**。

---

## 5. RViz 里能看到什么

| 显示项 | 话题 | 颜色 | 说明 |
|---|---|---|---|
| Static Map (forest) | `/map` | 灰 | 静态占据栅格（**未膨胀**的真实障碍） |
| Local / Global Costmap | `/local_costmap/costmap`、`/global_costmap/costmap` | costmap 配色 | 代价图（含障碍与膨胀） |
| LiDAR Cloud (World) | `/cloud` | 按 intensity | 实时点云（world 帧） |
| **Global Plan** | `/plan` | **绿** | 全局路径 |
| **MPPI Local Trajectory** | `/optimal_trajectory` | 青 | 局部规划器选中的轨迹（需 `visualize: true`） |
| **quad_pid Setpoint** | `/quad_pid/setpoint` | 黄箭头 | **控制器实际在追的那个点** |
| Navigation 2 面板 | — | — | 导航状态与按钮 |

### 两条容易误解的现象

1. **绿色路径"很小"或看不到。**
   `/plan` **只在成功规划之后才存在**——没发目标时该话题根本不存在，不是显示坏了。
   而且 Nav2 每个规划周期都**从无人机当前位置重规划到目标**，所以越接近目标，这条绿色
   路径就越短。想看完整路径，在刚发目标时看。

2. **`/quad_pid/setpoint` 是最有用的调试锚点。**
   它把"指令错"和"跟踪错"分开：
   - `/cmd_vel_nav` 有数据但这个点不动 → `cmd_vel_topic` 配错（**启动时只读**，需重启节点）
   - 这个点在动但无人机不动或乱飞 → 控制器侧问题，用 §8 的 `twist_test` 隔离

### 为什么没有 "LiDAR Sensor Cloud"

`/sensor_cloud` 的 `frame_id` 是 `"/sensor"`（MARSIM `opengl_render_node.cpp:261` 写死，还带前导斜杠），
而本包 TF 树里没有该帧，RViz 无法变换到固定坐标系 `world`，会报 status error。世界系的 `/cloud`
已足够调试，故不显示。

---

## 6. `cmd_vel` 链路与 QoS

Nav2 的 remap 链（`nav2_bringup/launch/navigation_launch.py`，已核对安装态源码）：

```
controller_server  ──cmd_vel → cmd_vel_nav──► /cmd_vel_nav
velocity_smoother  ──in cmd_vel_nav, out cmd_vel_smoothed──► /cmd_vel_smoothed
collision_monitor  ──in cmd_vel_smoothed, out cmd_vel──► /cmd_vel
```

**`quad_pid` 订阅 `/cmd_vel_nav`**（控制器原始输出），即**绕过** velocity_smoother 与
collision_monitor。理由：`quad_pid` 自带 leash、加速度限幅与速度调速器。

想启用平滑与碰撞监控，把 `quad_pid_nav.yaml` 的 `cmd_vel_topic` 改为 `/cmd_vel`
（**该参数启动时只读，必须重启节点**）。

QoS：`quad_pid` 侧为 **BEST_EFFORT**（可兼容 Nav2 的 RELIABLE 发布者；反之 RELIABLE
订阅者会从 BEST_EFFORT 发布者**静默收不到**）。核对：

```bash
ros2 topic info -v /cmd_vel_nav     # Subscription 里应出现 quad_pid_node
```

---

## 7. 高度：当前是「2.5D + 固定高度」，不能动态变高

**Nav2 是 2D 的**，四个环节都锁死：

| 环节 | 现状 | 后果 |
|---|---|---|
| 代价图 | `costmap_2d` 单张 XY 栅格 | z 仅用于**过滤**哪些点算障碍，之后丢弃。无法表达"1m 堵、3m 通" |
| 高度带 | `[0.4, 1.6]` 固定在**世界系** | 不跟随无人机 |
| 规划器 | `SmacPlanner2D` | 路径只有 XY；`NavigateToPose` 的 `z` 被忽略 |
| 高度来源 | `quad_pid.twist_target_height` | Nav2 **从不**下达 z；进入速度模式时**锁存**当前高度并保持 |

> Jazzy 的 Nav2 **没有 3D 规划器**（只有 `SmacPlanner2D`/`Hybrid`/`Lattice`）。
> `VoxelLayer` 名字里有 voxel，但只用于更正确的标记/清除，**输出仍是 2D 代价图**。

### ⚠️ 改高度的陷阱（静默、不报错）

只改 `twist_target_height` 而不同步高度带，代价图会停留在 0.4~1.6m，于是无人机在 3m 飞时
**固执绕开本可从上方飞过的东西，同时对 3m 处的障碍完全瞎**。三处必须同时改：

```bash
# 1) quad_pid/config/quad_pid_nav.yaml
twist_target_height: 3.0

# 2) marsim_nav/config/nav2_params.yaml —— local_costmap 与 global_costmap 两处
min_obstacle_height: 2.4     # 3.0 - 0.6
max_obstacle_height: 3.6     # 3.0 + 0.6

# 3) 用同一高度带重建静态图（见 §3，z_min/z_max 同上）
colcon build --packages-select marsim_nav
```

`twist_target_height` 默认 `-1.0` = **锁存**进入速度模式时的高度，这使高度带与飞行高度
自动一致、不会脱节——所以默认档最省心。

**要真正的 3D 避障**，Nav2 这条路走不通，需要换 3D 栈（Ego-Planner / Fast-Planner +
OctoMap/ESDF）。可复用的是 MARSIM 仿真、本包的 TF 树，以及 `quad_pid` 的 Twist 模式
（`twist_follow_z: true` + `twist_max_vz` 已是现成的 3D 速度接口）。

---

## 8. 参数耦合（改一侧必须同步另一侧）

| Nav2 (`nav2_params.yaml`) | quad_pid (`quad_pid_nav.yaml`) | 现值 |
|---|---|---|
| `FollowPath.vx_max` / `vy_max` | `twist_max_vx` / `twist_max_vy` | 0.5 / 0.5 |
| `FollowPath.wz_max` | `twist_max_wz` | 0.6 |
| — | `max_horiz_speed`（**达到**速度上限） | 0.5 |
| — | `max_horiz_dist`（**兼作 Twist 的 leash**） | 1.0 |

- **速度为什么是 0.5 而不是 1.4**：`quad_pid` 位置环 `ωn ≈ 0.7~1.0 rad/s`，1.4 m/s 时实测
  **冲过目标约 0.9m 并落入障碍格**，规划器随即以 `Start occupied` 中止。降到 0.5 后同一
  目标由 ABORTED 变 SUCCEEDED，最近障碍间距 0.10m → 0.28m（未膨胀口径 0.61m）。
- **`max_horiz_dist` 是 leash**：默认 5.0m 在密林里太长，参考点跑到机体前方 5m，位置 PD
  猛推着切内弯。收到 1.0m 让设定点紧贴机体、贴着全局路径飞。
- **障碍高度带 `[0.4, 1.6]`**：与巡航 1.0m 绑定（见 §7）。
- inflate 半径 `1.0`、`robot_radius 0.35`、分辨率 `0.1`。

---

## 9. 排障

| 症状 | 先查 |
|---|---|
| RViz 点目标无反应 | **用的是「Nav2 Goal」工具吗？**（「2D Goal Pose」不驱动 Nav2） |
| 导航栈没起来 | `ros2 lifecycle get /{bt_navigator,controller_server,planner_server,map_server}` 应全 `active` |
| 无人机不动，Nav2 日志正常 | `ros2 topic info -v /cmd_vel_nav` 的 Subscription 里有没有 `quad_pid_node` |
| 代价图没有障碍物 | `odom→world` TF 在不在（`ros2 run tf2_ros tf2_echo odom world`）；`/cloud` 有没有数据 |
| 规划器报 `Start occupied` | 无人机已冲进障碍格（多半是速度过快，见 §8）；或目标点落在障碍里 |
| 无人机贴障碍 | 检查高度带是否与飞行高度一致（见 §7） |
| 改了参数没效果 | **`colcon build` 了吗**（§1） |

### 控制器侧隔离（不依赖 Nav2）

`quad_pid` 自带注入器，用同一控制器绕开 Nav2，一眼区分"控制器坏"还是"Nav2 配置坏"：

```bash
ros2 launch quad_pid twist_test.launch.py pattern:=forward vx:=1.0 duration:=5
ros2 launch quad_pid twist_test.launch.py pattern:=forward vx:=1.0 yaw:=-54   # 绕圈 bug 回归
ros2 launch quad_pid twist_test.launch.py pattern:=strafe vy:=1.0
```

### ⚠️ 进程卫生（重要）

多次反复启动必须清干净。`pkill -f <名字>` 默认只发 SIGTERM 且**不杀 `ros2 launch` 父进程**，
子节点会持续存活。累积后会看到**负载飙升**，制造大量假故障：Nav2 服务响应超时、
lifecycle 节点卡在 `unconfigured`（而参数其实是好的）。

```bash
pkill -f "ros2 launch"; pkill -9 -f "ros2 run"
pkill -9 -f rviz2; pkill -9 -f opengl_render_node
pkill -9 -f "nav2_"; pkill -9 -f quad_pid_node
pkill -9 -f marsim_tf_broadcaster
```

> 教训：排查中曾累积 8 个 `ros2 launch` 父进程、4 个 RViz、5 个 OpenGL 渲染器，**负载 24+**，
> 一度把服务超时误判为 Fast-DDS 共享内存问题（**错误结论**）。清干净后不带任何环境变量
> 即稳定 11/11。

---

## 10. 文件与验证

```
marsim_nav/
├── marsim_nav/frames.py             # 帧名常量
├── marsim_nav/tf_broadcaster.py     # TF 树
├── marsim_nav/map_projector.py      # 点云→占据栅格（纯函数，6 个单测）
├── marsim_nav/occupancy_grid_node.py# 一次性建图
├── config/nav2_params.yaml          # Nav2 参数（Smac-2D + MPPI-Omni）
├── config/marsim_nav.rviz           # RViz 配置（含 Nav2 Goal 工具）
├── launch/tf.launch.py
├── launch/navigation.launch.py      # TF + map_server + Nav2（假设仿真已在跑）
└── launch/sim_navigation.launch.py  # 一把梭全栈
```

单元测试：

```bash
cd src/marsim_nav && python3 -m pytest test/ -q      # 6 passed
cd src/quad_pid   && python3 -m pytest test/ -q      # 88 passed
```

集成验证（安静机器、无 RViz 时最稳）：

- 11/11 lifecycle 节点 `active`
- `/cmd_vel_nav` 订阅者含 `quad_pid_node`
- `local_costmap` 数百个占据栅格，`frame=odom`
- 高度全程锁定（`z_min == z_max`）
- 短距导航 SUCCEEDED，`goal_err ≈ 0.22m`
