# Sim2Real 缺口报告：第一步（M1 装入 Base）

这里列出仿真中能跑、真机上**现在还没有**的部分，按优先级排列。每一项写明仿真里是怎么做的、真机需要什么。

## 1. 真机后端（必须新写）

Harness 本身（runner、协议、executor、motion、规划器）和硬件无关，所有硬件交互都通过一个后端对象完成。仿真用的是 `q0_harness/adapters/mujoco_backend.py` 里的 `NativeMujocoBackend`。上游 `controller/backend.py` 原本是一个远程后端，接收外部环境发来的观测，但它已被禁用（构造函数第一行直接 raise）。

真机后端至少要实现 `NativeMujocoBackend` 的这些接口：

| 类别 | 方法 | 真机要做的事 |
|---|---|---|
| 状态 | `joints()`、`grippers()`、`gripper_commands()`、`pose(arm)`、`fk(arm, q)` | 读编码器；正运动学用真机 URDF |
| 执行 | `step(q, g)` | 按 25 Hz（`control_dt` 0.04 s）下发两臂 6 个关节目标和夹爪指令 |
| 执行 | `hold_current()`、`cancel()`、`ended()`、`fail()` | 保持当前位置、急停、结束 |
| 柔顺 | `step_compliant(...)`、`end_compliance()`、`attach_executor(...)` | 见第 2 项 |
| 规划 | `ik`、`ik_candidates`、`plan_free`、`alternative_paths`、`planner_target_pose`、`nearest_joint_equivalent` | 可沿用 cuRobo（`controller/curobo_planner.py`），模型换成真机标定后的版本 |
| 碰撞 | `validate_pair`、`arm_world_collision`、`set_collision_grippers`、`gripper_estimates` | 双臂互碰和臂–桌碰撞检查，桌面高度和两臂底座按真机标定 |
| 预算 | `sim_time`、`remaining_steps`、`budget_state` | 计时和步数预算（`control_step_limit` 7500） |
| 观测 | `observation()` | 见下文 |

`observation()` 要生成 harness 期望的字段，格式参照仿真观测：
- `state.left/right`：`grasp_point`、`tool_axis`、`jaw_axis`、`gripper_pose_points`、`fingertip_center`、`commanded_gripper`、离 home 的误差，等等。
- `calibration.<cam>`：`K`、`camera_to_world`，约定为 USD/OpenGL（+X 右、+Y 上、−Z 前）。
- 三台相机的 RGB（`cam_head`、`cam_left_wrist`、`cam_right_wrist`，640×480）。
- 对齐到彩色图的深度：单位米，以 `.npy` 文件路径给出（`depth.files`），agent 的 `pose_tools` 靠它做 ICP。
- `station`：见第 3 项。
- `last_execution`：executor 自己生成，不用改。

仿真里用的是头顶一台 RealSense 加两只手腕各一台 D405。真机配置如有不同，需要相应修改相机名和 prompt 里的描述。

## 2. 柔顺推入（必须解决）

插入的最后一段用 group 的 `compliance` 字段：沿 `press_direction` 推，力不超过 `max_force_n`（第一步用 10 N）；侧向顺从，让零件自己对中。仿真里的实现见 `mujoco_backend.py:_impedance_torque`，是关节力矩控制下的笛卡尔阻抗：

```
F_a = clip(K_a·e_a − D_a·v_a, −F_max, F_max)        # 沿推入方向，K_a=20000 N/m，D_a=200
F   = F_a·axis + K_l·e_lat − D_l·v_lat              # 侧向，K_l=250 N/m，D_l=25
M   = K_r·rotvec(R_d Rᵀ) − D_r·ω                    # 转动，K_r=150，D_r=1.1
τ   = Jᵀ[F; M] + 重力/科氏补偿 − 1.0·q̇ + 零空间姿态项
```

没有力传感，只用指令力矩和关节状态。真机上的选择：
- Panthera-HT 支持力矩或电流模式：直接照搬上式，需要重力补偿模型。
- 只有位置模式：用导纳控制近似，靠电流或关节误差估计接触力，并限制最大推入深度。
- 都不行：只能位置控制推入，风险是卡死或把 Base 推走。

不管哪种，都要先在真机上标定实际最大推力，以及侧向能顺从多少。

## 3. "已紧固"信号（需要一个替代，粗略即可）

仿真里工位替代螺丝：每个动作结束时检查 M1 是否在门限内到位（横向 ≤ 0.25 mm，轴向 −0.05 到 0.6 mm，角度 ≤ 0.75°），到位就加一个焊接约束，观测里给 `station.fastened = true`。Agent 依赖这个信号决定何时松手（技能 `station-fastened-then-release`）。

真机上 M1 只是放进槽里，没有自动紧固。可选替代：
- 人工判定后按键，回填 `station.fastened`。
- 用视觉或卡尺测量是否到底，比如 M1 顶面到 Base 顶面的高度差。

还要相应修改任务卡里描述工位的那一句。

## 4. 视觉精度（真机会明显变差）

- 仿真里相机标定精确、深度无噪声。即便如此，ICP 在最好条件下的误差约 0.18 mm / 0.09°（第 6 步的测试，零件大且无遮挡）。横向门限只有 0.25 mm。
- 真机深度有噪声，标定也有误差。之前的实验室标定记录里，平面距离误差最大 6.3 mm，中位残差 1.4 mm，比门限大一个量级。
- 所以真机上不能只靠视觉对准，要靠柔顺推入（第 2 项）加 Base 槽口的导向自己对中。槽口约有 0.95 mm 余隙。
- 方盒形舵机看起来对称，ICP 会估成转了 180°。要用 `pose_tools.flip_check` 排查，再用可区分的特征（舵盘在 +X 端）确认。

## 5. 夹爪（需要实测）

- 仿真里夹爪只有开、合、保持三种指令；按宽度开合的指令被拒绝，因为垫片间隙和指令的对应关系还没验证。真机要测这条对应关系。
- 打印的夹指和 URDF 外形不一致，碰撞检查和抓取点（`GRASP_POINT_OFFSET_M`、`FINGERTIP_REACH_M`，见 `controller/robot_geometry.py`）要按真机夹指重新量。
- 夹紧力、指面摩擦都没有标定过。仿真里 M1 被抓后会在手里转动或滑动，所以 agent 抓起后要重新拟合位姿。

## 6. 只在仿真中使用的模块（真机替换或删除）

| 文件 | 作用 | 真机 |
|---|---|---|
| `adapters/mujoco_backend.py` | 仿真后端 | 用第 1 项的真机后端替换 |
| `adapters/so101_station.py` | 仿真工位和评估器（用真值），依赖仿真仓库里的 `run_assembly.pose_error` | 用第 3 项替换 |
| `adapters/model_loading.py`、`replay.py`、`mujoco_cameras.py` | 加载 MuJoCo 模型、回放、仿真相机 | 不需要 |
| `adapters/pod_render.py`、`pod_curobo.py` | 在远端 GPU 机器上渲染、跑 cuRobo | 不需要；cuRobo 可在本机 GPU 跑 |
| `adapters/codex_subscription.py` | 通过本地网关访问模型 | 换成自己的 OpenAI 兼容端点 |
| `station_queries.py` | 工位查询，在仿真几何上检查位姿和路线 | 要改用真机标定后的机器人模型 |
| `scripts/run_q0_episode.py` | 仿真启动脚本 | 照它的流程写真机启动脚本 |

## 7. 其他要注意的

- **初始摆放**：`layout/step1_sim_initial_layout.json` 只需大致相同。agent 从图像估计位姿，不依赖精确摆放；但两臂的可达范围（`robot_reach.*`）是按仿真底座位姿测的，真机底座位置不同时要重测。
- **经验和技能里的数值**都来自仿真：抓取点、推入超程 2–3 mm、Base 滑动约 1.7 mm、需要第二次推入。真机上方法可以沿用，具体数值要按真机首轮结果更新。
- **桌面摩擦**：推入时 Base 会滑动，靠左手按住。真机桌面材料不同，滑动量也会不同。
