# Sim2Real 缺口报告：第一步（M1 装入 Base）

运行方法见 [README.md](README.md)。这里记录真机上**还没有或还没验证**的部分，按优先级排列。

## 0. 已提供、已离线验证的部分

- harness 控制接口 `real/interface.py`，以及把它接到 harness 的 `real/backend.py`；
- 启动器、开机检查、操作员工位信号、RealSense 相机实现；
- 一键 pipeline `pipeline/step1_real.sh`。

**验证范围：** 只在 `real/mock.py`（用孪生模拟的硬件）上跑通过，内容如下。
- smoke agent 完整回合：3 个动作组、75 个控制步，观测、深度文件和工位信号格式与仿真一致。
- 开机检查：叠加图的投影正确，试运动的跟踪误差约为 0。
- 柔顺推入：自由空间里沿 −z 推 3 cm，到位误差 < 0.1 mm，转角 0.004°。
- 适配器没实现柔顺接口时，会退化为位置控制并打警告。

**没有验证：** 真机硬件；用模型（Astra）在 `RealBackend` 上跑完整回合；`real/realsense.py`；cuRobo 本地模式。

## 1. 适配器（必须由你们写）

用你们改过的 SDK 实现 `RobotIO`，模板在 `real/adapter_template.py`。

**关节约定。** 关节零位和正负号要和孪生 URDF 一致。孪生的起始姿态是全 0，关节 2、3 的下限都是 0。用 `pipeline/step1_real.sh check` 生成的叠加图验证。

**保持。** 两次调用之间要一直保持位置（重力补偿加 PD）。模型一轮思考可能要几分钟。

**速度。** 执行器给出的关节速度上限是 2 rad/s。`SLOWDOWN=2` 时真机最快 1 rad/s。

## 2. 柔顺推入（必须实现，插入前）

插入的最后一段用 plan 里的 `compliance` 字段：沿 `press_direction` 推，力不超过 `max_force_n`（第一步用 10 N），侧向顺从，让零件自己对中。仿真里是关节力矩控制下的笛卡尔阻抗，控制律和增益见 `real/compliance.py`：

```
F_a = clip(K_a·e_a − D_a·v_a, −F_max, F_max)        # 沿推入方向，K_a = 20000 N/m，D_a = 200
F   = F_a·axis + K_l·e_lat − D_l·v_lat              # 侧向，K_l = 250 N/m，D_l = 25
M   = K_r·rotvec(R_d Rᵀ) − D_r·ω                    # 转动，K_r = 150，D_r = 1.1
τ   = Jᵀ[F; M] + g(q) − 1.0·q̇ + 零空间姿态项
```

没有力传感，只用指令力矩和关节状态。后端每个控制步把 TCP 参考位姿、推入轴、力上限、当前雅可比和 TCP 位姿传给 `command_compliant`。

真机上要注意：
- 关节摩擦会吃掉一部分推力，实际最大推力要实测（比如推一台秤）。
- 侧向能顺从多少也要实测。
- 增益可能要按真机重调。

## 3. "已紧固"信号（已用人工确认代替）

仿真里工位替代螺丝：每个动作结束时检查 M1 是否在门限内到位，到位就固定，观测里给 `station.fastened = true`。agent 靠这个信号决定何时松手。

真机上由 `real/station.py` 实现：每个带柔顺推入的动作结束后，在终端问操作员。也可以 `touch <run>/FASTENED`。

判定标准建议统一，比如 M1 顶面与 Base 顶面平齐、不倾斜。如果以后改成视觉或卡尺自动测量，替换 `OperatorStation.after_action` 即可。

## 4. 视觉精度（真机会明显变差）

**仿真的上限。** 仿真里相机标定精确、深度无噪声。即便如此，ICP 在最好条件下的误差约 0.18 mm / 0.09°（第 6 步测试，零件大且无遮挡）。横向门限只有 0.25 mm。

**真机的现状。** 深度有噪声，标定有误差。之前的实验室标定记录里，平面距离误差最大 6.3 mm，中位残差 1.4 mm，比门限大一个量级。

**含义：**
- 真机上不能只靠视觉对准，要靠柔顺推入（第 2 项）加 Base 槽口的导向自己对中。槽口约有 0.95 mm 余隙。
- 方盒形舵机看起来对称，ICP 会估成转了 180°。要用 `pose_tools.flip_check` 排查。
- D405 近距离深度较好，头顶相机在 1 m 外，深度误差更大。

## 5. 零件外观与任务文本

任务卡、技能和 agent 的颜色分割都假设"蓝色 Base、橙色 M1"。真实 STS3215 是黑色，黑色在深度和颜色上都更难分割。

两种做法：
- 给 M1 贴橙色胶带或贴纸（推荐，任务文本不用改）；
- 修改 `q0_harness/assets/tasks/task_cards.json` 和第一步技能里的颜色描述。

CAD 包 README 里的 "In the simulator ..." 字样不影响使用。

## 6. 夹爪（需要实测）

- 仿真里夹爪只有开、合、保持三种指令；按宽度开合的指令会被拒绝。适配器里把 1/0 映射成你们夹爪的开、合位置，并设定夹紧力。
- 打印的夹指和 URDF 外形不一致，抓取点常量（`controller/robot_geometry.py` 里的 `GRASP_POINT_OFFSET_M`、`FINGERTIP_REACH_M`）最好按真机夹指核对。
- 仿真里 M1 抓起后会在手里转动或滑动，所以 agent 抓起后要重新拟合位姿。真机摩擦不同，表现会不同。

## 7. 其他要注意的

- **数值来自仿真。** 经验和技能里的抓取点、推入超程 2–3 mm、Base 滑动约 1.7 mm、需要第二次推入，都是仿真数值。方法可以沿用，数值要按真机首轮结果更新。真机回合目前不会自动写进经验库。
- **桌面摩擦不同。** 推入时 Base 会滑动，靠左手按住；真机桌面材料不同，滑动量也会不同。
- **"home" 的定义。** 回合开始时两臂所在的位置就是 home，agent 结束时要回到这里。
- **只用于仿真的模块。** `q0_harness/adapters/` 里的 `mujoco_backend.py`（`real/backend.py` 复用其中的接口逻辑）、`so101_station.py`、`model_loading.py`、`replay.py`、`pod_render.py`、`pod_curobo.py`、`codex_subscription.py`，以及 `scripts/run_q0_episode.py`。
