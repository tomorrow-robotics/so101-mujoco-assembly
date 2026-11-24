# SO-101 Sim2Real：第一步（M1 装入 Base）

两台 Panthera-HT 六轴臂用原厂平行夹爪，在普通桌面上把 SO-101 的第一个舵机 M1（STS3215）装进 Base。控制它们的是 LLM agent Astra（`gpt-6-astra`，基于 XPolicyLab Q0-Preview 改造）。这一步在 MuJoCo 里已经从散件开始连续成功 3 次（见文末），这个目录把它搬到真机上。

**你们要写的只有一个适配器**：用你们自己的 SDK 实现 `real/interface.py` 里的 `RobotIO`（读关节、跟随关节目标、夹爪，以及插入用的柔顺推入）。其余部分都已接好并在模拟硬件上测过：agent、动作协议、执行器、IK 和路径规划、双臂 2 mm 防碰撞、相机观测和深度文件、"已紧固"信号、记录。真机上还没验证的地方见 [REPORT.md](REPORT.md)。

## 一、先准备好

### 1. 硬件和场地
- **两台 Panthera-HT。** 底座位姿要和孪生模型 `twin/twin.xml` 一致（左臂 = 仿真里的 a 臂，带 1868 号手腕相机；右臂 = b 臂，带 1416 号）。孪生模型来自你们实验室的标定；如果重新标定或挪了臂，按下文"更新孪生模型"改。
- **三台 RealSense。** 头顶一台（261822072968），两只手腕各一台 D405（262422271868、262422271416），640×480。序列号不同可以用 `CAMERA_ARGS` 指定。
- **零件。** Base 按 `parts/source/` 里的 Base_SO101 v12 打印（`Base_SO101.step` 或 `print-base.stl`，仿真用的就是 v12），M1 用 STS3215。任务文本、技能和 agent 的颜色分割都按"蓝色 Base、橙色 M1"写；真实 STS3215 是黑色的，最好给 M1 包一层橙色胶带或贴纸，否则要改任务卡和技能里的颜色描述。
- **摆放。** 大致按 `layout/step1_sim_initial_layout.json`：Base 在 (−0.095, 0.345) m、yaw 90°；M1 在 (0.15, 0.245) m、yaw 135°（桌面坐标系）。agent 会从图像自己找零件，不需要精确。
- **起始姿态。** 两臂所有关节为 0，夹爪张开。仿真回合都从这里开始，经验里的可达范围也是按这个测的。
- **急停** 在手边。Base 第一步不固定在桌上，由左手按住。

### 2. 软件环境（控制机，Linux x86_64）
```bash
pip install mujoco==3.9.0 numpy scipy pillow pyyaml pyrealsense2
# agent 的 Python 工具沙箱：一个独立前缀，挂进 bubblewrap，只读、无网络
conda create -y -p /opt/q0_tools python=3.11 && /opt/q0_tools/bin/pip install -r q0_harness/requirements.txt
sudo apt install bubblewrap
```
- 模型：`real/agent.real.example.json` 里改 `base_url`（你们的 OpenAI 兼容端点）和 `python_prefix`，密钥放在环境变量 `Q0_PREVIEW_API_KEY`。
- cuRobo 可选（`PLANNER=curobo`，需要 GPU 和 cuRobo commit 895c651）。默认的 `mujoco` 规划器只用 CPU。
- 只有 mock 阶段需要渲染：有 GPU 用 `MUJOCO_GL=egl`，否则 `MUJOCO_GL=osmesa`。

### 3. 写适配器（必须）
照 `real/adapter_template.py` 写一个 `RobotIO` 子类，接你们的 SDK：

| 方法 | 要做的事 |
|---|---|
| `read_joints()` | 两臂 6 个关节的实测角度（rad），零位和正负号与孪生 URDF 一致 |
| `command(q, gripper, duration_s)` | 在 `duration_s` 内从上一个目标平滑过渡到 `q` 并保持；夹爪 1 = 开，0 = 合。之后必须一直保持住（模型思考可能要几分钟），所以要在你们自己的高频循环里做重力补偿加关节 PD |
| `command_compliant(...)` | 插入最后一段的力限推入：沿推入方向推力不超过 `max_force_n`（第一步是 10 N），侧向放软让零件自己对中，姿态保持住。参考实现 `real/compliance.py`（就是仿真里用的控制律），`real/mock.py` 里有跑通的例子。不实现时退化为硬位置控制并打警告，**不要用它在真机上做插入** |
| `read_gripper()`、`stop()`、`close()` | 可选：实测开口、安全停住、断开连接 |

遇到跟踪误差过大、过流、超限位、通信中断等故障，直接抛异常即可，回合会立即结束并记录。

## 二、怎么跑

```bash
cd sim2real
export ROBOT=your_package.your_module:YourRobot        # 你们的 RobotIO 适配器
export ROBOT_ARGS='{"left_config": "...", "right_config": "..."}'   # 可选，传给构造函数
pipeline/step1_real.sh mock     # 1. 不接机器人：模拟孪生 + 免 API 的 smoke agent，先确认环境没问题
pipeline/step1_real.sh check    # 2. 只读：读关节、拍三张图，把孪生的关节和夹爪点画到图上
pipeline/step1_real.sh move     # 3. 小幅试运动：每臂关节 1 ±0.05 rad，夹爪开合，报告跟踪误差
pipeline/step1_real.sh smoke    # 4. 整个闭环接真机，但只用 smoke agent（原地空动作），不调模型
Q0_PREVIEW_API_KEY=... pipeline/step1_real.sh run    # 5. Astra 正式做第一步
```

每一步的输出在 `sim2real/runs/<阶段>_<时间>/`。进入下一步前要满足：

| 阶段 | 通过标准 |
|---|---|
| mock | 两条命令都跑完；`mock_check_*/overlay_*.png` 里的点落在渲染出的手臂上 |
| check | `overlay_*.png` 里红点（TCP）、黄点（指垫）、青点（各连杆原点）落在**真实**夹爪和连杆上。整体偏移说明标定有误差；点落在手臂的错误位置，多半是关节零位或方向和孪生不一致 |
| move | 动作平稳，`check.json` 里的稳定跟踪误差小（建议 < 0.01 rad） |
| smoke | 回合正常结束（`RESULT.json` 里 `end.reason = integration_smoke_complete`），手臂全程不乱动 |
| run | 见下 |

**正式运行时操作员要做的：**
- 每次带柔顺推入的动作结束后，终端会问 "Is M1 fully seated in the Base?"，确认到位就答 `y`。agent 看到 `station.fastened = true` 后会松手撤离。也可以在另一个终端 `touch runs/<run>/FASTENED`。
- 想停：`touch runs/<run>/session/STOP`（agent 正常收尾），或 Ctrl-C（立即停住手臂）。
- 结束时终端会问几项结果（是否到位、是否松手撤离、可选的高度测量），写进 `RESULT.json`。

**可调参数**（环境变量）：`SLOWDOWN=2`（真机运动速度是仿真的 1/2，agent 看到的时间不变）、`PLANS=30`（规划轮数）、`PLANNER=mujoco|curobo`、`CAMERAS` / `CAMERA_ARGS`（换相机实现或序列号）、`AGENT_CONFIG`。

**输出文件：** `RESULT.json`（操作员判定与故障）、`trajectory.npz`（每步的目标和实测关节、夹爪、是否柔顺）、`session/`（agent 的全部观测、回复、工具输出和对话）、`final_*.png`。

## 三、更新孪生模型（重新标定后）
`twin/twin.xml` 是只含两台臂、桌面和相机的 MuJoCo 模型，从成功仿真回合的场景里去掉零件得到（来源见 `twin/provenance.json`）。它负责正运动学、IK、碰撞检查和手腕相机外参。标定变了就改这几处：
- `a_base_link` / `b_base_link` 的 `pos` 和 `quat`：两臂底座在桌面坐标系下的位姿（左 = a，右 = b）；
- `camera name="agent"`：头顶相机的位姿和内参；
- `wrist_1868` / `wrist_1416`：手腕相机相对 link6 的安装位姿；
- `lab_table`：桌面（顶面应在 z = 0）。

如果你们自己有相机外参，也可以不改孪生：在 `CameraIO.capture()` 里给每台相机返回 `camera_to_world`（OpenGL 轴：+X 右、+Y 上、−Z 前），后端会直接用它。

## 四、目录

| 路径 | 内容 |
|---|---|
| `pipeline/step1_real.sh` | 一键分阶段运行 |
| `real/interface.py` | **harness 控制接口**：`RobotIO`、`CameraIO` |
| `real/adapter_template.py` | 适配器模板 |
| `real/compliance.py` | 柔顺推入的参考控制律和仿真用的增益 |
| `real/backend.py` | 把接口接到 harness：硬件执行，孪生负责运动学和碰撞 |
| `real/run_real_episode.py`、`real/check_setup.py` | 回合启动器、开机检查（叠加图、试运动） |
| `real/station.py` | 操作员确认的"已紧固"信号和结果记录 |
| `real/realsense.py` | 三台 RealSense 的 `CameraIO` 实现 |
| `real/mock.py` | 用孪生模拟硬件，供离线测试 |
| `real/agent.real.example.json` | agent 配置模板 |
| `twin/` | 孪生模型（两臂碰撞网格、标定过的相机） |
| `parts/` | Base v12、STS3215 的 CAD 和装配位姿 |
| `layout/` | 仿真里的初始摆放 |
| `q0_harness/` | 改造后的 Q0 harness，以及第一步的 CAD 包、任务卡、prompts、经验、技能、位姿工具 |
| `third_party/` | 上游 Q0-Preview 快照（commit `6baccd5`，Apache-2.0）和共享工具 |
| `scripts/` | 仿真启动器（参考）和 cuRobo 服务 |
| `docs/q0_preview/` | harness 的迁移报告和运行手册 |

## 五、仿真中的结果（参考）

| 回合 | 横向 | 轴向 | 角度 | 用到的规划轮数 |
|---|---|---|---|---|
| E1 | 0.120 mm | 0.526 mm | 0.727° | 11 |
| G1 | 0.156 mm | 0.528 mm | 0.716° | 12 |
| H1 | 0.186 mm | 0.521 mm | 0.732° | 14 |

验收门限：横向 0.25 mm；轴向在 −0.05 到 0.6 mm 之间；角度 0.75°。

流程是：左手按住 Base，右手抓起 M1，沿 Base 的 +Y 方向以 10 N 力上限推入，目标设在 CAD 深度再往里 2–3 mm。Base 会在桌上滑约 1.7 mm，通常要推第二次。agent 拿不到任何物体真值位姿，只能用三台 RGB-D 相机加 CAD 自己估计。

## 六、不包含的内容与许可
- 不含仿真回合录像、本地模型网关（换成你们自己的端点），也不含 cuRobo 源码（只记 commit `895c651`）。
- `third_party/q0_preview` 及改造自它的 `q0_harness` 遵循 Apache-2.0。SO-101 CAD 与 STS3215 模型版权归原作者（TheRobotStudio / Feetech）。孪生里的机械臂网格来自 Panthera-HT SDK。
