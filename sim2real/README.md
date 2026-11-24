# SO-101 Sim2Real：第一步（M1 装入 Base）

两台 Panthera-HT 六轴臂，用原厂平行夹爪在普通桌面上，把 SO-101 的第一个舵机 M1（STS3215）装进 Base。控制它们的是 LLM agent Astra（`gpt-6-astra`），基于 XPolicyLab Q0-Preview 改造。本仓库收齐了把这一步搬到真机上需要的零件文件和 agent 侧全部内容。真机上**还缺什么**，写在 [REPORT.md](REPORT.md)。

## 仿真中的结果（MuJoCo 3.9.0，从散件开始，单次成功）

| 回合 | 横向误差 | 轴向误差 | 角度误差 | 用到的 plan 数 |
|---|---|---|---|---|
| E1 | 0.120 mm | 0.526 mm | 0.727° | 11 |
| G1 | 0.156 mm | 0.528 mm | 0.716° | 12 |
| H1 | 0.186 mm | 0.521 mm | 0.732° | 14 |

验收门限：横向 0.25 mm；轴向在 −0.05 到 0.6 mm 之间；角度 0.75°。三次都通过，工位的"已紧固"判定都发生在零件真正到位之后。

流程是：左手把 Base 按在桌上（第一步不夹持在桌上，Base 也不允许抬起），右手抓起 M1，沿 Base 的 +Y 方向推入。推入用柔顺模式，力上限 10 N，目标设在 CAD 深度再往里 2–3 mm。Base 会在桌上滑约 1.7 mm，所以通常需要第二次推入。Agent 拿不到任何物体真值位姿，只能从三台 RGB-D 相机的图像加 CAD 自己估计。

## 目录

| 路径 | 内容 |
|---|---|
| `parts/source/` | 原始 CAD：`Base_SO101.step`（Base v12）、`print-base.stl`、`base_so101_v2.stl`、`sts3215_03a_v1.stl`（M1）、`assembly.step`，附 `SHA256SUMS` |
| `parts/cad_report.json` | 两件的尺寸，以及装配位姿 `T_base_motor`（M1 在 Base 系下） |
| `layout/step1_sim_initial_layout.json` | 仿真里的初始摆放（桌面坐标，米）：Base 在 (−0.095, 0.345)、yaw 90°；M1 在 (0.15, 0.245)、yaw 135°。真机只需大致相同 |
| `q0_harness/` | 改造后的 Q0 harness（runner、sandbox、协议、executor、motion、cuRobo 规划器、观测与历史），以及 `adapters/`（仿真后端，真机需替换） |
| `q0_harness/assets/cad/` | 发给 agent 的 CAD 包：Base、M1 在各自体坐标系下的网格，加 `assembly.json` |
| `q0_harness/assets/tasks/task_cards.json` | 任务卡（第一步是 `so101_base_m1_insert`；文件里也有后续步骤，不影响使用） |
| `q0_harness/assets/prompts/` | system prompt、checkpoint prompt、几何上下文 |
| `q0_harness/assets/experience/` | 第一步的 guide 与实测记录（`so101_base_m1_insert_*`）、两臂可达范围（`robot_reach.*`） |
| `q0_harness/assets/experience_snapshots/E1_experience.md` | E1 运行时 agent 实际读到的经验文件 |
| `q0_harness/assets/skills/` | `step1-m1-into-base`，以及它用到的通用技能（扶住 Base、斜向够取、抓起后提起测试、按 CAD 轮廓拟合、深度位姿测量、推到位、紧固后松手、避让另一只手、规划前查可达） |
| `q0_harness/assets/tools/pose_tools.py` | agent 沙箱里的深度 ICP 位姿估计（`estimate_pose`）和 180° 翻转检查（`flip_check`） |
| `q0_harness/station_queries.py` | 工位查询工具（`check_pose` / `check_route`，基于离线几何检查） |
| `q0_harness/config/` | agent 配置示例（模型、沙箱、上下文） |
| `third_party/q0_preview/` | 上游 XPolicyLab Q0-Preview 原始快照，commit `6baccd57ff958570d0db1588370289ee36c8bb70`，Apache-2.0 |
| `third_party/xpolicylab_shared/` | 上游共享工具（图像编码、模型模板） |
| `scripts/run_q0_episode.py` | 仿真里跑一个回合的启动脚本（参考用；依赖完整的仿真仓库） |
| `docs/q0_preview/` | Harness 的迁移报告、运行手册、操作指南 |

## 有意不包含的内容

- **机器人与实验室几何**（臂的底座位姿、相机标定、Panthera-HT SDK 网格）：真机实验室要自己重新标定。
- **仿真回合的录像与日志**（轨迹、观测、请求日志）。
- **本地模型网关** `q0_harness_gateway`：它把 Codex 订阅转成 OpenAI 兼容接口，绑定在 127.0.0.1，依赖本机凭据。真机部署请换成自己的 OpenAI 兼容端点（见 `q0_harness/config/agent.gpt6astra.example.json`，密钥从环境变量 `Q0_PREVIEW_API_KEY` 读取）。
- **cuRobo 源码**：只记录版本，commit `895c6517243f8cb091c73c018c8167192d39599a`（NVlabs/curobo），请自行安装。

## 许可

- `third_party/q0_preview` 以及由它改造的 `q0_harness` 遵循 Apache-2.0（见 `q0_harness/LICENSE`）。
- SO-101 CAD 与 STS3215 模型的版权归原作者（TheRobotStudio / Feetech），再分发前请核对上游许可。
