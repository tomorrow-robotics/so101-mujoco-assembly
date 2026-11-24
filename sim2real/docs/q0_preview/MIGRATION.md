# Q0_Preview 选择性迁移报告

> **范围纠正（用户最新要求）**：目标是复用完整 Q0 harness 来运行本任务，不是只提取几何函数。下文表格记录第一轮较窄迁移，关于通用 runner/sandbox/history/executor“不迁移”的决定已撤回。以 FULL_HARNESS_SCOPE.md 为当前接入范围；完整运行链尚未接通。


## 本轮交付

已拉取并固定上游 commit `6baccd57ff958570d0db1588370289ee36c8bb70`。原始完整 Q0_Preview 子目录保留在 `../../third_party/q0_preview/upstream/`，没有运行其安装脚本。来源和逐文件 SHA256 在 `ORIGIN.json`，Apache-2.0 LICENSE 随原始快照保留。

实际迁移的是：

1. `../../scripts/q0_geometry.py`：从 `harness/helpers/vlm_geom.py` 选择性提取并注明来源的纯几何计算；增加原生 TCP 三点姿态编解码、显式 Panthera 工作点偏置、显式标定相机投影。
2. `../../scripts/q0_inspect.py`：读取当前 MuJoCo Base/M1 场景，导出状态、命令/实测关节区别、TCP 虚拟参考点、真实桌面平面和接触证据。原生 a/b 命名与现有 IK 接口保持一致。
3. `OPERATING_GUIDE.md`：经任务约束改写的双臂协作、阶段检查、观察、接触及证据规则。
4. `../../tests/test_q0_geometry.py` 与 `evidence/`：纯几何正/负例和真实模型静态接口验证。

**不是完整 Q0 policy server 的移植，不是新的规划器或 CAD 建模框架。** 尚未生成轨迹、执行动作、调用外部 VLM/API 或启动 RL。

## 源码各部分负责什么、如何处理

| 上游部分 | 实际职责 | 决定与原因 |
|---|---|---|
| `model.py`, `deploy.py/yml`, `eval.sh` | XPolicyLab 标准 observation/action 适配、worker生命周期；仅支持 arx_x5/joint | 不迁移运行入口；当前是原生 MuJoCo/Panthera，不具备同一 evaluator/WS接口 |
| `runtime/episode.py` | 观测发布→规划→动作组执行→检查点→结束；区分墙钟和控制步预算 | 保留生命周期原则到指南；不引入新的 episode 框架或原始100计划/3小时预算 |
| `runtime/protocol.py` | 三点姿态、单双臂动作组、夹爪事件、严格schema、CONTINUE/REPLAN | 迁移三点几何；保留一段一确认及失败取消后续动作原则。完整schema依赖X5夹爪上限，不直接复制 |
| `runtime/remote_observation.py` | 校验标准RGB、K、camera-to-world、机器人状态与频率 | 保留同帧/坐标约定；原字段名与25Hz协议不套用。当前新增原生只读状态检查器 |
| `runtime/task_context.py`, `assets/tasks/*` | RoboDojo任务定义、顺序、完成规则和控制步限制 | 全部排除作为本任务规则；插钥匙、倒液体、折衣等任务卡不是SO-101装配规范 |
| `runtime/termination.py` | benchmark中只有不可恢复才可abandon等判定 | 不继承其禁止因预算结束的策略；本项目按用户停止条件和物理失败处理 |
| `controller/backend.py` | evaluator状态/动作队列、X5 FK/IK/碰撞与CuRobo接口 | 不直接迁移；复用我们原生模型、标定、PD与命名 |
| `controller/curobo_planner.py`, `curobo_mount.py`, `ik_solutions.py` | CuRobo路径与IK候选、坐标系映射和solution解析 | 暂不接入。不能把X5配置改名成HT；本轮不替换规划器 |
| `controller/motion.py` | 路径候选、关节限位余量、转动绕路/大幅摆动诊断、双臂时序 | 保留“检查路径而非只看终点、避免小位移大关节绕路”的原则；数值阈值不照搬 |
| `controller/executor.py` | 双臂同步步进、before/after夹爪事件、到达与失败取消 | 迁移职责与检查点规则；不复制位置动作播放器来替换当前力矩PD，未声称执行层已接通 |
| `controller/collision.py` | X5 URDF与机器人碰撞球，双臂距离检查 | 排除其模型与碰撞球。它不覆盖完整场景和载荷；保留使用原生collider检查的要求 |
| `controller/joint_limits.py`, `robot_geometry.py`, `calibration/*` | X5关节/指爪参数、145mm工具偏置、88.8mm张口、0.765m桌高 | 不移植数值。HT TCP与关节来自当前模型；桌面从lab_table求得；工作点偏置必须显式测量 |
| `harness/runner.py`, `api.py`, `api_worker.py` | 模型工具循环、HTTP/SSE、请求与响应审计 | 暂不运行。当前仍是工程师设计 scripted expert；不增加模型调用、凭证或API等待 |
| `harness/sandbox.py` | bwrap/Docker离线Python工具、内存/时间限制、内核重置 | 不重建当前Codex沙箱；若将来独立部署模型服务再评估 |
| `harness/history.py`, `archive.py`, `model_context.py` | 首轮/近期上下文、短记忆、完整归档与摘要 | 迁移“计划与已验证事实分离、坐标带来源与有效性、失败留证”的记录规则 |
| `harness/observations.py`, `tool_images.py`, `helpers/observation_io.py` | 同帧图像归档、SHA、原分辨率裁剪、显示来源 | 采纳数据契约；未复制固定/work、三相机必填、1000px限制或完整存储格式 |
| `harness/helpers/vlm_geom.py` | 三点变换、相机射线/投影、平面交点、三角化、圆拟合 | 选择性迁移纯函数。去掉全局观测缓存依赖、X5默认偏置；暂不迁移圆拟合等当前未使用功能 |
| `assets/prompts/prompt.md` | 任务理解、双臂职责、接近/抓取/撤离、三点目标、有限分组 | 改写为本项目指南，尤其支撑手不得随意挪走，抓取要为后续插入留空间 |
| `checkpoint_prompt.md` | 独立VLM只判断第二组能否继续 | 采用边界语义；当前优先原生物理证据，不新增VLM调用。原文不是可直接执行的MuJoCo检查器 |
| `geometry_context.md`, `geometry/*` | 主要是X5几何说明、像素计算与历史经验 | 保留单位/坐标原则；排除全部X5尺寸、RoboDojo桌高及历史误差/等待时间，不能作为HT精度保证 |
| `config/*`, `tools/*`, `install.sh`, `INSTALLATION.md` | 模型配置、服务启动、环境依赖、调度与审计工具 | 仅参考。没有盲装依赖、降级numpy/scipy、启动Docker或把54个benchmark任务导入本项目 |

## 与当前用户要求的关键差异

- 上游并没有通用 CAD 分解/建模工具，也没有 SO-101 装配轨迹；它是一个观察—计算—动作—反馈的 manipulation harness。
- 上游禁止隐藏物体真值，是 benchmark 的公平性约束。本项目已允许 ground-truth scripted oracle，因此原生物体位姿/接触可以使用，必须标注为仿真真值而非视觉估计。
- 默认“抬高2–3cm确认抓取”不能套到当前 Base；Base应留桌面，验证真实双侧夹持、桌面支撑与稳定性。M1才可以进行抬升确认。
- 静态 contact plane 检查不是摩擦、载荷或稳定性的证明。两手各自末端到位，也不是对象装配到位。
- 上游允许双方同步运动；本任务先明确左手稳定/右手操作的职责，再选择是否同步，不将所有动作机械串行或同步。
- `run_assembly.py` 中旧的 `base_fixture_pad` 支撑流程仍不能直接当成本次无工装策略。本轮未调用该旧入口。

## 验证与复现

```bash
cd <repo>/mujoco_so101
python -m unittest discover -s tests -p test_q0_geometry.py -v
# 在拥有当前MuJoCo环境的27cf节点：
python scripts/q0_inspect.py /tmp/so101_step1_v11/scene.mjb /tmp/q0_step1_observation.json
```

单元测试覆盖20个随机刚体姿态往返，以及翻转指序会产生真实180°roll、退化/非有限输入、显式工作点偏置、相机投影/射线与错误标定、并行/后向射线、三角化和变换不修改输入等情况。

实际27cf静态接口检查：读取 v1.1 Base/M1 小场景，实测 table plane z=0，两个TCP往返最大误差约2.3e-16。qpos、qvel、ctrl、eq_active和sim_time均未改变；物理步数为0。这不是抓取/支撑/轨迹验证。`evidence/step1_observation.json` 保留实测输出。

下一步讨论轨迹时，可以直接用这个接口生成当前状态的三点表示，并复用指南规划阶段。还需要结合真实HT夹持面确定工作点、验证抓持与支撑、接通阶段执行和成功判定；这些没有被包装成已完成。

## 来源

- https://github.com/XPolicyLab/XPolicyLab/tree/6baccd57ff958570d0db1588370289ee36c8bb70/policy/Q0_Preview
- 本地原样快照及SHA：`../../third_party/q0_preview/ORIGIN.json`
- 改写的数学代码保留 Apache-2.0 来源说明；上游完整LICENSE位于快照根目录。

## 模型接入决定更新（2026-11-12）

用户明确指定主规划模型 Astra，走现有 ChatGPT/Codex subscription；独立 VLM 环节暂不启用。前文“API 服务不迁移”指不照搬上游的 API Key/HTTP worker，不代表后续不接主模型。下一步需要对接 Codex 官方会话入口，继续复用已迁移的几何和观测契约。当前已验证 ChatGPT 登录状态和本地 Astra 模型目录，未验证独立 harness 实际推理。官方认证说明：https://developers.openai.com/codex/auth 。


最新运行链迁移和实测状态以 [FULL_HARNESS_MIGRATION_REPORT.md](FULL_HARNESS_MIGRATION_REPORT.md) 为准：原版沙箱已通过兼容适配跑通，Astra订阅端到端无动作测试已完成。此前阻塞/待接线描述为历史记录。
