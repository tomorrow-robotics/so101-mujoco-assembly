# Q0 → Panthera/MuJoCo：完整运行链迁移报告

日期：2026-11-12。上游 commit：`6baccd57ff958570d0db1588370289ee36c8bb70`。

## 结论

**完整主模型—工具—动作组—反馈运行链已通过无动作端到端验收。** 这次实际复用了 Q0 的 Agent 工具循环、持久 Python 沙箱、观测校验、历史与归档、动作协议、Executor 和 motion 逻辑，修正了之前只摘几段几何工具的范围错误。

真实 Astra 订阅运行完成两轮规划、5次模型调用。Python变量跨调用及跨规划轮次保留，原Q0执行器接收当前位姿的零动作组，下一轮读取真实MuJoCo反馈。**物理步数0、sim_time=0，qpos/qvel/ctrl/eq_active不变。没有生成装配轨迹或执行抓取/插入。**

这证明harness运行链接通，不代表装配任务已完成，也不代表真实运动分支已验收。

## 当前架构

```text
MuJoCo/Panthera native state
  → Q0 observation manifest + read-only observation directory
  → Q0 Agent / PlanningHistory
  → Codex CLI authenticated with ChatGPT subscription, gpt-6-astra
  → Q0 function-call envelope
  → original Q0 run_python dispatcher
  → persistent isolated Python Sandbox
  → tool result / next model call
  → normalize_plan / validate_plan
  → original Q0 Executor + motion
  → native MuJoCo backend (dry-run lock)
  → execution feedback / next observation
```

Astra传输适配采用Codex结构化输出承载Q0工具调用，不是把Q0 Python替换成Codex自己的shell。由原Agent负责调度工具和处理结果；当前没有独立VLM reviewer，也不发送图片。图像归档/裁剪代码保留，当前配置明确关闭图像输入与返回。

## 保留与修改

| 部分 | 实际处理 |
|---|---|
| `harness/runner.py` | 保留主工具循环、结果处理、错误修复、记忆、归档及动作归一化；增加subscription factory、可配置相机列表、文本模式和关闭独立reviewer的分支 |
| `harness/sandbox.py` | 保留持久内核、stdout/control分离、超时、内存/文件大小限制、恢复及工作区；只加`mount_proc`兼容选项 |
| `history.py`, `archive.py`, `model_context.py`, `planning_budget.py` | 复用原实现，不另写历史/上下文框架 |
| `observations.py`, `helpers/observation_io.py` | 保留manifest归属、SHA校验、原观测只读、历史帧读取；text-only发布器使用空相机列表，不伪造RGB/K |
| `runtime/protocol.py` | 保留三点目标、动作组、gripper before/after、checkpoint和严格验证；改写X5手指身份说明，拒绝未校准的width命令 |
| `controller/executor.py`, `motion.py` | 保留动作组编译、检查、播放、到达验证、after动作取消、双臂同步和反馈结构；本轮走零动作分支 |
| `adapters/codex_subscription.py` | 新增订阅transport；继承原API消息/工具结果格式；调用官方CLI，检查ChatGPT登录，禁用native工具，不读写OAuth token，不回退API Key |
| `adapters/mujoco_backend.py` | 新增原生边界，读compiled model的关节/TCP/table/contacts；复用已有`Assembly.ik`，运动分支使用现有physics.yaml PD增益和模型力矩限位；默认关闭物理执行 |
| `assets/tasks/*` | 用SO-101任务定义替换54个RoboDojo任务卡；当前迁移验收受dry-run限制 |
| `assets/prompts/*`, `assets/geometry/*` | 保留通用操作规则，替换X5尺寸、固定桌高和禁止使用真值的benchmark限制；明确Base留在桌面支撑、M1可在后续抬起 |
| `controller/calibration/` | 从工作副本中移除X5配置；原样内容仍在上游参考快照。旧X5 backend 构造时明确拒绝，不能误启用 |
| RoboDojo部署脚本、原API provider代码 | 留作来源/参考，不是当前启动入口。未复制凭据、未启动Docker服务、未引入RL |

完整改动清单：`results/q0_harness_migration/reuse_manifest.json`；原始快照哈希：`third_party/q0_preview/ORIGIN.json`。工作副本为 `q0_harness/`，参考快照未改。

## 沙箱问题与解决

初始环境拒绝新`/proc`挂载；sudo方式在devbox与27cf均不能创建所需namespace。没有回退宿主exec。

最终使用原版非特权bubblewrap模式，保留`--unshare-all`、cap-drop、只读运行时、独立/tmp、隔离网络和只读observations；仅把`--proc /proc`换为创建空`/proc`目录。这样Python仍在真实OS隔离内运行。代价是基于`/proc/self/status`的可选内存统计为空，`resource.setrlimit`的内存限制仍存在。

独立工具环境使用上游固定NumPy1.26.4、SciPy1.15.3等依赖，避免将宿主用户目录、凭据、仿真资产或MuJoCo环境挂进沙箱。已验证：

- 跨调用变量保持。
- 宿主 `~/.codex` 与项目根目录不可见。
- 外部网络连接不可用。
- 观测目录不能写入。

证据：`results/q0_harness_migration/evidence/sandbox_isolation.json`。

## 实测验收

| 验收 | 结果 |
|---|---|
| 几何与边界单元/负例 | 17 tests PASS |
| 原生TCP三点往返、坐标变换与相机数学 | PASS；此前静态模型往返误差约2.3e-16 |
| 原版沙箱持久化和隔离 | PASS |
| Astra subscription实际调用 | PASS；smoke_01、smoke_02各5次 |
| 连续两轮观察—工具—提交—反馈 | PASS |
| 跨规划轮次Python变量 `migration_probe` | PASS；第二轮仍为17 |
| 原Q0 Executor零动作组 | 两轮COMPLETE，均0物理步 |
| dry-run新移动/夹爪动作 | 拒绝，状态未改 |
| 无reviewer的两组动作 | 执行前REPLAN |
| 未校准width / 无效JSON / 越限关节 | 拒绝 |
| STOP/deadline条件与图片禁用边界 | 单元检查PASS；没有另做付费模型请求中途取消实验 |
| 实际抓取、插入、载荷和支撑稳定性 | NOT RUN，属于后续轨迹阶段 |

最新闭环证据：`results/q0_harness_migration/smoke_02/RESULT.json`。逐次模型调用、工具代码/输出、原始观察、protocol_audit、execution、history_journal都在同一目录。`COMPLETE`只表示这个零动作组完成，不是任务成功。

## 启动和复现

```bash
cd <repo>/mujoco_so101
export PYTHONPATH=/tmp/q0-mujoco-runtime:$PWD
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
codex login status
python scripts/run_q0_harness.py \
  --scene /tmp/q0-step1-scene/scene.xml \
  --out results/q0_harness_migration/a_new_run \
  --python-prefix /tmp/q0-preview-python \
  --bwrap ~/.codex/packages/standalone/releases/0.167.2-x86_64-unknown-linux-musl/codex-resources/bwrap

python -m unittest discover -s tests -p 'test_q0*.py' -v
```

`--out`必须是新目录，沿用上游避免混淆运行记录的行为。外层运行环境需要允许启动bwrap；受限嵌套容器可能需要由具备相应执行权限的宿主进程启动，但模型Python始终在bwrap内。

临时环境重建（不改系统包）：

```bash
python -m venv --without-pip /tmp/q0-preview-python
python -m pip install --target /tmp/q0-preview-python/lib/python3.11/site-packages -r q0_harness/requirements.txt
python -m pip install --no-deps --target /tmp/q0-mujoco-runtime mujoco==3.9.0
mkdir -p /tmp/q0-step1-scene
tar -xzf <workspace>/so101_step1_v11/input.tar.gz -C /tmp/q0-step1-scene
```

当前工具venv约466MiB、独立MuJoCo绑定24MiB、小场景94MiB，均在/tmp；工作副本约1MiB。没有新增GPU keeper或改动现有keeper。

## 下一阶段边界

- 当前公开启动器锁定dry-run。真实动作分支代码已经适配原生IK/PD，但尚未物理验收，不能直接把接线成功当成可用装配策略。
- 原Q0 `motion`的关节路径不是保证直线的EEF插入轨迹。下一步必须按用户确认的抓法与插入轴生成小段动作，再验证真实接触。
- 要继续确认实际Panthera指垫工作点和张口映射；目前只开放keep/open/close，width明确拒绝。
- 路径检查使用native colliders，但当前没有完整的携带物体预测扫掠模型；不能宣称全程载荷碰撞已经证明安全。
- 到位判据、Base桌面支撑证据、Virtual Fastening触发及释放保持应在任务执行阶段接入。不能以末端到达或模型判断代替。
- 独立VLM仍关闭。两组动作若没有有效review callback会REPLAN；当前采用每轮一个动作组，靠新观测继续。

因此交付状态是：**harness运行链迁移与无动作验收完成；物理装配动作和轨迹尚未执行，待下一阶段设计与验证。**
