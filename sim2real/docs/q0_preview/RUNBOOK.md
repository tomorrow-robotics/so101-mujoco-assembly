# Q0 装配运行手册（Astra 驱动两只 Panthera 装 SO-101）

本文写的是已经跑通的流程。两条完整的链都是每一局一次成功：E1 → E2 → F1（2026-11-13/14），以及加入第 3 步技能后的 G1 → G2 → G3（2026-11-18；分别用 13、14、16 轮规划，没有一次规划被拒，视频在 `results/q0_videos/G_chain_1118/chain.mp4`）。第 3 步单独又跑过一次 F2b，也成功了。
迁移背景见 `MIGRATION.md`，通用操作经验见 `OPERATING_GUIDE.md`。

## 1. 组成

| 部分 | 位置 | 说明 |
|---|---|---|
| 上游 Q0 harness | `q0_harness/harness`、`runtime`、`controller` | 尽量与上游逐字节一致，不要改。`protocol.py` 是例外：手指名一行，加上动作组的可选 `compliance` 字段 |
| 本地网关 | `q0_harness_gateway/` | 把 Responses API 转到 `codex app-server`，用的是用户的 Codex 订阅。只绑 127.0.0.1 |
| 后端与评估 | `q0_harness/adapters/` | `mujoco_backend.py`（物理与相机）、`so101_station.py`（就位判定、紧固、评估）、`model_loading.py`（加载时修复法向量）、`replay.py` |
| 跨局记忆 | `q0_harness/experience.py`、`assets/experience/` | 每局结束自动记录；下一局开始时以只读方式放到 `/work/experience.md` |
| 技能库 | `q0_harness/skills.py`、`assets/skills/<name>/SKILL.md` | 每局放到 `/work/skills/`，附 `INDEX.md`，按 `applies_to` 筛选 |
| 启动器 | `scripts/run_q0_episode.py` | 物理在 devbox 上跑；渲染（`--render-on-pod`）和 cuRobo（`--planner curobo`）在 pod 上 |

## 2. 硬性约定

- **凭据**：`~/.codex/auth.json` 只留在 devbox 上，不拷到 pod，不放 HDFS。网关只绑 127.0.0.1。所以 episode 一律从 devbox 启动。`scripts/pod_q0_episode.sh` 需要 pod 上有 auth，按用户决定**不使用**。
- **配额**：网关带 `--quota-stop-percent N` 启动，周用量到 N% 后拒绝一切请求。当前是 20（用户 2026-11-18 要求：周配额剩 80% 时停）。
- **模型**：`gpt-6-astra`，见 `q0_harness/config/agent.gpt6astra.gateway.json`。
- **渲染**：只在 pod 上用 NVIDIA EGL。先 `source <shared>/robotwin_env/nvidia_egl_env.sh <GPU>`，再确认日志里 `GL_RENDERER` 是 NVIDIA。
- **资产**：不改冻结的 v1.1 目录。模型一律用 `load_model` 加载（修复 v1.1 针状面的非单位法向量），`NativeMujocoBackend` 会拒绝没修复的模型。第 3 步的叉架用 `results/v12_patch/fork_m1_interface` 补丁。
- **规则**：不把物体位姿给策略，评估器从真实仿真状态判定。Base 可以滑动，但抬起超过 1 mm 算违规。零件必须物理就位后才紧固。不允许瞬移零件。

## 3. 启动网关（devbox）

```bash
cd mujoco_so101
nohup python -m q0_harness_gateway --port 18090 --log-dir results/q0_gateway \
    --max-total-tokens 60000000 --quota-stop-percent 20 > results/q0_gateway/stdout_$(date +%m%d).log 2>&1 &
curl -s "http://127.0.0.1:18090/v1/quota?refresh=1" | python -m json.tool   # latest.primary.used_percent = 周用量
```

`--max-total-tokens` 默认 2000 万，一局长 episode 会碰到这个上限（Run C 就是这样中断的），所以设成 6000 万。

## 4. 三步链

pod 是 远端 GPU 作业（job 名由 `POD_JOB` 指定）。`scripts/pod_sync.sh <paths>` 把文件按同一绝对路径同步过去，`scripts/pod_exec.sh <name> '<cmd>'` 在 pod 上执行。

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 Q0_PREVIEW_API_KEY=local-gateway
CFG=q0_harness/config/agent.gpt6astra.gateway.json
COMMON="--render-on-pod --planner curobo --curobo-gpu 6 --agent-config $CFG --plans 30"

# 第 1 步：M1 装入 Base（场景：python scripts/build_step1_handhold_scene.py --out results/step1_handhold_scene）
python scripts/run_q0_episode.py --scene results/step1_handhold_scene/scene.xml --task so101_base_m1_insert \
    --out results/q0_episodes/<E1> $COMMON

# 第 2 步：盖子。把上一步终态映射到第 2 步场景（多了零件），然后启动
python scripts/map_step_state.py --from-state results/q0_episodes/<E1>/final_state.npz \
    --source-scene results/step1_handhold_scene/scene.xml --scene results/step2_holder_scene/scene.xml \
    --out results/step2_from_<E1>/initial_state.npz --settle-s 3
python scripts/run_q0_episode.py --scene results/step2_holder_scene/scene.xml --init-state results/step2_from_<E1>/initial_state.npz \
    --task so101_m1_cap_install --out results/q0_episodes/<E2> $COMMON

# 第 3 步：叉架（场景：build_step3_fork_scene.py --fork-patch results/v12_patch/fork_m1_interface --out results/step3_fork_scene_v12）
python scripts/map_step_state.py --from-state results/q0_episodes/<E2>/final_state.npz \
    --source-scene results/step2_holder_scene/scene.xml --scene results/step3_fork_scene_v12/scene.xml \
    --out results/step3_v12_from_<E2>/initial_state.npz --settle-s 3
python scripts/run_q0_episode.py --scene results/step3_fork_scene_v12/scene.xml --init-state results/step3_v12_from_<E2>/initial_state.npz \
    --task so101_shoulder_fork_install --out results/q0_episodes/<F> $COMMON
```

映射脚本先在 pod 上跑（`pod_sync.sh` 同步场景和状态文件，再用 `pod_exec.sh`），然后把 `initial_state.npz` 同步回来。启动 episode 前，把 `q0_harness scripts tests <scene> <state目录>` 同步到 pod。

## 5. 一局进行中

- **每一轮都要汇报**：每局每轮不超过 5 句，标明是哪一局。看 `session/execution.jsonl`（`plan`、`plan_cancelled`、`group_completed`），看 `session/observations/`。不要用阻塞 600 s 的命令去等，那样就停止汇报了。
- **评估方收局**：策略报告完成、只在原地保持不动时，立即收局（E2 就因为没收局多跑了约 17 轮）：
  ```bash
  echo "STOP by operator after plan N: <理由>" > results/q0_episodes/<run>/STOP_REASON.txt
  touch results/q0_episodes/<run>/session/STOP
  ```
- **基础设施中断后续跑**（网关上限、API 403 等）：回放到最后一轮结束，然后继续。
  ```bash
  python scripts/run_q0_episode.py --resume-from results/q0_episodes/<cut> --out results/q0_episodes/<cut>_resumed $COMMON
  # 只回放到某一轮（例如改了工位规则后，从犯错之前继续）：加 --resume-through-plan N
  ```
- 回放是逐命令重新执行，并和记录逐步比对（`q0_harness/adapters/replay.py`）。不一致数必须为 0。

## 6. 一局结束后

1. 结果：`RESULT.json`（评估器判定，策略看不到）。门限、Base 最大抬起、先前零件是否仍紧固、两手回位误差都在这里。
2. 经验已自动记入 `assets/experience/<task>.json`。要加运维备注：`python -m q0_harness.experience note <task> "..."`；补录：`python -m q0_harness.experience record <run_dir>`。
3. 在 pod 上渲染（GPU）：
   ```bash
   R=results/q0_episodes/<run>; mkdir -p $R/replay && cp $R/trajectory.npz $R/replay/
   bash scripts/pod_sync.sh $R/LAUNCH.json $R/replay/trajectory.npz <scene目录>
   bash scripts/pod_exec.sh render_X "source <shared>/robotwin_env/nvidia_egl_env.sh 0 >/dev/null && \
       python scripts/replay_q0_episode.py --run $R --render-only && tar -cf <shared>/so101_v11_pod/sync/X.tar -C $R/replay frames"
   tar -xf <shared>/so101_v11_pod/sync/X.tar -C $R/replay
   ```
   tar 里的路径要相对于 `-C` 目录，不要相对于 `/` 解包。
4. 多步拼接视频：`python scripts/build_q0_chain_video.py --out results/q0_videos/<new> --step <run> "<标题>" "<说明>" ...`（`--result-note <run> "<文字>"` 给结果卡加说明）。
5. 一步跑通后：把这一局提炼成该步的指南 `assets/experience/<task>_guide.md`，并加一个 `assets/skills/stepN-.../SKILL.md` 来 `{{include:}}` 它；通用的教训改到对应的通用技能里。参考 `step3-fork-onto-m1`。

## 7. 工具与测量

- 可达范围表：`python scripts/build_q0_reach_table.py --scene results/step2_holder_scene/scene.xml [--out-dir D]`，在 pod 上跑。输出 4 种倾斜方向（背离臂座、朝 world ±x 侧倾、朝臂座）。写入 `assets/experience/robot_reach.md`，并被 `reach-with-oblique-tool` 技能引用。
- 压入实测：`scripts/q0_cap_press_probe.py`、`q0_m1_press_probe.py`、`q0_fork_seat_probe.py`；说明文档由 `scripts/build_q0_cap_mating_note.py`、`build_q0_m1_mating_note.py` 生成到 `*_measured.md`。
- 测试（`tests/` 不是包，按脚本运行）：`python tests/test_q0_experience.py`，`test_q0_station_signal.py`，`test_q0_model_loading.py`，`test_q0_compliance.py`（其中物理压入测试还没修好）。
- 网关配额：`curl -s "http://127.0.0.1:18090/v1/quota?refresh=1"`。

## 8. 已知的坑

- **零件被甩飞 / 接触点在 1e6 m 外**：v1.1 针状面的法向量不是单位向量（MuJoCo 3.9.0）。一定要用 `load_model`。v1.2 才修资产。
- **紧固后又动零件**：图像在座附近的深度读数偏高约 0.5 mm，只信 `station.fastened`。紧固后只松手、撤离、回位（Run D：Base 被带起 21 mm）。
- **瞄准正好座位**：手臂受力让位，零件差约 1 mm 没到位。要瞄准座后 2 mm。
- **长距离限力滑入**：手被顶偏（F1：17.5 mm），零件在手里转了。限力只用于最后约 10 mm，或者不用。
- **竖直向下的工具**：两臂在 Base 附近都够不到。用 30–45° 倾斜，先查可达范围表。
- **另一只手的碰撞**：规划器不检查手里拿着的零件和另一只手臂是否相碰。
- **装上盖子以后**：Base 后壳的夹持宽度是 44.7 mm，不再是 34.7 mm。
- **我的操作失误**：阻塞的长命令停掉了逐轮汇报；pod 上没有 `/usr/bin/time`；tar 解包到 `/` 在 devbox 根目录建出了 `/results`。

## 9. 待定

- 叉架门限（暂定：横向 0.4 mm，轴向 −0.3..+0.8 mm，1.2°）；盖子轴向门限是否从 0.5 放宽到 0.8。两项都由用户决定。
- v1.2：把叉架↔M1 接口加入 13 接口验收；在补丁场景上复查静态无穿透；把补丁并入 v1.2；为公开的 v1.1 用户清理针状面。
