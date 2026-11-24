# Q0 harness 完整复用范围

用户明确目标：用Q0 harness运行MuJoCo SO-101任务；不是另写一个控制框架或只取几何函数。

## 保留的通用运行骨架

- harness/runner：主模型—工具—动作组反馈循环。
- harness/sandbox：持久Python进程、工具输出、超时、内存限制、隔离、失败恢复。直接复用原实现，禁止退回宿主exec冒充隔离。
- helpers/observation_io、observations、tool_images：观测文件、归属/哈希、裁剪和工具图片通道；当前主模型先消费真值文本，视觉能力保留但不启用独立VLM。
- history、archive、model_context：上下文窗口、阶段记忆、完整执行记录。
- runtime/episode、protocol：动作组、检查点、反馈、预算和生命周期；任务/机型专属字段与数值单独适配。
- controller/executor、motion：尽量保留动作阶段、到达检查、失败取消、同步与路径诊断的通用逻辑。其backend必须适配原生Panthera/MuJoCo，不能直接发送X5动作。

## 必要替换

1. 模型transport：Astra subscription，经官方Codex登录/会话接口；不复制OAuth凭据、不悄悄切API Key。上游HTTP API worker不能原样充当订阅入口。
2. simulator backend：读取MuJoCo状态、接收14维双臂/夹爪控制意图并交给已有原生执行器；M1内部servo状态另行保留。
3. robot configuration：替换X5 URDF、限位、工具/指爪偏置、碰撞模型和相机标定，使用当前实验室Panthera配置。
4. task context：替换RoboDojo任务卡、评分和限制，定义Base桌面夹持与M1插装任务。原脚本中的工装支撑逻辑不适用于本任务。
5. 独立VLM reviewer：暂不启用，但阶段反馈和物理检查仍保留，不能默认CONTINUE。

## 当前实现状态

原始70个文件已保留。纯几何工具和MuJoCo静态接口测试已通过。新增 scripts/q0_sandbox_smoke.py 直接引用未改动的上游 Sandbox，检验跨调用变量保留、隔离和观测只读。开发机测试被运行环境阻塞：受限执行下namespace不可见；直接启动原隔离器时 `/proc` 挂载 Operation not permitted。尚未完成沙箱运行验证，更未完成Astra→工具调用→MuJoCo反馈的端到端链路。

后续验收必须是：Astra通过订阅调用run_python，第二次调用保留第一次变量；工具读取绑定观测；输出动作意图经协议校验；先在拒绝实际动作的dry-run backend完成反馈闭环；再进入用户确认的轨迹设计与物理执行。

本轮没有执行抓取或生成轨迹。


最新运行链迁移和实测状态以 [FULL_HARNESS_MIGRATION_REPORT.md](FULL_HARNESS_MIGRATION_REPORT.md) 为准：原版沙箱已通过兼容适配跑通，Astra订阅端到端无动作测试已完成。此前阻塞/待接线描述为历史记录。
