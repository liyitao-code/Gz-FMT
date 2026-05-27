# 蜕变测试模块：集成逻辑、SDF 修改、生命周期与状态核验

本文档回答以下四类问题，并基于当前代码库（`randomsmith_meta.py` 等）给出具体说明。

---

## 一、模块触发与集成逻辑（Integration Logic）

### 问题

蜕变测试模块（如 randomsmith_meta.py）与第三章的 RL 模糊测试主框架（GzFuzz）是如何协同工作的？

是 RL 框架先生成一个合法的、未崩溃的初始场景（Seed Scene），然后将其移交给蜕变测试模块进行变异和关系核验？还是两者是独立的测试流水线？

### 答案

在当前实现中，**蜕变测试模块与“种子场景”的提供方是“调用者—被调用者”关系**，而不是两条完全独立的流水线：

1. **谁提供 Seed Scene**  
   - 实验目录 `directory` 和 SDF 文件名（默认 `a.sdf`）由**调用方**传入。  
   - 调用方负责在运行前准备好该目录，并在其中放置一个合法的世界 SDF（例如 `a.sdf`）。  
   - 例如：批量脚本从 SDF 池中随机选一个 world，复制到 `meta/test_23/_42/`，并确保存在 `a.sdf`；或 RL/多智能体框架（如 `muti_agent_smith_*.py`）在每轮中生成/选择状态对应的目录和 SDF。

2. **蜕变测试模块做什么**  
   - `SmithUnit(directory=exp_dir, sdf_name="a.sdf", ...)` 被实例化后，调用 `generate_and_test_commands()`。  
   - 该函数会：先清理残留 Gazebo 进程 → 在本进程内启动 **一次** `gz sim {directory}/{sdf_name}`（即用该 Seed Scene 启动仿真）→ 激活仿真引擎（play → 立即 pause）→ **随机选择一种蜕变关系**（如 joint_constraint_stability、mass_scaling_no_reset、environment_perturbation_robustness 等）→ 在该 world 上执行对应的“源用例 + 衍生用例”流程 → 根据状态提取结果做**关系核验**（通过/失败）→ 写回结果与日志并关闭 Gazebo。

3. **与 RL/GzFuzz 的协同方式**  
   - 若第三章的 GzFuzz 负责“生成或筛选未崩溃的初始场景”，则它可以：  
     - 将“合法、未崩溃”的 SDF 写入某实验目录（如 `meta/run_123/_5/a.sdf`），  
     - 然后调用 `SmithUnit(directory=..., sdf_name="a.sdf", ...).generate_and_test_commands()`，  
   - 即：**GzFuzz 提供 Seed Scene，蜕变测试模块在该场景上执行一次蜕变关系核验，不负责生成或变异 SDF 本身**。  
   - 本仓库中已有类似集成：`muti_agent_smith_*.py` 等用 RL 维护状态与策略，在每轮中构造 `exp_dir` 和 `a.sdf`，再调用同一套 `SmithUnit(...).generate_and_test_commands()` 或 `generate_and_test_commands_train(...)`，可视为“RL 框架 + 蜕变测试模块”的一种集成方式。

4. **小结**  
   - **不是**“两条完全独立的流水线”：蜕变测试依赖调用方提供的目录与 SDF。  
   - **是**“RL/上层框架先生成或选定合法初始场景，再交给蜕变测试模块做一次变异与关系核验”的协作方式；变异发生在**仿真内操作**（施力、改质量、插装饰物、reset 等），而不是对 SDF 文件做语法级变异。

---

## 二、SDF 文件的自动化修改机制（SDF Modification）

### 问题

在执行如“质量缩放（Mass Scaling）”这类需要修改模型固有属性的蜕变关系时，你的代码是如何自动化解析和修改 SDF 文件的？

是使用 Python 的 XML/DOM 解析库（如 lxml）在内存中动态修改 `<mass>` 标签，然后再重新加载模型吗？

### 答案

质量缩放（以及“无 Reset”的质量缩放变体）中，对模型质量的修改**不是**直接改磁盘上的 SDF 文件再重新加载整个 world，而是：

1. **运行时获取当前世界的 SDF**  
   - 通过 Gazebo 的 **`/world/<world_name>/generate_world_sdf`** 服务（代码中封装为 `dump_sdf(self.world_name)`）获取当前仿真世界的完整 SDF 字符串。  
   - 这是**运行时状态**的导出，而不是读实验目录下的 `a.sdf` 文件。

2. **在内存中解析并修改质量**  
   - 使用 Python 标准库 **`xml.etree.ElementTree`（ET）** 解析上述 SDF 字符串。  
   - 在解析得到的 DOM 中：定位到目标 `<model name="...">` → 遍历其下所有 `<link>` → 在每个 `<link>` 的 `<inertial><mass>` 中读取原质量，按目标总质量与当前总质量之比计算缩放因子，**在内存中**改写 `<mass>` 的文本值。  
   - 同时按同一比例缩放 `<inertial><inertia>` 中的 `ixx, ixy, ixz, iyy, iyz, izz`，以保持惯性张量形状一致。  
   - 项目中也有使用 **lxml** 的地方（如 `perturb_xml`、部分 XML 片段处理），但**质量缩放**的修改逻辑使用的是 **ET**，而不是 lxml。

3. **应用修改的方式：删除旧模型 + 用新 SDF 创建**  
   - **不**重新加载整个 world，也**不**写回磁盘文件。  
   - 步骤为：  
     - 调用 **`/world/<world_name>/remove`** 服务，请求删除目标模型（Entity）；  
     - 将修改后的 `<model>...</model>` 包装成合法 SDF 片段（带 `<sdf version="1.6">` 等），  
     - 调用 **`/world/<world_name>/create`** 服务（EntityFactory），传入新 SDF 字符串及当前位姿（由 `get_model_pose_from_scene` 与 scene 中的 orientation 得到），在**原位置**重新创建该模型。  
   - 因此，Gazebo 侧是“先删后建”，Python 侧是“dump_sdf → ET 解析 → 改 mass/inertia → 拼 SDF → remove + create”。

4. **小结**  
   - 解析：**ET** 解析 `dump_sdf` 得到的字符串，在内存中改 `<mass>` 与 `<inertia>`。  
   - 应用：通过 **remove + create** 服务在运行中的 world 里替换模型，**没有**“改磁盘上的 a.sdf 再重新加载”的步骤。  
   - 另外，**读取**质量时若 scene 中无 inertial 信息，会回退到从实验目录的 `a.sdf` 或再次 `dump_sdf` 用 ET 解析质量（`_get_model_mass_from_sdf`），这里同样用 ET，不是 lxml。

---

## 三、仿真生命周期与时序控制（Lifecycle & Timing Control）

### 问题

在执行“源测试用例”和“衍生测试用例”时，你是如何管理 Gazebo 进程的？是每次都彻底杀死并重启 gzserver，还是利用 Gazebo 的内置重置服务（如 reset_world / reset_time）？

很多蜕变关系（如力叠加、状态保持）对时间步长非常敏感。你的执行器是如何精确控制物理演化的？是否利用了 pause（暂停）和单步执行（Step）的 API 来确保在施加力或重置位姿时，仿真时间是绝对静止的？

### 答案

**1. Gazebo 进程管理**

- **绝大多数蜕变关系**：**单进程、单次启动**。  
  - 在 `generate_and_test_commands()` 开始时，先 **pkill** 清理残留的 `gz sim`、`ruby`、`gz-sim-server` 等进程，再在本进程内 **`subprocess.Popen("gz sim {directory}/{sdf_name} ...")`** 启动**一次** Gazebo。  
  - 整轮测试（选模型、施力、步进、取状态、判通过/失败）都在**同一个** Gazebo 进程中完成；需要“重置”时，使用 Gazebo 的 **WorldControl** 服务，**不**杀死进程再重启。

- **Determinism（双进程确定性）**：**唯一例外**。  
  - 先在同一目录下启动第一个 Gazebo 进程，执行“源用例”（施力 + 步进），读取终点位姿 P1 后 **kill 该进程**；  
  - 再 **pkill 清理**，等待数秒；  
  - 然后启动**第二个** Gazebo 进程，用**同一份 SDF** 执行相同脚本，得到 P2，比较 P1 与 P2。  
  - 因此：只有 Determinism 是“杀死并重启进程”；其余均为单进程 + 内置 reset 服务。

- **重置服务**  
  - 代码中使用 **`gz.msgs.WorldControl`** 的 **`reset: { all: true }, pause: true`**（封装在 `reset_simulation()` 中），即 Gazebo 官方的“世界重置”能力，将仿真恢复到初始状态并保持暂停。  
  - 没有使用单独的“reset_world / reset_time”命名，但语义上就是世界级重置。

**2. 时间步长与“仿真时间静止”控制**

- **pause + multi_step**：  
  - 启动后通过 **`pause: false` 再立即 `pause: true`** 做一次“激活”，使后续 `multi_step` 与 wrench topic 可用。  
  - 之后在需要“精确推进仿真时间”的地方，**不再**使用“resume → wall-clock sleep → pause”，而统一使用 **`WorldControl.multi_step: N`**（N 为步数）。  
  - `step_simulation(num_steps)` 内部即：发送 `multi_step: num_steps`，再根据步长（默认 0.001 s）做必要的 wall-clock 等待，最后再发一次 `pause: true` 确保仿真暂停。  
  - 因此：**在施加力、重置位姿、插入装饰物等操作时，仿真都处于 pause 状态；物理演化仅发生在显式调用 `step_simulation(...)` 的区间内**，从而保证“在施力或重置位姿的瞬间，仿真时间是静止的”，避免 wall-clock 带来的不确定性。

- **步长**：  
  - 步数 `num_steps` 与仿真时间的关系为：`sim_time = num_steps * 0.001`（秒），即默认 1 ms 步长。  
  - 所有“运行 T 秒”的测试（如 5 s、2 s）都换算成步数后调用 `step_simulation`，保证可重复性。

**3. 小结**

- 进程：除 Determinism 外均为**单进程**；Determinism 为**双进程**（先杀再起）。  
- 重置：使用 **WorldControl `reset: { all: true }, pause: true`**，不依赖重启进程。  
- 时序：**pause + multi_step** 精确控制物理演化；在施力、set_pose、reset 等操作前后仿真均处于暂停，实现“仿真时间绝对静止”的语义。

---

## 四、状态提取与比对核验机制（State Extraction & Verification）

### 问题

你的执行器是通过订阅哪些特定的 Topic 或调用哪些 Service 来获取模型执行后的真实物理状态（如绝对坐标、线速度、角速度）的？

当核验程序发现“衍生状态”与“期望状态”存在偏差（比如超出了容差阈值）时，系统是如何记录这个逻辑缺陷的？日志里会保存哪些上下文信息以便你后续复现并提交 Issue？

### 答案

**1. 状态获取：Topic 与 Service**

- **场景结构（模型/link 列表、初始位姿等）**  
  - **Service**：**`/world/<world_name>/scene/info`**，请求类型 `Empty`，响应类型 `Scene`。  
  - 用于：获取 world 中所有 model/link 的静态或初始信息；部分接口中也可拿到 orientation 等。  
  - 注意：scene 返回的是“场景结构”，**不**保证是仿真运行后的实时位姿，因此**实时位姿**不用 scene，而用下面的 pose topic。

- **实时位姿（位置 + 朝向，用于核验）**  
  - **Topic**：优先 **`/world/<world_name>/dynamic_pose/info`**（仅非静态模型），失败则回退到 **`/world/<world_name>/pose/info`**（含所有实体）。  
  - 消息类型为 **Pose_V**（pose 数组）。  
  - 由于在**暂停**状态下 Gazebo 可能不主动发布 pose，代码采用 **“先发 `multi_step: 1` 触发一帧，再订阅并读取一次 pose”** 的方式（例如 `get_model_pose_from_scene`、`get_all_entity_poses_from_scene`），从而在 pause 下也能拿到当前状态。  
  - 返回内容：模型的 **(x, y, z)** 及四元数 **(w, x, y, z)**；关节约束测试中会解析 **link 级**的 pose（同一 topic 中实体名可能为 `model::link` 或 `link`）。

- **线速度/角速度**  
  - 当前蜕变测试的**核验**主要依赖**位置（及 link 间距离）**，未在核验逻辑中显式订阅线速度/角速度 topic。  
  - 若需速度，可扩展订阅 Gazebo 提供的 velocity 相关 topic（若有），或由位置序列差分估算；当前实现未写入文档的“标准速度 topic”名称。

- **世界 SDF 导出**  
  - **Service**：**`/world/<world_name>/generate_world_sdf`**，用于质量缩放时获取当前世界 SDF 并在内存中修改质量。

**2. 核验与“逻辑缺陷”记录**

- **判定**：每个蜕变关系在代码中实现为若干阈值与条件（如位移差 < ε、无漂移、无跳变、d2/d1 在区间内等）。若条件不满足，则本次测试判为 **FAILED**。

- **写入结果与标记**：  
  - **`{experiment_directory}/metamorphic_test_result.txt`**：写入测试类型、模型名、关键数值（如位移、距离序列、相对误差）、PASSED/FAILED，以及一段 **Error Info**（含容差、采样点、违规点等）。  
  - 若结果为 FAILED，还会在实验目录下创建空标记文件 **`METAMORPHIC_TEST_FAILED`**，便于批量统计失败用例。

- **实验日志（复现与 Issue）**：  
  - **`{experiment_directory}/experiment_log.json`**：按时间顺序记录整轮测试的**所有操作**，包括：  
    - `launch`：启动命令（如 `gz sim .../a.sdf --record-path ...`）；  
    - `sleep`：每次等待及描述（如 “Wait for pause (joint constraint)”、“Stepping 396 steps (0.396s sim-time)”）；  
    - `service` / `topic`：每次调用的 **gz service** 或 **gz topic** 的完整参数（如 WorldControl 的 pause、multi_step，wrench/clear、wrench/persistent 的 Entity/EntityWrench 等）。  
  - 复现脚本 **`reproduce_experiment.py`** 会读取该 JSON，在全新 Gazebo 进程中**严格按顺序重放**这些命令，用于复现失败用例并对比“原始结果 vs 复现结果”。

- **为提交 Issue 保留的上下文**：  
  - 实验目录内通常保留：**a.sdf**（该次使用的世界）、**metamorphic_test_result.txt**（失败原因与数值）、**experiment_log.json**（完整操作序列）。  
  - 复现时还可生成 **reproduce_*/reproduction_report.txt**，对比原始与复现的位姿/距离等。  
  - 因此，**足够复现并撰写 Issue 的上下文**包括：世界 SDF、完整操作序列、报告中的容差与违规点、以及（若需要）复现报告。

**3. 小结**

- **状态获取**：Scene 用 **scene/info**；**实时位姿**用 **dynamic_pose/info** 或 **pose/info** + **multi_step: 1** 触发；质量缩放还会用 **generate_world_sdf**。  
- **缺陷记录**：**metamorphic_test_result.txt**（含 Error Info）+ **METAMORPHIC_TEST_FAILED** 标记 + **experiment_log.json**（完整操作序列）；复现与 Issue 可依赖该目录下的 SDF、结果文件和 JSON 日志。

---

*文档基于当前 `randomsmith_meta.py` 及 `reproduce_experiment.py` 实现整理，若后续接口或流程有变更，请以代码为准并同步更新本说明。*
