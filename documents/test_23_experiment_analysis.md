# test_23 实验数据分析报告

**数据路径**: `/home/liyitao/workspace/meta/test_23`  
**测试范围**: 三个新蜕变关系（joint_constraint_stability, mass_scaling_no_reset, environment_perturbation_robustness）  
**分析时间**: 基于当前目录下 1244 次运行、1241 份结果文件

---

## 一、基本情况

### 1.1 总体统计

| 指标 | 数值 |
|------|------|
| 总运行数 | 1244 |
| 有结果文件 | 1241 |
| **returned None**（未执行/跳过） | **999 (80.5%)** |
| **PASSED** | **220 (17.7%)** |
| **FAILED** | **22 (1.8%)** |
| 创建 METAMORPHIC_TEST_FAILED 标记 | 1021 |

### 1.2 按测试类型分布（仅统计“实际执行并写入了完整结果”的用例）

| 测试类型 | 执行到完成 | 其中 PASSED | 其中 FAILED | 完成时通过率 |
|----------|------------|-------------|-------------|----------------|
| 关节约束稳定性 (joint_constraint_stability) | 195 | 188 | 7 | **96.4%** |
| 环境扰动鲁棒性 (environment_perturbation_robustness) | 32 | 26 | 6 | **81.3%** |
| 质量缩放单次运行 (mass_scaling_no_reset) | 15 | 6 | 9 | **40.0%** |

### 1.3 “returned None” 按选定测试类型分布

（每次运行随机选一种测试类型，若该次未执行到底则记录为 None，且第一行多为“简短类型名”）

| 选定类型 | None 次数 |
|----------|-----------|
| mass_scaling_no_reset | 395 |
| environment_perturbation_robustness | 377 |
| joint_constraint_stability | 227 |

说明：约 **80%** 的运行因“未执行到底”而得到 None，多数为**场景/模型不满足前置条件**导致提前返回（见下文）。

---

## 二、可能存在的问题

### 2.1 高比例 “returned None” 的成因

- **mass_scaling_no_reset**  
  - 依赖“能从 SDF 或 scene 得到有效质量”的模型；若场景里多为无质量或质量解析失败，会直接返回 None。  
  - 已通过 `_get_model_mass_from_sdf` 回退有所缓解，但部分 SDF 仍可能无 `<inertial><mass>` 或解析失败。

- **environment_perturbation_robustness**  
  - 要求场景中存在“地面”模型（如 ground_plane / ground_model）；无地面（如纯浮力/水下 world）会跳过。  
  - 另有位移/高度合理性检查（|d|>50m 或 z<-10）会主动跳过，避免误判。

- **joint_constraint_stability**  
  - 要求至少一个**多 link** 且两 link 位姿都能在 pose 消息中解析到的模型；单 link、无关节或 link 名解析失败都会 None。  
  - pose/info 使用本地 link 名（如 `chassis`）而非 `model::link` 时，已用回退解析；若场景中无合适多 link 模型仍会 None。

这些都与**场景/模型库特性**和**前置条件设计**有关，不一定是流程错误，但会导致“大量跳过”。

### 2.2 FAILED 用例中的典型模式

**质量缩放 (mass_scaling_no_reset)**  
- 现象：d2 与 d1 的相对误差超过 25%，或 d2/d1 与理论 1.0 偏差大。  
- 典型：带 **DiffDrive / 控制器** 的 vehicle、或 **摩擦/接触** 明显的模型；改质量后 remove+create 会重置速度，但接触/约束状态不同，Phase B 的位移与 Phase A 不完全可比。  
- 部分 world 中“commanded”等模型可能受插件控制，力施加效果与纯 F=ma 不一致，易超容差。

**环境扰动 (environment_perturbation_robustness)**  
- 现象：Phase A 与 Phase B 位移差超过 15%。  
- 可能原因：reset 后状态与“仅多一个远处静态模型”的理想假设不完全一致；或插入装饰模型后**实体顺序/编号**变化，某些实现若依赖全局状态会带来差异。  
- pendulum2、vehicle_green 等多次出现在 FAILED 中，可能与初始位姿、reset 精度或插件行为有关。

**关节约束 (joint_constraint_stability)**  
- 现象：约束距离漂移超过 0.05m 或出现 >0.1m 跳变。  
- **cylinder** 模型在多个 FAILED 中出现，且 Violations 较多：若为滚动/滑动关节，两 link 间距离本就会随运动变化，0.05m 漂移容差可能过严。  
- 个别 **vehicle** 用例为少量 drift 违规（如 2 次），可能处于容差边界或数值噪声。

### 2.3 实验流程上可能存在的不足

1. **None 原因不可见**  
   - 当前仅写“Test failed to execute (returned None)”，未区分：无可用模型、无地面、质量解析失败、link 解析失败、位移/高度异常等。  
   - 建议：在返回 None 前将**跳过原因**写入 `metamorphic_test_result.txt`（或单独 reason 文件），便于统计“因何跳过”和优化场景/筛选逻辑。

2. **mass_scaling_no_reset 与带控制器的模型**  
   - 对带 DiffDrive、JointController 等的模型，Phase B 行为可能受控制器影响，与“仅质量变化”的假设不符，易误判为 FAILED。  
   - 建议：在 `is_model_testable` 或本测试内，排除已知带强控制器的模型（或按需放宽容差并注明）。

3. **joint_constraint_stability 的模型与容差**  
   - 对“约束距离会随运动合理变化”的机构（如部分 cylinder、滑动关节），当前 0.05m 漂移容差可能过严。  
   - 建议：对这类模型要么排除，要么使用更宽松的容差或“只检跳变、不检漂移”的策略，并在文档中说明。

4. **环境扰动中的 reset 与可复现性**  
   - Reset 后是否严格回到同一初始状态（含接触、内部状态）会影响 d_A 与 d_B 可比性。  
   - 建议：在结果中记录“是否为 reset-compare 且无插装饰物”的对照；若可行，可对同一 world 多跑几次看 d_A 自身方差，以评估“基准噪声”。

---

## 三、改进建议汇总

| 优先级 | 建议 | 说明 |
|--------|------|------|
| 高 | 记录并输出“returned None”的具体原因 | 便于区分无地面、无质量、无多 link、位移异常等，优化场景选择与统计。 |
| 中 | 质量缩放测试排除或标注“带控制器”模型 | 减少因 DiffDrive 等导致的误 FAILED，或单独分析。 |
| 中 | 关节约束对 cylinder/滑动类模型放宽或排除 | 避免对“约束距离会合理变化”的机构过严判 FAIL。 |
| 低 | 环境扰动记录 reset 与装饰物插入顺序 | 便于复现与排查实体顺序/全局状态类问题。 |
| 低 | 为三种测试分别做“适用 world 列表”或标签 | 便于后续只对“适合”的 world 跑对应测试，提高有效执行率。 |

---

## 四、结论

- **数据与流程整体可用**：三种新蜕变关系均有执行到底并产生 PASSED/FAILED 的用例，失败标记与结果文件一致。  
- **主要现象**：约 80% 为“未执行到底”(None)，属**前置条件不满足**或**主动合理性跳过**，需通过“记录 None 原因”和场景筛选来优化。  
- **FAILED 中**：质量缩放易受控制器/摩擦影响，环境扰动易受 reset 与实体顺序影响，关节约束对部分机构（如 cylinder）容差偏严；建议按上表做针对性放宽或排除，并区分“真实 bug”与“场景/模型不适配”。

若需要，我可以基于上述建议在 `randomsmith_meta.py` 里给出具体改动方案（例如 None 原因写入、可选容差或排除列表）。
