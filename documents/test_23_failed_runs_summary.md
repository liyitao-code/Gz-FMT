# test_23 FAILED 轮次汇总表

数据路径：`/home/liyitao/workspace/meta/test_23`，共 **22** 个 FAILED 运行。

---

## 总表（按运行编号排序）

| 编号 | 测试类型 | 被测模型 | 主要失败指标 | 备注 |
|------|----------|----------|--------------|------|
| **_5** | 关节约束稳定性 (Joint Constraint) | vehicle | 2 次违规：drift≈0.065 m (chassis–front_left_wheel) | d0=0.96 m，漂移超 0.05 m |
| **_61** | 质量缩放无重置 (Mass-Scaling No-Reset) | commanded | 相对误差 **43.07%**，d2/d1=1.757 | d1=0.34 m, d2=0.60 m，容差 25% |
| **_69** | 质量缩放无重置 | vehicle_blue | 相对误差 **55.04%**，d2/d1=0.450 | d1=12.51 m, d2=5.63 m，k=0.75 |
| **_85** | 环境扰动鲁棒性 (Env Perturbation) | pendulum_with_base_mimic_slow_follows_fast | \|d_A−d_B\|/max = **24.46%** | \|d_A\|=22.41 m, \|d_B\|=16.93 m，容差 15% |
| **_123** | 关节约束稳定性 | cylinder | **19 次**违规，drift/jump 极大 (d 最高 9381 m) | cylinder_link–cylinder_link2，d0≈1.50 m |
| **_288** | 质量缩放无重置 | commanded | 相对误差 **40.56%**，d2/d1=1.682 | 与 _61 同模型，类似偏差 |
| **_304** | 质量缩放无重置 | vehicle_green | 相对误差 **80.10%**，d2/d1=**5.024** | 不稳定检查 **Fail** (超出 [0.3, 3.3]) |
| **_322** | 质量缩放无重置 | vehicle | 相对误差 **55.09%**，d2/d1=0.449 | d1=11.90 m, d2=5.35 m，k=0.75 |
| **_338** | 环境扰动鲁棒性 | pendulum2 | \|d_A−d_B\|/max = **60.99%** | \|d_A\|=0.16 m, \|d_B\|=0.15 m |
| **_354** | 质量缩放无重置 | commanded | 相对误差 **41.52%**，d2/d1=1.710 | 与 _61/_288 同模型 |
| **_484** | 关节约束稳定性 | vehicle | 6 次违规：drift≈0.065 m + jump≈0.10 m | chassis–front_left_wheel |
| **_505** | 质量缩放无重置 | vehicle | 相对误差 **55.09%**，d2/d1=0.449 | 与 _322 同模型类型 |
| **_572** | 环境扰动鲁棒性 | pendulum2 | \|d_A−d_B\|/max = **154.03%** | \|d_A\|=0.57 m, \|d_B\|=0.62 m，差异极大 |
| **_620** | 关节约束稳定性 | cylinder | **19 次**违规，d 最高约 41894 m | 与 _123 同模型，爆炸式漂移 |
| **_629** | 质量缩放无重置 | pendulum_with_base_mimic_fast_follows_slow | 相对误差 **50.65%**，d2/d1=0.493 | d1=36.68 m, d2=18.10 m，k=0.5 |
| **_654** | 关节约束稳定性 | cylinder | **23 次**违规，d 最高约 20892 m | cylinder_link–cylinder_link2 |
| **_719** | 关节约束稳定性 | cylinder | **23 次**违规，d 最高约 71640 m | 同上 |
| **_751** | 环境扰动鲁棒性 | pendulum2 | \|d_A−d_B\|/max = **36.69%** | \|d_A\|=1.91 m, \|d_B\|=1.22 m |
| **_852** | 环境扰动鲁棒性 | pendulum2 | \|d_A−d_B\|/max = **37.12%** | \|d_A\|=2.09 m, \|d_B\|=1.33 m |
| **_1111** | 关节约束稳定性 | cylinder | **19 次**违规，d 最高约 45726 m | 同上 cylinder 模式 |
| **_1148** | 环境扰动鲁棒性 | vehicle_green | \|d_A−d_B\|/max = **21.49%** | \|d_A\|=3.70 m, \|d_B\|=2.90 m |
| **_1203** | 质量缩放无重置 | vehicle_green | 相对误差 **26.50%**，d2/d1=1.361 | 刚超 25% 容差 |

---

## 按测试类型分表

### 1. 关节约束稳定性（7 个）

| 编号 | 模型 | Link 对 | 违规次数 | 典型现象 |
|------|------|---------|----------|----------|
| _5 | vehicle | chassis, front_left_wheel | 2 | 漂移 0.065 m |
| _123 | cylinder | cylinder_link, cylinder_link2 | 19 | 漂移/跳变极大，d 达 km 级 |
| _484 | vehicle | chassis, front_left_wheel | 6 | 漂移 + 跳变 ~0.10 m |
| _620 | cylinder | cylinder_link, cylinder_link2 | 19 | 同上，d 最高 ~42 km |
| _654 | cylinder | cylinder_link, cylinder_link2 | 23 | 同上，d 最高 ~21 km |
| _719 | cylinder | cylinder_link, cylinder_link2 | 23 | 同上，d 最高 ~72 km |
| _1111 | cylinder | cylinder_link, cylinder_link2 | 19 | 同上，d 最高 ~46 km |

### 2. 质量缩放无重置（9 个）

| 编号 | 模型 | 相对误差 | d2/d1 | k | 备注 |
|------|------|----------|-------|---|------|
| _61 | commanded | 43.07% | 1.757 | 1.5 | 多次出现 |
| _69 | vehicle_blue | 55.04% | 0.450 | 0.75 | |
| _288 | commanded | 40.56% | 1.682 | 1.5 | |
| _304 | vehicle_green | **80.10%** | **5.024** | 1.5 | 不稳定检查 Fail |
| _322 | vehicle | 55.09% | 0.449 | 0.75 | |
| _354 | commanded | 41.52% | 1.710 | 1.5 | |
| _505 | vehicle | 55.09% | 0.449 | 0.75 | |
| _629 | pendulum_with_base_mimic_fast_follows_slow | 50.65% | 0.493 | 0.5 | |
| _1203 | vehicle_green | 26.50% | 1.361 | 0.5 | 刚超 25% |

### 3. 环境扰动鲁棒性（6 个）

| 编号 | 模型 | \|d_A−d_B\|/max | \|d_A\| (m) | \|d_B\| (m) |
|------|------|------------------|-------------|-------------|
| _85 | pendulum_with_base_mimic_slow_follows_fast | 24.46% | 22.41 | 16.93 |
| _338 | pendulum2 | 60.99% | 0.16 | 0.15 |
| _572 | pendulum2 | **154.03%** | 0.57 | 0.62 |
| _751 | pendulum2 | 36.69% | 1.91 | 1.22 |
| _852 | pendulum2 | 37.12% | 2.09 | 1.33 |
| _1148 | vehicle_green | 21.49% | 3.70 | 2.90 |

---

## 简要结论（供你分析用）

- **关节约束**：**cylinder**（cylinder_link vs cylinder_link2）在 5 个轮次中出现极端漂移/跳变（两 link 距离从 ~1.5 m 增至数 km），疑似选错 link 对或该模型为滑动/滚动机构导致距离非恒定；**vehicle**（chassis–front_left_wheel）在 2 个轮次中为小幅漂移/跳变（~0.06–0.10 m）。
- **质量缩放**：**commanded** 出现 3 次、**vehicle**/vehicle_blue/vehicle_green 共 4 次、**pendulum_with_base_mimic** 1 次；多为相对误差 40%–55% 或 d2/d1 偏离 1.0，**vehicle_green** 在 _304 出现 d2/d1=5.024 且不稳定检查 Fail。
- **环境扰动**：**pendulum2** 出现 4 次、**pendulum_with_base_mimic** 1 次、**vehicle_green** 1 次；\|d_A−d_B\|/max 从 21% 到 154% 不等，_572 的 154% 为同一模型两阶段位移方向/量级差异极大，值得单独复现。

如需对某一编号做更细的数值或复现步骤，可指定编号再展开。
