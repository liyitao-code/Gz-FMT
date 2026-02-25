# _36 实验内容分析（Determinism Test）

## 1. 实验概要

- **目录**: `/home/liyitao/workspace/meta/test_22/_36`
- **测试类型**: Determinism Test（确定性重复测试）
- **世界/模型**: world `shapes`，模型 `cylinder`
- **结果**: FAILED — 两次独立运行终点位置相差 **11.1 m**（容差 1 mm）

## 2. 实验在做什么

Determinism 测试的**本质**是：用**两个完全独立的 Gazebo 进程**，各自加载**同一份 SDF**，执行**完全相同的操作序列**（暂停 → 清力 → 施加相同力 → 推进相同步数 → 清力），最后比较**同一模型**的终点位置。若仿真确定，则两次结果应一致。

### 2.1 操作序列（与 experiment_log.json 对应）

**Run 1（第一个 Gazebo 进程）**

1. 启动 Gazebo：`gz sim .../a.sdf --record-path .../log`
2. 等待约 2 s 让 Gazebo 就绪
3. （脚本内未记录）激活仿真：play → 立即 pause → sleep 0.3
4. 暂停：`gz service .../control --req 'pause: true'`
5. 清力：`gz topic .../wrench/clear`（cylinder）
6. 施力：`gz topic .../wrench/persistent`，力 F = (-14.73, 19.39, -43.67) N
7. sleep 0.1
8. 推进仿真：`gz service .../control --req 'multi_step: 5041'`（约 5.041 s 仿真时间）
9. sleep 5.041 s（壁钟等待）
10. 清力：`gz topic .../wrench/clear`
11. **记录 cylinder 终点位置 → Run 1 位置 P1**
12. **关闭第一个 Gazebo 进程**

**Run 2（第二个 Gazebo 进程）**

13. sleep 2 s（对应日志里 “Wait for second Gazebo to start”）
14. **重新启动 Gazebo**：同一份 `a.sdf`，**不录像**（由脚本内 `subprocess.Popen` 完成，**未写入 experiment_log.json**）
15. 等待约 2 s 让第二个 Gazebo 就绪
16. （脚本内未记录）对第二个进程做激活：play → 立即 pause → sleep 0.3
17. 暂停、清力、施**同一力 F**、sleep 0.1、**同一 multi_step: 5041**、sleep 5.041、清力
18. **记录 cylinder 终点位置 → Run 2 位置 P2**

比较 P1 与 P2 → 若差异 &gt; 1 mm 则判 FAILED。

### 2.2 _36 的数值结果

- **力**: F = (-14.729, 19.386, -43.672) N  
- **步数**: 5041（约 5.04 s 仿真时间）  
- **Run 1 终点**: (142.90, -1.50, -81.74) m  
- **Run 2 终点**: (133.27, -1.50, -76.23) m  
- **位置差**: Δx ≈ 9.64 m，Δy = 0 m，Δz ≈ 5.52 m，**误差模长 ≈ 11.1 m**  

y 完全一致说明两次运行在 y 方向行为一致；x、z 差异大，说明在相同输入下两次仿真的轨迹不一致，即**非确定性**。

## 3. 为什么 reproduce_experiment.py 复现不了 Determinism

当前 `reproduce_experiment.py` 的复现逻辑是：

- **只启动一次 Gazebo**
- 按 `experiment_log.json` 的**顺序**执行每条 command 和 sleep

而 Determinism 的日志里：

- **没有**“第二次启动 Gazebo”的 command（第二次启动在 `randomsmith_meta.py` 里用 `subprocess.Popen` 完成，未写入 log）
- “Wait for second Gazebo to start” 之后的 pause/clear/施力/multi_step/clear 是发给**第二个**进程的

因此若按现有逻辑“一条条回放”：

- Run 1 的指令正确发给了**第一个**（也是唯一一个）Gazebo
- “Wait for second Gazebo to start” 之后的那段会被发给**同一个**进程（没有重启）
- 相当于：**同一进程里先跑完 Run 1，再在同一状态下继续跑 Run 2**（cylinder 已不在初始位置），得到的不是“第二次独立运行”的终点，无法复现“两进程、同初态、同操作、比终点”的 Determinism 流程。

结论：要复现 Determinism，必须在**第一次 wrench/clear（Run 1 结束）并记录 P1 后**：

1. **关闭当前 Gazebo**
2. **再启动一个新的 Gazebo**（同一 SDF）
3. 对新进程做**激活**（play → pause → sleep 0.3）
4. 再执行 Run 2 的 pause → clear → 施力 → multi_step → clear，并记录 P2

下面在复现脚本里为 Determinism 增加上述“双进程”分支，并支持从 `metamorphic_test_result.txt` 解析 Run 1 / Run 2 位置用于比较。
