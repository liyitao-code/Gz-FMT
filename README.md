# Gz-FMT

这是一个面向 Gazebo / gz-sim 的自动化测试项目。项目主要围绕 SDF 世界文件生成、Gazebo 服务 / Topic 调用、模型与插件组合、覆盖率收集、崩溃检测、实验复现，以及蜕变测试展开。

当前仓库里同时包含了核心测试脚本、辅助分析脚本、模型语料、历史实验输出和本地 Gazebo 源码 checkout。准备上传 GitHub 时，建议只保留代码、必要模型语料和文档；实验输出、日志、大型源码目录和临时文件通过 `.gitignore` 排除。

## 项目结构

主要目录如下：

- `models/`：主要 SDF 输入语料库。`randomsmith.py`、`randomsmith_meta.py`、`agent_smith_meta.py` 等脚本会从这里随机选择 `*.sdf` 作为测试对象。
- `model_fuza/`、`models_old/`、`models_test/`：额外或历史 SDF 模型集合，可作为补充测试语料或回归样例。
- `documents/`：实验说明、蜕变关系设计、复现说明、问题记录等文档。
- `tests/`：轻量测试，目前主要用于检查 `agent_smith_meta.py` 的强化学习蜕变关系选择逻辑是否接线完整。
- `src/`：本地 Gazebo 相关源码 checkout，用于调试和覆盖率分析；体积较大，不建议随本仓库上传。
- `exp_*`、`test_exp/`：历史实验输出目录，通常包含 `a.sdf`、`cmd_*.sh`、`gz.out`、`gz.err`、`id` 等。
- `tosubmit/`：整理过的崩溃复现包或待提交问题材料。
- `bug/`：崩溃日志、valgrind 报告和问题样例。
- `gcov/`：覆盖率中间文件和报告。

## 核心测试脚本

### `randomsmith.py`

随机 Gazebo 操作测试脚本。它从 `models/` 中复制一个 SDF 到实验目录，然后启动 `gz sim`，随机执行一系列 Gazebo 操作，例如：

- 加载随机模型；
- 移除模型；
- 修改模型位置；
- 给模型添加插件；
- 调用随机 service；
- 向随机 topic 发布消息；
- 记录命令、输出、覆盖率差异和崩溃信息。

主要依赖：

- `modelsmith.py`：SDF / model 生成与基础常量。
- `plugin_mining.py`：从 Gazebo world / model 中挖掘插件片段。
- `coverage_process.py`：gcov 覆盖率收集和差异计算。
- `sdf_diversity.py`：SDF 多样性判断。
- `crash_result.py`：解析 `gz.err` 中的 stack trace，用于去重崩溃。
- `search_plugin_in_model.py`、`search_plugin_in_world.py`、`search_model_with_plugin.py`：按索引检索插件或带插件模型。

示例：

```bash
python3 randomsmith.py -d exp_random -i 10 -n 10 -s 12345 -t 10000
```

常用参数：

- `-d, --directory`：实验输出目录前缀，实际输出为 `<prefix>_<轮次>`。
- `-i, --iteration`：迭代轮数。
- `-n, --num-seq`：每轮执行的 Gazebo 操作数量。
- `-s, --seed`：随机种子。
- `-p, --plugin`：启用插件挖掘语料。
- `-t, --timeout`：Gazebo service 请求超时时间。

### `muti_agent_smith.py`

强化学习版本的 Gazebo 操作测试脚本。它和 `randomsmith.py` 的测试目标类似，但不再完全随机选择操作，而是使用 actor-critic 模型选择操作序列。

脚本中的强化学习结构：

- 高层 actor：选择操作类型，例如添加模型、删除模型、添加插件、调用 service/topic 等。
- critic：估计当前状态价值。
- 低层 policy：当某些操作需要参数时，选择插件索引或模型索引等参数。
- reward：结合崩溃、覆盖率增长和操作序列多样性。

主要依赖：

- `coverage_process.py`：用于 coverage reward。
- `plugin_mining.py`、`search_*`：用于插件 / 模型参数选择。
- `sdf_diversity.py`：用于多样性反馈。
- `crash_result.py`：用于崩溃去重。
- `torch`、`numpy`、`scipy`：用于策略网络和多样性距离计算。

示例：

```bash
python3 muti_agent_smith.py -d exp_agent -i 10 -n 20 -s 12345 -t 10000
```

输出中除了普通实验目录，还会打印 reward、coverage 和 action sequence 相关调试信息。

### `randomsmith_meta.py`

随机蜕变测试脚本。它从 `models/` 中随机选择 SDF，启动 Gazebo，并随机选择一种蜕变关系进行测试。

已经实现的蜕变关系包括：

- `motion`
- `rewind`
- `force_additivity`
- `time_scaling`
- `mass_scaling`
- `determinism`
- `symmetry`
- `zero_input_stability`
- `force_isolation`
- `force_removal`
- `temporal_monotonicity`
- `joint_constraint_stability`
- `mass_scaling_no_reset`
- `environment_perturbation_robustness`

脚本会生成：

- `metamorphic_test_result.txt`：蜕变关系测试结果。
- `experiment_log.json`：实验过程、命令和等待时间记录。
- `playback_test_result.txt`：启用 playback 时的回放回溯测试结果。
- `METAMORPHIC_TEST_FAILED`、`PLAYBACK_TEST_FAILED`、`BOTH_CRASH_AND_METAMORPHIC_FAIL`：用于标记有价值的失败样例。

示例：

```bash
python3 randomsmith_meta.py -d exp_meta -i 10 -n 1 --disable-playback -s 12345
```

播放回溯测试默认启用，可按实验成本关闭：

```bash
python3 randomsmith_meta.py -d exp_meta -i 10 --disable-playback
```

### `agent_smith_meta.py`

强化学习版本的蜕变测试脚本。它保留 `randomsmith_meta.py` 的蜕变测试实现，但把“随机选择蜕变关系”替换成“使用 actor-critic 模型选择蜕变关系”。

和 `muti_agent_smith.py` 的区别：

- `muti_agent_smith.py` 是两层选择：先选操作，再对部分操作选参数。
- `agent_smith_meta.py` 只选蜕变关系，不做低层参数网络。
- 具体测试参数仍由每个 `metamorphic_test_*` 方法内部随机生成。

RL 状态主要来自当前 SDF：

- 模型数量；
- model 内插件数量；
- world 级插件数量；
- joint 数量；
- link 数量；
- static model 数量；
- 是否有 world；
- 当前蜕变关系历史选择分布。

reward 默认考虑：

- 蜕变测试失败；
- playback 失败；
- 新崩溃；
- 测试是否成功执行；
- 蜕变关系选择多样性。

示例：

```bash
python3 agent_smith_meta.py -d exp_agent_meta -i 10 -n 1 --disable-playback --epsilon 0.1 --lr 0.001
```

随机对照模式：

```bash
python3 agent_smith_meta.py -d exp_agent_meta_random -i 10 -n 1 --disable-playback --random-meta
```

额外参数：

- `--epsilon`：探索率。
- `--lr`：actor / critic 学习率。
- `--meta-history-size`：用于多样性 reward 的历史窗口长度。
- `--random-meta`：关闭 RL，使用随机蜕变关系选择，方便做对照实验。

训练指标会写入：

```text
<directory>_meta_training_metrics.csv
```

## 辅助脚本

### 模型与插件语料处理

- `modelsmith.py`：SDF / model 生成基础工具，定义 `RootGen`、`ModelGen`、`POSE`、`PLUGIN_DIR` 等。
- `plugin_mining.py`：扫描 Gazebo world / model，抽取可复用插件片段。
- `search_model_all.py`：搜索模型文件。
- `search_model_with_plugin.py`：搜索带插件模型，并支持按索引取回。
- `search_plugin_in_model.py`：搜索 model 内插件。
- `search_plugin_in_world.py`：搜索 world 级插件。
- `filter_plugins.py`：过滤插件列表。
- `extract_topic_service.py`：提取 Gazebo 内置 topic / service 信息。

常见用法：

```bash
python3 search_model_all.py
python3 search_model_with_plugin.py
python3 search_plugin_in_model.py
python3 search_plugin_in_world.py
python3 plugin_mining.py
```

这些脚本通常会读取 Gazebo 安装目录、`models/` 或项目中的插件列表文件，并生成 `models_all.txt`、`models_with_plugins.txt`、`unique_plugins.txt`、`world_level_plugins.txt` 等缓存文件。

### 覆盖率与结果分析

- `coverage_process.py`、`coverage_process_new.py`：收集 gcov 数据并计算新覆盖行 / 文件。
- `analyze_results.py`：分析实验输出结果。
- `analyze_experiments.py`：批量分析多个实验目录。
- `analyze_timing_filter.py`：分析和过滤 timing 相关结果。
- `screen_bug_candidates.py`：筛选潜在 bug 候选样例。
- `sdf_diversity.py`：计算或判断 SDF 输入多样性。

示例：

```bash
python3 analyze_results.py
python3 analyze_experiments.py
python3 screen_bug_candidates.py
python3 coverage_process.py
```

### 崩溃复现与 replay

- `replay.py`：基础 replay 脚本。
- `auto_replay.py`、`auto_replay_1.py`：自动 replay 辅助脚本。
- `replay_experiment.py`：按实验目录复现实验。
- `reproduce_experiment.py`：复现已有实验，并整理复现输出。
- `reproduce_experiment_timed.py`：带时间控制的复现脚本。
- `crash_result.py`：解析 stack trace 并做崩溃去重。
- `crash_search.py`、`crash_dirct.py`：查找和整理崩溃结果。

示例：

```bash
python3 replay_experiment.py
python3 reproduce_experiment.py
python3 reproduce_experiment_timed.py
python3 crash_search.py
```

### 其他功能脚本

- `servicesmith.py`：偏 service / topic 调用方向的测试脚本。
- `banditsmith.py`：多臂老虎机方向的早期测试策略脚本。
- `randomsmith_large.py`、`large_muti_agent_smith.py`：大参数空间或旧版本测试脚本。
- `randomsmith_fixture.py`、`testfixture_smith.py`、`testfixture_smith_1.py`：结合 test fixture 的测试脚本。
- `operator_executor.py`、`operator_executor_smith.py`：操作执行器封装。
- `gazebo_launcher.py`：Gazebo 启动封装。
- `random_controller.py`、`test_controller.py`：控制器相关测试。
- `random_fixture.py`、`random_fixture_1.py`、`free_testfixture.py`：测试夹具辅助逻辑。
- `classification.py`：结果分类辅助脚本。

## 运行环境

脚本主要面向 Linux + Gazebo / gz-sim 开发环境。部分脚本中有硬编码路径，例如：

```python
/home/liyitao/workspace/gz_lastest
/home/liyitao/workspace/install/lib/python
```

迁移到新机器时，需要检查：

- `FIRST_DIR`
- `DIR_FLAG`
- `sys.path.append(...)`
- Gazebo Python 绑定版本，例如 `gz.msgs10`、`gz.msgs11`、`gz.transport13`、`gz.transport14`
- `BUILD_DIR`、`GCOV_DIR`
- `PLUGIN_DIR`

Python 依赖主要包括：

- `lxml`
- `psutil`
- `func_timeout`
- `numpy`
- `torch`
- `scipy`
- `scikit-learn`
- `pytest`，仅用于轻量测试

## 输出文件说明

单轮实验目录通常包含：

- `a.sdf`：本轮输入 SDF。
- `cmd_*.sh`：生成的 Gazebo 命令。
- `world_*.sdf`：执行过程中 dump 出来的 world。
- `gz.out`、`gz.err`：Gazebo 标准输出和错误输出。
- `id`：当前执行到的命令编号。
- `experiment_log.json`：蜕变测试过程日志。
- `metamorphic_test_result.txt`：蜕变测试结果。
- `playback_test_result.txt`：playback 测试结果。
- `METAMORPHIC_TEST_FAILED` 等标记文件。

这些文件适合本地分析和复现，但一般不建议直接提交到 GitHub。

## 上传 GitHub 前建议

建议保留：

- 核心 Python 脚本；
- `models/` 中必要的最小测试语料；
- 必要文档；
- 小型配置文件，例如 `bridge_config.yaml`；
- `tests/` 中的轻量测试。

建议排除：

- `src/` 本地 Gazebo 源码 checkout；
- `gcov/` 覆盖率中间结果；
- `exp_*`、`test_exp/` 实验输出；
- `bug/`、`tosubmit/` 中的大型复现包和日志；
- `valgrind*.log`、`*.out`、`nohup.out`；
- 临时 `a.sdf`、`world_*.sdf`、根目录截图和 html；
- `code.tar.xz`、`*.tar.gz` 等压缩包；
- Python / pytest 缓存。

如果这些文件已经被 Git 跟踪，更新 `.gitignore` 不会自动取消跟踪。可以先检查：

```bash
git status --short
git ls-files | grep -E '^(src/|exp_|bug/|tosubmit/|gcov/)|(\.log$|\.out$|\.tar\.gz$|\.xz$)'
```

确认无误后，再用 `git rm --cached` 从索引中移除但保留本地文件，例如：

```bash
git rm -r --cached src gcov exp_0 exp_1 exp_2 exp_3 exp_4 test_exp bug tosubmit
git rm --cached code.tar.xz valgrind.log valgrind-1.log valgrind-debug.log valgrind-gz-sim.log nohup.out
```

执行前建议先备份或单独开分支确认，因为这会改变 Git 索引。

## 基本验证

不启动 Gazebo 的静态检查：

```bash
python -m py_compile agent_smith_meta.py
python -c "import tests.test_agent_smith_meta_rl as t; t.test_meta_relation_action_space_contains_all_known_relations(); t.test_meta_rl_components_are_wired_into_main_loop(); t.test_generate_and_test_commands_returns_structured_result(); print('manual static tests passed')"
```

如果安装了 `pytest`：

```bash
python -m pytest tests/test_agent_smith_meta_rl.py -q
```

端到端运行需要完整 Gazebo 环境。
