# 命令行复现施加力和速度的指令格式

本文档说明如何通过命令行手动复现 `func_apply_model_force()` 和 `func_set_model_velocity()` 的功能。

## 前置条件

确保你的 Gazebo 世界文件中加载了 `gz-sim-apply-link-wrench-system` 插件：

```xml
<plugin
  filename="gz-sim-apply-link-wrench-system"
  name="gz::sim::systems::ApplyLinkWrench">
</plugin>
```

## 1. 施加力（func_apply_model_force）

### 基本格式

```bash
gz topic -t /world/<world_name>/wrench -m gz.msgs.EntityWrench -p '<EntityWrench消息内容>'
```

### 瞬时力（persistent=False）

```bash
gz topic -t /world/<world_name>/wrench -m gz.msgs.EntityWrench -p 'entity: {name: "<model_name>", type: MODEL}, wrench: {force: {x: <force_x>, y: <force_y>, z: <force_z>}, torque: {x: <torque_x>, y: <torque_y>, z: <torque_z>}}'
```

### 持续力（persistent=True）

```bash
gz topic -t /world/<world_name>/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "<model_name>", type: MODEL}, wrench: {force: {x: <force_x>, y: <force_y>, z: <force_z>}, torque: {x: <torque_x>, y: <torque_y>, z: <torque_z>}}'
```

### 实际示例

假设：
- world_name = `world_0`
- model_name = `my_turtle`
- 施加沿 x 轴正方向 100 牛顿的力（持续）
- 无力矩

```bash
gz topic -t /world/world_0/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "my_turtle", type: MODEL}, wrench: {force: {x: 100.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

### 清除持续力

```bash
gz topic -t /world/<world_name>/wrench/clear -m gz.msgs.Entity -p 'name: "<model_name>", type: MODEL'
```

示例：
```bash
gz topic -t /world/world_0/wrench/clear -m gz.msgs.Entity -p 'name: "my_turtle", type: MODEL'
```

## 2. 设置速度（func_set_model_velocity）

`func_set_model_velocity()` 实际上是通过施加持续力来实现的。它会根据速度计算力的大小，然后调用 `func_apply_model_force()`。

### 计算方式

代码中的计算方式：
```python
force_magnitude = 100.0  # 固定系数
force_x = velocity_x * force_magnitude
force_y = velocity_y * force_magnitude
force_z = velocity_z * force_magnitude
```

### 实际命令

假设：
- world_name = `world_0`
- model_name = `my_turtle`
- 速度：x=1.0 m/s, y=0.0 m/s, z=0.0 m/s

计算力：
- force_x = 1.0 * 100.0 = 100.0 N
- force_y = 0.0 * 100.0 = 0.0 N
- force_z = 0.0 * 100.0 = 0.0 N

命令：
```bash
gz topic -t /world/world_0/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "my_turtle", type: MODEL}, wrench: {force: {x: 100.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 3. 完整复现示例

### 场景：让模型沿 x 轴运动 5 秒

1. **启动 Gazebo**（假设使用 world_0）：
```bash
gz sim your_world.sdf
```

2. **确保模拟器运行**（不是暂停状态）：
```bash
gz service -s /world/world_0/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
```

3. **施加持续力**（让模型沿 x 轴运动）：
```bash
gz topic -t /world/world_0/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "my_turtle", type: MODEL}, wrench: {force: {x: 100.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

4. **等待 5 秒**（让模型运动）

5. **清除力**（可选，如果想停止运动）：
```bash
gz topic -t /world/world_0/wrench/clear -m gz.msgs.Entity -p 'name: "my_turtle", type: MODEL'
```

## 4. 从测试结果文件复现

如果你有测试结果文件 `metamorphic_test_result.txt`，可以从中提取信息：

```
Model: my_turtle
Initial Position: (0.000, 0.000, -0.002)
Final Position: (2.116, 0.000, -0.002)
Expected Position: (2.116, 0.000, -0.002)
Test Duration: 3.55 s
```

假设测试中使用的速度是 1.0 m/s（根据 Expected Position 和 Test Duration 可以反推），那么：

1. 计算速度：velocity_x = (2.116 - 0.000) / 3.55 ≈ 0.596 m/s
2. 计算力：force_x = 0.596 * 100.0 ≈ 59.6 N
3. 复现命令：
```bash
gz topic -t /world/world_0/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "my_turtle", type: MODEL}, wrench: {force: {x: 59.6, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 5. 注意事项

1. **世界名称**：需要替换 `<world_name>` 为实际的世界名称（通常是 `world_0`）
2. **模型名称**：需要替换 `<model_name>` 为实际的模型名称
3. **力的单位**：牛顿（N）
4. **力矩的单位**：牛顿·米（N·m）
5. **持续力**：使用 `/wrench/persistent` topic，力会持续施加直到明确清除
6. **瞬时力**：使用 `/wrench` topic，力只施加一次
7. **插件要求**：必须加载 `gz-sim-apply-link-wrench-system` 插件

## 6. 获取模型位置信息

### ⚠️ 重要提示

**`scene/info` 服务返回的是模型的初始位置，不是实时位置！**

如果你在 Gazebo 图形界面中移动了模型，使用 `scene/info` 服务获取的位置仍然是初始位置，不会反映实时变化。

### 方法1：使用 pose/info 或 dynamic_pose/info topic（推荐，获取实时位置）

这些 topic 会实时发布模型的位置信息，反映模型的当前状态。

#### 获取所有模型的实时位置

```bash
gz topic -e -t /world/<world_name>/pose/info -n 1
```

**注意：** 使用 `-e` (或 `--echo`) 参数来输出消息内容，`-n 1` 表示只接收一条消息后退出。

#### 获取动态模型的实时位置

```bash
gz topic -e -t /world/<world_name>/dynamic_pose/info -n 1
```

#### 持续监听位置变化

如果想持续监听位置变化（不自动退出），可以去掉 `-n 1`：

```bash
gz topic -e -t /world/<world_name>/dynamic_pose/info
```

#### 查看特定模型的位置

```bash
# 获取所有模型的 pose 并过滤特定模型
gz topic -e -t /world/<world_name>/dynamic_pose/info -n 1 | grep -A 10 "name: \"<model_name>\""
```

例如，查看 "ellipsoid" 模型的实时位置：
```bash
gz topic -e -t /world/shapes/dynamic_pose/info -n 1 | grep -A 10 "name: \"ellipsoid\""
```

输出示例：
```
name: "ellipsoid"
id: 5
position {
  x: 0.0
  y: 3.0
  z: 0.5
}
orientation {
  w: 1.0
  x: 0.0
  y: 0.0
  z: 0.0
}
```

**重要提示：** 
- `-e` (或 `--echo`) 参数是必需的，用于输出消息内容
- `-n 1` 参数表示只接收一条消息后退出。如果想持续监控位置变化，可以去掉 `-n 1` 参数
- 不需要 `-m` 参数，因为接收消息时 topic 已经知道消息类型

### 方法2：使用 scene/info 服务（仅获取初始位置，不推荐用于实时位置）

```bash
gz service -s /world/<world_name>/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req ''
```

⚠️ **警告：** 这个方法返回的是模型的初始位置，不会反映模型在模拟过程中的位置变化。如果你在图形界面中移动了模型，这个服务仍然会返回初始位置。

### 方法3：查看特定模型的初始位置（不推荐用于实时位置）

```bash
# 获取场景信息并过滤特定模型
gz service -s /world/<world_name>/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req '' | grep -A 10 "name: \"<model_name>\""
```

例如，查看 "ellipsoid" 模型的初始位置：
```bash
gz service -s /world/shapes/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req '' | grep -A 10 "name: \"ellipsoid\""
```

输出示例：
```
name: "ellipsoid"
id: 5
pose {
  position {
    x: 0.0
    y: -1.5
    z: 0.5
  }
  orientation {
    w: 1.0
    x: 0.0
    y: 0.0
    z: 0.0
  }
}
```

### 方法3：使用 Python 脚本解析（更精确）

```python
from gz.transport14 import Node
from gz.msgs11.empty_pb2 import Empty
from gz.msgs11.scene_pb2 import Scene

node = Node()
world_name = "shapes"  # 替换为你的世界名称
service_name = f"/world/{world_name}/scene/info"

result, scene = node.request(service_name, Empty(), Empty, Scene, 3000)
if result:
    for model in scene.model:
        if model.name == "ellipsoid":  # 替换为你的模型名称
            print(f"Model: {model.name}")
            print(f"Position: ({model.pose.position.x}, {model.pose.position.y}, {model.pose.position.z})")
            break
```

## 7. 调试技巧

### 查看当前世界的所有模型
```bash
gz service -s /world/world_0/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req ''
```

### 查看模型位置（更详细的输出）
```bash
# 获取所有模型名称
gz service -s /world/world_0/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req '' | grep "name:"

# 查看特定模型的完整信息
gz service -s /world/world_0/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req '' | grep -A 15 "name: \"ellipsoid\""
```

### 检查模拟器是否运行
```bash
gz topic --echo --topic /stats -n 1
```
查看输出中的 `paused` 字段，如果为 `false` 则表示正在运行。

### 实时监控模型位置变化
```bash
# 每1秒获取一次模型位置
watch -n 1 'gz service -s /world/shapes/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req "" | grep -A 5 "name: \"ellipsoid\""'
```

## 8. 回溯功能（Rewind/Seek）

### 获取当前模拟时间

```bash
gz topic -e -t /stats -m gz.msgs.WorldStatistics -n 1
```

输出示例：
```
sim_time {
  sec: 5
  nsec: 123456789
}
real_time {
  sec: 5
  nsec: 234567890
}
iterations: 5000
paused: false
real_time_factor: 1.0
```

从输出中可以看到 `sim_time.sec` 和 `sim_time.nsec` 就是当前的模拟时间。

### 记录所有模型的状态

```bash
gz topic -e -t /world/<world_name>/pose/info -m gz.msgs.Pose_V -n 1
```

例如：
```bash
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1
```

输出示例：
```
pose {
  name: "ellipsoid"
  id: 5
  position {
    x: 0.0
    y: -1.5
    z: 0.5
  }
  orientation {
    w: 1.0
    x: 0.0
    y: 0.0
    z: 0.0
  }
}
pose {
  name: "box"
  id: 6
  position {
    x: 1.0
    y: 0.0
    z: 0.5
  }
  ...
}
```

### 回溯到指定的模拟时间

**重要：** `seek` 功能只改变模拟时间，**不会恢复模型的状态（位置、速度等）**。如果你需要恢复模型位置，必须手动使用 `set_pose` 服务。

#### 方法1：使用 seek（仅改变时间，不恢复状态）

```bash
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'seek: {sec: <target_sec>, nsec: <target_nsec>}'
```

例如，回溯到模拟时间 2.5 秒：
```bash
gz service -s /world/shapes/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'seek: {sec: 2, nsec: 500000000}'
```

**注意：** 
- `seek` 功能主要用于 log playback 模式
- 对于实时模拟，`seek` 可能不可用或只改变时间，不恢复模型状态
- 即使 `seek` 成功，模型的位置也不会改变

#### 方法2：恢复模型位置（推荐）

如果需要恢复模型位置，必须使用 `set_pose` 服务逐个恢复每个模型的位置：

```bash
# 恢复单个模型的位置
gz service -s /world/<world_name>/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "<model_name>", position: {x: <x>, y: <y>, z: <z>}, orientation: {w: <w>, x: <x>, y: <y>, z: <z>}'
```

例如，恢复 "ellipsoid" 模型的位置：
```bash
gz service -s /world/shapes/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "ellipsoid", position: {x: 0.0, y: -1.5, z: 0.5}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}'
```

**完整流程：**
1. 在记录时间点，记录所有模型的位置
2. 继续运行模拟
3. 使用 `set_pose` 服务逐个恢复每个模型的位置（不改变时间）
4. 对比恢复后的状态和记录的状态

**注意：**
- `sec` 是目标时间的秒部分（整数）
- `nsec` 是目标时间的纳秒部分（整数，0-999999999）
- 例如，2.5 秒 = `sec: 2, nsec: 500000000`
- 例如，3.123456789 秒 = `sec: 3, nsec: 123456789`

### 完整的手动复现流程

假设你要复现回溯测试：

1. **启动 Gazebo 模拟**：
```bash
gz sim your_world.sdf -r
```

2. **等待到记录时间点（例如 2 秒）**：
```bash
# 等待 2 秒（真实时间）
sleep 2

# 或者等待模拟时间达到 2 秒（更准确）
# 持续检查模拟时间直到达到目标
while true; do
  sim_time=$(gz topic -e -t /stats -m gz.msgs.WorldStatistics -n 1 | grep -A 2 "sim_time" | grep "sec:" | awk '{print $2}')
  if [ "$sim_time" -ge 2 ]; then
    break
  fi
  sleep 0.1
done
```

3. **记录所有模型的状态**：
```bash
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_before.txt
```

4. **继续运行一段时间（例如 3 秒）**：
```bash
# 等待模拟时间再增加 3 秒
sleep 3
```

5. **恢复模型位置到之前记录的状态**：
```bash
# 恢复每个模型的位置（需要根据之前记录的状态逐个恢复）
# 例如，恢复 "ellipsoid" 模型：
gz service -s /world/shapes/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 --req 'name: "ellipsoid", position: {x: 0.0, y: -1.5, z: 0.5}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}'

# 如果有多个模型，需要逐个恢复
# 注意：position 和 orientation 的值应该从 state_before.txt 中获取
```

6. **等待回溯完成**：
```bash
sleep 1
```

7. **获取回溯后的状态**：
```bash
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_after.txt
```

8. **对比两个状态文件**：
```bash
diff state_before.txt state_after.txt
```

如果回溯功能正常工作，两个文件应该相同（或非常接近，允许小的数值误差）。

### 其他控制命令

#### 重置到初始状态（Rewind to time 0）

```bash
gz service -s /world/<world_name>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'
```

或者使用 `rewind` 字段（如果支持）：
```bash
gz service -s /world/<world_name>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'rewind: true'
```

#### 暂停/恢复模拟

```bash
# 暂停
gz service -s /world/<world_name>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'

# 恢复
gz service -s /world/<world_name>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
```

### 注意事项

1. **时间精度**：模拟时间的精度是纳秒级别，但在实际使用中，毫秒级别的精度通常就足够了。

2. **回溯限制**：
   - 只能回溯到模拟开始之后的时间点，不能回溯到负时间
   - `seek` 功能主要用于 log playback 模式，对于实时模拟可能不适用

3. **Log Playback vs 实时模拟**：
   - `LogPlaybackControl` 的 `seek` 功能主要用于回放已记录的 log 文件
   - 对于实时模拟，可能需要先启用 log recording（使用 `gz sim --record`），然后使用 playback 模式
   - 如果需要在实时模拟中回溯，可能需要使用其他方法（如 `reset` 功能重置到初始状态）

4. **状态一致性**：回溯后，所有模型的状态（位置、速度、角度等）应该恢复到目标时间点的状态。

5. **性能影响**：回溯操作可能需要一些时间来完成，特别是如果回溯的时间跨度很大。

6. **实时时间 vs 模拟时间**：`/stats` topic 中的 `sim_time` 是模拟时间，`real_time` 是真实时间。回溯操作只影响模拟时间，不影响真实时间。

### 替代方案：使用 Reset 功能

如果 `seek` 功能不可用，可以使用 `reset` 功能重置模拟到初始状态：

```bash
# 重置所有状态（包括时间、模型位置等）
gz service -s /world/<world_name>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}'

# 只重置时间
gz service -s /world/<world_name>/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {time_only: true}'
```

**注意：** `reset` 只能重置到初始状态（时间 0），不能重置到任意时间点。

## 9. Log Playback 模式和 Seek 功能

### 重要说明

**Log Playback 模式是唯一支持完整状态回溯的方式**。在实时模拟中，`seek` 只改变时间，不恢复状态。但在 log playback 模式下，`seek` 可以从 log 文件中恢复完整的状态（包括模型位置、速度等）。

### 完整的手动测试流程

#### 步骤 1：记录模拟（Record）

首先，启动模拟并记录状态到 log 文件：

```bash
# 记录模拟到指定目录
gz sim your_world.sdf -r --record-path /tmp/test_log

# 或者记录到默认目录（~/.gz/sim/log/<timestamp>）
gz sim your_world.sdf -r --record
```

**重要参数：**
- `--record-path <path>`: 指定记录路径
- `--record`: 记录到默认路径
- `-r`: 自动运行模拟（不是暂停状态）
- `--log-overwrite`: 如果路径已存在，覆盖它（默认会追加数字）

**记录的内容：**
- 模拟状态（模型位置、速度等）
- 控制台日志
- 如果使用 `--record-resources`，还会记录模型和纹理

**停止记录：**
- 按 `Ctrl+C` 停止模拟，记录会自动保存

#### 步骤 2：回放模拟（Playback）

使用记录的 log 文件回放模拟：

```bash
# 回放之前记录的 log
gz sim -r -v 4 --playback /tmp/test_log
```

**重要参数：**
- `--playback <path>`: 指定 log 文件路径
- `-r`: 自动运行回放（不是暂停状态）
- `-v 4`: 详细输出（可选）

**⚠️ 重要：** 必须使用 `--playback` 参数启动模拟，`playback/control` 服务才可用。如果只是正常启动模拟（没有 `--playback`），该服务不存在，调用会超时。

#### 步骤 2.5：验证 Playback 模式是否启用

在尝试使用 `playback/control` 服务之前，先检查服务是否存在：

```bash
# 列出所有可用的服务
gz service --list

# 或者检查特定的 playback 服务是否存在
gz service -i -s /world/<world_name>/playback/control
```

如果服务不存在，你会看到错误信息。这意味着模拟器没有在 playback 模式下运行。

**检查方法：**
```bash
# 检查服务信息（如果服务存在，会显示信息；如果不存在，会报错）
gz service -i -s /world/shapes/playback/control
```

#### 步骤 3：在 Playback 模式下使用 Seek

在 playback 模式下，可以使用 `LogPlaybackControl` 的 `seek` 功能来回溯到指定时间：

**⚠️ 前提条件：** 确保模拟器是用 `--playback` 参数启动的，否则 `playback/control` 服务不存在。

**获取当前模拟时间：**
```bash
gz topic -e -t /stats -m gz.msgs.WorldStatistics -n 1
```

**回溯到指定时间（例如 2.5 秒）：**
```bash
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'seek: {sec: 2, nsec: 500000000}'
```

例如：
```bash
gz service -s /world/shapes/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'seek: {sec: 2, nsec: 500000000}'
```

**如果服务调用超时，可能的原因：**
1. **模拟器没有使用 `--playback` 参数启动**（最常见的原因）
2. 服务名称错误（检查 world_name 是否正确）
3. 模拟器还没有完全启动（等待几秒后重试）

**调试步骤：**
```bash
# 1. 确认模拟器正在运行
ps aux | grep "gz sim"

# 2. 检查服务是否存在（如果服务不存在，会报错）
gz service -i -s /world/shapes/playback/control

# 3. 列出所有可用服务，查找 playback 相关服务
gz service --list | grep playback

# 如果看不到 playback/control 服务，说明没有在 playback 模式下运行

# 4. 检查 world 名称是否正确
gz topic -e -t /stats -m gz.msgs.WorldStatistics -n 1
# 或者
gz service --list | grep "/world/"

# 5. 确认启动命令包含 --playback
# 正确的启动命令应该是：
# gz sim -r --playback /path/to/log
# 而不是：
# gz sim -r your_world.sdf  （这样不会启用 playback 模式）
```

**常见错误：**
- ❌ 错误：`gz sim your_world.sdf -r` （正常模式，没有 playback）
- ✅ 正确：`gz sim -r --playback /tmp/test_log` （playback 模式）

**重要提示：**
- 在 playback 模式下，不需要指定 SDF 文件，只需要指定 log 路径
- 如果同时指定了 SDF 文件和 `--playback`，可能会出错
- Playback 模式会从 log 文件中加载世界状态，而不是从 SDF 文件

**其他 Playback 控制命令：**

```bash
# 暂停回放
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'

# 继续回放
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'

# 回溯到开始
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'rewind: true'

# 向前跳转（相对）
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'multi_step: 100'

# 向后跳转（相对，负数）
gz service -s /world/<world_name>/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'multi_step: -100'
```

#### 步骤 4：验证状态恢复

在 playback 模式下使用 `seek` 后，验证模型状态是否正确恢复：

**1. 记录 seek 前的状态：**
```bash
# 在 seek 之前，记录当前模型位置
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_before_seek.txt
```

**2. 执行 seek：**
```bash
gz service -s /world/shapes/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'seek: {sec: 2, nsec: 0}'
```

**3. 等待 seek 完成：**
```bash
sleep 2  # 等待 seek 操作完成
```

**4. 记录 seek 后的状态：**
```bash
# 在 seek 之后，记录当前模型位置
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_after_seek.txt
```

**5. 对比状态：**
```bash
# 对比两个状态文件
diff state_before_seek.txt state_after_seek.txt
```

如果 `seek` 功能正常工作，模型的位置应该恢复到 log 文件中该时间点的状态。

### 完整的蜕变测试手动流程

假设你要测试：在时间 a 记录状态，运行到时间 b，然后回溯到时间 a，验证状态是否一致。

**1. 记录模拟（包含时间 a 到时间 b 的完整过程）：**
```bash
# 启动模拟并记录
gz sim your_world.sdf -r --record-path /tmp/test_log

# 让模拟运行足够长的时间（例如 10 秒），确保包含时间 a 和 b
# 然后按 Ctrl+C 停止
```

**2. 回放模拟并记录时间 a 的状态：**
```bash
# 启动 playback
gz sim -r --playback /tmp/test_log

# 等待到时间 a（例如 2 秒）
# 可以通过持续检查 /stats topic 来确认时间
while true; do
  sim_time=$(gz topic -e -t /stats -m gz.msgs.WorldStatistics -n 1 | grep -A 2 "sim_time" | grep "sec:" | awk '{print $2}')
  if [ "$sim_time" -ge 2 ]; then
    break
  fi
  sleep 0.1
done

# 记录时间 a 的状态
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_at_time_a.txt
```

**3. 继续运行到时间 b：**
```bash
# 等待到时间 b（例如 5 秒）
while true; do
  sim_time=$(gz topic -e -t /stats -m gz.msgs.WorldStatistics -n 1 | grep -A 2 "sim_time" | grep "sec:" | awk '{print $2}')
  if [ "$sim_time" -ge 5 ]; then
    break
  fi
  sleep 0.1
done

# 记录时间 b 的状态（应该与时间 a 不同）
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_at_time_b.txt
```

**4. 回溯到时间 a：**
```bash
# 使用 seek 回溯到时间 a（2 秒）
gz service -s /world/shapes/playback/control --reqtype gz.msgs.LogPlaybackControl --reptype gz.msgs.Boolean --timeout 3000 --req 'seek: {sec: 2, nsec: 0}'

# 等待 seek 完成
sleep 2
```

**5. 验证回溯后的状态：**
```bash
# 记录回溯后的状态
gz topic -e -t /world/shapes/pose/info -m gz.msgs.Pose_V -n 1 > state_after_rewind.txt

# 对比回溯后的状态和时间 a 的状态
diff state_at_time_a.txt state_after_rewind.txt
```

**如果 `seek` 功能正常工作：**
- `state_after_rewind.txt` 应该与 `state_at_time_a.txt` 相同（或非常接近）
- 模拟时间应该回到 2 秒左右

**如果 `seek` 功能有问题：**
- `state_after_rewind.txt` 与 `state_at_time_a.txt` 不同
- 或者模拟时间没有正确回溯

### 注意事项

1. **Playback 模式 vs 实时模拟**：
   - Playback 模式是从 log 文件回放，不是实时模拟
   - 在 playback 模式下，`seek` 可以从 log 文件恢复状态
   - 在实时模拟模式下，`seek` 只改变时间，不恢复状态

2. **Seek 的性能**：
   - 向后 seek（回溯）可能很慢，因为需要从开始重新播放所有状态
   - 向前 seek 通常更快

3. **Log 文件位置**：
   - 默认记录路径：`~/.gz/sim/log/<timestamp>`
   - 可以使用 `--record-path` 指定自定义路径

4. **验证方法**：
   - 使用 `/stats` topic 验证模拟时间
   - 使用 `/world/<world_name>/pose/info` topic 验证模型位置
   - 对比 seek 前后的状态文件

