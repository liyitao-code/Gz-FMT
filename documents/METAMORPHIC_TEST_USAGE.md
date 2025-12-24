# 蜕变测试使用指南

本文档说明如何在 Gazebo 模拟过程中对模型施加力或设置速度，用于蜕变测试。

## 功能概述

在 `randomsmith.py` 中添加了两个新函数：

1. **`func_apply_model_force()`**: 对模型施加力或力矩
2. **`func_set_model_velocity()`**: 设置模型的线性速度（通过施加持续力实现）
3. **`metamorphic_test_example()`**: 蜕变测试示例函数

## 前置条件

确保你的 Gazebo 世界文件中加载了 `gz-sim-apply-link-wrench-system` 插件。可以在 SDF 文件中添加：

```xml
<plugin
  filename="gz-sim-apply-link-wrench-system"
  name="gz::sim::systems::ApplyLinkWrench">
</plugin>
```

## 使用方法

### 方法1：对模型施加力

```python
# 创建 SmithUnit 实例（假设已经启动 Gazebo）
unit = SmithUnit(...)

# 对指定模型施加沿x轴正方向的力（10牛顿）
force_cmd = unit.func_apply_model_force(
    model_name="my_model",      # 模型名称
    force_x=10.0,                # x轴方向的力（牛顿）
    force_y=0.0,                 # y轴方向的力
    force_z=0.0,                 # z轴方向的力
    torque_x=0.0,                # 绕x轴的力矩
    torque_y=0.0,                # 绕y轴的力矩
    torque_z=0.0,                # 绕z轴的力矩
    persistent=True               # True=持续施加，False=只施加一次
)

# 执行命令
if force_cmd:
    force_cmd.execute()
```

### 方法2：设置模型速度（通过施加持续力）

```python
# 让模型以 1 m/s 的速度沿x轴正方向运动
velocity_cmd = unit.func_set_model_velocity(
    model_name="my_model",
    velocity_x=1.0,   # x轴速度（m/s）
    velocity_y=0.0,   # y轴速度
    velocity_z=0.0    # z轴速度
)

if velocity_cmd:
    velocity_cmd.execute()
```

### 方法3：完整的蜕变测试示例

```python
# 执行蜕变测试：模型从(0,0,0)开始，以1 m/s的速度沿x轴运动5秒
# 预期最终位置应该在(5, 0, 0)附近
result = unit.metamorphic_test_example(
    model_name="test_model",
    initial_pose=(0.0, 0.0, 0.0),
    velocity_x=1.0,
    test_duration=5.0
)

if result:
    initial_pos, final_pos, expected_pos, success = result
    print(f"初始位置: {initial_pos}")
    print(f"最终位置: {final_pos}")
    print(f"预期位置: {expected_pos}")
    print(f"测试结果: {'通过' if success else '失败'}")
```

## 蜕变测试场景示例

### 场景1：验证位置变化

```python
# 1. 获取模型的初始位置
scene, reserved = unit.get_scene()
model = None
for m in scene.model:
    if m.name == "target_model":
        model = m
        break

initial_x = model.pose.position.x
print(f"初始x位置: {initial_x}")

# 2. 施加力使模型沿x轴运动
force_cmd = unit.func_apply_model_force(
    model_name="target_model",
    force_x=50.0,  # 50牛顿的力
    persistent=True
)
force_cmd.execute()

# 3. 等待5秒
time.sleep(5.0)

# 4. 获取最终位置
scene, _ = unit.get_scene()
for m in scene.model:
    if m.name == "target_model":
        final_x = m.pose.position.x
        break

# 5. 验证：如果初始位置是0，速度是1 m/s，5秒后应该在5米附近
expected_x = initial_x + 1.0 * 5.0  # 简化计算，实际需要考虑物理引擎
print(f"最终x位置: {final_x}")
print(f"预期x位置: {expected_x}")
print(f"误差: {abs(final_x - expected_x)}")
```

### 场景2：验证速度一致性

```python
# 测试：如果模型以恒定速度运动，位置应该线性变化

# 记录多个时间点的位置
positions = []
times = []

# 施加持续力
force_cmd = unit.func_apply_model_force(
    model_name="test_model",
    force_x=100.0,
    persistent=True
)
force_cmd.execute()

# 每隔1秒记录一次位置
for i in range(6):
    time.sleep(1.0)
    scene, _ = unit.get_scene()
    for m in scene.model:
        if m.name == "test_model":
            positions.append(m.pose.position.x)
            times.append(i + 1)
            break

# 验证位置变化是否线性
# positions 应该大致呈线性增长
```

## 注意事项

1. **力的单位**: 力以牛顿(N)为单位。需要根据模型的质量调整力的大小。
   - 对于质量为1kg的模型，10N的力会产生10 m/s²的加速度
   - 对于质量为10kg的模型，需要100N的力才能产生相同的加速度

2. **持续力 vs 瞬时力**:
   - `persistent=True`: 力会持续施加，直到明确清除
   - `persistent=False`: 力只施加一次，之后模型会因摩擦等自然停止

3. **清除持续力**: 如果需要清除持续施加的力，可以发布一个清除消息：
   ```python
   # 通过topic清除
   clear_cmd = f"gz topic -t /world/{world_name}/wrench/clear -m gz.msgs.Entity -p 'name: \"model_name\", type: MODEL'"
   ```

4. **物理引擎限制**: 
   - 模型需要有适当的物理属性（质量、惯性等）
   - 如果模型是静态的（static=true），无法施加力
   - 如果模型有多个链接，力会施加到模型的canonical link上

5. **误差容忍度**: 由于物理引擎的数值误差、摩擦等因素，实际位置可能与理论计算有偏差，需要在测试中设置合理的误差阈值。

## 故障排除

1. **模型不动**: 
   - 检查模型是否有质量属性
   - 检查模型是否设置为静态（static）
   - 检查力的大小是否足够

2. **Topic 不存在**:
   - 确保世界中加载了 `gz-sim-apply-link-wrench-system` 插件
   - 检查 world_name 是否正确

3. **位置变化不符合预期**:
   - 考虑摩擦、重力等因素
   - 调整力的数值
   - 检查模型是否与其他物体碰撞



gz topic -t /world/shapes/wrench/persistent -m gz.msgs.EntityWrench -p 'entity: {name: "ellipsoid", type: MODEL}, wrench: {force: {x: 100.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'

gz service -s /world/shapes/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 3000 --req ''