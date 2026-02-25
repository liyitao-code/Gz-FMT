## Environment
* OS Version: Ubuntu 24.04.2 LTS
* Source or binary build?
  source build 
  gz-sim9 version: [d83d13526](https://github.com/gazebosim/gz-sim/commit/d83d1352600022ea9da20906000a963580c7f258)
  built with
  gcc (Ubuntu 13.3.0-6ubuntu2~24.04) 13.3.0
  build options: -DCMAKE_BUILD_TYPE=Coverage

## Description
* Expected behavior: After `reset: {all: true}`, the simulation should return to its initial state and behave identically to a fresh start. All system plugins, including `gz-sim-buoyancy-system`, should function correctly.
* Actual behavior: After `reset: {all: true}`, the buoyancy system stops working entirely. A model that floats on the water surface before reset immediately sinks after reset, even without any external forces applied. The model behaves as if submerged in a vacuum — only gravity acts on it.

## Steps to reproduce

Use the attached SDF file (`a.sdf`), which is a `buoyant_cylinder` world containing:
- `gz-sim-buoyancy-system` plugin with graded buoyancy (water density 1025 kg/m³ below z=0)
- A model `my_turtle` (mass=10 kg, box collision 1m×1m×0.01m) at position (0, 0, 0) — the water surface

1. Launch Gazebo and start the simulation:
   ```bash
   gz sim a.sdf
   ```
   ```bash
   gz service -s /world/buoyant_cylinder/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req 'pause: false'
   ```
   **Observe**: The model `my_turtle` floats on the water surface (z ≈ 0). Buoyancy works correctly.

2. Pause the simulation:
   ```bash
   gz service -s /world/buoyant_cylinder/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req 'pause: true'
   ```

3. Reset the simulation:
   ```bash
   gz service -s /world/buoyant_cylinder/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req 'reset: {all: true}, pause: true'
   ```

4. Resume the simulation:
   ```bash
   gz service -s /world/buoyant_cylinder/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 --req 'pause: false'
   ```
   **Observe**: The model immediately begins sinking below the water surface, as if buoyancy no longer exists.

## Root Cause Analysis

I traced through the source code and believe the issue is in how `SystemManager::Reset()` interacts with plugins that do not implement `ISystemReset`.

### Reset flow

When `reset: {all: true}` is processed:

1. **`EntityComponentManager::ResetTo(initialEntityCompMgr)`** restores all entity components to the snapshot saved at world creation time — **before** any system plugin had its first `PreUpdate` call.

2. **`SystemManager::Reset()`** checks each system plugin. Since `Buoyancy` does not implement `ISystemReset`, it is **destroyed and reloaded** (`SystemManager.cc:178-241`).

### Why buoyancy breaks

The `Buoyancy` plugin works by dynamically creating `Volume` and `CenterOfVolume` components on link entities during its first `PreUpdate` call:

```cpp
// Buoyancy.cc - CheckForNewEntities()
_ecm.EachNew<components::Link, components::Inertial>(
    [&](const Entity &_entity, ...) -> bool {
      // ... compute volume from collision geometry ...
      this->centerOfVolumes[_entity] = weightedPosInLinkSum / volumeSum;
      this->volumes[_entity] = volumeSum;
      return true;
    });
```

These `Volume` and `CenterOfVolume` components are then used in the buoyancy force calculation:

```cpp
// Buoyancy.cc - PreUpdate()
_ecm.Each<components::Link,
          components::Volume,
          components::CenterOfVolume>(
    [&](const Entity &_entity, ...) -> bool {
      // ... calculate and apply buoyancy force ...
      link.AddWorldWrench(_ecm, buoyancy, torque);
      return true;
    });
```

After reset:

1. `ResetTo(initialEntityCompMgr)` restores the ECM to the snapshot taken **before** the first `PreUpdate`, so the `Volume` and `CenterOfVolume` components **do not exist** in the restored ECM.

2. The reloaded `Buoyancy` plugin calls `CheckForNewEntities()` using `_ecm.EachNew<Link, Inertial>()`. However, since `ResetTo()` restored existing entities rather than creating new ones, `EachNew` does not return them.

3. As a result, `Volume` and `CenterOfVolume` components are never recreated. The buoyancy force calculation loop (`Each<Link, Volume, CenterOfVolume>`) finds no matching entities, and **no buoyancy forces are ever applied**.

### Scope of impact

This issue likely affects **any system plugin** that:
- Does not implement `ISystemReset`
- Dynamically creates components via `EachNew` during runtime (not at `Configure` time)

## Additional Information
- SDF file used for testing: [a.sdf] (attached)
- Test model: `my_turtle` (mass=10 kg, initial position (0, 0, 0) at water surface)
- Buoyancy config: graded buoyancy, water density=1025 kg/m³ below z=0, air density=1.125 kg/m³ above z=0
- The model's collision volume (1×1×0.01 m³) provides buoyancy ≈ 100.6 N when fully submerged, which exceeds gravity (98.1 N), so the model should float.
