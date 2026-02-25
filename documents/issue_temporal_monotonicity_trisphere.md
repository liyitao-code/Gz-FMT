## Environment

* OS Version: Ubuntu 24.04.2 LTS
* Source or binary build?  
  source build  
  gz-sim9 version: [d83d13526](https://github.com/gazebosim/gz-sim/commit/d83d1352600022ea9da20906000a963580c7f258)  
  built with  
  gcc (Ubuntu 13.3.0-6ubuntu2~24.04) 13.3.0  
  build options: -DCMAKE_BUILD_TYPE=Coverage

## Description

* **Expected behavior**: With a constant world force in +x applied to `trisphere_cycle0`, and the simulation stepped in equal time windows, the model's x-displacement should increase monotonically and change smoothly between windows.

* **Actual behavior**:
  - The original automatic **Temporal Monotonicity Test (Paradigm B)** for `test_22/_7` reports:
    - `Monotonic: Yes`
    - `Smooth: No`
    - 1 violation at `t = 0.83 s` with displacement jump ratio ≈ `3.99x` (delta: `5.806006 → 23.191853 m`).
  - I replayed the recorded `experiment_log.json` 5 times in fresh `gz sim` processes using a replay script. In every run:
    - The reproduced trajectory is also `Monotonic: Yes` but `Smooth: No`.
    - There is always exactly 1 violation, and it is always in the **second sampling window**.
    - The ratio between the first two x-displacement increments is consistently around **5–6x** (e.g. `0.0366 → 0.2146 m`, `0.0504 → 0.2614 m`, `0.0331 → 0.1902 m`).

---

## Steps to reproduce

Use the attached SDF file (`a.sdf` from the `test_22/_7` directory), which contains:

- World name: `wheel_slip`
- A dynamic model under test:
  - Model name: `trisphere_cycle0`
  - Subject to slip/friction configuration in the `wheel_slip` world

The Temporal Monotonicity Test applies a **constant world wrench in +x** to `trisphere_cycle0`, then samples the model pose at equal simulation-time intervals. The following manual steps reproduce the core behavior.

### 1. Launch Gazebo

```bash
gz sim /path/to/test_22/_7/a.sdf
```

### 2. Pause and clear existing wrenches

```bash
gz service -s /world/wheel_slip/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'pause: true'
```

```bash
gz topic -t /world/wheel_slip/wrench/clear \
  -m gz.msgs.Entity \
  -p 'name: "trisphere_cycle0", type: MODEL'
```

### 3. Apply a constant world wrench in +x

```bash
gz topic -t /world/wheel_slip/wrench/persistent \
  -m gz.msgs.EntityWrench \
  -p 'entity: {name: "trisphere_cycle0", type: MODEL}, \
      wrench: {force: {x: 47.03863393817074, y: 0.0, z: 0.0}, \
               torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

Wait briefly to let the wrench take effect:

```bash
sleep 0.1
```

### 4. Step the simulation in equal windows and sample pose

Repeat the following block 12 times (corresponding to the 12 samples in the original test, total test duration ≈ 5.0 s, each window ≈ 0.414 s sim time):

```bash
gz service -s /world/wheel_slip/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'multi_step: 414'

sleep 0.414

# Sample the pose of trisphere_cycle0 here (e.g. via /world/wheel_slip/dynamic_pose/info)
```

Record the x-displacement at:

- \( t = 0.00\,\text{s} \): after the initial pause and wrench application (before any stepping).
- \( t = 0.41, 0.83, 1.24, \dots, 4.98\,\text{s} \): after each `multi_step: 414` block.

### 5. Observe the x-displacement trajectory

In the original automatic test (`metamorphic_test_result.txt`), the engine reported:

- **Force Magnitude**: 47.04 N in +x
- **Samples**: 12 over 5.0 s
- **Result**: `Monotonic: Yes`, `Smooth: No`
- **Violation**: at \( t = 0.83\,\text{s} \), displacement jumped by ratio ≈ **3.99x** (delta: 5.806006 → 23.191853 m)

In the replayed experiments (using the same SDF and commands), my measured x-displacement sequence is **much smaller in magnitude** (tens of meters instead of thousands), but the **shape** is always the same:

- The first increment \(\Delta x_1\) is very small (~0.03–0.05 m).
- The second increment \(\Delta x_2\) is suddenly ~5–6x larger (~0.19–0.26 m).
- Later increments grow more smoothly.

---

## Additional Information

- Test directory: `test_22/_7`
  - SDF: `a.sdf` (wheel slip world with `trisphere_cycle0`)
  - Original automatic test result: `metamorphic_test_result.txt` (Temporal Monotonicity Test (Paradigm B))
  - Recorded commands: `experiment_log.json`
- Model under test: `trisphere_cycle0` in world `wheel_slip`.
- External wrench used:
  - `force ≈ (47.04, 0, 0) N`, `torque = (0, 0, 0)`.
  - Duration: 12 windows × 414 steps each (≈ 5.0 s simulation time).
- I also have a small replay script (`reproduce_experiment.py`) that:
  - Replays `experiment_log.json` into a fresh `gz sim` process.
  - Samples the pose after each stepping window.
  - Confirms the same “small first increment, larger second increment” pattern in 5 independent runs.
