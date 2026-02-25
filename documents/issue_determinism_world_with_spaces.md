## Description

* **Expected behavior**: For a given SDF world and model, running the same simulation twice in two independent `gz sim` processes with:
  - the same initial state,
  - the same external force (magnitude and direction),
  - and the same number of simulation steps
  
  should produce the **same final pose** for that model (deterministic simulation).

* **Actual behavior**: In the attached `world_with_spaces` SDF, applying a fixed force to the model `model with spaces` for a fixed number of steps in **two independent Gazebo runs** produces **significantly different final positions**, even though both runs start from the same initial pose. In our tests:
  - One pair of runs produced a final position difference of about **250 m** (original experiment).
  - A fresh pair of runs reproduced the phenomenon with a final position difference of about **70 m**, with both runs starting from exactly the same initial pose `(0, 0, 0.5)`.

This indicates non-deterministic behavior in the simulation for this scenario.

---

## Steps to reproduce

Use the attached SDF file (`a.sdf` from the `world_with_spaces` demo), which contains:

- World name: `world_with_spaces`
- A static ground plane at z=0
- A dynamic box model:
  - Model name: `model with spaces`
  - Size: 1×1×1 m
  - Mass: 1.0 kg
  - Initial pose: `(0, 0, 0.5, 0, 0, 0)` (sitting on the ground plane)

We apply the **same constant world wrench** to this model in two completely separate `gz sim` runs and compare the final poses.

### 1. First run (Process 1)

1. Launch Gazebo with the SDF:

   ```bash
   gz sim /path/to/a.sdf
   ```

2. Pause the simulation to prepare the test:

   ```bash
   gz service -s /world/world_with_spaces/control \
     --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 \
     --req 'pause: true'
   ```

3. Clear any existing wrenches on `model with spaces`:

   ```bash
   gz topic -t /world/world_with_spaces/wrench/clear \
     -m gz.msgs.Entity \
     -p 'name: "model with spaces", type: MODEL'
   ```

4. Apply a **persistent** wrench to `model with spaces`:

   ```bash
   gz topic -t /world/world_with_spaces/wrench/persistent \
     -m gz.msgs.EntityWrench \
     -p 'entity: {name: "model with spaces", type: MODEL}, \
         wrench: {force: {x: 39.8577, y: 14.7341, z: 26.3490}, \
                  torque: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

5. Step the simulation for a fixed number of iterations (here: 4688 steps ≈ 4.688 s sim time):

   ```bash
   gz service -s /world/world_with_spaces/control \
     --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 \
     --req 'multi_step: 4688'
   ```

6. Clear the wrench so that no further force is applied:

   ```bash
   gz topic -t /world/world_with_spaces/wrench/clear \
     -m gz.msgs.Entity \
     -p 'name: "model with spaces", type: MODEL'
   ```

7. Read the final pose of `model with spaces` (for example using a pose topic or your preferred inspection method) and record it as **P1**.

   In one of our runs, we observed a final pose approximately:

   - **P1 ≈ (617.0, 228.1, 255.5)**  (world coordinates)

8. Shut down this `gz sim` process.

### 2. Second run (Process 2)

Now repeat the **exact same sequence** in a **fresh `gz sim` process**:

1. Launch Gazebo again from the same SDF:

   ```bash
   gz sim /path/to/a.sdf
   ```

2. Pause the simulation:

   ```bash
   gz service -s /world/world_with_spaces/control \
     --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 \
     --req 'pause: true'
   ```

3. Clear wrenches on `model with spaces`:

   ```bash
   gz topic -t /world/world_with_spaces/wrench/clear \
     -m gz.msgs.Entity \
     -p 'name: "model with spaces", type: MODEL'
   ```

4. Apply the **same** persistent wrench as in the first run:

   ```bash
   gz topic -t /world/world_with_spaces/wrench/persistent \
     -m gz.msgs.EntityWrench \
     -p 'entity: {name: "model with spaces", type: MODEL}, \
         wrench: {force: {x: 39.8577, y: 14.7341, z: 26.3490}, \
                  torque: {x: 0.0, y: 0.0, z: 0.0}}'
   ```

5. Step the simulation for the **same** number of iterations:

   ```bash
   gz service -s /world/world_with_spaces/control \
     --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 10000 \
     --req 'multi_step: 4688'
   ```

6. Clear the wrench:

   ```bash
   gz topic -t /world/world_with_spaces/wrench/clear \
     -m gz.msgs.Entity \
     -p 'name: "model with spaces", type: MODEL'
   ```

7. Read the final pose of `model with spaces` and record it as **P2**.

   In our tests, one such second run produced:

   - **P2 ≈ (555.5, 205.4, 229.8)**

### 3. Compare results

Both runs start from the same initial pose `(0, 0, 0.5)` and use the same force and the same number of steps, but:

- The original automatic experiment (`test_22/_0`) measured:
  - Run 1: P1 ≈ (3024.1, 1117.9, 1256.1)
  - Run 2: P2 ≈ (3243.0, 1198.8, 1347.0)
  - Difference |P1 − P2| ≈ **250 m**

- A manual reproduction as above measured:
  - Run 1: P1 ≈ (617.0, 228.1, 255.5)
  - Run 2: P2 ≈ (555.5, 205.4, 229.8)
  - Difference |P1 − P2| ≈ **70 m**

While the absolute positions differ between runs (as expected due to environment/load differences), the **key point** is that, for each pair of runs, the two final poses (**P1** and **P2**) are **tens to hundreds of meters apart** despite using identical SDF, identical commands, and identical simulation step counts.

---

## Root Cause Analysis

I don’t yet have a code-level root cause, but from an external user’s perspective this scenario strongly suggests **non-deterministic simulation behavior**:

- Both runs start from the same initial pose (confirmed by reading the model pose before applying any external wrench in our reproduction), and the ground plane is flat (no complex contact/rolling on a slope).
- The external wrench, duration (number of steps), and world configuration are identical between the two runs.
- Nevertheless, the final positions differ by tens to hundreds of meters.

Possible areas to investigate on the engine side include:

- Non-determinism in the contact solver or integration (e.g., order-dependent summations, thread scheduling).
- Uninitialized state or ordering issues in systems that affect applied forces or integration.
- Any plugin or system that may introduce randomness or timing-dependent behavior even when the initial scene and commands are identical.

At this point, the report focuses on providing a **minimal, reproducible non-deterministic example** rather than a precise internal root cause.

---

## Additional Information

- SDF used: `world_with_spaces` demo world (`a.sdf`) with a single dynamic box `model with spaces` resting on a flat ground plane.
- Model under test: `model with spaces` (1×1×1 m cube, mass=1.0 kg, initial pose at rest on the plane).
- External wrench:
  - Approximate values: `force ≈ (39.86, 14.73, 26.35) N`, `torque = (0, 0, 0)`.
  - Duration: 4688 simulation steps at `max_step_size = 0.001` (≈ 4.688 seconds sim time).
- I have an automated script that:
  - Launches two independent `gz sim` processes with the same SDF.
  - Applies the same wrench and steps the same number of iterations in each.
  - Reads and compares the final poses, which consistently differ by **large amounts** in this scenario.

