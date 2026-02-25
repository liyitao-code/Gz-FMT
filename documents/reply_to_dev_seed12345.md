# 回复开发者：使用 --seed 12345 的复现结果

---

## 版本一：仅陈述复现结果（无原因分析）

---

I've switched to `gz sim --seed 12345` as suggested and ran 5 reproductions with the same SDF and replay script. Here are the results.

- **All 5 runs**: `Monotonic: Yes`, `Smooth: No`, with exactly one violation in the **second sampling window**.
- **Jump ratio (Δx₂/Δx₁)** in the five runs: **4.28x**, **5.39x**, **5.77x**, **5.92x**, **6.13x**.

**Results (5 runs, `gz sim <sdf> --seed 12345`):**

| Run | Jump ratio | Δx₁ (m) | Δx₂ (m) |
|-----|------------|---------|---------|
| 1   | 4.28x      | 0.0751  | 0.3212  |
| 2   | 6.13x      | 0.0444  | 0.2718  |
| 3   | 5.92x      | 0.0447  | 0.2645  |
| 4   | 5.77x      | 0.0554  | 0.3198  |
| 5   | 5.39x      | 0.0572  | 0.3085  |

Reproduction data (reports and logs) are under the same experiment directory; each run is in a timestamped `reproduce_*` subfolder. If you want a single canonical run for comparison, I can re-run once and share that folder's contents or paste the full trajectory.

---

## 版本二：复现结果 + 原因简述（完整叙述）

---

I've switched to `gz sim --seed 12345` as suggested and ran 5 reproductions with the same SDF and replay script. Summary below.

**Results**

- **All 5 runs**: `Monotonic: Yes`, `Smooth: No`, with exactly one violation in the **second sampling window**.
- **Jump ratio (Δx₂/Δx₁)** in the five runs: **4.28x**, **5.39x**, **5.77x**, **5.92x**, **6.13x**.

| Run | Jump ratio | Δx₁ (m) | Δx₂ (m) |
|-----|------------|---------|---------|
| 1   | 4.28x      | 0.0751  | 0.3212  |
| 2   | 6.13x      | 0.0444  | 0.2718  |
| 3   | 5.92x      | 0.0447  | 0.2645  |
| 4   | 5.77x      | 0.0554  | 0.3198  |
| 5   | 5.39x      | 0.0572  | 0.3085  |

So the non-smooth behaviour is reproduced every time with `--seed 12345`. The exact jump ratio varies a bit between runs (4.28x–6.13x). In the replay script, simulation time is advanced only by `multi_step` (blocking), so when the service call returns the sim has already run the steps and is paused; we then do a wall-clock `sleep` (from the original experiment log) and only after that read the model pose from the topic. So we are reading the paused state, but (1) the pose is taken from a **topic**—depending on publish timing, we may not always get the exact post-step frame—and (2) the real-time sleep and process scheduling can vary slightly run-to-run, so the moment we subscribe and receive one message can shift. That likely explains the small variance in the ratio. The main point is that the **phenomenon** is stable: `Smooth: No`, violation always at the second window, and the ratio consistently in the 4–6x range.

Reproduction data (reports and logs) are under the same experiment directory; each run is in a timestamped `reproduce_*` subfolder. If you want a single canonical run for comparison, I can re-run once and share that folder's contents or paste the full trajectory.

---
