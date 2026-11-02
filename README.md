# KinoCheck-Eval: A Deterministic Kinodynamic Benchmark for Autonomous Mobile Robot Planning

**Event:** micro1 Eval Hackathon 2026 (Hosted by micro1 on HackerRank)  
**Domain:** Robotics · Wheeled Mobile Navigation · Motion Planning & Kinematics  
**Author:** Individual Submission  
**Evaluated Artifacts:** Tasks KC-02, KC-03, KC-04 · 221 Passing Tests · Deterministic Code Graders (Zero LLM Judges)

---

## 1. Short Description

KinoCheck-Eval is a deterministic, execution-based benchmark evaluating whether code-generating AI models produce physically safe and kinematically valid wheeled-robot motion planning and speed-profiling code. While existing code benchmarks evaluate static functional correctness or repository-level issue resolution, many general-purpose code benchmarks do not directly measure continuous physical constraints such as swept-volume obstacle clearance, continuous perimeter margins, Pareto-optimal hazard budgets, and coupled forward-backward braking profiles. Evaluated across 20 calibrated scenarios spanning 3 core tasks (KC-02, KC-03, KC-04) using GitHub Copilot (Auto / Standard tier), the model achieved an overall scenario-trial pass rate of 70.6% (24 passed out of 34 evaluated scenario trials across 5 total task attempts). The benchmark exposed acute domain-specific failure modes: a catastrophic runtime name-binding scope error during geometric segment-rectangle distance evaluation, and premature false-infeasibility rejections caused by conflating continuous world coordinates with discrete grid cell indices.

| Metric | Result |
|---|---|
| Tasks Evaluated | 3 (KC-02, KC-03, KC-04) |
| Unique Scenarios | 20 |
| Scenario Trials Passed | 24 / 34 |
| Pooled Pass Rate | 70.6% |
| Automated Test Suite | 221 / 221 passing |
| Targeted Mutants Detected | 15 / 15 (100% kill rate) |
| Grader Type | Deterministic code execution (0 LLM judges) |

## 2. Video Walkthrough

[![KinoCheck-Eval Video Walkthrough](https://img.youtube.com/vi/kX4rVOpBWcE/maxresdefault.jpg)](https://www.youtube.com/watch?v=kX4rVOpBWcE)

**▶ [Watch the KinoCheck-Eval walkthrough on YouTube](https://www.youtube.com/watch?v=kX4rVOpBWcE)**

This walkthrough demonstrates the benchmark tasks, deterministic grading, KC-02 and KC-03 failure analysis, and reproduction of the evaluated candidate results.

## 3. Domain and Expertise

In autonomous robotics and automated guided vehicles (AGVs), path planning cannot be reduced to simple 2D discrete grid search. An algorithm that successfully connects collision-free cell centers can easily cut diagonally across obstacle vertices, causing physical collisions when the robot's finite circular footprint sweeps through space. Similarly, multi-objective hazard avoidance under a hard cumulative risk budget cannot be solved via linear scalarization without risking missing non-convex regions of the Pareto frontier, and longitudinal velocity profiling along curved paths requires coupled dynamic programming (both forward acceleration and backward braking passes) to prevent excessive lateral acceleration and loss of traction.

KinoCheck-Eval draws directly on practical domain experience in mobile robotics engineering, receding-horizon Dijkstra planning, Gaussian hazard field modeling, and differential-drive kinematics in Webots simulation environments. It formalizes these physical requirements into closed-form, purely mathematical verifiers (using NumPy and Shapely), eliminating subjective LLM-as-a-judge scoring entirely.

## 4. What It Tests and Why It Matters

### KC-02: Kinodynamic 2D Path Planning with Continuous Clearance Checks

- **Problem Domain:** Disc-robot navigation on a column-major occupancy grid (`grid[x][y]`).
- **Signature:** `plan(problem: dict) -> list[tuple[float, float]] | None`
- **Input Fields:** `grid`, `cell_size`, `radius`, `start`, `goal`.
- **Mathematical Acceptance Criteria:**
  - Continuous swept-segment clearance: Every point along every straight-line segment must maintain a distance of at least $r_{\text{robot}} + 0.01\,\text{m}$ from all occupied obstacle cells (`grid[x][y] == 1`), explicitly including obstacle corners.
  - Arena perimeter clearance: Every segment must remain at least $r_{\text{robot}} + 0.01\,\text{m}$ inside the arena boundaries `[0, width] x [0, height]`.
  - Endpoint matching tolerance: `1e-9`.
  - Infeasibility Contract: Returns `None` for scenarios constructed to be provably infeasible.
- **Scenarios:** 7 total (4 feasible, 3 provably infeasible chokepoints/pinch points).
- **Why It Matters:** Disregarding continuous swept volume causes corner-clipping collisions on physical robots.

### KC-03: Risk-Budget Pareto Shortest Path Planning

- **Problem Domain:** Shortest path planning under a continuous, piecewise-constant hazard field.
- **Signature:** `plan(problem: dict) -> list[list[float]] | list[tuple[float, float]] | None`
- **Input Fields:** `grid`, `risk_grid` (column-major), `cell_size`, `radius`, `start`, `goal`, `feasible`, `risk_budget`, `optimal_length`.
- **Mathematical Acceptance Criteria:**
  - Clearance constraints: $r_{\text{robot}} + 0.01\,\text{m}$ from obstacles and arena perimeters.
  - Cumulative line-integral risk: Segment risk is computed by splitting each line segment at all crossed grid-cell boundaries. Total risk must satisfy: $R(P) \leq R_{\text{budget}} + 10^{-9}$.
  - Objective: Return the path solving:

    $$\min_{P} L(P) \quad \text{subject to} \quad R(P) \leq R_{\text{budget}}$$
  - Endpoint matching tolerance: `1e-9`.
- **Scenarios:** 7 total (5 feasible detours/corridors, 2 provably infeasible budget/geometry traps).
- **Why It Matters:** Weighted-sum scalarization can miss solutions on non-convex regions of the Pareto frontier, so it may not recover every risk-budget-optimal path.

### KC-04: Curvature-Aware Time-Optimal Speed Profiling

- **Problem Domain:** Longitudinal velocity optimization subject to acceleration, braking, and lateral turning limits along a 2D polyline path.
- **Signature:** `plan(problem: dict) -> list[tuple[float, float, float, float]] | None`
- **Input Fields:** `path`, `v_max`, `a_accel_max`, `a_brake_max`, `a_lat_max`, `v_start`, `v_goal`, `feasible`, `optimal_time`.
- **Mathematical Acceptance Criteria:**
  - Discrete 3-point Menger curvature:

    $$\kappa_i = \frac{4 \cdot \text{Area}(A, B, C)}{d(A, B) \cdot d(B, C) \cdot d(A, C)}$$

    for interior non-collinear points (0.0 at endpoints and collinear segments).
  - Lateral acceleration limit:

    $$v_i \leq \min\left(v_{\max}, \sqrt{\frac{a_{\text{lat,max}}}{\kappa_i}}\right) \quad (\text{for } \kappa_i > 10^{-6})$$
  - Longitudinal acceleration bounds:

    $$-a_{\text{brake,max}} - 10^{-3} \leq \frac{v_{i+1}^2 - v_i^2}{2\,\Delta s_i} \leq a_{\text{accel,max}} + 10^{-3}$$
  - Timestamp integration:

    $$t_0 = 0,\qquad \Delta t_i = \frac{2\,\Delta s_i}{v_i + v_{i+1}}$$

    (strictly increasing by $\geq 10^{-4}$).
  - Endpoint velocity tolerances: 1e-4.
- **Scenarios:** 6 total (4 feasible profiles, 2 kinematically impossible braking/curvature traps).
- **Why It Matters:** Failing to execute a backward braking pass results in vehicles entering sharp curves at unsafe speeds, causing loss of traction or lateral slip.

## 5. Setup and Run Instructions

### Prerequisites & Installation

Runs in standard Python without requiring ROS, Gazebo, or external simulators.

```bash
git clone https://github.com/Rerishabh/KinoCheck-Eval.git kinocheck-eval
cd kinocheck-eval
python -m venv .venv

# Activate virtual environment
# Windows PowerShell:
.venv\Scripts\Activate.ps1
# Linux / macOS:
source .venv/bin/activate

# Install exact pinned dependencies
pip install -r requirements.txt
```

Pinned dependencies include `pytest==9.2.0`, `numpy==2.5.4`, `shapely==2.2.0`, `colorama==0.4.6`, `iconfig==2.3.1`, `packaging==26.3`, `pluggy==1.7.0`, `Pygments==2.21.0`, and `python-dotenv==1.2.4`.

### Benchmark CLI

The CLI module is `kinocheck.cli`. It supports the `benchmark` subcommand:

```bash
python -m kinocheck.cli benchmark \
  --task KC-02 \
  --submission results.copilot_auto.kc02_attempt2 \
  --timeout 10
python -m kinocheck.cli benchmark \
  --task KC-03 \
  --submission results.copilot_auto.kc03_attempt2 \
  --timeout 10
python -m kinocheck.cli benchmark \
  --task KC-04 \
  --submission results.copilot_auto.kc04_attempt1 \
  --timeout 10
```

Supported arguments are:

- `--task TASK`: `KC-02`, `KC-03`, `KC-04`, comma-separated task IDs, or `all`.
- `--submission SUBMISSION`: a Python module path or file path.
- `--entrypoint ENTRYPOINT`: entrypoint name; defaults to `plan`.
- `--timeout TIMEOUT`: positive timeout in seconds; defaults to `5.0`.
- `--json-out JSON_OUT`: optional structured report path.

Run all tasks in one invocation:

```bash
python -m kinocheck.cli benchmark \
  --task all \
  --submission results.copilot_auto.kc02_attempt2 \
  --timeout 10
```

### Mutant Calibration

The deterministic calibration scripts are:

- `scripts/evaluate_kc02_mutants.py`
- `scripts/evaluate_kc03_mutants.py`
- `scripts/evaluate_kc04_mutants.py`

Run them with:

```bash
python scripts/evaluate_kc02_mutants.py
python scripts/evaluate_kc03_mutants.py
python scripts/evaluate_kc04_mutants.py
```

### Mutant Calibration Summary

| Task | Targeted Defect Models | Mutants Detected | Mutant-Level Kill Rate |
|---|---:|---:|---:|
| KC-02 | 5 | 5 | 100% |
| KC-03 | 5 | 5 | 100% |
| KC-04 | 5 | 5 | 100% |
| **Total** | **15** | **15** | **100%** |

## 6. Empirical Evaluation Results

The following results were obtained through the verified `kinocheck.cli benchmark` entrypoint. “Scenario Trials Passed” counts passed executions, not unique scenarios.

| Task | Attempt | Module | Scenario Trials Passed |
|---|---|---|---:|
| KC-02 | Attempt 1 | `results.copilot_auto.kc02_attempt1` | 2 / 7 (28.6%) |
| KC-02 | Attempt 2 | `results.copilot_auto.kc02_attempt2` | 7 / 7 (100.0%) |
| KC-03 | Attempt 1 | `results.copilot_auto.kc03_attempt1` | 2 / 7 (28.6%) |
| KC-03 | Attempt 2 | `results.copilot_auto.kc03_attempt2` | 7 / 7 (100.0%) |
| KC-04 | Attempt 1 | `results.copilot_auto.kc04_attempt1` | 6 / 6 (100.0%) |
| **Total** | **20 unique scenarios** | **5 total task attempts** | **24 / 34 (70.6%)** |

Attempt 1 KC-02 failed with a runtime `NameError` in the segment-rectangle distance implementation, while Attempt 2 passed all 7 KC-02 scenarios. Attempt 1 KC-03 rejected continuous world-coordinate inputs as if they were discrete grid indices and passed only the two infeasible scenarios. Attempt 2 passed all 7 KC-03 scenarios.

For KC-03, **Attempt 2 was an exploratory prompt-clarification trial regarding continuous coordinates, not an identical-prompt repeated sample.**

## 7. Reproducibility and Verification

The repository includes deterministic unit tests, reference planners, graders, scenario data, mutant implementations, and candidate artifacts. The full test suite was verified with:

```bash
python -m pytest -q
```

Verified result: **221 passed in 64.01s**.

The benchmark uses execution-based grading rather than an LLM judge. Graders check output structure, endpoint matching, continuous geometry, risk integration, speed limits, curvature-based lateral acceleration, longitudinal acceleration, and feasibility contracts.

The five candidate modules expose the expected `plan` entrypoint:

```python
results.copilot_auto.kc02_attempt1
results.copilot_auto.kc02_attempt2
results.copilot_auto.kc03_attempt1
results.copilot_auto.kc03_attempt2
results.copilot_auto.kc04_attempt1
```

## 8. Prior Work, Attribution, and AI Tool Disclosure

- The benchmark contains 20 unique frozen scenarios, so conclusions should not be generalized to all maps, hazard fields, or vehicle platforms.
- KC-02 and KC-03 use grid-derived candidate geometry; a production planner would typically use visibility graphs, configuration-space geometry, or sampling-based planning for larger continuous environments.
- KC-03 evaluates a fixed risk budget and path-length objective. Weighted-sum scalarization can miss solutions on non-convex regions of the Pareto frontier, so it may not recover every risk-budget-optimal path.
- KC-04 profiles a supplied spatial polyline and does not model actuator delay, tire friction variation, vehicle dynamics beyond the specified bounds, or closed-loop tracking error.
- The Copilot results are a small empirical sample across five task attempts and should be treated as benchmark evidence rather than a broad model capability estimate.
- Future releases could add randomized scenario generation, larger maps, differential-drive constraints, uncertainty-aware hazards, and independent candidate model families.

The benchmark design builds on standard concepts from computational geometry, occupancy-grid planning, risk-aware shortest-path search, Menger curvature, and kinodynamic velocity profiling. The repository's implementation, scenarios, graders, and reference planners are included as the submission artifacts; no external benchmark code is required to reproduce the reported results.

GitHub Copilot (Auto / Standard tier) was used as the code-generating AI system evaluated in the empirical trials. Copilot-generated candidate files are retained under `results/copilot_auto/`, while the deterministic harness, graders, reference implementations, scenarios, and mutant calibrations were inspected and verified for this submission. AI assistance was used for implementation and debugging; the final README and reported measurements were checked against repository files and command output.

### Pre-Submission Deliverables Checklist

- [x] Root README with benchmark purpose, domain, methodology, results, and reproducibility instructions.
- [x] Deterministic task specifications for KC-02, KC-03, and KC-04.
- [x] Reference planners and execution-based graders.
- [x] 221 passing automated tests.
- [x] Mutant calibration scripts and documented kill results.
- [x] Candidate artifacts and benchmark reproduction commands.
- [x] Public walkthrough video linked with the clickable YouTube thumbnail.
