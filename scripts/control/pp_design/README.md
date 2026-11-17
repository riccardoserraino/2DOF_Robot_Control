# pp_design — pole placement for the 2DOF robot

Pole placement controller for the two joint motors (3165 and 3166):
Luenberger observer + state feedback + integral action + feedforward from the motor model.
Gains, observer and feedforward all use the same model: `k_xxxx_plant`, `tau_xxxx_plant`
of `scripts/parameters.m`.

Docs (in `docs/`):
- [`pp_design_guide.pdf`](docs/pp_design_guide.pdf) — the method: plant, control law, design A,
  every parameter and requirement with its references, the chatter limit, the results of the
  comparison and the chosen design
- [`pp_design_workflow.pdf`](docs/pp_design_workflow.pdf) — what runs, step by step: the call
  trees of `design_pp.m` and `observer_study.m`, the tools, who calls whom

---

## Workflow

```
1. tools/observer_study.m    list candidate designs -> requirements + one simulation each
        |                    report: data/sim/observer_study_<date>_<time>.txt
2. tools/analyze_study.m     accept within tolerances (Ms +3 %, chatter +6 %), rank by error
   (validate_controllers.m   plots and comparison with other controllers)
        |
3. design_pp.m               copy the chosen design (poles AND observer row), do_search = false
        |                    -> requirement table + Simulink variables
4. CNC_pp_obs.slx (robot)  or  Model_pp_obs.slx (re-check), saved with run_and_save
```

- **`observer_study.m`**: lists `wn_list`, `zeta_list`, `lambda_list`, `obs_list` (one row
  `[f1 f2]` per observer option), `plant_err` for robustness. Choose the trajectory in
  `Model_pp_obs.slx` before running it.
- **`analyze_study.m`**: set `study_file` and the tolerances; prints accepted designs best first.
- **`design_pp.m`**: copy `wn`, `zeta`, `lambda` and `my_req.obs_factor`; keep the same
  requirement settings as the study.

### Study or search?
- The **study** compares the designs you list, with simulation: it sees the real errors.
- The **search** (`do_search = true`) moves your poles to the nearest design that meets the
  strict limits of `design_pp.m`: linear model only, no errors, no tolerances. It can replace a
  design validated in simulation with one never simulated.
- Final design: **`do_search = false`**. Use the search only to try a value quickly or after a
  new identification; then set its limits to the accepted tolerances (Ms 1.648, chatter 0.29).
  Details: guide, section "Choosing the design".

---

## Current design (section 1 of `design_pp.m`)

| setting | value | meaning |
|---|---|---|
| `wn` | 39 rad/s | speed of the joint (complex pair) |
| `zeta` | 0.9 | damping of the pair |
| `lambda` | 11 rad/s | real pole = speed of the integral action |
| `obs_factor` | [6; 6] | observer poles = 6 x 39 = 234 rad/s (the study used [5.5; 6.5]: copy it to match) |
| `obs_max` | [0.5; 0.6]/Ts = 250, 300 rad/s | cap of the observer poles (Euler at 2 ms) |
| `Td` | Ts/2 + 3.5 ms = 4.5 ms | delay in the loop, used in every margin |
| `beta_mode` | `'one'` | trajectories (`'star'` for steps) |

`wn 39, zeta 0.9, lambda 11` is the best design of the second study within the tolerances
(Ms 1.647 = +3 %); in that study the observer was `[5.5; 6.5]`. Strictly feasible alternatives:
`wn 35, zeta 0.9, lambda 11` (guide, results section) and `wn 30, zeta 0.9, lambda 10` (smoothest).

## Requirements we want to meet

Checked on both motors, on a test step of `R` degrees, with the delay `Td`
(`utils/closed_loop.m`, `utils/pp_check.m`):

| requirement | limit | why |
|---|---|---|
| peak voltage | <= 6 V | amplifier saturation: the linear design stays valid |
| phase margin | >= 45 deg | robustness to delay and to errors in k, tau |
| Ms (maximum sensitivity, peak of \|1/(1+L)\|) | <= 1.6 | robustness to every model error: GM >= 8.5 dB and PM >= 36 deg together |
| chatter | <= 0.275 V | voltage jump caused by one encoder count at high frequency: limits motor vibrations (value calibrated on this robot, see the guide) |
| slowest pole | >= 10 rad/s | integral fast enough to remove friction errors within one side of the path |
| overshoot, settling | off with `'one'` (2 %, 0.2 s with `'star'`, test step 10 deg) | only meaningful for point-to-point steps |

On the wrong plants of `plant_err` (robustness test) the limits are Ms <= 2 and PM >= 30 deg.

---

## Folder

```
control/
├── pid_tune.m              PID design (only its gains are read, for the comparison)
└── pp_design/
    ├── design_pp.m         MAIN: the chosen design -> table + Simulink variables
    ├── utils/              functions only: defaults, never the values of the design
    │   ├── pp_setup.m      motor models (plant of parameters.m), PID, DEFAULT requirements
    │   ├── pp_gains.m      poles -> gains (Ackermann), beta*, observer gain
    │   ├── closed_loop.m   loop with observer and delay: step, margins, Ms, chatter
    │   ├── pp_check.m      the 7 requirements
    │   ├── pp_search.m     moves the poles as little as possible until every requirement is met
    │   ├── pp_report.m     requirement table and optional step plots
    │   └── pp_simulink.m   variables for Simulink
    ├── tools/
    │   ├── analyze_study.m  check designsmade by observer_study.m (step 2a)
    │   ├── observer_study.m  compare designs: requirements + simulations + report (step 1)
    │   ├── analyze_study.m   read a report, apply tolerances, rank by error (step 2)
    │   ├── design_B.m        three real poles instead of pair + real pole (search)
    │   ├── check_design.m    poles chosen by hand, no search
    │   ├── tune_poles.m      sliders for wn, zeta, alpha on both motors
    │   └── explore_poles.m   drag the poles of one motor, live plots
    └── docs/
        ├── pp_design_guide.tex / .pdf     method, requirements, references, results
        └── pp_design_workflow.tex / .pdf  call trees of the scripts
```

---

## Good to know

- The values in section 1 are the design; the requirements are guard rails.
- Higher `wn` lowers the error but raises Ms and the chatter: with the 4.5 ms delay and
  observer 6x, about 35–37 rad/s is the limit (table in the guide).
- A slower observer lowers the chatter but also the phase margin (it is inside the loop):
  below 6x the Ms and PM limits are not met.
- The requirement table uses the nominal model; the effect of friction, quantization and
  model errors shows only in simulation (`observer_study`, `use_real_plant = true`) or on the
  robot.
- `pp_setup.m` defaults are also the starting point of the scripts in `tools/`.
