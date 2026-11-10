# Saving runs and comparing controllers

This guide covers two scripts:

- `run_and_save.m` saves a run (simulation or robot) to a `.mat` file.
- `plot/validate_controllers.m` loads saved runs and compares them.

---

## 1. What a run file contains

`run_and_save` writes a file called `<model>_<yyyymmdd>_<HHMMSS>.mat`. Simulations go to
`data/sim/` and bench runs go to `data/lab/`. Each file holds three things:

| content | names | used for |
|---|---|---|
| the signals of the **To Workspace** blocks | `r_3165`, `y_3165`, `u_3165`, `usat_3165`, `d_3165` (same for `3166`), `xc_des`, `yc_des`, `xc_real`/`xc_sim`, `yc_real`/`yc_sim`, each with the suffix `_<tag>` | errors, voltage, disturbance and step KPIs |
| the **controller gains** found in the workspace | PID: `kp_xxxx`, `ki_xxxx`, `kd_xxxx`. PP: `Kx_xxxx`, `Ki_xxxx`, `Kaw_xxxx`, `A_obs_xxxx`, `B_obs_xxxx`, `Kffv_xxxx`, `Kffa_xxxx` | rebuilding the loop: margins, Ms, poles |
| the **plant model** | `k_xxxx_load`, `tau_xxxx_load`, `Vmax`, `run_tag` | the same |

The gains are read from the workspace **at the moment of saving**. Save right after the run,
before you run another design script.

---

## 2. Saving a run

### Simulation

1. Run a design script so the gains are in the workspace. Use `control/pid_tune.m` for the PID,
   or `control/pp_design/design_A.m`, `design_B.m` or `check_design.m` for pole placement.
2. In the command window:
   ```matlab
   run_and_save('Model_pp_obs')     % runs sim() and saves to data/sim
   ```

### Robot (QUARC, external mode)

1. Run the design script, then build and run `CNC_pp_obs.slx` (or `CNC_pid_ff.slx`) on the robot.
2. When the run has stopped:
   ```matlab
   run_and_save('CNC_pp_obs')       % does NOT run the model: saves the data just logged, to data/lab
   ```
   A model in external mode is detected automatically, so the file always goes to `data/lab`.

### Automatic saving (optional)

If the model's `StartFcn`/`StopFcn` callbacks call `run_and_save_deferred(bdroot, 'start')` and
`run_and_save_deferred(bdroot, 'stop')`, the save happens on its own when the run stops.
`run_and_save_deferred` waits up to about 20 s for the data, then calls `run_and_save`.
Never call `run_and_save_deferred` by hand.

**Tip.** Give the files readable names after saving, e.g. `CNC_pp_obs_20261109_A.mat`, so that
each simulation is clearly paired with its bench run. Use the **same gains** for both.

---

## 3. Comparing runs with `validate_controllers`

Open `plot/validate_controllers.m` and edit only the two blocks at the top.

**Runs to compare.** Each row is one file:

```matlab
runs = {
%  file                              type    label
  'CNC_pp_obs_20261109_A.mat',       'PP',   'real'
  'Model_pp_obs_20261109_A.mat',     'PP',   'model'
};
```

- **file**: the name alone is enough. It is searched in `data/`, `data/sim` and `data/lab`.
  You can also give `'sim/...'`, `'lab/...'` or a full path.
- **type**: `'PID'` or `'PP'`. It tells the script how to rebuild the controller from the saved gains.
- **label**: the column name in the tables and the legend name in the plots.

**Options.**

| option | meaning |
|---|---|
| `opt.N_pid` | derivative filter of the Simulink PID block (PID only) |
| `opt.enc_res` | one encoder count [deg] |
| `opt.k_var`, `opt.tau_var` | ranges of k and τ for the robustness check |
| `opt.Vmax` | saturation, used only if the file has no `Vmax` |
| `opt.t_skip` | initial seconds left out of the error KPIs (homing and start-up). Choose it longer than the slowest transient (e.g. 5 s) |

Then run the script. It prints three kinds of tables and draws the plots.

### What the tables mean

**MODEL** (from the gains and k, τ only; it does not use the measured signals):

- **Ms**: the peak of |S|, equal to 1/(distance of the Nyquist curve from −1). Below 1.4 is good,
  below 2 is acceptable. It guarantees GM ≥ Ms/(Ms−1) and PM ≥ 2·asin(1/(2Ms)).
- **Ms worst**: the same quantity with k and τ varied over the ranges in the options.
- **GM, PM, wc, delay margin**: the classic margins. The delay margin is PM/wc, the extra delay
  the loop can tolerate.
- **Encoder noise on u**: the voltage jump caused by one encoder count, i.e. max|C·S| times one count.
- **Slowest CL pole**: its decay rate and damping. It is the speed at which loads and friction are removed.

Two runs with the same gains give **identical** MODEL tables. This table compares
*controllers*, not simulation against reality.

**DATA** (from the logged signals, for each motor):

- **joint error** `r − y`: maximum and RMS, after `t_skip` and before any disturbance.
- **peak |u|** and **time in saturation**.
- **disturbance** and **step** KPIs. They are only computed if the run contains a disturbance
  (`d ≠ 0`) or a single step in `r`; otherwise they show `NaN`.

**TIP (x, y):**

- **contour error**: the distance of the tip from the desired *path*. This is what the drawing shows.
- **tracking error**: the distance from the desired *point at the same time*. It also includes time lag.

### Typical comparisons

| question | runs to list |
|---|---|
| does the model predict the robot? | same controller: `Model_...` + `CNC_...` |
| which controller is better on the robot? | `CNC_pid_ff_...` (type `'PID'`) + `CNC_pp_obs_...` (type `'PP'`) |
| what do new gains change? | two runs of the same model with different gains |

For a fair PID vs PP comparison, use the same path, the same `t_skip`, the same feedforward,
and a similar crossover wc.
