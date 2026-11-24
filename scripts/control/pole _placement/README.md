# Pole placement + observer

State feedback with integral action, a Luenberger observer for the speed, and feedforward
from the motor model, for the two joint motors (3165, 3166).

| file | role |
|---|---|
| `pp_core.m` | **main logic**: gains with Ackermann, requirement values, Simulink variables |
| `pp_tune.m` | simulates every combination of the design parameters and writes a report |
| `pp_tune_analyze.m` | reads the report, checks the requirements, ranks the designs |
| `pole_place.m` | final design: chosen parameters → gains, requirement table, Simulink variables |

## The poles

Each joint has 5 closed-loop poles (separation principle: chosen independently):

- **the pair**, roots of `s^2 + 2*zeta*wn*s + wn^2`: `wn` = speed (bandwidth), `zeta` = damping.
  With `zeta < 1` they are complex conjugate, with `zeta > 1` two real poles (no ringing);
- **the integral pole** `-lambda`, with `lambda = ifac*zeta*wn` (a multiple of the real part of
  the pair): it removes friction and model errors. Lower `ifac` = more phase margin, slower
  correction;
- **two observer poles** `obs_factor x` the fastest controller pole: how fast the speed
  estimate converges.

`pp_core.m` computes the gains with **Ackermann's formula**: controller gains `Kx`, `Ki` on the
motor model with the integral state added, observer gain `L` on the dual system.

## Requirements

Checked on the loop opened at the motor input, with the 4.5 ms loop delay, worst motor:

| requirement | limit | why |
|---|---|---|
| peak voltage (3 deg step) | ≤ 4.8 V | 80 % of the 6 V saturation |
| phase margin | ≥ 45 deg | little ringing at the corners |
| Ms = max \|1/(1+L)\| | ≤ 1.6 | robustness: GM ≥ 2.7, PM ≥ 36 deg |
| chatter | ≤ 0.1 V | voltage jump from one encoder count |
| slowest pole | ≥ 10 rad/s | integral errors gone in ~0.4 s |
| crossover wc | ≤ 40 rad/s | model reliable, low noise amplification |

## Workflow

1. **Check the trajectory.** Run `scripts/trajectory/check_trajectory.m` with this controller
   (see the [trajectory README](../../trajectory/README.md)).
2. **Try designs: `pp_tune.m`.** In its Settings choose `traj_id` and the values to try:
   `wn_list`, `zeta_list`, `ifac_list` and the observer options `obs_list` (one row each).
   It simulates every combination, saves each run in `data/sim/` and writes the report
   `pp_tune_<date>_<time>.txt`.
3. **Rank them: `pp_tune_analyze.m`, right after the sweep** (it needs the `.mat` run files).
   Set in its Settings the tolerance of each requirement (`tol`, fraction of the limit:
   0 = met exactly, negative = stricter), the margin targets `tgt` and `t_skip`. It ranks the
   accepted designs by joint, tip and contour error and prints a final report: best design
   per observer option, best per requirement and metric, overall best with coherent margins.
   Everything is saved in `data/sim/analysis_pp_tune_<date>_<time>.txt`.
4. **Clean up.** After the analysis the `.mat` run files can be deleted: keep the two `.txt`
   files of each sweep (report and analysis).
5. **Final design: `pole_place.m`.** Copy the chosen design label (`wn<wn> z<zeta> i<ifac>
   obs<obs_factor>`) into its Settings and run it. It prints the requirement table and all
   the poles, and writes the gains for `Model_pp_obs.slx` (simulation) and `CNC_pp_obs.slx`
   (robot).
