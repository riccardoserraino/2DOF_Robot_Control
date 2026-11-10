# pp_design — pole placement for the 2DOF Robot

Pole-placement controller for the two joint motors (3165 and 3166):
state feedback + integral action + Luenberger observer, with feedforward on trajectories.

Theory: [`docs/pp_design_theory.pdf`](docs/pp_design_theory.pdf).
Which script calls what: [`docs/pp_design_workflow.pdf`](docs/pp_design_workflow.pdf).

---

## Folder

```
pp_design/
├── design_A.m        poles: complex pair + one real pole   (search)
├── design_B.m        poles: three real poles               (search)
├── check_design.m    poles chosen by hand                  (no search)
├── explore_poles.m   interactive: drag the poles and watch step, voltage, Bode (one motor)
├── tune_poles.m      interactive: sliders for wn, zeta, alpha, both motors in one table
├── docs/             theory and workflow pdf
└── utils/
    ├── pp_setup.m    motor models + requirements     <- change the numbers here
    ├── pp_gains.m    poles -> gains and observer
    ├── closed_loop.m simulation of the loop (with the 3 ms delay)
    ├── pp_check.m    requirements check
    ├── pp_search.m   search of the poles
    ├── pp_report.m   table and plots
    └── pp_simulink.m variables for Simulink
```

`../pid_tune.m` is required: it provides the motor model (k, τ) and the starting poles.

---

## How to use it

1. **Requirements** — edit section 1 of `utils/pp_setup.m`
   (overshoot, settling time, peak voltage, phase margin, chatter, slowest pole `lam_min`).
   To switch one off, give it a limit that is always met (`Inf`, or `0` for phase margin and `lam_min`).
2. **Explore** (optional)
   - `explore_poles.m`: drag the three poles by hand, one motor at a time.
   - `tune_poles.m`: sliders for `wn`, `zeta`, `alpha` (design A pattern). With *lock alpha*
     the real (integral) pole stays at `lam_fix` (3 rad/s): `alpha = lam_fix/(zeta*wn)`.
     The table shows both motors (PID, β\*, β = 1) and β\* of each motor.
     *Write to Simulink* writes the same variables as `design_A.m`.
3. **Design** — set section 1 of one script and run it:

   | script | settings |
   |---|---|
   | `design_A.m` | `wn`, `zeta`, `alpha` |
   | `design_B.m` | `lambda` (three speeds) |
   | `check_design.m` | the poles by hand, motor(s) to show |

   Common settings:
   - `start_from_pid`: start from the PID poles (`true`) or from your values (`false`);
   - `do_search`: move the poles until the requirements are met (`true`) or keep them as given (`false`);
   - `beta_mode`: `'one'` for trajectories, `'star'` for point-to-point steps.
4. **Read the results** — table in the terminal (`*` = requirement not met) and step plots.
5. **Simulink** — the script (`pp_simulink.m`) first runs `scripts/parameters.m` in the
   workspace (section 5: `use_real_plant`, `w_eps`, `k_/tau_xxxx_plant`, `Td_xxxx`, `Vc_xxxx`),
   then writes the gains of both motors (`Kx_`, `Ki_`, `Kaw_`, `A_obs_`, `B_obs_`, `beta_`). Then run
   `slx/Model_pp_obs.slx` (simulation) or `slx/CNC_pp_obs.slx` (robot).
   To save and compare runs, see `scripts/README.md`.

---

## Good to know

- The search is soft: if two requirements conflict it returns the best compromise and prints
  that some requirements are not met. Check the `*` in the table.
- `lam_min` sets how fast the integral action removes friction and model errors on the robot.
- The model has no friction and no saturation: always validate on the robot.
