# pp_design — pole placement for the 2DOF Robot

Pole-placement controller for the two joint motors (3165 and 3166):
state feedback + integral action + Luenberger observer, with a feedforward corrected from data.

Docs:
- [`docs/pp_design_guide.pdf`](docs/pp_design_guide.pdf) — workflow, design A, every parameter, feedforward fit
- [`docs/ff_fit_explained.pdf`](docs/ff_fit_explained.pdf) — the feedforward fit (`../ff_correct.m`) line by line
- [`docs/pp_brief_professor.pdf`](docs/pp_brief_professor.pdf) — short justification with references
- [`docs/pp_design_theory.pdf`](docs/pp_design_theory.pdf) — derivations (some numbers are older than the guide)

---

## Folder

```
control/
├── pid_tune.m            PID design (also the motor models used by pp_setup)
├── ff_correct.m          feedforward fit on a run, for any controller (PP, PID)
└── pp_design/
    ├── design_pp.m       MAIN: design A poles + search + feedforward correction + runs
    ├── utils/
    │   ├── pp_setup.m    motor models + requirements     <- change the numbers here
    │   ├── pp_gains.m    poles -> gains and observer
    │   ├── closed_loop.m simulation of the loop (with the delay Td)
    │   ├── pp_check.m    requirements check
    │   ├── pp_search.m   search of the poles
    │   ├── pp_report.m   table and plots
    │   └── pp_simulink.m variables for Simulink (gains, observer, default feedforward)
    ├── tools/            exploration, not needed for the final workflow
    │   ├── design_B.m      three real poles (search)
    │   ├── check_design.m  poles chosen by hand (no search)
    │   ├── tune_poles.m    sliders for wn, zeta, alpha, both motors
    │   └── explore_poles.m drag the poles of one motor, live step / voltage / Bode
    └── docs/
```

---

## How to use it

1. **Requirements** — section 2 of `utils/pp_setup.m` (peak voltage, phase margin, Ms, chatter,
   slowest pole `lam_min`, delay `Td`). They hold for every design.
2. **Design** — section 1 of `design_pp.m`: `wn`, `zeta`, `alpha` (start values), `beta_mode`,
   `do_search`, and the feedforward source:

   | `ff_source` | what happens |
   |---|---|
   | `'model'` | feedforward = model, no simulation |
   | `'sim'`   | run `Model_pp_obs`, fit the feedforward, run again (two runs in `data/sim`) |
   | `'run'`   | fit on `run_file` (e.g. a robot run in `data/lab`), no simulation |

3. **Read the results** — table in the terminal (`*` = requirement not met) and step plots.
4. **Robot** — `ff_source = 'model'`, run `slx/CNC_pp_obs.slx` and save; then
   `ff_source = 'run'` with that file (or any earlier run with the same path); run the robot again.
5. **Compare** — `scripts/plot/validate_controllers.m`.

The same feedforward fit works for the PID: run `../pid_tune.m`, then
`ff_correct('Model_pid_ff', 'sim')` or `ff_correct([], 'run', 'lab/CNC_pid_ff_<date>.mat')`.

---

## Good to know

- The search is soft: if two requirements conflict it returns the best compromise; check the `*`.
- The real delay (`req.Td`) is what limits the speed of the loop.
- Running `design_pp.m` with `ff_source = 'model'` writes the default feedforward again:
  do not run it between the fit and the robot run that should use the fitted values.
