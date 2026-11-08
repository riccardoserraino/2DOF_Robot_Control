# pp_design — pole placement for the CNC plotter

Pole-placement design (state feedback + integral action + Luenberger observer) for the two
joint motors of the CNC plotter (3165 and 3166), as an alternative to the PID of
`scripts/control/pid_tune.m`.

The full theory, with short proofs, is in [`docs/pp_design_theory.pdf`](docs/pp_design_theory.pdf).
What every script calls, step by step, is in [`docs/pp_design_workflow.pdf`](docs/pp_design_workflow.pdf).

---

## 1. Summary of what we found

- **In our tests, the tuned PID (`pid_tune.m`) worked better than pole placement.** The PID
  stays the reference controller. Pole placement is kept as a documented alternative and as a
  way to understand where the closed-loop poles are and why.
- With pole placement we followed **two designs**, depending on what the robot must do:

| Goal | Reference weight β | What you get | What you pay |
|---|---|---|---|
| **Trajectory tracking** (square, circle paths) | **β = 1** | no lag on the path: with the feedforward of the model the tracking error is zero for any poles (nominal model) | overshoot on a pure step if the poles include a complex pair, which does not matter on a path |
| **Step following** (point-to-point moves) | **β\*** (both approaches) | no overshoot from the slow real pole and a smaller voltage kick at the start | a lag on paths (see below) |

- **Why β changes the step.** The reference path is θ/r = (β·a₁·s + a₀)/Δ(s), so β sets one
  zero at z = −a₀/(β·a₁).
  - **β = 1.** The zero is slower than every pole. It adds the derivative of the response: a fast rise, but overshoot, even with real poles.
  - **β\*.** Puts the zero exactly on the slowest real pole and cancels it in the step.

  β is outside the loop: it does not change the poles, the stability, the phase margin, the chatter or the load rejection.
  The scripts always compare three cases with the PID: **PID, β\*, β = 1**.

### Why β = 0 is not an option

With β = 0 the reference enters **only through the integrator** (u = −k₁θ̂ − k₂ω̂ − kᵢ·e_I): the angle
moves towards r only as fast as the integral action, so the response is dominated by the **slowest
pole**. With the slow pole at 2 rad/s (`lam_min`) the step settles in about 4/2 = 2 s, ten times slower
than the PID, and on a path the lag (a₁/a₀)·speed is the largest possible (about 10° at 50 °/s in the
model example below). It avoids overshoot only by being slow, so it is **not useful for our control**
and it was removed from all the scripts.

### Using β < 1 also for trajectories (possible, at a price)

A β < 1 scales the voltage the reference asks for. It can be used to meet a voltage or
overshoot requirement on paths too. It costs:

1. **Lag proportional to speed.** On a constant-speed move the joint stays behind by (1 − β)·(a₁/a₀)·speed.
   Model example, motor 3165 at 50 °/s: β = 0.713 lags 2.9°, β = 1 lags 0 (β = 0 would lag 10°).
2. **The feedforward no longer cancels the error.** With β = 1 and exact feedforward the error is zero.
   With β < 1 a part of the reference is "missing" and the integrator has to recover it.
3. **Slower rise and settling** on every reference change (corners of the square path), because the
   zero that speeds up the response is moved away or removed.
4. **More work for the integrator.** The slow mode of the integral action shows up more in the response.

In return you get a **smaller voltage jump at corners** (u jumps by β·k₁·Δr), a **lower peak voltage** and
**less overshoot at corners**.

---

## 2. Main theoretical concepts (see `docs/pp_design_theory.pdf`)

| Concept | Where | In short |
|---|---|---|
| Motor model | `utils/pp_setup.m` | τθ'' + θ' = b·u, b = −180k/π, state x = [θ, ω], only θ measured |
| State feedback + integral | all designs | u = β·k₁·r − k₁·θ̂ − k₂·ω̂ − kᵢ·e_I, with e_I' = r − θ |
| Pole placement (Ackermann) | `utils/pp_gains.m` | `acker(Ai, Bi, p)` gives the gains that put the three closed-loop poles in p. It is the same as matching s³ + a₂s² + a₁s + a₀ with (s − p₁)(s − p₂)(s − p₃) |
| Luenberger observer | `utils/pp_gains.m` | estimates ω from θ; poles 6× faster than the controller (≤ 250 rad/s), gain from `acker(A', C', p_obs)'` |
| Separation principle | theory pdf | the observer error does not depend on r, so the reference sees only the three controller poles |
| Reference weight β and zeros | theory pdf | β only moves the zero of θ/r; left-half-plane zeros are not dangerous, β < 0 would give a dangerous right-half-plane zero |
| PID as a state feedback | `utils/pp_setup.m` | u = −[Kp, Kd, −Ki]·[θ, ω, e_I], so its poles can be the start of the search |
| Delay of 3 ms | `utils/closed_loop.m` | sample-and-hold (Ts/2) + one sample of computation (Ts), Ts = 2 ms; it removes phase ω·Td, so margins are computed with it |
| Saturation ±6 V | Simulink + requirement | not linear, so not in the design: the requirement "peak voltage ≤ 6 V" avoids it; anti-windup `Kaw` handles it in Simulink |
| Chatter | `utils/closed_loop.m` | one encoder count (360/4096 °) times the controller gain at high frequency = voltage ripple |
| Search with penalty | `utils/pp_search.m` | `fminsearch` (Nelder–Mead) minimises J = Σ(log\|p\| − log\|p_start\|)² + 10⁴·Σ max(0, violation)². It moves the poles **as little as possible** from the start until **every requirement is met** on both motors |

### Requirements used by the search (`utils/pp_setup.m`)

| Requirement | Limit | Why |
|---|---|---|
| overshoot on a 10° step | ≤ 2 % | do not overshoot corners |
| settling time (2 %) | ≤ 0.2 s | fast enough for the paths |
| peak voltage | ≤ 6 V | no saturation, the linear design stays valid |
| phase margin (with 3 ms delay) | ≥ 45° | robustness to delay and model errors |
| chatter voltage | off (`Inf`) | fast poles amplify the encoder noise; set e.g. 0.2 V to use it |
| slowest pole | ≥ 2 rad/s (`lam_min`) | speed of the integral action: loads and model errors, also the pole hidden by β\*, must be removed in time |

To ignore a requirement, give it a limit that is always met (`Inf`, or `0` for phase margin and slowest pole).

---

## 3. The two pole patterns

| | **Approach A** — complex pair + real pole | **Approach B** — three real poles |
|---|---|---|
| poles | −ζωₙ ± jωₙ√(1−ζ²), −αζωₙ | −λ₁, −λ₂, −λ₃ |
| search variables | ωₙ, ζ, α (the pair moves as a pair) | λ₁, λ₂, λ₃ (each pole on its own) |
| step design | β\* = 1/(1 + 2αζ²): the step becomes ωₙ²/(s² + 2ζωₙs + ωₙ²) | β\*: hides λ₁, the step follows λ₂ and λ₃ (no oscillation) |
| trajectory design | β = 1 | β = 1 |
| weak point | relies on an exact cancellation | relies on an exact cancellation; no complex pair, so a slower rise |

Example poles found earlier with the model (they depend on the requirements and on the PID start):
A: ωₙ = 33 rad/s, ζ = 0.945, α = 0.225, β\* = 0.71. B: −9.7, −21.0, −35.9 rad/s.

---

## 4. Folder

```
pp_design/
├── README.md            this file
├── design_A.m           search the poles of approach A (complex pair + real pole)
├── design_B.m           search the poles of approach B (three real poles)
├── check_design.m       test poles chosen by hand (wn, zeta, alpha or the 3 poles), no search
├── explore_poles.m      interactive: drag or type the poles, see step, voltage, Bode, requirements (PID, β*, β = 1)
├── docs/
│   ├── pp_design_theory.pdf     theory, proofs, β and zeros, cost function, A vs B, noise-model proposal
│   └── pp_design_workflow.pdf   call tree of every script (which function calls which)
└── utils/
    ├── pp_setup.m       requirements + motor models + PID poles (runs pid_tune.m)   <- edit numbers here
    ├── pp_gains.m       poles -> gains (acker), observer gain, beta*
    ├── closed_loop.m    closed loop with 3 ms delay: step, voltage, margins, chatter
    ├── pp_check.m       the six requirements as violation numbers (<= 0 = met)
    ├── pp_search.m      the search (cost function + fminsearch)
    ├── pp_report.m      table in the terminal + step plots (PID, beta*, beta = 1)
    └── pp_simulink.m    writes the Simulink variables of both motors in the workspace
```

`pid_tune.m` (one folder up) is needed: it gives k, τ and the PID gains.

---

## 5. How to use it

1. **Requirements.** If needed, change the numbers in `utils/pp_setup.m` (section 1).
2. **Explore** (optional). Run `explore_poles.m`: drag the poles (keys `c` = pair, `r` = real, `p` = PID)
   or type them in the STATS window.
3. **Design.** Run one of the following. Each one sets everything in its **section 1** and is then run as a whole.
   - `design_A.m`: settings `start_from_pid`, `wn`, `zeta`, `alpha`, `beta_mode` (`'star'` or `'one'`), `do_search`;
   - `design_B.m`: the same, with `lambda` instead of `wn`, `zeta`, `alpha`;
   - `check_design.m`: give the poles by hand, choose `beta` (a name or a number) and the motor(s).

   | `start_from_pid` | `do_search` | poles used |
   |---|---|---|
   | true | true | PID poles, then moved by the search |
   | true | false | PID poles converted to the pattern, no search |
   | false | true | your values, then moved by the search |
   | false | false | exactly your values |

   β is **not** searched: `beta_mode` chooses it. With `'star'`, β\* is recomputed from the poles at every
   step of the search. Use `'star'` for step following, `'one'` for trajectories.
4. **Read the results.** The terminal prints a table for each motor with PID, β\* and β = 1
   (`*` = requirement not met), and a figure shows poles, angle and voltage.
5. **Simulink.** The script has already written in the workspace, for both motors (`_3165`, `_3166`):
   `Kx_`, `Ki_`, `Kaw_`, `A_obs_`, `B_obs_`, `beta_`, `k_*_load`, `tau_*_load`, the PID gains and `Vmax`.
   Run `slx/Model_pp_obs.slx` (simulation) or `slx/CNC_pp_obs.slx` (robot).

**One-time edit in Simulink:** in each `pp + luenberg` subsystem, add a Gain block `beta_3165`
(`beta_3166`) on the reference φ before `Mux1`. The 0.5 V load step in `Model_pp_obs.slx` is only
a test: set the *Final value* of `Step` and `Step1` to 0 to remove it.

---

## 6. Limits and next steps

- The model is linear (no saturation, no friction, no backlash) and the encoder is ideal. A proposal for
  measuring the real noise and friction and adding them to `Model_pp_obs.slx` is the last page of the theory pdf.
- The design scripts check both motors with the same poles. The gains differ, because k and τ differ.
- To use more of the motor (it can reach about 490 °/s at 6 V), first make the paths faster, then lower
  `ts_max`. Chatter, the 3 ms delay and the 1 A continuous current are the real limits (theory pdf, section 8).
