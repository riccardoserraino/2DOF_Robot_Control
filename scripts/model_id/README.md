# Model identification and kinematics

How we obtained the motor model and the geometry used by every controller.
Run these scripts **from this folder** (they use `../parameters.m` and `../../data/`).
Results are written by hand into `scripts/parameters.m`. Back to the [scripts README](../README.md).

| step | script |
|---|---|
| 1. Motor model | `actuator_model.m` |
| 2. Identification | `multisine_design.m` → `identification.m` |
| 3. Kinematics | `kinematics.m` (generates `FK_classic.m`, `J_classic.m`) |
| 4. Singularities and safe region | `singularities_check.m` |
| 5. Friction and delay of the simulated plant | values in `parameters.m`, section 5 |

## 1. Motor model

Each joint is a DC motor with a gearbox. The electrical pole $`R_m/L_m`$ is a few hundred
times faster than the mechanical one, so the inductance is neglected and the model from
voltage to angle is

```math
G(s)=\frac{\theta(s)}{V(s)}=\frac{k}{s(\tau s+1)}
```

an integrator plus one mechanical pole: only $`k`$ and $`\tau`$ have to be identified.
`actuator_model.m` compares the full and simplified models from the datasheet values.

## 2. Identification

$`k`$ and $`\tau`$ are estimated with a **grey-box** fit (`procest`, integrator + one pole) on
one experiment and validated on another one.
The input is a **periodic multisine** (`multisine_design.m`): a sum of sines on exact FFT
bins, repeated over several periods, that excites the whole band of interest in one test with
a low peak voltage. The test was run first on the motors alone, then with the linkage
attached: the **load** models are the ones used for control (`k_xxxx_load`, `tau_xxxx_load`).

## 3. Kinematics

With A at the origin and E at $`(2l, 0)`$, the motors set the elbows
$`B = l[\cos\varphi;\ \sin\varphi]`$ and $`D = [2l - l\sin\theta;\ l\cos\theta]`$.
The pen C is at distance $`l`$ from both (intersection of two circles), which gives a
closed-form **forward kinematics** and its **Jacobian**. The **inverse kinematics** is solved
numerically (Newton, starting from the previous sample); joint speeds come from
$`\dot q = J^{-1}\dot p`$. At home ($`\varphi=\theta=0`$) the linkage is a square and the
axes are almost decoupled.

## 4. Singularities and safe region

`singularities_check.m` draws, in joint and Cartesian space, where the linkage is singular:
**serial** (an arm stretched or folded), **parallel** (B, C, D aligned: the pen moves with the
motors locked) and **degenerate** (B = D). It also prints the **safe margin for both joints
together**: the largest square $`|\varphi|, |\theta| \le q_{safe}`$ around home with no
singularity. It is $`q_{safe} \approx 24.4°`$, limited by the parallel singularity; the
trajectories keep the joints below 20° to leave room for the tracking error.

## 5. Friction and delay in the simulated plant

The identified model has no friction and no delay, but the real motors do. A second fit of
the recorded multisine data with an **input delay** and **Coulomb friction** gave the values in
`parameters.m`, section 5 (`k/tau_xxxx_plant`, `Td_xxxx` = 3.5 ms, `Vc_xxxx` = 0.12 / 0.14 V).
They are used **only by the motor blocks of the Simulink models** (`Model_*.slx`): the Coulomb
friction is modelled as an opposing voltage $`V_c \tanh(\omega/\omega_\varepsilon)`$ (smooth
around zero speed) plus the delay at the motor input, so the simulation behaves like the bench.
`use_real_plant = false` gives a simulated plant equal to the model, without friction or delay.
