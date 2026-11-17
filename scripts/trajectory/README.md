# trajectory — reference paths for the tip

Functions that generate the path of the tip C (position, velocity, acceleration in m, m/s,
m/s²). The code of each one goes into its **MATLAB Function block** in `Model_*.slx` /
`CNC_*.slx`; the controller turns it into joint references with the inverse kinematics
and uses the speeds and accelerations for the feedforward.

| file | path |
|---|---|
| `square_gen_smooth.m` | square, side `s` |
| `triangle_gen_smooth.m` | equilateral triangle, side `s` (centroid in `c`) |
| `circle_traj_smooth.m` | circle, diameter `D` (the function inside is `circle_gen_smooth`) |
| `star_gen_smooth.m` | five-pointed star, outer diameter `D` |
| `check_trajectory.m` | checks a path in MATLAB before simulating (no Simulink); motor model read from `parameters.m` |

Every path: **straight approach from home** to the first point (no step), a short stop, then
the laps. Each side (each lap for the circle) uses a quintic time law: zero speed and
acceleration at every corner.

## Parameters to change (top of each function)

| parameter | default | what it does |
|---|---|---|
| `speed` | 1 | 2 = twice as fast: joint speeds x2, accelerations x4 |
| `T0` | 4 s (star 6 s) | lap time at `speed = 1` |
| `s` / `D` | 60 mm / 70 mm | size. Max for the ±20° joint limit (joints ≤ 18°): square 66, triangle 67, circle 78, star 78 mm |
| `c` | [0.127; 0.127] | centre of the path (home of the tip) |
| `home` | [0.127; 0.127] | where the tip is at t = 0 |
| `t_app`, `t_wait` | 1.0 s, 0.5 s | approach time, stop before the laps (`t_skip` in `validate_controllers` ≥ 1.5 s + one lap) |
| `n_laps` | Inf | laps, then stop at the first point |
| `ccw` | true | direction |
| `ratio` (star) | 0.382 | inner/outer radius (0.382 = regular star) |
| `th0` (circle) | 5π/4 | start point on the circle |

## `check_trajectory.m`: check before simulating

1. Set `traj = @square_gen_smooth;` (or another path) and, if you want, `u_target`.
2. Run it. It prints for each joint: angle range (**OK / WARNING above 18° / NOT OK above 20°**),
   peak speed [rad/s], peak acceleration [rad/s²], peak feedforward voltage; the tip peak speed
   [m/s] and acceleration [m/s²]; the factor to apply to `speed` so that the feedforward reaches
   `u_target` volts.
3. It plots the path, the joint angles with the limits, the joint speeds and the voltages.

Use it every time you change `speed` or a size. Keep `u_target` at 2–3 V: the feedback needs the
rest of the 6 V. The voltage is rarely the limit: the tracking error grows about with `speed`
and the contour error at the corners about with `speed`²; check them in simulation
(`observer_study`, `validate_controllers`).

## The math behind `check_trajectory.m` (benchmark)

**1. Time law and speed factor.** Each segment from A to B lasts T_k and follows
p(t) = A + σ(f)(B − A), f = t/T_k, with the quintic σ(f) = 10f³ − 15f⁴ + 6f⁵
(σ = σ' = σ'' = 0 at f = 0 and σ = 1, σ' = σ'' = 0 at f = 1). Peaks:
ṗ_max = 1.875·|B − A|/T_k at f = 0.5, p̈_max = 5.774·|B − A|/T_k² at f = 0.5 ∓ √3/6.
`speed = s` divides every T_k by s: at the same point of the path the tip speed is
multiplied by s and the acceleration by s².

**2. Kinematics** (four-bar linkage, all links l = 0.127 m, as in `model_id/kinematics.m`).
Forward: with a = 2 − sin θ − cos φ, b = cos θ − sin φ, d = √(a² + b²), h = l·√(1 − d²/4),

    x = l/2·(2 − sin θ + cos φ) − h·b/d
    y = l/2·(cos θ + sin φ)     + h·a/d

Inverse (no closed form used): Newton iterations q ← q − J(q)⁻¹(f(q) − p), with the Jacobian
J = ∂f/∂q by central differences (ε = 10⁻⁷), starting from the previous sample (from q = 0 at
t = 0, the home pose), until |f(q) − p| < 10⁻¹² m.
Joint speed: q̇ = J(q)⁻¹·ṗ. Joint acceleration: backward difference q̈ₙ = (q̇ₙ − q̇ₙ₋₁)/Ts,
Ts = 2 ms (the exact q̈ = J⁻¹(p̈ − J̇q̇) gives the same values to within the sampling error).

**3. Feedforward voltage** = the inverse of the motor model used by the controller,
τq̈ + q̇ = k·(u − V_c·sign q̇), solved for u (no simulation needed):

    u_ff = (q̇ + τ·q̈)/k + V_c·sign(q̇)        [q in rad, k in rad/(V s), u in V]

The script reports |q̇ + τq̈|/k + V_c (the worst sign). k, τ, V_c are `k/tau_xxxx_plant` and
`Vc_xxxx` of `parameters.m` (assumption: φ = motor 3165, θ = motor 3166). The feedback adds
its own voltage on top: only the simulation gives the total.

**4. Speed for a voltage target.** Over the laps q̇ scales with s and q̈ with s², so

    u(s) = max over the lap of  |s·q̇(t) + s²·τ·q̈(t)|/k + V_c

grows monotonically with s. Bisection on s ∈ [0.01, 50] (60 halvings) finds u(s) = `u_target`.
The approach from home (fixed `t_app`) is not scaled and is excluded.

**5. Benchmark** (defaults, `use_real_plant = true`, `speed = 1`; your run should match):

| path | joints [deg] | peak q̇ φ/θ [rad/s] | peak q̈ φ/θ [rad/s²] | tip v [m/s] / a [m/s²] | peak u_ff φ/θ [V] | `speed` for u_target = 3 V |
|---|---|---|---|---|---|---|
| square 60 mm, 4 s | −13.7 … 15.9 | 0.89 / 0.89 | 2.9 / 2.9 | 0.112 / 0.35 | 0.63 / 0.66 | 4.46 |
| triangle 60 mm, 4 s | −13.3 … 15.9 | 0.54 / 0.66 | 1.3 / 1.6 | 0.084 / 0.20 | 0.43 / 0.52 | 5.98 |
| circle 70 mm, 4 s | −16.0 … 16.1 | 0.80 / 0.82 | 2.0 / 2.9 | 0.103 / 0.30 | 0.57 / 0.62 | 4.88 |
| star 70 mm, 6 s | −14.8 … 16.0 | 0.60 / 0.65 | 3.2 / 3.4 | 0.079 / 0.41 | 0.47 / 0.53 | 4.72 |

The voltage allows `speed` ≈ 4.5–6 before 3 V of feedforward; the accuracy (tracking error
∝ s, contour error at the corners ∝ s²) usually limits much earlier: check it in simulation.

## Which path is hardest

With the same lap time: **star** (10 starts and stops per lap) > **square** > **triangle** >
**circle** (no corners; its weak point is friction, because each joint reverses while the tip
is moving).
