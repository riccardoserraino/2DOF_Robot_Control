# Trajectory paths

The trajectory functions generate the tip position, velocity, and acceleration. Their logic is also used by the MATLAB Function blocks in the Simulink models.

| File | Path |
|---|---|
| `square_gen_smooth.m` | Square |
| `triangle_gen_smooth.m` | Equilateral triangle |
| `circle_gen_smooth.m` | Circle |
| `star_gen_smooth.m` | Five-pointed star |

## Trajectory parameters

Edit these values in the settings section at the top of the selected trajectory function.

| Parameter | Default | Used by | Purpose |
|---|---:|---|---|
| `speed` | 1 | all paths | Scales the path traversal speed. |
| `T0` | 4 s; star: 6 s | all paths | Duration of one lap at `speed = 1`. |
| `s` | 60 mm | square, triangle | Side length. |
| `D` | 70 mm | circle, star | Circle diameter or star outer diameter. |
| `c` | `[0.127; 0.127]` m | all paths | Centre of the path. |
| `home` | `[0.127; 0.127]` m | all paths | Tip position at the start of the approach. |
| `t_app` | 1.0 s | all paths | Duration of the smooth approach to the first point. |
| `t_wait` | 0.5 s | all paths | Pause at the first point before starting laps. |
| `n_laps` | `Inf` | all paths | Number of laps before stopping at the first point. |
| `ccw` | `true` | all paths | Traversal direction; `false` reverses it. |
| `ratio` | 0.382 | star | Inner-to-outer radius ratio. |
| `th0` | `5*pi/4` | circle | Starting angle on the circle. |

Each path starts with a smooth approach from `home` to its first point, pauses there for `t_wait`, follows the path for `n_laps`, then stops at the first point. `ccw` selects the direction for closed paths.

## Check a trajectory

In `check_trajectory.m`, select the MATLAB trajectory with `traj`. Set `model_name` and `controller_setup` to the Simulink model and controller setup to use. The setup script should publish the variables required by its model to the base workspace. Set `controller_setup = ''` to use variables already loaded in MATLAB.

Run the script from the repository root. Make sure the trajectory selected inside the model matches `traj`.

The checker follows these steps:

1. Generate the selected tip position, velocity, and acceleration over time.
2. Use inverse kinematics to map tip position to joint angles, then the Jacobian to map tip velocity to joint speed; estimate joint acceleration by numerical differentiation.
3. Estimate feedforward voltage from the motor model and check the joint angle limits.
4. Prepare the selected controller and run its Simulink model.
5. Summarize the simulated voltage signals and use bisection to find the speed factor for the requested feedforward voltage.
6. Plot the desired and followed tip paths, joint reference and plant angles, joint speed, and voltage signals with saturation limits.
