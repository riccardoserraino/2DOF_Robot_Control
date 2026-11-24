# Trajectories

All the tip paths are generated **in this folder**. Each file returns the tip position,
velocity and acceleration at time `t`, and the Simulink models call these same files.

| `traj_id` | file | path |
|:-:|---|---|
| 1 | `square_single.m` | square, one speed |
| 2 | `square_speeds.m` | square, speed levels |
| 3 | `circle_single.m` | circle, one speed |
| 4 | `triangle_single.m` | triangle, one speed |
| 5 | `star_single.m` | star, one speed |
| 6 | `rhombus_single.m` | rhombus, one speed |
| 7 | `traj_sequence.m` | **benchmark**: circle, square, rhombus, each at 3 speeds |

Every path starts with a smooth approach from home to its first point, waits there, runs its
laps, then stops. Size, lap time, laps and speeds are set at the top of each file.
`*_single` runs at one `speed`: use them to tune the size and to find the speeds.
`square_speeds` runs `laps_per_level` laps at each value of `speeds`, with a short stop between
levels. `traj_sequence.m` is the **benchmark**: in one run it does, for each shape in `order`,
2 laps of 4 s, 2 laps at about 3 V and 2 laps at about 6 V (saturation), moving smoothly from
one shape to the next. Its Settings hold the order, the speeds of each shape, the sizes (keep
them equal to the single files) and all the times.

## How to use `check_trajectory.m`

Run it from the repository root. In section 1 set:
- `traj_id`: the path to check (table above);
- `controller_setup` and `model_name`: the controller to use, e.g. `pole_place.m` with
  `Model_pp_obs`. **To try another controller on the same path, change these two lines.**

Then use it in two steps:

1. **Safety check.** It converts the path into joint angles (inverse kinematics) and checks
   that both joints stay inside the safe region: `q_lim` = 24.4° is the singularity margin
   from `model_id/singularities_check.m`, `q_warn` = 20° leaves room for the tracking error.
   If a joint goes beyond, make the path smaller.
2. **Speeds for the benchmark.** From the motor model it estimates the feedforward voltage,
   and prints the `speed` that gives 2, 3, 4, 5, 6 V on the motors. Run it on the
   `*_single` file of each shape at speed 1, and copy the speeds for 3 V and 6 V into the row
   of that shape in the `speeds` matrix of `traj_sequence.m` (keep `1` first).

It also runs the Simulink model and prints, for each speed level, the peak voltage, the time
in saturation and the tip error, with plots of path, joints, error and voltage.
The times it prints: *motion ends at* = end of the last lap; *Stop Time to set* = that plus
0.5 s at rest.

## In Simulink

The models contain **one MATLAB Function block** for all the paths:

```matlab
function [x, y, vx, vy, ax, ay] = fcn(t, traj_id)
[x, y, vx, vy, ax, ay] = traj_select(t, traj_id);
```

`t` comes from a Clock, `traj_id` from a Constant block that reads the variable `traj_id` from
the MATLAB workspace: set it together with the controller parameters before running
(`check_trajectory` and `pp_tune` set it for you; the model's InitFcn uses 2 if it is missing).
`traj_select.m` calls the file of that path, so editing a file here changes the model too:
nothing is pasted in the blocks. The InitFcn adds this folder to the path relative to the
model file, so it works on any PC. `traj_select(-1, traj_id)` gives the length of the path
and can be used as Stop Time.

**New path:** write its file with the same 6 outputs and the same `t < 0` query as the
others, then add one `case` line in `traj_select.m`.
