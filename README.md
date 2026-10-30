# Contact Control for Barrett WAM

This repository contains C++ implementations of contact-based control strategies for the Barrett WAM robotic arm using `libbarrett`.

The project focuses on:
- Hybrid force/motion control
- External force and torque estimation
- Task-space to joint-space transformations
- Interaction with simple environments (e.g., planar contact)

The code is designed for real-time execution and experimentation on physical WAM hardware.

## Structure
- `src/` – main control implementations  
- `include/hybrid_control/` – original controller headers, including force estimation and task-frame transforms
- `include/hybrid_ft/` – shared F/T sensor, configuration, torque mapping, and logging utilities
- `include/hybrid_fv_ft/` – force/velocity controller and sinusoidal velocity reference
- `include/hybrid_fp_ft/` – force/position controller and position trajectory
- `config/` – robot configuration files  
- `scripts/` – utility scripts (e.g., CAN setup)

## Notes

### Runtime F/T controller tuning

Edit `config/hybrid_fv_ft.conf`, then stop and restart `hybrid_fv_ft`.
No rebuild is needed for tuning changes. The program prints the loaded file path
and rejects missing or invalid settings. Changes are not loaded during a run.
The default path points to this checkout, independent of the working directory.
To use a different file (or run a relocated executable), set:

```bash
export HYBRID_FV_FT_CONFIG=/absolute/path/to/hybrid_fv_ft.conf
```

This is separate from `BARRETT_CONFIG_FILE`, which selects robot calibration and
kinematics. The tuning file controls diagonal Cartesian P/D and P/I gains,
feedforward gain, integral bound, velocity/force axis selection, force target,
sinusoidal motion, feedback clamps, target joint pose, and logging cadence/path.
Joint hold gains use the selected robot configuration by default; set
`use_robot_hold_gains = false` to use `hold_kp` and `hold_kd` from the tuning file.
The planar joint assignment (joints 2/4 active), sensor sign convention, and
robot execution period remain structural settings rather than tuning options.
The current tuning values are in each executable's own configuration file.

Shared F/T utilities live in the `hybrid_ft` namespace and header folder.
The `hybrid_fv_ft` and `hybrid_fp_ft` namespaces/folders contain their respective
`controller.hpp` and motion-reference headers. Both use
`hybrid_ft/constant_output.hpp` for constant force references.

`hybrid_fv_ft` uses the Barrett F/T sensor's force instead of the torque-based
force estimate. It keeps the original hybrid controller gains, references,
force clamp, keyboard commands, and CSV columns. Logs use
`hybrid_fv_ft_datalog_NNNN.csv` in `log_directory` (relative to the working directory).
The sensor must be detected at startup. Its force is rotated from tool to base
coordinates; sensor axes are assumed aligned with tool axes. Velocity feedback,
references, controller output, and logged vectors all use the base frame. The
controller output maps directly to joint torques through the base-frame linear
Jacobian transpose on joints 2 and 4 only (zero-based indices 1 and 3).
All other joints, including wrist joints on a 7-DOF WAM, use position-hold PD
torques: `Kp * (q_hold - q) - Kd * qdot`. Gains come from the WAM joint-position
controller. Each press of `c` that starts contact captures the current joint
positions as hold targets; this hold operates only during contact control.
Gravity compensation remains enabled for all joints. This is a compliant servo
hold, not a mechanical lock or a fixed tool-orientation controller. The held
pose determines the motion plane; the references still use base x and z.
Sensor and base force prints both show environment-on-robot force; the
sensor-to-base system only rotates the vector. A separate gain of -1 converts
base force to robot-on-environment force for controller feedback and CSV logs.
Controller feedback is clamped
to +/-5 N on x/y and +/-20 N on z; this is not a contact-force or output-torque limit.
The `t` command tares the F/T sensor after the robot reaches the target and
before accepting the next command. Payload compensation is not applied.
Measured moments are not used.

Contact CSV logs preserve the original twelve velocity/force columns and append:
`time_s` (steady-clock seconds since the first logged sample),
`fsensor_x/y/z` (sensor-frame environment-on-robot force in N, including any tare),
`q1` through `q4` or `q7` (joint positions in radians, depending on robot DOF),
and `R_world_tool_00` through `R_world_tool_22` (row-major tool-to-world rotation).
The rotation is the exact matrix used by the force conversion; world equals base
when `world_to_base` is identity. Thus logged force satisfies
`fcur = -R_world_tool * fsensor`, before feedback clamping.
Each new contact log restarts `time_s` at zero. Existing CSV files are unchanged.

Build with `cmake -S . -B build` and
`cmake --build build --target hybrid_fv_ft hybrid_fp_ft`. The executables are
`build/bin/hybrid_fv_ft` and `build/bin/hybrid_fp_ft`.

This is an experimental research codebase for studying contact-rich manipulation and control on torque-controlled manipulators.

### Force/position controller (`hybrid_fp_ft`)

The former `hybrid_control_ft` executable/source/config is now `hybrid_fv_ft`.
The original torque-estimation `hybrid_control` remains separate. Shared F/T
headers use `include/hybrid_ft/`; controller-specific headers use the matching
`include/hybrid_fv_ft/` and `include/hybrid_fp_ft/` folders.
Use the newly named binary; an old build directory may still contain an obsolete
`hybrid_control_ft` binary that CMake does not remove.

`hybrid_fp_ft` reads `config/hybrid_fp_ft.conf`; override it with
`HYBRID_FP_FT_CONFIG`. It uses position PD on base x and force feedforward + PI
on the complementary axes:

`u = S * (Kp * (p_des - p) + Kd * (v_des - v)) + (I-S) * (gff * f_des + Kf * (f_des-f) + Ki * integral)`.

The position version requires `motion_axes = [1, 0, 0]`. With the supplied zero
y-force reference/gains, only x position and z force are controlled. Joints 2/4
receive Jacobian-transpose torques and the other joints hold their activation
positions, as in the force/velocity executable.

1. Enter `t`: move to the configured joint target, tare the sensor, and capture
   the actual Cartesian tool position as the reference origin.
2. Enter `c`: move toward `origin.x + position_distance` while maintaining the
   configured force. Default distance is **+0.10 m** along base x. A quintic
   reference takes **6 seconds** (`position_duration`), with zero reference
   velocity/acceleration at its endpoints. It then holds the x endpoint while
   force control continues. Physical tracking depends on gains/contact; the
   elapsed trajectory duration does not certify that the robot reached the goal.
3. Enter `x` to stop. Restarting with `c` interpolates from the current position
   to the same endpoint, without adding another 10 cm. Entering `t` captures a
   new origin. Enter returns home and invalidates that origin; `c` then requires
   another `t`. `q` exits.

`position_kp` (N/m) and `position_kd` (N*s/m) tune the position servo. The supplied
150/20 x gains are tuning values, not hardware-validated tuning. Force settings
are copied from the current FV configuration, including the -10 N z reference.
Force engagement still uses the existing constant reference; no force ramp or
payload compensation is added.

FP logs use `hybrid_fp_ft_datalog_NNNN.csv` and include all existing diagnostics
plus `pdes_x/y/z` and `pcur_x/y/z` in meters. Only x of the position reference is
controlled. The old experiment CSVs are left under their original names.

Hardware-free checks:

```bash
cmake -S . -B build -DCONTACT_CONTROL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Plot FP position x and force z against the recorded timestamps (requires Python 3
and `matplotlib`):

```bash
python3 scripts/plot_hybrid_fp_ft_csv.py data/hybrid_fp_ft_datalog_0001.csv
```

To save without opening a window:

```bash
python3 scripts/plot_hybrid_fp_ft_csv.py data/hybrid_fp_ft_datalog_0001.csv --save fp_plot.png --no-show
```

Both panels compare desired and measured values. Position is in meters; force
is in newtons using the robot-on-environment convention.

### Sinusoidal FP position reference

In `config/hybrid_fp_ft.conf`, select either `position_pattern = "move"` (the
existing one-way move and hold) or `position_pattern = "sine"`. Restart the
executable after editing, then use `t` followed by `c`.

For sine mode:

```text
position_pattern = "sine";
position_amplitude = 0.05; # meters, +/-5 cm around captured target x
position_period = 6.0;    # seconds per cycle
position_duration = 6.0; # startup ramp duration in seconds
```

After the ramp, `x_des = x_target + amplitude * sin(2*pi*t/period)`, where time
starts on `c`. A quintic envelope blends from the actual current x position
into this sine during `position_duration`, starting at zero reference velocity.
The damping reference is the analytic derivative, including the ramp term.
The pattern repeats until stopped; `position_distance` is unused in sine mode.
Each restart with `c` restarts phase and ramp, keeping the origin captured by
`t`. Force control and the FP CSV format are unchanged, so the FP plotting
script works with either pattern.

### Finite FV move reference

`config/hybrid_fv_ft.conf` supports `velocity_pattern = "sine"` (the existing
periodic reference, still selected by default) and `velocity_pattern = "move"`:

```text
velocity_pattern = "move";
velocity_axis = 0;
velocity_distance = 0.10; # signed meters of commanded displacement
velocity_duration = 6.0; # seconds
```

Move mode uses the derivative of the FP quintic move:
`v_des = (distance/duration) * 30*s^2*(1-s)^2`, with `s = t/duration`,
and zero velocity outside the move interval. It starts and ends smoothly; its
integral is `velocity_distance`. Peak speed is `1.875*abs(distance)/duration`.
`velocity_peak` and `velocity_period` apply only to sine mode.

Restart after editing, use `t` to reach/tare at the target, then `c` to start.
Each new activation with `c` restarts the full velocity pulse. After its duration,
the velocity reference remains zero while force control continues until stopped.
This is velocity feedback, so actual distance depends on tracking; there is no
position-error correction or absolute endpoint hold. Unlike FP, restarting FV
move mode commands another full relative displacement. CSV columns are unchanged.

Plot FV desired/measured x velocity and z force using logged timestamps:

```bash
python3 scripts/plot_hybrid_fv_ft_csv.py data/hybrid_fv_ft_datalog_0001.csv
```

Add `--save fv_plot.png --no-show` to save without opening a window. Requires
`matplotlib`. Velocity is in m/s and force is in N; both sine and move logs use
the same format.

New FV logs also append `pdes_x/y/z` and `pcur_x/y/z`. The desired position is
computed analytically by integrating the velocity reference and adding the
actual tool position captured on each `c` activation. It is a diagnostic
reference, not position feedback. Measured position comes from robot kinematics.
The FV plotting script shows position x, velocity x, and force z in three panels.
Older logs without position columns still show velocity and force only, with a
notice; they cannot provide measured Cartesian position through this script.

### Force low-pass filtering (FP and FV)

`force_filter_cutoff_hz = 20.0` enables a first-order low-pass on all three
base-frame, robot-on-environment force components. Set it to `0` to bypass;
older configs without this setting also bypass. Restart to apply changes.
The filter uses the execution-manager period and
`alpha = 1 - exp(-2*pi*cutoff_hz*dt)`, with `filtered += alpha*(input-filtered)`.
It initializes to the first reading and resets after the target-pose tare.
The path is sensor -> rotation -> sign conversion -> low-pass -> clamp -> controller.

`fcur_x/y/z` remain unfiltered, unclamped measurements. New `ffiltered_x/y/z`
columns record filtered force before clamping. Both FP and FV plotting scripts
show filtered z force when available and continue to support older logs.
Filtering introduces lag; 20 Hz is an initial setting, not hardware-validated
control tuning. It does not eliminate the physical startup force transient or
provide a contact-force limit.
