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
- `include/hybrid_control_ft/` – independent sensor-based controller headers, using base-frame control
- `config/` – robot configuration files  
- `scripts/` – utility scripts (e.g., CAN setup)

## Notes

### Runtime F/T controller tuning

Edit `config/hybrid_control_ft.conf`, then stop and restart `hybrid_control_ft`.
No rebuild is needed for tuning changes. The program prints the loaded file path
and rejects missing or invalid settings. Changes are not loaded during a run.
The default path points to this checkout, independent of the working directory.
To use a different file (or run a relocated executable), set:

```bash
export HYBRID_CONTROL_FT_CONFIG=/absolute/path/to/hybrid_control_ft.conf
```

This is separate from `BARRETT_CONFIG_FILE`, which selects robot calibration and
kinematics. The tuning file controls diagonal Cartesian P/D and P/I gains,
feedforward gain, integral bound, velocity/force axis selection, force target,
sinusoidal motion, feedback clamps, target joint pose, and logging cadence/path.
Joint hold gains use the selected robot configuration by default; set
`use_robot_hold_gains = false` to use `hold_kp` and `hold_kd` from the tuning file.
The planar joint assignment (joints 2/4 active), sensor sign convention, and
robot execution period remain structural settings rather than tuning options.
The checked-in tuning values preserve the source settings at migration:
force Kp z = 0.3, Ki z = 0, target z = -5 N, peak velocity = 0 m/s.

The F/T headers live in the `hybrid_control_ft` namespace. They provide sensor
acquisition (`barrett_ft_force.hpp`), sensor-to-base conversion
(`sensor_force_to_base.hpp`), the Jacobian (`jacobian.hpp`), references
(`references.hpp`), feedback clamping (`force_clamp.hpp`), the control law
(`base_frame_controller.hpp`), Jacobian-transpose torque mapping
(`joint_torque.hpp`), and base-frame CSV logging (`csv_logger.hpp`). They do not
include task-frame transforms or dynamics estimators. Each executable includes
its own folder explicitly, so its headers can be modified independently.

`hybrid_control_ft` uses the Barrett F/T sensor's force instead of the torque-based
force estimate. It keeps the original hybrid controller gains, references,
force clamp, keyboard commands, and CSV columns. Logs use
`../../data/hybrid_base_ft_datalog_NNNN.csv` relative to the working directory.
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
No automatic tare or
payload compensation is applied. Measured moments are not used.

Build with `cmake -S . -B build` and
`cmake --build build --target hybrid_control_ft`. The executable is
`build/bin/hybrid_control_ft`.

This is an experimental research codebase for studying contact-rich manipulation and control on torque-controlled manipulators.
