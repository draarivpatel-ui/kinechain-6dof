# KineChain 6DOF

An interactive spatial robotics workbench that runs entirely in the browser.
Denavit-Hartenberg forward kinematics, damped least squares inverse
kinematics, singularity analysis, quintic minimum-jerk trajectories and a
ROS URDF exporter, all computed locally by one zero-dependency engine with
no build step, no packages and no network requests.

**Live demo:** [https://aariv.me/kinechain-6dof/](https://aariv.me/kinechain-6dof/)

## The workbench

- Pick a robot arm: PUMA-560 and a UR5e-class arm (6 revolute joints each),
  or SCARA-4 (2 revolute, 1 prismatic, 1 revolute). Drive each joint slider
  or apply one of eight shipped pose presets.
- Two live projections redraw on every change: side view (X,Z) and top
  view (X,Y).
- Set a Cartesian target (position plus roll, pitch, yaw) and solve it with
  damped least squares IK, with optional sphere obstacles in the workspace.
- Plan a quintic pick-and-place trajectory and scrub it along its timeline,
  then download the model as URDF or the run as JSON and CSV.
- Queue up to twelve end-effector waypoints in the sequencer, each with an
  optional dwell hold, and play them through as a C2 cubic spline, a
  trapezoidal velocity profile or a quintic path. The sequencer chains
  position IK solve to solve so the arm stays in one branch, and reports
  unconverged targets, joint-limit, floor-clearance and obstacle contacts
  as honest warnings.
- Watch the live singularity risk monitor: Yoshikawa manipulability plus
  the sigma ratio fold into a four-band colour code on the stats, the
  reach envelope ring and the end marker in both views, a velocity
  ellipsoid at the tool point, and a risk heatmap strip under the
  trajectory scrubber.
- Export a portable bundle in one click: verified trajectory JSON with
  waypoint metadata and solve status, a CSV joint-angle table, a basic
  G-code toolpath and a URDF joint-state snapshot, each with a SHA-256
  checksum computed by a built-in pure JS implementation.
- Run the in-page self-test battery (FK/IK round trip, Jacobian finite
  difference parity below 1e-5, singular manipulability collapse, floor
  clearance, hash recovery and more) without a test runner.
- Bookmark the whole bench: robot, joint pose, waypoint list and
  interpolator mode serialise into the URL hash and restore on load.

## Architecture

`src/robotics.js` is the entire engine, a plain script that also loads
under Node via `require()`, so the test suite exercises the exact code the
page runs. Its modules cover:

- dense matrix math, symmetric eigendecomposition, SVD and matrix rank;
- quaternions, Euler angles and SE(3) rigid transforms;
- Denavit-Hartenberg forward kinematics and per-link frames;
- the geometric Jacobian, Yoshikawa manipulability and a singularity
  detector;
- damped least squares inverse kinematics with joint limits, multi-start
  solving and sphere-obstacle avoidance;
- quintic minimum-jerk trajectory planning, jerk cost and pick-and-place
  sequencing;
- clamped C2 cubic splines and trapezoidal velocity profiles for
  multi-waypoint joint paths;
- a Cartesian waypoint sequencer with IK chaining, dwell holds,
  floor-clearance and collision warnings;
- a singularity risk index, velocity ellipsoid computation and planar
  projection;
- deterministic exports: trajectory JSON/CSV, G-code, URDF joint states
  and SHA-256 checksums;
- URL hash state serialisation and the in-page self-test battery;
- sphere collision checking, link sampling and clearance profiles;
- URDF export with a built-in validator.

`app.js` is the browser shell: sliders, views, downloads, and an accessible
control surface. `index.html` and `style.css` ship the page as a static
site.

## Local development

Open `index.html` in any modern browser, or serve the folder over HTTP.

Run the test suite (Node 22+, no packages to install):

```sh
npm test
```

Verified: 338 tests, 338 passing, 0 failures. The suite covers transform
and quaternion identities, forward kinematics against hand-checked poses,
Jacobian analytical versus central finite-difference parity below 1e-5,
manipulability properties, singularity detection at elbow and wrist
alignments, IK convergence and joint limits, cubic spline and trapezoidal
boundary conditions, waypoint IK chaining, dwell holds, floor-clearance
and collision flags, deterministic export checksums, URL hash recovery,
URDF round-trip validation, and page-level UI invariants.

## License

MIT. See `LICENSE`.
