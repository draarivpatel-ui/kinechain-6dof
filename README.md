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

## Architecture

`src/robotics.js` is the entire engine, a plain script that also loads
under Node via `require()`, so the test suite exercises the exact code the
page runs. Its seventeen modules cover:

- dense matrix math, symmetric eigendecomposition, SVD and matrix rank;
- quaternions, Euler angles and SE(3) rigid transforms;
- Denavit-Hartenberg forward kinematics and per-link frames;
- the geometric Jacobian, Yoshikawa manipulability and a singularity
  detector;
- damped least squares inverse kinematics with joint limits, multi-start
  solving and sphere-obstacle avoidance;
- quintic minimum-jerk trajectory planning, jerk cost and pick-and-place
  sequencing;
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
# or
node --test --test-reporter=spec test/robotics.test.mjs test/invariants.test.mjs ui.test.cjs
```

Verified: 209 tests, 209 passing. The suite covers transform and quaternion
identities, forward kinematics against hand-checked poses, Jacobian and
manipulability properties, singularity detection, IK convergence and joint
limits, trajectory boundary conditions and jerk cost, collision
predicates, URDF round-trip validation, and page-level UI invariants.

## License

MIT. See `LICENSE`.
