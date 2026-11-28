# Security policy

## Reporting a vulnerability

Use the private GitHub Security Advisory form at [Report a vulnerability](https://github.com/draarivpatel-ui/kinechain-6dof/security/advisories/new). Do not open a public issue, pull request or discussion with exploit details.

Never include a live secret, token, cookie, private key, victim hostname or proprietary payload in a report. Use synthetic values and redacted transcripts.

## Supported versions

| Version | Status |
| --- | --- |
| latest `main` | Supported |

Security fixes are applied to the latest state of the default branch.

## Scope

This policy covers the static Kinechain 6DOF robotics workbench, kinematics solvers, trajectory planners, URDF export and hash-state sharing.

In scope:

- A cross-site scripting or HTML injection path through robot state, waypoint data, hash links, imported text or generated output.
- A URDF, CSV or report export flaw that makes generated output execute markup, a formula or script when opened.
- A serialization or hash-codec flaw that bypasses validation, corrupts another robot state, or executes attacker-controlled code.
- A solver-validation failure that accepts non-finite or out-of-range input while presenting the result as safe or verified.
- A browser-storage or offline-runtime flaw that serves attacker-controlled application code across an update.

Out of scope:

- Browser vulnerabilities, operating system compromise or physical access to the machine.
- Damage caused by modified copies of the static application that do not come from this repository.
- Documented approximation limits of engineering or geometry models, unless a report shows a specific validation or output integrity failure.

## Coordinated disclosure

- Acknowledgment: within 3 business days.
- Initial triage: within 14 calendar days.
- Fix or mitigation target: within 90 calendar days for confirmed reports.
- Coordinated public disclosure: up to 90 days by default, adjusted by mutual agreement when exploitation or user risk requires a different date.
