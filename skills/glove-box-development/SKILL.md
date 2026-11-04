---
name: glove-box-development
description: Develop and diagnose the glove_box_dev ROS 2 Jazzy EtherCAT workspace, including device profiles, shared bringup, GPIO clients and tool-changer logic. Use for this repository's onboarding, extensions, tests and hardware staging.
---

# Glove-box development

Find the active checkout from the user's workspace, not a fixed worktree path.
Read its `AGENTS.md`, `docs/DEVELOPMENT.md` and `docs/INITIAL_REVIEW.md`; verify
current source before using recorded results. The repository maintains this
skill at `skills/glove-box-development/SKILL.md`.

## Route the change

- Device identity/PDO/type changes: owning package's `config/drives/`, ESI and
  `system.py`. Per-device addressing, mechanics and tests: `config/profiles/`.
  Gateway membership: `config/gateways/`.
- Shared topology/rate: `glove_box_bringup/config/bringup.yaml`, `bus.py`,
  `launch/all.launch.py`. Preserve the `build(params, out_dir, positions)` contract
  and one controller per package.
- Protocol/state transitions: device `*_client.py`; sequencing: test commands or
  `tuven_ethercat/tool_logic.py`. Build a reusable application above these boundaries.
- Master timing/PDO internals: `ethercat_ros2_driver_mod-main`. This is a modified
  driver; preserve DC reference selection and synchronization during SAFEOP.

## Validate without hardware first

Use a fresh Jazzy shell, build the active checkout, source its install and run
`bash scripts/check_offline.sh`. For driver changes also build/run the relevant
C++ tests. Inspect failures and distinguish pre-existing failures from regressions.
Do not treat a successful build as device acceptance. New config/ESI assets must
be included in `setup.py`, not just present in source.

## Hardware-specific decisions

Check authorization for the actual requested operation. Do not launch hardware
as part of ordinary documentation, build or offline-test work.

Before commissioning, reconcile each configured alias/position and SII identity
with current topology. Nonzero alias positions are offsets; only alias zero uses
absolute ring overrides. The current `skip_absent` count heuristic fails open on
scan errors and ignores gateways. It does not verify identity or detect a missing
middle device. Never use it as proof that the selected hardware is correct.

Use subscriber-only `glove_box_bringup read_all` or topic inspection on an already
running approved system for passive feedback. MDX/EYOU/Elmo `read_test` constructs
clients that publish defaults. Launch itself can write startup SDOs and PDOs.
Enable tests energize drives; PP/PV/CSP, homing and tool-lock sequences actuate
hardware even when named tests. Avoid concurrent command clients.

Keep disabled feedback verification separate from enable and motion. Check fresh
finite telemetry, identity, mode, limits, scaling and fault/STO state for the actual
devices involved. Do not infer heartbeat protection from finite cached feedback,
or physical lock confirmation from an issued output command.

Record modified files, reproducible software checks, unresolved assumptions and
hardware results separately. Keep guides aligned with the implementation; old
profile names and README claims are not authoritative.
