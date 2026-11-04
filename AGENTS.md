# Working in this repository

Read `docs/DEVELOPMENT.md` for architecture and `docs/INITIAL_REVIEW.md` for the
initial baseline. Inspect current code/configuration before relying on either.
The reusable workflow is `skills/glove-box-development/SKILL.md`.

- Keep device-specific PDOs, profiles and protocol behavior in the owning package;
  keep shared bus composition in `glove_box_bringup`.
- Run `bash scripts/check_offline.sh` after building and sourcing this checkout.
  Do not use unrestricted pytest discovery: runtime files named `*_test.py` are
  hardware commands, and two unit tests probe the live bus unless deselected.
- A build/test/documentation request does not authorize hardware activation.
  Launches can configure startup SDOs and write cyclic outputs. MDX/EYOU/Elmo
  `read_test` clients also publish commands. `read_all` in bringup subscribes only.
- Resolve topology from current evidence before hardware work; `skip_absent` is
  only a ring-count filter, not identity validation, and does not filter gateways.
- Preserve safe defaults and device profiles; do not guess scaling, aliases,
  limits or physical population. Track assumptions as unresolved.
- Use only one owner per EtherCAT master and one command authority per device.
  Enable, homing, tool unlock and motion require task-specific authorization.
- Report software validation separately from physical commissioning. Update the
  relevant documentation when interfaces, defaults or supported devices change.
