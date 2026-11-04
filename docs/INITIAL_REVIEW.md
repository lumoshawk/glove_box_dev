# Initial source and development-environment review

Date: 2026-11-04. Scope: current checkout, source/configuration inspection, software
build and selected offline tests. No master startup, device launch, hardware bus
probe, enable, output actuation or motion was performed.

## Verified results

| Check | Result |
|---|---|
| `colcon list` | 14 packages discovered |
| Full `colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON` | All 14 built successfully |
| `bash scripts/check_offline.sh` | 51 passed, 1 failed, 2 hardware comparisons explicitly deselected |
| Selected C++ CTest targets across four packages | All 7 targets passed; 39 underlying gtest cases |
| Skill creator `quick_validate.py` | Valid frontmatter, name and scaffold checks |
| `bash -n scripts/check_offline.sh` | Passed |

The Python failure predates these documentation/helper additions:
`src/elmo_ethercat/test/test_elmo_ethercat.py::test_pp_test_defaults_move_100000_counts_and_back`
expects 100000, but `config/profiles/elmo_m1.yaml` sets `tests.pp_test.move_counts`
to 1000000. Neither value was changed. Confirm the intended commissioning profile
and then align the profile, regression test and older operating guides together.
The offline helper deliberately returns nonzero while this discrepancy remains.

C++ reproduction after building and sourcing Jazzy and this checkout:

```bash
colcon test --packages-select ethercat_interface ethercat_driver \
  ethercat_generic_slave ethercat_generic_cia402_drive \
  --ctest-args -R '^test_' --output-on-failure
colcon test-result --all
```

`colcon test-result` reports 46 results because it counts 7 CTest wrapper results
plus 39 gtest cases. This is not 46 independent test cases. Ament lint targets were
not included; no claim is made that unrestricted `colcon test` passes.
Build/test outputs remain in ignored `build/`, `install/`, `log/` directories.
Hardware behavior, external kernel/master patches and live topology remain unverified.

## Findings that affect the next development session

| Finding | Source evidence | Consequence / next action |
|---|---|---|
| Mixed historical topology | `glove_box_bringup/config/bringup.yaml`: both `mdx_y2` and `elmo_m1` record ring 5 | Reconcile physical inventory before launch. MDX uses alias 7 while Elmo uses alias zero, so duplicate recorded rings are inconsistent topology, not automatically identical effective addresses. |
| Presence filter is only a heuristic | `bus.py::present_entries`: count nonempty `ethercat slaves` lines, retain `ring < count`; scan errors retain everything | Does not validate identity, alias presence, missing middle slaves or duplicates; must not be treated as commissioning validation. |
| Gateways bypass presence filtering | `all.launch.py` filters `bus`, then appends all `gateways` | Absent gateways can still be configured and affect shared feedback. |
| “Read” clients may publish | MDX/EYOU/Elmo `read_test` constructs clients with command timers | Use subscriber-only bringup `read_all` for passive motor diagnostics; fix wording/API before using read tests alongside an active controller. |
| Large configured test motion | MDX `mdx_x2` PP requests 3600 degrees; Elmo PP requests 1000000 counts | Test names do not imply small motion. Review actual profiles and mechanical constraints before an authorized run. |
| Partial GCAN implementation | `gcan_ethercat/read_test.py`, `enable_test.py` exit unimplemented; raw watch and config generation exist | Do not claim working CANopen motor control based on package description/README. Decide whether to complete or explicitly retire it. |
| Manifest dependency mismatch | Device `package.xml` files depend on `ethercat_generic_plugins`; actual package is `ethercat_generic_slave` | This host builds, but clean dependency provisioning needs manifest correction. |
| EYOU scaling/rate remain open | `eyou_r*.yaml` scaling comments; shared 500 Hz versus standalone/client 1000 Hz | Verify output counts per revolution and timing before coordinated motion; no hardware verification done here. |
| Safety supervision is not established by snapshot checks | MDX `has_state()` checks NaN on cached values; bringup `read_all` checks finite fields without sample age | Add explicit freshness/ownership supervision and failure tests before building a robot application around these clients. |
| Documentation drift | Old root guides mention older profile names, smaller Elmo moves, different TUVEN delays and broader GCAN capability | Use the developer walkthrough and current source as the starting point; reconcile operational recipes when commissioning decisions are made. |

## Deliverables and maintenance

- `README.md`: onboarding entry point.
- `docs/DEVELOPMENT.md`: source map, data/configuration flow, units, commands,
  hardware staging and proposed development sequence.
- `AGENTS.md`: concise instructions for future agent work in this checkout.
- `scripts/check_offline.sh`: reviewed Python test selection without hardware probes.
- `skills/glove-box-development/SKILL.md`: versioned reusable development workflow;
  installed copy at `~/.codex/skills/glove-box-development/SKILL.md` for discovery.

The repository copy is the maintenance source for the skill. If it changes, copy
it to the installed skill directory and validate again. No runtime code, bus
configuration, motion profile or machine service was changed by this walkthrough.
