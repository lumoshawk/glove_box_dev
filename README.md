# Glove-box robot development workspace

ROS 2 **Jazzy**, `ros2_control`, and an IgH EtherCAT master integrate servo drives,
grippers, and tool-changer I/O. This repository currently provides device bringup,
raw interfaces, diagnostic clients, and individual device exercises. Robot geometry,
coordinated trajectories, and a complete task application are future integration work.

Start with [the developer walkthrough](docs/DEVELOPMENT.md): package ownership,
configuration flow, units, build/test commands, hardware boundaries, and next steps.
See [the initial verification record](docs/INITIAL_REVIEW.md) for what was actually checked.

```bash
# From this checkout, in a fresh terminal:
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
bash scripts/check_offline.sh
```

Building does not launch hardware. The walkthrough lists prerequisites and explains
why the checked-in bus layout must be reconciled before launching it.

Existing references:

- [Chinese workspace overview](workspace_brief_zh.md)
- [Usage guide](guide.md) and [command reference](cheatsheet.md): historical examples;
  current profile names and behavior are defined by source/configuration.
- [Device manuals and alias plan](ref/alias_def.md), with manuals/ESIs in `ref/`.
- [Reusable development skill](skills/glove-box-development/SKILL.md)
- [Agent working instructions](AGENTS.md)
