#!/usr/bin/env bash
# Run only the reviewed software unit tests; never launch a ROS node or query a bus.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -f install/setup.bash ]]; then
  echo 'Build this checkout first; see docs/DEVELOPMENT.md.' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$workspace_dir/install/setup.bash"
python3 -m pytest \
  src/dh_ethercat/test \
  src/tuven_ethercat/test \
  src/elmo_ethercat/test \
  src/glove_box_bringup/test \
  --deselect=src/dh_ethercat/test/test_dh_ethercat.py::test_esi_matches_the_slave_on_the_bus \
  --deselect=src/tuven_ethercat/test/test_tuven_ethercat.py::test_drive_matches_the_box_on_the_bus \
  -q
