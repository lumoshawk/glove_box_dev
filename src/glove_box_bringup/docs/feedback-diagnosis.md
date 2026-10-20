# All-motor NaN feedback: 2026-10-16

Aliases 6..10 correctly attach to ring positions 4..8. All five motors exchange process data (domain WKC 15/15), but fail EtherCAT OP with AL 0x002D. The ROS driver intentionally keeps processData gated until all configured motors are OP, so state interfaces remain NaN. Do not substitute zeros for unavailable measurements or bypass the command gate.

The current application requests alias 6 / offset 0 (mdx_x2, physical ring 4) as DC reference. Its startup log confirms that the API call succeeded. However, live `ethercat master` still reported reference slave 0. A rescan while configuration was present then reported `DC reference clock config (6-0) has no slave attached!` and selected no reference. Configurations were attached again after the scan.

The running kernel module is distro IgH 1.6.8 (srcversion 1035CB8F0E1D853CF937AD9) for kernel 7.0.0-38-generic. The local source/tools are 1.6.12-16-g2140c102. These observations demonstrate that API success alone did not apply the requested reference; version difference alone does not establish ABI incompatibility.

An isolated build is staged at `log/ethercat-reference-fix`, based on local commit 2140c102. This source attaches configurations before calculating DC after a scan. The accompanying patch also recalculates DC when an already-attached reference is selected, instead of leaving the idle-scan reference active until another scan. Both master and generic-device modules must be rebuilt for the running kernel and loaded together. No system module has been replaced.

Verification still required after an approved module replacement: `ethercat master` must identify slave 4 as reference, downstream propagation delays must be plausible, all five drives must reach OP, and read_all must show finite feedback with control words kept zero. Do not reset the MDX vendor fault 0xFF3F or enable any motor as part of that check.

The DH gateway remains excluded from application PDO configuration because its live CoE map differs from its profile. Its reported link/topology inconsistency remains a separate unresolved issue. No claim is made that the reference fix alone will resolve all hardware issues.

The failed launch was stopped. Master is idle and all slaves are PREOP while the candidate modules are built and reviewed.
