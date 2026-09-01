# Chef fork notes

Chef-specific context for this fork. Kept in its own file rather than in
`README.md` so it never conflicts when syncing vendor changes.

## `chef/humble` tracks the vendor's *release* line, not `humble`

This is the thing to know before syncing.

`chef/humble` descends from the vendor's release branches
(`release/humble-v2.2`), not from the branch named `humble`. Upstream lands
release work on those branches first and merges back to `humble` later, so at
any moment `humble` can be *behind* the release line.

Concretely, `humble` sits at `56a7927` ("Release/Flexiv ROS 2 Humble 2.1"),
while `chef/humble` carries the next commit on the release line:

```
91e3d44  fix: forward use_sn_prefix to the ros2_control prefix computation (#1)   <- chef
5f68f15  Task/Adapt to consolidated flexiv_description (humble-v2.2) (#115)       <- upstream
56a7927  Release/Flexiv ROS 2 Humble 2.1 (#110)                                   <- humble stops here
```

`5f68f15` is upstream's own commit, taken unmodified. Chef needs it because it
**created** the files chef's ROS 2 bringup loads:

- `flexiv_hardware/urdf/flexiv.urdf.xacro`
- `flexiv_hardware/ros2_control/flexiv.ros2_control.xacro`
- `flexiv_hardware/ros2_control/flexiv_dual.ros2_control.xacro`
- `flexiv_hardware/ros2_control/flexiv_mico.ros2_control.xacro`

As of driver v2.2 the description package became geometry-only and the
`ros2_control` block moved into `flexiv_hardware`. `flexiv_dual.ros2_control.xacro`
is the Enlight-LL's 14-joint dual-arm block, so without this commit there is no
dual-arm `ros2_control` support at all. The same commit bumps the required
`flexiv_description` from `humble-v2.1` to `humble-v3.0`.

**The trap:** merging `origin/humble` into `chef/humble` looks like a sync but
is a regression — the mainline is behind, so it appears to *delete* those
xacros. Before syncing, check what the chef work actually descends from:

```sh
git merge-base --is-ancestor origin/humble chef/humble   # may pass and still be misleading
git log --oneline origin/humble..chef/humble             # what would be re-applied
git log --oneline chef/humble..origin/release/humble-v2.2  # the real upstream delta
```

Sync from the release branch chef is following, or from a `humble` that has
caught up — verify by confirming the four files above still exist afterwards.

## RDK version is pinned to the robot's software

`flexiv.humble.repos` pins `flexiv_rdk` to `v2.1`, not upstream's
`release/v2.2`. The RDK refuses to connect on a version mismatch, and v2.x
pairs one-to-one with the Enlight-only `v3E.x` robot software line
(v2.0 with v3E.0, v2.1 with v3E.1). Chef's Enlight-LL runs v3E.1.

Moving that pin means moving the robot's software too; see the comment in
`flexiv.humble.repos`.

The pin also requires the `rdk_compat.hpp` shim, because v2.2 turned two inline
lookup maps into accessor functions and the same source cannot name both
directly. **Merge the shim first** — this change alone will not build.
