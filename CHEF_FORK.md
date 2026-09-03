# Chef fork notes

Chef-specific context for this fork. Kept in its own file rather than in
`README.md` so it never conflicts when syncing vendor changes.

## Branches

- `chef/humble-v2.1` — upstream `56a7927` plus these notes, nothing else.
- `chef/humble-v2.1-per-arm` — the above plus upstream's three
  `feature/independent-per-arm-control-humble` commits, cherry-picked
  unmodified. This is the branch chef builds.

## Independent per-arm control is an upstream *experimental* branch

`chef/humble-v2.1-per-arm` carries `832e6e5`, `a72c44e`, `daad9d1` from
upstream's `feature/independent-per-arm-control-humble`, which happens to sit
directly on `56a7927` — so they cherry-pick without modification.

Why chef needs it: the released driver exposes ONE 14-joint
`flexiv_arm_controller`, and `allow_partial_joints_goal` is not a workaround.
`ros2_control` implements a partial goal by holding position on the omitted
joints, so commanding one arm actively fights any trajectory the other arm is
following. The RDK itself supports independent per-arm control; only the driver
did not.

What the branch changes, in one line: an arm is now claimed as a whole joint
group, and `write()` evaluates each group's commands independently instead of
suppressing all motion when any joint's command is NaN.

Constraints worth knowing before designing against it:

- An arm must be claimed **whole** (all 7 joints) with a single interface type.
  6-of-7, or position+velocity on one arm, fails `prepare_command_mode_switch`.
- The RDK control mode is **global**: position on one arm and velocity on the
  other is fine, but not position on one and effort on the other. Effort
  requires every group claimed, or the unclaimed arm free-floats.
- An idle arm is **actively held** at its last position, not left uncommanded.
- `write()` now returns `ERROR` (rather than silently skipping) on mode
  mismatch, stream exceptions and GPIO failures, so the hardware component
  enters its error state instead of quietly doing nothing.

Being an experimental branch, expect this to be rebased or replaced upstream;
re-check it before any future sync.

## Upstream `chef/humble-v2.1` carries no chef changes

This branch is upstream `56a7927` ("Release/Flexiv ROS 2 Humble 2.1")
unmodified, and that is deliberate — do not "fix" the absence of a diff.

Chef's changes all live in `flexiv_description` on its own `chef/humble-v2.1`
branch (`armN` prefixes, `use_sn_prefix`, per-arm initial positions). On the
humble-v2.1 line the `<ros2_control>` block and the dual-arm macro are still in
`flexiv_description`, so there is nothing here to change.

The branch exists to pin the pairing: this driver release is the one that
matches RDK v2.1, which is what the Enlight's `v3E.1` software requires.

## Why not the consolidated `humble` line

Upstream `5f68f15` ("Adapt to consolidated flexiv_description") moved the
`<ros2_control>` block and the dual-arm xacro *into* this repo, under
`flexiv_hardware/urdf/` and `flexiv_hardware/ros2_control/`, and bumped the RDK
to `release/v2.2`.

That RDK bump is the problem: v2.x pairs one-to-one with the Enlight-only
`v3E.x` robot software line (v2.0 with v3E.0, v2.1 with v3E.1), and the RDK
refuses to connect on a mismatch. Chef's Enlight-LL runs v3E.1, so it needs
v2.1.

The v2.2 line also renamed two inline lookup maps into accessor functions
(`kJointGroupNames` -> `JointGroupNames()`), which is why an earlier chef branch
on that line needed a compatibility shim. On v2.1 the constant form is correct
and no shim is needed.

Forward-porting to v2.2 later means moving the `flexiv_description` changes into
`flexiv_hardware`'s copies of those xacros, and moving the robot's software to
`v3E.2`+ first.
