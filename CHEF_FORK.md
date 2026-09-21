# Chef fork notes

Chef-specific context for this fork. Kept in its own file rather than in
`README.md` so it never conflicts when syncing vendor changes.

## What this branch is

`chef/humble-v2.1` is the branch chef builds and the one `ChefAutonomy`'s
submodule tracks. It is upstream `56a7927` ("Release/Flexiv ROS 2 Humble 2.1")
plus upstream's three `feature/independent-per-arm-control-humble` commits,
chef's control-mode-loss handling, and these notes.

The branch exists to pin a pairing: this driver release is the one that matches
RDK v2.1, which is what the Enlight's `v3E.1` software requires. See "Why not
the consolidated `humble` line".

The description-side changes are not here. `armN` prefixes, `use_sn_prefix` and
per-arm initial positions all live in `flexiv_description` on its own
`chef/humble-v2.1` branch. On the humble-v2.1 line the `<ros2_control>` block
and the dual-arm macro are still that repo's, so there is nothing here to carry
them.

## Independent per-arm control comes from an upstream *experimental* branch

`57bddc5`, `bec3202` and `a2856a9` are upstream's `832e6e5`, `a72c44e` and
`daad9d1` from `feature/independent-per-arm-control-humble`, cherry-picked
unmodified — that branch sits directly on `56a7927`, so they apply as-is.

Why chef needs them: the released driver exposes ONE 14-joint
`flexiv_arm_controller`, and `allow_partial_joints_goal` is not a workaround.
`ros2_control` implements a partial goal by holding position on the omitted
joints, so commanding one arm actively fights any trajectory the other arm is
following. The RDK itself supports independent per-arm control; only the driver
did not.

What they change, in one line: an arm is claimed as a whole joint group, and
`write()` evaluates each group's commands independently instead of suppressing
all motion when any joint's command is NaN.

Constraints worth knowing before designing against it:

- An arm must be claimed **whole** (all 7 joints) with a single interface type.
  6-of-7, or position+velocity on one arm, fails `prepare_command_mode_switch`.
- The RDK control mode is **global**: position on one arm and velocity on the
  other is fine, but not position on one and effort on the other. Effort
  requires every group claimed, or the unclaimed arm free-floats.
- An idle arm is **actively held** at its last position, not left uncommanded.
  Upstream gates streaming on a group being actively commanded, which computes
  the hold targets and then discards them; chef removed that gate, so the hold
  is real here. See "Chef keeps the component alive across control-mode loss".

Being an experimental branch, expect this to be rebased or replaced upstream;
re-check it before any future sync.

## Chef keeps the component alive across control-mode loss

Upstream's `write()` returns `ERROR` on any mode mismatch, stream exception or
GPIO failure. That is terminal: `ros2_control` deactivates the component and
`on_error()` drops every claim. The controllers stay `active` over the corpse
and go on reporting "Goal reached, success!" while the arm does not move —
seen in on-hardware testing with both arms >120 deg from their commanded pose
while the stack reported success.

So `write()` here streams every cycle whose targets are valid, tries to
re-enter a lost mode (bounded, since the robot can leave a mode faster than it
can be put back), and tolerates a bounded run of consecutive stream failures.
Measured on a dual-arm cell across four runs, cycles completed per run: 10
stock, 54 with all three changes.

This makes the component much harder to kill but does not fix the underlying
defect — a controller must never report success when its hardware is
unavailable. Humble's `controller_manager` does not deactivate controllers when
a component errors, so that needs a chef-side watchdog or an upstream change.

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
