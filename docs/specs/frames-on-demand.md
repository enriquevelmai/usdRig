# Frames on demand

A Hydra reader can ask a rig output for another frame by sampling it at a
time offset: `GetValue(k)` on a published prim's `primvars:points` (or its
normals, extent, driven transform, scalar or matrix primvars) answers the
frame at the displayed time + k. Onion skins, exports and pickers read rigged
geometry at other frames this way, through standard Hydra, without knowing
that a rig produced it.

## The rule

- **Never evaluates on the reader's thread.** A pull reads the context's
  frame table (`libs/rigExecImaging/frameTable.h`). Storm's sync threads may
  pull.
- **Answers, in order:**
  1. k == 0: the displayed value.
  2. An explicit motion sample at k.
  3. The frame table's entry for the displayed time + k.
- **"Not yet" is an empty value.** On a miss, `GetValue(k)` returns an empty
  `VtValue` and records the frame as demand. A typed read
  (`GetTypedValue(k)`) returns the displayed value, the only answer a typed
  reader can take.
- **Answered later.** On the owner (UI) thread,
  `RigExecImagingRegistry::ServiceFrameDemand` turns demand into entries:
  - A frame the frame cache holds becomes an entry.
  - Any other frame is admitted as an **exact output request** on the
    existing background pool (docs/specs/output-subscriptions.md). Its key
    is the rig, the exact time, and the whole captured generation — the
    compiled epoch as the program identity and the stage edit serial as the
    input revision — never the rounded key warming coalesces on, so a
    recompile or an edit is a different request. It names every output that
    epoch's export catalog can produce, one request per frame per rig — a
    frame whose explicit or warm job is already in flight is asked for nothing
    — and it runs ahead of every speculative warm job and stays admitted when
    warming is off. Its job publishes the complete pose, so an entry still
    requires complete compatible rig coverage. A rig whose frame cache is off,
    or whose epoch has no catalog, is asked for nothing. The rewarm lane
    stays the fallback when no exact request can be built or a request cap
    declines, and it keeps the frame on the lane without ever queueing a
    second job for a frame whose request is already in flight.
  - A `.rigexec` playback produces the frame as a **detached
    generation**: the binary plays at the frame without publishing, so
    the displayed generation is never published over and never restored.
  - When entries are added, the results scene index sends PrimsDirtied for
    the rig's value leaves (primvars, extent) and its driven subtrees'
    transforms. Readers that got "not yet" read again.
- **Nothing waits forever.** On the idle path (`OnIdle`, never during
  playback, and never while a background job for that frame is in flight), a
  frame still pending after `kFrameDemandFallbackAge` (40) services is
  evaluated on the owner thread, at most one per call (about 2 s at the
  plugin's tick). That frame belongs to a rig an exact request cannot answer:
  its frame cache is off, it has no baked program, or the request caps
  declined.
- **Previews hold.** While an interactive preview stands, demand waits.
  Entries made before the drag keep answering, so ghosts do not follow a
  drag.
- **Edits drop the table.** Any of these makes every entry stale:
  - a new stage;
  - a recompiled rig;
  - an advanced stage edit serial;
  - moved upstream values.

  The next SetTime or service call drops the entries and sends the same
  dirties.
- **Motion blur is unchanged.** `GetContributingSampleTimesForInterval`
  reports explicit motion samples only, now limited to the interval plus one
  bracketing sample on each side. Frames on demand are never reported as
  motion samples.

## Driving it

- The usdview plugin ticks every 50 ms while a rig is active
  (`_TickFrameDemand` in `plugin/rigExecUsdview/rigExecUsdview.py`). Each
  tick makes one `HasFrameDemand` check, then calls `OnIdle` (which services
  first and then sweeps), or only `ServiceFrameDemand` during playback.
- C surface (stage-scoped: the stage's `UsdUtilsStageCache` id comes first,
  see [multi-stage imaging](../multistage-imaging.md)):
  - `RigExecImaging_ServiceFrameDemandForStage` (the playback path, no
    fallback) returns the entries added, or -1 when inactive.
  - `RigExecImaging_HasFrameDemandForStage` returns 1, 0, or -1 when
    inactive.

- Without a Hydra chain (tools, headless tests): set the context's time,
  then read what it published with `RigExecImaging_GetPublished*ForStage`
  (points, scalar primvar, transform revision) or `python/rigexec/imaging.py`.
  These read the displayed generation only and never evaluate.

## Limits

- A rig an exact output request cannot answer reaches other frames only
  through the idle fallback above, one frame per idle tick:
  - a rig with no baked program, or with no export catalog for its current
    compile epoch;
  - `RIGEXEC_FRAME_CACHE=off` (no servable pose).
- GPU-deformed points (`RIGEXEC_GPU_DEFORM=shader`) have no CPU points at any
  frame, so their offset reads stay empty.
- The table keeps 128 frames (farthest from the playhead dropped first) and
  256 outstanding demands.

Tests: `tests/testRigExecFrameDemand.cpp`.
