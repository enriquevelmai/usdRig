#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/outputInspectorModel.py: what the
output inspector lists, the exact frame it demands, the dispositions and
values it shows, and what a teardown releases.

Everything here runs with neither Qt nor usdview -- the exec-stack panel in
plugin/rigExecUsdview/execStackUI.py drives this same model, so the frame
semantics checked here are the ones an artist sees. The two contracts with
the sharpest edges:

  * selecting a mover mid-stack and demanding its own output at frame 2.5
    answers THAT pass, not the rig's final pose, and never moves the frame
    the viewer is showing. The stand-in host answers DisplayedTime() and
    records every other attribute the model reaches for, so "the inspector
    does not touch the playhead" is an observation rather than a reading of
    the source;
  * closing the model -- what the panel does on stage replacement and on
    close -- ends the demand: no line, no subscription, no service.

Answers are compared against the live evaluation of the same rig and against
the control values read straight off the stage, so a wrong frame is a wrong
number rather than one path replaying its own.

Usage: test_output_inspector_model.py [<generated schema resources dir>]
"""
import math
import sys
import time

# Sibling modules: this script's own directory is sys.path[0]. The bootstrap
# must run before the first pxr import so pxr resolves from the configured
# USD install, and the build tree must supply _rigexec and rigexec.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()
from test_rigexec_python import _setup_environment  # noqa: E402

_setup_environment()

from pxr import Sdf, Usd, UsdGeom  # noqa: E402
import rigexec  # noqa: E402
import outputInspectorModel as insp  # noqa: E402

BODY_PATH = "/Model/Geom/Body"
POINTS_TARGET = BODY_PATH + ".points"
EMPTY_PATH = "/Model/Geom/Empty"
EMPTY_TARGET = EMPTY_PATH + ".points"
DATA_PATH = "/Model/Data"
SCALAR_TARGET = DATA_PATH + ".amount"
CONTROL_A = "/Rig/Controls/Ctrl"
REST_POINTS = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 2.0, 0.0)]

# The two controls this rig drives: one moves the body along Z, the other
# along Y. Their authored defaults, plus samples that make the default
# frame, frame zero, a subframe and a negative frame answer differently.
DEFAULT_TZ = 2.0
DEFAULT_TY = 0.0
TZ_SAMPLES = {-6.0: -1.0, 0.0: 0.0, 10.0: 5.0}
TY_SAMPLES = {0.0: 0.0, 10.0: 3.0}

# The frame the stand-in host is showing: past both splines' last knots, so
# it answers the held values -- and it is a frame the rig is never asked for
# anywhere but here.
SHOWN_FRAME = 12.0


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, eps=1e-6):
    return abs(a - b) < eps


class _Host(object):
    """A usdview panel stand-in that answers the frame it is showing.

    `DisplayedTime` is the only entry point the model is allowed. Every other
    attribute is recorded and refused with the AttributeError a real host
    raises for a name it does not carry, so a model that reached for a
    playhead setter, a stage, or an evaluator is caught by the checks that
    follow each section rather than by inspection.
    """

    def __init__(self, frame=SHOWN_FRAME):
        self.frame = frame
        self.reached = []

    def DisplayedTime(self):
        return self.frame

    def __getattr__(self, name):
        self.reached.append(name)
        raise AttributeError(name)


def _author_rig(rigexec):
    """One Points prim driven by two matrix movers, and one float mover."""
    stage = Usd.Stage.CreateInMemory()
    body = UsdGeom.Points.Define(stage, BODY_PATH)
    body.CreatePointsAttr().Set(REST_POINTS)

    data = stage.DefinePrim(DATA_PATH, "Xform")
    data.CreateAttribute("amount", Sdf.ValueTypeNames.Float).Set(3.0)

    builder = rigexec.Builder.create(stage, "/Rig")
    ctrlZ = builder.add_control("Ctrl")
    ctrlY = builder.add_control("CtrlLift")
    tz = stage.GetPrimAtPath(ctrlZ.path).GetAttribute("avars:tz")
    ty = stage.GetPrimAtPath(ctrlY.path).GetAttribute("avars:ty")
    _Check(tz.IsValid() and ty.IsValid(), "the controls have no avar attrs")
    tz.Set(DEFAULT_TZ)
    ty.Set(DEFAULT_TY)
    for frame, value in sorted(TZ_SAMPLES.items()):
        tz.Set(value, Usd.TimeCode(frame))
    for frame, value in sorted(TY_SAMPLES.items()):
        ty.Set(value, Usd.TimeCode(frame))

    chain = builder.new_mover_chain("chain")
    # The chain applies in reverse add order, so `lift` runs first: its own
    # exported pass is an intermediate the later `move` pass still revises.
    lift = chain.add_matrix_mover("lift", ctrlY.path, None, POINTS_TARGET)
    move = chain.add_matrix_mover("move", ctrlZ.path, None, POINTS_TARGET)
    chain.add_float_math_mover("double", "multiply", 2.0, SCALAR_TARGET)

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    return stage, rig, tz, ty, str(lift.path), str(move.path)


def _author_empty_rig(rigexec):
    """One matrix mover over a Points prim authored with no points at all.

    A rig that publishes zero points is a valid answer rather than a failure:
    the extraction packs a zero-length lease for it.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Points.Define(stage, EMPTY_PATH).CreatePointsAttr().Set([])

    builder = rigexec.Builder.create(stage, "/Rig")
    ctrl = builder.add_control("Ctrl")
    tz = stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:tz")
    _Check(tz.IsValid(), "the control has no avar attrs")
    tz.Set(DEFAULT_TZ)
    chain = builder.new_mover_chain("chain")
    chain.add_matrix_mover("move", ctrl.path, None, EMPTY_TARGET)

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    return stage, rig


def _Sample(attr, frame):
    """The control's authored value at one exact frame, off the stage."""
    if frame is None:
        return attr.Get(Usd.TimeCode.Default())
    return attr.Get(Usd.TimeCode(frame))


def _RowsByKey(rows):
    return {row.semantic_key: row for row in rows}


def _Row(rows, binding):
    return _RowsByKey(rows).get(binding.semantic_key)


def _Statuses(rows):
    return [(row.target, row.computation, row.phase, row.frameText,
             row.status, row.reason) for row in rows]


def _PointRows(rows, binding):
    """A points answer as plain tuples."""
    value = _Row(rows, binding).packet.value
    _Check(value is not None, "no payload for %s" % binding.target)
    return [value.point(i) for i in range(value.elements)]


def _DrainReady(model, frame, count):
    """Manual mode: run the queued work, then insist every line answers.

    A chained demand (the answer, then its geometry, then its control)
    becomes runnable one link at a time, so the queue is run until every
    line has answered rather than once.
    """
    rows = model.Rows()
    for _ in range(60):
        model.Drain()
        rows = model.Rows()
        if (len(rows) == count
                and all(row.status == insp.READY for row in rows)):
            break
    _Check(len(rows) == count, "expected %d lines, got %r"
           % (count, _Statuses(rows)))
    _Check(all(row.status == insp.READY for row in rows),
           "a demanded line did not answer: %r" % (_Statuses(rows),))
    for row in rows:
        _Check(row.time == frame,
               "a line answers %r, not the demanded %r"
               % (row.time, frame))
    return rows


def _DrainStatus(model, frame, count, status):
    """Manual mode: run what is queued, then insist every line reports `status`.

    The stand-in for a panel tick that lands on a refusal rather than an
    answer: the frame stays demanded, the queue drains, and the line says why
    it has no value.
    """
    rows = model.Rows()
    for _ in range(60):
        model.Drain()
        rows = model.Rows()
        if (len(rows) == count
                and all(row.status == status for row in rows)):
            break
    _Check(len(rows) == count, "expected %d lines, got %r"
           % (count, _Statuses(rows)))
    _Check(all(row.status == status for row in rows),
           "a demanded line did not report %r: %r" % (status,
                                                      _Statuses(rows)))
    for row in rows:
        _Check(row.time == frame,
               "a line answers %r, not the demanded %r" % (row.time, frame))
    return rows


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    rigexec.load_schema_plugin(plugin_dir)
    stage, rig, tzAttr, tyAttr, liftPath, movePath = _author_rig(rigexec)
    # Inspecting compiles, samples and publishes; it never authors.
    authored = stage.GetSessionLayer().ExportToString()

    host = _Host()
    model = insp.OutputInspectorModel(host=host, workers=0)
    _Check(not model.attached, "a fresh model claims a rig it does not have")
    _Check(model.stats() is None and model.Poll() is False,
           "an unattached model polled something")

    # -- attaching lists the exported handles ----------------------------
    bindings = model.Attach(rig)
    _Check(model.attached and bindings, "a compiled rig exported nothing")
    kinds = {}
    for binding in bindings:
        kinds.setdefault(binding.kind, []).append(binding)
    for kind in ("points", "scalar", "transform"):
        _Check(kinds.get(kind), "no exported %s output" % kind)
    stats = model.stats()
    _Check(stats["subscriptions"] == 0 and stats["frames_demanded"] == 0,
           stats)
    _Check(model.Selected() == [] and model.Rows() == [],
           "an attach demanded something on its own")
    _Check("Nothing selected" in model.StatusText(), model.StatusText())

    # -- a prim, an attribute, and a mover each name their own outputs ----
    bodyHandles = model.HandlesFor(BODY_PATH)
    _Check([h.target for h in bodyHandles] == [POINTS_TARGET] * 4,
           [h.target for h in bodyHandles])
    _Check(model.HandlesFor(POINTS_TARGET) == bodyHandles,
           "the attribute target and its prim name different handles")
    # base, final, and one export per mover's own pass: the intermediate is
    # in the list, spelled by the phase the mover produced it at -- which is
    # the mover row's own path in the execution stack above.
    phases = [h.phase for h in bodyHandles]
    _Check("base" in phases and "final" in phases, phases)
    order = rig.mover_order
    if callable(order):
        order = order()
    pointEntries = [entry for entry in order
                    if POINTS_TARGET in (entry.get("targets") or ())]
    pointEntries.sort(key=lambda entry: entry["ordinal"])
    pointMovers = [entry["path"] for entry in pointEntries]
    _Check(len(pointMovers) == 2, pointMovers)
    midPath, lastPath = pointMovers[0], pointMovers[-1]
    _Check(sorted(pointMovers) == sorted([liftPath, movePath]), pointMovers)
    midHandles = model.HandlesFor(midPath)
    lastHandles = model.HandlesFor(lastPath)
    _Check(len(midHandles) == 1 and midHandles[0].phase == midPath,
           [h.phase for h in midHandles])
    _Check(len(lastHandles) == 1 and lastHandles[0].phase == lastPath,
           [h.phase for h in lastHandles])
    _Check(midHandles[0].semantic_key != lastHandles[0].semantic_key,
           "two movers share one exported output")
    _Check(model.HandlesFor(CONTROL_A),
           "the driven control's transform is not exported")
    _Check(not model.HandlesFor("/Model/NoSuchPrim"),
           "a prim this rig does not have names a handle")
    _Check(not model.HandlesFor(""), "an empty path names a handle")

    # -- an intermediate demanded at an exact frame, while owed -----------
    mid = midHandles[0]
    last = lastHandles[0]
    final = [h for h in bodyHandles if h.phase == "final"][0]
    base = [h for h in bodyHandles if h.phase == "base"][0]
    _Check(model.Select([mid]) == [mid.semantic_key], model.Selected())
    _Check(model.DemandFrame(2.5) == 2.5, "the demand did not land on 2.5")
    _Check(model.Frame() == 2.5 and model.FrameText() == "2.5",
           model.FrameText())
    _Check(model.Poll(), "the first poll reported no new lines")
    rows = model.Rows()
    _Check(len(rows) == 1, _Statuses(rows))
    _Check(rows[0].status == insp.PENDING,
           "a zero-worker host ran the work itself: %r" % (_Statuses(rows),))
    _Check(model.HasWork(), "an owed line did not report work")
    _Check("waiting" in model.StatusText(), model.StatusText())
    stats = model.stats()
    _Check(stats["subscriptions"] == 1, stats)
    _Check(stats["dispatches"] >= 1, stats)
    # One live demand group: the one output at the one frame now standing.
    _Check(stats["frames_demanded"] == 1, stats)

    rows = _DrainReady(model, 2.5, 1)
    lifted = _PointRows(rows, mid)
    ty = _Sample(tyAttr, 2.5)
    tz = _Sample(tzAttr, 2.5)
    _Check(_Close(ty, 0.75) and _Close(tz, 1.25), (ty, tz))
    # The mid-stack pass has exactly one of the two movers on it: one axis
    # carries its control's offset and the other is still the rest pose.
    movedY = [point[1] - rest[1] for point, rest in zip(lifted, REST_POINTS)]
    movedZ = [point[2] - rest[2] for point, rest in zip(lifted, REST_POINTS)]
    firstIsY = (_Close(movedY[0], ty) and _Close(movedY[1], ty)
                and _Close(movedZ[0], 0.0))
    firstIsZ = (_Close(movedZ[0], tz) and _Close(movedZ[1], tz)
                and _Close(movedY[0], 0.0))
    _Check(firstIsY or firstIsZ,
           ("the intermediate carries neither single pass", lifted, ty, tz))
    for index, rest in enumerate(REST_POINTS):
        _Check(_Close(lifted[index][0], rest[0]), (index, lifted[index]))
    row = _Row(rows, mid)
    _Check(row.time == 2.5 and row.kind == "points", (row.time, row.kind))
    _Check(row.label == "value" and row.color, row.label)
    _Check("input" in row.GenerationText(), row.GenerationText())

    # -- the same frame, the last pass, and the base pass ----------------
    model.Select([mid, last, final, base])
    rows = _DrainReady(model, 2.5, 4)
    moved = _PointRows(rows, last)
    finals = _PointRows(rows, final)
    rested = _PointRows(rows, base)
    # The intermediate is not the answer the viewer sees: the later pass has
    # revised it, and the inspector shows each on its own line.
    _Check(not _Close(lifted[1][1], finals[1][1])
           or not _Close(lifted[1][2], finals[1][2]),
           "the intermediate reads as the final pose")
    for index in range(len(REST_POINTS)):
        _Check(_Close(finals[index][2], REST_POINTS[index][2] + tz)
               and _Close(finals[index][1], REST_POINTS[index][1] + ty),
               (index, finals[index]))
        _Check(_Close(moved[index][1], finals[index][1])
               and _Close(moved[index][2], finals[index][2]),
               ("the last pass is not the final pose", index, moved[index]))
        _Check(_Close(rested[index][0], REST_POINTS[index][0])
               and _Close(rested[index][1], REST_POINTS[index][1])
               and _Close(rested[index][2], REST_POINTS[index][2]),
               ("base carries a revised pass", index, rested[index]))
    # The live path answers the same frame with the same numbers.
    live = [tuple(p) for p in rig.evaluate(2.5).moved_property(POINTS_TARGET)]
    for index in range(len(live)):
        _Check(_Close(finals[index][2], live[index][2]),
               (index, finals[index], live[index]))

    # -- the viewer did not move ----------------------------------------
    _Check(host.frame == SHOWN_FRAME, "the inspector moved the playhead")
    _Check(host.reached == [],
           "the inspector reached for the viewer: %r" % (host.reached,))

    # -- every exported output, at the frame the viewer shows ------------
    _Check(model.Select(model.Bindings())
           == [b.semantic_key for b in bindings],
           "selecting every handle dropped one")
    model.DemandDisplayedFrame()
    _Check(model.Frame() == SHOWN_FRAME, model.Frame())
    rows = _DrainReady(model, SHOWN_FRAME, len(bindings))
    ty = _Sample(tyAttr, SHOWN_FRAME)
    tz = _Sample(tzAttr, SHOWN_FRAME)
    _Check(_Close(ty, TY_SAMPLES[10.0]) and _Close(tz, TZ_SAMPLES[10.0]),
           (ty, tz))
    scalarBindings = [b for b in bindings if b.kind == "scalar"]
    amount = _Row(rows, [b for b in scalarBindings
                         if b.phase == "final"][0]).packet.value.scalar
    # The fixture authored 3.0 and one multiply-by-2 mover.
    _Check(_Close(amount, 6.0), amount)
    amountBase = _Row(rows, [b for b in scalarBindings
                             if b.phase == "base"][0]).packet.value.scalar
    _Check(_Close(amountBase, 3.0), amountBase)
    controlBinding = model.HandlesFor(CONTROL_A)[0]
    matrix = _Row(rows, controlBinding).packet.value.matrix
    _Check(_Close(matrix[12], 0.0) and _Close(matrix[13], 0.0)
           and _Close(matrix[14], tz), matrix)
    _Check("t (%.4f, %.4f, %.4f)" % (0.0, 0.0, tz)
           in _Row(rows, controlBinding).ValueText(),
           _Row(rows, controlBinding).ValueText())
    finals = _PointRows(rows, final)
    for index in range(len(REST_POINTS)):
        _Check(_Close(finals[index][1], REST_POINTS[index][1] + ty)
               and _Close(finals[index][2], REST_POINTS[index][2] + tz),
               (SHOWN_FRAME, index, finals[index]))
    # A payload with points in it is spelled as its count and its first point.
    _Check(_Row(rows, final).ValueText()
           == "%d pts, first (%.4f, %.4f, %.4f)"
           % (len(finals), finals[0][0], finals[0][1], finals[0][2]),
           _Row(rows, final).ValueText())
    # Reading the shown frame demanded it; it did not move it.
    _Check(host.frame == SHOWN_FRAME, "reading the frame moved the frame")
    _Check(host.reached == [], host.reached)

    # -- the default frame, zero, a subframe and a negative frame --------
    for frame, wantY, wantZ in ((None, DEFAULT_TY, DEFAULT_TZ),
                                (0.0, 0.0, 0.0),
                                (2.5, 0.75, 1.25),
                                (-3.0, 0.0, -0.5)):
        model.Select([final])
        model.DemandFrame(frame)
        rows = _DrainReady(model, frame, 1)
        finals = _PointRows(rows, final)
        _Check(_Close(finals[1][1], REST_POINTS[1][1] + wantY)
               and _Close(finals[1][2], REST_POINTS[1][2] + wantZ),
               (frame, finals[1], wantY, wantZ))
        row = _Row(rows, final)
        _Check(row.time == frame
               and row.frameText == ("default" if frame is None
                                     else "%g" % frame),
               (frame, row.time, row.frameText))
        # A demand landing on the frame the subscription already answers is
        # served without the packet naming it; the row carries the frame that
        # was asked for either way. Off that frame the packet says so itself.
        if frame not in (None, 0.0):
            _Check(row.frameNamed, (frame, row.frameNamed))
    # Each exact frame is answered as itself: the fractional and negative
    # codes carry their own interpolated control values, and the one live
    # demand group is the frame now standing -- replacing it retires the
    # previous frame's demand rather than accumulating one.
    _Check(model.stats()["frames_demanded"] == 1, model.stats())
    _Check(model.FrameText() == "-3", model.FrameText())
    # Signed zero is frame zero, spelled as frame zero.
    model.DemandFrame(-0.0)
    _Check(math.copysign(1.0, model.Frame()) > 0.0, model.Frame())
    rows = _DrainReady(model, 0.0, 1)
    _Check(_Close(_PointRows(rows, final)[1][2], 0.0),
           _Row(rows, final).ValueText())

    # -- a frame the model will not demand leaves the standing one alone --
    model.DemandFrame(4.0)
    standing = model.stats()["frames_demanded"]
    for bad in (float("nan"), float("inf"), float("-inf"), True, "7", [],
                {}):
        try:
            model.DemandFrame(bad)
        except ValueError:
            continue
        raise AssertionError("%r was accepted as a frame" % (bad,))
    _Check(model.Frame() == 4.0, "a refused frame replaced the standing one")
    _Check(model.stats()["frames_demanded"] == standing,
           "a refused frame demanded work: %r" % (model.stats(),))
    _Check(host.reached == [], host.reached)

    # -- an empty selection demands nothing ------------------------------
    _Check(model.Select([]) == [], model.Selected())
    _Check(model.Poll(), "clearing the selection did not change the lines")
    _Check(model.Rows() == [], model.Rows())
    _Check(not model.HasWork(), "an empty selection still owes work")
    _Check("Nothing selected" in model.StatusText(), model.StatusText())

    # -- teardown: stage replacement and panel close release everything ---
    model.Select(model.Bindings())
    model.DemandFrame(2.5)
    _DrainReady(model, 2.5, len(model.Bindings()))
    before = model.stats()
    _Check(before["subscriptions"] == 1 and before["frames_demanded"] == 1,
           before)
    _Check(before["pinned_payloads"] > 0 and before["pinned_bytes"] > 0,
           before)
    # The panel never holds the service itself; the test keeps the handle
    # only to read what the teardown did to its bookkeeping.
    service = model._service
    model.Close()
    model.Close()
    after = service.stats()
    _Check(after["subscriptions"] == 0 and after["demands_in_flight"] == 0
           and after["pinned_payloads"] == 0 and after["pinned_bytes"] == 0,
           after)
    _Check(not model.attached, "a closed model is still attached")
    _Check(model.Rows() == [] and model.Selected() == []
           and model.Bindings() == [], "a closed model kept state")
    _Check(model.stats() is None, "a closed model still has a service")
    _Check(model.Poll() is False, "a closed model polled something")
    _Check(not model.HasWork(), "a closed model owes work")
    _Check("No rig attached" in model.StatusText(), model.StatusText())
    _Check(host.reached == [], host.reached)

    # Reopening the panel re-attaches and answers again.
    _Check(model.Attach(rig), "a reopened panel found no exported outputs")
    model.Select([mid])
    model.DemandFrame(2.5)
    rows = _DrainReady(model, 2.5, 1)
    reopened = _PointRows(rows, mid)
    _Check(not _Close(reopened[1][1], REST_POINTS[1][1])
           or not _Close(reopened[1][2], REST_POINTS[1][2]), reopened)

    # -- a recompile: the retired demand says so, re-attach asks anew ----
    extra = rigexec.Builder.create(stage, "/Rig").new_mover_chain("second")
    extra.add_float_math_mover("again", "add", 1.0, SCALAR_TARGET)
    rig.compile()
    heldRow = _Row(rows, mid)
    heldText = heldRow.ValueText()
    # These handles name the retired compile. The pump refuses that frame
    # rather than sampling the new program into the old compile's job, so the
    # line says its epoch is stale, and the answer the panel already showed
    # keeps the bytes it had.
    staleRows = _DrainStatus(model, 2.5, 1, insp.INVALIDATED)
    stale = _Row(staleRows, mid)
    _Check("epoch" in stale.reason, _Statuses(staleRows))
    _Check(stale.ValueText() != heldText,
           "the retired answer kept reading as the live one")
    _Check(heldRow.ValueText() == heldText,
           "a held answer moved under the panel")
    # Re-attaching -- what the panel does on Refresh -- closes that service
    # and asks with the new compile's handles, so the chain the recompile
    # added lands in the answer.
    amountBefore = amount
    model.Attach(rig)
    _Check(all(binding is not mid for binding in model.Bindings()),
           "a re-attach kept a handle from the retired compile")
    model.SelectFor(DATA_PATH)
    model.DemandFrame(2.0)
    rows = _DrainReady(model, 2.0, len(model.HandlesFor(DATA_PATH)))
    liveAmount = float(rig.evaluate(2.0).moved_property(SCALAR_TARGET))
    answered = [_Row(rows, b).packet.value.scalar
                for b in model.HandlesFor(DATA_PATH) if b.phase == "final"]
    _Check(_Close(answered[0], liveAmount), (answered, liveAmount))
    _Check(not _Close(answered[0], amountBefore),
           "the recompiled chain is missing from the inspected answer")

    # -- a real pool answers the same frame, read on the owner's tick -----
    pooled = insp.OutputInspectorModel(host=host, workers=2)
    _Check(pooled.Attach(rig), "a pooled inspector found no outputs")
    _Check(pooled.Select([mid, final]), "no handle for the mover's pass")
    pooled.DemandFrame(7.0)
    deadline = time.monotonic() + 10.0
    while True:
        pooled.Poll()
        if not pooled.HasWork():
            break
        _Check(time.monotonic() < deadline,
               "the pool never answered: %r" % (_Statuses(pooled.Rows()),))
        time.sleep(0.005)
    rows = pooled.Rows()
    _Check(len(rows) == 2, _Statuses(rows))
    _Check(all(row.status == insp.READY for row in rows),
           _Statuses(rows))
    wantY = _Sample(tyAttr, 7.0)
    wantZ = _Sample(tzAttr, 7.0)
    # Both controls sit off their rest values at this frame, so the
    # mid-stack line and the answer line cannot coincide by accident.
    _Check(not _Close(wantY, 0.0) and not _Close(wantZ, 0.0),
           (wantY, wantZ))
    finals = _PointRows(rows, final)
    lifted = _PointRows(rows, mid)
    for index, rest in enumerate(REST_POINTS):
        _Check(_Close(finals[index][1], rest[1] + wantY)
               and _Close(finals[index][2], rest[2] + wantZ),
               (7.0, index, finals[index]))
    # The intermediate line is still the mid-stack pass, not the answer.
    _Check(not (_Close(lifted[2][1], finals[2][1])
                and _Close(lifted[2][2], finals[2][2])),
           ("the pooled intermediate reads as the final pose", lifted,
            finals))
    # The intermediate line is still the mid-stack pass, not the answer.
    _Check(not (_Close(lifted[2][1], finals[2][1])
                and _Close(lifted[2][2], finals[2][2])),
           ("the pooled intermediate reads as the final pose", lifted,
            finals))
    _Check(host.frame == SHOWN_FRAME, "the pool moved the playhead")
    _Check(host.reached == [], host.reached)
    pooled.Close()
    _Check(not pooled.attached, "a closed panel kept its pool")

    # -- nothing above authored a stage opinion ---------------------------
    _Check(stage.GetSessionLayer().ExportToString() == authored,
           "the inspector authored into the stage")

    # -- a payload with no points is an answer, not a missing value -------
    emptyStage, emptyRig = _author_empty_rig(rigexec)
    emptyAuthored = emptyStage.GetSessionLayer().ExportToString()
    emptyModel = insp.OutputInspectorModel(workers=0)
    emptyBindings = emptyModel.Attach(emptyRig)
    emptyPoints = [binding for binding in emptyBindings
                   if binding.kind == "points"]
    _Check(len(emptyPoints) >= 1, [binding.kind for binding in emptyBindings])
    _Check(emptyModel.Select(emptyPoints) == [b.semantic_key
                                              for b in emptyPoints],
           emptyModel.Selected())
    emptyRows = _DrainReady(emptyModel, None, len(emptyPoints))
    for row in emptyRows:
        value = row.packet.value
        _Check(value is not None and value.elements == 0,
               (row.phase, row.ValueText()))
        _Check(not row.payload_error, row.payload_error)
        # The line says zero without reaching for a first point the payload
        # does not have, and it still reads as a value rather than a status.
        _Check(row.ValueText() == "0 pts", row.ValueText())
        _Check(row.ValueText() not in insp.STATUS_LABELS.values(),
               row.ValueText())
        _Check(row.label == "value" and row.status == insp.READY,
               (row.status, row.label))
    _Check(not emptyModel.HasWork(), "an empty payload still owed work")
    # Reading it authored nothing, exactly as reading a payload with points.
    _Check(emptyStage.GetSessionLayer().ExportToString() == emptyAuthored,
           "inspecting an empty payload authored into the stage")
    emptyModel.Close()

    # Display fallbacks the view relies on: an unknown disposition still
    # paints, and a real frame still reads as one.
    _Check(insp.StatusLabel("unheard-of") == "unheard-of",
           insp.StatusLabel("unheard-of"))
    _Check(insp.StatusColor("unheard-of"), "no color for an unknown status")
    _Check(insp.IsFrame(None) and insp.IsFrame(0) and insp.IsFrame(-2.5),
           "a real frame read as no frame")
    for bad in (True, float("nan"), float("inf"), "7", type(None)):
        _Check(not insp.IsFrame(bad), "%r read as a frame" % (bad,))

    print("output inspector model: exact-frame demand, intermediate kept "
          "apart from the final pass, viewer untouched, statuses and "
          "teardown -- all checks passed")


if __name__ == "__main__":
    main()
