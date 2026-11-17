"""rigexec.subscriptions: evaluated outputs at exact frames, headless.

A small rig is authored with the Builder and compiled once. Subscription
answers are compared against the live evaluation of the same rig, which is
what makes the time semantics a real check rather than one path replaying its
own numbers:

  * ``None`` is the default frame and is kept apart from frame zero, and
    negative and subframe time codes are real times -- never rounded, never
    defaulted;
  * an answer is a read-only copy that stands unchanged across later runs,
    across its subscription's close, and across the service's close;
  * an edit at an already answered frame is demanded again, not answered from
    the retired record;
  * leaving a subscription's scope releases its demand, and so does closing it;
  * updating a request replaces its demand, and a recompile makes the handles
    it was built from stale -- for a subscription already open too;
  * two clients on one frame get one job, and a subscription for an already
    answered frame captures the frame and takes a coverage hit instead of
    pulling the evaluator;
  * the coverage budget charges the whole job -- the snapshot clone, the
    workspace clone its lane runs on, and the freeze-captured tables -- and
    refuses a frame that exceeds it before any lane exists;
  * a payload the rig published with zero elements is a Ready answer with an
    empty lease, not a missing buffer and not a failure.

The host drives everything with ``pump()`` and ``poll()`` -- no Qt, no timer,
and with ``workers=0`` not a thread. The last section runs a real pool.

Usage:
    python test_rigexec_subscriptions.py [schema_resources_dir]
"""

import os
import pathlib
import sys

# Keep os.add_dll_directory handles alive for the lifetime of the process:
# on Windows, CPython drops a DLL search directory when its handle is
# garbage-collected.
_DLL_DIR_HANDLES = []

POINTS_TARGET = "/Model/Geom/Body.points"
SCALAR_TARGET = "/Model/Data.amount"
EMPTY_PATH = "/Model/Geom/Empty"
EMPTY_TARGET = EMPTY_PATH + ".points"
CONTROL_PATH = "/Rig/Controls/Ctrl"
REST_POINTS = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 2.0, 0.0)]

# The control's tz: an authored default, plus samples that make the default
# frame, frame zero, a subframe and a negative frame answer differently.
DEFAULT_TZ = 2.0
TZ_SAMPLES = {-6.0: -1.0, 0.0: 0.0, 10.0: 5.0}


def _close(a, b, eps=1e-6):
    return abs(a - b) < eps


def _setup_environment():
    """Make this test runnable from ctest, a bare shell, or a helper.

    Same bootstrap as tests/python/test_rigexec_python.py: pxr must resolve
    from THIS USD install, and the build tree must supply _rigexec.
    """
    repo_root = pathlib.Path(__file__).resolve().parents[2]
    usd_install = pathlib.Path(
        os.environ.get("RIGEXEC_USD_INSTALL") or (repo_root.parent /
                                                  "usd-install"))

    site_candidates = []
    win_site = usd_install / "Lib" / "site-packages"
    if win_site.is_dir():
        site_candidates.append(win_site)
    site_candidates.extend(sorted(usd_install.glob("lib/python*/site-packages")))
    for site_packages in site_candidates:
        if site_packages.is_dir() and str(site_packages) not in sys.path:
            sys.path.insert(0, str(site_packages))

    pluginpath = usd_install / "lib" / "usd"
    if pluginpath.is_dir() and not os.environ.get("PXR_PLUGINPATH_NAME"):
        os.environ["PXR_PLUGINPATH_NAME"] = str(pluginpath)

    build_root = (pathlib.Path(sys.argv[1]).parents[2] if len(sys.argv) > 1
                  else repo_root / "build")
    py_dir = build_root / "python"
    candidates = []
    if py_dir.is_dir():
        candidates.append(py_dir)
        for sub in sorted(py_dir.iterdir()):
            if sub.is_dir() and any(sub.glob("_rigexec*")):
                candidates.append(sub)
    for d in reversed(candidates):
        if str(d) not in sys.path:
            sys.path.insert(0, str(d))

    if hasattr(os, "add_dll_directory"):
        dll_dirs = [usd_install / "lib", usd_install / "bin", build_root]
        if build_root.is_dir() and not (build_root / "rigExec.dll").is_file():
            for sub in sorted(build_root.iterdir()):
                if sub.is_dir() and (sub / "rigExec.dll").is_file():
                    dll_dirs.append(sub)
        for d in dll_dirs:
            try:
                if pathlib.Path(d).is_dir():
                    _DLL_DIR_HANDLES.append(os.add_dll_directory(str(d)))
            except OSError:
                pass

    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import rigexec_test_env
    rigexec_test_env.ScrubForeignPxrFinders(usd_install)


def _author_rig(Usd, Sdf, rigexec):
    """One control, one matrix mover over a Points prim, one float mover."""
    from pxr import UsdGeom
    stage = Usd.Stage.CreateInMemory()
    body = UsdGeom.Points.Define(stage, "/Model/Geom/Body")
    body.CreatePointsAttr().Set(REST_POINTS)

    data = stage.DefinePrim("/Model/Data", "Xform")
    data.CreateAttribute("amount", Sdf.ValueTypeNames.Float).Set(3.0)

    builder = rigexec.Builder.create(stage, "/Rig")
    ctrl = builder.add_control("Ctrl")
    tz = stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:tz")
    assert tz.IsValid(), "the control has no avars:tz"
    tz.Set(DEFAULT_TZ)
    for frame, value in sorted(TZ_SAMPLES.items()):
        tz.Set(value, Usd.TimeCode(frame))

    chain = builder.new_mover_chain("chain")
    chain.add_matrix_mover("move", ctrl.path, None, POINTS_TARGET)
    chain.add_float_math_mover("double", "multiply", 2.0, SCALAR_TARGET)

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    return stage, rig, ctrl.path, tz


def _author_empty_rig(Usd, rigexec):
    """One control-driven mover over a Points prim authored with no points.

    Zero elements is a payload the extraction packs on purpose: a zero-length
    lease rather than a buffer.
    """
    from pxr import UsdGeom
    stage = Usd.Stage.CreateInMemory()
    empty = UsdGeom.Points.Define(stage, EMPTY_PATH)
    empty.CreatePointsAttr().Set([])

    builder = rigexec.Builder.create(stage, "/Rig")
    ctrl = builder.add_control("Ctrl")
    stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:tz").Set(1.5)
    chain = builder.new_mover_chain("chain")
    chain.add_matrix_mover("move", ctrl.path, None, EMPTY_TARGET)

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()
    return stage, rig


def _expected_tz(Usd, attr, time):
    """The control's tz at one exact frame, read straight off the stage."""
    if time is None:
        return attr.Get(Usd.TimeCode.Default())
    return attr.Get(Usd.TimeCode(time))


def _find(bindings, target, kind=None, computation=None, phase=None):
    for binding in bindings:
        if binding.target != target:
            continue
        if kind is not None and binding.kind != kind:
            continue
        if computation is not None and binding.computation != computation:
            continue
        if phase is not None and binding.phase != phase:
            continue
        return binding
    return None


def _drain(subs):
    assert subs.settle(timeout=10.0), "the subscription queue never drained"


def _by_key(packets):
    return {(packet.semantic_key, packet.time): packet
            for packet in packets}


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    _setup_environment()

    from pxr import Sdf, Usd
    import rigexec
    from rigexec import subscriptions

    rigexec.load_schema_plugin(plugin_dir)
    stage, rig, control_path, tz_attr = _author_rig(Usd, Sdf, rigexec)
    # Subscriptions compile, sample, and publish; they never author.
    authored = stage.GetSessionLayer().ExportToString()

    # -- discovery names the outputs a subscription can ask for ------------
    subs = subscriptions.SubscriptionService(rig, workers=0)
    assert subs.workers == 0, subs.workers
    bindings = subs.discover()
    assert bindings, "discovery found no retained outputs"
    points = _find(bindings, POINTS_TARGET, kind="points", phase="final")
    scalar = _find(bindings, SCALAR_TARGET, kind="scalar", phase="final")
    transform = _find(bindings, control_path, kind="transform",
                      computation="computeMatrix")
    if not (points and scalar and transform):
        for binding in bindings:
            print("  %-9s %-30s %-24s %-8s %s" %
                  (binding.kind, binding.target, binding.computation,
                   binding.phase, binding.type))
        raise AssertionError("discovery is missing an expected output: "
                             "%r %r %r" % (points, scalar, transform))

    # -- None, zero, a subframe and a negative frame are four demands ------
    times = [None, 0.0, 2.5, -3.0]
    outputs = [points, transform, scalar]
    sub = subs.subscribe(outputs, times)
    assert sub.ok, (sub.status, sub.reason)
    assert sub.frames == 4, sub.frames
    stats = subs.stats()
    assert stats["subscriptions"] == 1, stats
    # The default frame never merges with frame zero, and a subframe or a
    # negative code is never rounded onto another frame: four exact frames are
    # four demanded frames.
    assert stats["frames_demanded"] == 4, stats
    _drain(subs)
    stats = subs.stats()
    assert stats["capture_failures"] == 0, stats
    assert stats["declined_coverage_budget"] == 0, stats
    assert stats["dispatches"] == 4, stats

    packets = sub.poll()
    assert len(packets) == 12, [(p.semantic_key, p.status, p.reason)
                                for p in packets]
    for packet in packets:
        assert packet.status == subscriptions.READY, (packet.semantic_key,
                                                      packet.status,
                                                      packet.reason)
    keyed = _by_key(packets)
    for time in times:
        for binding in outputs:
            assert (binding.semantic_key, time) in keyed, (binding, time)

    # The answer names the value revision it materialized, so a host can pin
    # exactly that in a later request instead of accepting whatever stands.
    standing = keyed[(points.semantic_key, 2.5)].handle["value_revision"]
    assert standing, keyed[(points.semantic_key, 2.5)].handle
    pinned = subs.subscribe([{**points.handle, "value_revision": standing}], 2.5)
    assert pinned.ok, (pinned.status, pinned.reason)
    _drain(subs)
    pinned_packet = _by_key(pinned.poll())[(points.semantic_key, 2.5)]
    assert pinned_packet.status == subscriptions.READY, (pinned_packet.status,
                                                        pinned_packet.reason)
    assert pinned_packet.handle["value_revision"] == standing, \
        pinned_packet.handle
    ahead = subs.subscribe([{**points.handle,
                            "value_revision": standing + 1}], 2.5)
    assert ahead.ok, (ahead.status, ahead.reason)
    _drain(subs)
    ahead_packet = _by_key(ahead.poll())[(points.semantic_key, 2.5)]
    assert ahead_packet.status == subscriptions.INVALIDATED, (
        ahead_packet.status, ahead_packet.reason)

    def matrix_at(time):
        return keyed[(transform.semantic_key, time)].value.matrix

    def points_at(time):
        return keyed[(points.semantic_key, time)].value

    def scalar_at(time):
        return keyed[(scalar.semantic_key, time)].value.scalar

    for time in times:
        tz = _expected_tz(Usd, tz_attr, time)
        assert _close(matrix_at(time)[14], tz), (time, matrix_at(time), tz)
        held = points_at(time)
        for index, rest in enumerate(REST_POINTS):
            point = held.point(index)
            assert _close(point[0], rest[0]), (time, point, rest)
            assert _close(point[1], rest[1]), (time, point, rest)
            assert _close(point[2], rest[2] + tz), (time, point, rest)

    # The default frame is not frame zero's answer: its control sits at the
    # authored default, where frame zero reads its first sample.
    assert matrix_at(None)[14] != matrix_at(0.0)[14]
    assert _close(matrix_at(None)[14], DEFAULT_TZ), matrix_at(None)
    assert _close(matrix_at(0.0)[14], 0.0), matrix_at(0.0)
    # A subframe and a negative frame interpolate; neither is defaulted nor
    # rounded to an integer frame.
    assert _close(matrix_at(2.5)[14], 1.25), matrix_at(2.5)
    assert _close(matrix_at(-3.0)[14], -0.5), matrix_at(-3.0)

    # Same rig, same frame, through the live path: the subscription answers
    # with the rig's own numbers. (Rig.evaluate maps a negative frame to the
    # default, so it checks the non-negative times.)
    for time in (None, 0.0, 2.5):
        live = rig.evaluate(-1.0 if time is None else time)
        moved = [tuple(p) for p in live.moved_property(POINTS_TARGET)]
        held = points_at(time)
        for index, point in enumerate(moved):
            assert _close(held.point(index)[2], point[2]), (time, index,
                                                            held.point(index),
                                                            point)
        assert _close(scalar_at(time),
                      float(live.moved_property(SCALAR_TARGET))), time

    # -- an answer is read-only --------------------------------------------
    retained = points_at(2.5)
    retained_matrix = matrix_at(-3.0)
    assert retained.doubles.readonly, "a retained payload is writable"
    snapshot = tuple(retained.doubles)
    assert len(snapshot) == 9, snapshot
    try:
        retained.doubles[0] = 1.0
    except TypeError:
        pass
    else:
        raise AssertionError("a retained payload can be written")

    # -- a second client on an answered frame pulls nothing ----------------
    before_hit = subs.stats()
    second = subs.subscribe([points], [2.5])
    assert second.ok, (second.status, second.reason)
    _drain(subs)
    after_hit = subs.stats()
    # The frame was already demanded and already answered: the second client
    # joins that group and takes coverage hits, so no job and no evaluator
    # pull happens for it.
    assert after_hit["frames_demanded"] == before_hit["frames_demanded"], after_hit
    assert after_hit["dispatches"] == before_hit["dispatches"], after_hit
    assert after_hit["coverage_hits"] > before_hit["coverage_hits"], after_hit
    hit = second.poll()
    assert len(hit) == 1 and hit[0].status == subscriptions.READY, hit
    assert _close(hit[0].value.point(0)[2], 1.25), hit[0].value.points

    # -- two clients on a fresh frame get one job --------------------------
    before_pair = subs.stats()
    fresh = subs.subscribe([points], [7.0])
    also = subs.subscribe([transform, scalar], [7.0])
    assert fresh.ok and also.ok, (fresh.status, also.status)
    paired = subs.stats()
    # One frame group for two clients, and one admission whose launched mask
    # covers the union of both requests.
    assert paired["frames_demanded"] == before_pair["frames_demanded"] + 1, paired
    assert paired["dispatches"] == before_pair["dispatches"] + 1, paired
    _drain(subs)
    fresh_packet = _by_key(fresh.poll())[(points.semantic_key, 7.0)]
    also_packets = _by_key(also.poll())
    assert all(packet.status == subscriptions.READY
               for packet in also_packets.values()), [(p.status, p.reason)
                                                      for p in also_packets.values()]
    assert _close(fresh_packet.value.point(0)[2], 3.5), fresh_packet.value.points
    assert _close(also_packets[(transform.semantic_key, 7.0)].value.matrix[14],
                  3.5), also_packets[(transform.semantic_key, 7.0)]

    # Nothing that ran since disturbed the answer held from an earlier frame.
    assert tuple(retained.doubles) == snapshot, "a retained payload moved"
    assert _close(retained.point(0)[2], 1.25), tuple(retained.doubles)
    assert _close(retained_matrix[14], -0.5), retained_matrix

    # -- an output that joined the frame while its job ran stays Pending -----
    # The launched mask is fixed at admission, so the narrow job answers only
    # its own output. Polling the output that joined mid-flight, before the
    # next pump, must never copy a Ready packet from its sibling.
    widening = subs.subscribe([scalar], [21.0])
    joined = subs.subscribe([points], [21.0])
    assert widening.ok and joined.ok, (widening.status, joined.status)
    assert subs.run_next_queued(), "the narrow job never ran"
    ran_first = _by_key(widening.poll())[(scalar.semantic_key, 21.0)]
    assert ran_first.status == subscriptions.READY, ran_first.reason
    unrun = _by_key(joined.poll())[(points.semantic_key, 21.0)]
    assert unrun.status == subscriptions.PENDING, (unrun.status, unrun.reason)
    assert not unrun.ready and unrun.value is None, unrun
    assert not unrun._bytes, unrun
    assert "next pump" in unrun.reason, unrun.reason

    # The next pump runs the widened demand, and that output then reads Ready
    # with the payload the frame's second run produced.
    subs.pump()
    assert subs.run_next_queued(), "the widened demand was never re-admitted"
    answered = _by_key(joined.poll())[(points.semantic_key, 21.0)]
    assert answered.status == subscriptions.READY, answered.reason
    assert _close(answered.value.point(0)[2],
                  _expected_tz(Usd, tz_attr, 21.0)), answered.value.points
    assert _by_key(widening.poll())[(scalar.semantic_key, 21.0)].status == \
        subscriptions.READY
    joined.close()
    widening.close()

    # -- an edit at an answered frame re-earns the answer ------------------
    before_edit = subs.stats()
    tz_attr.Set(9.0, Usd.TimeCode(7.0))
    _drain(subs)
    after_edit = subs.stats()
    # Same frame, same outputs, same demand: only what the frame's inputs
    # stand at moved. The pump reads that in its own capture, so the frame is
    # demanded again rather than answered from the retired record.
    assert after_edit["dispatches"] > before_edit["dispatches"], after_edit
    assert after_edit["capture_failures"] == 0, after_edit
    edited = _by_key(fresh.poll())
    edited_also = _by_key(also.poll())
    for keyed in (edited, edited_also):
        for packet in keyed.values():
            assert packet.status == subscriptions.READY, (packet.semantic_key,
                                                          packet.status,
                                                          packet.reason)
    assert _close(edited[(points.semantic_key, 7.0)].value.point(0)[2],
                  9.0), edited[(points.semantic_key, 7.0)].value.points
    assert _close(edited_also[(transform.semantic_key, 7.0)].value.matrix[14],
                  9.0), edited_also[(transform.semantic_key, 7.0)].value.matrix
    # The answer polled before the edit still reads what it read then.
    assert _close(fresh_packet.value.point(0)[2], 3.5), fresh_packet.value.points

    # -- update replaces the demand ----------------------------------------
    receipt = second.update([points], [11.0, 12.0])
    assert receipt["status"] == subscriptions.ACCEPTED, receipt
    assert second.ok and second.frames == 2, (second.status, second.frames)
    updated = second.poll()
    assert {packet.time for packet in updated} == {11.0, 12.0}, updated
    _drain(subs)
    for packet in second.poll():
        assert packet.status == subscriptions.READY, (packet.status,
                                                      packet.reason)

    # A refused update changes nothing: the old request still stands, and the
    # refusal is reported rather than applied.
    bad_handle = dict(points.handle)
    bad_handle["semantic_key"] = "rigexec:no-such-output"
    refused = second.update([bad_handle], [13.0])
    assert refused["status"] == "unknown output", refused
    assert "no-such-output" in refused["reason"], refused
    assert second.ok and second.frames == 2, (second.status, second.frames)
    assert second.rejected, second.rejected
    assert {packet.time for packet in second.poll()} == {11.0, 12.0}

    # -- leaving a subscription's scope releases its demand ----------------
    before = subs.stats()
    with subs.subscribe([points], [21.0]) as scoped:
        assert scoped.ok, scoped.reason
        assert subs.stats()["frames_demanded"] == before["frames_demanded"] + 1, subs.stats()
        _drain(subs)
        assert scoped.poll()[0].status == subscriptions.READY
    after = subs.stats()
    assert after["subscriptions"] == before["subscriptions"], after
    assert after["frames_demanded"] == before["frames_demanded"], after
    assert scoped.closed and not scoped.ok
    try:
        scoped.poll()
    except ValueError:
        pass
    else:
        raise AssertionError("a closed subscription still polls")

    # A bare close does the same, and does it idempotently.
    closed = subs.subscribe([points], [22.0])
    assert closed.ok, closed.reason
    _drain(subs)
    closed.close()
    closed.close()
    assert subs.stats()["frames_demanded"] == before["frames_demanded"], subs.stats()

    # -- a real pool answers the same frames, on the owner's pump ----------
    pooled = subscriptions.SubscriptionService(rig, workers=2)
    assert pooled.workers == 2, pooled.workers
    with pooled.subscribe(outputs, [None, 1.5, -2.0]) as live:
        assert live.ok, (live.status, live.reason)
        assert pooled.settle(timeout=10.0)
        keyed_live = _by_key(live.poll())
        assert len(keyed_live) == 9, [(p.semantic_key, p.time, p.status,
                                       p.reason) for p in live.poll()]
        for time in (None, 1.5, -2.0):
            tz = _expected_tz(Usd, tz_attr, time)
            assert _close(keyed_live[(transform.semantic_key,
                                      time)].value.matrix[14], tz), time
            assert _close(keyed_live[(points.semantic_key,
                                      time)].value.point(1)[2],
                          REST_POINTS[1][2] + tz), time
    assert pooled.stats()["subscriptions"] == 0, pooled.stats()
    del pooled
    assert stage.GetSessionLayer().ExportToString() == authored, \
        "a subscription authored into the stage"

    # -- handles name the rig instance, not just the prim path --------------
    # A second, independent stage with the very same prim path and the same
    # program shape: its handles and this rig's are never interchangeable,
    # even while their prim paths and compile epochs read alike.
    stage2, rig2, _, tz2 = _author_rig(Usd, Sdf, rigexec)
    tz2.Set(7.0, Usd.TimeCode(10.0))  # stage B answers its own control value
    other = subscriptions.SubscriptionService(rig2, workers=0)
    foreign_bindings = other.discover()
    foreign_points = _find(foreign_bindings, POINTS_TARGET, kind="points",
                           phase="final")
    assert foreign_points, "the second rig discovered no points output"
    assert foreign_points.handle["rig_identity"] != \
        points.handle["rig_identity"], (foreign_points.handle,
                                        points.handle)
    assert foreign_points.handle["rig_identity"].startswith("/Rig")
    assert points.handle["rig_identity"].startswith("/Rig")

    crossed = subs.subscribe([foreign_points], [10.0])
    assert not crossed.ok, crossed.reason
    assert crossed.status == "stale compile epoch", crossed.status
    assert foreign_points.handle["rig_identity"] in crossed.reason, crossed.reason
    back = other.subscribe([points], [10.0])
    assert not back.ok, back.reason
    assert back.status == "stale compile epoch", back.status

    # The foreign handle is refused in company too: a subscription that
    # names a local output beside the foreign one refuses the whole request
    # rather than silently answering half of it.
    mixed = subs.subscribe([points, foreign_points], [10.0])
    assert not mixed.ok, mixed.reason

    # The second stage answers its own stage: frame 10 moves by stage B's
    # tz, not stage A's.
    own = other.subscribe([foreign_points], [10.0])
    assert own.ok, (own.status, own.reason)
    assert other.settle(timeout=10.0), "the second service never drained"
    own_packets = own.poll()
    assert len(own_packets) == 1, own_packets
    assert own_packets[0].status == subscriptions.READY, own_packets[0].reason
    assert _close(own_packets[0].value.point(1)[2], REST_POINTS[1][2] + 7.0), \
        own_packets[0].value
    own.close()

    # Same-rig sharing stands: another service over the same rigexec.Rig
    # minted handles carries the same instance identity, and handles
    # discovered on one service answer on the other.
    sharing = subscriptions.SubscriptionService(rig, workers=0)
    shared = sharing.subscribe([points], [3.0])
    assert shared.ok, (shared.status, shared.reason)
    assert sharing.settle(timeout=10.0)
    shared_packets = shared.poll()
    assert len(shared_packets) == 1, shared_packets
    assert shared_packets[0].status == subscriptions.READY
    assert _close(shared_packets[0].value.point(1)[2],
                  REST_POINTS[1][2] + _expected_tz(Usd, tz_attr, 3.0)), \
        shared_packets[0].value
    sharing.close_all()
    del sharing, other, rig2, stage2
    assert stage.GetSessionLayer().ExportToString() == authored, \
        "a subscription authored into a stage"

    # -- a recompile makes the handles it was built from stale -------------
    extra = rigexec.Builder.create(stage, "/Rig").new_mover_chain("second")
    extra.add_float_math_mover("again", "add", 1.0, SCALAR_TARGET)
    rig.compile()

    # A subscription already open was built from the retired epoch's handles.
    # The pump refuses its frames rather than running the old epoch's job
    # closure over the new program, and stops answering them from the retired
    # epoch's record.
    _drain(subs)
    stale_frames = sub.poll()
    assert len(stale_frames) == 12, len(stale_frames)
    for packet in stale_frames:
        assert packet.status == subscriptions.INVALIDATED, (packet.semantic_key,
                                                            packet.time,
                                                            packet.status,
                                                            packet.reason)
        assert "epoch" in packet.reason, packet.reason

    stale = subs.subscribe([points], [0.0])
    assert stale.status == "stale compile epoch", (stale.status, stale.reason)
    assert not stale.ok and stale.id == 0, stale

    # Re-discovered handles answer the recompiled rig at the same frame; the
    # answer held from before the recompile never moves.
    fresh_bindings = {binding.semantic_key: binding
                      for binding in subs.discover()}
    resubscribed = subs.subscribe([fresh_bindings[points.semantic_key]], [0.0])
    assert resubscribed.ok, (resubscribed.status, resubscribed.reason)
    _drain(subs)
    now = resubscribed.poll()[0]
    assert now.status == subscriptions.READY, now.reason
    live = rig.evaluate(0.0)
    moved = [tuple(p) for p in live.moved_property(POINTS_TARGET)]
    assert _close(now.value.point(1)[2], moved[1][2]), (now.value.points, moved)
    assert tuple(retained.doubles) == snapshot, "a retained payload moved"
    resubscribed.close()

    # -- the budget pays for the whole job before this host builds a lane --
    # A demanded frame that misses coverage is charged what the job will own:
    # the snapshot clone, the workspace clone its lane runs on, and the
    # freeze-captured tables both read. Charging only one clone let a frame
    # in that the cache could not actually hold.
    estimate = subs.coverage_estimate()
    assert estimate > 0, estimate
    standing_cap = subs.coverage_cap()
    before = subs.stats()
    subs.set_coverage_cap(estimate - 1)
    assert subs.coverage_cap() == estimate - 1, subs.coverage_cap()
    tight = subs.subscribe([fresh_bindings[scalar.semantic_key]], [41.0])
    assert tight.ok, (tight.status, tight.reason)
    _drain(subs)
    stats = subs.stats()
    assert stats["declined_coverage_budget"] >= \
        before["declined_coverage_budget"] + 1, stats
    # The refusals came before a lane existed: the pool saw no job at all,
    # so no snapshot and no workspace were allocated for this frame, and
    # every retry re-declined at the same gate.
    assert stats["dispatches"] == before["dispatches"], stats
    declined_packets = tight.poll()
    assert declined_packets, "the refused frame answered nothing"
    for packet in declined_packets:
        assert packet.status == subscriptions.PENDING, \
            (packet.status, packet.reason)
        assert "budget" in (packet.reason or ""), packet.reason

    # Room for the whole job plus the frame's own sampled bytes admits the
    # same frame: the charge is the estimate, not a fraction of it.
    subs.set_coverage_cap(estimate + (1 << 20))
    subs.pump()
    _drain(subs)
    admitted = tight.poll()
    assert admitted and all(p.status == subscriptions.READY for p in admitted), \
        [(p.status, p.reason) for p in admitted]
    subs.set_coverage_cap(standing_cap)
    tight.close()

    # -- a closed subscription does not spend the budget forever -----------
    # An answered frame leaves a coverage record charged for what its run
    # captured, and closing the subscription releases the pins its polls took
    # but keeps that charge. The record is not a permanent reservation: at a
    # cap that holds one standing record and one job, a host that closes each
    # subscription before demanding the next frame still gets every frame
    # answered, with the cap and the service untouched.
    cycleStage, cycleRig, cycleCtrl, cycleTz = _author_rig(Usd, Sdf, rigexec)
    cycle = subscriptions.SubscriptionService(cycleRig, workers=0)
    cycleScalar = _find(cycle.discover(), SCALAR_TARGET, kind="scalar",
                        phase="final")
    assert cycleScalar, "no scalar output to cycle"
    warm = cycle.subscribe([cycleScalar], [None])
    assert warm.ok, (warm.status, warm.reason)
    _drain(cycle)
    assert all(p.status == subscriptions.READY for p in warm.poll()), \
        [(p.status, p.reason) for p in warm.poll()]
    warm.close()
    warmStats = cycle.stats()
    standing = warmStats["coverage_bytes"]
    # Closing gave the pins back and kept the record's own charge.
    assert standing > 0, warmStats
    assert warmStats["pinned_bytes"] == 0, warmStats
    assert warmStats["coverage_records"] == 1, warmStats

    # The cap holds that record beside one job's workspace with a fraction of
    # the record to spare: a standing record leaves no room for the next job,
    # so each later cycle lands only if admission takes the stale record back.
    cap = cycle.coverage_estimate() + standing + max(1, standing // 8)
    cycle.set_coverage_cap(cap)
    for frame in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0):
        nxt = cycle.subscribe([cycleScalar], [frame])
        assert nxt.ok, (frame, nxt.status, nxt.reason)
        _drain(cycle)
        answered = nxt.poll()
        assert answered and all(p.status == subscriptions.READY
                                for p in answered), \
            (frame, [(p.status, p.reason) for p in answered])
        nxt.close()
    stats = cycle.stats()
    # Every cycle was admitted on its own ask: nothing waited for a later pump
    # to retry, and the store never grew past the frame being answered.
    assert stats["declined_coverage_budget"] == 0, stats
    assert stats["coverage_records"] == 1, stats
    assert cycle.coverage_cap() == cap, cycle.coverage_cap()
    cycle.close_all()
    assert cycle.stats()["pinned_payloads"] == 0, cycle.stats()

    # -- a payload with zero elements is an answer, not a missing buffer ---
    emptyStage, emptyRig = _author_empty_rig(Usd, rigexec)
    emptyAuthored = emptyStage.GetSessionLayer().ExportToString()
    emptySubs = subscriptions.SubscriptionService(emptyRig, workers=0)
    emptyOutputs = [binding for binding in emptySubs.discover()
                    if binding.kind == "points"]
    assert emptyOutputs, "an empty Points prim exports no points output"
    emptySub = emptySubs.subscribe(emptyOutputs, [None])
    assert emptySub.ok, (emptySub.status, emptySub.reason)
    _drain(emptySubs)
    emptyPackets = emptySub.poll()
    assert len(emptyPackets) == len(emptyOutputs), emptyPackets
    for packet in emptyPackets:
        assert packet.status == subscriptions.READY, (packet.semantic_key,
                                                      packet.status,
                                                      packet.reason)
        assert packet.elements == 0, packet.elements
        assert packet.payload_error == "", packet.payload_error
        answer = packet.value
        assert answer is not None and len(answer) == 0, answer
        assert tuple(answer.points) == (), tuple(answer.points)
        # Strictness is not relaxed to accommodate the empty case: an empty
        # payload still has no point zero to hand back.
        try:
            answer.point(0)
        except IndexError:
            pass
        else:
            raise AssertionError("point(0) answered for a zero-point payload")
    # The live path answers the same rig with the same nothing, and the
    # subscription authored nothing into the stage.
    assert len(emptyRig.evaluate().moved_property(EMPTY_TARGET)) == 0
    assert emptyStage.GetSessionLayer().ExportToString() == emptyAuthored
    emptySub.close()
    emptySubs.close_all()
    assert emptySubs.stats()["pinned_payloads"] == 0, emptySubs.stats()

    # -- the service owns its pool and cache -------------------------------
    subs.close_all()
    assert subs.stats()["subscriptions"] == 0, subs.stats()
    assert subs.stats()["frames_demanded"] == 0, subs.stats()
    # Closing released the pins the polls took, and the answer held from them
    # still reads what it read then.
    assert tuple(retained.doubles) == snapshot, "a retained payload moved"
    assert _close(retained.point(2)[2], REST_POINTS[2][2] + 1.25), snapshot

    # A frame is a finite number or the default, never a bool, a string, or a
    # non-finite value that would answer some other frame.
    for bad in (float("nan"), float("inf"), True, "7"):
        try:
            subs.subscribe([points], [bad])
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError("%r was accepted as a frame" % (bad,))
    for bad_times in ([], ""):
        try:
            subs.subscribe([points], bad_times)
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError("times %r were accepted" % (bad_times,))
    try:
        subs.subscribe([], [0.0])
    except (TypeError, ValueError):
        pass
    else:
        raise AssertionError("an empty output list was accepted")

    # A service over a rig that never compiled refuses instead of crashing.
    uncompiled = rigexec.Rig(stage, "/Rig")
    bare = subscriptions.SubscriptionService(uncompiled)
    try:
        bare.discover()
    except ValueError:
        pass
    else:
        raise AssertionError("discovery answered without a compiled program")
    no_program = bare.subscribe([points], [0.0])
    assert no_program.status == "unknown rig", no_program.status
    del bare
    del uncompiled

    print("rigexec.subscriptions: exact frames, shared demand, read-only "
          "answers -- all checks passed")


if __name__ == "__main__":
    main()
