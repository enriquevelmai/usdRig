#
# RigExec usdview plugin: one rig's exported outputs, at exact frames,
# without Qt or pxr.
#
# Headless by contract. This module imports neither Qt nor pxr, and it
# reaches evaluation through exactly one door -- the rigexec.subscriptions
# facade (docs/specs/output-subscriptions.md). It holds no evaluator, reads
# no stage, and has no fallback path that could answer a frame the service
# did not publish, so the exec-stack panel, the headless test, and any other
# host share one frame semantics, one set of dispositions, and one value
# spelling instead of drifting into three answers.
#
# The frame the viewer shows is READ here and never written. A host hands
# over the frame it is displaying and gets back the outputs demanded at that
# exact frame: there is no SetTime path in this module, and nothing it calls
# authors a stage opinion, so inspecting an intermediate at frame 2.5 while
# the artist sits at frame 12 demands 2.5 and leaves 12 on screen.
#
import math

from rigexec import subscriptions


# The service's dispositions, spelled once by the facade.
PENDING = subscriptions.PENDING
READY = subscriptions.READY
INVALIDATED = subscriptions.INVALIDATED
UNSUPPORTED = subscriptions.UNSUPPORTED
FAILED = subscriptions.FAILED

STATUSES = (PENDING, READY, INVALIDATED, UNSUPPORTED, FAILED)

# What each disposition is called in the panel, and the color it paints in.
# Defined once here: the view reuses them rather than naming its own words
# and colors (cacheStripModel.py is the same split).
STATUS_LABELS = {
    READY: "value",
    PENDING: "waiting",
    INVALIDATED: "stale",
    UNSUPPORTED: "not exported",
    FAILED: "failed",
}

STATUS_COLORS = {
    READY: "#4CAF50",
    PENDING: "#42A5F5",
    INVALIDATED: "#EF5350",
    UNSUPPORTED: "#9E9E9E",
    FAILED: "#EF5350",
}


def StatusLabel(status):
    """The panel's word for a disposition; an unknown one reads verbatim."""
    return STATUS_LABELS.get(status, status or "unknown")


def StatusColor(status):
    """The panel's color for a disposition."""
    return STATUS_COLORS.get(status, "#616161")


def IsFrame(value):
    """Whether `value` is a frame this model can demand.

    None is the default frame; anything else is one exact finite number.
    A bool is not a frame -- True would demand frame 1 -- and neither is an
    infinity or a NaN, which would answer some frame nobody asked for.
    """
    if value is None:
        return True
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    return isinstance(value, float) and math.isfinite(value)


def _AsFrame(value):
    """`value` as a demandable frame; a default time code reads as None.

    A host may hand back its own time-code spelling. Its default code reads
    as the default frame rather than as frame zero, which is the difference
    between the frame the author left and the frame the rig was compiled at.
    """
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        pass
    try:
        if value.IsDefault():
            return None
        return float(value.GetValue())
    except Exception:
        return None


class Row(object):
    """One (output, frame) line of the inspector."""

    __slots__ = ("target", "computation", "phase", "type", "kind",
                 "semantic_key", "time", "frameNamed", "status", "reason",
                 "generation", "payload_error", "packet")

    def __init__(self, binding, packet, askedFrame=None):
        self.packet = packet
        self.semantic_key = packet.semantic_key
        # A subscription bound at the rig's default frame answers without
        # spelling the frame; the frame this model asked for is the frame
        # that answer is for, and frameNamed says the packet spelled it.
        self.time = packet.time
        self.frameNamed = packet.time is not None
        if self.time is None:
            self.time = askedFrame
        self.status = packet.status
        self.reason = packet.reason
        self.generation = dict(packet.generation)
        self.payload_error = packet.payload_error
        # The packet's own spelling of what it answers: the handle's type,
        # and the payload's layout and element count beside it.
        self.type = packet.handle.get("type", "")
        self.target = (binding.target if binding is not None
                       else self.semantic_key)
        self.computation = (binding.computation if binding is not None
                            else "")
        self.phase = binding.phase if binding is not None else ""
        self.kind = binding.kind if binding is not None else ""

    @property
    def ready(self):
        return self.status == READY

    @property
    def label(self):
        return StatusLabel(self.status)

    @property
    def color(self):
        return StatusColor(self.status)

    @property
    def frameText(self):
        return "default" if self.time is None else ("%g" % (self.time,))

    def ValueText(self):
        """The answer in one line: a number, a matrix's translation, or a
        point count with its first point -- a payload with no points has no
        first point to show. Anything else says why instead."""
        if self.status != READY:
            return self.reason or StatusLabel(self.status)
        if self.payload_error:
            return self.payload_error
        try:
            value = self.packet.value
        except ValueError as error:  # a payload that does not match its type
            return str(error)
        if value is None:
            return StatusLabel(self.status)
        if value.layout == subscriptions.SCALAR:
            return "%.6g" % value.scalar
        if value.layout == subscriptions.MATRIX4X4_ROW:
            # Row-major with the translation in the last row: USD's
            # row-vector convention puts p * M's offset at 12..14.
            matrix = value.matrix
            return "t (%.4f, %.4f, %.4f)" % (matrix[12], matrix[13],
                                             matrix[14])
        # A Ready point payload may hold zero elements: the service packs a
        # zero-length lease rather than refusing the answer.
        if not value.elements:
            return "0 pts"
        first = value.point(0)
        return "%d pts, first (%.4f, %.4f, %.4f)" % (value.elements,
                                                     first[0], first[1],
                                                     first[2])

    def GenerationText(self):
        """What produced this answer: the program's identity and the input
        revision its capture was keyed on, in hex."""
        return "prog %08x  input %016x" % (
            int(self.generation.get("program_identity", 0)) & 0xFFFFFFFF,
            int(self.generation.get("input_revision", 0)))

    def Key(self):
        """What has to change for the panel to repaint this line."""
        return (self.semantic_key, self.time, self.status, self.reason,
                tuple(sorted(self.generation.items())))

    def __repr__(self):
        return "<Row %s at %s %s>" % (self.target, self.frameText,
                                      self.status)


class OutputInspectorModel(object):
    """What the output inspector knows about one compiled rig.

    `host` is the panel, or any stand-in that answers `DisplayedTime()` with
    the frame it is showing: the model reads that one fact and reaches for
    nothing else. `workers` sizes the service's own pool; zero is the
    facade's manual mode, which is how a test pins the demand bookkeeping
    down without a thread in the picture.

    The lifecycle is the panel's: `Attach` the rig it just compiled,
    `Select` the outputs to look at, `DemandFrame` or
    `DemandDisplayedFrame` for the exact frame, `Poll` on the host's tick,
    `NoteStageEdit` when the host sees the stage move under a standing
    demand, and `Close` on stage replacement or panel teardown -- which ends
    the demand and releases the pool.
    """

    def __init__(self, host=None, workers=None):
        self._host = host
        self._workers = workers
        self._rig = None
        self._service = None
        self._bindings = ()
        self._byKey = {}
        self._subscription = None
        self._selected = []
        self._time = None
        self._rows = []
        self._note = ""
        self._edited = False

    def __repr__(self):
        return "<OutputInspectorModel %s frame=%s selected=%d>" % (
            "attached" if self._service else "detached",
            self.FrameText(), len(self._selected))

    # -- lifecycle -------------------------------------------------------

    def Attach(self, rig):
        """Inspect `rig`, a compiled ``rigexec.Rig`` the host just built.

        The previous rig's service is closed first, which ends its demand,
        releases its payload pins and joins its pool: the handles it handed
        out name the compile it was built from, and a recompiled rig needs
        the new compile's handles. Returns the exported handles now
        listable.
        """
        self.Close()
        self._rig = rig
        self._service = subscriptions.SubscriptionService(rig,
                                                          self._workers)
        try:
            self._bindings = tuple(self._service.discover())
            self._note = ""
        except ValueError as error:  # a rig with no compiled program
            self._bindings = ()
            self._note = str(error)
        self._byKey = {}
        for binding in self._bindings:
            self._byKey[binding.semantic_key] = binding
        return self.Bindings()

    def Close(self):
        """End every demand and release the pool. Idempotent.

        The panel calls this when its stage is replaced and when it closes,
        so a closed panel holds no subscription, no pinned payload and no
        worker.
        """
        if self._subscription is not None:
            self._subscription.close()
            self._subscription = None
        if self._service is not None:
            self._service.close_all()
            # Dropping the service joins its lanes; nothing here outlives it.
            self._service = None
        self._rig = None
        self._bindings = ()
        self._byKey = {}
        self._rows = []
        self._selected = []
        self._note = ""
        self._edited = False

    @property
    def attached(self):
        return self._service is not None

    def stats(self):
        """The service's counters, or None once closed."""
        return None if self._service is None else self._service.stats()

    def SetHost(self, host):
        """Give the model the panel it reads its displayed frame from."""
        self._host = host

    # -- the exported handles --------------------------------------------

    def Bindings(self):
        """Every exported handle of the attached rig, in catalog order."""
        return list(self._bindings)

    def HandlesFor(self, path):
        """The exported outputs named by `path`: a prim, an attribute target,
        or a mover or solver whose own pass is exported.

        A geometry or property target spells its prim and the attribute
        (`/Body.points`), a control target is the control path itself, and a
        mover's own pass over a target is exported at the read phase named by
        that mover's path -- which is what makes a mid-stack output
        inspectable at all, and what a mover row in the execution stack
        names.
        """
        text = "" if path is None else str(path).strip()
        if not text:
            return []
        return [binding for binding in self._bindings
                if (text.startswith("/") and binding.phase == text)
                or binding.target == text
                or binding.target.split(".")[0] == text]

    def Select(self, outputs):
        """Inspect `outputs`, the bindings `Bindings` or `HandlesFor` gave.

        Returns the keys now selected. An empty selection demands nothing:
        the lines clear and no work is admitted.
        """
        keys = []
        for output in (outputs or ()):
            key = getattr(output, "semantic_key", None)
            if not key or key not in self._byKey or key in keys:
                continue
            keys.append(key)
        self._selected = keys
        return list(keys)

    def SelectFor(self, path):
        """Select the outputs of the prim at `path`; returns those keys."""
        return self.Select(self.HandlesFor(path))

    def Selected(self):
        return list(self._selected)

    def Binding(self, semantic_key):
        return self._byKey.get(semantic_key)

    # -- the frame being inspected ---------------------------------------

    def DemandFrame(self, frame):
        """Inspect at `frame`: None for the default frame, else one exact
        finite time code, so zero, negative frames and subframes are real
        times and never round onto another frame.

        Raises ValueError for anything else, leaving the standing demand
        alone: a bool, a string or a non-finite number would answer a frame
        the artist never asked to see.
        """
        if not IsFrame(frame):
            raise ValueError("%r is not a frame: use None for the default "
                             "frame, or one finite number" % (frame,))
        # Adding zero folds a signed-zero time code onto frame zero, which is
        # the frame the rig was compiled at rather than a frame of its own.
        self._time = None if frame is None else (float(frame) + 0.0)
        return self._time

    def DemandDisplayedFrame(self):
        """Inspect at the frame the host is showing.

        One-way by construction: the host is asked what it shows and is
        never told to change it, so an open inspector cannot move the
        viewport it is looking at. With no host, or a host that cannot say,
        the default frame is the demand.
        """
        return self.DemandFrame(self._HostFrame())

    def Frame(self):
        """The frame being inspected; None is the default frame."""
        return self._time

    def FrameText(self):
        return "default" if self._time is None else ("%g" % (self._time,))

    def _HostFrame(self):
        host = self._host
        read = getattr(host, "DisplayedTime", None)
        if read is None:
            return None
        return _AsFrame(read())

    # -- asking, and reading the answer ----------------------------------

    def NoteStageEdit(self):
        """Tell the model the host saw its stage change.

        The service reads the live stage in its own capture, so an edit only
        reaches an answered line through a poll: without this note the host
        has no reason to take one, and the line keeps showing the value
        captured before the edit. A host that ticks on `HasWork` therefore
        owes one more tick, which either finds the standing coverage record
        still answers the frame -- and reports no work again -- or re-demands
        it. A host with no timer calls `Poll` itself. The edit may name prims
        the rig never reads; that is what the tick is for.
        """
        self._edited = True

    def Poll(self):
        """One host tick: re-dispatch what is owed, then read the lines.

        True when a line changed, so the view repaints only on news. Polling
        evaluates nothing on this thread: the service's pool runs the work,
        `pump` delivers what it completed and re-dispatches what a widened
        request left owed, and the packets are copies of what the coverage
        records hold.
        """
        # The capture this poll takes reads whatever the edit changed, so the
        # rows it returns decide what is owed next.
        self._edited = False
        if self._service is None:
            changed = bool(self._rows)
            self._rows = []
            return changed
        outputs = [self._byKey[key] for key in self._selected
                   if key in self._byKey]
        if not outputs:
            if self._subscription is not None:
                self._subscription.close()
                self._subscription = None
            changed = bool(self._rows)
            self._rows = []
            return changed
        self._Request(outputs)
        self._service.pump()
        rows = self._ReadRows()
        changed = ([row.Key() for row in rows]
                   != [row.Key() for row in self._rows])
        self._rows = rows
        return changed

    def _Request(self, outputs):
        """Place the standing request, or replace it.

        A refusal leaves the old request, its demand and its answers exactly
        as they were and lands on `note`, which is what the panel shows in
        place of a value it does not have.
        """
        times = [self._time]
        if self._subscription is None or not self._subscription.ok:
            self._subscription = self._service.subscribe(outputs, times)
            self._note = self._subscription.rejected
            return
        receipt = self._subscription.update(outputs, times)
        self._note = "" if receipt["ok"] else (receipt["reason"]
                                               or receipt["status"])

    def RunQueued(self):
        """Manual mode (``workers=0``): run the highest-priority queued job
        on the calling thread. False when the queue is empty."""
        return (False if self._service is None
                else self._service.run_next_queued())

    def Drain(self):
        """Manual mode: run every queued job on this thread, then read.

        The synchronous form of a host's tick, for a caller that wants the
        answer before it returns rather than on the next repaint.
        """
        while self.RunQueued():
            pass
        return self.Poll()

    def _ReadRows(self):
        if self._subscription is None or not self._subscription.ok:
            return []
        return [Row(self._byKey.get(packet.semantic_key), packet, self._time)
                for packet in self._subscription.poll()]

    def Rows(self):
        """One line per demanded (output, frame) pair, in request order."""
        return list(self._rows)

    def HasWork(self):
        """True while a demanded line is still owed, or a reported edit has
        not been read by a poll yet."""
        return self._edited or any(row.status == PENDING
                                   for row in self._rows)

    @property
    def note(self):
        """Why the standing request is not standing, or why there is nothing
        to export; empty when all is well."""
        return self._note

    def Unsupported(self):
        """Handles the standing request names but cannot produce."""
        if self._subscription is None or not self._subscription.ok:
            return []
        return [handle.get("semantic_key", "")
                for handle in self._subscription.unsupported]

    def StatusText(self):
        """One line for the panel: what stands, or why it cannot."""
        if self._service is None:
            return "No rig attached, so nothing is demanded."
        if not self._bindings:
            return self._note or "This rig exports no retained outputs."
        if not self._selected:
            return ("Nothing selected: pick an output to inspect at %s."
                    % self.FrameText())
        counts = {}
        for row in self._rows:
            counts[row.status] = counts.get(row.status, 0) + 1
        bits = ["%d %s at %s" % (count, StatusLabel(status),
                                 self.FrameText())
                for status, count in sorted(counts.items())]
        if self._note:
            bits.append(self._note)
        for key in self.Unsupported():
            bits.append("%s is named but not produced" % key)
        return "  ".join(bits) or "Nothing demanded yet."
