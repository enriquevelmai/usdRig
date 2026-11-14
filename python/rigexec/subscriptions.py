"""Evaluated rig outputs at exact frames, for hosts with no viewer.

A subscription names the outputs a host wants and the exact frames it wants
them at. RigExec admits that work to its own background pool, runs it through
the same operation graph a full frame runs on, stores each answer as an
immutable coverage record, and hands packets back when the host asks. A
headless host drives it with its own ``pump()`` and ``poll()`` -- there is no
timer, no Qt event loop, and nothing evaluated on a reader's thread
(docs/specs/output-subscriptions.md).

    import rigexec
    from rigexec import subscriptions

    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()

    subs = subscriptions.SubscriptionService(rig)
    points = [b for b in subs.discover() if b.target.endswith(".points")]
    sub = subs.subscribe(points, times=[None, 0, 2.5, -3])
    subs.settle()
    for packet in sub.poll():
        print(packet.status, packet.time, packet.value.points[0:3])
    sub.close()

``None`` is the default frame and is never frame zero; a number is one exact
finite USD time code, so zero, negative frames and subframes are real times
and never round. Every answer is a copy of the published bytes, read back as
a read-only array, so holding a packet never aliases a live result buffer.
"""

import array
import time as _time

import _rigexec as _native

# The payload layouts outputDemand.h defines, spelled as a handle's type.
SCALAR = "double"
MATRIX4X4_ROW = "matrix4d"
POINT3_ARRAY = "point3d[]"

# A packet's disposition.
PENDING = "pending"
READY = "ready"
INVALIDATED = "invalidated"
UNSUPPORTED = "unsupported"
FAILED = "failed"

# A subscription receipt's status.
ACCEPTED = "accepted"

# Doubles per element of each layout, and the byte width the layout assumes.
_DOUBLES_PER_ELEMENT = {
    SCALAR: 1,
    MATRIX4X4_ROW: 16,
    POINT3_ARRAY: 3,
}
_DOUBLE_BYTES = 8


class OutputBinding:
    """One discoverable output of the rig's standing compiled program."""

    __slots__ = ("target", "computation", "phase", "type", "semantic_key",
                 "kind", "status", "reason", "handle")

    def __init__(self, record):
        self.target = record["target"]
        self.computation = record["computation"]
        self.phase = record["phase"]
        self.type = record["type"]
        self.semantic_key = record["semantic_key"]
        self.kind = record["kind"]
        self.status = record["status"]
        self.reason = record["reason"]
        self.handle = record["handle"]

    @property
    def producible(self):
        """False when the catalog names this output but cannot produce it."""
        return self.status != UNSUPPORTED

    def __repr__(self):
        return "<OutputBinding %s %r phase=%r type=%r>" % (
            self.kind, self.target, self.phase, self.type)


class OutputValue:
    """One answer's values, read back as read-only arrays.

    The bytes are a copy taken when the packet was built, so the answer cannot
    change under a holder -- not when the lane's arena is reused, not when a
    later run replaces the coverage record it came from.
    """

    __slots__ = ("layout", "elements", "_doubles")

    def __init__(self, layout, elements, raw):
        doubles_per_element = _DOUBLES_PER_ELEMENT.get(layout)
        if doubles_per_element is None:
            raise ValueError("no payload layout is defined for %r" % (layout,))
        if array.array("d").itemsize != _DOUBLE_BYTES:
            raise ValueError("this host's double is not the layout's 8 bytes")
        expected = elements * doubles_per_element * _DOUBLE_BYTES
        if len(raw) != expected:
            raise ValueError(
                "%s payload holds %d bytes for %d elements, expected %d" %
                (layout, len(raw), elements, expected))
        values = array.array("d")
        values.frombytes(bytes(raw))
        self.layout = layout
        self.elements = elements
        self._doubles = memoryview(values).toreadonly()

    @property
    def doubles(self):
        """Every value as one read-only array of float64, in layout order."""
        return self._doubles

    @property
    def scalar(self):
        if self.layout != SCALAR:
            raise ValueError("%r is not a scalar payload" % (self.layout,))
        return float(self._doubles[0])

    @property
    def matrix(self):
        """One 4x4 matrix, row-major, USD row-vector form (p' = p * M)."""
        if self.layout != MATRIX4X4_ROW or self.elements != 1:
            raise ValueError("%r with %d elements is not one matrix" %
                             (self.layout, self.elements))
        return tuple(self._doubles)

    @property
    def points(self):
        """The point triples as one read-only array: 3 * elements doubles."""
        if self.layout != POINT3_ARRAY:
            raise ValueError("%r is not a point payload" % (self.layout,))
        return self._doubles

    def point(self, index):
        if self.layout != POINT3_ARRAY:
            raise ValueError("%r is not a point payload" % (self.layout,))
        start = 3 * index
        if start < 0 or start + 3 > len(self._doubles):
            raise IndexError("point %d is not in a %d-point payload" %
                             (index, self.elements))
        return (float(self._doubles[start]), float(self._doubles[start + 1]),
                float(self._doubles[start + 2]))

    def __len__(self):
        return self.elements

    def __repr__(self):
        return "<OutputValue %s x%d>" % (self.layout, self.elements)


class Packet:
    """One (output, frame) answer, in the order it was demanded."""

    __slots__ = ("handle", "semantic_key", "time", "status", "reason",
                 "generation", "layout", "elements", "_bytes", "_value",
                 "payload_error")

    def __init__(self, record):
        self.handle = record["handle"]
        self.semantic_key = self.handle["semantic_key"]
        self.time = record["time"]
        self.status = record["status"]
        self.reason = record["reason"]
        self.generation = record["generation"]
        self.layout = record["layout"]
        self.elements = record["elements"]
        self.payload_error = record["payload_error"]
        self._bytes = record["bytes"]
        self._value = None

    @property
    def ready(self):
        return self.status == READY

    @property
    def value(self):
        """The answer as an ``OutputValue``, or None when it is not Ready."""
        if self.status != READY:
            return None
        if self._value is None:
            self._value = OutputValue(self.layout, self.elements, self._bytes)
        return self._value

    def __repr__(self):
        return "<Packet %r at %r %s>" % (self.semantic_key, self.time,
                                         self.status)


class Subscription:
    """One live subscription. Closing it releases the demand no other
    client holds; leaving its scope closes it."""

    __slots__ = ("service", "id", "status", "reason", "frames", "unsupported",
                 "rejected", "_closed")

    def __init__(self, service, receipt):
        self.service = service
        self.id = receipt["subscription"]
        self.status = receipt["status"]
        self.reason = receipt["reason"]
        self.frames = receipt["frames"]
        self.unsupported = tuple(receipt["unsupported"])
        self.rejected = "" if receipt["ok"] else (receipt["reason"]
                                                 or receipt["status"])
        self._closed = not self.id

    @property
    def ok(self):
        """False when the service refused the request, changing nothing."""
        return self.status == ACCEPTED and not self._closed

    @property
    def closed(self):
        return self._closed

    def poll(self):
        """What stands for each demanded (output, frame) pair, in request
        order. Polling evaluates and dispatches nothing."""
        if self._closed:
            raise ValueError("this subscription is closed")
        return [Packet(record) for record in self.service._records(self.id)]

    def update(self, outputs, times=None):
        """Replace this subscription's request. A refusal leaves the old
        request, its demand and its answers exactly as they were: the
        subscription stays live and ``ok``, and the refusal lands on the
        receipt and on ``rejected``."""
        if self._closed:
            raise ValueError("this subscription is closed")
        return self._apply(self.service.update(self.id, outputs, times))

    def _apply(self, receipt):
        """Bookkeeping for a receipt against this subscription."""
        if receipt["ok"]:
            self.status = receipt["status"]
            self.reason = receipt["reason"]
            self.frames = receipt["frames"]
            self.unsupported = tuple(receipt["unsupported"])
            self.rejected = ""
        else:
            self.rejected = receipt["reason"] or receipt["status"]
        return receipt

    def close(self):
        """End the subscription now. Idempotent."""
        if not self._closed:
            self.service.close(self.id)
            self._closed = True
            self.status = "closed"

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

    def __repr__(self):
        return "<Subscription %d %s frames=%d>" % (self.id, self.status,
                                                   self.frames)


class SubscriptionService:
    """Subscriptions for one compiled rig, with its own pool and cache.

    The host owns both, exactly as the native service does: closing the
    service joins the pool's lanes, which publish into that cache. Keep the
    ``rigexec.Rig`` alive while this lives -- a capture samples it on this
    thread, and it is the only live-USD read in the path.
    """

    __slots__ = ("_native",)

    def __init__(self, rig, workers=None):
        # Zero workers is the manual mode: nothing runs until the host calls
        # run_next_queued(), which is how a test pins the demand bookkeeping
        # down without a thread in the picture. Unset leaves the pool the size
        # the scheduler ships with.
        self._native = (_native.Subscriptions(rig) if workers is None
                        else _native.Subscriptions(rig, workers))

    @property
    def workers(self):
        """The pool's lane count; zero runs nothing until the host runs the
        queue itself."""
        return self._native.workers

    def discover(self):
        """Every explicitly retained output of the rig's standing compiled
        program, each with the handle a subscription names back."""
        return [OutputBinding(record) for record in self._native.discover()]

    def subscribe(self, outputs, times=None):
        """Demand ``outputs`` at ``times``.

        ``outputs`` are the bindings ``discover()`` returned; ``times`` is one
        frame or an iterable of them, ``None`` being the default frame and any
        other value one exact finite time code. A refusal is reported on the
        returned subscription -- ``status``, ``reason`` and ``unsupported`` --
        and moves no demand.
        """
        return Subscription(self, self._native.subscribe(outputs, times))

    def update(self, subscription, outputs, times=None):
        """Replace a subscription's request by object or bare id. The receipt
        is returned; an object subscription also records an accepted
        request's frame count and a refusal's reason."""
        receipt = self._native.update(_subscription_id(subscription), outputs,
                                      times)
        if isinstance(subscription, Subscription):
            subscription._apply(receipt)
        return receipt

    def poll(self, subscription):
        """``Subscription.poll`` through the service, for a bare id."""
        return [Packet(record)
                for record in self._records(_subscription_id(subscription))]

    def _records(self, subscription):
        """The native packet records for a live subscription id."""
        return self._native.poll(subscription)

    def pump(self):
        """Run what the pool completed since the last pump and re-dispatch
        the demand a retired generation or a widened demand left owed."""
        return self._native.pump()

    def run_next_queued(self):
        """Manual mode (``workers=0``): run the highest-priority queued job
        on this thread. False when the queue is empty."""
        return self._native.run_next_queued()

    def settle(self, timeout=5.0):
        """Run and pump until nothing is in flight, waiting at most
        ``timeout`` seconds. True when the queue drained.

        A widened demand re-dispatches once the job it widened is done, so
        one pump can leave more work owed; the loop keeps running and pumping
        until the in-flight count is empty. A frame the pool or the coverage
        budget declined is not in flight; it waits for the next pump and its
        packet says so in ``status``.
        """
        deadline = _time.monotonic() + timeout
        while True:
            if self.workers == 0:
                while self._native.run_next_queued():
                    pass
            self._native.pump()
            if not self._native.stats()["demands_in_flight"]:
                return True
            if _time.monotonic() >= deadline:
                return False
            _time.sleep(0.002)

    def close(self, subscription):
        """End one subscription: its demand is dropped unless another live
        client holds it, and its payload pins come back."""
        self._native.close(_subscription_id(subscription))
        if isinstance(subscription, Subscription):
            subscription._closed = True
            subscription.status = "closed"

    def close_all(self):
        """End every subscription this service holds."""
        self._native.close_all()

    def stats(self):
        """The service's lifecycle counters: subscriptions, frames demanded,
        demand in flight, coverage hits, dispatches, declines, and the bytes
        pinned for live clients."""
        return self._native.stats()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close_all()
        return False


def _subscription_id(subscription):
    return getattr(subscription, "id", subscription)
