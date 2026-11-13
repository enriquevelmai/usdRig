# Output subscriptions

A host that wants rig results — a Hydra reader, an export, a game adapter,
a bake, a preview — asks for **outputs at exact frames**. The contract that
connects that request to the evaluator lives in `libs/rigExecGraph/outputDemand.h`.
Scene lowering, frame-cache publication, bake, binary playback, imaging and
host adapters speak these types and may not declare their own time, handle,
status, packet, payload layout, payload lease or closure.

The contract is USD-free, like the rest of `rigExecGraph`: compiled rig
identity travels as text plus a compile epoch, and a payload is an immutable
byte lease. Nothing here reads a stage. See
[evaluated versus baked rigs](../concepts/baked-vs-dynamic.md) for the one
loop these requests are answered by, and
[frames on demand](frames-on-demand.md) for the imaging reader that uses it.

## The types

| Type | Names |
| --- | --- |
| `RigExecOutputTime` | One frame: the context's default frame, or one exact finite USD time code. |
| `RigExecOutputHandle` | One output of one compiled rig: rig identity, compile epoch, semantic key, payload type, read phase, value revision. |
| `RigExecOutputTimeDemand` | The outputs wanted at one exact time. |
| `RigExecOutputRequest` | What a host wants: one demand per exact time, with an explicit or speculative policy. |
| `RigExecOutputGeneration` | The full captured generation a value came from: program, input, preview and history revision. |
| `RigExecOutputPacket` | One answered output: the request, its generation key, a status with a reason, and the payload. |
| `RigExecOutputStatus` | `Pending`, `Ready`, `Invalidated`, `Unsupported`, `Failed`. |
| `RigExecPayloadKind` | The payload layout: `Scalar`, `Matrix4x4Row`, `Point3` (`Unknown` is never readable). |
| `RigExecOutputPayload` | A shared, immutable byte lease plus the layout and element count the producer declared. |
| `RigExecPayloadView` | Bounds-checked, memcpy-based element access to one lease. |
| `RigExecDemandEdges` | Declared validity, effect and history requirements between canonical operations. |
| `RigExecDemandClosure` | The operations one output needs, ascending, with the reason each is required. |
| `RigExecRequirementKind` | `Predecessor`, `Validity`, `Effect`, `History` — the edge that first reached an operation. |
| `RigExecOutputDemandEntry` | One index entry: a handle and the graph value its producer writes. |
| `RigExecOutputDemandIndex` | The per-compiled-rig index that resolves handles to producers and closures. |

## The rules

- **A time is a USD time code, and it is an identity.** `TimeCode()` carries
  `UsdTimeCode::GetValue()`: a count of frames on the stage's time code scale,
  never seconds and never a value with a rate baked in. A host holding seconds
  converts with the stage's rate before building the time. The default frame is
  its own value: it never equals numeric zero, and it sorts before every
  numeric frame, so grouping by time keeps it apart from frame 0. Subframe and
  negative codes survive exactly. Negative zero is stored as zero, so equal
  times compare and hash alike. `TryTimeCode` refuses NaN and infinity and
  leaves the destination untouched, so an invalid time can never enter a
  request.
- **A handle names the output, not one value of it.** `SameTarget` compares rig
  identity, compile epoch, semantic key, type and read phase; the value
  revision is excluded from the target comparison and from `Hash`, so a handle
  at a new revision still resolves to the same output. The type and read phase
  travel with the key so a consumer can validate a payload without re-deriving
  either from a stage.
- **A request is a set of per-time demands.** Outputs union inside the demand
  for one exact time; a demand for one time never carries an output asked for
  only at another. `RigExecRequestWants` answers whether one (output, time)
  pair was really requested.
- **Grouping checks every handle.** `RigExecRequestGroup` returns the one rig
  identity and compile epoch a request names, and fails when any handle of any
  demand disagrees. `RigExecRequestsCompatible` compares the groups: same rig
  identity, same compile epoch, and one value revision per output demanded at
  the same exact time. One representative handle never decides compatibility:
  a request whose second handle names another rig, epoch or revision is not
  compatible, however its first handle looks.
- **A union never invents a request.** `RigExecUnionRequests` merges handles
  only inside the demand for the same time, never across times, so merging
  never produces a Cartesian product of handles and times that no host asked
  for. A target demanded at one time at two revisions is refused: the merged
  request comes back empty with a diagnostic, and the caller dispatches the
  two groups separately. An explicit request outranks a speculative one.
- **Every packet carries its status and its whole generation key.** An owner
  never infers a status from an empty payload: `Ready` means the payload
  matches the handle at the requested time, `Invalidated` means a captured
  input or generation moved, `Unsupported` means the rig cannot name or produce
  that output, and `Failed` means the owner's dispatch or callback failed. The
  service answers a request from a packet only when program, input, preview
  and history revision all match the generation the request belongs to; no
  field is optional, and `RigExecOutputPacket::Answers` checks the handle, the
  time, the generation key and the payload layout together.
- **The payload layout is declared once, here.** `Scalar` is one double,
  `Matrix4x4Row` is 16 doubles in row-major order with USD row-vector
  semantics (a point transforms as `p' = p * M`), and `Point3` is three
  doubles, x y z. A handle's `type` carries the payload spelling
  (`RigExecPayloadText`), which scene lowering maps from the USD value type;
  the payload is not the attribute's storage form. `RigExecPayloadByteCount`
  computes and overflow-checks the size before any allocation, and
  `RigExecValidatePayload` requires the bytes to be exactly
  `elementCount * elementBytes` — never short, never with trailing bytes — and
  a declared layout. `RigExecPayloadView` reads one element at a time with
  `memcpy` and bounds-checks against the declared element count: the byte lease
  carries no alignment guarantee, so nothing reinterprets it as a `double`
  array. A partial packet is never a full `RigPose`.
- **Requirements come from declared edges, never from a walk.** A closure is
  the graph predecessors of the producer plus the operations reachable through
  the validity edges (a read is meaningful only after that operation ran), the
  effect edges (`RigExecOpCallbacks::skipEffects`) and the history edges of
  windowed operations (`rigExecGraph/windowHistory.h`). An operation's kind is
  the first edge that reaches it. Closures are ascending, so a closure is
  ready to hand to the executor as an `active` mask; `RigExecClosureIsClosed`
  checks the predecessor-closure the mask requires.
- **The index is built once per compiled rig version,** before any dispatch.
  It refuses edge lists that do not match the operation count, edge targets
  that name an absent operation, and entries with no semantic key. An output
  whose value has no canonical producer is reported `Unsupported` rather than
  answered by a stage-side fallback.

## Driving it

`RigExecOutputDemandIndex::ResolveRequest` turns a request into the index
slots it names, refusing a handle the index does not know instead of dropping
it; `Union` and `ActiveMask` then give the operations one dispatch runs for
those slots. `DemandedBy` answers the reverse question, which outputs need a
canonical operation — the lookup invalidation and cache publication use to
decide what to re-evaluate. The mask feeds `RigExecOpCallbacks::active`
directly: operations it marks 0 are never candidates, and history ticks run
only the operations a windowed operation depends on.
