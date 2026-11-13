# Output subscriptions

A host that wants rig results — a Hydra reader, an export, a game adapter,
a bake, a preview — asks for **outputs at exact frames**. The contract that
connects that request to the evaluator lives in `libs/rigExecGraph/outputDemand.h`.
Scene lowering, frame-cache publication, bake, binary playback, imaging and
host adapters speak these types and may not declare their own time, handle,
status, packet, payload lease or closure.

The contract is USD-free, like the rest of `rigExecGraph`: compiled rig
identity travels as text plus a compile epoch, and a payload is an immutable
byte lease. Nothing here reads a stage. See
[evaluated versus baked rigs](../concepts/baked-vs-dynamic.md) for the one
loop these requests are answered by, and
[frames on demand](frames-on-demand.md) for the imaging reader that uses it.

## The types

| Type | Names |
| --- | --- |
| `RigExecOutputTime` | One frame: the context's default frame, or one exact finite time in seconds. |
| `RigExecOutputHandle` | One output of one compiled rig: rig identity, compile epoch, semantic key, payload type, read phase, value revision. |
| `RigExecOutputRequest` | What a host wants: handles and the exact times, with an explicit or speculative policy. |
| `RigExecOutputPacket` | One answered output: the request, the program that produced it, its input/preview/history revisions, a status with a reason, and the payload. |
| `RigExecOutputStatus` | `Pending`, `Ready`, `Invalidated`, `Unsupported`, `Failed`. |
| `RigExecOutputPayload` | A shared, immutable byte buffer plus the element count the producer declared. |
| `RigExecDemandEdges` | Declared validity, effect and history requirements between canonical operations. |
| `RigExecDemandClosure` | The operations one output needs, ascending, with the reason each is required. |
| `RigExecRequirementKind` | `Predecessor`, `Validity`, `Effect`, `History` — the edge that first reached an operation. |
| `RigExecOutputDemandEntry` | One index entry: a handle and the graph value its producer writes. |
| `RigExecOutputDemandIndex` | The per-compiled-rig index that resolves handles to producers and closures. |

## The rules

- **A time is an identity, not a number you can guess.** The default frame is
  its own value: it never equals numeric zero, and it sorts before every
  numeric frame, so grouping by time keeps it apart from frame 0. Subframe and
  negative offsets survive exactly. Negative zero is stored as zero, so equal
  times compare and hash alike. `TryNumeric` refuses NaN and infinity and
  leaves the destination untouched, so an invalid time can never enter a
  request.
- **A handle names the output, not one value of it.** `SameTarget` compares rig
  identity, compile epoch, semantic key, type and read phase; the value
  revision is excluded from the target comparison and from `Hash`, so a handle
  at a new revision still resolves to the same output. The type and read phase
  travel with the key so a consumer can validate a payload without re-deriving
  either from a stage.
- **Every packet carries its status.** An owner never infers a status from an
  empty payload: `Ready` means the payload matches the handle at the requested
  time, `Invalidated` means a captured input or generation moved, `Unsupported`
  means the rig cannot name or produce that output, and `Failed` means the
  owner's dispatch or callback failed.
- **A payload's element count is declared by the producer.** Consumers never
  re-derive it from the byte size. Copies share the bytes; a writer detaches
  before changing them, following the copy-on-write rule for published results.
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
- **Requests of one rig union before dispatch.** `RigExecUnionRequests`
  deduplicates handles and times and lets an explicit request outrank a
  speculative one. `RigExecRequestsCompatible` is the test for whether two
  requests may share a dispatch: same rig identity and compile epoch. A shared
  ancestor operation appears once in the union and is executed once, exactly
  as the compiled graph schedules it.

## Driving it

`RigExecOutputDemandIndex::Union` and `ActiveMask` give the operations one
dispatch runs for a set of outputs; `DemandedBy` answers the reverse question,
which outputs need a canonical operation — the lookup invalidation and cache
publication use to decide what to re-evaluate. The mask feeds
`RigExecOpCallbacks::active` directly: operations it marks 0 are never
candidates, and history ticks run only the operations a windowed operation
depends on.
