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

## Running what was asked for

`RigExecBakedProgram::RunRequested` (with `RigExecRequestedRunReport` and
`RigExecValueMaterialization` in `libs/rigExec/bakedProgram.h`) is the
baked program's answer to a demand closure. It runs the same frame `Run`
runs — the same prologue, the same executor, schedule, kernels and skip
effects — restricted to the closure's operations. It is additive:
`Run(UsdTimeCode, RigExecRigPose*)` keeps the whole-rig behavior, and there
is no second evaluator, no fallback walk and no operation named by text.

| Declaration | Says |
| --- | --- |
| `RunRequested(time, closure, report, error)` | Run the frame over `closure` only. Nothing is published: a partial run is not a full pose. |
| `RigExecRequestedRunReport` | Which operations ran, how many clusters they closed, and the per-value facts below. |
| `RigExecValueMaterialization` | Per canonical value the closure writes: `produced` (this run's body wrote it), `reused` (clean retained storage stood), `wasAbsent` (the arena held no published value when the run began), and its `revision` after the run. |

- **The closure is checked, not assumed.** The operations must belong to this
  program's canonical graph, and `RigExecClosureIsClosed` must accept the
  closure: a mask that is not predecessor closed is refused before any body
  runs, and the program is left as it was. A step the compiler set aside over
  a cycle is not in the canonical graph, so no closure can name its body, and
  `RequestFullRun`'s whole-program request keeps the SCC authority the
  compiler retains.
- **An operation outside the closure is not asked about.** It is not a seed,
  not a volatile run, not a successor, and not a candidate, so its body does
  not run and its skip effects do not run either: the cone skip's bookkeeping
  (a geometry revision's dirty flags, a head chain's status) is the
  bookkeeping of an operation that was asked and found clean. Its input keys
  stay behind the generation it last ran in, which is what makes it change on
  the run that asks for it. `skipEffects` is filtered to the mask for the
  run.
- **What a run leaves out is owed, not trusted.** Every operation outside the
  closure goes on a debt list, and the next run seeds the ones whose values
  this arena has never published and offers the rest as candidates. An
  operation can therefore never be read as clean on a key it has never
  compared, which is what a sequence of partial runs — or a partial run
  followed by a whole one — needs to land on the whole run's answer.
- **A missing value is not a clean one.** Inside the closure, an operation
  whose write list holds storage this arena never published is seeded however
  clean its inputs look, so a request that stops partway leaves no slot a
  later request would answer from. Selectors are still conservative: a blend
  weight's endpoints never prune a branch on their own, and the closure the
  index declares — with its validity, effect and history edges — is what
  runs.
- **The facts are the run's, not an inference.** Declared writes alone prove
  nothing, so the report is gathered after the join from the executor's own
  record and the value states: what ran, and per value whether this run
  produced it, whether clean retained storage stands for it, and whether it
  was missing when the run began. An extractor reads a value only when the
  closure names its producer and the facts say it materialized.

## Running it on a lane

The same mechanism runs a request behind the owner thread. The mask,
the run's facts and the extraction are library entries shared by the live
program and a detached lane's clone: `RigExecBakedBeginRequestedRun`,
`RigExecBakedEndRequestedRun` and `RigExecBakedFillRequestedRunReport`
(`libs/rigExec/bakedOpGraph.h`), and `RigExecBakedExtractRequestedOutputs`
(`libs/rigExec/bakedProgramImpl.h`). A lane installs the mask the owner
thread closed, runs the shared executor over its own arena, gathers the
facts after the join, answers the outputs from that arena, and clears the
mask — same prologue, schedule, kernels and skip effects as the lane's
whole-frame job, and nothing published: a partial run is not a full pose.

| Declaration | Says |
| --- | --- |
| `RigExecMakeFrozenRequestedRun(catalog, demand, captured, out, error)` | Owner thread: resolve one exact frame's demand into the job a lane runs — the union of the named handles' closures, the handles, and the captured generation. False, having dispatched nothing, when the catalog does not know a handle or an output has no producer: the caller recaptures. |
| `RigExecFrozenRequestedRun` | That job: closure, output handles, captured generation, and the catalog of the epoch that minted the handles. Inputs and history ride the same `RigExecFrameInputs` a warming job carries. |
| `RigExecEvaluateFrozenRequested(context, inputs, requested, runner, scheduler, rig, trace)` | Run the job over `requested`'s closure under the entry point's checks and both generation fences, and answer its outputs. `RigExecFrozenRequestedResult` says which outputs answered, and `error` says why a run declined or was dropped. |
| `RigExecMakeProductionRequestedStepRunner()` | The serial runner behind that entry: the lane's clone, the job's inputs, the masked executor, the extraction, no publication. |

- **Capture stays on the owner thread.** The catalog, the demand index and the
  closure read compiled tables; the lane sees a shared_ptr to the frozen
  snapshot, the sampled vector and the job. No stage, evaluator, registry or
  live handle reaches a worker, and the lane's mutable storage stays its own.
- **A refusal is reported, never papered over.** An open closure, a catalog of
  another epoch, a lane that cannot place the inputs, a retired generation or
  an unadmitted frame answers with no packets and a reason; an output the run
  did not materialize answers `Pending` with a reason even when the arena
  holds another generation's bytes for it. There is no fallback evaluation:
  the owner recaptures and re-admits, or reports the miss.
- **The trace says so.** With a report, the entry point's run leaves the
  ordinary operation trace, so a caller can check that no body outside the
  closure ran and that an output outside the closure was not asked about.

## Where the entries come from

`RigExecDiscoverOutputs` (`libs/rigExec/outputCatalog.h`) builds the catalog of
outputs one compiled program epoch can name, straight from the baked program's
binding tables. Each entry is a semantic address — target, public computation
and phase, spelled as a `RigExecValueAddress` — bound to the graph value its
canonical producer writes, plus the retained storage an extractor reads after
the graph joins.

- **Only retained storage is exported.** Property results, provider base and
  final matrices, frame records, chain bases, published chain points and a
  revision fuse's publication record have an enumerator; scratch packets,
  candidates, chunk and staging slots do not, and discovery refuses an entry
  that binds one.
- **The payload layout is checked before the entry is indexed.** `double` goes
  with a scalar, `matrix4d` with a transform, `point3d[]` with a point array; an
  entry whose kind and layout disagree fails discovery.
- **GPU-only point chains are exported as `Unsupported` with a reason.** There
  is no CPU fallback for them, and no evaluator answers them specially.
- **A missing output is refused, not guessed.** An address with no retained
  producer resolves `Unsupported`; a handle naming another rig identity or an
  older compile epoch resolves `Invalidated`. `Resolve` reports both before any
  dispatch, so no host reads a stage to fill in what the rig does not export.

Scene lowering maps an authored value to one of those spellings in
`RigExecResolveOutputDescriptor` (`libs/rigExecScene/outputDescriptors.h`),
which reads stage identity and type metadata only and authors nothing.

## Background admission

A subscription service runs the work behind a request on the existing
background pool. `libs/rigExec/backgroundScheduler.h` declares the admission
it calls; the pool itself does not change, and no second queue appears.

| Declaration | Says |
| --- | --- |
| `RigExecBackgroundWorkClass` | `Explicit` or `Warm`: the pool runs a host request ahead of speculative warming. |
| `RigExecExplicitRequest` | One admission: the rig, the exact `RigExecOutputTime`, the whole `RigExecOutputGeneration`, the outputs of that one demand, the rig edit generation, and the bytes the request pins. |
| `RigExecExplicitJobKey` | The exact identity the admission is keyed and coalesced by: rig, exact time, whole generation. |
| `AdmitExplicitOutput` | Reserves a job slot and the request's bytes, then queues the work. Returns false, having reserved nothing, when the request has no work or no output, mixes rig identities or compile epochs, is under a stale edit generation, passes the per-rig job cap or the pool's byte cap, or the pool is stopping. |
| `RigExecExplicitWork` | The job body: serial, from values captured at admission, no live stage, evaluator or registry access, reporting a `RigExecExplicitOutcome` the way warm work reports its disposition. |
| `IsExplicitRequestCurrent`, `CurrentExplicitFenceToken` | The scheduler's half of the publish fence: the rig's edit generation and the time's fence token. The default frame has a token of its own, re-issued by `CancelGenerationTimes`, because a scoped edit moves its inputs too. |
| `ExplicitInFlightCount`, `ReservedExplicitBytes`, `ReservedExplicitBytesFor`, `QueuedExplicitKeys`, `InFlightExplicitKeys` | What a rig has in flight, in jobs and in reserved bytes, and the exact keys behind them. |
| `SetExplicitPerRigJobCap`, `SetExplicitByteCap` | The two bounds, independent of the warm job cap: how many requests one rig may hold, and how many bytes the pool may hold reserved across every rig. |

- **Explicit demand is not warming.** `RIGEXEC_FRAME_CACHE=off`, `warm-off`,
  and `RIGEXEC_ENABLE_PARALLEL_EVAL=0` close speculative warming; they do not
  close admission, because a frame a host asked for is not a guess. A host
  that needs the answer on its own thread still evaluates live: the pool runs
  work whose result is published behind the fences and read later.
- **Coalescing is exact.** Two admissions merge only when the rig, the exact
  time and every generation field agree. The default frame never merges with
  frame zero, a subframe step is its own job, and a different program, input,
  preview or history revision is a different job. The warm path's rounded
  `(rig, time value)` key never merges an explicit request, in either
  direction.
- **Fairness and bounds.** Warm rank is the priority (neighbors before
  sweep); explicit rank is the job's ordinal within its rig, so two rigs take
  turns instead of one rig's burst running first. Every queued job carries its
  reservation, and every terminal path releases it -- published, declined
  inside, threw, dropped stale, purged by `CancelGeneration`,
  `CancelGenerationTimes` or `ResetRig`, or dropped at shutdown -- so
  `ReservedExplicitBytes` is zero once the queue has drained. Byte counts come
  from `RigExecPayloadByteCount`, which refuses an overflowing element count
  before anything allocates.
- **The counters split by class.** `RigExecBackgroundSchedulerStats` keeps the
  warm equations as they were and adds the `explicit*` counters beside them;
  the transition hook tags every move with its class and says when the time
  names no frame at all.

## The subscription lifecycle

`libs/rigExec/outputSubscription.h` puts a bounded lifecycle on that admission.
A host subscribes outputs at exact frames; the service captures one frame's job
on the owner thread, admits it, stores the answer as a coverage record in the
frame cache, and hands packets back at `PollResults`. One service per pool and
cache: the host owns both and keeps them alive until the pool has joined its
lanes.

| Declaration | Says |
| --- | --- |
| `Subscribe(rig, request, receipt)` | Validate the request against the rig's current catalog, then demand its frames. The receipt carries the refusal reason and the outputs the catalog can name but cannot produce. |
| `RigExecSubscriptionHandle` | A live subscription: movable, not copyable. Destroying it closes the subscription, so a host cannot leak one by losing the handle. |
| `UpdateSubscription(subscription, request)` | Replaces a subscription's request. The new request is validated before the old demand becomes unreferenced. |
| `PollResults(subscription)` | What stands for each demanded (output, frame) pair, in request order: a packet copied from a coverage record, or the status that stopped it and the reason. Polling evaluates and dispatches nothing. |
| `PumpCallbacks()` | Runs what the pool completed since the last pump, delivers the queued events, and re-dispatches the demand a retired generation or a launched-mask expansion left owed. |
| `SetCompletionCallback` | Installs the owner-thread completion callback (null uninstalls). |
| `Close`, `CloseAll` | Ends one subscription, or every client, with the same release for each demand that no other client holds. |
| `RigExecSubscriptionHost` | What the service needs from the host: the pool, the cache, a catalog hook, a capture hook, and the lane's step runner. |
| `RigExecSubscriptionCapture` | One exact frame's captured job: the sampled vector, the frozen owners, the captured generation, and the byte estimate admission is bounded by. |
| `RigExecEstimateCaptureBytes` | A coarse container-size estimate for one sampled vector, which is what the capture charges the coverage budget. |

- **The owner thread owns the lifecycle.** `Subscribe`, `UpdateSubscription`,
  `PollResults`, `PumpCallbacks` and `Close` run there, and the capture hook is
  the only live-USD read in the path. A capture holds no stage, evaluator,
  registry or live handle, which is what lets the job be queued. The state lock
  is never held across the pool, a cache call, a fence or a run.
- **Demand is shared; work is not.** Live subscriptions group by rig, exact
  time and compile epoch, and their outputs union into that group's demand.
  Two clients on one frame get one job, one coverage record, and one lease per
  payload, and a second subscription for an already-answered frame captures and
  admits nothing.
- **A launched mask is fixed at admission.** An expansion noticed while the job
  runs is owed the next launch, asked for as the union; the dispatched mask is
  never rewritten.
- **Unreferenced demand leaves.** The last client to close a frame retires its
  queued job and returns its byte and coverage reservations. A coverage lease
  another owner holds is never touched, and the record stays for the next host.
- **The publish fence runs twice**, before the bodies and again before the
  publish, so an edit that lands while a job runs publishes nothing: the frame
  answers `Invalidated`, and the owner's next pump re-captures under the live
  fence token. A scoped edit retires only the times it names.
- **Answers are pinned, not copied.** A poll that answers from a record pins
  that payload for as long as the subscription lives; the pin is shared between
  clients that ask for the same output, and closing releases it. Byte bounds
  refuse before allocation: the cache's coverage budget and the pool's job and
  byte caps each decline in their own counter, and a refused admission returns
  its reservation.
- **Callbacks are queued, never run on a worker.** A completion lands at the
  owner's `PumpCallbacks`, and an event for a client that closed first is
  dropped.
