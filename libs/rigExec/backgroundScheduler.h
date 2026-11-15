// RigExec background scheduler: the frame-job queue and the generation fence
// in front of it.
// Frame jobs warm the per-frame cache on a small pool of below-normal OS
// priority workers, each job running the baked serial executor end to end
// under a frozen serial scope. Workers never call Work*/TBB: a kernel-level
// parallel call would run its tasks on the shared arena at normal priority,
// leaking the work back past the priority boundary at exactly the moment it
// should hold. The pool only warms: the playhead always evaluates live on the
// calling thread, so the playhead frame is never enqueued here. Background
// order is scrub neighbors (+-1..N of the playhead) first, then the
// nearest-frame-first sweep of the surrounding range.
// Warming is triggered by edit-commit (drag release / value commit), not by
// idle detection, with an idle signal as a secondary trigger. There are no
// timers inside the scheduler: both triggers are explicit calls from the
// imaging/registry layer. Any new edit bumps the rig's generation and cancels
// its in-flight jobs: queued jobs are purged promptly, a job checks its
// generation before it runs, and the frozen worker rechecks again before it
// publishes -- a mismatch at any fence drops the result. A dropped job frees
// its inputs promptly; the frame it would have warmed simply evaluates live
// when asked.
// Explicit output admission is the other half of the queue: a host asked for
// these outputs at this exact frame, so the job runs ahead of every warm job
// and stays eligible when speculative warming is off (warm-off, or
// RIGEXEC_ENABLE_PARALLEL_EVAL=0). It is keyed by the exact demand identity in
// rigExecGraph/outputDemand.h -- the rig, the exact output time (the default
// frame is its own time, never numeric zero) and the whole captured
// generation -- never by the rounded (rig, time) key warming coalesces on, so
// two requests differing in program, input, preview or history revision are
// two jobs. Admission reserves the job slot and the bytes the request pins
// before anything is queued, and releases both on every terminal path.
// Foreground host evaluation stays outside this pool: a host that needs the
// answer on its own thread evaluates live; the pool runs work whose result is
// published behind the same fences and read later.
// Switches: RIGEXEC_FRAME_CACHE={on,off,warm-off}, default on,
// where off restores exact current behavior and warm-off keeps cache reads
// while disabling background fill; RIGEXEC_ENABLE_PARALLEL_EVAL=0 likewise
// disables background fill while reads are still served. When fill is off
// every Enqueue declines and every trigger enqueues nothing, so the caller
// evaluates live -- the fail-closed answer -- while the generation fence
// keeps working: edits still bump, tokens still compare.
#ifndef RIGEXEC_BACKGROUND_SCHEDULER_H
#define RIGEXEC_BACKGROUND_SCHEDULER_H

#include "profiler.h"

#include "rigExecGraph/outputDemand.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/timeCode.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Default background worker count: two below-normal workers drain the
/// default scrub neighborhood without contending with the UI thread.
constexpr int kRigExecBackgroundSchedulerDefaultWorkers = 2;

/// Most workers a scheduler ever runs. Past four the pool contends with the
/// UI thread it exists to stay out of the way of; the constructor clamps to
/// this.
constexpr int kRigExecBackgroundSchedulerMaxWorkers = 4;

/// Default scrub-neighbor radius: +-8 (16 neighbors), sampled in one burst at
/// enqueue and drained in the background by the default workers.
constexpr int kRigExecFrameCacheDefaultNeighborRadius = 8;

/// Default per-rig in-flight cap: queued plus running jobs for one rig. One
/// full default commit burst is 16 neighbors plus a 64-frame sweep; 96 holds
/// that burst plus one more neighbor set while the first drains, and bounds
/// the queued input vectors however fast
/// commits arrive. Enqueue past the cap declines and the caller evaluates
/// live.
constexpr size_t kRigExecBackgroundSchedulerDefaultPerRigJobs = 96;

/// Default per-rig cap on explicit output requests in flight (queued plus
/// running). It is separate from the warm cap: queued warming can neither
/// block a frame a host asked for nor be crowded out by one request burst.
constexpr size_t kRigExecBackgroundSchedulerDefaultExplicitPerRigJobs = 64;

/// Default total bytes the pool may hold reserved for explicit requests
/// across every rig: the values each job closes over plus the payloads it
/// publishes. Admission past the cap declines and the caller answers the
/// request live, so the reservation is a bound, never a swap.
constexpr size_t kRigExecBackgroundSchedulerDefaultExplicitBytes =
    (size_t(64) << 20);

/// A rig's edit generation. Bumped by CancelGeneration on every rig-affecting
/// edit; a job enqueued under an older one is stale and its result is
/// dropped. Starts at zero for a rig the scheduler has never seen.
/// A rig's edit generation. Bumped by CancelGeneration on every rig-affecting
/// edit; a job enqueued under an older one is stale and its result is
/// dropped. Starts at zero for a rig the scheduler has never seen.
using RigExecFrameGeneration = uint64_t;

/// A per-time publish fence token. Scoped cancellation
/// (CancelGenerationTimes) purges queued jobs for affected times WITHOUT
/// bumping the generation -- old and requeued jobs would then share one
/// generation token -- so the purge assigns each affected time a fresh
/// token from the rig's monotonic counter. A warm request carries the
/// token current at enqueue time, and the publish fence requires a token
/// match beside the generation check: an old job for a requeued time
/// drops on mismatch and can never overwrite the new result, while
/// unaffected times keep their tokens and their running jobs publish
/// normally. Zero is the never-purged token.
using RigExecWarmFenceToken = uint64_t;

/// Warming order. Lower runs first; the playhead itself is never enqueued.
enum class RigExecWarmPriority {
    /// A scrub neighbor of the playhead (+-1..N).
    Neighbor = 0,
    /// The edit-commit sweep of the surrounding range.
    Sweep = 1,
};

/// The RIGEXEC_FRAME_CACHE switch.
enum class RigExecFrameCacheMode {
    /// Warming on, reads served. The default.
    On,
    /// Exact current behavior: no warming, no reads. The scheduler's half of
    /// off is declining every Enqueue and enqueueing nothing from triggers;
    /// reads are the publish fence's half.
    Off,
    /// Reads served, background fill off. The scheduler's half is the same
    /// decline as Off.
    WarmOff,
};

/// Parses a RIGEXEC_FRAME_CACHE value: "on", "off", "warm-off". Anything else
/// -- including empty -- answers On, the default.
RigExecFrameCacheMode RigExecParseFrameCacheMode(const std::string &value);

/// Reads RIGEXEC_FRAME_CACHE from the environment, live on every call, so a
/// host can flip warming without restarting. Unset answers On. An
/// unrecognized value warns once per process and answers On.
RigExecFrameCacheMode RigExecFrameCacheModeFromEnvironment();

/// The background-fill gate as a pure function of its two inputs: fill runs
/// only when the frame-cache mode is On AND parallel evaluation is enabled.
/// RIGEXEC_ENABLE_PARALLEL_EVAL=0 therefore disables background fill while
/// cache reads are still served, exactly like warm-off.
bool RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode mode,
                                     bool parallelEvalEnabled);

/// The background-fill gate from the environment: the RIGEXEC_FRAME_CACHE
/// mode (live) and RigExecParallelEvaluationEnabled (the library's cached
/// switch). Enqueue and the triggers consult this on every call.
bool RigExecBackgroundWarmingEnabled();

/// The most phase marks one warm job may stamp: a start plus one end per
/// phase.
constexpr size_t kRigExecWarmPhaseMarks = 4;

/// Clock reads a running warm job stamps for its own phases, owned by the
/// worker frame that runs it. The job writes only integers and
/// string-literal names; the scheduler records them as one profiler event
/// when the job finishes, so no profiler lock or string building happens
/// inside the job.
struct RigExecWarmPhaseTimings {
    /// NowUs() marks; phase i spans marks[i] to marks[i + 1].
    uint64_t marks[kRigExecWarmPhaseMarks] = {};
    /// Phase names, string literals only (never freed, never copied).
    const char *names[kRigExecWarmPhaseMarks - 1] = {};
    size_t markCount = 0;

    /// Stamps the start, or ends phase \p name. Ignored when full.
    void Mark(const char *name = nullptr)
    {
        if (markCount >= kRigExecWarmPhaseMarks) {
            return;
        }
        if (markCount > 0) {
            names[markCount - 1] = name;
        }
        marks[markCount++] = RigExecProfiler::NowUs();
    }
};

/// Microsecond clock a scheduler stamps its queue events with: the sampling
/// budget's millisecond stop, the queue-wait stamps and the run stamps all
/// read through it. An empty clock means the profiler's steady clock. A test
/// injects a deterministic clock so those stops and waits come from supplied
/// readings instead of wall time. The clock is fixed at construction, so no
/// thread can swap it out while a job runs, so an injected clock must be safe
/// to read from any worker, and it is a leaf: it must not call back into the
/// scheduler, since some stamps are taken under the scheduler mutex.
using RigExecSchedulerClock = std::function<uint64_t()>;

/// One warming request: evaluate \p time for \p rig, publishing into the
/// frame cache behind the generation fence.
struct RigExecWarmRequest {
    SdfPath rig;
    UsdTimeCode time = UsdTimeCode::Default();
    RigExecWarmPriority priority = RigExecWarmPriority::Neighbor;
    RigExecFrameGeneration generation = 0;
    /// The per-time fence token current at enqueue time, stamped by the
    /// scheduler on insert. The publish fence requires a match (see
    /// IsWarmRequestCurrent); callers never set this.
    RigExecWarmFenceToken fenceToken = 0;
    /// Set by the scheduler on the copy it hands a running job, and only
    /// while its profiler records; null otherwise and on every queued copy.
    /// A job that times its phases marks here and never calls the profiler.
    RigExecWarmPhaseTimings *timings = nullptr;
    /// Scheduler-clock stamps the scheduler owns: the moment the job entered
    /// the queue, and the moment a worker popped it. Queue wait is poppedUs -
    /// enqueuedUs; run time is the terminal stamp minus poppedUs.
    uint64_t enqueuedUs = 0;
    uint64_t poppedUs = 0;
};

/// One ran job's publish disposition, reported by the work instead of void
/// so completion accounting splits on what actually happened: a "ran" job
/// that declined inside does not count as a publish.
enum class RigExecWarmOutcome {
    /// Evaluated and published into the frame cache.
    Published,
    /// Declined to publish: the evaluation declined, the pose was invalid,
    /// the cache refused the entry -- or the work threw (the scheduler
    /// warns and lands this, fail-closed). The frame evaluates live when
    /// asked.
    DeclinedInvalid,
    /// Dropped by the publish fence: an edit landed mid-run. The frame
    /// requeues under the live generation.
    DeclinedGeneration,
};

/// The per-job evaluation a worker runs: serial, from frozen inputs only,
/// never Work/TBB. The scheduler invokes it under a frozen serial scope --
/// every kernel variant the job reaches takes its serial form on the worker
/// thread, whatever the process-wide switch says -- after a generation check
/// at start; the work itself (production: RigExecEvaluateFrozen plus publish)
/// rechecks before publish and reports the disposition. Sampled inputs travel
/// inside the closure, built on the UI thread at enqueue time: the worker
/// receives values, never live state, stage handles, or queries.
using RigExecWarmWork =
    std::function<RigExecWarmOutcome(const RigExecWarmRequest &)>;

/// Why a factory built no work for a frame. Reasoned skips count the
/// aggregate declined plus their per-reason breakout; unreasoned skips
/// (trigger shape, not warmability) skip silently, as before.
enum class RigExecWarmSkipReason {
    /// Work was built, or the skip carries no warmability verdict.
    None,
    /// The rig refused the freeze: no snapshot, no job. The detail is the
    /// session's named freeze cause.
    FreezeRefused,
    /// The frame could not be sampled faithfully. The detail names the
    /// sampler gate.
    Unsampleable,
    /// No baked program, so the rig evaluates dynamically and never
    /// warms.
    D7Exempt,
};

/// One factory call's answer: the work (empty when the frame skips) plus
/// the skip reason. Implicitly constructible from bare work so test and
/// bench factories -- which carry no verdict -- keep compiling unchanged;
/// an unreasoned empty build skips silently.
struct RigExecWarmFactoryResult {
    RigExecWarmWork work;
    RigExecWarmSkipReason skip = RigExecWarmSkipReason::None;
    std::string detail;
    RigExecWarmFactoryResult() = default;
    RigExecWarmFactoryResult(RigExecWarmWork built) : work(std::move(built))
    {
    }
};

/// Builds one frame's work on the calling (UI) thread at enqueue time: sample
/// the frame's input vector, close over it, and return the closure the worker
/// runs. An empty build skips the frame -- it could not be sampled
/// faithfully -- and the frame evaluates live when asked. Called only while
/// the generation is current and the fill gate is open, and skipped without
/// sampling for a time already queued (best-effort under concurrent
/// triggers: a time that queues between the check and the sample merges
/// after the sample, freeing it).
using RigExecWarmJobFactory =
    std::function<RigExecWarmFactoryResult(UsdTimeCode)>;

/// Per-trigger sampling budget: bounds the UI-thread factory sampling one
/// trigger performs (the in-flight cap does not bound one tick's sampling).
/// Already-queued merges consume none -- they sample nothing -- so a tick
/// over warm ground is nearly free however large the sweep vector is.
struct RigExecWarmSamplingBudget {
    /// Max factory invocations per trigger. Unlimited by default.
    size_t maxFactoryInvocations = std::numeric_limits<size_t>::max();
    /// Millisecond stop, measured from \p startUs. Unlimited by default.
    double maxMs = std::numeric_limits<double>::infinity();
    /// Stamped by the caller (the registry stamps after burst prep, so the
    /// stop bounds factory sampling, not prep). Zero measures from the
    /// trigger's entry.
    uint64_t startUs = 0;
};

/// Which class of background work a queue entry belongs to. One class runs
/// ahead of the other: what a host asked for before what warming guessed.
enum class RigExecBackgroundWorkClass {
    /// An explicit output request from a host. Ahead of every warm job.
    Explicit = 0,
    /// Speculative warming: scrub neighbors and the surrounding sweep.
    Warm = 1,
};

/// The exact identity an explicit admission is keyed and coalesced by: the
/// rig, the exact output time, and the whole captured generation the result
/// must come from. Every field counts, so two requests differing in any one
/// of them are two jobs and never merge; the scheduler never rounds a time or
/// widens a key. The default frame is its own time and sorts before every
/// numeric frame, so it never coalesces with frame zero.
struct RigExecExplicitJobKey {
    SdfPath rig;
    RigExecOutputTime time;
    RigExecOutputGeneration generation;

    friend bool operator==(const RigExecExplicitJobKey &a,
                           const RigExecExplicitJobKey &b)
    {
        return a.rig == b.rig && a.time == b.time &&
               a.generation == b.generation;
    }
    friend bool operator!=(const RigExecExplicitJobKey &a,
                           const RigExecExplicitJobKey &b)
    {
        return !(a == b);
    }
    /// Total order for the scheduler's maps: rig path, then the exact time
    /// (default first), then the generation fields.
    bool operator<(const RigExecExplicitJobKey &other) const
    {
        if (rig != other.rig) {
            return rig < other.rig;
        }
        if (!(time == other.time)) {
            return time < other.time;
        }
        if (generation.programIdentity != other.generation.programIdentity) {
            return generation.programIdentity <
                   other.generation.programIdentity;
        }
        if (generation.inputRevision != other.generation.inputRevision) {
            return generation.inputRevision < other.generation.inputRevision;
        }
        if (generation.previewRevision != other.generation.previewRevision) {
            return generation.previewRevision <
                   other.generation.previewRevision;
        }
        return generation.historyRevision < other.generation.historyRevision;
    }
};

/// One explicit request: produce \p outputs for \p rig at the exact \p time
/// from the captured \p generation, publishing them behind the existing
/// fences. The scheduler stamps fenceToken on the copy it queues; the caller
/// fills the rest, grouped as outputDemand.h groups a request: one rig
/// identity and compile epoch per admission, with the compile epoch carried as
/// generation.programIdentity so a recompile is a different key.
struct RigExecExplicitRequest {
    /// The rig, spelled as the same path the warm path uses for it.
    SdfPath rig;
    /// The exact frame: the default frame or one finite USD time code.
    RigExecOutputTime time;
    /// The whole captured generation the result must come from: the compiled
    /// program (its compile epoch), the sampled inputs, the preview state and
    /// the history window. A partial generation never identifies a result.
    RigExecOutputGeneration generation;
    /// The outputs to produce at this time, unioned inside this one demand.
    std::vector<RigExecOutputHandle> outputs;
    /// The rig's edit generation this request belongs to. A request under a
    /// superseded one declines: the compiled rig moved under it.
    RigExecFrameGeneration editGeneration = 0;
    /// The fence token current for this request's time at admission, stamped
    /// by the scheduler. The publish fence requires a match beside the
    /// generation check (see IsExplicitRequestCurrent); callers never set this.
    RigExecWarmFenceToken fenceToken = 0;
    /// Bytes this request pins: the captured values its work closes over plus
    /// the payloads it will publish, counted with RigExecPayloadByteCount
    /// before admission. Reserved on accept, released on every terminal path.
    size_t reservationBytes = 0;

    /// The key this request is keyed and coalesced by.
    RigExecExplicitJobKey Key() const
    {
        RigExecExplicitJobKey key;
        key.rig = rig;
        key.time = time;
        key.generation = generation;
        return key;
    }
};

/// The publish disposition of an explicit job, reported the same way warm
/// work reports it: a job that declined inside does not count as a publish.
using RigExecExplicitOutcome = RigExecWarmOutcome;

/// The per-job work an explicit request runs, under the same rules as warm
/// work: serial, from values captured at admission only, never Work/TBB, no
/// live stage, evaluator or imaging-registry access, and no thread hops. The
/// work publishes into the caller's coverage store behind the fences and
/// reports the disposition; it never returns values to the scheduler.
using RigExecExplicitWork =
    std::function<RigExecExplicitOutcome(const RigExecExplicitRequest &)>;

/// The seam at the end of an accepted explicit admission: it names the job
/// that admission queued, after the queue, counters and reservation record it
/// and after the lanes are notified, with no scheduler lock held -- so the
/// job's own terminal state can land while the seam runs. Null by default
/// (one branch, no cost). Tests install one; production never does.
using RigExecExplicitAdmissionReturnHook =
    std::function<void(const RigExecExplicitJobKey &)>;

/// One per-frame queue transition, reported to the transition hook: the
/// registry records it into the shared warm-frame index, which is what the
/// per-frame query API reads. Delivered under the scheduler lock, in the
/// same critical section as the counter it explains, so the hook must be
/// fast, leaf-only, and never re-enter the scheduler or the registry.
enum class RigExecWarmTransitionKind {
    /// Enqueued (direct or trigger): the frame is warming-from-queue.
    Queued,
    /// Popped by a worker (or the manual drain): warming-from-running.
    Running,
    /// Ran to completion: \p outcome says what happened.
    Finished,
    /// Purged by CancelGeneration.
    Canceled,
    /// Purged by NotePlaybackAdvanced.
    Shed,
    /// Popped, then an edit landed before the pre-run check.
    DroppedStale,
    /// Still queued at shutdown.
    DroppedAtShutdown,
};

/// \p outcome is meaningful for Finished only.
struct RigExecWarmTransition {
    SdfPath rig;
    double timeValue = 0.0;
    RigExecWarmTransitionKind kind = RigExecWarmTransitionKind::Queued;
    RigExecWarmOutcome outcome = RigExecWarmOutcome::Published;
    /// Which class the move belongs to: warming or an explicit request.
    RigExecBackgroundWorkClass workClass = RigExecBackgroundWorkClass::Warm;
    /// True when timeValue names no frame: the request is for the default
    /// frame, which has no numeric value and never means frame zero.
    bool timeIsDefault = false;
};

/// Per-frame queue visibility for the warm-frame index. Null by default
/// (no reporting cost beyond one branch); the registry installs one that
/// records into shared state. Set before the first trigger -- workers
/// report through it -- and never from inside it.
using RigExecWarmTransitionHook =
    std::function<void(const RigExecWarmTransition &)>;

/// Sets the calling thread to below-normal OS priority for background
/// warming. Returns whether the platform honored it; when it cannot -- no API
/// on the platform, or the call failed -- it warns once per process and the
/// worker runs at normal priority (the logged no-op fallback). Idempotent.
bool RigExecSetBackgroundThreadPriority();

/// Names the RigExecSetBackgroundThreadPriority backend on this platform:
/// "SetThreadPriority" (Windows), "sched-batch" (Linux), "qos-utility"
/// (macOS), or "none" (logged no-op fallback). Never null. For the
/// bench report and for diagnosing whether priority applied; gate tests never
/// assert OS priority values.
const char *RigExecBackgroundPriorityBackend();

/// Why an Enqueue was refused. Every declined Enqueue lands in exactly one
/// of these; a factory skip that carried a reason keeps its own breakout.
enum class RigExecWarmDeclineReason {
    /// The cache mode is off, or parallel evaluation is disabled.
    FillGateClosed,
    /// A newer edit generation exists for the rig.
    StaleGeneration,
    /// Default, NaN or infinite frame time.
    UnusableTime,
    /// No work closure was handed over.
    EmptyWork,
    /// The rig already holds its share of the queue.
    PerRigCap,
    /// The scheduler is shutting down.
    Stopping,
};

/// Lifetime counters. Every counter only moves forward except across
/// destruction. Warm jobs that enter the queue balance exactly: queued-ever ==
/// published + declinedInvalid + declinedGeneration + droppedStale +
/// canceled + shed + droppedAtShutdown + queuedDepth + running, where
/// queued-ever is accepts minus merges -- coalesced and upgraded merges
/// accept without queueing -- and declined counts every Enqueue refusal.
/// Explicit admissions balance under the same shape against their own
/// counters: accepted minus coalesced == explicitPublished +
/// explicitDeclinedInvalid + explicitDeclinedGeneration + explicitDroppedStale
/// + explicitCanceled + explicitDroppedAtShutdown + explicitQueuedDepth +
/// explicitRunning. The two classes share one queue, so queuedDepth and
/// running cover both; the explicit split is broken out below.
/// completed ("ran", kept for compat) == published + declinedInvalid +
/// declinedGeneration; a throwing job warns and lands declinedInvalid, in
/// either class.
/// declined == the six Enqueue reasons plus the three reasoned factory
/// skips, and queuedNeighbor + queuedSweep + explicitQueuedDepth covers every
/// queued job.
struct RigExecBackgroundSchedulerStats {
    /// Jobs queued and waiting for a worker.
    size_t queuedDepth = 0;
    /// Jobs popped and executing right now.
    size_t running = 0;
    /// Jobs whose work ran to completion. (Whether the work published is the
    /// work's own publish fence to decide; completion means "ran".)
    size_t completed = 0;
    /// Ran jobs that evaluated and published into the frame cache.
    size_t published = 0;
    /// Ran jobs that declined to publish: the evaluation declined, the pose
    /// was invalid, the cache refused the entry -- or the work threw. The
    /// frame evaluates live when asked.
    size_t declinedInvalid = 0;
    /// Ran jobs dropped by the publish fence: an edit landed mid-run. The
    /// frame requeues under the live generation.
    size_t declinedGeneration = 0;
    /// Jobs dropped by the pre-run generation check: popped, then an edit
    /// landed before the check. Rare by construction -- CancelGeneration
    /// purges queued jobs first -- but the race is real, so the fence stands.
    size_t droppedStale = 0;
    /// Queued jobs purged by CancelGeneration.
    size_t canceled = 0;
    /// Queued sweep jobs purged by NotePlaybackAdvanced.
    size_t shed = 0;
    /// Enqueues merged into an already-queued job at the same priority.
    size_t coalesced = 0;
    /// Enqueues merged into an already-queued sweep job by promoting it to
    /// neighbor. Disjoint from coalesced.
    size_t upgraded = 0;
    /// Enqueue refusals: fill gate closed, stale generation, unusable time,
    /// empty work, per-rig cap, or shutdown -- plus reasoned factory skips,
    /// which never entered the queue either (broken out below).
    size_t declined = 0;
    /// Reasoned factory skips by cause, each also counted in declined: the
    /// rig refused the freeze, the frame was unsampleable, the rig has no
    /// baked program.
    size_t declinedFreezeRefused = 0;
    size_t declinedUnsampleable = 0;
    size_t declinedD7Exempt = 0;
    /// Enqueue refusals broken out by reason; with the three factory-skip
    /// counts above, these add up to declined exactly.
    size_t declinedFillGate = 0;
    size_t declinedStaleGeneration = 0;
    size_t declinedUnusableTime = 0;
    size_t declinedEmptyWork = 0;
    size_t declinedPerRigCap = 0;
    size_t declinedStopping = 0;
    /// Trigger-path stops: a burst ended before its whole wave reached the
    /// queue. A closed fill gate is an Enqueue decline per frame instead, so
    /// it never appears here; these stops are not Enqueue refusals and stay
    /// out of the balance.
    size_t triggerStops = 0;       // sum of the three causes below
    size_t triggerStopsStale = 0;  // generation moved, or the pool stopped
    size_t triggerStopsBudget = 0; // invocation or millisecond budget hit
    size_t triggerStopsCap = 0;    // the per-rig cap closed the burst
    /// Queued jobs dropped by the destructor. Running jobs finish; pending
    /// jobs are dropped and their frames evaluate live when asked.
    size_t droppedAtShutdown = 0;
    /// Factory invocations across triggers: sampling attempts, whether they
    /// enqueued, merged, or skipped. Not part of the balance -- it counts
    /// trigger work, not queue jobs -- but the per-trigger delta proves a
    /// trigger held its sampling budget.
    size_t factoryInvocations = 0;
    /// Highest queue depth and running count the scheduler has ever held.
    size_t maxQueuedDepth = 0;
    size_t maxRunning = 0;
    /// The queue split by priority right now, over the warm jobs:
    /// queuedNeighbor + queuedSweep == queuedDepth - explicitQueuedDepth. An
    /// upgrade moves a job from one bucket to the other.
    size_t queuedNeighbor = 0;
    size_t queuedSweep = 0;
    /// Jobs accepted into the queue, by the priority they were accepted at.
    size_t enqueuedNeighbor = 0;
    size_t enqueuedSweep = 0;
    /// Queue wait (enqueue to pop) for every job a worker popped, and run
    /// time (pop to the terminal stamp) for the jobs that ran, in
    /// microseconds on the scheduler's clock.
    uint64_t totalQueueWaitUs = 0;
    uint64_t maxQueueWaitUs = 0;
    uint64_t totalRunUs = 0;
    uint64_t maxRunUs = 0;
    /// Explicit admissions the scheduler accepted, coalescings included. A
    /// coalescing is an accept that queued nothing, like a warm merge, so
    /// accepted minus coalesced is the jobs that reserved a slot.
    size_t explicitAccepted = 0;
    /// Admissions merged into an already-queued job under the identical key.
    /// Only an identical rig, exact time and whole generation coalesce.
    size_t explicitCoalesced = 0;
    /// Admission refusals: no work, no output, a mixed provenance group, a
    /// stale edit generation, the per-rig job cap, the byte cap, or shutdown.
    size_t explicitDeclined = 0;
    /// Refusals caused by the byte cap, also counted in explicitDeclined.
    size_t explicitDeclinedBytes = 0;
    /// Explicit jobs whose work ran to completion, split by disposition.
    size_t explicitCompleted = 0;
    size_t explicitPublished = 0;
    size_t explicitDeclinedInvalid = 0;
    size_t explicitDeclinedGeneration = 0;
    /// Explicit jobs popped, then dropped by the pre-run generation check.
    size_t explicitDroppedStale = 0;
    /// Queued explicit jobs purged by CancelGeneration,
    /// CancelGenerationTimes, CancelExplicitJob or ResetRig.
    size_t explicitCanceled = 0;
    /// Queued explicit jobs dropped by the destructor.
    size_t explicitDroppedAtShutdown = 0;
    /// Explicit jobs queued and running right now.
    size_t explicitQueuedDepth = 0;
    size_t explicitRunning = 0;
    /// Bytes currently reserved by queued plus running explicit jobs. Zero
    /// with an empty, idle queue: every terminal path releases.
    size_t explicitReservedBytes = 0;
};

/// The frame-job queue. Thread-safe; the lock is held only across token and
/// queue edits, never across input sampling or evaluation.
///
/// One scheduler serves every rig and both work classes: generations and
/// in-flight counts are per rig inside it, but the queue itself is one global
/// priority map (explicit requests before warm neighbors before sweep, FIFO
/// within a priority), so one rig's neighbors preempt another rig's sweep.
/// Within the explicit class the pool is fair per rig -- the k-th queued
/// request of one rig runs ahead of the (k+1)-th, and a rig's burst cannot
/// starve another rig's request. Harmless at one session. The destructor
/// stops the pool -- running jobs finish, pending jobs of both classes are
/// dropped, and every explicit byte reservation is released -- and joins
/// every worker, so no Enqueue, AdmitExplicitOutput, trigger, or ForTesting
/// call may run concurrently with destruction.
class RigExecBackgroundScheduler {
public:
    /// Starts \p workerCount below-normal workers (clamped to
    /// [0, kRigExecBackgroundSchedulerMaxWorkers]). Zero runs no threads:
    /// the manual mode, where queued jobs wait for RunNextQueuedForTesting /
    /// DrainQueueForTesting on the calling thread. Production uses the
    /// default.
    /// \p clock stamps the queue events (queue waits, run times, and the
    /// sampling budget's millisecond stop). An empty clock reads the profiler
    /// steady clock.
    explicit RigExecBackgroundScheduler(
        int workerCount = kRigExecBackgroundSchedulerDefaultWorkers,
        RigExecSchedulerClock clock = nullptr);
    ~RigExecBackgroundScheduler();

    RigExecBackgroundScheduler(const RigExecBackgroundScheduler &) = delete;
    RigExecBackgroundScheduler &
    operator=(const RigExecBackgroundScheduler &) = delete;

    /// The rig's current generation: the token its next job is enqueued
    /// under. Zero for a rig the scheduler has never seen.
    RigExecFrameGeneration CurrentGeneration(const SdfPath &rig) const;

    /// Queued plus running jobs for the rig (0 for a rig with none in
    /// Queued plus running warm jobs for the rig (0 for a rig with none in
    /// flight). The registry's activation consults this to skip its commit
    /// cancel when idle -- an idle bump retires nothing and would orphan
    /// the initial evaluation's memos, which stamped the pre-commit
    /// generation. Explicit output requests have their own count, beside
    /// this one: see ExplicitInFlightCount.
    size_t InFlightCount(const SdfPath &rig) const;

    /// Cancels the rig's in-flight jobs of both classes by bumping its
    /// generation and purging its queued jobs, whose inputs and byte
    /// reservations free promptly. Every rig-affecting edit calls this; the
    /// edit-commit trigger then enqueues fresh warm jobs and the subscription
    /// service re-admits fresh explicit requests under the new token. Running
    /// jobs are not preempted: the pre-run and publish fences drop their
    /// results instead.
    void CancelGeneration(const SdfPath &rig);
    /// Drops the rig's fence tokens and queued jobs of both classes -- but
    /// not its generation, which the warm index keys completions by -- as on
    /// deactivation, so a reactivated rig never inherits stale tokens. Its
    /// explicit job ordinals go with it; queued byte reservations release with
    /// the jobs, and a running job releases its own when it finishes. Global
    /// stats stand.
    void ResetRig(const SdfPath &rig);

    /// Cancels the rig's queued jobs for \p times only, WITHOUT bumping
    /// the generation (scoped cancellation): the outcome-driven
    /// notice path calls this for patch/stamp-bump edits, where the compiled rig
    /// stands and only affected times requeue. Each affected time gets a
    /// fresh per-time fence token from the rig's monotonic counter, so an
    /// old job for a requeued time drops at the publish fence on token
    /// mismatch; unaffected times keep their tokens and their running
    /// jobs publish normally. A scoped edit moves the default frame's inputs
    /// too, so the rig's default-frame token (the one an explicit request for
    /// the default frame carries) is re-issued by the same counter and its
    /// queued job is purged with the affected times. Queued jobs for other
    /// times and other rigs are untouched. Answers how many jobs were purged.
    /// Global CancelGeneration stays for compiled rig moves only.
    size_t CancelGenerationTimes(const SdfPath &rig,
                                 const std::vector<UsdTimeCode> &times);

    /// Drops the rig's one queued explicit job keyed exactly by \p key -- the
    /// demand-retirement operation a subscription does when its last client
    /// stops asking for a frame. No generation bump and no fence token moves:
    /// the compiled rig did not move, so every other job stays queued and its
    /// running jobs publish normally. Warm jobs, and explicit jobs under any
    /// other key, are untouched, and the purged job's byte reservation comes
    /// back. A job that already popped cannot be purged; its caller-side
    /// liveness flag is what drops its answer. Answers whether a queued job
    /// left the queue.
    bool CancelExplicitJob(const RigExecExplicitJobKey &key);

    /// The rig's current per-time fence token for \p time: zero when the
    /// time was never scoped-purged, otherwise the token the purge
    /// assigned. A requeued job is stamped with this at enqueue.
    RigExecWarmFenceToken CurrentFenceToken(const SdfPath &rig,
                                            UsdTimeCode time) const;

    /// Whether a warm request is still current: \p generation is the
    /// rig's current one AND \p fenceToken matches the time's current
    /// token (zero matches only a never-purged time). The publish fence
    /// asks this; the pre-run fence asks IsGenerationCurrent alone (a
    /// popped job's token can only have been superseded by a purge whose
    /// requeue has not run yet -- still droppable at publish, never
    /// runnable twice).
    bool IsWarmRequestCurrent(const SdfPath &rig,
                              RigExecFrameGeneration generation,
                              UsdTimeCode time,
                              RigExecWarmFenceToken fenceToken) const;

    /// The rig's currently queued warm times, in index order. Explicit
    /// output requests are keyed by the exact demand time, not by a
    /// UsdTimeCode: see QueuedExplicitKeys and InFlightExplicitKeys.
    std::vector<UsdTimeCode> QueuedTimes(const SdfPath &rig) const;

    /// The rig's queued and running warm times, unique and in time order.
    /// Scoped edit retirement includes these even when a first-time warm
    /// has not published any provenance or completion yet.
    std::vector<UsdTimeCode> InFlightTimes(const SdfPath &rig) const;

    /// The publish fence mutex: worker publish and cancel/clear
    /// share it. The background completion path holds it across the
    /// fence-check (generation + per-time token) and the cache insert, and
    /// the fenced clear holds it across generation-bump + queue-purge +
    /// cache-clear + index-reset -- so a running job can never pass its
    /// check before the bump and insert after the clear. Lock order: fence
    /// outermost, then the scheduler mutex, then cache shards, then the
    /// warm index -- never inverted. Acquire it holding neither the
    /// scheduler mutex nor the registry mutex.
    std::mutex &FenceMutex() const
    {
        return _fenceMutex;
    }

    /// Whether \p generation is still the rig's current one. A worker asks
    /// before it runs and the frozen entry point asks again before publish.
    bool IsGenerationCurrent(const SdfPath &rig,
                             RigExecFrameGeneration generation) const;

    /// Enqueues \p time for warming at \p priority, running \p work on a
    /// worker. Returns false -- without enqueueing, counting declined -- when
    /// the fill gate is closed, \p generation is stale, \p time is default or
    /// non-finite, \p work is empty, the rig is at its in-flight cap, or the
    /// pool is stopping: either way the caller evaluates live instead.
    ///
    /// A time already queued for the rig coalesces instead of queueing twice:
    /// returns true -- warming is coming -- counting coalesced, or promoting
    /// a queued sweep to neighbor and counting upgraded. First-wins: the
    /// queued work stands and the new closure is destroyed promptly. The
    /// coalescing key is the warm one, (rig, time value): it never merges an
    /// explicit request, and an explicit request never merges into a warm job.
    ///
    /// The playhead frame must never be enqueued; the pool only warms. The
    /// triggers enforce this (they know the playhead); this primitive trusts
    /// its caller.
    bool Enqueue(const SdfPath &rig, UsdTimeCode time,
                 RigExecWarmPriority priority,
                 RigExecFrameGeneration generation, RigExecWarmWork work);

    /// The primary warming trigger: call on drag release / value commit, on
    /// the UI thread, after CancelGeneration for the edit. Enqueues the
    /// scrub neighbors +-1..\p neighborRadius of \p playhead closest-first
    /// (-1, +1, -2, +2, ...) as Neighbor, then \p sweepTimes in order as
    /// Sweep, skipping the playhead itself wherever it appears, skipping
    /// default sweep entries, and building each job's work through \p factory
    /// on this thread. Returns how many jobs were enqueued.
    ///
    /// Stops early -- returning the count so far -- when \p generation goes
    /// stale mid-trigger (an edit landed; its own commit follows), when the
    /// rig hits its in-flight cap, when the sampling \p budget runs out
    /// (invocations or millisecond stop), or when a frame's factory returns
    /// empty (that frame is skipped, the trigger continues). A stale
    /// generation, a closed fill gate, or a default playhead enqueues
    /// nothing and never calls the factory. The gate is read once per
    /// burst: a flip mid-burst lands on the next trigger. \p neighborRadius
    /// <= 0 enqueues no neighbors.
    size_t OnEditCommitted(
        const SdfPath &rig, UsdTimeCode playhead,
        RigExecFrameGeneration generation,
        const std::vector<UsdTimeCode> &sweepTimes,
        RigExecWarmJobFactory factory,
        int neighborRadius = kRigExecFrameCacheDefaultNeighborRadius,
        RigExecWarmSamplingBudget budget = RigExecWarmSamplingBudget());

    /// The secondary warming trigger: call from the frame loop when the UI is
    /// idle. Enqueues \p sweepTimes in order as Sweep -- neighbors belong to
    /// the commit trigger -- with the same skipping, factory, cap, staleness,
    /// budget, and gate rules as OnEditCommitted. Returns how many jobs were
    /// enqueued.
    size_t
    OnIdle(const SdfPath &rig, UsdTimeCode playhead,
           RigExecFrameGeneration generation,
           const std::vector<UsdTimeCode> &sweepTimes,
           RigExecWarmJobFactory factory,
           RigExecWarmSamplingBudget budget = RigExecWarmSamplingBudget());

    /// Admits one explicit output request to the same pool, running \p work
    /// on a worker ahead of every warm job. This is the subscription path:
    /// what a host asked for at an exact frame, not a guess. It is eligible in
    /// every frame-cache mode and whatever RIGEXEC_ENABLE_PARALLEL_EVAL says,
    /// because those switches gate speculative warming alone; a host that
    /// needs the result on its own thread still evaluates live rather than
    /// admitting the frame it is waiting on.
    ///
    /// The request is keyed by rig, exact output time and the whole captured
    /// generation. An identical key already queued coalesces: returns true --
    /// the work is coming -- counting explicitCoalesced, first-wins, and the
    /// new closure is destroyed promptly. A request differing in any key field
    /// is a second job and never merges, so a rounded time, a recompiled
    /// program, or a moved input, preview or history revision can never be
    /// answered by another generation's job.
    ///
    /// Returns false -- reserving nothing and counting explicitDeclined --
    /// when \p work is empty, the request names no output or mixes rig
    /// identities or compile epochs across its handles, \p editGeneration is
    /// stale, the rig is at its explicit per-rig job cap, the request's bytes
    /// would pass the pool's byte cap (counted in explicitDeclinedBytes too),
    /// or the pool is stopping. On accept the request's reservationBytes are
    /// reserved until the job reaches a terminal state -- published, declined,
    /// dropped, purged, canceled or dropped at shutdown -- and the queued copy
    /// carries the current fence token.
    bool AdmitExplicitOutput(const RigExecExplicitRequest &request,
                             RigExecExplicitWork work);

    /// Whether an admitted explicit request is still current: its
    /// editGeneration is the rig's current one AND its fenceToken matches the
    /// token current for its exact time (the default frame has a token of its
    /// own, re-issued by scoped cancellation; zero matches only a never-purged
    /// time). The work asks this before it publishes, holding FenceMutex, and
    /// compares the generation key it captured against its own caller-side
    /// state: the scheduler fences the rig's edit and per-time generations,
    /// the caller's coverage store fences program, input, preview and history.
    bool IsExplicitRequestCurrent(const RigExecExplicitRequest &request) const;

    /// The token an explicit request for \p time is stamped with right now:
    /// the numeric time's per-time token, or the rig's default-frame token for
    /// the default frame. Zero when that time was never scoped-purged.
    RigExecWarmFenceToken
    CurrentExplicitFenceToken(const SdfPath &rig, RigExecOutputTime time) const;

    /// The rig's queued explicit keys, ascending in key order.
    std::vector<RigExecExplicitJobKey>
    QueuedExplicitKeys(const SdfPath &rig) const;

    /// The rig's queued and running explicit keys, unique and in key order:
    /// what a scoped edit must account for beyond the warm times.
    std::vector<RigExecExplicitJobKey>
    InFlightExplicitKeys(const SdfPath &rig) const;

    /// Queued plus running explicit requests for the rig, bounded by the
    /// explicit per-rig cap. Independent of the warm count.
    size_t ExplicitInFlightCount(const SdfPath &rig) const;

    /// Bytes reserved by queued plus running explicit requests, in total and
    /// for one rig. Zero once the queue has drained.
    size_t ReservedExplicitBytes() const;
    size_t ReservedExplicitBytesFor(const SdfPath &rig) const;

    /// Sheds queued sweep work the playhead already passed during active
    /// (forward) playback: purges the rig's queued Sweep jobs at or before \p
    /// playhead, whose frames the live path serves now, and returns how many
    /// were purged. Neighbor jobs are untouched -- the frames ahead of the
    /// playhead still warm the imminent scrub path -- and the standing
    /// neighbor-before-sweep order is what deprioritizes the sweep that
    /// remains. A default playhead purges nothing.
    size_t NotePlaybackAdvanced(const SdfPath &rig, UsdTimeCode playhead);

    /// The per-rig in-flight cap for warm jobs (queued plus running).
    /// Defaults to kRigExecBackgroundSchedulerDefaultPerRigJobs. Reducing it
    /// declines new warm jobs until the rig drains below it; it never purges,
    /// and it does not gate explicit admission -- see SetExplicitPerRigJobCap.
    void SetPerRigJobCap(size_t cap);
    size_t GetPerRigJobCap() const;

    /// The per-rig cap on explicit requests in flight (queued plus running).
    /// Defaults to kRigExecBackgroundSchedulerDefaultExplicitPerRigJobs.
    /// Reducing it declines new requests until the rig drains below it; it
    /// never purges.
    void SetExplicitPerRigJobCap(size_t cap);
    size_t GetExplicitPerRigJobCap() const;

    /// The byte cap the pool holds reserved for explicit requests across
    /// every rig. Defaults to kRigExecBackgroundSchedulerDefaultExplicitBytes.
    /// Reducing it declines new requests whose reservation would pass it,
    /// including one already queued and still running; it never purges.
    void SetExplicitByteCap(size_t cap);
    size_t GetExplicitByteCap() const;

    /// Installs the per-frame transition hook (null uninstalls). The
    /// registry installs one recording into the shared warm-frame index;
    /// see RigExecWarmTransition for the delivery contract.
    void SetTransitionHook(RigExecWarmTransitionHook hook);

    /// Installs the explicit admission-return seam (null uninstalls). It runs
    /// on the admitting thread just before AdmitExplicitOutput returns true,
    /// outside the scheduler lock, so a test can force the admitted job's
    /// terminal disposition before its own admission returns. Set before the
    /// first admission and never from inside it.
    void
    SetAdmissionReturnHookForTesting(RigExecExplicitAdmissionReturnHook hook);

    /// The worker count this scheduler runs (after clamping). Zero is the
    /// manual mode.
    int GetWorkerCount() const
    {
        return _workerCount;
    }

    RigExecBackgroundSchedulerStats Stats() const;

    /// The scheduler's profiler: queue-depth samples and purge points record
    /// here. Off by default; when off, each sample site costs one branch.
    const RigExecProfiler &GetProfiler() const
    {
        return _profiler;
    }
    RigExecProfiler *MutableProfiler()
    {
        return &_profiler;
    }

    /// The scheduler clock's current reading, in microseconds. A scheduler
    /// built without a clock reads the profiler steady clock.
    uint64_t ClockNowUs() const;

    /// Blocks until no job is queued or running. Event-driven, not timed:
    /// returns when the last completion, drop, purge, or shed lands.
    void WaitUntilIdle();

    /// Pops the highest-priority queued job -- explicit requests before warm
    /// neighbors before sweep, FIFO within a priority, per-rig round-robin
    /// within the explicit class -- and runs it on the calling thread under
    /// the same serial scope and pre-run fence a worker applies. Returns false
    /// when the queue is empty. For deterministic tests with zero workers;
    /// production warms on the pool. A stale pop counts droppedStale (or
    /// explicitDroppedStale) and returns true: the queue moved.
    bool RunNextQueuedForTesting();

    /// Runs queued jobs of both classes on the calling thread until the queue
    /// is empty. Returns how many jobs' work ran (stale pops are dropped, not
    /// run). For deterministic tests with zero workers.
    size_t DrainQueueForTesting();

private:
    // One queued job of either class. Exactly one class's request and work are
    // set: a warm job carries the UI-sampled closure keyed by (rig, time
    // value); an explicit job carries the demand identity, its outputs, its
    // byte reservation and the work the service supplied.
    struct _QueuedJob {
        RigExecBackgroundWorkClass workClass = RigExecBackgroundWorkClass::Warm;
        RigExecWarmRequest request;
        RigExecExplicitRequest explicitRequest;
        RigExecWarmWork warmWork;
        RigExecExplicitWork explicitWork;
    };
    // What a popped job needs after its closures have been freed: the queue's
    // bookkeeping, the fence check, and the reservation to release.
    struct _JobIdentity {
        SdfPath rig;
        // The warm coalescing key; meaningless when timeIsDefault.
        double timeValue = 0.0;
        bool timeIsDefault = false;
        RigExecBackgroundWorkClass workClass = RigExecBackgroundWorkClass::Warm;
        RigExecFrameGeneration generation = 0;
        size_t reservationBytes = 0;
        // The exact key for an explicit job: its coalescing and running index.
        RigExecExplicitJobKey explicitKey;
        // Queue-event stamps copied from the queued job: the shared pool
        // measures the queue wait and run time of every job it pops, in
        // either class.
        uint64_t enqueuedUs = 0;
        uint64_t poppedUs = 0;
    };
    // Queue order: work class first (an explicit host request ahead of every
    // speculative warm job), then rank, then enqueue sequence. Warm rank is
    // the priority (neighbor before sweep), so the commit trigger's
    // closest-first enqueue order stands within a priority. Explicit rank is
    // the job's ordinal within its rig, which makes the pool take the rigs in
    // turn instead of draining one rig's whole burst first.
    struct _QueueKey {
        int workClass = int(RigExecBackgroundWorkClass::Warm);
        uint64_t rank = 0;
        uint64_t sequence = 0;
        bool operator<(const _QueueKey &other) const
        {
            if (workClass != other.workClass) {
                return workClass < other.workClass;
            }
            if (rank != other.rank) {
                return rank < other.rank;
            }
            return sequence < other.sequence;
        }
    };

    /// Stamps for queue insert, pop and the terminal state.
    uint64_t _NowUs() const
    {
        return _clock ? _clock() : RigExecProfiler::NowUs();
    }
    // Notes the queue's shape after every mutation, so the depth and running
    // maxima cannot miss a change. Call with _mutex held.
    void _RecordQueueStateLocked();
    // Records one Enqueue refusal in the total and its reason split. Call
    // with _mutex held.
    void _NoteDeclineLocked(RigExecWarmDeclineReason reason);
    // Why a trigger burst ended before its whole wave reached the queue.
    enum _StopCause { Stale, Budget, Cap };
    // Records a burst stop in the trigger-stop split. Call with _mutex held.
    void _NoteTriggerStopLocked(_StopCause cause);

    void _WorkerMain();
    // The identity of a queued job, for reporting, bookkeeping and the
    // terminal state.
    static _JobIdentity _IdentityOf(const _QueuedJob &job);
    // The rig a queued job belongs to, whichever class it is.
    static const SdfPath &_JobRig(const _QueuedJob &job);
    // Pops the next queued job under the lock; the caller runs it outside the
    // lock. When \p wait, blocks until a job arrives or the pool stops,
    // otherwise answers at once on an empty queue. Answers whether *job owns
    // a job; a false from the waiting form means "stopping, exit".
    bool _PopJob(_QueuedJob *job, bool wait);
    // Runs a popped job outside the lock: the pre-run generation fence,
    // then the work under a frozen serial scope, capturing its outcome
    // (threw maps to declinedInvalid, warn preserved), then the terminal
    // state. Answers whether the work ran. Shared by the workers and the
    // ForTesting drain so both apply exactly the same fence and scope.
    bool _ExecutePoppedJob(_QueuedJob *job);
    // Reports one transition to the hook, if any. Call with _mutex held:
    // the hook runs inside the scheduler lock, so it must be leaf-only.
    void _ReportTransitionLocked(
        const _JobIdentity &identity, RigExecWarmTransitionKind kind,
        RigExecWarmOutcome outcome = RigExecWarmOutcome::Published);
    // Retires a queued job under the lock: leaves the coalescing index,
    // reports \p kind, releases an explicit byte reservation, drops the rig's
    // in-flight count, and increments \p counter. The caller moves the job --
    // and the closures it owns -- out first and lets them die after the lock
    // closes.
    void _RetireQueuedJobLocked(const _QueuedJob &job,
                                RigExecWarmTransitionKind kind);
    // Records a popped job's terminal state under the lock: \p ran true
    // counts a completion (the work already ran, outside the lock) and
    // splits it on \p outcome; false counts a stale drop (the work never
    // ran, \p outcome ignored). Either way the rig's in-flight count falls,
    // an explicit job's reservation is released, and idleness is
    // re-signalled.
    // A ran job's \p timings, when given, are recorded as one profiler
    // event after the lock drops.
    void _FinishJob(const _JobIdentity &identity, bool ran,
                    RigExecWarmOutcome outcome,
                    const RigExecWarmPhaseTimings *timings = nullptr);
    // The time's current fence token: the sidecar's entry, or zero when the
    // time was never scoped-purged. Call with _mutex held.
    RigExecWarmFenceToken _CurrentFenceTokenLocked(const SdfPath &rig,
                                                   double timeValue) const;
    // The token an explicit request for \p time carries: the numeric time's
    // per-time token, or the rig's default-frame token. Call with _mutex held.
    RigExecWarmFenceToken
    _CurrentExplicitFenceTokenLocked(const SdfPath &rig,
                                     RigExecOutputTime time) const;
    // Releases \p bytes of explicit reservation for \p rig, from the rig's
    // total and the pool's. Call with _mutex held.
    void _ReleaseReservationLocked(const SdfPath &rig, size_t bytes);
    // The rig's queued plus running explicit count, and the same after a
    // pop, a completion or a purge. Call with _mutex held.
    size_t _ExplicitInFlightLocked(const SdfPath &rig) const;
    void _ChangeExplicitInFlightLocked(const SdfPath &rig, int delta);
    // Merges a request for an already-queued (rig, time): coalesces, or
    // promotes a queued sweep to neighbor keeping its sequence. Answers
    // whether the time was queued (and merged). Call with _mutex held.
    bool _MergeIfQueuedLocked(const std::pair<SdfPath, double> &coalesceKey,
                              RigExecWarmPriority priority);
    // One trigger time's outcome: enqueued (count it), skipped (already
    // queued or unsampleable -- the trigger continues), or stop (stale
    // generation, per-rig cap, or sampling budget -- the trigger ends).
    enum class _TriggerOutcome { Enqueued, Skipped, Stop };
    // One trigger's sampling-budget ledger, threaded through the per-frame
    // path (re-entrancy-safe: a factory that triggers again gets its own).
    struct _BudgetLedger {
        const RigExecWarmSamplingBudget *budget = nullptr;
        size_t invocations = 0;
        uint64_t startUs = 0;
    };
    // Enqueues one trigger time at \p priority, sampling its work through
    // \p factory outside the lock (sampling is slow; the lock never spans
    // it) and deciding staleness, coalescing, and cap under it. \p ledger
    // counts the invocation against the trigger's budget.
    _TriggerOutcome _EnqueueTriggerTime(const SdfPath &rig, UsdTimeCode time,
                                        RigExecWarmPriority priority,
                                        RigExecFrameGeneration generation,
                                        const RigExecWarmJobFactory &factory,
                                        _BudgetLedger *ledger);
    // The sweep core shared by OnEditCommitted and OnIdle: \p sweepTimes in
    // order as Sweep, skipping the playhead and default entries.
    size_t _EnqueueSweepTimes(const SdfPath &rig, UsdTimeCode playhead,
                              RigExecFrameGeneration generation,
                              const std::vector<UsdTimeCode> &sweepTimes,
                              const RigExecWarmJobFactory &factory,
                              _BudgetLedger *ledger);

    mutable std::mutex _mutex;
    // The publish fence mutex (see FenceMutex): outermost in the lock
    // order, ahead of _mutex.
    mutable std::mutex _fenceMutex;
    std::condition_variable _cv;
    // Fixed at construction: workers and triggers read it without a lock.
    const RigExecSchedulerClock _clock;
    std::map<SdfPath, RigExecFrameGeneration> _generations;
    std::map<_QueueKey, _QueuedJob> _queue;
    // Warm coalescing index: (rig, time value) -> queue key. Queued jobs
    // only; a popped job leaves the index, so a re-enqueue while it runs
    // queues honestly beside it (idempotent warming, not a duplicate run).
    std::map<std::pair<SdfPath, double>, _QueueKey> _queuedByRigAndTime;
    // Explicit coalescing index: the exact demand key -> queue key. Queued
    // jobs only, and keyed by the whole key: no rounding, no widening, so a
    // different generation is a different entry.
    std::map<RigExecExplicitJobKey, _QueueKey> _queuedExplicitByKey;
    // Running warm jobs remain visible to scoped edit retirement after they
    // leave the queue. Counts cover overlapping jobs at the same time.
    std::map<std::pair<SdfPath, double>, size_t> _runningByRigAndTime;
    // Running explicit jobs, same purpose, keyed exactly.
    std::map<RigExecExplicitJobKey, size_t> _runningExplicitByKey;
    // Per-time publish fence tokens: (rig, time value) ->
    // current token, assigned by CancelGenerationTimes from the rig's
    // monotonic counter below. Absent means never-purged (token zero).
    std::map<std::pair<SdfPath, double>, RigExecWarmFenceToken> _fenceTokens;
    // The default frame's token, same counter: a scoped edit moves the
    // default frame's inputs too, so an explicit request for the default
    // frame is fenced by this instead of by a numeric value it does not have.
    std::map<SdfPath, RigExecWarmFenceToken> _defaultFenceTokens;
    std::map<SdfPath, RigExecWarmFenceToken> _fenceCounters;
    // In-flight warm jobs per rig: queued plus popped-and-executing. Drives
    // the warm cap.
    std::map<SdfPath, size_t> _inFlight;
    // In-flight explicit requests per rig, and the ordinal the next one gets:
    // the explicit queue rank, which is what makes the pool fair between rigs.
    std::map<SdfPath, size_t> _explicitInFlight;
    std::map<SdfPath, uint64_t> _explicitOrdinal;
    // Bytes reserved by queued plus running explicit requests, per rig and in
    // total. Reserved at admission, released on every terminal path.
    std::map<SdfPath, size_t> _reservedBytesByRig;
    size_t _reservedBytes = 0;
    std::vector<std::thread> _workers;
    uint64_t _sequence = 0;
    size_t _running = 0;
    size_t _completed = 0;
    size_t _published = 0;
    size_t _declinedInvalid = 0;
    size_t _declinedGeneration = 0;
    size_t _droppedStale = 0;
    size_t _canceled = 0;
    size_t _shed = 0;
    size_t _coalesced = 0;
    size_t _upgraded = 0;
    size_t _declined = 0;
    size_t _declinedFillGate = 0;
    size_t _declinedStaleGeneration = 0;
    size_t _declinedUnusableTime = 0;
    size_t _declinedEmptyWork = 0;
    size_t _declinedPerRigCap = 0;
    size_t _declinedStopping = 0;
    size_t _triggerStops = 0;
    size_t _triggerStopsStale = 0;
    size_t _triggerStopsBudget = 0;
    size_t _triggerStopsCap = 0;
    size_t _declinedFreezeRefused = 0;
    size_t _declinedUnsampleable = 0;
    size_t _declinedD7Exempt = 0;
    size_t _factoryInvocations = 0;
    size_t _droppedAtShutdown = 0;
    size_t _maxQueuedDepth = 0;
    size_t _maxRunning = 0;
    size_t _enqueuedNeighbor = 0;
    size_t _enqueuedSweep = 0;
    uint64_t _totalQueueWaitUs = 0;
    uint64_t _maxQueueWaitUs = 0;
    uint64_t _totalRunUs = 0;
    uint64_t _maxRunUs = 0;
    size_t _explicitAccepted = 0;
    size_t _explicitCoalesced = 0;
    size_t _explicitDeclined = 0;
    size_t _explicitDeclinedBytes = 0;
    size_t _explicitCompleted = 0;
    size_t _explicitPublished = 0;
    size_t _explicitDeclinedInvalid = 0;
    size_t _explicitDeclinedGeneration = 0;
    size_t _explicitDroppedStale = 0;
    size_t _explicitCanceled = 0;
    size_t _explicitDroppedAtShutdown = 0;
    size_t _explicitRunning = 0;
    RigExecWarmTransitionHook _transitionHook;
    RigExecExplicitAdmissionReturnHook _admissionReturnHook;
    size_t _perRigCap = kRigExecBackgroundSchedulerDefaultPerRigJobs;
    size_t _explicitPerRigCap =
        kRigExecBackgroundSchedulerDefaultExplicitPerRigJobs;
    size_t _explicitByteCap = kRigExecBackgroundSchedulerDefaultExplicitBytes;
    int _workerCount = 0;
    bool _stopping = false;
    // The profiler's lock is a leaf: recording sites run under _mutex, and
    // the profiler never calls back into the scheduler.
    RigExecProfiler _profiler;
};

} // namespace rigExec

#endif // RIGEXEC_BACKGROUND_SCHEDULER_H
