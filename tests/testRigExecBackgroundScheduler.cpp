// RigExecBackgroundScheduler, Stream C: the frame-job queue behind the
// generation fence.
// The fence tests from the Stream 0 skeleton stand unchanged -- per-rig
// tokens, monotonic bumps, independence, staleness -- and the stub's "a
// current job declines" test is replaced by the pool's own contract, one rule
// per test in the testRigExecStaticInputCache style:
//   * FENCE. Tokens, bumps, independence, stale refusal (kept from Stream 0).
//   * SWITCH. RIGEXEC_FRAME_CACHE parses to on/off/warm-off, the fill gate is
//     the mode AND parallel-eval, and a closed gate declines warming while the
//     fence keeps working.
//   * ORDER. Neighbors before sweep, FIFO within a priority, commit neighbors
//     closest-first -- all drained on the calling thread, so the sequences
//     are exact.
//   * COALESCE. Duplicate times merge (sweep promotes to neighbor), factory
//     sampling is skipped for queued times, merges never consume the cap.
//   * CAP. The per-rig in-flight cap declines past its limit, frees as jobs
//     run or purge, and never crosses rigs.
//   * CANCEL. CancelGeneration purges queued jobs promptly; a bump inside a
//     trigger stops it; running jobs keep their publish fence (Stream B).
//   * TRIGGER. OnEditCommitted enqueues neighbors then sweep and never the
//     playhead; OnIdle enqueues sweep only; both stop at staleness and cap.
//   * SHED. NotePlaybackAdvanced purges passed sweep jobs, keeps neighbors
//     and ahead frames, and frees the cap.
//   * POOL. Workers run every job exactly once under the frozen serial scope,
//     shutdown joins running jobs and drops pending ones, and the priority
//     backend reports honestly. Waits are event-driven (queue-empty, started,
//     released) -- no sleeps, no timing assertions, no wall clock.
//   * OUTCOME. Ran jobs split completed by disposition (published,
//     declined-invalid, declined-generation); threw lands
//     declined-invalid; completed still equals the three.
//   * HOOK. The transition hook reports every per-frame queue move --
//     queued, running, finished, canceled, shed, dropped-at-shutdown --
//     in order, on the manual drain.
//   * FAIRNESS. One worker preserves the pop order exactly (neighbors
//     closest-first, then sweep); under a two-worker flood both rigs
//     still complete every job and the books balance.
//   * EXPLICIT. Explicit output admission shares the pool: it runs ahead of
//     warming, is keyed exactly (rig, exact output time, whole captured
//     generation) and coalesces only on that key, stays eligible when
//     speculative warming is off, takes the rigs in turn, holds a bounded job
//     and byte reservation that every terminal path releases, and is fenced
//     by the same generation and per-time tokens -- with a token of its own
//     for the default frame.
//   * BUDGET. The sampling budget bounds factory invocations per trigger;
//     the millisecond stop binds sampling; already-queued merges consume
//     nothing; the unlimited default preserves the full burst shape.
//   * STAMPS. An injected clock makes queue wait and run time exact, the
//     decline reasons add up to declined, the depth and running maxima hold
//     the peak, and a stopped trigger burst says which cause stopped it.
// Two process modes: most tests force RIGEXEC_FRAME_CACHE=on (live-read) and
// run the warm path, skipping with a note when the ambient
// RIGEXEC_ENABLE_PARALLEL_EVAL=0 closes the gate -- that cached switch cannot
// be flipped in-process, so its off mode is covered by the fallback test plus
// the validation-plan runs of this binary under =0. The fallback test forces
// the gate closed and passes in every ambient mode.
#include "rigExec/backgroundScheduler.h"
#include "rigExec/frozenContext.h"
#include "rigExec/parallel.h"

#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

namespace {

// Sets RIGEXEC_FRAME_CACHE for the test's duration, restoring the ambient
// value -- or unsetting when it was unset -- on destruction.
class ScopedFrameCacheMode {
public:
    explicit ScopedFrameCacheMode(const char *value)
    {
        // Presence, not value: an unset variable restores to unset, and a
        // set-but-empty one restores to empty. (_dupenv_s on MSVC, where
        // getenv is deprecated.)
#if defined(_WIN32)
        char *ambient = nullptr;
        size_t length = 0;
        _had = (_dupenv_s(&ambient, &length, "RIGEXEC_FRAME_CACHE") == 0 &&
                ambient != nullptr);
        if (_had) {
            _saved = ambient;
            free(ambient);
        }
#else
        const char *ambient = std::getenv("RIGEXEC_FRAME_CACHE");
        _had = (ambient != nullptr);
        if (_had) {
            _saved = ambient;
        }
#endif
        TfSetenv("RIGEXEC_FRAME_CACHE", value);
    }
    ~ScopedFrameCacheMode()
    {
        if (_had) {
            TfSetenv("RIGEXEC_FRAME_CACHE", _saved);
        } else {
            TfUnsetenv("RIGEXEC_FRAME_CACHE");
        }
    }

private:
    bool _had = false;
    std::string _saved;
};

// One executed job, as the recording work observed it.
struct RunRecord {
    SdfPath rig;
    double time = 0.0;
    RigExecWarmPriority priority = RigExecWarmPriority::Sweep;
    RigExecFrameGeneration generation = 0;
    bool serialActive = false;
};

// Builds recording work and counting factories. Thread-safe: workers record
// while the test asserts afterwards on its own thread.
class Recorder {
public:
    RigExecWarmWork Work()
    {
        return [this](const RigExecWarmRequest &request) {
            RunRecord record;
            record.rig = request.rig;
            record.time = request.time.GetValue();
            record.priority = request.priority;
            record.generation = request.generation;
            record.serialActive = RigExecFrozenSerialActive();
            std::lock_guard<std::mutex> lock(_mutex);
            _runs.push_back(record);
            return RigExecWarmOutcome::Published;
        };
    }

    // A factory returning recording work and counting its own invocations.
    // Invocations happen on the triggering thread only, so the count needs
    // no lock -- but the member lives here for convenience.
    RigExecWarmJobFactory Factory()
    {
        return [this](UsdTimeCode time) -> RigExecWarmWork {
            ++_factoryCalls;
            (void)time;
            return Work();
        };
    }

    std::vector<RunRecord> Runs() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _runs;
    }

    size_t FactoryCalls() const
    {
        return _factoryCalls;
    }

private:
    mutable std::mutex _mutex;
    std::vector<RunRecord> _runs;
    size_t _factoryCalls = 0;
};

// The times of \p runs, in run order.
std::vector<double>
RunTimes(const std::vector<RunRecord> &runs)
{
    std::vector<double> times;
    for (const RunRecord &run : runs) {
        times.push_back(run.time);
    }
    return times;
}

bool
TimesEqual(const std::vector<double> &a, const std::vector<double> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// An unseen rig sits at generation zero: "no edit yet" is a token like any
// other, and the first edit must move it.
void
TestAnUnseenRigSitsAtGenerationZero()
{
    RigExecBackgroundScheduler scheduler;
    CHECK(scheduler.CurrentGeneration(SdfPath("/Asset/Rig")) == 0);
    CHECK(scheduler.IsGenerationCurrent(SdfPath("/Asset/Rig"), 0));
    CHECK(!scheduler.IsGenerationCurrent(SdfPath("/Asset/Rig"), 1));
}

// CancelGeneration bumps monotonically: every edit retires every job sampled
// before it, and no token is ever reused.
void
TestCancelBumpsMonotonically()
{
    RigExecBackgroundScheduler scheduler;
    const SdfPath rig("/Asset/Rig");
    CHECK(scheduler.CurrentGeneration(rig) == 0);
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.CurrentGeneration(rig) == 1);
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.CurrentGeneration(rig) == 2);
    CHECK(scheduler.IsGenerationCurrent(rig, 2));
    CHECK(!scheduler.IsGenerationCurrent(rig, 1));
    CHECK(!scheduler.IsGenerationCurrent(rig, 0));
}

// Generations are per rig: an edit to one rig cancels nothing of another's.
void
TestRigsAreIndependent()
{
    RigExecBackgroundScheduler scheduler;
    const SdfPath first("/Asset/RigA");
    const SdfPath second("/Asset/RigB");
    scheduler.CancelGeneration(first);
    CHECK(scheduler.CurrentGeneration(first) == 1);
    CHECK(scheduler.CurrentGeneration(second) == 0);
    CHECK(scheduler.IsGenerationCurrent(second, 0));
}

// A job enqueued under a superseded token is refused for staleness: the edit
// it was sampled for is gone, and warming the old inputs would publish a
// pose no frame asks for anymore.
void
TestAStaleJobIsRefused()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration sampled = scheduler.CurrentGeneration(rig);
    scheduler.CancelGeneration(rig);
    CHECK(!scheduler.IsGenerationCurrent(rig, sampled));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                             RigExecWarmPriority::Neighbor, sampled,
                             recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                             sampled, recorder.Work()));
    CHECK(scheduler.Stats().declined == 2);
    const RigExecBackgroundSchedulerStats refused = scheduler.Stats();
    CHECK(refused.declinedFillGate + refused.declinedStaleGeneration == 2);
    if (RigExecBackgroundWarmingEnabled()) {
        CHECK(refused.declinedStaleGeneration == 2);
    }
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// The switch parses to its three modes; anything else is the default.
void
TestFrameCacheModeParses()
{
    CHECK(RigExecParseFrameCacheMode("on") == RigExecFrameCacheMode::On);
    CHECK(RigExecParseFrameCacheMode("off") == RigExecFrameCacheMode::Off);
    CHECK(RigExecParseFrameCacheMode("warm-off") ==
          RigExecFrameCacheMode::WarmOff);
    CHECK(RigExecParseFrameCacheMode("") == RigExecFrameCacheMode::On);
    CHECK(RigExecParseFrameCacheMode("ON") == RigExecFrameCacheMode::On);
    CHECK(RigExecParseFrameCacheMode("bogus") == RigExecFrameCacheMode::On);
}

// The fill gate is the mode AND parallel-eval: warm-off and parallel-off both
// mean "reads served, fill off", and only on+enabled warms.
void
TestWarmingGateTruthTable()
{
    CHECK(RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::On, true));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::On, false));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::Off, true));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::Off, false));
    CHECK(
        !RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::WarmOff, true));
    CHECK(!RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode::WarmOff,
                                           false));
}

// The mode reads live from the environment: a host flips warming without
// restarting. (The bogus value warns once; the parse still answers On.)
void
TestFrameCacheModeReadsLiveFromTheEnvironment()
{
    {
        ScopedFrameCacheMode on("on");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::On);
    }
    {
        ScopedFrameCacheMode off("off");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::Off);
    }
    {
        ScopedFrameCacheMode warmOff("warm-off");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::WarmOff);
    }
    {
        ScopedFrameCacheMode bogus("bogus");
        CHECK(RigExecFrameCacheModeFromEnvironment() ==
              RigExecFrameCacheMode::On);
    }
    // The environment-ambient gate agrees with the pure gate fed the live
    // mode and the library's cached parallel switch.
    ScopedFrameCacheMode on("on");
    CHECK(
        RigExecBackgroundWarmingEnabled() ==
        RigExecBackgroundWarmingEnabled(RigExecFrameCacheModeFromEnvironment(),
                                        RigExecParallelEvaluationEnabled()));
}

// A closed gate declines warming while the fence keeps working: every
// Enqueue refuses, every trigger enqueues nothing without sampling, and edits
// still bump. Passes in every ambient mode -- it forces the gate closed.
void
TestClosedGateDeclinesWarmingButKeepsTheFence()
{
    for (const char *mode : {"off", "warm-off"}) {
        ScopedFrameCacheMode closed(mode);
        CHECK(!RigExecBackgroundWarmingEnabled());
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                 RigExecWarmPriority::Neighbor, current,
                                 recorder.Work()));
        CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                 RigExecWarmPriority::Sweep, current,
                                 recorder.Work()));
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(4.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                        recorder.Factory()) == 0);
        CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                               recorder.Factory()) == 0);
        CHECK(recorder.FactoryCalls() == 0);
        CHECK(scheduler.Stats().declined == 2);
        CHECK(scheduler.Stats().declinedFillGate == 2);
        CHECK(scheduler.Stats().queuedDepth == 0);
        // The fence is unaffected by the gate: edits still bump.
        scheduler.CancelGeneration(rig);
        CHECK(scheduler.CurrentGeneration(rig) == current + 1);
        CHECK(!scheduler.IsGenerationCurrent(rig, current));
    }
}

// A current job is accepted and runs: the ordering is neighbors before sweep
// whatever order they enqueue in.
void
TestACurrentJobIsAcceptedNeighborBeforeSweep()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestACurrentJobIsAcceptedNeighborBeforeSweep: "
                    "warming unavailable in this process\n");
        return;
    }
    CHECK(int(RigExecWarmPriority::Neighbor) < int(RigExecWarmPriority::Sweep));
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    // Enqueued backwards on purpose: the sweep must still run second.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.Stats().queuedDepth == 2);
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(!scheduler.RunNextQueuedForTesting());
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(runs.size() == 2);
    CHECK(runs[0].time == 2.0);
    CHECK(runs[0].priority == RigExecWarmPriority::Neighbor);
    CHECK(runs[1].time == 1.0);
    CHECK(runs[1].priority == RigExecWarmPriority::Sweep);
    CHECK(scheduler.Stats().completed == 2);
    CHECK(scheduler.Stats().queuedDepth == 0);
    // The accepted requests moved no token.
    CHECK(scheduler.CurrentGeneration(rig) == current);
}

// Within one priority the queue is FIFO: enqueue order, not time order.
void
TestFifoWithinAPriority()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestFifoWithinAPriority: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 3);
    CHECK(TimesEqual(RunTimes(recorder.Runs()), {3.0, 1.0, 2.0}));
}

// A duplicate time coalesces: the second Enqueue accepts -- warming is coming
// -- without queueing twice, and the job runs once.
void
TestDuplicateTimesCoalesce()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestDuplicateTimesCoalesce: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Stats().queuedDepth == 1);
    CHECK(scheduler.Stats().coalesced == 1);
    CHECK(scheduler.Stats().upgraded == 0);
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().size() == 1);
}

// A neighbor request for a queued sweep time promotes it: the frame matters
// more than the trigger thought. Re-enqueueing a sweep for a queued neighbor
// changes nothing.
void
TestSweepPromotesToNeighborOnReenqueue()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestSweepPromotesToNeighborOnReenqueue: "
                    "warming unavailable in this process\n");
        return;
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
        CHECK(scheduler.Stats().queuedDepth == 1);
        CHECK(scheduler.Stats().upgraded == 1);
        CHECK(scheduler.Stats().coalesced == 0);
        CHECK(scheduler.DrainQueueForTesting() == 1);
        CHECK(recorder.Runs().size() == 1);
        CHECK(recorder.Runs()[0].priority == RigExecWarmPriority::Neighbor);
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(4.0),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(4.0),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
        CHECK(scheduler.Stats().coalesced == 1);
        CHECK(scheduler.Stats().upgraded == 0);
        CHECK(scheduler.DrainQueueForTesting() == 1);
        CHECK(recorder.Runs()[0].priority == RigExecWarmPriority::Neighbor);
    }
}

// A default, non-finite, or work-less Enqueue declines: none of them can ever
// become a job.
void
TestUnusableEnqueuesDecline()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode::Default(),
                             RigExecWarmPriority::Neighbor, current,
                             recorder.Work()));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(nan), RigExecWarmPriority::Sweep,
                             current, recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(inf), RigExecWarmPriority::Sweep,
                             current, recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                             RigExecWarmPriority::Neighbor, current,
                             RigExecWarmWork()));
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.declined == 4);
    CHECK(stats.declined ==
          stats.declinedFillGate + stats.declinedStaleGeneration +
              stats.declinedUnusableTime + stats.declinedEmptyWork +
              stats.declinedPerRigCap + stats.declinedStopping +
              stats.declinedFreezeRefused + stats.declinedUnsampleable +
              stats.declinedD7Exempt);
    if (RigExecBackgroundWarmingEnabled()) {
        CHECK(stats.declinedUnusableTime == 3);
        CHECK(stats.declinedEmptyWork == 1);
    }
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// The per-rig in-flight cap declines past its limit, frees as jobs run, and
// never crosses rigs. A coalesced duplicate does not consume the cap.
void
TestPerRigCapDeclinesPastItsLimit()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPerRigCapDeclinesPastItsLimit: "
                    "warming unavailable in this process\n");
        return;
    }
    CHECK(RigExecBackgroundScheduler(/*workerCount=*/0).GetPerRigJobCap() ==
          kRigExecBackgroundSchedulerDefaultPerRigJobs);
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(2);
    CHECK(scheduler.GetPerRigJobCap() == 2);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const SdfPath other("/Asset/Other");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                             current, recorder.Work()));
    // A duplicate of a queued time merges instead of declining: it asks for
    // no new warming.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Stats().coalesced == 1);
    // Another rig has its own budget.
    const RigExecFrameGeneration otherCurrent =
        scheduler.CurrentGeneration(other);
    CHECK(scheduler.Enqueue(other, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            otherCurrent, recorder.Work()));
    // Running one frees one.
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Stats().declined == 1);
    CHECK(scheduler.Stats().declinedPerRigCap == 1);
}

// CancelGeneration purges the rig's queued jobs -- inputs freed, cap space
// back -- and touches no other rig. Fresh jobs enqueue under the new token.
void
TestCancelPurgesQueuedJobsPromptly()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestCancelPurgesQueuedJobsPromptly: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const SdfPath other("/Asset/Other");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const RigExecFrameGeneration otherCurrent =
        scheduler.CurrentGeneration(other);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(other, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            otherCurrent, recorder.Work()));
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.Stats().canceled == 1);
    CHECK(scheduler.Stats().queuedDepth == 1);
    // The other rig's job survived the purge.
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().size() == 1);
    CHECK(recorder.Runs()[0].rig == other);
    // The purge freed the cap: a fresh job under the new token enqueues.
    const RigExecFrameGeneration fresh = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                            fresh, recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().size() == 2);
    CHECK(scheduler.Stats().completed == 2);
}

// The commit trigger enqueues neighbors closest-first, then the sweep in
// order, and never the playhead -- even when the sweep names it.
void
TestCommitTriggerEnqueuesNeighborsThenSweepNeverPlayhead()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf(
            "SKIP TestCommitTriggerEnqueuesNeighborsThenSweepNeverPlayhead: "
            "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(10.0), UsdTimeCode(5.0),
                                         UsdTimeCode(6.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    recorder.Factory(),
                                    /*neighborRadius=*/2) == 6);
    CHECK(recorder.FactoryCalls() == 6);
    CHECK(scheduler.DrainQueueForTesting() == 6);
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(TimesEqual(RunTimes(runs), {9.0, 11.0, 8.0, 12.0, 5.0, 6.0}));
    for (size_t i = 0; i < 4; ++i) {
        CHECK(runs[i].priority == RigExecWarmPriority::Neighbor);
    }
    for (size_t i = 4; i < 6; ++i) {
        CHECK(runs[i].priority == RigExecWarmPriority::Sweep);
    }
    for (const RunRecord &run : runs) {
        CHECK(run.time != 10.0);
    }
}

// The trigger skips default sweep entries without sampling them, merges
// duplicates without sampling them twice, and stops at the cap.
void
TestTriggerSkipsUnsampleableAndStopsAtCap()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTriggerSkipsUnsampleableAndStopsAtCap: "
                    "warming unavailable in this process\n");
        return;
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        const std::vector<UsdTimeCode> sweep{
            UsdTimeCode::Default(), UsdTimeCode(5.0), UsdTimeCode(5.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                        recorder.Factory(),
                                        /*neighborRadius=*/0) == 1);
        CHECK(recorder.FactoryCalls() == 1);
        CHECK(scheduler.Stats().coalesced == 1);
        CHECK(scheduler.DrainQueueForTesting() == 1);
    }
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetPerRigJobCap(2);
        Recorder recorder;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                        recorder.Factory(),
                                        /*neighborRadius=*/8) == 2);
        CHECK(recorder.FactoryCalls() == 2);
        CHECK(scheduler.DrainQueueForTesting() == 2);
        CHECK(TimesEqual(RunTimes(recorder.Runs()), {9.0, 11.0}));
    }
    // A null factory builds nothing and crashes nothing.
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0)};
        CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                        RigExecWarmJobFactory(),
                                        /*neighborRadius=*/2) == 0);
        CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                               RigExecWarmJobFactory()) == 0);
        CHECK(scheduler.Stats().queuedDepth == 0);
    }
}

// A stale generation, or a default playhead, enqueues nothing and never calls
// the factory: there is nothing faithful to sample.
void
TestTriggerWithStaleGenerationOrDefaultPlayheadEnqueuesNothing()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration sampled = scheduler.CurrentGeneration(rig);
    scheduler.CancelGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), sampled, sweep,
                                    recorder.Factory()) == 0);
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), sampled, sweep,
                           recorder.Factory()) == 0);
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode::Default(), current, sweep,
                                    recorder.Factory()) == 0);
    CHECK(scheduler.OnIdle(rig, UsdTimeCode::Default(), current, sweep,
                           recorder.Factory()) == 0);
    CHECK(recorder.FactoryCalls() == 0);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// An edit landing mid-trigger stops it: the trigger rechecks the generation
// around every sample, so the bump inside the factory ends the commit and the
// bump's own purge retires what the trigger had queued. Deterministic -- the
// factory itself bumps -- so no wall clock and no sleep.
void
TestTriggerStopsWhenAnEditLandsMidTrigger()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTriggerStopsWhenAnEditLandsMidTrigger: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    size_t calls = 0;
    RigExecWarmJobFactory bumping =
        [&](UsdTimeCode time) -> RigExecWarmFactoryResult {
        ++calls;
        if (calls == 2) {
            scheduler.CancelGeneration(rig);
        }
        return recorder.Factory()(time);
    };
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(5.0), UsdTimeCode(6.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    bumping,
                                    /*neighborRadius=*/2) == 1);
    CHECK(calls == 2);
    // The bump purged the one job the trigger had queued.
    CHECK(scheduler.Stats().canceled == 1);
    CHECK(scheduler.Stats().queuedDepth == 0);
    CHECK(scheduler.Stats().triggerStops == 1);
    CHECK(scheduler.Stats().triggerStopsStale == 1);
    CHECK(scheduler.DrainQueueForTesting() == 0);
    CHECK(recorder.Runs().empty());
}

// The idle trigger enqueues sweep only -- neighbors belong to the commit --
// and skips the playhead like every trigger.
void
TestIdleTriggerEnqueuesSweepOnly()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestIdleTriggerEnqueuesSweepOnly: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(10.0), UsdTimeCode(5.0),
                                         UsdTimeCode(6.0)};
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory()) == 2);
    CHECK(recorder.FactoryCalls() == 2);
    CHECK(scheduler.DrainQueueForTesting() == 2);
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(TimesEqual(RunTimes(runs), {5.0, 6.0}));
    for (const RunRecord &run : runs) {
        CHECK(run.priority == RigExecWarmPriority::Sweep);
        CHECK(run.time != 10.0);
    }
}

// Playback advance sheds queued sweep at or behind the playhead -- the live
// path serves those frames now -- keeps neighbors and ahead frames, and frees
// the cap for what is still worth warming.
void
TestPlaybackAdvanceShedsPassedSweep()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPlaybackAdvanceShedsPassedSweep: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(4);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(8.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(10.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(11.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 2);
    CHECK(scheduler.Stats().shed == 2);
    CHECK(scheduler.Stats().queuedDepth == 2);
    // The shed freed the cap: an ahead frame enqueues.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(12.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 3);
    // Neighbor first, then the ahead sweep in FIFO order.
    CHECK(TimesEqual(RunTimes(recorder.Runs()), {8.0, 11.0, 12.0}));
    CHECK(recorder.Runs()[0].priority == RigExecWarmPriority::Neighbor);
    // Nothing to shed: default playhead, unknown rig, empty queue.
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode::Default()) == 0);
    CHECK(scheduler.NotePlaybackAdvanced(SdfPath("/Asset/Other"),
                                         UsdTimeCode(10.0)) == 0);
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 0);
    CHECK(scheduler.Stats().shed == 2);
}

// A NaN playhead sheds nothing: NaN is numeric but orders against nothing,
// so every at-or-behind test is false and -- without the finiteness guard --
// the call would purge the whole sweep.
void
TestPlaybackAdvanceWithNaNShedsNothing()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPlaybackAdvanceWithNaNShedsNothing: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(10.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(nan)) == 0);
    CHECK(scheduler.Stats().shed == 0);
    CHECK(scheduler.Stats().queuedDepth == 2);
    // The queue is intact: a real advance still sheds what it should.
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 2);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

// The scheduler lane records in production: queue-depth samples on every
// queue mutation and purge points with causes -- off by default, exact when
// enabled.
void
TestProfilerLaneRecordsQueueAndCancels()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestProfilerLaneRecordsQueueAndCancels: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(10.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    // Disabled by default: queue mutations record nothing.
    CHECK(scheduler.GetProfiler().GetEventCount() == 0);

    scheduler.MutableProfiler()->SetEnabled(true);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(8.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    scheduler.CancelGeneration(rig);
    const RigExecFrameGeneration next = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(11.0), RigExecWarmPriority::Sweep,
                            next, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(12.0), RigExecWarmPriority::Sweep,
                            next, recorder.Work()));
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(12.0)) == 2);
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(13.0)) == 0);

    std::vector<std::string> names;
    std::vector<std::map<std::string, double>> queues;
    std::vector<std::pair<std::string, std::string>> cancels;
    for (const RigExecProfileEvent &event :
         scheduler.GetProfiler().GetEvents()) {
        names.push_back(event.name);
        if (event.kind == RigExecProfileEventKind::Counter &&
            event.name == "warmQueue") {
            queues.push_back(event.counters);
        }
        if (event.kind == RigExecProfileEventKind::Instant &&
            event.name == "warmCancel") {
            cancels.push_back(
                {event.args.at("purged"), event.args.at("cause")});
        }
    }
    const std::vector<std::string> expectedNames(
        {"warmQueue", "warmCancel", "warmQueue", "warmQueue", "warmQueue",
         "warmCancel", "warmQueue", "warmQueue"});
    CHECK(names == expectedNames);
    CHECK(queues.size() == 6);
    const double depths[6] = {3, 0, 1, 2, 0, 0};
    const double canceled[6] = {0, 3, 3, 3, 3, 3};
    for (size_t i = 0; i < queues.size(); ++i) {
        CHECK(queues[i].at("queuedDepth") == depths[i]);
        CHECK(queues[i].at("running") == 0);
        CHECK(queues[i].at("canceled") == canceled[i]);
    }
    const std::vector<std::pair<std::string, std::string>> expectedCancels(
        {{"3", "edit"}, {"2", "playback"}});
    CHECK(cancels == expectedCancels);
}

// The counters balance exactly over a mixed scenario: every job that entered
// the queue ends completed, canceled, or shed, and merges never enter it.
void
TestStatsBalanceOverAMixedScenario()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestStatsBalanceOverAMixedScenario: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Neighbor, current,
                            recorder.Work()));
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(recorder.Runs().size() == 1);
    CHECK(recorder.Runs()[0].time == 2.0);
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(1.5)) == 1);
    scheduler.CancelGeneration(rig);
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.completed == 1);
    CHECK(stats.coalesced == 1);
    CHECK(stats.upgraded == 1);
    CHECK(stats.shed == 1);
    CHECK(stats.canceled == 1);
    CHECK(stats.droppedStale == 0);
    CHECK(stats.declined == 0);
    CHECK(stats.queuedDepth == 0);
    CHECK(stats.running == 0);
    // The one ran job published; completed still equals the split.
    CHECK(stats.published == 1);
    CHECK(stats.declinedInvalid == 0);
    CHECK(stats.declinedGeneration == 0);
    CHECK(stats.completed ==
          stats.published + stats.declinedInvalid + stats.declinedGeneration);
    // Three jobs entered the queue (five accepts minus two merges); all three
    // reached a terminal state, under the split equation.
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.canceled + stats.shed + stats.droppedStale +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          3);
    // The peak the queue reached, and the split of what entered it.
    CHECK(stats.maxQueuedDepth == 3);
    CHECK(stats.maxRunning == 1);
    CHECK(stats.enqueuedNeighbor + stats.enqueuedSweep == 3);
    CHECK(stats.enqueuedNeighbor == 1);
    CHECK(stats.enqueuedSweep == 2);
    CHECK(stats.declined ==
          stats.declinedFillGate + stats.declinedStaleGeneration +
              stats.declinedUnusableTime + stats.declinedEmptyWork +
              stats.declinedPerRigCap + stats.declinedStopping +
              stats.declinedFreezeRefused + stats.declinedUnsampleable +
              stats.declinedD7Exempt);
}

// The pool runs every job exactly once: enqueue eight, wait for idle, and all
// eight ran. The wait is event-driven -- queue empty and none running -- not
// timed.
void
TestPoolRunsEveryJobExactlyOnce()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPoolRunsEveryJobExactlyOnce: "
                    "warming unavailable in this process\n");
        return;
    }
    CHECK(!RigExecFrozenSerialActive());
    RigExecBackgroundScheduler scheduler;
    CHECK(scheduler.GetWorkerCount() ==
          kRigExecBackgroundSchedulerDefaultWorkers);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    for (int i = 0; i < 8; ++i) {
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(double(i)),
                                i % 2 == 0 ? RigExecWarmPriority::Neighbor
                                           : RigExecWarmPriority::Sweep,
                                current, recorder.Work()));
    }
    scheduler.WaitUntilIdle();
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(runs.size() == 8);
    CHECK(scheduler.Stats().completed == 8);
    CHECK(scheduler.Stats().queuedDepth == 0);
    CHECK(scheduler.Stats().running == 0);
    // Every enqueued time ran exactly once (order across two workers is
    // unscheduled, so the comparison is a set, not a sequence).
    std::vector<double> times = RunTimes(runs);
    std::sort(times.begin(), times.end());
    CHECK(TimesEqual(times, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0}));
    // Every job ran under the frozen serial scope: the per-job serial
    // executor boundary, whatever the process-wide switch says. The scope is
    // thread-local to the workers -- this thread never entered one.
    for (const RunRecord &run : runs) {
        CHECK(run.serialActive);
    }
    CHECK(!RigExecFrozenSerialActive());
    // Idleness composes: a second wait on an idle pool returns.
    scheduler.WaitUntilIdle();
}

// A cancel racing the pool still balances the books: every job lands in
// exactly one terminal state, whatever the interleaving.
void
TestThreadedCancelBalancesTheBooks()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestThreadedCancelBalancesTheBooks: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler;
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    size_t accepted = 0;
    for (int i = 0; i < 32; ++i) {
        accepted += scheduler.Enqueue(rig, UsdTimeCode(double(i)),
                                      RigExecWarmPriority::Sweep, current,
                                      recorder.Work())
                        ? 1
                        : 0;
    }
    scheduler.CancelGeneration(rig);
    scheduler.WaitUntilIdle();
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.queuedDepth == 0);
    CHECK(stats.running == 0);
    CHECK(stats.completed + stats.droppedStale + stats.canceled == accepted);
    CHECK(recorder.Runs().size() == stats.completed);
}

// Destruction with pending jobs drops them without running them: the frames
// evaluate live when asked.
void
TestShutdownDropsPendingJobs()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestShutdownDropsPendingJobs: "
                    "warming unavailable in this process\n");
        return;
    }
    Recorder recorder;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
        CHECK(scheduler.Stats().queuedDepth == 2);
    }
    CHECK(recorder.Runs().empty());
}

// Destruction with a job running joins it: the release arrives on a second
// thread once the job starts, so both the worker and the joining destructor
// move on events, and the running job finishes instead of stranding the join.
void
TestShutdownJoinsARunningJob()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestShutdownJoinsARunningJob: "
                    "warming unavailable in this process\n");
        return;
    }
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    size_t ran = 0;
    {
        RigExecBackgroundScheduler scheduler;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        RigExecWarmWork blocking = [&](const RigExecWarmRequest &) {
            {
                std::lock_guard<std::mutex> lock(gateMutex);
                started = true;
            }
            gateCv.notify_all();
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return released; });
            ++ran;
            return RigExecWarmOutcome::Published;
        };
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                                RigExecWarmPriority::Neighbor, current,
                                blocking));
        // The release arrives from a second thread once the job starts, so
        // the destructor below -- which joins the worker running the job --
        // cannot deadlock however the threads interleave.
        std::thread releaser([&] {
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return started; });
            released = true;
            lock.unlock();
            gateCv.notify_all();
        });
        releaser.join();
    }
    CHECK(ran == 1);
}

// A cancel purging while a job runs drops the queued jobs behind it: one
// worker holds the running job, so the sweep behind is still queued when the
// purge lands, and it never runs.
void
TestCancelPurgesWhileAJobRuns()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestCancelPurgesWhileAJobRuns: "
                    "warming unavailable in this process\n");
        return;
    }
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    RigExecBackgroundScheduler scheduler(/*workerCount=*/1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecWarmWork blocking = [&](const RigExecWarmRequest &) {
        {
            std::lock_guard<std::mutex> lock(gateMutex);
            started = true;
        }
        gateCv.notify_all();
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCv.wait(lock, [&] { return released; });
        return RigExecWarmOutcome::Published;
    };
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Neighbor, current, blocking));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    {
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCv.wait(lock, [&] { return started; });
    }
    // The worker holds the neighbor; the sweep is still queued, so the purge
    // retires exactly it.
    scheduler.CancelGeneration(rig);
    CHECK(scheduler.Stats().canceled == 1);
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        released = true;
    }
    gateCv.notify_all();
    scheduler.WaitUntilIdle();
    CHECK(recorder.Runs().empty());
    CHECK(scheduler.Stats().completed == 1);
    CHECK(scheduler.Stats().queuedDepth == 0);
}

void
TestScopedRetirementIncludesRunningFirstFills()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestScopedRetirementIncludesRunningFirstFills: "
                    "warming unavailable in this process\n");
        return;
    }
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    RigExecBackgroundScheduler scheduler(/*workerCount=*/1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const SdfPath other("/Other/Rig");
    const RigExecFrameGeneration generation = scheduler.CurrentGeneration(rig);
    RigExecWarmWork blocking = [&](const RigExecWarmRequest &request) {
        {
            std::lock_guard<std::mutex> lock(gateMutex);
            started = true;
        }
        gateCv.notify_all();
        {
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return released; });
        }
        return scheduler.IsWarmRequestCurrent(request.rig, request.generation,
                                              request.time, request.fenceToken)
                   ? RigExecWarmOutcome::Published
                   : RigExecWarmOutcome::DeclinedGeneration;
    };
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1), RigExecWarmPriority::Sweep,
                            generation, blocking));
    {
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCv.wait(lock, [&] { return started; });
    }
    // The running frame has never completed. A second request at its
    // time and another frame wait behind it, while another rig is untouched.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1), RigExecWarmPriority::Sweep,
                            generation, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2), RigExecWarmPriority::Sweep,
                            generation, recorder.Work()));
    CHECK(scheduler.Enqueue(other, UsdTimeCode(4), RigExecWarmPriority::Sweep,
                            scheduler.CurrentGeneration(other),
                            recorder.Work()));
    const auto inFlight = scheduler.InFlightTimes(rig);
    CHECK(inFlight ==
          std::vector<UsdTimeCode>({UsdTimeCode(1), UsdTimeCode(2)}));
    CHECK(scheduler.InFlightTimes(other) ==
          std::vector<UsdTimeCode>({UsdTimeCode(4)}));
    CHECK(scheduler.CancelGenerationTimes(rig, inFlight) == 2);
    CHECK(scheduler.CurrentGeneration(rig) == generation);
    CHECK(!scheduler.IsWarmRequestCurrent(rig, generation, UsdTimeCode(1), 0));
    CHECK(scheduler.QueuedTimes(rig).empty());
    CHECK(scheduler.InFlightTimes(rig) ==
          std::vector<UsdTimeCode>({UsdTimeCode(1)}));
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        released = true;
    }
    gateCv.notify_all();
    scheduler.WaitUntilIdle();
    CHECK(scheduler.Stats().declinedGeneration == 1);
    CHECK(scheduler.Stats().published == 1);
    CHECK(scheduler.InFlightTimes(rig).empty());
    CHECK(scheduler.InFlightTimes(other).empty());
    CHECK(TimesEqual(RunTimes(recorder.Runs()), {4.0}));
    // Fresh work can reuse the same generation after scoped cancellation.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1), RigExecWarmPriority::Sweep,
                            generation, recorder.Work()));
    scheduler.WaitUntilIdle();
    CHECK(scheduler.Stats().published == 2);
    CHECK(scheduler.InFlightTimes(rig).empty());
}

// Worker counts clamp to the documented range: production's 2-4, and zero
// for the manual mode.
void
TestWorkerCountClampsToRange()
{
    CHECK(RigExecBackgroundScheduler().GetWorkerCount() ==
          kRigExecBackgroundSchedulerDefaultWorkers);
    CHECK(RigExecBackgroundScheduler(0).GetWorkerCount() == 0);
    CHECK(RigExecBackgroundScheduler(4).GetWorkerCount() == 4);
    CHECK(RigExecBackgroundScheduler(17).GetWorkerCount() ==
          kRigExecBackgroundSchedulerMaxWorkers);
    CHECK(RigExecBackgroundScheduler(-3).GetWorkerCount() == 0);
    CHECK(kRigExecBackgroundSchedulerDefaultWorkers >= 2);
    CHECK(kRigExecBackgroundSchedulerDefaultWorkers <=
          kRigExecBackgroundSchedulerMaxWorkers);
}

// The priority backend reports honestly: a known name, and a callable setter
// whose answer the test never interprets -- gate tests assert progress, not
// OS priority values.
void
TestPriorityBackendReportsHonestly()
{
    const char *backend = RigExecBackgroundPriorityBackend();
    CHECK(backend != nullptr);
    const std::string name = backend ? backend : "";
    CHECK(name == "SetThreadPriority" || name == "sched-batch" ||
          name == "qos-utility" || name == "none");
    (void)RigExecSetBackgroundThreadPriority();
}

// OUTCOME. Ran jobs split completed by disposition: published,
// declined-invalid, declined-generation; a throwing job warns and lands
// declined-invalid; completed still equals the three.
void
TestOutcomesSplitCompletion()
{
    std::printf("progress: TestOutcomesSplitCompletion\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestOutcomesSplitCompletion: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const auto outcome = [](RigExecWarmOutcome o) {
        return [o](const RigExecWarmRequest &) { return o; };
    };
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::Published)));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::Published)));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::DeclinedInvalid)));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(4.0),
                            RigExecWarmPriority::Neighbor, current,
                            outcome(RigExecWarmOutcome::DeclinedGeneration)));
    CHECK(scheduler.Enqueue(
        rig, UsdTimeCode(5.0), RigExecWarmPriority::Neighbor, current,
        [](const RigExecWarmRequest &) -> RigExecWarmOutcome {
            throw std::runtime_error("test throw");
        }));
    CHECK(scheduler.DrainQueueForTesting() == 5);
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.completed == 5);
    CHECK(stats.published == 2);
    CHECK(stats.declinedInvalid == 2);
    CHECK(stats.declinedGeneration == 1);
    CHECK(stats.completed ==
          stats.published + stats.declinedInvalid + stats.declinedGeneration);
    // Five accepts, no merges, all five terminal under the new equation.
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.droppedStale + stats.canceled + stats.shed +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          5);
}

// Records transitions for the hook test. Thread-safe (workers report while
// the test asserts afterwards), like Recorder.
class TransitionRecorder {
public:
    RigExecWarmTransitionHook Hook()
    {
        return [this](const RigExecWarmTransition &transition) {
            std::lock_guard<std::mutex> lock(_mutex);
            _transitions.push_back(transition);
        };
    }

    std::vector<RigExecWarmTransition> Transitions() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _transitions;
    }

private:
    mutable std::mutex _mutex;
    std::vector<RigExecWarmTransition> _transitions;
};

// HOOK. The transition hook reports every per-frame queue move -- queued,
// running, finished, canceled, shed, dropped-at-shutdown -- in order, on
// the manual drain.
void
TestTransitionHookReportsQueueVisibility()
{
    std::printf("progress: TestTransitionHookReportsQueueVisibility\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTransitionHookReportsQueueVisibility: "
                    "warming unavailable in this process\n");
        return;
    }
    TransitionRecorder recorder;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetTransitionHook(recorder.Hook());
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        RigExecWarmWork published = [](const RigExecWarmRequest &) {
            return RigExecWarmOutcome::Published;
        };
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0),
                                RigExecWarmPriority::Neighbor, current,
                                published));
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                                RigExecWarmPriority::Sweep, current,
                                published));
        CHECK(scheduler.RunNextQueuedForTesting());
        CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(2.0)) == 1);
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                RigExecWarmPriority::Sweep, current,
                                published));
        scheduler.CancelGeneration(rig);
    }
    const std::vector<RigExecWarmTransition> got = recorder.Transitions();
    CHECK(got.size() == 7);
    const auto expect =
        [&got](size_t i, double time, RigExecWarmTransitionKind kind,
               RigExecWarmOutcome outcome = RigExecWarmOutcome::Published) {
            return i < got.size() && got[i].timeValue == time &&
                   got[i].kind == kind && got[i].outcome == outcome;
        };
    CHECK(expect(0, 1.0, RigExecWarmTransitionKind::Queued));
    CHECK(expect(1, 2.0, RigExecWarmTransitionKind::Queued));
    CHECK(expect(2, 1.0, RigExecWarmTransitionKind::Running));
    CHECK(expect(3, 1.0, RigExecWarmTransitionKind::Finished,
                 RigExecWarmOutcome::Published));
    CHECK(expect(4, 2.0, RigExecWarmTransitionKind::Shed));
    CHECK(expect(5, 3.0, RigExecWarmTransitionKind::Queued));
    CHECK(expect(6, 3.0, RigExecWarmTransitionKind::Canceled));

    // A scheduler destroyed with queued jobs reports the shutdown drops.
    TransitionRecorder shutdown;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetTransitionHook(shutdown.Hook());
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        RigExecWarmWork published = [](const RigExecWarmRequest &) {
            return RigExecWarmOutcome::Published;
        };
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(5.0),
                                RigExecWarmPriority::Sweep, current,
                                published));
    }
    const std::vector<RigExecWarmTransition> lost = shutdown.Transitions();
    CHECK(lost.size() == 2);
    if (lost.size() == 2) {
        CHECK(lost[0].timeValue == 5.0 &&
              lost[0].kind == RigExecWarmTransitionKind::Queued);
        CHECK(lost[1].timeValue == 5.0 &&
              lost[1].kind == RigExecWarmTransitionKind::DroppedAtShutdown);
    }
}

// FAIRNESS. One worker preserves the pop order exactly: neighbors
// closest-first, then the sweep in vector order.
void
TestPoolPreservesPriorityOrder()
{
    std::printf("progress: TestPoolPreservesPriorityOrder\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPoolPreservesPriorityOrder: "
                    "warming unavailable in this process\n");
        return;
    }
    // One worker: pops serialize with runs, so the execution order is the
    // pop order, exactly.
    RigExecBackgroundScheduler scheduler(/*workerCount=*/1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(20.0), UsdTimeCode(21.0)};
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    recorder.Factory(),
                                    /*neighborRadius=*/2) == 6);
    scheduler.WaitUntilIdle();
    const std::vector<RunRecord> runs = recorder.Runs();
    CHECK(runs.size() == 6);
    const double expected[] = {9.0, 11.0, 8.0, 12.0, 20.0, 21.0};
    for (size_t i = 0; i < runs.size() && i < 6; ++i) {
        CHECK(runs[i].time == expected[i]);
        CHECK(runs[i].priority == (i < 4 ? RigExecWarmPriority::Neighbor
                                         : RigExecWarmPriority::Sweep));
    }
}

// FAIRNESS. Under a two-worker flood no rig starves: both rigs complete
// every job and the books balance.
void
TestNoRigStarvesUnderFlood()
{
    std::printf("progress: TestNoRigStarvesUnderFlood\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestNoRigStarvesUnderFlood: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/2);
    Recorder recorder;
    const SdfPath rigA("/Asset/RigA");
    const SdfPath rigB("/Asset/RigB");
    size_t accepts = 0;
    for (int t = 1; t <= 60; ++t) {
        accepts += scheduler.Enqueue(
                       rigA, UsdTimeCode(double(t)), RigExecWarmPriority::Sweep,
                       scheduler.CurrentGeneration(rigA), recorder.Work())
                       ? 1
                       : 0;
    }
    for (int t = 1; t <= 60; ++t) {
        accepts += scheduler.Enqueue(
                       rigB, UsdTimeCode(double(t)), RigExecWarmPriority::Sweep,
                       scheduler.CurrentGeneration(rigB), recorder.Work())
                       ? 1
                       : 0;
    }
    CHECK(accepts == 120);
    scheduler.WaitUntilIdle();
    size_t ranA = 0;
    size_t ranB = 0;
    for (const RunRecord &run : recorder.Runs()) {
        if (run.rig == rigA) {
            ++ranA;
        } else if (run.rig == rigB) {
            ++ranB;
        }
    }
    CHECK(ranA == 60);
    CHECK(ranB == 60);
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.published == 120);
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.droppedStale + stats.canceled + stats.shed +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          120);
}

// BUDGET. The sampling budget bounds factory invocations per trigger; the
// millisecond stop binds sampling; already-queued merges consume nothing;
// the unlimited default preserves the full burst shape.
void
TestSamplingBudgetBoundsFactoryInvocations()
{
    std::printf("progress: TestSamplingBudgetBoundsFactoryInvocations\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestSamplingBudgetBoundsFactoryInvocations: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    std::vector<UsdTimeCode> sweep;
    for (int t = 20; t < 40; ++t) {
        sweep.push_back(UsdTimeCode(double(t)));
    }
    RigExecWarmSamplingBudget budget;
    budget.maxFactoryInvocations = 5;
    // Five invocations, five jobs; the trigger stops at the budget.
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), budget) == 5);
    CHECK(scheduler.Stats().factoryInvocations == 5);
    // The same sweep again: the five queued times merge without sampling
    // and the rest sample up to the budget.
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), budget) == 5);
    CHECK(scheduler.Stats().factoryInvocations == 10);
    CHECK(scheduler.DrainQueueForTesting() == 10);
    // Drained, the next trigger samples the same first five again: the
    // scheduler keeps no visited-memory (the registry cursor skips
    // visited frames; the scheduler re-samples whatever it is given).
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), budget) == 5);
    CHECK(scheduler.Stats().factoryInvocations == 15);
    // A zero millisecond stop samples nothing (deterministic: elapsed 0
    // meets the stop before the first sample).
    RigExecWarmSamplingBudget stopped;
    stopped.maxMs = 0.0;
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, sweep,
                                    recorder.Factory(), 2, stopped) == 0);
    CHECK(scheduler.Stats().factoryInvocations == 15);
    CHECK(scheduler.DrainQueueForTesting() == 5);
    // The unlimited default preserves the full burst shape.
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory()) == 20);
    CHECK(scheduler.Stats().factoryInvocations == 35);
    // The factory-side count agrees with the scheduler's.
    CHECK(recorder.FactoryCalls() == 35);
}

// A clock that advances a fixed step on every reading. The queue stamps and
// the millisecond budget stop read it, so a test can assert exact waits and
// stops without touching wall time. It takes no locks, and the tests that
// inject it run with no workers, so only the calling thread stamps.
class SteppingClock {
public:
    explicit SteppingClock(uint64_t step = 1000) : _step(step)
    {
    }
    uint64_t Now()
    {
        const uint64_t value = _now;
        _now += _step;
        return value;
    }

private:
    uint64_t _step = 1000;
    uint64_t _now = 0;
};

// The scheduler stamps every job with its own clock: queue wait is enqueue to
// pop, run time is pop to the terminal state. With an injected clock the two
// are exact numbers, not wall-clock noise.
void
TestInjectedClockStampsQueueWaitAndRunTime()
{
    std::printf("progress: TestInjectedClockStampsQueueWaitAndRunTime\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestInjectedClockStampsQueueWaitAndRunTime: "
                    "warming unavailable in this process\n");
        return;
    }
    SteppingClock clock;
    RigExecBackgroundScheduler scheduler(
        /*workerCount=*/0, [&clock] { return clock.Now(); });
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    // Reads: enqueue 0, enqueue 1000, pop 2000, finish 3000, pop 4000,
    // finish 5000. Job one waits 2000us and runs 1000us; job two waits 3000us
    // and runs 1000us.
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.RunNextQueuedForTesting());
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.completed == 2);
    CHECK(stats.totalQueueWaitUs == 5000);
    CHECK(stats.maxQueueWaitUs == 3000);
    CHECK(stats.totalRunUs == 2000);
    CHECK(stats.maxRunUs == 1000);
    // The seam is the clock the caller handed in, not the steady clock.
    const uint64_t before = scheduler.ClockNowUs();
    CHECK(scheduler.ClockNowUs() == before + 1000);
}

// Every refused Enqueue lands in exactly one reason, and the reasons add up to
// the declined total.
void
TestDeclinedEnqueuesSplitByReason()
{
    std::printf("progress: TestDeclinedEnqueuesSplitByReason\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestDeclinedEnqueuesSplitByReason: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetPerRigJobCap(1);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode::Default(),
                             RigExecWarmPriority::Sweep, current,
                             recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(nan), RigExecWarmPriority::Sweep,
                             current, recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                             current, RigExecWarmWork()));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(2.0), RigExecWarmPriority::Sweep,
                             current, recorder.Work()));
    CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0), RigExecWarmPriority::Sweep,
                             current + 1, recorder.Work()));
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.declinedUnusableTime == 2);
    CHECK(stats.declinedEmptyWork == 1);
    CHECK(stats.declinedPerRigCap == 1);
    CHECK(stats.declinedStaleGeneration == 1);
    CHECK(stats.declinedFillGate == 0);
    CHECK(stats.declinedStopping == 0);
    CHECK(stats.declined ==
          stats.declinedFillGate + stats.declinedStaleGeneration +
              stats.declinedUnusableTime + stats.declinedEmptyWork +
              stats.declinedPerRigCap + stats.declinedStopping +
              stats.declinedFreezeRefused + stats.declinedUnsampleable +
              stats.declinedD7Exempt);
}

// The depth and running maxima hold the peak the queue reached, and the queue
// split adds up to the depth in both priorities.
void
TestQueueDepthMaximaAndPrioritySplit()
{
    std::printf("progress: TestQueueDepthMaximaAndPrioritySplit\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestQueueDepthMaximaAndPrioritySplit: "
                    "warming unavailable in this process\n");
        return;
    }
    SteppingClock clock;
    RigExecBackgroundScheduler scheduler(
        /*workerCount=*/0, [&clock] { return clock.Now(); });
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    for (int i = 0; i < 3; ++i) {
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(double(i + 1)),
                                RigExecWarmPriority::Sweep, current,
                                recorder.Work()));
    }
    for (int i = 0; i < 2; ++i) {
        CHECK(scheduler.Enqueue(rig, UsdTimeCode(double(i + 5)),
                                RigExecWarmPriority::Neighbor, current,
                                recorder.Work()));
    }
    RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.queuedDepth == 5);
    CHECK(stats.queuedNeighbor == 2);
    CHECK(stats.queuedSweep == 3);
    CHECK(stats.maxQueuedDepth == 5);
    CHECK(stats.enqueuedNeighbor == 2);
    CHECK(stats.enqueuedSweep == 3);
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.RunNextQueuedForTesting());
    stats = scheduler.Stats();
    CHECK(stats.maxRunning == 1);
    CHECK(stats.queuedDepth == 3);
    CHECK(stats.maxQueuedDepth == 5);
    CHECK(stats.queuedNeighbor + stats.queuedSweep == stats.queuedDepth);
    scheduler.CancelGeneration(rig);
    stats = scheduler.Stats();
    CHECK(stats.queuedDepth == 0);
    CHECK(stats.maxQueuedDepth == 5);
}

// A trigger burst that ends early says why: staleness, the sampling budget
// (invocations or the millisecond stop), or the per-rig cap.
void
TestTriggerStopsSplitByCause()
{
    std::printf("progress: TestTriggerStopsSplitByCause\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestTriggerStopsSplitByCause: "
                    "warming unavailable in this process\n");
        return;
    }
    SteppingClock clock;
    RigExecBackgroundScheduler scheduler(
        /*workerCount=*/0, [&clock] { return clock.Now(); });
    scheduler.SetPerRigJobCap(2);
    Recorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    // Radius 2 asks for 9, 11, 8, 12: the cap closes the burst at the third.
    CHECK(scheduler.OnEditCommitted(rig, UsdTimeCode(10.0), current, {},
                                    recorder.Factory(), 2) == 2);
    CHECK(scheduler.Stats().triggerStopsCap == 1);
    CHECK(scheduler.Stats().triggerStops == 1);
    // Draining frees the cap without bumping the generation token.
    CHECK(scheduler.DrainQueueForTesting() == 2);

    // Invocation budget: one sample, then the burst stops.
    RigExecWarmSamplingBudget invocations;
    invocations.maxFactoryInvocations = 1;
    const std::vector<UsdTimeCode> sweep{UsdTimeCode(20.0), UsdTimeCode(21.0),
                                         UsdTimeCode(22.0)};
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), invocations) == 1);
    CHECK(scheduler.Stats().triggerStopsBudget == 1);
    CHECK(scheduler.DrainQueueForTesting() == 1);

    // Millisecond budget, on the injected clock: every reading advances
    // 1000us, so the first frame sees 1000us elapsed and the second 3000us
    // against a 1.5ms stop.
    RigExecWarmSamplingBudget msStop;
    msStop.maxMs = 1.5;
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                           recorder.Factory(), msStop) == 1);
    CHECK(scheduler.Stats().triggerStopsBudget == 2);
    CHECK(scheduler.DrainQueueForTesting() == 1);

    // An edit landing mid-burst stops the burst as stale.
    size_t calls = 0;
    RigExecWarmJobFactory bumping =
        [&](UsdTimeCode time) -> RigExecWarmFactoryResult {
        ++calls;
        if (calls == 2) {
            scheduler.CancelGeneration(rig);
        }
        return recorder.Factory()(time);
    };
    CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep, bumping) ==
          1);
    CHECK(scheduler.Stats().triggerStopsStale == 1);
    CHECK(scheduler.Stats().triggerStops ==
          scheduler.Stats().triggerStopsStale +
              scheduler.Stats().triggerStopsBudget +
              scheduler.Stats().triggerStopsCap);
}

// ---------------------------------------------------------------------------
// EXPLICIT. Explicit output admission shares the pool with warming.
// ---------------------------------------------------------------------------

// An explicit request for one rig, exact time and whole captured generation,
// with \p outputCount outputs of one rig identity and compile epoch.
RigExecExplicitRequest
MakeExplicitRequest(const SdfPath &rig, RigExecOutputTime time,
                    uint64_t programIdentity = 1, uint64_t inputRevision = 0,
                    uint64_t previewRevision = 0, uint64_t historyRevision = 0,
                    size_t reservationBytes = 0, size_t outputCount = 1)
{
    RigExecExplicitRequest request;
    request.rig = rig;
    request.time = time;
    request.generation.programIdentity = programIdentity;
    request.generation.inputRevision = inputRevision;
    request.generation.previewRevision = previewRevision;
    request.generation.historyRevision = historyRevision;
    request.reservationBytes = reservationBytes;
    for (size_t i = 0; i < outputCount; ++i) {
        RigExecOutputHandle handle;
        handle.rigIdentity = rig.GetString();
        handle.compileEpoch = programIdentity;
        handle.semanticKey = "/Out/out" + std::to_string(i);
        handle.type = RigExecPayloadText::point3;
        handle.readPhase = "final";
        request.outputs.push_back(handle);
    }
    return request;
}

// An exact output time: the default frame, or \p timeCode.
RigExecOutputTime
NumericTime(double timeCode)
{
    RigExecOutputTime time;
    RigExecOutputTime::TryTimeCode(timeCode, &time);
    return time;
}

// One ordered log shared by warm and explicit work: each job names itself, so
// a mixed-class test asserts the whole pop order at once.
class OrderLog {
public:
    RigExecWarmWork WarmWork(const std::string &label)
    {
        return [this, label](const RigExecWarmRequest &) {
            Add(label);
            return RigExecWarmOutcome::Published;
        };
    }
    RigExecExplicitWork ExplicitWork(const std::string &label)
    {
        return [this, label](const RigExecExplicitRequest &) {
            Add(label);
            return RigExecWarmOutcome::Published;
        };
    }

    std::vector<std::string> Entries() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _entries;
    }

private:
    void Add(const std::string &label)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _entries.push_back(label);
    }

    mutable std::mutex _mutex;
    std::vector<std::string> _entries;
};

// One executed explicit job, as the recording work observed it.
struct ExplicitRun {
    SdfPath rig;
    double time = 0.0;
    bool timeIsDefault = false;
    uint64_t programIdentity = 0;
    size_t outputs = 0;
    size_t reservationBytes = 0;
    RigExecWarmFenceToken fenceToken = 0;
    bool serialActive = false;
    bool currentAtPublish = false;
};

// Records explicit runs, and can publish only through the publish fence the
// way the production work does: ask IsExplicitRequestCurrent, decline when it
// says the request is gone.
class ExplicitRecorder {
public:
    explicit ExplicitRecorder(
        const RigExecBackgroundScheduler *scheduler = nullptr)
        : _scheduler(scheduler)
    {
    }

    RigExecExplicitWork
    Work(RigExecWarmOutcome outcome = RigExecWarmOutcome::Published)
    {
        return [this, outcome](const RigExecExplicitRequest &request) {
            const bool current = _Record(request);
            return current ? outcome : RigExecWarmOutcome::DeclinedGeneration;
        };
    }

    // A recorded job that waits inside the lane until the test releases it:
    // the bytes and the in-flight key it carries definitely stand while the
    // caller reads the books, and the release comes from the caller's own
    // thread, so a later WaitUntilIdle cannot deadlock however the threads
    // interleave.
    RigExecExplicitWork
    HeldWork(std::mutex &gateMutex, std::condition_variable &gateCv,
             bool &started, const bool &released,
             RigExecWarmOutcome outcome = RigExecWarmOutcome::Published)
    {
        return [this, &gateMutex, &gateCv, &started, &released,
                outcome](const RigExecExplicitRequest &request) {
            const bool current = _Record(request);
            {
                std::lock_guard<std::mutex> lock(gateMutex);
                started = true;
            }
            gateCv.notify_all();
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&released] { return released; });
            return current ? outcome : RigExecWarmOutcome::DeclinedGeneration;
        };
    }

    // A work closure that throws after recording: the books must still land.
    RigExecExplicitWork ThrowingWork()
    {
        return
            [this](
                const RigExecExplicitRequest &request) -> RigExecWarmOutcome {
                {
                    std::lock_guard<std::mutex> lock(_mutex);
                    _runs.push_back(ExplicitRun());
                }
                throw std::runtime_error("test throw");
            };
    }

    // Waits until a HeldWork job has reached the lane.
    static bool Started(std::mutex &gateMutex, std::condition_variable &gateCv,
                        const bool &started)
    {
        std::unique_lock<std::mutex> lock(gateMutex);
        return gateCv.wait_for(lock, std::chrono::seconds(10),
                               [&started] { return started; });
    }

    std::vector<ExplicitRun> Runs() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _runs;
    }

private:
    // Records one run and reports whether its request was still current.
    bool _Record(const RigExecExplicitRequest &request)
    {
        ExplicitRun run;
        run.rig = request.rig;
        run.timeIsDefault = request.time.IsDefault();
        run.time = run.timeIsDefault ? 0.0 : request.time.TimeCode();
        run.programIdentity = request.generation.programIdentity;
        run.outputs = request.outputs.size();
        run.reservationBytes = request.reservationBytes;
        run.fenceToken = request.fenceToken;
        run.serialActive = RigExecFrozenSerialActive();
        run.currentAtPublish =
            _scheduler ? _scheduler->IsExplicitRequestCurrent(request) : true;
        std::lock_guard<std::mutex> lock(_mutex);
        _runs.push_back(run);
        return run.currentAtPublish;
    }
    mutable std::mutex _mutex;
    std::vector<ExplicitRun> _runs;
    const RigExecBackgroundScheduler *_scheduler;
};

// The pool runs every explicit request ahead of every warm job, in the order
// the queue ranks them: explicit first, then warm neighbors, then sweep.
void
TestExplicitOutputRunsAheadOfWarming()
{
    std::printf("progress: TestExplicitOutputRunsAheadOfWarming\n");
    std::fflush(stdout);
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestExplicitOutputRunsAheadOfWarming: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    OrderLog log;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(1.0), RigExecWarmPriority::Sweep,
                            current, log.WarmWork("W1")));
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(2.0),
                            RigExecWarmPriority::Neighbor, current,
                            log.WarmWork("W2")));
    RigExecExplicitRequest five = MakeExplicitRequest(rig, NumericTime(5.0));
    five.editGeneration = current;
    RigExecExplicitRequest def = MakeExplicitRequest(rig, RigExecOutputTime());
    def.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(five, log.ExplicitWork("E5")));
    CHECK(scheduler.AdmitExplicitOutput(def, log.ExplicitWork("ED")));
    CHECK(scheduler.Stats().queuedDepth == 4);
    CHECK(scheduler.DrainQueueForTesting() == 4);
    const std::vector<std::string> entries = log.Entries();
    CHECK(entries == std::vector<std::string>({"E5", "ED", "W2", "W1"}));
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.completed == 2);
    CHECK(stats.explicitCompleted == 2);
    CHECK(stats.published == 2);
    CHECK(stats.explicitPublished == 2);
    CHECK(stats.explicitAccepted == 2);
    CHECK(stats.queuedDepth == 0);
    // The warm balance equation still counts warm accepts only, and the
    // explicit one balances against its own counters.
    CHECK(stats.published + stats.declinedInvalid + stats.declinedGeneration +
              stats.droppedStale + stats.canceled + stats.shed +
              stats.droppedAtShutdown + stats.queuedDepth + stats.running ==
          2);
    CHECK(stats.explicitAccepted - stats.explicitCoalesced ==
          stats.explicitPublished + stats.explicitDeclinedInvalid +
              stats.explicitDeclinedGeneration + stats.explicitDroppedStale +
              stats.explicitCanceled + stats.explicitDroppedAtShutdown +
              stats.explicitQueuedDepth + stats.explicitRunning);
}

// The exact time is an identity: the default frame, frame zero, negative zero,
// a subframe and a negative frame are five different demands, and negative
// zero coalesces with zero because the time normalizes it.
void
TestExplicitKeysKeepDefaultZeroAndSubframesApart()
{
    ScopedFrameCacheMode on("on");
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    ExplicitRecorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const RigExecOutputTime times[] = {RigExecOutputTime(), NumericTime(0.0),
                                       NumericTime(-0.0), NumericTime(3.5),
                                       NumericTime(-2.25)};
    size_t accepted = 0;
    for (const RigExecOutputTime &time : times) {
        RigExecExplicitRequest request = MakeExplicitRequest(rig, time);
        request.editGeneration = current;
        accepted +=
            scheduler.AdmitExplicitOutput(request, recorder.Work()) ? 1 : 0;
    }
    CHECK(accepted == 5);
    CHECK(scheduler.Stats().explicitCoalesced == 1);
    const std::vector<RigExecExplicitJobKey> keys =
        scheduler.QueuedExplicitKeys(rig);
    CHECK(keys.size() == 4);
    // Key order is rig, then the exact time with the default frame first.
    CHECK(keys[0].time.IsDefault());
    CHECK(!keys[1].time.IsDefault() && keys[1].time.TimeCode() == -2.25);
    CHECK(!keys[2].time.IsDefault() && keys[2].time.TimeCode() == 0.0);
    CHECK(!keys[3].time.IsDefault() && keys[3].time.TimeCode() == 3.5);
    CHECK(scheduler.DrainQueueForTesting() == 4);
    CHECK(recorder.Runs().size() == 4);
}

// Provenance is part of the key: a different program, input, preview or
// history revision is a second job, and warming a time never coalesces with
// an explicit request for the same time.
void
TestExplicitGenerationFieldsNeverCoalesce()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestExplicitGenerationFieldsNeverCoalesce: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    ExplicitRecorder recorder;
    OrderLog log;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    const RigExecOutputTime five = NumericTime(5.0);
    const uint64_t generations[][4] = {
        {1, 0, 0, 0}, {2, 0, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}};
    for (const auto &generation : generations) {
        RigExecExplicitRequest request =
            MakeExplicitRequest(rig, five, generation[0], generation[1],
                                generation[2], generation[3]);
        request.editGeneration = current;
        CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()));
    }
    // The identical key coalesces: the same demand twice is one job.
    RigExecExplicitRequest again = MakeExplicitRequest(rig, five, 1, 0, 0, 0);
    again.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(again, recorder.Work()));
    CHECK(scheduler.Stats().explicitCoalesced == 1);
    CHECK(scheduler.QueuedExplicitKeys(rig).size() == 5);
    // A warm job for the same rig and time is a separate job: the rounded
    // (rig, time) warm key never merges an explicit request.
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(5.0), RigExecWarmPriority::Sweep,
                            current, log.WarmWork("W5")));
    CHECK(scheduler.Stats().coalesced == 0);
    CHECK(scheduler.Stats().upgraded == 0);
    CHECK(scheduler.QueuedExplicitKeys(rig).size() == 5);
    CHECK(scheduler.Stats().queuedDepth == 6);
    CHECK(scheduler.DrainQueueForTesting() == 6);
    CHECK(recorder.Runs().size() == 5);
}

// Explicit demand is eligible whatever the speculative-warming switches say:
// warm-off and off close the warm queue only, and a host that asked for a
// frame still gets it from the pool.
void
TestExplicitDemandIsEligibleWithWarmingOff()
{
    for (const char *mode : {"warm-off", "off"}) {
        ScopedFrameCacheMode closed(mode);
        CHECK(!RigExecBackgroundWarmingEnabled());
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        ExplicitRecorder recorder(&scheduler);
        OrderLog warm;
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        CHECK(!scheduler.Enqueue(rig, UsdTimeCode(3.0),
                                 RigExecWarmPriority::Neighbor, current,
                                 warm.WarmWork("W3")));
        const std::vector<UsdTimeCode> sweep{UsdTimeCode(4.0)};
        CHECK(scheduler.OnIdle(rig, UsdTimeCode(10.0), current, sweep,
                               RigExecWarmJobFactory()) == 0);
        RigExecExplicitRequest request =
            MakeExplicitRequest(rig, NumericTime(4.0));
        request.editGeneration = current;
        request.reservationBytes = 24 * 8;
        CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()));
        CHECK(scheduler.ReservedExplicitBytes() == 24 * 8);
        CHECK(scheduler.RunNextQueuedForTesting());
        CHECK(recorder.Runs().size() == 1);
        CHECK(recorder.Runs()[0].currentAtPublish);
        CHECK(scheduler.Stats().explicitPublished == 1);
        CHECK(scheduler.ReservedExplicitBytes() == 0);
        // The fence still works with warming off.
        scheduler.CancelGeneration(rig);
        CHECK(scheduler.CurrentGeneration(rig) == current + 1);
    }
}

// The explicit class is fair per rig: with two rigs queued, the pool takes
// them in turn instead of draining the first rig's whole burst first.
void
TestExplicitRequestsTakeTheRigsInTurn()
{
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    OrderLog log;
    const SdfPath rigA("/Asset/RigA");
    const SdfPath rigB("/Asset/RigB");
    const RigExecFrameGeneration currentA = scheduler.CurrentGeneration(rigA);
    const RigExecFrameGeneration currentB = scheduler.CurrentGeneration(rigB);
    for (int frame = 1; frame <= 3; ++frame) {
        RigExecExplicitRequest request =
            MakeExplicitRequest(rigA, NumericTime(double(frame)));
        request.editGeneration = currentA;
        CHECK(scheduler.AdmitExplicitOutput(
            request, log.ExplicitWork("A" + std::to_string(frame))));
    }
    for (int frame = 1; frame <= 3; ++frame) {
        RigExecExplicitRequest request =
            MakeExplicitRequest(rigB, NumericTime(double(frame)));
        request.editGeneration = currentB;
        CHECK(scheduler.AdmitExplicitOutput(
            request, log.ExplicitWork("B" + std::to_string(frame))));
    }
    CHECK(scheduler.ExplicitInFlightCount(rigA) == 3);
    CHECK(scheduler.ExplicitInFlightCount(rigB) == 3);
    CHECK(scheduler.DrainQueueForTesting() == 6);
    const std::vector<std::string> entries = log.Entries();
    CHECK(entries ==
          std::vector<std::string>({"A1", "B1", "A2", "B2", "A3", "B3"}));
    CHECK(scheduler.ExplicitInFlightCount(rigA) == 0);
}

// An admission with no work, no output, or handles from two different
// provenances declines and reserves nothing: a mixed group would answer two
// compile epochs from one job.
void
TestExplicitDeclinesForNoWorkNoOutputOrMixedProvenance()
{
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    ExplicitRecorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecExplicitRequest request =
        MakeExplicitRequest(rig, NumericTime(3.0),
                            /*programIdentity=*/1,
                            /*inputRevision=*/0,
                            /*previewRevision=*/0,
                            /*historyRevision=*/0,
                            /*reservationBytes=*/64);
    request.editGeneration = current;
    CHECK(!scheduler.AdmitExplicitOutput(request, RigExecExplicitWork()));
    RigExecExplicitRequest noOutputs = MakeExplicitRequest(
        rig, NumericTime(3.0), 1, 0, 0, 0, 64, /*outputCount=*/0);
    noOutputs.editGeneration = current;
    CHECK(!scheduler.AdmitExplicitOutput(noOutputs, recorder.Work()));
    RigExecExplicitRequest mixedEpoch = MakeExplicitRequest(
        rig, NumericTime(3.0), 1, 0, 0, 0, 64, /*outputCount=*/2);
    mixedEpoch.editGeneration = current;
    mixedEpoch.outputs.back().compileEpoch = 2;
    CHECK(!scheduler.AdmitExplicitOutput(mixedEpoch, recorder.Work()));
    RigExecExplicitRequest mixedRig = MakeExplicitRequest(
        rig, NumericTime(3.0), 1, 0, 0, 0, 64, /*outputCount=*/2);
    mixedRig.editGeneration = current;
    mixedRig.outputs.back().rigIdentity = "/Asset/Other";
    CHECK(!scheduler.AdmitExplicitOutput(mixedRig, recorder.Work()));
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.explicitDeclined == 4);
    CHECK(stats.explicitAccepted == 0);
    CHECK(stats.queuedDepth == 0);
    CHECK(scheduler.ReservedExplicitBytes() == 0);
}

// The explicit caps are independent of the warm cap: the per-rig explicit cap
// declines, a stale edit generation declines, and a freed slot admits again.
void
TestExplicitCapAndStaleGenerationDecline()
{
    RigExecBackgroundScheduler probe;
    CHECK(probe.GetExplicitPerRigJobCap() ==
          kRigExecBackgroundSchedulerDefaultExplicitPerRigJobs);
    CHECK(probe.GetExplicitByteCap() ==
          kRigExecBackgroundSchedulerDefaultExplicitBytes);
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetExplicitPerRigJobCap(2);
    CHECK(scheduler.GetExplicitPerRigJobCap() == 2);
    ExplicitRecorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    for (int frame = 1; frame <= 3; ++frame) {
        RigExecExplicitRequest request =
            MakeExplicitRequest(rig, NumericTime(double(frame)));
        request.editGeneration = current;
        const bool expected = frame <= 2;
        CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()) ==
              expected);
    }
    CHECK(scheduler.Stats().explicitDeclined == 1);
    CHECK(scheduler.ExplicitInFlightCount(rig) == 2);
    // A run frees a slot.
    CHECK(scheduler.RunNextQueuedForTesting());
    RigExecExplicitRequest fourth = MakeExplicitRequest(rig, NumericTime(4.0));
    fourth.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(fourth, recorder.Work()));
    // A stale edit generation declines: the compiled rig moved under it.
    scheduler.CancelGeneration(rig);
    RigExecExplicitRequest stale = MakeExplicitRequest(rig, NumericTime(7.0));
    stale.editGeneration = current;
    CHECK(!scheduler.AdmitExplicitOutput(stale, recorder.Work()));
    CHECK(scheduler.Stats().explicitDeclined == 2);
    // The cancel purged the rig's queued explicit jobs too.
    CHECK(scheduler.Stats().explicitCanceled == 2);
    CHECK(scheduler.QueuedExplicitKeys(rig).empty());
    CHECK(scheduler.ExplicitInFlightCount(rig) == 0);
    RigExecExplicitRequest fresh = MakeExplicitRequest(rig, NumericTime(7.0));
    fresh.editGeneration = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.AdmitExplicitOutput(fresh, recorder.Work()));
}

// The byte reservation is a bound checked before anything queues: a request
// past it declines, and running or purging a job frees its bytes.
void
TestExplicitByteReservationDeclinesPastTheCap()
{
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    scheduler.SetExplicitByteCap(256);
    CHECK(scheduler.GetExplicitByteCap() == 256);
    ExplicitRecorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecExplicitRequest tooBig = MakeExplicitRequest(
        rig, NumericTime(1.0), 1, 0, 0, 0, /*reservationBytes=*/257);
    tooBig.editGeneration = current;
    CHECK(!scheduler.AdmitExplicitOutput(tooBig, recorder.Work()));
    CHECK(scheduler.Stats().explicitDeclinedBytes == 1);
    for (int frame = 1; frame <= 2; ++frame) {
        RigExecExplicitRequest request = MakeExplicitRequest(
            rig, NumericTime(double(frame)), 1, 0, 0, 0, /*bytes=*/128);
        request.editGeneration = current;
        CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()));
    }
    CHECK(scheduler.ReservedExplicitBytes() == 256);
    CHECK(scheduler.ReservedExplicitBytesFor(rig) == 256);
    RigExecExplicitRequest oneMore =
        MakeExplicitRequest(rig, NumericTime(3.0), 1, 0, 0, 0, /*bytes=*/1);
    oneMore.editGeneration = current;
    CHECK(!scheduler.AdmitExplicitOutput(oneMore, recorder.Work()));
    CHECK(scheduler.Stats().explicitDeclinedBytes == 2);
    // A ran job releases its bytes; then the room is there.
    CHECK(scheduler.RunNextQueuedForTesting());
    CHECK(scheduler.ReservedExplicitBytes() == 128);
    CHECK(scheduler.AdmitExplicitOutput(oneMore, recorder.Work()));
    CHECK(scheduler.ReservedExplicitBytes() == 129);
    // A purge releases them too.
    scheduler.CancelGenerationTimes(rig, {UsdTimeCode(2.0), UsdTimeCode(1.0)});
    CHECK(scheduler.ReservedExplicitBytes() == 1);
    CHECK(scheduler.ReservedExplicitBytesFor(rig) == 1);
    // The reservation is the payload arithmetic the contract declares: a
    // 100-point Point3 payload is 2400 bytes, and an overflowing count is
    // refused before any allocation.
    uint64_t bytes = 0;
    CHECK(RigExecPayloadByteCount(RigExecPayloadKind::Point3, 100, &bytes));
    CHECK(bytes == 2400);
    CHECK(!RigExecPayloadByteCount(RigExecPayloadKind::Point3,
                                   std::numeric_limits<uint64_t>::max(),
                                   &bytes));
}

// Every terminal path releases the reservation: published, declined inside,
// threw, and purged all leave no bytes held, and the drop-at-shutdown path
// runs the same retire for a job still queued at destruction.
void
TestExplicitReservationReleasesOnEveryPath()
{
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        ExplicitRecorder recorder;
        const double frames[] = {1.0, 2.0, 3.0, 4.0, 5.0};
        RigExecExplicitWork declined = [](const RigExecExplicitRequest &) {
            return RigExecWarmOutcome::DeclinedInvalid;
        };
        const RigExecExplicitWork works[] = {recorder.Work(),
                                             recorder.ThrowingWork(), declined,
                                             recorder.Work(), recorder.Work()};
        for (size_t i = 0; i < 5; ++i) {
            RigExecExplicitRequest request = MakeExplicitRequest(
                rig, NumericTime(frames[i]), 1, 0, 0, 0, /*bytes=*/64);
            request.editGeneration = current;
            CHECK(scheduler.AdmitExplicitOutput(request, works[i]));
        }
        CHECK(scheduler.ReservedExplicitBytes() == 5 * 64);
        // Pop them one at a time: published, threw, declined inside, ran.
        for (int pop = 0; pop < 4; ++pop) {
            CHECK(scheduler.RunNextQueuedForTesting());
        }
        const RigExecBackgroundSchedulerStats running = scheduler.Stats();
        CHECK(running.explicitPublished == 2);
        CHECK(running.explicitDeclinedInvalid == 2);
        CHECK(running.explicitReservedBytes == 64);
        // A scoped purge retires the job that is still queued.
        CHECK(scheduler.CancelGenerationTimes(rig, {UsdTimeCode(5.0)}) == 1);
        CHECK(scheduler.Stats().explicitCanceled == 1);
        CHECK(scheduler.ReservedExplicitBytes() == 0);
        CHECK(scheduler.QueuedExplicitKeys(rig).empty());
    }
    // The shutdown retire path is the same one, and it runs for a job that
    // never started.
    TransitionRecorder transitions;
    {
        RigExecBackgroundScheduler stopping(/*workerCount=*/0);
        stopping.SetTransitionHook(transitions.Hook());
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = stopping.CurrentGeneration(rig);
        ExplicitRecorder recorder;
        RigExecExplicitRequest request =
            MakeExplicitRequest(rig, RigExecOutputTime(), 1, 0, 0, 0, 64);
        request.editGeneration = current;
        CHECK(stopping.AdmitExplicitOutput(request, recorder.Work()));
        CHECK(stopping.ReservedExplicitBytes() == 64);
    }
    const std::vector<RigExecWarmTransition> lost = transitions.Transitions();
    size_t droppedAtShutdown = 0;
    for (const RigExecWarmTransition &transition : lost) {
        if (transition.kind == RigExecWarmTransitionKind::DroppedAtShutdown) {
            ++droppedAtShutdown;
            CHECK(transition.workClass == RigExecBackgroundWorkClass::Explicit);
            CHECK(transition.timeIsDefault);
        }
    }
    CHECK(droppedAtShutdown == 1);
}

// The books balance across a cancel racing the pool: every explicit job lands
// in exactly one terminal state, and no byte stays reserved.
void
TestExplicitBalancesUnderACancelRace()
{
    std::printf("progress: TestExplicitBalancesUnderACancelRace\n");
    std::fflush(stdout);
    RigExecBackgroundScheduler scheduler(/*workerCount=*/2);
    ExplicitRecorder recorder;
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    size_t accepted = 0;
    for (int frame = 1; frame <= 32; ++frame) {
        RigExecExplicitRequest request = MakeExplicitRequest(
            rig, NumericTime(double(frame)), 1, 0, 0, 0, /*bytes=*/128);
        request.editGeneration = current;
        // The first job is held inside the lane, so the reservations and the
        // in-flight keys the loop took out definitely stand while the books
        // are read below, and the cancel lands on a job that is mid-run.
        RigExecExplicitWork work =
            frame == 1 ? recorder.HeldWork(gateMutex, gateCv, started, released)
                       : recorder.Work();
        accepted +=
            scheduler.AdmitExplicitOutput(request, std::move(work)) ? 1 : 0;
    }
    CHECK(accepted == 32);
    CHECK(ExplicitRecorder::Started(gateMutex, gateCv, started));
    CHECK(scheduler.ReservedExplicitBytes() > 0);
    scheduler.CancelGeneration(rig);
    // The release arrives from this thread, so WaitUntilIdle below cannot
    // deadlock however the threads interleave. The flag moves under the gate
    // mutex, which the held job re-reads before sleeping.
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        released = true;
    }
    gateCv.notify_all();
    scheduler.WaitUntilIdle();
    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.queuedDepth == 0);
    CHECK(stats.running == 0);
    CHECK(stats.explicitQueuedDepth == 0);
    CHECK(stats.explicitRunning == 0);
    CHECK(scheduler.ReservedExplicitBytes() == 0);
    CHECK(scheduler.ReservedExplicitBytesFor(rig) == 0);
    CHECK(stats.explicitAccepted - stats.explicitCoalesced ==
          stats.explicitPublished + stats.explicitDeclinedInvalid +
              stats.explicitDeclinedGeneration + stats.explicitDroppedStale +
              stats.explicitCanceled + stats.explicitDroppedAtShutdown +
              stats.explicitQueuedDepth + stats.explicitRunning);
    CHECK(stats.explicitPublished + stats.explicitDeclinedInvalid +
              stats.explicitDeclinedGeneration ==
          stats.explicitCompleted);
    CHECK(recorder.Runs().size() == stats.explicitCompleted);
}

// Scoped cancellation re-fences explicit requests, including the default
// frame, which has no numeric time to key on: an old request never answers,
// and the re-admitted one carries the fresh token.
void
TestScopedCancellationRefencesExplicitRequests()
{
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    ExplicitRecorder recorder(&scheduler);
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecExplicitRequest five =
        MakeExplicitRequest(rig, NumericTime(5.0), 1, 0, 0, 0, /*bytes=*/16);
    five.editGeneration = current;
    RigExecExplicitRequest def =
        MakeExplicitRequest(rig, RigExecOutputTime(), 1, 0, 0, 0, /*bytes=*/16);
    def.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(five, recorder.Work()));
    CHECK(scheduler.AdmitExplicitOutput(def, recorder.Work()));
    CHECK(scheduler.ReservedExplicitBytes() == 32);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, NumericTime(5.0)) == 0);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, RigExecOutputTime()) == 0);
    // A patch at frame 9 moves the default frame's inputs, so the
    // default-frame demand is retired even though no listed time names it;
    // frame 5 keeps its token, its job and its fence.
    CHECK(scheduler.CancelGenerationTimes(rig, {UsdTimeCode(9.0)}) == 1);
    CHECK(scheduler.QueuedExplicitKeys(rig).size() == 1);
    CHECK(scheduler.ReservedExplicitBytes() == 16);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, NumericTime(5.0)) == 0);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, RigExecOutputTime()) != 0);
    CHECK(scheduler.IsExplicitRequestCurrent(five));
    CHECK(!scheduler.IsExplicitRequestCurrent(def));
    // A patch at frame 5 retires frame 5's job too.
    CHECK(scheduler.CancelGenerationTimes(rig, {UsdTimeCode(5.0)}) == 1);
    CHECK(scheduler.Stats().explicitCanceled == 2);
    CHECK(scheduler.QueuedExplicitKeys(rig).empty());
    CHECK(scheduler.ReservedExplicitBytes() == 0);
    CHECK(scheduler.CurrentGeneration(rig) == current);
    // The old requests are fenced out: their token is superseded.
    CHECK(!scheduler.IsExplicitRequestCurrent(five));
    CHECK(!scheduler.IsExplicitRequestCurrent(def));
    CHECK(scheduler.CurrentExplicitFenceToken(rig, NumericTime(5.0)) != 0);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, RigExecOutputTime()) != 0);
    // Re-admitted under the new tokens they run and publish.
    RigExecExplicitRequest freshFive =
        MakeExplicitRequest(rig, NumericTime(5.0));
    freshFive.editGeneration = current;
    RigExecExplicitRequest freshDef =
        MakeExplicitRequest(rig, RigExecOutputTime());
    freshDef.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(freshFive, recorder.Work()));
    CHECK(scheduler.AdmitExplicitOutput(freshDef, recorder.Work()));
    CHECK(scheduler.DrainQueueForTesting() == 2);
    const std::vector<ExplicitRun> runs = recorder.Runs();
    CHECK(runs.size() == 2);
    CHECK(runs[0].currentAtPublish);
    CHECK(runs[0].fenceToken != 0);
    CHECK(runs[1].currentAtPublish);
    CHECK(runs[1].timeIsDefault);
    CHECK(scheduler.Stats().explicitPublished == 2);
    CHECK(scheduler.Stats().explicitDeclinedGeneration == 0);
}

// Deactivation drops the rig's explicit state with its warm state: queued
// jobs, in-flight count, reservations and fence tokens go, the generation
// stands, and nothing leaks into the reactivated rig.
void
TestResetRigClearsExplicitState()
{
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    ExplicitRecorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecExplicitRequest request =
        MakeExplicitRequest(rig, NumericTime(5.0), 1, 0, 0, 0, /*bytes=*/64);
    request.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()));
    CHECK(scheduler.CancelGenerationTimes(rig, {UsdTimeCode(5.0)}) == 1);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, NumericTime(5.0)) != 0);
    RigExecExplicitRequest requeue =
        MakeExplicitRequest(rig, NumericTime(6.0), 1, 0, 0, 0, /*bytes=*/64);
    requeue.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(requeue, recorder.Work()));
    CHECK(scheduler.ReservedExplicitBytes() == 64);
    scheduler.ResetRig(rig);
    CHECK(scheduler.QueuedExplicitKeys(rig).empty());
    CHECK(scheduler.ExplicitInFlightCount(rig) == 0);
    CHECK(scheduler.ReservedExplicitBytes() == 0);
    CHECK(scheduler.ReservedExplicitBytesFor(rig) == 0);
    CHECK(scheduler.CurrentExplicitFenceToken(rig, NumericTime(5.0)) == 0);
    CHECK(scheduler.CurrentGeneration(rig) == current);
    RigExecExplicitRequest again = MakeExplicitRequest(rig, NumericTime(5.0));
    again.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(again, recorder.Work()));
    CHECK(scheduler.IsExplicitRequestCurrent(again));
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(recorder.Runs().back().currentAtPublish);
}

// A cancellation landing while an explicit job runs must not release its
// reservation twice: the running job releases its own bytes when it finishes.
void
TestExplicitMidRunCancelKeepsTheBooksBalanced()
{
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;

    RigExecBackgroundScheduler scheduler(1);
    const SdfPath rig("/Rig/Mid");
    RigExecExplicitRequest running = MakeExplicitRequest(rig, NumericTime(4.0));
    running.reservationBytes = 4096;
    CHECK(scheduler.AdmitExplicitOutput(
        running, [&](const RigExecExplicitRequest &request) {
            (void)request;
            {
                std::lock_guard<std::mutex> lock(gateMutex);
                started = true;
            }
            gateCv.notify_all();
            std::unique_lock<std::mutex> lock(gateMutex);
            gateCv.wait(lock, [&] { return released; });
            return RigExecExplicitOutcome::Published;
        }));

    // The cancel lands while the job is held in its wait, so the purge is
    // definitely mid-run with nothing queued behind it. The release then
    // arrives from the same thread, so WaitUntilIdle below cannot deadlock
    // however the threads interleave.
    std::thread releaser([&] {
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCv.wait(lock, [&] { return started; });
        lock.unlock();
        // The cancel runs with the gate closed: it takes the scheduler's own
        // book and must not nest under the gate mutex.
        scheduler.CancelGeneration(rig);
        {
            std::lock_guard<std::mutex> releaseLock(gateMutex);
            released = true;
        }
        gateCv.notify_all();
    });
    releaser.join();
    scheduler.WaitUntilIdle();

    const RigExecBackgroundSchedulerStats stats = scheduler.Stats();
    CHECK(stats.explicitAccepted == 1);
    CHECK(stats.explicitCompleted == 1);
    CHECK(stats.explicitReservedBytes == 0);
    CHECK(scheduler.ReservedExplicitBytes() == 0);
    CHECK(scheduler.InFlightExplicitKeys(rig).empty());
    CHECK(scheduler.ExplicitInFlightCount(rig) == 0);

    // The books still admit a fresh request under the new generation.
    ExplicitRecorder recorder(&scheduler);
    RigExecExplicitRequest again = MakeExplicitRequest(rig, NumericTime(4.0));
    again.editGeneration = scheduler.CurrentGeneration(rig);
    again.reservationBytes = 4096;
    CHECK(scheduler.AdmitExplicitOutput(again, recorder.Work()));
    scheduler.WaitUntilIdle();
    CHECK(scheduler.Stats().explicitReservedBytes == 0);
    CHECK(recorder.Runs().back().currentAtPublish);
}

// Playback advance sheds speculative sweep only: a frame a host asked for is
// never shed, and its reservation stands until the job runs.
void
TestPlaybackAdvanceNeverShedsExplicitOutput()
{
    ScopedFrameCacheMode on("on");
    if (!RigExecBackgroundWarmingEnabled()) {
        std::printf("SKIP TestPlaybackAdvanceNeverShedsExplicitOutput: "
                    "warming unavailable in this process\n");
        return;
    }
    RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
    OrderLog warm;
    ExplicitRecorder recorder;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    CHECK(scheduler.Enqueue(rig, UsdTimeCode(9.0), RigExecWarmPriority::Sweep,
                            current, warm.WarmWork("W9")));
    RigExecExplicitRequest request =
        MakeExplicitRequest(rig, NumericTime(9.0), 1, 0, 0, 0, /*bytes=*/64);
    request.editGeneration = current;
    CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()));
    CHECK(scheduler.NotePlaybackAdvanced(rig, UsdTimeCode(10.0)) == 1);
    CHECK(scheduler.Stats().shed == 1);
    CHECK(scheduler.QueuedExplicitKeys(rig).size() == 1);
    CHECK(scheduler.ReservedExplicitBytes() == 64);
    CHECK(scheduler.DrainQueueForTesting() == 1);
    CHECK(scheduler.ReservedExplicitBytes() == 0);
    CHECK(recorder.Runs().size() == 1);
}

// The transition hook sees explicit moves, tagged by class, and says when the
// time names no frame at all.
void
TestTransitionHookReportsExplicitMoves()
{
    TransitionRecorder transitions;
    {
        RigExecBackgroundScheduler scheduler(/*workerCount=*/0);
        scheduler.SetTransitionHook(transitions.Hook());
        const SdfPath rig("/Asset/Rig");
        const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
        ExplicitRecorder recorder;
        RigExecExplicitRequest request = MakeExplicitRequest(
            rig, RigExecOutputTime(), 1, 0, 0, 0, /*bytes=*/8);
        request.editGeneration = current;
        CHECK(scheduler.AdmitExplicitOutput(request, recorder.Work()));
        CHECK(scheduler.RunNextQueuedForTesting());
    }
    const std::vector<RigExecWarmTransition> got = transitions.Transitions();
    CHECK(got.size() == 3);
    if (got.size() == 3) {
        CHECK(got[0].kind == RigExecWarmTransitionKind::Queued);
        CHECK(got[0].workClass == RigExecBackgroundWorkClass::Explicit);
        CHECK(got[0].timeIsDefault);
        CHECK(got[1].kind == RigExecWarmTransitionKind::Running);
        CHECK(got[1].workClass == RigExecBackgroundWorkClass::Explicit);
        CHECK(got[2].kind == RigExecWarmTransitionKind::Finished);
        CHECK(got[2].outcome == RigExecWarmOutcome::Published);
        CHECK(got[2].workClass == RigExecBackgroundWorkClass::Explicit);
    }
}

// The work runs on the same serial boundary as warming, receives the request
// it was admitted with -- its outputs, its generation, its reservation and
// the stamped token -- and in-flight explicit keys stay visible to a scoped
// retirement query.
void
TestExplicitWorkRunsUnderTheSerialScopeWithItsRequest()
{
    std::printf("progress: "
                "TestExplicitWorkRunsUnderTheSerialScopeWithItsRequest\n");
    std::fflush(stdout);
    RigExecBackgroundScheduler scheduler(/*workerCount=*/1);
    ExplicitRecorder recorder(&scheduler);
    std::mutex gateMutex;
    std::condition_variable gateCv;
    bool started = false;
    bool released = false;
    const SdfPath rig("/Asset/Rig");
    const RigExecFrameGeneration current = scheduler.CurrentGeneration(rig);
    RigExecExplicitRequest request = MakeExplicitRequest(
        rig, NumericTime(-2.5), /*programIdentity=*/7, /*input=*/3,
        /*preview=*/2, /*history=*/1, /*bytes=*/96, /*outputs=*/4);
    request.editGeneration = current;
    // The job is held in the lane, so its key and its bytes are in the books
    // for the query below rather than gone before the caller reads them.
    CHECK(scheduler.AdmitExplicitOutput(
        request, recorder.HeldWork(gateMutex, gateCv, started, released)));
    CHECK(ExplicitRecorder::Started(gateMutex, gateCv, started));
    const std::vector<RigExecExplicitJobKey> running =
        scheduler.InFlightExplicitKeys(rig);
    CHECK(running.size() == 1);
    CHECK(scheduler.ExplicitInFlightCount(rig) == 1);
    CHECK(scheduler.ReservedExplicitBytes() == 96);
    {
        std::lock_guard<std::mutex> lock(gateMutex);
        released = true;
    }
    gateCv.notify_all();
    scheduler.WaitUntilIdle();
    const std::vector<ExplicitRun> runs = recorder.Runs();
    CHECK(runs.size() == 1);
    CHECK(runs[0].rig == rig);
    CHECK(!runs[0].timeIsDefault);
    CHECK(runs[0].time == -2.5);
    CHECK(runs[0].programIdentity == 7);
    CHECK(runs[0].outputs == 4);
    CHECK(runs[0].reservationBytes == 96);
    CHECK(runs[0].fenceToken == 0);
    CHECK(runs[0].serialActive);
    CHECK(runs[0].currentAtPublish);
    CHECK(!RigExecFrozenSerialActive());
    CHECK(scheduler.InFlightExplicitKeys(rig).empty());
    CHECK(scheduler.ReservedExplicitBytes() == 0);
}

} // namespace

int
main()
{
    TestAnUnseenRigSitsAtGenerationZero();
    TestCancelBumpsMonotonically();
    TestRigsAreIndependent();
    TestAStaleJobIsRefused();
    TestFrameCacheModeParses();
    TestWarmingGateTruthTable();
    TestFrameCacheModeReadsLiveFromTheEnvironment();
    TestClosedGateDeclinesWarmingButKeepsTheFence();
    TestACurrentJobIsAcceptedNeighborBeforeSweep();
    TestFifoWithinAPriority();
    TestDuplicateTimesCoalesce();
    TestSweepPromotesToNeighborOnReenqueue();
    TestUnusableEnqueuesDecline();
    TestPerRigCapDeclinesPastItsLimit();
    TestCancelPurgesQueuedJobsPromptly();
    TestCommitTriggerEnqueuesNeighborsThenSweepNeverPlayhead();
    TestTriggerSkipsUnsampleableAndStopsAtCap();
    TestTriggerWithStaleGenerationOrDefaultPlayheadEnqueuesNothing();
    TestTriggerStopsWhenAnEditLandsMidTrigger();
    TestIdleTriggerEnqueuesSweepOnly();
    TestPlaybackAdvanceShedsPassedSweep();
    TestPlaybackAdvanceWithNaNShedsNothing();
    TestProfilerLaneRecordsQueueAndCancels();
    TestStatsBalanceOverAMixedScenario();
    TestOutcomesSplitCompletion();
    TestTransitionHookReportsQueueVisibility();
    TestPoolPreservesPriorityOrder();
    TestNoRigStarvesUnderFlood();
    TestSamplingBudgetBoundsFactoryInvocations();
    TestInjectedClockStampsQueueWaitAndRunTime();
    TestDeclinedEnqueuesSplitByReason();
    TestQueueDepthMaximaAndPrioritySplit();
    TestTriggerStopsSplitByCause();
    TestExplicitOutputRunsAheadOfWarming();
    TestExplicitKeysKeepDefaultZeroAndSubframesApart();
    TestExplicitGenerationFieldsNeverCoalesce();
    TestExplicitDemandIsEligibleWithWarmingOff();
    TestExplicitRequestsTakeTheRigsInTurn();
    TestExplicitDeclinesForNoWorkNoOutputOrMixedProvenance();
    TestExplicitCapAndStaleGenerationDecline();
    TestExplicitByteReservationDeclinesPastTheCap();
    TestExplicitReservationReleasesOnEveryPath();
    TestExplicitBalancesUnderACancelRace();
    TestScopedCancellationRefencesExplicitRequests();
    TestResetRigClearsExplicitState();
    TestExplicitMidRunCancelKeepsTheBooksBalanced();
    TestPlaybackAdvanceNeverShedsExplicitOutput();
    TestTransitionHookReportsExplicitMoves();
    TestExplicitWorkRunsUnderTheSerialScopeWithItsRequest();
    TestPoolRunsEveryJobExactlyOnce();
    TestThreadedCancelBalancesTheBooks();
    TestShutdownDropsPendingJobs();
    TestShutdownJoinsARunningJob();
    TestCancelPurgesWhileAJobRuns();
    TestScopedRetirementIncludesRunningFirstFills();
    TestWorkerCountClampsToRange();
    TestPriorityBackendReportsHonestly();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBackgroundScheduler: all tests passed\n");
    return 0;
}
