// RigExec background scheduler. See backgroundScheduler.h.
// The pool is std::thread only: no Work, no TBB anywhere in this file. A
// kernel-level parallel call on a worker would run its tasks on the shared
// arena at normal priority, past the below-normal boundary the pool exists to
// hold -- so each job runs under a RigExecFrozenSerialScope instead, and the
// OS priority call sits behind the one function the plan's risk table asks
// for, with a logged no-op fallback.
#include "backgroundScheduler.h"

#include "frozenContext.h"
#include "parallel.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <set>

#if defined(_WIN32)
// NOMINMAX and WIN32_LEAN_AND_MEAN arrive on the command line; windows.h
// stays in this translation unit only.
#include <windows.h>
#elif defined(__linux__)
#include <pthread.h>
#include <sched.h>
#elif defined(__APPLE__)
#include <pthread.h>
#include <sys/qos.h>
#endif

namespace rigExec {

namespace {

// Warns once per process per call site: the priority fallback, the thread
// shortfall, and an unrecognized switch value are host configuration, worth
// one line each, never a per-frame line.
void
_WarnPriorityOnce(const char *message)
{
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
        TF_WARN("rigExec: %s", message);
    }
}

void
_WarnSwitchOnce(const char *message)
{
    static std::atomic<bool> warned{false};
    if (!warned.exchange(true)) {
        TF_WARN("rigExec: %s", message);
    }
}

// True when every handle of one admission names the same rig identity and
// compile epoch, and names an output at all: the group rule outputDemand.h
// states for a whole request, applied to one exact-time demand before the
// pool reserves a byte for it. A mixed group would answer two provenances
// from one job.
bool
_OneProvenanceGroup(const std::vector<RigExecOutputHandle> &outputs)
{
    if (outputs.empty()) {
        return false;
    }
    const std::string &rigIdentity = outputs.front().rigIdentity;
    const uint64_t compileEpoch = outputs.front().compileEpoch;
    if (rigIdentity.empty()) {
        return false;
    }
    for (const RigExecOutputHandle &handle : outputs) {
        if (handle.rigIdentity != rigIdentity ||
            handle.compileEpoch != compileEpoch || handle.semanticKey.empty() ||
            handle.type.empty()) {
            return false;
        }
    }
    return true;
}

} // namespace

RigExecFrameCacheMode
RigExecParseFrameCacheMode(const std::string &value)
{
    if (value == "on") {
        return RigExecFrameCacheMode::On;
    }
    if (value == "off") {
        return RigExecFrameCacheMode::Off;
    }
    if (value == "warm-off") {
        return RigExecFrameCacheMode::WarmOff;
    }
    return RigExecFrameCacheMode::On;
}

RigExecFrameCacheMode
RigExecFrameCacheModeFromEnvironment()
{
    const std::string value = TfGetenv("RIGEXEC_FRAME_CACHE", "on");
    const RigExecFrameCacheMode mode = RigExecParseFrameCacheMode(value);
    if (mode == RigExecFrameCacheMode::On && value != "on") {
        _WarnSwitchOnce("RIGEXEC_FRAME_CACHE has an unrecognized value; "
                        "using on (want one of: on, off, warm-off)");
    }
    return mode;
}

bool
RigExecBackgroundWarmingEnabled(RigExecFrameCacheMode mode,
                                bool parallelEvalEnabled)
{
    return mode == RigExecFrameCacheMode::On && parallelEvalEnabled;
}

bool
RigExecBackgroundWarmingEnabled()
{
    return RigExecBackgroundWarmingEnabled(
        RigExecFrameCacheModeFromEnvironment(),
        RigExecParallelEvaluationEnabled());
}

bool
RigExecSetBackgroundThreadPriority()
{
#if defined(_WIN32)
    // Below-normal, not background mode: the workers should run hot when the
    // UI is idle and yield when it is not, which is THREAD_PRIORITY_-
    // BELOW_NORMAL -- not the I/O-throttled background mode -- for CPU work.
    if (SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL)) {
        return true;
    }
    _WarnPriorityOnce("background workers run at normal thread priority: "
                      "SetThreadPriority failed");
    return false;
#elif defined(__linux__)
    // SCHED_BATCH is the closest unprivileged per-thread below-normal: the
    // worker stays on the fair runqueue but never preempts a normal task on
    // wakeup. (A nice value would be process-wide -- it would deprioritize
    // the UI thread too -- and SCHED_IDLE would starve warming outright on a
    // loaded machine.) Failure degrades to the logged no-op: warming still
    // runs, just without the priority boundary.
    struct sched_param param;
    param.sched_priority = 0;
    if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &param) == 0) {
        return true;
    }
    _WarnPriorityOnce("background workers run at normal thread priority: "
                      "pthread_setschedparam(SCHED_BATCH) failed");
    return false;
#elif defined(__APPLE__)
    // Utility QoS: below default, above background -- the band for work the
    // user did not ask to wait for but still wants done.
    if (pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0) == 0) {
        return true;
    }
    _WarnPriorityOnce("background workers run at default QoS: "
                      "pthread_set_qos_class_self_np failed");
    return false;
#else
    _WarnPriorityOnce("background workers run at normal thread priority: no "
                      "below-normal API on this platform");
    return false;
#endif
}

const char *
RigExecBackgroundPriorityBackend()
{
#if defined(_WIN32)
    return "SetThreadPriority";
#elif defined(__linux__)
    return "sched-batch";
#elif defined(__APPLE__)
    return "qos-utility";
#else
    return "none";
#endif
}

RigExecBackgroundScheduler::RigExecBackgroundScheduler(
    int workerCount, RigExecSchedulerClock clock)
    : _clock(std::move(clock))
{
    if (workerCount < 0) {
        workerCount = 0;
    }
    if (workerCount > kRigExecBackgroundSchedulerMaxWorkers) {
        workerCount = kRigExecBackgroundSchedulerMaxWorkers;
    }
    for (int i = 0; i < workerCount; ++i) {
        try {
            _workers.emplace_back(&RigExecBackgroundScheduler::_WorkerMain,
                                  this);
        } catch (const std::exception &) {
            // A pool that cannot spawn is a smaller pool, not a failed
            // scheduler: the queued jobs wait for the workers that did start
            // (or for the manual drain at zero), and the caller always keeps
            // its live fallback.
            _WarnPriorityOnce("background scheduler started fewer workers "
                              "than asked: thread creation failed");
            break;
        }
    }
    _workerCount = int(_workers.size());
}

RigExecBackgroundScheduler::~RigExecBackgroundScheduler()
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _stopping = true;
    }
    _cv.notify_all();
    for (std::thread &worker : _workers) {
        worker.join();
    }
    // Running jobs finished above (join waits); what is still queued is
    // dropped, fail-closed -- each frame evaluates live when asked, and each
    // explicit request is re-admitted by its service -- and counted so the
    // books balance. Every explicit byte reservation goes with it.
    std::lock_guard<std::mutex> lock(_mutex);
    for (const auto &entry : _queue) {
        _RetireQueuedJobLocked(entry.second,
                               RigExecWarmTransitionKind::DroppedAtShutdown);
    }
    if (!_queue.empty()) {
        _profiler.RecordSchedulerCancel(_queue.size(), "shutdown");
    }
    _queue.clear();
    _queuedByRigAndTime.clear();
    _queuedExplicitByKey.clear();
}

uint64_t
RigExecBackgroundScheduler::ClockNowUs() const
{
    return _NowUs();
}

// The queue's shape after every mutation: the maxima cannot miss a change,
// and the profiler call keeps its existing leaf-lock contract.
void
RigExecBackgroundScheduler::_RecordQueueStateLocked()
{
    if (_queue.size() > _maxQueuedDepth) {
        _maxQueuedDepth = _queue.size();
    }
    if (_running > _maxRunning) {
        _maxRunning = _running;
    }
    const size_t queued = _queue.size();
    _profiler.RecordSchedulerQueue(queued, _running, _canceled);
}

void
RigExecBackgroundScheduler::_NoteDeclineLocked(RigExecWarmDeclineReason reason)
{
    ++_declined;
    switch (reason) {
    case RigExecWarmDeclineReason::FillGateClosed:
        ++_declinedFillGate;
        break;
    case RigExecWarmDeclineReason::StaleGeneration:
        ++_declinedStaleGeneration;
        break;
    case RigExecWarmDeclineReason::UnusableTime:
        ++_declinedUnusableTime;
        break;
    case RigExecWarmDeclineReason::EmptyWork:
        ++_declinedEmptyWork;
        break;
    case RigExecWarmDeclineReason::PerRigCap:
        ++_declinedPerRigCap;
        break;
    case RigExecWarmDeclineReason::Stopping:
        ++_declinedStopping;
        break;
    }
}

// A trigger burst ended before its whole wave reached the queue. The gate is
// read once per burst, so a mid-burst flip never stops one: the causes here
// are staleness (or shutdown), the sampling budget, and the per-rig cap.
void
RigExecBackgroundScheduler::_NoteTriggerStopLocked(_StopCause cause)
{
    ++_triggerStops;
    switch (cause) {
    case _StopCause::Stale:
        ++_triggerStopsStale;
        break;
    case _StopCause::Budget:
        ++_triggerStopsBudget;
        break;
    case _StopCause::Cap:
        ++_triggerStopsCap;
        break;
    }
}

void
RigExecBackgroundScheduler::SetTransitionHook(RigExecWarmTransitionHook hook)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _transitionHook = std::move(hook);
}

void
RigExecBackgroundScheduler::SetAdmissionReturnHookForTesting(
    RigExecExplicitAdmissionReturnHook hook)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _admissionReturnHook = std::move(hook);
}

void
RigExecBackgroundScheduler::_ReportTransitionLocked(
    const _JobIdentity &identity, RigExecWarmTransitionKind kind,
    RigExecWarmOutcome outcome)
{
    if (!_transitionHook) {
        return;
    }
    RigExecWarmTransition transition;
    transition.rig = identity.rig;
    transition.timeValue = identity.timeValue;
    transition.kind = kind;
    transition.outcome = outcome;
    transition.workClass = identity.workClass;
    transition.timeIsDefault = identity.timeIsDefault;
    _transitionHook(transition);
}

const SdfPath &
RigExecBackgroundScheduler::_JobRig(const _QueuedJob &job)
{
    return job.workClass == RigExecBackgroundWorkClass::Explicit
               ? job.explicitRequest.rig
               : job.request.rig;
}

RigExecBackgroundScheduler::_JobIdentity
RigExecBackgroundScheduler::_IdentityOf(const _QueuedJob &job)
{
    _JobIdentity identity;
    identity.workClass = job.workClass;
    identity.enqueuedUs = job.request.enqueuedUs;
    identity.poppedUs = job.request.poppedUs;
    if (job.workClass == RigExecBackgroundWorkClass::Explicit) {
        identity.rig = job.explicitRequest.rig;
        identity.timeIsDefault = job.explicitRequest.time.IsDefault();
        identity.timeValue =
            identity.timeIsDefault ? 0.0 : job.explicitRequest.time.TimeCode();
        identity.generation = job.explicitRequest.editGeneration;
        identity.reservationBytes = job.explicitRequest.reservationBytes;
        identity.explicitKey = job.explicitRequest.Key();
        return identity;
    }
    identity.rig = job.request.rig;
    identity.timeValue = job.request.time.GetValue();
    identity.generation = job.request.generation;
    return identity;
}

void
RigExecBackgroundScheduler::_ReleaseReservationLocked(const SdfPath &rig,
                                                      size_t bytes)
{
    if (bytes == 0) {
        return;
    }
    _reservedBytes -= bytes;
    const auto found = _reservedBytesByRig.find(rig);
    if (found == _reservedBytesByRig.end()) {
        return;
    }
    found->second -= std::min(found->second, bytes);
    if (found->second == 0) {
        _reservedBytesByRig.erase(found);
    }
}

size_t
RigExecBackgroundScheduler::_ExplicitInFlightLocked(const SdfPath &rig) const
{
    const auto found = _explicitInFlight.find(rig);
    return found == _explicitInFlight.end() ? 0 : found->second;
}

void
RigExecBackgroundScheduler::_ChangeExplicitInFlightLocked(const SdfPath &rig,
                                                          int delta)
{
    if (delta > 0) {
        ++_explicitInFlight[rig];
        return;
    }
    const auto found = _explicitInFlight.find(rig);
    if (found == _explicitInFlight.end()) {
        return;
    }
    if (--(found->second) == 0) {
        _explicitInFlight.erase(found);
    }
}

void
RigExecBackgroundScheduler::_RetireQueuedJobLocked(
    const _QueuedJob &job, RigExecWarmTransitionKind kind)
{
    const bool explicitJob =
        job.workClass == RigExecBackgroundWorkClass::Explicit;
    const _JobIdentity identity = _IdentityOf(job);
    if (explicitJob) {
        _queuedExplicitByKey.erase(job.explicitRequest.Key());
        _ReleaseReservationLocked(identity.rig,
                                  job.explicitRequest.reservationBytes);
    } else {
        _queuedByRigAndTime.erase(
            std::make_pair(identity.rig, identity.timeValue));
    }
    _ReportTransitionLocked(identity, kind);
    if (explicitJob) {
        _ChangeExplicitInFlightLocked(identity.rig, -1);
        if (kind == RigExecWarmTransitionKind::Canceled) {
            ++_explicitCanceled;
        } else if (kind == RigExecWarmTransitionKind::DroppedAtShutdown) {
            ++_explicitDroppedAtShutdown;
        }
        return;
    }
    const auto inFlight = _inFlight.find(identity.rig);
    if (inFlight != _inFlight.end() && inFlight->second > 0) {
        if (--(inFlight->second) == 0) {
            _inFlight.erase(inFlight);
        }
    }
    if (kind == RigExecWarmTransitionKind::Canceled) {
        ++_canceled;
    } else if (kind == RigExecWarmTransitionKind::Shed) {
        ++_shed;
    } else if (kind == RigExecWarmTransitionKind::DroppedAtShutdown) {
        ++_droppedAtShutdown;
    }
}

RigExecFrameGeneration
RigExecBackgroundScheduler::CurrentGeneration(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _generations.find(rig);
    if (found == _generations.end()) {
        return 0;
    }
    return found->second;
}

size_t
RigExecBackgroundScheduler::InFlightCount(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _inFlight.find(rig);
    return found == _inFlight.end() ? 0 : found->second;
}

void
RigExecBackgroundScheduler::CancelGeneration(const SdfPath &rig)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // A rig never seen before enters at 1: generation 0 means "no edit yet",
    // so the first edit must move it.
    _generations[rig] += 1;
    // Purge the rig's queued jobs under the same lock as the bump: nothing
    // queued can be stale afterwards, so the pre-run check below only ever
    // fires for the pop race (a worker that popped just ahead of this bump).
    // The purged closures -- and the input vectors sampled for the retired
    // edit -- are moved out and freed promptly, but AFTER the lock closes:
    // up to a rig's in-flight cap of multi-megabyte vectors is not a "map
    // edit", and a worker's pop must not wait on their teardown.
    std::vector<_QueuedJob> retired;
    for (auto it = _queue.begin(); it != _queue.end();) {
        if (_JobRig(it->second) != rig) {
            ++it;
            continue;
        }
        _RetireQueuedJobLocked(it->second, RigExecWarmTransitionKind::Canceled);
        retired.push_back(std::move(it->second));
        it = _queue.erase(it);
    }
    if (!retired.empty()) {
        _profiler.RecordSchedulerCancel(retired.size(), "edit");
    }
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
    // retired dies here, outside the lock.
}

void
RigExecBackgroundScheduler::ResetRig(const SdfPath &rig)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // Generations stand: the warm index keys its completions by them
    // and erasing here would desync the memo/lookup generation
    // comparison. Only the per-time fence tokens (which would fail
    // a reactivated rig's publishes) and queued jobs reset.
    _fenceCounters.erase(rig);
    _defaultFenceTokens.erase(rig);
    for (auto it = _fenceTokens.begin(); it != _fenceTokens.end();) {
        if (it->first.first == rig) {
            it = _fenceTokens.erase(it);
        } else {
            ++it;
        }
    }
    std::vector<_QueuedJob> retired;
    for (auto it = _queue.begin(); it != _queue.end();) {
        if (_JobRig(it->second) != rig) {
            ++it;
            continue;
        }
        _RetireQueuedJobLocked(it->second, RigExecWarmTransitionKind::Canceled);
        retired.push_back(std::move(it->second));
        it = _queue.erase(it);
    }
    _inFlight.erase(rig);
    _explicitInFlight.erase(rig);
    _explicitOrdinal.erase(rig);
    // Byte reservations are not swept here: purged jobs released theirs, and
    // what remains belongs to jobs still running, which release it when they
    // finish. Subtracting the map entry would count those bytes twice.
    if (!retired.empty()) {
        _profiler.RecordSchedulerCancel(retired.size(), "reset");
    }
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
}

RigExecWarmFenceToken
RigExecBackgroundScheduler::_CurrentFenceTokenLocked(const SdfPath &rig,
                                                     double timeValue) const
{
    const auto found = _fenceTokens.find(std::make_pair(rig, timeValue));
    return found == _fenceTokens.end() ? 0 : found->second;
}

size_t
RigExecBackgroundScheduler::CancelGenerationTimes(
    const SdfPath &rig, const std::vector<UsdTimeCode> &times)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // No generation bump: unaffected times keep their tokens and their
    // running jobs publish normally. Every affected time gets a fresh
    // per-time fence token from the rig's monotonic counter instead, so
    // an old job for a requeued time drops at the publish fence on token
    // mismatch and can never overwrite the new result.
    std::map<double, bool> affected;
    for (const UsdTimeCode &time : times) {
        if (time.IsNumeric() && std::isfinite(time.GetValue())) {
            affected[time.GetValue()] = true;
        }
    }
    for (const auto &entry : affected) {
        _fenceTokens[std::make_pair(rig, entry.first)] = ++_fenceCounters[rig];
    }
    if (!affected.empty()) {
        // The default frame has no numeric time to key, but a scoped edit
        // moves its inputs like every other frame, so its explicit requests
        // are fenced by the same counter.
        _defaultFenceTokens[rig] = ++_fenceCounters[rig];
    }
    // Purge the rig's queued jobs at those times, same loop shape as
    // CancelGeneration: the purged closures free AFTER the lock closes. An
    // explicit request for the default frame is purged with them.
    std::vector<_QueuedJob> retired;
    for (auto it = _queue.begin(); it != _queue.end();) {
        const _QueuedJob &job = it->second;
        bool affectedTime = false;
        if (_JobRig(job) == rig) {
            if (job.workClass == RigExecBackgroundWorkClass::Explicit) {
                const RigExecOutputTime &time = job.explicitRequest.time;
                affectedTime = time.IsDefault() ||
                               affected.find(time.TimeCode()) != affected.end();
            } else {
                affectedTime = affected.find(job.request.time.GetValue()) !=
                               affected.end();
            }
        }
        if (!affectedTime) {
            ++it;
            continue;
        }
        _RetireQueuedJobLocked(it->second, RigExecWarmTransitionKind::Canceled);
        retired.push_back(std::move(it->second));
        it = _queue.erase(it);
    }
    if (!retired.empty()) {
        _profiler.RecordSchedulerCancel(retired.size(), "edit-times");
    }
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
    // retired dies here, outside the lock.
    return retired.size();
}

bool
RigExecBackgroundScheduler::CancelExplicitJob(const RigExecExplicitJobKey &key)
{
    std::unique_lock<std::mutex> lock(_mutex);
    const auto index = _queuedExplicitByKey.find(key);
    if (index == _queuedExplicitByKey.end()) {
        return false;
    }
    // The index is the exact key's own entry, so this purge can only name
    // this job: no fence token moves, no other time or class is looked at.
    auto queued = _queue.find(index->second);
    if (queued == _queue.end()) {
        _queuedExplicitByKey.erase(index);
        return false;
    }
    _QueuedJob retired = std::move(queued->second);
    // Retiring leaves the index entry, so `index` is dangling from here on.
    _RetireQueuedJobLocked(retired, RigExecWarmTransitionKind::Canceled);
    _queue.erase(queued);
    _profiler.RecordSchedulerCancel(1, "demand");
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
    // retired's closures die here, outside the lock.
    return true;
}

RigExecWarmFenceToken
RigExecBackgroundScheduler::CurrentFenceToken(const SdfPath &rig,
                                              UsdTimeCode time) const
{
    if (!time.IsNumeric() || !std::isfinite(time.GetValue())) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _CurrentFenceTokenLocked(rig, time.GetValue());
}

bool
RigExecBackgroundScheduler::IsWarmRequestCurrent(
    const SdfPath &rig, RigExecFrameGeneration generation, UsdTimeCode time,
    RigExecWarmFenceToken fenceToken) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (currentGeneration != generation) {
        return false;
    }
    if (!time.IsNumeric() || !std::isfinite(time.GetValue())) {
        return fenceToken == 0;
    }
    return _CurrentFenceTokenLocked(rig, time.GetValue()) == fenceToken;
}

std::vector<UsdTimeCode>
RigExecBackgroundScheduler::QueuedTimes(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<UsdTimeCode> times;
    for (const auto &entry : _queuedByRigAndTime) {
        if (entry.first.first == rig) {
            times.emplace_back(entry.first.second);
        }
    }
    return times;
}

std::vector<UsdTimeCode>
RigExecBackgroundScheduler::InFlightTimes(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::set<double> values;
    for (const auto &entry : _queuedByRigAndTime) {
        if (entry.first.first == rig) {
            values.insert(entry.first.second);
        }
    }
    for (const auto &entry : _runningByRigAndTime) {
        if (entry.first.first == rig) {
            values.insert(entry.first.second);
        }
    }
    std::vector<UsdTimeCode> times;
    times.reserve(values.size());
    for (const double value : values) {
        times.emplace_back(value);
    }
    return times;
}

bool
RigExecBackgroundScheduler::IsGenerationCurrent(
    const SdfPath &rig, RigExecFrameGeneration generation) const
{
    return CurrentGeneration(rig) == generation;
}

bool
RigExecBackgroundScheduler::Enqueue(const SdfPath &rig, UsdTimeCode time,
                                    RigExecWarmPriority priority,
                                    RigExecFrameGeneration generation,
                                    RigExecWarmWork work)
{
    // Declines first, without touching the lock: the gate is the library's
    // live answer, and an unusable time or an empty closure can never become
    // a job. (A non-finite time would corrupt the coalescing index -- NaN
    // compares equivalent to everything -- so it declines like a default.)
    if (!RigExecBackgroundWarmingEnabled()) {
        std::lock_guard<std::mutex> lock(_mutex);
        _NoteDeclineLocked(RigExecWarmDeclineReason::FillGateClosed);
        return false;
    }
    if (!time.IsNumeric() || !std::isfinite(time.GetValue()) || !work) {
        std::lock_guard<std::mutex> lock(_mutex);
        _NoteDeclineLocked(time.IsNumeric() && std::isfinite(time.GetValue())
                               ? RigExecWarmDeclineReason::EmptyWork
                               : RigExecWarmDeclineReason::UnusableTime);
        return false;
    }
    const double timeValue = time.GetValue();
    const std::pair<SdfPath, double> coalesceKey =
        std::make_pair(rig, timeValue);
    std::unique_lock<std::mutex> lock(_mutex);
    if (_stopping) {
        _NoteDeclineLocked(RigExecWarmDeclineReason::Stopping);
        return false;
    }
    // Inline, not via IsGenerationCurrent: the lock is already held and
    // std::mutex does not nest.
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (currentGeneration != generation) {
        _NoteDeclineLocked(RigExecWarmDeclineReason::StaleGeneration);
        return false;
    }
    // Coalesce: warming for this frame is already coming. First-wins on the
    // work: the queued closure stands and the new one is destroyed on
    // return, freeing its inputs promptly.
    if (_MergeIfQueuedLocked(coalesceKey, priority)) {
        return true;
    }
    const auto inFlight = _inFlight.find(rig);
    const size_t flying = inFlight == _inFlight.end() ? 0 : inFlight->second;
    if (flying >= _perRigCap) {
        _NoteDeclineLocked(RigExecWarmDeclineReason::PerRigCap);
        return false;
    }
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = time;
    request.priority = priority;
    request.generation = generation;
    request.fenceToken = _CurrentFenceTokenLocked(rig, timeValue);
    request.enqueuedUs = _NowUs();
    const _QueueKey key{int(RigExecBackgroundWorkClass::Warm),
                        uint64_t(priority), _sequence++};
    _QueuedJob queuedJob;
    queuedJob.request = request;
    queuedJob.warmWork = std::move(work);
    const _JobIdentity identity = _IdentityOf(queuedJob);
    _queue.emplace(key, std::move(queuedJob));
    _queuedByRigAndTime.emplace(coalesceKey, key);
    _inFlight[rig] = flying + 1;
    if (priority == RigExecWarmPriority::Neighbor) {
        ++_enqueuedNeighbor;
    } else {
        ++_enqueuedSweep;
    }
    _ReportTransitionLocked(identity, RigExecWarmTransitionKind::Queued);
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
    return true;
}

RigExecWarmFenceToken
RigExecBackgroundScheduler::_CurrentExplicitFenceTokenLocked(
    const SdfPath &rig, RigExecOutputTime time) const
{
    if (time.IsDefault()) {
        const auto found = _defaultFenceTokens.find(rig);
        return found == _defaultFenceTokens.end() ? 0 : found->second;
    }
    return _CurrentFenceTokenLocked(rig, time.TimeCode());
}

RigExecWarmFenceToken
RigExecBackgroundScheduler::CurrentExplicitFenceToken(
    const SdfPath &rig, RigExecOutputTime time) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _CurrentExplicitFenceTokenLocked(rig, time);
}

bool
RigExecBackgroundScheduler::IsExplicitRequestCurrent(
    const RigExecExplicitRequest &request) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto current = _generations.find(request.rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (currentGeneration != request.editGeneration) {
        return false;
    }
    return _CurrentExplicitFenceTokenLocked(request.rig, request.time) ==
           request.fenceToken;
}

bool
RigExecBackgroundScheduler::AdmitExplicitOutput(
    const RigExecExplicitRequest &request, RigExecExplicitWork work)
{
    // Refusals that need no lock and reserve nothing. The frame-cache fill
    // gate is deliberately absent here: it gates speculative warming, while a
    // host that asked for this frame is not speculating.
    if (!work || !_OneProvenanceGroup(request.outputs) ||
        (!request.time.IsDefault() &&
         !std::isfinite(request.time.TimeCode()))) {
        std::lock_guard<std::mutex> lock(_mutex);
        ++_explicitDeclined;
        return false;
    }
    std::unique_lock<std::mutex> lock(_mutex);
    if (_stopping) {
        ++_explicitDeclined;
        return false;
    }
    // Inline, not via IsGenerationCurrent: the lock is already held and
    // std::mutex does not nest.
    const auto current = _generations.find(request.rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (currentGeneration != request.editGeneration) {
        ++_explicitDeclined;
        return false;
    }
    // Coalesce only on the exact key: the same rig, the same exact time, and
    // the same whole captured generation. Anything else is a second job, so a
    // recompiled program, a moved input, a different preview or history
    // revision, or a subframe step can never be answered by -- or overwrite
    // -- the queued job's answer.
    const RigExecExplicitJobKey key = request.Key();
    if (_queuedExplicitByKey.find(key) != _queuedExplicitByKey.end()) {
        ++_explicitCoalesced;
        return true;
    }
    if (_ExplicitInFlightLocked(request.rig) >= _explicitPerRigCap) {
        ++_explicitDeclined;
        return false;
    }
    // The byte reservation is bounded before anything is allocated or queued,
    // fail-closed on overflow. The caller answers the request live instead.
    const size_t bytes = request.reservationBytes;
    if (bytes > _explicitByteCap || _reservedBytes > _explicitByteCap - bytes) {
        ++_explicitDeclined;
        ++_explicitDeclinedBytes;
        return false;
    }
    RigExecExplicitRequest stamped = request;
    stamped.fenceToken =
        _CurrentExplicitFenceTokenLocked(request.rig, request.time);
    _QueuedJob job;
    job.workClass = RigExecBackgroundWorkClass::Explicit;
    job.explicitRequest = stamped;
    job.explicitWork = std::move(work);
    // The rank is the job's ordinal within its rig, so another rig's request
    // queues beside this one, not behind its whole burst.
    const _QueueKey queueKey{int(RigExecBackgroundWorkClass::Explicit),
                             _explicitOrdinal[request.rig]++, _sequence++};
    // The queue-wait stamps live on the job's request record for both
    // classes: the shared pool measures every job it pops.
    job.request.enqueuedUs = _NowUs();
    const _JobIdentity identity = _IdentityOf(job);
    _queue.emplace(queueKey, std::move(job));
    _queuedExplicitByKey.emplace(key, queueKey);
    _ChangeExplicitInFlightLocked(request.rig, +1);
    _reservedBytes += bytes;
    if (bytes > 0) {
        _reservedBytesByRig[request.rig] += bytes;
    }
    ++_explicitAccepted;
    _ReportTransitionLocked(identity, RigExecWarmTransitionKind::Queued);
    _RecordQueueStateLocked();
    // Taken under the lock, run after it closes: the seam sees this exact key
    // with the whole queue recorded and holds no scheduler lock, so the job's
    // terminal state can land inside the admission that named it.
    RigExecExplicitAdmissionReturnHook seam;
    if (_admissionReturnHook) {
        seam = _admissionReturnHook;
    }
    lock.unlock();
    _cv.notify_all();
    if (seam) {
        seam(key);
    }
    return true;
}

size_t
RigExecBackgroundScheduler::OnEditCommitted(
    const SdfPath &rig, UsdTimeCode playhead, RigExecFrameGeneration generation,
    const std::vector<UsdTimeCode> &sweepTimes, RigExecWarmJobFactory factory,
    int neighborRadius, RigExecWarmSamplingBudget budget)
{
    // Nothing to sample from without a playhead: no neighborhood to compute
    // and no playhead frame to protect. Fail-closed, before any factory call.
    if (!playhead.IsNumeric() || !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The fill gate, once per burst: the per-frame path below does not
    // re-read it, so a flip mid-burst takes effect at the next trigger
    // rather than stopping this one. (Memoizing the mode across bursts
    // would need a reset hook -- tests toggle it mid-run -- while one live
    // read per burst keeps the host-flippable contract.)
    if (!RigExecBackgroundWarmingEnabled()) {
        return 0;
    }
    _BudgetLedger ledger;
    ledger.budget = &budget;
    ledger.startUs = budget.startUs != 0 ? budget.startUs : _NowUs();
    size_t enqueued = 0;
    if (neighborRadius > 0 && factory) {
        // Closest first: the +-1 pair, then +-2, so FIFO within Neighbor
        // serves the frame beside the playhead next.
        const double center = playhead.GetValue();
        for (int step = 1; step <= neighborRadius; ++step) {
            for (int side = 0; side < 2; ++side) {
                const double offset = side == 0 ? -double(step) : double(step);
                const UsdTimeCode time(center + offset);
                // Center +- step can round back to center at extreme
                // magnitudes; the playhead is never queued, however it
                // arises.
                if (time == playhead) {
                    continue;
                }
                const _TriggerOutcome outcome = _EnqueueTriggerTime(
                    rig, time, RigExecWarmPriority::Neighbor, generation,
                    factory, &ledger);
                if (outcome == _TriggerOutcome::Stop) {
                    return enqueued;
                }
                if (outcome == _TriggerOutcome::Enqueued) {
                    ++enqueued;
                }
            }
        }
    }
    enqueued += _EnqueueSweepTimes(rig, playhead, generation, sweepTimes,
                                   factory, &ledger);
    return enqueued;
}

size_t
RigExecBackgroundScheduler::OnIdle(const SdfPath &rig, UsdTimeCode playhead,
                                   RigExecFrameGeneration generation,
                                   const std::vector<UsdTimeCode> &sweepTimes,
                                   RigExecWarmJobFactory factory,
                                   RigExecWarmSamplingBudget budget)
{
    if (!playhead.IsNumeric() || !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // Once per burst, as in OnEditCommitted: the per-frame path assumes it.
    if (!RigExecBackgroundWarmingEnabled()) {
        return 0;
    }
    _BudgetLedger ledger;
    ledger.budget = &budget;
    ledger.startUs = budget.startUs != 0 ? budget.startUs : _NowUs();
    return _EnqueueSweepTimes(rig, playhead, generation, sweepTimes, factory,
                              &ledger);
}

size_t
RigExecBackgroundScheduler::NotePlaybackAdvanced(const SdfPath &rig,
                                                 UsdTimeCode playhead)
{
    // NaN is numeric but orders against nothing: every `request.time >
    // playhead` test below is false, so without the finiteness check a NaN
    // playhead would shed the whole sweep.
    if (!playhead.IsNumeric() || !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    size_t shed = 0;
    for (auto it = _queue.begin(); it != _queue.end();) {
        const RigExecWarmRequest &request = it->second.request;
        if (it->second.workClass != RigExecBackgroundWorkClass::Warm ||
            request.rig != rig ||
            request.priority != RigExecWarmPriority::Sweep ||
            request.time > playhead) {
            ++it;
            continue;
        }
        // A sweep job at or behind the playhead: the live path serves that
        // frame now, so warming it is pointless. Neighbors stand -- the
        // frames ahead still warm the imminent scrub path -- and the standing
        // neighbor-before-sweep order deprioritizes the sweep that remains.
        // An explicit request never sheds: a host asked for that frame.
        _RetireQueuedJobLocked(it->second, RigExecWarmTransitionKind::Shed);
        it = _queue.erase(it);
        ++shed;
    }
    if (shed > 0) {
        _profiler.RecordSchedulerCancel(shed, "playback");
        _cv.notify_all();
    }
    _RecordQueueStateLocked();
    return shed;
}

void
RigExecBackgroundScheduler::SetPerRigJobCap(size_t cap)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _perRigCap = cap;
}

size_t
RigExecBackgroundScheduler::GetPerRigJobCap() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _perRigCap;
}

void
RigExecBackgroundScheduler::SetExplicitPerRigJobCap(size_t cap)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _explicitPerRigCap = cap;
}

size_t
RigExecBackgroundScheduler::GetExplicitPerRigJobCap() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _explicitPerRigCap;
}

void
RigExecBackgroundScheduler::SetExplicitByteCap(size_t cap)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _explicitByteCap = cap;
}

size_t
RigExecBackgroundScheduler::GetExplicitByteCap() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _explicitByteCap;
}

size_t
RigExecBackgroundScheduler::ExplicitInFlightCount(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _ExplicitInFlightLocked(rig);
}

size_t
RigExecBackgroundScheduler::ReservedExplicitBytes() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _reservedBytes;
}

size_t
RigExecBackgroundScheduler::ReservedExplicitBytesFor(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _reservedBytesByRig.find(rig);
    return found == _reservedBytesByRig.end() ? 0 : found->second;
}

std::vector<RigExecExplicitJobKey>
RigExecBackgroundScheduler::QueuedExplicitKeys(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<RigExecExplicitJobKey> keys;
    for (const auto &entry : _queuedExplicitByKey) {
        if (entry.first.rig == rig) {
            keys.push_back(entry.first);
        }
    }
    return keys;
}

std::vector<RigExecExplicitJobKey>
RigExecBackgroundScheduler::InFlightExplicitKeys(const SdfPath &rig) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<RigExecExplicitJobKey> keys;
    for (const auto &entry : _queuedExplicitByKey) {
        if (entry.first.rig == rig) {
            keys.push_back(entry.first);
        }
    }
    for (const auto &entry : _runningExplicitByKey) {
        if (entry.first.rig == rig) {
            keys.push_back(entry.first);
        }
    }
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    return keys;
}

RigExecBackgroundSchedulerStats
RigExecBackgroundScheduler::Stats() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    RigExecBackgroundSchedulerStats stats;
    stats.queuedDepth = _queue.size();
    stats.running = _running;
    stats.completed = _completed;
    stats.published = _published;
    stats.declinedInvalid = _declinedInvalid;
    stats.declinedGeneration = _declinedGeneration;
    stats.droppedStale = _droppedStale;
    stats.canceled = _canceled;
    stats.shed = _shed;
    stats.coalesced = _coalesced;
    stats.upgraded = _upgraded;
    stats.declined = _declined;
    stats.declinedFreezeRefused = _declinedFreezeRefused;
    stats.declinedUnsampleable = _declinedUnsampleable;
    stats.declinedD7Exempt = _declinedD7Exempt;
    stats.droppedAtShutdown = _droppedAtShutdown;
    stats.factoryInvocations = _factoryInvocations;
    stats.declinedFillGate = _declinedFillGate;
    stats.declinedStaleGeneration = _declinedStaleGeneration;
    stats.declinedUnusableTime = _declinedUnusableTime;
    stats.declinedEmptyWork = _declinedEmptyWork;
    stats.declinedPerRigCap = _declinedPerRigCap;
    stats.declinedStopping = _declinedStopping;
    stats.triggerStops = _triggerStops;
    stats.triggerStopsStale = _triggerStopsStale;
    stats.triggerStopsBudget = _triggerStopsBudget;
    stats.triggerStopsCap = _triggerStopsCap;
    stats.maxQueuedDepth = _maxQueuedDepth;
    stats.maxRunning = _maxRunning;
    stats.enqueuedNeighbor = _enqueuedNeighbor;
    stats.enqueuedSweep = _enqueuedSweep;
    for (const auto &entry : _queue) {
        if (entry.second.workClass != RigExecBackgroundWorkClass::Warm) {
            continue;
        }
        if (entry.second.request.priority == RigExecWarmPriority::Neighbor) {
            ++stats.queuedNeighbor;
        } else {
            ++stats.queuedSweep;
        }
    }
    stats.totalQueueWaitUs = _totalQueueWaitUs;
    stats.maxQueueWaitUs = _maxQueueWaitUs;
    stats.totalRunUs = _totalRunUs;
    stats.maxRunUs = _maxRunUs;
    stats.explicitAccepted = _explicitAccepted;
    stats.explicitCoalesced = _explicitCoalesced;
    stats.explicitDeclined = _explicitDeclined;
    stats.explicitDeclinedBytes = _explicitDeclinedBytes;
    stats.explicitCompleted = _explicitCompleted;
    stats.explicitPublished = _explicitPublished;
    stats.explicitDeclinedInvalid = _explicitDeclinedInvalid;
    stats.explicitDeclinedGeneration = _explicitDeclinedGeneration;
    stats.explicitDroppedStale = _explicitDroppedStale;
    stats.explicitCanceled = _explicitCanceled;
    stats.explicitDroppedAtShutdown = _explicitDroppedAtShutdown;
    stats.explicitQueuedDepth = _queuedExplicitByKey.size();
    stats.explicitRunning = _explicitRunning;
    stats.explicitReservedBytes = _reservedBytes;
    return stats;
}

void
RigExecBackgroundScheduler::WaitUntilIdle()
{
    std::unique_lock<std::mutex> lock(_mutex);
    // A stopping pool never drains its queue, so waiting on emptiness alone
    // would hang against a concurrent shutdown (or a zero-worker pool).
    _cv.wait(lock,
             [this] { return _stopping || (_queue.empty() && _running == 0); });
}

bool
RigExecBackgroundScheduler::RunNextQueuedForTesting()
{
    _QueuedJob job;
    if (!_PopJob(&job, /*wait=*/false)) {
        return false;
    }
    _ExecutePoppedJob(&job);
    return true;
}

size_t
RigExecBackgroundScheduler::DrainQueueForTesting()
{
    size_t ran = 0;
    _QueuedJob job;
    while (_PopJob(&job, /*wait=*/false)) {
        if (_ExecutePoppedJob(&job)) {
            ++ran;
        }
    }
    return ran;
}

void
RigExecBackgroundScheduler::_WorkerMain()
{
    RigExecSetBackgroundThreadPriority();
    _QueuedJob job;
    while (_PopJob(&job, /*wait=*/true)) {
        _ExecutePoppedJob(&job);
    }
}

bool
RigExecBackgroundScheduler::_PopJob(_QueuedJob *job, bool wait)
{
    std::unique_lock<std::mutex> lock(_mutex);
    if (wait) {
        _cv.wait(lock, [this] { return _stopping || !_queue.empty(); });
        if (_stopping) {
            return false;
        }
    } else if (_stopping || _queue.empty()) {
        return false;
    }
    auto next = _queue.begin();
    *job = std::move(next->second);
    job->request.poppedUs = _NowUs();
    const _JobIdentity identity = _IdentityOf(*job);
    if (identity.workClass == RigExecBackgroundWorkClass::Explicit) {
        _queuedExplicitByKey.erase(identity.explicitKey);
    } else {
        _queuedByRigAndTime.erase(
            std::make_pair(identity.rig, identity.timeValue));
    }
    _queue.erase(next);
    // Still in-flight -- the cap counts popped-and-executing -- until
    // _FinishJob lands.
    ++_running;
    if (identity.workClass == RigExecBackgroundWorkClass::Explicit) {
        ++_runningExplicitByKey[identity.explicitKey];
        ++_explicitRunning;
    } else {
        ++_runningByRigAndTime[std::make_pair(identity.rig,
                                              identity.timeValue)];
    }
    _RecordQueueStateLocked();
    _ReportTransitionLocked(identity, RigExecWarmTransitionKind::Running);
    return true;
}

bool
RigExecBackgroundScheduler::_ExecutePoppedJob(_QueuedJob *job)
{
    // The pre-run fence: a bump that landed between the pop and this check
    // -- CancelGeneration purges queued jobs, but this one already popped --
    // drops the work unrun and frees its sampled inputs and its byte
    // reservation promptly.
    const _JobIdentity identity = _IdentityOf(*job);
    const bool explicitJob =
        identity.workClass == RigExecBackgroundWorkClass::Explicit;
    if (!IsGenerationCurrent(identity.rig, identity.generation)) {
        *job = _QueuedJob();
        _FinishJob(identity, /*ran=*/false,
                   RigExecWarmOutcome::DeclinedInvalid);
        return false;
    }
    // The serial-variant boundary: the serial scope forces every kernel variant this run
    // reaches into its serial form on this thread, whatever the
    // process-wide switch says, so no work leaks back onto the shared arena
    // at normal priority. The work must also never hop threads: the scope
    // constrains this thread and no other.
    // A throwing job must not take the worker -- or the calling thread on
    // the manual drain -- down with it: fail-closed, warn, and land the
    // terminal state the books require. The frame evaluates live when asked.
    RigExecWarmOutcome outcome = RigExecWarmOutcome::DeclinedInvalid;
    // The job stamps its phases here, on this worker's frame; _FinishJob
    // records them once.
    RigExecWarmPhaseTimings timings;
    RigExecWarmRequest running;
    if (!explicitJob) {
        running = job->request;
        running.timings = _profiler.IsEnabled() ? &timings : nullptr;
    }
    try {
        RigExecFrozenSerialScope serial;
        if (explicitJob) {
            outcome = job->explicitWork(job->explicitRequest);
        } else {
            outcome = job->warmWork(running);
        }
    } catch (const std::exception &error) {
        TF_WARN("rigExec: background job threw: %s", error.what());
    } catch (...) {
        TF_WARN("rigExec: background job threw an unknown exception");
    }
    *job = _QueuedJob();
    _FinishJob(identity, /*ran=*/true, outcome,
               running.timings ? &timings : nullptr);
    return true;
}

void
RigExecBackgroundScheduler::_FinishJob(const _JobIdentity &identity, bool ran,
                                       RigExecWarmOutcome outcome,
                                       const RigExecWarmPhaseTimings *timings)
{
    // One event per job, spanning its marks, with each phase's duration as
    // an arg: one profiler lock at this boundary, none inside the job.
    // Recorded before the books change, outside the scheduler mutex, so a
    // caller that sees the job finished also sees its event.
    if (ran && timings && timings->markCount > 1 && _profiler.IsEnabled()) {
        std::map<std::string, std::string> phases;
        for (size_t i = 1; i < timings->markCount; ++i) {
            const char *name = timings->names[i - 1];
            const uint64_t begin = timings->marks[i - 1];
            const uint64_t end = timings->marks[i];
            phases[name ? name : "phase"] =
                std::to_string(end >= begin ? end - begin : 0);
        }
        _profiler.Record("Warm.Worker", "imagingWarm", timings->marks[0],
                         timings->marks[timings->markCount - 1],
                         std::move(phases));
    }
    std::unique_lock<std::mutex> lock(_mutex);
    if (_running > 0) {
        --_running;
    }
    const bool explicitJob =
        identity.workClass == RigExecBackgroundWorkClass::Explicit;
    if (explicitJob) {
        if (_explicitRunning > 0) {
            --_explicitRunning;
        }
        const auto running = _runningExplicitByKey.find(identity.explicitKey);
        if (running != _runningExplicitByKey.end() && --running->second == 0) {
            _runningExplicitByKey.erase(running);
        }
        // The reservation covers the values the job closed over and the
        // payload it published; either way the job is over, so the bytes are
        // free again.
        _ReleaseReservationLocked(identity.rig, identity.reservationBytes);
        _ChangeExplicitInFlightLocked(identity.rig, -1);
    } else {
        const auto running = _runningByRigAndTime.find(
            std::make_pair(identity.rig, identity.timeValue));
        if (running != _runningByRigAndTime.end() && --running->second == 0) {
            _runningByRigAndTime.erase(running);
        }
        const auto inFlight = _inFlight.find(identity.rig);
        if (inFlight != _inFlight.end() && inFlight->second > 0) {
            if (--(inFlight->second) == 0) {
                _inFlight.erase(inFlight);
            }
        }
    }
    if (ran) {
        if (explicitJob) {
            ++_explicitCompleted;
            switch (outcome) {
            case RigExecWarmOutcome::Published:
                ++_explicitPublished;
                break;
            case RigExecWarmOutcome::DeclinedGeneration:
                ++_explicitDeclinedGeneration;
                break;
            case RigExecWarmOutcome::DeclinedInvalid:
                ++_explicitDeclinedInvalid;
                break;
            }
            _ReportTransitionLocked(
                identity, RigExecWarmTransitionKind::Finished, outcome);
        } else {
            ++_completed;
            switch (outcome) {
            case RigExecWarmOutcome::Published:
                ++_published;
                break;
            case RigExecWarmOutcome::DeclinedGeneration:
                ++_declinedGeneration;
                break;
            case RigExecWarmOutcome::DeclinedInvalid:
                ++_declinedInvalid;
                break;
            }
            _ReportTransitionLocked(
                identity, RigExecWarmTransitionKind::Finished, outcome);
        }
    } else {
        if (explicitJob) {
            ++_explicitDroppedStale;
        } else {
            ++_droppedStale;
        }
        _ReportTransitionLocked(identity,
                                RigExecWarmTransitionKind::DroppedStale);
    }
    if (identity.poppedUs >= identity.enqueuedUs) {
        const uint64_t waitUs = identity.poppedUs - identity.enqueuedUs;
        _totalQueueWaitUs += waitUs;
        if (waitUs > _maxQueueWaitUs) {
            _maxQueueWaitUs = waitUs;
        }
    }
    if (ran) {
        const uint64_t nowUs = _NowUs();
        if (nowUs > identity.poppedUs) {
            const uint64_t runUs = nowUs - identity.poppedUs;
            _totalRunUs += runUs;
            if (runUs > _maxRunUs) {
                _maxRunUs = runUs;
            }
        }
    }
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
}

bool
RigExecBackgroundScheduler::_MergeIfQueuedLocked(
    const std::pair<SdfPath, double> &coalesceKey, RigExecWarmPriority priority)
{
    const auto queued = _queuedByRigAndTime.find(coalesceKey);
    if (queued == _queuedByRigAndTime.end()) {
        return false;
    }
    // A neighbor request promotes a queued sweep -- the frame matters more
    // than the trigger thought -- keeping its enqueue sequence: FIFO within
    // neighbor still serves the earliest-sampled frame next.
    const _QueueKey oldKey = queued->second;
    const auto job = _queue.find(oldKey);
    if (job != _queue.end() && priority == RigExecWarmPriority::Neighbor &&
        job->second.request.priority == RigExecWarmPriority::Sweep) {
        _QueuedJob promoted = std::move(job->second);
        promoted.request.priority = RigExecWarmPriority::Neighbor;
        _queue.erase(job);
        const _QueueKey newKey{int(RigExecBackgroundWorkClass::Warm),
                               uint64_t(RigExecWarmPriority::Neighbor),
                               oldKey.sequence};
        _queue.emplace(newKey, std::move(promoted));
        queued->second = newKey;
        ++_upgraded;
    } else {
        ++_coalesced;
    }
    return true;
}

RigExecBackgroundScheduler::_TriggerOutcome
RigExecBackgroundScheduler::_EnqueueTriggerTime(
    const SdfPath &rig, UsdTimeCode time, RigExecWarmPriority priority,
    RigExecFrameGeneration generation, const RigExecWarmJobFactory &factory,
    _BudgetLedger *ledger)
{
    if (!time.IsNumeric() || !std::isfinite(time.GetValue())) {
        return _TriggerOutcome::Skipped;
    }
    // No gate check here: the triggers check once per burst, and re-reading
    // per frame cost an env lookup per warmed frame. Mid-burst staleness is
    // still decided per frame below (stopping, generation, queued set, cap).
    const double timeValue = time.GetValue();
    const std::pair<SdfPath, double> coalesceKey =
        std::make_pair(rig, timeValue);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        const auto current = _generations.find(rig);
        const RigExecFrameGeneration currentGeneration =
            current == _generations.end() ? 0 : current->second;
        if (_stopping || currentGeneration != generation) {
            _NoteTriggerStopLocked(_StopCause::Stale);
            return _TriggerOutcome::Stop;
        }
        // Already queued: merge without sampling, since the queued closure
        // stands either way.
        if (_MergeIfQueuedLocked(coalesceKey, priority)) {
            return _TriggerOutcome::Skipped;
        }
        const auto inFlight = _inFlight.find(rig);
        const size_t flying =
            inFlight == _inFlight.end() ? 0 : inFlight->second;
        if (flying >= _perRigCap) {
            _NoteTriggerStopLocked(_StopCause::Cap);
            return _TriggerOutcome::Stop;
        }
        // The sampling budget: the merge above consumed none (no factory
        // call); this frame would sample, so it counts. The stop ends the
        // trigger (Stop), not the frame: the budget is per-trigger.
        if (ledger->invocations >= ledger->budget->maxFactoryInvocations) {
            _NoteTriggerStopLocked(_StopCause::Budget);
            return _TriggerOutcome::Stop;
        }
        if (ledger->budget->maxMs < std::numeric_limits<double>::infinity() &&
            double(_NowUs() - ledger->startUs) / 1000.0 >=
                ledger->budget->maxMs) {
            _NoteTriggerStopLocked(_StopCause::Budget);
            return _TriggerOutcome::Stop;
        }
    }
    // Sampling runs on the calling (UI) thread, outside the lock: it reads
    // attribute queries and resolved inputs, and the lock never spans it.
    // An empty factory -- or an empty build -- skips the frame, and the
    // trigger continues with the next.
    RigExecWarmFactoryResult built;
    if (factory) {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Scheduler.WarmFactorySample", "scheduler");
        built = factory(time);
        ++ledger->invocations;
        std::lock_guard<std::mutex> lock(_mutex);
        ++_factoryInvocations;
    }
    if (!built.work) {
        // A reasoned skip counts declined (it never entered the queue)
        // plus its per-reason breakout; an unreasoned one skips silently,
        // as before.
        if (built.skip != RigExecWarmSkipReason::None) {
            std::lock_guard<std::mutex> lock(_mutex);
            ++_declined;
            switch (built.skip) {
            case RigExecWarmSkipReason::FreezeRefused:
                ++_declinedFreezeRefused;
                break;
            case RigExecWarmSkipReason::Unsampleable:
                ++_declinedUnsampleable;
                break;
            case RigExecWarmSkipReason::D7Exempt:
                ++_declinedD7Exempt;
                break;
            case RigExecWarmSkipReason::None:
                break;
            }
        }
        return _TriggerOutcome::Skipped;
    }
    RigExecWarmWork work = std::move(built.work);
    std::unique_lock<std::mutex> lock(_mutex);
    // Recheck under the lock: the factory above runs arbitrary caller code,
    // and an edit may have landed mid-sample -- the generation, the queued
    // set, and the cap are all re-decided, never trusted across the call.
    // (A worker racing this trigger can only have popped jobs -- never
    // queued -- so a miss here still inserts honestly beside a running twin.)
    const auto current = _generations.find(rig);
    const RigExecFrameGeneration currentGeneration =
        current == _generations.end() ? 0 : current->second;
    if (_stopping || currentGeneration != generation) {
        _NoteTriggerStopLocked(_StopCause::Stale);
        return _TriggerOutcome::Stop;
    }
    if (_MergeIfQueuedLocked(coalesceKey, priority)) {
        return _TriggerOutcome::Skipped;
    }
    const auto inFlight = _inFlight.find(rig);
    const size_t flying = inFlight == _inFlight.end() ? 0 : inFlight->second;
    if (flying >= _perRigCap) {
        _NoteTriggerStopLocked(_StopCause::Cap);
        return _TriggerOutcome::Stop;
    }
    RigExecWarmRequest request;
    request.rig = rig;
    request.time = time;
    request.priority = priority;
    request.generation = generation;
    request.fenceToken = _CurrentFenceTokenLocked(rig, timeValue);
    request.enqueuedUs = _NowUs();
    const _QueueKey key{int(RigExecBackgroundWorkClass::Warm),
                        uint64_t(priority), _sequence++};
    _QueuedJob queuedJob;
    queuedJob.request = request;
    queuedJob.warmWork = std::move(work);
    const _JobIdentity identity = _IdentityOf(queuedJob);
    _queue.emplace(key, std::move(queuedJob));
    _queuedByRigAndTime.emplace(coalesceKey, key);
    _inFlight[rig] = flying + 1;
    if (priority == RigExecWarmPriority::Neighbor) {
        ++_enqueuedNeighbor;
    } else {
        ++_enqueuedSweep;
    }
    _ReportTransitionLocked(identity, RigExecWarmTransitionKind::Queued);
    _RecordQueueStateLocked();
    lock.unlock();
    _cv.notify_all();
    return _TriggerOutcome::Enqueued;
}

size_t
RigExecBackgroundScheduler::_EnqueueSweepTimes(
    const SdfPath &rig, UsdTimeCode playhead, RigExecFrameGeneration generation,
    const std::vector<UsdTimeCode> &sweepTimes,
    const RigExecWarmJobFactory &factory, _BudgetLedger *ledger)
{
    if (!factory) {
        return 0;
    }
    size_t enqueued = 0;
    for (const UsdTimeCode &time : sweepTimes) {
        // The playhead is never queued, wherever the range puts it.
        if (time == playhead) {
            continue;
        }
        const _TriggerOutcome outcome = _EnqueueTriggerTime(
            rig, time, RigExecWarmPriority::Sweep, generation, factory, ledger);
        if (outcome == _TriggerOutcome::Stop) {
            return enqueued;
        }
        if (outcome == _TriggerOutcome::Enqueued) {
            ++enqueued;
        }
    }
    return enqueued;
}

} // namespace rigExec
