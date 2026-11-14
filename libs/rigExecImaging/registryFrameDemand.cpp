// RigExec imaging frames on demand: the stamp that scopes the frame table,
// the locked read of one frame from the sessions' frame caches, and the
// service call that turns Hydra readers' demand into table entries. This is
// how a host reads a frame the display does not hold (docs/specs/
// frames-on-demand.md). It never evaluates a live rig except through the
// documented opt-in fallback, never samples topology per frame, and adds no
// time record to any .rigexec. A frame the cache does not hold becomes an
// exact output request on the existing pool (docs/specs/
// output-subscriptions.md): what a reader asked for is not speculative
// warming, so it runs ahead of the warm sweep and stays admitted when
// speculative warming is off. The job still publishes a complete pose, and
// the table entry is still built from complete compatible rig coverage.
#include "registry.h"

#include "frameTable.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/outputCatalog.h"

#include "pxr/base/work/utils.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// Coarse per-job reservation for one demanded full-frame request: the
// sampled vector plus the pose its completion publishes. The pool bounds
// queued bytes by it before allocating; the per-payload charge belongs to
// the subscription lifecycle (outputSubscription.h), not to this pose
// route.
constexpr size_t kRigExecDemandJobReservationBytes = size_t(4) * 1024 * 1024;

} // namespace

uint64_t
RigExecImagingRegistry::_FrameTableStampLocked() const
{
    // Everything an entry depends on besides its time: the stage, the
    // compiled rigs, their edit serials and their upstream values.
    uint64_t hash = 1469598103934665603ULL;
    const auto mix = [&hash](uint64_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    };
    mix(reinterpret_cast<uintptr_t>(get_pointer(_stage)));
    mix(_publishedCompiledRigId);
    for (const RigSession &session : _sessions) {
        mix(session.rigPath.GetHash());
        mix(session.upstreamSerial);
        if (session.playback) {
            mix(session.playback->GetCompiledRigSignature());
        } else if (session.bridge) {
            mix(session.bridge->GetCompiledRigSignature());
            mix(session.bridge->GetEvaluator().GetStageEditSerial());
        }
    }
    return hash;
}

bool
RigExecImagingRegistry::_CachedFrameLocked(
    double time, std::shared_ptr<RigExecImagingSnapshot> *frame, bool evaluate)
{
    const UsdTimeCode at(time);
    auto combined = std::make_shared<RigExecImagingSnapshot>();
    combined->stage = UsdStageWeakPtr(_stage);
    combined->sampleTimeIsDefault = false;
    combined->sampleTime = time;
    SdfPath commonAssetRoot;
    bool firstRoot = true;
    for (RigSession &session : _sessions) {
        // The upstream values that time reads, handed over the way SetTime
        // hands them, and put back afterwards with the dirty flag: the
        // displayed generation stays exactly as it was.
        const bool wasDirty = session.dirty;
        std::vector<RigExecUpstreamValue> held =
            session.playback ? session.playback->GetUpstreamInputs()
                             : session.bridge->GetUpstreamInputs();
        _HandOffUpstreamLocked(session, at);
        RigExecImagingSnapshotConstPtr part;
        bool ok = false;
        if (session.playback) {
            // A .rigexec playback has no frame cache: it plays its binary
            // at the time into a detached generation (the chains read the
            // combined store, never this one). The production never
            // publishes, so the displayed generation is never touched and
            // never restored.
            ok = session.playback->ProduceAtTimeWithoutPublishing(at, &part);
            ok = ok && part && part->Describes(_stage, at);
        } else if (session.bridge) {
            std::string diagnostic;
            bool evaluated = false;
            ok = session.bridge->SnapshotAtTimeWithoutPublishing(
                at, evaluate, &part, &diagnostic, &evaluated);
            if (evaluated) {
                ++session.evaluationCount;
            }
        }
        if (session.playback) {
            if (session.playback->GetUpstreamInputs() != held) {
                session.playback->SetUpstreamInputs(std::move(held));
            }
        } else if (session.bridge &&
                   session.bridge->GetUpstreamInputs() != held) {
            session.bridge->SetUpstreamInputs(std::move(held));
        }
        session.dirty = wasDirty;
        if (!ok || !part) {
            if (!session.playback) {
                // Not cached yet: the background workers warm it (the idle
                // sweep visits the rewarm lane first).
                auto &lane = session.pendingRewarm;
                if (std::find(lane.begin(), lane.end(), time) == lane.end()) {
                    if (lane.size() >= RigExecFrameTable::kMaxDemand) {
                        lane.erase(lane.begin());
                    }
                    lane.push_back(time);
                }
            }
            return false;
        }
        combined->xformResetPaths.insert(part->xformResetPaths.begin(),
                                         part->xformResetPaths.end());
        if (firstRoot) {
            commonAssetRoot = session.assetRoot;
            firstRoot = false;
        } else if (commonAssetRoot != session.assetRoot) {
            commonAssetRoot = SdfPath();
        }
        for (const auto &[path, published] : part->prims) {
            // Two rigs never publish one prim (SetTime refuses it).
            combined->prims.emplace(path, published);
        }
    }
    combined->assetRoot = commonAssetRoot;
    // What RigExecSnapshotStore::Publish derives for a published generation.
    for (const auto &[path, prim] : combined->prims) {
        if (prim.hasXform) {
            combined->hasDrivenXforms = true;
            break;
        }
    }
    *frame = std::move(combined);
    return true;
}

size_t
RigExecImagingRegistry::ServiceFrameDemand(size_t maxFrames, bool allowFallback)
{
    size_t added = 0;
    bool reset = false;
    // Frames this call found missing from every cache: admitted as exact
    // output requests below, outside the lock.
    std::vector<double> admit;
    bool previewing = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_stage) {
            _frameTable->Clear();
            _pendingFrameDemand.clear();
            _pendingFrameAge.clear();
            return 0;
        }
        reset = _frameTable->ResetIfStale(_FrameTableStampLocked());
        // A live preview's displayed pose is not the authored one; frames
        // asked for meanwhile wait for it to end.
        previewing = _previewActive;
        for (const RigSession &session : _sessions) {
            previewing = previewing ||
                         (session.bridge &&
                          !session.bridge->GetInteractiveOverrides().empty());
        }
        std::vector<double> wanted = _frameTable->TakeDemand();
        wanted.insert(wanted.end(), _pendingFrameDemand.begin(),
                      _pendingFrameDemand.end());
        _pendingFrameDemand.clear();
        const double playhead =
            _lastTime.IsNumeric() ? _lastTime.GetValue() : 0.0;
        std::sort(wanted.begin(), wanted.end(), [playhead](double a, double b) {
            const double da = std::abs(a - playhead);
            const double db = std::abs(b - playhead);
            return da != db ? da < db : a > b;
        });
        std::set<int64_t> seen;
        bool fallbackUsed = false;
        std::map<int64_t, int> ages;
        for (const double time : wanted) {
            const int64_t key = RigExecFrameTable::Key(time);
            if (!std::isfinite(time) || !seen.insert(key).second ||
                _frameTable->Contains(time)) {
                continue;
            }
            const auto aged = _pendingFrameAge.find(key);
            const int age =
                (aged == _pendingFrameAge.end() ? 0 : aged->second) + 1;
            std::shared_ptr<RigExecImagingSnapshot> frame;
            if (!previewing && added < maxFrames) {
                bool ready = _CachedFrameLocked(time, &frame);
                if (!ready && allowFallback && !fallbackUsed &&
                    age >= kFrameDemandFallbackAge &&
                    !_DemandWorkInFlightLocked(time)) {
                    // Still not warmed and no background job for it is
                    // in flight: one evaluation here, so the frame is
                    // answered even when no worker ever will.
                    fallbackUsed = true;
                    ready = _CachedFrameLocked(time, &frame,
                                               /* evaluate = */ true);
                }
                if (ready) {
                    _frameTable->Insert(time, std::move(frame), playhead);
                    ++added;
                    continue;
                }
                admit.push_back(time);
            }
            if (_pendingFrameDemand.size() < RigExecFrameTable::kMaxDemand) {
                _pendingFrameDemand.push_back(time);
                ages[key] = age;
            }
        }
        _pendingFrameAge.swap(ages);
    }
    // Unlocked: the exact-request admission samples its jobs on this
    // thread (BuildWarmWork takes _mutex per factory call), and the
    // scheduler owns admission, coalescing and the fences.
    if (!previewing && !admit.empty()) {
        _AdmitDemandWork(admit);
    }
    if (added || reset) {
        _BroadcastFramesChanged();
    }
    return added;
}

bool
RigExecImagingRegistry::_DemandWorkInFlightLocked(double time) const
{
    if (!_scheduler) {
        return false;
    }
    for (const RigSession &session : _sessions) {
        if (session.playback) {
            continue;
        }
        for (const UsdTimeCode &at :
             _scheduler->InFlightTimes(session.rigPath)) {
            if (at.IsNumeric() && at.GetValue() == time) {
                return true;
            }
        }
        for (const RigExecExplicitJobKey &key :
             _scheduler->InFlightExplicitKeys(session.rigPath)) {
            if (!key.time.IsDefault() && key.time.TimeCode() == time) {
                return true;
            }
        }
    }
    return false;
}

size_t
RigExecImagingRegistry::_AdmitDemandWork(const std::vector<double> &times)
{
    if (times.empty()) {
        return 0;
    }
    // What each non-playback rig still owes, resolved under short locks;
    // the sampling below runs outside _mutex, exactly as the commit and
    // idle triggers sample outside theirs.
    struct Candidate {
        SdfPath rig;
        std::vector<double> times;
        RigExecFrameGeneration generation = 0;
        // The identity every request of this rig shares: program epoch,
        // input revision, output handles, edit generation, reservation.
        RigExecExplicitRequest base;
    };
    std::vector<Candidate> candidates;
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_scheduler || !_stage) {
            return 0;
        }
        scheduler = _scheduler;
        for (RigSession &session : _sessions) {
            if (session.playback || !session.bridge) {
                continue;
            }
            // A pose the query lookup refuses to serve answers nothing:
            // with the cache off the demand path's only route stays the
            // aged calling-thread fill.
            if (session.bridge->GetFrameCacheMode() ==
                RigExecFrameCacheMode::Off) {
                continue;
            }
            const RigExecRigEvaluator &evaluator =
                session.bridge->GetEvaluator();
            const uint64_t epoch = evaluator.GetCompiledRigSignature();
            _DemandCatalogEntry &entry = _demandCatalogs[session.rigPath];
            if (entry.epoch != epoch) {
                entry.epoch = epoch;
                entry.catalog.reset();
                auto catalog = std::make_shared<RigExecOutputCatalog>();
                std::string error;
                const RigExecBakedProgram *program =
                    evaluator.GetBakedProgram();
                if (program && RigExecDiscoverOutputs(
                                   *program, session.rigPath.GetString(), epoch,
                                   catalog.get(), &error)) {
                    entry.catalog = std::move(catalog);
                }
            }
            if (!entry.catalog) {
                continue; // this epoch cannot answer an exact request
            }
            std::vector<RigExecOutputHandle> outputs;
            for (const RigExecOutputBinding &binding :
                 entry.catalog->GetBindings()) {
                if (binding.status == RigExecOutputStatus::Pending) {
                    outputs.push_back(binding.handle);
                }
            }
            if (outputs.empty()) {
                continue;
            }
            const RigExecFrameGeneration generation =
                _scheduler->CurrentGeneration(session.rigPath);
            Candidate candidate;
            candidate.rig = session.rigPath;
            candidate.generation = generation;
            candidate.base.rig = session.rigPath;
            // The exact identity: the compiled epoch as the program
            // identity, the stage serial as the input revision. The time
            // is per request and never rounded.
            candidate.base.generation.programIdentity = epoch;
            candidate.base.generation.inputRevision =
                evaluator.GetStageEditSerial();
            candidate.base.generation.previewRevision = 0;
            candidate.base.generation.historyRevision = 0;
            candidate.base.outputs = std::move(outputs);
            candidate.base.editGeneration = generation;
            candidate.base.reservationBytes = kRigExecDemandJobReservationBytes;
            const std::vector<RigExecWarmFrameState> states =
                _warmIndex
                    ? _warmIndex->States(
                          session.rigPath, times, generation,
                          RigExecFrameCacheCompiledRigSignature(evaluator))
                    : std::vector<RigExecWarmFrameState>();
            std::set<double> explicitInFlight;
            for (const RigExecExplicitJobKey &key :
                 _scheduler->InFlightExplicitKeys(session.rigPath)) {
                if (!key.time.IsDefault()) {
                    explicitInFlight.insert(key.time.TimeCode());
                }
            }
            const std::vector<UsdTimeCode> warmInFlight =
                _scheduler->InFlightTimes(session.rigPath);
            for (size_t i = 0; i < times.size(); ++i) {
                const double time = times[i];
                if (!std::isfinite(time)) {
                    continue;
                }
                // Cached frames need no job; a Warming state or an
                // in-flight job of either class means the answer is
                // already coming -- never admit a duplicate of it.
                if (i < states.size() &&
                    states[i] != RigExecWarmFrameState::Uncached) {
                    continue;
                }
                if (explicitInFlight.count(time)) {
                    continue;
                }
                bool warmRunning = false;
                for (const UsdTimeCode &at : warmInFlight) {
                    if (at.IsNumeric() && at.GetValue() == time) {
                        warmRunning = true;
                        break;
                    }
                }
                if (warmRunning) {
                    continue;
                }
                candidate.times.push_back(time);
                if (candidate.times.size() >= _warmSamplingMaxInvocations) {
                    break;
                }
            }
            if (!candidate.times.empty()) {
                candidates.push_back(std::move(candidate));
            }
        }
    }
    if (!scheduler) {
        return 0;
    }
    size_t admitted = 0;
    for (const Candidate &candidate : candidates) {
        for (const double time : candidate.times) {
            // The same owner-thread factory the warm triggers use: sample
            // the frame's input vector, take the freshness proof, and
            // build the frozen job that publishes the complete pose
            // behind the fences.
            RigExecWarmFactoryResult built =
                BuildWarmWork(candidate.rig, UsdTimeCode(time),
                              candidate.generation, RigExecFrozenStepRunner());
            if (!built.work) {
                continue; // reasoned skip: the rewarm lane stays its route
            }
            RigExecExplicitRequest request = candidate.base;
            if (!RigExecOutputTime::TryTimeCode(time, &request.time)) {
                continue;
            }
            const RigExecWarmWork warm = std::move(built.work);
            // The warm job body is the work; it re-checks the fence with
            // the stamped token before publishing. Numeric times carry
            // the same per-time token for both classes.
            RigExecExplicitWork work =
                [warm](const RigExecExplicitRequest &stamped) {
                    RigExecWarmRequest legacy;
                    legacy.rig = stamped.rig;
                    legacy.time = stamped.time.IsDefault()
                                      ? UsdTimeCode::Default()
                                      : UsdTimeCode(stamped.time.TimeCode());
                    legacy.generation = stamped.editGeneration;
                    legacy.fenceToken = stamped.fenceToken;
                    return warm(legacy);
                };
            if (scheduler->AdmitExplicitOutput(request, std::move(work))) {
                ++admitted;
            }
            // A decline (job or byte cap) leaves the frame pending: the
            // rewarm lane and the aged fill stay its route.
        }
    }
    return admitted;
}

bool
RigExecImagingRegistry::HasFrameDemand()
{
    if (_frameTable->HasDemand()) {
        return true;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return !_pendingFrameDemand.empty();
}

void
RigExecImagingRegistry::_BroadcastFramesChanged()
{
    // Unlocked, like _Broadcast: readers re-enter through their dirties.
    for (const Chain &chain : _BoundChains()) {
        if (chain.results) {
            chain.results->NotifyFramesChanged();
        }
    }
}

} // namespace rigExec
