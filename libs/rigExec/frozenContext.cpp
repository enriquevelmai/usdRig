// Frozen evaluation API, input containers, serial scopes, and purity audit.

#include "rigExecSchema/schemaNames.h"
#include "frozenContextInternal.h"
#include "inputReplay.h"
#include "frameCache.h"
#include "generation.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((inputsEnabled, SchemaAttrText::inputsEnabled))
    ((inputsDefaultWeight, SchemaAttrText::inputsDefaultWeight))
    ((rigExecOperation, SchemaAttrText::rigExecOperation))
    ((inputsValue, SchemaAttrText::inputsValue))
    ((inputsMin, SchemaAttrText::inputsMin))
    ((inputsMax, SchemaAttrText::inputsMax))
    ((inputsKeys, SchemaAttrText::inputsKeys))
    ((inputsTangents, SchemaAttrText::inputsTangents))
    ((rigExecWeightObject, SchemaAttrText::rigExecWeightObject))
);

// Thread-local serial depth. A plain counter rather than a bool so scopes
// nest; the query is "inside at least one".
thread_local size_t _frozenSerialDepth = 0;

} // namespace

namespace frozenDetail {

thread_local RigExecFrozenRunReport *_frozenRunReport = nullptr;

} // namespace frozenDetail

std::vector<RigExecUpstreamValue>
RigExecUpstreamValuesOf(const std::vector<RigExecValueOverride> &inputs)
{
    std::map<SdfPath, const VtValue *> byPath;
    for (const RigExecValueOverride &o : inputs) {
        if (!o.attribute.IsEmpty() && o.computation.IsEmpty() &&
            !o.prim.IsEmpty()) {
            byPath[o.prim.AppendProperty(o.attribute)] = &o.value;
        }
    }
    std::vector<RigExecUpstreamValue> values;
    values.reserve(byPath.size());
    for (const auto &[path, value] : byPath) {
        values.push_back(RigExecUpstreamValue{path, *value,
                                              RigExecUpstreamFoldHash(*value)});
    }
    return values;
}

bool
RigExecFrameInputs::HasChainResolvedInputs() const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.viaChain) {
            return true;
        }
    }
    return false;
}

namespace {

// The constant head leaf keyed \p path, or -1.
int
_HeadConstantAt(const RigExecHeadLeafConstants *constants, const SdfPath &path)
{
    if (!constants) {
        return -1;
    }
    for (size_t j = 0; j < constants->keys.size(); ++j) {
        if (constants->keys[j] == path && !constants->varying[j]) {
            return int(j);
        }
    }
    return -1;
}

} // namespace

const VtValue *
RigExecFrameInputs::Find(const SdfPath &path) const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.path == path) {
            return &sampled.value;
        }
    }
    const int j = _HeadConstantAt(headLeafConstants.get(), path);
    return j < 0 ? nullptr : &headLeafConstants->values[size_t(j)];
}

bool
RigExecFrameInputs::Contains(const SdfPath &path) const
{
    for (const RigExecSampledInput &sampled : values) {
        if (sampled.path == path) {
            return true;
        }
    }
    return _HeadConstantAt(headLeafConstants.get(), path) >= 0;
}

void
RigExecFrameInputs::SetOverrides(const std::vector<RigExecValueOverride> &list)
{
    overrides = list;
    overridePaths = _FrozenOverridePaths(list);
}

void
RigExecFrameInputs::Clear()
{
    time = UsdTimeCode::Default();
    oraclePublications.reset();
    oracleWeightInputs.clear();
    values.clear();
    layoutLeaves.clear();
    layoutSourcePaths.clear();
    varyingLayoutRows.clear();
    varyingRevisionLeaves.clear();
    revisionLeaves.clear();
    derivedLeaves.clear();
    stageSeeds = RigExecStageFrameSeeds();
    overrides.clear();
    overridePaths.clear();
    headLeafConstants.reset();
    staticSamples.reset();
    digestOrder.reset();
    upstream.clear();
    windowRings.clear();
}

bool
RigExecFrozenArena::ResizeFor(const RigExecFrozenEvalContext &context)
{
    return Resize(context.slotCount);
}

bool
RigExecFrozenArena::Resize(size_t slots)
{
    if (slots > kMaxFrozenArenaSlots) {
        return false;
    }
    _slots.assign(slots, 0.0);
    return true;
}

void
RigExecFrozenArena::Clear()
{
    std::fill(_slots.begin(), _slots.end(), 0.0);
}

bool
RigExecFrozenSerialActive()
{
    return _frozenSerialDepth > 0;
}

RigExecFrozenSerialScope::RigExecFrozenSerialScope() : _active(true)
{
    ++_frozenSerialDepth;
}

RigExecFrozenSerialScope::~RigExecFrozenSerialScope()
{
    if (_active && _frozenSerialDepth > 0) {
        --_frozenSerialDepth;
    }
}

RigExecRigPose
RigExecEvaluateFrozen(const RigExecFrozenEvalContext &context,
                      const RigExecFrameInputs &inputs,
                      RigExecFrozenStepRunner runner,
                      const RigExecBackgroundScheduler *scheduler,
                      const SdfPath &rig, RigExecFrozenRunReport *report)
{
    // Background lanes record nothing; see RigExecFrozenProgram.
    std::optional<RigExecInputReplayComparisonScope> replayComparison;
    if (context.frozen && context.frozen->RecordsInputReplay()) {
        replayComparison.emplace("RigExecEvaluateFrozen");
    }
    if (report) {
        report->Clear();
    }
    RigExecRigPose declined;
    declined.time = inputs.time;
    declined.valid = false;

    // A refusal rig has no program a worker could run. The context
    // carries the flag so that even a job enqueued by mistake -- a caller
    // that never asked RigExecShouldEnqueueBackgroundJob -- declines here
    // rather than evaluating against a vector sampled for nothing.
    if (context.flags & kRigExecFrozenBakeRefused) {
        return declined;
    }
    // The vector and the context were sampled for different frames, or the
    // vector was truncated in flight. Running against a partial input set
    // would publish a pose no digest names: dropped, never served.
    if (inputs.values.size() != context.varyingInputCount) {
        return declined;
    }
    if (!runner) {
        return declined;
    }
    if (scheduler &&
        !RigExecGenerationCheckAtStart(*scheduler, rig, context.generation)) {
        return declined;
    }
    RigExecFrozenArena arena;
    if (!arena.ResizeFor(context)) {
        return declined;
    }
    // The serial scope is the serial-variant boundary: every kernel variant the runner
    // reaches takes its serial form on this thread, whatever the
    // process-wide switch says, so no work leaks back onto the shared arena
    // at normal priority. The runner must also never hop threads: the scope
    // constrains this thread and no other.
    RigExecRigPose pose;
    pose.time = inputs.time;
    pose.valid = false;
    bool ran = false;
    {
        RigExecFrozenSerialScope serial;
        // The production runner fills the caller's report on this thread;
        // the pointer is withdrawn however the runner leaves.
        struct ReportScope {
            explicit ReportScope(RigExecFrozenRunReport *r)
            {
                _frozenRunReport = r;
            }
            ~ReportScope()
            {
                _frozenRunReport = nullptr;
            }
        } reportScope(report);
        ran = runner(context, inputs, arena, &pose);
    }
    if (!ran || !pose.valid) {
        if (report) {
            report->Clear();
        }
        return declined;
    }
    pose.time = inputs.time;
    if (scheduler && !RigExecGenerationCheckBeforePublish(*scheduler, rig,
                                                          context.generation)) {
        return declined;
    }
    return pose;
}

RigExecRigPose
RigExecEvaluateFrozen(const RigExecFrozenEvalContext &context,
                      const RigExecFrameInputs &inputs)
{
    std::optional<RigExecInputReplayComparisonScope> replayComparison;
    if (context.frozen && context.frozen->RecordsInputReplay()) {
        replayComparison.emplace("RigExecEvaluateFrozen");
    }
    // Fail-closed stub: without a step runner no request can prove
    // bit-identity, so every request answers invalid -- the fail-closed
    // answer -- while still carrying the requested time on that pose.
    (void)context;
    RigExecRigPose pose;
    pose.time = inputs.time;
    pose.valid = false;
    return pose;
}

bool
RigExecRunFrozenHistoryTick(const RigExecFrozenEvalContext &context,
                            const RigExecFrameInputs &inputs,
                            const std::vector<RigExecWindowRing> &rings,
                            std::vector<RigExecWindowPacketPtr> *captures,
                            std::string *error)
{
    const auto fail = [error](const char *why) {
        if (error)
            *error = why;
        return false;
    };
    if (!captures)
        return fail("no capture destination");
    captures->clear();
    if (!context.frozen)
        return fail("no frozen program");
    if (context.flags & kRigExecFrozenBakeRefused)
        return fail("no admitted program");
    if (inputs.values.size() != context.varyingInputCount)
        return fail("history inputs were sampled for another program");
    if (!_RunFrozenHistoryTick(context, inputs, rings, captures))
        return fail("the history tick declined");
    return true;
}

RigExecFrozenStepRunner
RigExecMakeProductionStepRunner()
{
    return [](const RigExecFrozenEvalContext &context,
              const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
              RigExecRigPose *pose) {
        (void)arena;
        if (!pose) {
            return false;
        }
        *pose = RigExecRigPose();
        if (!context.frozen) {
            // No program to run: the rig was not frozen (refused at freeze
            // time, or built before the executor landed). Decline -- hand
            // the generation back -- and the frame evaluates live when
            // asked.
            return false;
        }
        if (context.flags & kRigExecFrozenBakeRefused) {
            return false;
        }
        // The arena sizes the private working state; the runner carries its
        // own clone instead, so the check is only that the entry point sized
        // what the context promised.
        if (arena.Size() < context.slotCount) {
            return false;
        }
        return _RunFrozen(context, inputs, pose);
    };
}

RigExecFrozenRequestedStepRunner
RigExecMakeProductionRequestedStepRunner()
{
    return [](const RigExecFrozenEvalContext &context,
              const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
              const RigExecFrozenRequestedRun &requested,
              RigExecRequestedRunReport *report,
              std::vector<RigExecOutputPacket> *packets, std::string *error) {
        if (!report || !packets) {
            return false;
        }
        *report = RigExecRequestedRunReport{};
        packets->clear();
        if (!context.frozen || !requested.catalog) {
            if (error)
                *error = context.frozen ? "the job names no output catalog"
                                        : "no frozen program to run";
            return false;
        }
        if (context.flags & kRigExecFrozenBakeRefused) {
            if (error)
                *error = "no admitted compiled program";
            return false;
        }
        // The arena sizes the private working state; the runner carries its
        // own clone instead, so the check is only that the entry point sized
        // what the context promised.
        if (arena.Size() < context.slotCount) {
            if (error)
                *error = "the arena is smaller than the context promised";
            return false;
        }
        return _RunFrozenRequested(context, inputs, requested, report, packets,
                                   error);
    };
}

bool
RigExecMakeFrozenRequestedRun(
    const std::shared_ptr<const RigExecOutputCatalog> &catalog,
    const RigExecOutputTimeDemand &demand,
    const RigExecOutputGeneration &captured, RigExecFrozenRequestedRun *out,
    std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    if (!out)
        return fail("no requested-run destination");
    if (!catalog)
        return fail("no output catalog to resolve the request against");
    RigExecOutputRequest request;
    request.demands.push_back(demand);
    std::string reason;
    if (!request.IsWellFormed(&reason))
        return fail(reason);
    const RigExecOutputDemandIndex &index = catalog->GetDemandIndex();
    std::vector<size_t> slots;
    if (!index.ResolveRequest(request, &slots, &reason))
        return fail(reason);
    // An output with no canonical producer would enter the union as nothing
    // and answer Pending forever: the caller recaptures instead.
    std::vector<size_t> producible;
    producible.reserve(slots.size());
    for (size_t slot : slots) {
        if (index.ProducerStatus(slot) != RigExecOutputStatus::Pending) {
            return fail("output " + index.Handle(slot).semanticKey +
                        " has no canonical producer");
        }
        producible.push_back(slot);
    }
    RigExecFrozenRequestedRun job;
    job.closure = index.Union(producible);
    if (job.closure.Empty())
        return fail("the request closes no operation of this program");
    // The catalog's handle names no value revision; the demand's handle is the
    // constraint extraction answers against, so it travels into the job. One
    // target named at two revisions has no answer one run could give.
    for (size_t slot : producible) {
        RigExecOutputHandle output = index.Handle(slot);
        uint64_t pinned = 0;
        bool named = false;
        for (const RigExecOutputHandle &asked : demand.handles) {
            if (!asked.SameTarget(output))
                continue;
            if (named && pinned != asked.valueRevision)
                return fail(output.semanticKey + " is named at two value "
                                                 "revisions; one run answers "
                                                 "one revision");
            pinned = asked.valueRevision;
            named = true;
        }
        output.valueRevision = pinned;
        job.outputs.push_back(std::move(output));
    }
    job.captured = captured;
    job.catalog = catalog;
    *out = std::move(job);
    if (error)
        error->clear();
    return true;
}

RigExecFrozenRequestedResult
RigExecEvaluateFrozenRequested(const RigExecFrozenEvalContext &context,
                               const RigExecFrameInputs &inputs,
                               const RigExecFrozenRequestedRun &requested,
                               RigExecFrozenRequestedStepRunner runner,
                               const RigExecBackgroundScheduler *scheduler,
                               const SdfPath &rig,
                               RigExecFrozenRunReport *trace)
{
    // Background lanes record nothing; see RigExecFrozenProgram.
    std::optional<RigExecInputReplayComparisonScope> replayComparison;
    if (context.frozen && context.frozen->RecordsInputReplay()) {
        replayComparison.emplace("RigExecEvaluateFrozenRequested");
    }
    if (trace) {
        trace->Clear();
    }
    RigExecFrozenRequestedResult result;
    // A refusal rig has no program a worker could run, and a vector sampled
    // for another frame is a job no digest names: both decline here rather
    // than run a closure over inputs nobody sampled for it.
    if (context.flags & kRigExecFrozenBakeRefused) {
        result.error = "no admitted compiled program";
        return result;
    }
    if (inputs.values.size() != context.varyingInputCount) {
        result.error = "the inputs were sampled for another frame";
        return result;
    }
    if (!requested.catalog) {
        result.error = "the job names no output catalog";
        return result;
    }
    // The catalog must name the compilation this context runs, checked before
    // any operation executes: a job built from another program's catalog would
    // run this closure and read this storage under another rig's bindings.
    // A lane cloned from this snapshot holds the same affinity.
    if (context.frozen && context.frozen->program &&
        requested.catalog->GetProgramAffinity() !=
            context.frozen->program->programAffinity) {
        result.error = "the job's output catalog is not this program's";
        return result;
    }
    if (requested.closure.Empty()) {
        result.error = "the job names no operation";
        return result;
    }
    if (requested.outputs.empty()) {
        result.error = "the job names no output to answer";
        return result;
    }
    if (!runner) {
        result.error = "no requested-run step runner";
        return result;
    }
    if (scheduler &&
        !RigExecGenerationCheckAtStart(*scheduler, rig, context.generation)) {
        result.error = "the generation was retired before the job started";
        return result;
    }
    RigExecFrozenArena arena;
    if (!arena.ResizeFor(context)) {
        result.error = "the context's slot count is absurd";
        return result;
    }
    bool ran = false;
    {
        RigExecFrozenSerialScope serial;
        struct TraceScope {
            explicit TraceScope(RigExecFrozenRunReport *t)
            {
                _frozenRunReport = t;
            }
            ~TraceScope()
            {
                _frozenRunReport = nullptr;
            }
        } traceScope(trace);
        ran = runner(context, inputs, arena, requested, &result.report,
                     &result.packets, &result.error);
    }
    if (!ran) {
        // Nothing stands: an executed-but-refused run's facts and packets are
        // as unverifiable as a declined one's, and the owner recaptures.
        result.ran = false;
        result.packets.clear();
        result.report = RigExecRequestedRunReport{};
        if (trace)
            trace->Clear();
        if (result.error.empty())
            result.error = "the requested run declined";
        return result;
    }
    if (scheduler && !RigExecGenerationCheckBeforePublish(*scheduler, rig,
                                                          context.generation)) {
        // The bodies ran, but the generation they ran under is retired: these
        // bytes answer a frame no request owns, so they are reported and
        // dropped rather than served.
        result.ran = false;
        result.packets.clear();
        result.report = RigExecRequestedRunReport{};
        if (trace)
            trace->Clear();
        result.error = "the generation was retired while the job ran";
        return result;
    }
    result.ran = true;
    return result;
}

bool
RigExecBindChainSampleInputs(const RigExecRigEvaluator &evaluator,
                             RigExecChainSampleBindings *out,
                             std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no bindings to bind into");
    }
    const UsdStageRefPtr stage = evaluator.GetEvaluationStage();
    if (!stage) {
        return fail("no stage to bind the chains against");
    }
    std::vector<_ChainMoverDesc> movers;
    if (!_DiscoverChainMovers(evaluator, &movers, error)) {
        return false;
    }
    std::map<SdfPath, std::vector<_ChainMoverDesc>> grouped;
    for (const _ChainMoverDesc &mover : movers) {
        grouped[mover.target].push_back(mover);
    }
    std::vector<SdfPath> order;
    if (!_OrderDiscoveredChains(stage, grouped, &order, error)) {
        return false;
    }
    RigExecChainSampleBindings bound;
    const auto &T = *_tokens;
    for (const SdfPath &targetPath : order) {
        RigExecChainSampleChain chain;
        chain.targetPath = targetPath;
        chain.target = stage->GetAttributeAtPath(targetPath);
        if (chain.target) {
            chain.targetQuery = UsdAttributeQuery(chain.target);
            chain.valueType = chain.target.GetTypeName();
        }
        for (const _ChainMoverDesc &mover : grouped[targetPath]) {
            RigExecChainSampleRevision revision;
            revision.moverPath = mover.moverPath;
            revision.moverPrim = stage->GetPrimAtPath(mover.moverPath);
            if (const UsdRelationship rel =
                    revision.moverPrim.GetRelationship(T.rigExecWeightObject)) {
                rel.GetTargets(&revision.weightObjects);
            }
            const UsdPrim &prim = revision.moverPrim;
            revision.enabled = _BindChainInput(prim, T.inputsEnabled);
            revision.defaultWeight =
                _BindChainInput(prim, T.inputsDefaultWeight);
            revision.operation = _BindChainInput(prim, T.rigExecOperation);
            revision.value = _BindChainInput(prim, T.inputsValue);
            revision.minimum = _BindChainInput(prim, T.inputsMin);
            revision.maximum = _BindChainInput(prim, T.inputsMax);
            revision.keys = _BindChainInput(prim, T.inputsKeys);
            revision.tangents = _BindChainInput(prim, T.inputsTangents);
            chain.revisions.push_back(std::move(revision));
        }
        for (const RigExecPhasedConnection &connection :
             evaluator.GetPhasedConnections()) {
            if (connection.target == targetPath) {
                chain.phased.push_back(connection);
            }
        }
        bound.chains.push_back(std::move(chain));
    }
    *out = std::move(bound);
    return true;
}

} // namespace rigExec
