// Stage-side program snapshot construction and constant patching.

#include "rigExecSchema/schemaNames.h"
#include "frozenContextInternal.h"
#include "inputReplay.h"
#include "oracleDispatch.h"
#include "bakedOpValues.h"
#include "pxr/base/tf/getenv.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace rigExec {

using namespace frozenDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((inputsEnabled, SchemaAttrText::inputsEnabled))
    ((inputsDefaultWeight, SchemaAttrText::inputsDefaultWeight))
    ((rigExecSkinningMethod, SchemaAttrText::rigExecSkinningMethod))
);

} // namespace

namespace frozenDetail {

namespace {

// Lets go of a scratch buffer whose contents are not state.
template <class T>
void
_DropScratch(std::vector<T> *scratch)
{
    std::vector<T>().swap(*scratch);
}

// A revision's idle scratch: every step that reads one of these buffers
// fills it first in the same step.
void
_DropRevisionScratch(RigExecBakedProgramImpl::GeomRevision *revision)
{
    _DropScratch(&revision->resolveScratch);
    _DropScratch(&revision->deltasSpare);
    _DropScratch(&revision->wholeEntering);
    for (RigExecBakedPointsBinding &binding : revision->pointBindings) {
        _DropScratch(&binding.gather);
    }
    for (auto &channel : revision->blendChannels) {
        for (auto &sample : channel.samples) {
            _DropScratch(&sample.pointBinding.gather);
        }
    }
}

// Whether the op keys \p copy holds, copied from \p src, stand: \p src
// completed a run, and every value an op writes is initialized, exactly
// keyed, and still described by its key in \p copy. A pure function of
// the two programs' contents.
bool
_CloneKeysStand(const RigExecBakedProgramImpl &src,
                const RigExecBakedProgramImpl &copy)
{
    bool retainedComplete =
        src.everRan && src.opAdapter.compiled && src.opAdapter.everRan &&
        src.opAdapter.inputKeys.size() == src.steps.size() &&
        src.opAdapter.sourceKeys.size() == src.steps.size() &&
        src.opAdapter.inputExact.size() == src.steps.size();
    for (const auto &op : src.opGraph.ops)
        for (const auto id : op.descriptor.writes) {
            if (id >= copy.opAdapter.values.size()) {
                retainedComplete = false;
                continue;
            }
            const auto &value = copy.opAdapter.values[size_t(id)];
            const auto domain = RigExecBakedSlotDomain(value.domain);
            if (!value.initialized ||
                !RigExecBakedOpValueKeyIsExact(copy, domain, value.slot)) {
                retainedComplete = false;
                continue;
            }
            if (!RigExecBakedOpValueKeyStands(copy, domain, value.slot,
                                              value.key))
                retainedComplete = false;
        }
    return retainedComplete;
}

void
_AdoptCloneVerdict(RigExecBakedProgramImpl *program, bool stands)
{
    program->opAdapter.everRan = stands;
    if (!stands)
        program->opAdapter.retainedFirst.clear();
}

} // namespace

// Memberwise program clone. The copy is explicit, field by field, in struct
// order. Live pointers are nulled -- the worker repoints
// the ones it uses at worker-owned state -- and every other field is
// copied, including the per-frame working state (the history the frozen
// run branches from) and the USD handles (copied but dead; see the audit).
// When a field is added to RigExecBakedProgramImpl, it must be added here:
// a missing compiled rig field silently changes the frozen run's answers, and a
// missing per-frame field silently changes its history. The bit-identity
// test is the backstop, not the discipline.
void
_CloneImpl(const RigExecBakedProgramImpl &src, RigExecBakedProgramImpl *dst,
           _CloneVerdict verdict)
{
    RigExecBakedProgramImpl &D = *dst;
    D.evaluator = nullptr;
    D.stage = UsdStageRefPtr();
    D.assetRootPath = src.assetRootPath;
    D.resolvedInputs = nullptr;
    D.profiler = nullptr;
    D.recordOpTimings = false;
    D.interactiveOverrides = nullptr;
    D.jointSolverBinding = nullptr;
    D.ownedJointSolverBinding = src.ownedJointSolverBinding;
    D.cycleExclusionProof = src.cycleExclusionProof;
    D.cycleSkipReasons = src.cycleSkipReasons;
    D.solverGuidesEnabled = nullptr;
    D.hasPropertyChains = src.hasPropertyChains;
    D.programAffinity = src.programAffinity;
    D.paths = src.paths;
    D.pathTexts = src.pathTexts;
    D.index = src.index;
    D.slotKind = src.slotKind;
    D.providerActive = src.providerActive;
    D.parent = src.parent;
    D.propParent = src.propParent;
    D.xformSlots = src.xformSlots;
    D.xformPrimsBySlot = src.xformPrimsBySlot;
    D.assetRoot = src.assetRoot;
    D.xformBase = src.xformBase;
    D.lastXformBase = src.lastXformBase;
    D.constraintArrays = src.constraintArrays;
    D.nativeSources = src.nativeSources;
    D.nativeFrames = src.nativeFrames;
    D.lastNativeFrames = src.lastNativeFrames;
    D.nativeFrameOk = src.nativeFrameOk;
    D.lastNativeFrameOk = src.lastNativeFrameOk;
    D.ladders = src.ladders;
    D.interveningSlots = src.interveningSlots;
    D.interveningAnchors = src.interveningAnchors;
    D.ladderVarying = src.ladderVarying;
    D.ladderOverrides = src.ladderOverrides;
    D.restChainVaries = src.restChainVaries;
    D.posedAuthored = src.posedAuthored;
    D.posedAuthoredM = src.posedAuthoredM;
    D.restM = src.restM;
    D.restPts = src.restPts;
    D.restFrames = src.restFrames;
    D.selfD = src.selfD;
    D.posedD = src.posedD;
    D.parentSpaceM = src.parentSpaceM;
    D.parentSpaceAuthored = src.parentSpaceAuthored;
    D.lastPosedD = src.lastPosedD;
    D.lastParentSpaceM = src.lastParentSpaceM;
    D.lastParentSpaceAuthored = src.lastParentSpaceAuthored;
    D.lastRotationSign = src.lastRotationSign;
    D.parentDinv = src.parentDinv;
    D.rotOrder = src.rotOrder;
    D.restRoundTrip = src.restRoundTrip;
    D.defaultRoundTrip = src.defaultRoundTrip;
    D.lastRestM = src.lastRestM;
    D.lastSelfD = src.lastSelfD;
    D.lastParentDinv = src.lastParentDinv;
    D.lastPosedAuthoredM = src.lastPosedAuthoredM;
    D.lastPosedAuthored = src.lastPosedAuthored;
    D.lastRotOrder = src.lastRotOrder;
    D.restChanged = src.restChanged;
    D.ladderChanged = src.ladderChanged;
    D.restMoved = src.restMoved;
    D.ladderMoved = src.ladderMoved;
    D.xyzToken = src.xyzToken;
    D.noScaleAvars = src.noScaleAvars;
    D.rotationSign = src.rotationSign;
    D.poseInterpolators = src.poseInterpolators;
    D.poseWeightPaths = src.poseWeightPaths;
    D.poseWeights = src.poseWeights;
    D.poseWeightIndex = src.poseWeightIndex;
    D.avarConstants = src.avarConstants;
    D.avarBindings = src.avarBindings;
    D.avarConstantBindings = src.avarConstantBindings;
    D.avarBindingBegin = src.avarBindingBegin;
    D.avarConstantBindingBegin = src.avarConstantBindingBegin;
    D.patchableAvars = src.patchableAvars;
    D.promotedAvars = src.promotedAvars;
    D.boundInputs = src.boundInputs;
    D.varyingInputs = src.varyingInputs;
    D.avars = src.avars;
    D.posedM = src.posedM;
    D.base = src.base;
    D.fin = src.fin;
    D.finLast = src.finLast;
    D.baseLast = src.baseLast;
    D.propertyResults = src.propertyResults;
    D.frameRecords = src.frameRecords;
    D.frameMatrix = src.frameMatrix;
    D.frameMatrixValid = src.frameMatrixValid;
    D.finalMatrix = src.finalMatrix;
    D.baseMatrix = src.baseMatrix;
    D.needFinal = src.needFinal;
    D.needBase = src.needBase;
    D.solvers = src.solvers;
    for (auto &solver : D.solvers) {
        _DropScratch(&solver.ribbonPointsBinding.gather);
    }
    D.guideSolvers = src.guideSolvers;
    D.solverIndex = src.solverIndex;
    D.aggregates = src.aggregates;
    D.constraints = src.constraints;
    // The windowed ops, their mask and the history the source last held:
    // a clone's first run reads the rings it was handed, so the rings and
    // captures travel as per-frame state.
    D.windowedOps = src.windowedOps;
    D.historyMask = src.historyMask;
    D.historyTick = false;
    D.windowCaptures = src.windowCaptures;
    D.windowRings = src.windowRings;
    D.windowRingChanged = src.windowRingChanged;
    D.windowRingSerial = src.windowRingSerial;
    D.windowOfOp = src.windowOfOp;
    D.walkSteps = src.walkSteps;
    D.steps = src.steps;
    // The steps' output travels with them, so the set of the steps holding
    // any does too; the clone counts its own verification mismatches.
    D.epilogue = src.epilogue;
    D.epilogue.mismatches = 0;
    D.excludedSteps = src.excludedSteps;
    D.opGraph = src.opGraph;
    D.opAdapter = src.opAdapter;
    // The reset judge only: the clone's first execution resets whole.
    D.opWorkspace.verifyReset = src.opWorkspace.verifyReset;
    D.verifyChainVersions = src.verifyChainVersions;
    D.chainContentKeys = src.chainContentKeys;
    D.chainVersionMismatches = 0;
    D.verifyPacketVersions = src.verifyPacketVersions;
    D.packetContentKeys = src.packetContentKeys;
    D.packetVersionMismatches = 0;
    D.verifyRangeChains = src.verifyRangeChains;
    D.rangeVerifyMismatches = 0;
    // The values the copied source keys were built from, with their shared
    // index: the clone's runs compare its own leaves with them.
    D.sourceWatch = src.sourceWatch;
    D.verifySourceKeys = src.verifySourceKeys;
    D.sourceKeyMismatches = 0;
    D.sourceKeysBuilt = 0;
    D.sourceWatchVisits = 0;
    D.composeGroups = src.composeGroups;
    // The space switches, and the per-slot index the compose step
    // asks before it takes the switched branch. Left out, the
    // frozen compose falls straight through to the namespace
    // parent -- which moves every switched control and everything
    // under it, and reads as a parity mismatch on 1509 control
    // frames of a rig that is standing in the same place.
    D.spaceSwitches = src.spaceSwitches;
    D.autoClavicles = src.autoClavicles;
    D.autoClavicleBySlot = src.autoClavicleBySlot;
    D.switchFrameContexts = src.switchFrameContexts;
    D.switchFrames = src.switchFrames;
    D.spaceSwitchBySlot = src.spaceSwitchBySlot;
    D.commits = src.commits;
    D.revisionIndex = src.revisionIndex;
    D.derivedIndex = src.derivedIndex;
    D.chainRevisionBegin = src.chainRevisionBegin;
    D.chainRevisionEnd = src.chainRevisionEnd;
    D.revisionChunkBase = src.revisionChunkBase;
    D.revisionChunkCount = src.revisionChunkCount;
    D.chainChunkBegin = src.chainChunkBegin;
    D.chainChunkEnd = src.chainChunkEnd;
    D.revisionFuseStep = src.revisionFuseStep;
    D.chunkRevision = src.chunkRevision;
    D.skinTopologyLayouts = src.skinTopologyLayouts;
    D.clustering = src.clustering;
    // Build's settings: the clone's partition settings and the worker's
    // kernels are the live program's choices, and its bodies count into
    // their own audit counter.
    D.chunkVertexTarget = src.chunkVertexTarget;
    D.gpuDeformStats = src.gpuDeformStats;
    D.scheduleParallel = src.scheduleParallel;
    D.scheduleReport = src.scheduleReport;
    D.scheduleCalibrate = src.scheduleCalibrate;
    D.scheduleStepTiming = src.scheduleStepTiming;
    D.traceAllSteps = src.traceAllSteps;
    D.calibrationFrames = src.calibrationFrames;
    D.stepTimingFrames = src.stepTimingFrames;
    D.chunkCap = src.chunkCap;
    D.rangeChains = src.rangeChains;
    D.groupVertexTarget = src.groupVertexTarget;
    D.groupCap = src.groupCap;
    D.groupGates = src.groupGates;
    D.roleMode = src.roleMode;
    D.exportKeep = src.exportKeep;
    D.exportPinnedPaths = src.exportPinnedPaths;
    D.gateViolations.store(0, std::memory_order_relaxed);
    D.useSimd = src.useSimd;
    D.purityAudit = src.purityAudit;
    D.purityViolations.count.store(0, std::memory_order_relaxed);
    D.cones = src.cones;
    D.closedSteps = src.closedSteps;
    D.closed = src.closed;
    D.lastAvars = src.lastAvars;
    D.lastOverridden = src.lastOverridden;
    D.lastHaveBase = src.lastHaveBase;
    D.lastTime = src.lastTime;
    D.everRan = src.everRan;
    D.programStamp = src.programStamp;
    D.lastProgramStamp = src.lastProgramStamp;
    // Pending value edits travel with the stamps: a clone first run at the
    // source's lastTime -- a rewarm after a generation the live program did
    // not answer -- is the run that owes them, and time alone would not
    // dirty the steps they reached.
    D.edited = src.edited;
    D.anyEdited = src.anyEdited;
    D.valueEditSerial = src.valueEditSerial;
    D.editSerial = src.editSerial;
    D.lastClosedClusters = src.lastClosedClusters;
    D.lastClosedSteps = src.lastClosedSteps;
    D.jointSlots = src.jointSlots;
    D.jointPaths = src.jointPaths;
    D.controlSlots = src.controlSlots;
    D.controlPaths = src.controlPaths;
    D.solverArrays = src.solverArrays;
    D.jointPublishOrder = src.jointPublishOrder;
    D.controlPublishOrder = src.controlPublishOrder;
    D.solverPublishOrder = src.solverPublishOrder;
    D.jointPathsAscending = src.jointPathsAscending;
    D.controlPathsAscending = src.controlPathsAscending;
    D.solverArraysAscending = src.solverArraysAscending;
    D.timedPrologueUs = 0;
    D.timedRegionUs = 0;
    D.timedEpilogueUs = 0;
    D.timedFrames = 0;
    D.calibrationFramesSeen = 0;
    D.stepTimingFramesSeen = 0;
    D.coldPrologueUs = D.coldRegionUs = D.coldEpilogueUs = 0;
    D.coldFrames = 0;
    D.measureColdRuns = src.measureColdRuns;
    D.coldRunExcluded = false;
    D.measurementSuspended = false;
    // The steps above carry the source's stamps; the clone clears them too.
    D.stampedSteps = src.stampedSteps;
    D.timedClusters = src.timedClusters;
    D.jointMatrixPublished = src.jointMatrixPublished;
    D.chains = src.chains;
    // The chains' GPU ownership travels with them, so the mode does too;
    // the clone indexes its own derived steps at its first boundary.
    D.gpuDeformMode = src.gpuDeformMode;
    D.gpuDerivedIndexed = false;
    // Per-consumer overlays borrow the live source layer only during a body.
    // A frozen clone rebases them on its own source layer when consumed.
    // Published and computed vertex groups stay shared with the source; the
    // clone lets go of the spare buffers, so the source's writers still find
    // their scratch unique and the clone allocates on its first write. The
    // same for the revisions' idle scratch, whose contents are not state.
    for (auto &chain : D.chains) {
        // A clone solves every window on the CPU, inside its own graph.
        chain.windowDeviceDerivedSteps.clear();
        for (auto &revision : chain.revisions) {
            revision.windowDevice = false;
            revision.devicePending = false;
            revision.deviceTicks.clear();
            revision.revisionInputs.Clear();
            for (auto &group : revision.groups)
                RigExecDropGroupSpare(&group);
            _DropRevisionScratch(&revision);
        }
        for (auto &group : chain.baseGroups)
            RigExecDropGroupSpare(&group);
        for (auto &derived : chain.derived) {
            derived.revision.revisionInputs.Clear();
            _DropRevisionScratch(&derived.revision);
        }
    }
    D.deltaValues = src.deltaValues;
    D.deltaPresent = src.deltaPresent;
    D.deltaBasePaths = src.deltaBasePaths;
    D.deltaBaseMatrix = src.deltaBaseMatrix;
    D.lastDeltaBaseMatrix = src.lastDeltaBaseMatrix;
    D.deltaBaseOk = src.deltaBaseOk;
    D.lastDeltaBaseOk = src.lastDeltaBaseOk;
    D.weightObjects = src.weightObjects;
    D.weightProgram = src.weightProgram;
    D.weightCycleBlocked = src.weightCycleBlocked;
    D.weightFields = src.weightFields;
    for (auto &field : D.weightFields) {
        _DropScratch(&field.enteringGather);
        for (auto &input : field.pointReads)
            _DropScratch(&input.binding.gather);
    }
    D.volumePlacementBase = src.volumePlacementBase;
    D.weightIndex = src.weightIndex;
    D.volumePlacement = src.volumePlacement;
    D.placedVolumes = src.placedVolumes;
    D.currentPhaseWeights = src.currentPhaseWeights;
    D.falloffLuts = src.falloffLuts;
    // Sized but empty: every packet is rebuilt by its step during the run
    // (reads always follow writes in program order, as live), so copying
    // the previous run's dense fields per job would be pure waste.
    // Completed signatures require their owned packet payloads.
    D.weightPackets = src.weightPackets;
    D.rebuild = src.rebuild;
    D.sourceBackedPaths = src.sourceBackedPaths;
    D.named = src.named;
    D.prims = src.prims;
    D.xformPrims = src.xformPrims;
    D.overridden = src.overridden;
    D.overridableInputs = src.overridableInputs;
    // The leaves, which the worker re-samples whole on every job.
    D.leaves = src.leaves;
    D.leafRefs = src.leafRefs;
    D.leafOfOverride = src.leafOfOverride;
    D.leafByPath = src.leafByPath;
    D.routedOverrides = src.routedOverrides;
    D.lastRoutedOverrides = src.lastRoutedOverrides;
    // The upstream layer as the source's last run placed it: its avar
    // slots, leaves and outputs hold those values, so a job diffs its own
    // table against `lastUpstream` (rule 8) before it runs.
    D.upstreamAdmissible = src.upstreamAdmissible;
    D.upstreamOracle = src.upstreamOracle;
    D.upstreamSetsBuilt = src.upstreamSetsBuilt;
    D.upstream = src.upstream;
    D.lastUpstream = src.lastUpstream;
    D.upstreamOn = src.upstreamOn;
    D.upstreamChanged = src.upstreamChanged;
    D.upstreamMovedThisRun = src.upstreamMovedThisRun;
    D.leafSamples = src.leafSamples;
    // The path leaves' bookkeeping. Their values ride the chains and weight
    // objects above; a frozen job takes its revisions' and derived targets'
    // leaves from the job's vector (RigExecFrameInputs::revisionLeaves,
    // derivedLeaves) and samples none itself.
    D.pathLeafRefs = src.pathLeafRefs;
    D.pathLeafChainResults = src.pathLeafChainResults;
    D.pathLeafChainSerial = src.pathLeafChainSerial;
    D.pathLeafSamples = src.pathLeafSamples;
    // The run the copied leaves' observed values and versions refer to.
    D.pathLeafRun = src.pathLeafRun;
    // The prologue's Build knobs and the stamps its flags answer to; a stamp
    // that disagrees only makes the clone's next pass whole. The sparse
    // index and the override paths are rebuilt by their owners.
    D.pathLeafGating = src.pathLeafGating;
    D.verifyPathLeafGating = src.verifyPathLeafGating;
    D.pathLeafGateMismatches = 0;
    D.leafFlagCompiledRig = src.leafFlagCompiledRig;
    D.avarConstantSerial = src.avarConstantSerial;
    D.sparseSampling = src.sparseSampling;
    D.verifySparseSampling = src.verifySparseSampling;
    D.sparseSamplingMismatches = 0;
    D.leafVisits = 0;
    D.resolvedRoutedPrims = src.resolvedRoutedPrims;
    D.avarsDisturbed = src.avarsDisturbed;
    D.folded = src.folded;
    D.anyOverridden = src.anyOverridden;
    D.publishWeightFields = src.publishWeightFields;
    // Retained property values and bindings. Frozen jobs sample source facts
    // and execute the same compiled graph with no attribute reads in workers.
    D.propertyChains = src.propertyChains;
    D.oraclePublications = src.oraclePublications;
    D.oracleWeightInputs = src.oracleWeightInputs;
    D.propertyRecords = src.propertyRecords;
    D.propertyRecordById = src.propertyRecordById;
    D.poseDependencyPaths = src.poseDependencyPaths;
    D.compiledProviderProgram = src.compiledProviderProgram;
    if (!D.compiledProviderProgram)
        D.providerProgram = src.providerProgram;
    D.providerValues = src.providerValues;
    D.providerLeaves = src.providerLeaves;
    D.providerLeafBlocked = src.providerLeafBlocked;
    D.providerLeafEditSerial = src.providerLeafEditSerial;
    // The leaf keys in opAdapter answer for these samples only together
    // with the reasons publication has not yet consumed.
    D.spaceLeafIndex = src.spaceLeafIndex;
    D.spaceLeafRekey = src.spaceLeafRekey;
    D.spaceLeafKeys = src.spaceLeafKeys;
    D.providerLeafValues = src.providerLeafValues;
    D.providerLeafChains = src.providerLeafChains;
    D.providerRoutedReads = src.providerRoutedReads;
    D.providerFrozenKeys = src.providerFrozenKeys;
    D.providerExternalSlots = src.providerExternalSlots;
    D.providerFrameInputs = src.providerFrameInputs;
    D.poseProviderInputs = src.poseProviderInputs;
    D.connectedPoseProviders = src.connectedPoseProviders;
    D.providerParentRawLeaves = src.providerParentRawLeaves;
    D.providerPruneViolations = src.providerPruneViolations;
    D.providerStepsPruned = src.providerStepsPruned;
    D.providerPruneRoots = src.providerPruneRoots;
    D.providerRefreshes = src.providerRefreshes;
    D.providerRefreshBefore = src.providerRefreshBefore;
    D.propertyVersionCount = src.propertyVersionCount;
    D.headLeaves = src.headLeaves;
    D.headOverrideSlots = src.headOverrideSlots;
    D.headOverrideSlotsByName = src.headOverrideSlotsByName;
    D.propertyValues = src.propertyValues;
    D.propertyVersionValid = src.propertyVersionValid;
    D.propertyChanged = src.propertyChanged;
    D.chainValid = src.chainValid;
    D.chainFinal = src.chainFinal;
    D.recordValues = src.recordValues;
    D.recordStoodAside = src.recordStoodAside;
    D.headOverrides = src.headOverrides;
    D.lastHeadOverrides = src.lastHeadOverrides;
    D.headOverrideMoved = src.headOverrideMoved;
    D.headLeavesSampled = src.headLeavesSampled;
    D.headLeafTime = src.headLeafTime;
    D.headLeafStamp = src.headLeafStamp;
    D.headEverRan = src.headEverRan;
    D.headStamp = src.headStamp;
    D.headLeafSamples = src.headLeafSamples;
    D.headOpsRun = src.headOpsRun;
    // Reader bindings resolve captured source facts and current typed outputs.
    D.readerWalks = src.readerWalks;
    D.crossDomainReads = src.crossDomainReads;
    D.crossDomainErrors = src.crossDomainErrors;
    D.crossDomainOrdinals = src.crossDomainOrdinals;
    D.readerWalkMoved = src.readerWalkMoved;
    D.readerWalkChanged = src.readerWalkChanged;
    D.avarHeadReads = src.avarHeadReads;
    D.verifyCloneKeys = src.verifyCloneKeys;
    D.cloneVerdictMismatches = 0;
    // Adopt only a completed snapshot whose owned outputs still match the
    // signatures copied from that completion. A patched or poisoned payload
    // must run cold rather than borrowing a signature for a different value.
    switch (verdict) {
    case _CloneVerdict::Compute:
        _AdoptCloneVerdict(&D, _CloneKeysStand(src, D));
        break;
    case _CloneVerdict::Defer:
        // everRan and retainedFirst as copied; the caller settles them.
        break;
    case _CloneVerdict::Inherit:
        // The check is a pure function of contents the copy shares with its
        // settled source, so the source's verdict is the copy's.
        _AdoptCloneVerdict(&D, src.opAdapter.everRan);
        if (src.verifyCloneKeys &&
            _CloneKeysStand(src, D) != src.opAdapter.everRan) {
            ++D.cloneVerdictMismatches;
        }
        break;
    }
}

// ------------------------------------------------------------
// Frozen lane storage
// ------------------------------------------------------------

namespace {

// A frozen lane owns a field-by-field clone of the compiled program
// (_CloneImpl above), so the arena's bytes are the program's container
// storage. The sums below are the bound a host charges the coverage budget
// for a lane before that arena exists.
//
// Shared immutable storage -- a shared_ptr's target, a VtValue's array,
// interned path and token text -- is aliased by the copy rather than
// re-owned, so it contributes only the stored size of the handle that points
// at it. Containers are counted by element, and an element's own containers
// one level down; the idle scratch the clone lets go of is still counted,
// because the bound is taken before anything is dropped.

// A red-black node carries its value, a colour and three links; a hash
// bucket array holds one pointer per bucket.
constexpr size_t kTreeLinks = 4 * sizeof(void *);
constexpr size_t kHashLinks = 2 * sizeof(void *);
// A short string lives inside the object; only a longer one allocates.
constexpr size_t kStringInline = 16;

// Whether one element type owns containers of its own: most containers the
// arena clones hold plain values, and walking an element that owns nothing is
// pure cost, so the walk is compiled out for them.
template <class T> struct _OwnsStorage : std::false_type {};

template <class T>
size_t
_OwnedBytes(const T &)
{
    return 0;
}

size_t _OwnedBytes(const std::string &text);
size_t _OwnedBytes(const RigExecCompiledGraph &graph);
size_t _OwnedBytes(const RigExecCompiledOp &op);
size_t _OwnedBytes(const RigExecOpCluster &cluster);
size_t _OwnedBytes(const RigExecOpValueState &value);
size_t _OwnedBytes(const RigExecOpAdapterState &state);
size_t _OwnedBytes(const RigExecOpExclusionProof &proof);
size_t _OwnedBytes(const RigExecBakedLeafPools &pools);
size_t _OwnedBytes(const RigExecBakedPathLeaves &leaves);
size_t _OwnedBytes(const RigExecBakedSourceWatch &watch);
size_t _OwnedBytes(const RigExecBakedCones &cones);
size_t _OwnedBytes(const RigExecBakedClusterSet &set);
size_t _OwnedBytes(const RigExecBakedProgramImpl::ConstraintArrays &arrays);
size_t _OwnedBytes(const RigExecBakedProgramImpl::Constraint &constraint);
size_t _OwnedBytes(const RigExecBakedProgramImpl::Solver &solver);
size_t _OwnedBytes(const RigExecBakedProgramImpl::GeomChain &chain);
size_t _OwnedBytes(const RigExecBakedProgramImpl::GeomRevision &revision);
size_t _OwnedBytes(const RigExecBakedProgramImpl::WeightObject &object);
size_t _OwnedBytes(const RigExecBakedProgramImpl::WeightField &field);

template <> struct _OwnsStorage<std::string> : std::true_type {};
template <> struct _OwnsStorage<RigExecCompiledGraph> : std::true_type {};
template <> struct _OwnsStorage<RigExecCompiledOp> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpCluster> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpValueState> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpAdapterState> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpExclusionProof> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedLeafPools> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedPathLeaves> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedSourceWatch> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedCones> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedClusterSet> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::ConstraintArrays>
    : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::Constraint> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::Solver> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::GeomChain> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::GeomRevision> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::WeightObject> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::WeightField> : std::true_type {};

size_t
_OwnedBytes(const std::string &text)
{
    return text.capacity() >= kStringInline ? text.capacity() + 1 : 0;
}

template <class T, class Alloc>
size_t
_OwnedBytes(const std::vector<T, Alloc> &values)
{
    size_t bytes = values.capacity() * sizeof(T);
    if constexpr (_OwnsStorage<T>::value) {
        for (const T &value : values) {
            bytes += _OwnedBytes(value);
        }
    }
    return bytes;
}

template <class Key, class Value, class Compare, class Alloc>
size_t
_OwnedBytes(const std::map<Key, Value, Compare, Alloc> &entries)
{
    size_t bytes =
        entries.size() * (sizeof(std::pair<const Key, Value>) + kTreeLinks);
    if constexpr (_OwnsStorage<Value>::value) {
        for (const auto &entry : entries) {
            bytes += _OwnedBytes(entry.second);
        }
    }
    return bytes;
}

template <class Key, class Compare, class Alloc>
size_t
_OwnedBytes(const std::set<Key, Compare, Alloc> &keys)
{
    return keys.size() * (sizeof(Key) + kTreeLinks);
}

template <class Key, class Value, class Hash, class Equal, class Alloc>
size_t
_OwnedBytes(const std::unordered_map<Key, Value, Hash, Equal, Alloc> &entries)
{
    size_t bytes =
        entries.bucket_count() * sizeof(void *) +
        entries.size() * (sizeof(std::pair<const Key, Value>) + kHashLinks);
    if constexpr (_OwnsStorage<Value>::value) {
        for (const auto &entry : entries) {
            bytes += _OwnedBytes(entry.second);
        }
    }
    return bytes;
}

template <class T>
size_t
_OwnedBytes(const std::optional<T> &held)
{
    return held ? _OwnedBytes(*held) : 0;
}

size_t
_OwnedBytes(const RigExecBakedClusterSet &set)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(set.words);
    return total;
}

size_t
_OwnedBytes(const RigExecCompiledOp &op)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(op.predecessors);
    add(op.successors);
    return total;
}

size_t
_OwnedBytes(const RigExecOpCluster &cluster)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(cluster.members);
    add(cluster.predecessors);
    add(cluster.successors);
    return total;
}

size_t
_OwnedBytes(const RigExecOpValueState &value)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(value.key);
    add(value.scratch);
    return total;
}

size_t
_OwnedBytes(const RigExecCompiledGraph &graph)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(graph.ops);
    add(graph.canonicalIndex);
    add(graph.readers);
    add(graph.cycles);
    add(graph.cycleMembers);
    add(graph.clusters);
    add(graph.opClusters);
    add(graph.volatileOps);
    return total;
}

size_t
_OwnedBytes(const RigExecOpAdapterState &state)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(state.values);
    add(state.leaves);
    add(state.changedLeaves);
    add(state.changedValues);
    add(state.excludedValues);
    add(state.skipEffects);
    add(state.seeds);
    add(state.candidateOps);
    add(state.inputRevisions);
    add(state.inputKeys);
    add(state.inputScratch);
    add(state.sourceKeys);
    add(state.sourceScratch);
    add(state.coveredPropertyInputs);
    add(state.coveredTypedInputs);
    add(state.inputExact);
    add(state.retainedFirst);
    add(state.constantSource);
    add(state.sourceVisits);
    add(state.contentSourceKeys);
    add(state.contentInputKeys);
    add(state.leafVersionMismatch);
    return total;
}

size_t
_OwnedBytes(const RigExecOpExclusionProof &proof)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(proof.keys);
    add(proof.removedKeys);
    add(proof.cycles);
    add(proof.cycleMembers);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedLeafPools &pools)
{
    size_t total = 0;
    pools.ForEach([&total](const auto &pool) {
        const auto add = [&total](const auto &member) {
            total += _OwnedBytes(member);
        };
        add(pool.value);
        add(pool.changed);
        add(pool.mustSample);
        add(pool.id);
        add(pool.changedList);
        add(pool.markedList);
        add(pool.editSerial);
    });
    return total;
}

size_t
_OwnedBytes(const RigExecBakedPathLeaves &leaves)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(leaves.attributes);
    add(leaves.hops);
    add(leaves.varying);
    add(leaves.compiledRig);
    add(leaves.overrideReached);
    add(leaves.walks);
    add(leaves.values);
    add(leaves.changed);
    add(leaves.versions);
    add(leaves.observed);
    add(leaves.observedVersions);
    add(leaves.observedRuns);
    add(leaves.mustSample);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedSourceWatch &watch)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(watch.values);
    add(watch.words);
    add(watch.exact);
    add(watch.dirty);
    add(watch.overrideSlotsHeld);
    add(watch.dirtyList);
    add(watch.standing);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedCones &cones)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(cones.avarCluster);
    add(cones.chainBaseClusters);
    add(cones.solverPointsClusters);
    add(cones.revisionClusters);
    add(cones.revisionStaticCluster);
    add(cones.nativeSourceClusters);
    add(cones.deltaBaseClusters);
    add(cones.constraintArrayClusters);
    add(cones.varyingSteps);
    add(cones.overrideSteps);
    add(cones.headReaders);
    add(cones.walkReaders);
    add(cones.fieldReaders);
    add(cones.fieldSteps);
    add(cones.restReaders);
    add(cones.ladderReaders);
    add(cones.editRoute);
    add(cones.avarStep);
    add(cones.avarVersionSteps);
    add(cones.chainBaseSteps);
    add(cones.solverPointsSteps);
    add(cones.revisionSteps);
    add(cones.revisionStaticStep);
    add(cones.nativeSourceSteps);
    add(cones.deltaBaseSteps);
    add(cones.constraintArraySteps);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::ConstraintArrays &arrays)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(arrays.pathText);
    add(arrays.weights);
    add(arrays.translationOffsets);
    add(arrays.rotationOffsets);
    add(arrays.diagnostics);
    add(arrays.lastWeights);
    add(arrays.lastTranslationOffsets);
    add(arrays.lastRotationOffsets);
    add(arrays.lastDiagnostics);
    add(arrays.poleWeights);
    add(arrays.lastPoleWeights);
    add(arrays.poleDiagnostics);
    add(arrays.lastPoleDiagnostics);
    add(arrays.attributes);
    add(arrays.keys);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::Constraint &constraint)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(constraint.pathText);
    add(constraint.sourcePathTexts);
    add(constraint.deltaBasePathText);
    add(constraint.weightScratch);
    add(constraint.weightError);
    add(constraint.targetSlots);
    add(constraint.snapshotTargets);
    add(constraint.sources);
    add(constraint.sourceNatives);
    add(constraint.sourcePaths);
    add(constraint.ikRestLive);
    add(constraint.poleObjects);
    add(constraint.poleObjectNatives);
    add(constraint.matchScalars);
    add(constraint.matchPoleSourceRides);
    add(constraint.matchShown);
    add(constraint.matchRiders);
    add(constraint.harmonicsScalars);
    add(constraint.harmonicsPositions);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::Solver &solver)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(solver.pathText);
    add(solver.relationshipRequirements);
    add(solver.solveDescriptorKey);
    add(solver.restSlots);
    add(solver.restRefs);
    add(solver.restReads);
    add(solver.restOverrides);
    add(solver.controls);
    add(solver.bone2dDoubles);
    add(solver.bone2dBools);
    add(solver.bone2dSelectors);
    add(solver.jointGroupSelectors);
    add(solver.ribbonPoints);
    add(solver.lastRibbonPoints);
    add(solver.ribbonConstantPoints);
    add(solver.outputs);
    add(solver.outputSnapshots);
    add(solver.outFrames);
    add(solver.outPresent);
    add(solver.outPosition);
    add(solver.fallbackSlots);
    add(solver.controlReads);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomChain &chain)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(chain.revisions);
    add(chain.groupBounds);
    add(chain.baseGroups);
    add(chain.resultIds);
    add(chain.spareIds);
    add(chain.derived);
    add(chain.gpuLastFrame);
    add(chain.gpuScratch);
    add(chain.gpuDerivedSteps);
    add(chain.windowDeviceDerivedSteps);
    add(chain.gpuKeyOps);
    add(chain.gpuExtentSample);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(revision.moverPathText);
    add(revision.blendChannels);
    add(revision.blendSamples);
    add(revision.influenceSlots);
    add(revision.output);
    add(revision.stagingOutput);
    add(revision.gpuChannels);
    add(revision.pointBindings);
    add(revision.transformRecords);
    add(revision.influenceRecords);
    add(revision.packetInfluences);
    add(revision.influences);
    add(revision.chunks);
    add(revision.partitionProducerSets);
    add(revision.envelope);
    add(revision.resolveScratch);
    add(revision.rows);
    add(revision.palette);
    add(revision.passedPoints);
    add(revision.layoutOverlay);
    add(revision.windowCache);
    add(revision.deviceTicks);
    add(revision.joinSeen);
    add(revision.groupWritten);
    add(revision.enteringWriter);
    add(revision.haloGroups);
    add(revision.haloPoints);
    add(revision.mushGroups);
    add(revision.groups);
    add(revision.groupIds);
    add(revision.wholeEntering);
    add(revision.pins);
    add(revision.deltasSpare);
    add(revision.layoutLeaves);
    add(revision.leaves);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::WeightObject &object)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(object.oracleFrozenKeys);
    add(object.oracleName);
    add(object.oracleAxisName);
    add(object.oracleBoundsName);
    add(object.values);
    add(object.indices);
    add(object.inputs);
    add(object.combineTargetPoints);
    add(object.targetPoints);
    add(object.samplePoints);
    add(object.curvePoints);
    add(object.falloffCurve);
    add(object.packetPointViews);
    add(object.packetPointScratch);
    add(object.packetLocalCurve);
    add(object.packetInputRefs);
    add(object.oracleLeaves);
    add(object.pointLeaves);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::WeightField &field)
{
    size_t total = 0;
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    add(field.volumes);
    add(field.availableChains);
    add(field.scalarReads);
    add(field.scalarObjects);
    add(field.scalarMembers);
    add(field.pointReads);
    add(field.values);
    add(field.enteringGather);
    add(field.error);
    return total;
}

size_t
_ProgramStorageBytes(const RigExecBakedProgramImpl &src)
{
    size_t total = sizeof(RigExecBakedProgramImpl);
    const auto add = [&total](const auto &member) {
        total += _OwnedBytes(member);
    };
    // Every field the program clone copies (_CloneImpl above, in its order):
    // a container contributes its elements, anything else contributes
    // nothing. A new compiled-rig field belongs in both lists.
    add(src.assetRootPath);
    add(src.ownedJointSolverBinding);
    add(src.cycleExclusionProof);
    add(src.cycleSkipReasons);
    add(src.hasPropertyChains);
    add(src.programAffinity);
    add(src.paths);
    add(src.pathTexts);
    add(src.index);
    add(src.slotKind);
    add(src.providerActive);
    add(src.parent);
    add(src.propParent);
    add(src.xformSlots);
    add(src.xformPrimsBySlot);
    add(src.assetRoot);
    add(src.xformBase);
    add(src.lastXformBase);
    add(src.constraintArrays);
    add(src.nativeSources);
    add(src.nativeFrames);
    add(src.lastNativeFrames);
    add(src.nativeFrameOk);
    add(src.lastNativeFrameOk);
    add(src.ladders);
    add(src.interveningSlots);
    add(src.interveningAnchors);
    add(src.ladderVarying);
    add(src.ladderOverrides);
    add(src.restChainVaries);
    add(src.posedAuthored);
    add(src.posedAuthoredM);
    add(src.restM);
    add(src.restPts);
    add(src.restFrames);
    add(src.selfD);
    add(src.posedD);
    add(src.parentSpaceM);
    add(src.parentSpaceAuthored);
    add(src.lastPosedD);
    add(src.lastParentSpaceM);
    add(src.lastParentSpaceAuthored);
    add(src.lastRotationSign);
    add(src.parentDinv);
    add(src.rotOrder);
    add(src.restRoundTrip);
    add(src.defaultRoundTrip);
    add(src.lastRestM);
    add(src.lastSelfD);
    add(src.lastParentDinv);
    add(src.lastPosedAuthoredM);
    add(src.lastPosedAuthored);
    add(src.lastRotOrder);
    add(src.restChanged);
    add(src.ladderChanged);
    add(src.restMoved);
    add(src.ladderMoved);
    add(src.xyzToken);
    add(src.noScaleAvars);
    add(src.rotationSign);
    add(src.poseInterpolators);
    add(src.poseWeightPaths);
    add(src.poseWeights);
    add(src.poseWeightIndex);
    add(src.avarConstants);
    add(src.avarBindings);
    add(src.avarConstantBindings);
    add(src.avarBindingBegin);
    add(src.avarConstantBindingBegin);
    add(src.patchableAvars);
    add(src.promotedAvars);
    add(src.boundInputs);
    add(src.varyingInputs);
    add(src.avars);
    add(src.posedM);
    add(src.base);
    add(src.fin);
    add(src.finLast);
    add(src.baseLast);
    add(src.propertyResults);
    add(src.frameRecords);
    add(src.frameMatrix);
    add(src.frameMatrixValid);
    add(src.finalMatrix);
    add(src.baseMatrix);
    add(src.needFinal);
    add(src.needBase);
    add(src.solvers);
    add(src.guideSolvers);
    add(src.solverIndex);
    add(src.aggregates);
    add(src.constraints);
    add(src.windowedOps);
    add(src.historyMask);
    add(src.windowCaptures);
    add(src.windowRings);
    add(src.windowRingChanged);
    add(src.windowRingSerial);
    add(src.windowOfOp);
    add(src.walkSteps);
    add(src.steps);
    add(src.epilogue);
    add(src.excludedSteps);
    add(src.opGraph);
    add(src.opAdapter);
    add(src.verifyChainVersions);
    add(src.chainContentKeys);
    add(src.verifyPacketVersions);
    add(src.packetContentKeys);
    add(src.verifyRangeChains);
    add(src.sourceWatch);
    add(src.verifySourceKeys);
    add(src.composeGroups);
    add(src.spaceSwitches);
    add(src.autoClavicles);
    add(src.autoClavicleBySlot);
    add(src.switchFrameContexts);
    add(src.switchFrames);
    add(src.spaceSwitchBySlot);
    add(src.commits);
    add(src.revisionIndex);
    add(src.derivedIndex);
    add(src.chainRevisionBegin);
    add(src.chainRevisionEnd);
    add(src.revisionChunkBase);
    add(src.revisionChunkCount);
    add(src.chainChunkBegin);
    add(src.chainChunkEnd);
    add(src.revisionFuseStep);
    add(src.chunkRevision);
    add(src.skinTopologyLayouts);
    add(src.clustering);
    add(src.chunkVertexTarget);
    add(src.gpuDeformStats);
    add(src.scheduleParallel);
    add(src.scheduleReport);
    add(src.scheduleCalibrate);
    add(src.scheduleStepTiming);
    add(src.traceAllSteps);
    add(src.calibrationFrames);
    add(src.stepTimingFrames);
    add(src.chunkCap);
    add(src.rangeChains);
    add(src.groupVertexTarget);
    add(src.groupCap);
    add(src.groupGates);
    add(src.roleMode);
    add(src.exportKeep);
    add(src.exportPinnedPaths);
    add(src.useSimd);
    add(src.purityAudit);
    add(src.cones);
    add(src.closedSteps);
    add(src.closed);
    add(src.lastAvars);
    add(src.lastOverridden);
    add(src.lastHaveBase);
    add(src.lastTime);
    add(src.everRan);
    add(src.programStamp);
    add(src.lastProgramStamp);
    add(src.edited);
    add(src.anyEdited);
    add(src.valueEditSerial);
    add(src.editSerial);
    add(src.lastClosedClusters);
    add(src.lastClosedSteps);
    add(src.jointSlots);
    add(src.jointPaths);
    add(src.controlSlots);
    add(src.controlPaths);
    add(src.solverArrays);
    add(src.jointPublishOrder);
    add(src.controlPublishOrder);
    add(src.solverPublishOrder);
    add(src.jointPathsAscending);
    add(src.controlPathsAscending);
    add(src.solverArraysAscending);
    add(src.measureColdRuns);
    add(src.stampedSteps);
    add(src.timedClusters);
    add(src.jointMatrixPublished);
    add(src.chains);
    add(src.gpuDeformMode);
    add(src.deltaValues);
    add(src.deltaPresent);
    add(src.deltaBasePaths);
    add(src.deltaBaseMatrix);
    add(src.lastDeltaBaseMatrix);
    add(src.deltaBaseOk);
    add(src.lastDeltaBaseOk);
    add(src.weightObjects);
    add(src.weightProgram);
    add(src.weightCycleBlocked);
    add(src.weightFields);
    add(src.volumePlacementBase);
    add(src.weightIndex);
    add(src.volumePlacement);
    add(src.placedVolumes);
    add(src.currentPhaseWeights);
    add(src.falloffLuts);
    add(src.weightPackets);
    add(src.rebuild);
    add(src.sourceBackedPaths);
    add(src.named);
    add(src.prims);
    add(src.xformPrims);
    add(src.overridden);
    add(src.overridableInputs);
    add(src.leaves);
    add(src.leafRefs);
    add(src.leafOfOverride);
    add(src.leafByPath);
    add(src.routedOverrides);
    add(src.lastRoutedOverrides);
    add(src.upstreamAdmissible);
    add(src.upstreamOracle);
    add(src.upstreamSetsBuilt);
    add(src.upstream);
    add(src.lastUpstream);
    add(src.upstreamOn);
    add(src.upstreamChanged);
    add(src.upstreamMovedThisRun);
    add(src.leafSamples);
    add(src.pathLeafRefs);
    add(src.pathLeafChainResults);
    add(src.pathLeafChainSerial);
    add(src.pathLeafSamples);
    add(src.pathLeafRun);
    add(src.pathLeafGating);
    add(src.verifyPathLeafGating);
    add(src.leafFlagCompiledRig);
    add(src.avarConstantSerial);
    add(src.sparseSampling);
    add(src.verifySparseSampling);
    add(src.resolvedRoutedPrims);
    add(src.avarsDisturbed);
    add(src.folded);
    add(src.anyOverridden);
    add(src.publishWeightFields);
    add(src.propertyChains);
    add(src.oraclePublications);
    add(src.oracleWeightInputs);
    add(src.propertyRecords);
    add(src.propertyRecordById);
    add(src.poseDependencyPaths);
    add(src.compiledProviderProgram);
    add(src.providerProgram);
    add(src.providerValues);
    add(src.providerLeaves);
    add(src.providerLeafBlocked);
    add(src.providerLeafEditSerial);
    add(src.spaceLeafIndex);
    add(src.spaceLeafRekey);
    add(src.spaceLeafKeys);
    add(src.providerLeafValues);
    add(src.providerLeafChains);
    add(src.providerRoutedReads);
    add(src.providerFrozenKeys);
    add(src.providerExternalSlots);
    add(src.providerFrameInputs);
    add(src.poseProviderInputs);
    add(src.connectedPoseProviders);
    add(src.providerParentRawLeaves);
    add(src.providerPruneViolations);
    add(src.providerStepsPruned);
    add(src.providerPruneRoots);
    add(src.providerRefreshes);
    add(src.providerRefreshBefore);
    add(src.propertyVersionCount);
    add(src.headLeaves);
    add(src.headOverrideSlots);
    add(src.headOverrideSlotsByName);
    add(src.propertyValues);
    add(src.propertyVersionValid);
    add(src.propertyChanged);
    add(src.chainValid);
    add(src.chainFinal);
    add(src.recordValues);
    add(src.recordStoodAside);
    add(src.headOverrides);
    add(src.lastHeadOverrides);
    add(src.headOverrideMoved);
    add(src.headLeavesSampled);
    add(src.headLeafTime);
    add(src.headLeafStamp);
    add(src.headEverRan);
    add(src.headStamp);
    add(src.headLeafSamples);
    add(src.headOpsRun);
    add(src.readerWalks);
    add(src.crossDomainReads);
    add(src.crossDomainErrors);
    add(src.crossDomainOrdinals);
    add(src.readerWalkMoved);
    add(src.readerWalkChanged);
    add(src.avarHeadReads);
    add(src.verifyCloneKeys);
    return total;
}

} // namespace

void
_SettleCloneVerdict(RigExecBakedProgramImpl *program)
{
    _AdoptCloneVerdict(program, _CloneKeysStand(*program, *program));
}

} // namespace frozenDetail

bool
RigExecCanFreezeProgram(const RigExecRigEvaluator &evaluator,
                        std::string *error)
{
    if (evaluator.cpuReference && !RigExecOracleAvailable(error))
        return false;
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return fail("no compiled program is available to freeze");
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    // Property chains and weight envelopes run as declared graph operations.
    // Source discovery must match the compiled rig and every envelope
    // must name its compiled field producer.
    {
        RigExecChainSampleBindings bound;
        std::string bindError;
        if (!RigExecBindChainSampleInputs(evaluator, &bound, &bindError)) {
            return fail("cannot name the property chains: " + bindError);
        }
        if (bound.chains.empty() == B.hasPropertyChains) {
            return fail(
                "chain discovery disagrees with the program about whether "
                "chains exist");
        }
        for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
            for (const RigExecBakedPropertyChain::Revision &revision :
                 chain.revisions) {
                if (!revision.weightObject.IsEmpty() &&
                    revision.weightField < 0) {
                    return fail("diag " + revision.mover.GetString() +
                                " has no compiled weight field for its "
                                "property envelope");
                }
            }
        }
    }
    // No gate on xform slots, native sources, delta bases, or
    // geometry-domain constraints: the seeds sample per frame through the
    // program's hook, and the constraint steps are the shared bodies, so a
    // frozen run reproduces them from the transported seeds.
    // Pose constraint envelopes require compiled field bindings. Geometry
    // constraints consume the revision's per-point packet instead.
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty() && constraint.weightField < 0) {
            return fail("constraint " + constraint.path.GetString() +
                        " has no compiled weight field for its envelope");
        }
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin &&
                revision.op != RigExecRevisionOp::RecomputeNormals &&
                revision.op != RigExecRevisionOp::RecomputeExtent &&
                revision.op != RigExecRevisionOp::Matrix &&
                revision.op != RigExecRevisionOp::Wire &&
                revision.op != RigExecRevisionOp::BlendShape &&
                revision.op != RigExecRevisionOp::VolumeCorrect &&
                revision.op != RigExecRevisionOp::Smooth &&
                revision.op != RigExecRevisionOp::DeltaMush &&
                revision.op != RigExecRevisionOp::Wrinkle &&
                revision.op != RigExecRevisionOp::Lattice &&
                revision.op != RigExecRevisionOp::SurfaceProject &&
                revision.op != RigExecRevisionOp::Ribbon &&
                revision.op != RigExecRevisionOp::EmitGuidePoints &&
                revision.op != RigExecRevisionOp::External) {
                return fail("revision " + revision.moverPath.GetString() +
                            " runs op '" +
                            RigExecRevisionKindToken(revision.op).GetString() +
                            "', which the frozen executor does not "
                            "implement");
            }
            if (revision.op == RigExecRevisionOp::External &&
                (!revision.binding.handler || !revision.leaves.decl.assembles))
                return fail("external revision " +
                            revision.moverPath.GetString() +
                            " has no compiled stage-free handler declaration");
            // Phase reads consume the worker's current typed point/frame
            // versions. Revision assembly uses captured source facts.
            // Dense blend samples use those bindings; sparse blend-shape
            // offsets are authored layout data, without point-chain phases.
            if (revision.weightCurrentPhase && revision.weightField < 0) {
                return fail(
                    "revision " + revision.moverPath.GetString() +
                    " has no compiled field for its current-phase weight");
            }
            // Driver frames and the geometry-delta hand-off use the
            // worker's own Solve-step aggregate, read in program order,
            // and the delta through the shared constraint step and
            // the shared fold -- the seeds patch the delta bases the
            // constraint step measures against, the step stashes the delta,
            // and FoldInfluences reads the stash, all shared bodies in
            // program order.
            // Skin packet walks resolve the worker's current property
            // versions through the same declared reads as live assembly.
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (derived.revision.op != RigExecRevisionOp::RecomputeNormals &&
                derived.revision.op != RigExecRevisionOp::RecomputeExtent &&
                !RigExecIsDerivedMatrixOp(derived.revision.op)) {
                return fail("derived target " + derived.target.GetString() +
                            " runs an op the frozen executor does not "
                            "implement");
            }
            // Derived phases are copied typed point/frame bindings; their
            // shared body resolves them after the declared producers join.
        }
    }

    return true;
}

bool
RigExecFreezeProgram(const RigExecRigEvaluator &evaluator,
                     std::shared_ptr<const RigExecFrozenProgram> *frozen,
                     std::string *error)
{
    RigExecInputReplayComparisonScope replayComparison("RigExecFreezeProgram");
    if (!frozen) {
        if (error) {
            *error = "no snapshot to freeze into";
        }
        return false;
    }
    if (!RigExecCanFreezeProgram(evaluator, error)) {
        return false;
    }
    const RigExecBakedProgramImpl &B =
        evaluator.GetBakedProgram()->GetStepGraph();
    auto snapshot = std::make_shared<RigExecFrozenProgram>();
    auto program = std::make_shared<RigExecBakedProgramImpl>();
    auto bindings = std::make_shared<RigExecFrozenBindings>();
    snapshot->program = program;
    snapshot->bindings = bindings;
    snapshot->inputReplayCapture = RigExecInputReplayCaptureRequested();
    snapshot->inputReplayOwner = std::this_thread::get_id();
    // Settled below, after the snapshot's own edits; its lanes inherit it.
    _CloneImpl(B, program.get(), _CloneVerdict::Defer);
    program->verifyCloneKeys = TfGetenvBool("RIGEXEC_VERIFY_CLONE_KEYS", false);
    // Workers clone this snapshot and index its spelled slot paths unchecked.
    if (!TF_VERIFY(program->pathTexts &&
                   program->pathTexts->size() == program->paths.size())) {
        if (error) {
            *error = "slot path texts do not match the slot table";
        }
        return false;
    }
    if (evaluator.cpuReference) {
        // Freeze is an owning-thread source boundary. Enabling the judge
        // after Evaluate must capture its independent inputs here, rather
        // than require another production generation or retain old facts.
        auto &reference = *program;
        struct SourceCaptureScope {
            RigExecBakedProgramImpl &program;
            ~SourceCaptureScope()
            {
                program.stage = UsdStageRefPtr();
                program.resolvedInputs = nullptr;
            }
        } sourceScope{reference};
        reference.stage = B.stage;
        reference.resolvedInputs = B.resolvedInputs;
        reference.oraclePublications.emplace();
        RigExecBakedBeginOracleReference(&reference, 0, B.lastTime);
    }
    _SettleCloneVerdict(program.get());
    if (B.jointSolverBinding) {
        bindings->jointSolverBinding = *B.jointSolverBinding;
    }
    bindings->solverGuidesPresent = !B.solverArrays.empty();
    // Handle identities, captured here (UI thread) as plain data: the
    // worker must not even ask a handle whether it is valid.
    _ForEachPatchableInput(
        B,
        [&bindings](const auto &input) {
            bindings->inputHeadPaths.push_back(input.head ? input.head.GetPath()
                                                          : SdfPath());
            bindings->inputConstants.push_back(VtValue(
                input.sourceBacked ? input.sourceFallback : input.constant));
        },
        /*includeIntervening=*/false);
    // The samplers' own keys, empty where they read nothing: one source of
    // keys for the sampler, the worker and the snapshot.
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        const bool valid = arrays.prim.IsValid();
        for (size_t k = 0; k < arrays.keys.size(); ++k) {
            bindings->arrayKeys.push_back(
                valid && arrays.Sampled(k) ? arrays.keys[k] : SdfPath());
        }
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        bindings->chainBaseQueryValid.push_back(chain.baseQuery.IsValid() ? 1
                                                                          : 0);
    }
    for (const auto &[chainIndex, derivedIndex] : B.derivedIndex) {
        const RigExecBakedProgramImpl::GeomChain::Derived &derived =
            B.chains[size_t(chainIndex)].derived[size_t(derivedIndex)];
        bindings->derivedBaseQueryValid.push_back(
            derived.baseQuery.IsValid() ? 1 : 0);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        bindings->ribbonQueryValid.push_back(
            solver.ribbonPointsQuery.IsValid() ? 1 : 0);
    }
    const auto &T = *_tokens;
    for (const auto &[chainIndex, revisionIndex] : B.revisionIndex) {
        const RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
        const UsdPrim &moverPrim = revision.moverPrim;
        bindings->moverHasEnabled.push_back(
            moverPrim && moverPrim.GetAttribute(T.inputsEnabled) ? 1 : 0);
        bindings->moverHasDefaultWeight.push_back(
            moverPrim && moverPrim.GetAttribute(T.inputsDefaultWeight) ? 1 : 0);
        bindings->moverHasMethod.push_back(
            moverPrim && moverPrim.GetAttribute(T.rigExecSkinningMethod) ? 1
                                                                         : 0);
        // The sampler's key for the mover's defaultWeight sample.
        bindings->moverDefaultWeightKeys.push_back(
            revision.moverPath.AppendProperty(T.inputsDefaultWeight));
    }
    // The sampler's synthetic weight-array keys, by the same function.
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        for (const TfToken &role :
             {_frozenWeightTokens->targetPointsKey,
              _frozenWeightTokens->samplePointsKey,
              _frozenWeightTokens->curvePointsKey,
              _frozenWeightTokens->combineTargetCountKey}) {
            bindings->weightArrayKeys.push_back(
                _FrozenWeightArrayKey(object.path, role));
        }
    }
    *frozen = std::move(snapshot);
    return true;
}

bool
RigExecFrozenSnapshotOwesLiveEdits(const RigExecFrozenProgram &base,
                                   const RigExecBakedProgram &live)
{
    if (!base.program)
        return true;
    const RigExecBakedProgramImpl &L = live.GetStepGraph();
    const auto *patch = base.avarPatch.get();
    const uint64_t serial =
        patch ? patch->valueEditSerial : base.program->valueEditSerial;
    const uint64_t stamp =
        patch ? patch->programStamp : base.program->programStamp;
    return L.valueEditSerial != serial || L.programStamp != stamp;
}

uint64_t
RigExecFrozenAvarRegionDigest(const RigExecBakedProgram &program)
{
    // Exactly the fields RigExecProgramAvarPatch writes, plus the table it
    // keeps beside them: per-binding constants and varying flags, the
    // promoted set, the varying count, and the constant table. Working
    // state (avars, lastAvars) is history, not region, and is excluded --
    // it converges on the next run whatever the snapshot holds.
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    uint64_t h = 1469598103934665603ULL;
    h = _HashBytes(h, &B.varyingInputs, sizeof(B.varyingInputs));
    for (double constant : B.avarConstants) {
        h = _HashBytes(h, &constant, sizeof(constant));
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        h = _HashBytes(h, &binding.input.constant,
                       sizeof(binding.input.constant));
        const unsigned char varying = binding.input.varying ? 1 : 0;
        h = _HashBytes(h, &varying, sizeof(varying));
    }
    for (size_t promoted : B.promotedAvars) {
        h = _HashBytes(h, &promoted, sizeof(promoted));
    }
    return h;
}

// The owned storage one frozen execution lane clones from `live`, the bound a
// host charges the coverage budget for a frame before its arena exists. See
// docs/specs/output-subscriptions.md, "Retained bytes and backpressure".
size_t
RigExecEstimateFrozenArenaBytes(const RigExecBakedProgram &live)
{
    return frozenDetail::_ProgramStorageBytes(live.GetStepGraph());
}

} // namespace rigExec
