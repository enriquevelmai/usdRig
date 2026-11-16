// Stage-side program snapshot construction and constant patching.

#include "rigExecSchema/schemaNames.h"
#include "frozenContextInternal.h"
#include "inputReplay.h"
#include "oracleDispatch.h"
#include "bakedOpValues.h"
#include "pxr/base/tf/getenv.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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
// at it. Containers are counted by element and walked to any depth, so an
// inner container with the outer length fixed is still charged; the idle
// scratch the clone lets go of is counted too, because the bound is taken
// before anything is dropped. A class that hides its storage behind private
// members -- a GPU deform translation's key table, an oracle publication
// context's records -- reports its own retained bytes to the walk.

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

size_t _OwnedBytes(const RigExecBakedProgramImpl::NativeXformSource &record);
size_t _OwnedBytes(const RigExecProviderProgram &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::ProviderRefresh &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::PoseInterpolator &record);
size_t _OwnedBytes(const RigExecPointFrameArray &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::WindowedOp &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::WalkStep &record);
size_t _OwnedBytes(const RigExecBakedStep &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::SpaceCheckpoint &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::AutoClavicle &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::SpaceSwitch &record);
size_t _OwnedBytes(const RigExecBakedComposeGroup &record);
size_t _OwnedBytes(const RigExecBakedCommit &record);
size_t _OwnedBytes(const RigExecBakedClustering &record);
size_t _OwnedBytes(const RigExecWeightRecord &record);
size_t _OwnedBytes(const RigExecOpWorkspace &record);
size_t _OwnedBytes(const RigExecOpExecution &record);
size_t _OwnedBytes(const RigExecWeightPacket &record);
size_t _OwnedBytes(const RigExecBakedSparseInputs &record);
size_t _OwnedBytes(const RigExecCrossDomainRead &record);
size_t _OwnedBytes(const RigExecBakedPropertyChain &record);
size_t _OwnedBytes(const RigExecBakedPropertyRecord &record);
size_t _OwnedBytes(const RigExecWeightReferenceContext &record);
size_t _OwnedBytes(const RigExecGpuDeformTranslation &state);
size_t _OwnedBytes(const RigExecOraclePublicationContext &context);
size_t _OwnedBytes(const RigExecOracleScene &scene);
size_t _OwnedBytes(const RigExecOraclePrim &prim);
size_t _OwnedBytes(const RigExecOracleAttribute &attribute);
size_t _OwnedBytes(const RigExecOracleRelationship &relationship);
size_t _OwnedBytes(const RigExecBakedReaderWalk &record);
size_t _OwnedBytes(const RigExecProviderExternalInput &record);
size_t _OwnedBytes(const RigExecProviderOp &record);
size_t _OwnedBytes(const RigExecOpDescriptor &record);
size_t _OwnedBytes(const RigExecRevisionLeafDecl &record);
size_t
_OwnedBytes(const RigExecBakedProgramImpl::ProviderRefresh::Carry &record);
size_t _OwnedBytes(const RigExecSolverInputs &record);
size_t _OwnedBytes(const RigExecSolverWorkspace &record);
size_t _OwnedBytes(const RigExecBakedPointsBinding &record);
size_t _OwnedBytes(const RigExecTwoBoneIkParams &record);
size_t _OwnedBytes(const RigExecSplineIkRest &record);
size_t _OwnedBytes(const RigExecSolverRecord &record);
size_t _OwnedBytes(const RigExecConstraintRecord &record);
size_t _OwnedBytes(const RigExecConstraintInputs &record);
size_t _OwnedBytes(const RigExecConstraintResult &record);
size_t _OwnedBytes(const RigExecBoundAutoClavicle &record);
size_t _OwnedBytes(const RigExecTypedValueStore &record);
size_t
_OwnedBytes(const RigExecBakedProgramImpl::AutoClavicle::FrameRead &record);
size_t
_OwnedBytes(const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion &record);
size_t _OwnedBytes(const RigExecConstraintSource &record);
size_t _OwnedBytes(const RigExecBakedCluster &record);
template <class Point>
size_t _OwnedBytes(const RigExecGroupState<Point> &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::GeomChain::Derived &record);
size_t _OwnedBytes(const RigExecGpuDeformFrame &record);
size_t _OwnedBytes(const RigExecWeightOracleFacts &record);
size_t _OwnedBytes(const RigExecWeightPacketWorkspace &record);
size_t
_OwnedBytes(const RigExecBakedProgramImpl::WeightField::PointInput &record);
size_t _OwnedBytes(const RigExecWeightFieldWorkspace &record);
size_t _OwnedBytes(const RigExecBakedPropertyChain::Revision &record);
size_t _OwnedBytes(const RigExecWeightReferencePrim &record);
size_t _OwnedBytes(const RigExecBakedWalk &record);
size_t _OwnedBytes(const RigExecFkChainElement &record);
size_t _OwnedBytes(const RigExecSplineIkResult &record);
size_t _OwnedBytes(const RigExecSkeleton2dScratch &record);
size_t _OwnedBytes(const RigExecPositionConstraintParams &record);
size_t _OwnedBytes(const RigExecRotationConstraintParams &record);
size_t _OwnedBytes(const RigExecScaleConstraintParams &record);
size_t _OwnedBytes(const RigExecParentConstraintParams &record);
size_t _OwnedBytes(const RigExecAimConstraintParams &record);
size_t _OwnedBytes(const RigExecAutoClavicleConstants &record);
size_t _OwnedBytes(const RigExecGraphTypedRead &record);
size_t _OwnedBytes(const RigExecTypedValueState &record);
size_t _OwnedBytes(const RigExecGeometryRecord &record);
size_t _OwnedBytes(const RigExecSceneGeometryDescriptor &record);
size_t _OwnedBytes(const RigExecRevisionBinding &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::GeomBlendChannel &record);
size_t _OwnedBytes(const RigExecBlendChannel &record);
size_t _OwnedBytes(const RigExecMoverParameters &record);
size_t _OwnedBytes(const RigExecMoverStatus &record);
size_t _OwnedBytes(const RigExecBakedProgramImpl::GeomChunk &record);
size_t _OwnedBytes(const RigExecScaledDualQuat &record);
size_t _OwnedBytes(const RigExecRevisionRangeInputs &record);
size_t _OwnedBytes(const RigExecResolvedInputs &resolved);
size_t _OwnedBytes(const RigExecWireBasisCache &cache);
template <class Point, class Wide>
size_t _OwnedBytes(const RigExecDeltaMushGroupState<Point, Wide> &record);
template <class Point>
size_t _OwnedBytes(const RigExecSparseWalkMemo<Point> &record);
size_t _OwnedBytes(const RigExecWeightReferenceRelationship &record);
size_t _OwnedBytes(const RigExecSplineIkJoint &record);
size_t _OwnedBytes(const RigExecSkeleton2dWorkspace &record);
size_t _OwnedBytes(const RigExecSceneBoundInput &record);
size_t _OwnedBytes(const RigExecSceneTypedRead &record);
size_t _OwnedBytes(const RigExecSceneGeometryBlendChannel &record);
size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &record);
size_t _OwnedBytes(const RigExecBlendSampleData &record);
size_t _OwnedBytes(const RigExecDeltaMushSettings &record);
size_t _OwnedBytes(const RigExecWrinkleSettings &record);
size_t _OwnedBytes(const RigExecSurfaceSnapSettings &record);
size_t _OwnedBytes(const RigExecLatticeSettings &record);
size_t _OwnedBytes(const RigExecSceneGeometryBlendSample &record);

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

template <>
struct _OwnsStorage<RigExecBakedProgramImpl::NativeXformSource>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecProviderProgram> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::ProviderRefresh> : std::true_type {
};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::PoseInterpolator>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecPointFrameArray> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::WindowedOp> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::WalkStep> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedStep> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::SpaceCheckpoint> : std::true_type {
};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::AutoClavicle> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::SpaceSwitch> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedComposeGroup> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedCommit> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedClustering> : std::true_type {};
template <> struct _OwnsStorage<RigExecWeightRecord> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpWorkspace> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpExecution> : std::true_type {};
template <> struct _OwnsStorage<RigExecWeightPacket> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedSparseInputs> : std::true_type {};
template <> struct _OwnsStorage<RigExecCrossDomainRead> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedPropertyChain> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedPropertyRecord> : std::true_type {};
template <>
struct _OwnsStorage<RigExecWeightReferenceContext> : std::true_type {};
template <>
struct _OwnsStorage<RigExecGpuDeformTranslation> : std::true_type {};
template <>
struct _OwnsStorage<RigExecOraclePublicationContext> : std::true_type {};
template <> struct _OwnsStorage<RigExecOracleScene> : std::true_type {};
template <> struct _OwnsStorage<RigExecOraclePrim> : std::true_type {};
template <> struct _OwnsStorage<RigExecOracleAttribute> : std::true_type {};
template <> struct _OwnsStorage<RigExecOracleRelationship> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedReaderWalk> : std::true_type {};
template <>
struct _OwnsStorage<RigExecProviderExternalInput> : std::true_type {};
template <> struct _OwnsStorage<RigExecProviderOp> : std::true_type {};
template <> struct _OwnsStorage<RigExecOpDescriptor> : std::true_type {};
template <> struct _OwnsStorage<RigExecRevisionLeafDecl> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::ProviderRefresh::Carry>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecSolverInputs> : std::true_type {};
template <> struct _OwnsStorage<RigExecSolverWorkspace> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedPointsBinding> : std::true_type {};
template <> struct _OwnsStorage<RigExecTwoBoneIkParams> : std::true_type {};
template <> struct _OwnsStorage<RigExecSplineIkRest> : std::true_type {};
template <> struct _OwnsStorage<RigExecSolverRecord> : std::true_type {};
template <> struct _OwnsStorage<RigExecConstraintRecord> : std::true_type {};
template <> struct _OwnsStorage<RigExecConstraintInputs> : std::true_type {};
template <> struct _OwnsStorage<RigExecConstraintResult> : std::true_type {};
template <> struct _OwnsStorage<RigExecBoundAutoClavicle> : std::true_type {};
template <> struct _OwnsStorage<RigExecTypedValueStore> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::AutoClavicle::FrameRead>
    : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::SpaceSwitch::FrameVersion>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecConstraintSource> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedCluster> : std::true_type {};
template <class Point>
struct _OwnsStorage<RigExecGroupState<Point>> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::GeomChain::Derived>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecGpuDeformFrame> : std::true_type {};
template <> struct _OwnsStorage<RigExecWeightOracleFacts> : std::true_type {};
template <>
struct _OwnsStorage<RigExecWeightPacketWorkspace> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::WeightField::PointInput>
    : std::true_type {};
template <>
struct _OwnsStorage<RigExecWeightFieldWorkspace> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedPropertyChain::Revision> : std::true_type {};
template <> struct _OwnsStorage<RigExecWeightReferencePrim> : std::true_type {};
template <> struct _OwnsStorage<RigExecBakedWalk> : std::true_type {};
template <> struct _OwnsStorage<RigExecFkChainElement> : std::true_type {};
template <> struct _OwnsStorage<RigExecSplineIkResult> : std::true_type {};
template <> struct _OwnsStorage<RigExecSkeleton2dScratch> : std::true_type {};
template <>
struct _OwnsStorage<RigExecPositionConstraintParams> : std::true_type {};
template <>
struct _OwnsStorage<RigExecRotationConstraintParams> : std::true_type {};
template <>
struct _OwnsStorage<RigExecScaleConstraintParams> : std::true_type {};
template <>
struct _OwnsStorage<RigExecParentConstraintParams> : std::true_type {};
template <> struct _OwnsStorage<RigExecAimConstraintParams> : std::true_type {};
template <>
struct _OwnsStorage<RigExecAutoClavicleConstants> : std::true_type {};
template <> struct _OwnsStorage<RigExecGraphTypedRead> : std::true_type {};
template <> struct _OwnsStorage<RigExecTypedValueState> : std::true_type {};
template <> struct _OwnsStorage<RigExecGeometryRecord> : std::true_type {};
template <>
struct _OwnsStorage<RigExecSceneGeometryDescriptor> : std::true_type {};
template <> struct _OwnsStorage<RigExecRevisionBinding> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::GeomBlendChannel>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecBlendChannel> : std::true_type {};
template <> struct _OwnsStorage<RigExecMoverParameters> : std::true_type {};
template <> struct _OwnsStorage<RigExecMoverStatus> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::GeomChunk> : std::true_type {};
template <> struct _OwnsStorage<RigExecScaledDualQuat> : std::true_type {};
template <> struct _OwnsStorage<RigExecRevisionRangeInputs> : std::true_type {};
template <> struct _OwnsStorage<RigExecResolvedInputs> : std::true_type {};
template <> struct _OwnsStorage<RigExecWireBasisCache> : std::true_type {};
template <class Point, class Wide>
struct _OwnsStorage<RigExecDeltaMushGroupState<Point, Wide>> : std::true_type {
};
template <class Point>
struct _OwnsStorage<RigExecSparseWalkMemo<Point>> : std::true_type {};
template <>
struct _OwnsStorage<RigExecWeightReferenceRelationship> : std::true_type {};
template <> struct _OwnsStorage<RigExecSplineIkJoint> : std::true_type {};
template <> struct _OwnsStorage<RigExecSkeleton2dWorkspace> : std::true_type {};
template <> struct _OwnsStorage<RigExecSceneBoundInput> : std::true_type {};
template <> struct _OwnsStorage<RigExecSceneTypedRead> : std::true_type {};
template <>
struct _OwnsStorage<RigExecSceneGeometryBlendChannel> : std::true_type {};
template <>
struct _OwnsStorage<RigExecBakedProgramImpl::GeomBlendChannel::Sample>
    : std::true_type {};
template <> struct _OwnsStorage<RigExecBlendSampleData> : std::true_type {};
template <> struct _OwnsStorage<RigExecDeltaMushSettings> : std::true_type {};
template <> struct _OwnsStorage<RigExecWrinkleSettings> : std::true_type {};
template <> struct _OwnsStorage<RigExecSurfaceSnapSettings> : std::true_type {};
template <> struct _OwnsStorage<RigExecLatticeSettings> : std::true_type {};
template <>
struct _OwnsStorage<RigExecSceneGeometryBlendSample> : std::true_type {};

// Every owning standard container is both measured and walkable, so an
// element that is itself a container, a pair, or a record with containers is
// walked rather than just sized: growing an inner list while the outer length
// stays fixed grows the bound.
template <class T, class Alloc>
size_t _OwnedBytes(const std::vector<T, Alloc> &values);
template <class T, class Alloc>
size_t _OwnedBytes(const std::deque<T, Alloc> &values);
template <class Key, class Value, class Compare, class Alloc>
size_t _OwnedBytes(const std::map<Key, Value, Compare, Alloc> &entries);
template <class Key, class Compare, class Alloc>
size_t _OwnedBytes(const std::set<Key, Compare, Alloc> &keys);
template <class Key, class Value, class Hash, class Equal, class Alloc>
size_t
_OwnedBytes(const std::unordered_map<Key, Value, Hash, Equal, Alloc> &entries);
template <class Key, class Hash, class Equal, class Alloc>
size_t _OwnedBytes(const std::unordered_set<Key, Hash, Equal, Alloc> &keys);
template <class T, size_t N> size_t _OwnedBytes(const std::array<T, N> &values);
template <class A, class B> size_t _OwnedBytes(const std::pair<A, B> &entry);
template <class T> size_t _OwnedBytes(const std::optional<T> &held);

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
    if constexpr (_OwnsStorage<Key>::value) {
        for (const auto &entry : entries) {
            bytes += _OwnedBytes(entry.first);
        }
    }
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
    size_t bytes = keys.size() * (sizeof(Key) + kTreeLinks);
    if constexpr (_OwnsStorage<Key>::value) {
        for (const Key &key : keys) {
            bytes += _OwnedBytes(key);
        }
    }
    return bytes;
}

template <class Key, class Hash, class Equal, class Alloc>
size_t
_OwnedBytes(const std::unordered_set<Key, Hash, Equal, Alloc> &keys)
{
    size_t bytes = keys.bucket_count() * sizeof(void *) +
                   keys.size() * (sizeof(Key) + kHashLinks);
    if constexpr (_OwnsStorage<Key>::value) {
        for (const Key &key : keys) {
            bytes += _OwnedBytes(key);
        }
    }
    return bytes;
}

template <class T, class Alloc>
size_t
_OwnedBytes(const std::deque<T, Alloc> &values)
{
    size_t bytes = values.size() * sizeof(T);
    if constexpr (_OwnsStorage<T>::value) {
        for (const T &value : values) {
            bytes += _OwnedBytes(value);
        }
    }
    return bytes;
}

// A std::array has no allocation of its own; its elements can still own one.
template <class T, size_t N>
size_t
_OwnedBytes(const std::array<T, N> &values)
{
    if constexpr (_OwnsStorage<T>::value) {
        size_t bytes = 0;
        for (const T &value : values) {
            bytes += _OwnedBytes(value);
        }
        return bytes;
    }
    return 0;
}

template <class A, class B>
size_t
_OwnedBytes(const std::pair<A, B> &entry)
{
    return _OwnedBytes(entry.first) + _OwnedBytes(entry.second);
}

template <class Key, class Value, class Hash, class Equal, class Alloc>
size_t
_OwnedBytes(const std::unordered_map<Key, Value, Hash, Equal, Alloc> &entries)
{
    size_t bytes =
        entries.bucket_count() * sizeof(void *) +
        entries.size() * (sizeof(std::pair<const Key, Value>) + kHashLinks);
    if constexpr (_OwnsStorage<Key>::value) {
        for (const auto &entry : entries) {
            bytes += _OwnedBytes(entry.first);
        }
    }
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

// A nested container owns its own allocation, so an element that is itself a
// container, a pair, or a record with containers is walked, not just sized:
// growing an inner list with the outer length fixed grows the bound.
template <class T, class A>
struct _OwnsStorage<std::vector<T, A>> : std::true_type {};
template <class T, class A>
struct _OwnsStorage<std::deque<T, A>> : std::true_type {};
template <class T, size_t N>
struct _OwnsStorage<std::array<T, N>> : _OwnsStorage<T> {};
template <class Key, class T, class Compare, class A>
struct _OwnsStorage<std::map<Key, T, Compare, A>> : std::true_type {};
template <class Key, class T, class Hash, class Equal, class A>
struct _OwnsStorage<std::unordered_map<Key, T, Hash, Equal, A>>
    : std::true_type {};
template <class Key, class Compare, class A>
struct _OwnsStorage<std::set<Key, Compare, A>> : std::true_type {};
template <class Key, class Hash, class Equal, class A>
struct _OwnsStorage<std::unordered_set<Key, Hash, Equal, A>> : std::true_type {
};
template <class T> struct _OwnsStorage<std::optional<T>> : _OwnsStorage<T> {};
template <class A, class B>
struct _OwnsStorage<std::pair<A, B>>
    : std::conditional_t<_OwnsStorage<A>::value || _OwnsStorage<B>::value,
                         std::true_type, std::false_type> {};

// One entry point for a record's owning members: the member pointers name
// exactly the fields a copy of the record re-owns. A new owning field on a
// compiled-rig record belongs in its record's list beside the clone.
template <class Record, class... Members>
size_t
_MembersBytes(const Record &record, Members... members)
{
    size_t total = 0;
    ((total += _OwnedBytes(record.*members)), ...);
    return total;
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
    add(op.descriptor);
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
    add(leaves.decl);
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
    add(watch.typed);
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
    add(cones.always);
    add(cones.poseClusters);
    add(cones.alwaysSteps);
    add(cones.poseSteps);
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
    add(constraint.kernelRecord);
    add(constraint.kernelInputs);
    add(constraint.kernelResult);
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
    add(solver.kernelInputs);
    add(solver.kernelWorkspace);
    add(solver.ribbonPointsBinding);
    add(solver.controlRests);
    add(solver.jointRests);
    add(solver.restIsLive);
    add(solver.ikParams);
    add(solver.twistWeights);
    add(solver.ribbonRestPoints);
    add(solver.splineRest);
    add(solver.splineRestFrames);
    add(solver.splineRestWeights);
    add(solver.splineJointRests);
    add(solver.bone2dOrder);
    add(solver.bone2dSetup);
    add(solver.bone2dSpaceSources);
    add(solver.bone2dTargetCandidates);
    add(solver.jointGroups);
    add(solver.jointGroupMembers);
    add(solver.jointGroupParents);
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
    add(chain.gpuTranslation);
    add(chain.gpuFrame);
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
    add(revision.kernelRecord);
    add(revision.sceneGeometry);
    add(revision.binding);
    add(revision.lastParameters);
    add(revision.lastStatus);
    add(revision.parameters);
    add(revision.status);
    add(revision.currentPhasePacket);
    add(revision.rangeInputs);
    add(revision.wireBasis);
    add(revision.revisionInputs);
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
    add(object.oracleFacts);
    add(object.packetWorkspace);
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
    add(field.objects);
    add(field.scalarReadIndex);
    add(field.workspace);
    return total;
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::NativeXformSource &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::NativeXformSource::ancestorSlots);
}

size_t
_OwnedBytes(const RigExecProviderProgram &record)
{
    return _MembersBytes(
        record, &RigExecProviderProgram::valueKeys,
        &RigExecProviderProgram::valueIds, &RigExecProviderProgram::sampled,
        &RigExecProviderProgram::rawInputs,
        &RigExecProviderProgram::attributeValues,
        &RigExecProviderProgram::externalInputs,
        &RigExecProviderProgram::routedInputs, &RigExecProviderProgram::leaves,
        &RigExecProviderProgram::ops, &RigExecProviderProgram::descriptors,
        &RigExecProviderProgram::ownerTexts);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::ProviderRefresh &record)
{
    return _MembersBytes(record, &RigExecBakedProgramImpl::ProviderRefresh::key,
                         &RigExecBakedProgramImpl::ProviderRefresh::carries,
                         &RigExecBakedProgramImpl::ProviderRefresh::blocked);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::PoseInterpolator &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::PoseInterpolator::pathText,
        &RigExecBakedProgramImpl::PoseInterpolator::extraDriverSlots,
        &RigExecBakedProgramImpl::PoseInterpolator::extraParentSlots,
        &RigExecBakedProgramImpl::PoseInterpolator::valueIsFloat,
        &RigExecBakedProgramImpl::PoseInterpolator::values,
        &RigExecBakedProgramImpl::PoseInterpolator::primvarSlots,
        &RigExecBakedProgramImpl::PoseInterpolator::primvarSources,
        &RigExecBakedProgramImpl::PoseInterpolator::poseSlots,
        &RigExecBakedProgramImpl::PoseInterpolator::disabledSlots,
        &RigExecBakedProgramImpl::PoseInterpolator::scratch);
}

size_t
_OwnedBytes(const RigExecPointFrameArray &record)
{
    return _MembersBytes(record, &RigExecPointFrameArray::frames,
                         &RigExecPointFrameArray::rests);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::WindowedOp &record)
{
    return _MembersBytes(record, &RigExecBakedProgramImpl::WindowedOp::inner);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::WalkStep &record)
{
    return _MembersBytes(record,
                         &RigExecBakedProgramImpl::WalkStep::batchSolvers);
}

size_t
_OwnedBytes(const RigExecBakedStep &record)
{
    return _MembersBytes(
        record, &RigExecBakedStep::leaves, &RigExecBakedStep::overrideSlots,
        &RigExecBakedStep::bindingLeaves, &RigExecBakedStep::lines,
        &RigExecBakedStep::reads, &RigExecBakedStep::writes,
        &RigExecBakedStep::preds, &RigExecBakedStep::succs,
        &RigExecBakedStep::semanticPredecessorKeys,
        &RigExecBakedStep::overrideInputs, &RigExecBakedStep::readerWalks,
        &RigExecBakedStep::diagnostics, &RigExecBakedStep::counters,
        &RigExecBakedStep::label);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::SpaceCheckpoint &record)
{
    return _MembersBytes(record, &RigExecBakedProgramImpl::SpaceCheckpoint::key,
                         &RigExecBakedProgramImpl::SpaceCheckpoint::recompose);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::AutoClavicle &record)
{
    return _MembersBytes(record,
                         &RigExecBakedProgramImpl::AutoClavicle::operation,
                         &RigExecBakedProgramImpl::AutoClavicle::values,
                         &RigExecBakedProgramImpl::AutoClavicle::frames,
                         &RigExecBakedProgramImpl::AutoClavicle::scalars);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::SpaceSwitch &record)
{
    return _MembersBytes(record,
                         &RigExecBakedProgramImpl::SpaceSwitch::sourceSlots,
                         &RigExecBakedProgramImpl::SpaceSwitch::sourceReads,
                         &RigExecBakedProgramImpl::SpaceSwitch::parentRead,
                         &RigExecBakedProgramImpl::SpaceSwitch::spaceRead,
                         &RigExecBakedProgramImpl::SpaceSwitch::labels,
                         &RigExecBakedProgramImpl::SpaceSwitch::matchValues);
}

size_t
_OwnedBytes(const RigExecBakedComposeGroup &record)
{
    return _MembersBytes(record, &RigExecBakedComposeGroup::parentSlots);
}

size_t
_OwnedBytes(const RigExecBakedCommit &record)
{
    return _MembersBytes(
        record, &RigExecBakedCommit::moverPathText, &RigExecBakedCommit::slots,
        &RigExecBakedCommit::present, &RigExecBakedCommit::frames,
        &RigExecBakedCommit::closestPos, &RigExecBakedCommit::deltas,
        &RigExecBakedCommit::deltaOk, &RigExecBakedCommit::staged,
        &RigExecBakedCommit::outcome, &RigExecBakedCommit::sources,
        &RigExecBakedCommit::slotReads, &RigExecBakedCommit::slotWrites,
        &RigExecBakedCommit::slotBaseWrites,
        &RigExecBakedCommit::descendantReads, &RigExecBakedCommit::closestReads,
        &RigExecBakedCommit::descendantWrites,
        &RigExecBakedCommit::descendantBaseWrites,
        &RigExecBakedCommit::slotCarry, &RigExecBakedCommit::slotBaseCarry,
        &RigExecBakedCommit::descendantCarry,
        &RigExecBakedCommit::descendantBaseCarry,
        &RigExecBakedCommit::sourceReads, &RigExecBakedCommit::targetReads,
        &RigExecBakedCommit::effectorAncestors, &RigExecBakedCommit::poleReads,
        &RigExecBakedCommit::poleAncestors, &RigExecBakedCommit::ikChain,
        &RigExecBakedCommit::ikPrepared, &RigExecBakedCommit::ikRest,
        &RigExecBakedCommit::ikSolved, &RigExecBakedCommit::sourceAncestors,
        &RigExecBakedCommit::worldUpAncestors);
}

size_t
_OwnedBytes(const RigExecBakedClustering &record)
{
    return _MembersBytes(record, &RigExecBakedClustering::clusters,
                         &RigExecBakedClustering::clusterOf,
                         &RigExecBakedClustering::topologicalOrder);
}

size_t
_OwnedBytes(const RigExecWeightRecord &record)
{
    return _MembersBytes(
        record, &RigExecWeightRecord::name, &RigExecWeightRecord::staticError,
        &RigExecWeightRecord::phaseError, &RigExecWeightRecord::inputs,
        &RigExecWeightRecord::indices, &RigExecWeightRecord::values,
        &RigExecWeightRecord::falloffCurve);
}

size_t
_OwnedBytes(const RigExecOpWorkspace &record)
{
    return _MembersBytes(
        record, &RigExecOpWorkspace::seeded,
        &RigExecOpWorkspace::clusterCandidates, &RigExecOpWorkspace::pending,
        &RigExecOpWorkspace::ready, &RigExecOpWorkspace::unresolved,
        &RigExecOpWorkspace::candidateClusters);
}

size_t
_OwnedBytes(const RigExecOpExecution &record)
{
    return _MembersBytes(
        record, &RigExecOpExecution::candidates, &RigExecOpExecution::ran,
        &RigExecOpExecution::completion, &RigExecOpExecution::touched);
}

size_t
_OwnedBytes(const RigExecWeightPacket &record)
{
    return _MembersBytes(record, &RigExecWeightPacket::values,
                         &RigExecWeightPacket::indices);
}

size_t
_OwnedBytes(const RigExecBakedSparseInputs &record)
{
    return _MembersBytes(record, &RigExecBakedSparseInputs::varying,
                         &RigExecBakedSparseInputs::numberBegin,
                         &RigExecBakedSparseInputs::numberLeaves,
                         &RigExecBakedSparseInputs::visited);
}

size_t
_OwnedBytes(const RigExecCrossDomainRead &record)
{
    return _MembersBytes(record, &RigExecCrossDomainRead::phase,
                         &RigExecCrossDomainRead::frames,
                         &RigExecCrossDomainRead::unavailable);
}

size_t
_OwnedBytes(const RigExecBakedPropertyChain &record)
{
    return _MembersBytes(record, &RigExecBakedPropertyChain::revisions,
                         &RigExecBakedPropertyChain::records,
                         &RigExecBakedPropertyChain::linePrefix);
}

size_t
_OwnedBytes(const RigExecBakedPropertyRecord &record)
{
    return _MembersBytes(record, &RigExecBakedPropertyRecord::hopSlots);
}

size_t
_OwnedBytes(const RigExecWeightReferenceContext &record)
{
    return _MembersBytes(record, &RigExecWeightReferenceContext::prims,
                         &RigExecWeightReferenceContext::points,
                         &RigExecWeightReferenceContext::canonicalPoints,
                         &RigExecWeightReferenceContext::placements);
}

size_t
_OwnedBytes(const RigExecGpuDeformTranslation &state)
{
    return state.RetainedBytes();
}

size_t
_OwnedBytes(const RigExecOracleAttribute &attribute)
{
    return _MembersBytes(attribute, &RigExecOracleAttribute::connections,
                         &RigExecOracleAttribute::resolvedTime,
                         &RigExecOracleAttribute::resolvedDefault);
}

size_t
_OwnedBytes(const RigExecOracleRelationship &relationship)
{
    return _OwnedBytes(relationship.targets);
}

size_t
_OwnedBytes(const RigExecOraclePrim &prim)
{
    return _MembersBytes(prim, &RigExecOraclePrim::attributes,
                         &RigExecOraclePrim::relationships);
}

size_t
_OwnedBytes(const RigExecOracleScene &scene)
{
    // Begin() seals the capture builder's prims, primOrder and blendShapes
    // into one shared blob the clone aliases; unsealed they are owned and
    // charged here, and the overlay is always owned.
    return _MembersBytes(
        scene, &RigExecOracleScene::prims, &RigExecOracleScene::primOrder,
        &RigExecOracleScene::overlay, &RigExecOracleScene::blendShapes);
}

size_t
_OwnedBytes(const RigExecOraclePublicationContext &context)
{
    // A Chain row is its two completion scalars plus its write vector; the
    // writes' SdfPath and VtValue elements are handles to shared storage.
    size_t total =
        context.ChainCount() *
            (sizeof(RigExecOraclePublicationContext::Writes) +
             2 * sizeof(uint64_t)) +
        context.WriteCount() * sizeof(std::pair<SdfPath, VtValue>) +
        context.ProtectedPathCount() * (sizeof(SdfPath) + kTreeLinks);
    return total + _OwnedBytes(context.SourceScene());
}

size_t
_OwnedBytes(const RigExecBakedReaderWalk &record)
{
    return _MembersBytes(
        record, &RigExecBakedReaderWalk::walk, &RigExecBakedReaderWalk::leaves,
        &RigExecBakedReaderWalk::slots, &RigExecBakedReaderWalk::versions);
}

size_t
_OwnedBytes(const RigExecProviderExternalInput &record)
{
    return _MembersBytes(record, &RigExecProviderExternalInput::computation);
}

size_t
_OwnedBytes(const RigExecProviderOp &record)
{
    return _MembersBytes(record, &RigExecProviderOp::inputs,
                         &RigExecProviderOp::xforms);
}

size_t
_OwnedBytes(const RigExecOpDescriptor &record)
{
    return _MembersBytes(
        record, &RigExecOpDescriptor::key, &RigExecOpDescriptor::reads,
        &RigExecOpDescriptor::writes, &RigExecOpDescriptor::predecessors);
}

size_t
_OwnedBytes(const RigExecRevisionLeafDecl &record)
{
    return _MembersBytes(record, &RigExecRevisionLeafDecl::keys);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::ProviderRefresh::Carry &record)
{
    return _MembersBytes(
        record,
        &RigExecBakedProgramImpl::ProviderRefresh::Carry::blockingSlots);
}

size_t
_OwnedBytes(const RigExecSolverInputs &record)
{
    return _MembersBytes(
        record, &RigExecSolverInputs::controls, &RigExecSolverInputs::start,
        &RigExecSolverInputs::root, &RigExecSolverInputs::mid,
        &RigExecSolverInputs::end, &RigExecSolverInputs::pole,
        &RigExecSolverInputs::ribbonPoints, &RigExecSolverInputs::space,
        &RigExecSolverInputs::bone2dChannels,
        &RigExecSolverInputs::bone2dSelectors,
        &RigExecSolverInputs::jointGroupSelectors);
}

size_t
_OwnedBytes(const RigExecSolverWorkspace &record)
{
    return _MembersBytes(record, &RigExecSolverWorkspace::fkElements,
                         &RigExecSolverWorkspace::spacedFrames,
                         &RigExecSolverWorkspace::spacedRest,
                         &RigExecSolverWorkspace::splineResult,
                         &RigExecSolverWorkspace::bone2d,
                         &RigExecSolverWorkspace::bone2dExternalPoses,
                         &RigExecSolverWorkspace::jointGroupSlots,
                         &RigExecSolverWorkspace::jointGroupActive);
}

size_t
_OwnedBytes(const RigExecBakedPointsBinding &record)
{
    return _MembersBytes(record, &RigExecBakedPointsBinding::phase,
                         &RigExecBakedPointsBinding::candidates,
                         &RigExecBakedPointsBinding::missDiagnostic);
}

size_t
_OwnedBytes(const RigExecTwoBoneIkParams &record)
{
    return _MembersBytes(record, &RigExecTwoBoneIkParams::limb);
}

size_t
_OwnedBytes(const RigExecSplineIkRest &record)
{
    return _MembersBytes(
        record, &RigExecSplineIkRest::rootControl,
        &RigExecSplineIkRest::midControl, &RigExecSplineIkRest::endControl,
        &RigExecSplineIkRest::joints, &RigExecSplineIkRest::segmentLengths,
        &RigExecSplineIkRest::volumeWeights);
}

size_t
_OwnedBytes(const RigExecSolverRecord &record)
{
    return _MembersBytes(
        record, &RigExecSolverRecord::controlRests,
        &RigExecSolverRecord::jointRests, &RigExecSolverRecord::restIsLive,
        &RigExecSolverRecord::ikParams, &RigExecSolverRecord::twistWeights,
        &RigExecSolverRecord::ribbonRestPoints,
        &RigExecSolverRecord::splineRest, &RigExecSolverRecord::splineParams,
        &RigExecSolverRecord::splineRestFrames,
        &RigExecSolverRecord::splineRootRest,
        &RigExecSolverRecord::splineMidRest,
        &RigExecSolverRecord::splineEndRest,
        &RigExecSolverRecord::splineRestWeights,
        &RigExecSolverRecord::splineJointRests,
        &RigExecSolverRecord::bone2dOrder, &RigExecSolverRecord::bone2dSetup,
        &RigExecSolverRecord::bone2dSpaceSources,
        &RigExecSolverRecord::bone2dTargetCandidates,
        &RigExecSolverRecord::jointGroups,
        &RigExecSolverRecord::jointGroupMembers,
        &RigExecSolverRecord::jointGroupParents);
}

size_t
_OwnedBytes(const RigExecConstraintRecord &record)
{
    return _MembersBytes(
        record, &RigExecConstraintRecord::position,
        &RigExecConstraintRecord::rotation, &RigExecConstraintRecord::scale,
        &RigExecConstraintRecord::parent, &RigExecConstraintRecord::aim,
        &RigExecConstraintRecord::singleChain);
}

size_t
_OwnedBytes(const RigExecConstraintInputs &record)
{
    return _MembersBytes(
        record, &RigExecConstraintInputs::incoming,
        &RigExecConstraintInputs::sources, &RigExecConstraintInputs::carry,
        &RigExecConstraintInputs::chain, &RigExecConstraintInputs::effector,
        &RigExecConstraintInputs::worldUp,
        &RigExecConstraintInputs::poleObjects);
}

size_t
_OwnedBytes(const RigExecConstraintResult &record)
{
    return _MembersBytes(record, &RigExecConstraintResult::frame,
                         &RigExecConstraintResult::chain);
}

size_t
_OwnedBytes(const RigExecBoundAutoClavicle &record)
{
    return _MembersBytes(record, &RigExecBoundAutoClavicle::constants,
                         &RigExecBoundAutoClavicle::frames,
                         &RigExecBoundAutoClavicle::scalars,
                         &RigExecBoundAutoClavicle::reads,
                         &RigExecBoundAutoClavicle::unavailable);
}

size_t
_OwnedBytes(const RigExecTypedValueStore &record)
{
    return _MembersBytes(record, &RigExecTypedValueStore::values);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::AutoClavicle::FrameRead &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::AutoClavicle::FrameRead::computation,
        &RigExecBakedProgramImpl::AutoClavicle::FrameRead::recompose);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::SpaceSwitch::FrameVersion &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::SpaceSwitch::FrameVersion::recompose);
}

size_t
_OwnedBytes(const RigExecConstraintSource &record)
{
    return _MembersBytes(record, &RigExecConstraintSource::frame);
}

size_t
_OwnedBytes(const RigExecBakedCluster &record)
{
    return _MembersBytes(record, &RigExecBakedCluster::members,
                         &RigExecBakedCluster::preds,
                         &RigExecBakedCluster::succs);
}

template <class Point>
size_t
_OwnedBytes(const RigExecGroupState<Point> &record)
{
    return _MembersBytes(record, &RigExecGroupState<Point>::walk);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomChain::Derived &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::GeomChain::Derived::targetText,
        &RigExecBakedProgramImpl::GeomChain::Derived::revision);
}

size_t
_OwnedBytes(const RigExecGpuDeformFrame &record)
{
    return _MembersBytes(record, &RigExecGpuDeformFrame::frame);
}

size_t
_OwnedBytes(const RigExecWeightOracleFacts &record)
{
    return _MembersBytes(record, &RigExecWeightOracleFacts::samples,
                         &RigExecWeightOracleFacts::curve,
                         &RigExecWeightOracleFacts::phaseError,
                         &RigExecWeightOracleFacts::staticError);
}

size_t
_OwnedBytes(const RigExecWeightPacketWorkspace &record)
{
    return _MembersBytes(record, &RigExecWeightPacketWorkspace::support,
                         &RigExecWeightPacketWorkspace::supportOrder,
                         &RigExecWeightPacketWorkspace::combineField);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::WeightField::PointInput &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::WeightField::PointInput::binding);
}

size_t
_OwnedBytes(const RigExecWeightFieldWorkspace &record)
{
    return _MembersBytes(record, &RigExecWeightFieldWorkspace::seen,
                         &RigExecWeightFieldWorkspace::localCurve);
}

size_t
_OwnedBytes(const RigExecBakedPropertyChain::Revision &record)
{
    return _MembersBytes(record,
                         &RigExecBakedPropertyChain::Revision::pathPoints,
                         &RigExecBakedPropertyChain::Revision::path,
                         &RigExecBakedPropertyChain::Revision::linePrefix);
}

size_t
_OwnedBytes(const RigExecWeightReferencePrim &record)
{
    return _MembersBytes(record, &RigExecWeightReferencePrim::attributes,
                         &RigExecWeightReferencePrim::relationships,
                         &RigExecWeightReferencePrim::weightTargetPhase,
                         &RigExecWeightReferencePrim::phaseError,
                         &RigExecWeightReferencePrim::falloff);
}

size_t
_OwnedBytes(const RigExecBakedWalk &record)
{
    return _MembersBytes(record, &RigExecBakedWalk::hops,
                         &RigExecBakedWalk::doubleHops);
}

size_t
_OwnedBytes(const RigExecFkChainElement &record)
{
    return _MembersBytes(record, &RigExecFkChainElement::restPoints,
                         &RigExecFkChainElement::posePoints,
                         &RigExecFkChainElement::outRestPoints);
}

size_t
_OwnedBytes(const RigExecSplineIkResult &record)
{
    return _MembersBytes(record, &RigExecSplineIkResult::joints);
}

size_t
_OwnedBytes(const RigExecSkeleton2dScratch &record)
{
    return _MembersBytes(
        record, &RigExecSkeleton2dScratch::solve,
        &RigExecSkeleton2dScratch::rests,
        &RigExecSkeleton2dScratch::externalRests,
        &RigExecSkeleton2dScratch::externalPoses,
        &RigExecSkeleton2dScratch::heights, &RigExecSkeleton2dScratch::channels,
        &RigExecSkeleton2dScratch::pose, &RigExecSkeleton2dScratch::deform);
}

size_t
_OwnedBytes(const RigExecPositionConstraintParams &record)
{
    return _MembersBytes(record, &RigExecPositionConstraintParams::affect);
}

size_t
_OwnedBytes(const RigExecRotationConstraintParams &record)
{
    return _MembersBytes(record, &RigExecRotationConstraintParams::affect);
}

size_t
_OwnedBytes(const RigExecScaleConstraintParams &record)
{
    return _MembersBytes(record, &RigExecScaleConstraintParams::affect);
}

size_t
_OwnedBytes(const RigExecParentConstraintParams &record)
{
    return _MembersBytes(record,
                         &RigExecParentConstraintParams::translationAxes,
                         &RigExecParentConstraintParams::rotationAxes);
}

size_t
_OwnedBytes(const RigExecAimConstraintParams &record)
{
    return _MembersBytes(record, &RigExecAimConstraintParams::worldUpDirection,
                         &RigExecAimConstraintParams::affectRotation);
}

size_t
_OwnedBytes(const RigExecAutoClavicleConstants &record)
{
    return _MembersBytes(record, &RigExecAutoClavicleConstants::swings,
                         &RigExecAutoClavicleConstants::widths,
                         &RigExecAutoClavicleConstants::gains,
                         &RigExecAutoClavicleConstants::weights);
}

size_t
_OwnedBytes(const RigExecGraphTypedRead &record)
{
    return _MembersBytes(record, &RigExecGraphTypedRead::hops,
                         &RigExecGraphTypedRead::doubleHops,
                         &RigExecGraphTypedRead::unavailable,
                         &RigExecGraphTypedRead::elementUnavailable);
}

size_t
_OwnedBytes(const RigExecTypedValueState &record)
{
    return _MembersBytes(record, &RigExecTypedValueState::error);
}

size_t
_OwnedBytes(const RigExecGeometryRecord &record)
{
    return _MembersBytes(record, &RigExecGeometryRecord::binding,
                         &RigExecGeometryRecord::leaves);
}

size_t
_OwnedBytes(const RigExecSceneGeometryDescriptor &record)
{
    return _MembersBytes(record, &RigExecSceneGeometryDescriptor::record,
                         &RigExecSceneGeometryDescriptor::inputs,
                         &RigExecSceneGeometryDescriptor::bound,
                         &RigExecSceneGeometryDescriptor::typedLeaves,
                         &RigExecSceneGeometryDescriptor::blendChannels);
}

size_t
_OwnedBytes(const RigExecRevisionBinding &record)
{
    return _MembersBytes(record, &RigExecRevisionBinding::externalStructure,
                         &RigExecRevisionBinding::externalCompileSources,
                         &RigExecRevisionBinding::externalInputs,
                         &RigExecRevisionBinding::influences,
                         &RigExecRevisionBinding::shaderDials,
                         &RigExecRevisionBinding::blendInputs,
                         &RigExecRevisionBinding::blendSamples,
                         &RigExecRevisionBinding::phases,
                         &RigExecRevisionBinding::transformPhase);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomBlendChannel &record)
{
    return _MembersBytes(record,
                         &RigExecBakedProgramImpl::GeomBlendChannel::samples);
}

size_t
_OwnedBytes(const RigExecBlendChannel &record)
{
    return _MembersBytes(record, &RigExecBlendChannel::samples);
}

size_t
_OwnedBytes(const RigExecMoverParameters &record)
{
    return _MembersBytes(
        record, &RigExecMoverParameters::weights,
        &RigExecMoverParameters::blendDeltas,
        &RigExecMoverParameters::mushSettings,
        &RigExecMoverParameters::wrinkleSettings,
        &RigExecMoverParameters::surfaceSettings,
        &RigExecMoverParameters::latticeSettings,
        &RigExecMoverParameters::topologyCounts,
        &RigExecMoverParameters::topologyIndices,
        &RigExecMoverParameters::auxPoints, &RigExecMoverParameters::auxPointsB,
        &RigExecMoverParameters::restPoints,
        &RigExecMoverParameters::bindCoords, &RigExecMoverParameters::frames,
        &RigExecMoverParameters::curveKnots, &RigExecMoverParameters::widths,
        &RigExecMoverParameters::skinTransforms,
        &RigExecMoverParameters::skinIndices,
        &RigExecMoverParameters::skinWeights);
}

size_t
_OwnedBytes(const RigExecMoverStatus &record)
{
    return _MembersBytes(record, &RigExecMoverStatus::firstBadAddress);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomChunk &record)
{
    return _MembersBytes(record, &RigExecBakedProgramImpl::GeomChunk::key,
                         &RigExecBakedProgramImpl::GeomChunk::blendDeltas,
                         &RigExecBakedProgramImpl::GeomChunk::transforms,
                         &RigExecBakedProgramImpl::GeomChunk::rows,
                         &RigExecBakedProgramImpl::GeomChunk::palette);
}

size_t
_OwnedBytes(const RigExecScaledDualQuat &record)
{
    return _MembersBytes(record, &RigExecScaledDualQuat::rigid);
}

size_t
_OwnedBytes(const RigExecRevisionRangeInputs &record)
{
    return _MembersBytes(record, &RigExecRevisionRangeInputs::matrixWeights,
                         &RigExecRevisionRangeInputs::blendWeights,
                         &RigExecRevisionRangeInputs::wireBasis);
}

// The property-chain overlay a lane clones re-owns its map nodes; a chained
// base reports the effective count, which is what the clone materialises.
// The VtValue payloads are type-erased and charged at their inline size.
size_t
_OwnedBytes(const RigExecResolvedInputs &resolved)
{
    return resolved.GetSize() *
           (sizeof(std::pair<const SdfPath, VtValue>) + kHashLinks);
}

// A cloned wire-basis cache owns its nodes; the bases stay shared.
size_t
_OwnedBytes(const RigExecWireBasisCache &cache)
{
    return cache.Size() *
           (sizeof(uint64_t) + sizeof(std::shared_ptr<void>) + kHashLinks);
}

template <class Point, class Wide>
size_t
_OwnedBytes(const RigExecDeltaMushGroupState<Point, Wide> &record)
{
    return _MembersBytes(
        record, &RigExecDeltaMushGroupState<Point, Wide>::in,
        &RigExecDeltaMushGroupState<Point, Wide>::smoothPosed,
        &RigExecDeltaMushGroupState<Point, Wide>::current,
        &RigExecDeltaMushGroupState<Point, Wide>::next,
        &RigExecDeltaMushGroupState<Point, Wide>::posedNormals);
}

template <class Point>
size_t
_OwnedBytes(const RigExecSparseWalkMemo<Point> &record)
{
    return _MembersBytes(record, &RigExecSparseWalkMemo<Point>::params,
                         &RigExecSparseWalkMemo<Point>::indices,
                         &RigExecSparseWalkMemo<Point>::weights,
                         &RigExecSparseWalkMemo<Point>::in,
                         &RigExecSparseWalkMemo<Point>::out);
}

size_t
_OwnedBytes(const RigExecWeightReferenceRelationship &record)
{
    return _MembersBytes(record, &RigExecWeightReferenceRelationship::phase,
                         &RigExecWeightReferenceRelationship::phaseError);
}

size_t
_OwnedBytes(const RigExecSplineIkJoint &record)
{
    return _MembersBytes(record, &RigExecSplineIkJoint::frame);
}

size_t
_OwnedBytes(const RigExecSkeleton2dWorkspace &record)
{
    return _MembersBytes(
        record, &RigExecSkeleton2dWorkspace::pose,
        &RigExecSkeleton2dWorkspace::localAngle,
        &RigExecSkeleton2dWorkspace::scale,
        &RigExecSkeleton2dWorkspace::baseAngle,
        &RigExecSkeleton2dWorkspace::restLocalX,
        &RigExecSkeleton2dWorkspace::restLocalY,
        &RigExecSkeleton2dWorkspace::localX,
        &RigExecSkeleton2dWorkspace::localY,
        &RigExecSkeleton2dWorkspace::spaceA,
        &RigExecSkeleton2dWorkspace::spaceB,
        &RigExecSkeleton2dWorkspace::spaceBlend,
        &RigExecSkeleton2dWorkspace::tipX, &RigExecSkeleton2dWorkspace::tipY,
        &RigExecSkeleton2dWorkspace::external,
        &RigExecSkeleton2dWorkspace::chain,
        &RigExecSkeleton2dWorkspace::countedChildren,
        &RigExecSkeleton2dWorkspace::dirty,
        &RigExecSkeleton2dWorkspace::effSetup, &RigExecSkeleton2dWorkspace::eff,
        &RigExecSkeleton2dWorkspace::active,
        &RigExecSkeleton2dWorkspace::ctrlDone,
        &RigExecSkeleton2dWorkspace::groupSlot);
}

size_t
_OwnedBytes(const RigExecSceneBoundInput &record)
{
    return _MembersBytes(record, &RigExecSceneBoundInput::values,
                         &RigExecSceneBoundInput::available,
                         &RigExecSceneBoundInput::blocked,
                         &RigExecSceneBoundInput::computed);
}

size_t
_OwnedBytes(const RigExecSceneTypedRead &record)
{
    return _MembersBytes(record, &RigExecSceneTypedRead::hops,
                         &RigExecSceneTypedRead::doubleHops);
}

size_t
_OwnedBytes(const RigExecSceneGeometryBlendChannel &record)
{
    return _MembersBytes(record, &RigExecSceneGeometryBlendChannel::samples);
}

size_t
_OwnedBytes(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &record)
{
    return _MembersBytes(
        record, &RigExecBakedProgramImpl::GeomBlendChannel::Sample::phase,
        &RigExecBakedProgramImpl::GeomBlendChannel::Sample::pointBinding,
        &RigExecBakedProgramImpl::GeomBlendChannel::Sample::lastPoints);
}

size_t
_OwnedBytes(const RigExecBlendSampleData &record)
{
    return _MembersBytes(record, &RigExecBlendSampleData::points);
}

size_t
_OwnedBytes(const RigExecDeltaMushSettings &record)
{
    return _MembersBytes(record, &RigExecDeltaMushSettings::smoothWeights,
                         &RigExecDeltaMushSettings::edges);
}

size_t
_OwnedBytes(const RigExecWrinkleSettings &record)
{
    return _MembersBytes(record, &RigExecWrinkleSettings::pinPoints);
}

size_t
_OwnedBytes(const RigExecSurfaceSnapSettings &record)
{
    return _MembersBytes(record, &RigExecSurfaceSnapSettings::mask,
                         &RigExecSurfaceSnapSettings::triangles);
}

size_t
_OwnedBytes(const RigExecLatticeSettings &record)
{
    return _MembersBytes(record, &RigExecLatticeSettings::mask);
}

size_t
_OwnedBytes(const RigExecSceneGeometryBlendSample &record)
{
    return _MembersBytes(record,
                         &RigExecSceneGeometryBlendSample::additionalPoints);
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
    add(src.opWorkspace);
    add(src.resolvedInputs);
    return total;
}

// The freeze-captured tables one snapshot owns, sized by the program counts
// that fill them: a copy of the evaluator's joint/solver binding map, one
// head path and typed fallback per patchable binding, four operator-array
// keys per constraintArrays entry, one flag per chain, derived target,
// solver and revision input, and the mover and weight-object sample keys.
size_t
_FrozenTableBytes(const RigExecBakedProgramImpl &B)
{
    const size_t heads = B.leafRefs.size();
    const size_t revisions = B.revisionIndex.size();
    size_t bindingEntries = 0;
    size_t bindingPairs = 0;
    if (B.jointSolverBinding) {
        bindingEntries = B.jointSolverBinding->size();
        for (const auto &entry : *B.jointSolverBinding) {
            bindingPairs += entry.second.size();
        }
    }
    return heads * (sizeof(SdfPath) + sizeof(VtValue)) +
           4 * B.constraintArrays.size() * sizeof(SdfPath) +
           (B.chains.size() + B.derivedIndex.size() + B.solvers.size() +
            3 * revisions) *
               sizeof(char) +
           (revisions + 4 * B.weightObjects.size()) * sizeof(SdfPath) +
           bindingEntries *
               (sizeof(std::pair<const SdfPath,
                                 std::vector<std::pair<SdfPath, int>>>) +
                kTreeLinks) +
           bindingPairs * sizeof(std::pair<SdfPath, int>);
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
    return RigExecEstimateFrozenArenaBytes(live.GetStepGraph());
}

size_t
RigExecEstimateFrozenArenaBytes(const RigExecBakedProgramImpl &stepGraph)
{
    return frozenDetail::_ProgramStorageBytes(stepGraph);
}

// A job builds two clones of the standing program -- the frame's snapshot and
// the workspace the lane runs on -- plus one set of freeze-captured tables
// both read, including the copy of the evaluator's joint/solver binding map
// the freeze captures. An aliased immutable target is charged once per clone
// that re-owns a handle to it, which the arena bound already accounts for.
size_t
RigExecEstimateFrozenJobBytes(const RigExecBakedProgram &live)
{
    return RigExecEstimateFrozenJobBytes(live.GetStepGraph());
}

size_t
RigExecEstimateFrozenJobBytes(const RigExecBakedProgramImpl &stepGraph)
{
    const size_t lane = frozenDetail::_ProgramStorageBytes(stepGraph);
    const size_t tables = frozenDetail::_FrozenTableBytes(stepGraph);
    const size_t limit = (std::numeric_limits<size_t>::max)();
    if (lane > limit / 2 || tables > limit - lane * 2) {
        return limit;
    }
    return lane * 2 + tables;
}

} // namespace rigExec
