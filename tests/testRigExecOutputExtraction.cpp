// Requested execution and extraction: run the compiled graph over the
// closure of the outputs a host asked for, report what each value that
// closure writes did, and answer those outputs as immutable typed payloads
// read from the storage the run left (docs/specs/output-subscriptions.md).
// What this suite holds to account: a request executes nothing outside its
// closure, a union shares an ancestor once, a value never materialized is
// computed even when its inputs are clean, an operation a request left out
// keeps its debt, a sequence of partial runs lands on the same answer as one
// whole run, and an extraction equals full evaluation -- with an explicit
// status for every output it cannot answer, the intermediate a coarsened
// cluster stopped at retained for the run that continues past it, and the
// bytes it handed out unchanged by every later run.
#include "rigExec/outputCatalog.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecGraph/outputDemand.h"
#include "rigExecOpTrace.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/setenv.h"

#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

const char *const kBody = "/Asset/Container/Geometry/Body.points";
const char *const kOther = "/Asset/Container/Geometry/Other.points";
const char *const kAmount = "/Asset/Container/Character/PropertyHome.amount";
const char *const kTrail = "/Asset/Container/Character/Trail.mark";

const char *const kStageText = R"USDA(#usda 1.0

def Xform "Asset"
{
    def Scope "Container"
    {
        def RigExecRoot "Character"
        {
            def Scope "Controls"
            {
                def RigExecControl "Drive"
                {
                    double avars:ty = 2
                    double avars:ty.timeSamples = {
                        2: 2,
                        8: 5,
                    }
                }
            }

            def Scope "StackA"
            {
                def RigExecMatrixMover "First" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    float inputs:defaultWeight = 1
                    rel rigExec:moves = </Asset/Container/Geometry/Body.points>
                    rel rigExec:transform = </Asset/Container/Character/Controls/Drive>
                }
                def RigExecMatrixMover "Second" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    float inputs:defaultWeight = 1
                    rel rigExec:moves = </Asset/Container/Geometry/Body.points>
                    rel rigExec:transform = </Asset/Container/Character/Controls/Drive>
                }
            }

            def Scope "StackB"
            {
                def RigExecMatrixMover "Sole" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    float inputs:defaultWeight = 1
                    rel rigExec:moves = </Asset/Container/Geometry/Other.points>
                    rel rigExec:transform = </Asset/Container/Character/Controls/Drive>
                }
            }

            def Scope "PropertyHome"
            {
                float amount = 3

                def RigExecFloatMathMover "AmountDoubler" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </Asset/Container/Character/PropertyHome.amount>
                }
            }

            def Scope "Trail"
            {
                float mark = 1

                def RigExecFloatMathMover "T1" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </Asset/Container/Character/Trail.mark>
                }
                def RigExecFloatMathMover "T2" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </Asset/Container/Character/Trail.mark>
                }
                def RigExecFloatMathMover "T3" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </Asset/Container/Character/Trail.mark>
                }
            }
        }

        def Scope "Geometry"
        {
            def Points "Body"
            {
                point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]
            }
            def Points "Other"
            {
                point3f[] points = [(0, 0, 0), (0, 0, 1), (0, 1, 1)]
            }
        }
    }
}
)USDA";

struct Rig {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    // The evaluator hands out its own program as const; a requested run is
    // asked for by a host, so this suite builds the program it runs, the way
    // the schedule suite does. Declared after the evaluator: the program
    // holds a pointer to it.
    std::unique_ptr<RigExecBakedProgram> holder;
    RigExecBakedProgram *program = nullptr;
    RigExecOutputCatalog catalog;
};

Rig
BuildRigFrom(const char *stageText, const char *rigPath)
{
    Rig rig;
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer->ImportFromString(stageText));
    rig.stage = UsdStage::Open(layer);
    CHECK(rig.stage);
    rig.evaluator =
        std::make_unique<RigExecRigEvaluator>(rig.stage, SdfPath(rigPath));
    std::vector<std::string> errors;
    CHECK(rig.evaluator->Compile(&errors));
    for (const std::string &error : errors)
        std::printf("    compile: %s\n", error.c_str());
    std::vector<std::string> reasons;
    rig.holder = RigExecBakedProgram::Build(rig.evaluator.get(), &reasons);
    for (const std::string &reason : reasons)
        std::printf("    bake: %s\n", reason.c_str());
    rig.program = rig.holder.get();
    CHECK(rig.program);
    if (!rig.program)
        return rig;
    std::string error;
    CHECK(RigExecDiscoverOutputs(*rig.program, "requested-rig", 1, &rig.catalog,
                                 &error));
    if (!error.empty())
        std::printf("    catalog: %s\n", error.c_str());
    return rig;
}

Rig
BuildRig()
{
    return BuildRigFrom(kStageText, "/Asset/Container/Character");
}

/// The index slots whose binding reads from \p target.
std::vector<size_t>
SlotsForTarget(const Rig &rig, const std::string &target)
{
    std::vector<size_t> slots;
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.descriptor.target != target)
            continue;
        size_t slot = 0;
        if (rig.catalog.GetDemandIndex().Find(binding.handle, &slot))
            slots.push_back(slot);
    }
    return slots;
}

/// The slots naming one semantic address of \p target.
size_t
SlotForPhase(const Rig &rig, const std::string &target,
             const std::string &phase)
{
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.descriptor.target != target ||
            binding.descriptor.phase != phase)
            continue;
        size_t slot = 0;
        if (rig.catalog.GetDemandIndex().Find(binding.handle, &slot))
            return slot;
    }
    return size_t(-1);
}

/// The phase names the catalog exports for \p target, in catalog order.
std::vector<std::string>
PhasesFor(const Rig &rig, const std::string &target)
{
    std::vector<std::string> phases;
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings())
        if (binding.descriptor.target == target)
            phases.push_back(binding.descriptor.phase);
    return phases;
}

RigExecDemandClosure
ClosureFor(const Rig &rig, const std::vector<size_t> &slots)
{
    return rig.catalog.GetDemandIndex().Union(slots);
}

/// Runs one request and reports what it did.
bool
Request(Rig &rig, UsdTimeCode time, const RigExecDemandClosure &closure,
        RigExecRequestedRunReport *report, std::string *error)
{
    return rig.program->RunRequested(time, closure, report, error);
}

bool
InClosure(const RigExecDemandClosure &closure, uint32_t op)
{
    return closure.Contains(op);
}

/// Every trace entry whose canonical operation stands outside \p closure.
std::vector<RigExecOpTraceEntry>
TraceOutside(const RigExecBakedProgram &program,
             const RigExecDemandClosure &closure)
{
    std::vector<RigExecOpTraceEntry> outside;
    for (const RigExecOpTraceEntry &entry : program.GetLastOpTrace())
        if (!InClosure(closure, uint32_t(entry.step)))
            outside.push_back(entry);
    return outside;
}

const RigExecValueMaterialization *
FactFor(const RigExecRequestedRunReport &report, RigExecValueId value)
{
    for (const RigExecValueMaterialization &fact : report.values)
        if (fact.value == value)
            return &fact;
    return nullptr;
}

/// The value id the catalog binding for one semantic address names.
RigExecValueId
ProducerValue(const Rig &rig, const std::string &target,
              const std::string &phase)
{
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings())
        if (binding.descriptor.target == target &&
            binding.descriptor.phase == phase)
            return binding.producerValue;
    return kRigExecNoOutputValue;
}

VtValue
Moved(const RigExecRigPose &pose, const std::string &path)
{
    const auto found = pose.movedProperties.find(SdfPath(path));
    return found == pose.movedProperties.end() ? VtValue() : found->second;
}

// Every operation the request executed stands inside the closure, and the
// trace names no body outside it: an unrelated subtree runs nothing.
void
TestRequestRunsOnlyItsClosure()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;

    const std::vector<size_t> bodySlots = SlotsForTarget(rig, kBody);
    const std::vector<size_t> otherSlots = SlotsForTarget(rig, kOther);
    const std::vector<size_t> amountSlots = SlotsForTarget(rig, kAmount);
    CHECK(!bodySlots.empty());
    CHECK(!otherSlots.empty());
    CHECK(!amountSlots.empty());

    const RigExecDemandClosure bodyClosure = ClosureFor(rig, bodySlots);
    const RigExecDemandClosure otherClosure = ClosureFor(rig, otherSlots);
    const RigExecDemandClosure amountClosure = ClosureFor(rig, amountSlots);
    CHECK(!bodyClosure.Empty());
    CHECK(RigExecClosureIsClosed(rig.program->GetStepGraph().opGraph,
                                 bodyClosure));

    RigExecRequestedRunReport report;
    std::string error;
    CHECK(Request(rig, UsdTimeCode::Default(), bodyClosure, &report, &error));
    CHECK(error.empty());
    CHECK(report.requestedOperations == bodyClosure.ops.size());
    CHECK(report.executedOperations == report.executed.size());
    size_t otherBodies = 0, propertyBodies = 0;
    for (uint32_t op : report.executed) {
        CHECK(InClosure(bodyClosure, op));
        // Nothing the other subtree needs on its own ran.
        if (InClosure(otherClosure, op) && !InClosure(bodyClosure, op))
            ++otherBodies;
        if (InClosure(amountClosure, op) && !InClosure(bodyClosure, op))
            ++propertyBodies;
    }
    CHECK(otherBodies == 0);
    CHECK(propertyBodies == 0);
    CHECK(TraceOutside(*rig.program, bodyClosure).empty());
    const std::vector<std::string> order = rigExecTest::CheckTraceRespectsEdges(
        rig.program->GetLastOpTrace(), rig.program->GetOpGraph());
    for (const std::string &violation : order)
        std::printf("    order: %s\n", violation.c_str());
    CHECK(order.empty());

    // The whole-rig run that follows is unchanged by having been asked for
    // one subtree first: the pose matches a whole run of a fresh program.
    RigExecRigPose partial;
    CHECK(rig.program->Run(UsdTimeCode::Default(), &partial));
    Rig fresh = BuildRig();
    RigExecRigPose whole;
    CHECK(fresh.program->Run(UsdTimeCode::Default(), &whole));
    CHECK(Moved(partial, kBody) == Moved(whole, kBody));
    CHECK(Moved(partial, kOther) == Moved(whole, kOther));
    CHECK(Moved(partial, kAmount) == Moved(whole, kAmount));
}

// Two outputs that read a shared ancestor: the union names the ancestor once
// and its body runs once.
void
TestUnionSharesAncestorsOnce()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;
    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    const size_t otherFinal = SlotForPhase(rig, kOther, "final");
    CHECK(bodyFinal != size_t(-1));
    CHECK(otherFinal != size_t(-1));

    const RigExecDemandClosure bodyOnly = ClosureFor(rig, {bodyFinal});
    const RigExecDemandClosure otherOnly = ClosureFor(rig, {otherFinal});
    const RigExecDemandClosure both = ClosureFor(rig, {bodyFinal, otherFinal});
    CHECK(both.ops.size() < bodyOnly.ops.size() + otherOnly.ops.size());
    // Each ancestor the two closures share is named once by the union.
    size_t shared = 0;
    for (uint32_t op : bodyOnly.ops)
        if (otherOnly.Contains(op))
            ++shared;
    CHECK(shared > 0);

    RigExecRequestedRunReport report;
    std::string error;
    CHECK(Request(rig, UsdTimeCode::Default(), both, &report, &error));
    CHECK(report.executedOperations == report.executed.size());
    CHECK(std::adjacent_find(report.executed.begin(), report.executed.end()) ==
          report.executed.end());
    const std::vector<RigExecOpTraceEntry> trace =
        rig.program->GetLastOpTrace();
    CHECK(trace.size() == report.executed.size());
    // The trace is a valid completion order over the whole graph: an op
    // appears once, so the union shares an ancestor rather than running it
    // per consumer, and every executed predecessor finished first.
    const std::vector<std::string> order =
        rigExecTest::CheckTraceRespectsEdges(trace, rig.program->GetOpGraph());
    for (const std::string &violation : order)
        std::printf("    order: %s\n", violation.c_str());
    CHECK(order.empty());
    CHECK(report.executed.size() <= both.ops.size());
    // The shared ancestor is among what ran, once.
    size_t sharedRan = 0;
    for (uint32_t op : report.executed)
        if (bodyOnly.Contains(op) && otherOnly.Contains(op))
            ++sharedRan;
    CHECK(sharedRan == shared);
}

// A value the arena never produced is missing, not clean: the request that
// needs it computes it even though nothing its producer reads has moved.
void
TestPartialExpansionComputesMissingCleanInputs()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;
    const std::vector<std::string> phases = PhasesFor(rig, kBody);
    CHECK(phases.size() >= 3); // base, each mover phase, final

    // Ask for the first phase only: the deeper revision never runs.
    std::string firstPhaseName;
    for (const std::string &phase : phases)
        if (phase.find("First") != std::string::npos)
            firstPhaseName = phase;
    CHECK(!firstPhaseName.empty());
    const size_t firstPhase = SlotForPhase(rig, kBody, firstPhaseName);
    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    CHECK(firstPhase != size_t(-1));
    CHECK(bodyFinal != size_t(-1));

    const RigExecDemandClosure shallow = ClosureFor(rig, {firstPhase});
    RigExecRequestedRunReport shallowReport;
    std::string error;
    CHECK(
        Request(rig, UsdTimeCode::Default(), shallow, &shallowReport, &error));
    const RigExecValueId finalValue = ProducerValue(rig, kBody, "final");
    CHECK(finalValue != kRigExecNoOutputValue);
    const RigExecValueId firstValue = ProducerValue(rig, kBody, firstPhaseName);
    CHECK(firstValue != kRigExecNoOutputValue);
    const RigExecValueMaterialization *shallowFinal =
        FactFor(shallowReport, finalValue);
    // The final producer is outside the shallow request: no fact, because
    // nothing the shallow closure writes stands in for it.
    CHECK(!shallowFinal);
    const RigExecValueMaterialization *shallowFirst =
        FactFor(shallowReport, firstValue);
    CHECK(shallowFirst && shallowFirst->produced && shallowFirst->wasAbsent);

    // Now the whole chain, same time, nothing moved. The final producer's
    // inputs are clean by every key the program holds, and its value was
    // never materialized, so it runs anyway.
    const RigExecDemandClosure deep = ClosureFor(rig, {bodyFinal});
    RigExecRequestedRunReport deepReport;
    CHECK(Request(rig, UsdTimeCode::Default(), deep, &deepReport, &error));
    const RigExecValueMaterialization *deepFinal =
        FactFor(deepReport, finalValue);
    CHECK(deepFinal && deepFinal->produced && deepFinal->wasAbsent);
    const RigExecValueMaterialization *deepFirst =
        FactFor(deepReport, firstValue);
    CHECK(deepFirst && !deepFirst->produced && deepFirst->reused &&
          !deepFirst->wasAbsent);

    // And the answer matches a whole run.
    RigExecRigPose partial;
    CHECK(rig.program->Run(UsdTimeCode::Default(), &partial));
    Rig fresh = BuildRig();
    RigExecRigPose whole;
    CHECK(fresh.program->Run(UsdTimeCode::Default(), &whole));
    CHECK(Moved(partial, kBody) == Moved(whole, kBody));
}

// A sequence of partial runs, then a whole run, lands on a fresh whole run's
// answer: nothing a request left out is ever read back as clean.
void
TestPartialToFullMatchesAFreshFullRun()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;
    const UsdTimeCode time(12.0);

    // One output at a time, in an order that leaves every subtree partially
    // materialized behind it.
    for (const char *target : {kAmount, kOther, kBody}) {
        for (const std::string &phase : PhasesFor(rig, target)) {
            const size_t slot = SlotForPhase(rig, target, phase);
            if (slot == size_t(-1))
                continue;
            const RigExecDemandClosure closure = ClosureFor(rig, {slot});
            RigExecRequestedRunReport report;
            std::string error;
            if (!Request(rig, time, closure, &report, &error)) {
                ++failures;
                std::printf("FAIL %s:%d: requested run for %s %s: %s\n",
                            __FILE__, __LINE__, target, phase.c_str(),
                            error.c_str());
            }
        }
    }

    RigExecRigPose partial;
    CHECK(rig.program->Run(time, &partial));
    CHECK(partial.valid);

    Rig fresh = BuildRig();
    RigExecRigPose whole;
    CHECK(fresh.program->Run(time, &whole));
    CHECK(whole.valid);
    CHECK(Moved(partial, kBody) == Moved(whole, kBody));
    CHECK(Moved(partial, kOther) == Moved(whole, kOther));
    CHECK(Moved(partial, kAmount) == Moved(whole, kAmount));

    // A different frame is a different answer. The partial run at the later
    // frame re-runs what the earlier one materialized, and the pose still
    // matches a whole run of a fresh rig at that frame.
    const size_t otherFinal = SlotForPhase(rig, kOther, "final");
    CHECK(otherFinal != size_t(-1));
    const RigExecDemandClosure other = ClosureFor(rig, {otherFinal});
    RigExecRequestedRunReport report;
    std::string error;
    const UsdTimeCode later(3.0);
    CHECK(Request(rig, later, other, &report, &error));
    RigExecRigPose atLater;
    CHECK(rig.program->Run(later, &atLater));
    RigExecRigPose wholeAtLater;
    CHECK(fresh.program->Run(later, &wholeAtLater));
    CHECK(Moved(atLater, kBody) == Moved(wholeAtLater, kBody));
    CHECK(Moved(atLater, kOther) == Moved(wholeAtLater, kOther));
    CHECK(Moved(atLater, kAmount) == Moved(wholeAtLater, kAmount));
    // The later frame is a different answer than the earlier one: the request
    // did not read a stale value back as current.
    CHECK(Moved(wholeAtLater, kOther) != Moved(whole, kOther));
    CHECK(Moved(atLater, kOther) != Moved(whole, kOther));
}

// A mask that is not predecessor closed is refused before any body runs.
void
TestRefusesAnOpenClosure()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;
    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    CHECK(bodyFinal != size_t(-1));
    RigExecDemandClosure closure = ClosureFor(rig, {bodyFinal});
    CHECK(closure.ops.size() > 1);
    // Drop one predecessor of the producer: what is left is no longer closed.
    const RigExecCompiledGraph &graph = rig.program->GetStepGraph().opGraph;
    const uint32_t producer = rig.catalog.GetDemandIndex().Producer(bodyFinal);
    CHECK(producer != kRigExecNoOp);
    CHECK(!graph.ops[size_t(producer)].predecessors.empty());
    const uint32_t dropped = graph.ops[size_t(producer)].predecessors.front();
    const auto at = std::find(closure.ops.begin(), closure.ops.end(), dropped);
    CHECK(at != closure.ops.end());
    const size_t position = size_t(at - closure.ops.begin());
    closure.ops.erase(at);
    closure.kinds.erase(closure.kinds.begin() + std::ptrdiff_t(position));

    RigExecRequestedRunReport report;
    std::string error;
    CHECK(!rig.program->RunRequested(UsdTimeCode::Default(), closure, &report,
                                     &error));
    CHECK(!error.empty());
    CHECK(error.find("predecessor closed") != std::string::npos);
    CHECK(report.executed.empty());
    CHECK(report.values.empty());
    // A closure spells its operations ascending, so a hand-built mask that
    // does not is refused before any body runs rather than run under a mask
    // that means something else.
    RigExecDemandClosure unordered = ClosureFor(rig, {bodyFinal});
    std::reverse(unordered.ops.begin(), unordered.ops.end());
    std::reverse(unordered.kinds.begin(), unordered.kinds.end());
    error.clear();
    CHECK(!rig.program->RunRequested(UsdTimeCode::Default(), unordered, &report,
                                     &error));
    CHECK(error.find("out of order") != std::string::npos);
    CHECK(report.executed.empty());
    // The refusal ran nothing, so a whole run after it is a whole run.
    RigExecRigPose pose;
    CHECK(rig.program->Run(UsdTimeCode::Default(), &pose));
    Rig fresh = BuildRig();
    RigExecRigPose whole;
    CHECK(fresh.program->Run(UsdTimeCode::Default(), &whole));
    CHECK(Moved(pose, kBody) == Moved(whole, kBody));
    CHECK(Moved(pose, kOther) == Moved(whole, kOther));
}

// A cluster is the scheduling unit, so a request that stops inside one must
// still run only its own operations, and the intermediate it stopped at must
// stand, unrecomputed, for the run that continues past it.
void
TestClusterBoundariesHoldTheMask()
{
    // Ask for wide clusters: the executor runs a cluster's candidate members,
    // never its whole span, whatever grain the schedule packed.
    TfSetenv("RIGEXEC_BAKED_GRAIN_US", "1000000");
    Rig rig = BuildRig();
    TfUnsetenv("RIGEXEC_BAKED_GRAIN_US");
    if (!rig.program)
        return;
    const std::vector<size_t> bodySlots = SlotsForTarget(rig, kBody);
    CHECK(!bodySlots.empty());

    // A chain of property movers: a request for the first link stops partway
    // down it.
    // The phase named for T3 stands on the chain's first revision, so a
    // request for it stops partway down the chain.
    std::string firstTrailPhase;
    for (const std::string &phase : PhasesFor(rig, kTrail))
        if (phase.find("/T3") != std::string::npos)
            firstTrailPhase = phase;
    CHECK(!firstTrailPhase.empty());
    const size_t trailSlot = SlotForPhase(rig, kTrail, firstTrailPhase);
    CHECK(trailSlot != size_t(-1));
    const RigExecDemandClosure trailClosure = ClosureFor(rig, {trailSlot});

    std::string error;
    RigExecRequestedRunReport trailReport;
    CHECK(Request(rig, UsdTimeCode::Default(), trailClosure, &trailReport,
                  &error));
    CHECK(trailReport.clustersRun >= 1);
    CHECK(trailReport.executedOperations <= trailClosure.ops.size());
    for (uint32_t op : trailReport.executed)
        CHECK(InClosure(trailClosure, op));
    CHECK(TraceOutside(*rig.program, trailClosure).empty());
    // The deeper links of the same chain were not run by this request.
    const RigExecValueId trailFinal = ProducerValue(rig, kTrail, "final");
    CHECK(trailFinal != kRigExecNoOutputValue);
    CHECK(!FactFor(trailReport, trailFinal));
    const RigExecValueId trailStopped =
        ProducerValue(rig, kTrail, firstTrailPhase);
    CHECK(trailStopped != kRigExecNoOutputValue);
    CHECK(trailStopped != trailFinal);
    const RigExecValueMaterialization *stoppedTrail =
        FactFor(trailReport, trailStopped);
    CHECK(stoppedTrail && stoppedTrail->produced);

    // A request that stops at an intermediate keeps it, unrecomputed, for the
    // run that continues past it.
    const RigExecDemandClosure bodyClosure = ClosureFor(rig, bodySlots);
    RigExecRequestedRunReport report;
    CHECK(Request(rig, UsdTimeCode::Default(), bodyClosure, &report, &error));
    CHECK(report.clustersRun >= 1);
    CHECK(report.executedOperations <= bodyClosure.ops.size());
    for (uint32_t op : report.executed)
        CHECK(InClosure(bodyClosure, op));
    CHECK(TraceOutside(*rig.program, bodyClosure).empty());

    // The retained intermediate the request stopped at survives the run that
    // continues past it: it is read, not recomputed.
    const std::vector<std::string> phases = PhasesFor(rig, kBody);
    std::string firstPhaseName;
    for (const std::string &phase : phases)
        if (phase.find("First") != std::string::npos)
            firstPhaseName = phase;
    CHECK(!firstPhaseName.empty());
    const RigExecValueId firstValue = ProducerValue(rig, kBody, firstPhaseName);
    const RigExecValueMaterialization *stopped = FactFor(report, firstValue);
    CHECK(stopped && stopped->produced);
    const uint64_t stoppedRevision = stopped->revision;

    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    const RigExecDemandClosure deep = ClosureFor(rig, {bodyFinal});
    RigExecRequestedRunReport deeper;
    CHECK(Request(rig, UsdTimeCode::Default(), deep, &deeper, &error));
    const RigExecValueMaterialization *retained = FactFor(deeper, firstValue);
    CHECK(retained && retained->reused && !retained->produced);
    CHECK(retained->revision == stoppedRevision);

    RigExecRigPose pose;
    CHECK(rig.program->Run(UsdTimeCode::Default(), &pose));
    Rig fresh = BuildRig();
    RigExecRigPose whole;
    CHECK(fresh.program->Run(UsdTimeCode::Default(), &whole));
    CHECK(Moved(pose, kBody) == Moved(whole, kBody));
    CHECK(Moved(pose, kOther) == Moved(whole, kOther));
    CHECK(Moved(pose, kAmount) == Moved(whole, kAmount));
    CHECK(Moved(pose, kTrail) == Moved(whole, kTrail));
}

// One requested run at a numeric frame, then the same request at another: the
// frame is an identity, and a moved frame re-runs what reads it.
void
TestRequestedRunAnswersAtItsFrame()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;
    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    CHECK(bodyFinal != size_t(-1));
    const RigExecDemandClosure closure = ClosureFor(rig, {bodyFinal});

    RigExecRequestedRunReport first;
    std::string error;
    CHECK(Request(rig, UsdTimeCode(3.0), closure, &first, &error));
    CHECK(!first.executed.empty());
    RigExecRequestedRunReport again;
    CHECK(Request(rig, UsdTimeCode(3.0), closure, &again, &error));
    // Same frame, same inputs: every value the closure writes now stands.
    for (const RigExecValueMaterialization &fact : again.values)
        CHECK(fact.reused || fact.produced);
    size_t reused = 0;
    for (const RigExecValueMaterialization &fact : again.values)
        reused += fact.reused ? 1 : 0;
    CHECK(reused > 0);

    RigExecRigPose atThree;
    CHECK(rig.program->Run(UsdTimeCode(3.0), &atThree));
    Rig fresh = BuildRig();
    RigExecRigPose wholeAtThree;
    CHECK(fresh.program->Run(UsdTimeCode(3.0), &wholeAtThree));
    CHECK(Moved(atThree, kBody) == Moved(wholeAtThree, kBody));
}

// ---------------------------------------------------------------------------
// Extraction: answering the outputs a requested run materialized.

// A rig with the domains the payload layouts cover: a joint a constraint
// revises and a matrix mover reads at that constraint's phase (so a provider
// matrix and a frame record both exist), a property chain, a chain whose
// target has no authored value (a producer that runs and declines), and two
// meshes.
const char *const kLeg = "/PhaseAsset/Container/Geometry/Leg.points";
const char *const kStale = "/PhaseAsset/Container/Geometry/Stale.points";
const char *const kPhaseRig = "/PhaseAsset/Container/Character";
const char *const kEmptyAmount =
    "/PhaseAsset/Container/Character/EmptyHome.amount";
const char *const kPhaseJoint = "/PhaseAsset/Container/Character/Joints/X";

const char *const kPhaseStageText = R"USDA(#usda 1.0

def Xform "PhaseAsset"
{
    def Scope "Container"
    {
        def RigExecRoot "Character"
        {
            def Scope "Controls"
            {
                def RigExecControl "Lead"
                {
                    double avars:tx = 1
                    double avars:tx.timeSamples = {
                        2: 1,
                        8: 3,
                    }
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                }
            }

            def Scope "Joints"
            {
                def RigExecJoint "X"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0.5, 0, 1) )
                }
            }

            def Scope "PropertyHome"
            {
                float amount = 3

                def RigExecFloatMathMover "AmountDoubler" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </PhaseAsset/Container/Character/PropertyHome.amount>
                }
            }

            def Scope "EmptyHome"
            {
                float amount

                def RigExecFloatMathMover "EmptyDoubler" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    uniform token rigExec:operation = "multiply"
                    float inputs:value = 2
                    rel rigExec:moves = </PhaseAsset/Container/Character/EmptyHome.amount>
                }
            }

            # The last child runs first: Constrain revises X before Readers.
            def Scope "Movers"
            {
                def Scope "Readers"
                {
                    def RigExecMatrixMover "LegReader" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        float inputs:defaultWeight = 1
                        rel rigExec:moves = </PhaseAsset/Container/Geometry/Leg.points>
                        rel rigExec:transform = </PhaseAsset/Container/Character/Joints/X> (
                            rigExecReadPhase = "/PhaseAsset/Container/Character/Movers/Constrain/Hold"
                        )
                    }

                    def RigExecMatrixMover "StaleReader" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        float inputs:defaultWeight = 1
                        rel rigExec:moves = </PhaseAsset/Container/Geometry/Stale.points>
                        rel rigExec:transform = </PhaseAsset/Container/Character/Joints/X>
                    }
                }

                def Scope "Constrain"
                {
                    def RigExecPositionConstraint "Hold" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        rel rigExec:moves = </PhaseAsset/Container/Character/Joints/X>
                        rel rigExec:sources = </PhaseAsset/Container/Character/Controls/Lead>
                    }
                }
            }
        }

        def Scope "Geometry"
        {
            def Points "Leg"
            {
                point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
            }

            # A base with no value at the default frame: a chain that runs at
            # a sampled frame and stands aside at the default one.
            def Points "Stale"
            {
                point3f[] points.timeSamples = {
                    3: [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)],
                }
            }
        }
    }
}
)USDA";

/// Every catalog slot a dispatch can answer: an exported output with a
/// producer, not one the rig reports unsupported from the start.
std::vector<size_t>
AnswerableSlots(const Rig &rig)
{
    std::vector<size_t> slots;
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.status != RigExecOutputStatus::Pending)
            continue;
        size_t slot = 0;
        if (rig.catalog.GetDemandIndex().Find(binding.handle, &slot))
            slots.push_back(slot);
    }
    return slots;
}

std::vector<RigExecOutputHandle>
HandlesForSlots(const Rig &rig, const std::vector<size_t> &slots)
{
    std::vector<RigExecOutputHandle> handles;
    handles.reserve(slots.size());
    for (size_t slot : slots)
        handles.push_back(rig.catalog.GetDemandIndex().Handle(slot));
    return handles;
}

/// The generation a request here belongs to: this catalog's compile epoch as
/// the program identity, with no input, preview or history revision moved.
RigExecOutputGeneration
Captured(const Rig &rig)
{
    RigExecOutputGeneration captured;
    captured.programIdentity = rig.catalog.GetCompileEpoch();
    return captured;
}

/// Runs the closure of \p requestSlots, then answers \p extractSlots from
/// what that run left.
bool
Answer(Rig &rig, UsdTimeCode time, const std::vector<size_t> &requestSlots,
       const std::vector<size_t> &extractSlots,
       std::vector<RigExecOutputPacket> *packets,
       RigExecRequestedRunReport *report, std::string *error)
{
    if (!Request(rig, time, ClosureFor(rig, requestSlots), report, error))
        return false;
    return rig.program->ExtractRequestedOutputs(
        rig.catalog, *report, time, Captured(rig),
        HandlesForSlots(rig, extractSlots), packets, error);
}

const RigExecOutputPacket *
PacketFor(const Rig &rig, const std::vector<RigExecOutputPacket> &packets,
          const std::string &target, const std::string &phase)
{
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.descriptor.target != target ||
            binding.descriptor.phase != phase)
            continue;
        for (const RigExecOutputPacket &packet : packets) {
            if (packet.handle.semanticKey == binding.descriptor.semanticKey)
                return &packet;
        }
    }
    return nullptr;
}

/// The phase-named point versions of \p target: one per mover of its chain.
std::vector<std::string>
PointPhases(const Rig &rig, const std::string &target)
{
    std::vector<std::string> phases;
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.descriptor.target == target &&
            binding.kind == RigExecOutputKind::Points &&
            binding.descriptor.phase != "base" &&
            binding.descriptor.phase != "final")
            phases.push_back(binding.descriptor.phase);
    }
    return phases;
}

bool
ReadScalar(const RigExecOutputPacket &packet, double *out)
{
    RigExecPayloadView view;
    std::string error;
    return view.Open(packet.handle, packet.payload, &error) &&
           view.ReadScalar(0, out);
}

bool
ReadMatrix(const RigExecOutputPacket &packet, std::array<double, 16> *out)
{
    RigExecPayloadView view;
    std::string error;
    return view.Open(packet.handle, packet.payload, &error) &&
           view.ReadMatrix4x4Row(0, out->data());
}

bool
ReadPoints(const RigExecOutputPacket &packet, std::vector<double> *out)
{
    RigExecPayloadView view;
    std::string error;
    if (!view.Open(packet.handle, packet.payload, &error))
        return false;
    out->assign(size_t(view.ElementCount()) * 3, 0.0);
    for (size_t i = 0; i < view.ElementCount(); ++i) {
        if (!view.ReadPoint3(i, out->data() + i * 3))
            return false;
    }
    return true;
}

/// The compiled producer id behind one exported output.
uint64_t
ProducerValueFor(const Rig &rig, const std::string &target,
                 const std::string &phase)
{
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.descriptor.target == target &&
            binding.descriptor.phase == phase)
            return binding.producerValue;
    }
    return 0;
}

/// Every payload the packet holds, as its raw bytes: what a retained lease
/// must keep byte for byte across a later run.
std::vector<uint8_t>
BytesOf(const RigExecOutputPacket &packet)
{
    return packet.payload.bytes ? *packet.payload.bytes
                                : std::vector<uint8_t>{};
}

std::vector<double>
Doubles(const VtVec3fArray &points)
{
    std::vector<double> out;
    out.reserve(points.size() * 3);
    for (const GfVec3f &point : points) {
        out.push_back(double(point[0]));
        out.push_back(double(point[1]));
        out.push_back(double(point[2]));
    }
    return out;
}

std::vector<double>
AuthoredPoints(const UsdStageRefPtr &stage, const std::string &attributePath)
{
    const UsdPrim prim =
        stage->GetPrimAtPath(SdfPath(attributePath).GetPrimPath());
    VtValue value;
    if (prim)
        prim.GetAttribute(UsdGeomTokens->points)
            .Get(&value, UsdTimeCode::Default());
    if (!value.IsHolding<VtVec3fArray>())
        return {};
    return Doubles(value.Get<VtVec3fArray>());
}

bool
SameMatrix(const std::array<double, 16> &payload, const GfMatrix4d &matrix)
{
    for (size_t row = 0; row < 4; ++row)
        for (size_t column = 0; column < 4; ++column)
            if (payload[row * 4 + column] != matrix[row][column])
                return false;
    return true;
}

// Every payload an extraction hands out is the value the same rig publishes
// when it is run whole: scalar, points base, phase and final, joint matrix.
void
TestExtractionEqualsFullEvaluation()
{
    Rig rig = BuildRig();
    Rig fresh = BuildRig();
    if (!rig.program || !fresh.program)
        return;
    const std::vector<size_t> slots = AnswerableSlots(rig);
    CHECK(!slots.empty());

    std::vector<RigExecOutputPacket> packets;
    RigExecRequestedRunReport report;
    std::string error;
    CHECK(Request(rig, UsdTimeCode::Default(), ClosureFor(rig, slots), &report,
                  &error));
    const size_t executed = rig.program->GetStepGraph().opExecution.executed;
    const std::vector<RigExecOutputHandle> handles =
        HandlesForSlots(rig, slots);
    CHECK(rig.program->ExtractRequestedOutputs(
        rig.catalog, report, UsdTimeCode::Default(), Captured(rig), handles,
        &packets, &error));
    CHECK(error.empty());
    CHECK(packets.size() == slots.size());
    // Extraction runs no body: the program's own count of executed
    // operations is exactly what the run left behind.
    CHECK(rig.program->GetStepGraph().opExecution.executed == executed);

    RigExecRigPose pose;
    CHECK(fresh.program->Run(UsdTimeCode::Default(), &pose));
    CHECK(pose.valid);

    size_t ready = 0;
    for (const RigExecOutputPacket &packet : packets) {
        std::string viewError;
        RigExecPayloadView view;
        if (!view.Open(packet.handle, packet.payload, &viewError))
            continue;
        ++ready;
        CHECK(packet.status == RigExecOutputStatus::Ready);
        CHECK(!packet.payload.IsEmpty());
    }
    CHECK(ready == packets.size());

    // Scalar: the double the published property holds.
    double amount = 0;
    const RigExecOutputPacket *scalar =
        PacketFor(rig, packets, kAmount, "final");
    CHECK(scalar && scalar->status == RigExecOutputStatus::Ready);
    CHECK(scalar && ReadScalar(*scalar, &amount));
    const VtValue movedAmount = Moved(pose, kAmount);
    CHECK(movedAmount.IsHolding<float>());
    CHECK(amount == double(movedAmount.Get<float>()));

    // Points: the authored base, and the published final array.
    std::vector<double> authored;
    CHECK(ReadPoints(*PacketFor(rig, packets, kBody, "base"), &authored));
    CHECK(authored == AuthoredPoints(rig.stage, kBody));
    std::vector<double> extracted;
    const RigExecOutputPacket *bodyFinal =
        PacketFor(rig, packets, kBody, "final");
    CHECK(bodyFinal && bodyFinal->status == RigExecOutputStatus::Ready);
    CHECK(bodyFinal && ReadPoints(*bodyFinal, &extracted));
    const VtValue movedBody = Moved(pose, kBody);
    CHECK(movedBody.IsHolding<VtVec3fArray>());
    CHECK(extracted == Doubles(movedBody.Get<VtVec3fArray>()));

    // An intermediate stands between the base and the final: a request that
    // stops at it answers the same value a request that runs past it does.
    const std::vector<std::string> phases = PointPhases(rig, kBody);
    CHECK(!phases.empty());
    for (const std::string &phase : phases) {
        const size_t slot = SlotForPhase(rig, kBody, phase);
        CHECK(slot != size_t(-1));
        std::vector<RigExecOutputPacket> stopped;
        RigExecRequestedRunReport stoppedReport;
        CHECK(Answer(rig, UsdTimeCode::Default(), {slot}, {slot}, &stopped,
                     &stoppedReport, &error));
        std::vector<double> stoppedPoints;
        CHECK(!stopped.empty() &&
              stopped.front().status == RigExecOutputStatus::Ready);
        CHECK(ReadPoints(stopped.front(), &stoppedPoints));
        CHECK(stoppedPoints.size() == authored.size());
        CHECK(stoppedPoints != authored);

        Rig deep = BuildRig();
        std::vector<RigExecOutputPacket> deeper;
        RigExecRequestedRunReport deeperReport;
        CHECK(Answer(deep, UsdTimeCode::Default(), {slot}, {slot}, &deeper,
                     &deeperReport, &error));
        std::vector<double> deeperPoints;
        CHECK(ReadPoints(deeper.front(), &deeperPoints));
        CHECK(deeperPoints == stoppedPoints);
    }
}

// A joint's matrix, its base matrix, and the frame record a phase read
// leaves: each payload is the matrix the pose publishes for the same run.
void
TestTransformExtractionMatchesThePublishedJoint()
{
    Rig rig = BuildRigFrom(kPhaseStageText, kPhaseRig);
    Rig fresh = BuildRigFrom(kPhaseStageText, kPhaseRig);
    if (!rig.program || !fresh.program)
        return;
    const UsdTimeCode time(2.0);
    const std::vector<size_t> slots = AnswerableSlots(rig);
    CHECK(!slots.empty());

    std::vector<RigExecOutputPacket> packets;
    RigExecRequestedRunReport report;
    std::string error;
    CHECK(Answer(rig, time, slots, slots, &packets, &report, &error));
    CHECK(error.empty());

    RigExecRigPose pose;
    CHECK(fresh.program->Run(time, &pose));
    CHECK(pose.valid);
    const auto published = pose.jointMatricesFinal.find(SdfPath(kPhaseJoint));
    CHECK(published != pose.jointMatricesFinal.end());

    const RigExecOutputPacket *finalMatrix =
        PacketFor(rig, packets, kPhaseJoint, "final");
    CHECK(finalMatrix && finalMatrix->status == RigExecOutputStatus::Ready);
    std::array<double, 16> cells{};
    CHECK(ReadMatrix(*finalMatrix, &cells));
    CHECK(SameMatrix(cells, published->second));
    // A moved joint is not an untouched slot read back as zeros, so the
    // comparison above is not an agreement between two empty payloads.
    const std::array<double, 16> zeros{};
    CHECK(cells != zeros);

    // The base the phase read is the joint as it stands before the walk:
    // nothing revised it, so it is its own rest matrix.
    const RigExecOutputPacket *baseMatrix =
        PacketFor(rig, packets, kPhaseJoint, "base");
    CHECK(baseMatrix);
    if (baseMatrix) {
        CHECK(baseMatrix->status == RigExecOutputStatus::Ready);
        std::array<double, 16> baseCells{};
        CHECK(ReadMatrix(*baseMatrix, &baseCells));
        CHECK(SameMatrix(baseCells, GfMatrix4d(1.0)));
    }

    // The frame record the phase read names: a matrix for the writer it
    // stands on, and with one constraint revising the joint it is the
    // matrix the joint publishes.
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.kind != RigExecOutputKind::Transform ||
            binding.descriptor.target != kPhaseJoint ||
            binding.descriptor.phase == "base" ||
            binding.descriptor.phase == "final")
            continue;
        const RigExecOutputPacket *record = PacketFor(
            rig, packets, binding.descriptor.target, binding.descriptor.phase);
        CHECK(record && record->status == RigExecOutputStatus::Ready);
        if (!record)
            continue;
        std::array<double, 16> recordCells{};
        CHECK(ReadMatrix(*record, &recordCells));
        CHECK(SameMatrix(recordCells, published->second));
    }

    // The mesh whose mover reads the joint at the constraint's phase follows
    // the joint it read, not the joint as it stands.
    const RigExecOutputPacket *legFinal =
        PacketFor(rig, packets, kLeg, "final");
    CHECK(legFinal && legFinal->status == RigExecOutputStatus::Ready);
    std::vector<double> legBase;
    CHECK(ReadPoints(*PacketFor(rig, packets, kLeg, "base"), &legBase));
    CHECK(legBase == AuthoredPoints(rig.stage, kLeg));
    std::vector<double> legMoved;
    CHECK(ReadPoints(*legFinal, &legMoved));
    const VtValue movedLeg = Moved(pose, kLeg);
    CHECK(movedLeg.IsHolding<VtVec3fArray>());
    CHECK(legMoved == Doubles(movedLeg.Get<VtVec3fArray>()));
    CHECK(legMoved != legBase);

    // The full-pose epilogue belongs to a whole request: a requested run and
    // its extraction publish no pose, and the whole run that follows is the
    // first thing to write the epilogue's publication table.
    for (char publishedFlag : rig.program->GetStepGraph().jointMatrixPublished)
        CHECK(publishedFlag == 0);
    RigExecRigPose second;
    CHECK(rig.program->Run(time, &second));
    bool anyPublished = false;
    for (char publishedFlag : rig.program->GetStepGraph().jointMatrixPublished)
        anyPublished = anyPublished || publishedFlag != 0;
    CHECK(anyPublished);
    CHECK(second.jointMatricesFinal == pose.jointMatricesFinal);
}

// An output the run did not materialize, an address the rig does not export,
// and a stale compile epoch each answer with a status and a reason, and with
// no payload -- never with a Ready payload of zeros.
void
TestAnOutputItCannotAnswerHasAStatus()
{
    Rig rig = BuildRig();
    if (!rig.program)
        return;
    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    const size_t otherFinal = SlotForPhase(rig, kOther, "final");
    CHECK(bodyFinal != size_t(-1));
    CHECK(otherFinal != size_t(-1));

    // The Body chain is requested; Other is asked about. Its producer stands
    // outside the closure, so its storage is not this request's answer.
    std::vector<RigExecOutputPacket> packets;
    RigExecRequestedRunReport report;
    std::string error;
    CHECK(Answer(rig, UsdTimeCode::Default(), {bodyFinal},
                 {bodyFinal, otherFinal}, &packets, &report, &error));
    CHECK(packets.size() == 2);
    const RigExecOutputPacket *asked = PacketFor(rig, packets, kOther, "final");
    CHECK(asked && asked->status == RigExecOutputStatus::Pending);
    CHECK(asked && asked->payload.IsEmpty());
    CHECK(asked && !asked->reason.empty());
    if (asked) {
        RigExecPayloadView view;
        std::string viewError;
        CHECK(!view.Open(asked->handle, asked->payload, &viewError));
    }
    const RigExecOutputPacket *answered =
        PacketFor(rig, packets, kBody, "final");
    CHECK(answered && answered->status == RigExecOutputStatus::Ready);

    // An address this rig does not export, and a handle from an earlier
    // compile epoch: resolved before any read, with no payload.
    RigExecOutputHandle unknown = rig.catalog.GetBindings().front().handle;
    unknown.semanticKey = "9:not-an-output";
    RigExecOutputHandle stale = rig.catalog.GetBindings().front().handle;
    stale.compileEpoch = rig.catalog.GetCompileEpoch() + 1;
    packets.clear();
    CHECK(rig.program->ExtractRequestedOutputs(
        rig.catalog, report, UsdTimeCode::Default(), Captured(rig),
        {unknown, stale}, &packets, &error));
    CHECK(packets.size() == 2);
    CHECK(packets[0].status == RigExecOutputStatus::Unsupported);
    CHECK(packets[1].status == RigExecOutputStatus::Invalidated);
    for (const RigExecOutputPacket &packet : packets) {
        CHECK(packet.payload.IsEmpty());
        CHECK(!packet.reason.empty());
    }

    // A generation from another compiled program is never labeled current.
    RigExecOutputGeneration foreign = Captured(rig);
    foreign.programIdentity = rig.catalog.GetCompileEpoch() + 7;
    packets.clear();
    CHECK(rig.program->ExtractRequestedOutputs(
        rig.catalog, report, UsdTimeCode::Default(), foreign,
        {rig.catalog.GetDemandIndex().Handle(bodyFinal)}, &packets, &error));
    CHECK(packets.size() == 1);
    CHECK(packets[0].status == RigExecOutputStatus::Invalidated);
    CHECK(packets[0].payload.IsEmpty());

    // A revision the run no longer holds is a different answer.
    RigExecOutputHandle ahead = rig.catalog.GetDemandIndex().Handle(bodyFinal);
    ahead.valueRevision = uint64_t(-1);
    packets.clear();
    CHECK(rig.program->ExtractRequestedOutputs(
        rig.catalog, report, UsdTimeCode::Default(), Captured(rig), {ahead},
        &packets, &error));
    CHECK(packets.size() == 1);
    CHECK(packets[0].status == RigExecOutputStatus::Invalidated);
    CHECK(packets[0].payload.IsEmpty());

    // A producer that ran and declined publishes no value: the rig itself
    // publishes nothing for it, so a Ready zero would be an invention.
    Rig phase = BuildRigFrom(kPhaseStageText, kPhaseRig);
    if (!phase.program)
        return;
    Rig phaseFresh = BuildRigFrom(kPhaseStageText, kPhaseRig);
    RigExecRigPose whole;
    CHECK(phaseFresh.program->Run(UsdTimeCode(2.0), &whole));
    CHECK(whole.movedProperties.find(SdfPath(kEmptyAmount)) ==
          whole.movedProperties.end());
    const size_t emptyFinal = SlotForPhase(phase, kEmptyAmount, "final");
    CHECK(emptyFinal != size_t(-1));
    if (emptyFinal != size_t(-1)) {
        std::vector<RigExecOutputPacket> declined;
        RigExecRequestedRunReport declinedReport;
        CHECK(Answer(phase, UsdTimeCode(2.0), {emptyFinal}, {emptyFinal},
                     &declined, &declinedReport, &error));
        CHECK(declined.size() == 1);
        CHECK(declined.front().status == RigExecOutputStatus::Failed);
        CHECK(declined.front().payload.IsEmpty());
        CHECK(!declined.front().reason.empty());
    }
}

// A payload lease is immutable and a retained intermediate is read, not
// recomputed: coarsen the clusters so a request stops partway down a chain,
// take the intermediate, then run past it, publish a whole pose, and move to
// another frame.
void
TestRetainedPayloadsStandAcrossRuns()
{
    TfSetenv("RIGEXEC_BAKED_GRAIN_US", "1000000");
    Rig rig = BuildRig();
    TfUnsetenv("RIGEXEC_BAKED_GRAIN_US");
    if (!rig.program)
        return;
    const std::vector<std::string> phases = PointPhases(rig, kBody);
    CHECK(!phases.empty());
    const size_t firstPhase = SlotForPhase(rig, kBody, phases.front());
    const size_t bodyFinal = SlotForPhase(rig, kBody, "final");
    const size_t amountFinal = SlotForPhase(rig, kAmount, "final");
    CHECK(firstPhase != size_t(-1));
    CHECK(bodyFinal != size_t(-1));
    CHECK(amountFinal != size_t(-1));

    std::vector<RigExecOutputPacket> taken;
    RigExecRequestedRunReport takenReport;
    std::string error;
    CHECK(Answer(rig, UsdTimeCode(3.0), {firstPhase, amountFinal},
                 {firstPhase, amountFinal}, &taken, &takenReport, &error));
    CHECK(taken.size() == 2);
    for (const RigExecOutputPacket &packet : taken)
        CHECK(packet.status == RigExecOutputStatus::Ready);
    const std::vector<uint8_t> heldPoints = BytesOf(taken.front());
    const std::vector<uint8_t> heldScalar = BytesOf(taken.back());
    CHECK(!heldPoints.empty());
    CHECK(!heldScalar.empty());
    const uint64_t pointsRevision = taken.front().handle.valueRevision;

    // A request that runs past the intermediate in the same arena reads it:
    // the same value, the same revision, the same bytes.
    std::vector<RigExecOutputPacket> deeper;
    RigExecRequestedRunReport deeperReport;
    CHECK(Answer(rig, UsdTimeCode(3.0), {bodyFinal}, {firstPhase}, &deeper,
                 &deeperReport, &error));
    CHECK(deeper.size() == 1);
    CHECK(deeper.front().status == RigExecOutputStatus::Ready);
    CHECK(deeper.front().handle.valueRevision == pointsRevision);
    CHECK(BytesOf(deeper.front()) == heldPoints);
    const RigExecValueMaterialization *retained =
        FactFor(deeperReport, ProducerValueFor(rig, kBody, phases.front()));
    CHECK(retained && retained->reused && !retained->produced);

    // A whole pose published at the same frame, then a run at another one:
    // the leases taken first still hold the bytes they were handed.
    RigExecRigPose pose;
    CHECK(rig.program->Run(UsdTimeCode(3.0), &pose));
    RigExecRigPose other;
    CHECK(rig.program->Run(UsdTimeCode(8.0), &other));
    const VtValue movedThree = Moved(pose, kBody);
    const VtValue movedEight = Moved(other, kBody);
    CHECK(movedThree.IsHolding<VtVec3fArray>() &&
          movedEight.IsHolding<VtVec3fArray>());
    CHECK(Doubles(movedThree.Get<VtVec3fArray>()) !=
          Doubles(movedEight.Get<VtVec3fArray>()));
    CHECK(BytesOf(taken.front()) == heldPoints);
    CHECK(BytesOf(taken.back()) == heldScalar);

    std::vector<RigExecOutputPacket> atEight;
    RigExecRequestedRunReport atEightReport;
    CHECK(Answer(rig, UsdTimeCode(8.0), {bodyFinal}, {firstPhase}, &atEight,
                 &atEightReport, &error));
    CHECK(atEight.size() == 1);
    CHECK(atEight.front().status == RigExecOutputStatus::Ready);
    CHECK(BytesOf(atEight.front()) != heldPoints);
    CHECK(BytesOf(taken.front()) == heldPoints);
}

// A chain that read a base at one frame and none at the next publishes
// nothing at the second, and its previous points are not an answer: the
// extraction says so, and the lease taken at the first frame still holds.
void
TestNoBaseAtTheFrameAnswersWithAStatus()
{
    Rig rig = BuildRigFrom(kPhaseStageText, kPhaseRig);
    if (!rig.program)
        return;
    const size_t staleFinal = SlotForPhase(rig, kStale, "final");
    CHECK(staleFinal != size_t(-1));
    if (staleFinal == size_t(-1))
        return;

    const UsdTimeCode sampled(3.0);
    RigExecRigPose wholeSampled;
    CHECK(rig.program->Run(sampled, &wholeSampled));
    CHECK(wholeSampled.movedProperties.find(SdfPath(kStale)) !=
          wholeSampled.movedProperties.end());

    std::vector<RigExecOutputPacket> atSampled;
    RigExecRequestedRunReport sampledReport;
    std::string error;
    CHECK(Answer(rig, sampled, {staleFinal}, {staleFinal}, &atSampled,
                 &sampledReport, &error));
    CHECK(atSampled.size() == 1);
    CHECK(atSampled.front().status == RigExecOutputStatus::Ready);
    std::vector<double> points;
    CHECK(ReadPoints(atSampled.front(), &points));
    const VtValue movedStale = Moved(wholeSampled, kStale);
    CHECK(movedStale.IsHolding<VtVec3fArray>());
    CHECK(points == Doubles(movedStale.Get<VtVec3fArray>()));
    const std::vector<uint8_t> held = BytesOf(atSampled.front());

    // The default frame has no value for that mesh at all. The whole run
    // publishes nothing for it, and the extraction refuses to answer with
    // the points the chain still holds from the sampled frame.
    RigExecRigPose wholeDefault;
    CHECK(rig.program->Run(UsdTimeCode::Default(), &wholeDefault));
    CHECK(wholeDefault.movedProperties.find(SdfPath(kStale)) ==
          wholeDefault.movedProperties.end());
    std::vector<RigExecOutputPacket> atDefault;
    RigExecRequestedRunReport defaultReport;
    CHECK(Answer(rig, UsdTimeCode::Default(), {staleFinal}, {staleFinal},
                 &atDefault, &defaultReport, &error));
    CHECK(atDefault.size() == 1);
    CHECK(atDefault.front().status == RigExecOutputStatus::Failed);
    CHECK(atDefault.front().payload.IsEmpty());
    CHECK(!atDefault.front().reason.empty());
    std::vector<size_t> staleSlots;
    for (const RigExecOutputBinding &binding : rig.catalog.GetBindings()) {
        if (binding.descriptor.target != kStale ||
            binding.kind != RigExecOutputKind::Points)
            continue;
        size_t slot = 0;
        if (rig.catalog.GetDemandIndex().Find(binding.handle, &slot))
            staleSlots.push_back(slot);
    }
    CHECK(staleSlots.size() >= 2);
    std::vector<RigExecOutputPacket> refused;
    RigExecRequestedRunReport refusedReport;
    CHECK(Request(rig, UsdTimeCode::Default(), ClosureFor(rig, staleSlots),
                  &refusedReport, &error));
    CHECK(rig.program->ExtractRequestedOutputs(
        rig.catalog, refusedReport, UsdTimeCode::Default(), Captured(rig),
        HandlesForSlots(rig, staleSlots), &refused, &error));
    CHECK(refused.size() == staleSlots.size());
    for (const RigExecOutputPacket &packet : refused) {
        CHECK(packet.status == RigExecOutputStatus::Failed);
        CHECK(packet.payload.IsEmpty());
        CHECK(!packet.reason.empty());
    }
    CHECK(BytesOf(atSampled.front()) == held);
}

/// A catalog from another compilation, however alike the two graphs look,
/// cannot answer this program's values.
void
TestForeignCatalogCannotAnswerThisProgram()
{
    Rig rig = BuildRig();
    Rig twin = BuildRig();
    if (!rig.program || !twin.program)
        return;

    // Two independent compilations of the same stage: the same operation
    // count, the same semantic keys, and no shared compiled-program identity.
    CHECK(rig.catalog.GetDemandIndex().OperationCount() ==
          twin.catalog.GetDemandIndex().OperationCount());
    CHECK(rig.catalog.GetProgramAffinity());
    CHECK(rig.catalog.GetProgramAffinity() !=
          twin.catalog.GetProgramAffinity());
    CHECK(rig.program->GetStepGraph().programAffinity ==
          rig.catalog.GetProgramAffinity());

    const size_t amount = SlotForPhase(rig, kAmount, "final");
    const size_t twinAmount = SlotForPhase(twin, kAmount, "final");
    if (amount == size_t(-1) || twinAmount == size_t(-1))
        return;

    RigExecRequestedRunReport report;
    std::string error;
    CHECK(Request(rig, UsdTimeCode::Default(), ClosureFor(rig, {amount}),
                  &report, &error));

    // The twin's catalog, its handles, its captured epoch, and this program's
    // run: the request is refused, so no byte computed here is ever labeled
    // with the twin's rig identity.
    std::vector<RigExecOutputPacket> foreign;
    CHECK(!rig.program->ExtractRequestedOutputs(
        twin.catalog, report, UsdTimeCode::Default(), Captured(twin),
        HandlesForSlots(twin, {twinAmount}), &foreign, &error));
    CHECK(foreign.empty());
    CHECK(error.find("not this program's compiled graph") != std::string::npos);

    // Its own catalog answers the same output.
    std::vector<RigExecOutputPacket> own;
    CHECK(rig.program->ExtractRequestedOutputs(
        rig.catalog, report, UsdTimeCode::Default(), Captured(rig),
        HandlesForSlots(rig, {amount}), &own, &error));
    CHECK(own.size() == 1);
    if (!own.empty()) {
        CHECK(own.front().status == RigExecOutputStatus::Ready);
        CHECK(own.front().handle.rigIdentity == "requested-rig");
    }
}

} // namespace

int
main()
{
    TestRequestRunsOnlyItsClosure();
    TestUnionSharesAncestorsOnce();
    TestPartialExpansionComputesMissingCleanInputs();
    TestPartialToFullMatchesAFreshFullRun();
    TestRefusesAnOpenClosure();
    TestClusterBoundariesHoldTheMask();
    TestRequestedRunAnswersAtItsFrame();
    TestExtractionEqualsFullEvaluation();
    TestTransformExtractionMatchesThePublishedJoint();
    TestAnOutputItCannotAnswerHasAStatus();
    TestNoBaseAtTheFrameAnswersWithAStatus();
    TestRetainedPayloadsStandAcrossRuns();
    TestForeignCatalogCannotAnswerThisProgram();
    if (failures)
        std::printf("%d failures\n", failures);
    return failures ? 1 : 0;
}
