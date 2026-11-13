// Requested execution: run the compiled graph over the closure of the outputs
// a host asked for, and report what each value that closure writes did
// (docs/specs/output-subscriptions.md). Typed payload extraction is a separate
// step; what this suite holds to account is the run itself -- that a request
// executes nothing outside its closure, that a union shares an ancestor once,
// that a value never materialized is computed even when its inputs are clean,
// that an operation a request left out keeps its debt, and that a sequence of
// partial runs lands on the same answer as one whole run.
#include "rigExec/outputCatalog.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecGraph/outputDemand.h"
#include "rigExecOpTrace.h"

#include "pxr/base/tf/setenv.h"

#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
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
BuildRig()
{
    Rig rig;
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer->ImportFromString(kStageText));
    rig.stage = UsdStage::Open(layer);
    CHECK(rig.stage);
    rig.evaluator = std::make_unique<RigExecRigEvaluator>(
        rig.stage, SdfPath("/Asset/Container/Character"));
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
    if (failures)
        std::printf("%d failures\n", failures);
    return failures ? 1 : 0;
}
