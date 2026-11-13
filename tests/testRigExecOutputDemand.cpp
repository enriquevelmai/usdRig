// The output demand contract: exact-frame identity, the canonical producer of
// each requested output, and the operation closure one dispatch must run.
// Compiled with rigExecGraph alone - the contract reads no stage.
#include "rigExecGraph/outputDemand.h"

#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

RigExecOpDescriptor
Op(const char *key, std::vector<RigExecValueId> reads,
   std::vector<RigExecValueId> writes)
{
    RigExecOpDescriptor op;
    op.key = key;
    op.reads = std::move(reads);
    op.writes = std::move(writes);
    return op;
}

// A.source -> B.left, C.right -> D.join, plus U.unrelated on its own branch.
// Canonical order is A, B, C, D, U.
RigExecCompiledGraph
DiamondGraph()
{
    RigExecCompiledGraph graph;
    CHECK(RigExecCompileOpGraph(
        {Op("A.source", {0}, {1}), Op("B.left", {1}, {2}),
         Op("C.right", {1}, {3}), Op("D.join", {2, 3}, {4}),
         Op("U.unrelated", {5}, {6})},
        {0, 5}, RigExecCyclePolicy::Reject, &graph));
    return graph;
}

RigExecOutputHandle
Handle(const char *key, uint64_t revision = 0)
{
    RigExecOutputHandle handle;
    handle.rigIdentity = "rig/hero";
    handle.compileEpoch = 1;
    handle.semanticKey = key;
    handle.type = "mat4x4row";
    handle.readPhase = "final";
    handle.valueRevision = revision;
    return handle;
}

RigExecDemandEdges
DiamondEdges(size_t count)
{
    RigExecDemandEdges edges;
    edges.validity.assign(count, {});
    edges.effects.assign(count, {});
    edges.history.assign(count, {});
    // B.left's read of the U branch is meaningful only after U ran; C.right
    // reads state U's skip would have changed; D.join needs U's last tick.
    edges.validity[1] = {4};
    edges.effects[2] = {4};
    edges.history[3] = {4};
    return edges;
}

// leftPose comes from B.left, jointTransforms from D.join, driverField from
// U.unrelated, and ghostField from a value nothing writes.
RigExecOutputDemandIndex
BuildIndex(const RigExecCompiledGraph &graph)
{
    std::vector<RigExecOutputDemandEntry> entries{
        {Handle("leftPose"), RigExecValueId(2)},
        {Handle("jointTransforms"), RigExecValueId(4)},
        {Handle("driverField"), RigExecValueId(6)},
        {Handle("ghostField"), RigExecValueId(99)}};
    RigExecOutputDemandIndex index;
    std::string error;
    CHECK(index.Build(graph, DiamondEdges(graph.ops.size()), entries, &error));
    if (!error.empty())
        std::printf("FAIL build: %s\n", error.c_str());
    return index;
}

std::vector<uint32_t>
MaskedOps(const std::vector<char> &mask)
{
    std::vector<uint32_t> ops;
    for (uint32_t i = 0; i < mask.size(); ++i)
        if (mask[i])
            ops.push_back(i);
    return ops;
}

void
TestOutputTimeIdentity()
{
    const RigExecOutputTime def = RigExecOutputTime::Default();
    CHECK(def.IsDefault());
    RigExecOutputTime zero;
    CHECK(RigExecOutputTime::TryNumeric(0.0, &zero));
    CHECK(!zero.IsDefault() && zero.Seconds() == 0.0);
    // The default frame is an identity, not the number zero.
    CHECK(def != zero);
    CHECK(def.Hash() != zero.Hash());

    // Negative zero is the same frame as zero, so equal times hash alike.
    RigExecOutputTime negZero;
    CHECK(RigExecOutputTime::TryNumeric(-0.0, &negZero));
    CHECK(negZero == zero && negZero.Hash() == zero.Hash());

    // Subframe and negative offsets survive exactly; the default sorts before
    // every numeric frame, so grouping keeps it apart from frame zero.
    RigExecOutputTime subframe, beforeStart;
    CHECK(RigExecOutputTime::TryNumeric(1.0 / 3.0, &subframe));
    CHECK(subframe.Seconds() == 1.0 / 3.0);
    CHECK(RigExecOutputTime::TryNumeric(-1.0 / 3.0, &beforeStart));
    CHECK(beforeStart.Seconds() == -1.0 / 3.0);
    CHECK(def < beforeStart && beforeStart < zero && zero < subframe);
    CHECK(subframe != beforeStart);
    CHECK(subframe.Seconds() != 0.0);

    // An invalid time is refused and the destination is left untouched.
    RigExecOutputTime keep = RigExecOutputTime::Default();
    CHECK(!RigExecOutputTime::TryNumeric(
        std::numeric_limits<double>::quiet_NaN(), &keep));
    CHECK(keep.IsDefault());
    CHECK(!RigExecOutputTime::TryNumeric(
        std::numeric_limits<double>::infinity(), &keep));
    CHECK(!RigExecOutputTime::TryNumeric(
        -std::numeric_limits<double>::infinity(), &keep));
    CHECK(RigExecOutputTime::TryNumeric(2.5, &keep) && keep.Seconds() == 2.5);

    std::vector<RigExecOutputTime> frames{def,     def,      zero,
                                          negZero, subframe, beforeStart};
    std::sort(frames.begin(), frames.end());
    CHECK(frames[0].IsDefault() && frames[1].IsDefault());
    CHECK(frames[2] == beforeStart && frames[3] == zero &&
          frames[4] == negZero && frames[5] == subframe);
}

void
TestHandleIdentity()
{
    const RigExecOutputHandle base = Handle("jointTransforms", 1);
    RigExecOutputHandle newer = base;
    newer.valueRevision = 2;
    // A new revision is the same output, so a lookup by target still finds it.
    CHECK(base.SameTarget(newer) && !(base == newer));
    CHECK(base.Hash() == newer.Hash());
    RigExecOutputHandle otherPhase = base;
    otherPhase.readPhase = "base";
    CHECK(!base.SameTarget(otherPhase));
    RigExecOutputHandle otherRig = base;
    otherRig.compileEpoch = 2;
    CHECK(!base.SameTarget(otherRig));
}

void
TestWellFormedRequest()
{
    std::string error;
    RigExecOutputRequest request;
    request.handles = {Handle("leftPose")};
    CHECK(request.IsWellFormed(&error) && error.empty());
    // The exact frame set: the default, zero, and a negative subframe offset.
    RigExecOutputTime subframe;
    CHECK(RigExecOutputTime::TryNumeric(-1.0 / 3.0, &subframe));
    request.times = {RigExecOutputTime::Default(), RigExecOutputTime::Default(),
                     subframe};
    CHECK(request.IsWellFormed(&error));

    CHECK(!RigExecOutputRequest{}.IsWellFormed(&error));
    CHECK(error == "output request has no handles");

    RigExecOutputHandle missingType = Handle("leftPose");
    missingType.type.clear();
    RigExecOutputRequest badHandle;
    badHandle.handles = {missingType};
    CHECK(!badHandle.IsWellFormed(&error));
    CHECK(error.find("payload type") != std::string::npos);

    RigExecOutputRequest invalidTime;
    invalidTime.handles = {Handle("leftPose")};
    invalidTime.times = {RigExecOutputTime::Default(),
                         RigExecOutputTime::Default()};
    // A refused time never enters a request, so the request stays well formed.
    RigExecOutputTime untouched;
    CHECK(!RigExecOutputTime::TryNumeric(
        std::numeric_limits<double>::quiet_NaN(), &untouched));
    CHECK(untouched.IsDefault());
    CHECK(invalidTime.IsWellFormed(&error));
}

void
TestUnionOfRequests()
{
    RigExecOutputRequest a;
    a.handles = {Handle("leftPose", 1), Handle("jointTransforms")};
    a.times = {RigExecOutputTime::Default(), RigExecOutputTime::Default()};
    a.policy = RigExecDemandPolicy::Explicit;
    RigExecOutputRequest b;
    b.handles = {Handle("leftPose", 2), Handle("driverField")};
    b.times = {RigExecOutputTime::Default()};
    b.policy = RigExecDemandPolicy::Speculative;
    CHECK(RigExecRequestsCompatible(a, b));

    const RigExecOutputRequest merged = RigExecUnionRequests({a, b});
    CHECK(merged.policy == RigExecDemandPolicy::Explicit);
    CHECK(merged.handles.size() == 3);
    CHECK(merged.times.size() == 1);
    // Two speculative requests stay speculative.
    CHECK(RigExecUnionRequests({b, b}).policy ==
          RigExecDemandPolicy::Speculative);
    RigExecOutputRequest otherRig = b;
    otherRig.handles.front().rigIdentity = "rig/other";
    CHECK(!RigExecRequestsCompatible(a, otherRig));
}

void
TestProducersAndClosures()
{
    const RigExecCompiledGraph graph = DiamondGraph();
    const RigExecOutputDemandIndex index = BuildIndex(graph);
    CHECK(index.Size() == 4);
    CHECK(index.OperationCount() == graph.ops.size());

    CHECK(index.Producer(0) == 1); // B.left
    CHECK(index.Producer(1) == 3); // D.join
    CHECK(index.Producer(2) == 4); // U.unrelated
    CHECK(index.Producer(3) == kRigExecNoOp);
    CHECK(index.ProducerStatus(3) == RigExecOutputStatus::Unsupported);
    CHECK(index.ProducerStatus(0) == RigExecOutputStatus::Pending);

    size_t slot = 0;
    CHECK(index.Find(Handle("jointTransforms"), &slot) && slot == 1);
    CHECK(index.Find(Handle("jointTransforms", 7), &slot) && slot == 1);
    CHECK(!index.Find(Handle("notAnOutput"), &slot));

    // leftPose: B.left, its graph predecessor A.source, and U pulled in by
    // the declared validity edge.
    const RigExecDemandClosure &left = index.Closure(0);
    CHECK(left.ops == std::vector<uint32_t>({0, 1, 4}));
    CHECK(left.KindOf(1) == RigExecRequirementKind::Predecessor);
    CHECK(left.KindOf(0) == RigExecRequirementKind::Predecessor);
    CHECK(left.KindOf(4) == RigExecRequirementKind::Validity);
    CHECK(left.Contains(4) && !left.Contains(3));

    // jointTransforms: D.join, B.left, C.right, A.source, and U as history.
    const RigExecDemandClosure &joined = index.Closure(1);
    CHECK(joined.ops == std::vector<uint32_t>({0, 1, 2, 3, 4}));
    CHECK(joined.KindOf(3) == RigExecRequirementKind::Predecessor);
    CHECK(joined.KindOf(4) == RigExecRequirementKind::History);

    CHECK(index.Closure(2).ops == std::vector<uint32_t>({4}));
    CHECK(index.Closure(3).Empty());

    std::string error;
    CHECK(RigExecClosureIsClosed(graph, left) &&
          RigExecClosureIsClosed(graph, joined));
    RigExecDemandClosure broken;
    broken.ops = {1, 3};
    broken.kinds = {RigExecRequirementKind::Predecessor,
                    RigExecRequirementKind::Predecessor};
    CHECK(!RigExecClosureIsClosed(graph, broken, &error));
    CHECK(error.find("not predecessor closed") != std::string::npos);

    // The shared ancestor A.source is demanded by leftPose and
    // jointTransforms, and by nothing else.
    CHECK(index.DemandedBy(0) == std::vector<uint32_t>({0, 1}));
    CHECK(index.DemandedBy(1) == std::vector<uint32_t>({0, 1}));
    CHECK(index.DemandedBy(2) == std::vector<uint32_t>({1}));
    CHECK(index.DemandedBy(4) == std::vector<uint32_t>({0, 1, 2}));
    CHECK(index.DemandedBy(9).empty());
}

void
TestUnionRunsSharedAncestorOnce()
{
    const RigExecCompiledGraph graph = DiamondGraph();
    const RigExecOutputDemandIndex index = BuildIndex(graph);
    const RigExecDemandClosure merged = index.Union({0, 1});
    CHECK(merged.ops == std::vector<uint32_t>({0, 1, 2, 3, 4}));
    CHECK(merged.ops.size() == merged.kinds.size());
    CHECK(index.Union({}).ops.empty());
    CHECK(index.Union({0, 0, 99}).ops == std::vector<uint32_t>({0, 1, 4}));

    std::vector<char> changedValues(7, 0);
    std::vector<uint32_t> ran, skipped;
    RigExecOpCallbacks callbacks;
    callbacks.run = [&](uint32_t i) {
        ran.push_back(i);
        for (RigExecValueId value : graph.ops[i].descriptor.writes)
            changedValues[value] = 1;
        return true;
    };
    callbacks.skip = [&](uint32_t i) { skipped.push_back(i); };
    callbacks.changed = [&](RigExecValueId value) {
        return changedValues[value] != 0;
    };

    // Both closures, then leftPose's closure alone: the mask is what the
    // executor runs, and the ops it marks 0 never appear.
    const auto dispatch = [&](const std::vector<char> &mask) {
        changedValues.assign(7, 0);
        changedValues[0] = 1;
        changedValues[5] = 1;
        ran.clear();
        skipped.clear();
        const std::vector<uint32_t> candidates = MaskedOps(mask);
        callbacks.active = &mask;
        RigExecOpExecution execution;
        std::string error;
        CHECK(RigExecExecuteOpGraph(graph, {0, 5}, {}, false, callbacks,
                                    &execution, &error, nullptr, &candidates));
        if (!error.empty())
            std::printf("FAIL dispatch: %s\n", error.c_str());
        // Ops outside the mask are never candidates; an unmarked successor
        // may still be scheduled as a potential reader, but its body never
        // runs and it never reaches the run callback.
        CHECK(execution.executed == candidates.size());
        CHECK(ran.size() == candidates.size());
        for (uint32_t op = 0; op < mask.size(); ++op)
            CHECK(mask[op] ? execution.ran[op] != 0 : execution.ran[op] == 0);
        // The executor's finish order is not part of the contract; the set of
        // operations it ran is, and each appears exactly once.
        std::sort(ran.begin(), ran.end());
        return ran;
    };

    // The two closures share A.source, B.left and U, so one dispatch runs each
    // of them once; the union covers the whole graph.
    CHECK(index.Union({0, 1}).ops == std::vector<uint32_t>({0, 1, 2, 3, 4}));
    CHECK(index.ActiveMask({0, 1}) == std::vector<char>(graph.ops.size(), 1));
    CHECK(dispatch(index.ActiveMask({0, 1})) ==
          std::vector<uint32_t>({0, 1, 2, 3, 4}));
    // leftPose alone leaves C.right and D.join unmarked, so they never appear.
    const std::vector<char> leftOnly = index.ActiveMask({0});
    CHECK(leftOnly == std::vector<char>({1, 1, 0, 0, 1}));
    CHECK(dispatch(leftOnly) == std::vector<uint32_t>({0, 1, 4}));
    // An unsupported output demands nothing.
    CHECK(index.ActiveMask({3}) == std::vector<char>(graph.ops.size(), 0));
}

void
TestIndexRefusals()
{
    const RigExecCompiledGraph graph = DiamondGraph();
    std::string error;
    RigExecOutputDemandIndex index;

    RigExecDemandEdges shortEdges;
    shortEdges.validity = {{0}};
    CHECK(!index.Build(graph, shortEdges,
                       {{Handle("leftPose"), RigExecValueId(2)}}, &error));
    CHECK(error.find("do not match the operation count") != std::string::npos);

    RigExecDemandEdges edges = DiamondEdges(graph.ops.size());
    edges.effects[0] = {static_cast<uint32_t>(graph.ops.size())};
    CHECK(!index.Build(graph, edges, {{Handle("leftPose"), RigExecValueId(2)}},
                       &error));
    CHECK(error.find("names an absent operation") != std::string::npos);

    // A value with no canonical writer is unsupported, not fabricated.
    edges = DiamondEdges(graph.ops.size());
    CHECK(index.Build(graph, edges, {{Handle("ghost"), RigExecValueId(77)}},
                      &error));
    CHECK(index.Size() == 1 && index.Producer(0) == kRigExecNoOp);
    CHECK(index.Closure(0).Empty());

    RigExecOutputHandle anonymous;
    anonymous.rigIdentity = "rig/hero";
    CHECK(!index.Build(graph, edges, {{anonymous, RigExecValueId(2)}}, &error));
    CHECK(error.find("no semantic key") != std::string::npos);
}

} // namespace

int
main()
{
    TestOutputTimeIdentity();
    TestHandleIdentity();
    TestWellFormedRequest();
    TestUnionOfRequests();
    TestProducersAndClosures();
    TestUnionRunsSharedAncestorOnce();
    TestIndexRefusals();
    std::printf("OutputDemand: %d failures\n", failures);
    return failures ? 1 : 0;
}
