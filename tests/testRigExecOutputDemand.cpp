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
    handle.type = RigExecPayloadText::matrix4x4Row;
    handle.readPhase = "final";
    handle.valueRevision = revision;
    return handle;
}

// The same output of a second compiled rig instance.
RigExecOutputHandle
OtherRigHandle(const char *key)
{
    RigExecOutputHandle handle = Handle(key);
    handle.rigIdentity = "rig/other";
    return handle;
}

RigExecOutputTime
Code(double timeCode)
{
    RigExecOutputTime time;
    CHECK(RigExecOutputTime::TryTimeCode(timeCode, &time));
    return time;
}

// One host's demand for `keys` at `time`.
RigExecOutputRequest
RequestAt(const RigExecOutputTime &time,
          const std::vector<RigExecOutputHandle> &handles,
          RigExecDemandPolicy policy = RigExecDemandPolicy::Explicit)
{
    RigExecOutputRequest request;
    RigExecOutputTimeDemand demand;
    demand.time = time;
    demand.handles = handles;
    request.demands.push_back(demand);
    request.policy = policy;
    return request;
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
    CHECK(RigExecOutputTime::TryTimeCode(0.0, &zero));
    CHECK(!zero.IsDefault() && zero.TimeCode() == 0.0);
    // The default frame is an identity, not the number zero.
    CHECK(def != zero);
    CHECK(def.Hash() != zero.Hash());

    // Negative zero is the same frame as zero, so equal times hash alike.
    RigExecOutputTime negZero;
    CHECK(RigExecOutputTime::TryTimeCode(-0.0, &negZero));
    CHECK(negZero == zero && negZero.Hash() == zero.Hash());

    // Subframe and negative time codes survive exactly: the value is a USD
    // time code on the stage's scale, so no rate is applied on the way in.
    RigExecOutputTime subframe, beforeStart;
    CHECK(RigExecOutputTime::TryTimeCode(1.0 / 3.0, &subframe));
    CHECK(subframe.TimeCode() == 1.0 / 3.0);
    CHECK(RigExecOutputTime::TryTimeCode(-1.0 / 3.0, &beforeStart));
    CHECK(beforeStart.TimeCode() == -1.0 / 3.0);
    CHECK(def < beforeStart && beforeStart < zero && zero < subframe);
    CHECK(subframe != beforeStart);
    CHECK(subframe.TimeCode() != 0.0);
    RigExecOutputTime farOut;
    CHECK(RigExecOutputTime::TryTimeCode(100000.5, &farOut));
    CHECK(farOut.TimeCode() == 100000.5);

    // An invalid time code is refused and the destination is left untouched.
    RigExecOutputTime keep = RigExecOutputTime::Default();
    CHECK(!RigExecOutputTime::TryTimeCode(
        std::numeric_limits<double>::quiet_NaN(), &keep));
    CHECK(keep.IsDefault());
    CHECK(!RigExecOutputTime::TryTimeCode(
        std::numeric_limits<double>::infinity(), &keep));
    CHECK(!RigExecOutputTime::TryTimeCode(
        -std::numeric_limits<double>::infinity(), &keep));
    CHECK(RigExecOutputTime::TryTimeCode(2.5, &keep) && keep.TimeCode() == 2.5);

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
    request = RequestAt(RigExecOutputTime::Default(), {Handle("leftPose")});
    CHECK(request.IsWellFormed(&error) && error.empty());

    CHECK(!RigExecOutputRequest{}.IsWellFormed(&error));
    CHECK(error == "output request has no demands");

    RigExecOutputRequest emptyDemand;
    emptyDemand.demands.push_back({RigExecOutputTime::Default(), {}});
    CHECK(!emptyDemand.IsWellFormed(&error));
    CHECK(error == "output demand has no handles");

    RigExecOutputHandle missingType = Handle("leftPose");
    missingType.type.clear();
    CHECK(!RequestAt(RigExecOutputTime::Default(), {missingType})
               .IsWellFormed(&error));
    CHECK(error.find("payload type") != std::string::npos);

    RigExecOutputHandle anonymous;
    anonymous.semanticKey = "leftPose";
    anonymous.type = RigExecPayloadText::matrix4x4Row;
    CHECK(!RequestAt(RigExecOutputTime::Default(), {anonymous})
               .IsWellFormed(&error));
    CHECK(error.find("no rig identity") != std::string::npos);

    // A refused time code never enters a request.
    RigExecOutputTime untouched;
    CHECK(!RigExecOutputTime::TryTimeCode(
        std::numeric_limits<double>::quiet_NaN(), &untouched));
    CHECK(untouched.IsDefault());
}

void
TestRequestGroupValidation()
{
    std::string error;
    // The default, zero and a negative subframe code are three distinct times.
    const RigExecOutputTime def = RigExecOutputTime::Default();
    const RigExecOutputTime zero = Code(0.0);
    const RigExecOutputTime subframe = Code(-1.0 / 3.0);

    RigExecOutputRequest mixedEpoch;
    mixedEpoch.demands.push_back({def, {Handle("leftPose")}});
    RigExecOutputHandle recompiled = Handle("jointTransforms");
    recompiled.compileEpoch = 2;
    mixedEpoch.demands.back().handles.push_back(recompiled);
    std::string rig;
    uint64_t epoch = 0;
    CHECK(!RigExecRequestGroup(mixedEpoch, &rig, &epoch, &error));
    CHECK(error == "request mixes compile epochs");

    RigExecOutputRequest mixedRig;
    mixedRig.demands.push_back({zero, {Handle("leftPose")}});
    mixedRig.demands.back().handles.push_back(OtherRigHandle("leftPose"));
    CHECK(!RigExecRequestGroup(mixedRig, &rig, &epoch, &error));
    CHECK(error.find("mixes rig identities") != std::string::npos);

    RigExecOutputRequest uniform;
    uniform.demands.push_back({def, {Handle("leftPose")}});
    uniform.demands.push_back({subframe, {Handle("jointTransforms")}});
    CHECK(RigExecRequestGroup(uniform, &rig, &epoch, &error) &&
          rig == "rig/hero" && epoch == 1);

    // A demand that names no output is not a group either.
    RigExecOutputRequest gap;
    gap.demands.push_back({zero, {}});
    gap.demands.push_back({subframe, {Handle("leftPose")}});
    CHECK(!RigExecRequestGroup(gap, &rig, &epoch, &error));
    CHECK(error == "output demand has no handles");

    // Every handle is checked, not the first one: two requests whose first
    // handles agree still split when a later handle names another rig.
    RigExecOutputRequest a, b;
    a.demands.push_back({def, {Handle("leftPose"), Handle("jointTransforms")}});
    b.demands.push_back(
        {def, {Handle("leftPose"), OtherRigHandle("driverField")}});
    CHECK(!RigExecRequestsCompatible(a, b, &error));
    CHECK(error.find("mixes rig identities") != std::string::npos);

    RigExecOutputRequest otherRig =
        RequestAt(def, {OtherRigHandle("leftPose")});
    CHECK(!RigExecRequestsCompatible(a, otherRig, &error));
    CHECK(error.find("different rigs") != std::string::npos);

    RigExecOutputRequest recompiledDemand =
        RequestAt(def, {Handle("leftPose"), recompiled});
    CHECK(!RigExecRequestsCompatible(a, recompiledDemand, &error));

    // The same output demanded at the same time at two revisions is two
    // groups; a dispatch answers one revision, never one per handle.
    RigExecOutputRequest atOne = RequestAt(zero, {Handle("leftPose", 1)});
    RigExecOutputRequest atTwo = RequestAt(zero, {Handle("leftPose", 2)});
    CHECK(!RigExecRequestsCompatible(atOne, atTwo, &error));
    CHECK(error.find("mismatched value revisions") != std::string::npos);

    // The same output at the same revision, and the same output at another
    // time, are one group.
    CHECK(RigExecRequestsCompatible(atOne,
                                    RequestAt(zero, {Handle("leftPose", 1)},
                                              RigExecDemandPolicy::Speculative),
                                    &error));
    CHECK(RigExecRequestsCompatible(
        atOne, RequestAt(subframe, {Handle("leftPose", 2)})));
    // An empty request is not a group.
    CHECK(!RigExecRequestsCompatible(RigExecOutputRequest{}, atOne));
}

void
TestUnionKeepsPerTimeDemands()
{
    const RigExecOutputTime def = RigExecOutputTime::Default();
    const RigExecOutputTime zero = Code(0.0);
    const RigExecOutputTime frame12 = Code(12.0);
    const RigExecOutputTime frame13 = Code(13.0);
    const RigExecOutputTime subframe = Code(-1.0 / 3.0);

    // Two hosts, two disjoint time demands, one compiled rig.
    const RigExecOutputRequest poses =
        RequestAt(frame12, {Handle("leftPose"), Handle("jointTransforms")});
    const RigExecOutputRequest driver = RequestAt(def, {Handle("driverField")});
    const RigExecOutputRequest merged = RigExecUnionRequests({poses, driver});
    std::string error;
    CHECK(merged.IsWellFormed(&error));
    CHECK(merged.demands.size() == 2);
    // No Cartesian product: an output stays inside the demand its host asked
    // for, at the exact time it asked for.
    CHECK(RigExecRequestWants(merged, Handle("leftPose"), frame12));
    CHECK(RigExecRequestWants(merged, Handle("jointTransforms"), frame12));
    CHECK(RigExecRequestWants(merged, Handle("driverField"), def));
    CHECK(!RigExecRequestWants(merged, Handle("leftPose"), def));
    CHECK(!RigExecRequestWants(merged, Handle("driverField"), frame12));
    CHECK(!RigExecRequestWants(merged, Handle("driverField"), frame13));

    // Overlapping times union their outputs; duplicates appear once.
    const RigExecOutputRequest overlapping =
        RequestAt(frame12, {Handle("driverField"), Handle("leftPose")});
    const RigExecOutputRequest unioned =
        RigExecUnionRequests({poses, overlapping});
    CHECK(unioned.demands.size() == 1);
    CHECK(unioned.demands[0].handles.size() == 3);
    CHECK(RigExecRequestWants(unioned, Handle("driverField"), frame12));

    // The default frame, numeric zero and a negative subframe code stay
    // separate demands: equal-spelling times never merge, and a demand never
    // borrows another demand's output.
    const RigExecOutputRequest defaultFrame =
        RequestAt(def, {Handle("leftPose")});
    const RigExecOutputRequest numericZero =
        RequestAt(zero, {Handle("leftPose")});
    const RigExecOutputRequest beforeStart =
        RequestAt(subframe, {Handle("jointTransforms")});
    const RigExecOutputRequest both =
        RigExecUnionRequests({defaultFrame, numericZero, beforeStart});
    CHECK(both.demands.size() == 3);
    CHECK(RigExecRequestWants(both, Handle("leftPose"), def));
    CHECK(RigExecRequestWants(both, Handle("leftPose"), zero));
    CHECK(RigExecRequestWants(both, Handle("jointTransforms"), subframe));
    CHECK(!RigExecRequestWants(both, Handle("jointTransforms"), zero));

    // An explicit request outranks a speculative one; two speculative stay
    // speculative.
    const RigExecOutputRequest speculative = RequestAt(
        frame12, {Handle("leftPose")}, RigExecDemandPolicy::Speculative);
    CHECK(RigExecUnionRequests({speculative, poses}).policy ==
          RigExecDemandPolicy::Explicit);
    CHECK(RigExecUnionRequests({speculative, speculative}).policy ==
          RigExecDemandPolicy::Speculative);

    // Differing revisions for one output at one time are refused outright:
    // the union keeps neither and the caller must dispatch two groups.
    const RigExecOutputRequest stale =
        RequestAt(frame12, {Handle("leftPose", 4)});
    const RigExecOutputRequest refused =
        RigExecUnionRequests({poses, stale}, &error);
    CHECK(refused.demands.empty());
    CHECK(error.find("mismatched value revisions") != std::string::npos);

    // Another rig or epoch is a different group too.
    error.clear();
    CHECK(RigExecUnionRequests(
              {poses, RequestAt(frame12, {OtherRigHandle("leftPose")})}, &error)
              .demands.empty());
    CHECK(error.find("different rigs") != std::string::npos);
    RigExecOutputHandle recompiled = Handle("leftPose");
    recompiled.compileEpoch = 2;
    error.clear();
    CHECK(
        RigExecUnionRequests({poses, RequestAt(frame12, {recompiled})}, &error)
            .demands.empty());
    CHECK(error.find("compile epochs") != std::string::npos);
}

// One (time, target) has one value revision, however many times a request
// writes it. Compatibility checks every occurrence and never depends on which
// request is named first, an internally conflicting request is refused by the
// request's own gate, and a union refuses a conflict it finds inside the first
// demand it merges. Duplicates that repeat one revision coalesce, and a
// revision at another time stays independent.
void
TestDuplicateOccurrencesAreChecked()
{
    const RigExecOutputTime frame1 = Code(1.0);
    const RigExecOutputTime frame2 = Code(2.0);
    const RigExecOutputHandle first = Handle("leftPose", 1);
    const RigExecOutputHandle second = Handle("leftPose", 2);
    std::string error;

    // The same demand written twice, and the same handle twice inside one
    // demand, are one demand.
    RigExecOutputRequest repeated;
    repeated.demands.push_back({frame1, {first}});
    repeated.demands.push_back({frame1, {first}});
    repeated.demands.push_back({frame1, {first, first}});
    CHECK(repeated.IsWellFormed(&error));
    CHECK(RigExecRequestRevisionsAgree(repeated, &error));
    CHECK(RigExecRequestWants(repeated, first, frame1));
    CHECK(!RigExecRequestWants(repeated, second, frame1));

    const RigExecOutputRequest single = RequestAt(frame1, {first});
    CHECK(RigExecRequestsCompatible(single, repeated, &error));
    CHECK(RigExecRequestsCompatible(repeated, single, &error));
    const RigExecOutputRequest coalesced =
        RigExecUnionRequests({repeated, single, repeated}, &error);
    CHECK(coalesced.demands.size() == 1);
    CHECK(coalesced.demands.size() == 1 &&
          coalesced.demands[0].handles.size() == 1);
    CHECK(RigExecRequestWants(coalesced, first, frame1));

    // A request pinning one output at one time to two revisions cannot be
    // answered by one run, whichever half of it carries the second demand.
    RigExecOutputRequest splitTime;
    splitTime.demands.push_back({frame1, {first}});
    splitTime.demands.push_back({frame1, {second}});
    const RigExecOutputRequest splitHandles =
        RequestAt(frame1, {first, second});
    CHECK(!splitTime.IsWellFormed(&error));
    CHECK(error.find("mismatched value revisions") != std::string::npos);
    CHECK(!splitHandles.IsWellFormed(&error));
    CHECK(error.find("mismatched value revisions") != std::string::npos);
    CHECK(!RigExecRequestRevisionsAgree(splitTime, &error));
    CHECK(!RigExecRequestRevisionsAgree(splitHandles, &error));
    CHECK(!RigExecRequestsCompatible(single, splitTime, &error));
    CHECK(!RigExecRequestsCompatible(splitTime, single, &error));
    CHECK(!RigExecRequestsCompatible(single, splitHandles, &error));
    CHECK(!RigExecRequestsCompatible(splitHandles, single, &error));
    CHECK(!RigExecRequestsCompatible(splitTime, splitHandles, &error));

    // The conflict is on the initial demand, so nothing merges: the union of
    // that request alone, and of it with a compatible one, comes back empty.
    CHECK(RigExecUnionRequests({splitTime}, &error).demands.empty());
    CHECK(error.find("mismatched value revisions") != std::string::npos);
    CHECK(RigExecUnionRequests({splitHandles}, &error).demands.empty());
    CHECK(error.find("mismatched value revisions") != std::string::npos);
    error.clear();
    CHECK(RigExecUnionRequests({splitTime, single}, &error).demands.empty());
    CHECK(error.find("mismatched value revisions") != std::string::npos);

    // A conflict carried by the second demand at one time is still one: here
    // neither request conflicts with itself, and the answer cannot depend on
    // which request is named first.
    RigExecOutputRequest hidden;
    hidden.demands.push_back({frame1, {Handle("jointTransforms", 7)}});
    hidden.demands.push_back({frame1, {Handle("leftPose", 2)}});
    const RigExecOutputRequest pinned =
        RequestAt(frame1, {Handle("leftPose", 1)});
    CHECK(hidden.IsWellFormed(&error));
    CHECK(pinned.IsWellFormed(&error));
    error.clear();
    CHECK(!RigExecRequestsCompatible(pinned, hidden, &error));
    CHECK(error.find("mismatched value revisions") != std::string::npos);
    CHECK(!RigExecRequestsCompatible(hidden, pinned, &error));
    error.clear();
    CHECK(RigExecUnionRequests({hidden, pinned}, &error).demands.empty());
    CHECK(error.find("mismatched value revisions") != std::string::npos);
    error.clear();
    CHECK(RigExecUnionRequests({pinned, hidden}, &error).demands.empty());

    // Distinct times keep their own revisions, in one request and across two,
    // and a repeated time keeps every output named at it.
    RigExecOutputRequest frames;
    frames.demands.push_back({frame1, {Handle("leftPose", 1)}});
    frames.demands.push_back({frame2, {Handle("leftPose", 2)}});
    CHECK(frames.IsWellFormed(&error));
    CHECK(RigExecRequestsCompatible(frames, single, &error));
    CHECK(RigExecRequestsCompatible(single, frames, &error));
    RigExecOutputRequest beside;
    beside.demands.push_back({frame1, {Handle("jointTransforms", 7)}});
    beside.demands.push_back({frame1, {Handle("driverField", 3)}});
    CHECK(beside.IsWellFormed(&error));
    const RigExecOutputRequest kept =
        RigExecUnionRequests({frames, beside}, &error);
    CHECK(kept.IsWellFormed(&error));
    CHECK(kept.demands.size() == 2);
    CHECK(RigExecRequestWants(kept, Handle("leftPose", 1), frame1));
    CHECK(RigExecRequestWants(kept, Handle("leftPose", 2), frame2));
    CHECK(RigExecRequestWants(kept, Handle("jointTransforms", 7), frame1));
    CHECK(RigExecRequestWants(kept, Handle("driverField", 3), frame1));
    CHECK(!RigExecRequestWants(kept, Handle("leftPose", 2), frame1));
    CHECK(!RigExecRequestWants(kept, Handle("leftPose", 1), frame2));
}

void
TestPayloadLayout()
{
    CHECK(std::string(RigExecPayloadText::scalar) == "double");
    CHECK(std::string(RigExecPayloadText::matrix4x4Row) == "matrix4d");
    CHECK(std::string(RigExecPayloadText::point3) == "point3d[]");
    CHECK(RigExecPayloadElementBytes(RigExecPayloadKind::Scalar) == 8);
    CHECK(RigExecPayloadElementBytes(RigExecPayloadKind::Matrix4x4Row) == 128);
    CHECK(RigExecPayloadElementBytes(RigExecPayloadKind::Point3) == 24);
    CHECK(RigExecPayloadElementBytes(RigExecPayloadKind::Unknown) == 0);

    RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
    CHECK(RigExecPayloadKindForType("double", &kind) &&
          kind == RigExecPayloadKind::Scalar);
    CHECK(RigExecPayloadKindForType("matrix4d", &kind) &&
          kind == RigExecPayloadKind::Matrix4x4Row);
    CHECK(RigExecPayloadKindForType("point3d[]", &kind) &&
          kind == RigExecPayloadKind::Point3);
    CHECK(!RigExecPayloadKindForType("quatd", &kind));
    CHECK(!RigExecPayloadKindForType("", &kind));
    CHECK(std::string(RigExecPayloadTypeSpelling(RigExecPayloadKind::Point3)) ==
          "point3d[]");

    // Reservation is overflow checked before any allocation.
    uint64_t bytes = 0;
    CHECK(RigExecPayloadByteCount(RigExecPayloadKind::Point3, 3, &bytes) &&
          bytes == 72);
    CHECK(!RigExecPayloadByteCount(RigExecPayloadKind::Matrix4x4Row,
                                   std::numeric_limits<uint64_t>::max(),
                                   &bytes));
    CHECK(!RigExecPayloadByteCount(RigExecPayloadKind::Unknown, 1, &bytes));

    std::string error;
    // Scalar, matrix and points round trip through the lease.
    RigExecOutputPayload scalar;
    const double one = -0.25;
    CHECK(RigExecPackPayload(RigExecPayloadKind::Scalar, 1, &one, &scalar,
                             &error));
    CHECK(scalar.ByteCount() == 8 && scalar.elementCount == 1);
    RigExecPayloadView scalarView;
    CHECK(scalarView.Open(scalar, &error));
    CHECK(scalarView.ElementCount() == 1);
    double value = 0.0;
    CHECK(scalarView.ReadScalar(0, &value) && value == one);
    CHECK(!scalarView.ReadScalar(1, &value));
    CHECK(!scalarView.ReadPoint3(0, &value));

    std::vector<double> matrices;
    for (size_t i = 0; i < 16 * 3; ++i)
        matrices.push_back(double(i) * 0.5 - 1.0);
    RigExecOutputPayload matrix;
    CHECK(RigExecPackPayload(RigExecPayloadKind::Matrix4x4Row, 3,
                             matrices.data(), &matrix, &error));
    CHECK(matrix.ByteCount() == 3 * 128);
    RigExecPayloadView matrixView;
    CHECK(matrixView.Open(matrix, &error));
    double row[16] = {};
    CHECK(matrixView.ReadMatrix4x4Row(2, row));
    for (size_t i = 0; i < 16; ++i)
        CHECK(row[i] == matrices[size_t(2 * 16 + i)]);
    // Row-major: the row-vector translation column is the last four doubles.
    CHECK(row[12] == matrices[2 * 16 + 12] && row[15] == matrices[2 * 16 + 15]);
    CHECK(!matrixView.ReadMatrix4x4Row(3, row));
    CHECK(!matrixView.ReadMatrix4x4Row(0, nullptr));
    CHECK(!matrixView.ReadScalar(0, row));

    const std::vector<double> points = {0.0,  -0.0, 1.0 / 3.0,
                                        -7.5, 2.0,  100000.5};
    RigExecOutputPayload pointsPayload;
    CHECK(RigExecPackPayload(RigExecPayloadKind::Point3, 2, points.data(),
                             &pointsPayload, &error));
    CHECK(pointsPayload.ByteCount() == 48 && pointsPayload.elementCount == 2);
    RigExecPayloadView xyzView;
    CHECK(xyzView.Open(pointsPayload, &error));
    double point[3] = {};
    CHECK(xyzView.ReadPoint3(0, point));
    CHECK(point[0] == 0.0 && point[2] == 1.0 / 3.0);
    CHECK(xyzView.ReadPoint3(1, point));
    CHECK(point[0] == -7.5 && point[1] == 2.0 && point[2] == 100000.5);
    CHECK(!xyzView.ReadPoint3(2, point));

    // A zero-element payload is a declared empty value, not a missing one.
    RigExecOutputPayload none;
    CHECK(RigExecPackPayload(RigExecPayloadKind::Point3, 0, nullptr, &none,
                             &error));
    CHECK(RigExecValidatePayload(none, &error));
    RigExecPayloadView noneView;
    CHECK(noneView.Open(none, &error) && noneView.ElementCount() == 0);
    CHECK(!noneView.ReadPoint3(0, point));

    // Malformed leases are refused rather than read past their end.
    RigExecOutputPayload shorty;
    shorty.bytes = std::make_shared<const std::vector<uint8_t>>(127, 0);
    shorty.kind = RigExecPayloadKind::Matrix4x4Row;
    shorty.elementCount = 1;
    CHECK(!RigExecValidatePayload(shorty, &error));
    CHECK(error.find("bytes for 1 elements") != std::string::npos);
    RigExecPayloadView shortView;
    CHECK(!shortView.Open(shorty, &error));

    RigExecOutputPayload extra;
    extra.bytes = std::make_shared<const std::vector<uint8_t>>(25, 0);
    extra.kind = RigExecPayloadKind::Point3;
    extra.elementCount = 1;
    CHECK(!RigExecValidatePayload(extra, &error));
    CHECK(error.find("bytes for 1 elements") != std::string::npos);

    RigExecOutputPayload noBytes;
    noBytes.kind = RigExecPayloadKind::Scalar;
    noBytes.elementCount = 4;
    CHECK(!RigExecValidatePayload(noBytes, &error));
    CHECK(error == "payload has no bytes");

    RigExecOutputPayload undeclared;
    undeclared.bytes = std::make_shared<const std::vector<uint8_t>>(8, 0);
    undeclared.elementCount = 1;
    CHECK(undeclared.kind == RigExecPayloadKind::Unknown);
    CHECK(!RigExecValidatePayload(undeclared, &error));
    CHECK(error.find("no declared layout") != std::string::npos);

    RigExecOutputPayload overflow;
    overflow.kind = RigExecPayloadKind::Point3;
    overflow.elementCount = std::numeric_limits<uint64_t>::max();
    CHECK(!RigExecValidatePayload(overflow, &error));
    CHECK(error.find("overflows") != std::string::npos);

    RigExecOutputPayload emptyWithBytes;
    emptyWithBytes.bytes = std::make_shared<const std::vector<uint8_t>>(8, 0);
    emptyWithBytes.kind = RigExecPayloadKind::Scalar;
    emptyWithBytes.elementCount = 0;
    CHECK(!RigExecValidatePayload(emptyWithBytes, &error));
    CHECK(error.find("declares no elements") != std::string::npos);

    // The handle's type spelling and the payload layout have to agree.
    RigExecOutputHandle matrixHandle = Handle("leftPose");
    CHECK(RigExecPayloadMatchesHandle(matrixHandle, matrix, &error));
    matrixHandle.type = RigExecPayloadText::point3;
    CHECK(!RigExecPayloadMatchesHandle(matrixHandle, matrix, &error));
    CHECK(error.find("but its payload is") != std::string::npos);
    RigExecOutputHandle unknownType = Handle("leftPose");
    unknownType.type = "quatd";
    CHECK(!RigExecPayloadMatchesHandle(unknownType, matrix, &error));
    CHECK(error.find("no defined payload layout") != std::string::npos);
    CHECK(RigExecPackPayload(RigExecPayloadKind::Unknown, 1, &one, &scalar,
                             &error) == false);
}

void
TestGenerationKeyAndPacket()
{
    const RigExecOutputTime frame12 = Code(12.0);
    RigExecOutputHandle handle = Handle("leftPose");
    RigExecOutputPayload payload;
    std::vector<double> matrix(16, 1.0);
    std::string error;
    CHECK(RigExecPackPayload(RigExecPayloadKind::Matrix4x4Row, 1, matrix.data(),
                             &payload, &error));

    RigExecOutputPacket packet;
    packet.handle = handle;
    packet.time = frame12;
    packet.programIdentity = 7;
    packet.inputRevision = 3;
    packet.previewRevision = 11;
    packet.historyRevision = 5;
    packet.status = RigExecOutputStatus::Ready;
    packet.payload = payload;

    // Every part of the captured generation is part of the key.
    const RigExecOutputGeneration captured = packet.Generation();
    CHECK(captured.programIdentity == 7 && captured.inputRevision == 3 &&
          captured.previewRevision == 11 && captured.historyRevision == 5);
    CHECK(packet.Answers(handle, frame12, captured));
    CHECK(RigExecOutputGeneration() == RigExecOutputGeneration());
    RigExecOutputGeneration moved = captured;
    moved.historyRevision += 1;
    CHECK(moved != captured && !packet.Answers(handle, frame12, moved));
    moved = captured;
    moved.previewRevision = 0;
    CHECK(!packet.Answers(handle, frame12, moved));
    moved = captured;
    moved.inputRevision += 1;
    CHECK(!packet.Answers(handle, frame12, moved));
    moved = captured;
    moved.programIdentity += 1;
    CHECK(!packet.Answers(handle, frame12, moved));
    // Two generations of the same value still hash apart.
    const RigExecOutputGeneration keep{7, 3, 11, 5};
    const RigExecOutputGeneration movedHistory{7, 3, 11, 6};
    CHECK(keep.Hash() != movedHistory.Hash());

    // A packet answers the handle at the revision it was read against, at the
    // exact time it was produced for, and nothing else.
    CHECK(!packet.Answers(Handle("leftPose", 9), frame12, captured));
    CHECK(!packet.Answers(handle, Code(13.0), captured));
    CHECK(!packet.Answers(handle, RigExecOutputTime::Default(), captured));
    CHECK(!packet.Answers(Handle("jointTransforms"), frame12, captured));
    RigExecOutputPacket pending = packet;
    pending.status = RigExecOutputStatus::Pending;
    CHECK(!pending.Answers(handle, frame12, captured));
    RigExecOutputPacket unreadable = packet;
    unreadable.payload.kind = RigExecPayloadKind::Point3;
    CHECK(!unreadable.Answers(handle, frame12, captured));
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
TestResolvedRequestRunsOneMask()
{
    const RigExecCompiledGraph graph = DiamondGraph();
    const RigExecOutputDemandIndex index = BuildIndex(graph);
    const RigExecOutputTime frame12 = Code(12.0);
    const RigExecOutputTime frame13 = Code(13.0);

    // One handle demanded at two times resolves to one slot, and the mask the
    // dispatch runs is the union of the resolved closures.
    RigExecOutputRequest request;
    request.demands.push_back({frame12, {Handle("leftPose")}});
    request.demands.push_back(
        {frame13, {Handle("leftPose"), Handle("jointTransforms")}});
    std::string error;
    std::vector<size_t> slots;
    CHECK(index.ResolveRequest(request, &slots, &error));
    CHECK(slots == std::vector<size_t>({0, 1}));
    const std::vector<char> mask = index.ActiveMask(slots);
    CHECK(mask == std::vector<char>(graph.ops.size(), 1));
    RigExecDemandClosure closure = index.Union(slots);
    std::string closed;
    CHECK(RigExecClosureIsClosed(graph, closure, &closed));

    // An output the index does not know is refused, and no partial mask is
    // produced for a partially resolved request.
    request.demands.front().handles.push_back(Handle("notAnOutput"));
    CHECK(!index.ResolveRequest(request, &slots, &error));
    CHECK(slots.empty());
    CHECK(error.find("does not know") != std::string::npos);

    // An unsupported output resolves but demands no operation.
    RigExecOutputRequest ghost = RequestAt(frame12, {Handle("ghostField")});
    CHECK(index.ResolveRequest(ghost, &slots, &error));
    CHECK(slots == std::vector<size_t>({3}));
    CHECK(index.ActiveMask(slots) == std::vector<char>(graph.ops.size(), 0));
    CHECK(index.Union(slots).Empty());
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
    TestRequestGroupValidation();
    TestUnionKeepsPerTimeDemands();
    TestDuplicateOccurrencesAreChecked();
    TestPayloadLayout();
    TestGenerationKeyAndPacket();
    TestProducersAndClosures();
    TestResolvedRequestRunsOneMask();
    TestUnionRunsSharedAncestorOnce();
    TestIndexRefusals();
    std::printf("OutputDemand: %d failures\n", failures);
    return failures ? 1 : 0;
}
