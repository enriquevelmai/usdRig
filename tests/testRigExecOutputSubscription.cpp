// The bounded subscription lifecycle: demand dedup and union, atomic updates,
// cancellation, destroy-all release, in-flight invalidation, exact times across
// rigs and stages, explicit demand with warming switched off, coverage cache
// hits that pull nothing, and the byte bounds that refuse before allocating.
//
// The catalog, the demand index, the pool's admission and fences, and the
// coverage cache are all real. Only the lane's executor and the sampled vector
// are injected, which is what makes the answers deterministic without a rig
// under the runner: the entry point, both fences and the coverage bookkeeping
// stay the service's.
#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/outputSubscription.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecSchema/schemaNames.h"

#include "pxr/base/tf/setenv.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/stage.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
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

const char *const kStageA = R"USDA(#usda 1.0

def Xform "Asset"
{
    def Scope "UnusualContainer"
    {
        def RigExecRoot "Character"
        {
            def Scope "InputsAnywhere"
            {
                def RigExecControl "Drive"
                {
                    double avars:ty = 2
                }
            }

            def Scope "WorkAnywhere"
            {
                def Scope "PropertyStack"
                {
                    def RigExecFloatMathMover "Second" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        uniform token rigExec:operation = "multiply"
                        float inputs:value = 2
                        rel rigExec:moves = </Asset/UnusualContainer/Character/RandomData.amount>
                    }

                    def RigExecFloatMathMover "First" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        uniform token rigExec:operation = "add"
                        float inputs:value = 1
                        rel rigExec:moves = </Asset/UnusualContainer/Character/RandomData.amount>
                    }
                }

                def Scope "PointStack"
                {
                    def RigExecMatrixMover "Second" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        float inputs:defaultWeight = 1
                        rel rigExec:moves = </Asset/UnusualContainer/Geometry/Body.points>
                        rel rigExec:transform = </Asset/UnusualContainer/Character/InputsAnywhere/Drive>
                    }
                }
            }

            def Scope "RandomData"
            {
                float amount = 3
            }
        }

        def Scope "Geometry"
        {
            def Points "Body"
            {
                point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
            }
        }
    }
}
)USDA";

const char *const kStageB = R"USDA(#usda 1.0

def Xform "World"
{
    def Scope "Depths"
    {
        def RigExecRoot "Rig"
        {
            def Scope "ControlsAnywhere"
            {
                def RigExecControl "Drive"
                {
                    double avars:tz = 5
                }
            }

            def Scope "Numbers"
            {
                float dial = 4
            }

            def Scope "WorkAnywhere"
            {
                def Scope "DialStack"
                {
                    def RigExecFloatMathMover "Twice" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        uniform token rigExec:operation = "multiply"
                        float inputs:value = 2
                        rel rigExec:moves = </World/Depths/Rig/Numbers.dial>
                    }
                }

                def Scope "ShellStack"
                {
                    def RigExecMatrixMover "Shift" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        float inputs:defaultWeight = 1
                        rel rigExec:moves = </World/Depths/Model/Shell.points>
                        rel rigExec:transform = </World/Depths/Rig/ControlsAnywhere/Drive>
                    }
                }
            }
        }

        def Scope "Model"
        {
            def Points "Shell"
            {
                point3f[] points = [(0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0)]
            }
        }
    }
}
)USDA";

UsdTimeCode
TimeCodeOf(const RigExecOutputTime &time)
{
    if (time.IsDefault()) {
        return UsdTimeCode::Default();
    }
    return UsdTimeCode(time.TimeCode());
}

RigExecOutputTime
TimeOf(double timeCode)
{
    RigExecOutputTime time;
    RigExecOutputTime::TryTimeCode(timeCode, &time);
    return time;
}

size_t
ElementCount(RigExecPayloadKind kind)
{
    switch (kind) {
    case RigExecPayloadKind::Scalar:
        return 1;
    case RigExecPayloadKind::Matrix4x4Row:
        return 1;
    case RigExecPayloadKind::Point3:
        return 3;
    case RigExecPayloadKind::Unknown:
        break;
    }
    return 0;
}

size_t
DoublesPerElement(RigExecPayloadKind kind)
{
    switch (kind) {
    case RigExecPayloadKind::Scalar:
        return 1;
    case RigExecPayloadKind::Matrix4x4Row:
        return 16;
    case RigExecPayloadKind::Point3:
        return 3;
    case RigExecPayloadKind::Unknown:
        break;
    }
    return 0;
}

// The value one lane stamps into every cell it produces: the rig, the exact
// frame and the captured input revision folded together, so a payload that
// crossed a rig, a frame or a generation carries a different number.
double
MarkerFor(const std::string &rigIdentity, const RigExecOutputTime &time,
          uint64_t inputRevision)
{
    size_t hash = std::hash<std::string>()(rigIdentity);
    hash ^= time.Hash() + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    hash ^= inputRevision + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    return double(hash % 90000ull) + 1000.5;
}

double
FirstCell(const RigExecOutputPayload &payload)
{
    RigExecPayloadView view;
    std::string error;
    if (!view.Open(payload, &error)) {
        return -1.0;
    }
    double out[16] = {};
    switch (view.Kind()) {
    case RigExecPayloadKind::Scalar:
        if (!view.ReadScalar(0, out)) {
            return -2.0;
        }
        break;
    case RigExecPayloadKind::Matrix4x4Row:
        if (!view.ReadMatrix4x4Row(0, out)) {
            return -2.0;
        }
        break;
    case RigExecPayloadKind::Point3:
        if (!view.ReadPoint3(0, out)) {
            return -2.0;
        }
        break;
    case RigExecPayloadKind::Unknown:
        return -3.0;
    }
    return out[0];
}

// One element of the demanded layout: what the service floors a job's payload
// estimate at before it knows the point count.
size_t
PayloadFloorBytes(const RigExecOutputHandle &handle)
{
    RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
    if (!RigExecPayloadKindForType(handle.type, &kind)) {
        return 0;
    }
    uint64_t elementBytes = 0;
    if (!RigExecPayloadByteCount(kind, 1, &elementBytes)) {
        return 0;
    }
    return size_t(elementBytes);
}

size_t
TotalBytes(const RigExecCoverageBytes &bytes)
{
    size_t total = 0;
    bytes.TryTotal(&total);
    return total;
}

struct RigFixture {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    SdfPath root;
    uint64_t epoch = 0;
    std::shared_ptr<RigExecOutputCatalog> catalog;
    RigExecOutputHandle scalar;
    RigExecOutputHandle transform;
    RigExecOutputHandle points;
};

const RigExecOutputBinding *
FindBinding(const RigExecOutputCatalog &catalog, const std::string &target,
            const std::string &computation, const std::string &phase)
{
    for (const RigExecOutputBinding &binding : catalog.GetBindings()) {
        if (binding.descriptor.target == target &&
            binding.descriptor.publicComputation == computation &&
            binding.descriptor.phase == phase) {
            return &binding;
        }
    }
    // Which phases a target exports depends on who reads it, which this test
    // does not pin; any phase of the right address is the right output.
    for (const RigExecOutputBinding &binding : catalog.GetBindings()) {
        if (binding.descriptor.target == target &&
            binding.descriptor.publicComputation == computation) {
            return &binding;
        }
    }
    return nullptr;
}

RigFixture
BuildRig(const char *stageText, const SdfPath &root, const std::string &target,
         const std::string &transformTarget, const std::string &pointsTarget,
         const std::string &identity, uint64_t epoch)
{
    RigFixture fixture;
    fixture.root = root;
    fixture.epoch = epoch;
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer->ImportFromString(stageText));
    fixture.stage = UsdStage::Open(layer);
    CHECK(fixture.stage);
    fixture.evaluator =
        std::make_unique<RigExecRigEvaluator>(fixture.stage, root);
    std::vector<std::string> errors;
    CHECK(fixture.evaluator->Compile(&errors));
    for (const std::string &error : errors) {
        std::printf("    compile: %s\n", error.c_str());
    }
    const RigExecBakedProgram *program = fixture.evaluator->GetBakedProgram();
    CHECK(program);
    fixture.catalog = std::make_shared<RigExecOutputCatalog>();
    std::string error;
    CHECK(RigExecDiscoverOutputs(*program, identity, epoch,
                                 fixture.catalog.get(), &error));
    if (!error.empty()) {
        std::printf("    discovery: %s\n", error.c_str());
    }
    const RigExecOutputBinding *scalar =
        FindBinding(*fixture.catalog, target, "", "final");
    const RigExecOutputBinding *transform =
        FindBinding(*fixture.catalog, transformTarget,
                    SchemaComputationText::computeMatrix, "base");
    const RigExecOutputBinding *points =
        FindBinding(*fixture.catalog, pointsTarget, "", "final");
    CHECK(scalar);
    CHECK(transform);
    CHECK(points);
    if (scalar) {
        fixture.scalar = scalar->handle;
    }
    if (transform) {
        fixture.transform = transform->handle;
    }
    if (points) {
        fixture.points = points->handle;
    }
    return fixture;
}

// One exact frame's demand over `handles`.
RigExecOutputRequest
RequestAt(const RigExecOutputTime &time,
          const std::vector<RigExecOutputHandle> &handles)
{
    RigExecOutputTimeDemand demand;
    demand.time = time;
    demand.handles = handles;
    RigExecOutputRequest request;
    request.demands.push_back(std::move(demand));
    return request;
}

// A real catalog, a real pool and a real coverage cache; only the lane's
// executor and the sampled vector are invented. `workers` is the pool's lane
// count: zero keeps every job queued until the test drains it by hand, one
// lets a job reach the fence while the test holds the gate.
struct Harness {
    // The cache is declared first so it outlives the pool, whose destructor
    // joins the lanes that publish into it.
    RigExecFrameCache cache;
    std::unique_ptr<RigExecBackgroundScheduler> scheduler;

    std::map<SdfPath, std::shared_ptr<const RigExecOutputCatalog>> catalogs;

    // Knobs.
    bool captureOk = true;
    uint64_t programIdentityOverride = 0;
    uint64_t inputRevision = 1;
    size_t captureBytes = 4096;
    bool gateOpen = true;

    // Observations.
    std::atomic<int> captures{0};
    std::atomic<int> laneRuns{0};
    std::vector<std::vector<std::string>> launchOutputs;
    std::mutex logMutex;
    std::mutex gateMutex;
    std::condition_variable gateCv;

    explicit Harness(size_t workers = 0)
        : scheduler(std::make_unique<RigExecBackgroundScheduler>(workers))
    {
    }

    ~Harness()
    {
        OpenGate();
    }

    RigExecOutputSubscriptionService MakeService()
    {
        RigExecSubscriptionHost host;
        host.scheduler = scheduler.get();
        host.frameCache = &cache;
        host.resolveCatalog = [this](const SdfPath &rig) {
            const auto found = catalogs.find(rig);
            return found == catalogs.end() ? nullptr : found->second;
        };
        host.capture =
            [this](const SdfPath &rig, const RigExecOutputTimeDemand &demand,
                   const RigExecOutputCatalog &catalog,
                   RigExecSubscriptionCapture *out, std::string *error) {
                (void)rig;
                (void)catalog;
                ++captures;
                if (!captureOk) {
                    *error = "the harness refused the capture";
                    return false;
                }
                out->inputs.time = TimeCodeOf(demand.time);
                out->context.varyingInputCount = 0;
                out->context.slotCount = 0;
                out->context.compiledRigSignature = 7;
                out->context.programDigest = 11;
                out->captured.inputRevision = inputRevision;
                out->captured.previewRevision = 0;
                out->captured.historyRevision = 0;
                out->captured.programIdentity = programIdentityOverride;
                out->upstreamRevision = 0;
                out->bytes.capture = captureBytes;
                return true;
            };
        host.stepRunner = [this](const RigExecFrozenEvalContext &context,
                                 const RigExecFrameInputs &inputs,
                                 RigExecFrozenArena &arena,
                                 const RigExecFrozenRequestedRun &requested,
                                 RigExecRequestedRunReport *report,
                                 std::vector<RigExecOutputPacket> *packets,
                                 std::string *error) {
            (void)context;
            (void)arena;
            WaitForGate();
            ++laneRuns;
            RigExecOutputTime at;
            if (!inputs.time.IsDefault() &&
                !RigExecOutputTime::TryTimeCode(inputs.time.GetValue(), &at)) {
                *error = "the harness lane got a non-finite time";
                return false;
            }
            {
                std::lock_guard<std::mutex> lock(logMutex);
                std::vector<std::string> names;
                for (const RigExecOutputHandle &output : requested.outputs) {
                    names.push_back(output.semanticKey);
                }
                launchOutputs.push_back(std::move(names));
            }
            *report = RigExecRequestedRunReport{};
            packets->clear();
            const double marker =
                MarkerFor(requested.catalog->GetRigIdentity(), at,
                          requested.captured.inputRevision);
            for (const RigExecOutputHandle &output : requested.outputs) {
                RigExecOutputPacket packet;
                packet.handle = output;
                packet.time = at;
                packet.programIdentity = requested.catalog->GetCompileEpoch();
                packet.inputRevision = requested.captured.inputRevision;
                packet.previewRevision = requested.captured.previewRevision;
                packet.historyRevision = requested.captured.historyRevision;
                RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
                std::string reason;
                if (!RigExecPayloadKindForType(output.type, &kind)) {
                    packet.status = RigExecOutputStatus::Failed;
                    packet.reason = "no layout for " + output.type;
                    packets->push_back(std::move(packet));
                    continue;
                }
                const size_t elements = ElementCount(kind);
                std::vector<double> cells(elements * DoublesPerElement(kind),
                                          marker);
                if (!RigExecPackPayload(kind, elements, cells.data(),
                                        &packet.payload, &reason)) {
                    packet.status = RigExecOutputStatus::Failed;
                    packet.reason = reason;
                } else {
                    packet.status = RigExecOutputStatus::Ready;
                }
                packets->push_back(std::move(packet));
            }
            return true;
        };
        return RigExecOutputSubscriptionService(std::move(host));
    }

    void WaitForGate()
    {
        std::unique_lock<std::mutex> wait(gateMutex);
        gateCv.wait(wait, [this] { return gateOpen; });
    }

    void OpenGate()
    {
        {
            std::lock_guard<std::mutex> lock(gateMutex);
            gateOpen = true;
        }
        gateCv.notify_all();
    }

    /// Waits until a queued job has reached the gated lane.
    bool WaitUntilRunning()
    {
        for (int spin = 0; spin < 4000; ++spin) {
            if (scheduler->Stats().explicitRunning == 1) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    std::vector<std::string> LaunchOutput(size_t launch)
    {
        std::lock_guard<std::mutex> lock(logMutex);
        if (launch >= launchOutputs.size()) {
            return {};
        }
        return launchOutputs[launch];
    }

    bool LaunchNames(size_t launch, const std::string &semanticKey)
    {
        for (const std::string &name : LaunchOutput(launch)) {
            if (name == semanticKey) {
                return true;
            }
        }
        return false;
    }
};

bool
AllPending(const std::vector<RigExecOutputPacket> &packets)
{
    for (const RigExecOutputPacket &packet : packets) {
        if (packet.status != RigExecOutputStatus::Pending) {
            return false;
        }
    }
    return !packets.empty();
}

bool
AllReady(const std::vector<RigExecOutputPacket> &packets)
{
    for (const RigExecOutputPacket &packet : packets) {
        if (packet.status != RigExecOutputStatus::Ready ||
            packet.payload.IsEmpty()) {
            return false;
        }
    }
    return !packets.empty();
}

RigFixture
StageAAt(uint64_t epoch)
{
    return BuildRig(kStageA, SdfPath("/Asset/UnusualContainer/Character"),
                    "/Asset/UnusualContainer/Character/RandomData.amount",
                    "/Asset/UnusualContainer/Character/InputsAnywhere/Drive",
                    "/Asset/UnusualContainer/Geometry/Body.points", "stage-A",
                    epoch);
}

RigFixture
StageA()
{
    return StageAAt(17);
}

RigFixture
StageB()
{
    return BuildRig(kStageB, SdfPath("/World/Depths/Rig"),
                    "/World/Depths/Rig/Numbers.dial",
                    "/World/Depths/Rig/ControlsAnywhere/Drive",
                    "/World/Depths/Model/Shell.points", "stage-B", 17);
}

void
TestValidationAndRaii()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputRequest request =
        RequestAt(TimeOf(12.0), {a.scalar, a.points});

    RigExecSubscriptionReceipt receipt;
    CHECK(!service.Subscribe(SdfPath("/NotARig"), request, &receipt).IsValid());
    CHECK(receipt.status == RigExecSubscriptionStatus::UnknownRig);
    CHECK(service.Stats().subscriptions == 0);

    RigExecOutputRequest empty;
    CHECK(!service.Subscribe(a.root, empty, &receipt).IsValid());
    CHECK(receipt.status == RigExecSubscriptionStatus::InvalidRequest);

    RigExecOutputHandle stale = a.scalar;
    stale.compileEpoch = a.epoch + 1;
    CHECK(!service.Subscribe(a.root, RequestAt(TimeOf(12.0), {stale}), &receipt)
               .IsValid());
    CHECK(receipt.status == RigExecSubscriptionStatus::StaleEpoch);
    // Two epochs in one request is a malformed request, not two groups.
    CHECK(!service
               .Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar, stale}),
                          &receipt)
               .IsValid());
    CHECK(receipt.status == RigExecSubscriptionStatus::InvalidRequest);

    RigExecOutputHandle unknown = a.scalar;
    unknown.semanticKey = RigExecOutputSemanticKey("nowhere", "", "final");
    CHECK(
        !service.Subscribe(a.root, RequestAt(TimeOf(12.0), {unknown}), &receipt)
             .IsValid());
    CHECK(receipt.status == RigExecSubscriptionStatus::UnknownOutput);
    CHECK(receipt.reason.find("nowhere") != std::string::npos);

    {
        auto sub = service.Subscribe(a.root, request);
        CHECK(sub.IsValid());
        CHECK(sub.Id() != 0);
        CHECK(service.Stats().subscriptions == 1);
        CHECK(service.PollResults(sub).size() == 2);
        CHECK(service.UpdateSubscription(0, request).status ==
              RigExecSubscriptionStatus::UnknownSubscription);
    }
    CHECK(service.Stats().subscriptions == 0);
    CHECK(service.PollResults(1).empty());
    CHECK(!service.IsStopping());
}

void
TestTwoClientDedupAndExpansion()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);

    auto first =
        service.Subscribe(a.root, RequestAt(frame, {a.scalar, a.points}));
    auto second =
        service.Subscribe(a.root, RequestAt(frame, {a.scalar, a.transform}));
    CHECK(first.IsValid());
    CHECK(second.IsValid());
    // Two clients, one frame, one queued job: the shared output is pulled once.
    CHECK(service.Stats().framesDemanded == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.scheduler->Stats().explicitCoalesced == 0);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(h.laneRuns == 0);
    CHECK(AllPending(service.PollResults(first)));
    CHECK(AllPending(service.PollResults(second)));

    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    // The launched mask is exactly what was admitted for the first client.
    CHECK(h.LaunchOutput(0).size() == 2);
    CHECK(h.LaunchNames(0, a.scalar.semanticKey));
    CHECK(h.LaunchNames(0, a.points.semanticKey));
    CHECK(!h.LaunchNames(0, a.transform.semanticKey));

    // The second client's extra output is owed the next launch, asked for as
    // the union; the dispatched mask is never rewritten.
    CHECK(service.PumpCallbacks() == 0);
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 2);
    CHECK(h.LaunchOutput(1).size() == 3);
    CHECK(h.LaunchNames(1, a.transform.semanticKey));

    const std::vector<RigExecOutputPacket> one = service.PollResults(first);
    const std::vector<RigExecOutputPacket> two = service.PollResults(second);
    CHECK(AllReady(one));
    CHECK(AllReady(two));
    const double marker = MarkerFor("stage-A", frame, h.inputRevision);
    for (const RigExecOutputPacket &packet : one) {
        CHECK(FirstCell(packet.payload) == marker);
        CHECK(packet.time == frame);
    }
    // Two clients, one payload: one pin and one shared buffer per output.
    const RigExecSubscriptionStats stats = service.Stats();
    CHECK(stats.pinnedPayloads == 3);
    for (const RigExecOutputPacket &firstPacket : one) {
        for (const RigExecOutputPacket &secondPacket : two) {
            if (firstPacket.handle.semanticKey ==
                secondPacket.handle.semanticKey) {
                CHECK(firstPacket.payload.bytes == secondPacket.payload.bytes);
            }
        }
    }
    // One coverage key for the frame: the second record replaced the first
    // under it rather than standing beside it.
    CHECK(h.cache.CoverageRecordCount() == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(h.cache.CoverageHeld().pinned > 0);
}

void
TestUpdateAtomicity()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    auto sub = service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(sub.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.cache.CoverageHeld().capture == h.captureBytes);

    // A refused update moves nothing.
    RigExecOutputRequest empty;
    CHECK(service.UpdateSubscription(sub, empty).status ==
          RigExecSubscriptionStatus::InvalidRequest);
    RigExecOutputHandle stale = a.scalar;
    stale.compileEpoch = a.epoch + 3;
    CHECK(service.UpdateSubscription(sub, RequestAt(TimeOf(12.0), {stale}))
              .status == RigExecSubscriptionStatus::StaleEpoch);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(h.cache.CoverageHeld().capture == h.captureBytes);
    CHECK(AllPending(service.PollResults(sub)));

    // An accepted update retires the frame no client demands any more and
    // queues the one that does.
    const RigExecSubscriptionReceipt moved =
        service.UpdateSubscription(sub, RequestAt(TimeOf(20.0), {a.transform}));
    CHECK(moved.IsAccepted());
    CHECK(moved.frames == 1);
    CHECK(h.scheduler->Stats().explicitCanceled == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() ==
          h.captureBytes + PayloadFloorBytes(a.transform));
    CHECK(h.cache.CoverageHeld().capture == h.captureBytes);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    CHECK(AllReady(service.PollResults(sub)));
}

void
TestCancellationKeepsSharedDemand()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputRequest request = RequestAt(TimeOf(12.0), {a.scalar});
    auto first = service.Subscribe(a.root, request);
    auto second = service.Subscribe(a.root, request);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);

    // One client leaving leaves the shared demand, its job and its byte
    // reservation standing.
    service.Close(first);
    CHECK(service.Stats().subscriptions == 1);
    CHECK(h.scheduler->Stats().explicitCanceled == 0);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() > 0);

    service.Close(second);
    CHECK(service.Stats().subscriptions == 0);
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(h.scheduler->Stats().explicitCanceled == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(h.cache.CoverageHeld().capture == 0);
    CHECK(h.scheduler->Stats().explicitCompleted == 0);

    service.Close(second);
    CHECK(h.scheduler->Stats().explicitCanceled == 1);
}

void
TestDestroyAllReleasesEverything()
{
    Harness h;
    const RigFixture a = StageA();
    const RigFixture b = StageB();
    h.catalogs[a.root] = a.catalog;
    h.catalogs[b.root] = b.catalog;
    auto service = h.MakeService();
    std::vector<RigExecSubscriptionHandle> subs;
    subs.push_back(
        service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar})));
    subs.push_back(
        service.Subscribe(a.root, RequestAt(TimeOf(13.0), {a.points})));
    subs.push_back(
        service.Subscribe(b.root, RequestAt(RigExecOutputTime::Default(),
                                            {b.transform, b.points})));
    CHECK(subs.size() == 3);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 3);
    for (const RigExecSubscriptionHandle &sub : subs) {
        CHECK(AllReady(service.PollResults(sub)));
    }
    CHECK(service.Stats().pinnedPayloads == 4);
    CHECK(service.Stats().pinnedBytes > 0);
    CHECK(h.cache.CoverageHeld().pinned > 0);

    service.CloseAll();
    CHECK(service.Stats().subscriptions == 0);
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(service.Stats().pinnedPayloads == 0);
    CHECK(service.Stats().pinnedBytes == 0);
    CHECK(h.cache.CoverageHeld().pinned == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    // The records stay: another client can still be answered from them.
    CHECK(h.cache.CoverageRecordCount() == 3);
    CHECK(h.cache.CoverageHeld().payload > 0);
    CHECK(service.PollResults(subs[0]).empty());
}

void
TestScopedRetirementOfAQueuedJobRedispatches()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);
    auto sub = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    CHECK(sub.IsValid());

    // A scoped edit purges the queued job and gives the time a fresh fence
    // token; the demand still stands, so the owner's pump re-admits it.
    CHECK(h.scheduler->CancelGenerationTimes(a.root, {UsdTimeCode(12.0)}) == 1);
    CHECK(h.scheduler->Stats().explicitCanceled == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(h.scheduler->DrainQueueForTesting() == 0);
    CHECK(h.laneRuns == 0);
    CHECK(h.cache.CoverageRecordCount() == 0);

    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    const std::vector<RigExecOutputPacket> answered = service.PollResults(sub);
    CHECK(AllReady(answered));
    CHECK(FirstCell(answered[0].payload) ==
          MarkerFor("stage-A", frame, h.inputRevision));
}

void
TestGenerationBumpWhileRunningPublishesNothing()
{
    Harness h(1);
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    h.gateOpen = false;
    auto sub = service.Subscribe(
        a.root, RequestAt(RigExecOutputTime::Default(), {a.scalar}));
    CHECK(sub.IsValid());
    CHECK(h.WaitUntilRunning());
    CHECK(h.laneRuns == 0);
    // The edit lands while the bodies are running.
    h.scheduler->CancelGeneration(a.root);
    h.OpenGate();
    h.scheduler->WaitUntilIdle();

    CHECK(h.laneRuns == 1);
    CHECK(h.scheduler->Stats().explicitDeclinedGeneration == 1);
    CHECK(h.cache.CoverageRecordCount() == 0);
    const std::vector<RigExecOutputPacket> stale = service.PollResults(sub);
    CHECK(stale.size() == 1);
    CHECK(stale[0].status == RigExecOutputStatus::Invalidated);
    CHECK(h.cache.CoverageHeld().payload == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);

    // The owner's pump re-captures against the live generation and lands.
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    h.scheduler->WaitUntilIdle();
    CHECK(h.laneRuns == 2);
    const std::vector<RigExecOutputPacket> fresh = service.PollResults(sub);
    CHECK(AllReady(fresh));
    CHECK(h.scheduler->Stats().explicitPublished == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
}

void
TestScopedRetirementWhileRunningInvalidates()
{
    Harness h(1);
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);

    h.gateOpen = false;
    auto sub = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    CHECK(sub.IsValid());
    CHECK(h.WaitUntilRunning());
    // A scoped edit names this time: its running job loses the fence while an
    // unaffected time keeps its own token.
    CHECK(h.scheduler->CancelGenerationTimes(a.root, {UsdTimeCode(12.0)}) == 0);
    h.OpenGate();
    h.scheduler->WaitUntilIdle();

    CHECK(h.laneRuns == 1);
    CHECK(h.scheduler->Stats().explicitDeclinedGeneration == 1);
    CHECK(h.cache.CoverageRecordCount() == 0);
    const std::vector<RigExecOutputPacket> retired = service.PollResults(sub);
    CHECK(retired.size() == 1);
    CHECK(retired[0].status == RigExecOutputStatus::Invalidated);
    CHECK(retired[0].reason.find("retired") != std::string::npos);
    CHECK(h.cache.CoverageHeld().payload == 0);

    service.PumpCallbacks();
    h.scheduler->WaitUntilIdle();
    CHECK(h.laneRuns == 2);
    CHECK(AllReady(service.PollResults(sub)));
}

void
TestExactTimesAcrossRigsAndStages()
{
    Harness h;
    const RigFixture a = StageA();
    const RigFixture b = StageB();
    h.catalogs[a.root] = a.catalog;
    h.catalogs[b.root] = b.catalog;
    auto service = h.MakeService();

    const RigExecOutputTime halfPastFive = TimeOf(5.5);
    const RigExecOutputTime zero = TimeOf(0.0);
    const RigExecOutputTime negative = TimeOf(-3.25);
    auto aHalf = service.Subscribe(a.root, RequestAt(halfPastFive, {a.scalar}));
    auto aDefault = service.Subscribe(
        a.root, RequestAt(RigExecOutputTime::Default(), {a.scalar}));
    auto aZero = service.Subscribe(a.root, RequestAt(zero, {a.scalar}));
    auto bDefault =
        service.Subscribe(b.root, RequestAt(RigExecOutputTime::Default(),
                                            {b.transform, b.points}));
    auto bNegative = service.Subscribe(b.root, RequestAt(negative, {b.points}));
    CHECK(aHalf.IsValid() && aDefault.IsValid() && aZero.IsValid() &&
          bDefault.IsValid() && bNegative.IsValid());
    CHECK(service.Stats().framesDemanded == 5);
    CHECK(h.scheduler->Stats().explicitAccepted == 5);

    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 5);
    CHECK(h.cache.CoverageRecordCount() == 5);

    struct Case {
        RigExecSubscriptionHandle *sub;
        const char *identity;
        RigExecOutputTime time;
        size_t count;
    } cases[] = {
        {&aHalf, "stage-A", halfPastFive, 1},
        {&aDefault, "stage-A", RigExecOutputTime::Default(), 1},
        {&aZero, "stage-A", zero, 1},
        {&bDefault, "stage-B", RigExecOutputTime::Default(), 2},
        {&bNegative, "stage-B", negative, 1},
    };
    for (const Case &case_ : cases) {
        const std::vector<RigExecOutputPacket> packets =
            service.PollResults(*case_.sub);
        CHECK(packets.size() == case_.count);
        CHECK(AllReady(packets));
        const double marker =
            MarkerFor(case_.identity, case_.time, h.inputRevision);
        for (const RigExecOutputPacket &packet : packets) {
            CHECK(packet.time == case_.time);
            CHECK(FirstCell(packet.payload) == marker);
        }
    }
    // The default frame is its own time: it never merged with frame zero.
    const std::vector<RigExecOutputPacket> fromDefault =
        service.PollResults(aDefault);
    const std::vector<RigExecOutputPacket> fromZero =
        service.PollResults(aZero);
    CHECK(fromDefault[0].payload.bytes != fromZero[0].payload.bytes);
    CHECK(FirstCell(fromDefault[0].payload) != FirstCell(fromZero[0].payload));
    CHECK(service.Stats().pinnedPayloads == 6);
}

void
TestExplicitDemandRunsWhileWarmingIsOff()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const bool warmingAtStart = RigExecBackgroundWarmingEnabled();

    TfSetenv("RIGEXEC_FRAME_CACHE", "warm-off");
    CHECK(!RigExecBackgroundWarmingEnabled());
    CHECK(!h.scheduler->Enqueue(
        a.root, UsdTimeCode(3.0), RigExecWarmPriority::Neighbor,
        h.scheduler->CurrentGeneration(a.root), [](const RigExecWarmRequest &) {
            return RigExecWarmOutcome::Published;
        }));

    auto sub = service.Subscribe(a.root,
                                 RequestAt(TimeOf(12.0), {a.scalar, a.points}));
    CHECK(sub.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    CHECK(AllReady(service.PollResults(sub)));
    TfSetenv("RIGEXEC_FRAME_CACHE", "on");
    CHECK(RigExecBackgroundWarmingEnabled() == warmingAtStart);
}

void
TestCoverageHitPullsNothing()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputRequest request =
        RequestAt(TimeOf(12.0), {a.scalar, a.points});

    auto first = service.Subscribe(a.root, request);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    CHECK(AllReady(service.PollResults(first)));

    const int capturesBefore = h.captures;
    const size_t acceptedBefore = h.scheduler->Stats().explicitAccepted;
    auto second = service.Subscribe(a.root, request);
    CHECK(second.IsValid());
    // The capture names the generation the frame stands at now, so it runs;
    // the record at that key already answers, so no job and no lane pull
    // happens for the second client.
    CHECK(h.captures == capturesBefore + 1);
    CHECK(h.scheduler->Stats().explicitAccepted == acceptedBefore);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(service.Stats().coverageHits >= 1);
    CHECK(AllReady(service.PollResults(second)));
    CHECK(h.laneRuns == 1);
    // The second client reads the record's bytes, it does not copy them into
    // a second pin.
    CHECK(service.Stats().pinnedPayloads == 2);
}

void
TestBoundsRefuseBeforeAllocating()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    // The cache's cap refuses the job before the lane allocates anything.
    h.cache.SetByteCap(256);
    auto sub = service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(sub.IsValid());
    CHECK(service.Stats().declinedCoverageBudget == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 0);
    CHECK(h.cache.CoverageRecordCount() == 0);
    CHECK(TotalBytes(h.cache.CoverageHeld()) == 0);
    const std::vector<RigExecOutputPacket> declined = service.PollResults(sub);
    CHECK(declined.size() == 1);
    CHECK(declined[0].status == RigExecOutputStatus::Pending);
    CHECK(declined[0].reason.find("budget") != std::string::npos);

    h.cache.SetByteCap(size_t(8) << 20);
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    h.scheduler->DrainQueueForTesting();
    CHECK(AllReady(service.PollResults(sub)));

    // The pool's own byte bound declines beside the cache's, and the coverage
    // reservation comes back with the refused admission.
    const RigExecCoverageBytes standing = h.cache.CoverageHeld();
    h.scheduler->SetExplicitByteCap(64);
    auto other = service.Subscribe(a.root, RequestAt(TimeOf(20.0), {a.points}));
    CHECK(other.IsValid());
    CHECK(h.scheduler->Stats().explicitDeclinedBytes == 1);
    CHECK(service.Stats().declinedAdmission == 1);
    CHECK(h.cache.CoverageHeld() == standing);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    h.scheduler->SetExplicitByteCap(size_t(8) << 20);
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    h.scheduler->DrainQueueForTesting();
    CHECK(AllReady(service.PollResults(other)));
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(TotalBytes(h.cache.CoverageHeld()) > TotalBytes(standing));
}

void
TestCaptureFailureAndBadGeneration()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    h.captureOk = false;
    auto failed =
        service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(failed.IsValid());
    CHECK(service.Stats().captureFailures == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 0);
    const std::vector<RigExecOutputPacket> refused =
        service.PollResults(failed);
    CHECK(refused.size() == 1);
    CHECK(refused[0].status == RigExecOutputStatus::Failed);
    CHECK(refused[0].reason.find("capture") != std::string::npos);
    h.captureOk = true;
    service.PumpCallbacks();
    h.scheduler->DrainQueueForTesting();
    CHECK(AllReady(service.PollResults(failed)));

    // A capture naming another compiled program answers Invalidated: the
    // coverage key and the admission key would name an epoch this catalog is
    // not.
    h.programIdentityOverride = a.epoch + 40;
    auto other = service.Subscribe(a.root, RequestAt(TimeOf(20.0), {a.points}));
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    const std::vector<RigExecOutputPacket> mismatch =
        service.PollResults(other);
    CHECK(mismatch.size() == 1);
    CHECK(mismatch[0].status == RigExecOutputStatus::Invalidated);
    h.programIdentityOverride = 0;
}

void
TestCompletionCallback()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    std::vector<RigExecSubscriptionEvent> seen;
    service.SetCompletionCallback(
        [&seen](const RigExecSubscriptionEvent &event) {
            seen.push_back(event);
        });

    auto done = service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    h.scheduler->DrainQueueForTesting();
    CHECK(service.Stats().callbacksQueued == 1);
    CHECK(service.PumpCallbacks() == 1);
    CHECK(seen.size() == 1);
    CHECK(seen[0].status == RigExecOutputStatus::Ready);
    CHECK(seen[0].subscription == done.Id());
    CHECK(seen[0].answered == 1);

    auto gone = service.Subscribe(a.root, RequestAt(TimeOf(13.0), {a.points}));
    h.scheduler->DrainQueueForTesting();
    service.Close(gone);
    CHECK(service.PumpCallbacks() == 0);
    CHECK(seen.size() == 1);
}

// An edit at a frame whose answers already stand moves the generation, not the
// time or the demand: the next pump has to notice through its own capture and
// re-earn the answer instead of handing the retired one out again.
void
TestEditAtTheSameFrameRedemandsTheAnswer()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);
    auto sub =
        service.Subscribe(a.root, RequestAt(frame, {a.scalar, a.points}));
    CHECK(sub.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    const std::vector<RigExecOutputPacket> before = service.PollResults(sub);
    CHECK(AllReady(before));
    const double oldMarker = MarkerFor("stage-A", frame, h.inputRevision);
    for (const RigExecOutputPacket &packet : before) {
        CHECK(FirstCell(packet.payload) == oldMarker);
        CHECK(packet.inputRevision == h.inputRevision);
    }

    // Same frame, same outputs, new input value.
    h.inputRevision = 2;
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    CHECK(service.Stats().coverageHits == 0);
    const std::vector<RigExecOutputPacket> waiting = service.PollResults(sub);
    CHECK(AllPending(waiting));
    for (const RigExecOutputPacket &packet : waiting) {
        CHECK(packet.payload.IsEmpty());
    }

    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 2);
    const std::vector<RigExecOutputPacket> after = service.PollResults(sub);
    CHECK(AllReady(after));
    const double newMarker = MarkerFor("stage-A", frame, h.inputRevision);
    CHECK(newMarker != oldMarker);
    for (const RigExecOutputPacket &packet : after) {
        CHECK(FirstCell(packet.payload) == newMarker);
        CHECK(packet.inputRevision == 2);
    }
    // The retired record stays in the cache; it is simply no longer the key
    // this frame is probed by.
    CHECK(h.cache.CoverageRecordCount() == 2);
    // And the packets handed out before the edit still read what they read
    // then, with their payloads alive.
    for (const RigExecOutputPacket &packet : before) {
        CHECK(FirstCell(packet.payload) == oldMarker);
        CHECK(packet.inputRevision == 1);
    }
    // Each poll holds its own generation's leases until the client leaves.
    CHECK(service.Stats().pinnedPayloads == 4);
}

// A subscription made at one compile epoch cannot be answered from another:
// the capture would run the old epoch's job closure over the new program and
// label the answers with the retired epoch.
void
TestExistingSubscriptionRefusesARecompiledRig()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);
    auto sub = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    CHECK(sub.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    const std::vector<RigExecOutputPacket> answered = service.PollResults(sub);
    CHECK(AllReady(answered));

    // The host's catalog hook now names another epoch: the rig recompiled.
    const RigFixture recompiled = StageAAt(a.epoch + 1);
    h.catalogs[a.root] = recompiled.catalog;
    service.PumpCallbacks();
    const std::vector<RigExecOutputPacket> refused = service.PollResults(sub);
    CHECK(refused.size() == 1);
    CHECK(refused[0].status == RigExecOutputStatus::Invalidated);
    CHECK(refused[0].reason.find("epoch") != std::string::npos);
    CHECK(refused[0].payload.IsEmpty());
    service.PumpCallbacks();
    CHECK(h.laneRuns == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(service.PollResults(sub)[0].status ==
          RigExecOutputStatus::Invalidated);

    // Handles from the current catalog are answered afresh.
    auto fresh =
        service.Subscribe(a.root, RequestAt(frame, {recompiled.scalar}));
    CHECK(fresh.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 2);
    const std::vector<RigExecOutputPacket> now = service.PollResults(fresh);
    CHECK(AllReady(now));
    CHECK(now[0].programIdentity == a.epoch + 1);
    CHECK(now[0].payload.bytes != answered[0].payload.bytes);
}

// Destroying the service with clients still live releases every payload lease
// it holds, and the release walks the pin map it is editing.
void
TestServiceDestructionReleasesEveryLivePin()
{
    Harness h;
    const RigFixture a = StageA();
    const RigFixture b = StageB();
    h.catalogs[a.root] = a.catalog;
    h.catalogs[b.root] = b.catalog;
    // The handles outlive the service on purpose: the shutdown pass, not a
    // client closing, is what has to release the pins.
    std::vector<RigExecSubscriptionHandle> subs;
    {
        auto service = h.MakeService();
        subs.push_back(service.Subscribe(
            a.root, RequestAt(TimeOf(12.0), {a.scalar, a.points})));
        subs.push_back(
            service.Subscribe(a.root, RequestAt(TimeOf(13.0), {a.transform})));
        subs.push_back(service.Subscribe(
            b.root, RequestAt(TimeOf(14.0), {b.points, b.transform})));
        subs.push_back(service.Subscribe(
            b.root, RequestAt(RigExecOutputTime::Default(), {b.scalar})));
        h.scheduler->DrainQueueForTesting();
        CHECK(h.laneRuns == 4);
        for (const RigExecSubscriptionHandle &sub : subs) {
            CHECK(AllReady(service.PollResults(sub)));
        }
        CHECK(service.Stats().pinnedPayloads == 6);
        CHECK(h.cache.CoverageHeld().pinned > 0);
    }
    // Every lease came back exactly once: the cache charges no pinned bytes
    // for a service that no longer exists, and the records stay.
    CHECK(h.cache.CoverageHeld().pinned == 0);
    CHECK(h.cache.CoverageRecordCount() == 4);
    CHECK(h.cache.CoverageHeld().payload > 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
}

// The last client to leave a frame takes its queued job and its reservation
// with it, whether it closes or moves its demand.
void
TestLastClientLeavingAQueuedJobRetiresIt()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);
    auto first = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    auto second = service.Subscribe(a.root, RequestAt(frame, {a.points}));
    CHECK(first.IsValid() && second.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() > 0);

    // One client leaving leaves the shared queued job standing.
    service.Close(first);
    CHECK(h.scheduler->Stats().explicitCanceled == 0);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(service.Stats().demandsInFlight == 1);

    // The last one retires it: the queue loses the job, the group's in-flight
    // key goes, and the reservation comes back.
    service.Close(second);
    CHECK(h.scheduler->Stats().explicitCanceled == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(service.Stats().framesDemanded == 0);
    CHECK(h.scheduler->DrainQueueForTesting() == 0);
    CHECK(h.laneRuns == 0);

    // The same frame demanded again is admitted again.
    auto later = service.Subscribe(a.root, RequestAt(frame, {a.points}));
    CHECK(later.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 2);

    // An update that moves the demand retires that frame's job while it is
    // still only queued, and the queue really loses it.
    const size_t canceledBefore = h.scheduler->Stats().explicitCanceled;
    const RigExecSubscriptionReceipt moved = service.UpdateSubscription(
        later, RequestAt(TimeOf(20.0), {a.transform}));
    CHECK(moved.IsAccepted());
    CHECK(h.scheduler->Stats().explicitCanceled == canceledBefore + 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);
    CHECK(service.Stats().framesDemanded == 1);
    CHECK(h.scheduler->DrainQueueForTesting() == 1);
    CHECK(h.laneRuns == 1);
    CHECK(AllReady(service.PollResults(later)));
}

// A frame that already answered is answerable only from the generation its
// latest capture named. When the replacement work that generation needs is
// refused -- the coverage budget, the pool's byte bound, or a capture that
// cannot run -- the retired payload stops answering the frame, while the
// packet handed out before the edit keeps its bytes.
void
TestRefusalOfReplacementWorkEndsTheRetiredAnswer()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);
    auto sub = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    CHECK(sub.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    const std::vector<RigExecOutputPacket> before = service.PollResults(sub);
    CHECK(AllReady(before));
    const double oldMarker = MarkerFor("stage-A", frame, h.inputRevision);
    CHECK(FirstCell(before[0].payload) == oldMarker);
    CHECK(h.cache.CoverageRecordCount() == 1);

    // An edit at the answered frame, and the coverage budget declines the job
    // the new generation needs.
    h.inputRevision = 2;
    h.captureBytes = size_t(1) << 40;
    service.PumpCallbacks();
    CHECK(service.Stats().declinedCoverageBudget == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    const std::vector<RigExecOutputPacket> overBudget =
        service.PollResults(sub);
    CHECK(overBudget.size() == 1);
    CHECK(overBudget[0].status == RigExecOutputStatus::Pending);
    CHECK(overBudget[0].reason.find("budget") != std::string::npos);
    CHECK(overBudget[0].payload.IsEmpty());

    // The pool's own byte bound declines the same frame, and its coverage
    // reservation comes back with the refusal.
    h.captureBytes = 4096;
    h.scheduler->SetExplicitByteCap(64);
    service.PumpCallbacks();
    CHECK(service.Stats().declinedAdmission == 1);
    CHECK(h.scheduler->Stats().explicitDeclinedBytes == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    const std::vector<RigExecOutputPacket> declined = service.PollResults(sub);
    CHECK(declined.size() == 1);
    CHECK(declined[0].status == RigExecOutputStatus::Pending);
    CHECK(declined[0].reason.find("pool declined") != std::string::npos);
    CHECK(declined[0].payload.IsEmpty());

    // A capture that cannot read the frame leaves it Failed: nothing validated
    // this frame's generation, so no standing record answers it.
    h.scheduler->SetExplicitByteCap(size_t(8) << 20);
    h.captureOk = false;
    service.PumpCallbacks();
    CHECK(service.Stats().captureFailures == 1);
    const std::vector<RigExecOutputPacket> uncaptured =
        service.PollResults(sub);
    CHECK(uncaptured.size() == 1);
    CHECK(uncaptured[0].status == RigExecOutputStatus::Failed);
    CHECK(uncaptured[0].reason.find("capture") != std::string::npos);
    CHECK(uncaptured[0].payload.IsEmpty());
    h.captureOk = true;

    // The retired record stayed in the cache the whole time, simply not
    // consulted: lifting the refusals re-earns the frame at the live
    // generation, and the packet handed out before the edit never moved.
    CHECK(h.cache.CoverageRecordCount() == 1);
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 2);
    const std::vector<RigExecOutputPacket> after = service.PollResults(sub);
    CHECK(AllReady(after));
    const double newMarker = MarkerFor("stage-A", frame, h.inputRevision);
    CHECK(newMarker != oldMarker);
    CHECK(FirstCell(after[0].payload) == newMarker);
    CHECK(after[0].inputRevision == 2);
    CHECK(h.cache.CoverageRecordCount() == 2);
    CHECK(FirstCell(before[0].payload) == oldMarker);
    CHECK(before[0].inputRevision == 1);
}

void
TestEstimateAndStatusText()
{
    RigExecFrameInputs inputs;
    CHECK(RigExecEstimateCaptureBytes(inputs) == 0);
    inputs.Add(SdfPath("/R/Joints/A"), VtValue(1.f));
    CHECK(RigExecEstimateCaptureBytes(inputs) >= sizeof(RigExecSampledInput));
    CHECK(inputs.values.size() == 1);
    CHECK(std::string(RigExecSubscriptionStatusText(
              RigExecSubscriptionStatus::NothingProducible)) ==
          "no producible output");
}

} // namespace

int
main()
{
    TestValidationAndRaii();
    TestTwoClientDedupAndExpansion();
    TestUpdateAtomicity();
    TestCancellationKeepsSharedDemand();
    TestDestroyAllReleasesEverything();
    TestScopedRetirementOfAQueuedJobRedispatches();
    TestGenerationBumpWhileRunningPublishesNothing();
    TestScopedRetirementWhileRunningInvalidates();
    TestExactTimesAcrossRigsAndStages();
    TestExplicitDemandRunsWhileWarmingIsOff();
    TestCoverageHitPullsNothing();
    TestEditAtTheSameFrameRedemandsTheAnswer();
    TestExistingSubscriptionRefusesARecompiledRig();
    TestRefusalOfReplacementWorkEndsTheRetiredAnswer();
    TestServiceDestructionReleasesEveryLivePin();
    TestLastClientLeavingAQueuedJobRetiresIt();
    TestBoundsRefuseBeforeAllocating();
    TestCaptureFailureAndBadGeneration();
    TestCompletionCallback();
    TestEstimateAndStatusText();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("all tests passed\n");
    return 0;
}
