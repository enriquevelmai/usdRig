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
#include <algorithm>
#include <functional>
#include <limits>
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

// `gpuPointsChain` marks the points chain GPU-resident before discovery, so
// its revisions keep their semantic address in the demand index and lose the
// CPU payload -- how a real rig exports an output no run can answer.
RigFixture
BuildRig(const char *stageText, const SdfPath &root, const std::string &target,
         const std::string &transformTarget, const std::string &pointsTarget,
         const std::string &identity, uint64_t epoch,
         bool gpuPointsChain = false)
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
    if (gpuPointsChain) {
        RigExecBakedProgramImpl &stepGraph =
            const_cast<RigExecBakedProgramImpl &>(program->GetStepGraph());
        for (auto &chain : stepGraph.chains) {
            if (chain.target.GetAsString() == pointsTarget) {
                chain.gpu = true;
            }
        }
    }
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
    bool laneOk = true;
    uint64_t programIdentityOverride = 0;
    uint64_t inputRevision = 1;
    size_t captureBytes = 4096;
    bool gateOpen = true;

    // Observations.
    // A test can install this to read the service's demand count at the
    // moment a job enters the lane: the pool may see a job no earlier than
    // the service owns it, so the running minimum is a per-job ordering check.
    std::function<size_t()> demandOwnedProbe;
    std::atomic<size_t> demandOwnedMin{std::numeric_limits<size_t>::max()};
    std::atomic<size_t> demandOwnedAtQueued{0};
    std::atomic<int> demandOwnedObservations{0};
    // The pool's admission-return seam: runs on the admitting thread with the
    // job queued, counted and announced to the lanes, holding no scheduler
    // lock, just before that same AdmitExplicitOutput returns. SealAdmission-
    // Return installs it; the hook names the exact key that admission queued.
    std::function<void(const RigExecExplicitJobKey &key)> atAdmissionReturn;
    // WatchAdmission installs the scheduler's transition hook: Queued
    // fires on the admitting thread inside the admission call, Finished
    // after the pool closed the job's books. The probes land on both.
    std::atomic<int> queuedExplicit{0};
    std::atomic<int> finishedExplicit{0};
    std::atomic<int> captures{0};
    std::atomic<int> laneRuns{0};
    // Counts the jobs that reached the gated lane and wait there.
    std::atomic<int> laneArrivals{0};
    // One record per explicit job's terminal transition, in the order the
    // pool reported it, kept under the seam's own lock so a wait for one
    // frame's answer never holds the scheduler lock.
    struct _Terminal {
        SdfPath rig;
        bool timeIsDefault = false;
        double timeValue = 0.0;
        RigExecWarmOutcome outcome = RigExecWarmOutcome::Published;
    };
    std::vector<std::vector<std::string>> launchOutputs;
    std::vector<_Terminal> terminals;
    std::mutex logMutex;
    std::mutex gateMutex;
    std::mutex terminalMutex;
    std::condition_variable gateCv;
    std::condition_variable terminalCv;

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
            // The arrival is the barrier the tests wait on: a job that reached
            // the gate is inside its bodies, so an edit applied afterwards
            // lands while they run.
            {
                std::lock_guard<std::mutex> lock(gateMutex);
                ++laneArrivals;
            }
            gateCv.notify_all();
            WaitForGate();
            if (!laneOk) {
                *error = "the harness lane declined the run";
                return false;
            }
            ++laneRuns;
            if (demandOwnedProbe) {
                _NoteDemandOwned(demandOwnedProbe());
            }
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
                // A production extraction stamps the revision its run left, so
                // the harness names one per captured generation: a poll's
                // handle can then be pinned by a later request.
                packet.handle.valueRevision =
                    requested.captured.inputRevision * 1000ull + 17ull;
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

    void _NoteDemandOwned(size_t seen)
    {
        ++demandOwnedObservations;
        size_t prev = demandOwnedMin.load(std::memory_order_relaxed);
        while (seen < prev &&
               !demandOwnedMin.compare_exchange_weak(prev, seen)) {
        }
    }

    // Probe every explicit queue move: the demand the pool is about to
    // see is read on the admitting thread, inside the admission call.
    void WatchAdmission()
    {
        scheduler->SetTransitionHook([this](const RigExecWarmTransition
                                                &transition) {
            if (transition.workClass != RigExecBackgroundWorkClass::Explicit) {
                return;
            }
            if (transition.kind == RigExecWarmTransitionKind::Queued) {
                ++queuedExplicit;
                if (demandOwnedProbe) {
                    const size_t owned = demandOwnedProbe();
                    demandOwnedAtQueued.store(owned, std::memory_order_relaxed);
                    _NoteDemandOwned(owned);
                }
            } else if (transition.kind == RigExecWarmTransitionKind::Finished) {
                ++finishedExplicit;
                {
                    std::lock_guard<std::mutex> lock(terminalMutex);
                    terminals.push_back(
                        _Terminal{transition.rig, transition.timeIsDefault,
                                  transition.timeValue, transition.outcome});
                }
                terminalCv.notify_all();
                gateCv.notify_all();
            }
        });
    }

    // Hands the pool the seam at the end of an accepted explicit admission.
    void SealAdmissionReturn()
    {
        scheduler->SetAdmissionReturnHookForTesting(
            [this](const RigExecExplicitJobKey &key) {
                if (atAdmissionReturn) {
                    atAdmissionReturn(key);
                }
            });
    }

    const _Terminal *_FindTerminalLocked(const RigExecExplicitJobKey &key) const
    {
        for (const _Terminal &terminal : terminals) {
            if (terminal.rig == key.rig &&
                terminal.timeIsDefault == key.time.IsDefault() &&
                (key.time.IsDefault() ||
                 terminal.timeValue == key.time.TimeCode())) {
                return &terminal;
            }
        }
        return nullptr;
    }

    // Bounded wait for the terminal transition of the explicit job at \p key.
    // The caller holds no scheduler, service or cache lock while waiting and
    // holds none when this returns, so the lane can close the job's books and
    // the seam can read them.
    struct TerminalAnswer {
        bool landed = false;
        RigExecWarmOutcome outcome = RigExecWarmOutcome::Published;
    };

    TerminalAnswer WaitTerminal(const RigExecExplicitJobKey &key)
    {
        TerminalAnswer answer;
        std::unique_lock<std::mutex> wait(terminalMutex);
        answer.landed =
            terminalCv.wait_for(wait, std::chrono::seconds(10), [&] {
                return _FindTerminalLocked(key) != nullptr;
            });
        if (answer.landed) {
            answer.outcome = _FindTerminalLocked(key)->outcome;
        }
        wait.unlock();
        return answer;
    }

    /// Waits until a job has reached the gated lane and waits there. The
    /// pool's running count turns on earlier, before the job's pre-body
    /// fences, so waiting on it alone lets a test land an edit or a cancel
    /// while the job still declines before its bodies run.
    bool WaitUntilRunning()
    {
        for (int spin = 0; spin < 4000; ++spin) {
            {
                std::lock_guard<std::mutex> lock(gateMutex);
                if (laneArrivals >= 1) {
                    return true;
                }
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

// A demand that widens while a frame's job is queued leaves outputs that job
// never ran. Between that job's completion and the next pump, the un-run
// output polls Pending with no payload and its event says the frame is not
// Ready; after the redispatch it reads Ready with the payload the frame's
// second run produced.
void
TestExpandedOutputStaysPendingUntilItsRun()
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
    const RigExecOutputTime frame = TimeOf(12.0);

    auto narrow = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    auto widened = service.Subscribe(a.root, RequestAt(frame, {a.transform}));
    CHECK(narrow.IsValid());
    CHECK(widened.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 1);

    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    // The launched mask stays exactly what was admitted.
    CHECK(h.LaunchOutput(0).size() == 1);
    CHECK(h.LaunchNames(0, a.scalar.semanticKey));
    CHECK(!h.LaunchNames(0, a.transform.semanticKey));

    const std::vector<RigExecOutputPacket> answered =
        service.PollResults(narrow);
    CHECK(answered.size() == 1);
    CHECK(AllReady(answered));
    const double marker = MarkerFor("stage-A", frame, h.inputRevision);
    CHECK(FirstCell(answered.front().payload) == marker);

    const std::vector<RigExecOutputPacket> owed = service.PollResults(widened);
    CHECK(owed.size() == 1);
    CHECK(owed.front().status == RigExecOutputStatus::Pending);
    CHECK(owed.front().payload.IsEmpty());
    CHECK(owed.front().reason.find("next pump") != std::string::npos);

    // The completion event says the same for every client of that frame: the
    // record answers one of the two demanded outputs, so the frame is not
    // Ready and no client is told it is.
    CHECK(service.PumpCallbacks() == 2);
    CHECK(seen.size() == 2);
    for (const RigExecSubscriptionEvent &event : seen) {
        CHECK(event.status == RigExecOutputStatus::Pending);
        CHECK(event.answered == 1);
        CHECK(event.demanded == 1);
        CHECK(event.reason.find("demanded outputs") != std::string::npos);
    }

    // The next pump runs the rest as the union, and the owed output then reads
    // Ready with a real payload.
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 2);
    CHECK(h.LaunchOutput(1).size() == 2);
    CHECK(h.LaunchNames(1, a.transform.semanticKey));
    CHECK(h.LaunchNames(1, a.scalar.semanticKey));

    const std::vector<RigExecOutputPacket> after = service.PollResults(widened);
    CHECK(after.size() == 1);
    CHECK(AllReady(after));
    CHECK(FirstCell(after.front().payload) == marker);
    CHECK(!after.front().payload.IsEmpty());
    CHECK(AllReady(service.PollResults(narrow)));

    CHECK(service.PumpCallbacks() == 2);
    CHECK(seen.size() == 4);
    for (size_t i = 2; i < seen.size(); ++i) {
        CHECK(seen[i].status == RigExecOutputStatus::Ready);
        CHECK(seen[i].answered == 2);
        CHECK(seen[i].demanded == 2);
        CHECK(seen[i].reason.empty());
    }
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
TestPolledHandleCarriesTheRecordRevision()
{
    Harness h;
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    auto open = service.Subscribe(
        a.root, RequestAt(TimeOf(12.0), {a.scalar, a.points}));
    CHECK(open.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 1);
    const std::vector<RigExecOutputPacket> polled = service.PollResults(open);
    CHECK(polled.size() == 2);
    CHECK(AllReady(polled));

    // The poll copies the coverage record's packet, so its handle names the
    // value revision that record materialized even though the request asked
    // with unconstrained handles.
    RigExecCoverageKey key;
    key.rigIdentity = a.catalog->GetRigIdentity();
    key.compileEpoch = a.epoch;
    key.time = TimeOf(12.0);
    key.generation.programIdentity = a.epoch;
    key.generation.inputRevision = h.inputRevision;
    RigExecCoverageRecord record;
    CHECK(h.cache.LookupCoverage(key, &record));
    CHECK(record.packets.size() == 2);
    uint64_t standing = 0;
    for (const RigExecOutputPacket &packet : polled) {
        const RigExecOutputPacket *held = nullptr;
        for (const RigExecOutputPacket &candidate : record.packets) {
            if (candidate.handle.SameTarget(packet.handle))
                held = &candidate;
        }
        CHECK(held != nullptr);
        if (!held)
            continue;
        CHECK(packet.handle.valueRevision == held->handle.valueRevision);
        CHECK(packet.handle.valueRevision != 0);
        standing = held->handle.valueRevision;
    }
    CHECK(standing != 0);

    // A request pinned to the revision just read answers from that record.
    std::vector<RigExecOutputHandle> pinned;
    for (const RigExecOutputPacket &packet : polled)
        pinned.push_back(packet.handle);
    auto exact = service.Subscribe(a.root, RequestAt(TimeOf(12.0), pinned));
    CHECK(exact.IsValid());
    h.scheduler->DrainQueueForTesting();
    const std::vector<RigExecOutputPacket> exactPackets =
        service.PollResults(exact);
    CHECK(exactPackets.size() == 2);
    CHECK(AllReady(exactPackets));
    for (const RigExecOutputPacket &packet : exactPackets)
        CHECK(packet.handle.valueRevision == standing);
    CHECK(h.laneRuns == 1);

    // A revision the record does not hold is a different answer: the poll says
    // so with no payload, and it pulls no second run to invent one.
    std::vector<RigExecOutputHandle> stale = {a.scalar};
    stale[0].valueRevision = standing + 1;
    auto staleSub = service.Subscribe(a.root, RequestAt(TimeOf(12.0), stale));
    CHECK(staleSub.IsValid());
    h.scheduler->DrainQueueForTesting();
    const std::vector<RigExecOutputPacket> stalePackets =
        service.PollResults(staleSub);
    CHECK(stalePackets.size() == 1);
    CHECK(stalePackets[0].status == RigExecOutputStatus::Invalidated);
    CHECK(stalePackets[0].payload.IsEmpty());
    CHECK(stalePackets[0].reason.find("the request named") !=
          std::string::npos);
    CHECK(h.laneRuns == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
}

// A program rebuild can leave the compile epoch standing. The host then
// resolves a catalog of the new compilation, while this subscription's handles
// name the previous one: the service reports the rebuild rather than pulling a
// run no program of this rig could answer.
void
TestProgramRebuildAtTheStandingEpochRefusesTheOldCatalog()
{
    Harness h(1);
    const RigFixture a = StageA();
    const RigFixture rebuilt = StageA();
    // The same rig identity and compile epoch, another compilation: the same
    // operation count and semantic addresses, no shared compiled-program
    // identity.
    CHECK(rebuilt.catalog->GetRigIdentity() == a.catalog->GetRigIdentity());
    CHECK(rebuilt.catalog->GetCompileEpoch() == a.catalog->GetCompileEpoch());
    CHECK(rebuilt.catalog->GetProgramAffinity() !=
          a.catalog->GetProgramAffinity());
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    auto sub = service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(sub.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(AllReady(service.PollResults(sub)));
    CHECK(h.captures == 1);
    CHECK(h.laneRuns == 1);

    // The evaluator rebuilds its program at the standing epoch, and the host
    // resolves the new compilation's catalog for this rig.
    h.catalogs[a.root] = rebuilt.catalog;
    h.scheduler->CancelGeneration(a.root);
    service.PumpCallbacks();
    h.scheduler->WaitUntilIdle();

    const std::vector<RigExecOutputPacket> refused = service.PollResults(sub);
    CHECK(refused.size() == 1);
    CHECK(refused[0].status == RigExecOutputStatus::Invalidated);
    CHECK(refused[0].reason.find("rebuilt its program") != std::string::npos);
    // Nothing was captured or admitted against a catalog this rig's program
    // does not answer.
    CHECK(h.captures == 1);
    CHECK(h.laneRuns == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
}

// A re-subscription with the current catalog recovers while its
// old-compilation sibling stays Invalidated: demand groups carry the
// compilation that minted the handles, so a rebuild at the standing epoch strands
// neither client, and closing the old one after the fresh one exists changes
// nothing for the fresh one.
void
TestFreshSubscriptionRecoversAfterAStandingEpochRebuild()
{
    Harness h(1);
    const RigFixture a = StageA();
    const RigFixture rebuilt = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    auto old = service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(old.IsValid());
    h.scheduler->WaitUntilIdle();
    CHECK(AllReady(service.PollResults(old)));
    CHECK(h.laneRuns == 1);

    h.catalogs[a.root] = rebuilt.catalog;
    h.scheduler->CancelGeneration(a.root);
    service.PumpCallbacks();
    h.scheduler->WaitUntilIdle();
    const std::vector<RigExecOutputPacket> stale = service.PollResults(old);
    CHECK(stale.size() == 1);
    CHECK(stale[0].status == RigExecOutputStatus::Invalidated);

    // Re-subscribing with the current catalog is exactly what that refusal
    // asks for: it lands in its own group and answers.
    auto fresh =
        service.Subscribe(a.root, RequestAt(TimeOf(12.0), {rebuilt.scalar}));
    CHECK(fresh.IsValid());
    h.scheduler->WaitUntilIdle();
    service.PumpCallbacks();
    const std::vector<RigExecOutputPacket> recovered =
        service.PollResults(fresh);
    CHECK(recovered.size() == 1);
    CHECK(AllReady(recovered));
    CHECK(service.Stats().subscriptions == 2);
    // Two compilations of one epoch hold separate frames of demand.
    CHECK(service.Stats().framesDemanded == 2);

    // The old client keeps its own disposition, and it is stable across polls.
    const std::vector<RigExecOutputPacket> stillStale =
        service.PollResults(old);
    CHECK(stillStale.size() == 1);
    CHECK(stillStale[0].status == RigExecOutputStatus::Invalidated);
    CHECK(stillStale[0].reason.find("rebuilt its program") !=
          std::string::npos);

    // A frame no record stands for runs on the new compilation.
    const int runsBefore = h.laneRuns;
    auto freshFrame =
        service.Subscribe(a.root, RequestAt(TimeOf(9.0), {rebuilt.points}));
    CHECK(freshFrame.IsValid());
    h.scheduler->WaitUntilIdle();
    const std::vector<RigExecOutputPacket> atNine =
        service.PollResults(freshFrame);
    CHECK(AllReady(atNine));
    CHECK(h.laneRuns == runsBefore + 1);
    // The frame cache binds a record to the compile epoch and captured
    // generation, not to which build minted the handles, so the standing record
    // answers the re-subscribed frame at that time as well; the new
    // compilation's own run is the one above.

    // Closing the old client strands nothing the fresh demand depends on.
    old.Close();
    service.PumpCallbacks();
    h.scheduler->WaitUntilIdle();
    CHECK(AllReady(service.PollResults(fresh)));
    CHECK(AllReady(service.PollResults(freshFrame)));
    CHECK(service.Stats().subscriptions == 2);
    CHECK(service.Stats().framesDemanded == 2);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
}

// A completion answers only the clients of its own compilation: the rebuild
// leaves the old client with no event from the fresh run, and the fresh client
// with its own.
void
TestRebuildCallbacksStayWithTheirCompilation()
{
    Harness h(1);
    const RigFixture a = StageA();
    const RigFixture rebuilt = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    auto old = service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(old.IsValid());
    h.scheduler->WaitUntilIdle();
    CHECK(AllReady(service.PollResults(old)));

    std::vector<RigExecSubscriptionEvent> events;
    service.SetCompletionCallback(
        [&events](const RigExecSubscriptionEvent &event) {
            events.push_back(event);
        });

    h.catalogs[a.root] = rebuilt.catalog;
    h.scheduler->CancelGeneration(a.root);
    service.PumpCallbacks();
    h.scheduler->WaitUntilIdle();
    // The old frame's refusal is a poll disposition, not a completion: no event
    // names the old client.
    // An event queued by the first run is still waiting when the callback goes
    // in, so measure from here rather than from an empty queue.
    const size_t queued = events.size();

    // The same frame, an output the old frame's run never launched: the fresh
    // compilation has to run it, and its completion belongs to that frame.
    auto fresh =
        service.Subscribe(a.root, RequestAt(TimeOf(12.0), {rebuilt.points}));
    CHECK(fresh.IsValid());
    h.scheduler->WaitUntilIdle();
    service.PumpCallbacks();
    CHECK(AllReady(service.PollResults(fresh)));
    CHECK(events.size() == queued + 1);
    if (events.size() > queued) {
        CHECK(events.back().subscription == fresh.Id());
        CHECK(events.back().status == RigExecOutputStatus::Ready);
        CHECK(events.back().time == TimeOf(12.0));
    }
    // A Ready event for this frame would contradict the old client's own poll,
    // which says the rig rebuilt its program under it.
    for (size_t i = queued; i < events.size(); ++i) {
        CHECK(events[i].subscription != old.Id());
    }
    // The old client is untouched: still Invalidated, still asking for a
    // re-subscribe, and no second run was made for its frame.
    const std::vector<RigExecOutputPacket> stale = service.PollResults(old);
    CHECK(stale.size() == 1);
    CHECK(stale[0].status == RigExecOutputStatus::Invalidated);
    CHECK(h.laneRuns == 2);
}

// A running job from the previous compilation still owns its frame's key when a
// re-subscribed client asks for that frame. The fresh pump defers instead of
// registering a demand the pool would fold into the running job, and the fresh
// client lands once that job reaches its terminal state.
void
TestRunningForeignJobCannotOccupyTheFreshFrame()
{
    Harness h(1);
    const RigFixture a = StageA();
    const RigFixture rebuilt = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const RigExecOutputTime frame = TimeOf(12.0);

    h.gateOpen = false;
    auto old = service.Subscribe(a.root, RequestAt(frame, {a.scalar}));
    CHECK(old.IsValid());
    // The old compilation's job is inside its bodies.
    CHECK(h.WaitUntilRunning());
    CHECK(h.laneRuns == 0);

    // The program rebuilds at the standing epoch while that job runs.
    h.catalogs[a.root] = rebuilt.catalog;
    auto fresh = service.Subscribe(a.root, RequestAt(frame, {rebuilt.points}));
    CHECK(fresh.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.laneRuns == 0);
    const std::vector<RigExecOutputPacket> waiting = service.PollResults(fresh);
    CHECK(waiting.size() == 1);
    CHECK(waiting[0].status == RigExecOutputStatus::Pending);

    // Once the other compilation's job lands, the fresh group's own pump
    // admits and runs the frame it demands.
    h.OpenGate();
    h.scheduler->WaitUntilIdle();
    service.PumpCallbacks();
    h.scheduler->WaitUntilIdle();
    CHECK(h.laneRuns == 2);
    const std::vector<RigExecOutputPacket> recovered =
        service.PollResults(fresh);
    CHECK(AllReady(recovered));
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
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

    // Handles from the current catalog are answered afresh. A completion for
    // that frame answers the current-epoch client only: the stale client's
    // poll says the rig recompiled under it, and no Ready event contradicts
    // that.
    std::vector<RigExecSubscriptionEvent> seen;
    service.SetCompletionCallback(
        [&seen](const RigExecSubscriptionEvent &event) {
            seen.push_back(event);
        });
    auto fresh =
        service.Subscribe(a.root, RequestAt(frame, {recompiled.scalar}));
    CHECK(fresh.IsValid());
    h.scheduler->DrainQueueForTesting();
    CHECK(h.laneRuns == 2);
    CHECK(service.PumpCallbacks() == 1);
    CHECK(seen.size() == 1);
    if (!seen.empty()) {
        CHECK(seen[0].subscription == fresh.Id());
        CHECK(seen[0].status == RigExecOutputStatus::Ready);
    }
    CHECK(sub.IsValid());
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

// Closing a client releases exactly the work that client held. The pool's
// scoped-edit operation is an edit: it re-fences the numeric times, re-fences
// the default frame, and purges that frame's work of both classes, so demand
// retirement names the job's own key instead. Another client's queued
// default-frame job, and unrelated warming at the closed frame, stay queued
// with their tokens standing, and only the retired demand's reservation comes
// back.
void
TestClosingOneClientCancelsOnlyItsOwnJob()
{
    Harness h(/*workers=*/1);
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    h.gateOpen = false;
    // Two numeric frames from the client that closes: the first takes the lane
    // and waits there, the second stays queued behind it. Then a second
    // client's default-frame job, queued last, and warming at the closed frame
    // that belongs to no client at all.
    RigExecOutputRequest owed;
    owed.demands.push_back(RigExecOutputTimeDemand{TimeOf(12.0), {a.scalar}});
    owed.demands.push_back(RigExecOutputTimeDemand{TimeOf(13.0), {a.scalar}});
    RigExecSubscriptionHandle closable = service.Subscribe(a.root, owed);
    const RigExecSubscriptionHandle keeper = service.Subscribe(
        a.root, RequestAt(RigExecOutputTime::Default(), {a.scalar}));
    CHECK(closable.IsValid() && keeper.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 3);
    CHECK(h.WaitUntilRunning());
    // Unrelated warming at the frame being closed belongs to no client.
    // Warming can be off in this process, so its queue entries are only
    // checked where it is available; the demand purge's own books are read
    // either way.
    const bool warming = RigExecBackgroundWarmingEnabled();
    if (warming) {
        CHECK(h.scheduler->Enqueue(a.root, UsdTimeCode(12.0),
                                   RigExecWarmPriority::Sweep,
                                   h.scheduler->CurrentGeneration(a.root),
                                   [](const RigExecWarmRequest &) {
                                       return RigExecWarmOutcome::Published;
                                   }));
    }
    CHECK(h.scheduler->QueuedExplicitKeys(a.root).size() == 2);
    const size_t bytesOwed = h.scheduler->ReservedExplicitBytesFor(a.root);
    // Three identical reservations: the running job, the closed client's
    // queued job, and the other client's queued default-frame job.
    CHECK(bytesOwed > 0 && bytesOwed % 3 == 0);

    service.Close(closable);

    // The closed client's queued job left the pool and exactly its share of
    // the reservation came back. The other client's default-frame job is still
    // queued under its own key, and no time was re-fenced.
    CHECK(service.Stats().demandsInFlight == 1);
    CHECK(service.Stats().retiredInFlight == 0);
    CHECK(service.Stats().framesDemanded == 1);
    CHECK(h.scheduler->Stats().explicitCanceled == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 2);
    const std::vector<RigExecExplicitJobKey> stillOwed =
        h.scheduler->QueuedExplicitKeys(a.root);
    CHECK(stillOwed.size() == 1);
    CHECK(stillOwed.size() == 1 && stillOwed[0].time.IsDefault());
    CHECK(h.scheduler->ReservedExplicitBytesFor(a.root) ==
          bytesOwed - bytesOwed / 3);
    CHECK(h.scheduler->CurrentExplicitFenceToken(a.root, TimeOf(12.0)) == 0);
    CHECK(h.scheduler->CurrentExplicitFenceToken(a.root, TimeOf(13.0)) == 0);
    CHECK(h.scheduler->CurrentExplicitFenceToken(a.root, RigExecOutputTime()) ==
          0);
    CHECK(h.scheduler->CurrentFenceToken(a.root, UsdTimeCode(12.0)) == 0);
    const std::vector<UsdTimeCode> warmQueued =
        h.scheduler->QueuedTimes(a.root);
    CHECK(warmQueued.size() == (warming ? 1u : 0u));
    CHECK(warmQueued.size() == (warming ? 1u : 0u) &&
          (warmQueued.empty() || warmQueued[0] == UsdTimeCode(12.0)));

    // The gate came back on the owner's thread, so the wait cannot hang.
    h.OpenGate();
    h.scheduler->WaitUntilIdle();
    // The closed client's running body published nothing, and nothing
    // re-fenced it; the default client's job ran once under its standing token
    // and answered with no re-dispatch; frame 12's warming ran as well.
    CHECK(h.scheduler->Stats().explicitAccepted == 3);
    CHECK(h.scheduler->Stats().explicitDeclinedInvalid == 1);
    CHECK(h.scheduler->Stats().explicitDeclinedGeneration == 0);
    CHECK(h.scheduler->Stats().explicitPublished == 1);
    CHECK(h.scheduler->Stats().published == (warming ? 1u : 0u));
    CHECK(h.laneRuns == 2);
    CHECK(h.cache.CoverageRecordCount() == 1);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    const std::vector<RigExecOutputPacket> kept = service.PollResults(keeper);
    CHECK(kept.size() == 1);
    CHECK(AllReady(kept));
    CHECK(FirstCell(kept[0].payload) ==
          MarkerFor("stage-A", RigExecOutputTime::Default(), h.inputRevision));
    CHECK(service.PollResults(closable).empty());
}

// Admission notifies the lanes and returns without a completion barrier, so a
// worker can reach its terminal state inside the call. The service owns the
// demand and its launched mask before the pool sees the job, and the mask
// clears only as the job retires, so the frame's next bookkeeping -- the
// coverage answer of an answered frame, or the refusal of a declined one --
// can only be reached with that job's terminal disposition standing.
//
// The seam is the pool's own admission-return boundary: it runs on the
// admitting thread with the job queued, counted and announced to the lanes,
// and with no scheduler lock held, just before that same admission returns.
// The test waits there -- bounded, and holding no lock the lane needs -- for
// the terminal transition of the exact key that admission named, and asserts
// across the wait that the job closed its own demand, in-flight count and
// byte reservation. Nothing pumps, drains, or re-subscribes in between.
void
TestWorkEndingInsideAdmissionKeepsItsDisposition()
{
    Harness h(/*workers=*/1);
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    h.demandOwnedProbe = [&service]() {
        return service.Stats().demandsInFlight;
    };
    h.WatchAdmission();
    h.OpenGate();

    // The seam: the job this admission named has to close terminally before
    // the call that queued it returns. Each phase names one frame, so the
    // terminal match is this job's and no other's.
    int seams = 0;
    RigExecWarmOutcome expected = RigExecWarmOutcome::Published;
    h.atAdmissionReturn = [&](const RigExecExplicitJobKey &key) {
        ++seams;
        const Harness::TerminalAnswer answer = h.WaitTerminal(key);
        CHECK(answer.landed);
        CHECK(answer.outcome == expected);
        // The pool saw this job only once the service owned its demand: the
        // launched mask is registered before admission, never after it.
        CHECK(h.demandOwnedAtQueued.load() == 1);
        // Every book closed by the job itself while this call is still open:
        // the run retired the demand, and the pool released the reservation
        // and the in-flight count. No prune and no pump stands behind these.
        CHECK(service.Stats().demandsInFlight == 0);
        CHECK(service.Stats().retiredInFlight == 0);
        CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
        CHECK(h.scheduler->ReservedExplicitBytes() == 0);
        CHECK(h.scheduler->QueuedExplicitKeys(a.root).empty());
        // Ordering across jobs: this job's terminal transition, and every
        // earlier one, landed while its own admission had not returned.
        CHECK(h.finishedExplicit.load() == seams);
        CHECK(h.queuedExplicit.load() == seams);
    };
    h.SealAdmissionReturn();

    // The answered frame: its job publishes a record, so once Subscribe
    // returns the frame can only answer that record.
    const RigExecSubscriptionHandle answered =
        service.Subscribe(a.root, RequestAt(TimeOf(5.0), {a.scalar}));
    CHECK(answered.IsValid());
    CHECK(seams == 1);
    CHECK(h.queuedExplicit.load() == 1);
    CHECK(h.finishedExplicit.load() == 1);
    // The disposition stood on return: nothing re-admitted the frame, nothing
    // was left as a demand the pool will never see again, and the published
    // answer is what the poll reads.
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(service.Stats().retiredInFlight == 0);
    CHECK(service.Stats().declinedAdmission == 0);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    const std::vector<RigExecOutputPacket> first =
        service.PollResults(answered);
    CHECK(first.size() == 1);
    CHECK(AllReady(first));
    if (first.size() == 1) {
        CHECK(FirstCell(first[0].payload) ==
              MarkerFor("stage-A", TimeOf(5.0), h.inputRevision));
    }
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    CHECK(h.scheduler->Stats().explicitPublished == 1);

    // The refused frame: the lane declines the run, so the frame's answer is
    // that refusal -- Failed with the lane's reason -- standing when
    // Subscribe returns. A demand registered behind the admission would stamp
    // it back to Pending and leave a demand in flight the pool never closes.
    h.laneOk = false;
    expected = RigExecWarmOutcome::DeclinedInvalid;
    const RigExecSubscriptionHandle refused =
        service.Subscribe(a.root, RequestAt(TimeOf(9.0), {a.scalar}));
    CHECK(refused.IsValid());
    CHECK(seams == 2);
    CHECK(h.queuedExplicit.load() == 2);
    CHECK(h.finishedExplicit.load() == 2);
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(service.Stats().retiredInFlight == 0);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(h.scheduler->QueuedExplicitKeys(a.root).empty());
    const std::vector<RigExecOutputPacket> refusedPackets =
        service.PollResults(refused);
    CHECK(refusedPackets.size() == 1);
    for (const RigExecOutputPacket &packet : refusedPackets) {
        CHECK(packet.status == RigExecOutputStatus::Failed);
        CHECK(packet.reason.find("harness lane") != std::string::npos);
    }
    // Only the published job ran a body to the counter, and both admissions
    // were probed with their demand already owned.
    CHECK(h.laneRuns == 1);
    CHECK(h.scheduler->Stats().explicitAccepted == 2);
    CHECK(h.scheduler->Stats().explicitPublished == 1);
    CHECK(h.scheduler->Stats().explicitDeclinedInvalid == 1);
    CHECK(h.demandOwnedMin.load() == 1);
    CHECK(h.demandOwnedObservations.load() == 3);
}

// An output this generation cannot produce answers on its own. It must stay
// out of the demand, where it would refuse the whole requested run and leave
// the supported siblings of the same frame unanswered forever.
void
TestUnsupportedHandleKeepsItsSiblingsRunnable()
{
    Harness h(/*workers=*/1);
    RigFixture a =
        BuildRig(kStageA, SdfPath("/Asset/UnusualContainer/Character"),
                 "/Asset/UnusualContainer/Character/RandomData.amount",
                 "/Asset/UnusualContainer/Character/InputsAnywhere/Drive",
                 "/Asset/UnusualContainer/Geometry/Body.points", "stage-A", 17,
                 /*gpuPointsChain=*/true);
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();

    RigExecSubscriptionReceipt receipt;
    const RigExecSubscriptionHandle sub = service.Subscribe(
        a.root, RequestAt(TimeOf(12.0), {a.scalar, a.points}), &receipt);
    CHECK(sub.IsValid());
    CHECK(receipt.status == RigExecSubscriptionStatus::Accepted);
    CHECK(receipt.unsupported.size() == 1);
    CHECK(receipt.unsupported.size() == 1 &&
          receipt.unsupported[0].semanticKey == a.points.semanticKey);

    h.scheduler->WaitUntilIdle();
    // The one job the pool ran named only the output it can answer.
    CHECK(h.laneRuns == 1);
    CHECK(h.launchOutputs.size() == 1);
    CHECK(h.launchOutputs.size() == 1 && h.launchOutputs[0].size() == 1 &&
          h.launchOutputs[0][0] == a.scalar.semanticKey);
    CHECK(service.Stats().declinedAdmission == 0);

    const std::vector<RigExecOutputPacket> packets = service.PollResults(sub);
    const RigExecOutputPacket *scalar = nullptr;
    const RigExecOutputPacket *points = nullptr;
    for (const RigExecOutputPacket &packet : packets) {
        if (packet.handle.semanticKey == a.scalar.semanticKey) {
            scalar = &packet;
        } else if (packet.handle.semanticKey == a.points.semanticKey) {
            points = &packet;
        }
    }
    CHECK(scalar != nullptr);
    CHECK(scalar && scalar->status == RigExecOutputStatus::Ready);
    CHECK(points != nullptr);
    CHECK(points && points->status == RigExecOutputStatus::Unsupported);
    CHECK(points && points->reason.find("GPU") != std::string::npos);
}

// The refused half of the admission window is deterministic: the pool
// reserved nothing, so the demand, its launched mask and the byte
// reservation all go back inside the call. The books are read here with no
// second pump, and the frame stays owed to the next one.
void
TestRefusedAdmissionRollsBackItsDemand()
{
    Harness h(/*workers=*/1);
    const RigFixture a = StageA();
    h.catalogs[a.root] = a.catalog;
    auto service = h.MakeService();
    const size_t jobCap = h.scheduler->GetExplicitPerRigJobCap();
    h.scheduler->SetExplicitPerRigJobCap(0);

    const RigExecSubscriptionHandle refused =
        service.Subscribe(a.root, RequestAt(TimeOf(12.0), {a.scalar}));
    CHECK(refused.IsValid());
    CHECK(h.scheduler->Stats().explicitAccepted == 0);
    CHECK(h.scheduler->Stats().explicitDeclined == 1);
    CHECK(h.scheduler->ExplicitInFlightCount(a.root) == 0);
    CHECK(h.scheduler->QueuedExplicitKeys(a.root).empty());
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
    CHECK(service.Stats().declinedAdmission == 1);
    CHECK(service.Stats().dispatches == 0);
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(service.Stats().retiredInFlight == 0);
    const std::vector<RigExecOutputPacket> waiting =
        service.PollResults(refused);
    CHECK(waiting.size() == 1);
    CHECK(AllPending(waiting));
    CHECK(waiting.size() == 1 &&
          waiting[0].reason.find("declined this admission") !=
              std::string::npos);

    // With room at the pool the same pump admits the frame and answers it:
    // the rollback freed the launched mask instead of parking a demand.
    h.scheduler->SetExplicitPerRigJobCap(jobCap);
    service.PumpCallbacks();
    CHECK(h.scheduler->Stats().explicitAccepted == 1);
    h.scheduler->WaitUntilIdle();
    CHECK(AllReady(service.PollResults(refused)));
    CHECK(service.Stats().demandsInFlight == 0);
    CHECK(h.scheduler->ReservedExplicitBytes() == 0);
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
    TestExpandedOutputStaysPendingUntilItsRun();
    TestUpdateAtomicity();
    TestCancellationKeepsSharedDemand();
    TestDestroyAllReleasesEverything();
    TestScopedRetirementOfAQueuedJobRedispatches();
    TestGenerationBumpWhileRunningPublishesNothing();
    TestScopedRetirementWhileRunningInvalidates();
    TestExactTimesAcrossRigsAndStages();
    TestExplicitDemandRunsWhileWarmingIsOff();
    TestCoverageHitPullsNothing();
    TestPolledHandleCarriesTheRecordRevision();
    TestEditAtTheSameFrameRedemandsTheAnswer();
    TestExistingSubscriptionRefusesARecompiledRig();
    TestRefusalOfReplacementWorkEndsTheRetiredAnswer();
    TestServiceDestructionReleasesEveryLivePin();
    TestLastClientLeavingAQueuedJobRetiresIt();
    TestClosingOneClientCancelsOnlyItsOwnJob();
    TestWorkEndingInsideAdmissionKeepsItsDisposition();
    TestRefusedAdmissionRollsBackItsDemand();
    TestUnsupportedHandleKeepsItsSiblingsRunnable();
    TestProgramRebuildAtTheStandingEpochRefusesTheOldCatalog();
    TestFreshSubscriptionRecoversAfterAStandingEpochRebuild();
    TestRebuildCallbacksStayWithTheirCompilation();
    TestRunningForeignJobCannotOccupyTheFreshFrame();
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
