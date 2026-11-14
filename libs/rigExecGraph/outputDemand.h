// Output demand contract: the types that name an evaluated rig output and the
// exact frame a host wants from it, plus the reverse demand index that turns a
// set of requested outputs into the canonical operations one dispatch must run
// (docs/specs/output-subscriptions.md).
//
// This is the only place these types are declared: scene lowering, cache
// publication, bake, runtime, imaging and host adapters include this header and
// may not declare their own time, handle, status, packet, semantic key,
// payload layout, payload lease or closure type.
//
// USD-free by contract, like the rest of rigExecGraph: no stage, path or
// schema type appears here. Compiled rig identity travels as plain text plus a
// compile epoch, a time is a USD time code spelled as a double, and a payload
// is an immutable byte lease whose layout and element count are declared here
// and by the producer.
#ifndef RIGEXEC_GRAPH_OUTPUT_DEMAND_H
#define RIGEXEC_GRAPH_OUTPUT_DEMAND_H

#include "opGraph.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

// A canonical operation index that names no operation.
constexpr uint32_t kRigExecNoOp = 0xffffffffu;

// A graph value id that names no value: an output address the compiled graph
// retains no canonical producer for.
constexpr RigExecValueId kRigExecNoOutputValue =
    std::numeric_limits<RigExecValueId>::max();

// Namespace-scope so an absent demand list needs no guard at call time.
const std::vector<uint32_t> kRigExecNoOutputs;

// One requested frame: the context's default frame, or one exact finite USD
// time code carried as a double -- UsdTimeCode::GetValue(), a count of frames
// on the stage's time code scale, never seconds. A host that holds seconds
// converts with the stage's rate before building a time; nothing here applies
// or records a rate. The default is an identity, not a number: it never equals
// numeric zero, and negative or subframe codes survive unchanged. Negative
// zero is stored as zero so equal times compare and hash the same.
struct RigExecOutputTime {
    RigExecOutputTime() = default;

    static RigExecOutputTime Default()
    {
        return RigExecOutputTime();
    }

    // false for NaN or infinity; *out is then unchanged.
    static bool TryTimeCode(double timeCode, RigExecOutputTime *out)
    {
        if (!out || !std::isfinite(timeCode))
            return false;
        out->timeCode_ = (timeCode == 0.0) ? 0.0 : timeCode;
        out->hasTime_ = true;
        return true;
    }

    bool IsDefault() const
    {
        return !hasTime_;
    }
    // The USD time code. Meaningless for the default frame.
    double TimeCode() const
    {
        return timeCode_;
    }

    // The default frame sorts before every numeric frame, so grouping by time
    // keeps it apart from frame zero.
    friend bool operator==(const RigExecOutputTime &a,
                           const RigExecOutputTime &b)
    {
        if (a.hasTime_ != b.hasTime_)
            return false;
        if (!a.hasTime_)
            return true;
        return a.timeCode_ == b.timeCode_;
    }
    friend bool operator!=(const RigExecOutputTime &a,
                           const RigExecOutputTime &b)
    {
        return !(a == b);
    }
    friend bool operator<(const RigExecOutputTime &a,
                          const RigExecOutputTime &b)
    {
        if (a.hasTime_ != b.hasTime_)
            return !a.hasTime_;
        return a.timeCode_ < b.timeCode_;
    }

    std::size_t Hash() const
    {
        // A nonzero constant for the default frame, so its hash differs from
        // the hash of numeric zero.
        if (!hasTime_)
            return 0x9e3779b97f4a7c15ULL;
        uint64_t bits;
        std::memcpy(&bits, &timeCode_, sizeof(bits));
        bits ^= bits >> 33;
        bits *= 0xff51afd7ed558ccdULL;
        bits ^= bits >> 29;
        bits *= 0xc4ce93c9ac9793c7ULL;
        return std::size_t(bits);
    }

private:
    bool hasTime_ = false;
    double timeCode_ = 0.0;
};

struct RigExecOutputTimeHash {
    std::size_t operator()(const RigExecOutputTime &time) const
    {
        return time.Hash();
    }
};

// How far a requested output got. Every packet carries one of these, so an
// owner never infers a status from an empty payload.
enum class RigExecOutputStatus {
    Pending,     // requested, not produced yet
    Ready,       // the payload matches the handle at the requested time
    Invalidated, // produced, but a captured input or generation moved
    Unsupported, // the rig cannot name or produce this output
    Failed,      // the owner's dispatch or owner callback failed
};

inline const char *
RigExecOutputStatusText(RigExecOutputStatus status)
{
    switch (status) {
    case RigExecOutputStatus::Pending:
        return "pending";
    case RigExecOutputStatus::Ready:
        return "ready";
    case RigExecOutputStatus::Invalidated:
        return "invalidated";
    case RigExecOutputStatus::Unsupported:
        return "unsupported";
    case RigExecOutputStatus::Failed:
        return "failed";
    }
    return "unknown";
}

// One evaluated output of one compiled rig. The semantic key is the canonical
// key the export catalog assigns; the type and read phase travel with it so a
// consumer can validate a payload without re-deriving either from a stage.
struct RigExecOutputHandle {
    std::string rigIdentity;    // compiled rig instance, as spelled text
    uint64_t compileEpoch = 0;  // raised by every recompilation of that rig
    std::string semanticKey;    // canonical semantic key of the output
    std::string type;           // exact payload type spelling
    std::string readPhase;      // base / afterMover / final
    uint64_t valueRevision = 0; // value revision the handle was read against

    // The addressed output, ignoring the revision: the same output at a new
    // revision is still the same output.
    bool SameTarget(const RigExecOutputHandle &other) const
    {
        return rigIdentity == other.rigIdentity &&
               compileEpoch == other.compileEpoch &&
               semanticKey == other.semanticKey && type == other.type &&
               readPhase == other.readPhase;
    }

    friend bool operator==(const RigExecOutputHandle &a,
                           const RigExecOutputHandle &b)
    {
        return a.SameTarget(b) && a.valueRevision == b.valueRevision;
    }
    friend bool operator!=(const RigExecOutputHandle &a,
                           const RigExecOutputHandle &b)
    {
        return !(a == b);
    }

    // Identity hash, revision excluded, so a handle can be looked up by
    // target alone.
    std::size_t Hash() const
    {
        std::size_t seed = std::hash<std::string>()(rigIdentity);
        const auto mix = [&seed](std::size_t value) {
            seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        };
        mix(std::hash<std::string>()(semanticKey));
        mix(std::hash<std::string>()(type));
        mix(std::hash<std::string>()(readPhase));
        mix(compileEpoch);
        return seed;
    }
};

struct RigExecOutputHandleHash {
    std::size_t operator()(const RigExecOutputHandle &handle) const
    {
        return handle.Hash();
    }
};

// The durable semantic key of a target/public-computation/phase address.
// Dense storage and value ids never enter it, and each field is
// length-prefixed, so no field text can imitate the next field's length.
// Discovery, a .rigexec's export catalog, and any consumer rebuilding an
// address from compiled tables share this one spelling.
inline std::string
RigExecOutputSemanticKey(const std::string &target,
                         const std::string &publicComputation,
                         const std::string &phase)
{
    return std::to_string(target.size()) + ":" + target +
           std::to_string(publicComputation.size()) + ":" + publicComputation +
           std::to_string(phase.size()) + ":" + phase;
}

// Explicit: a host asked for this output. Speculative: the owner warmed it on
// its own and a host may take it.
enum class RigExecDemandPolicy {
    Explicit,
    Speculative,
};

// One host's request: the outputs it wants, each at the exact times it wants
// them at. A request is a set of per-time demands: outputs union inside one
// demand, and a demand for one time never carries an output asked for only at
// another time. A request is well formed when every demand names at least one
// output, every handle names an output, and every time is the default frame or
// a finite time code.
struct RigExecOutputTimeDemand {
    RigExecOutputTime time;
    std::vector<RigExecOutputHandle> handles;
};

struct RigExecOutputRequest {
    std::vector<RigExecOutputTimeDemand> demands;
    RigExecDemandPolicy policy = RigExecDemandPolicy::Explicit;

    bool IsWellFormed(std::string *error) const
    {
        const auto fail = [&](const std::string &message) {
            if (error)
                *error = message;
            return false;
        };
        if (demands.empty())
            return fail("output request has no demands");
        for (const RigExecOutputTimeDemand &demand : demands) {
            if (demand.handles.empty())
                return fail("output demand has no handles");
            for (const RigExecOutputHandle &handle : demand.handles) {
                if (handle.rigIdentity.empty())
                    return fail("output handle has no rig identity");
                if (handle.semanticKey.empty())
                    return fail("output handle has no semantic key");
                if (handle.type.empty())
                    return fail("output handle has no payload type");
            }
        }
        return true;
    }
};

// The index of the merged demand for this exact time, or -1 when the request
// has none yet.
inline int
RigExecFindDemand(const RigExecOutputRequest &request,
                  const RigExecOutputTime &time)
{
    for (size_t i = 0; i < request.demands.size(); ++i)
        if (request.demands[i].time == time)
            return int(i);
    return -1;
}

// The position of a handle naming the same output, or -1 when the list has
// none. Matching ignores the value revision, as RigExecOutputHandle::SameTarget
// does.
inline int
RigExecFindTarget(const std::vector<RigExecOutputHandle> &handles,
                  const RigExecOutputHandle &handle)
{
    for (size_t i = 0; i < handles.size(); ++i)
        if (handles[i].SameTarget(handle))
            return int(i);
    return -1;
}

// True when the request asks for this output at this exact time. Revision is
// part of the comparison: the same output at another revision is a different
// answer.
inline bool
RigExecRequestWants(const RigExecOutputRequest &request,
                    const RigExecOutputHandle &handle,
                    const RigExecOutputTime &time)
{
    const int demand = RigExecFindDemand(request, time);
    if (demand < 0)
        return false;
    const std::vector<RigExecOutputHandle> &handles =
        request.demands[size_t(demand)].handles;
    const int at = RigExecFindTarget(handles, handle);
    return at >= 0 && handles[size_t(at)] == handle;
}

// The one rig identity and compile epoch a request names, checked across
// every handle of every demand. False with a diagnostic for an empty request,
// a handle with no identity, or a request that mixes rigs or epochs: those
// are two groups, and no dispatch answers both.
inline bool
RigExecRequestGroup(const RigExecOutputRequest &request,
                    std::string *rigIdentity, uint64_t *compileEpoch,
                    std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    bool seen = false;
    std::string group;
    uint64_t epoch = 0;
    for (const RigExecOutputTimeDemand &demand : request.demands) {
        if (demand.handles.empty())
            return fail("output demand has no handles");
        for (const RigExecOutputHandle &handle : demand.handles) {
            if (handle.rigIdentity.empty())
                return fail("output handle has no rig identity");
            if (!seen) {
                group = handle.rigIdentity;
                epoch = handle.compileEpoch;
                seen = true;
                continue;
            }
            if (handle.rigIdentity != group)
                return fail("request mixes rig identities " + group + " and " +
                            handle.rigIdentity);
            if (handle.compileEpoch != epoch)
                return fail("request mixes compile epochs");
        }
    }
    if (!seen)
        return fail("request has no handles");
    if (rigIdentity)
        *rigIdentity = group;
    if (compileEpoch)
        *compileEpoch = epoch;
    return true;
}

// True when two requests can be answered by one dispatch. Every handle of
// every demand is checked, never one representative handle: both requests
// must name one rig identity and one compile epoch, and an output demanded at
// the same exact time by both must be demanded at one value revision.
// Requests that disagree are two groups; merging them would answer one handle
// from one revision and another handle from a second.
inline bool
RigExecRequestsCompatible(const RigExecOutputRequest &a,
                          const RigExecOutputRequest &b,
                          std::string *error = nullptr)
{
    std::string aRig, bRig;
    uint64_t aEpoch = 0, bEpoch = 0;
    if (!RigExecRequestGroup(a, &aRig, &aEpoch, error))
        return false;
    if (!RigExecRequestGroup(b, &bRig, &bEpoch, error))
        return false;
    if (aRig != bRig) {
        if (error)
            *error = "requests name different rigs " + aRig + " and " + bRig;
        return false;
    }
    if (aEpoch != bEpoch) {
        if (error)
            *error = "requests name different compile epochs";
        return false;
    }
    for (const RigExecOutputTimeDemand &demand : a.demands) {
        const int at = RigExecFindDemand(b, demand.time);
        if (at < 0)
            continue;
        const std::vector<RigExecOutputHandle> &other =
            b.demands[size_t(at)].handles;
        for (const RigExecOutputHandle &handle : demand.handles) {
            const int found = RigExecFindTarget(other, handle);
            if (found >= 0 &&
                other[size_t(found)].valueRevision != handle.valueRevision) {
                if (error)
                    *error = "output " + handle.semanticKey +
                             " is demanded at mismatched value revisions";
                return false;
            }
        }
    }
    return true;
}

// The payload layouts the contract defines. A produced value is one of these
// or it is not an exportable output; anything else is scratch.
//
// The wire form is host-order IEEE-754 doubles with no padding, and this
// header is its only declaration: extraction, cache publication and the
// runtime read these layouts and never re-declare them. Element sizes:
//   Scalar        1 double   (8 bytes)
//   Matrix4x4Row 16 doubles  (128 bytes), row-major, USD row-vector form, so a
//                point transforms as p' = p * M.
//   Point3        3 doubles  (24 bytes), x y z in the value's own space
enum class RigExecPayloadKind {
    Unknown,      // no layout declared: never a readable payload
    Scalar,       // one double
    Matrix4x4Row, // 16 doubles, row-major
    Point3,       // 3 doubles, x y z
};

// The exact handle-type spellings for these layouts. A handle's `type` names
// the payload layout, not the USD storage type of the attribute the output
// came from: scene lowering maps a USD value type to one of these spellings
// (converting to double where the attribute stores floats) and a consumer
// never invents another spelling.
namespace RigExecPayloadText {
inline constexpr const char *const scalar = "double";
inline constexpr const char *const matrix4x4Row = "matrix4d";
inline constexpr const char *const point3 = "point3d[]";
} // namespace RigExecPayloadText

// Bytes per element of a layout, 0 for an unknown kind.
inline std::size_t
RigExecPayloadElementBytes(RigExecPayloadKind kind)
{
    switch (kind) {
    case RigExecPayloadKind::Scalar:
        return sizeof(double);
    case RigExecPayloadKind::Matrix4x4Row:
        return 16 * sizeof(double);
    case RigExecPayloadKind::Point3:
        return 3 * sizeof(double);
    case RigExecPayloadKind::Unknown:
        break;
    }
    return 0;
}

// The layout a handle-type spelling names. false for a spelling this header
// does not define.
inline bool
RigExecPayloadKindForType(const std::string &type, RigExecPayloadKind *kind)
{
    if (!kind)
        return false;
    if (type == RigExecPayloadText::scalar)
        *kind = RigExecPayloadKind::Scalar;
    else if (type == RigExecPayloadText::matrix4x4Row)
        *kind = RigExecPayloadKind::Matrix4x4Row;
    else if (type == RigExecPayloadText::point3)
        *kind = RigExecPayloadKind::Point3;
    else
        return false;
    return true;
}

// The spelling a layout travels as, "unknown" for an undeclared kind.
inline const char *
RigExecPayloadTypeSpelling(RigExecPayloadKind kind)
{
    switch (kind) {
    case RigExecPayloadKind::Scalar:
        return RigExecPayloadText::scalar;
    case RigExecPayloadKind::Matrix4x4Row:
        return RigExecPayloadText::matrix4x4Row;
    case RigExecPayloadKind::Point3:
        return RigExecPayloadText::point3;
    case RigExecPayloadKind::Unknown:
        break;
    }
    return "unknown";
}

// Byte count for a layout and element count, false when the count would
// overflow the byte count. Reservation happens here, before any allocation.
inline bool
RigExecPayloadByteCount(RigExecPayloadKind kind, uint64_t elementCount,
                        uint64_t *bytes)
{
    const std::size_t elementBytes = RigExecPayloadElementBytes(kind);
    if (!bytes || elementBytes == 0)
        return false;
    if (elementCount >
        uint64_t(std::numeric_limits<std::size_t>::max()) / elementBytes)
        return false;
    *bytes = elementCount * uint64_t(elementBytes);
    return true;
}

// Immutable lease on one produced value. The producer declares the layout and
// the element count; consumers never re-derive either from the buffer size,
// and the buffer carries no alignment guarantee, so elements are read one
// memcpy at a time (RigExecPayloadView). Copies share the bytes; a writer
// detaches before changing them.
struct RigExecOutputPayload {
    std::shared_ptr<const std::vector<uint8_t>> bytes;
    RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
    uint64_t elementCount = 0;

    bool IsEmpty() const
    {
        return !bytes || bytes->empty();
    }
    std::size_t ByteCount() const
    {
        return bytes ? bytes->size() : 0;
    }
};

// True when the lease matches its declared layout: a declared kind, bytes
// exactly elementCount * elementBytes (never short, never trailing extra
// bytes), and a buffer for a non-empty payload. false with a diagnostic.
inline bool
RigExecValidatePayload(const RigExecOutputPayload &payload, std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    const std::size_t elementBytes = RigExecPayloadElementBytes(payload.kind);
    if (elementBytes == 0)
        return fail(std::string("payload ") +
                    RigExecPayloadTypeSpelling(payload.kind) +
                    " has no declared layout");
    uint64_t expected = 0;
    if (!RigExecPayloadByteCount(payload.kind, payload.elementCount, &expected))
        return fail("payload element count overflows its byte count");
    if (expected == 0) {
        if (payload.ByteCount() != 0)
            return fail("payload declares no elements but carries bytes");
        return true;
    }
    if (!payload.bytes)
        return fail("payload has no bytes");
    if (uint64_t(payload.ByteCount()) != expected)
        return fail(std::string("payload ") +
                    RigExecPayloadTypeSpelling(payload.kind) + " has " +
                    std::to_string(payload.ByteCount()) + " bytes for " +
                    std::to_string(payload.elementCount) + " elements");
    return true;
}

// True when the payload is readable for the handle that names it: the handle's
// type spelling declares the payload's layout. A payload for an output whose
// type has no defined layout is never answered with a guessed one.
inline bool
RigExecPayloadMatchesHandle(const RigExecOutputHandle &handle,
                            const RigExecOutputPayload &payload,
                            std::string *error)
{
    RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
    if (!RigExecPayloadKindForType(handle.type, &kind)) {
        if (error)
            *error = "output " + handle.semanticKey + " has type " +
                     handle.type + " with no defined payload layout";
        return false;
    }
    if (payload.kind != kind) {
        if (error)
            *error = "output " + handle.semanticKey + " is declared " +
                     handle.type + " but its payload is " +
                     RigExecPayloadTypeSpelling(payload.kind);
        return false;
    }
    return RigExecValidatePayload(payload, error);
}

// Copy a producer's contiguous doubles into an immutable lease. src must hold
// elementCount * (element bytes / sizeof(double)) doubles. The byte count is
// computed and checked before the buffer is allocated. This runs at the
// extraction boundary after the join, never inside an operation body or a
// kernel: it allocates and publishes a shared owner.
inline bool
RigExecPackPayload(RigExecPayloadKind kind, uint64_t elementCount,
                   const double *src, RigExecOutputPayload *out,
                   std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    if (!out)
        return fail("no payload to pack into");
    uint64_t bytes = 0;
    if (!RigExecPayloadByteCount(kind, elementCount, &bytes))
        return fail(std::string("cannot pack ") +
                    RigExecPayloadTypeSpelling(kind) + " payload");
    if (bytes == 0) {
        out->bytes = nullptr;
        out->kind = kind;
        out->elementCount = 0;
        return true;
    }
    if (!src)
        return fail("payload has no source values");
    const size_t byteCount = size_t(bytes);
    std::vector<uint8_t> buffer(byteCount);
    std::memcpy(buffer.data(), src, byteCount);
    out->bytes =
        std::make_shared<const std::vector<uint8_t>>(std::move(buffer));
    out->kind = kind;
    out->elementCount = elementCount;
    return RigExecValidatePayload(*out, error);
}

// Element access for one lease. Every read copies sizeof(double) bytes with
// memcpy from the declared offset: the byte buffer has no alignment guarantee,
// so nothing here reinterprets it as a double array. Reads are bounds checked
// against the declared element count, not against the buffer size.
class RigExecPayloadView {
public:
    bool Open(const RigExecOutputPayload &payload, std::string *error)
    {
        if (!RigExecValidatePayload(payload, error))
            return false;
        bytes_ = payload.bytes;
        kind_ = payload.kind;
        count_ = payload.elementCount;
        return true;
    }

    // Opens for the handle that names the payload, checking the declared type
    // against the layout first.
    bool Open(const RigExecOutputHandle &handle,
              const RigExecOutputPayload &payload, std::string *error)
    {
        if (!RigExecPayloadMatchesHandle(handle, payload, error))
            return false;
        return Open(payload, error);
    }

    RigExecPayloadKind Kind() const
    {
        return kind_;
    }

    uint64_t ElementCount() const
    {
        return count_;
    }

    bool ReadScalar(size_t element, double *out) const
    {
        return kind_ == RigExecPayloadKind::Scalar && Read(element, 1, out);
    }
    bool ReadMatrix4x4Row(size_t element, double *out) const
    {
        return kind_ == RigExecPayloadKind::Matrix4x4Row &&
               Read(element, 16, out);
    }
    bool ReadPoint3(size_t element, double *out) const
    {
        return kind_ == RigExecPayloadKind::Point3 && Read(element, 3, out);
    }

private:
    bool Read(size_t element, size_t doublesPerElement, double *out) const
    {
        if (!out || !bytes_ || element >= count_)
            return false;
        const size_t stride = doublesPerElement * sizeof(double);
        const size_t offset = element * stride;
        if (offset / stride != element || offset >= bytes_->size())
            return false;
        std::memcpy(out, bytes_->data() + offset, stride);
        return true;
    }

    std::shared_ptr<const std::vector<uint8_t>> bytes_;
    RigExecPayloadKind kind_ = RigExecPayloadKind::Unknown;
    uint64_t count_ = 0;
};

// The full captured generation a value was produced from. Every field is part
// of the key: the program that produced the value, the inputs it sampled, the
// preview generation, and the history generation. A partial key (the program
// alone, or the history revision alone) never identifies a generation, and the
// service compares all four before a packet answers a request.
struct RigExecOutputGeneration {
    uint64_t programIdentity = 0;
    uint64_t inputRevision = 0;
    uint64_t previewRevision = 0;
    uint64_t historyRevision = 0;

    friend bool operator==(const RigExecOutputGeneration &a,
                           const RigExecOutputGeneration &b)
    {
        return a.programIdentity == b.programIdentity &&
               a.inputRevision == b.inputRevision &&
               a.previewRevision == b.previewRevision &&
               a.historyRevision == b.historyRevision;
    }
    friend bool operator!=(const RigExecOutputGeneration &a,
                           const RigExecOutputGeneration &b)
    {
        return !(a == b);
    }

    std::size_t Hash() const
    {
        std::size_t seed = std::size_t(programIdentity);
        const auto mix = [&seed](std::size_t value) {
            seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        };
        mix(std::size_t(inputRevision));
        mix(std::size_t(previewRevision));
        mix(std::size_t(historyRevision));
        return seed;
    }
};

// One answered output: what was requested, which program produced it, the
// generation it was captured from, and its status with a reason.
struct RigExecOutputPacket {
    RigExecOutputHandle handle;
    RigExecOutputTime time;
    uint64_t programIdentity = 0; // compiled program that produced the value
    uint64_t inputRevision = 0;   // captured scene inputs behind the value
    uint64_t previewRevision = 0; // preview generation, 0 when there is none
    uint64_t historyRevision = 0; // history generation the value belongs to
    RigExecOutputStatus status = RigExecOutputStatus::Pending;
    std::string reason;
    RigExecOutputPayload payload;

    // The packet's full captured generation key.
    RigExecOutputGeneration Generation() const
    {
        RigExecOutputGeneration generation;
        generation.programIdentity = programIdentity;
        generation.inputRevision = inputRevision;
        generation.previewRevision = previewRevision;
        generation.historyRevision = historyRevision;
        return generation;
    }

    bool IsUsable() const
    {
        return status == RigExecOutputStatus::Ready;
    }
    // A Ready packet answers only when its payload is readable and every part
    // of the generation key it was captured from matches.
    bool Answers(const RigExecOutputHandle &requested,
                 const RigExecOutputTime &at,
                 const RigExecOutputGeneration &captured) const
    {
        if (status != RigExecOutputStatus::Ready)
            return false;
        if (!(handle == requested) || !(time == at))
            return false;
        if (Generation() != captured)
            return false;
        std::string error;
        return RigExecPayloadMatchesHandle(handle, payload, &error);
    }
};

// Requirement edges a backend declares between canonical operations, indexed
// by canonical operation index. Validity: the consumer's read is meaningful
// only after that operation ran. Effect: a skipped operation changes state the
// consumer reads (RigExecOpCallbacks::skipEffects). History: a windowed
// operation needs the operations of its earlier ticks (windowHistory.h).
struct RigExecDemandEdges {
    std::vector<std::vector<uint32_t>> validity;
    std::vector<std::vector<uint32_t>> effects;
    std::vector<std::vector<uint32_t>> history;
};

// Why an operation is in a closure. An operation's kind is the first edge that
// reaches it, so an operation pulled in by both a predecessor and a history
// edge is reported as a predecessor.
enum class RigExecRequirementKind {
    Predecessor,
    Validity,
    Effect,
    History,
};

// The canonical operations one requested output needs: ascending, and closed
// under graph predecessors and the declared validity, effect and history
// edges. An empty closure means the output has no canonical producer.
struct RigExecDemandClosure {
    std::vector<uint32_t> ops;
    std::vector<RigExecRequirementKind> kinds;

    bool Empty() const
    {
        return ops.empty();
    }
    std::size_t Size() const
    {
        return ops.size();
    }
    bool Contains(uint32_t op) const
    {
        return std::binary_search(ops.begin(), ops.end(), op);
    }
    RigExecRequirementKind KindOf(uint32_t op) const
    {
        const auto at = std::lower_bound(ops.begin(), ops.end(), op);
        if (at == ops.end() || *at != op)
            return RigExecRequirementKind::Predecessor;
        return kinds[size_t(at - ops.begin())];
    }
};

// One output the index can resolve, with the graph value its handle names.
struct RigExecOutputDemandEntry {
    RigExecOutputHandle handle;
    RigExecValueId producerValue;
};

// Merge requests of one compiled rig into one request. Handles merge only
// inside the demand for the same exact time: an output asked for at frame 12
// never picks up a frame it was not asked for, so a union can never name a
// (handle, time) pair no host requested. A target demanded at one time by two
// requests at different value revisions is refused rather than resolved to
// one of them: the merged request comes back empty with a diagnostic, and the
// caller dispatches the groups separately.
inline RigExecOutputRequest
RigExecUnionRequests(const std::vector<RigExecOutputRequest> &requests,
                     std::string *error = nullptr)
{
    RigExecOutputRequest merged;
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        merged = RigExecOutputRequest{};
        return merged;
    };
    bool explicitRequested = false;
    std::string groupRig;
    uint64_t groupEpoch = 0;
    bool grouped = false;
    for (const RigExecOutputRequest &request : requests) {
        std::string rig;
        uint64_t epoch = 0;
        if (!RigExecRequestGroup(request, &rig, &epoch, error)) {
            merged = RigExecOutputRequest{};
            return merged;
        }
        if (!grouped) {
            groupRig = rig;
            groupEpoch = epoch;
            grouped = true;
        } else if (rig != groupRig) {
            return fail("requests name different rigs " + groupRig + " and " +
                        rig);
        } else if (epoch != groupEpoch) {
            return fail("requests name different compile epochs");
        }
        explicitRequested |= request.policy == RigExecDemandPolicy::Explicit;
        for (const RigExecOutputTimeDemand &demand : request.demands) {
            int at = RigExecFindDemand(merged, demand.time);
            if (at < 0) {
                merged.demands.push_back(demand);
                continue;
            }
            std::vector<RigExecOutputHandle> &existing =
                merged.demands[size_t(at)].handles;
            for (const RigExecOutputHandle &handle : demand.handles) {
                const int found = RigExecFindTarget(existing, handle);
                if (found < 0) {
                    existing.push_back(handle);
                    continue;
                }
                if (existing[size_t(found)].valueRevision !=
                    handle.valueRevision)
                    return fail("output " + handle.semanticKey +
                                " is demanded at mismatched value revisions");
            }
        }
    }
    merged.policy = explicitRequested ? RigExecDemandPolicy::Explicit
                                      : RigExecDemandPolicy::Speculative;
    return merged;
}

// A closure is usable for execution only when it is predecessor closed: every
// graph predecessor of an operation in it is in it too. Validity and history
// edges are inside a built closure by construction; Expand walks them.
inline bool
RigExecClosureIsClosed(const RigExecCompiledGraph &graph,
                       const RigExecDemandClosure &closure,
                       std::string *error = nullptr)
{
    for (uint32_t op : closure.ops) {
        if (op >= graph.ops.size()) {
            if (error)
                *error = "closure names an operation outside the graph";
            return false;
        }
        for (uint32_t predecessor : graph.ops[op].predecessors) {
            if (!closure.Contains(predecessor)) {
                if (error)
                    *error = "closure of operation " + std::to_string(op) +
                             " is not predecessor closed";
                return false;
            }
        }
    }
    return true;
}

// The demand index for one compiled rig version: which canonical operation
// produces each requested output, the operations each needs, and which
// outputs an operation is demanded by. Built once per compiled rig version,
// before any dispatch.
class RigExecOutputDemandIndex {
public:
    bool Build(const RigExecCompiledGraph &graph,
               const RigExecDemandEdges &edges,
               const std::vector<RigExecOutputDemandEntry> &entries,
               std::string *error)
    {
        const auto fail = [&](const std::string &message) {
            if (error)
                *error = message;
            return false;
        };
        const size_t count = graph.ops.size();
        if (count > kRigExecNoOp)
            return fail("too many operations for a demand index");
        const auto checkEdges =
            [&](const std::vector<std::vector<uint32_t>> &list,
                const char *name) {
                if (list.size() != count)
                    return fail(std::string(name) +
                                " edges do not match the operation count");
                for (const std::vector<uint32_t> &opEdges : list)
                    for (uint32_t target : opEdges)
                        if (target >= count)
                            return fail(std::string(name) +
                                        " edge names an absent operation");
                return true;
            };
        if (!checkEdges(edges.validity, "validity") ||
            !checkEdges(edges.effects, "effect") ||
            !checkEdges(edges.history, "history"))
            return false;

        // Canonical producer per value, mirroring the uniqueness the graph
        // compiler already established.
        std::unordered_map<RigExecValueId, uint32_t> producer;
        producer.reserve(count * 2);
        for (uint32_t op = 0; op < count; ++op)
            for (RigExecValueId value : graph.ops[op].descriptor.writes)
                if (!producer.emplace(value, op).second)
                    return fail("value has more than one canonical producer");

        handles_.assign(entries.size(), RigExecOutputHandle{});
        producers_.assign(entries.size(), kRigExecNoOp);
        closures_.assign(entries.size(), RigExecDemandClosure{});
        demanders_.assign(count, std::vector<uint32_t>{});
        slots_.clear();
        for (size_t slot = 0; slot < entries.size(); ++slot) {
            slots_[entries[slot].handle.Hash()].push_back(uint32_t(slot));
        }

        for (size_t slot = 0; slot < entries.size(); ++slot) {
            const RigExecOutputHandle &handle = entries[slot].handle;
            if (handle.semanticKey.empty())
                return fail("demand entry " + std::to_string(slot) +
                            " has no semantic key");
            handles_[slot] = handle;
            const auto found = producer.find(entries[slot].producerValue);
            if (found == producer.end()) {
                // No canonical producer: the index reports the output
                // unsupported instead of guessing a stage-side fallback.
                continue;
            }
            producers_[slot] = found->second;
            Expand(graph, edges, found->second, &closures_[slot]);
            for (uint32_t op : closures_[slot].ops)
                demanders_[op].push_back(uint32_t(slot));
        }
        return true;
    }

    std::size_t Size() const
    {
        return handles_.size();
    }
    bool Empty() const
    {
        return handles_.empty();
    }
    std::size_t OperationCount() const
    {
        return demanders_.size();
    }

    const RigExecOutputHandle &Handle(size_t output) const
    {
        return handles_[output];
    }
    // The canonical operation producing the output, or kRigExecNoOp when the
    // graph has no producer for it.
    uint32_t Producer(size_t output) const
    {
        return producers_[output];
    }
    RigExecOutputStatus ProducerStatus(size_t output) const
    {
        return producers_[output] == kRigExecNoOp
                   ? RigExecOutputStatus::Unsupported
                   : RigExecOutputStatus::Pending;
    }
    const RigExecDemandClosure &Closure(size_t output) const
    {
        return closures_[output];
    }

    bool Find(const RigExecOutputHandle &handle, size_t *output) const
    {
        const auto found = slots_.find(handle.Hash());
        if (found == slots_.end())
            return false;
        for (uint32_t slot : found->second) {
            if (handles_[slot].SameTarget(handle)) {
                if (output)
                    *output = size_t(slot);
                return true;
            }
        }
        return false;
    }

    // Outputs whose closure includes this canonical operation, ascending.
    const std::vector<uint32_t> &DemandedBy(uint32_t op) const
    {
        return op < demanders_.size() ? demanders_[op] : kRigExecNoOutputs;
    }

    // The index slots a request names, one per distinct handle, in request
    // order. Every handle of every demand is resolved: a handle the index does
    // not know is refused with a diagnostic instead of dropped, so a partial
    // demand never produces a partial mask.
    bool ResolveRequest(const RigExecOutputRequest &request,
                        std::vector<size_t> *slots, std::string *error) const
    {
        if (!slots)
            return false;
        slots->clear();
        for (const RigExecOutputTimeDemand &demand : request.demands) {
            for (const RigExecOutputHandle &handle : demand.handles) {
                size_t slot = 0;
                if (!Find(handle, &slot)) {
                    if (error)
                        *error = "request names an output the index does not "
                                 "know: " +
                                 handle.semanticKey;
                    slots->clear();
                    return false;
                }
                if (std::find(slots->begin(), slots->end(), slot) ==
                    slots->end())
                    slots->push_back(slot);
            }
        }
        return true;
    }

    // The operations one dispatch must run for a set of outputs: the union of
    // their closures, each operation named once.
    RigExecDemandClosure Union(const std::vector<size_t> &outputs) const
    {
        std::vector<char> seen(demanders_.size(), 0);
        std::vector<RigExecRequirementKind> kind(
            demanders_.size(), RigExecRequirementKind::Predecessor);
        for (size_t slot : outputs) {
            if (slot >= closures_.size())
                continue;
            for (size_t i = 0; i < closures_[slot].ops.size(); ++i) {
                const uint32_t op = closures_[slot].ops[i];
                if (seen[op])
                    continue;
                seen[op] = 1;
                kind[op] = closures_[slot].kinds[i];
            }
        }
        RigExecDemandClosure merged;
        for (size_t op = 0; op < demanders_.size(); ++op) {
            if (seen[op]) {
                merged.ops.push_back(uint32_t(op));
                merged.kinds.push_back(kind[op]);
            }
        }
        return merged;
    }

    // The union as an operation mask, ready for RigExecOpCallbacks::active.
    std::vector<char> ActiveMask(const std::vector<size_t> &outputs) const
    {
        std::vector<char> mask(demanders_.size(), 0);
        for (uint32_t op : Union(outputs).ops)
            mask[op] = 1;
        return mask;
    }

private:
    static void Expand(const RigExecCompiledGraph &graph,
                       const RigExecDemandEdges &edges, uint32_t seed,
                       RigExecDemandClosure *closure)
    {
        const size_t count = graph.ops.size();
        std::vector<char> seen(count, 0);
        std::vector<RigExecRequirementKind> kind(
            count, RigExecRequirementKind::Predecessor);
        std::vector<uint32_t> queue;
        queue.reserve(count);
        queue.push_back(seed);
        seen[seed] = 1;
        for (size_t head = 0; head < queue.size(); ++head) {
            const uint32_t op = queue[head];
            for (uint32_t next : graph.ops[op].predecessors)
                Push(next, RigExecRequirementKind::Predecessor, seen, kind,
                     &queue);
            for (uint32_t next : edges.validity[op])
                Push(next, RigExecRequirementKind::Validity, seen, kind,
                     &queue);
            for (uint32_t next : edges.effects[op])
                Push(next, RigExecRequirementKind::Effect, seen, kind, &queue);
            for (uint32_t next : edges.history[op])
                Push(next, RigExecRequirementKind::History, seen, kind, &queue);
        }
        closure->ops = queue;
        std::sort(closure->ops.begin(), closure->ops.end());
        closure->kinds.clear();
        closure->kinds.reserve(closure->ops.size());
        for (uint32_t op : closure->ops)
            closure->kinds.push_back(kind[op]);
    }

    static void Push(uint32_t op, RigExecRequirementKind requirement,
                     std::vector<char> &seen,
                     std::vector<RigExecRequirementKind> &kind,
                     std::vector<uint32_t> *queue)
    {
        if (seen[op])
            return;
        seen[op] = 1;
        kind[op] = requirement;
        queue->push_back(op);
    }

    std::vector<RigExecOutputHandle> handles_;
    std::vector<uint32_t> producers_;
    std::vector<RigExecDemandClosure> closures_;
    std::vector<std::vector<uint32_t>> demanders_;
    std::unordered_map<std::size_t, std::vector<uint32_t>> slots_;
};

} // namespace rigExec

#endif // RIGEXEC_GRAPH_OUTPUT_DEMAND_H
