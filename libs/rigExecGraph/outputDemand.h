// Output demand contract: the types that name an evaluated rig output and the
// exact frame a host wants from it, plus the reverse demand index that turns a
// set of requested outputs into the canonical operations one dispatch must run
// (docs/specs/output-subscriptions.md).
//
// This is the only place these types are declared: scene lowering, cache
// publication, bake, runtime, imaging and host adapters include this header and
// may not declare their own time, handle, status, packet, payload lease or
// closure type.
//
// USD-free by contract, like the rest of rigExecGraph: no stage, path or
// schema type appears here. Compiled rig identity travels as plain text plus a
// compile epoch, and a payload is an immutable byte lease whose element count
// is declared by the producer.
#ifndef RIGEXEC_GRAPH_OUTPUT_DEMAND_H
#define RIGEXEC_GRAPH_OUTPUT_DEMAND_H

#include "opGraph.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

// A canonical operation index that names no operation.
constexpr uint32_t kRigExecNoOp = 0xffffffffu;

// Namespace-scope so an absent demand list needs no guard at call time.
const std::vector<uint32_t> kRigExecNoOutputs;

// One requested frame: the context's default frame, or one exact finite time
// in seconds. The default is an identity, not a number: it never equals
// numeric zero, and negative or subframe times survive unchanged. Negative
// zero is stored as zero so equal times compare and hash the same.
struct RigExecOutputTime {
    RigExecOutputTime() = default;

    static RigExecOutputTime Default()
    {
        return RigExecOutputTime();
    }

    // false for NaN or infinity; *out is then unchanged.
    static bool TryNumeric(double seconds, RigExecOutputTime *out)
    {
        if (!out || !std::isfinite(seconds))
            return false;
        out->seconds_ = (seconds == 0.0) ? 0.0 : seconds;
        out->hasTime_ = true;
        return true;
    }

    bool IsDefault() const
    {
        return !hasTime_;
    }
    // The frame in seconds. Meaningless for the default frame.
    double Seconds() const
    {
        return seconds_;
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
        return a.seconds_ == b.seconds_;
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
        return a.seconds_ < b.seconds_;
    }

    std::size_t Hash() const
    {
        // A nonzero constant for the default frame, so its hash differs from
        // the hash of numeric zero.
        if (!hasTime_)
            return 0x9e3779b97f4a7c15ULL;
        uint64_t bits;
        std::memcpy(&bits, &seconds_, sizeof(bits));
        bits ^= bits >> 33;
        bits *= 0xff51afd7ed558ccdULL;
        bits ^= bits >> 29;
        bits *= 0xc4ce93c9ac9793c7ULL;
        return std::size_t(bits);
    }

private:
    bool hasTime_ = false;
    double seconds_ = 0.0;
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

// Explicit: a host asked for this output. Speculative: the owner warmed it on
// its own and a host may take it.
enum class RigExecDemandPolicy {
    Explicit,
    Speculative,
};

// One host's request: the outputs it wants and the exact frames it wants them
// at. A request is well formed when every handle names an output and every
// time is the default frame or a finite number.
struct RigExecOutputRequest {
    std::vector<RigExecOutputHandle> handles;
    std::vector<RigExecOutputTime> times;
    RigExecDemandPolicy policy = RigExecDemandPolicy::Explicit;

    bool IsWellFormed(std::string *error) const
    {
        const auto fail = [&](const std::string &message) {
            if (error)
                *error = message;
            return false;
        };
        if (handles.empty())
            return fail("output request has no handles");
        for (const RigExecOutputHandle &handle : handles) {
            if (handle.semanticKey.empty())
                return fail("output handle has no semantic key");
            if (handle.type.empty())
                return fail("output handle has no payload type");
        }
        for (const RigExecOutputTime &time : times) {
            if (!time.IsDefault() && !std::isfinite(time.Seconds()))
                return fail("output request time is not finite");
        }
        return true;
    }
};

// Immutable lease on one produced value. The producer declares the element
// count; consumers never re-derive it from the buffer size. Copies share the
// bytes; a writer detaches before changing them.
struct RigExecOutputPayload {
    std::shared_ptr<const std::vector<uint8_t>> bytes;
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

    bool IsUsable() const
    {
        return status == RigExecOutputStatus::Ready;
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

// True when two requests can be answered by one dispatch: same rig identity
// and compile epoch.
inline bool
RigExecRequestsCompatible(const RigExecOutputRequest &a,
                          const RigExecOutputRequest &b)
{
    if (a.handles.empty() || b.handles.empty())
        return false;
    return a.handles.front().rigIdentity == b.handles.front().rigIdentity &&
           a.handles.front().compileEpoch == b.handles.front().compileEpoch;
}

// Merge requests of one compiled rig into one request: duplicate handles and
// times appear once, and an explicit request outranks a speculative one.
inline RigExecOutputRequest
RigExecUnionRequests(const std::vector<RigExecOutputRequest> &requests)
{
    RigExecOutputRequest merged;
    bool explicitRequested = false;
    for (const RigExecOutputRequest &request : requests) {
        for (const RigExecOutputHandle &handle : request.handles) {
            const auto found =
                std::find_if(merged.handles.begin(), merged.handles.end(),
                             [&](const RigExecOutputHandle &existing) {
                                 return existing.SameTarget(handle);
                             });
            if (found == merged.handles.end())
                merged.handles.push_back(handle);
        }
        for (const RigExecOutputTime &time : request.times) {
            if (std::find(merged.times.begin(), merged.times.end(), time) ==
                merged.times.end())
                merged.times.push_back(time);
        }
        explicitRequested |= request.policy == RigExecDemandPolicy::Explicit;
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
