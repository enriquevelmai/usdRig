// The subscription lifecycle implementation: demand bookkeeping on the owner
// thread, one bounded job per exact (rig, epoch, frame, captured generation) on
// the existing pool, coverage publication, and pinned answers for polling
// clients.
//
// Lock discipline. The state mutex guards the client, group, demand and pin
// maps only. It is never held across a host hook, a frame-cache call, a pool
// call, or an evaluation: those run with it closed, and each map edit takes it
// briefly. A worker takes it twice per job -- once for liveness, once to record
// a terminal state -- and holds nothing else while it does. The pool's own
// order stays outermost: fence, pool, cache shard.
#include "outputSubscription.h"

#include <algorithm>
#include <map>
#include <set>
#include <utility>

namespace rigExec {
namespace {

// One frame of one rig at one compiled epoch: the unit demand unions into, and
// the unit one launched mask belongs to.
struct _GroupKey {
    SdfPath rig;
    RigExecOutputTime time;
    uint64_t compileEpoch = 0;

    bool operator<(const _GroupKey &other) const
    {
        if (rig != other.rig) {
            return rig < other.rig;
        }
        if (!(time == other.time)) {
            return time < other.time;
        }
        return compileEpoch < other.compileEpoch;
    }
};

bool
_HandleLess(const RigExecOutputHandle &a, const RigExecOutputHandle &b)
{
    if (a.rigIdentity != b.rigIdentity) {
        return a.rigIdentity < b.rigIdentity;
    }
    if (a.semanticKey != b.semanticKey) {
        return a.semanticKey < b.semanticKey;
    }
    if (a.readPhase != b.readPhase) {
        return a.readPhase < b.readPhase;
    }
    if (a.type != b.type) {
        return a.type < b.type;
    }
    if (a.compileEpoch != b.compileEpoch) {
        return a.compileEpoch < b.compileEpoch;
    }
    return a.valueRevision < b.valueRevision;
}

// One payload the service pins for its live clients. The frame cache holds one
// lease per (record, output), so a payload two clients share is pinned once.
struct _PinKey {
    RigExecCoverageKey coverage;
    RigExecOutputHandle handle;

    bool operator<(const _PinKey &other) const
    {
        if (!(coverage == other.coverage)) {
            return coverage < other.coverage;
        }
        return _HandleLess(handle, other.handle);
    }
};

void
_UnionHandle(std::vector<RigExecOutputHandle> *list,
             const RigExecOutputHandle &handle)
{
    if (RigExecFindTarget(*list, handle) < 0) {
        list->push_back(handle);
    }
}

// A record's answer for one output: Ready, for the record's own time and
// generation, with a readable payload. Targets are matched as outputDemand.h
// matches them, so a handle pinned to a value revision still resolves to the
// output the record answers; the revision is the caller's to compare.
const RigExecOutputPacket *
_RecordAnswer(const RigExecCoverageRecord &record,
              const RigExecOutputHandle &handle)
{
    for (const RigExecOutputPacket &packet : record.packets) {
        if (packet.status != RigExecOutputStatus::Ready ||
            !(packet.time == record.key.time) ||
            !(packet.Generation() == record.key.generation) ||
            !packet.handle.SameTarget(handle)) {
            continue;
        }
        std::string error;
        if (!RigExecPayloadMatchesHandle(packet.handle, packet.payload,
                                         &error)) {
            continue;
        }
        return &packet;
    }
    return nullptr;
}

// One element of the declared layout per demanded output: the smallest answer a
// run could publish. A host that can state the real element count sets the
// estimate on its capture instead.
size_t
_PayloadFloorBytes(const std::vector<RigExecOutputHandle> &handles)
{
    size_t bytes = 0;
    for (const RigExecOutputHandle &handle : handles) {
        RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
        if (!RigExecPayloadKindForType(handle.type, &kind)) {
            continue;
        }
        uint64_t elementBytes = 0;
        if (RigExecPayloadByteCount(kind, 1, &elementBytes)) {
            bytes += size_t(elementBytes);
        }
    }
    return bytes;
}

// How many generations one frame remembers for coverage probes. Older records
// stay in the cache until the cache retires them; they are simply not probed.
constexpr size_t kRigExecSubscriptionGenerationWindow = 8;

} // namespace

class RigExecSubscriptionState
    : public std::enable_shared_from_this<RigExecSubscriptionState> {
public:
    explicit RigExecSubscriptionState(RigExecSubscriptionHost host)
        : _host(std::move(host))
    {
        if (!_host.stepRunner) {
            _host.stepRunner = RigExecMakeProductionRequestedStepRunner();
        }
    }

    RigExecSubscriptionReceipt Subscribe(const SdfPath &rig,
                                         const RigExecOutputRequest &request)
    {
        RigExecSubscriptionReceipt receipt;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_stopping) {
                receipt.status = RigExecSubscriptionStatus::Stopping;
                receipt.reason = "the subscription service is closing";
                return receipt;
            }
        }
        std::string error;
        auto client = std::make_shared<Client>();
        if (!_Validate(request, rig, /*currentCatalog=*/true, client.get(),
                       &receipt, &error)) {
            return receipt;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_stopping) {
                receipt.status = RigExecSubscriptionStatus::Stopping;
                receipt.reason = "the subscription service is closing";
                return receipt;
            }
            client->id = ++_nextId;
            _clients.emplace(client->id, client);
        }
        receipt.subscription = client->id;
        _DispatchDue();
        return receipt;
    }

    RigExecSubscriptionReceipt Update(uint64_t id,
                                      const RigExecOutputRequest &request)
    {
        RigExecSubscriptionReceipt receipt;
        std::shared_ptr<Client> current;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_stopping) {
                receipt.status = RigExecSubscriptionStatus::Stopping;
                receipt.reason = "the subscription service is closing";
                return receipt;
            }
            const auto found = _clients.find(id);
            if (found == _clients.end()) {
                receipt.status = RigExecSubscriptionStatus::UnknownSubscription;
                receipt.reason = "no live subscription has that id";
                return receipt;
            }
            current = found->second;
        }
        // Validated against the catalog that minted this client's handles,
        // before the old request is dropped: a refused update changes nothing.
        std::shared_ptr<Client> staged = std::make_shared<Client>();
        *staged = *current;
        staged->request = request;
        staged->unsupported.clear();
        std::string error;
        if (!_Validate(request, current->rig, /*currentCatalog=*/false,
                       staged.get(), &receipt, &error)) {
            return receipt;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto found = _clients.find(id);
            if (found == _clients.end() || _stopping) {
                receipt.status = RigExecSubscriptionStatus::UnknownSubscription;
                receipt.reason =
                    "the subscription closed while the update was validated";
                return receipt;
            }
            found->second->request = staged->request;
            found->second->unsupported = std::move(staged->unsupported);
        }
        receipt.subscription = id;
        _DispatchDue();
        return receipt;
    }

    std::vector<RigExecOutputPacket> Poll(uint64_t id)
    {
        std::shared_ptr<Client> client;
        std::map<_GroupKey, _GroupView> views;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto found = _clients.find(id);
            if (found == _clients.end()) {
                return {};
            }
            client = found->second;
            for (const RigExecOutputTimeDemand &demand :
                 client->request.demands) {
                const _GroupKey key = _GroupOf(*client, demand.time);
                if (views.count(key)) {
                    continue;
                }
                _GroupView view;
                const auto group = _groups.find(key);
                if (group != _groups.end()) {
                    view.captured = group->second.captured;
                    view.status = group->second.status;
                    view.reason = group->second.reason;
                } else {
                    view.status = RigExecOutputStatus::Pending;
                    view.reason = "the demand has not been reconciled yet";
                }
                views.emplace(key, std::move(view));
            }
        }

        std::vector<RigExecOutputPacket> packets;
        std::vector<_PinKey> toPin;
        for (const RigExecOutputTimeDemand &demand : client->request.demands) {
            const _GroupKey groupKey = _GroupOf(*client, demand.time);
            const _GroupView &view = views[groupKey];
            for (const RigExecOutputHandle &handle : demand.handles) {
                RigExecOutputPacket packet;
                packet.handle = handle;
                packet.time = demand.time;
                const RigExecOutputHandle *refused = _Refused(*client, handle);
                if (refused) {
                    packet.status = RigExecOutputStatus::Unsupported;
                    packet.reason = _RefusalReason(*client, handle);
                    packets.push_back(std::move(packet));
                    continue;
                }
                const RigExecOutputPacket *answer = nullptr;
                RigExecCoverageRecord record;
                const RigExecCoverageKey *answeredKey = nullptr;
                for (const RigExecCoverageKey &key : view.captured) {
                    if (!_host.frameCache ||
                        !_host.frameCache->LookupCoverage(key, &record)) {
                        continue;
                    }
                    answer = _RecordAnswer(record, handle);
                    if (answer) {
                        answeredKey = &key;
                        break;
                    }
                }
                if (answer) {
                    if (handle.valueRevision != 0 &&
                        answer->handle.valueRevision != handle.valueRevision) {
                        packet.status = RigExecOutputStatus::Invalidated;
                        packet.reason =
                            handle.semanticKey + " stands at " +
                            std::to_string(answer->handle.valueRevision) +
                            "; the request named " +
                            std::to_string(handle.valueRevision);
                        packets.push_back(std::move(packet));
                        continue;
                    }
                    packet = *answer;
                    packet.handle = handle;
                    const _PinKey pinKey{*answeredKey, answer->handle};
                    if (!client->pins.count(pinKey)) {
                        toPin.push_back(pinKey);
                    }
                    packets.push_back(std::move(packet));
                    continue;
                }
                packet.status = view.status;
                packet.reason = view.reason;
                packets.push_back(std::move(packet));
            }
        }
        for (const _PinKey &pinKey : toPin) {
            _Pin(pinKey, id);
        }
        return packets;
    }

    size_t Pump()
    {
        _DispatchDue();
        std::vector<RigExecSubscriptionEvent> events;
        RigExecSubscriptionCallback callback;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            events.swap(_events);
            callback = _callback;
            if (callback) {
                for (const RigExecSubscriptionEvent &event : events) {
                    if (_clients.count(event.subscription)) {
                        _delivered.push_back(event);
                    }
                }
            }
        }
        if (!callback) {
            return 0;
        }
        std::vector<RigExecSubscriptionEvent> delivered;
        delivered.swap(_delivered);
        for (const RigExecSubscriptionEvent &event : delivered) {
            callback(event);
        }
        return delivered.size();
    }

    void SetCallback(RigExecSubscriptionCallback callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _callback = std::move(callback);
    }

    void Close(uint64_t id)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _CloseClientLocked(id);
        }
        _ReleaseStagedPins();
        _DispatchDue();
    }

    void CloseAll()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (uint64_t id : _clientIds()) {
                _CloseClientLocked(id);
            }
        }
        _ReleaseStagedPins();
        _DispatchDue();
    }

    RigExecSubscriptionStats Stats() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        RigExecSubscriptionStats stats = _stats;
        stats.subscriptions = _clients.size();
        stats.demandsInFlight = _demands.size();
        stats.framesDemanded = _groups.size();
        for (const auto &pin : _pins) {
            ++stats.pinnedPayloads;
            stats.pinnedBytes += pin.second.lease.ByteCount();
        }
        stats.callbacksQueued = _events.size();
        return stats;
    }

    bool IsStopping() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _stopping;
    }

    /// Closes every client, releases every lease and reservation this service
    /// owns, and joins the pool -- holding no lock while it does.
    void Shutdown()
    {
        std::vector<uint64_t> ids;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _stopping = true;
            _callback = nullptr;
            _events.clear();
            ids = _clientIds();
            for (const auto &entry : _demands) {
                entry.second->live = false;
            }
            _clients.clear();
            _demands.clear();
            _groups.clear();
        }
        for (uint64_t id : ids) {
            _StageClientPins(id);
        }
        _ReleaseStagedPins();
        if (_host.scheduler && _host.scheduler->GetWorkerCount() > 0) {
            _host.scheduler->WaitUntilIdle();
        }
    }

private:
    struct Client {
        uint64_t id = 0;
        SdfPath rig;
        std::string rigIdentity;
        uint64_t compileEpoch = 0;
        RigExecOutputRequest request;
        std::shared_ptr<const RigExecOutputCatalog> catalog;
        // Named outputs the catalog cannot produce, with its reason.
        std::vector<std::pair<RigExecOutputHandle, std::string>> unsupported;
        std::set<_PinKey> pins;
    };

    struct Demand {
        RigExecExplicitJobKey key;
        RigExecCoverageKey coverage;
        // The union this job answers, fixed at admission: a launched mask is
        // never widened afterwards.
        std::vector<RigExecOutputHandle> outputs;
        std::shared_ptr<RigExecSubscriptionCapture> capture;
        RigExecFrozenRequestedRun requested;
        RigExecCoverageReservation reservation;
        bool live = true;
    };

    struct Group {
        std::shared_ptr<const RigExecOutputCatalog> catalog;
        std::vector<RigExecOutputHandle> desired;
        std::vector<RigExecExplicitJobKey> inFlight;
        // The coverage keys this frame was captured under, newest first.
        std::vector<RigExecCoverageKey> captured;
        RigExecOutputStatus status = RigExecOutputStatus::Pending;
        std::string reason;
    };

    struct Pin {
        RigExecCoverageLease lease;
        std::set<uint64_t> clients;
    };

    struct _GroupView {
        std::vector<RigExecCoverageKey> captured;
        RigExecOutputStatus status = RigExecOutputStatus::Pending;
        std::string reason;
    };

    // Validates a request against the rig's catalog and fills `client`'s
    // identity and unsupported list. `currentCatalog` asks the host for the
    // rig's current epoch (a new subscription) or keeps the client's own
    // (a replacement for an existing one).
    bool _Validate(const RigExecOutputRequest &request, const SdfPath &rig,
                   bool currentCatalog, Client *client,
                   RigExecSubscriptionReceipt *receipt, std::string *error)
    {
        const auto refuse = [&](RigExecSubscriptionStatus status,
                                const std::string &message) {
            receipt->status = status;
            receipt->reason = message;
            return false;
        };
        if (!request.IsWellFormed(error)) {
            return refuse(RigExecSubscriptionStatus::InvalidRequest, *error);
        }
        std::string rigIdentity;
        uint64_t compileEpoch = 0;
        if (!RigExecRequestGroup(request, &rigIdentity, &compileEpoch, error)) {
            return refuse(RigExecSubscriptionStatus::InvalidRequest, *error);
        }
        std::shared_ptr<const RigExecOutputCatalog> catalog = client->catalog;
        if (currentCatalog) {
            catalog =
                _host.resolveCatalog ? _host.resolveCatalog(rig) : nullptr;
        }
        if (!catalog) {
            return refuse(RigExecSubscriptionStatus::UnknownRig,
                          "no output catalog names this rig");
        }
        if (catalog->GetRigIdentity() != rigIdentity ||
            catalog->GetCompileEpoch() != compileEpoch) {
            return refuse(RigExecSubscriptionStatus::StaleEpoch,
                          "the request names rig " + rigIdentity + " epoch " +
                              std::to_string(compileEpoch) + "; this rig is " +
                              catalog->GetRigIdentity() + " epoch " +
                              std::to_string(catalog->GetCompileEpoch()));
        }
        client->rig = rig;
        client->rigIdentity = rigIdentity;
        client->compileEpoch = compileEpoch;
        client->request = request;
        client->catalog = catalog;
        size_t producible = 0;
        for (const RigExecOutputTimeDemand &demand : request.demands) {
            for (const RigExecOutputHandle &handle : demand.handles) {
                size_t slot = 0;
                if (!catalog->GetDemandIndex().Find(handle, &slot)) {
                    return refuse(
                        RigExecSubscriptionStatus::UnknownOutput,
                        handle.semanticKey +
                            " is not a semantic address this catalog exports");
                }
                std::string reason;
                const RigExecOutputStatus resolved =
                    catalog->Resolve(handle, nullptr, &reason);
                if (resolved == RigExecOutputStatus::Invalidated) {
                    return refuse(RigExecSubscriptionStatus::StaleEpoch,
                                  handle.semanticKey + ": " + reason);
                }
                if (resolved == RigExecOutputStatus::Unsupported) {
                    client->unsupported.emplace_back(handle, reason);
                    continue;
                }
                ++producible;
            }
        }
        if (!producible) {
            return refuse(RigExecSubscriptionStatus::NothingProducible,
                          "no named output has a retained producer");
        }
        receipt->status = RigExecSubscriptionStatus::Accepted;
        receipt->frames = request.demands.size();
        for (const auto &entry : client->unsupported) {
            receipt->unsupported.push_back(entry.first);
        }
        return true;
    }

    static _GroupKey _GroupOf(const Client &client,
                              const RigExecOutputTime &time)
    {
        return _GroupKey{client.rig, time, client.compileEpoch};
    }

    static const RigExecOutputHandle *_Refused(const Client &client,
                                               const RigExecOutputHandle &want)
    {
        for (const auto &entry : client.unsupported) {
            if (entry.first.SameTarget(want)) {
                return &entry.first;
            }
        }
        return nullptr;
    }

    static std::string _RefusalReason(const Client &client,
                                      const RigExecOutputHandle &want)
    {
        for (const auto &entry : client.unsupported) {
            if (entry.first.SameTarget(want)) {
                return entry.second;
            }
        }
        return std::string();
    }

    std::vector<uint64_t> _clientIds() const
    {
        std::vector<uint64_t> ids;
        ids.reserve(_clients.size());
        for (const auto &entry : _clients) {
            ids.push_back(entry.first);
        }
        return ids;
    }

    // Call with the state lock held. Stages the client's last lease on each
    // payload so the caller can drop it outside the lock.
    void _CloseClientLocked(uint64_t id)
    {
        const auto found = _clients.find(id);
        if (found == _clients.end()) {
            return;
        }
        for (const _PinKey &pinKey : found->second->pins) {
            const auto pin = _pins.find(pinKey);
            if (pin == _pins.end()) {
                continue;
            }
            pin->second.clients.erase(id);
            if (pin->second.clients.empty()) {
                _StagesPinLocked(pinKey);
            }
        }
        _clients.erase(found);
        _events.erase(
            std::remove_if(_events.begin(), _events.end(),
                           [id](const RigExecSubscriptionEvent &event) {
                               return event.subscription == id;
                           }),
            _events.end());
    }

    void _StagesPinLocked(const _PinKey &pinKey)
    {
        const auto pin = _pins.find(pinKey);
        if (pin == _pins.end()) {
            return;
        }
        _stagedPins.push_back(std::move(pin->second.lease));
        _pins.erase(pin);
    }

    // Pins are the owner thread's; releasing one takes a cache shard lock, so
    // the lease always dies with the state lock closed.
    void _StageClientPins(uint64_t id)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (auto &pin : _pins) {
            if (pin.second.clients.erase(id) && pin.second.clients.empty()) {
                _StagesPinLocked(pin.first);
            }
        }
    }

    void _ReleaseStagedPins()
    {
        std::vector<RigExecCoverageLease> staged;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            staged.swap(_stagedPins);
        }
        staged.clear();
    }

    void _Pin(const _PinKey &pinKey, uint64_t id)
    {
        if (!_host.frameCache) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto pin = _pins.find(pinKey);
            if (pin != _pins.end()) {
                pin->second.clients.insert(id);
                const auto client = _clients.find(id);
                if (client != _clients.end()) {
                    client->second->pins.insert(pinKey);
                }
                return;
            }
        }
        RigExecCoverageLease lease;
        if (!_host.frameCache->PinCoverage(pinKey.coverage, pinKey.handle,
                                           &lease)) {
            return;
        }
        std::lock_guard<std::mutex> lock(_mutex);
        Pin &pin = _pins[pinKey];
        if (pin.lease.IsValid()) {
            // Another client pinned it while this poll was outside the lock;
            // the extra lease dies here.
            return;
        }
        pin.lease = std::move(lease);
        pin.clients.insert(id);
        const auto client = _clients.find(id);
        if (client != _clients.end()) {
            client->second->pins.insert(pinKey);
        }
    }

    // Owner-thread reconcile: prune jobs the pool already retired, re-union the
    // live demand, then capture and admit what is still owed. Every host hook,
    // cache call and pool call here runs with the state lock closed.
    void _DispatchDue()
    {
        if (!_host.scheduler || !_host.frameCache) {
            return;
        }
        std::vector<_GroupKey> owed;
        std::vector<std::pair<SdfPath, std::vector<UsdTimeCode>>> purges;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _PruneRetiredLocked();
            _RebuildDemandLocked(&purges);
            for (const auto &group : _groups) {
                if (group.second.desired.empty()) {
                    continue;
                }
                // Work already launched for this frame owns it: a launched
                // mask is fixed at admission, so an expansion joins it when
                // that job's record covers the demand and re-dispatches
                // beside it once the job reaches a terminal state.
                if (!group.second.inFlight.empty()) {
                    continue;
                }
                owed.push_back(group.first);
            }
        }
        for (const auto &purge : purges) {
            // Retirement for a frame no live client demands any more: its
            // queued job leaves and its byte reservation comes back. An empty
            // time list retires the default frame's job, which has no numeric
            // value to name it by.
            _host.scheduler->CancelGenerationTimes(purge.first, purge.second);
        }
        for (const _GroupKey &group : owed) {
            _CaptureAndAdmit(group);
        }
    }

    // A key the pool no longer reports in flight reached a terminal state this
    // service could not observe -- purged by another host's cancel, or dropped
    // at shutdown. Its demand is owed again.
    void _PruneRetiredLocked()
    {
        std::map<SdfPath, std::vector<RigExecExplicitJobKey>> queued;
        for (const auto &entry : _demands) {
            queued[entry.first.rig].push_back(entry.first);
        }
        for (const auto &entry : queued) {
            const std::vector<RigExecExplicitJobKey> standing =
                _host.scheduler->InFlightExplicitKeys(entry.first);
            for (const RigExecExplicitJobKey &key : entry.second) {
                if (std::find(standing.begin(), standing.end(), key) !=
                    standing.end()) {
                    continue;
                }
                _RetireDemandLocked(key, RigExecOutputStatus::Pending,
                                    "the pool retired this frame's job; the "
                                    "next pump re-admits it");
                ++_stats.retiredInFlight;
            }
        }
    }

    // Re-union every live client's demand. A frame no client demands leaves
    // with its queued job.
    void _RebuildDemandLocked(
        std::vector<std::pair<SdfPath, std::vector<UsdTimeCode>>> *purges)
    {
        for (auto &group : _groups) {
            group.second.desired.clear();
        }
        for (const auto &entry : _clients) {
            const Client &client = *entry.second;
            for (const RigExecOutputTimeDemand &demand :
                 client.request.demands) {
                Group &group = _groups[_GroupOf(client, demand.time)];
                if (!group.catalog) {
                    group.catalog = client.catalog;
                }
                for (const RigExecOutputHandle &handle : demand.handles) {
                    _UnionHandle(&group.desired, handle);
                }
            }
        }
        for (auto it = _groups.begin(); it != _groups.end();) {
            if (!it->second.desired.empty()) {
                ++it;
                continue;
            }
            std::vector<UsdTimeCode> times;
            if (!it->first.time.IsDefault()) {
                times.push_back(UsdTimeCode(it->first.time.TimeCode()));
            }
            const bool hadWork = !it->second.inFlight.empty();
            for (const RigExecExplicitJobKey &key : it->second.inFlight) {
                _RetireDemandLocked(key, RigExecOutputStatus::Pending,
                                    "no live subscription demands this frame");
            }
            if (hadWork) {
                purges->emplace_back(it->first.rig, std::move(times));
            }
            it = _groups.erase(it);
        }
    }

    // Drop one demand and its group link. The queued closure, and with it the
    // byte reservation, dies wherever the pool retires the job.
    void _RetireDemandLocked(const RigExecExplicitJobKey &key,
                             RigExecOutputStatus status,
                             const std::string &reason)
    {
        const auto demand = _demands.find(key);
        if (demand != _demands.end()) {
            demand->second->live = false;
            _demands.erase(demand);
        }
        for (auto &group : _groups) {
            if (!(group.first.rig == key.rig && group.first.time == key.time &&
                  group.first.compileEpoch == key.generation.programIdentity)) {
                continue;
            }
            group.second.inFlight.erase(
                std::remove(group.second.inFlight.begin(),
                            group.second.inFlight.end(), key),
                group.second.inFlight.end());
            group.second.status = status;
            group.second.reason = reason;
        }
    }

    // Capture this frame's job on the owner thread and admit it -- unless the
    // cache already answers the whole demand, which pulls nothing.
    void _CaptureAndAdmit(const _GroupKey &groupKey)
    {
        std::shared_ptr<const RigExecOutputCatalog> catalog;
        std::vector<RigExecOutputHandle> desired;
        std::vector<RigExecCoverageKey> captured;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto group = _groups.find(groupKey);
            if (group == _groups.end() || group->second.desired.empty() ||
                !group->second.inFlight.empty() || _stopping) {
                return;
            }
            catalog = group->second.catalog;
            desired = group->second.desired;
            captured = group->second.captured;
        }
        if (!catalog || !_host.capture) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Failed,
                              "no capture hook for this rig");
            return;
        }
        if (_CoverageCovers(captured, desired)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Ready,
                              "the coverage record already answers this frame");
            ++_stats.coverageHits;
            return;
        }

        RigExecOutputTimeDemand demand;
        demand.time = groupKey.time;
        demand.handles = desired;
        auto capture = std::make_shared<RigExecSubscriptionCapture>();
        std::string error;
        if (!_host.capture(groupKey.rig, demand, *catalog, capture.get(),
                           &error)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Failed,
                              "capture: " + error);
            ++_stats.captureFailures;
            return;
        }
        capture->BindContext();
        const RigExecFrameGeneration editGeneration =
            _host.scheduler->CurrentGeneration(groupKey.rig);
        capture->context.generation = editGeneration;
        if (capture->captured.programIdentity == 0) {
            capture->captured.programIdentity = catalog->GetCompileEpoch();
        } else if (capture->captured.programIdentity !=
                   catalog->GetCompileEpoch()) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(
                groupKey, RigExecOutputStatus::Invalidated,
                "the capture names program " +
                    std::to_string(capture->captured.programIdentity) +
                    "; this catalog is epoch " +
                    std::to_string(catalog->GetCompileEpoch()));
            return;
        }

        RigExecCoverageKey coverageKey;
        coverageKey.rigIdentity = catalog->GetRigIdentity();
        coverageKey.compileEpoch = catalog->GetCompileEpoch();
        coverageKey.time = groupKey.time;
        coverageKey.generation = capture->captured;
        coverageKey.upstreamRevision = capture->upstreamRevision;
        // The capture names the generation that stands now, and its answers
        // already stand: serve them, dispatch nothing.
        if (_CoverageCovers({coverageKey}, desired)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _RememberCoverageLocked(groupKey, coverageKey);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Ready,
                              "the coverage record answers this frame");
            ++_stats.coverageHits;
            return;
        }

        RigExecCoverageBytes budget = capture->bytes;
        if (budget.payload == 0) {
            budget.payload = _PayloadFloorBytes(desired);
        }
        RigExecCoverageReservation reservation;
        if (!_host.frameCache->TryReserveCoverage(budget, &reservation)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Pending,
                              "the coverage budget declined this frame; the "
                              "next pump retries it");
            ++_stats.declinedCoverageBudget;
            return;
        }

        RigExecFrozenRequestedRun requested;
        if (!RigExecMakeFrozenRequestedRun(catalog, demand, capture->captured,
                                           &requested, &error)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Unsupported,
                              error);
            return;
        }

        auto job = std::make_shared<Demand>();
        job->key.rig = groupKey.rig;
        job->key.time = groupKey.time;
        job->key.generation = capture->captured;
        job->coverage = coverageKey;
        job->outputs = desired;
        job->capture = capture;
        job->requested = std::move(requested);
        job->reservation = std::move(reservation);

        RigExecExplicitRequest admission;
        admission.rig = groupKey.rig;
        admission.time = groupKey.time;
        admission.generation = capture->captured;
        admission.outputs = desired;
        admission.editGeneration = editGeneration;
        size_t reservedBytes = 0;
        budget.TryTotal(&reservedBytes);
        admission.reservationBytes = reservedBytes;

        RigExecExplicitWork work = [state = shared_from_this(), job](
                                       const RigExecExplicitRequest &request) {
            return state->RunDemand(job, request);
        };
        if (!_host.scheduler->AdmitExplicitOutput(admission, std::move(work))) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Pending,
                              "the pool declined this admission; the next pump "
                              "retries it");
            ++_stats.declinedAdmission;
            return;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _demands.emplace(job->key, job);
            ++_stats.dispatches;
            const auto group = _groups.find(groupKey);
            if (group == _groups.end()) {
                return;
            }
            group->second.inFlight.push_back(job->key);
            _RememberCoverageLocked(groupKey, coverageKey);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Pending,
                              "a job for this frame is queued");
        }
    }

    // Every demanded output already stands in one of these records.
    bool _CoverageCovers(const std::vector<RigExecCoverageKey> &keys,
                         const std::vector<RigExecOutputHandle> &desired) const
    {
        if (!_host.frameCache) {
            return false;
        }
        for (const RigExecCoverageKey &key : keys) {
            RigExecCoverageRecord record;
            if (!_host.frameCache->LookupCoverage(key, &record)) {
                continue;
            }
            bool all = true;
            for (const RigExecOutputHandle &handle : desired) {
                if (!_RecordAnswer(record, handle)) {
                    all = false;
                    break;
                }
            }
            if (all) {
                return true;
            }
        }
        return false;
    }

    void _NoteStatusLocked(const _GroupKey &key, RigExecOutputStatus status,
                           const std::string &reason)
    {
        const auto group = _groups.find(key);
        if (group == _groups.end()) {
            return;
        }
        group->second.status = status;
        group->second.reason = reason;
    }

    void _RememberCoverageLocked(const _GroupKey &key,
                                 const RigExecCoverageKey &coverageKey)
    {
        const auto group = _groups.find(key);
        if (group == _groups.end()) {
            return;
        }
        std::vector<RigExecCoverageKey> &captured = group->second.captured;
        captured.erase(
            std::remove(captured.begin(), captured.end(), coverageKey),
            captured.end());
        captured.insert(captured.begin(), coverageKey);
        if (captured.size() > kRigExecSubscriptionGenerationWindow) {
            captured.resize(kRigExecSubscriptionGenerationWindow);
        }
    }

    // The job body, on a pool worker. The capture, the reservation and the
    // coverage key are this job's alone; the maps are read under the state
    // lock, which is never held across the run, a fence, or the publish.
    RigExecExplicitOutcome RunDemand(const std::shared_ptr<Demand> &job,
                                     const RigExecExplicitRequest &request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (!job->live || _stopping) {
                return RigExecExplicitOutcome::DeclinedInvalid;
            }
        }
        // The pool's fence first: a scoped edit or a generation bump since this
        // job was queued means its answer belongs to a retired frame.
        if (!_host.scheduler->IsExplicitRequestCurrent(request)) {
            return _RetireRun(job, request,
                              RigExecExplicitOutcome::DeclinedGeneration,
                              RigExecOutputStatus::Invalidated,
                              "the pool retired this frame while its job ran");
        }
        const RigExecFrozenRequestedResult result =
            RigExecEvaluateFrozenRequested(
                job->capture->context, job->capture->inputs, job->requested,
                _host.stepRunner, _host.scheduler, request.rig);
        if (!result.ran) {
            const bool retired =
                result.error.find("generation") != std::string::npos;
            return _RetireRun(job, request,
                              retired
                                  ? RigExecExplicitOutcome::DeclinedGeneration
                                  : RigExecExplicitOutcome::DeclinedInvalid,
                              retired ? RigExecOutputStatus::Invalidated
                                      : RigExecOutputStatus::Failed,
                              "the requested run declined: " + result.error);
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (!job->live || _stopping) {
                // The bodies ran, but no live demand owns this frame: the
                // reservation comes back and nothing is published.
                return RigExecExplicitOutcome::DeclinedInvalid;
            }
        }
        // The publish fence, beside the generation check the run itself makes:
        // an edit that landed while the bodies were running owns this frame
        // now, so an answer under the retired token never lands.
        if (!_host.scheduler->IsExplicitRequestCurrent(request)) {
            return _RetireRun(job, request,
                              RigExecExplicitOutcome::DeclinedGeneration,
                              RigExecOutputStatus::Invalidated,
                              "the pool retired this frame while its job ran");
        }
        size_t payload = 0;
        for (const RigExecOutputPacket &packet : result.packets) {
            payload += packet.payload.ByteCount();
        }
        if (payload > 0 &&
            !job->reservation.Extend(RigExecCoverageDomain::Payload, payload)) {
            return _RetireRun(
                job, request, RigExecExplicitOutcome::DeclinedInvalid,
                RigExecOutputStatus::Failed,
                "the coverage budget declined this frame's payloads");
        }
        // The lane's arena dies with the job, so the record keeps the payloads
        // and claims no retained workspace.
        if (!_host.frameCache->PublishCoverage(job->coverage, result.packets,
                                               std::move(job->reservation),
                                               false)) {
            return _RetireRun(job, request,
                              RigExecExplicitOutcome::DeclinedInvalid,
                              RigExecOutputStatus::Failed,
                              "the frame cache declined this coverage record");
        }
        _CompleteRun(job, request, result.packets);
        return RigExecExplicitOutcome::Published;
    }

    RigExecExplicitOutcome _RetireRun(const std::shared_ptr<Demand> &job,
                                      const RigExecExplicitRequest &request,
                                      RigExecExplicitOutcome outcome,
                                      RigExecOutputStatus status,
                                      const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _RetireDemandLocked(job->key, status, reason);
        _QueueEventsLocked(request.rig, job->coverage.time, status, reason, 0,
                           job->outputs.size());
        return outcome;
    }

    // Records what landed. An expansion noticed while this job ran waits for
    // the owner's next pump: the launched mask stays exactly as admitted.
    void _CompleteRun(const std::shared_ptr<Demand> &job,
                      const RigExecExplicitRequest &request,
                      const std::vector<RigExecOutputPacket> &packets)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        size_t answered = 0;
        for (const RigExecOutputHandle &handle : job->outputs) {
            for (const RigExecOutputPacket &packet : packets) {
                if (packet.status == RigExecOutputStatus::Ready &&
                    packet.handle.SameTarget(handle)) {
                    ++answered;
                    break;
                }
            }
        }
        const bool complete = answered == job->outputs.size();
        const RigExecOutputStatus status =
            complete ? RigExecOutputStatus::Ready : RigExecOutputStatus::Failed;
        const std::string reason =
            complete
                ? std::string()
                : "the run answered " + std::to_string(answered) + " of " +
                      std::to_string(job->outputs.size()) + " demanded outputs";
        const auto demand = _demands.find(job->key);
        if (demand != _demands.end()) {
            demand->second->live = false;
            _demands.erase(demand);
        }
        for (auto &group : _groups) {
            if (!(group.first.rig == job->key.rig &&
                  group.first.time == job->key.time &&
                  group.first.compileEpoch ==
                      job->key.generation.programIdentity)) {
                continue;
            }
            group.second.inFlight.erase(
                std::remove(group.second.inFlight.begin(),
                            group.second.inFlight.end(), job->key),
                group.second.inFlight.end());
            _RememberCoverageLocked(group.first, job->coverage);
            group.second.status = status;
            group.second.reason = reason;
        }
        _QueueEventsLocked(request.rig, job->coverage.time, status, reason,
                           answered, job->outputs.size());
    }

    // One event per live client of the frame. Pump drops an event whose client
    // closed before it ran.
    void _QueueEventsLocked(const SdfPath &rig, const RigExecOutputTime &time,
                            RigExecOutputStatus status,
                            const std::string &reason, size_t answered,
                            size_t demanded)
    {
        for (const auto &entry : _clients) {
            const Client &client = *entry.second;
            if (!(client.rig == rig)) {
                continue;
            }
            size_t handles = 0;
            for (const RigExecOutputTimeDemand &demand :
                 client.request.demands) {
                if (demand.time == time) {
                    handles = demand.handles.size();
                    break;
                }
            }
            if (!handles) {
                continue;
            }
            RigExecSubscriptionEvent event;
            event.subscription = client.id;
            event.rig = rig;
            event.time = time;
            event.status = status;
            event.reason = reason;
            event.answered = answered;
            event.demanded = demanded ? demanded : handles;
            _events.push_back(std::move(event));
        }
    }

    mutable std::mutex _mutex;
    RigExecSubscriptionHost _host;
    std::map<uint64_t, std::shared_ptr<Client>> _clients;
    std::map<_GroupKey, Group> _groups;
    std::map<RigExecExplicitJobKey, std::shared_ptr<Demand>> _demands;
    std::map<_PinKey, Pin> _pins;
    std::vector<RigExecCoverageLease> _stagedPins;
    std::vector<RigExecSubscriptionEvent> _events;
    std::vector<RigExecSubscriptionEvent> _delivered;
    RigExecSubscriptionCallback _callback;
    RigExecSubscriptionStats _stats;
    uint64_t _nextId = 0;
    bool _stopping = false;
};

RigExecSubscriptionHandle::~RigExecSubscriptionHandle()
{
    Close();
}

void
RigExecSubscriptionHandle::Close()
{
    if (!_state || _id == 0) {
        return;
    }
    _state->Close(_id);
    _id = 0;
}

RigExecOutputSubscriptionService::RigExecOutputSubscriptionService(
    RigExecSubscriptionHost host)
    : _state(std::make_shared<RigExecSubscriptionState>(std::move(host)))
{
}

RigExecOutputSubscriptionService::~RigExecOutputSubscriptionService()
{
    if (_state) {
        _state->Shutdown();
    }
}

RigExecSubscriptionHandle
RigExecOutputSubscriptionService::Subscribe(const SdfPath &rig,
                                            const RigExecOutputRequest &request,
                                            RigExecSubscriptionReceipt *receipt)
{
    const RigExecSubscriptionReceipt result = _state->Subscribe(rig, request);
    if (receipt) {
        *receipt = result;
    }
    if (!result.IsAccepted()) {
        return RigExecSubscriptionHandle();
    }
    return RigExecSubscriptionHandle(_state, result.subscription);
}

RigExecSubscriptionReceipt
RigExecOutputSubscriptionService::UpdateSubscription(
    const RigExecSubscriptionHandle &subscription,
    const RigExecOutputRequest &request)
{
    return _state->Update(subscription.Id(), request);
}

RigExecSubscriptionReceipt
RigExecOutputSubscriptionService::UpdateSubscription(
    uint64_t subscription, const RigExecOutputRequest &request)
{
    return _state->Update(subscription, request);
}

std::vector<RigExecOutputPacket>
RigExecOutputSubscriptionService::PollResults(
    const RigExecSubscriptionHandle &subscription)
{
    return _state->Poll(subscription.Id());
}

std::vector<RigExecOutputPacket>
RigExecOutputSubscriptionService::PollResults(uint64_t subscription)
{
    return _state->Poll(subscription);
}

size_t
RigExecOutputSubscriptionService::PumpCallbacks()
{
    return _state->Pump();
}

void
RigExecOutputSubscriptionService::SetCompletionCallback(
    RigExecSubscriptionCallback callback)
{
    _state->SetCallback(std::move(callback));
}

void
RigExecOutputSubscriptionService::Close(
    const RigExecSubscriptionHandle &subscription)
{
    _state->Close(subscription.Id());
}

void
RigExecOutputSubscriptionService::Close(uint64_t subscription)
{
    _state->Close(subscription);
}

void
RigExecOutputSubscriptionService::CloseAll()
{
    _state->CloseAll();
}

RigExecSubscriptionStats
RigExecOutputSubscriptionService::Stats() const
{
    return _state->Stats();
}

bool
RigExecOutputSubscriptionService::IsStopping() const
{
    return _state->IsStopping();
}

size_t
RigExecEstimateCaptureBytes(const RigExecFrameInputs &inputs)
{
    size_t bytes = inputs.values.size() * sizeof(RigExecSampledInput);
    for (const std::vector<VtValue> &row : inputs.layoutLeaves) {
        bytes += row.size() * sizeof(VtValue) + sizeof(std::vector<VtValue>);
    }
    for (const std::vector<SdfPath> &row : inputs.layoutSourcePaths) {
        bytes += row.size() * sizeof(SdfPath) + sizeof(std::vector<SdfPath>);
    }
    for (const std::vector<VtValue> &row : inputs.revisionLeaves) {
        bytes += row.size() * sizeof(VtValue) + sizeof(std::vector<VtValue>);
    }
    for (const std::vector<VtValue> &row : inputs.derivedLeaves) {
        bytes += row.size() * sizeof(VtValue) + sizeof(std::vector<VtValue>);
    }
    bytes += inputs.overridePaths.size() * sizeof(SdfPath);
    bytes += inputs.overrides.size() * sizeof(RigExecValueOverride);
    bytes += inputs.upstream.size() * sizeof(RigExecUpstreamValue);
    return bytes;
}

} // namespace rigExec
