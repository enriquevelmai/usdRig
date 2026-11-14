// rigExecRuntime export catalog: the outputs a .rigexec names, bound at Open
// to this program's own storage, and the requested run that answers them
// (docs/specs/output-subscriptions.md). The file spells each output's semantic
// address and the retained storage behind it; a handle's rig identity and
// compile epoch are this reader's, because a .rigexec never carries the live
// identity its exporter held. USD-free.
#include "rigExecRuntime/store.h"

#include "rigExecSchema/schemaNames.h"

#include <algorithm>
#include <atomic>
#include <string>
#include <vector>

namespace rigExec {

namespace {

// The read-phase spellings discovery gives a chain's first and last version;
// they match RigExecOutputCatalog's, which is what makes one semantic address
// name one output in every evaluator.
constexpr const char *const kPhaseBase = "base";
constexpr const char *const kPhaseFinal = "final";

std::string
_Row(size_t index)
{
    return "export_catalog.entries[" + std::to_string(index) + "]";
}

// One compile per reader of a file: each Open is a separate program, and its
// handles must not answer against another reader's. Namespace scope and
// constant-initialized, so no guard runs at Open.
std::atomic<uint64_t> _rrCompileEpoch(0);

const char *
_PayloadText(fb::ExportPayload payload)
{
    switch (payload) {
    case fb::ExportPayload::Double:
        return RigExecPayloadText::scalar;
    case fb::ExportPayload::Matrix4d:
        return RigExecPayloadText::matrix4x4Row;
    case fb::ExportPayload::Point3dArray:
        return RigExecPayloadText::point3;
    }
    return nullptr;
}

// The graph value one entry's storage names: a property chain's version, a
// provider slot's base or final matrix, a frame record, or one point version
// of a chain. False with the row's own reason for an address the tables do not
// hold. The format validator bounds the producer value's domain; this binds
// the slot, which is what stops one entry answering another object's storage.
bool
_ExpectedValue(const RrProgram &program, const RigExecWireExportEntry &entry,
               const std::string &row, uint32_t *domain, uint32_t *slot,
               std::string *why)
{
    using D = RigExecWireSlotDomain;
    const RigExecWireFile &file = *program.file;
    const auto bad = [&](const std::string &message) {
        if (why)
            *why = row + ": " + message;
        return false;
    };
    switch (entry.storage) {
    case fb::ExportStorage::PropertyValue: {
        if (entry.object >= file.propertyChains.size())
            return bad("object " + std::to_string(entry.object) +
                       " is no property chain");
        const auto &chain = file.propertyChains[entry.object];
        if (entry.version > chain.revisions.size())
            return bad("version " + std::to_string(entry.version) +
                       " is past property chain " +
                       std::to_string(entry.object) + "'s " +
                       std::to_string(chain.revisions.size()) + " revisions");
        *domain = uint32_t(D::PropertyResult);
        *slot = chain.versionBase + entry.version;
        return true;
    }
    case fb::ExportStorage::ProviderMatrix: {
        if (entry.object >= program.slotMeta->paths.size())
            return bad("object " + std::to_string(entry.object) +
                       " is no provider slot");
        if (entry.phase == kPhaseBase)
            *domain = uint32_t(D::BaseMatrix);
        else if (entry.phase == kPhaseFinal)
            *domain = uint32_t(D::FinalMatrix);
        else
            return bad("a provider matrix answers phase base or final, not " +
                       entry.phase);
        *slot = entry.object;
        return true;
    }
    case fb::ExportStorage::FrameMatrix:
        if (entry.object >= file.pose->frameRecords.size())
            return bad("object " + std::to_string(entry.object) +
                       " is no frame record");
        *domain = uint32_t(D::FrameMatrix);
        *slot = entry.object;
        return true;
    case fb::ExportStorage::PointVersion: {
        if (entry.object >= file.geometry->chains.size())
            return bad("object " + std::to_string(entry.object) +
                       " is no geometry chain");
        const auto &chain = file.geometry->chains[entry.object];
        if (entry.version > chain.revisions.size())
            return bad("version " + std::to_string(entry.version) +
                       " is past chain " + std::to_string(entry.object) +
                       "'s " + std::to_string(chain.revisions.size()) +
                       " point versions");
        if (entry.phase == kPhaseFinal) {
            *domain = uint32_t(D::ChainPoints);
            *slot = entry.object;
            return true;
        }
        if (entry.version == 0) {
            *domain = uint32_t(D::ChainBase);
            *slot = entry.object;
            return true;
        }
        // RevisionDone is a revision fuse's publication record: the value that
        // names the point version the fuse left.
        *domain = uint32_t(D::RevisionDone);
        *slot = 0;
        for (size_t r = 0; r < file.geometry->revisionIndex.size(); ++r) {
            const RigExecWireIntPair &id = file.geometry->revisionIndex[r];
            if (id.first == int32_t(entry.object) &&
                id.second == int32_t(entry.version - 1)) {
                *slot = uint32_t(r);
                return true;
            }
        }
        return bad("chain " + std::to_string(entry.object) + " version " +
                   std::to_string(entry.version) +
                   " has no canonical revision");
    }
    }
    return bad("an export enum is out of range");
}

// The phase an entry names must be the phase its storage and version imply:
// "base" for version 0, "final" for the last version, or the path of the mover
// that published that version. Extraction dispatches on the phase -- a
// provider matrix reads its base or its final table -- so a file whose phase
// disagrees with its version is refused, not answered from the wrong storage.
// The semantic key itself is the file's: discovery spells it from the compiled
// rig's stage paths, and the reader carries it unchanged.
bool
_CheckPhase(const RrProgram &program, const RigExecWireExportEntry &entry,
            const std::string &row, std::string *why)
{
    const RigExecWireFile &file = *program.file;
    const auto bad = [&](const std::string &message) {
        if (why)
            *why = row + ": " + message;
        return false;
    };
    switch (entry.storage) {
    case fb::ExportStorage::PropertyValue:
    case fb::ExportStorage::PointVersion: {
        const size_t revisions =
            entry.storage == fb::ExportStorage::PropertyValue
                ? file.propertyChains[entry.object].revisions.size()
                : file.geometry->chains[entry.object].revisions.size();
        if (entry.phase == kPhaseBase)
            return entry.version == 0 ? true
                                      : bad("phase base names version 0, not " +
                                            std::to_string(entry.version));
        if (entry.phase == kPhaseFinal)
            return entry.version == revisions
                       ? true
                       : bad("phase final names version " +
                             std::to_string(revisions) + ", not " +
                             std::to_string(entry.version));
        if (entry.version == 0 || entry.version > revisions)
            return bad("phase " + entry.phase + " names no version of " +
                       std::to_string(revisions));
        const std::string mover =
            entry.storage == fb::ExportStorage::PropertyValue
                ? RigExecFormatPathText(
                      file, file.propertyChains[entry.object]
                                .revisions[size_t(entry.version - 1)]
                                .mover)
                : RigExecFormatPathText(
                      file, file.geometry->chains[entry.object]
                                .revisions[size_t(entry.version - 1)]
                                .moverPath);
        if (entry.phase != mover)
            return bad("phase " + entry.phase + " is not the mover " + mover +
                       " that published version " +
                       std::to_string(entry.version));
        return true;
    }
    case fb::ExportStorage::ProviderMatrix:
        if (entry.phase != kPhaseBase && entry.phase != kPhaseFinal)
            return bad("a provider matrix answers phase base or final, not " +
                       entry.phase);
        return true;
    case fb::ExportStorage::FrameMatrix:
        if (entry.phase.empty())
            return bad("a frame record names no writer");
        return true;
    }
    return bad("an export enum is out of range");
}

} // namespace

bool
RrBindExportCatalog(RrProgram *program, std::string *error)
{
    const auto fail = [&](const std::string &why) {
        if (error)
            *error = why;
        return false;
    };
    program->exports.clear();
    program->exportEdges = RigExecDemandEdges();
    program->exportIndex = RigExecOutputDemandIndex();
    program->exportIndexReady = false;
    program->rigIdentity.clear();
    program->compileEpoch = 0;
    // A program assembled by hand, and a file older than the export catalog,
    // name no outputs: discovery answers nothing and a request is refused.
    if (!program->file || !program->file->exportCatalog)
        return true;
    const std::vector<RigExecWireExportEntry> &entries =
        program->file->exportCatalog->entries;
    if (entries.empty())
        return true;
    program->rigIdentity =
        RigExecFormatPathText(*program->file, program->file->rig);
    if (program->rigIdentity.empty())
        return fail("the file's rig path spells no text for its exports");
    // Open mints the compile epoch: one program per reader, so a handle never
    // repeats across readers or outlives the reader that minted it.
    program->compileEpoch =
        _rrCompileEpoch.fetch_add(1, std::memory_order_relaxed) + 1;

    program->exports.reserve(entries.size());
    for (size_t k = 0; k < entries.size(); ++k) {
        const std::string row = _Row(k);
        const RigExecWireExportEntry &entry = entries[k];
        const char *const type = _PayloadText(entry.payload);
        if (!type)
            return fail(row + ": names a payload layout this reader cannot "
                              "read");
        uint32_t domain = 0, valueSlot = 0;
        std::string why;
        if (!_ExpectedValue(*program, entry, row, &domain, &valueSlot, &why))
            return fail(why);
        if (!_CheckPhase(*program, entry, row, &why))
            return fail(why);

        RrExport output;
        output.object = entry.object;
        output.version = entry.version;
        output.storage = entry.storage;
        output.handle.rigIdentity = program->rigIdentity;
        output.handle.compileEpoch = program->compileEpoch;
        output.handle.semanticKey = entry.semanticKey;
        output.handle.type = type;
        output.handle.readPhase = entry.phase;
        output.extractable = entry.capability != fb::ExportCapability::Unusable;
        output.reason = entry.reason;
        if (!output.extractable && entry.reason.empty())
            return fail(row + ": an unusable export names no reason");
        if (entry.producerValue == -1) {
            if (output.extractable)
                return fail(row + ": no producer value, so it is not "
                                  "extractable");
            program->exports.push_back(std::move(output));
            continue;
        }
        if (size_t(entry.producerValue) >=
            program->store.opAdapter.values.size())
            return fail(row + ": producer_value " +
                        std::to_string(size_t(entry.producerValue)) +
                        " is no graph value");
        const RigExecOpValueState &value =
            program->store.opAdapter.values[size_t(entry.producerValue)];
        if (value.domain != domain || value.slot != valueSlot)
            return fail(row + ": producer_value names value (" +
                        std::to_string(value.domain) + ", " +
                        std::to_string(value.slot) + "), not the storage (" +
                        std::to_string(domain) + ", " +
                        std::to_string(valueSlot) + ") its entry binds");
        if (!RigExecFormatExportDomainMatches(entry.storage, value.domain))
            return fail(row + ": producer_value binds a slot domain its "
                              "storage does not answer");
        output.producerValue = RigExecValueId(entry.producerValue);
        program->exports.push_back(std::move(output));
    }
    return true;
}

bool
RrBuildExportIndex(RrProgram *program, std::string *error)
{
    const auto fail = [&](const std::string &why) {
        if (error)
            *error = why;
        return false;
    };
    if (program->exportIndexReady)
        return true;
    if (program->exports.empty())
        return true;
    if (!program->store.opAdapter.compiled)
        return fail("this program has no canonical operation graph");
    // The requirement edges the demand contract asks for, compiled from this
    // graph: an effect edge across every predecessor whose skip changes state,
    // and each windowed operation's inner operations as history edges.
    const size_t count = program->opGraph.ops.size();
    RigExecDemandEdges &edges = program->exportEdges;
    edges.validity.assign(count, {});
    edges.effects.assign(count, {});
    edges.history.assign(count, {});
    std::vector<char> hasSkipEffect(count, 0);
    for (uint32_t op : program->store.opAdapter.skipEffects)
        if (op < count)
            hasSkipEffect[op] = 1;
    for (uint32_t op = 0; op < count; ++op)
        for (uint32_t predecessor : program->opGraph.ops[op].predecessors)
            if (hasSkipEffect[predecessor])
                edges.effects[op].push_back(predecessor);
    for (const RrProgram::WindowedOp &window : program->windowedOps) {
        if (window.op >= count)
            continue;
        std::vector<uint32_t> &list = edges.history[window.op];
        for (uint32_t inner : window.inner)
            if (inner < count)
                list.push_back(inner);
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
    }

    std::vector<RigExecOutputDemandEntry> demand;
    demand.reserve(program->exports.size());
    for (const RrExport &output : program->exports)
        demand.push_back({output.handle, output.producerValue});
    std::string why;
    if (!program->exportIndex.Build(program->opGraph, edges, demand, &why))
        return fail("output demand index: " + why);
    for (size_t i = 0; i < program->exports.size(); ++i) {
        RrExport &output = program->exports[i];
        if (output.extractable &&
            program->exportIndex.Producer(i) == kRigExecNoOp) {
            output.extractable = false;
            if (output.reason.empty())
                output.reason = "exported output has no canonical producer: " +
                                output.handle.semanticKey;
        }
    }
    program->exportIndexReady = true;
    return true;
}

bool
RrExtractExport(const RrProgram *program, const RrExport &output,
                RigExecOutputPayload *payload, std::string *reason)
{
    const auto fail = [&](const std::string &message) {
        if (reason)
            *reason = message;
        return false;
    };
    const RigExecWireFile &file = *program->file;
    const RrStore &store = program->store;
    switch (output.storage) {
    case fb::ExportStorage::PropertyValue: {
        if (output.object >= file.propertyChains.size())
            return fail("property output " + output.handle.semanticKey +
                        " names a chain the program does not hold");
        const auto &chain = file.propertyChains[output.object];
        // Discovery exports only the float and double arms, whose payload is
        // one double; the matrix and vec3f arms have no layout to pack.
        if (chain.valueType != RigExecWirePropertyValueType::Float &&
            chain.valueType != RigExecWirePropertyValueType::Double)
            return fail("property chain " + output.handle.semanticKey +
                        " has no scalar payload for its type");
        const size_t version =
            size_t(chain.versionBase) + size_t(output.version);
        if (version >= store.propertyVersions.size() ||
            version >= store.propertyVersionValid.size())
            return fail("property output " + output.handle.semanticKey +
                        " names a version the program does not hold");
        // The body running is not the body writing: a chain whose base read
        // failed, or whose authored base was not finite, leaves its storage
        // untouched and its valid byte 0.
        if (!store.propertyVersionValid[version])
            return fail("property chain " + output.handle.semanticKey +
                        " produced no value");
        const RrPropertyValue &stored = store.propertyVersions[version];
        const double value = stored.tag == RrPropertyValue::Tag::Double
                                 ? stored.f64
                                 : double(stored.f32);
        return RigExecPackPayload(RigExecPayloadKind::Scalar, 1, &value,
                                  payload, reason);
    }
    case fb::ExportStorage::ProviderMatrix: {
        const std::vector<RrMat4d> &table =
            output.handle.readPhase == kPhaseBase ? store.baseMatrix
                                                  : store.finalMatrix;
        if (output.object >= table.size())
            return fail("transform output " + output.handle.semanticKey +
                        " names a provider slot the program does not hold");
        // Row-major, USD row-vector form: p' = p * M, the order RrMat4d stores
        // and the order the payload layout declares.
        return RigExecPackPayload(RigExecPayloadKind::Matrix4x4Row, 1,
                                  table[size_t(output.object)]._mtx[0], payload,
                                  reason);
    }
    case fb::ExportStorage::FrameMatrix: {
        if (output.object >= store.frameMatrix.size() ||
            output.object >= store.frameMatrixValid.size())
            return fail("transform output " + output.handle.semanticKey +
                        " names a frame record the program does not hold");
        // The FrameMatrix step writes both fields every run, so a valid byte
        // of 0 is this run's refusal, not a leftover from an earlier one.
        if (!store.frameMatrixValid[size_t(output.object)])
            return fail("frame record " + output.handle.semanticKey +
                        " declined to record a matrix");
        return RigExecPackPayload(
            RigExecPayloadKind::Matrix4x4Row, 1,
            store.frameMatrix[size_t(output.object)]._mtx[0], payload, reason);
    }
    case fb::ExportStorage::PointVersion:
        return RrGeoExtractPoints(program, output, payload, reason);
    }
    return fail("export " + output.handle.semanticKey +
                " names storage this reader does not answer");
}

RigExecOutputStatus
RrResolveOutput(const RrProgram *program, const RigExecOutputHandle &handle,
                std::string *reason)
{
    const auto finish = [&](RigExecOutputStatus status,
                            const std::string &message) {
        if (reason)
            *reason = message;
        return status;
    };
    if (handle.rigIdentity != program->rigIdentity)
        return finish(RigExecOutputStatus::Invalidated,
                      "output belongs to another rig instance");
    if (handle.compileEpoch != program->compileEpoch)
        return finish(RigExecOutputStatus::Invalidated,
                      "output compile epoch is stale");
    for (const RrExport &output : program->exports)
        if (output.handle.SameTarget(handle))
            return finish(output.extractable ? RigExecOutputStatus::Pending
                                             : RigExecOutputStatus::Unsupported,
                          output.reason);
    return finish(RigExecOutputStatus::Unsupported,
                  "output semantic address is not exported");
}

namespace {

// What stands for one requested output once the run joined: the catalog's own
// answer for an address it cannot produce, otherwise this run's fact about the
// producer value, then the typed read of the storage that value lives in.
void
_Answer(const RrProgram &program, const RrExport &output,
        const RigExecOutputHandle &requested, RigExecOutputPacket *packet)
{
    if (!output.extractable) {
        packet->status = RigExecOutputStatus::Unsupported;
        packet->reason = output.reason;
        return;
    }
    const RigExecOpAdapterState &state = program.store.opAdapter;
    const RigExecOpValueState &value =
        state.values[size_t(output.producerValue)];
    const uint32_t producer =
        program.exportIndex.Producer(size_t(&output - program.exports.data()));
    const bool ran = producer < program.store.opExecution.ran.size() &&
                     program.store.opExecution.ran[producer] != 0;
    // The run's own facts decide what stands: a value the closure never named
    // may hold an earlier generation's bytes, and those are not this
    // request's answer.
    const bool reused = !ran && value.initialized;
    if (!ran && !reused) {
        packet->status = RigExecOutputStatus::Pending;
        packet->reason = "the requested run did not materialize " +
                         output.handle.semanticKey;
        return;
    }
    // A handle that names a revision asks for that revision. The value moved
    // past it, so this run's bytes are a different answer.
    if (requested.valueRevision != 0 &&
        requested.valueRevision != value.revision) {
        packet->status = RigExecOutputStatus::Invalidated;
        packet->reason = output.handle.semanticKey + " stands at " +
                         std::to_string(value.revision) +
                         "; the request named " +
                         std::to_string(requested.valueRevision);
        return;
    }
    std::string reason;
    if (!RrExtractExport(&program, output, &packet->payload, &reason)) {
        packet->payload = RigExecOutputPayload();
        packet->status = RigExecOutputStatus::Failed;
        packet->reason = reason;
        return;
    }
    std::string mismatch;
    if (!RigExecPayloadMatchesHandle(output.handle, packet->payload,
                                     &mismatch)) {
        packet->payload = RigExecOutputPayload();
        packet->status = RigExecOutputStatus::Failed;
        packet->reason = output.handle.semanticKey + " " + mismatch;
        return;
    }
    packet->handle = output.handle;
    packet->handle.valueRevision = value.revision;
    packet->status = RigExecOutputStatus::Ready;
}

} // namespace

bool
RrRequestOutputs(RrProgram *program,
                 const std::vector<RigExecOutputHandle> &requested,
                 const RigExecOutputTime &at,
                 const RigExecOutputGeneration &captured,
                 std::vector<RigExecOutputPacket> *packets,
                 size_t *requestedOperations, size_t *executedOperations,
                 size_t *clustersRun, std::string *error)
{
    const auto fail = [&](const std::string &why) {
        if (error)
            *error = why;
        return false;
    };
    if (!packets)
        return fail("no output packet list");
    *packets = {};
    if (!RrBuildExportIndex(program, error))
        return false;
    if (program->exports.empty())
        return fail("the file names no outputs to request");
    if (requested.empty())
        return fail("requested run names no output");

    // Every handle resolves against this reader's own catalog before anything
    // runs: a handle naming another rig instance or a stale compile epoch, or
    // an address the file does not export, is answered here rather than
    // dropped, so a partial demand never builds a partial mask.
    const bool staleGeneration =
        captured.programIdentity != 0 &&
        captured.programIdentity != program->compileEpoch;
    std::vector<RigExecOutputPacket> answers;
    answers.reserve(requested.size());
    std::vector<int64_t> bound(requested.size(), -1);
    for (size_t i = 0; i < requested.size(); ++i) {
        const RigExecOutputHandle &handle = requested[i];
        RigExecOutputPacket packet;
        packet.handle = handle;
        packet.time = at;
        packet.programIdentity = program->compileEpoch;
        packet.inputRevision = captured.inputRevision;
        packet.previewRevision = captured.previewRevision;
        packet.historyRevision = captured.historyRevision;
        size_t index = 0;
        if (!program->exportIndex.Find(handle, &index)) {
            std::string reason;
            const RigExecOutputStatus status =
                RrResolveOutput(program, handle, &reason);
            packet.status = status;
            packet.reason = std::move(reason);
        } else if (staleGeneration) {
            packet.status = RigExecOutputStatus::Invalidated;
            packet.reason = "the captured generation names program " +
                            std::to_string(captured.programIdentity) +
                            "; this catalog is epoch " +
                            std::to_string(program->compileEpoch);
        } else {
            bound[i] = int64_t(index);
        }
        answers.push_back(std::move(packet));
    }

    std::vector<size_t> slots;
    for (int64_t index : bound)
        if (index >= 0 &&
            std::find(slots.begin(), slots.end(), size_t(index)) == slots.end())
            slots.push_back(size_t(index));
    const RigExecDemandClosure closure = program->exportIndex.Union(slots);
    if (closure.ops.empty()) {
        // Nothing runnable was asked for; every answer above already names
        // why, and no operation of this program runs.
        *packets = std::move(answers);
        return true;
    }

    std::string why;
    if (!RrBeginRequestedRun(program, closure, &why))
        return fail(why);
    // A throw between here and the join owes the restriction lifted too.
    struct RequestedRunReset {
        RrProgram *program;
        ~RequestedRunReset()
        {
            RrEndRequestedRun(program);
        }
    } reset{program};

    RrStore &store = program->store;
    store.runTrace.clear();
    RrInputsApplyTouched(program);
    // A requested run publishes nothing: the prologues run for the storage the
    // extraction reads, and their diagnostics go nowhere.
    RrProloguePose(program);
    std::vector<std::string> diagnostics;
    const bool prepared = RrPrologueGeometry(program, &diagnostics, &why);
    const bool ran = prepared && RrRunSteps(program, false, &why);
    RrShareLatticeBinds(program);
    size_t requestedCount = 0, executed = 0, clusters = 0;
    RrRequestedRunCounts(program, &requestedCount, &executed, &clusters);
    if (requestedOperations)
        *requestedOperations = requestedCount;
    if (executedOperations)
        *executedOperations = executed;
    if (clustersRun)
        *clustersRun = clusters;
    if (!ran) {
        for (RigExecOutputPacket &packet : answers)
            if (packet.status == RigExecOutputStatus::Pending) {
                packet.status = RigExecOutputStatus::Failed;
                packet.reason = why;
            }
        *packets = std::move(answers);
        return fail("requested execution did not complete: " + why);
    }
    for (size_t i = 0; i < answers.size(); ++i) {
        if (bound[i] < 0)
            continue;
        _Answer(*program, program->exports[size_t(bound[i])], requested[i],
                &answers[i]);
    }
    *packets = std::move(answers);
    return true;
}

} // namespace rigExec
