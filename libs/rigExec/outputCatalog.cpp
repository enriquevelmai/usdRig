#include "outputCatalog.h"

#include "bakedProgram.h"
#include "bakedProgramImpl.h"

#include "rigExecSchema/schemaNames.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <algorithm>
#include <map>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

using Domain = RigExecBakedSlotDomain;

std::string
PathText(const SdfPath &path)
{
    return path.GetString();
}

RigExecValueId
FindValue(const RigExecBakedProgramImpl &program, Domain domain, uint32_t slot)
{
    const uint32_t domainValue = uint32_t(domain);
    const auto found = std::lower_bound(
        program.opAdapter.values.begin(), program.opAdapter.values.end(),
        std::make_pair(domainValue, slot),
        [](const RigExecOpValueState &value,
           const std::pair<uint32_t, uint32_t> &key) {
            return std::make_pair(value.domain, value.slot) < key;
        });
    if (found == program.opAdapter.values.end() ||
        found->domain != domainValue || found->slot != slot) {
        return kRigExecNoOutputValue;
    }
    return RigExecValueId(found - program.opAdapter.values.begin());
}

uint32_t
RevisionId(const RigExecBakedProgramImpl &program, size_t chain,
           size_t revision)
{
    for (size_t i = 0; i < program.revisionIndex.size(); ++i) {
        if (program.revisionIndex[i] ==
            std::make_pair(int(chain), int(revision))) {
            return uint32_t(i);
        }
    }
    return uint32_t(program.revisionIndex.size());
}

RigExecOutputDescriptor
Descriptor(const std::string &target, const std::string &computation,
           const std::string &phase, const std::string &type)
{
    RigExecOutputDescriptor descriptor;
    descriptor.target = target;
    descriptor.publicComputation = computation;
    descriptor.phase = phase;
    descriptor.type = type;
    descriptor.semanticKey =
        RigExecOutputSemanticKey(target, computation, phase);
    return descriptor;
}

RigExecOutputBinding
Binding(const std::string &rigIdentity, uint64_t compileEpoch,
        RigExecOutputDescriptor descriptor, RigExecOutputKind kind,
        RigExecOutputStorage storage, RigExecValueId producerValue,
        uint32_t object, uint32_t version,
        RigExecOutputStatus status = RigExecOutputStatus::Pending,
        const std::string &reason = {})
{
    RigExecOutputBinding binding;
    binding.descriptor = std::move(descriptor);
    binding.handle.rigIdentity = rigIdentity;
    binding.handle.compileEpoch = compileEpoch;
    binding.handle.semanticKey = binding.descriptor.semanticKey;
    binding.handle.type = binding.descriptor.type;
    binding.handle.readPhase = binding.descriptor.phase;
    binding.kind = kind;
    binding.storage = storage;
    binding.producerValue = producerValue;
    binding.object = object;
    binding.version = version;
    binding.status = status;
    binding.reason = reason;
    return binding;
}

bool
AddBinding(RigExecOutputBinding binding,
           std::map<std::string, size_t> *semanticKeys,
           std::vector<RigExecOutputBinding> *bindings, std::string *error)
{
    // A semantic address with no graph value is reported unsupported rather
    // than answered from the stage: the demand index refuses to index a
    // pending export whose producer is absent.
    if (binding.status == RigExecOutputStatus::Pending &&
        binding.producerValue == kRigExecNoOutputValue) {
        binding.status = RigExecOutputStatus::Unsupported;
        binding.reason = "no canonical producer retained for " +
                         binding.descriptor.target + " at phase " +
                         binding.descriptor.phase;
    }
    const auto inserted =
        semanticKeys->emplace(binding.descriptor.semanticKey, bindings->size());
    if (!inserted.second) {
        if (error) {
            *error = "duplicate output semantic address: " +
                     binding.descriptor.target + " " + binding.descriptor.phase;
        }
        return false;
    }
    bindings->push_back(std::move(binding));
    return true;
}

void
BuildRequirements(const RigExecBakedProgramImpl &program,
                  RigExecDemandEdges *requirements)
{
    const size_t count = program.opGraph.ops.size();
    requirements->validity.assign(count, {});
    requirements->effects.assign(count, {});
    requirements->history.assign(count, {});

    std::vector<char> hasSkipEffect(count, 0);
    for (uint32_t op : program.opAdapter.skipEffects) {
        if (op < count)
            hasSkipEffect[op] = 1;
    }
    for (uint32_t op = 0; op < count; ++op) {
        for (uint32_t predecessor : program.opGraph.ops[op].predecessors) {
            if (hasSkipEffect[predecessor])
                requirements->effects[op].push_back(predecessor);
        }
    }
    for (const auto &window : program.windowedOps) {
        if (window.op >= count)
            continue;
        for (uint32_t inner : window.inner) {
            if (inner < count)
                requirements->history[window.op].push_back(inner);
        }
        std::sort(requirements->history[window.op].begin(),
                  requirements->history[window.op].end());
        requirements->history[window.op].erase(
            std::unique(requirements->history[window.op].begin(),
                        requirements->history[window.op].end()),
            requirements->history[window.op].end());
    }
}

bool
AddPropertyOutputs(const RigExecBakedProgramImpl &program,
                   const std::string &rigIdentity, uint64_t compileEpoch,
                   std::map<std::string, size_t> *semanticKeys,
                   std::vector<RigExecOutputBinding> *bindings,
                   std::string *error)
{
    for (size_t c = 0; c < program.propertyChains.size(); ++c) {
        const RigExecBakedPropertyChain &chain = program.propertyChains[c];
        if (chain.valueType != SdfValueTypeNames->Float &&
            chain.valueType != SdfValueTypeNames->Double) {
            continue;
        }
        const std::string target = PathText(chain.target);
        const size_t versions = chain.revisions.size() + 1;
        for (size_t version = 0; version < versions; ++version) {
            const std::string phase =
                version == 0 ? "base"
                             : PathText(chain.revisions[version - 1].mover);
            const uint32_t slot = chain.versionBase + uint32_t(version);
            if (!AddBinding(
                    Binding(rigIdentity, compileEpoch,
                            Descriptor(target, {}, phase,
                                       RigExecPayloadText::scalar),
                            RigExecOutputKind::Scalar,
                            RigExecOutputStorage::PropertyValue,
                            FindValue(program, Domain::PropertyResult, slot),
                            uint32_t(c), uint32_t(version)),
                    semanticKeys, bindings, error)) {
                return false;
            }
        }
        const uint32_t finalVersion = uint32_t(chain.revisions.size());
        if (!AddBinding(Binding(rigIdentity, compileEpoch,
                                Descriptor(target, {}, "final",
                                           RigExecPayloadText::scalar),
                                RigExecOutputKind::Scalar,
                                RigExecOutputStorage::PropertyValue,
                                FindValue(program, Domain::PropertyResult,
                                          chain.versionBase + finalVersion),
                                uint32_t(c), finalVersion),
                        semanticKeys, bindings, error)) {
            return false;
        }
    }
    return true;
}

bool
AddTransformOutputs(const RigExecBakedProgramImpl &program,
                    const std::string &rigIdentity, uint64_t compileEpoch,
                    std::map<std::string, size_t> *semanticKeys,
                    std::vector<RigExecOutputBinding> *bindings,
                    std::string *error)
{
    const std::string computation = SchemaComputationText::computeMatrix;
    for (size_t slot = 0; slot < program.paths.size(); ++slot) {
        const std::string target = PathText(program.paths[slot]);
        if (slot < program.needBase.size() && program.needBase[slot]) {
            if (!AddBinding(
                    Binding(
                        rigIdentity, compileEpoch,
                        Descriptor(target, computation, "base",
                                   RigExecPayloadText::matrix4x4Row),
                        RigExecOutputKind::Transform,
                        RigExecOutputStorage::ProviderMatrix,
                        FindValue(program, Domain::BaseMatrix, uint32_t(slot)),
                        uint32_t(slot), 0),
                    semanticKeys, bindings, error)) {
                return false;
            }
        }
        if (slot < program.needFinal.size() && program.needFinal[slot]) {
            if (!AddBinding(
                    Binding(
                        rigIdentity, compileEpoch,
                        Descriptor(target, computation, "final",
                                   RigExecPayloadText::matrix4x4Row),
                        RigExecOutputKind::Transform,
                        RigExecOutputStorage::ProviderMatrix,
                        FindValue(program, Domain::FinalMatrix, uint32_t(slot)),
                        uint32_t(slot), 0),
                    semanticKeys, bindings, error)) {
                return false;
            }
        }
    }
    for (size_t record = 0; record < program.frameRecords.size(); ++record) {
        const RigExecBakedFrameRecord &frame = program.frameRecords[record];
        if (frame.slot < 0 || size_t(frame.slot) >= program.paths.size())
            continue;
        if (!AddBinding(
                Binding(
                    rigIdentity, compileEpoch,
                    Descriptor(PathText(program.paths[size_t(frame.slot)]),
                               computation, PathText(frame.mover),
                               RigExecPayloadText::matrix4x4Row),
                    RigExecOutputKind::Transform,
                    RigExecOutputStorage::FrameMatrix,
                    FindValue(program, Domain::FrameMatrix, uint32_t(record)),
                    uint32_t(record), uint32_t(record + 1)),
                semanticKeys, bindings, error)) {
            return false;
        }
    }
    return true;
}

bool
AddPointOutputs(const RigExecBakedProgramImpl &program,
                const std::string &rigIdentity, uint64_t compileEpoch,
                std::map<std::string, size_t> *semanticKeys,
                std::vector<RigExecOutputBinding> *bindings, std::string *error)
{
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const auto &chain = program.chains[c];
        if (chain.target.GetNameToken() != UsdGeomTokens->points)
            continue;
        const std::string target = PathText(chain.target);
        if (!AddBinding(
                Binding(
                    rigIdentity, compileEpoch,
                    Descriptor(target, {}, "base", RigExecPayloadText::point3),
                    RigExecOutputKind::Points,
                    RigExecOutputStorage::PointVersion,
                    FindValue(program, Domain::ChainBase, uint32_t(c)),
                    uint32_t(c), 0),
                semanticKeys, bindings, error)) {
            return false;
        }
        const RigExecOutputStatus status =
            chain.gpu ? RigExecOutputStatus::Unsupported
                      : RigExecOutputStatus::Pending;
        const std::string reason =
            chain.gpu ? "GPU-only points export has no CPU payload" : "";
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const uint32_t revision = RevisionId(program, c, r);
            if (revision >= program.revisionIndex.size()) {
                if (error)
                    *error = "geometry output has no canonical revision";
                return false;
            }
            const RigExecValueId producer =
                chain.gpu ? kRigExecNoOutputValue
                          : FindValue(program, Domain::RevisionDone, revision);
            if (!AddBinding(
                    Binding(rigIdentity, compileEpoch,
                            Descriptor(target, {},
                                       PathText(chain.revisions[r].moverPath),
                                       RigExecPayloadText::point3),
                            RigExecOutputKind::Points,
                            RigExecOutputStorage::PointVersion, producer,
                            uint32_t(c), uint32_t(r + 1), status, reason),
                    semanticKeys, bindings, error)) {
                return false;
            }
        }
        const RigExecValueId producer =
            chain.gpu ? kRigExecNoOutputValue
                      : FindValue(program, Domain::ChainPoints, uint32_t(c));
        if (!AddBinding(Binding(rigIdentity, compileEpoch,
                                Descriptor(target, {}, "final",
                                           RigExecPayloadText::point3),
                                RigExecOutputKind::Points,
                                RigExecOutputStorage::PointVersion, producer,
                                uint32_t(c), uint32_t(chain.revisions.size()),
                                status, reason),
                        semanticKeys, bindings, error)) {
            return false;
        }
    }
    return true;
}

} // namespace

std::string
RigExecOutputSemanticKey(const std::string &target,
                         const std::string &publicComputation,
                         const std::string &phase)
{
    return std::to_string(target.size()) + ":" + target +
           std::to_string(publicComputation.size()) + ":" + publicComputation +
           std::to_string(phase.size()) + ":" + phase;
}

RigExecOutputStatus
RigExecOutputCatalog::Resolve(const RigExecOutputHandle &handle,
                              const RigExecOutputBinding **binding,
                              std::string *reason) const
{
    if (binding)
        *binding = nullptr;
    const auto finish = [&](RigExecOutputStatus status,
                            const std::string &message) {
        if (reason)
            *reason = message;
        return status;
    };
    if (handle.rigIdentity != rigIdentity_)
        return finish(RigExecOutputStatus::Invalidated,
                      "output belongs to another rig instance");
    if (handle.compileEpoch != compileEpoch_)
        return finish(RigExecOutputStatus::Invalidated,
                      "output compile epoch is stale");
    for (const RigExecOutputBinding &candidate : bindings_) {
        if (!candidate.handle.SameTarget(handle))
            continue;
        if (binding)
            *binding = &candidate;
        return finish(candidate.status, candidate.reason);
    }
    return finish(RigExecOutputStatus::Unsupported,
                  "output semantic address is not exported");
}

namespace {

// Every export names a payload layout the demand contract can read, binds it to
// retained storage (never a scratch packet, candidate, chunk or staging slot),
// and states a kind that agrees with that layout.
bool
BindingIsSound(const RigExecBakedProgramImpl &program,
               const RigExecOutputBinding &binding, std::string *error)
{
    RigExecPayloadKind payload = RigExecPayloadKind::Unknown;
    const bool known = RigExecPayloadKindForType(binding.handle.type, &payload);
    const RigExecOutputKind expected =
        payload == RigExecPayloadKind::Scalar ? RigExecOutputKind::Scalar
        : payload == RigExecPayloadKind::Matrix4x4Row
            ? RigExecOutputKind::Transform
        : payload == RigExecPayloadKind::Point3 ? RigExecOutputKind::Points
                                                : RigExecOutputKind::Scalar;
    if (!known || payload == RigExecPayloadKind::Unknown ||
        expected != binding.kind) {
        if (error)
            *error = "output " + binding.descriptor.target + " at phase " +
                     binding.descriptor.phase +
                     " has no payload layout for type " + binding.handle.type;
        return false;
    }
    if (binding.producerValue == kRigExecNoOutputValue)
        return true;
    if (size_t(binding.producerValue) >= program.opAdapter.values.size()) {
        if (error)
            *error = "output " + binding.descriptor.target + " at phase " +
                     binding.descriptor.phase +
                     " names a value the compiled graph does not hold";
        return false;
    }
    const uint32_t domain =
        program.opAdapter.values[size_t(binding.producerValue)].domain;
    // RevisionDone is a revision fuse's publication record: it is the graph
    // value that names the chain's point version that fuse publishes.
    const bool retained = domain == uint32_t(Domain::PropertyResult) ||
                          domain == uint32_t(Domain::BaseMatrix) ||
                          domain == uint32_t(Domain::FinalMatrix) ||
                          domain == uint32_t(Domain::FrameMatrix) ||
                          domain == uint32_t(Domain::ChainBase) ||
                          domain == uint32_t(Domain::ChainPoints) ||
                          domain == uint32_t(Domain::RevisionDone);
    if (!retained) {
        if (error)
            *error = "output " + binding.descriptor.target + " at phase " +
                     binding.descriptor.phase + " binds scratch storage";
        return false;
    }
    return true;
}

} // namespace

bool
RigExecDiscoverOutputs(const RigExecBakedProgram &program,
                       const std::string &rigIdentity, uint64_t compileEpoch,
                       RigExecOutputCatalog *output, std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    if (!output)
        return fail("output catalog destination is null");
    if (rigIdentity.empty())
        return fail("output catalog rig identity is empty");

    const RigExecBakedProgramImpl &compiled = program.GetStepGraph();
    if (!compiled.opAdapter.compiled)
        return fail("baked program has no canonical operation graph");

    RigExecOutputCatalog result;
    result.rigIdentity_ = rigIdentity;
    result.compileEpoch_ = compileEpoch;
    std::map<std::string, size_t> semanticKeys;
    if (!AddPropertyOutputs(compiled, rigIdentity, compileEpoch, &semanticKeys,
                            &result.bindings_, error) ||
        !AddTransformOutputs(compiled, rigIdentity, compileEpoch, &semanticKeys,
                             &result.bindings_, error) ||
        !AddPointOutputs(compiled, rigIdentity, compileEpoch, &semanticKeys,
                         &result.bindings_, error)) {
        return false;
    }
    std::sort(result.bindings_.begin(), result.bindings_.end(),
              [](const RigExecOutputBinding &a, const RigExecOutputBinding &b) {
                  return a.descriptor.semanticKey < b.descriptor.semanticKey;
              });
    for (const RigExecOutputBinding &binding : result.bindings_) {
        std::string reason;
        if (!BindingIsSound(compiled, binding, &reason))
            return fail(reason);
    }

    BuildRequirements(compiled, &result.requirements_);
    std::vector<RigExecOutputDemandEntry> entries;
    entries.reserve(result.bindings_.size());
    for (const RigExecOutputBinding &binding : result.bindings_)
        entries.push_back({binding.handle, binding.producerValue});
    std::string indexError;
    if (!result.demandIndex_.Build(compiled.opGraph, result.requirements_,
                                   entries, &indexError)) {
        return fail("output demand index: " + indexError);
    }
    for (size_t i = 0; i < result.bindings_.size(); ++i) {
        const bool hasProducer = result.demandIndex_.ProducerStatus(i) !=
                                 RigExecOutputStatus::Unsupported;
        if (result.bindings_[i].status == RigExecOutputStatus::Pending &&
            !hasProducer) {
            return fail("exported output has no canonical producer: " +
                        result.bindings_[i].descriptor.target + " " +
                        result.bindings_[i].descriptor.phase);
        }
        size_t slot = 0;
        if (!result.demandIndex_.Find(result.bindings_[i].handle, &slot))
            continue;
        const uint32_t producer = result.demandIndex_.Producer(slot);
        result.bindings_[i].requiresValidity =
            result.bindings_[i].producerValue != kRigExecNoOutputValue;
        result.bindings_[i].requiresEffects =
            producer != kRigExecNoOp &&
            !result.requirements_.effects[producer].empty();
    }
    if (error)
        error->clear();
    *output = std::move(result);
    return true;
}

} // namespace rigExec
