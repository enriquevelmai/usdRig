// Discoverable evaluated outputs and their exact bindings into one baked
// program. Semantic addresses follow RigExecValueAddress: target, public
// computation and read phase. Dense storage and value ids are compile-local;
// handles carry the caller's rig identity and compile epoch.
#ifndef RIGEXEC_OUTPUT_CATALOG_H
#define RIGEXEC_OUTPUT_CATALOG_H

#include "rigExecGraph/outputDemand.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecBakedProgram;

/// The identity of one immutable compilation. A baked program mints it at
/// Build, every frozen clone of that program inherits it, and a catalog
/// discovered from the program holds it, so a consumer can tell that a catalog
/// names THIS compilation before reading either. Nothing else can produce a
/// matching handle: only the program and its own clones share the owner.
struct RigExecProgramAffinity {};

/// The USD-free spelling of a RigExecValueAddress plus its exact payload type.
struct RigExecOutputDescriptor {
    std::string target;
    std::string publicComputation;
    std::string phase;
    std::string type;
    std::string semanticKey;
};

enum class RigExecOutputKind {
    Scalar,
    Transform,
    Points,
};

/// Retained storage an extractor reads after the graph joins. Scratch packet,
/// candidate, chunk and staging slots deliberately have no enumerator here.
enum class RigExecOutputStorage {
    PropertyValue,
    ProviderMatrix,
    FrameMatrix,
    PointVersion,
};

/// One discoverable output bound to a canonical graph value and retained
/// storage. object and version are interpreted by storage: property chain and
/// property version, provider slot and phase, frame-record index, or geometry
/// chain and point version.
struct RigExecOutputBinding {
    RigExecOutputDescriptor descriptor;
    RigExecOutputHandle handle;
    RigExecOutputKind kind = RigExecOutputKind::Scalar;
    RigExecOutputStorage storage = RigExecOutputStorage::PropertyValue;
    RigExecValueId producerValue = kRigExecNoOutputValue;
    uint32_t object = 0;
    uint32_t version = 0;
    bool requiresValidity = true;
    bool requiresEffects = false;
    RigExecOutputStatus status = RigExecOutputStatus::Pending;
    std::string reason;
};

/// Immutable catalog for one compiled program epoch. Resolve distinguishes a
/// stale epoch from an unsupported semantic address before execution starts.
class RigExecOutputCatalog {
public:
    /// The compilation this catalog was discovered from. Equal to that
    /// program's affinity and to that of any of its frozen clones, and to no
    /// other compilation, however alike their operation graphs look.
    const std::shared_ptr<const RigExecProgramAffinity> &
    GetProgramAffinity() const
    {
        return programAffinity_;
    }
    const std::string &GetRigIdentity() const
    {
        return rigIdentity_;
    }
    uint64_t GetCompileEpoch() const
    {
        return compileEpoch_;
    }
    const std::vector<RigExecOutputBinding> &GetBindings() const
    {
        return bindings_;
    }
    const RigExecDemandEdges &GetRequirements() const
    {
        return requirements_;
    }
    const RigExecOutputDemandIndex &GetDemandIndex() const
    {
        return demandIndex_;
    }

    RigExecOutputStatus Resolve(const RigExecOutputHandle &handle,
                                const RigExecOutputBinding **binding,
                                std::string *reason = nullptr) const;

private:
    friend bool RigExecDiscoverOutputs(const RigExecBakedProgram &,
                                       const std::string &, uint64_t,
                                       RigExecOutputCatalog *, std::string *);

    std::string rigIdentity_;
    uint64_t compileEpoch_ = 0;
    std::shared_ptr<const RigExecProgramAffinity> programAffinity_;
    std::vector<RigExecOutputBinding> bindings_;
    RigExecDemandEdges requirements_;
    RigExecOutputDemandIndex demandIndex_;
};

/// Discovers every explicitly retained scalar, transform and complete CPU
/// points output of program. The caller supplies the rig-instance identity and
/// monotonically changing compile epoch owned by its lifecycle. The function
/// runs on the compile owner thread, reads compiled tables only, and leaves
/// output unchanged on failure.
bool RigExecDiscoverOutputs(const RigExecBakedProgram &program,
                            const std::string &rigIdentity,
                            uint64_t compileEpoch, RigExecOutputCatalog *output,
                            std::string *error = nullptr);

} // namespace rigExec

#endif // RIGEXEC_OUTPUT_CATALOG_H
