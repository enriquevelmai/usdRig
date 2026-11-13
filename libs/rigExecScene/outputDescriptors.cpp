#include "outputDescriptors.h"

#include "rigExecSchema/schemaNames.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

// An authored attribute's USD type is not its payload: the demand contract
// declares one layout per kind, and a handle's type carries that spelling.
bool
SupportedPropertyPayload(const SdfValueTypeName &type, std::string *payload)
{
    if (type == SdfValueTypeNames->Float || type == SdfValueTypeNames->Double) {
        *payload = RigExecPayloadText::scalar;
        return true;
    }
    if (type == SdfValueTypeNames->Matrix4d) {
        *payload = RigExecPayloadText::matrix4x4Row;
        return true;
    }
    if (type == SdfValueTypeNames->Point3fArray ||
        type == SdfValueTypeNames->Point3dArray) {
        *payload = RigExecPayloadText::point3;
        return true;
    }
    return false;
}

bool
ValidPhase(const TfToken &phase)
{
    if (phase.IsEmpty())
        return false;
    const std::string text = phase.GetString();
    if (text == "base" || text == "preceding" || text == "final")
        return true;
    return SdfPath(text).IsAbsoluteRootOrPrimPath();
}

} // namespace

bool
RigExecResolveOutputDescriptor(const UsdStageRefPtr &stage,
                               const RigExecValueAddress &address,
                               RigExecOutputDescriptor *descriptor,
                               std::string *error)
{
    const auto fail = [&](const std::string &message) {
        if (error)
            *error = message;
        return false;
    };
    if (!stage)
        return fail("output descriptor stage is null");
    if (!descriptor)
        return fail("output descriptor destination is null");
    if (!address.target.IsAbsolutePath())
        return fail("output target is not an absolute path");
    if (!ValidPhase(address.phase))
        return fail("output read phase is not base, preceding, final, or an "
                    "absolute prim path");

    RigExecOutputDescriptor result;
    result.target = address.target.GetString();
    result.publicComputation = address.publicComputation.GetString();
    result.phase = address.phase.GetString();

    if (address.target.IsPropertyPath()) {
        if (!address.publicComputation.IsEmpty())
            return fail("a property output cannot name a public computation");
        const UsdAttribute attribute =
            stage->GetAttributeAtPath(address.target);
        if (!attribute)
            return fail("output property does not exist");
        const SdfValueTypeName type = attribute.GetTypeName();
        if (!SupportedPropertyPayload(type, &result.type))
            return fail("output property type has no payload layout");
    } else if (address.target.IsPrimPath()) {
        if (address.publicComputation.GetString() !=
            SchemaComputationText::computeMatrix) {
            return fail("public computation is not an exported computation");
        }
        if (!stage->GetPrimAtPath(address.target))
            return fail("output prim does not exist");
        result.type = RigExecPayloadText::matrix4x4Row;
    } else {
        return fail("output target is neither a prim nor a property");
    }

    result.semanticKey = RigExecOutputSemanticKey(
        result.target, result.publicComputation, result.phase);
    if (error)
        error->clear();
    *descriptor = std::move(result);
    return true;
}

} // namespace rigExec
