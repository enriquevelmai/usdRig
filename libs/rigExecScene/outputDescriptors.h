// Owner-thread resolution of authored RigExecValueAddress values into the
// USD-free descriptors used by the output catalog. Resolution reads stage
// identity and type metadata only; it authors no opinions.
#ifndef RIGEXEC_SCENE_OUTPUT_DESCRIPTORS_H
#define RIGEXEC_SCENE_OUTPUT_DESCRIPTORS_H

#include "rigExec/outputCatalog.h"
#include "rigExec/tapSet.h"

#include "pxr/usd/usd/stage.h"

#include <string>

namespace rigExec {

/// Resolves an authored property or public prim computation without assuming
/// a scope, parent name, sibling layout or nesting depth. Supported public
/// exports are scalar properties, matrix transforms and point arrays. The
/// descriptor's type is the payload layout spelling from RigExecPayloadText
/// ("double", "matrix4d", "point3d[]"), not the authored USD type name.
bool RigExecResolveOutputDescriptor(const UsdStageRefPtr &stage,
                                    const RigExecValueAddress &address,
                                    RigExecOutputDescriptor *descriptor,
                                    std::string *error = nullptr);

} // namespace rigExec

#endif // RIGEXEC_SCENE_OUTPUT_DESCRIPTORS_H
