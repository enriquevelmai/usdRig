#ifndef RIGEXEC_BAKED_OP_GRAPH_H

#define RIGEXEC_BAKED_OP_GRAPH_H

#include "bakedProgramImpl.h"

namespace rigExec {

struct RigExecBakedOpIdentityRemap;
bool
RigExecBakedEffectiveMemo(const RigExecBakedProgramImpl &, uint32_t,
                          std::string *, std::vector<uint32_t> *,
                          std::vector<std::pair<uint32_t, uint32_t>> *,
                          const RigExecBakedOpIdentityRemap *remap = nullptr,
                          bool contentLeaves = false);
void RigExecBakedAdoptSkinOpState(RigExecBakedProgramImpl *,
                                  const RigExecBakedProgramImpl &);
bool RigExecBakedCompileOpGraph(RigExecBakedProgramImpl *, std::string *);

bool RigExecBakedTranslateOpGraph(RigExecBakedProgramImpl *, std::string *);

bool RigExecBakedExecuteOpGraph(RigExecBakedProgramImpl *, UsdTimeCode, bool);

/// Installs \p requested as this program's run restriction: the closure is
/// checked against the canonical graph, then becomes the executor's active
/// mask. False, having changed nothing, when the graph is not compiled, the
/// closure is empty or out of range, or the closure is not predecessor
/// closed. The caller runs the graph, reads the facts while the mask stands,
/// then ends the run.
bool RigExecBakedBeginRequestedRun(RigExecBakedProgramImpl *,
                                   const RigExecDemandClosure &, std::string *);

/// Clears the restriction and the run facts it carries. The debt it left for
/// the next run stands: that is what a later request or whole run rebuilds.
void RigExecBakedEndRequestedRun(RigExecBakedProgramImpl *);

/// The facts of the requested run whose mask still stands, read after the
/// join: which bodies ran, and per value the closure writes whether this run
/// produced it or clean retained storage stood.
void RigExecBakedFillRequestedRunReport(const RigExecBakedProgramImpl &,
                                        RigExecRequestedRunReport *);

} // namespace rigExec

#endif
