# Runtime pose evaluation

The runtime reads binary program tables without linking USD. Pose evaluation
is divided by responsibility:

| File | Responsibility |
| --- | --- |
| `pose.cpp` | Scratch allocation, input prologue, ladder composition |
| `poseSteps.cpp` | Step dispatch, commit deltas, propagation |
| `poseSolvers.cpp` | FK, two-bone IK, spline IK, ribbon, twist solvers |
| `poseConstraints.cpp` | Constraint sources, constraint kernels, single-chain IK |
| `poseInterpolation.cpp` | Radial-basis interpolation from solved wire tables |
| `poseMath.cpp` | Frame decomposition and blending shared by pose operations |

Registered mover modules supply USD-free typed definitions through
`rigExecBinary/moverDefinition.h`. `rigexec_add_mover(BUILTIN ...)` generates
the runtime catalog and direct factory references; `Open` binds matching
kernels before execution. Standalone hosts pass additional module definitions
to the `Open` overload. The USD adapter is a separate build input. See
[registered modules](../../docs/concepts/external-movers.md#build-a-mover-repository).

`poseInternal.h` contains private scratch types and phase interfaces. Keep
kernel implementations in their `.cpp` files and retain the baked path's
floating-point operation order and precision. Interpolation consumes the
already-solved weights; regularization and matrix inversion belong to baking.

The runtime pose and binary round-trip tests compare these results against the
baked evaluator. Run the repository's build helper to build and run CTest.

`geometry.cpp` evaluates geometry revisions, including the quasistatic Wrinkle
mover, with the same pure kernels as the USD evaluator. Wrinkle supports cloth
and surface-strut constraints, point pins, tangent-plane collisions, and its
final displacement blend. Every evaluation starts from the incoming geometry;
playback does not depend on frame history. Wrinkle uses revision opcode 15;
earlier readers reject files containing that opcode. Existing revision ordinals
and binary records retain their meaning.

Delta Mush, lattice and surface revisions also read the settings format 21
adds, through their movers' path reads and leaf sites like every other mover
input: smoothing and frame transport, smoothing weights, explicit edges,
smoothing-only output and a computation space; regular-grid lattices with
their interpolation, origin, spacing, strength, mask and cage and target
spaces; surface snap modes, offset, mask, explicit triangles and surface and
target spaces. Their frame providers are the revision's influences. Default
settings run the legacy kernels; a format-20 file holds none of them and
plays the legacy deformers.

Format 21 also adds affine frame expressions (copy, mapped, skin influence,
armature parent, bone and constraint frames) to the provider program. The
runtime runs them through `rigExecMath/affineFrameKernel.h`, the definition
native evaluation instantiates over Gf, over its own math mirror
(`affineMath.h`).

Plugin movers use revision opcode 16 and the file's `external_movers` table. The
runtime stores their bytes and calls the kernel a host installs with
`SetExternalKernel` (see `rigExecBinary/external.h` and
[External mover plugins](../../docs/concepts/external-movers.md)). A type with
no kernel passes its points through, with a warning in every `Execute`. A
kernel that sets `applyWithProviders` also receives the binding's transform
and influence matrices and the chain's base points as playback evaluated
them; the layered skin, surface binding and patch movers play this way.

Playback runs the steps in index order, source steps first, and never reads
their edges. `Open` therefore refuses a file whose step graph that order does
not satisfy: a predecessor at or after its step, predecessor and successor
lists that disagree, or a source step that depends on a step outside the
source pass. It also refuses a read of a slot that no step running before
the reader writes, except in the domains a run fills before any step (avars,
property-chain results, chain bases and solver points), and a cluster graph
that is not an acyclic quotient of the step graph. The retired `Snapshots`
domain and `SnapshotFinals` step kind are refused outright, ahead of every
rule about what a step of a given kind reads or writes. The rules live in
`rigExecBinary/stepGraph.h`, which the FlatBuffer validator shares.

Array inputs (`int[]`, `float[]`, `double[]`, `float2[]` and `float3[]`
attributes the file lists) are set with `SetInputArray` and read back with
`GetInputArrayAt`. The reader copies the elements; a set keeps the default's
element count. An authored set reaches every read of the attribute, the
Default-time ones included. `SetSampledInputArrayAt` takes a stage's own
value at a sampled time, of any count: only the reads at the evaluation time
take it, and each reader judges the count as the evaluators do. Topology is
the exception: a skin's joint indices, a mesh's face counts and indices, a
curve's order and knots and a sparse blend shape's offsets and point indices
are compiled rig state, which a reader never recompiles, so a sampled set of
another count is refused with the reason. A fixed skin
layout's arrays, a chain's base points and admitted weight-oracle points
read through such inputs. Structural painted arrays and excluded point reads
keep private storage slots, inaccessible to the public input APIs; `ResetInput` returns each to the
file's value, and a skin layout to the one `Open` expanded unless the layout
standing equals it by value, which stays, as the evaluator's layout op keeps
it. The input sampler samples the animated array inputs the file reads at
the evaluation time.

## Requested outputs

Format 32 adds the file's `export_catalog`: the outputs the compiled program
exports explicitly, each one a semantic address (target, public computation
and read phase), the payload layout it answers with, and the retained storage
behind it — a property chain's version, a provider slot's base or final
matrix, a frame record, or one point version of a chain. `Open` validates that
table (distinct addresses in ascending order, payload/kind/storage agreement,
object and version bounds, and a producer value whose domain matches the
storage the entry names) and binds each entry to this reader's own retained
storage. The types and rules these calls speak are declared once, in
`rigExecGraph/outputDemand.h`; see
[output subscriptions](../../docs/specs/output-subscriptions.md).

| Call | What it answers |
| --- | --- |
| `GetOutputCount` / `GetOutputAt` | The file's own list, as it wrote it, each entry with its status and the file's reason when it cannot be produced. |
| `FindOutput(semanticKey, index)` | This reader's index of an address a host learned from another program, so it can mint the handle this reader answers. |
| `GetOutputRigIdentity` / `GetOutputCompileEpoch` | The identity and epoch this reader's handles carry. `Open` mints the epoch from a process counter: a file never carries the live identity its exporter held, so a handle from another evaluator, or from another reader of the same file, resolves `Invalidated`. |
| `ResolveOutput(handle, reason)` | What an address means to this reader before anything runs: `Pending` for an output it can answer, `Unsupported` for an address it does not export or cannot produce, `Invalidated` for another rig instance or compile epoch. |
| `ExecuteRequested(requested, at, captured, packets, counts, error)` | Run exactly the union of the named closures over this reader's store, publish nothing, and answer each handle from the storage that run left. `RigExecRuntimeRequestedCounts` reports the operations the closure named, how many ran, and how many clusters the scheduler closed. One packet per handle, in request order, even when the call returns false because the request named no output or the run did not complete. |

- **The inputs say the frame.** The runtime does not track time: set the
  inputs for the frame being asked about first, as playback does.
- **A GPU-only point chain stays `Unsupported`.** The file recorded the reason,
  and the runtime reports it rather than producing those points on the CPU.
- **An older bake names nothing.** A file older than format 32, or a program
  assembled in memory, exposes no outputs and refuses a request with a reason
  instead of resolving an address it never recorded.
- **A restricted run trusts nothing the store never produced.** Inside its
  closure, an operation whose writes were never published is seeded whatever
  its inputs say, and if the run before it was restricted too, the closure is
  recomputed rather than read from what that partial run left behind. What the
  closure omits keeps its debt for the next run, and a whole `Execute` clears
  that restricted state.
- **A stale handle is reported, not answered.** A handle naming a revision it
  no longer stands at, or a capture naming another program, answers
  `Invalidated` with why; a handle this file does not name at all answers
  `Unsupported` and never enters the closure.
