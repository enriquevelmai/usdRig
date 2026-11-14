#include "rigExecSchema/schemaTokens.h"
// RigExecFrameCache, Stream A: the (compiledRigSignature, controlDigest) store.
// The key, entry, and stats types keep the semantics Stream 0 asserted, so
// those tests stand unchanged below. The stub half of the Stream 0 file is
// replaced here by the store's own contract, one rule per test group in the
// testRigExecStaticInputCache style:
//   * KEYING. A pose is a function of its two digests and nothing else:
//     either half misses, time is not key material, and a hit serves the
//     published pose bit-identically.
//   * DIGEST. RigExecControlStateDigest covers the sampled sources plus the
//     interactive overrides: a control change misses, an unrelated edit
//     hits, enqueue order never matters, and floating point folds bitwise.
//     RigExecRefusalControlDigest is the D7 time-plus-serial fallback for
//     rigs with no baked program: the frame moves it, an edit moves it at
//     every frame, a drag moves it, and it never equals a sampled digest
//     under the same compiled rig. Both fold the frame's upstream values as a
//     block of their own, which folds nothing when there are none.
//   * LRU. A per-rig byte cap with least-recently-used eviction, accounted
//     in RigExecFrameCachePoseBytes; oversized entries are dropped, never
//     partially stored.
//   * THREAD. Lookup runs concurrently with background Publish: the stress
//     below hammers both and holds the counters to account, and the cache
//     is still functional afterwards.
//   * DROP. Corrupt input is declined, never served: invalid poses,
//     oversized entries, and null out-params all miss.
#include "rigExec/frameCache.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"

// These headers only name the baked program's implementation and the
// frozen snapshot, so editing either rebuilds just the sources that read
// them.
#if defined(RIGEXEC_BAKED_PROGRAM_IMPL_H) || defined(RIGEXEC_FROZEN_PROGRAM_H)
#error "a frame-cache or frozen-context header includes a program definition"
#endif

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/vt/types.h"

#include <atomic>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

namespace {

// A valid pose whose bytes do not move with the tag: the tag rides a point
// coordinate, so same-shape poses are same-bytes poses and the LRU tests can
// size caps in whole entries.
RigExecRigPose
MakePose(double tag, size_t joints)
{
    RigExecRigPose pose;
    pose.valid = true;
    for (size_t i = 0; i < joints; ++i) {
        RigExecPointFrame frame;
        frame.points[0] = GfVec3d(tag, double(i), 0.0);
        pose.jointFramesFinal[SdfPath("/Joint" + std::to_string(i))] = frame;
    }
    return pose;
}

bool
PosesMatch(const RigExecRigPose &a, const RigExecRigPose &b)
{
    return a.valid == b.valid && a.jointFramesBase == b.jointFramesBase &&
           a.jointFramesFinal == b.jointFramesFinal &&
           a.jointMatricesFinal == b.jointMatricesFinal &&
           a.controlFrames == b.controlFrames &&
           a.providerXforms == b.providerXforms &&
           a.providerBaseXforms == b.providerBaseXforms &&
           a.solverFrames == b.solverFrames &&
           a.movedProperties == b.movedProperties &&
           a.diagnostics == b.diagnostics;
}

RigExecFrameInputs
MakeInputs(std::initializer_list<std::pair<const char *, double>> samples)
{
    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(3.0);
    for (const auto &s : samples) {
        inputs.Add(SdfPath(s.first), VtValue(s.second));
    }
    return inputs;
}

RigExecValueOverride
MakeOverride(const char *prim, double value)
{
    RigExecValueOverride o;
    o.prim = SdfPath(prim);
    o.computation = SchemaComputation::computePointFrame;
    o.value = VtValue(value);
    return o;
}

// A held type VtValue cannot hash and the digest has no fold for: frames
// carrying one must bypass the cache.
struct Unhashable {
    int x = 0;
    bool operator==(const Unhashable &o) const
    {
        return x == o.x;
    }
};

// Two digests, both significant: keys differing in either half are different
// keys, and identical halves are identical keys.
void
TestKeyEqualitySeesBothDigests()
{
    const RigExecFrameCacheKey base{7, 11};
    const RigExecFrameCacheKey same{7, 11};
    const RigExecFrameCacheKey otherCompiledRig{8, 11};
    const RigExecFrameCacheKey otherControl{7, 12};
    CHECK(base == same);
    CHECK(!(base != same));
    CHECK(base != otherCompiledRig);
    CHECK(base != otherControl);
    CHECK(!(base == otherCompiledRig));
}

// Ordered containers of keys stay usable: strict-weak ordering over the
// compiled rig half first, the control half second.
void
TestKeyOrderingIsCompiledRigThenControl()
{
    const RigExecFrameCacheKey a{1, 9};
    const RigExecFrameCacheKey b{2, 0};
    const RigExecFrameCacheKey c{1, 10};
    CHECK(a < b);
    CHECK(!(b < a));
    CHECK(a < c);
    CHECK(!(c < a));
    CHECK(!(a < a));
}

// Equal keys hash equal, so a hashed store cannot lose an entry it holds.
// (Unequal keys hashing unequal is not asserted: a collision is allowed, a
// lost entry is not.)
void
TestEqualKeysHashEqual()
{
    const RigExecFrameCacheKeyHash hash;
    CHECK(hash(RigExecFrameCacheKey{3, 5}) == hash(RigExecFrameCacheKey{3, 5}));
}

// Time rides the entry, never the key: the same key names the same inputs at
// whatever frame they were evaluated at, and the entry remembers which frame
// that was.
void
TestTimeRidesTheEntry()
{
    RigExecFrameCacheEntry entry;
    entry.time = UsdTimeCode(12.0);
    CHECK(entry.time == UsdTimeCode(12.0));
    CHECK(entry.bytes == 0);
}

// An empty cache misses, and the miss is counted. The pose out-param is
// untouched -- a miss must never publish a pose it did not read.
void
TestAnEmptyCacheMisses()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{1, 2};
    RigExecRigPose pose;
    pose.valid = true;
    CHECK(!cache.Lookup(key, &pose));
    CHECK(pose.valid);
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 0);
    CHECK(stats.misses == 1);
    CHECK(stats.published == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
}

// A null out-param misses rather than faults, and the miss is counted like
// any other.
void
TestLookupWithNullPoseMisses()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{1, 2};
    CHECK(cache.Publish(key, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(!cache.Lookup(key, nullptr));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 0);
    CHECK(stats.misses == 1);
    CHECK(stats.entryCount == 1);
}

// Publish then lookup: the hit serves the published pose bit-identically,
// and the counters say one publish, one hit.
void
TestPublishThenLookupHits()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{4, 9};
    const RigExecRigPose published = MakePose(2.5, 3);
    CHECK(cache.Publish(key, UsdTimeCode(6.0), published));
    RigExecRigPose found;
    CHECK(cache.Lookup(key, &found));
    CHECK(PosesMatch(published, found));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 1);
    CHECK(stats.misses == 0);
    CHECK(stats.published == 1);
    CHECK(stats.evictions == 0);
    CHECK(stats.entryCount == 1);
    CHECK(stats.bytes == RigExecFrameCachePoseBytes(published));
}

// Either digest half misses: the compiled rig the program was built for and the
// control state are both key material, and neighbors of a held key are cold.
void
TestEitherDigestHalfMisses()
{
    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(!cache.Lookup({9, 2}, &pose));
    CHECK(!cache.Lookup({1, 8}, &pose));
    CHECK(cache.Lookup({1, 2}, &pose));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 1);
    CHECK(stats.misses == 2);
    CHECK(stats.entryCount == 1);
}

// Time is not key material: publishing the same key at a second frame
// replaces the pose in place -- one entry, latest pose served.
void
TestTimeIsNotKeyMaterial()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{1, 2};
    CHECK(cache.Publish(key, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish(key, UsdTimeCode(2.0), MakePose(2.0, 1)));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 2);
    CHECK(stats.entryCount == 1);
    RigExecRigPose found;
    CHECK(cache.Lookup(key, &found));
    CHECK(PosesMatch(MakePose(2.0, 1), found));
}

// An invalid pose is not a pose: Publish declines it and the store is
// unchanged, so a fail-closed evaluation can never poison the cache.
void
TestPublishInvalidPoseDeclines()
{
    RigExecFrameCache cache;
    RigExecRigPose pose;
    pose.valid = false;
    CHECK(!cache.Publish({1, 2}, UsdTimeCode(1.0), pose));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
    RigExecRigPose found;
    CHECK(!cache.Lookup({1, 2}, &found));
}

// Evict drops the entry and counts it; a second evict of the same key finds
// nothing and counts nothing.
void
TestEvictDropsAndCounts()
{
    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Evict({1, 2}));
    RigExecRigPose pose;
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(!cache.Evict({1, 2}));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 1);
    CHECK(stats.evictions == 1);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
}

// The byte cap round-trips, and Clear drops every entry and zeroes the
// counters while leaving the cap -- the size the rig was given -- in place.
void
TestCapRoundTripsAndClearZeroesCounters()
{
    RigExecFrameCache cache;
    CHECK(cache.GetByteCap() == kRigExecFrameCacheDefaultByteCap);
    cache.SetByteCap(1024 * 1024);
    CHECK(cache.GetByteCap() == 1024 * 1024);

    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup({1, 2}, &pose));
    CHECK(!cache.Lookup({9, 9}, &pose));
    CHECK(cache.Stats().hits == 1);
    CHECK(cache.Stats().misses == 1);
    cache.Clear();
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 0);
    CHECK(stats.misses == 0);
    CHECK(stats.published == 0);
    CHECK(stats.evictions == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
    CHECK(cache.GetByteCap() == 1024 * 1024);
    CHECK(!cache.Lookup({1, 2}, &pose));
}

// Pose bytes are the struct shell plus every published map: an empty pose
// costs the shell, one joint costs a key plus a frame, and Publish accounts
// exactly what the function measures.
void
TestPoseBytesCountsMaps()
{
    RigExecRigPose empty;
    empty.valid = true;
    CHECK(RigExecFrameCachePoseBytes(empty) == sizeof(RigExecRigPose));

    const RigExecRigPose one = MakePose(1.0, 1);
    const size_t expect =
        sizeof(RigExecRigPose) + sizeof(SdfPath) + sizeof(RigExecPointFrame);
    CHECK(RigExecFrameCachePoseBytes(one) == expect);
    // The tag rides a coordinate, so same-shape poses are same-bytes poses.
    CHECK(RigExecFrameCachePoseBytes(MakePose(7.5, 1)) == expect);

    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), one));
    CHECK(cache.Stats().bytes == expect);
}

// The digest covers the value, not the sampling order: shuffled inputs name
// the same control state.
void
TestDigestIgnoresEnqueueOrder()
{
    const RigExecFrameInputs fwd =
        MakeInputs({{"/Rig/A", 1.0}, {"/Rig/B", 2.0}, {"/Rig/C", 3.0}});
    const RigExecFrameInputs rev =
        MakeInputs({{"/Rig/C", 3.0}, {"/Rig/B", 2.0}, {"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigest(fwd) == RigExecControlStateDigest(rev));
}

// A control change misses: any sampled value moving moves the digest --
// including +0.0 to -0.0, which hash equal under TfHash but need not
// evaluate equal.
void
TestDigestSeesValueChanges()
{
    const RigExecFrameInputs base = MakeInputs({{"/Rig/A", 1.0}});
    const RigExecFrameInputs moved = MakeInputs({{"/Rig/A", 1.5}});
    CHECK(RigExecControlStateDigest(base) != RigExecControlStateDigest(moved));

    const RigExecFrameInputs plusZero = MakeInputs({{"/Rig/A", 0.0}});
    const RigExecFrameInputs minusZero = MakeInputs({{"/Rig/A", -0.0}});
    CHECK(RigExecControlStateDigest(plusZero) !=
          RigExecControlStateDigest(minusZero));

    RigExecFrameInputs floats;
    floats.Add(SdfPath("/Rig/A"), VtValue(1.0f));
    RigExecFrameInputs doubles;
    doubles.Add(SdfPath("/Rig/A"), VtValue(1.0));
    CHECK(RigExecControlStateDigest(floats) !=
          RigExecControlStateDigest(doubles));
}

// The mixer chains non-commutatively: duplicated values do not cancel, and
// permuted values do not collide. An xor-only fold maps [W,W] to the seed
// for every W -- changing both duplicates would serve a stale pose -- and
// [A,B] to [B,A].
void
TestDigestSeesDuplicateValues()
{
    // The same double on two paths, changed on both: the digest moves.
    const RigExecFrameInputs before =
        MakeInputs({{"/Rig/A", 10.2}, {"/Rig/B", 10.2}});
    const RigExecFrameInputs after =
        MakeInputs({{"/Rig/A", 11.7}, {"/Rig/B", 11.7}});
    CHECK(RigExecControlStateDigest(before) !=
          RigExecControlStateDigest(after));
    // One of the pair moved: moves too.
    const RigExecFrameInputs half =
        MakeInputs({{"/Rig/A", 10.2}, {"/Rig/B", 11.7}});
    CHECK(RigExecControlStateDigest(before) != RigExecControlStateDigest(half));
    CHECK(RigExecControlStateDigest(after) != RigExecControlStateDigest(half));
    // Swapped values across paths: order matters.
    const RigExecFrameInputs ab =
        MakeInputs({{"/Rig/A", 1.0}, {"/Rig/B", 2.0}});
    const RigExecFrameInputs ba =
        MakeInputs({{"/Rig/A", 2.0}, {"/Rig/B", 1.0}});
    CHECK(RigExecControlStateDigest(ab) != RigExecControlStateDigest(ba));
    // Identical vectors still digest identically.
    const RigExecFrameInputs again =
        MakeInputs({{"/Rig/A", 10.2}, {"/Rig/B", 10.2}});
    CHECK(RigExecControlStateDigest(before) ==
          RigExecControlStateDigest(again));
}

// Paths and types are digest material: the same value at another path, or
// under another type, names another control state.
void
TestDigestSeesPathsAndTypes()
{
    const RigExecFrameInputs atA = MakeInputs({{"/Rig/A", 1.0}});
    const RigExecFrameInputs atB = MakeInputs({{"/Rig/B", 1.0}});
    CHECK(RigExecControlStateDigest(atA) != RigExecControlStateDigest(atB));

    RigExecFrameInputs ints;
    ints.Add(SdfPath("/Rig/A"), VtValue(1));
    CHECK(RigExecControlStateDigest(atA) != RigExecControlStateDigest(ints));

    RigExecFrameInputs tokens;
    tokens.Add(SdfPath("/Rig/A"), VtValue(TfToken("one")));
    RigExecFrameInputs strings;
    strings.Add(SdfPath("/Rig/A"), VtValue(std::string("one")));
    CHECK(RigExecControlStateDigest(tokens) !=
          RigExecControlStateDigest(strings));
}

// An unrelated edit hits: inputs the frame never sampled are not digest
// material, so the digest over the same vector is unchanged -- while a
// source that held no value digests distinctly from every valued source.
void
TestDigestHitsAcrossUnrelatedEdits()
{
    const RigExecFrameInputs base = MakeInputs({{"/Rig/A", 1.0}});
    const RigExecFrameInputs same = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigest(base) == RigExecControlStateDigest(same));

    RigExecFrameInputs valueless;
    valueless.Add(SdfPath("/Rig/A"), VtValue(), /*hasValue=*/false);
    CHECK(RigExecControlStateDigest(base) !=
          RigExecControlStateDigest(valueless));
    // And the valueless digest is stable: a stale value riding along cannot
    // move it.
    RigExecFrameInputs stale;
    stale.Add(SdfPath("/Rig/A"), VtValue(9.0), /*hasValue=*/false);
    CHECK(RigExecControlStateDigest(valueless) ==
          RigExecControlStateDigest(stale));
}

// The first sample at a path wins, matching FrameInputs::Find: a resampled
// path digests as its enqueue-time value, never a later one.
void
TestDigestFirstSampleWins()
{
    RigExecFrameInputs inputs;
    inputs.Add(SdfPath("/Rig/A"), VtValue(1.0));
    inputs.Add(SdfPath("/Rig/A"), VtValue(9.0));
    const RigExecFrameInputs first = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigest(inputs) ==
          RigExecControlStateDigest(first));
}

// Overrides are digest material -- they never reach the stage, so folding
// them in is what stops a drag serving a pre-drag pose -- while override
// order never matters and a repeated key keeps the last value, matching the
// evaluator's replace-not-join rule.
void
TestDigestSeesOverrides()
{
    const RigExecFrameInputs inputs = MakeInputs({{"/Rig/A", 1.0}});
    const std::vector<RigExecValueOverride> none;
    const std::vector<RigExecValueOverride> held{MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecControlStateDigest(inputs, none) !=
          RigExecControlStateDigest(inputs, held));

    const std::vector<RigExecValueOverride> moved{MakeOverride("/Rig/C", 3.0)};
    CHECK(RigExecControlStateDigest(inputs, held) !=
          RigExecControlStateDigest(inputs, moved));

    const std::vector<RigExecValueOverride> elsewhere{
        MakeOverride("/Rig/D", 2.0)};
    CHECK(RigExecControlStateDigest(inputs, held) !=
          RigExecControlStateDigest(inputs, elsewhere));

    const std::vector<RigExecValueOverride> fwd{MakeOverride("/Rig/C", 2.0),
                                                MakeOverride("/Rig/D", 3.0)};
    const std::vector<RigExecValueOverride> rev{MakeOverride("/Rig/D", 3.0),
                                                MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecControlStateDigest(inputs, fwd) ==
          RigExecControlStateDigest(inputs, rev));

    const std::vector<RigExecValueOverride> dup{MakeOverride("/Rig/C", 2.0),
                                                MakeOverride("/Rig/C", 5.0)};
    const std::vector<RigExecValueOverride> last{MakeOverride("/Rig/C", 5.0)};
    CHECK(RigExecControlStateDigest(inputs, dup) ==
          RigExecControlStateDigest(inputs, last));
}

// Time is digest-excluded: the same control state at another frame digests
// equal, which is what lets a scrub hit across times.
void
TestDigestIgnoresTime()
{
    RigExecFrameInputs early = MakeInputs({{"/Rig/A", 1.0}});
    early.time = UsdTimeCode(1.0);
    RigExecFrameInputs late = MakeInputs({{"/Rig/A", 1.0}});
    late.time = UsdTimeCode(48.0);
    CHECK(RigExecControlStateDigest(early) == RigExecControlStateDigest(late));
}

// Digestibility is the fail-closed gate: standard held types pass, and a
// frame carrying a value no hash can name must bypass the cache.
void
TestDigestibleFlagsUnhashableValues()
{
    const RigExecFrameInputs plain = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigestible(plain));

    RigExecFrameInputs empty;
    CHECK(RigExecControlStateDigestible(empty));

    RigExecFrameInputs held;
    held.Add(SdfPath("/Rig/A"), VtValue(1.0));
    held.Add(SdfPath("/Rig/B"), VtValue(Unhashable{3}));
    CHECK(!held.values.back().value.CanHash());
    CHECK(!RigExecControlStateDigestible(held));

    RigExecValueOverride o = MakeOverride("/Rig/C", 1.0);
    o.value = VtValue(Unhashable{4});
    CHECK(!RigExecControlStateDigestible(plain, {o}));
    CHECK(RigExecControlStateDigestible(plain, {MakeOverride("/Rig/C", 1.0)}));
}

// The recorded fold order and the static table's level-1s are a faster
// route to the plain digest, never another fold: a vector carrying both
// digests and classifies exactly as its copy without them, first-win
// duplicates, valueless and blocked samples, overrides, upstream values and
// the burst route included. A vector whose path sequence moved since the
// order was recorded, or a mark the table does not hold at its path, folds
// plainly; and a marked sample folds its entry's level-1, which is why the
// samplers serve entries verbatim.
void
TestRecordedOrderAndStaticTableFoldAsPlain()
{
    auto table = std::make_shared<RigExecFrozenStaticSamples>();
    const auto entry = [](RigExecFrozenStaticSamples *t, const char *path,
                          const VtValue &value, bool hasValue, bool blocked) {
        RigExecSampledInput sample;
        sample.path = SdfPath(path);
        sample.value = value;
        sample.hasValue = hasValue;
        sample.valueBlocked = blocked;
        sample.staticSample = int32_t(t->samples.size());
        t->level1.push_back(RigExecSampleDigest(sample));
        t->digestible.push_back(RigExecSampleDigestible(sample) ? 1 : 0);
        t->samples.push_back(sample);
    };
    entry(table.get(), "/Rig/B.rigExec:providerRaw:rest:tx", VtValue(0.5), true,
          false);
    entry(table.get(), "/Rig/A.rigExec:providerRaw:parent:space",
          VtValue(GfMatrix4d(2.0)), true, false);
    entry(table.get(), "/Rig/C.rigExec:providerRaw:avars:rotationOrder",
          VtValue(TfToken("zxy")), true, false);
    entry(table.get(), "/Rig/D.rigExec:providerRaw:rest:rz", VtValue(), false,
          true);

    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(3.0);
    inputs.Add(SdfPath("/Rig/Z.avars:tx"), VtValue(1.25));
    for (const RigExecSampledInput &sample : table->samples) {
        inputs.values.push_back(sample);
    }
    inputs.Add(SdfPath("/Rig/A.avars:ty"), VtValue(VtFloatArray{1.0f, -0.0f}));
    // A later sample at a served path: the served one wins.
    inputs.Add(SdfPath("/Rig/B.rigExec:providerRaw:rest:tx"), VtValue(9.0));
    inputs.Add(SdfPath("/Rig/E.inputs:weight"), VtValue(),
               /*hasValue=*/false);
    inputs.staticSamples = table;
    inputs.digestOrder = RigExecRecordFrameDigestOrder(inputs.values);
    CHECK(RigExecFrameDigestOrderMatches(inputs.digestOrder.get(),
                                         inputs.values));
    CHECK(!RigExecFrameDigestOrderMatches(nullptr, inputs.values));

    const auto plainOf = [](const RigExecFrameInputs &v) {
        RigExecFrameInputs plain = v;
        plain.staticSamples.reset();
        plain.digestOrder.reset();
        return plain;
    };
    const RigExecFrameInputs plain = plainOf(inputs);
    const uint64_t plainDigest = RigExecControlStateDigest(plain);
    const std::vector<RigExecValueOverride> held{MakeOverride("/Rig/Z", 4.0)};
    RigExecUpstreamValue up;
    up.path = SdfPath("/Rig/A.avars:tz");
    up.value = VtValue(2.0);
    CHECK(RigExecControlStateDigest(inputs) == plainDigest);
    CHECK(RigExecControlStateDigest(inputs, held) ==
          RigExecControlStateDigest(plain, held));
    CHECK(RigExecControlStateDigest(inputs, held, {up}) ==
          RigExecControlStateDigest(plain, held, {up}));
    CHECK(RigExecControlStateDigestible(inputs));
    CHECK(RigExecControlStateDigestible(plain));

    // The burst route: its first frame records its own order and the next
    // reuses it, with or without the sampler's.
    RigExecBurstSampleCache cache;
    cache.usable = true;
    CHECK(RigExecControlStateDigestWithBurstCache(inputs, {}, &cache) ==
          plainDigest);
    CHECK(RigExecControlStateDigestWithBurstCache(inputs, {}, &cache) ==
          plainDigest);
    RigExecFrameInputs unordered = inputs;
    unordered.digestOrder.reset();
    CHECK(RigExecControlStateDigestWithBurstCache(unordered, {}, &cache) ==
          plainDigest);

    // A fresh sample's value moves the digest through the recorded order.
    RigExecFrameInputs moved = inputs;
    moved.values[0].value = VtValue(1.5);
    CHECK(
        RigExecFrameDigestOrderMatches(moved.digestOrder.get(), moved.values));
    CHECK(RigExecControlStateDigest(moved) ==
          RigExecControlStateDigest(plainOf(moved)));
    CHECK(RigExecControlStateDigest(moved) != plainDigest);

    // Another path sequence than the recorded one folds plainly.
    RigExecFrameInputs grown = inputs;
    grown.Add(SdfPath("/Rig/F.inputs:x"), VtValue(2.0));
    CHECK(
        !RigExecFrameDigestOrderMatches(grown.digestOrder.get(), grown.values));
    CHECK(RigExecControlStateDigest(grown) ==
          RigExecControlStateDigest(plainOf(grown)));
    CHECK(RigExecControlStateDigest(grown) != plainDigest);

    // A mark naming an entry at another path is not that entry's.
    RigExecFrameInputs foreign = inputs;
    foreign.values[0].staticSample = 1;
    CHECK(RigExecControlStateDigest(foreign) == plainDigest);

    // A marked sample folds its entry's level-1, never its own bytes.
    auto skewed = std::make_shared<RigExecFrozenStaticSamples>(*table);
    skewed->level1[0] ^= 1;
    RigExecFrameInputs viaSkewed = inputs;
    viaSkewed.staticSamples = skewed;
    CHECK(RigExecControlStateDigest(viaSkewed) != plainDigest);

    // An unhashable entry is undigestible through the table, as plainly.
    auto opaque = std::make_shared<RigExecFrozenStaticSamples>(*table);
    entry(opaque.get(), "/Rig/G.rigExec:providerRaw:custom",
          VtValue(Unhashable{3}), true, false);
    CHECK(!opaque->digestible.back());
    RigExecFrameInputs undigestible = inputs;
    undigestible.staticSamples = opaque;
    undigestible.values.push_back(opaque->samples.back());
    CHECK(!RigExecControlStateDigestible(undigestible));
    CHECK(!RigExecControlStateDigestible(plainOf(undigestible)));
}

// The D7 fallback digest keys the frame plus the stage-edit serial plus the
// standing overrides: the same frame re-digests equal, another frame digests
// apart (Default most of all), an edit (a moved serial) digests apart at
// every frame, a drag always digests apart from the authored frame, override
// order never matters, and the domain tag keeps it from ever equaling a
// sampled digest under the same compiled rig.
void
TestRefusalDigestKeysTimeAndOverrides()
{
    const std::vector<RigExecValueOverride> none;
    const std::vector<RigExecValueOverride> held{MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode(3.0), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode::Default(), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 8, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, held));

    const std::vector<RigExecValueOverride> moved{MakeOverride("/Rig/C", 3.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, held) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, moved));

    const std::vector<RigExecValueOverride> fwd{MakeOverride("/Rig/C", 2.0),
                                                MakeOverride("/Rig/D", 3.0)};
    const std::vector<RigExecValueOverride> rev{MakeOverride("/Rig/D", 3.0),
                                                MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, fwd) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, rev));

    const std::vector<RigExecValueOverride> dup{MakeOverride("/Rig/C", 2.0),
                                                MakeOverride("/Rig/C", 5.0)};
    const std::vector<RigExecValueOverride> last{MakeOverride("/Rig/C", 5.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, dup) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, last));

    // The domain tag: a mode toggle crosses sampled and refusal digests
    // without moving the compiled rig, so the two must never agree.
    const RigExecFrameInputs inputs = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(3.0), 7, none) !=
          RigExecControlStateDigest(inputs));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(3.0), 7, none) !=
          RigExecControlStateDigest(inputs, held));

    // Digestibility is the same fail-closed gate: time always folds, so
    // only the overrides can carry an unhashable type.
    CHECK(RigExecRefusalControlDigestible(none));
    CHECK(RigExecRefusalControlDigestible(held));
    RigExecValueOverride o = MakeOverride("/Rig/C", 1.0);
    o.value = VtValue(Unhashable{5});
    CHECK(!o.value.CanHash());
    CHECK(!RigExecRefusalControlDigestible({o}));
}

RigExecUpstreamValue
MakeUpstream(const char *path, const VtValue &value)
{
    return RigExecUpstreamValue{SdfPath(path), value, 0};
}

// Upstream values are key material in every digest: an entry, a moved
// value, another path or another type digests apart, an empty list folds
// nothing, the order given never matters, and the forms without an explicit
// list fold the vector's own. An unhashable value bypasses the cache.
void
TestDigestSeesUpstream()
{
    const RigExecFrameInputs plain = MakeInputs({{"/Rig/A", 1.0}});
    const std::vector<RigExecValueOverride> none;
    const std::vector<RigExecUpstreamValue> noUpstream;
    CHECK(RigExecControlStateDigest(plain, none, noUpstream) ==
          RigExecControlStateDigest(plain, none));

    RigExecFrameInputs placed = plain;
    placed.upstream = {MakeUpstream("/Rig/U.avars:rz", VtValue(30.0))};
    CHECK(RigExecControlStateDigest(placed) !=
          RigExecControlStateDigest(plain));
    CHECK(RigExecControlStateDigest(placed) ==
          RigExecControlStateDigest(plain, none, placed.upstream));

    RigExecFrameInputs moved = plain;
    moved.upstream = {MakeUpstream("/Rig/U.avars:rz", VtValue(31.0))};
    CHECK(RigExecControlStateDigest(moved) !=
          RigExecControlStateDigest(placed));
    RigExecFrameInputs elsewhere = plain;
    elsewhere.upstream = {MakeUpstream("/Rig/U.avars:ry", VtValue(30.0))};
    CHECK(RigExecControlStateDigest(elsewhere) !=
          RigExecControlStateDigest(placed));
    RigExecFrameInputs retyped = plain;
    retyped.upstream = {MakeUpstream("/Rig/U.avars:rz", VtValue(30.0f))};
    CHECK(RigExecControlStateDigest(retyped) !=
          RigExecControlStateDigest(placed));

    const std::vector<RigExecUpstreamValue> fwd{
        MakeUpstream("/Rig/U.a", VtValue(1.0)),
        MakeUpstream("/Rig/U.b", VtValue(2.0))};
    const std::vector<RigExecUpstreamValue> rev{fwd[1], fwd[0]};
    CHECK(RigExecControlStateDigest(plain, none, fwd) ==
          RigExecControlStateDigest(plain, none, rev));

    // The constant fold and the burst form carry the block as well.
    CHECK(RigExecControlStateDigestWithConstants(plain, none, fwd, 9) ==
          RigExecFoldConstantDigest(RigExecControlStateDigest(plain, none, fwd),
                                    9));
    CHECK(RigExecControlStateDigestWithConstants(plain, none, fwd, 9) !=
          RigExecControlStateDigestWithConstants(plain, none, 9));
    CHECK(RigExecControlStateDigestWithBurstCache(placed, none, nullptr) ==
          RigExecControlStateDigest(placed));

    // The refusal digest: the dynamic walk reads upstream values too.
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none, noUpstream) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none,
                                      placed.upstream) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none));
    CHECK(
        RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none,
                                    placed.upstream) !=
        RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none, moved.upstream));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none,
                                      placed.upstream) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none,
                                      placed.upstream));

    // Digestibility: an unhashable upstream value is a bypass.
    CHECK(RigExecControlStateDigestible(placed));
    RigExecFrameInputs unhashable = plain;
    unhashable.upstream = {MakeUpstream("/Rig/U.x", VtValue(Unhashable{6}))};
    CHECK(!RigExecControlStateDigestible(unhashable));
    CHECK(RigExecRefusalControlDigestible(none, placed.upstream));
    CHECK(!RigExecRefusalControlDigestible(none, unhashable.upstream));
}

// An upstream array keys by its fold hash: RigExecUpstreamFoldHash folds
// the type and bytes once (never 0; 0 for a scalar), equal arrays hash
// equal, and the digest folds the hash, not the bytes, so an entry carrying
// another hash keys apart even over the same array. One handed over with
// no hash folds its bytes.
void
TestDigestFoldsUpstreamArrayHashes()
{
    const RigExecFrameInputs plain = MakeInputs({{"/Rig/A", 1.0}});
    const VtFloatArray weights{1.0f, 0.5f, 0.25f};
    const VtFloatArray same{1.0f, 0.5f, 0.25f};
    const VtFloatArray other{1.0f, 0.5f, 0.75f};
    CHECK(RigExecUpstreamFoldHash(VtValue(30.0)) == 0);
    const uint64_t hash = RigExecUpstreamFoldHash(VtValue(weights));
    CHECK(hash != 0);
    CHECK(RigExecUpstreamFoldHash(VtValue(same)) == hash);
    CHECK(RigExecUpstreamFoldHash(VtValue(other)) != hash);
    // Same bytes, another element type: another hash.
    CHECK(RigExecUpstreamFoldHash(VtValue(VtIntArray{1, 2, 3})) !=
          RigExecUpstreamFoldHash(VtValue(VtFloatArray{1.0f, 2.0f, 3.0f})));

    const auto with = [&plain](const VtValue &value, uint64_t foldHash) {
        RigExecFrameInputs inputs = plain;
        inputs.upstream = {RigExecUpstreamValue{
            SdfPath("/Rig/S.rigExec:jointWeights"), value, foldHash}};
        return RigExecControlStateDigest(inputs);
    };
    const uint64_t keyed = with(VtValue(weights), hash);
    CHECK(keyed != RigExecControlStateDigest(plain));
    CHECK(with(VtValue(same), hash) == keyed);
    CHECK(with(VtValue(other), RigExecUpstreamFoldHash(VtValue(other))) !=
          keyed);
    // The hash is what folds.
    CHECK(with(VtValue(weights), hash + 1) != keyed);
    // No hash: the bytes fold, distinctly from the hashed form.
    CHECK(with(VtValue(weights), 0) == with(VtValue(same), 0));
    CHECK(with(VtValue(weights), 0) != with(VtValue(other), 0));
    // RigExecUpstreamValuesOf hashes an array and leaves a scalar at 0.
    const std::vector<RigExecUpstreamValue> converted = RigExecUpstreamValuesOf(
        {RigExecValueOverride{SdfPath("/Rig/S"), TfToken(),
                              TfToken("rigExec:jointWeights"),
                              VtValue(weights)},
         RigExecValueOverride{SdfPath("/Rig/U"), TfToken(), TfToken("avars:rz"),
                              VtValue(30.0)}});
    CHECK(converted.size() == 2);
    CHECK(converted.size() == 2 && converted[0].foldHash == hash &&
          converted[1].foldHash == 0);
}

// The cap fits two entries: the third publish evicts the
// least-recently-used one, and the held bytes never exceed the cap.
void
TestLRUEvictsLeastRecentlyUsed()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(2 * entry);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    CHECK(cache.Stats().evictions == 0);
    CHECK(cache.Publish({1, 3}, UsdTimeCode(3.0), MakePose(3.0, 1)));

    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.evictions == 1);
    CHECK(stats.entryCount == 2);
    CHECK(stats.bytes == 2 * entry);
    CHECK(stats.bytes <= cache.GetByteCap());

    RigExecRigPose pose;
    CHECK(!cache.Lookup({1, 1}, &pose));
    CHECK(cache.Lookup({1, 2}, &pose));
    CHECK(PosesMatch(MakePose(2.0, 1), pose));
    CHECK(cache.Lookup({1, 3}, &pose));
}

// A lookup refreshes recency: the touched entry survives the next eviction
// and the untouched one -- published later but read never -- goes.
void
TestLookupRefreshesRecency()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(2 * entry);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup({1, 1}, &pose));
    CHECK(cache.Publish({1, 3}, UsdTimeCode(3.0), MakePose(3.0, 1)));

    CHECK(cache.Lookup({1, 1}, &pose));
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(cache.Lookup({1, 3}, &pose));
    CHECK(cache.Stats().entryCount == 2);
}

// A republish refreshes recency too: overwriting an entry makes it newest,
// so the next eviction takes its untouched sibling.
void
TestOverwriteRefreshesRecency()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(2 * entry);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    CHECK(cache.Publish({1, 1}, UsdTimeCode(3.0), MakePose(9.0, 1)));
    CHECK(cache.Publish({1, 3}, UsdTimeCode(4.0), MakePose(3.0, 1)));

    RigExecRigPose pose;
    CHECK(cache.Lookup({1, 1}, &pose));
    CHECK(PosesMatch(MakePose(9.0, 1), pose));
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(cache.Lookup({1, 3}, &pose));
}

// An entry larger than the whole cap is dropped rather than stored: the
// publish declines and the store is unchanged.
void
TestOversizedPublishDeclines()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(entry - 1);
    CHECK(!cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
}

// Reducing the cap evicts down to it immediately, least-recently-used
// first: the newest entry is the one left standing.
void
TestLoweringCapEvictsDown()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    CHECK(cache.Publish({1, 3}, UsdTimeCode(3.0), MakePose(3.0, 1)));
    cache.SetByteCap(entry);

    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.entryCount == 1);
    CHECK(stats.bytes == entry);
    CHECK(stats.evictions == 2);
    RigExecRigPose pose;
    CHECK(!cache.Lookup({1, 1}, &pose));
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(cache.Lookup({1, 3}, &pose));
}

// Lookup runs concurrently with background Publish: four threads hammer a
// sixteen-key cache five hundred times each, and every lookup is counted
// exactly once while the held bytes stay under the cap.
void
TestConcurrentLookupDuringPublish()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1024 * 1024);
    constexpr int kKeys = 16;
    for (int i = 0; i < kKeys; ++i) {
        const RigExecFrameCacheKey key{7, uint64_t(i)};
        CHECK(
            cache.Publish(key, UsdTimeCode(double(i)), MakePose(double(i), 1)));
    }

    constexpr int kThreads = 4;
    constexpr int kOps = 500;
    std::atomic<int> lookups{0};
    std::atomic<int> publishes{0};
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                const RigExecFrameCacheKey key{7, uint64_t((n + t) % kKeys)};
                if (n % 3 == 0) {
                    if (cache.Publish(key, UsdTimeCode(double(n)),
                                      MakePose(double(n), 1))) {
                        ++publishes;
                    }
                } else {
                    RigExecRigPose pose;
                    cache.Lookup(key, &pose);
                    ++lookups;
                }
            }
        });
    }
    for (auto &w : workers) {
        w.join();
    }

    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits + stats.misses == size_t(lookups.load()));
    CHECK(stats.published == size_t(kKeys) + size_t(publishes.load()));
    CHECK(stats.entryCount == size_t(kKeys));
    CHECK(stats.bytes <= cache.GetByteCap());
    CHECK(stats.evictions == 0);

    // Still functional afterwards: a fresh key round-trips.
    const RigExecFrameCacheKey fresh{7, 999};
    CHECK(cache.Publish(fresh, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup(fresh, &pose));
    CHECK(PosesMatch(MakePose(1.0, 1), pose));
}

// Concurrent publishers of one key converge: hundreds of overwrites leave a
// single entry, every publish counted, and the served pose is one of the
// published ones -- valid, never torn.
void
TestConcurrentPublishOfOneKey()
{
    RigExecFrameCache cache;
    constexpr int kThreads = 4;
    constexpr int kOps = 200;
    const RigExecFrameCacheKey key{3, 3};
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                CHECK(cache.Publish(key, UsdTimeCode(double(n)),
                                    MakePose(double(t), 1)));
            }
        });
    }
    for (auto &w : workers) {
        w.join();
    }
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == size_t(kThreads * kOps));
    CHECK(stats.entryCount == 1);
    RigExecRigPose pose;
    CHECK(cache.Lookup(key, &pose));
    CHECK(pose.valid);
    CHECK(pose.jointFramesFinal.size() == 1);
}

// Clear races background publishers without losing count: three threads
// publish and evict across many keys while a fourth clears repeatedly. An
// entry that lands in an already-cleared shard survives with its bytes
// still accounted, so evicting everything at the end brings both counters
// to exactly zero -- never below, where a wrapped byte count would make
// every later publish drain the store.
void
TestClearDuringConcurrentPublishKeepsAccountingHonest()
{
    RigExecFrameCache cache;
    constexpr int kThreads = 3;
    constexpr int kOps = 400;
    constexpr int kKeys = 64;
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                const RigExecFrameCacheKey key{5,
                                               uint64_t((n * 7 + t) % kKeys)};
                if (n % 5 == 4) {
                    cache.Evict(key);
                } else {
                    cache.Publish(key, UsdTimeCode(double(n)),
                                  MakePose(double(n), 1 + (n % 3)));
                }
            }
        });
    }
    workers.emplace_back([&]() {
        for (int n = 0; n < kOps / 4; ++n) {
            cache.Clear();
        }
    });
    for (auto &w : workers) {
        w.join();
    }

    // Whatever survived is counted, and only that: dropping every key one
    // by one (plus a final Clear for anything the loop's view missed) ends
    // at exactly zero bytes and zero entries.
    for (int i = 0; i < kKeys; ++i) {
        cache.Evict({5, uint64_t(i)});
    }
    cache.Clear();
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);

    // A wrapped counter would evict this immediately after publishing it.
    const RigExecFrameCacheKey fresh{5, 999};
    CHECK(cache.Publish(fresh, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup(fresh, &pose));
    CHECK(cache.Stats().entryCount == 1);
}

// ------------------------------------------------------------
// Coverage: partial records, generation keys, and lease accounting
// ------------------------------------------------------------

RigExecOutputHandle
MakeHandle(const std::string &key,
           const std::string &type = RigExecPayloadText::scalar)
{
    RigExecOutputHandle handle;
    handle.rigIdentity = "rigA";
    handle.compileEpoch = 1;
    handle.semanticKey = key;
    handle.type = type;
    handle.readPhase = "final";
    handle.valueRevision = 1;
    return handle;
}

RigExecOutputTime
MakeTime(double timeCode)
{
    RigExecOutputTime time;
    CHECK(RigExecOutputTime::TryTimeCode(timeCode, &time));
    return time;
}

RigExecOutputGeneration
MakeGeneration(uint64_t program, uint64_t input, uint64_t preview,
               uint64_t history)
{
    RigExecOutputGeneration generation;
    generation.programIdentity = program;
    generation.inputRevision = input;
    generation.previewRevision = preview;
    generation.historyRevision = history;
    return generation;
}

RigExecCoverageKey
MakeCoverageKey(uint64_t epoch, double time, const RigExecOutputGeneration &gen,
                uint64_t upstream = 0)
{
    RigExecCoverageKey key;
    key.rigIdentity = "rigA";
    key.compileEpoch = epoch;
    key.time = MakeTime(time);
    key.generation = gen;
    key.upstreamRevision = upstream;
    return key;
}

// Elements of one layout, with distinct bytes, so a released or shared buffer
// can be told apart from any other.
RigExecOutputPayload
MakePayload(RigExecPayloadKind kind, size_t elements)
{
    const size_t elementBytes = RigExecPayloadElementBytes(kind);
    std::vector<uint8_t> bytes(elements * elementBytes);
    for (size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = uint8_t((i * 31 + 7) & 0xff);
    }
    RigExecOutputPayload payload;
    payload.bytes =
        std::make_shared<const std::vector<uint8_t>>(std::move(bytes));
    payload.kind = kind;
    payload.elementCount = elements;
    return payload;
}

RigExecOutputPacket
MakePacket(const RigExecOutputHandle &handle, const RigExecOutputTime &time,
           const RigExecOutputGeneration &gen, RigExecPayloadKind kind,
           size_t elements)
{
    RigExecOutputPacket packet;
    packet.handle = handle;
    packet.time = time;
    packet.programIdentity = gen.programIdentity;
    packet.inputRevision = gen.inputRevision;
    packet.previewRevision = gen.previewRevision;
    packet.historyRevision = gen.historyRevision;
    packet.status = RigExecOutputStatus::Ready;
    packet.payload = MakePayload(kind, elements);
    return packet;
}

// Charges capture/workspace/payload against cache. The returned reservation is
// invalid when the cap refused it.
RigExecCoverageReservation
Reserve(RigExecFrameCache &cache, size_t capture, size_t workspace,
        size_t payload)
{
    RigExecCoverageReservation reservation;
    cache.TryReserveCoverage(
        RigExecCoverageBytes{capture, workspace, payload, 0}, &reservation);
    return reservation;
}

size_t
CoverageTotal(const RigExecFrameCache &cache)
{
    size_t total = 0;
    CHECK(cache.CoverageHeld().TryTotal(&total));
    return total;
}

// Every part of the key names a different run: epoch, frame, program, input,
// preview, history, and upstream revision each decide the record.
void
TestCoverageKeySeesEveryGenerationField()
{
    const RigExecOutputGeneration gen = MakeGeneration(1, 2, 3, 4);
    const RigExecCoverageKey base = MakeCoverageKey(1, 5.0, gen, 7);
    RigExecCoverageKeyHash hash;

    CHECK(base == MakeCoverageKey(1, 5.0, gen, 7));
    CHECK(hash(base) == hash(MakeCoverageKey(1, 5.0, gen, 7)));
    CHECK(base != MakeCoverageKey(2, 5.0, gen, 7));
    CHECK(base != MakeCoverageKey(1, 6.0, gen, 7));
    CHECK(base != MakeCoverageKey(1, 5.0, MakeGeneration(9, 2, 3, 4), 7));
    CHECK(base != MakeCoverageKey(1, 5.0, MakeGeneration(1, 2, 3, 5), 7));
    CHECK(base != MakeCoverageKey(1, 5.0, gen, 8));

    // The default frame is not frame 0.
    RigExecCoverageKey defaultFrame = base;
    defaultFrame.time = RigExecOutputTime();
    CHECK(base != defaultFrame);
    CHECK(hash(base) != hash(defaultFrame));

    // Ordering agrees with equality, so a sorted index can hold keys.
    CHECK(base < MakeCoverageKey(1, 5.0, MakeGeneration(2, 2, 3, 4), 7));
    CHECK(!(MakeCoverageKey(1, 5.0, MakeGeneration(2, 2, 3, 4), 7) < base));
}

// Admission refuses an over-budget request before anything is allocated, and a
// record with no outputs never lands.
void
TestCoverageAdmissionRefusesBeforeAllocating()
{
    RigExecFrameCache cache;
    cache.SetByteCap(100);

    RigExecCoverageReservation admitted = Reserve(cache, 0, 0, 80);
    CHECK(admitted.IsValid());
    CHECK(cache.CoverageHeld().payload == 80);
    CHECK(!Reserve(cache, 0, 0, 200).IsValid());
    // The refused charge left nothing behind: the admitted 80 is still there.
    CHECK(cache.CoverageHeld().capture == 0);
    CHECK(cache.CoverageHeld().workspace == 0);
    CHECK(cache.CoverageHeld().payload == 80);
    CHECK(cache.CoverageHeld().pinned == 0);

    CHECK(admitted.Release());
    CHECK(!admitted.IsValid());
    CHECK(CoverageTotal(cache) == 0);

    RigExecCoverageReservation refused = Reserve(cache, 0, 0, 200);
    CHECK(!refused.IsValid());
    CHECK(CoverageTotal(cache) == 0);

    const RigExecCoverageKey key =
        MakeCoverageKey(1, 1.0, MakeGeneration(1, 1, 0, 0));
    CHECK(!cache.PublishCoverage(key, {}, Reserve(cache, 0, 0, 100), false));
    CHECK(cache.CoverageRecordCount() == 0);
    CHECK(CoverageTotal(cache) == 0);
}

// The reservation is what a run was allowed to allocate; the record is charged
// for what exists. Surplus comes back, and a payload larger than the
// reservation is declined rather than stored over budget.
void
TestPublishChargesWhatExistsAndReturnsSurplus()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 3.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");

    RigExecCoverageReservation reservation = Reserve(cache, 100, 200, 400);
    CHECK(reservation.IsValid());
    CHECK(cache.CoverageHeld().capture == 100);
    CHECK(cache.CoverageHeld().workspace == 200);
    CHECK(cache.CoverageHeld().payload == 400);

    CHECK(cache.PublishCoverage(
        key,
        {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10)},
        std::move(reservation), true));
    CHECK(cache.CoverageHeld().capture == 100);
    CHECK(cache.CoverageHeld().workspace == 200);
    CHECK(cache.CoverageHeld().payload == 80);
    CHECK(cache.CoverageRecordCount() == 1);

    RigExecCoverageRecord record;
    CHECK(cache.LookupCoverage(key, &record));
    CHECK(record.bytes.capture == 100);
    CHECK(record.bytes.workspace == 200);
    CHECK(record.bytes.payload == 80);
    CHECK(record.arenaRetained);
    CHECK(record.leases == 0);

    CHECK(!cache.PublishCoverage(
        MakeCoverageKey(1, 4.0, gen),
        {MakePacket(handle, MakeTime(4.0), gen,
                    RigExecPayloadKind::Matrix4x4Row, 2)},
        Reserve(cache, 0, 0, 100), false));
    CHECK(cache.CoverageRecordCount() == 1);
    CHECK(cache.CoverageHeld().capture == 100);
    CHECK(cache.CoverageHeld().workspace == 200);
    CHECK(cache.CoverageHeld().payload == 80);
}

// Re-publishing a key retires the old record and its bytes, so one run is never
// counted twice.
void
TestCoveragePublishReplacingARecordBalancesBytes()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 2.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");

    CHECK(cache.PublishCoverage(
        key,
        {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10)},
        Reserve(cache, 50, 0, 80), false));
    CHECK(cache.CoverageHeld().capture == 50);
    CHECK(cache.CoverageHeld().payload == 80);

    CHECK(cache.PublishCoverage(
        key, {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 5)},
        Reserve(cache, 20, 0, 40), false));
    CHECK(cache.CoverageRecordCount() == 1);
    CHECK(cache.CoverageHeld().capture == 20);
    CHECK(cache.CoverageHeld().payload == 40);

    CHECK(cache.EvictCoverage(key));
    CHECK(cache.CoverageRecordCount() == 0);
    CHECK(cache.CoverageHeld().capture == 0);
    CHECK(cache.CoverageHeld().workspace == 0);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(cache.CoverageHeld().pinned == 0);
}

// A partial record never answers a complete-pose lookup, and publishing one
// leaves the pose store's counters and complete marker untouched.
void
TestPartialCoverageNeverSatisfiesPoseLookup()
{
    RigExecFrameCache cache;
    cache.SetByteCap(10000);
    const RigExecFrameCacheKey poseKey{1, 2};
    CHECK(cache.Publish(poseKey, UsdTimeCode(1.0), MakePose(1.0, 2)));
    CHECK(cache.Stats().entryCount == 1);

    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey covKey = MakeCoverageKey(1, 1.0, gen);
    CHECK(cache.PublishCoverage(
        covKey,
        {MakePacket(MakeHandle("joint.frame"), covKey.time, gen,
                    RigExecPayloadKind::Scalar, 4)},
        Reserve(cache, 64, 128, 32), true));

    RigExecRigPose pose;
    CHECK(cache.Lookup(poseKey, &pose));
    CHECK(cache.Stats().entryCount == 1);
    CHECK(cache.Stats().bytes == RigExecFrameCachePoseBytes(pose));
    CHECK(cache.Stats().entryCount == 1);

    CHECK(!cache.Lookup({1, 1}, &pose));
    RigExecCoverageRecord record;
    CHECK(!cache.LookupCoverage(MakeCoverageKey(1, 2.0, gen), &record));
    CHECK(cache.LookupCoverage(covKey, &record));
    CHECK(record.packets.size() == 1);
}

// Every part of the key decides the hit, and a packet answers only the exact
// handle, frame, generation, and layout it was captured from.
void
TestStaleGenerationMissesCoverage()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 2, 3);
    const RigExecCoverageKey key = MakeCoverageKey(1, 4.0, gen, 9);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    CHECK(cache.PublishCoverage(
        key,
        {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10)},
        Reserve(cache, 0, 0, 80), true));

    RigExecCoverageRecord record;
    CHECK(cache.LookupCoverage(key, &record));
    CHECK(!cache.LookupCoverage(
        MakeCoverageKey(1, 4.0, MakeGeneration(2, 1, 2, 3), 9), &record));
    CHECK(!cache.LookupCoverage(
        MakeCoverageKey(1, 4.0, MakeGeneration(1, 2, 2, 3), 9), &record));
    CHECK(!cache.LookupCoverage(
        MakeCoverageKey(1, 4.0, MakeGeneration(1, 1, 3, 3), 9), &record));
    CHECK(!cache.LookupCoverage(
        MakeCoverageKey(1, 4.0, MakeGeneration(1, 1, 2, 4), 9), &record));
    CHECK(!cache.LookupCoverage(MakeCoverageKey(1, 4.0, gen, 10), &record));
    CHECK(!cache.LookupCoverage(MakeCoverageKey(2, 4.0, gen, 9), &record));
    CHECK(!cache.LookupCoverage(MakeCoverageKey(1, 5.0, gen, 9), &record));

    CHECK(cache.CoverageAnswers(key, handle, nullptr));
    CHECK(!cache.CoverageAnswers(key, MakeHandle("other.frame"), nullptr));
    CHECK(!cache.CoverageAnswers(
        key, MakeHandle("joint.frame", RigExecPayloadText::point3), nullptr));
    RigExecOutputHandle stale = handle;
    stale.valueRevision = 2;
    CHECK(!cache.CoverageAnswers(key, stale, nullptr));

    RigExecOutputPacket packet;
    CHECK(cache.CoverageAnswers(key, handle, &packet));
    CHECK(packet.status == RigExecOutputStatus::Ready);
    CHECK(packet.payload.ByteCount() == 80);

    const std::vector<RigExecOutputHandle> asked{handle,
                                                 MakeHandle("other.frame")};
    std::vector<RigExecOutputPacket> answered;
    CHECK(cache.CoverageAnswerCount(key, asked, &answered) == 1);
    CHECK(answered.size() == 1);
}

// Payload reuse is not workspace-resume proof: retiring the arena releases the
// workspace charge and leaves every payload answerable and unchanged.
void
TestArenaRetirementKeepsPayloadsAnswerable()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    const RigExecOutputPacket packet =
        MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10);
    CHECK(cache.PublishCoverage(key, {packet}, Reserve(cache, 32, 512, 80),
                                true));
    CHECK(cache.CoverageArenaRetained(key));
    CHECK(cache.CoverageHeld().workspace == 512);

    CHECK(cache.RetireCoverageArena(key));
    CHECK(!cache.CoverageArenaRetained(key));
    CHECK(cache.CoverageHeld().workspace == 0);
    CHECK(cache.CoverageHeld().capture == 32);
    CHECK(cache.CoverageHeld().payload == 80);

    RigExecOutputPacket served;
    CHECK(cache.CoverageAnswers(key, handle, &served));
    CHECK(served.payload.bytes == packet.payload.bytes);
    CHECK(served.payload.ByteCount() == packet.payload.ByteCount());
    CHECK(cache.CoverageAnswers(key, handle, nullptr));

    CHECK(!cache.RetireCoverageArena(key));
    CHECK(cache.CoverageHeld().workspace == 0);
}

// Dropping a payload releases exactly its bytes; a record with no arena and no
// payloads left is dropped from the store.
void
TestPayloadReleaseBalancesAndRetiresTheRecord()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    const RigExecOutputHandle a = MakeHandle("joint.frame");
    const RigExecOutputHandle b =
        MakeHandle("joint.matrix", RigExecPayloadText::matrix4x4Row);
    CHECK(cache.PublishCoverage(
        key,
        {MakePacket(a, key.time, gen, RigExecPayloadKind::Scalar, 10),
         MakePacket(b, key.time, gen, RigExecPayloadKind::Matrix4x4Row, 1)},
        Reserve(cache, 16, 256, 208), true));
    CHECK(cache.CoverageHeld().payload == 80 + 128);

    CHECK(cache.ReleaseCoveragePayload(key, a));
    CHECK(cache.CoverageHeld().payload == 128);
    CHECK(!cache.CoverageAnswers(key, a, nullptr));
    CHECK(cache.CoverageAnswers(key, b, nullptr));

    CHECK(!cache.ReleaseCoveragePayload(key, MakeHandle("missing")));
    CHECK(cache.CoverageHeld().payload == 128);

    CHECK(cache.ReleaseCoveragePayload(key, b));
    // The payloads are gone but the arena is still held, so the record stays.
    CHECK(cache.CoverageArenaRetained(key));
    CHECK(cache.CoverageRecordCount() == 1);
    CHECK(cache.CoverageHeld().capture == 16);
    CHECK(cache.CoverageHeld().workspace == 256);
    CHECK(cache.CoverageHeld().payload == 0);

    // Retiring the last thing the record held retires the record itself.
    CHECK(cache.RetireCoverageArena(key));
    CHECK(cache.CoverageRecordCount() == 0);
    CHECK(cache.CoverageHeld().capture == 0);
    CHECK(cache.CoverageHeld().workspace == 0);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(cache.CoverageHeld().pinned == 0);
}

// A pin moves a payload's bytes from Payload to Pinned: the same bytes are never
// counted twice, and a record a lease is holding cannot be evicted.
void
TestPinHoldsBytesAndBlocksEviction()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    CHECK(cache.PublishCoverage(
        key,
        {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10)},
        Reserve(cache, 0, 64, 80), true));
    CHECK(cache.CoverageHeld().workspace == 64);
    CHECK(cache.CoverageHeld().payload == 80);
    CHECK(cache.CoverageHeld().pinned == 0);

    RigExecCoverageLease lease;
    CHECK(cache.PinCoverage(key, handle, &lease));
    CHECK(lease.IsValid());
    CHECK(lease.ByteCount() == 80);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(cache.CoverageHeld().pinned == 80);
    CHECK(cache.CoverageHeld().workspace == 64);

    CHECK(!cache.EvictCoverage(key));
    CHECK(cache.ClearCoverageUnpinned() == 0);
    CHECK(cache.CoverageAnswers(key, handle, nullptr));
    CHECK(!cache.PinCoverage(key, handle, &lease));

    RigExecCoverageRecord record;
    CHECK(cache.LookupCoverage(key, &record));
    CHECK(record.leases == 1);

    CHECK(lease.Release());
    CHECK(!lease.IsValid());
    CHECK(!lease.Release());
    CHECK(cache.CoverageHeld().pinned == 0);
    CHECK(cache.CoverageHeld().workspace == 64);
    CHECK(cache.EvictCoverage(key));
    CHECK(cache.CoverageRecordCount() == 0);
    CHECK(cache.CoverageHeld().workspace == 0);
}

// A lease outlives the record that produced it: the pinned bytes stay charged
// until the lease releases them, the record keeps no claim on them, and nothing
// else can serve that handle.
void
TestLeaseOutlivesTheRecord()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    CHECK(cache.PublishCoverage(
        key,
        {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10)},
        Reserve(cache, 0, 0, 80), false));

    RigExecCoverageLease lease;
    CHECK(cache.PinCoverage(key, handle, &lease));
    CHECK(!cache.RetireCoverageArena(key));
    // The record drops its reference to a pinned payload; the charge is the
    // lease's until the lease ends.
    CHECK(cache.ReleaseCoveragePayload(key, handle));
    CHECK(cache.CoverageHeld().pinned == 80);
    CHECK(cache.CoverageRecordCount() == 1);

    RigExecCoverageRecord record;
    CHECK(cache.LookupCoverage(key, &record));
    CHECK(!record.arenaRetained);
    CHECK(!cache.CoverageAnswers(key, handle, nullptr));

    CHECK(lease.IsValid());
    CHECK(lease.ByteCount() == 80);
    CHECK(lease.Packet().IsUsable());
    CHECK(lease.Packet().Answers(handle, key.time, key.generation));

    CHECK(lease.Release());
    CHECK(CoverageTotal(cache) == 0);
    CHECK(cache.CoverageRecordCount() == 0);
}

// Replacing a record under a live lease releases the old record's charges
// except the pinned bytes the lease owns: the account balances, the lease keeps
// its packet, and the new record is served and pinned on its own account.
void
TestReplacingARecordUnderALeaseBalancesBytes()
{
    RigExecFrameCache cache;
    cache.SetByteCap(2000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    const RigExecOutputPacket held =
        MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 10);
    CHECK(cache.PublishCoverage(key, {held}, Reserve(cache, 16, 64, 80), true));

    RigExecCoverageLease lease;
    CHECK(cache.PinCoverage(key, handle, &lease));
    CHECK(CoverageTotal(cache) == 160);

    const RigExecOutputPacket fresh =
        MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 5);
    CHECK(cache.PublishCoverage(key, {fresh}, Reserve(cache, 8, 32, 40), true));
    // The replaced record released its capture and workspace; only the lease's
    // pinned bytes survive it.
    CHECK(cache.CoverageHeld().capture == 8);
    CHECK(cache.CoverageHeld().workspace == 32);
    CHECK(cache.CoverageHeld().payload == 40);
    CHECK(cache.CoverageHeld().pinned == 80);
    CHECK(cache.CoverageRecordCount() == 1);

    RigExecOutputPacket served;
    CHECK(cache.CoverageAnswers(key, handle, &served));
    CHECK(served.payload.bytes == fresh.payload.bytes);
    CHECK(lease.IsValid());
    CHECK(lease.Packet().payload.bytes == held.payload.bytes);

    RigExecCoverageLease freshLease;
    CHECK(cache.PinCoverage(key, handle, &freshLease));
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(cache.CoverageHeld().pinned == 120);
    CHECK(!cache.EvictCoverage(key));

    CHECK(lease.Release());
    CHECK(cache.CoverageHeld().pinned == 40);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(!cache.EvictCoverage(key));
    CHECK(freshLease.Release());
    CHECK(cache.CoverageHeld().pinned == 0);
    CHECK(CoverageTotal(cache) == 80);
    CHECK(cache.EvictCoverage(key));
    CHECK(CoverageTotal(cache) == 0);
    CHECK(cache.CoverageRecordCount() == 0);
}

// Two records that captured the same buffer keep sharing it: dropping one
// record's reference neither changes the other's bytes nor its charge.
void
TestSharedPayloadStaysSharedWhenARecordDropsIt()
{
    RigExecFrameCache cache;
    cache.SetByteCap(2000);
    const RigExecOutputGeneration first = MakeGeneration(1, 1, 0, 0);
    const RigExecOutputGeneration second = MakeGeneration(2, 1, 0, 0);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    const RigExecCoverageKey keyA = MakeCoverageKey(1, 1.0, first);
    const RigExecCoverageKey keyB = MakeCoverageKey(1, 1.0, second);
    const RigExecOutputPayload buffer =
        MakePayload(RigExecPayloadKind::Scalar, 10);
    CHECK(buffer.bytes.use_count() == 1);

    RigExecOutputPacket packetA =
        MakePacket(handle, keyA.time, first, RigExecPayloadKind::Scalar, 10);
    RigExecOutputPacket packetB =
        MakePacket(handle, keyB.time, second, RigExecPayloadKind::Scalar, 10);
    packetA.payload = buffer;
    packetB.payload = buffer;

    CHECK(cache.PublishCoverage(keyA, {packetA}, Reserve(cache, 0, 0, 80),
                                false));
    CHECK(cache.PublishCoverage(keyB, {packetB}, Reserve(cache, 0, 0, 80),
                                false));
    CHECK(cache.CoverageHeld().payload == 160);

    RigExecOutputPacket servedA;
    RigExecOutputPacket servedB;
    CHECK(cache.CoverageAnswers(keyA, handle, &servedA));
    CHECK(cache.CoverageAnswers(keyB, handle, &servedB));
    // Two records, one buffer: unrelated storage stays shared.
    CHECK(servedA.payload.bytes == servedB.payload.bytes);

    CHECK(cache.EvictCoverage(keyA));
    CHECK(cache.CoverageHeld().payload == 80);
    CHECK(!cache.CoverageAnswers(keyA, handle, nullptr));
    CHECK(cache.CoverageAnswers(keyB, handle, &servedB));
    CHECK(servedB.payload.bytes == buffer.bytes);
    CHECK(servedB.payload.elementCount == 10);
    for (size_t i = 0; i < servedB.payload.bytes->size(); ++i) {
        CHECK((*servedB.payload.bytes)[i] == (*buffer.bytes)[i]);
    }
    CHECK(buffer.bytes.use_count() >= 2);

    // 80 of the 2000 cap is held, so one byte more than the room left is
    // refused.
    CHECK(!Reserve(cache, 0, 0, 1921).IsValid());
    CHECK(CoverageTotal(cache) == 80);
}

// A release is balanced or refused: an unbalanced release never wraps, and an
// uncommitted reservation gives its charge back when it dies.
void
TestReleaseIsBalancedOrDeclined()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    CHECK(
        cache.PublishCoverage(key,
                              {MakePacket(MakeHandle("joint.frame"), key.time,
                                          gen, RigExecPayloadKind::Scalar, 10)},
                              Reserve(cache, 50, 0, 80), false));

    RigExecCoverageBytes tooMuch;
    tooMuch.payload = 81;
    CHECK(!cache.ReleaseCoverage(tooMuch));
    CHECK(cache.CoverageHeld().payload == 80);

    RigExecCoverageBytes exact;
    exact.capture = 50;
    exact.payload = 80;
    CHECK(cache.ReleaseCoverage(exact));
    CHECK(cache.CoverageHeld().capture == 0);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(!cache.ReleaseCoverage(exact));

    RigExecCoverageReservation reservation = Reserve(cache, 20, 30, 40);
    CHECK(reservation.IsValid());
    CHECK(CoverageTotal(cache) == 90);
    CHECK(!reservation.Extend(RigExecCoverageDomain::Workspace, 1000));
    CHECK(cache.CoverageHeld().workspace == 30);
    CHECK(reservation.Extend(RigExecCoverageDomain::Workspace, 910));
    CHECK(cache.CoverageHeld().workspace == 940);
    CHECK(!reservation.Extend(RigExecCoverageDomain::Workspace, 1));
    CHECK(cache.CoverageHeld().workspace == 940);
    CHECK(reservation.Release());
    CHECK(!reservation.IsValid());
    CHECK(CoverageTotal(cache) == 0);
    CHECK(!reservation.Release());

    // A reservation left uncommitted releases itself when it dies.
    {
        RigExecCoverageReservation scoped = Reserve(cache, 10, 10, 10);
        CHECK(scoped.IsValid());
        CHECK(CoverageTotal(cache) == 30);
    }
    CHECK(CoverageTotal(cache) == 0);
}

// Coverage bytes count against the same cap as poses, and Clear drops every
// record while a live lease keeps its bytes charged and its packet served.
void
TestCoverageSharesTheCapAndClearSweepsIt()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1000);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    const size_t poseBytes = cache.Stats().bytes;
    CHECK(poseBytes > 0);
    CHECK(poseBytes < 1000);
    // One byte past the room the poses leave is refused.
    CHECK(!Reserve(cache, 0, 1000 - poseBytes + 1, 0).IsValid());
    CHECK(CoverageTotal(cache) == 0);
    CHECK(Reserve(cache, 0, 1000 - poseBytes, 0).IsValid());

    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecCoverageKey key = MakeCoverageKey(1, 1.0, gen);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");
    CHECK(cache.PublishCoverage(
        key, {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 1)},
        Reserve(cache, 0, 0, 8), true));
    CHECK(cache.Stats().bytes == poseBytes);
    CHECK(CoverageTotal(cache) == 8);

    RigExecCoverageLease lease;
    CHECK(cache.PinCoverage(key, handle, &lease));

    cache.Clear();
    CHECK(cache.Stats().entryCount == 0);
    CHECK(cache.CoverageRecordCount() == 0);
    CHECK(cache.CoverageHeld().capture == 0);
    CHECK(cache.CoverageHeld().workspace == 0);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(cache.CoverageHeld().pinned == 8);
    CHECK(lease.IsValid());
    CHECK(lease.Packet().Answers(handle, key.time, gen));
    CHECK(lease.Release());
    CHECK(CoverageTotal(cache) == 0);
    CHECK(cache.CoverageRecordCount() == 0);

    CHECK(cache.PublishCoverage(
        key, {MakePacket(handle, key.time, gen, RigExecPayloadKind::Scalar, 1)},
        Reserve(cache, 4, 4, 8), true));
    CHECK(cache.ClearCoverageUnpinned() == 1);
    CHECK(CoverageTotal(cache) == 0);
}

// Publish and lookup on the same keys from several threads keep the account
// honest: every surviving record is charged exactly once, and emptying the
// store ends at zero.
void
TestConcurrentCoveragePublishKeepsAccountingHonest()
{
    RigExecFrameCache cache;
    cache.SetByteCap(size_t(1) << 20);
    constexpr int kThreads = 3;
    constexpr int kOps = 300;
    constexpr int kKeys = 16;
    const RigExecOutputGeneration gen = MakeGeneration(1, 1, 0, 0);
    const RigExecOutputHandle handle = MakeHandle("joint.frame");

    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                const RigExecCoverageKey key =
                    MakeCoverageKey(1, double((n * 3 + t) % kKeys), gen);
                RigExecCoverageReservation reservation =
                    Reserve(cache, 8, 16, 24);
                if (!reservation.IsValid()) {
                    continue;
                }
                CHECK(cache.PublishCoverage(
                    key,
                    {MakePacket(handle, key.time, gen,
                                RigExecPayloadKind::Scalar, 2)},
                    std::move(reservation), true));
                RigExecCoverageRecord record;
                CHECK(cache.LookupCoverage(key, &record));
            }
        });
    }
    for (auto &w : workers) {
        w.join();
    }

    struct Survivor {
        RigExecCoverageKey key;
        size_t bytes;
    };
    std::vector<Survivor> survivors;
    for (int i = 0; i < kKeys; ++i) {
        const RigExecCoverageKey key = MakeCoverageKey(1, double(i), gen);
        RigExecCoverageRecord record;
        if (cache.LookupCoverage(key, &record)) {
            size_t total = 0;
            CHECK(record.bytes.TryTotal(&total));
            survivors.push_back(Survivor{key, total});
        }
    }

    size_t heldByRecords = 0;
    for (const Survivor &survivor : survivors) {
        CHECK(RigExecCheckedAdd(heldByRecords, survivor.bytes, &heldByRecords));
    }
    CHECK(heldByRecords == CoverageTotal(cache));

    for (const Survivor &survivor : survivors) {
        size_t remaining = 0;
        CHECK(RigExecCheckedSub(heldByRecords, survivor.bytes, &remaining));
        heldByRecords = remaining;
        CHECK(cache.EvictCoverage(survivor.key));
        CHECK(CoverageTotal(cache) == heldByRecords);
    }
    CHECK(cache.CoverageRecordCount() == 0);
    CHECK(cache.CoverageHeld().capture == 0);
    CHECK(cache.CoverageHeld().workspace == 0);
    CHECK(cache.CoverageHeld().payload == 0);
    CHECK(cache.CoverageHeld().pinned == 0);
}

} // namespace

int
main()
{
    TestKeyEqualitySeesBothDigests();
    TestKeyOrderingIsCompiledRigThenControl();
    TestEqualKeysHashEqual();
    TestTimeRidesTheEntry();
    TestAnEmptyCacheMisses();
    TestLookupWithNullPoseMisses();
    TestPublishThenLookupHits();
    TestEitherDigestHalfMisses();
    TestTimeIsNotKeyMaterial();
    TestPublishInvalidPoseDeclines();
    TestEvictDropsAndCounts();
    TestCapRoundTripsAndClearZeroesCounters();
    TestPoseBytesCountsMaps();
    TestDigestIgnoresEnqueueOrder();
    TestDigestSeesValueChanges();
    TestDigestSeesDuplicateValues();
    TestDigestSeesPathsAndTypes();
    TestDigestHitsAcrossUnrelatedEdits();
    TestDigestFirstSampleWins();
    TestDigestSeesOverrides();
    TestDigestIgnoresTime();
    TestDigestibleFlagsUnhashableValues();
    TestRecordedOrderAndStaticTableFoldAsPlain();
    TestRefusalDigestKeysTimeAndOverrides();
    TestDigestSeesUpstream();
    TestDigestFoldsUpstreamArrayHashes();
    TestLRUEvictsLeastRecentlyUsed();
    TestLookupRefreshesRecency();
    TestOverwriteRefreshesRecency();
    TestOversizedPublishDeclines();
    TestLoweringCapEvictsDown();
    TestConcurrentLookupDuringPublish();
    TestConcurrentPublishOfOneKey();
    TestClearDuringConcurrentPublishKeepsAccountingHonest();

    TestCoverageKeySeesEveryGenerationField();
    TestCoverageAdmissionRefusesBeforeAllocating();
    TestPublishChargesWhatExistsAndReturnsSurplus();
    TestCoveragePublishReplacingARecordBalancesBytes();
    TestPartialCoverageNeverSatisfiesPoseLookup();
    TestStaleGenerationMissesCoverage();
    TestArenaRetirementKeepsPayloadsAnswerable();
    TestPayloadReleaseBalancesAndRetiresTheRecord();
    TestPinHoldsBytesAndBlocksEviction();
    TestLeaseOutlivesTheRecord();
    TestReplacingARecordUnderALeaseBalancesBytes();
    TestSharedPayloadStaysSharedWhenARecordDropsIt();
    TestReleaseIsBalancedOrDeclined();
    TestCoverageSharesTheCapAndClearSweepsIt();
    TestConcurrentCoveragePublishKeepsAccountingHonest();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecFrameCache: all tests passed\n");
    return 0;
}
