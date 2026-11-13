#include "rigExec/outputCatalog.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecScene/outputDescriptors.h"
#include "rigExecSchema/schemaNames.h"

#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

const char *const kStageText = R"USDA(#usda 1.0

def Xform "Asset"
{
    def Scope "UnusualContainer"
    {
        def RigExecRoot "Character"
        {
            def Scope "InputsAnywhere"
            {
                def RigExecControl "Drive"
                {
                    double avars:ty = 2
                }
            }

            def Scope "WorkAnywhere"
            {
                def Scope "PropertyStack"
                {
                    def RigExecFloatMathMover "Second" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        uniform token rigExec:operation = "multiply"
                        float inputs:value = 2
                        rel rigExec:moves = </Asset/UnusualContainer/Character/RandomData.amount>
                    }

                    def RigExecFloatMathMover "First" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        uniform token rigExec:operation = "add"
                        float inputs:value = 1
                        rel rigExec:moves = </Asset/UnusualContainer/Character/RandomData.amount>
                    }
                }

                def Scope "PointStack"
                {
                    def RigExecMatrixMover "Second" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        float inputs:defaultWeight = 1
                        rel rigExec:moves = </Asset/UnusualContainer/Geometry/Body.points>
                        rel rigExec:transform = </Asset/UnusualContainer/Character/InputsAnywhere/Drive>
                    }

                    def RigExecMatrixMover "First" (
                        prepend apiSchemas = ["RigExecMoverAPI"]
                    )
                    {
                        float inputs:defaultWeight = 1
                        rel rigExec:moves = </Asset/UnusualContainer/Geometry/Body.points>
                        rel rigExec:transform = </Asset/UnusualContainer/Character/InputsAnywhere/Drive>
                    }
                }
            }

            def Scope "RandomData"
            {
                float amount = 3
            }
        }

        def Scope "Geometry"
        {
            def Points "Body"
            {
                point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
            }
        }
    }
}
)USDA";

struct Fixture {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    const RigExecBakedProgram *program = nullptr;
};

Fixture
BuildFixture()
{
    Fixture fixture;
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer->ImportFromString(kStageText));
    fixture.stage = UsdStage::Open(layer);
    CHECK(fixture.stage);
    fixture.evaluator = std::make_unique<RigExecRigEvaluator>(
        fixture.stage, SdfPath("/Asset/UnusualContainer/Character"));
    std::vector<std::string> errors;
    CHECK(fixture.evaluator->Compile(&errors));
    for (const std::string &error : errors)
        std::printf("    compile: %s\n", error.c_str());
    fixture.program = fixture.evaluator->GetBakedProgram();
    CHECK(fixture.program);
    return fixture;
}

const RigExecOutputBinding *
Find(const RigExecOutputCatalog &catalog, const std::string &target,
     const std::string &computation, const std::string &phase)
{
    for (const RigExecOutputBinding &binding : catalog.GetBindings()) {
        if (binding.descriptor.target == target &&
            binding.descriptor.publicComputation == computation &&
            binding.descriptor.phase == phase) {
            return &binding;
        }
    }
    return nullptr;
}

RigExecValueId
FindCanonicalValue(const RigExecBakedProgramImpl &program,
                   RigExecBakedSlotDomain domain, uint32_t slot)
{
    for (size_t i = 0; i < program.opAdapter.values.size(); ++i) {
        const RigExecOpValueState &value = program.opAdapter.values[i];
        if (value.domain == uint32_t(domain) && value.slot == slot)
            return RigExecValueId(i);
    }
    return kRigExecNoOutputValue;
}

RigExecPayloadKind
PayloadOf(const std::string &type)
{
    RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
    return RigExecPayloadKindForType(type, &kind) ? kind
                                                  : RigExecPayloadKind::Unknown;
}

bool
HasCanonicalProducer(const RigExecBakedProgramImpl &program,
                     RigExecValueId value)
{
    size_t writers = 0;
    for (const RigExecCompiledOp &op : program.opGraph.ops) {
        writers += std::count(op.descriptor.writes.begin(),
                              op.descriptor.writes.end(), value);
    }
    return writers == 1;
}

void
TestCatalogBindsRetainedOutputs()
{
    Fixture fixture = BuildFixture();
    if (!fixture.program)
        return;

    RigExecOutputCatalog catalog;
    std::string error;
    CHECK(RigExecDiscoverOutputs(*fixture.program, "stage-A/character", 17,
                                 &catalog, &error));
    CHECK(error.empty());
    CHECK(catalog.GetRigIdentity() == "stage-A/character");
    CHECK(catalog.GetCompileEpoch() == 17);

    const RigExecBakedProgramImpl &program = fixture.program->GetStepGraph();
    const std::string property =
        "/Asset/UnusualContainer/Character/RandomData.amount";
    const std::string points = "/Asset/UnusualContainer/Geometry/Body.points";
    const std::string drive =
        "/Asset/UnusualContainer/Character/InputsAnywhere/Drive";

    const RigExecOutputBinding *scalar = Find(catalog, property, "", "final");
    CHECK(scalar);
    if (scalar) {
        CHECK(scalar->kind == RigExecOutputKind::Scalar);
        CHECK(scalar->storage == RigExecOutputStorage::PropertyValue);
        CHECK(scalar->handle.type == RigExecPayloadText::scalar);
        CHECK(PayloadOf(scalar->handle.type) == RigExecPayloadKind::Scalar);
        CHECK(scalar->status == RigExecOutputStatus::Pending);
        CHECK(scalar->requiresValidity);
        CHECK(scalar->requiresEffects);
        CHECK(HasCanonicalProducer(program, scalar->producerValue));
        if (program.propertyChains.size() == 1) {
            const auto &chain = program.propertyChains.front();
            CHECK(scalar->producerValue ==
                  FindCanonicalValue(
                      program, RigExecBakedSlotDomain::PropertyResult,
                      chain.versionBase + uint32_t(chain.revisions.size())));
        }
    }

    const RigExecOutputBinding *transform =
        Find(catalog, drive, SchemaComputationText::computeMatrix, "base");
    CHECK(transform);
    if (transform) {
        CHECK(transform->kind == RigExecOutputKind::Transform);
        CHECK(transform->storage == RigExecOutputStorage::ProviderMatrix);
        CHECK(transform->handle.type == RigExecPayloadText::matrix4x4Row);
        CHECK(PayloadOf(transform->handle.type) ==
              RigExecPayloadKind::Matrix4x4Row);
        CHECK(HasCanonicalProducer(program, transform->producerValue));
        const auto slot = std::find(program.paths.begin(), program.paths.end(),
                                    SdfPath(drive));
        CHECK(slot != program.paths.end());
        if (slot != program.paths.end()) {
            CHECK(transform->producerValue ==
                  FindCanonicalValue(program,
                                     RigExecBakedSlotDomain::BaseMatrix,
                                     uint32_t(slot - program.paths.begin())));
        }
    }

    const RigExecOutputBinding *basePoints = Find(catalog, points, "", "base");
    CHECK(basePoints);
    if (basePoints) {
        CHECK(basePoints->kind == RigExecOutputKind::Points);
        CHECK(basePoints->storage == RigExecOutputStorage::PointVersion);
        CHECK(basePoints->version == 0);
        CHECK(
            basePoints->producerValue ==
            FindCanonicalValue(program, RigExecBakedSlotDomain::ChainBase, 0));
    }

    const RigExecOutputBinding *finalPoints =
        Find(catalog, points, "", "final");
    CHECK(finalPoints);
    if (finalPoints) {
        CHECK(finalPoints->kind == RigExecOutputKind::Points);
        CHECK(finalPoints->storage == RigExecOutputStorage::PointVersion);
        CHECK(finalPoints->handle.type == RigExecPayloadText::point3);
        CHECK(PayloadOf(finalPoints->handle.type) ==
              RigExecPayloadKind::Point3);
        CHECK(finalPoints->status == RigExecOutputStatus::Pending);
        CHECK(HasCanonicalProducer(program, finalPoints->producerValue));
        CHECK(finalPoints->producerValue ==
              FindCanonicalValue(program, RigExecBakedSlotDomain::ChainPoints,
                                 0));
    }

    CHECK(program.propertyChains.size() == 1);
    if (program.propertyChains.size() == 1) {
        const auto &chain = program.propertyChains.front();
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            const std::string phase = chain.revisions[i].mover.GetString();
            const RigExecOutputBinding *intermediate =
                Find(catalog, property, "", phase);
            CHECK(intermediate);
            if (intermediate) {
                CHECK(intermediate->storage ==
                      RigExecOutputStorage::PropertyValue);
                CHECK(intermediate->version == i + 1);
                CHECK(
                    HasCanonicalProducer(program, intermediate->producerValue));
                CHECK(intermediate->producerValue ==
                      FindCanonicalValue(program,
                                         RigExecBakedSlotDomain::PropertyResult,
                                         chain.versionBase + uint32_t(i + 1)));
            }
        }
    }

    CHECK(program.chains.size() == 1);
    if (program.chains.size() == 1) {
        const auto &chain = program.chains.front();
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            const std::string phase = chain.revisions[i].moverPath.GetString();
            const RigExecOutputBinding *intermediate =
                Find(catalog, points, "", phase);
            CHECK(intermediate);
            if (intermediate) {
                CHECK(intermediate->storage ==
                      RigExecOutputStorage::PointVersion);
                CHECK(intermediate->version == i + 1);
                CHECK(
                    HasCanonicalProducer(program, intermediate->producerValue));
            }
        }
    }

    bool hasEffectRequirement = false;
    for (const auto &edges : catalog.GetRequirements().effects)
        hasEffectRequirement |= !edges.empty();
    CHECK(hasEffectRequirement);

    for (const RigExecOutputBinding &binding : catalog.GetBindings()) {
        CHECK(binding.storage == RigExecOutputStorage::PropertyValue ||
              binding.storage == RigExecOutputStorage::ProviderMatrix ||
              binding.storage == RigExecOutputStorage::FrameMatrix ||
              binding.storage == RigExecOutputStorage::PointVersion);
        if (binding.producerValue != kRigExecNoOutputValue) {
            // Retained storage only: no export may bind a scratch packet,
            // candidate, chunk or staging slot.
            const RigExecOpValueState &value =
                program.opAdapter.values[size_t(binding.producerValue)];
            CHECK(
                value.domain ==
                    uint32_t(RigExecBakedSlotDomain::PropertyResult) ||
                value.domain == uint32_t(RigExecBakedSlotDomain::BaseMatrix) ||
                value.domain == uint32_t(RigExecBakedSlotDomain::FinalMatrix) ||
                value.domain == uint32_t(RigExecBakedSlotDomain::FrameMatrix) ||
                value.domain == uint32_t(RigExecBakedSlotDomain::ChainBase) ||
                value.domain == uint32_t(RigExecBakedSlotDomain::ChainPoints) ||
                value.domain == uint32_t(RigExecBakedSlotDomain::RevisionDone));
            CHECK(value.scratch.empty());
        }
        if (binding.status == RigExecOutputStatus::Pending) {
            size_t output = 0;
            CHECK(catalog.GetDemandIndex().Find(binding.handle, &output));
            CHECK(catalog.GetDemandIndex().Producer(output) != kRigExecNoOp);
            CHECK(catalog.GetDemandIndex().Closure(output).Contains(
                catalog.GetDemandIndex().Producer(output)));
        }
    }
}

void
TestEpochAndDescriptorResolution()
{
    Fixture fixture = BuildFixture();
    if (!fixture.program)
        return;

    RigExecOutputCatalog catalog;
    std::string error;
    CHECK(RigExecDiscoverOutputs(*fixture.program, "stage-A/character", 23,
                                 &catalog, &error));
    const RigExecOutputBinding *binding =
        Find(catalog, "/Asset/UnusualContainer/Character/RandomData.amount", "",
             "final");
    CHECK(binding);
    if (!binding)
        return;

    const RigExecOutputBinding *resolved = nullptr;
    std::string reason;
    CHECK(catalog.Resolve(binding->handle, &resolved, &reason) ==
          RigExecOutputStatus::Pending);
    CHECK(resolved == binding);

    RigExecOutputHandle newer = binding->handle;
    ++newer.valueRevision;
    resolved = nullptr;
    CHECK(catalog.Resolve(newer, &resolved, &reason) ==
          RigExecOutputStatus::Pending);
    CHECK(resolved == binding);

    RigExecOutputHandle stale = binding->handle;
    ++stale.compileEpoch;
    resolved = binding;
    CHECK(catalog.Resolve(stale, &resolved, &reason) ==
          RigExecOutputStatus::Invalidated);
    CHECK(!resolved);
    CHECK(reason.find("compile epoch") != std::string::npos);

    std::string before;
    CHECK(fixture.stage->GetRootLayer()->ExportToString(&before));
    RigExecOutputDescriptor descriptor;
    CHECK(RigExecResolveOutputDescriptor(
        fixture.stage,
        RigExecValueAddress::Property(
            SdfPath("/Asset/UnusualContainer/Character/RandomData.amount")),
        &descriptor, &error));
    CHECK(descriptor.target ==
          "/Asset/UnusualContainer/Character/RandomData.amount");
    CHECK(descriptor.publicComputation.empty());
    CHECK(descriptor.phase == "final");
    CHECK(descriptor.type == RigExecPayloadText::scalar);
    CHECK(descriptor.semanticKey == binding->handle.semanticKey);

    const RigExecOutputBinding *matrix =
        Find(catalog, "/Asset/UnusualContainer/Character/InputsAnywhere/Drive",
             SchemaComputationText::computeMatrix, "base");
    CHECK(matrix);
    if (!matrix)
        return;
    RigExecOutputDescriptor matrixDescriptor;
    CHECK(RigExecResolveOutputDescriptor(
        fixture.stage,
        RigExecValueAddress::Prim(
            SdfPath("/Asset/UnusualContainer/Character/InputsAnywhere/Drive"),
            TfToken(SchemaComputationText::computeMatrix), TfToken("base")),
        &matrixDescriptor, &error));
    CHECK(matrixDescriptor.semanticKey == matrix->handle.semanticKey);
    CHECK(matrixDescriptor.type == RigExecPayloadText::matrix4x4Row);

    RigExecOutputDescriptor scratch;
    CHECK(!RigExecResolveOutputDescriptor(
        fixture.stage,
        RigExecValueAddress::Prim(
            SdfPath("/Asset/UnusualContainer/Character/InputsAnywhere/Drive"),
            TfToken(SchemaComputationText::computePointFrameArray)),
        &scratch, &error));
    CHECK(error.find("not an exported computation") != std::string::npos);

    std::string after;
    CHECK(fixture.stage->GetRootLayer()->ExportToString(&after));
    CHECK(before == after);
}

void
TestGpuOnlyPointsAreUnsupported()
{
    Fixture fixture = BuildFixture();
    if (!fixture.program)
        return;
    RigExecBakedProgramImpl &program =
        const_cast<RigExecBakedProgramImpl &>(fixture.program->GetStepGraph());
    CHECK(program.chains.size() == 1);
    if (program.chains.size() != 1)
        return;
    program.chains.front().gpu = true;

    RigExecOutputCatalog catalog;
    std::string error;
    CHECK(RigExecDiscoverOutputs(*fixture.program, "stage-A/character", 31,
                                 &catalog, &error));
    const RigExecOutputBinding *points = Find(
        catalog, "/Asset/UnusualContainer/Geometry/Body.points", "", "final");
    CHECK(points);
    if (points) {
        CHECK(points->status == RigExecOutputStatus::Unsupported);
        CHECK(points->reason.find("GPU") != std::string::npos);
        CHECK(points->producerValue == kRigExecNoOutputValue);
        size_t output = 0;
        CHECK(catalog.GetDemandIndex().Find(points->handle, &output));
        CHECK(catalog.GetDemandIndex().ProducerStatus(output) ==
              RigExecOutputStatus::Unsupported);
    }
    for (const RigExecOutputBinding &binding : catalog.GetBindings()) {
        if (binding.kind != RigExecOutputKind::Points)
            continue;
        if (binding.descriptor.phase == "base") {
            CHECK(binding.status == RigExecOutputStatus::Pending);
            continue;
        }
        CHECK(binding.status == RigExecOutputStatus::Unsupported);
    }
}

void
TestScratchAddressesResolveUnsupported()
{
    Fixture fixture = BuildFixture();
    if (!fixture.program)
        return;

    RigExecOutputCatalog catalog;
    std::string error;
    CHECK(RigExecDiscoverOutputs(*fixture.program, "stage-A/character", 41,
                                 &catalog, &error));

    // A dense program slot is never a semantic address: nothing resolves to it,
    // and the index refuses it instead of falling back to the stage.
    RigExecOutputHandle scratch;
    scratch.rigIdentity = "stage-A/character";
    scratch.compileEpoch = 41;
    scratch.readPhase = "scratchChunk";
    scratch.semanticKey = RigExecOutputSemanticKey(
        "/Asset/UnusualContainer/Geometry/Body.points", "", "scratchChunk");
    scratch.type = RigExecPayloadText::point3;
    const RigExecOutputBinding *binding = nullptr;
    std::string reason;
    CHECK(catalog.Resolve(scratch, &binding, &reason) ==
          RigExecOutputStatus::Unsupported);
    CHECK(!binding);
    CHECK(reason.find("not exported") != std::string::npos);
    size_t output = 0;
    CHECK(!catalog.GetDemandIndex().Find(scratch, &output));

    RigExecOutputHandle otherRig = scratch;
    otherRig.rigIdentity = "stage-B/character";
    CHECK(catalog.Resolve(otherRig, &binding, &reason) ==
          RigExecOutputStatus::Invalidated);
    CHECK(reason.find("another rig") != std::string::npos);
}

} // namespace

int
main()
{
    TestCatalogBindsRetainedOutputs();
    TestEpochAndDescriptorResolution();
    TestGpuOnlyPointsAreUnsupported();
    TestScratchAddressesResolveUnsupported();
    if (failures)
        std::printf("%d failures\n", failures);
    return failures ? 1 : 0;
}
