#include "GenericMeshConstructor.h"
#include "./GenericStrategies/PoissonSurfaceReconstruction.h"
#include "./GenericStrategies/AdvancingFrontSurfaceReconstruction.h"
#include "./GenericStrategies/ScaleSpaceSurfaceReconstruction.h"



void GenericMeshConstructor::constructMesh(BimElement& element) const {
    auto& geomData = element.getGeometryData();
    if (geomData.hasPoints()) {
        const auto& pointCloud = geomData.getPointCloud();
        auto ps3 = Conversions::pclToCgal(*pointCloud);
        geomData.setFineMesh(reconstructMesh(ps3, Method::ADVANCING_FRONT));
    }
}

geom::Mesh GenericMeshConstructor::reconstructMesh(const geom::PS3& pointSet, Method method) {
    switch (method) {
        case Method::POISSON:
            return PoissonSurfaceReconstruction::run(pointSet);
        case Method::ADVANCING_FRONT:
            return AdvancingFrontSurfaceReconstruction::run(pointSet);
        case Method::SCALE_SPACE:
            return ScaleSpaceSurfaceReconstruction::run(pointSet);
        default:
            throw std::invalid_argument("Unknown reconstruction method.");
    }
}

