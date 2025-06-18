#include "../viewer/BimElementScene.h"

CGAL::Graphics_scene BimElementScene::createEmptyScene() {
    return CGAL::Graphics_scene();
}

// Méthode utilitaire pour éviter la duplication
static BimElementScene::RepType resolveRepType(const BimElementScene::GeomData& data, BimElementScene::RepType requested) {
    using RepType = BimElementScene::RepType;

    if (requested != RepType::BEST_AVAILABLE)
        return requested;

    if (data.hasCoarseMesh()) return RepType::COARSE_MESH;
    if (data.hasFineMesh())   return RepType::FINE_MESH;
    return RepType::POINT_SET;
}

void BimElementScene::addBimElementToScene(CGAL::Graphics_scene& scene, const BimElement& bimElement, ColoringMode mode, RepType repType) {
    const GeomData& data = bimElement.getGeometryData();
    repType = resolveRepType(data, repType);

    switch (repType) {
        case RepType::COARSE_MESH:
            MeshScene::addCoarseMeshToScene(scene, data, mode);
            break;
        case RepType::FINE_MESH:
            MeshScene::addFineMeshToScene(scene, data, mode);
            break;
        case RepType::POINT_SET:
        default:
            PointSetScene::addPointSetToSceneFromData(scene, data, mode);
            break;
    }
}

void BimElementScene::addBimElementToScene(CGAL::Graphics_scene& scene, const BimElement& bimElement, int r, int g, int b, RepType repType) {
    const GeomData& data = bimElement.getGeometryData();
    repType = resolveRepType(data, repType);

    switch (repType) {
        case RepType::COARSE_MESH:
            MeshScene::addCoarseMeshToScene(scene, data, r, g, b);
            break;
        case RepType::FINE_MESH:
            MeshScene::addFineMeshToScene(scene, data, r, g, b);
            break;
        case RepType::POINT_SET:
        default:
            PointSetScene::addPointSetToSceneFromData(scene, data, r, g, b);
            break;
    }
}

