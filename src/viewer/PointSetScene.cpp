#include "../viewer/PointSetScene.h"

#include <CGAL/Graphics_scene_options.h>
#include <CGAL/draw_point_set_3.h>

using It = PS3::const_iterator;
using Options = CGAL::Graphics_scene_options<PS3, It, It, It>;

CGAL::Graphics_scene PointSetScene::createEmptyScene() {
    return CGAL::Graphics_scene();
}

void PointSetScene::addPointSetToSceneFromData(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode) {

    auto pointSet = Conversions::pclToCgal(*data.pointCloud);
    switch (mode) {
        case ColoringMode::NORMAL_ORIENTATION_RGB:
            CGAL::add_to_graphics_scene(pointSet, scene, NormalColorOptions());
            break;
        case ColoringMode::UNIFORM_GREEN:
            CGAL::add_to_graphics_scene(pointSet, scene, UniformColorOptions(0, 220, 0));
            break;
        case ColoringMode::UNIFORM_BLUE:
            CGAL::add_to_graphics_scene(pointSet, scene, UniformColorOptions(0, 0, 220));
            break;
        case ColoringMode::UNIFORM_RED:
            CGAL::add_to_graphics_scene(pointSet, scene, UniformColorOptions(220, 0, 0));
            break;
        default:
            CGAL::add_to_graphics_scene(pointSet, scene, UniformColorOptions(128, 128, 128));
            break;
    }
}

void PointSetScene::addPointSetToSceneFromData(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b) {
    auto pointSet = Conversions::pclToCgal(*data.pointCloud);
    CGAL::add_to_graphics_scene(pointSet, scene, UniformColorOptions(r, g, b));
}

