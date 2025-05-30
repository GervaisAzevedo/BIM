#include "../../headers/viewer/MeshVisualizer.h"

#include <CGAL/Graphics_scene_options.h>
#include<CGAL/draw_polyhedron.h>


using It = PS3::const_iterator;
using Options = CGAL::Graphics_scene_options<PS3, It, It, It>;

CGAL::Graphics_scene MeshVisualizer::createEmptyScene() {
    return CGAL::Graphics_scene();
}

void MeshVisualizer::addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data) {
    auto mesh = data.coarseMesh;
    CGAL::add_to_graphics_scene(mesh, scene);
}

void MeshVisualizer::addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data) {
    auto mesh = data.fineMesh;
    CGAL::add_to_graphics_scene(mesh, scene);
}

void MeshVisualizer::addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode) {
    auto mesh = data.coarseMesh;
/*
    switch (mode) {
        case ColoringMode::UNIFORM_GREEN:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(0, 220, 0));
            break;
        case ColoringMode::UNIFORM_BLUE:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(0, 0, 220));
            break;
        case ColoringMode::UNIFORM_RED:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(220, 0, 0));
            break;
        default:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(128, 128, 128));
            break;
    }
   */
}

void MeshVisualizer::addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode) {
    auto mesh = data.fineMesh;
    /*
    switch (mode) {
        case ColoringMode::UNIFORM_GREEN:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(0, 220, 0));
            break;
        case ColoringMode::UNIFORM_BLUE:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(0, 0, 220));
            break;
        case ColoringMode::UNIFORM_RED:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(220, 0, 0));
            break;
        default:
            CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(128, 128, 128));
            break;
    }
    */
}

void MeshVisualizer::addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b) {
    const auto& mesh = data.coarseMesh;
    // CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(r, g, b));
}

void MeshVisualizer::addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b) {
    const auto& mesh = data.fineMesh;
    // CGAL::add_to_graphics_scene(mesh, scene, UniformColorOptions(r, g, b));
}

