 #pragma once

#include "../viewer/coloring/MeshColoringOptions.h"
#include "../viewer/coloring/ColoringMode.h"
#include "../utils/GeometryData.h"
#include "../utils/Conversions.h"
#include <CGAL/Graphics_scene.h>
#include <string>

#include <random>

class MeshScene {
public:
    using Mesh = geom::Mesh;
    using Polyhedron = geom::Polyhedron;
    using GeomData = geom::GeometryData;
    using PS3 = geom::PS3;
    using It = PS3::const_iterator;
    using Options = CGAL::Graphics_scene_options<PS3, It, It, It>;
    
    static CGAL::Graphics_scene createEmptyScene(); 

    static void addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data);
    static void addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data);
    
    static void addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode);
    static void addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode);

    static void addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b);
    static void addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b);
};

