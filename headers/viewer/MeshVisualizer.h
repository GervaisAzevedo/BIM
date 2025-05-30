#pragma once

#include "../../headers/viewer/coloring/ColoringOptions.h"
#include "../../headers/viewer/coloring/ColoringMode.h"
#include "../../headers/utils/GeometryData.h"
#include "../../headers/utils/Conversions.h"
#include <CGAL/Graphics_scene.h>
#include <string>

class MeshVisualizer {
public:
    using GeomData = geom::GeometryData;
    using PS3 = geom::PS3;
    static CGAL::Graphics_scene createEmptyScene(); 

    static void addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data);
    static void addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data);
    
    static void addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode);
    static void addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode);

    static void addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b);
    static void addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b);
};

