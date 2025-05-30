#pragma once

#include "../../headers/viewer/coloring/ColoringOptions.h"
#include "../../headers/viewer/coloring/ColoringMode.h"
#include "../../headers/utils/GeometryData.h"
#include "../../headers/utils/Conversions.h"
#include <CGAL/Graphics_scene.h>
#include <string>

class PointSetVisualizer {
public:
    using GeomData = geom::GeometryData;
    using PS3 = geom::PS3;
    static CGAL::Graphics_scene createEmptyScene(); 


    static void addPointSetToSceneFromData(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode);
    static void addPointSetToSceneFromData(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b);
};

