#pragma once

#include "../viewer/coloring/PointSetColoringOptions.h"
#include "../viewer/coloring/ColoringMode.h"
#include "../utils/GeometryData.h"
#include "../utils/Conversions.h"
#include <CGAL/Graphics_scene.h>
#include <string>

class PointSetScene {
public:
    using GeomData = geom::GeometryData;
    using PS3 = geom::PS3;
    static CGAL::Graphics_scene createEmptyScene(); 


    static void addPointSetToSceneFromData(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode);
    static void addPointSetToSceneFromData(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b);
};

