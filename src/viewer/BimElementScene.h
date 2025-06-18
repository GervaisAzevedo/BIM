#pragma once


#include "../bim/BimElement.h"
#include "../viewer/MeshScene.h"
#include "../viewer/PointSetScene.h"

#include "../viewer/coloring/PointSetColoringOptions.h"
#include "../viewer/coloring/MeshColoringOptions.h"
#include "../viewer/coloring/ColoringMode.h"
#include "../utils/GeometryData.h"
#include "../utils/Conversions.h"
#include <CGAL/Graphics_scene.h>
#include <string>

enum class BimElementRepresentationType {
    BEST_AVAILABLE,
    POINT_SET,
    FINE_MESH,
    COARSE_MESH
};

class BimElementScene {
public:
    using GeomData = geom::GeometryData;
    using PS3 = geom::PS3;
    using It = PS3::const_iterator;
    using Options = CGAL::Graphics_scene_options<PS3, It, It, It>;
    using RepType = BimElementRepresentationType;

    static CGAL::Graphics_scene createEmptyScene(); 


    static void addBimElementToScene(CGAL::Graphics_scene& scene, const BimElement& bimElement, ColoringMode mode, RepType repType = RepType::BEST_AVAILABLE);
    
    static void addBimElementToScene(CGAL::Graphics_scene& scene, const BimElement& bimElement, int r, int g, int b, RepType repType = RepType::BEST_AVAILABLE);
};

