#pragma once


#include "../bim/Bim.h"
#include "../bim/BimElement.h"
#include "../bim/GroundElement.h"
#include "../bim/WallElement.h"
#include "../bim/RoofElement.h"
#include "../viewer/BimElementScene.h"

#include "../viewer/coloring/PointSetColoringOptions.h"
#include "../viewer/coloring/MeshColoringOptions.h"
#include "../viewer/coloring/ColoringMode.h"
#include "../utils/GeometryData.h"
#include "../utils/Conversions.h"
#include <CGAL/Graphics_scene.h>
#include <string>

class BimScene {
public:
    using Color = CGAL::IO::Color;

    BimScene();
    CGAL::Graphics_scene createEmptyScene(); 
    Color generateVariation(Color& baseColor, int index, int total);
    void addBimToScene(CGAL::Graphics_scene& scene, const Bim& bim);

private:
    
    Color m_groundBaseColor = Color(0, 50, 100);   
    Color m_wallBaseColor = Color(100, 0, 0);   // rouge
    Color m_roofBaseColor = Color(0, 0, 100);   // bleu
    Color m_otherBaseColor = Color(0, 100, 0);  // vert

};

