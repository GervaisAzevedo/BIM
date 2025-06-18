#include "BimScene.h"

using Color = CGAL::IO::Color;


BimScene::BimScene() {}

Color BimScene::generateVariation(Color& baseColor, int index, int total) {
    int variation = (total <= 1) ? 0 : (index * 100 / (total - 1)); 

    int r = std::min(255, static_cast<int>(baseColor.red()) + (static_cast<int>(baseColor.red()) > 0 ? variation : 0));
    int g = std::min(255, static_cast<int>(baseColor.green()) + (static_cast<int>(baseColor.green()) > 0 ? variation : 0));
    int b = std::min(255, static_cast<int>(baseColor.blue()) + (static_cast<int>(baseColor.blue()) > 0 ? variation : 0));

    return Color(r, g, b);
}

CGAL::Graphics_scene BimScene::createEmptyScene() {
    return CGAL::Graphics_scene();
}


void BimScene::addBimToScene(CGAL::Graphics_scene& scene, const Bim& bim) {
    
    // Ground
    const auto* ground = bim.getGround();
    BimElementScene::addBimElementToScene(scene, *ground, 
            static_cast<int>(m_groundBaseColor.red()),
            static_cast<int>(m_groundBaseColor.green()),
            static_cast<int>(m_groundBaseColor.blue())
            );
   
    
    // Wall
    int i = 0;
    int nbElements = bim.getWallElements().size();
    for (const auto& e : bim.getWallElements()) {
        Color c = generateVariation(m_wallBaseColor, i, nbElements);
        BimElementScene::addBimElementToScene(scene, *e, 
            static_cast<int>(c.red()),
            static_cast<int>(c.green()),
            static_cast<int>(c.blue())
            /*BimElementRepresentationType::POINT_SET*/
            );
            
        i++;
    }
    
    // Roof
    i = 0;
    nbElements = bim.getRoofElements().size();
    for (const auto& e : bim.getRoofElements()) {
        Color c = generateVariation(m_roofBaseColor, i, nbElements);
        BimElementScene::addBimElementToScene(scene, *e, 
            static_cast<int>(c.red()),
            static_cast<int>(c.green()),
            static_cast<int>(c.blue()));
        i++;
    }
    
    // Other
    i = 0;
    nbElements = bim.getOtherElements().size();
    for (const auto& e : bim.getOtherElements()) {
        Color c = generateVariation(m_otherBaseColor, i, nbElements);
        BimElementScene::addBimElementToScene(scene, *e,
            static_cast<int>(c.red()),
            static_cast<int>(c.green()),
            static_cast<int>(c.blue()));
        i++;
    }
}

