#include "../viewer/MeshScene.h"
#include <fstream>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/draw_surface_mesh.h>

#include <CGAL/IO/PLY.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <filesystem>  // C++17

namespace fs = std::filesystem;

using Mesh = geom::Mesh;
using Polyhedron = geom::Polyhedron;
using GeomData = geom::GeometryData;
using PS3 = geom::PS3;
using It = PS3::const_iterator;

using MeshOptions = CGAL::Graphics_scene_options<
    Mesh, 
    typename Mesh::Vertex_index, 
    typename Mesh::Edge_index, 
    typename Mesh::Face_index>;


// Fonction helper interne pour ajouter un mesh avec une couleur personnalisée
static void addMeshWithColor(CGAL::Graphics_scene& scene, 
                             const Mesh& mesh, 
                             int r = 255, int g = 0, int b = 0)
{

    CGAL::add_to_graphics_scene(mesh, scene,  MeshUniformColorOptions(r,g,b));
}

CGAL::Graphics_scene MeshScene::createEmptyScene() {
    return CGAL::Graphics_scene();
}

void MeshScene::addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data) {
    addMeshWithColor(scene, data.coarseMesh);
}

void MeshScene::addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data) {
    std::cout << "[MeshScene] addFineMeshToScene : fineMesh a " << data.fineMesh.number_of_faces() << " faces." << std::endl;
    
    addMeshWithColor(scene, data.fineMesh);
}




void MeshScene::addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode) {
    // TODO: Implémenter la coloration selon le mode
    // Par défaut, on colore en rouge
    addMeshWithColor(scene, data.coarseMesh);
}

void MeshScene::addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, ColoringMode mode) {
    // TODO: Implémenter la coloration selon le mode
    // Par défaut, on colore en rouge
    addMeshWithColor(scene, data.fineMesh);
}

void MeshScene::addCoarseMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b) {
    addMeshWithColor(scene, data.coarseMesh, r, g, b);
}

void MeshScene::addFineMeshToScene(CGAL::Graphics_scene& scene, const GeomData& data, int r, int g, int b) {
    addMeshWithColor(scene, data.fineMesh, r, g, b);
}

