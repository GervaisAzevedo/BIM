#pragma once


#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>
#include <cmath>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_3;
using Vector = Kernel::Vector_3;
using Mesh = CGAL::Surface_mesh<Point>;
using VertexDescriptor = typename boost::graph_traits<Mesh>::vertex_descriptor;
using EdgeDescriptor = typename boost::graph_traits<Mesh>::edge_descriptor;
using FaceDescriptor = typename boost::graph_traits<Mesh>::face_descriptor;
using MeshColoringOptions = CGAL::Graphics_scene_options<Mesh, VertexDescriptor, EdgeDescriptor, FaceDescriptor>;


 struct MeshUniformColorOptions : public MeshColoringOptions {
  int r, g, b;

  MeshUniformColorOptions(int r_, int g_, int b_)
      : r(r_), g(g_), b(b_) {}


  
  bool colored_face(const Mesh&, FaceDescriptor) const {
    return true;
  }
  bool colored_vertex(const Mesh&, VertexDescriptor) const {
    return true;
  }

  // Couleur appliquée à chaque face
  CGAL::IO::Color face_color(const Mesh&, FaceDescriptor) const {
    return CGAL::IO::Color(r, g, b);
  }
  
  // Couleur appliquée à chaque face
  CGAL::IO::Color vertex_color(const Mesh&, VertexDescriptor) const {
    return CGAL::IO::Color(r, g, b);
  }
};




struct MeshRandomizedColorOptions: public MeshColoringOptions {
  // All vertices are colored.
  bool colored_vertex(const Mesh&, VertexDescriptor) const
  { return false; }
 
  CGAL::IO::Color vertex_color(const Mesh&, VertexDescriptor) const
  {
    static bool v_green=true;
    v_green=!v_green;
    if(v_green) // 1 vertex out of two green (randomly)
    { return CGAL::IO::Color(0,220,0); }
    else // the others are blue
    { return CGAL::IO::Color(0,0,220); }
  }
};
 
struct MeshHeightColorOptions: public MeshColoringOptions {
  bool colored_vertex(const Mesh&, VertexDescriptor) const { return true; }

  CGAL::IO::Color vertex_color(const Mesh& ps, VertexDescriptor vd) const {
      auto z = ps.point(vd).z();  // <- ici pas de *
      unsigned char c = static_cast<unsigned char>(
          std::min(255.0, std::max(0.0, (z + 1.0) * 127.5))
      );
      return CGAL::IO::Color(c, c, 255 - c);
  }
};

struct MeshNormalColorOptions : public MeshColoringOptions {
    using NormalMap = Mesh::template Property_map<VertexDescriptor, Vector>;
    NormalMap normal_map;
    bool has_normals;

    MeshNormalColorOptions(const Mesh& mesh)
    {
        auto nm = mesh.property_map<VertexDescriptor, Vector>("v:normal");
        has_normals = nm.has_value();  // vérifier que la property_map existe
        if (has_normals) normal_map = *nm;  // accéder à la valeur de l'optional
    }

    bool colored_vertex(const Mesh& , VertexDescriptor) const {
        return has_normals;
    }

    CGAL::IO::Color vertex_color(const Mesh& , VertexDescriptor v) const {
        if (!has_normals) return CGAL::IO::Color(255,255,255); // blanc par défaut
        const Vector& n = normal_map[v];
        return CGAL::IO::Color (
            static_cast<unsigned char>(std::abs(n.x()) * 255),
            static_cast<unsigned char>(std::abs(n.y()) * 255),
            static_cast<unsigned char>(std::abs(n.z()) * 255)
        );
    }
};

