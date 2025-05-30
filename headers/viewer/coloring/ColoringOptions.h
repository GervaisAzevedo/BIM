#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>
#include <cmath>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_3;
using Vector = Kernel::Vector_3;
using PS3 = CGAL::Point_set_3<Point, Vector>;
using It = typename PS3::const_iterator;
using Options = CGAL::Graphics_scene_options<PS3, It, It, It>;

// Functor pour un coloriage uniforme


struct UniformColorOptions: public Options {
  int r, g, b;
  UniformColorOptions(int r_, int g_, int b_) 
        : r(r_), g(g_), b(b_) {}

  bool colored_vertex(const PS3&, It) const
  { return true; }
  CGAL::IO::Color vertex_color(const PS3&, It) const
  { return CGAL::IO::Color(r,g,b); }
};



struct HeightColorOptions: public Options {
  bool colored_vertex(const PS3&, It) const { return true; }

    CGAL::IO::Color vertex_color(const PS3& ps, It it) const {
        auto z = ps.point(*it).z();
        unsigned char c = static_cast<unsigned char>(
            std::min(255.0, std::max(0.0, (z + 1.0) * 127.5))
        );
        return CGAL::IO::Color(c, c, 255 - c);
    }
};
 
 
struct NormalColorOptions: public Options {
bool colored_vertex(const PS3& ps, It it) const {
        return ps.has_normal_map();
    }
   

    CGAL::IO::Color vertex_color(const PS3& ps, It it) const {
        const Vector& n = ps.normal(*it);
	return CGAL::IO::Color (
    static_cast<unsigned char>(std::abs(n.x()) * 255),
    static_cast<unsigned char>(std::abs(n.y()) * 255),
    static_cast<unsigned char>(std::abs(n.z()) * 255)
);
    }
};
