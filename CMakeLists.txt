cmake_minimum_required(VERSION 3.12...3.31)
project(TestPCL)

add_definitions(-DCGAL_USE_BASIC_VIEWER)

# ==== Find Dependencies ====
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)
find_package(Eigen3 3.1.0)
include(CGAL_Eigen3_support)

# ==== Include and Link Directories ====
include_directories(
    ${PCL_INCLUDE_DIRS}
    # Tu peux réactiver headers si besoin, mais vérifie bien qu'ils n'importent pas Qt ou macros conflictuelles
    # ${CMAKE_SOURCE_DIR}/headers
)

link_directories(${PCL_LIBRARY_DIRS})

# Attention aux définitions PCL, teste sans au début pour isoler problème
# add_definitions(${PCL_DEFINITIONS})

# ==== Gather source files ====
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    ./*.cpp
)

# ==== Create Executable ====
add_executable(main
    main.cpp
    ${SOURCES}
)

# ==== Link Libraries ====

# CGAL Qt6 viewer avant PCL
if(CGAL_Qt6_FOUND)
  target_link_libraries(main PRIVATE CGAL::CGAL_Basic_viewer)
  message(STATUS "CGAL_Qt6 was found, linking CGAL::CGAL_Basic_viewer")
else()
  message(WARNING "CGAL_Qt6 not configured: examples that require Qt will not be compiled.")
endif()

if(TARGET CGAL::Eigen3_support)
  target_link_libraries(main PRIVATE CGAL::Eigen3_support)
endif()

# Enfin PCL à la fin
target_link_libraries(main PRIVATE ${PCL_LIBRARIES})


