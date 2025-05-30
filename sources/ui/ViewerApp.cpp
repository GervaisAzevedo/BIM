#include "../../headers/ui/ViewerApp.h"
#include "../../headers/io/PointSetLoader.h"
#include "../../headers/viewer/PointSetVisualizer.h"
#include "../../headers/utils/Conversions.h"
#include "../../headers/utils/GeometryData.h"
#include "../../headers/recognition/RecognizeWall.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/draw_point_set_3.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/Qt/Basic_viewer.h>
#include "../../headers/viewer/coloring/ColoringMode.h"

#ifdef CGAL_USE_BASIC_VIEWER
#include <QMainWindow>
#endif



void ViewerApp::run(const std::string& file1, const std::string& file2) {
    auto scene1 = PointSetVisualizer::createEmptyScene();
    auto scene2 = PointSetVisualizer::createEmptyScene();
     // auto scene1 = PointSetVisualizer::createSceneFromFile(file1, ColoringMode::RED);
     // auto scene2 = PointSetVisualizer::createSceneFromFile(file2, ColoringMode::NORMAL_ORIENTATION_RGB);
     // PointSetVisualizer::addPointSetToSceneFromFile(scene1, file2, ColoringMode::UNIFORM_BLUE);
     
     geom::GeometryData data;
     if (!PointSetLoader::loadPLY(file1, data))
        PointSetLoader::loadXYZ(file1, data);
     
    PointSetVisualizer::addPointSetToSceneFromData(scene1, data, ColoringMode::NORMAL_ORIENTATION_RGB);
            

    pcl::PointCloud<pcl::PointNormal> pclPointCloud =  *data.rawPoints;
    PlaneRecognition::RecognizeWall recognizer;
    recognizer.setInputCloud(pclPointCloud.makeShared());
    recognizer.detectWalls();

    auto walls = recognizer.getDetectedWalls();
    int r = 90;
    int step = walls.empty() ? 0 : 255 / static_cast<int>(walls.size());
    for (const auto& wall : walls) {
        auto cgalWall = Conversions::pclToCgal(*wall);
        auto wallData = geom::GeometryData(wall);
            PointSetVisualizer::addPointSetToSceneFromData(scene2, wallData, r, 100, 100);
            r += step; 
    }
    auto rest = recognizer.getRemainingCloud();
    auto restData = geom::GeometryData(rest);
    PointSetVisualizer::addPointSetToSceneFromData(scene2, restData, ColoringMode::UNIFORM_GREEN);


#ifdef CGAL_USE_BASIC_VIEWER

#if defined(CGAL_TEST_SUITE)
    bool cgal_test_suite = true;
#else
    bool cgal_test_suite = qEnvironmentVariableIsSet("CGAL_TEST_SUITE");
#endif

    if (cgal_test_suite) return;

    int argc = 1;
    const char* argv[2] = { "Draw several windows example", nullptr };
    QApplication app(argc, const_cast<char**>(argv));

    QMainWindow* mainWindow = new QMainWindow;
    QWidget* centralWidget = new QWidget(mainWindow);
    QHBoxLayout* layout = new QHBoxLayout(mainWindow);

    CGAL::Qt::Basic_viewer bv1(mainWindow, scene1);
    CGAL::Qt::Basic_viewer bv2(mainWindow, scene2);

    bv1.draw_vertices(true);
    bv2.draw_vertices(true);

    layout->addWidget(&bv1);
    layout->addWidget(&bv2);

    centralWidget->setLayout(layout);
    mainWindow->setCentralWidget(centralWidget);
    mainWindow->show();
    
    app.exec();
#endif
}

