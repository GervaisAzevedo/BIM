#include "ViewerApp.h"

#include "../bim/Bim.h"
#include "../io/PointSetLoader.h"
#include "../viewer/BimScene.h"
#include "../utils/Conversions.h"
#include "../utils/GeometryData.h"
#include "../recognition/ElementRecognizer.h"
#include "../reconstruction/GenericMeshConstructor.h"
#include "../viewer/coloring/ColoringMode.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/draw_point_set_3.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/Qt/Basic_viewer.h>

#ifdef CGAL_USE_BASIC_VIEWER
#include <QMainWindow>
#endif



void ViewerApp::run(const std::string& file1) {
    auto scene1 = BimScene::createEmptyScene();
    auto scene2 = BimScene::createEmptyScene();
    
    geom::GeometryData data;
    if (!PointSetLoader::loadPLY(file1, data))
        PointSetLoader::loadXYZ(file1, data);
        
    PointSetScene::addPointSetToSceneFromData(scene1, data, ColoringMode::NORMAL_ORIENTATION_RGB);
    
    ElementRecognizer elementRecognizer;
    Bim bim = elementRecognizer.createBimFromCloud(*data.pointCloud); 
    bim.printBimInfo();
    
    bim.constructAllMeshes();
    
    BimScene bimScene;
    bimScene.addBimToScene(scene2, bim);
                
   

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

