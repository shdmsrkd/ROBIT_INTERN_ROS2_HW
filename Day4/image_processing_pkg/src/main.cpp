#include <QApplication>
#include "../include/image_processing_pkg/camera_viewer.hpp"
#include "../include/image_processing_pkg/parameter_controller.hpp"

// 창 띄우기 
int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    
    CameraViewer* viewer = new CameraViewer();
    viewer->show();
    
    ParameterController* controller = new ParameterController(viewer->getQNode());
    controller->show();
    
    return a.exec();
}
