#include "../include/image_processing_pkg/camera_viewer.hpp"
#include "ui_camera_viewer.h"

// 카메라 이미지 표시 및 ui 업데이트 반영 
CameraViewer::CameraViewer(QWidget* parent) 
    : QMainWindow(parent)
    , ui(new Ui::CameraViewerDesign)
{
    ui->setupUi(this);
    
    qnode = new QNode();
    
    setupConnections();
    
    ui->imageLabel->setFixedSize(640, 480);
    ui->imageLabel->setAlignment(Qt::AlignCenter);
    
    setWindowTitle("Camera Viewer");
}

void CameraViewer::setupConnections()
{
    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
    QObject::connect(qnode, SIGNAL(imageReceived(QImage)), this, SLOT(updateCameraImage(QImage)));
}

void CameraViewer::updateCameraImage(const QImage &image)
{
    if (image.isNull()) {
        qDebug() << "Received null image";
        return;
    }
    
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaled = pixmap.scaled(ui->imageLabel->size(), 
                                 Qt::KeepAspectRatio,  
                                 Qt::SmoothTransformation);  
    
    ui->imageLabel->setPixmap(scaled);
}

void CameraViewer::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

CameraViewer::~CameraViewer()
{
    delete ui;
    delete qnode;
}
