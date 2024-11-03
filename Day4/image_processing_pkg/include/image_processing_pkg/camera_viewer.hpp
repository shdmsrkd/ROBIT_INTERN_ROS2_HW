#ifndef CAMERA_VIEWER_HPP
#define CAMERA_VIEWER_HPP

#include <QMainWindow>
#include <QImage>
#include <QDebug>  
#include <QPixmap>
#include <QCloseEvent>
#include "qnode.hpp"

namespace Ui {
class CameraViewerDesign;
}

class CameraViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit CameraViewer(QWidget* parent = nullptr);
    ~CameraViewer();
    QNode* getQNode() { return qnode; }

protected:
    void closeEvent(QCloseEvent* event);

private Q_SLOTS:
    void updateCameraImage(const QImage &image);

private:
    void setupConnections();
    
    Ui::CameraViewerDesign* ui;
    QNode* qnode;
};

#endif // CAMERA_VIEWER_HPP
