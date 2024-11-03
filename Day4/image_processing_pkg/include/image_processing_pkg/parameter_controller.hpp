#ifndef PARAMETER_CONTROLLER_HPP
#define PARAMETER_CONTROLLER_HPP

#include <QMainWindow>
#include "../include/image_processing_pkg/qnode.hpp"

namespace Ui {
class ParameterControllerDesign;
}

class ParameterController : public QMainWindow
{
    Q_OBJECT

public:
    explicit ParameterController(QNode* node, QWidget* parent = nullptr);
    ~ParameterController();

private Q_SLOTS:
    void updateHSVValues();
    void updateMorphologyIterations();
    void updateCannyParameters();
    void updateROIParameters();
    void updateHoughParameters();
    void onBinaryModeChanged(int state);
    void onCannyModeChanged(int state);
    void onROIModeChanged(int state);
    void onHoughModeChanged(int state);
    void onObjectDetectionModeChanged(int state);
    void onBinaryMode1Changed(int state);
    void onBinaryMode2Changed(int state);
    void onBinaryMode3Changed(int state);

private:
    Ui::ParameterControllerDesign* ui;
    QNode* qnode;
    
    void setupHSVControls();
    void setupMorphologyControls();
    void setupCannyControls();
    void setupROIControls();
    void setupHoughControls();
    void setupConnections();
    void loadInitialParameters();
};

#endif // PARAMETER_CONTROLLER_HPP
