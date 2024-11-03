#include "../include/image_processing_pkg/parameter_controller.hpp"
#include "ui_parameter_controller.h"

// ui 초기 설정 및 ui로 변경한 값을 qnode에 정의한 함수의 매개변수로 이용해 파라미터 값을 변경 

ParameterController::ParameterController(QNode* node, QWidget* parent) 
    : QMainWindow(parent)
    , ui(new Ui::ParameterControllerDesign)
    , qnode(node)
{
    if (!qnode) {
        throw std::runtime_error("QNode pointer is null");
    }
    
    ui->setupUi(this);
    
    
    
    setupHSVControls();
    setupMorphologyControls();
    setupCannyControls();
    setupROIControls();
    setupHoughControls();
    setupConnections();
    
    ui->horizontalSlider_H_lower->setValue(100);
    
    setWindowTitle("Image Processing Parameters");
    
    loadInitialParameters();
}

void ParameterController::loadInitialParameters()
{
    // HSV 값 로드
    ui->horizontalSlider_H_lower->setValue(qnode->getNode()->get_parameter("h_lower").as_int());
    ui->horizontalSlider_H_upper->setValue(qnode->getNode()->get_parameter("h_upper").as_int());
    ui->horizontalSlider_S_lower->setValue(qnode->getNode()->get_parameter("s_lower").as_int());
    ui->horizontalSlider_S_upper->setValue(qnode->getNode()->get_parameter("s_upper").as_int());
    ui->horizontalSlider_V_lower->setValue(qnode->getNode()->get_parameter("v_lower").as_int());
    ui->horizontalSlider_V_upper->setValue(qnode->getNode()->get_parameter("v_upper").as_int());
    
    // HSV2 값 로드
    ui->horizontalSlider_H_lower_2->setValue(qnode->getNode()->get_parameter("h_lower_2").as_int());
    ui->horizontalSlider_H_upper_2->setValue(qnode->getNode()->get_parameter("h_upper_2").as_int());
    ui->horizontalSlider_S_lower_2->setValue(qnode->getNode()->get_parameter("s_lower_2").as_int());
    ui->horizontalSlider_S_upper_2->setValue(qnode->getNode()->get_parameter("s_upper_2").as_int());
    ui->horizontalSlider_V_lower_2->setValue(qnode->getNode()->get_parameter("v_lower_2").as_int());
    ui->horizontalSlider_V_upper_2->setValue(qnode->getNode()->get_parameter("v_upper_2").as_int());

    // HSV3 값 로드
    ui->horizontalSlider_H_lower_3->setValue(qnode->getNode()->get_parameter("h_lower_3").as_int());
    ui->horizontalSlider_H_upper_3->setValue(qnode->getNode()->get_parameter("h_upper_3").as_int());
    ui->horizontalSlider_S_lower_3->setValue(qnode->getNode()->get_parameter("s_lower_3").as_int());
    ui->horizontalSlider_S_upper_3->setValue(qnode->getNode()->get_parameter("s_upper_3").as_int());
    ui->horizontalSlider_V_lower_3->setValue(qnode->getNode()->get_parameter("v_lower_3").as_int());
    ui->horizontalSlider_V_upper_3->setValue(qnode->getNode()->get_parameter("v_upper_3").as_int());
    
    // 모폴로지 값 로드
    ui->horizontalSlider_erode->setValue(qnode->getNode()->get_parameter("erode_iterations").as_int());
    ui->horizontalSlider_dilate->setValue(qnode->getNode()->get_parameter("dilate_iterations").as_int());
    
    // Canny 값 로드
    ui->horizontalSlider_canny1->setValue(qnode->getNode()->get_parameter("canny_threshold1").as_int());
    ui->horizontalSlider_canny2->setValue(qnode->getNode()->get_parameter("canny_threshold2").as_int());
    
    // ROI 값 로드
    ui->horizontalSlider_roi_x->setValue(qnode->getNode()->get_parameter("roi_x").as_int());
    ui->horizontalSlider_roi_y->setValue(qnode->getNode()->get_parameter("roi_y").as_int());
    ui->horizontalSlider_roi_width->setValue(qnode->getNode()->get_parameter("roi_width").as_int());
    ui->horizontalSlider_roi_height->setValue(qnode->getNode()->get_parameter("roi_height").as_int());
    
    // Hough 값 로드
    ui->horizontalSlider_hough_rho->setValue(qnode->getNode()->get_parameter("hough_rho").as_int());
    ui->horizontalSlider_hough_theta->setValue(qnode->getNode()->get_parameter("hough_theta").as_int());
    ui->horizontalSlider_hough_threshold->setValue(qnode->getNode()->get_parameter("hough_threshold").as_int());
    ui->horizontalSlider_hough_minline->setValue(qnode->getNode()->get_parameter("hough_min_line_length").as_int());
    ui->horizontalSlider_hough_maxgap->setValue(qnode->getNode()->get_parameter("hough_max_line_gap").as_int());
    
    // 모드 상태 로드
    ui->checkBox_hsv_enable->setChecked(qnode->getNode()->get_parameter("binary_mode1").as_bool());
    ui->checkBox_hsv_enable_2->setChecked(qnode->getNode()->get_parameter("binary_mode2").as_bool());
    ui->checkBox_hsv_enable_3->setChecked(qnode->getNode()->get_parameter("binary_mode3").as_bool());
    
    ui->checkBox_canny->setChecked(qnode->getNode()->get_parameter("canny_mode").as_bool());
    ui->checkBox_roi->setChecked(qnode->getNode()->get_parameter("roi_mode").as_bool());
    ui->checkBox_hough->setChecked(qnode->getNode()->get_parameter("hough_mode").as_bool());
    ui->checkBox_object_detection->setChecked(qnode->getNode()->get_parameter("object_detection_mode").as_bool());
}

void ParameterController::setupHSVControls()
{

    // 기존 HSV1 범위 설정
    ui->horizontalSlider_H_lower->setRange(0, 179);
    ui->horizontalSlider_H_upper->setRange(0, 179);
    ui->spinBox_H_lower->setRange(0, 179);
    ui->spinBox_H_upper->setRange(0, 179);
    
    ui->horizontalSlider_S_lower->setRange(0, 255);
    ui->horizontalSlider_S_upper->setRange(0, 255);
    ui->spinBox_S_lower->setRange(0, 255);
    ui->spinBox_S_upper->setRange(0, 255);
    
    ui->horizontalSlider_V_lower->setRange(0, 255);
    ui->horizontalSlider_V_upper->setRange(0, 255);
    ui->spinBox_V_lower->setRange(0, 255);
    ui->spinBox_V_upper->setRange(0, 255);

    // HSV2 범위 설정
    ui->horizontalSlider_H_lower_2->setRange(0, 179);
    ui->horizontalSlider_H_upper_2->setRange(0, 179);
    ui->spinBox_H_lower_2->setRange(0, 179);
    ui->spinBox_H_upper_2->setRange(0, 179);
    
    ui->horizontalSlider_S_lower_2->setRange(0, 255);
    ui->horizontalSlider_S_upper_2->setRange(0, 255);
    ui->spinBox_S_lower_2->setRange(0, 255);
    ui->spinBox_S_upper_2->setRange(0, 255);
    
    ui->horizontalSlider_V_lower_2->setRange(0, 255);
    ui->horizontalSlider_V_upper_2->setRange(0, 255);
    ui->spinBox_V_lower_2->setRange(0, 255);
    ui->spinBox_V_upper_2->setRange(0, 255);

    // HSV3 범위 설정
    ui->horizontalSlider_H_lower_3->setRange(0, 179);
    ui->horizontalSlider_H_upper_3->setRange(0, 179);
    ui->spinBox_H_lower_3->setRange(0, 179);
    ui->spinBox_H_upper_3->setRange(0, 179);
    
    ui->horizontalSlider_S_lower_3->setRange(0, 255);
    ui->horizontalSlider_S_upper_3->setRange(0, 255);
    ui->spinBox_S_lower_3->setRange(0, 255);
    ui->spinBox_S_upper_3->setRange(0, 255);
    
    ui->horizontalSlider_V_lower_3->setRange(0, 255);
    ui->horizontalSlider_V_upper_3->setRange(0, 255);
    ui->spinBox_V_lower_3->setRange(0, 255);
    ui->spinBox_V_upper_3->setRange(0, 255);
}


void ParameterController::setupMorphologyControls()
{
    ui->horizontalSlider_erode->setRange(0, 10);
    ui->spinBox_erode->setRange(0, 10);
    
    ui->horizontalSlider_dilate->setRange(0, 10);
    ui->spinBox_dilate->setRange(0, 10);
}

void ParameterController::setupCannyControls()
{
    ui->horizontalSlider_canny1->setRange(0, 255);
    ui->spinBox_canny1->setRange(0, 255);
    
    ui->horizontalSlider_canny2->setRange(0, 255);
    ui->spinBox_canny2->setRange(0, 255);
}

void ParameterController::setupROIControls()
{
    ui->horizontalSlider_roi_x->setRange(0, 1920);
    ui->spinBox_roi_x->setRange(0, 1920);
    
    ui->horizontalSlider_roi_y->setRange(0, 1080);
    ui->spinBox_roi_y->setRange(0, 1080);
    
    ui->horizontalSlider_roi_width->setRange(1, 1920);
    ui->spinBox_roi_width->setRange(1, 1920);
    
    ui->horizontalSlider_roi_height->setRange(1, 1080);
    ui->spinBox_roi_height->setRange(1, 1080);
}

void ParameterController::setupHoughControls()
{
    ui->horizontalSlider_hough_rho->setRange(1, 10);
    ui->spinBox_hough_rho->setRange(1, 10);
    
    ui->horizontalSlider_hough_theta->setRange(1, 180);
    ui->spinBox_hough_theta->setRange(1, 180);
    
    ui->horizontalSlider_hough_threshold->setRange(1, 200);
    ui->spinBox_hough_threshold->setRange(1, 200);
    
    ui->horizontalSlider_hough_minline->setRange(1, 200);
    ui->spinBox_hough_minline->setRange(1, 200);
    
    ui->horizontalSlider_hough_maxgap->setRange(1, 100);
    ui->spinBox_hough_maxgap->setRange(1, 100);
}

void ParameterController::setupConnections()
{
    // HSV1 (기존) 연결
    QObject::connect(ui->horizontalSlider_H_lower, SIGNAL(valueChanged(int)),
                     ui->spinBox_H_lower, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_H_lower, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_H_lower, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_H_upper, SIGNAL(valueChanged(int)),
                     ui->spinBox_H_upper, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_H_upper, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_H_upper, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_S_lower, SIGNAL(valueChanged(int)),
                     ui->spinBox_S_lower, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_S_lower, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_S_lower, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_S_upper, SIGNAL(valueChanged(int)),
                     ui->spinBox_S_upper, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_S_upper, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_S_upper, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_V_lower, SIGNAL(valueChanged(int)),
                     ui->spinBox_V_lower, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_V_lower, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_V_lower, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_V_upper, SIGNAL(valueChanged(int)),
                     ui->spinBox_V_upper, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_V_upper, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_V_upper, SLOT(setValue(int)));

    // HSV2 연결
    QObject::connect(ui->horizontalSlider_H_lower_2, SIGNAL(valueChanged(int)),
                     ui->spinBox_H_lower_2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_H_lower_2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_H_lower_2, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_H_upper_2, SIGNAL(valueChanged(int)),
                     ui->spinBox_H_upper_2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_H_upper_2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_H_upper_2, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_S_lower_2, SIGNAL(valueChanged(int)),
                     ui->spinBox_S_lower_2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_S_lower_2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_S_lower_2, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_S_upper_2, SIGNAL(valueChanged(int)),
                     ui->spinBox_S_upper_2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_S_upper_2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_S_upper_2, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_V_lower_2, SIGNAL(valueChanged(int)),
                     ui->spinBox_V_lower_2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_V_lower_2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_V_lower_2, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_V_upper_2, SIGNAL(valueChanged(int)),
                     ui->spinBox_V_upper_2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_V_upper_2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_V_upper_2, SLOT(setValue(int)));

    // HSV3 연결
    QObject::connect(ui->horizontalSlider_H_lower_3, SIGNAL(valueChanged(int)),
                     ui->spinBox_H_lower_3, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_H_lower_3, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_H_lower_3, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_H_upper_3, SIGNAL(valueChanged(int)),
                     ui->spinBox_H_upper_3, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_H_upper_3, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_H_upper_3, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_S_lower_3, SIGNAL(valueChanged(int)),
                     ui->spinBox_S_lower_3, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_S_lower_3, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_S_lower_3, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_S_upper_3, SIGNAL(valueChanged(int)),
                     ui->spinBox_S_upper_3, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_S_upper_3, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_S_upper_3, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_V_lower_3, SIGNAL(valueChanged(int)),
                     ui->spinBox_V_lower_3, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_V_lower_3, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_V_lower_3, SLOT(setValue(int)));

    QObject::connect(ui->horizontalSlider_V_upper_3, SIGNAL(valueChanged(int)),
                     ui->spinBox_V_upper_3, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_V_upper_3, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_V_upper_3, SLOT(setValue(int)));

    // HSV Enable 체크박스 연결
    QObject::connect(ui->checkBox_hsv_enable, SIGNAL(stateChanged(int)),
                     this, SLOT(onHSV1ModeChanged(int)));
    QObject::connect(ui->checkBox_hsv_enable_2, SIGNAL(stateChanged(int)),
                     this, SLOT(onHSV2ModeChanged(int)));
    QObject::connect(ui->checkBox_hsv_enable_3, SIGNAL(stateChanged(int)),
                     this, SLOT(onHSV3ModeChanged(int)));

    // HSV1 값 변경 시그널 연결
    QObject::connect(ui->horizontalSlider_H_lower, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_H_upper, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_S_lower, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_S_upper, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_V_lower, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_V_upper, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));

    // HSV2 값 변경 시그널 연결
    QObject::connect(ui->horizontalSlider_H_lower_2, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_H_upper_2, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_S_lower_2, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_S_upper_2, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_V_lower_2, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_V_upper_2, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));

    // HSV3 값 변경 시그널 연결
    QObject::connect(ui->horizontalSlider_H_lower_3, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_H_upper_3, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_S_lower_3, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_S_upper_3, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_V_lower_3, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));
    QObject::connect(ui->horizontalSlider_V_upper_3, SIGNAL(valueChanged(int)), 
                     this, SLOT(updateHSVValues()));

    // 모폴로지 연결
    QObject::connect(ui->horizontalSlider_erode, SIGNAL(valueChanged(int)),
                     ui->spinBox_erode, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_erode, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_erode, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_erode, SIGNAL(valueChanged(int)),
                     this, SLOT(updateMorphologyIterations()));

    QObject::connect(ui->horizontalSlider_dilate, SIGNAL(valueChanged(int)),
                     ui->spinBox_dilate, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_dilate, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_dilate, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_dilate, SIGNAL(valueChanged(int)),
                     this, SLOT(updateMorphologyIterations()));

    // Canny 연결
    QObject::connect(ui->horizontalSlider_canny1, SIGNAL(valueChanged(int)),
                     ui->spinBox_canny1, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_canny1, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_canny1, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_canny1, SIGNAL(valueChanged(int)),
                     this, SLOT(updateCannyParameters()));

    QObject::connect(ui->horizontalSlider_canny2, SIGNAL(valueChanged(int)),
                     ui->spinBox_canny2, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_canny2, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_canny2, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_canny2, SIGNAL(valueChanged(int)),
                     this, SLOT(updateCannyParameters()));

    QObject::connect(ui->checkBox_canny, SIGNAL(stateChanged(int)),
                     this, SLOT(onCannyModeChanged(int)));

    // ROI 연결
    QObject::connect(ui->horizontalSlider_roi_x, SIGNAL(valueChanged(int)),
                     ui->spinBox_roi_x, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_roi_x, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_roi_x, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_roi_x, SIGNAL(valueChanged(int)),
                     this, SLOT(updateROIParameters()));

    QObject::connect(ui->horizontalSlider_roi_y, SIGNAL(valueChanged(int)),
                     ui->spinBox_roi_y, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_roi_y, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_roi_y, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_roi_y, SIGNAL(valueChanged(int)),
                     this, SLOT(updateROIParameters()));

    QObject::connect(ui->horizontalSlider_roi_width, SIGNAL(valueChanged(int)),
                     ui->spinBox_roi_width, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_roi_width, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_roi_width, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_roi_width, SIGNAL(valueChanged(int)),
                     this, SLOT(updateROIParameters()));

    QObject::connect(ui->horizontalSlider_roi_height, SIGNAL(valueChanged(int)),
                     ui->spinBox_roi_height, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_roi_height, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_roi_height, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_roi_height, SIGNAL(valueChanged(int)),
                     this, SLOT(updateROIParameters()));

    QObject::connect(ui->checkBox_roi, SIGNAL(stateChanged(int)),
                     this, SLOT(onROIModeChanged(int)));

    // Hough 연결
    QObject::connect(ui->horizontalSlider_hough_rho, SIGNAL(valueChanged(int)),
                     ui->spinBox_hough_rho, SLOT(setValue(int)));
    QObject::connect(ui->spinBox_hough_rho, SIGNAL(valueChanged(int)),
                     ui->horizontalSlider_hough_rho, SLOT(setValue(int)));
    QObject::connect(ui->horizontalSlider_hough_rho, SIGNAL(valueChanged(int)),
                     this, SLOT(updateHoughParameters()));

   QObject::connect(ui->horizontalSlider_hough_theta, SIGNAL(valueChanged(int)),
                    ui->spinBox_hough_theta, SLOT(setValue(int)));
   QObject::connect(ui->spinBox_hough_theta, SIGNAL(valueChanged(int)),
                    ui->horizontalSlider_hough_theta, SLOT(setValue(int)));
   QObject::connect(ui->horizontalSlider_hough_theta, SIGNAL(valueChanged(int)),
                    this, SLOT(updateHoughParameters()));

   QObject::connect(ui->horizontalSlider_hough_threshold, SIGNAL(valueChanged(int)),
                    ui->spinBox_hough_threshold, SLOT(setValue(int)));
   QObject::connect(ui->spinBox_hough_threshold, SIGNAL(valueChanged(int)),
                    ui->horizontalSlider_hough_threshold, SLOT(setValue(int)));
   QObject::connect(ui->horizontalSlider_hough_threshold, SIGNAL(valueChanged(int)),
                    this, SLOT(updateHoughParameters()));

   QObject::connect(ui->horizontalSlider_hough_minline, SIGNAL(valueChanged(int)),
                    ui->spinBox_hough_minline, SLOT(setValue(int)));
   QObject::connect(ui->spinBox_hough_minline, SIGNAL(valueChanged(int)),
                    ui->horizontalSlider_hough_minline, SLOT(setValue(int)));
   QObject::connect(ui->horizontalSlider_hough_minline, SIGNAL(valueChanged(int)),
                    this, SLOT(updateHoughParameters()));

   QObject::connect(ui->horizontalSlider_hough_maxgap, SIGNAL(valueChanged(int)),
                    ui->spinBox_hough_maxgap, SLOT(setValue(int)));
   QObject::connect(ui->spinBox_hough_maxgap, SIGNAL(valueChanged(int)),
                    ui->horizontalSlider_hough_maxgap, SLOT(setValue(int)));
   QObject::connect(ui->horizontalSlider_hough_maxgap, SIGNAL(valueChanged(int)),
                    this, SLOT(updateHoughParameters()));

   QObject::connect(ui->checkBox_hough, SIGNAL(stateChanged(int)),
                    this, SLOT(onHoughModeChanged(int)));
                    
   QObject::connect(ui->checkBox_object_detection, SIGNAL(stateChanged(int)),
                    this, SLOT(onObjectDetectionModeChanged(int)));

   QObject::connect(ui->checkBox_hsv_enable, SIGNAL(stateChanged(int)),
                   this, SLOT(onBinaryMode1Changed(int)));
   QObject::connect(ui->checkBox_hsv_enable_2, SIGNAL(stateChanged(int)),
                   this, SLOT(onBinaryMode2Changed(int))); 
   QObject::connect(ui->checkBox_hsv_enable_3, SIGNAL(stateChanged(int)),
                   this, SLOT(onBinaryMode3Changed(int)));
}

void ParameterController::updateHSVValues()
{
    if (!qnode) return;
    
    qnode->getNode()->set_parameter(rclcpp::Parameter("h_lower", ui->horizontalSlider_H_lower->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("h_upper", ui->horizontalSlider_H_upper->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("s_lower", ui->horizontalSlider_S_lower->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("s_upper", ui->horizontalSlider_S_upper->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("v_lower", ui->horizontalSlider_V_lower->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("v_upper", ui->horizontalSlider_V_upper->value()));
    
    
    qnode->getNode()->set_parameter(rclcpp::Parameter("h_lower_2", ui->horizontalSlider_H_lower_2->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("h_upper_2", ui->horizontalSlider_H_upper_2->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("s_lower_2", ui->horizontalSlider_S_lower_2->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("s_upper_2", ui->horizontalSlider_S_upper_2->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("v_lower_2", ui->horizontalSlider_V_lower_2->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("v_upper_2", ui->horizontalSlider_V_upper_2->value()));

    // HSV3 파라미터 업데이트
    qnode->getNode()->set_parameter(rclcpp::Parameter("h_lower_3", ui->horizontalSlider_H_lower_3->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("h_upper_3", ui->horizontalSlider_H_upper_3->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("s_lower_3", ui->horizontalSlider_S_lower_3->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("s_upper_3", ui->horizontalSlider_S_upper_3->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("v_lower_3", ui->horizontalSlider_V_lower_3->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("v_upper_3", ui->horizontalSlider_V_upper_3->value()));
}

void ParameterController::updateMorphologyIterations()
{
    if (!qnode) return;
    
    qnode->getNode()->set_parameter(rclcpp::Parameter("erode_iterations", ui->horizontalSlider_erode->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("dilate_iterations", ui->horizontalSlider_dilate->value()));
}

void ParameterController::updateCannyParameters()
{
    if (!qnode) return;
    
    qnode->getNode()->set_parameter(rclcpp::Parameter("canny_threshold1", ui->horizontalSlider_canny1->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("canny_threshold2", ui->horizontalSlider_canny2->value()));
}

void ParameterController::updateROIParameters()
{
    if (!qnode) return;
    
    qnode->getNode()->set_parameter(rclcpp::Parameter("roi_x", ui->horizontalSlider_roi_x->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("roi_y", ui->horizontalSlider_roi_y->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("roi_width", ui->horizontalSlider_roi_width->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("roi_height", ui->horizontalSlider_roi_height->value()));
}

void ParameterController::updateHoughParameters()
{
    if (!qnode) return;
    
    qnode->getNode()->set_parameter(rclcpp::Parameter("hough_rho", ui->horizontalSlider_hough_rho->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("hough_theta", ui->horizontalSlider_hough_theta->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("hough_threshold", ui->horizontalSlider_hough_threshold->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("hough_min_line_length", ui->horizontalSlider_hough_minline->value()));
    qnode->getNode()->set_parameter(rclcpp::Parameter("hough_max_line_gap", ui->horizontalSlider_hough_maxgap->value()));
}

void ParameterController::onBinaryMode1Changed(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("binary_mode1", state == Qt::Checked));
}

void ParameterController::onBinaryMode2Changed(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("binary_mode2", state == Qt::Checked));
}

void ParameterController::onBinaryMode3Changed(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("binary_mode3", state == Qt::Checked));
}


void ParameterController::onBinaryModeChanged(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("binary_mode", state == Qt::Checked));
}

void ParameterController::onCannyModeChanged(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("canny_mode", state == Qt::Checked));
}

void ParameterController::onROIModeChanged(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("roi_mode", state == Qt::Checked));
}

void ParameterController::onHoughModeChanged(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("hough_mode", state == Qt::Checked));
}

void ParameterController::onObjectDetectionModeChanged(int state)
{
    if (!qnode) return;
    qnode->getNode()->set_parameter(rclcpp::Parameter("object_detection_mode", state == Qt::Checked));
}


ParameterController::~ParameterController()
{
    delete ui;
}
