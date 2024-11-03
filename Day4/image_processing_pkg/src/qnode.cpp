#include "../include/image_processing_pkg/qnode.hpp"

// 파라미터 선언 및 파라미터 값 불러와서 이미지에 반영하는 함수들 관리 
QNode::QNode()
{
   int argc = 0;
   char** argv = NULL;
   rclcpp::init(argc, argv);
   
   node = rclcpp::Node::make_shared("image_processing_pkg");
   
   declareParameters();
   
   image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
       "/camera1/camera/image_raw", 10, 
       std::bind(&QNode::imageCallback, this, std::placeholders::_1));
   
   this->start();
}

void QNode::setObjectDetectionMode(bool enable)
{
    node->set_parameter(rclcpp::Parameter("object_detection_mode", enable));
}


void QNode::declareParameters()
{
    // HSV 파라미터
    node->declare_parameter("h_lower", 0);
    node->declare_parameter("h_upper", 179);
    node->declare_parameter("s_lower", 0);
    node->declare_parameter("s_upper", 255);
    node->declare_parameter("v_lower", 0);
    node->declare_parameter("v_upper", 255);
    
    node->declare_parameter("h_lower_2", 0);
    node->declare_parameter("h_upper_2", 179);
    node->declare_parameter("s_lower_2", 0);
    node->declare_parameter("s_upper_2", 255);
    node->declare_parameter("v_lower_2", 0);
    node->declare_parameter("v_upper_2", 255);

    node->declare_parameter("h_lower_3", 0);
    node->declare_parameter("h_upper_3", 179);
    node->declare_parameter("s_lower_3", 0);
    node->declare_parameter("s_upper_3", 255);
    node->declare_parameter("v_lower_3", 0);
    node->declare_parameter("v_upper_3", 255);
    
    // 모폴로지 파라미터
    node->declare_parameter("erode_iterations", 2);
    node->declare_parameter("dilate_iterations", 2);
    
    // Canny 파라미터
    node->declare_parameter("canny_threshold1", 100);
    node->declare_parameter("canny_threshold2", 200);
    
    // ROI 파라미터
    node->declare_parameter("roi_x", 0);
    node->declare_parameter("roi_y", 0);
    node->declare_parameter("roi_width", 640);
    node->declare_parameter("roi_height", 480);
    
    // Hough 파라미터
    node->declare_parameter("hough_rho", 1);
    node->declare_parameter("hough_theta", 180);
    node->declare_parameter("hough_threshold", 50);
    node->declare_parameter("hough_min_line_length", 50);
    node->declare_parameter("hough_max_line_gap", 10);
    
    // 모드 파라미터
    node->declare_parameter("binary_mode", false);
    node->declare_parameter("canny_mode", false);
    node->declare_parameter("roi_mode", false);
    node->declare_parameter("hough_mode", false);
    node->declare_parameter("object_detection_mode", false);    
    
    node->declare_parameter("binary_mode1", false);
    node->declare_parameter("binary_mode2", false);
    node->declare_parameter("binary_mode3", false);

}

QNode::~QNode()
{
   if (rclcpp::ok())
   {
       rclcpp::shutdown();
   }
}

void QNode::run()
{
   rclcpp::WallRate loop_rate(20);
   while (rclcpp::ok())
   {
       rclcpp::spin_some(node);
       loop_rate.sleep();
   }
   rclcpp::shutdown();
   Q_EMIT rosShutDown();
}

cv::Mat QNode::convertToBinary(const cv::Mat& input)
{
    cv::Mat hsv_image;
    cv::cvtColor(input, hsv_image, cv::COLOR_BGR2HSV);
    
    cv::Mat result = cv::Mat::zeros(input.size(), CV_8UC1);  // 결과 이미지 초기화
    
    // HSV1 처리
    if (node->get_parameter("binary_mode1").as_bool()) {
        cv::Mat binary_image1;
        cv::Scalar lower_bound1(
            node->get_parameter("h_lower").as_int(),
            node->get_parameter("s_lower").as_int(),
            node->get_parameter("v_lower").as_int()
        );
        cv::Scalar upper_bound1(
            node->get_parameter("h_upper").as_int(),
            node->get_parameter("s_upper").as_int(),
            node->get_parameter("v_upper").as_int()
        );
        cv::inRange(hsv_image, lower_bound1, upper_bound1, binary_image1);
        cv::bitwise_or(result, binary_image1, result);
    }
    
    // HSV2 처리
    if (node->get_parameter("binary_mode2").as_bool()) {
        cv::Mat binary_image2;
        cv::Scalar lower_bound2(
            node->get_parameter("h_lower_2").as_int(),
            node->get_parameter("s_lower_2").as_int(),
            node->get_parameter("v_lower_2").as_int()
        );
        cv::Scalar upper_bound2(
            node->get_parameter("h_upper_2").as_int(),
            node->get_parameter("s_upper_2").as_int(),
            node->get_parameter("v_upper_2").as_int()
        );
        cv::inRange(hsv_image, lower_bound2, upper_bound2, binary_image2);
        cv::bitwise_or(result, binary_image2, result);
    }
    
    // HSV3 처리
    if (node->get_parameter("binary_mode3").as_bool()) {
        cv::Mat binary_image3;
        cv::Scalar lower_bound3(
            node->get_parameter("h_lower_3").as_int(),
            node->get_parameter("s_lower_3").as_int(),
            node->get_parameter("v_lower_3").as_int()
        );
        cv::Scalar upper_bound3(
            node->get_parameter("h_upper_3").as_int(),
            node->get_parameter("s_upper_3").as_int(),
            node->get_parameter("v_upper_3").as_int()
        );
        cv::inRange(hsv_image, lower_bound3, upper_bound3, binary_image3);
        cv::bitwise_or(result, binary_image3, result);
    }
    
    return result;
}

cv::Mat QNode::applyMorphology(const cv::Mat& input)
{
    cv::Mat output = input.clone();
    int erode_iterations = node->get_parameter("erode_iterations").as_int();
    int dilate_iterations = node->get_parameter("dilate_iterations").as_int();
    
    if (erode_iterations > 0) 
    {
        cv::erode(output, output, cv::Mat(), cv::Point(-1,-1), erode_iterations);
    }
    
    if (dilate_iterations > 0) 
    {
        cv::dilate(output, output, cv::Mat(), cv::Point(-1,-1), dilate_iterations);
    }
    
    return output;
}

cv::Mat QNode::applyCannyEdge(const cv::Mat& input)
{
    cv::Mat output;
    int threshold1 = node->get_parameter("canny_threshold1").as_int();
    int threshold2 = node->get_parameter("canny_threshold2").as_int();
    cv::Canny(input, output, threshold1, threshold2);
    return output;
}

cv::Mat QNode::applyROI(const cv::Mat& input)
{
    int x = node->get_parameter("roi_x").as_int();
    int y = node->get_parameter("roi_y").as_int();
    int width = node->get_parameter("roi_width").as_int();
    int height = node->get_parameter("roi_height").as_int();

    x = std::min(std::max(x, 0), input.cols - 1);
    y = std::min(std::max(y, 0), input.rows - 1);
    width = std::min(width, input.cols - x);
    height = std::min(height, input.rows - y);

    if (width <= 0 || height <= 0) 
    {
        return input.clone();  
    }
    
    cv::Rect roi(x, y, width, height);
    return input(roi).clone();
}

cv::Mat QNode::applyHoughLines(const cv::Mat& input)
{
    cv::Mat result = input.clone();
    cv::Mat gray_image;

    if (input.channels() == 3) 
    {
        cv::cvtColor(input, gray_image, cv::COLOR_BGR2GRAY);
    } 
    
    else 
    {
        gray_image = input.clone();
    }
    
    cv::Mat edges;
    cv::Canny(gray_image, edges, 50, 150);
    
    std::vector<cv::Vec4i> lines;
    int rho = node->get_parameter("hough_rho").as_int();
    int theta = node->get_parameter("hough_theta").as_int();
    int threshold = node->get_parameter("hough_threshold").as_int();
    int minLineLength = node->get_parameter("hough_min_line_length").as_int();
    int maxLineGap = node->get_parameter("hough_max_line_gap").as_int();
    
    cv::HoughLinesP(edges, lines, rho, CV_PI/theta, threshold, minLineLength, maxLineGap);
    
    if (!lines.empty()) 
    {
        cv::Vec4i longest_line = lines[0];
        double max_length = 0;
        
        
	for (const auto& line : lines) 
	{
	    cv::line(result, 
		cv::Point(line[0], line[1]),  
		cv::Point(line[2], line[3]),  
		cv::Scalar(255, 255, 0), 2); 
		
	    RCLCPP_INFO(node->get_logger(), 
		"Line detected - Start: (%d, %d), End: (%d, %d)",
		line[0], line[1], line[2], line[3]);
	}
    }
    
    return result;
}

cv::Mat QNode::detectAndDrawObjects(const cv::Mat& input)
{
    cv::Mat result = input.clone();
    cv::Mat img_labels, stats, centroids;
    
    // 연결된 컴포넌트 분석 수행
    int numOfLabels = cv::connectedComponentsWithStats(input, img_labels, stats, centroids, 8, CV_32S);

    // 객체 검출 (라벨 0은 배경이므로 1부터 시작)
    for(int i = 1; i < numOfLabels; i++)
    {
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        
        double centerX = centroids.at<double>(i, 0);
        double centerY = centroids.at<double>(i, 1);

        // 바운딩 박스 그리기
        cv::rectangle(result, cv::Point(x, y), 
                     cv::Point(x + width, y + height),
                     cv::Scalar(0, 255, 0), 2);
        
        // 중심점 그리기
        cv::circle(result, cv::Point(centerX, centerY), 3, 
                  cv::Scalar(0, 0, 255), -1);
        
        // 객체 정보 표시
        std::string objectInfo = "ID: " + std::to_string(i);
        cv::putText(result, objectInfo, cv::Point(x, y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(255, 0, 0), 1);
        
        // 로그 출력
        RCLCPP_INFO(node->get_logger(), 
            "Object %d - Position: (%d, %d), Size: %dx%d, Area: %d, Center: (%.1f, %.1f)",
            i, x, y, width, height, area, centerX, centerY);
    }
    
    return result;
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    if (node->get_parameter("roi_mode").as_bool()) 
    {
        cv_image = applyROI(cv_image);
    }

    // 바이너리 모드 체크 - 하나라도 활성화되어 있으면 실행
    if (node->get_parameter("binary_mode1").as_bool() || 
        node->get_parameter("binary_mode2").as_bool() || 
        node->get_parameter("binary_mode3").as_bool()) 
    {
        cv_image = convertToBinary(cv_image);
        int erode_iterations = node->get_parameter("erode_iterations").as_int();
        int dilate_iterations = node->get_parameter("dilate_iterations").as_int();
        
        if (erode_iterations > 0 || dilate_iterations > 0) 
        {
            cv_image = applyMorphology(cv_image);
        }
        
        if (node->get_parameter("object_detection_mode").as_bool()) {
            cv_image = detectAndDrawObjects(cv_image);
        }
        
        // 결과를 BGR로 변환
        if (cv_image.channels() == 1) {
            cv::cvtColor(cv_image, cv_image, cv::COLOR_GRAY2BGR);
        }
    }

    if (node->get_parameter("canny_mode").as_bool()) 
    {
        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
        cv_image = applyCannyEdge(gray_image);
        cv::cvtColor(cv_image, cv_image, cv::COLOR_GRAY2BGR);
    }

    if (node->get_parameter("hough_mode").as_bool()) 
    {
        cv_image = applyHoughLines(cv_image);
    }
    
    QImage qimg(cv_image.data, cv_image.cols, cv_image.rows, 
               cv_image.step, QImage::Format_BGR888);
    
    Q_EMIT imageReceived(qimg);
}
