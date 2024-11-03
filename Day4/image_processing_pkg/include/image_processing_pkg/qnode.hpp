// qnode.hpp
#ifndef Q_NODE_HPP
#define Q_NODE_HPP

#include <QObject>
#include <QThread>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

class QNode : public QThread
{
    Q_OBJECT

public:
    QNode();
    virtual ~QNode();
    void run();
    void setImageSize(int width, int height);
    rclcpp::Node::SharedPtr getNode() { return node; }
    void setObjectDetectionMode(bool enable);

Q_SIGNALS:
    void rosShutDown();
    void imageReceived(const QImage& image);

private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    cv::Mat convertToBinary(const cv::Mat& input);
    cv::Mat applyMorphology(const cv::Mat& input);
    cv::Mat applyCannyEdge(const cv::Mat& input);
    cv::Mat applyROI(const cv::Mat& input);
    cv::Mat applyHoughLines(const cv::Mat& input);
    cv::Mat detectAndDrawObjects(const cv::Mat& input);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void declareParameters();

    std::mutex hsv_mutex_;
    std::mutex canny_mutex_;
    std::mutex roi_mutex_;
    std::mutex hough_mutex_;

    int image_width_;
    int image_height_;
};

#endif // Q_NODE_HPP
