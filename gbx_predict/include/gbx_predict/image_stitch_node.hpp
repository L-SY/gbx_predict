#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <mutex>
#include <memory>

namespace gbx_predict
{

class ImageStitchNode : public rclcpp::Node
{
public:
    ImageStitchNode();
    ~ImageStitchNode();

private:
    // ROS2 callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);
    
    // Core processing functions
    void processImage(const cv::Mat& img, const std_msgs::msg::Header& header);
    void extractROI(const cv::Mat& img, cv::Mat& roi, cv::Rect& roi_rect);
    void publishDebugImage(const cv::Mat& img, const cv::Rect& roi_rect, 
                          const std_msgs::msg::Header& header);
    void stitchImages(const cv::Mat& current_roi, const std_msgs::msg::Header& header);
    void publishStitchedImage(const std_msgs::msg::Header& header);
    void resetPanorama();
    
    // Computer vision functions
    bool detectFeatures(const cv::Mat& img1, const cv::Mat& img2,
                       std::vector<cv::KeyPoint>& kp1, std::vector<cv::KeyPoint>& kp2,
                       cv::Mat& desc1, cv::Mat& desc2);
    bool matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2,
                      std::vector<cv::DMatch>& good_matches);
    bool estimateTransform(const std::vector<cv::KeyPoint>& kp1,
                          const std::vector<cv::KeyPoint>& kp2,
                          const std::vector<cv::DMatch>& matches,
                          double& offset);
    void performStitching(const cv::Mat& current_roi, int shift);
    
    // ROS2 publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_roi_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_matches_pub_;
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Image processing variables
    cv::Mat panorama_;
    cv::Mat last_roi_;
    std::mutex panorama_mutex_;
    
    // ORB detector
    cv::Ptr<cv::ORB> orb_detector_;
    
    // Dynamic parameters - ROI related
    double roi_x_offset_ratio_;
    double roi_y_offset_ratio_;
    double roi_width_ratio_;
    double roi_height_ratio_;
    
    // Dynamic parameters - Stitching related
    int min_shift_;
    int max_shift_;
    int max_width_;
    bool auto_reset_;
    bool reset_now_;
    bool stitch_along_y_;
    
    // Feature detection parameters
    int orb_features_;
    double match_ratio_;
    int min_matches_;
    
    // Topic names
    std::string input_topic_;
    std::string output_topic_;
    std::string debug_topic_;
};

} // namespace gbx_predict 