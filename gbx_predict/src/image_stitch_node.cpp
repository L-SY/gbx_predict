#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <mutex>
#include <memory>

class ImageStitchNode : public rclcpp::Node
{
public:
    ImageStitchNode() : Node("image_stitch_node")
    {
        // Declare parameters with default values
        this->declare_parameter("input_topic", "/camera/image_raw");
        this->declare_parameter("output_topic", "/stitched_image");
        this->declare_parameter("debug_topic", "/debug_image");
        
        // ROI parameters
        this->declare_parameter("roi_x_offset_ratio", 0.1);
        this->declare_parameter("roi_y_offset_ratio", 0.1);
        this->declare_parameter("roi_width_ratio", 0.7);
        this->declare_parameter("roi_height_ratio", 0.8);
        
        // Stitching parameters
        this->declare_parameter("min_shift", 1);
        this->declare_parameter("max_shift", 200);
        this->declare_parameter("max_width", 10000000);
        this->declare_parameter("auto_reset", false);
        this->declare_parameter("reset_now", false);
        this->declare_parameter("stitch_along_y", true);
        
        // Feature detection parameters
        this->declare_parameter("orb_features", 1000);
        this->declare_parameter("match_ratio", 0.75);
        this->declare_parameter("min_matches", 4);
        
        // Get initial parameter values
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        debug_topic_ = this->get_parameter("debug_topic").as_string();
        
        roi_x_offset_ratio_ = this->get_parameter("roi_x_offset_ratio").as_double();
        roi_y_offset_ratio_ = this->get_parameter("roi_y_offset_ratio").as_double();
        roi_width_ratio_ = this->get_parameter("roi_width_ratio").as_double();
        roi_height_ratio_ = this->get_parameter("roi_height_ratio").as_double();
        
        min_shift_ = this->get_parameter("min_shift").as_int();
        max_shift_ = this->get_parameter("max_shift").as_int();
        max_width_ = this->get_parameter("max_width").as_int();
        auto_reset_ = this->get_parameter("auto_reset").as_bool();
        reset_now_ = this->get_parameter("reset_now").as_bool();
        stitch_along_y_ = this->get_parameter("stitch_along_y").as_bool();
        
        orb_features_ = this->get_parameter("orb_features").as_int();
        match_ratio_ = this->get_parameter("match_ratio").as_double();
        min_matches_ = this->get_parameter("min_matches").as_int();
        
        // Initialize ORB detector
        orb_detector_ = cv::ORB::create(orb_features_);
        
        // Set up parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ImageStitchNode::parametersCallback, this, std::placeholders::_1));
        
        // Create publishers
        stitched_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic_, 10);
        debug_roi_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic_ + "/roi", 10);
        debug_matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_topic_ + "/matches", 10);
        
        // Create subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10, std::bind(&ImageStitchNode::imageCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "ImageStitchNode initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat img = cv_ptr->image;
        if (img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }
        
        processImage(img, msg->header);
    }
    
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto & param : parameters) {
            if (param.get_name() == "roi_x_offset_ratio") {
                roi_x_offset_ratio_ = param.as_double();
            } else if (param.get_name() == "roi_y_offset_ratio") {
                roi_y_offset_ratio_ = param.as_double();
            } else if (param.get_name() == "roi_width_ratio") {
                roi_width_ratio_ = param.as_double();
            } else if (param.get_name() == "roi_height_ratio") {
                roi_height_ratio_ = param.as_double();
            } else if (param.get_name() == "min_shift") {
                min_shift_ = param.as_int();
            } else if (param.get_name() == "max_shift") {
                max_shift_ = param.as_int();
            } else if (param.get_name() == "max_width") {
                max_width_ = param.as_int();
            } else if (param.get_name() == "auto_reset") {
                auto_reset_ = param.as_bool();
            } else if (param.get_name() == "reset_now") {
                reset_now_ = param.as_bool();
                if (reset_now_) {
                    resetPanorama();
                    RCLCPP_INFO(this->get_logger(), "Panorama reset");
                }
            } else if (param.get_name() == "stitch_along_y") {
                stitch_along_y_ = param.as_bool();
            } else if (param.get_name() == "orb_features") {
                orb_features_ = param.as_int();
                orb_detector_ = cv::ORB::create(orb_features_);
            } else if (param.get_name() == "match_ratio") {
                match_ratio_ = param.as_double();
            } else if (param.get_name() == "min_matches") {
                min_matches_ = param.as_int();
            }
        }
        
        return result;
    }
    
    void processImage(const cv::Mat& img, const std_msgs::msg::Header& header)
    {
        // Extract ROI
        cv::Mat roi;
        cv::Rect roi_rect;
        extractROI(img, roi, roi_rect);
        
        // Publish debug image with ROI highlighted
        publishDebugImage(img, roi_rect, header);
        
        // Stitch images
        stitchImages(roi, header);
    }
    
    void extractROI(const cv::Mat& img, cv::Mat& roi, cv::Rect& roi_rect)
    {
        int W = img.cols;
        int H = img.rows;
        
        // Calculate ROI dimensions
        int x_offset = static_cast<int>(W * roi_x_offset_ratio_);
        int y_offset = static_cast<int>(H * roi_y_offset_ratio_);
        int width = static_cast<int>(W * roi_width_ratio_);
        int height = static_cast<int>(H * roi_height_ratio_);
        
        // Ensure ROI is within image bounds
        x_offset = std::max(0, std::min(x_offset, W - 1));
        y_offset = std::max(0, std::min(y_offset, H - 1));
        width = std::max(1, std::min(width, W - x_offset));
        height = std::max(1, std::min(height, H - y_offset));
        
        roi_rect = cv::Rect(x_offset, y_offset, width, height);
        roi = img(roi_rect).clone();
    }
    
    void publishDebugImage(const cv::Mat& img, const cv::Rect& roi_rect, 
                          const std_msgs::msg::Header& header)
    {
        cv::Mat debug_img = img.clone();
        
        // Draw ROI rectangle in green
        cv::rectangle(debug_img, roi_rect, cv::Scalar(0, 255, 0), 2);
        
        // Add text annotation
        std::string text = "ROI: " + std::to_string(roi_rect.x) + "," + std::to_string(roi_rect.y) + 
                          " " + std::to_string(roi_rect.width) + "x" + std::to_string(roi_rect.height);
        cv::putText(debug_img, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        
        // Publish debug image
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = "bgr8";
        cv_image.image = debug_img;
        debug_pub_->publish(*cv_image.toImageMsg());
    }
    
    void stitchImages(const cv::Mat& current_roi, const std_msgs::msg::Header& header)
    {
        std::lock_guard<std::mutex> lock(panorama_mutex_);
        
        // Initialize panorama if empty or reset requested
        if (auto_reset_ || panorama_.empty() || last_roi_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Initializing panorama with first frame");
            panorama_ = current_roi.clone();
            last_roi_ = current_roi.clone();
            publishStitchedImage(header);
            return;
        }
        
        // Resize if size mismatch
        cv::Mat roi_to_process = current_roi;
        if (current_roi.size() != last_roi_.size()) {
            RCLCPP_WARN(this->get_logger(), "Size mismatch, resizing current ROI");
            cv::resize(current_roi, roi_to_process, last_roi_.size());
        }
        
        // Detect and match features
        std::vector<cv::KeyPoint> kp_prev, kp_cur;
        cv::Mat desc_prev, desc_cur;
        
        if (!detectFeatures(last_roi_, roi_to_process, kp_prev, kp_cur, desc_prev, desc_cur)) {
            RCLCPP_WARN(this->get_logger(), "Feature detection failed");
            last_roi_ = roi_to_process.clone();
            return;
        }
        
        std::vector<cv::DMatch> good_matches;
        if (!matchFeatures(desc_prev, desc_cur, good_matches)) {
            RCLCPP_WARN(this->get_logger(), "Feature matching failed");
            last_roi_ = roi_to_process.clone();
            return;
        }
        
        // Estimate transform
        double offset;
        if (!estimateTransform(kp_prev, kp_cur, good_matches, offset)) {
            RCLCPP_WARN(this->get_logger(), "Transform estimation failed");
            last_roi_ = roi_to_process.clone();
            return;
        }
        
        // Publish match visualization
        cv::Mat match_img;
        cv::drawMatches(last_roi_, kp_prev, roi_to_process, kp_cur, good_matches, match_img);
        cv_bridge::CvImage match_cv_image;
        match_cv_image.header = header;
        match_cv_image.encoding = "bgr8";
        match_cv_image.image = match_img;
        debug_matches_pub_->publish(*match_cv_image.toImageMsg());
        
        // Check shift validity
        int shift = static_cast<int>(std::round(offset));
        if (std::abs(shift) < min_shift_ || std::abs(shift) > max_shift_) {
            RCLCPP_WARN(this->get_logger(), "Shift %d out of valid range [%d, %d]", 
                       shift, min_shift_, max_shift_);
            last_roi_ = roi_to_process.clone();
            return;
        }
        
        // Perform stitching
        performStitching(roi_to_process, shift);
        
        // Update last ROI
        last_roi_ = roi_to_process.clone();
        
        // Publish stitched result
        publishStitchedImage(header);
    }
    
    bool detectFeatures(const cv::Mat& img1, const cv::Mat& img2,
                       std::vector<cv::KeyPoint>& kp1, std::vector<cv::KeyPoint>& kp2,
                       cv::Mat& desc1, cv::Mat& desc2)
    {
        // Convert to grayscale
        cv::Mat gray1, gray2;
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);
        
        // Apply histogram equalization
        cv::equalizeHist(gray1, gray1);
        cv::equalizeHist(gray2, gray2);
        
        // Detect and compute features
        orb_detector_->detectAndCompute(gray1, cv::noArray(), kp1, desc1);
        orb_detector_->detectAndCompute(gray2, cv::noArray(), kp2, desc2);
        
        RCLCPP_DEBUG(this->get_logger(), "Detected %zu and %zu keypoints", kp1.size(), kp2.size());
        
        return !desc1.empty() && !desc2.empty();
    }
    
    bool matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2,
                      std::vector<cv::DMatch>& good_matches)
    {
        if (desc1.empty() || desc2.empty()) {
            return false;
        }
        
        // BF matcher with KNN
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(desc1, desc2, knn_matches, 2);
        
        // Apply Lowe's ratio test
        good_matches.clear();
        for (const auto& m : knn_matches) {
            if (m.size() == 2 && m[0].distance < match_ratio_ * m[1].distance) {
                good_matches.push_back(m[0]);
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Found %zu good matches", good_matches.size());
        
        return good_matches.size() >= static_cast<size_t>(min_matches_);
    }
    
    bool estimateTransform(const std::vector<cv::KeyPoint>& kp1,
                          const std::vector<cv::KeyPoint>& kp2,
                          const std::vector<cv::DMatch>& matches,
                          double& offset)
    {
        if (matches.size() < static_cast<size_t>(min_matches_)) {
            return false;
        }
        
        // Extract matched points
        std::vector<cv::Point2f> pts1, pts2;
        for (const auto& match : matches) {
            pts1.push_back(kp1[match.queryIdx].pt);
            pts2.push_back(kp2[match.trainIdx].pt);
        }
        
        // Estimate affine transform using RANSAC
        cv::Mat inliers;
        cv::Mat H = cv::estimateAffinePartial2D(pts1, pts2, inliers, cv::RANSAC, 3.0);
        
        if (H.empty()) {
            return false;
        }
        
        // Extract offset
        offset = stitch_along_y_ ? H.at<double>(1, 2) : H.at<double>(0, 2);
        
        RCLCPP_DEBUG(this->get_logger(), "Estimated offset: %.2f (inliers: %d/%d)", 
                    offset, cv::countNonZero(inliers), static_cast<int>(matches.size()));
        
        return true;
    }
    
    void performStitching(const cv::Mat& current_roi, int shift)
    {
        int add_len = std::abs(shift);
        if (add_len == 0) return;
        
        cv::Mat strip;
        
        if (!stitch_along_y_) {
            // Horizontal stitching
            if (shift > 0) {
                strip = current_roi(cv::Rect(0, 0, add_len, current_roi.rows)).clone();
                cv::hconcat(strip, panorama_, panorama_);
            } else {
                strip = current_roi(cv::Rect(current_roi.cols - add_len, 0, add_len, current_roi.rows)).clone();
                cv::hconcat(panorama_, strip, panorama_);
            }
            
            // Crop if too wide
            if (panorama_.cols > max_width_) {
                int off = panorama_.cols - max_width_;
                panorama_ = panorama_(cv::Rect(off, 0, max_width_, panorama_.rows)).clone();
            }
        } else {
            // Vertical stitching
            if (panorama_.cols != current_roi.cols) {
                RCLCPP_WARN(this->get_logger(), "Column mismatch in vertical stitching, resetting");
                panorama_ = current_roi.clone();
                return;
            }
            
            if (shift > 0) {
                strip = current_roi(cv::Rect(0, 0, current_roi.cols, add_len)).clone();
                cv::vconcat(strip, panorama_, panorama_);
            } else {
                strip = current_roi(cv::Rect(0, current_roi.rows - add_len, current_roi.cols, add_len)).clone();
                cv::vconcat(panorama_, strip, panorama_);
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Stitched with shift: %d, panorama size: %dx%d", 
                    shift, panorama_.cols, panorama_.rows);
    }
    
    void publishStitchedImage(const std_msgs::msg::Header& header)
    {
        if (panorama_.empty()) return;
        
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = "bgr8";
        cv_image.image = panorama_;
        stitched_pub_->publish(*cv_image.toImageMsg());
    }
    
    void resetPanorama()
    {
        std::lock_guard<std::mutex> lock(panorama_mutex_);
        panorama_.release();
        last_roi_.release();
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stitched_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_roi_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_matches_pub_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    cv::Mat panorama_;
    cv::Mat last_roi_;
    std::mutex panorama_mutex_;
    
    cv::Ptr<cv::ORB> orb_detector_;
    
    // Parameters
    double roi_x_offset_ratio_, roi_y_offset_ratio_, roi_width_ratio_, roi_height_ratio_;
    int min_shift_, max_shift_, max_width_;
    bool auto_reset_, reset_now_, stitch_along_y_;
    int orb_features_, min_matches_;
    double match_ratio_;
    std::string input_topic_, output_topic_, debug_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageStitchNode>());
    rclcpp::shutdown();
    return 0;
} 