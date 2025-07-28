/*
 Author 2025 Oriol Martínez @fetty31

 */

#pragma once

#include "semantic_inference/segmenter.h"
#include "semantic_inference/image_rotator.h"
#include "semantic_inference/image_recolor.h"

#include "semantic_inference_msgs/msg/labels.hpp"

#include <memory>
#include <chrono>

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/commandline.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/types/path.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace semantic_inference_ros {

using semantic_inference::Segmenter;
using semantic_inference::ImageRotator;
using semantic_inference::ImageRecolor;

class SegmentatorNode : public rclcpp::Node,
                        public std::enable_shared_from_this<SegmentatorNode>
{
    public:
        struct Config {
            Segmenter::Config segmenter;
            ImageRotator::Config image_rotator;
            ImageRecolor::Config recolor;
            bool show_config = true;
            bool show_output_config = false;
            bool publish_color = true;
            bool publish_overlay = true;
            bool publish_labels = true;
            double overlay_alpha = 0.4;
        } const config;

        SegmentatorNode(const rclcpp::NodeOptions& options);
        void init();

        ~SegmentatorNode();

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        
    private:

        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Publisher segmented_pub_;
        image_transport::Publisher color_pub_;
        image_transport::Publisher overlay_pub_;
        image_transport::Subscriber sub_;

        ImageRotator image_rotator_;
        ImageRecolor image_recolor_;

        cv_bridge::CvImage label_image_;
        cv_bridge::CvImage color_image_;
        cv_bridge::CvImage overlay_image_;
        
        std::unique_ptr<Segmenter> segmenter_;

        rclcpp::Publisher<semantic_inference_msgs::msg::Labels>::SharedPtr labels_pub_;
        
        void runSegmentation(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

        void publish(const std_msgs::msg::Header& header,
                    const cv::Mat& labels,
                    const cv::Mat& color);

        semantic_inference_msgs::msg::Labels extractLabels();
};

} // semantic_inference_ros