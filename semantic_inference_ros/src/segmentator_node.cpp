/*
 Author 2025 Oriol Martínez @fetty31

 */

#include "semantic_inference_ros/segmentator_node.hpp"

namespace semantic_inference_ros {

void declare_config(SegmentatorNode::Config& config) {
  using namespace config;
  name("SegmentatorNode::Config");
  field(config.segmenter, "segmenter");
  field(config.image_rotator, "image_rotator");
  field(config.recolor, "recolor");
  field(config.show_config, "show_config");
  field(config.publish_color, "publish_color");
  field(config.publish_overlay, "publish_overlay");
  field(config.publish_labels, "publish_labels");
  field(config.overlay_alpha, "overlay_alpha");
}

SegmentatorNode::SegmentatorNode(const rclcpp::NodeOptions& options)
            : Node("segmentator_node", options),
                config(config::fromCLI<Config>(options.arguments())),
                image_rotator_(config.image_rotator),
                image_recolor_(config.recolor)
{
    config::checkValid(config);

    if (config.show_config)
        std::cout << config::toString(config) << std::endl;

    std::cout << "SegmentatorNode::SegmentatorNode() config checked\n";
}

SegmentatorNode::~SegmentatorNode() = default;

void SegmentatorNode::init()
{
    try {
        segmenter_ = std::make_unique<semantic_inference::Segmenter>(config.segmenter);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "SEGMENTATOR_NODE:: Segmenter constructor exception: %s", e.what());
        throw e;
    }

    if (config.publish_labels) {
        labels_pub_ = this->create_publisher<semantic_inference_msgs::msg::Labels>("~/semantic/labels", 1);
    }

    if (config.publish_color) {
        color_pub_ = image_transport::create_publisher(this, "~/semantic_color/image_raw");
        
    }

    if (config.publish_overlay) {
        overlay_pub_ = image_transport::create_publisher(this, "~/semantic_overlay/image_raw");
    }

    segmented_pub_ = image_transport::create_publisher(this, "~/semantic/image_raw");

    sub_ = image_transport::create_subscription(this,
                                                "color/image_raw",
                                                std::bind(&SegmentatorNode::imageCallback, this, std::placeholders::_1),
                                                "raw"  // transport hint
                                                );
}

void SegmentatorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    runSegmentation(msg);
}

void SegmentatorNode::runSegmentation(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{

    auto start_time = std::chrono::system_clock::now();

    cv_bridge::CvImageConstPtr img_ptr;
    try {
        img_ptr = cv_bridge::toCvShare(msg, "rgb8");
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "SEGMENTATOR_NODE:: cv_bridge exception: %s", e.what());
        return;
    }

    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), 
            "Encoding: " << img_ptr->encoding << " size: " << img_ptr->image.cols
                << " x " << img_ptr->image.rows << " x " << img_ptr->image.channels()
                << " is right type? "
                << (img_ptr->image.type() == CV_8UC3 ? "yes" : "no")
            );

    auto r_start = std::chrono::system_clock::now();
    const auto rotated = image_rotator_.rotate(img_ptr->image);
    auto r_end = std::chrono::system_clock::now();

    auto m_start = std::chrono::system_clock::now();
    const auto result = segmenter_->infer(rotated);
    auto m_end = std::chrono::system_clock::now();

    if (!result) {
        RCLCPP_ERROR(this->get_logger(), "SEGMENTATOR_NODE:: failed to run inference!");
        return;
    }

    auto r2_start = std::chrono::system_clock::now();
    const auto derotated = image_rotator_.derotate(result.labels);
    auto r2_end = std::chrono::system_clock::now();

    auto p_start = std::chrono::system_clock::now();
    publish(img_ptr->header, derotated, img_ptr->image);
    auto p_end = std::chrono::system_clock::now();

    static std::chrono::duration<double> elapsed_time, m_elapsed_time, r_elapsed_time, r2_elapsed_time, p_elapsed_time;
    elapsed_time = p_end - start_time;
    m_elapsed_time = m_end - m_start;
    r_elapsed_time = r_end - r_start;
    r2_elapsed_time = r2_end - r2_start;
    p_elapsed_time = p_end - p_start;

    RCLCPP_INFO_STREAM(this->get_logger(), "Segmentation total time: " << elapsed_time.count()*1000.0 << " ms");
    RCLCPP_INFO_STREAM(this->get_logger(), "   only inference: " << m_elapsed_time.count()*1000.0 << " ms");
    RCLCPP_INFO_STREAM(this->get_logger(), "   only rotate: " << r_elapsed_time.count()*1000.0 << " ms");
    RCLCPP_INFO_STREAM(this->get_logger(), "   only derotate: " << r2_elapsed_time.count()*1000.0 << " ms");
    RCLCPP_INFO_STREAM(this->get_logger(), "   only output publish: " << p_elapsed_time.count()*1000.0 << " ms");
}

void SegmentatorNode::publish(const std_msgs::msg::Header& header,
                                const cv::Mat& labels,
                                const cv::Mat& color) 
{

    if (labels.empty() || color.empty()) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "SEGMENTATOR_NODE:: invalid inputs: color=" 
                            << std::boolalpha << !color.empty()
                            << ", labels=" << !labels.empty()
                            );
        return;
    }

    // Publish segmented image
    if (label_image_.image.empty()) {
        // we can't support 32SC1, so we do 16SC1 signed to distinguish from depth
        label_image_.encoding = "16SC1";
        label_image_.image = cv::Mat(color.rows, color.cols, CV_16SC1);
    }

    label_image_.header = header;
    image_recolor_.relabelImage(labels, label_image_.image);
    segmented_pub_.publish(label_image_.toImageMsg());

    if (!config.publish_color && !config.publish_overlay) {
        return;
    }

    // Publish colored image
    if (color_image_.image.empty()) {
        color_image_.encoding = "rgb8";
        color_image_.image = cv::Mat(color.rows, color.cols, CV_8UC3);
    }

    color_image_.header = header;
    image_recolor_.recolorImage(label_image_.image, color_image_.image);
    if (config.publish_color) {
        color_pub_.publish(color_image_.toImageMsg());
    }

    if (!config.publish_overlay) {
        return;
    }

    // Publish overlay image
    if (overlay_image_.image.empty()) {
        overlay_image_.encoding = "rgb8";
        overlay_image_.image = cv::Mat(color.rows, color.cols, CV_8UC3);
    }

    overlay_image_.header = header;
    cv::addWeighted(color_image_.image,
                    config.overlay_alpha,
                    color,
                    (1.0 - config.overlay_alpha),
                    0.0,
                    overlay_image_.image);
    overlay_pub_.publish(overlay_image_.toImageMsg());

    // Publish detected labels
    if(config.publish_labels)
        labels_pub_->publish(extractLabels());
}

semantic_inference_msgs::msg::Labels SegmentatorNode::extractLabels()
{
    semantic_inference_msgs::msg::Labels msg;

    using semantic_inference::Label; // int16_t
    static auto name_map = image_recolor_.getNameRemap(); // std::map<Label, std::string>
    std::unordered_set<std::string> name_set;

    for (int r = 0; r < label_image_.image.rows; ++r) {
        for (int c = 0; c < label_image_.image.cols; ++c) {
            const auto class_id = label_image_.image.at<Label>(r, c);

            const auto iter = name_map.find(class_id);
            if (iter != name_map.end()) {
                name_set.insert(iter->second.data());
            }else{
                name_set.insert("unknown");
            }
        }
    }

    for(std::string key : name_set){
        msg.labels.push_back(key);
    }

    return msg;
}

} // semantic_inference_ros

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    std::cout << "SegmentatorNode starting...\n";

    // semantic_inference_ros::SegmentatorNode::Config my_config = config::fromCLI<
    //                                                                 semantic_inference_ros::
    //                                                                     SegmentatorNode::Config>(argc, argv);

    // std::cout << "SegmentatorNode config loaded\n";

    // std::cout << config::toString(my_config) << std::endl;

    std::vector<std::string> args(argv + 1, argv + argc);
    rclcpp::NodeOptions options;
    options.arguments(args);

    auto seg = std::make_shared<semantic_inference_ros::SegmentatorNode>(options);

    std::cout << "SegementatorNode constructed\n";

    seg->init(); 

    std::cout << "SegementatorNode initialized\n";

    rclcpp::spin(seg);
    return 0;
}