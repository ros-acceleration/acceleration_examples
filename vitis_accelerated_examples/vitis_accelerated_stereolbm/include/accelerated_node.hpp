
#ifndef IMAGE_PROC_RESIZE_FPGA_HPP_
#define IMAGE_PROC_RESIZE_FPGA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <vitis_common/common/ros_opencl_120.hpp>
#include <message_filters/subscriber.h>

class AcceleratedNode  : public rclcpp::Node
{
public:
  AcceleratedNode(const rclcpp::NodeOptions& options);

protected:

  int 			interpolation_;
  bool 			use_scale_;
  bool 			profile_;
  double 		scale_height_;
  double 		scale_width_;
  int 			height_;
  int 			width_;

  cl::Kernel* 		krnl_;
  cl::Context* 		context_;
  cl::CommandQueue* 	queue_;
  std::mutex 		connect_mutex_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;


 // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_left;
 // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_right;
 
//  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriber_left;
 // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subscriber_right;
 
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_left;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_right;
 
 

  void connectCb();
  size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  size_t get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

  void imageCbLeft(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void imageCbRight(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void imageCbSync(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

private:

  cv_bridge::CvImagePtr cv_ptr_left;		//Stores input image from img_msg in  subscriber callback
  cv_bridge::CvImagePtr cv_ptr_right;		//Stores input image from img_msg in  subscriber callback
  cv::Mat  		result_hls;	// stores result after kernel execution on FPGA
  cv::Mat  		result_hls_8u;	// stores 8 bit result after kernel execution on FPGA
  cv_bridge::CvImage 	output_image; 	// Create CV image from result_hls, required to publish image msg.
  
  void InitKernel();
  void ExecuteKernel();
};


#endif  // IMAGE_PROC_RESIZE_FPGA_HPP_
