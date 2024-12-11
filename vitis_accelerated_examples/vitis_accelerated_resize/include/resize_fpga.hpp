/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2024, AMD®.
    \   \   \/    Author: Jasvinder Khurana <jasvinder.khurana@amd.com>
     \   \
     /   /        Licensed under the Apache License, Version 2.0 (the "License");
    /___/   /\    you may not use this file except in compliance with the License.
    \   \  /  \   You may obtain a copy of the License at
     \___\/\___\            http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

*/

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

#include <vitis_common/common/ros_opencl_120.hpp>


class AcceleratedResize  : public rclcpp::Node
{
public:
  AcceleratedResize(const rclcpp::NodeOptions& options);

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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
//  image_transport::CameraPublisher 	pub_image_;
//  image_transport::CameraSubscriber 	sub_image_;

  void connectCb();
  size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  size_t get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);


  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

//    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
//    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

private:

  cv::Mat  		result_hls;
  cv_bridge::CvImage 	output_image;
  cv_bridge::CvImagePtr cv_ptr;
  void InitKernel();
  void ExecuteKernel(bool gray, int src_width, int src_height, int dst_width, int dst_height);
};


#endif  // IMAGE_PROC_RESIZE_FPGA_HPP_
