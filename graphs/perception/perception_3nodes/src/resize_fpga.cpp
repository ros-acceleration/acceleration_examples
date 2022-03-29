/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
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

    Inspired by resize.cpp authored by Kentaro Wada, Joshua Whitley
*/

#include <memory>
#include <mutex>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vitis_common/common/xf_headers.hpp>
#include <vitis_common/common/utilities.hpp>

#include "image_proc/xf_resize_config.h"
#include "resize_fpga.hpp"
#include "tracetools_image_pipeline/tracetools.h"


// // Forward declaration of utility functions included at the end of this file
// std::vector<cl::Device> get_xilinx_devices();
// char* read_binary_file(const std::string &xclbin_file_name, unsigned &nb);

namespace perception_3nodes
{

ResizeNodeFPGAStreamlined::ResizeNodeFPGAStreamlined(const rclcpp::NodeOptions & options)
: rclcpp::Node("ResizeNodeFPGAStreamlined", options)
{
  // Create image pub
  pub_image_ = image_transport::create_camera_publisher(this, "resize");
  
  // Create image sub
  sub_image_ = image_transport::create_camera_subscription(
    this, "image",
    std::bind(
      &ResizeNodeFPGAStreamlined::imageCb, this,
      std::placeholders::_1,
      std::placeholders::_2), "raw");

  interpolation_ = this->declare_parameter("interpolation", 1);
  use_scale_ = this->declare_parameter("use_scale", true);
  scale_height_ = this->declare_parameter("scale_height", 1.0);
  scale_width_ = this->declare_parameter("scale_width", 1.0);
  height_ = this->declare_parameter("height", -1);
  width_ = this->declare_parameter("width", -1);
  profile_ = this->declare_parameter("profile", true);

  cl_int err;
  unsigned fileBufSize;

  // Get the device:
  std::vector<cl::Device> devices = get_xilinx_devices();
  // devices.resize(1);  // done below
  cl::Device device = devices[0];

  // Context, command queue and device name:
  OCL_CHECK(err, context_ = new cl::Context(device, NULL, NULL, NULL, &err));
  OCL_CHECK(err, queue_ = new cl::CommandQueue(*context_, device,
                                    CL_QUEUE_PROFILING_ENABLE, &err));
  OCL_CHECK(err, std::string device_name =
                                  device.getInfo<CL_DEVICE_NAME>(&err));

  std::cout << "INFO: Device found - " << device_name << std::endl;

  // Load binary:
  // NOTE: hardcoded path according to dfx-mgrd conventions
  // TODO: generalize this using launch extra_args for composable Nodes
  // see https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/descriptions/composable_node.py#L45
  char* fileBuf = read_binary_file(
        "/lib/firmware/xilinx/perception_3nodes/perception_3nodes.xclbin",
        fileBufSize);
  cl::Program::Binaries bins{{fileBuf, fileBufSize}};
  devices.resize(1);
  OCL_CHECK(err, cl::Program program(*context_, devices, bins, NULL, &err));

  // Create a kernel:
  OCL_CHECK(err, krnl_ = new cl::Kernel(program, "resize_accel_streamlined", &err));
}

void ResizeNodeFPGAStreamlined::imageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  // std::cout << "ResizeNodeFPGAStreamlined::imageCb" << std::endl;
  TRACEPOINT(
    image_proc_resize_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));

  if (pub_image_.getNumSubscribers() < 1) {
    TRACEPOINT(
      image_proc_resize_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)));
    return;
  }

  this->get_parameter("profile", profile_);  // Update profile_

  // Converting ROS image messages to OpenCV images, for diggestion
  // with the Vitis Vision Library
  // see http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
  cv::Mat img, result_hls;
  cv_bridge::CvImagePtr cv_ptr;
  bool gray =
    (sensor_msgs::image_encodings::numChannels(image_msg->encoding) == 1);

  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    TRACEPOINT(
      image_proc_resize_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)));
    return;
  }

  // Prepare output CameraInfo with desired dimensions
  sensor_msgs::msg::CameraInfo::SharedPtr dst_info_msg =
    std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);

  double scale_y;
  double scale_x;

  if (use_scale_) {
    scale_y = scale_height_;
    scale_x = scale_width_;
    dst_info_msg->height = static_cast<int>(info_msg->height * scale_height_);
    dst_info_msg->width = static_cast<int>(info_msg->width * scale_width_);
  } else {
    scale_y = static_cast<double>(height_) / info_msg->height;
    scale_x = static_cast<double>(width_) / info_msg->width;
    dst_info_msg->height = height_;
    dst_info_msg->width = width_;
  }

  // Rescale the relevant entries of the intrinsic and extrinsic matrices
  dst_info_msg->k[0] = dst_info_msg->k[0] * scale_x;  // fx
  dst_info_msg->k[2] = dst_info_msg->k[2] * scale_x;  // cx
  dst_info_msg->k[4] = dst_info_msg->k[4] * scale_y;  // fy
  dst_info_msg->k[5] = dst_info_msg->k[5] * scale_y;  // cy

  dst_info_msg->p[0] = dst_info_msg->p[0] * scale_x;  // fx
  dst_info_msg->p[2] = dst_info_msg->p[2] * scale_x;  // cx
  dst_info_msg->p[3] = dst_info_msg->p[3] * scale_x;  // T
  dst_info_msg->p[5] = dst_info_msg->p[5] * scale_y;  // fy
  dst_info_msg->p[6] = dst_info_msg->p[6] * scale_y;  // cy

  TRACEPOINT(
    image_proc_resize_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));
  // OpenCL section:
  cl_int err;
  size_t image_in_size_bytes, image_out_size_bytes;

  if (gray) {
    result_hls.create(cv::Size(dst_info_msg->width,
                                dst_info_msg->height), CV_8UC1);
    image_in_size_bytes = info_msg->height * info_msg->width *
                                  1 * sizeof(unsigned char);
    image_out_size_bytes = dst_info_msg->height * dst_info_msg->width *
                                  1 * sizeof(unsigned char);
  } else {
    result_hls.create(cv::Size(dst_info_msg->width,
                                dst_info_msg->height), CV_8UC3);
    image_in_size_bytes = info_msg->height * info_msg->width *
                                  3 * sizeof(unsigned char);
    image_out_size_bytes = dst_info_msg->height * dst_info_msg->width *
                                  3 * sizeof(unsigned char);
  }

  // Allocate the buffers:
  // OCL_CHECK(err, cl::Buffer imageToDevice(*context_, CL_MEM_READ_ONLY,
  //                                         image_in_size_bytes, NULL, &err));
  // OCL_CHECK(err, cl::Buffer imageFromDevice(*context_, CL_MEM_WRITE_ONLY,
  //                                           image_out_size_bytes, NULL, &err));

  // Set the kernel arguments
  // OCL_CHECK(err, err = krnl_->setArg(0, imageToDevice));
  // OCL_CHECK(err, err = krnl_->setArg(1, imageFromDevice));
  OCL_CHECK(err, err = krnl_->setArg(2, info_msg->height));
  OCL_CHECK(err, err = krnl_->setArg(3, info_msg->width));
  OCL_CHECK(err, err = krnl_->setArg(4, dst_info_msg->height));
  OCL_CHECK(err, err = krnl_->setArg(5, dst_info_msg->width));

  // /* Copy input vectors to memory */
  // OCL_CHECK(err,
  //   queue_->enqueueWriteBuffer(imageToDevice,     // buffer on the FPGA
  //                        CL_TRUE,                 // blocking call
  //                        0,                       // buffer offset in bytes
  //                        image_in_size_bytes,     // Size in bytes
  //                        cv_ptr->image.data));    // Pointer to the data to copy

  cl::Event event_sp;

  // Execute the kernel:
  OCL_CHECK(err, err = queue_->enqueueTask(*krnl_, NULL, &event_sp));

  // Copying Device result data to Host memory
  // OCL_CHECK(err,
  //     queue_->enqueueReadBuffer(
  //                         imageFromDevice,   // This buffers data will be read
  //                         CL_TRUE,           // blocking call
  //                         0,                 // offset
  //                         image_out_size_bytes,
  //                         result_hls.data));  // data will be stored here
  queue_->finish();


  // // Set the output image
  // cv_bridge::CvImage output_image;
  // output_image.header = cv_ptr->header;
  // output_image.encoding = cv_ptr->encoding;
  // if (gray) {
  //   output_image.image =
  //         cv::Mat{
  //             static_cast<int>(dst_info_msg->height),
  //             static_cast<int>(dst_info_msg->width),
  //             CV_8UC1,
  //             result_hls.data
  //         };
  // } else {
  //   output_image.image =
  //         cv::Mat{
  //             static_cast<int>(dst_info_msg->height),
  //             static_cast<int>(dst_info_msg->width),
  //             CV_8UC3,
  //             result_hls.data
  //         };
  // }


  TRACEPOINT(
    image_proc_resize_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));

  // pub_image_.publish(*output_image.toImageMsg(), *dst_info_msg);

  TRACEPOINT(
    image_proc_resize_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));

}

}  // namespace perception_3nodes

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception_3nodes::ResizeNodeFPGAStreamlined)
