// Copyright 2022 Víctor Mayoral-Vilches
// All rights reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <mutex>
#include <vector>

#include "tracetools_image_pipeline/tracetools.h"
#include "harris_fpga.hpp"
#include <vitis_common/common/xf_headers.hpp>
#include "hls_stream.h"
#include "ap_int.h"
#include <vitis_common/common/xf_common.hpp>
#include <vitis_common/common/xf_utility.hpp>
#include <vitis_common/features/xf_harris.hpp>
#include <vitis_common/common/utilities.hpp>
#include "xf_ocv_ref.hpp"
#include <rclcpp/serialization.hpp>


#define FILTER_WIDTH 3
#define BLOCK_WIDTH 3
#define NMS_RADIUS 1
#define MAXCORNERS 1024
#define WIDTH 1280
#define HEIGHT 960

namespace perception_3nodes
{

HarrisNodeFPGA::HarrisNodeFPGA(const rclcpp::NodeOptions & options)
: rclcpp::Node("HarrisNodeFPGA", options)
{
  // Create image pub
  // pub_image_ = image_transport::create_camera_publisher(this, "resize");
  pub_image_ = image_transport::create_publisher(this, "harris");
  // Create image sub
  sub_image_ = image_transport::create_camera_subscription(
    this, "image",
    std::bind(
      &HarrisNodeFPGA::imageCb, this,
      std::placeholders::_1,
      std::placeholders::_2), "raw");

  myHarris_qualityLevel =
    this->declare_parameter<int>("myHarris_qualityLevel", 50);
  max_qualityLevel = this->declare_parameter<int>("max_qualityLevel", 100);
  blockSize_harris = this->declare_parameter<int>("blockSize_harris", 3);
  apertureSize = this->declare_parameter<int>("apertureSize", 3);

  cl_int err;
  unsigned fileBufSize;

  // OpenCL section to prepare hardware acceleration kernel
  std::vector<cl::Device> devices = get_xilinx_devices();
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
  OCL_CHECK(err, krnl_ = new cl::Kernel(program, "cornerHarris_accel", &err));  
}

void HarrisNodeFPGA::harrisImage_fpga(
    const cv::Mat& in_img,
    cv::Mat& harris_img) const
{
  cv::Mat img_gray;
  cv::Mat hls_out_img, hls_out_raw_img;
  cv::Mat hlspnts;
  cv::RNG rng(12345);

  uint16_t Thresh;  // Threshold for HLS
  float Th;
  if (FILTER_WIDTH == 3) {
      Th = 30532960.00;
      Thresh = 442;
  } else if (FILTER_WIDTH == 5) {
      Th = 902753878016.0;
      Thresh = 3109;
  } else if (FILTER_WIDTH == 7) {
      Th = 41151168289701888.000000;
      Thresh = 566;
  }

  // Convert rgb into grayscale
  cvtColor(in_img, img_gray, CV_BGR2GRAY);
  hls_out_img.create(HEIGHT, WIDTH, CV_8U); // create memory for hls output image  // NOLINT
  hls_out_raw_img.create(HEIGHT, WIDTH, CV_8U); // create memory for hls output image  // NOLINT  

  float K = 0.04;
  uint16_t k = K * (1 << 16);  // Convert to Q0.16 format
  uint32_t nCorners = 0;
  uint16_t imgwidth = in_img.cols;
  uint16_t imgheight = in_img.rows;

  // OpenCL section:
  // size_t image_in_size_bytes = in_img.rows * in_img.cols * sizeof(unsigned char);
  // size_t image_out_size_bytes = in_img.rows * in_img.cols * sizeof(unsigned char);
  size_t image_out_size_bytes = HEIGHT * WIDTH * sizeof(unsigned char);
  size_t image_out_raw_size_bytes = HEIGHT * WIDTH * sizeof(unsigned char);

  cl_int err;
  std::cout << "INFO: Running OpenCL section." << std::endl;

  // Initialize the buffers:
  cl::Event event;

  std::vector<cl::Memory> inBufVec, outBufVec;

  // Allocate the buffers:
  // OCL_CHECK(err, cl::Buffer imageToDevice(*context_, CL_MEM_READ_ONLY, image_in_size_bytes, NULL, &err));
  OCL_CHECK(err, cl::Buffer imageFromDevice(*context_, CL_MEM_WRITE_ONLY, image_out_size_bytes, NULL, &err));
  OCL_CHECK(err, cl::Buffer imageFromDevice_raw(*context_, CL_MEM_WRITE_ONLY, image_out_raw_size_bytes, NULL, &err));

  // OCL_CHECK(err,
  //           q.enqueueWriteBuffer(imageToDevice,       // buffer on the FPGA
  //                                 CL_TRUE,             // blocking call
  //                                 0,                   // buffer offset in bytes
  //                                 image_in_size_bytes, // Size in bytes
  //                                 in_img.data,         // Pointer to the data to copy
  //                                 nullptr, &event));

  // Set the kernel arguments
  // krnl_->setArg(0, imageToDevice);
  krnl_->setArg(1, imageFromDevice);
  krnl_->setArg(2, imageFromDevice_raw);
  krnl_->setArg(3, (int) HEIGHT);
  krnl_->setArg(4, (int)in_img.cols);
  krnl_->setArg(5, (int)Thresh);
  krnl_->setArg(6, (int)k);

  // // Profiling Objects
  // cl_ulong start = 0;
  // cl_ulong end = 0;
  // double diff_prof = 0.0f;
  
  cl::Event event_sp;

  // Launch the kernel
  queue_->enqueueTask(*krnl_, NULL, &event_sp);
  clWaitForEvents(1, (const cl_event*)&event_sp);

  // event_sp.getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
  // event_sp.getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
  // diff_prof = end - start;
  // std::cout << (diff_prof / 1000000) << "ms" << std::endl;

  queue_->enqueueReadBuffer(imageFromDevice, // This buffers data will be read
                      CL_TRUE,         // blocking call
                      0,               // offset
                      image_out_size_bytes,
                      hls_out_img.data, // Data will be stored here
                      nullptr, &event);

  queue_->enqueueReadBuffer(imageFromDevice_raw, // This buffers data will be read
                      CL_TRUE,         // blocking call
                      0,               // offset
                      image_out_raw_size_bytes,
                      hls_out_raw_img.data, // Data will be stored here
                      nullptr, &event);

  queue_->finish();

  // // Mark with circles in resulting image
  // harris_img = in_img.clone();
  harris_img = hls_out_raw_img;

  /// Drawing a circle around corners
  for (int j = 1; j < hls_out_img.rows - 1; j++) {
      for (int i = 1; i < hls_out_img.cols - 1; i++) {
          if ((int) hls_out_img.at<unsigned char>(j, i)) {
            cv::circle(
              harris_img,
              cv::Point(i, j),
              4,
              cv::Scalar(
                rng.uniform(0, 255),
                rng.uniform(0, 255),
                rng.uniform(0, 255)),
              -1, 8, 0);
          }
      }
  }
}

void HarrisNodeFPGA::harrisImage2(
    const cv::Mat& in_img,
    cv::Mat& harris_img) const
{
  cv::Mat ocv_out_img, img_gray;
  cv::RNG rng(12345);

  uint16_t Thresh;  // Threshold for HLS
  float Th;
  if (FILTER_WIDTH == 3) {
      Th = 30532960.00;
      Thresh = 442;
  } else if (FILTER_WIDTH == 5) {
      Th = 902753878016.0;
      Thresh = 3109;
  } else if (FILTER_WIDTH == 7) {
      Th = 41151168289701888.000000;
      Thresh = 566;
  }
  
  // Convert rgb into grayscale
  cvtColor(in_img, img_gray, CV_BGR2GRAY);

  // hls_out_img.create(in_img.rows, in_img.cols, CV_8U); // create memory for hls output image  // NOLINT
  ocv_out_img.create(img_gray.rows, img_gray.cols, CV_8U); // create memory for opencv output image  // NOLINT  
  ocv_ref(img_gray, ocv_out_img, Th);
  harris_img = in_img.clone();

  /// Drawing a circle around corners
  for (int j = 1; j < ocv_out_img.rows - 1; j++) {
      for (int i = 1; i < ocv_out_img.cols - 1; i++) {
          if ((int) ocv_out_img.at<unsigned char>(j, i)) {
            cv::circle(
              harris_img,
              cv::Point(i, j),
              4,
              cv::Scalar(
                rng.uniform(0, 255),
                rng.uniform(0, 255),
                rng.uniform(0, 255)),
              -1, 8, 0);
          }
      }
  }
}

size_t HarrisNodeFPGA::get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg){
  //Serialize the Image and CameraInfo messages
  rclcpp::SerializedMessage serialized_data_img;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
  const void* image_ptr = reinterpret_cast<const void*>(image_msg.get());
  image_serialization.serialize_message(image_ptr, &serialized_data_img);
  size_t image_msg_size = serialized_data_img.size();
  return image_msg_size;
}

size_t HarrisNodeFPGA::get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg){
  rclcpp::SerializedMessage serialized_data_info;
  rclcpp::Serialization<sensor_msgs::msg::CameraInfo> info_serialization;
  const void* info_ptr = reinterpret_cast<const void*>(info_msg.get());
  info_serialization.serialize_message(info_ptr, &serialized_data_info);
  size_t info_msg_size = serialized_data_info.size();
  return info_msg_size;
}


void HarrisNodeFPGA::imageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  TRACEPOINT(
    image_proc_harris_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec,
    get_msg_size(image_msg),
    get_msg_size(info_msg));

  if (pub_image_.getNumSubscribers() < 1) {
    TRACEPOINT(
      image_proc_harris_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      get_msg_size(info_msg));
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg);
  } catch (cv_bridge::Exception & e) {
    TRACEPOINT(
      image_proc_harris_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      get_msg_size(info_msg));
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    TRACEPOINT(
      image_proc_harris_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec,
      get_msg_size(image_msg),
      get_msg_size(info_msg));
    return;
  }

  TRACEPOINT(
    image_proc_harris_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);

  cv::Mat in_img, ocv_out_img;
  cv_ptr->image.copyTo(in_img);
  // harrisImage(in_img, ocv_out_img);
  harrisImage2(in_img, ocv_out_img);

  TRACEPOINT(
    image_proc_harris_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);

  // Allocate new rectified image message
  sensor_msgs::msg::Image::SharedPtr harris_msg = cv_bridge::CvImage(
      image_msg->header,
      image_msg->encoding,
      ocv_out_img).toImageMsg();

  pub_image_.publish(harris_msg);


  TRACEPOINT(
    image_proc_harris_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec,
    get_msg_size(image_msg),
    get_msg_size(info_msg));
}

}  // namespace perception_3nodes

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception_3nodes::HarrisNodeFPGA)

