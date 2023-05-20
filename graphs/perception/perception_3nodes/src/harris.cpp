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
#include "harris.hpp"
#include "xf_ocv_ref.hpp"

#define FILTER_WIDTH 3
#define BLOCK_WIDTH 3
#define NMS_RADIUS 1
#define MAXCORNERS 1024

namespace image_proc
{

HarrisNode::HarrisNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("HarrisNode", options)
{
  // Create image pub
  // pub_image_ = image_transport::create_camera_publisher(this, "resize");
  pub_image_ = image_transport::create_publisher(this, "harris");
  // Create image sub
  sub_image_ = image_transport::create_camera_subscription(
    this, "resize",
    std::bind(
      &HarrisNode::imageCb, this,
      std::placeholders::_1,
      std::placeholders::_2), "raw");

  myHarris_qualityLevel =
    this->declare_parameter<int>("myHarris_qualityLevel", 50);
  max_qualityLevel = this->declare_parameter<int>("max_qualityLevel", 100);
  blockSize_harris = this->declare_parameter<int>("blockSize_harris", 3);
  apertureSize = this->declare_parameter<int>("apertureSize", 3);
}

void HarrisNode::harrisImage(
    const cv::Mat& in_img,
    cv::Mat& harris_img) const
{
  cv::Mat img, img_gray, myHarris_dst;
  double myHarris_minVal, myHarris_maxVal;
  cv::RNG rng(12345);

  // gray scale the input image
  // cv_ptr->image.copyTo(img);
  cv::cvtColor(in_img, img_gray, cv::COLOR_BGR2GRAY);

  myHarris_dst = cv::Mat::zeros(img_gray.size(), CV_32FC(6));
  cv::Mat Mc = cv::Mat::zeros(img_gray.size(), CV_32FC1);

  cv::cornerEigenValsAndVecs(
    img_gray,
    myHarris_dst,
    blockSize_harris,
    apertureSize,
    cv::BORDER_DEFAULT);

  for (int j = 0; j < img_gray.rows; j++)
    for (int i = 0; i < img_gray.cols; i++) {
      float lambda_1 = myHarris_dst.at<cv::Vec6f>(j, i)[0];
      float lambda_2 = myHarris_dst.at<cv::Vec6f>(j, i)[1];
      Mc.at<float> (j, i) =
        lambda_1*lambda_2 - 0.04f*pow((lambda_1 + lambda_2), 2);
      }

  cv::minMaxLoc(Mc, &myHarris_minVal, &myHarris_maxVal, 0, 0, cv::Mat());

  // this->myHarris_function(img, img_gray);
  harris_img = in_img.clone();

  int new_harris_quality_level = myHarris_qualityLevel;
  try {
    this->get_parameter("myHarris_qualityLevel", new_harris_quality_level);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException& ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter value provided: %s", ex.what());
  }

  if (new_harris_quality_level < 1)
    new_harris_quality_level = 1;

  for (int j = 0; j < img_gray.rows; j++)
    for (int i = 0; i < img_gray.cols; i++)
      if (Mc.at<float>(j, i) > myHarris_minVal +
            (myHarris_maxVal - myHarris_minVal)
            * new_harris_quality_level/max_qualityLevel)
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

void HarrisNode::harrisImage2(
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

void HarrisNode::imageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  TRACEPOINT(
    image_proc_harris_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);    

  if (pub_image_.getNumSubscribers() < 1) {
    TRACEPOINT(
      image_proc_harris_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec);
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
      image_msg->header.stamp.sec);
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    TRACEPOINT(
      image_proc_harris_cb_fini,
      static_cast<const void *>(this),
      static_cast<const void *>(&(*image_msg)),
      static_cast<const void *>(&(*info_msg)),
      image_msg->header.stamp.nanosec,
      image_msg->header.stamp.sec);
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
    image_msg->header.stamp.sec);
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::HarrisNode)

