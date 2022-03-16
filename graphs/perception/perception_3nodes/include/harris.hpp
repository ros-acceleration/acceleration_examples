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

#ifndef IMAGE_PROC__HARRIS_HPP_
#define IMAGE_PROC__HARRIS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

namespace image_proc
{

class HarrisNode
  : public rclcpp::Node
{
public:
  explicit HarrisNode(const rclcpp::NodeOptions &);

protected:
  // image_transport::CameraPublisher pub_image_;
  image_transport::Publisher pub_image_;
  image_transport::CameraSubscriber sub_image_;

  int myHarris_qualityLevel;
  int max_qualityLevel;
  int blockSize_harris;
  int apertureSize;

  cv::Mat myHarris_copy, Mc;
  double myHarris_minVal, myHarris_maxVal;

  std::mutex connect_mutex_;

  void imageCb(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
};

}  // namespace image_proc

#endif  // IMAGE_PROC__HARRIS_HPP_

