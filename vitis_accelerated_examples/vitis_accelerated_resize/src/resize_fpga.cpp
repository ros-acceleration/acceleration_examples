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


#if ROS_VITIS
#include <vitis_common/common/xf_headers.hpp>
#include <vitis_common/common/utilities.hpp>
#include "image_proc/xf_resize_config.h"
#endif

#include "tracetools_image_pipeline/tracetools.h"
#include <rclcpp/serialization.hpp>
#include "resize_fpga.hpp"


// Forward declaration of utility functions included at the end of this file
std::vector<cl::Device> get_xilinx_devices();
char* read_binary_file(const std::string &xclbin_file_name, unsigned &nb);


#define XCLBIN_NAME "/lib/firmware/xilinx/vitis_accelerated_resize/resize_accel.xclbin"
#define KERNEL_NAME "resize_accel"

void AcceleratedResize::InitKernel()
{
	cl_int err;
	unsigned fileBufSize;

	// Get the device:
	std::vector<cl::Device> devices 	= get_xilinx_devices();
	cl::Device device 			= devices[0];


	// Context, command queue and device name:
	OCL_CHECK(err, context_ 		= new cl::Context(device, NULL, NULL, NULL, &err));
	OCL_CHECK(err, queue_ 			= new cl::CommandQueue(*context_, device, CL_QUEUE_PROFILING_ENABLE, &err));
	OCL_CHECK(err, std::string device_name 	= device.getInfo<CL_DEVICE_NAME>(&err));
	
	std::cout << "INFO: Device found - " << device_name << std::endl;

	char* fileBuf 				= read_binary_file(XCLBIN_NAME, fileBufSize);

	cl::Program::Binaries bins{{fileBuf, fileBufSize}};
	devices.resize(1);
	OCL_CHECK(err, cl::Program program(*context_, devices, bins, NULL, &err));

	// Create a kernel:
	OCL_CHECK(err, krnl_ 			= new cl::Kernel(program, KERNEL_NAME, &err));

}




void AcceleratedResize::ExecuteKernel(bool gray, int src_width, int src_height, int dst_width,int dst_height)
{
	
	// OpenCL section:
	cl_int err;
	size_t image_in_size_bytes, image_out_size_bytes;

	if (gray) 
	{
		result_hls.create(cv::Size(dst_width, dst_height), CV_8UC1);
		image_in_size_bytes 	= src_height * src_width * 1 * sizeof(unsigned char);
		image_out_size_bytes 	= dst_height * dst_width * 1 * sizeof(unsigned char);
	} 
	else 
	{
		result_hls.create(cv::Size(dst_width,	dst_height), CV_8UC3);
		image_in_size_bytes 	= src_height * src_width * 3 * sizeof(unsigned char);
		image_out_size_bytes 	= dst_height * dst_width * 3 * sizeof(unsigned char);
	}

	// Allocate the buffers:
	OCL_CHECK(err, cl::Buffer imageToDevice(*context_, CL_MEM_READ_ONLY, image_in_size_bytes, NULL, &err));
	OCL_CHECK(err, cl::Buffer imageFromDevice(*context_, CL_MEM_WRITE_ONLY,	image_out_size_bytes, NULL, &err));

	if(krnl_ == NULL)
		std::cout << "kernel is null, please check if it is properly loaded in fpga  " << std::endl;
	// Set the kernel arguments
	OCL_CHECK(err, err = krnl_->setArg(0, imageToDevice));
	OCL_CHECK(err, err = krnl_->setArg(1, imageFromDevice));
	OCL_CHECK(err, err = krnl_->setArg(2, src_height));
	OCL_CHECK(err, err = krnl_->setArg(3, src_width));
	OCL_CHECK(err, err = krnl_->setArg(4, dst_height));
	OCL_CHECK(err, err = krnl_->setArg(5, dst_width));



		

	/* Copy input vectors to memory */
	OCL_CHECK(err,	queue_->enqueueWriteBuffer(imageToDevice,     	// buffer on the FPGA
			CL_TRUE,                 			// blocking call
			0,                       			// buffer offset in bytes
			image_in_size_bytes,     			// Size in bytes
			cv_ptr->image.data));    			// Pointer to the data to copy



	// Profiling Objects
	// cl_ulong start = 0;
	// cl_ulong end = 0;
	// double diff_prof = 0.0f;
	cl::Event event_sp;

	// Execute the kernel:
	OCL_CHECK(err, err = queue_->enqueueTask(*krnl_, NULL, &event_sp));

	// // Profiling Objects
	// if (profile_) {
	//   clWaitForEvents(1, (const cl_event*)&event_sp);
	//   event_sp.getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
	//   event_sp.getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
	//   diff_prof = end - start;
	//   std::cout << "FPGA: " << (diff_prof / 1000000) << "ms" << std::endl;
	// }

	// Copying Device result data to Host memory
	OCL_CHECK(err, queue_->enqueueReadBuffer(
			imageFromDevice,   			// This buffers data will be read
			CL_TRUE,           			// blocking call
			0,                 			// offset
			image_out_size_bytes,
			result_hls.data));  			// data will be stored here


	output_image.header 	= cv_ptr->header;
	output_image.encoding 	= cv_ptr->encoding;

	if (gray) 
	{
		output_image.image = cv::Mat{ dst_height,dst_width, CV_8UC1, result_hls.data};
	} else 
	{
		output_image.image = cv::Mat{ dst_height,dst_width, CV_8UC3, result_hls.data};
	}

	queue_->finish();

}


AcceleratedResize::AcceleratedResize(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: rclcpp::Node("AcceleratedResize", options)
{

	interpolation_ 		= this->declare_parameter("interpolation", 1);
	use_scale_ 		= this->declare_parameter("use_scale", true);
	scale_height_ 		= this->declare_parameter("scale_height", 1.0);
	scale_width_ 		= this->declare_parameter("scale_width", 1.0);
	height_ 		= this->declare_parameter("height", -1);
	width_ 			= this->declare_parameter("width", -1);
	profile_ 		= this->declare_parameter("profile", true);


	// Create image pub
	publisher_ 		= this->create_publisher<sensor_msgs::msg::Image>("resize", 10); 
	// Create image sub
	subscriber_ 		= this->create_subscription<sensor_msgs::msg::Image>("random_image", 10, std::bind(&AcceleratedResize::imageCb, this, std::placeholders::_1));		

	InitKernel();
}

void AcceleratedResize::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg )
{


	// Get subscribed image
	//-------------------------------------------------------------------------------------------------------
	

	// Converting ROS image messages to OpenCV images, for diggestion
	// with the Vitis Vision Library
	// see http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

	//std::cout << "INside imageCb function " << std::endl;

	bool gray = (sensor_msgs::image_encodings::numChannels(image_msg->encoding) == 1);
	try 
	{
		cv_ptr = cv_bridge::toCvCopy(image_msg);
	} 
	catch (cv_bridge::Exception & e) 
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}



	// Process Image and run the accelerated Kernel
	//-------------------------------------------------------------------------------------------------------

	//this->get_parameter("profile", profile_);  // Update profile_

	ExecuteKernel(gray, 640, 480, 320,240);
	//ExecuteKernel(gray, info_msg->width, info_msg->height, dst_info_msg->width,dst_info_msg->height);

 	sensor_msgs::msg::Image::SharedPtr msg_ = output_image.toImageMsg();

	// Publish processed image
	//-------------------------------------------------------------------------------------------------------
	publisher_->publish(*msg_.get());
	//publisher_->publish(*image_msg.get());

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(AcceleratedResize)
