
#ifndef MINIMAL_IMAGE_PUBLISHER_HPP_
#define MINIMAL_IMAGE_PUBLISHER_HPP_



#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node 
{
public:
  MinimalImagePublisher() : Node("random_image_publisher"), count_(0)
  {
    	publisher_ 	= this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    	timer_ 		= this->create_wall_timer( 500ms, std::bind(&MinimalImagePublisher::TimerCallback, this));
    	m_image_name 	= "";
  }

  MinimalImagePublisher(std::string image_name) : Node("opencv_image_publisher"), count_(0) 
  {
	publisher_ 	= this->create_publisher<sensor_msgs::msg::Image>(image_name, 10);
	timer_ 		= this->create_wall_timer(500ms, std::bind(&MinimalImagePublisher::TimerCallback, this));
	m_image_name 	= image_name;
  }


private:

  cv::Mat GenerateRandomImage()
  {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(640, 480), CV_8UC3);

    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    return my_image;
  }


  cv::Mat LoadGrayScaleImage()
  {
  	std::string imagePath = m_image_name + ".png";
	cv::Mat grayscaleImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
	if (grayscaleImage.empty()) 
	{
        	std::cerr << "Error: Could not read the image at " << imagePath << std::endl;
    	}
	return grayscaleImage;
  }

  void TimerCallback() 
  {

	  if(m_image_name.empty())
	  {
		  cv::Mat my_image = this->GenerateRandomImage();
		  
		  // Write message to be sent. Member function toImageMsg() converts a CvImage
		  // into a ROS image message
		  msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
			  .toImageMsg();
	  }
	  else
	  {
		  cv::Mat my_image = LoadGrayScaleImage();
		  // Write message to be sent. Member function toImageMsg() converts a CvImage
		  // into a ROS image message
			
		  if(my_image.empty())
		  {
			  // NOt able to load image  
			  return;
		  }


		  msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", my_image)
			  .toImageMsg();


	  }

	  // Publish the image to the topic defined in the publisher
	  publisher_->publish(*msg_.get());

	  //print after every 20 images
	  if(count_ % 20 == 0)
          {
		  RCLCPP_INFO(this->get_logger(), "Image published, count = %ld ", count_);
          }
	  count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  std::string m_image_name;
};

#endif
