/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /
    /___/   /\
    \   \  /  \
     \___\/\___\

An Adaptive Component built using using an adaptive_component ROS 2 package.

Publishes vadd operations while supporting different compute substrates (CPU,
FPGA), allowing for on-the-go changes following adaptive_component conventions.
*/

#include "rclcpp/rclcpp.hpp"
#include "doublevadd_component.hpp"
#include "doublevadd_component_fpga.hpp"
#include "doublevadd_component_adaptive_stateful.hpp"

using namespace std::chrono_literals;  // NOLINT
using NodeCPU = composition::DoubleVaddComponent;
using NodeFPGA = composition::DoubleVaddComponentFPGA;

namespace composition
{
DoubleVaddComponentAdaptiveStateful::DoubleVaddComponentAdaptiveStateful(const rclcpp::NodeOptions & options)
      : AdaptiveComponent("doublevadd_publisher_adaptive",        // name of the adaptive Node
      options,                                // Node options
                                              // CPU
      std::make_shared<NodeCPU>("_doublevadd_publisher_adaptive_cpu", options),
                                              // FPGA
      std::make_shared<NodeCPU>("_doublevadd_publisher_adaptive_fpga", options),
                                              // GPU
      nullptr)
{
  timer_ = create_wall_timer(1s, std::bind(&DoubleVaddComponentAdaptiveStateful::on_timer, this));
}

void
DoubleVaddComponentAdaptiveStateful::on_timer()
{
  try{
    int new_adaptive_value_;
    this->get_parameter("adaptive", new_adaptive_value_);

    if (new_adaptive_value_ != adaptive_value_) {  // if there's change

      // Remove nodes
      if (adaptive_value_ == Hardware::FPGA) {
        if (fpga_node_) {
          exec_.remove_node(fpga_node_);
        }
      } else if (adaptive_value_ == Hardware::CPU) {
        if (cpu_node_) {
          exec_.remove_node(cpu_node_);
        }
      } else if (adaptive_value_ == Hardware::GPU) {
        if (gpu_node_) {
          exec_.remove_node(gpu_node_);
        }
      }

      // // TODO: Consider instead a different approach with mutexes if
      // // appropriate. See https://stackoverflow.com/a/25655803
      //
      // // Cancel/stop spin-ing executor
      // exec_.cancel();  // stop ongoing activities
      //                  // this will stop the associated thread as well
      //                  //
      //                  // TODO: review this carefully as it
      //                  // leads to seg fault with the middleware
      //                  // if there are pending tasks and the Node has been removed
      //                  // from the executor

      // Back up state
      if (adaptive_value_ == Hardware::FPGA) {
        if (fpga_node_) {
          count = std::static_pointer_cast<DoubleVaddComponentFPGA>(fpga_node_)->publish_count_;
        }
      } else if (adaptive_value_ == Hardware::CPU) {
        if (cpu_node_) {
          count = std::static_pointer_cast<DoubleVaddComponent>(cpu_node_)->publish_count_;
        }
      } else if (adaptive_value_ == Hardware::GPU) {
        if (gpu_node_) {
          // Define a GPU class above, if appropriate - not applicable in this example
        }
      }
      RCLCPP_INFO(this->get_logger(), "Saved state: %d (will restart from here).", count);

      // Reset state in new Node
      if (new_adaptive_value_ == Hardware::FPGA) {
        if (fpga_node_) {
          std::static_pointer_cast<DoubleVaddComponent>(fpga_node_)->publish_count_ = count;  // restore value in new Node
        }
      } else if (new_adaptive_value_ == Hardware::CPU) {
        if (cpu_node_) {
          std::static_pointer_cast<DoubleVaddComponent>(cpu_node_)->publish_count_ = count;  // restore value in new Node
        }
      } else if (new_adaptive_value_ == Hardware::GPU) {
        if (gpu_node_) {
          // Define a GPU class above, if appropriate - not applicable in this example
        }
      }

      // Add new Nodes to executor
      if (new_adaptive_value_ == Hardware::FPGA) {
        if (fpga_node_) {
          exec_.add_node(fpga_node_);
        }
        else
          RCLCPP_ERROR(this->get_logger(), "No FPGA Node available for computations.");
      } else if (new_adaptive_value_ == Hardware::CPU) {
        if (cpu_node_) {
          exec_.add_node(cpu_node_);
        }
        else
          RCLCPP_ERROR(this->get_logger(), "No CPU Node available for computations.");
      } else if (new_adaptive_value_ == Hardware::GPU) {
        if (gpu_node_) {
          exec_.add_node(gpu_node_);
        }
        else
          RCLCPP_ERROR(this->get_logger(), "No GPU Node available for computations.");

      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Invalid new 'adaptive' parameter value: %d", new_adaptive_value_);
      }

      // // TODO: More synthetic implementation
      // //    re-engage if compute options start exploding,
      // //    needs to properly consider bounds and avoid nullptr issues.
      // //
      //
      // // Back up state, ensure DoubleVaddComponentFPGA is a subclass of DoubleVaddComponent
      // count = std::static_pointer_cast<DoubleVaddComponent>(compute_resources_[adaptive_value_])->publish_count_;
      //
      // // Remove nodes
      // if (compute_resources_[adaptive_value_])
      //   exec_.remove_node(compute_resources_[adaptive_value_]);
      //
      // // Reset state in new Node
      // std::static_pointer_cast<DoubleVaddComponent>(compute_resources_[new_adaptive_value_])->publish_count_ = count;
      //
      // // Add new Nodes to executor
      // if (compute_resources_[new_adaptive_value_])
      //   exec_.add_node(compute_resources_[new_adaptive_value_]);
      // else
      //   RCLCPP_ERROR(this->get_logger(),
      //     "No Node available for computational resource: %d (%s)",
      //     new_adaptive_value_,
      //     compute_resources_names_[new_adaptive_value_]
      //   );

      adaptive_value_ = new_adaptive_value_;

      // // TODO: Consider instead a different approach with mutexes if
      // // appropriate. See https://stackoverflow.com/a/25655803
      //
      // // re-spin internal executor in a detached thread
      // std::thread (&DoubleVaddComponentAdaptiveStateful::spin, this).detach();
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException& ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter value provided: %s", ex.what());
  }

}  // on_timer

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::DoubleVaddComponentAdaptiveStateful)
