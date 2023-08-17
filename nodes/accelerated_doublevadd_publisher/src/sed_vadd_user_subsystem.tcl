###################################################################################
# 
# Copyright (C) 2023 Acceleration Robotics
#
# Derived from the original work of 2022 Intel Corporation.
# 
# This software and the related documents are Intel copyrighted materials, and 
# your use of them is governed by the express license under which they were 
# provided to you ("License"). Unless the License provides otherwise, you may 
# not use, modify, copy, publish, distribute, disclose or transmit this software 
# or the related documents without Intel's prior written permission.
# 
# This software and the related documents are provided as is, with no express 
# or implied warranties, other than those that are expressly stated in the License.
###################################################################################

# create script specific parameters and default values

# N/A

set_shell_parameter ROS2_SDT_PACKAGE_SRC_FILE "/tmp/ros2_sdr_package_src"
set_shell_parameter ROS2_FIRMWARE_PATH "/tmp/ros2_firmware_path"
set_shell_parameter KRNL_NAME_PATH "/tmp/krnl_name"
set_shell_parameter AVMM_HOST {auto}

# define the procedures used by the create_subsystems_qsys.tcl script

proc pre_creation_step {} {
  puts "DEBUG: transfer_files() called"
  transfer_files
  puts "DEBUG: build_kernel() called"
  build_kernel
}

proc creation_step {} {
  create_custom_subsystem
}

proc post_creation_step {} {
  edit_top_level_qsys
  add_auto_connections
}

proc build_kernel {} {
  set v_subsystem_ip_path       [get_shell_parameter SUBSYSTEM_IP_PATH]
  set v_subsystem_ip_path_user  [get_shell_parameter SUBSYSTEM_IP_PATH_USER]  
  set v_target_device           [get_shell_parameter DEVICE]
  set v_kernel_name_path        [get_shell_parameter KRNL_NAME_PATH]

  # kernel name
  if {[file exists $v_kernel_name_path]} {
      set fileId [open $v_kernel_name_path "r"]
      set v_kernel_name [read $fileId]
      close $fileId
  } else {
      puts "File $v_kernel_name_path does not exist"
      exit
  }
  set v_kernel_name [string trim $v_kernel_name]

  # Set up the CMake infrastructure
  set init_dir [pwd]
  cd $v_subsystem_ip_path_user
  exec mv $v_subsystem_ip_path_user/CMakeLists.txt $v_subsystem_ip_path_user/CMakeLists.txt.ros
  exec mv $v_subsystem_ip_path_user/../../CMakeLists.txt $v_subsystem_ip_path_user/CMakeLists.txt
  file delete -force build
  file mkdir build
  cd build

  # Ubuntu setting
  set ld_preload_paths "export LD_PRELOAD=\"/usr/lib/x86_64-linux-gnu/libstdc++.so.6 /usr/lib/x86_64-linux-gnu/libcurl.so.4\"; "
  # # Internal ARC setting
  # set ld_preload_paths ""

set cmd $ld_preload_paths
  append cmd "cmake .. -DFPGA_DEVICE=$v_target_device"
  set result_failed [catch {exec sh -c "$cmd"}  result_text]
  puts $cmd
  puts "result_failed -> $result_failed"  

  set cmd $ld_preload_paths
  append cmd "make fpga_ip_export"
  set result_failed [catch {exec sh -c "$cmd"}  result_text]
  puts $cmd
  puts "result_failed -> $result_failed"

  cd $v_kernel_name.fpga_ip_export.prj
  exec python ${v_kernel_name}_fpga_ip_export_di_hw_tcl_adjustment_script.py

  cd $init_dir
}

#==========================================================

# copy files from the shell install directory to the target project directory

# copy files from the shell install directory to the target project directory

proc transfer_files {} {

  set v_subsystem_source_file   [get_shell_parameter ROS2_SDT_PACKAGE_SRC_FILE]
  set v_subsystem_ip_path       [get_shell_parameter SUBSYSTEM_IP_PATH]
  set v_project_path            [get_shell_parameter PROJECT_PATH]
  set v_firmware_path           [get_shell_parameter ROS2_FIRMWARE_PATH]
  set v_subsystem_ip_path_user  "$v_project_path/non_qpds_ip/user"
  
  set_shell_parameter SUBSYSTEM_IP_PATH_USER "$v_project_path/non_qpds_ip/user"

  # package source code
  if {[file exists $v_subsystem_source_file]} {
      set fileId [open $v_subsystem_source_file "r"]
      set v_subsystem_source_path [read $fileId]
      close $fileId
  } else {
      puts "File $v_subsystem_source_file does not exist"
      exit
  }
  set v_subsystem_source_path [string trim $v_subsystem_source_path]

  # firmware path
  if {[file exists $v_firmware_path]} {
      set fileId [open $v_firmware_path "r"]
      set v_subsystem_firmware_path [read $fileId]
      close $fileId
  } else {
      puts "File $v_firmware_path does not exist"
      exit
  }
  set v_subsystem_firmware_path [string trim $v_subsystem_firmware_path]

  puts "v_project_path () -> $v_project_path"
  puts "v_subsystem_ip_path () -> $v_subsystem_ip_path"
  puts "v_subsystem_ip_path_user () -> $v_subsystem_ip_path_user"
  puts "v_subsystem_source_path -> $v_subsystem_source_path"
  puts "v_subsystem_firmware_path -> $v_subsystem_firmware_path"

  # NOTE: $v_subsystem_ip_path -> $v_project_path/non_qpds_ip/shell
  #  but need to copy things to $v_project_path/non_qpds_ip/user since
  #  the ROS 2 subsystem should operate at that level

  # NOTE: file_copy does not seem to work for directories within qsys-script (probably due to version differences)
  #       hence use the command cp instead as a temporary workaround TODO
  exec cp -rf $v_subsystem_source_path/.         $v_subsystem_ip_path_user
  
  # puts "transfer_file () -> $v_project_path/scripts/ext/"
  file mkdir  $v_project_path/scripts/ext
  exec cp -rf $v_subsystem_firmware_path/boot/hps_debug.ihex        $v_project_path/scripts/ext/hps_debug.ihex
  exec cp -rf $v_subsystem_firmware_path/boot/u-boot-spl-dtb.hex    $v_project_path/scripts/ext/u-boot-spl-dtb.hex
  # exec cp -rf $v_firmware_path/sadd_sample.sh        $v_project_path/scripts/ext/sadd_sample.sh
  # exec cp -rf $v_subsystem_source_path/sdimage.tar.gz        $v_project_path/scripts/ext/sdimage.tar.gz
}


#==========================================================

# create the custom subsystem and leave empty

proc create_custom_subsystem {} {

  set v_project_path  [get_shell_parameter PROJECT_PATH]
  set v_instance_name [get_shell_parameter INSTANCE_NAME]
  # set v_kernel_name   "fpga_template"
  set v_kernel_name_path        [get_shell_parameter KRNL_NAME_PATH]

  # kernel name
  if {[file exists $v_kernel_name_path]} {
      set fileId [open $v_kernel_name_path "r"]
      set v_kernel_name [read $fileId]
      close $fileId
  } else {
      puts "File $v_kernel_name_path does not exist"
      exit
  }
  set v_kernel_name [string trim $v_kernel_name]

  create_system $v_instance_name
  save_system   $v_project_path/rtl/user/$v_instance_name.qsys

  load_system   $v_project_path/rtl/user/$v_instance_name.qsys

  add_instance  cpu_clk_bridge              altera_clock_bridge
  add_instance  cpu_rst_bridge              altera_reset_controller
  add_instance  cpu_mm_bridge               altera_avalon_mm_bridge
  add_instance  vadd_kernel                 ${v_kernel_name}_fpga_ip_export_di
  add_instance  kernel_mem                  intel_onchip_memory
  add_instance  user_sysid                  altera_avalon_sysid_qsys


  #----------------------------------------------------------------
  # cpu_clk_bridge
  set_instance_parameter_value  cpu_clk_bridge  EXPLICIT_CLOCK_RATE     {100000000.0}

  set_instance_parameter_value  cpu_rst_bridge   NUM_RESET_INPUTS        1
  set_instance_parameter_value  cpu_rst_bridge   MIN_RST_ASSERTION_TIME  {3}
  set_instance_parameter_value  cpu_rst_bridge   OUTPUT_RESET_SYNC_EDGES {deassert}
  set_instance_parameter_value  cpu_rst_bridge   SYNC_DEPTH              {2}	
  set_instance_parameter_value  cpu_rst_bridge   RESET_REQUEST_PRESENT   {0}
  set_instance_parameter_value  cpu_rst_bridge   USE_RESET_REQUEST_INPUT {0}

  #----------------------------------------------------------------
  # cpu_mm_bridge

  set_instance_parameter_value cpu_mm_bridge SYNC_RESET             {0}
  set_instance_parameter_value cpu_mm_bridge DATA_WIDTH             {64}
  set_instance_parameter_value cpu_mm_bridge SYMBOL_WIDTH           {8}
  set_instance_parameter_value cpu_mm_bridge ADDRESS_WIDTH          {0}
  set_instance_parameter_value cpu_mm_bridge USE_AUTO_ADDRESS_WIDTH {1}
  set_instance_parameter_value cpu_mm_bridge ADDRESS_UNITS          {SYMBOLS}
  set_instance_parameter_value cpu_mm_bridge MAX_BURST_SIZE         {1}
  set_instance_parameter_value cpu_mm_bridge LINEWRAPBURSTS         {0}
  set_instance_parameter_value cpu_mm_bridge MAX_PENDING_RESPONSES  {4}
  set_instance_parameter_value cpu_mm_bridge MAX_PENDING_WRITES     {0}
  set_instance_parameter_value cpu_mm_bridge PIPELINE_COMMAND       {1}
  set_instance_parameter_value cpu_mm_bridge PIPELINE_RESPONSE      {1}
  set_instance_parameter_value cpu_mm_bridge USE_RESPONSE           {0}
  set_instance_parameter_value cpu_mm_bridge USE_WRITERESPONSE      {0}

  #----------------------------------------------------------------
  # kernel_mem
  set_instance_parameter_value kernel_mem   AXI_interface                1
  set_instance_parameter_value kernel_mem   interfaceType                0
  set_instance_parameter_value kernel_mem   idWidth                      2
  set_instance_parameter_value kernel_mem   memorySize                   {300000.0}
  set_instance_parameter_value kernel_mem   initMemContent               1
  set_instance_parameter_value kernel_mem   useNonDefaultInitFile        0
  set_instance_parameter_value kernel_mem   initializationFileName       ""
  set_instance_parameter_value kernel_mem   dataWidth                    32

  #---------------------------------------------------------------
  # user_sysid
  set_instance_parameter_value user_sysid    id     0x00020230
  

  # create internal subsystem connections

  add_connection  cpu_clk_bridge.out_clk    cpu_rst_bridge.clk
  add_connection  cpu_clk_bridge.out_clk    cpu_mm_bridge.clk
  add_connection  cpu_clk_bridge.out_clk    vadd_kernel.clock
  # This 2x clock output is removed from the generated IP by oneAPI-IP-Authoring flow starting from 2023.1
  add_connection  cpu_clk_bridge.out_clk    vadd_kernel.clock2x
  add_connection  cpu_clk_bridge.out_clk    kernel_mem.clk1
  add_connection  cpu_clk_bridge.out_clk    user_sysid.clk

  add_connection  cpu_rst_bridge.reset_out   cpu_mm_bridge.reset
  add_connection  cpu_rst_bridge.reset_out   kernel_mem.reset1
  add_connection  cpu_rst_bridge.reset_out   user_sysid.reset
  add_connection  cpu_rst_bridge.reset_out   vadd_kernel.resetn


  add_connection  vadd_kernel.avm_mem_gmem0_0_port_0_0_rw   kernel_mem.s1
  add_connection  cpu_mm_bridge.m0                          vadd_kernel.csr_ring_root_avs

  add_connection  cpu_mm_bridge.m0                          kernel_mem.s1
  add_connection  cpu_mm_bridge.m0                          user_sysid.control_slave

  # add interfaces to the boundary of the subsystem

  add_interface           i_cpu_clk               clock       sink
  set_interface_property  i_cpu_clk               export_of   cpu_clk_bridge.in_clk

  add_interface           i_cpu_rst               reset       sink
  set_interface_property  i_cpu_rst               export_of   cpu_rst_bridge.reset_in0

  add_interface           c_cpu_mm_ctrl_in         avalon      agent
  set_interface_property  c_cpu_mm_ctrl_in         export_of   cpu_mm_bridge.s0

  add_interface           ${v_instance_name}_device_exception_bus      conduit     end
  set_interface_property  ${v_instance_name}_device_exception_bus      export_of   vadd_kernel.device_exception_bus

  add_interface           ${v_instance_name}_kernel_irqs      conduit     end
  set_interface_property  ${v_instance_name}_kernel_irqs      export_of   vadd_kernel.kernel_irqs

  sync_sysinfo_parameters
  save_system

}

# insert the user subsystem into the top level qsys system

proc edit_top_level_qsys {} {

  set v_project_name  [get_shell_parameter PROJECT_NAME]
  set v_project_path  [get_shell_parameter PROJECT_PATH]
  set v_instance_name [get_shell_parameter INSTANCE_NAME]

  load_system $v_project_path/rtl/${v_project_name}_qsys.qsys

  add_instance $v_instance_name $v_instance_name

  # create top level connections

  # add interfaces to the boundary of the subsystem

  sync_sysinfo_parameters
  save_system

}

# enable a subset of subsystem interfaces to be available for auto-connection
# to other subsystems at the top qsys level

proc add_auto_connections {} {

  set v_instance_name [get_shell_parameter INSTANCE_NAME]
  set v_avmm_host     [get_shell_parameter AVMM_HOST]

  add_auto_connection $v_instance_name i_cpu_clk 100000000

  add_auto_connection $v_instance_name i_cpu_rst 100000000

  if {($v_avmm_host != "NONE") && ($v_avmm_host != "")} {
  	add_auto_connection $v_instance_name c_cpu_mm_ctrl_in ${v_avmm_host}_avmm_host
  }

}
