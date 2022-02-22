# perception_2nodes

A simple perception computational graph composed by 2 Nodes, rectify and resize operations. Used to demonstrate the value of accelerating image_pipeline computational graphs.

Launch files and rationale below, after nomenclature:

- `integrated`[^1]: "Integrated" components include ROS functionality into a single ROS Component.
- `streamlined`: Streamlined components use separated kernels (at least 2) and a stream interface between them to pass the data from one to the other.

[^1]: `integrated` can also be used in the context of accelerators, in which case it refers to various kernels smashed into one single kernel that uses streams between the various functions inside of it.

|  Launch file name | Rationale | Accelerator | Accel. container |
|-------------------|-----------|-------------|--------|
| `trace_rectify.launch.py` | Simple CPU `RectifyNode` launched  in  **1 ROS Component** container| | |
| `trace_rectify.launch.py` | FPGA-offloaded `RectifyNodeFPGA` launched in **1 ROS Component, 1 kernel** container| `resize_accel` | `resize_accel` |
| `trace_resize_fpga.launch.py` | Simple CPU `RectifyNode` launched  in **1 ROS Component** container| | |
| `trace_resize_fpga.launch.py` | FPGA-offloaded `RectifyNodeFPGA` launched in **1 ROS Component, 1 kernel** container| `resize_accel` | `resize_accel` |
||||
| `trace_rectify_resize.launch.py` | CPU pipeline, **2 ROS Components** | | |
| `trace_rectify_resize_fpga.launch.py` | FPGA-offloaded pipeline, **2 ROS Components, 2 kernels** | `resize_accel`, `rectify_accel` | `image_proc` |
||||
| `trace_rectify_resize_fpga_integrated.launch.py` | FPGA-offloaded pipeline, **1 ROS Component, 1 kernel** | `rectify_resize_accel` | `image_proc_integrated` |
| `trace_rectify_resize_fpga_integrated_node.launch.py` | FPGA-offloaded pipeline, **1 ROS Node, 1 kernel** | `rectify_resize_accel` | `image_proc_integrated` |
| `trace_rectify_resize_fpga_integrated_xrt.launch.py` | FPGA-offloaded pipeline, **1 ROS Component, 1 kernel**, with XRT | `rectify_resize_accel` | `image_proc_integrated` |
||||
| `trace_rectify_resize_fpga_streamlined.launch.py`  | FPGA-offloaded pipeline, **2 ROS Components, 2 kernels using stream interfaces**. *Note: to connect the pipeline, the second function, resize, subscribes also to the raw image topic, but doesn't use it (fetches data from accelerator).*. |`resize_accel_streamlined`,  `rectify_accel_streamlined` | `image_proc_streamlined` |
| `trace_rectify_resize_fpga_streamlined_xrt.launch.py` | FPGA-offloaded pipeline, **2 ROS Components, 2 kernels using stream interfaces**, with XRT. *Note: to connect the pipeline, the second function, resize, subscribes also to the raw image topic, but doesn't use it (fetches data from accelerator).*. | `resize_accel_streamlined`, `rectify_accel_streamlined` | `image_proc_streamlined` |
||||
| :x: `trace_rectify_resize_fpga_integrated_streamlined.launch.py` | FPGA-offloaded pipeline, **1 ROS Component, 2 kernels using stream interfaces** |`resize_accel_streamlined`,  `rectify_accel_streamlined` | `image_proc_streamlined` |
| `trace_rectify_resize_fpga_integrated_streamlined_xrt.launch.py` | FPGA-offloaded pipeline, **1 ROS Component, 2 kernels using stream interfaces**, with XRT |`resize_accel_streamlined`,  `rectify_accel_streamlined` | `image_proc_streamlined` |