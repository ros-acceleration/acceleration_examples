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

#include "hls_stream.h"
#include "ap_axi_sdata.h"
#include "ap_int.h"

#include <vitis_common/common/xf_common.hpp>
#include <vitis_common/common/xf_utility.hpp>
#include <vitis_common/imgproc/xf_remap.hpp>

#define XF_INTERPOLATION_TYPE XF_INTERPOLATION_BILINEAR  // interpolation type
#define XF_USE_URAM false  // Enable to map some structures
                           // to UltraRAM instead of BRAM
#define PTR_IMG_WIDTH 32  // if RGB
#define TYPE XF_8UC3
#define CHANNELS 3

#define TYPE_XY XF_32FC1
#define PTR_MAP_WIDTH 32
#define NPC XF_NPPC1  // Number of pixels to be processed per cycle
#define XF_WIN_ROWS 50  // Number of input image rows to be buffered inside

#define HEIGHT 480  // 480 with default RealSense drivers
#define WIDTH 640  // 640 with default RealSense drivers

#define XF_CAMERA_MATRIX_SIZE 9
#define XF_DIST_COEFF_SIZE 5

#define DWIDTH 24
const int dwidth_tripcount = WIDTH/NPC;
const int height_tripcount = HEIGHT;


extern "C" {

    void rectify_accel_streamlined(
        ap_uint<PTR_IMG_WIDTH>* img_in,
        float* map_x,
        float* map_y,
        hls::stream<ap_axiu<DWIDTH, 0, 0, 0>>& img_out,
        int rows,
        int cols) {

        #pragma HLS INTERFACE m_axi      port=img_in        offset=slave  bundle=gmem0  // NOLINT
        #pragma HLS INTERFACE m_axi      port=map_x         offset=slave  bundle=gmem1  // NOLINT
        #pragma HLS INTERFACE m_axi      port=map_y         offset=slave  bundle=gmem2  // NOLINT
        #pragma HLS INTERFACE s_axilite  port=rows  // NOLINT
        #pragma HLS INTERFACE s_axilite  port=cols  // NOLINT
        #pragma HLS INTERFACE s_axilite  port=return  // NOLINT

        // Get arguments for remap operation (remap)
        xf::cv::Mat<TYPE, HEIGHT, WIDTH, NPC> imgInput(rows, cols);
        xf::cv::Mat<TYPE_XY, HEIGHT, WIDTH, NPC> mapX(rows, cols);
        xf::cv::Mat<TYPE_XY, HEIGHT, WIDTH, NPC> mapY(rows, cols);
        xf::cv::Mat<TYPE, HEIGHT, WIDTH, NPC> imgOutput(rows, cols);

        const int HEIGHT_WIDTH_LOOPCOUNT =
            HEIGHT * WIDTH / XF_NPIXPERCYCLE(NPC);
        for (unsigned int i = 0; i < rows * cols; ++i) {
            #pragma HLS LOOP_TRIPCOUNT min=1 max=HEIGHT_WIDTH_LOOPCOUNT  // NOLINT
            #pragma HLS PIPELINE II=1  // NOLINT

            float map_x_val = map_x[i];
            float map_y_val = map_y[i];
            mapX.write_float(i, map_x_val);
            mapY.write_float(i, map_y_val);
        }

        #pragma HLS STREAM variable=imgInput.data depth=2  // NOLINT
        #pragma HLS STREAM variable=mapX.data depth=2  // NOLINT
        #pragma HLS STREAM variable=mapY.data depth=2  // NOLINT
        #pragma HLS STREAM variable=imgOutput.data depth=2  // NOLINT

        #pragma HLS DATAFLOW

        // Retrieve xf::cv::Mat objects from img_in data:
        xf::cv::Array2xfMat<PTR_IMG_WIDTH, TYPE, HEIGHT, WIDTH, NPC>(img_in, imgInput);

        // // obtain a mapping between the distorted image and an undistorted one
        // xf::cv::InitUndistortRectifyMapInverse<XF_CAMERA_MATRIX_SIZE, XF_DIST_COEFF_SIZE, XF_32FC1, HEIGHT, WIDTH,
        //                                  XF_NPPC1>(K_binned_fix, distC_l_fix, irA_l_fix, mapxLMat, mapyLMat,
        //                                            _cm_size, _dc_size);

        // remap accordingly
        xf::cv::remap<XF_WIN_ROWS, XF_INTERPOLATION_TYPE, TYPE,
                    TYPE_XY, TYPE, HEIGHT, WIDTH, NPC, XF_USE_URAM>(
            imgInput, imgOutput, mapX, mapY);

        int readindex = 0;
    Row_Loop:
        for (auto row = 0; row < HEIGHT; row++) {
    #pragma HLS LOOP_TRIPCOUNT min=height_tripcount max=height_tripcount  // NOLINT
    #pragma HLS LOOP_FLATTEN off
        Col_Loop:
            for (auto col = 0; col < WIDTH; col++) {
    #pragma HLS LOOP_TRIPCOUNT min=dwidth_tripcount max=dwidth_tripcount  // NOLINT
    #pragma HLS pipeline
                ap_uint<DWIDTH> aux = imgOutput.read(readindex++);
                ap_axiu<DWIDTH, 0, 0, 0> stream_out;
                stream_out.data = aux;
                img_out.write(stream_out);
            }
        }

        return;
    }  // End of kernel
}  // End of extern C
