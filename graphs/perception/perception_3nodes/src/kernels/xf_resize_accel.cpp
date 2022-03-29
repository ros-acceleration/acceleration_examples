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
#include <vitis_common/imgproc/xf_resize.hpp>

/* resize kernel configuration */

#define RO 0  // Resource Optimized (8-pixel implementation)
#define NO 1  // Normal Operation (1-pixel implementation)

// port widths
#define INPUT_PTR_WIDTH 128
#define OUTPUT_PTR_WIDTH 128

// For Nearest Neighbor & Bilinear Interpolation, max down
// scale factor 2 for all 1-pixel modes, and for upscale in x direction
#define MAXDOWNSCALE 2

#define RGB 1
#define GRAY 0
/* Interpolation type*/
#define INTERPOLATION 1
// 0 - Nearest Neighbor Interpolation
// 1 - Bilinear Interpolation
// 2 - AREA Interpolation

/* Input image Dimensions */
#define WIDTH 640  // Maximum Input image width
#define HEIGHT 480  // Maximum Input image height

/* Output image Dimensions */
#define NEWWIDTH 1280  // Maximum output image width
#define NEWHEIGHT 960  // Maximum output image height

/* Interface types */
#if RO

#if RGB
#define NPC_T XF_NPPC4
#else
#define NPC_T XF_NPPC8
#endif

#else
#define NPC_T XF_NPPC1
#endif

#if RGB
#define TYPE XF_8UC3
#define CH_TYPE XF_RGB
#else
#define TYPE XF_8UC1
#define CH_TYPE XF_GRAY
#endif


#define DWIDTH 24
const int dwidth_tripcount = WIDTH/NPC_T;
const int height_tripcount = HEIGHT;

extern "C" {
    void resize_accel_streamlined(
                    hls::stream<ap_axiu<DWIDTH, 0, 0, 0>>& img_inp,
                    hls::stream<ap_axiu<DWIDTH, 0, 0, 0>>& img_out,
                    int rows_in,
                    int cols_in,
                    int rows_out,
                    int cols_out) {

        #pragma HLS INTERFACE s_axilite port=rows_in  // NOLINT
        #pragma HLS INTERFACE s_axilite port=cols_in  // NOLINT
        #pragma HLS INTERFACE s_axilite port=rows_out  // NOLINT
        #pragma HLS INTERFACE s_axilite port=cols_out  // NOLINT
        #pragma HLS INTERFACE s_axilite port=return  // NOLINT

        xf::cv::Mat<TYPE, HEIGHT, WIDTH, NPC_T> in_mat(rows_in, cols_in);
        xf::cv::Mat<
            TYPE, NEWHEIGHT, NEWWIDTH, NPC_T> out_mat(rows_out, cols_out);

        #pragma HLS stream variable=in_mat.data depth=2  // NOLINT
        #pragma HLS stream variable=out_mat.data depth=2  // NOLINT
        #pragma HLS DATAFLOW

        // fetch data from the stream
        auto writeindex = 0;
    Row_Loop_in:
        for (auto row = 0; row < HEIGHT; row++) {
    #pragma HLS LOOP_TRIPCOUNT min=height_tripcount max=height_tripcount  // NOLINT
    #pragma HLS LOOP_FLATTEN off
        Col_Loop_in:
            for (auto col = 0; col < WIDTH; col++) {
    #pragma HLS LOOP_TRIPCOUNT min=dwidth_tripcount max=dwidth_tripcount  // NOLINT
    #pragma HLS pipeline
                ap_axiu<DWIDTH, 0, 0, 0> aux;
                aux = img_inp.read();
                in_mat.write(writeindex++, aux.data);
            }
        }

        // resize operation
        xf::cv::resize<
            INTERPOLATION, TYPE, HEIGHT, WIDTH, NEWHEIGHT,
            NEWWIDTH, NPC_T, MAXDOWNSCALE>
                (in_mat, out_mat);

        // push data to the stream
        int readindex = 0;
    Row_Loop_out:
        for (auto row = 0; row < HEIGHT; row++) {
    #pragma HLS LOOP_TRIPCOUNT min=height_tripcount max=height_tripcount  // NOLINT
    #pragma HLS LOOP_FLATTEN off
        Col_Loop_out:
            for (auto col = 0; col < WIDTH; col++) {
    #pragma HLS LOOP_TRIPCOUNT min=dwidth_tripcount max=dwidth_tripcount  // NOLINT
    #pragma HLS pipeline
                ap_uint<DWIDTH> aux = out_mat.read(readindex++);
                ap_axiu<DWIDTH, 0, 0, 0> stream_out;
                stream_out.data = aux;
                img_out.write(stream_out);
            }
        }
        return;
    }
}
