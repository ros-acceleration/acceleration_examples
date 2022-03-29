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
#include "ap_int.h"
#include <vitis_common/common/xf_common.hpp>
#include <vitis_common/common/xf_utility.hpp>
#include <vitis_common/features/xf_harris.hpp>

#define INPUT_PTR_WIDTH 64
#define OUTPUT_PTR_WIDTH 64
#define RO 0
#define NO 1

#define FILTER_WIDTH 3
#define BLOCK_WIDTH 3
#define NMS_RADIUS 1
#define MAXCORNERS 1024
#define XF_USE_URAM false

#define CH_TYPE XF_GRAY  // TODO: figure out what's this for
#define INPUT_PTR_WIDTH 64
#define OUTPUT_PTR_WIDTH 64

#define WIDTH 1280
#define HEIGHT 960

#if RO
#define NPIX XF_NPPC8
#endif
#if NO
#define NPIX XF_NPPC1
#endif

#define IMGSIZE WIDTH* HEIGHT

#if RO
#define IN_TYPE ap_uint<64>
#endif
#if NO
#define IN_TYPE ap_uint<8>
#endif


#define DWIDTH 24
const int dwidth_tripcount = WIDTH/NPIX;
const int height_tripcount = HEIGHT;

extern "C" {
void cornerHarris_accel(
        hls::stream<ap_axiu<DWIDTH, 0, 0, 0>>& img_inp,
        ap_uint<OUTPUT_PTR_WIDTH>* img_out,
        ap_uint<OUTPUT_PTR_WIDTH>* img_out_raw,
        int rows,
        int cols,
        int threshold,
        int k) {

    #pragma HLS INTERFACE m_axi port=img_out  offset=slave bundle=gmem2  // NOLINT
    #pragma HLS INTERFACE s_axilite port=rows  // NOLINT
    #pragma HLS INTERFACE s_axilite port=cols  // NOLINT
    #pragma HLS INTERFACE s_axilite port=threshold  // NOLINT
    #pragma HLS INTERFACE s_axilite port=k  // NOLINT
    #pragma HLS INTERFACE s_axilite port=return  // NOLINT

    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, NPIX> in_mat(rows, cols);
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, NPIX> out_mat(rows, cols);
    #pragma HLS stream variable=in_mat.data depth=2  // NOLINT
    #pragma HLS stream variable=out_mat.data depth=2  // NOLINT

    #pragma HLS DATAFLOW  // NOLINT

    auto writeindex = 0;
Row_Loop:
    for (auto row = 0; row < HEIGHT; row++) {
#pragma HLS LOOP_TRIPCOUNT min=height_tripcount max=height_tripcount  // NOLINT
#pragma HLS LOOP_FLATTEN off
    Col_Loop:
        for (auto col = 0; col < WIDTH; col++) {
#pragma HLS LOOP_TRIPCOUNT min=dwidth_tripcount max=dwidth_tripcount  // NOLINT
#pragma HLS pipeline
            ap_axiu<DWIDTH, 0, 0, 0> aux;
            aux = img_inp.read();
            in_mat.write(writeindex++, aux.data);
        }
    }

    // Duplicate the stream data
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, NPIX> in_mat_1(rows, cols);
    xf::cv::Mat<XF_8UC1, HEIGHT, WIDTH, NPIX> in_mat_2(rows, cols);
    #pragma HLS stream variable=in_mat_1.data depth=2  // NOLINT
    #pragma HLS stream variable=in_mat_2.data depth=2  // NOLINT


    xf::cv::cornerHarris<
        FILTER_WIDTH, BLOCK_WIDTH, NMS_RADIUS, XF_8UC1,
        HEIGHT, WIDTH, NPIX, XF_USE_URAM>
            (in_mat_1, out_mat, threshold, k);

    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_8UC1, HEIGHT, WIDTH, NPIX>
        (out_mat, img_out);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_8UC1, HEIGHT, WIDTH, NPIX>
        (in_mat_2, img_out_raw);
    return;
}
}
