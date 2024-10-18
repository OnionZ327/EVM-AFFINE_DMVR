/* ====================================================================================================================

  The copyright in this software is being made available under the License included below.
  This software may be subject to other third party and contributor rights, including patent rights, and no such
  rights are granted under this license.

  Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
  Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
  Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
  Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, are permitted only for
  the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
  promoting such standards. The following conditions are required to be met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
      the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
      the following disclaimer in the documentation and/or other materials provided with the distribution.
    * The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. may not be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* ====================================================================================================================
*/

#include "com_ComAdaptiveLoopFilter.h"

int tbl_weights_shape1_sym[ALF_MAX_NUM_COEF + 1] =
{
    2,
    2,
    2, 2, 2,
    2, 2, 2, 1,
    1
};
#if ALF_SHAPE
int tbl_weights_shape2_sym[ALF_MAX_NUM_COEF_SHAPE2 + 1] =
{
    2,
    2, 2, 2, 2, 2,
    2, 2, 2, 2, 2,
    2, 2, 2, 1,
    1
};
#endif

void reconstruct_coefficients(ALF_PARAM *alf_param, int **filter_coeff)
{
#if ALF_SHIFT
    const int alf_shift_enable = alf_param->num_coeff > ALF_MAX_NUM_COEF_SHAPE2;
#endif
    int g, sum, i, coeff_pred;
    for (g = 0; g < alf_param->filters_per_group; g++)
    {
        sum = 0;
#if ALF_SHIFT
        for (i = 0; i < alf_param->num_coeff - 1 - alf_shift_enable; i++)
#else
        for (i = 0; i < alf_param->num_coeff - 1; i++)
#endif
        {
            sum += (2 * alf_param->coeff_multi[g][i]);
            filter_coeff[g][i] = alf_param->coeff_multi[g][i];
        }
#if ALF_SHIFT
        if (alf_shift_enable)
        {
            coeff_pred = (1 << (ALF_NUM_BIT_SHIFT + alf_param->coeff_multi[g][alf_param->num_coeff - 1])) - sum;
            filter_coeff[g][alf_param->num_coeff - 2] = coeff_pred + alf_param->coeff_multi[g][alf_param->num_coeff - 2];
            filter_coeff[g][alf_param->num_coeff - 1] = alf_param->coeff_multi[g][alf_param->num_coeff - 1];
        }
        else
        {
#endif
        coeff_pred = (1 << ALF_NUM_BIT_SHIFT) - sum;
        filter_coeff[g][alf_param->num_coeff - 1] = coeff_pred + alf_param->coeff_multi[g][alf_param->num_coeff - 1];
#if ALF_SHIFT
        }
#endif
    }
}

void reconstruct_coef_info(int comp_idx, ALF_PARAM *alf_param, int **filter_coeff, int *var_ind_tab)
{
    int i;
    if (comp_idx == Y_C)
    {
        memset(var_ind_tab, 0, NO_VAR_BINS * sizeof(int));
        if (alf_param->filters_per_group > 1)
        {
#if ALF_IMP
            for (i = 1; i < alf_param->max_filter_num; ++i)
#else
            for (i = 1; i < NO_VAR_BINS; ++i)
#endif
            {
                if (alf_param->filter_pattern[i])
                {
                    var_ind_tab[i] = var_ind_tab[i - 1] + 1;
                }
                else
                {
                    var_ind_tab[i] = var_ind_tab[i - 1];
                }
            }
        }
    }
    reconstruct_coefficients(alf_param, filter_coeff);
}

void extend_pic_border(pel *img, int height, int width, int margin_y, int margin_x, pel *img_ext)
{
    int   x, y, stride;
    pel  *pin;
    pel  *pout;
    pin = img;
    pout = img_ext + margin_y * (width + 2 * margin_x) + margin_x;
    for (y = 0; y < height; y++)
    {
        memcpy(pout, pin, width * sizeof(pel));
        pin = pin + width;
        pout = pout + (width + 2 * margin_x);
    }
    pout = img_ext + margin_y * (width + 2 * margin_x) + margin_x;
    stride = (width + 2 * margin_x);
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < margin_x; x++)
        {
            pout[-margin_x + x] = pout[0];
            pout[width + x] = pout[width - 1];
        }
        pout += stride;
    }
    pout -= (stride + margin_x);
    for (y = 0; y < margin_y; y++)
    {
        memcpy(pout + (y + 1)*stride, pout, sizeof(pel) * (width + (margin_x << 1)));
    }
    pout -= ((height - 1) * stride);
    for (y = 0; y < margin_y; y++)
    {
        memcpy(pout - (y + 1)*stride, pout, sizeof(pel) * (width + (margin_x << 1)));
    }
}

void alf_ctu_padding(pel *img_pad, pel *img_rec, int x_pos, int y_pos, int width, int height, int stride, int is_left_avail, int is_right_avail, int is_above_avail, int is_below_avail
#if ALF_SHAPE_IMPROVEMENT
    , int input_pad_size
#endif
)
{
#if ALF_SHAPE_IMPROVEMENT
    const int pad_size = input_pad_size;
    int start_pos_y = is_above_avail ? (y_pos - pad_size - 1) : y_pos;
    int end_pos_y   = is_below_avail ? (y_pos + height - pad_size - 1) : (y_pos + height);

    int start_pos_x = is_left_avail ? (x_pos - pad_size) : x_pos;
    int end_pos_x   = is_right_avail ? (x_pos + width + pad_size) : (x_pos + width);

    int wBuf = width + (pad_size << 1);
    int hBuf = end_pos_y - start_pos_y + (pad_size << 1);

    // copy the CTU buffer
    int avaiableW = end_pos_x - start_pos_x;
    for (int i = start_pos_y; i < end_pos_y; i++) 
    {
        memcpy(img_pad + (i - start_pos_y + pad_size) * wBuf + (start_pos_x - x_pos + pad_size), img_rec + i * stride + start_pos_x, sizeof(pel) * avaiableW);
    }

    for (int i = 0; i < pad_size; i++)
    {
        // up padding
        memcpy(img_pad + i * wBuf + pad_size, img_pad + pad_size * wBuf + pad_size, sizeof(pel) * width);
    }

    // down padding
    for (int i = 1; i < pad_size + 1; i++)
    {
        memcpy(img_pad + (hBuf - i) * wBuf + pad_size, img_pad + (hBuf - pad_size - 1) * wBuf + pad_size, sizeof(pel) * width);
    }

    // left upLeft downLeft
    if (!is_left_avail) 
    {
        for (int i = start_pos_y - pad_size; i < end_pos_y + pad_size; i++) 
        {
            for (int j = 0; j < pad_size; j++)
            {
                img_pad[(i - start_pos_y + pad_size) * wBuf + j] = img_pad[(i - start_pos_y + pad_size) * wBuf + pad_size];
            }
        }
    }
    else 
    {
        for (int i = start_pos_y - pad_size; i < start_pos_y; i++) 
        {
            for (int j = 0; j < pad_size; j++)
            {
                img_pad[(i - start_pos_y + pad_size) * wBuf + j] = img_pad[(i - start_pos_y + pad_size) * wBuf + pad_size];
            }
        }
        for (int i = end_pos_y; i < end_pos_y + pad_size; i++) 
        {
            for (int j = 0; j < pad_size; j++)
            {
                img_pad[(i - start_pos_y + pad_size) * wBuf + j] = img_pad[(i - start_pos_y + pad_size) * wBuf + pad_size];
            }
        }
    }

    // right upRight downRight
    if (!is_right_avail) 
    {
        for (int i = start_pos_y - pad_size; i < end_pos_y + pad_size; i++) 
        {
            for (int j = 1; j < pad_size + 1; j++)
            {
                img_pad[(i - start_pos_y + pad_size) * wBuf + wBuf - j] = img_pad[(i - start_pos_y + pad_size) * wBuf + (wBuf - pad_size - 1)];
            }
        }
    }
    else 
    {
        for (int i = start_pos_y - pad_size; i < start_pos_y; i++) 
        {
            for (int j = 1; j < pad_size + 1; j++)
            {
                img_pad[(i - start_pos_y + pad_size) * wBuf + wBuf - j] = img_pad[(i - start_pos_y + pad_size) * wBuf + (wBuf - pad_size - 1)];
            }
        }
        for (int i = end_pos_y; i < end_pos_y + pad_size; i++) 
        {
            for (int j = 1; j < pad_size + 1; j++)
            {
                img_pad[(i - start_pos_y + pad_size) * wBuf + wBuf - j] = img_pad[(i - start_pos_y + pad_size) * wBuf + (wBuf - pad_size - 1)];
            }
        }
    }
#else
    int start_pos_y = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos_y   = is_below_avail ? (y_pos + height - 4) : (y_pos + height);

    int start_pos_x = is_left_avail ? (x_pos - 3) : x_pos;
    int end_pos_x   = is_right_avail ? (x_pos + width + 3) : (x_pos + width);

    int wBuf = width + 6;
    int hBuf = end_pos_y - start_pos_y + 6;

    // copy the CTU buffer
    int avaiableW = end_pos_x - start_pos_x;
    for (int i = start_pos_y;i < end_pos_y;i++) {
        memcpy(img_pad + (i - start_pos_y + 3) * wBuf + (start_pos_x - x_pos + 3), img_rec + i * stride + start_pos_x, sizeof(pel) * avaiableW);
    }

    // up padding
    memcpy(img_pad +            3, img_pad + 3 * wBuf + 3, sizeof(pel) * width);
    memcpy(img_pad +     wBuf + 3, img_pad + 3 * wBuf + 3, sizeof(pel) * width);
    memcpy(img_pad + 2 * wBuf + 3, img_pad + 3 * wBuf + 3, sizeof(pel) * width);

    // down padding
    memcpy(img_pad + (hBuf - 1) * wBuf + 3, img_pad + (hBuf - 4) * wBuf + 3, sizeof(pel) * width);
    memcpy(img_pad + (hBuf - 2) * wBuf + 3, img_pad + (hBuf - 4) * wBuf + 3, sizeof(pel) * width);
    memcpy(img_pad + (hBuf - 3) * wBuf + 3, img_pad + (hBuf - 4) * wBuf + 3, sizeof(pel) * width);

    // left upLeft downLeft
    if (!is_left_avail) {
        for (int i = start_pos_y - 3;i < end_pos_y + 3;i++) {
            img_pad[(i - start_pos_y + 3) * wBuf + 0] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
            img_pad[(i - start_pos_y + 3) * wBuf + 1] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
            img_pad[(i - start_pos_y + 3) * wBuf + 2] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
        }
    }
    else {
        for (int i = start_pos_y - 3;i < start_pos_y;i++) {
            img_pad[(i - start_pos_y + 3) * wBuf + 0] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
            img_pad[(i - start_pos_y + 3) * wBuf + 1] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
            img_pad[(i - start_pos_y + 3) * wBuf + 2] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
        }
        for (int i = end_pos_y;i < end_pos_y + 3;i++) {
            img_pad[(i - start_pos_y + 3) * wBuf + 0] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
            img_pad[(i - start_pos_y + 3) * wBuf + 1] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
            img_pad[(i - start_pos_y + 3) * wBuf + 2] = img_pad[(i - start_pos_y + 3) * wBuf + 3];
        }
    }

    // right upRight downRight
    if (!is_right_avail) {
        for (int i = start_pos_y - 3;i < end_pos_y + 3;i++) {
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 1] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 2] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 3] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
        }
    }
    else {
        for (int i = start_pos_y - 3;i < start_pos_y;i++) {
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 1] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 2] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 3] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
        }
        for (int i = end_pos_y;i < end_pos_y + 3;i++) {
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 1] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 2] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
            img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 3] = img_pad[(i - start_pos_y + 3) * wBuf + wBuf - 4];
        }
    }
#endif
}

void filter_one_comp_region(pel *img_res, pel *img_pad, int stride
    , int padStride
    , BOOL is_chroma, int y_pos, int lcu_height, int x_pos,
    int lcu_width, int **filter_set, int *merge_table, pel **var_img,
    int sample_bit_depth, int is_left_avail, int is_right_avail, int is_above_avail, int is_below_avail
#if ALF_SHAPE || ALF_SHIFT
    , BOOL alf_enhance_flag
#endif
)
{
#if !ALF_SHIFT
    int offset = (1 << ((int)ALF_NUM_BIT_SHIFT - 1));
#endif
    pel *img_pad1, *img_pad2, *img_pad3, *img_pad4, *img_pad5, *img_pad6;
#if ALF_SHAPE_IMPROVEMENT
    pel* img_pad7, * img_pad8;
#endif
    pel *var = var_img[y_pos >> LOG2_VAR_SIZE_H] + (x_pos >> LOG2_VAR_SIZE_W);
    int i, j, pixel_int;
    int *coef = (is_chroma) ? filter_set[0] : filter_set[merge_table[*(var)]];
#if ALF_SHAPE_IMPROVEMENT
    int pad_size = 4;
    pixel_int = 0;
    int start_pos = is_above_avail ? (y_pos - pad_size - 1) : y_pos;
    int end_pos = is_below_avail ? (y_pos + lcu_height - pad_size - 1) : (y_pos + lcu_height);
    int x_offset_left = is_left_avail ? -pad_size : 0;
    int x_offset_right = is_right_avail ? pad_size : 0;
    int x_pos_end = x_pos + lcu_width;

    img_pad += (pad_size * padStride + pad_size);
    img_res += (start_pos * stride);
#else
    int start_pos = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos = is_below_avail ? (y_pos + lcu_height - 4) : (y_pos + lcu_height);
    int x_offset_left = is_left_avail ? -3 : 0;
    int x_offset_right = is_right_avail ? 3 : 0;
    int x_pos_end = x_pos + lcu_width;

    img_pad += (3 * padStride + 3);
    img_res += (start_pos * stride);
#endif
#if ALF_SHIFT
    // last number in coefficient buffer is shift offset
    int offset;
    int shift;
    if (alf_enhance_flag)
    {
        int shiftoffset;

        shiftoffset = coef[15];
        shiftoffset += (ALF_NUM_BIT_SHIFT - 1);
        offset = 1 << shiftoffset;
        shift = shiftoffset + 1;
    }
    else
    {
        offset = (1 << ((int)ALF_NUM_BIT_SHIFT - 1));
        shift = ALF_NUM_BIT_SHIFT;
    }
#endif
#if ALF_SHAPE
    if (!alf_enhance_flag)
    {
#endif
    for (i = start_pos; i < end_pos; i++)
    {
        img_pad1 =  img_pad + padStride;
        img_pad2 =  img_pad - padStride;
        img_pad3 = img_pad1 + padStride;
        img_pad4 = img_pad2 - padStride;
        img_pad5 = img_pad3 + padStride;
        img_pad6 = img_pad4 - padStride;
        for (j = x_pos; j < x_pos_end; j++)
        {
            int cur = j - x_pos;

            pixel_int  = coef[0] * (img_pad5[cur    ] + img_pad6[cur    ]);

            pixel_int += coef[1] * (img_pad3[cur    ] + img_pad4[cur    ]);

            pixel_int += coef[2] * (img_pad1[cur + 1] + img_pad2[cur - 1]);
            pixel_int += coef[3] * (img_pad1[cur    ] + img_pad2[cur    ]);
            pixel_int += coef[4] * (img_pad1[cur - 1] + img_pad2[cur + 1]);

            pixel_int += coef[5] * ( img_pad[cur - 3] +  img_pad[cur + 3]);
            pixel_int += coef[6] * ( img_pad[cur - 2] +  img_pad[cur + 2]);
            pixel_int += coef[7] * ( img_pad[cur - 1] +  img_pad[cur + 1]);
            pixel_int += coef[8] * ( img_pad[cur    ]);
#if ALF_SHIFT
            pixel_int = (int)((pixel_int + offset) >> shift);
#else
            pixel_int = (int)((pixel_int + offset) >> ALF_NUM_BIT_SHIFT);
#endif
            img_res[j] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1), pixel_int);
        }
        img_pad += padStride;
        img_res += stride;
    }
#if ALF_SHAPE
    }
    else
    {
        for (i = start_pos; i < end_pos; i++)
        {
            img_pad1 =  img_pad + padStride;
            img_pad2 =  img_pad - padStride;
            img_pad3 = img_pad1 + padStride;
            img_pad4 = img_pad2 - padStride;
            img_pad5 = img_pad3 + padStride;
            img_pad6 = img_pad4 - padStride;
#if ALF_SHAPE_IMPROVEMENT
            img_pad7 = img_pad5 + padStride;
            img_pad8 = img_pad6 - padStride;
#endif

            for (j = x_pos; j < x_pos_end; j++)
            {
                int cur = j - x_pos;
#if ALF_SHAPE_IMPROVEMENT
                    pixel_int  =  coef[0] * (img_pad7[cur    ] + img_pad8[cur    ]);

                    pixel_int +=  coef[1] * (img_pad5[cur + 3] + img_pad6[cur - 3]);
                    pixel_int +=  coef[2] * (img_pad5[cur    ] + img_pad6[cur    ]);
                    pixel_int +=  coef[3] * (img_pad5[cur - 3] + img_pad6[cur + 3]);

                    pixel_int +=  coef[4] * (img_pad3[cur + 2] + img_pad4[cur - 2]);
                    pixel_int +=  coef[5] * (img_pad3[cur    ] + img_pad4[cur    ]);
                    pixel_int +=  coef[6] * (img_pad3[cur - 2] + img_pad4[cur + 2]);

                    pixel_int +=  coef[7] * (img_pad1[cur + 1] + img_pad2[cur - 1]);
                    pixel_int +=  coef[8] * (img_pad1[cur    ] + img_pad2[cur    ]);
                    pixel_int +=  coef[9] * (img_pad1[cur - 1] + img_pad2[cur + 1]);

                    pixel_int += coef[10] * ( img_pad[cur - 4] +  img_pad[cur + 4]);
                    pixel_int += coef[11] * ( img_pad[cur - 3] +  img_pad[cur + 3]);
                    pixel_int += coef[12] * ( img_pad[cur - 2] +  img_pad[cur + 2]);
                    pixel_int += coef[13] * ( img_pad[cur - 1] +  img_pad[cur + 1]);
                    pixel_int += coef[14] * ( img_pad[cur    ]);

#else
                pixel_int  =  coef[0] * (img_pad5[cur    ] + img_pad6[cur    ]);

                pixel_int +=  coef[1] * (img_pad3[cur + 2] + img_pad4[cur - 2]);
                pixel_int +=  coef[2] * (img_pad3[cur + 1] + img_pad4[cur - 1]);
                pixel_int +=  coef[3] * (img_pad3[cur    ] + img_pad4[cur    ]);
                pixel_int +=  coef[4] * (img_pad3[cur - 1] + img_pad4[cur + 1]);
                pixel_int +=  coef[5] * (img_pad3[cur - 2] + img_pad4[cur + 2]);

                pixel_int +=  coef[6] * (img_pad1[cur + 2] + img_pad2[cur - 2]);
                pixel_int +=  coef[7] * (img_pad1[cur + 1] + img_pad2[cur - 1]);
                pixel_int +=  coef[8] * (img_pad1[cur    ] + img_pad2[cur    ]);
                pixel_int +=  coef[9] * (img_pad1[cur - 1] + img_pad2[cur + 1]);
                pixel_int += coef[10] * (img_pad1[cur - 2] + img_pad2[cur + 2]);

                pixel_int += coef[11] * ( img_pad[cur - 3] +  img_pad[cur + 3]);
                pixel_int += coef[12] * ( img_pad[cur - 2] +  img_pad[cur + 2]);

                pixel_int += coef[13] * ( img_pad[cur - 1] +  img_pad[cur + 1]);
                pixel_int += coef[14] * ( img_pad[cur    ]);
#endif
#if ALF_SHIFT
                pixel_int = (int)((pixel_int + offset) >> shift);
#else
                pixel_int = (int)((pixel_int + offset) >> ALF_NUM_BIT_SHIFT);
#endif
                img_res[j] = (pel)COM_CLIP3(0, ((1 << sample_bit_depth) - 1), pixel_int);
            }
            img_pad += padStride;
            img_res += stride;
        }

    }
#endif
}

void check_filter_coeff_value(int *filter, int filter_length)
{
    int i;
    int max_value_non_center;
    int min_value_non_center;
    int max_value_center;
    int min_value_center;
    max_value_non_center = 1 * (1 << ALF_NUM_BIT_SHIFT) - 1;
    min_value_non_center = 0 - 1 * (1 << ALF_NUM_BIT_SHIFT);
    max_value_center = 2 * (1 << ALF_NUM_BIT_SHIFT) - 1;
    min_value_center = 0;
    for (i = 0; i < filter_length - 1; i++)
    {
        filter[i] = COM_CLIP3(min_value_non_center, max_value_non_center, filter[i]);
    }
    filter[filter_length - 1] = COM_CLIP3(min_value_center, max_value_center, filter[filter_length - 1]);
}

void copy_alf_param(ALF_PARAM *dst, ALF_PARAM *src
#if ALF_SHAPE
                  , int num_coef
#endif
)
{
    int j;
    dst->alf_flag = src->alf_flag;
    dst->component_id = src->component_id;
    dst->filters_per_group = src->filters_per_group;
#if ALF_IMP
    dst->dir_index = src->dir_index;
    dst->max_filter_num = src->max_filter_num;
#endif
    dst->num_coeff = src->num_coeff;
    switch (src->component_id)
    {
    case Y_C:
        for (j = 0; j < NO_VAR_BINS; j++)
        {
#if ALF_SHAPE
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], num_coef * sizeof(int));
#else
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], ALF_MAX_NUM_COEF * sizeof(int));
#endif
        }
        memcpy(dst->filter_pattern, src->filter_pattern, NO_VAR_BINS * sizeof(int));
        break;
    case U_C:
    case V_C:
        for (j = 0; j < 1; j++)
        {
#if ALF_SHAPE
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], num_coef * sizeof(int));
#else
            memcpy(dst->coeff_multi[j], src->coeff_multi[j], ALF_MAX_NUM_COEF * sizeof(int));
#endif
        }
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
}

int get_lcu_ctrl_ctx_idx(int ctu, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int comp_idx, BOOL **alf_lcu_enabled)
{
    int row, col, a, b;
    row = ctu / num_lcu_in_pic_width;
    col = ctu % num_lcu_in_pic_width;
    if (row == 0)
    {
        b = 0;
    }
    else
    {
        b = (int)alf_lcu_enabled[ctu - num_lcu_in_pic_width][comp_idx];
    }
    if (col == 0)
    {
        a = 0;
    }
    else
    {
        a = (int)alf_lcu_enabled[ctu - 1][comp_idx];
    }
    return (a + 2 * b);
}

void set_filter_image(pel *dec_Y, pel *dec_U, pel *dec_V, int in_stride, pel *img_Y_out, pel * img_U_out, pel * img_V_out, int out_stride, int img_width, int img_height)
{
    int j, i, width_cr, in_stride_cr, out_stride_cr, height_cr;
    pel *src;
    src = dec_Y;
    for (j = 0; j < img_height; j++)
    {
        for (i = 0; i < img_width; i++)
        {
            img_Y_out[i] = src[i];
        }
        img_Y_out += out_stride;
        src += in_stride;
    }
    width_cr = img_width >> 1;
    height_cr = img_height >> 1;
    in_stride_cr = in_stride >> 1;
    out_stride_cr = out_stride >> 1;
    src = dec_U;
    for (j = 0; j < height_cr; j++)
    {
        for (i = 0; i < width_cr; i++)
        {
            img_U_out[i] = src[i];
        }
        img_V_out += out_stride_cr;
        src += in_stride_cr;
    }
    src = dec_V;
    for (j = 0; j < height_cr; j++)
    {
        for (i = 0; i < width_cr; i++)
        {
            img_V_out[i] = src[i];
        }
        img_V_out += out_stride_cr;
        src += in_stride_cr;
    }
}

int get_mem_1D_int(int **array1D, int num)
{
    if ((*array1D = (int *)calloc(num, sizeof(int))) == NULL)
    {
        assert(0);
    }
    return num * sizeof(int);
}

int get_mem_2D_int(int ***array2D, int rows, int columns)
{
    int i;
    if ((*array2D = (int **)calloc(rows, sizeof(int *))) == NULL)
    {
        assert(0);
    }
    if (((*array2D)[0] = (int *)calloc(rows * columns, sizeof(int))) == NULL)
    {
        assert(0);
    }
    for (i = 1; i < rows; i++)
    {
        (*array2D)[i] = (*array2D)[i - 1] + columns;
    }
    return rows * columns * sizeof(int);
}

void free_mem_1D_int(int *array1D)
{
    if (array1D)
    {
        free(array1D);
    }
    else
    {
        assert(0);
    }
}

void free_mem_2D_int(int **array2D)
{
    if (array2D)
    {
        if (array2D[0])
        {
            free(array2D[0]);
            array2D[0] = NULL;
        }
        else
        {
            assert(0);
        }
        free(array2D);
    }
    else
    {
        assert(0);
    }
}


void copy_frame_for_alf(COM_PIC *pic_dst, COM_PIC *pic_src)
{
    int i, j;
    int src_stride, dst_stride;
    pel* src;
    pel* dst;
    src_stride = pic_src->stride_luma;
    dst_stride = pic_dst->stride_luma;
    src = pic_src->y;
    dst = pic_dst->y;
    for (j = 0; j < pic_src->height_luma; j++)
    {
        for (i = 0; i < pic_src->width_luma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src_stride = pic_src->stride_chroma;
    dst_stride = pic_dst->stride_chroma;
    src = pic_src->u;
    dst = pic_dst->u;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
    src = pic_src->v;
    dst = pic_dst->v;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        dst += dst_stride;
        src += src_stride;
    }
}
