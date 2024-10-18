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

#include "dec_DecAdaptiveLoopFilter.h"

#define Clip_post(high,val) ((val > high)? high: val)

/*
*************************************************************************
* Function: ALF decoding process top function
* Input:
*         ctx  : CONTEXT used for decoding process
*     pic_rec  : picture of the the ALF input image
*     pic_dec  : picture of the the ALF reconstructed image
*************************************************************************
*/
void alf_process_dec(DEC_CTX* ctx, ALF_PARAM **alf_param, COM_PIC* pic_rec, COM_PIC* pic_dec)
{
    int bit_depth = ctx->info.bit_depth_internal;
    pel *rec_Y, *dec_Y;
    pel *rec_UV[2], *dec_UV[2];
    int  lcu_idx, num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int  lcu_y_pos, lcu_x_pos, lcu_height, lcu_width;
    int  comp_idx, luma_stride, chroma_stride, img_height, img_width;
    BOOL is_left_avail, is_right_avail, is_above_avail, is_below_avail;
    BOOL is_above_left_avail, is_above_right_avail;
    lcu_height = 1 << ctx->info.log2_max_cuwh;
    lcu_width  = lcu_height;
    img_height = ctx->info.pic_height;
    img_width  = ctx->info.pic_width;
    num_lcu_in_pic_width  = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width  += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    luma_stride = pic_rec->stride_luma;
    chroma_stride = pic_rec->stride_chroma;
    rec_Y = pic_rec->y;
    rec_UV[0] = pic_rec->u;
    rec_UV[1] = pic_rec->v;
    dec_Y = pic_dec->y;
    dec_UV[0] = pic_dec->u;
    dec_UV[1] = pic_dec->v;
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
    BOOL alf_enhance_flag = ctx->info.sqh.adaptive_leveling_filter_enhance_flag;
#endif
    for (lcu_idx = 0; lcu_idx < num_lcu_in_frame; lcu_idx++)
    {
        lcu_y_pos = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
        lcu_x_pos = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
        int cur_lcu_height = (lcu_y_pos + lcu_height > img_height) ? (img_height - lcu_y_pos) : lcu_height;
        int cur_lcu_width = (lcu_x_pos + lcu_width  > img_width) ? (img_width - lcu_x_pos) : lcu_width;
        //derive CTU boundary availabilities
        derive_boundary_avail(ctx, num_lcu_in_pic_width, num_lcu_in_pic_height, lcu_idx, &is_left_avail, &is_right_avail, &is_above_avail,
                            &is_below_avail, &is_above_left_avail, &is_above_right_avail);
        for (comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            if (!ctx->dec_alf->alf_lcu_enabled[lcu_idx][comp_idx])
            {
                continue;
            }
            if (comp_idx == Y_C)
            {
                filter_one_ctb(ctx->dec_alf, rec_Y, dec_Y, (comp_idx == Y_C) ? (luma_stride) : (chroma_stride)
                             , comp_idx, bit_depth, alf_param[comp_idx], lcu_y_pos, cur_lcu_height, lcu_x_pos, cur_lcu_width
                             , is_above_avail, is_below_avail, is_left_avail, is_right_avail, is_above_left_avail, is_above_right_avail, bit_depth
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
                             , alf_enhance_flag
#endif
                );
            }
            else
            {
                filter_one_ctb(ctx->dec_alf, rec_UV[comp_idx - U_C], dec_UV[comp_idx - U_C], (comp_idx == Y_C) ? (luma_stride) : (chroma_stride)
                             , comp_idx, bit_depth, alf_param[comp_idx], lcu_y_pos, cur_lcu_height, lcu_x_pos, cur_lcu_width
                             , is_above_avail, is_below_avail, is_left_avail, is_right_avail, is_above_left_avail, is_above_right_avail, bit_depth
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
                             , alf_enhance_flag
#endif
                );
            }
        }
    }
}

/*
*************************************************************************
* Function: ALF filter on CTB
*************************************************************************
*/
void filter_one_ctb(DEC_ALF_VAR * dec_alf, pel *rec, pel *dec, int stride, int comp_idx, int bit_depth, ALF_PARAM *alf_param
                  , int lcu_y_pos, int lcu_height, int lcu_x_pos, int lcu_width
                  , BOOL is_above_avail, BOOL is_below_avail, BOOL is_left_avail, BOOL is_right_avail, BOOL is_above_left_avail
                  , BOOL is_above_right_avail, int sample_bit_depth
#if ALF_SHAPE  || ALF_IMP || ALF_SHIFT
                  , BOOL alf_enhance_flag
#endif
                 )
{
    const int  format_shift = (comp_idx == Y_C) ? 0 : 1;
    int y_pos, x_pos, height, width;
    //reconstruct coefficients to m_filterCoeffSym and m_varIndTab
    reconstruct_coef_info(comp_idx, alf_param, dec_alf->filter_coeff_sym,
                        dec_alf->var_ind_tab); //reconstruct ALF coefficients & related parameters
    //derive CTB start positions, width, and height. If the boundary is not available, skip boundary samples.
    y_pos = (lcu_y_pos >> format_shift);
    height = (lcu_height >> format_shift);
    x_pos = (lcu_x_pos >> format_shift);
    width = (lcu_width >> format_shift);

#if ALF_SHAPE_IMPROVEMENT
    int pad_size = 4;

    int start_pos_y = is_above_avail ? (y_pos - pad_size - 1) : y_pos;
    int end_pos_y   = is_below_avail ? (y_pos + height - pad_size - 1) : (y_pos + height);
    int wBuf = width + (pad_size << 1);
    int hBuf = end_pos_y - start_pos_y + (pad_size << 1);
#else
    int start_pos_y = is_above_avail ? (y_pos - 4) : y_pos;
    int end_pos_y   = is_below_avail ? (y_pos + height - 4) : (y_pos + height);

    int wBuf = width + 6;
    int hBuf = end_pos_y - start_pos_y + 6;
#endif
    pel* img_buf = (pel *)malloc((wBuf * hBuf) * sizeof(pel));;

    alf_ctu_padding(img_buf, dec, x_pos, y_pos, width, height, stride, is_left_avail, is_right_avail, is_above_avail, is_below_avail
#if ALF_SHAPE_IMPROVEMENT
        , pad_size
#endif
    );

    filter_one_comp_region(rec, img_buf, stride, wBuf, (comp_idx != Y_C)
        , y_pos, height, x_pos, width, dec_alf->filter_coeff_sym, dec_alf->var_ind_tab
#if ALF_IMP
        , alf_enhance_flag ? (comp_idx == Y_C ? dec_alf->var_img_arr[alf_param->dir_index] : dec_alf->var_img_arr[0]) : dec_alf->var_img
#else
        , dec_alf->var_img
#endif
        , sample_bit_depth, is_left_avail, is_right_avail, is_above_avail, is_below_avail
#if ALF_SHAPE || ALF_SHIFT
        , alf_enhance_flag
#endif
    );
    free(img_buf);
}

void derive_alf_boundary_availibility(DEC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int lcu_idx,
        BOOL *is_left_avail, BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail)
{
    int num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    BOOL is_above_left_avail;
    BOOL is_above_right_avail;
    BOOL is_below_left_avail;
    BOOL is_below_right_avail;
    int  lcu_height = 1 << ctx->info.log2_max_cuwh;
    int  lcu_width = lcu_height;
    int  img_height = ctx->info.pic_height;
    int  img_width = ctx->info.pic_width;
    int  pic_x;
    int  pic_y;
    int  mb_x;
    int  mb_y;
    int  mb_curr;
    int  smb_mb_width;
    int  smb_mb_height;
    int  pic_mb_width = img_width / MIN_CU_SIZE;
    int  pic_mb_height = img_height / MIN_CU_SIZE;
    int  mb_left;
    int  mb_right;
    int  mb_above;
    int  mb_below;
    int  mb_above_left;
    int  mb_above_right;
    int  mb_below_left;
    int  mb_below_right;
    int  mb_curr_patch_idx, mb_neighbor_patch_idx, mb_neighbor2_patch_idx, mb_neighbor3_patch_idx;
    pic_x = (lcu_idx % num_lcu_in_pic_width) * lcu_width;
    pic_y = (lcu_idx / num_lcu_in_pic_width) * lcu_height;
    pic_mb_width += (img_width  % MIN_CU_SIZE) ? 1 : 0;
    pic_mb_height += (img_height % MIN_CU_SIZE) ? 1 : 0;
    mb_x = pic_x / MIN_CU_SIZE;
    mb_y = pic_y / MIN_CU_SIZE;
    mb_curr = mb_y * pic_mb_width + mb_x;
    smb_mb_width = lcu_width >> MIN_CU_LOG2;
    smb_mb_height = lcu_height >> MIN_CU_LOG2;
    *is_left_avail = (lcu_idx % num_lcu_in_pic_width != 0);
    *is_right_avail = (lcu_idx % num_lcu_in_pic_width != num_lcu_in_pic_width - 1);
    *is_above_avail = (lcu_idx >= num_lcu_in_pic_width);
    *is_below_avail = (lcu_idx  < num_lcu_in_frame - num_lcu_in_pic_width);
    is_above_left_avail = (*is_above_avail && *is_left_avail);
    is_above_right_avail = (*is_above_avail && *is_right_avail);
    is_below_left_avail = (*is_below_avail && *is_left_avail);
    is_below_right_avail = (*is_below_avail && *is_right_avail);
    s8 *map_patch_idx = ctx->map.map_patch_idx;
    mb_left = *is_left_avail ? (mb_curr - 1) : -1;
    mb_right = *is_right_avail ? (mb_curr + 1) : -1;
    mb_above = *is_above_avail ? (mb_curr - pic_mb_width) : -1;
    mb_below = *is_below_avail ? (mb_curr + pic_mb_width) : -1;
    mb_above_left = is_above_left_avail ? (mb_curr - pic_mb_width - 1) : -1;
    mb_above_right = is_above_right_avail ? (mb_curr - pic_mb_width + 1) : -1;
    mb_below_left = is_below_left_avail ? (mb_curr + pic_mb_width - 1) : -1;
    mb_below_right = is_below_right_avail ? (mb_curr + pic_mb_width + 1) : -1;
    *is_left_avail = *is_right_avail = *is_above_avail = *is_below_avail = FALSE;
    mb_curr_patch_idx = map_patch_idx[mb_curr];
    if (mb_left != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_left];
        if (mb_curr_patch_idx == mb_neighbor_patch_idx)
        {
            *is_left_avail = TRUE;
        }
    }
    if (mb_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_right];
        if (mb_curr_patch_idx == mb_neighbor_patch_idx)
        {
            *is_right_avail = TRUE;
        }
    }
    if (mb_above != -1 && mb_above_left != -1 && mb_above_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_above];
        mb_neighbor2_patch_idx = map_patch_idx[mb_above_left];
        mb_neighbor3_patch_idx = map_patch_idx[mb_above_right];
        if ((mb_curr_patch_idx == mb_neighbor_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor2_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor3_patch_idx))
        {
            *is_above_avail = TRUE;
        }
    }
    if (mb_below != -1 && mb_below_left != -1 && mb_below_right != -1)
    {
        mb_neighbor_patch_idx = map_patch_idx[mb_below];
        mb_neighbor2_patch_idx = map_patch_idx[mb_below_left];
        mb_neighbor3_patch_idx = map_patch_idx[mb_below_right];
        if ((mb_curr_patch_idx == mb_neighbor_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor2_patch_idx)
                && (mb_neighbor_patch_idx == mb_neighbor3_patch_idx))
        {
            *is_below_avail = TRUE;
        }
    }
}

void create_alf_global_buffer(DEC_CTX *ctx)
{
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int n, i, g;
#if ALF_IMP 
    int alf_enhance_flag = ctx->info.sqh.adaptive_leveling_filter_enhance_flag;
    int tbl_region[NO_VAR_BINS_16] = { 0, 1, 4, 5, 15, 2, 3, 6, 14, 11, 10, 7, 13, 12,  9,  8 };
    int tbl_region_arr[ITER_NUM][NO_VAR_BINS] = {
        { 63,60,59,58,5,4,3,0,62,61,56,57,6,7,2,1,49,50,55,54,9,8,13,14,48,51,52,53,10,11,12,15,47,46,33,32,31,30,17,16,44,45,34,35,28,29,18,19,43,40,39,36,27,24,23,20,42,41,38,37,26,25,22,21 },
        { 42,43,44,47,48,49,62,63,41,40,45,46,51,50,61,60,38,39,34,33,52,55,56,59,37,36,35,32,53,54,57,58,26,27,28,31,10,9,6,5,25,24,29,30,11,8,7,4,22,23,18,17,12,13,2,3,21,20,19,16,15,14,1,0 },
        { 21,22,25,26,37,38,41,42,20,23,24,27,36,39,40,43,19,18,29,28,35,34,45,44,16,17,30,31,32,33,46,47,15,12,11,10,53,52,51,48,14,13,8,9,54,55,50,49,1,2,7,6,57,56,61,62,0,3,4,5,58,59,60,63 },
        { 0,1,14,15,16,19,20,21,3,2,13,12,17,18,23,22,4,7,8,11,30,29,24,25,5,6,9,10,31,28,27,26,58,57,54,53,32,35,36,37,59,56,55,52,33,34,39,38,60,61,50,51,46,45,40,41,63,62,49,48,47,44,43,42 }
    };
#else
    int tbl_region[NO_VAR_BINS] = { 0, 1, 4, 5, 15, 2, 3, 6, 14, 11, 10, 7, 13, 12,  9,  8 };
#endif
    int x_interval;
    int y_interval;
    int y_index, x_index;
    int y_index_offset;
#if ALF_IMP 
    int y_index2 = 0;
    int x_index2 = 0;
    int y_index_offset2=0;
    int y_st_offset = 0;
    int x_st_offset = 0;
    int y_cnt = 0;
    int x_cnt = 0;
#endif
    lcu_height = 1 << (ctx->info.log2_max_cuwh);
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
#if ALF_IMP 
    if (alf_enhance_flag)
    {
        x_interval = ((((img_width + lcu_width - 1) / lcu_width) + INTERVAL_X_OFFSET) / INTERVAL_NUM_X * lcu_width);
        y_interval = ((((img_height + lcu_height - 1) / lcu_height) + INTERVAL_Y_OFFSET) / INTERVAL_NUM_Y * lcu_height);
        if (y_interval == 0)
        {
            y_st_offset = 0;
        }
        else
        {
            y_cnt = Clip_post(INTERVAL_NUM_Y, (img_height + y_interval - 1) / y_interval);
            y_st_offset = img_height - y_interval*(y_cnt - 1);
            y_st_offset = (y_st_offset + lcu_height / 2) / lcu_height*lcu_height;
        }
        if (x_interval == 0)
        {
            x_st_offset = 0;
        }
        else
        {
            x_cnt = Clip_post(INTERVAL_NUM_X, (img_width + x_interval - 1) / x_interval);
            x_st_offset = img_width - x_interval*(x_cnt - 1);
            x_st_offset = (x_st_offset + lcu_width / 2) / lcu_width*lcu_width;
        }
    }
    else
    {
#endif
        x_interval = ((((img_width + lcu_width - 1) / lcu_width) + 1) / 4 * lcu_width);
        y_interval = ((((img_height + lcu_height - 1) / lcu_height) + 1) / 4 * lcu_height);
#if ALF_IMP
    }
#endif

    ctx->dec_alf = (DEC_ALF_VAR *)malloc(1 * sizeof(DEC_ALF_VAR));
    ctx->dec_alf->alf_lcu_enabled = (BOOL **)malloc(num_lcu_in_frame * sizeof(BOOL *));
#if ALF_SHAPE
    int num_coef = (ctx->info.sqh.adaptive_leveling_filter_enhance_flag) ? ALF_MAX_NUM_COEF_SHAPE2 : ALF_MAX_NUM_COEF;
#endif
#if ALF_SHIFT
    // extend buffer size if shift flag is enabled
    num_coef += (int)(ctx->info.sqh.adaptive_leveling_filter_enhance_flag);
#endif
    for (n = 0; n < num_lcu_in_frame; n++)
    {
        ctx->dec_alf->alf_lcu_enabled[n] = (BOOL *)malloc(N_C * sizeof(BOOL));
        memset(ctx->dec_alf->alf_lcu_enabled[n], FALSE, N_C * sizeof(BOOL));
    }  
#if ALF_IMP
    for (i = 0; i < ITER_NUM; i++)
    {
        ctx->dec_alf->var_img_arr[i] = (pel **)malloc(img_height * sizeof(pel *));
        for (n = 0; n < img_height; n++)
        {
            ctx->dec_alf->var_img_arr[i][n] = (pel *)malloc(img_width * sizeof(pel));
            memset(ctx->dec_alf->var_img_arr[i][n], 0, img_width * sizeof(pel));
        }
    }
#endif
    ctx->dec_alf->var_img = (pel **)malloc(img_height * sizeof(pel *));
    for (n = 0; n < img_height; n++)
    {
        ctx->dec_alf->var_img[n] = (pel *)malloc(img_width * sizeof(pel));
        memset(ctx->dec_alf->var_img[n], 0, img_width * sizeof(pel));
    }
    ctx->dec_alf->filter_coeff_sym = (int **)malloc(NO_VAR_BINS * sizeof(int *));
    for (g = 0; g < (int)NO_VAR_BINS; g++)
    {
#if ALF_SHAPE
        ctx->dec_alf->filter_coeff_sym[g] = (int *)malloc(num_coef * sizeof(int));
        memset(ctx->dec_alf->filter_coeff_sym[g], 0, num_coef * sizeof(int));
#else
        ctx->dec_alf->filter_coeff_sym[g] = (int *)malloc(ALF_MAX_NUM_COEF * sizeof(int));
        memset(ctx->dec_alf->filter_coeff_sym[g], 0, ALF_MAX_NUM_COEF * sizeof(int));
#endif
    }
    for (i = 0; i < img_height; i = i + 4)
    {
#if ALF_IMP
        if (alf_enhance_flag)
        {
            y_index = (y_interval == 0) ? (INTERVAL_NUM_Y - 1) : (Clip_post(INTERVAL_NUM_Y - 1, i / y_interval));
            y_index_offset = y_index * INTERVAL_NUM_X;
            y_index2 = (y_interval == 0 || i < y_st_offset) ? (0) : (Clip_post(INTERVAL_NUM_Y - 2, (i - y_st_offset) / y_interval) + 1);
            y_index_offset2 = y_index2 * INTERVAL_NUM_X;
        }
        else
        {
#endif
            y_index = (y_interval == 0) ? (3) : (Clip_post(3, i / y_interval));
            y_index_offset = y_index * 4;
#if ALF_IMP
        }
#endif
        for (g = 0; g < img_width; g = g + 4)
        {
#if ALF_IMP
            if (alf_enhance_flag)
            {
                x_index = (x_interval == 0) ? (INTERVAL_NUM_X - 1) : (Clip_post(INTERVAL_NUM_X - 1, g / x_interval));
                x_index2 = (x_interval == 0 || g < x_st_offset) ? (0) : (Clip_post(INTERVAL_NUM_X - 2, (g - x_st_offset) / x_interval) + 1);
            }
            else
            {
#endif
                x_index = (x_interval == 0) ? (3) : (Clip_post(3, g / x_interval));
#if ALF_IMP
            }
#endif
#if ALF_IMP 
            if (alf_enhance_flag)
            {
                ctx->dec_alf->var_img_arr[0][i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region_arr[0][y_index_offset + x_index];
                ctx->dec_alf->var_img_arr[1][i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region_arr[1][y_index_offset + x_index2];
                ctx->dec_alf->var_img_arr[2][i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region_arr[2][y_index_offset2 + x_index2];
                ctx->dec_alf->var_img_arr[3][i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region_arr[3][y_index_offset2 + x_index];
            }
            else
            {
#endif
                ctx->dec_alf->var_img[i >> LOG2_VAR_SIZE_H][g >> LOG2_VAR_SIZE_W] = (pel)tbl_region[y_index_offset + x_index];
#if ALF_IMP
            }
#endif
        }
    }
    ctx->dec_alf->alf_picture_param = (ALF_PARAM **)malloc((N_C) * sizeof(ALF_PARAM *));
    for (i = 0; i < N_C; i++)
    {
        ctx->dec_alf->alf_picture_param[i] = NULL;
        allocate_alf_param(&(ctx->dec_alf->alf_picture_param[i]), i
#if ALF_SHAPE
                        , num_coef
#endif
        );
    }
}

void release_alf_global_buffer(DEC_CTX *ctx)
{
    int lcu_height, lcu_width, img_height, img_width;
    int num_lcu_in_frame, num_lcu_in_pic_width, num_lcu_in_pic_height;
    int n, g;
    lcu_height = 1 << (ctx->info.log2_max_cuwh);
    lcu_width = lcu_height;
    img_height = ctx->info.pic_height;
    img_width = ctx->info.pic_width;
    num_lcu_in_pic_width = img_width / lcu_width;
    num_lcu_in_pic_height = img_height / lcu_height;
    num_lcu_in_pic_width += (img_width % lcu_width) ? 1 : 0;
    num_lcu_in_pic_height += (img_height % lcu_height) ? 1 : 0;
    num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    if (ctx->dec_alf)
    {
#if ALF_IMP 
        for (int i = 0; i < ITER_NUM; i++)
        {
            if (ctx->dec_alf->var_img_arr[i])
            {
                for (n = 0; n < img_height; n++)
                {
                    free(ctx->dec_alf->var_img_arr[i][n]);
                }
                free(ctx->dec_alf->var_img_arr[i]);
                ctx->dec_alf->var_img_arr[i] = NULL;
            }
        }
#endif
        if (ctx->dec_alf->var_img)
        {
            for (n = 0; n < img_height; n++)
            {
                free(ctx->dec_alf->var_img[n]);
            }
            free(ctx->dec_alf->var_img);
            ctx->dec_alf->var_img = NULL;
        }
        if (ctx->dec_alf->alf_lcu_enabled)
        {
            for (n = 0; n < num_lcu_in_frame; n++)
            {
                free(ctx->dec_alf->alf_lcu_enabled[n]);
            }
            free(ctx->dec_alf->alf_lcu_enabled);
            ctx->dec_alf->alf_lcu_enabled = NULL;
        }
        if (ctx->dec_alf->filter_coeff_sym)
        {
            for (g = 0; g < (int)NO_VAR_BINS; g++)
            {
                free(ctx->dec_alf->filter_coeff_sym[g]);
            }
            free(ctx->dec_alf->filter_coeff_sym);
            ctx->dec_alf->filter_coeff_sym = NULL;
        }
        if (ctx->dec_alf->alf_picture_param)
        {
            for (g = 0; g < (int)N_C; g++)
            {
                free_alf_param(ctx->dec_alf->alf_picture_param[g], g);
            }
            free(ctx->dec_alf->alf_picture_param);
            ctx->dec_alf->alf_picture_param = NULL;
        }
        if (ctx->dec_alf)
        {
            free(ctx->dec_alf);
            ctx->dec_alf = NULL;
        }
    }
}

void allocate_alf_param(ALF_PARAM **alf_param, int comp_idx
#if ALF_SHAPE
                    , int num_coef
#endif
)
{
    *alf_param = (ALF_PARAM *)malloc(sizeof(ALF_PARAM));
    (*alf_param)->alf_flag = 0;
#if ALF_SHAPE
    (*alf_param)->num_coeff = num_coef;
#else
    (*alf_param)->num_coeff = ALF_MAX_NUM_COEF;
#endif
    (*alf_param)->filters_per_group = 1;
#if ALF_IMP
    (*alf_param)->dir_index = 0;
    (*alf_param)->max_filter_num = (comp_idx == Y_C) ? NO_VAR_BINS : 1;
#endif
    (*alf_param)->component_id = comp_idx;
    (*alf_param)->coeff_multi = NULL;
    (*alf_param)->filter_pattern = NULL;
    switch (comp_idx)
    {
    case Y_C:
#if ALF_SHAPE
        get_mem_2D_int(&((*alf_param)->coeff_multi), NO_VAR_BINS, num_coef);
#else
        get_mem_2D_int(&((*alf_param)->coeff_multi), NO_VAR_BINS, ALF_MAX_NUM_COEF);
#endif
        get_mem_1D_int(&((*alf_param)->filter_pattern), NO_VAR_BINS);
        break;
    case U_C:
    case V_C:
#if ALF_SHAPE
        get_mem_2D_int(&((*alf_param)->coeff_multi), 1, num_coef);
#else
        get_mem_2D_int(&((*alf_param)->coeff_multi), 1, ALF_MAX_NUM_COEF);
#endif
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
}

void free_alf_param(ALF_PARAM *alf_param, int comp_idx)
{
    switch (comp_idx)
    {
    case Y_C:
        free_mem_2D_int(alf_param->coeff_multi);
        free_mem_1D_int(alf_param->filter_pattern);
        break;
    case U_C:
    case V_C:
        free_mem_2D_int(alf_param->coeff_multi);
        break;
    default:
        printf("Not a legal component ID\n");
        assert(0);
        exit(-1);
    }
    free(alf_param);
    alf_param = NULL;
}

void derive_boundary_avail(DEC_CTX *ctx, int num_lcu_in_pic_width, int num_lcu_in_pic_height, int ctu, BOOL *is_left_avail,
                         BOOL *is_right_avail, BOOL *is_above_avail, BOOL *is_below_avail, BOOL *is_above_left_avail, BOOL *is_above_right_avail)
{
    int  num_lcu_in_frame = num_lcu_in_pic_height * num_lcu_in_pic_width;
    int  lcu_height = 1 << ctx->info.log2_max_cuwh;
    int  lcu_width = lcu_height;
    int  pic_x;
    int  pic_y;
    int  mb_x;
    int  mb_y;
    int  mb_curr;
    int  pic_mb_width = ctx->info.pic_width_in_scu;
    int  mb_left;
    int  mb_right;
    int  mb_above;
    int  mb_curr_patch_idx, mb_neighbor_patch_idx;
    int  mb_above_left;
    int  mb_above_right;
    int cross_patch_flag = ctx->info.sqh.cross_patch_loop_filter;
    pic_x = (ctu % num_lcu_in_pic_width) * lcu_width;
    pic_y = (ctu / num_lcu_in_pic_width) * lcu_height;

    mb_x = pic_x / MIN_CU_SIZE;
    mb_y = pic_y / MIN_CU_SIZE;
    mb_curr = mb_y * pic_mb_width + mb_x;
    *is_left_avail = (ctu % num_lcu_in_pic_width != 0);
    *is_right_avail = (ctu % num_lcu_in_pic_width != num_lcu_in_pic_width - 1);
    *is_above_avail = (ctu >= num_lcu_in_pic_width);
    *is_below_avail = (ctu  < num_lcu_in_frame - num_lcu_in_pic_width);
    *is_above_left_avail = *is_above_avail && *is_left_avail;
    *is_above_right_avail = *is_above_avail && *is_right_avail;
    s8* map_patch_idx = ctx->map.map_patch_idx;
    mb_left = *is_left_avail ? (mb_curr - 1) : -1;
    mb_right = *is_right_avail ? (mb_curr + (lcu_width >> MIN_CU_LOG2)) : -1;
    mb_above = *is_above_avail ? (mb_curr - pic_mb_width) : -1;
    mb_above_left = *is_above_left_avail ? (mb_curr - pic_mb_width - 1) : -1;
    mb_above_right = *is_above_right_avail ? (mb_curr - pic_mb_width + (lcu_width >> MIN_CU_LOG2)) : -1;
    if (!cross_patch_flag)
    {
        *is_left_avail = *is_right_avail = *is_above_avail = FALSE;
        *is_above_left_avail = *is_above_right_avail = FALSE;
        mb_curr_patch_idx = map_patch_idx[mb_curr];
        if (mb_left != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_left];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_left_avail = TRUE;
            }
        }
        if (mb_right != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_right];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_right_avail = TRUE;
            }
        }
        if (mb_above != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_above];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_above_avail = TRUE;
            }
        }
        if (mb_above_left != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_above_left];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_above_left_avail = TRUE;
            }
        }
        if (mb_above_right != -1)
        {
            mb_neighbor_patch_idx = map_patch_idx[mb_above_right];
            if (mb_curr_patch_idx == mb_neighbor_patch_idx)
            {
                *is_above_right_avail = TRUE;
            }
        }
    }
}