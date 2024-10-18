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


#include "enc_def.h"
#include "enc_mode.h"
#include "com_tbl.h"
#include "com_df.h"
#include <math.h>
#include "com_util.h"
#if ESAO || CCSAO
#include "enc_esao.h"
#include "com_esao.h"
#endif

#if ESAO
void get_multi_classes_statistics_for_esao(COM_PIC *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA *esao_state_data, int bit_depth, int comp_idx, int lcu_height, int lcu_width,
    int label_index, int lcu_pos, int pix_y, int pix_x, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
    int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int x, y;
    pel *rec_pic, *org_pic;
    rec_pic = NULL;
    org_pic = NULL;
    int rec_stride, org_stride;
    int shift_count = (label_index + 1);
    switch (comp_idx)
    {
    case Y_C:
        rec_stride = pic_esao->stride_luma;
        org_stride = pic_org->stride_luma;
        rec_pic = pic_esao->y;
        org_pic = pic_org->y;
        break;
    case U_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->u;
        org_pic = pic_org->u;
        break;
    case V_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->v;
        org_pic = pic_org->v;
        break;
    default:
        rec_stride = 0;
        org_stride = 0;
        rec_pic = NULL;
        org_pic = NULL;
        assert(0);
    }
    int  start_x_r, end_x_r, start_y, end_y;
    rec_pic += pix_y * rec_stride + pix_x;
    org_pic += pix_y * org_stride + pix_x;
    if (comp_idx != 0)
    {
        start_x_r = 0;
        start_y = 0;
        end_y = lcu_height;
        end_x_r = lcu_width;
    }
    else
    {
        start_x_r = (lcu_available_left) ? 0 : 1;
        start_y = (lcu_available_up) ? 0 : 1;
        end_y = (lcu_available_down) ? lcu_height : lcu_height - 1;
        end_x_r = (lcu_available_right) ? lcu_width : lcu_width - 1;
    }
    for (y = start_y; y < end_y; y++)
    {
        for (x = start_x_r; x < end_x_r; x++)
        {
            if (comp_idx == 0)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                int diff21 = 0, diff22 = 0, diff23 = 0, diff24 = 0;
                int diff25 = 0, diff26 = 0, diff27 = 0, diff28 = 0;
                //up
                if (rec_pic[(y - 1) * rec_stride + x] > rec_pic[y * rec_stride + x])
                {
                    diff2 = 1;
                    diff22 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x] < rec_pic[y * rec_stride + x])
                {
                    diff2 = -1;
                }
                //upleft
                if (rec_pic[(y - 1) * rec_stride + x - 1] > rec_pic[y * rec_stride + x])
                {
                    diff1 = 1;
                    diff21 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x - 1] < rec_pic[y * rec_stride + x])
                {
                    diff1 = -1;
                }
                //upright
                if (rec_pic[(y - 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff3 = 1;
                    diff23 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff3 = -1;
                }
                //left
                if (rec_pic[(y)* rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff4 = 1;
                    diff24 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff4 = -1;
                }
                //right
                if (rec_pic[(y)* rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff5 = 1;
                    diff25 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff5 = -1;
                }
                //down
                if (rec_pic[(y + 1) * rec_stride + x] > rec_pic[(y)* rec_stride + x])
                {
                    diff7 = 1;
                    diff27 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x] < rec_pic[(y)* rec_stride + x])
                {
                    diff7 = -1;
                }
                //leftdown
                if (rec_pic[(y + 1) * rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff6 = 1;
                    diff26 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff6 = -1;
                }
                //rightdown
                if (rec_pic[(y + 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff8 = 1;
                    diff28 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff8 = -1;
                }
                int diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8 + 8;
                int band_type = (rec_pic[y  * rec_stride + x] * shift_count) >> bit_depth;
                int true_Index = band_type * NUM_ESAO_LUMA_TYPE0 + diff_count;
                esao_state_data[0].diff[true_Index] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                esao_state_data[0].count[true_Index] ++;
                if (ESAO_LUMA_TYPES > 1)
                {
                    diff_count = diff21 + diff22 + diff23 + diff24 + diff25 + diff26 + diff27 + diff28;
                    true_Index = band_type * NUM_ESAO_LUMA_TYPE1 + diff_count;
                    esao_state_data[1].diff[true_Index] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                    esao_state_data[1].count[true_Index] ++;
                }
            }
            else
            {
                assert(comp_idx > 0 && comp_idx < 3);
                int index_shift = tab_esao_chroma_class[label_index];
                int band_type = (rec_pic[y  * rec_stride + x] * index_shift) >> bit_depth;
                esao_state_data[comp_idx - 1].diff[band_type] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                esao_state_data[comp_idx - 1].count[band_type] ++;
            }
        }
    }
}

#if ESAO_ENH
void get_frame_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA ***esao_luma_state_data, ESAO_STAT_DATA ***esao_chroma_state_data)
#else
void get_frame_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA **esao_luma_state_data, ESAO_STAT_DATA **esao_chroma_state_data)
#endif
{
    int bit_depth = info->bit_depth_internal;
    int lcu_pix_width ;
    int lcu_pix_height ;
    int comp_idx;
    int lcu_pix_height_t, lcu_pix_width_t;
    int is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_max_size_in_bit = info->log2_max_cuwh;
    int pix_y, pix_x, pix_x_t, pix_y_t;
    for (pix_y = 0; pix_y < pic_pix_height; pix_y += lcu_pix_height)
    {
        lcu_pix_height = min(1 << (input_max_size_in_bit), (pic_pix_height - pix_y));
        for (pix_x = 0; pix_x < pic_pix_width; pix_x += lcu_pix_width)
        {
            int x_in_lcu = pix_x >> info->log2_max_cuwh;
            int y_in_lcu = pix_y >> info->log2_max_cuwh;
            int lcu_pos = x_in_lcu + y_in_lcu * info->pic_width_in_lcu;
            lcu_pix_width = min(1 << (input_max_size_in_bit), (pic_pix_width - pix_x));
            for (comp_idx = Y_C; comp_idx < N_C; comp_idx++)
            {
                lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1)) : (lcu_pix_width);
                lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1)) : (lcu_pix_height);
                pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
                pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
                check_boundary_available_for_esao(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_avail,
                      &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
                int label_count = (comp_idx == Y_C) ? ESAO_LABEL_NUM_Y : ((comp_idx == U_C) ? ESAO_LABEL_NUM_U : ESAO_LABEL_NUM_V);
                for (int label_index = 0; label_index < label_count; label_index++)
                {
#if ESAO_ENH
                    get_multi_classes_statistics_for_esao(pic_org, pic_esao, ((comp_idx == Y_C) ? esao_luma_state_data[lcu_pos][label_index] : esao_chroma_state_data[lcu_pos][label_index]), bit_depth, comp_idx, lcu_pix_height_t, lcu_pix_width_t,
                        label_index, lcu_pos, pix_y_t, pix_x_t, is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
                    
#else
                    get_multi_classes_statistics_for_esao(pic_org, pic_esao, ((comp_idx == Y_C) ? esao_luma_state_data[label_index] : esao_chroma_state_data[label_index]), bit_depth, comp_idx, lcu_pix_height_t, lcu_pix_width_t,
                        label_index, lcu_pos, pix_y_t, pix_x_t, is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
#endif
                }
            }
        }
    }
}

long long int  distortion_cal_esao(long long int count, int offset, long long int diff)
{
    return (count * (long long int)offset * (long long int)offset - diff * offset * 2);
}

unsigned int esao_uvlcBitrate_estimate(int val)
{
    unsigned int length = 1;
    val++;
    assert(val);
    while (1 != val)
    {
        val >>= 1;
        length += 2;
    }
    return ((length >> 1) + ((length + 1) >> 1));
}

#if ESAO_PH_SYNTAX
static unsigned int esao_svlc_bit_estimate(int val)
{
    return esao_uvlcBitrate_estimate((val <= 0) ? (-val << 1) : ((val << 1) - 1));
}
#endif

int offset_esao_estimation(double lambda, int offset_ori, int count, long long int diff, double *best_cost)
{
    int cur_offset = offset_ori;
    int offset_best = 0;
    int lower_bd, upper_bd, Th;
    int temp_offset, start_offset, end_offset;
    int temp_rate;
    long long int temp_dist;
    double temp_cost, min_cost;
    int offset_type;
    int offset_step;
    offset_type = 0;
    lower_bd = esao_clip[offset_type][0];
    upper_bd = esao_clip[offset_type][1];
    Th = esao_clip[offset_type][2];
    offset_step = 1;
    cur_offset = COM_CLIP3(lower_bd, upper_bd, cur_offset);
    start_offset = cur_offset >= 0 ? 0 : cur_offset;
    end_offset = cur_offset >= 0 ? cur_offset : 0;
    min_cost = MAX_COST;
    for (temp_offset = start_offset; temp_offset <= end_offset; temp_offset += offset_step)
    {
        int offset = temp_offset;
        temp_rate = abs(offset);
        temp_rate = temp_rate ? (temp_rate + 1) : 0;
        temp_rate = (temp_rate == Th) ? temp_rate : (temp_rate + 1);
        temp_dist = distortion_cal_esao(count, temp_offset, diff);
        temp_cost = (double)temp_dist + lambda * (double)temp_rate;
        if (temp_cost < min_cost)
        {
            min_cost = temp_cost;
            offset_best = temp_offset;
            *best_cost = temp_cost;
        }
    }
    return offset_best;
}

#if ESAO_ENH
void find_esao_offset(ENC_CTX *ctx, int comp_idx, ESAO_REFINE_DATA *esao_state_date, ESAO_BLK_PARAM *esao_blk_param, double lambda)
{
    double class_cost[ESAO_LABEL_CLASSES_MAX];
    int set_num = esao_blk_param->set_num;
    int *type = esao_blk_param->luma_type;
    int *mode = esao_blk_param->mode;
    int class_num[ESAO_SET_NUM] = { 0 };
    for (int set = 0; set < set_num; set++)
    {
        if (comp_idx == Y_C)
        {
            class_num[set] = (mode[set] + 1)*(type[set] == 0 ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1);
        }
        else
        {
            class_num[set] = tab_esao_chroma_class[mode[set]];
        }
    }
    for (int set = 0; set < ESAO_SET_NUM; set++)
    {
        memset(esao_blk_param->offset[set], 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
        memset(esao_state_date->count[set], 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
        memset(esao_state_date->diff[set], 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
    }
    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        if (esao_blk_param->lcu_flag[lcu_pos])
        {
            int set = esao_blk_param->lcu_flag[lcu_pos] - 1;
            if (comp_idx == Y_C)
            {
                for (int class_i = 0; class_i < class_num[set]; class_i++)
                {
                    esao_state_date->count[set][class_i] += ctx->esao_luma_data[lcu_pos][mode[set]][type[set]].count[class_i];
                    esao_state_date->diff[set][class_i] += ctx->esao_luma_data[lcu_pos][mode[set]][type[set]].diff[class_i];
                }
            }
            else
            {
                for (int class_i = 0; class_i < class_num[set]; class_i++)
                {
                    esao_state_date->count[set][class_i] += ctx->esao_chroma_data[lcu_pos][mode[set]][comp_idx - 1].count[class_i];
                    esao_state_date->diff[set][class_i] += ctx->esao_chroma_data[lcu_pos][mode[set]][comp_idx - 1].diff[class_i];
                }
            }

        }
    }
    for (int set = 0; set < set_num; set++)
    {
        for (int class_i = 0; class_i < class_num[set]; class_i++)
        {
            if (esao_state_date->count[set][class_i] == 0)
            {
                esao_blk_param->offset[set][class_i] = 0;
                continue;
            }
            double offth = esao_state_date->diff[set][class_i] > 0 ? 0.5 : (esao_state_date->diff[set][class_i] < 0 ?
                -0.5 : 0);
            esao_blk_param->offset[set][class_i] = (int)((double)esao_state_date->diff[set][class_i] /
                (double)esao_state_date->count[set][class_i] + offth);
        }
        for (int class_i = 0; class_i < class_num[set]; class_i++)
        {
            if (esao_state_date->count[set][class_i] == 0)
            {
                continue;
            }
            esao_blk_param->offset[set][class_i] = offset_esao_estimation(lambda, esao_blk_param->offset[set][class_i],
                esao_state_date->count[set][class_i], esao_state_date->diff[set][class_i], &(class_cost[class_i]));
        }
    }
}

int compare_esao_lcu_cost(const void *a, const void *b)
{
    int    a_pos = ((ESAO_LCU_COST *)a)->lcu_pos, b_pos = ((ESAO_LCU_COST *)b)->lcu_pos;
    double a_cost = ((ESAO_LCU_COST *)a)->lcu_cost, b_cost = ((ESAO_LCU_COST *)b)->lcu_cost;
    if (a_cost == b_cost) return (a_pos < b_pos ? -1 : 1);
    else                  return (a_cost < b_cost ? -1 : 1);
}

int compare_esao_set_cnt(const void *a, const void *b)
{
    int a_set = ((ESAO_SET_CNT *)a)->set, b_set = ((ESAO_SET_CNT *)b)->set;
    int a_cnt = ((ESAO_SET_CNT *)a)->cnt, b_cnt = ((ESAO_SET_CNT *)b)->cnt;
    if (a_cnt == b_cnt) return (a_set < b_set ? -1 : 1);
    else                return (a_cnt > b_cnt ? -1 : 1);  //从大到小
}

void setup_temp_esao_param(ENC_CTX *ctx, int comp, int set_num, int type, int mode, ESAO_BLK_PARAM *temp_ccsao_param, ESAO_BLK_PARAM *init_ccsao_param)
{

    copy_esao_param_for_one_component(temp_ccsao_param, init_ccsao_param, ctx->info.f_lcu);
    temp_ccsao_param->set_num                = set_num;
    temp_ccsao_param->luma_type[set_num - 1] = type;
    temp_ccsao_param->mode[set_num - 1]      = mode;
}

void setup_init_esao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int set_num, ESAO_BLK_PARAM *init_esao_param, ESAO_BLK_PARAM *best_esao_param)
{
    if (set_num == 1)
    {
        for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
        {
            init_esao_param->lcu_flag[lcu_pos] = 1;
        }
        return;
    }

    copy_esao_param_for_one_component(init_esao_param, best_esao_param, ctx->info.f_lcu);

    int lcu_on_cnt = 0;
    ESAO_LCU_COST *lcu_cost = (ESAO_LCU_COST *)malloc(ctx->info.f_lcu * sizeof(ESAO_LCU_COST));
    ENC_SBAC *esao_sbac = GET_SBAC_ENC(ccsao_bs_temp);
    SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));
    for (int lcu_idx = 0; lcu_idx < ctx->info.f_lcu; lcu_idx++)
    {
        int lcu_rate = enc_get_bit_number(esao_sbac);
        enc_eco_esao_lcu_control_flag(esao_sbac, ccsao_bs_temp, init_esao_param->lcu_flag[lcu_idx], set_num);
        lcu_rate = enc_get_bit_number(esao_sbac) - lcu_rate;
        long long lcu_dist = 0;

        if (init_esao_param->lcu_flag[lcu_idx])
        {
            int set = init_esao_param->lcu_flag[lcu_idx] - 1;
            int type = init_esao_param->luma_type[set];
            int mode = init_esao_param->mode[set];
            lcu_dist = get_distortion_esao(comp, (comp == Y_C ? ctx->esao_luma_data[lcu_idx][mode][type] : ctx->esao_chroma_data[lcu_idx][mode][comp - 1]), init_esao_param, set);
            lcu_on_cnt++;
        }

        lcu_cost[lcu_idx].lcu_pos = lcu_idx;
        lcu_cost[lcu_idx].lcu_cost = (double)lcu_dist + RATE_TO_COST_LAMBDA(ctx->lambda_esao[comp], lcu_rate);
    }

    qsort(lcu_cost, ctx->info.f_lcu, sizeof(ESAO_LCU_COST), compare_esao_lcu_cost);

    for (int lcu_idx = 0; lcu_idx < ctx->info.f_lcu; lcu_idx++)
    {
        int lcu_pos = lcu_cost[lcu_idx].lcu_pos;
        if (lcu_idx < lcu_on_cnt)
        {
            if (lcu_idx * set_num > lcu_on_cnt * (set_num - 1))
            {
                init_esao_param->lcu_flag[lcu_pos] = set_num;
            }
        }
        else
        {
            init_esao_param->lcu_flag[lcu_pos] = 0;
        }
    }
    com_mfree(lcu_cost);
}

void update_temp_esao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, int comp, int iter, ESAO_BLK_PARAM *temp_esao_param, int *curr_rate, long long int *curr_dist)
{
    int       rate = 0;
    long long dist = 0;
    ENC_SBAC *esao_sbac = GET_SBAC_ENC(esao_bs_temp);

    if (temp_esao_param->set_num == 1 && iter == 0)
    {
        int mode = temp_esao_param->mode[0];
        int type = temp_esao_param->luma_type[0];

        for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
        {
            dist += get_distortion_esao(comp, (comp == Y_C ? ctx->esao_luma_data[lcu_pos][mode][type] : ctx->esao_chroma_data[lcu_pos][mode][comp - 1]), temp_esao_param, 0);
        }

        *curr_rate += rate;
        *curr_dist += dist;
        return;
    }

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        int       best_set = 0;
        double    lcu_cost_set = 0;
        int       lcu_rate_on = 0;
        long long lcu_dist_on = 0;
        double    lcu_cost_on = MAX_COST;

        for (int cand_set = 0; cand_set < temp_esao_param->set_num; cand_set++)
        {
            SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
            int lcu_rate_set = enc_get_bit_number(esao_sbac);
            enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, cand_set + 1, temp_esao_param->set_num);
            lcu_rate_set = enc_get_bit_number(esao_sbac) - lcu_rate_set;
            int cand_type = temp_esao_param->luma_type[cand_set];
            int cand_mode = temp_esao_param->mode[cand_set];

            long long lcu_dist_set = get_distortion_esao(comp, (comp == Y_C ? ctx->esao_luma_data[lcu_pos][cand_mode][cand_type] : ctx->esao_chroma_data[lcu_pos][cand_mode][comp - 1]), temp_esao_param, cand_set);
            lcu_cost_set = (double)lcu_dist_set + RATE_TO_COST_LAMBDA(ctx->lambda_esao[comp], lcu_rate_set);

            if (lcu_cost_set < lcu_cost_on)
            {
                best_set = cand_set;
                lcu_rate_on = lcu_rate_set;
                lcu_dist_on = lcu_dist_set;
                lcu_cost_on = lcu_cost_set;
                SBAC_STORE((core->s_esao_lcu_open), (*esao_sbac));
            }
        }

        SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
        int lcu_rate_off = enc_get_bit_number(esao_sbac);
        enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, FALSE, temp_esao_param->set_num);
        lcu_rate_off = enc_get_bit_number(esao_sbac) - lcu_rate_off;
        double lcu_cost_off = RATE_TO_COST_LAMBDA(ctx->lambda_esao[comp], lcu_rate_off);
        SBAC_STORE((core->s_esao_lcu_close), (*esao_sbac));

        if (lcu_cost_on < lcu_cost_off)
        {
            temp_esao_param->lcu_flag[lcu_pos] = best_set + 1;
            rate += lcu_rate_on;
            dist += lcu_dist_on;
            SBAC_STORE((core->s_esao_lcu_loop), (core->s_esao_lcu_open));
        }
        else
        {
            temp_esao_param->lcu_flag[lcu_pos] = FALSE;
            rate += lcu_rate_off;
            dist += 0;
            SBAC_STORE((core->s_esao_lcu_loop), (core->s_esao_lcu_close));
        }
    }

    ESAO_SET_CNT set_cnt[ESAO_SET_NUM];
    for (int idx = 0; idx < ESAO_SET_NUM; idx++)
    {
        set_cnt[idx].en = FALSE;
        set_cnt[idx].set = idx;
        set_cnt[idx].cnt = 0;
    }

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        if (temp_esao_param->lcu_flag[lcu_pos])
        {
            int set = temp_esao_param->lcu_flag[lcu_pos] - 1;
            set_cnt[set].en = TRUE;
            set_cnt[set].cnt++;
        }
    }

    qsort(set_cnt, ESAO_SET_NUM, sizeof(ESAO_SET_CNT), compare_esao_set_cnt);

    s8 set_map[ESAO_SET_NUM];
    memset(set_map, -1, sizeof(s8) * ESAO_SET_NUM);
    for (int set = 0; set < ESAO_SET_NUM; set++)
    {
        for (int idx = 0; idx < ESAO_SET_NUM; idx++)
        {
            if (set_cnt[idx].en == TRUE && set_cnt[idx].set == set)
            {
                set_map[set] = idx;
                break;
            }
        }
    }

    BOOL remap = FALSE;
    for (int set = 0; set < temp_esao_param->set_num; set++)
    {
        remap = set_map[set] != set ? TRUE : remap;
    }

    if (!remap)
    {
        *curr_rate += rate;
        *curr_dist += dist;
        return;
    }

    int temp_type[ESAO_SET_NUM];
    int temp_mode[ESAO_SET_NUM];
    int temp_chroma_flag[ESAO_SET_NUM];
    int temp_chroma_len[ESAO_SET_NUM];
    int temp_chroma_star_band[ESAO_SET_NUM];
    int temp_offset[ESAO_SET_NUM][ESAO_LABEL_CLASSES_MAX];
    memcpy(temp_type, temp_esao_param->luma_type, sizeof(int) * ESAO_SET_NUM);
    memcpy(temp_mode, temp_esao_param->mode, sizeof(int) * ESAO_SET_NUM);
    memcpy(temp_chroma_flag, temp_esao_param->chroma_band_flag, sizeof(int) * ESAO_SET_NUM);
    memcpy(temp_chroma_len, temp_esao_param->chroma_band_length, sizeof(int) * ESAO_SET_NUM);
    memcpy(temp_chroma_star_band, temp_esao_param->chroma_start_band, sizeof(int) * ESAO_SET_NUM);
    memcpy(temp_offset, temp_esao_param->offset, sizeof(int) * ESAO_SET_NUM * ESAO_LABEL_CLASSES_MAX);

    temp_esao_param->set_num = 0;
    for (int idx = 0; idx < ESAO_SET_NUM; idx++)
    {
        temp_esao_param->set_num += set_cnt[idx].en ? 1 : 0;
        int set = set_cnt[idx].set;
        temp_esao_param->luma_type[idx] = temp_type[set];
        temp_esao_param->mode[idx] = temp_mode[set];

        temp_esao_param->chroma_band_flag[idx] = temp_chroma_flag[set];
        temp_esao_param->chroma_band_length[idx] = temp_chroma_len[set];
        temp_esao_param->chroma_start_band[idx] = temp_chroma_star_band[set];



        memcpy(temp_esao_param->offset[idx], temp_offset[set], sizeof(int) * ESAO_LABEL_CLASSES_MAX);
    }

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        int lcu_flag = temp_esao_param->lcu_flag[lcu_pos];
        if (lcu_flag)
        {
            int set = lcu_flag - 1;
            temp_esao_param->lcu_flag[lcu_pos] = set_map[set] + 1;
        }
    }

    SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
    rate = enc_get_bit_number(esao_sbac);
    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, temp_esao_param->lcu_flag[lcu_pos], temp_esao_param->set_num);
    }
    rate = enc_get_bit_number(esao_sbac) - rate;

    *curr_rate += rate;
    *curr_dist += dist;
}

long long int get_distortion_esao(int comp_idx, ESAO_STAT_DATA esao_state_data, ESAO_BLK_PARAM *esao_cur_param, int set)
{
    int class_idc;
    volatile long long int dist = 0;
    int class_index;
    int mode_index = esao_cur_param->mode[set];
    int types = esao_cur_param->luma_type[set];
    if (comp_idx == Y_C)
    {
        int num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
        class_index = (mode_index + 1) * num;
    }
    else
    {
        class_index = tab_esao_chroma_class[mode_index];
    }
    for (class_idc = 0; class_idc < class_index; class_idc++)
    {
        dist += distortion_cal_esao(esao_state_data.count[class_idc], esao_cur_param->offset[set][class_idc],
            esao_state_data.diff[class_idc]);
    }
    return dist;
}

int uv_not_equal_count(ESAO_BLK_PARAM *esao_cur_param, int *begins, int mode_index, int set_num)
{
    int count = 0;
    int class_idc;
    int class_index = tab_esao_chroma_class[mode_index];
    for (class_idc = 0; class_idc < class_index; class_idc++)
    {
        if (esao_cur_param->offset[set_num][class_idc] != 0)
        {
            begins[0] = class_idc; //start the band not equal zero
            break;
        }
    }
    for (class_idc = class_index - 1; class_idc >= 0; class_idc--)
    {
        if (esao_cur_param->offset[set_num][class_idc] != 0)

        {
            begins[1] = class_idc; //end the band not equal zero
            break;
        }
    }
    count = begins[1] - begins[0] + 1;
    return count;
}

int get_esao_offset_rate(ENC_CORE *core, int comp, COM_BSW *esao_bs_temp, ESAO_BLK_PARAM *temp_esao_param)
{
    int rate = 0;
    for (int set = 0; set < temp_esao_param->set_num; set++)
    {
        int num = temp_esao_param->luma_type[set] == 0 ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
        int class_count = comp == Y_C ? num * (temp_esao_param->mode[set] + 1) : tab_esao_chroma_class[temp_esao_param->mode[set]];

        int begins[2] = { 0,0 }, cur_rate = 0;
        if (comp != Y_C)
        {
            int opt_count = uv_not_equal_count(temp_esao_param, begins, temp_esao_param->mode[set], set);

            cur_rate = 1;
            cur_rate += esao_uvlcBitrate_estimate(begins[0]);
            cur_rate += esao_uvlcBitrate_estimate(opt_count);
            for (int i = 0; i < opt_count; i++)
            {
                cur_rate += esao_svlc_bit_estimate(temp_esao_param->offset[set][i + begins[0]]);
            }
            int off_rate = 1;
            for (int i = 0; i < class_count; i++)
            {
                off_rate += esao_svlc_bit_estimate(temp_esao_param->offset[set][i]);
            }
            if (off_rate < cur_rate)
            {
                cur_rate = off_rate;
                temp_esao_param->chroma_band_flag[set] = 0;
            }
            else
            {
                temp_esao_param->chroma_band_flag[set] = 1;
                temp_esao_param->chroma_start_band[set] = begins[0];
                temp_esao_param->chroma_band_length[set] = opt_count;
            }
        }
        else
        {
            cur_rate = 0;
            for (int i = 0; i < class_count; i++)
            {
                cur_rate += esao_svlc_bit_estimate(temp_esao_param->offset[set][i]);
            }
        }
        rate += cur_rate;
    }
    return rate;
}

void esao_rdcost_for_mode_new_yuv_each(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, ESAO_BLK_PARAM *best_esao_param, ESAO_BLK_PARAM *temp_esao_param, int mode_index, int set_num, int type,
    double *best_cost, double *temp_cost, int comp_idx)
{

    ENC_SBAC *esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    double    cur_cost = 0;
    double    min_cost = 0;
    int       iter = 0;
    int       max_iter = 15;
    BOOL      improved = FALSE;
    BOOL      optimizing = TRUE;

    ESAO_REFINE_DATA esao_refine_data;

    while (optimizing)
    {
        SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));

        find_esao_offset(ctx, comp_idx, &esao_refine_data, temp_esao_param, ctx->lambda_esao[comp_idx]);
        int cur_rate = 0;
        long long int cur_dist = 0;
        SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac));
        update_temp_esao_param(ctx, core, esao_bs_temp, comp_idx, iter, temp_esao_param, &cur_rate, &cur_dist);

        cur_rate += get_esao_offset_rate(core, comp_idx, esao_bs_temp, temp_esao_param);
        cur_rate += (comp_idx == Y_C) ? ESAO_LABEL_NUM_IN_BIT_Y + 1 + 1 + ESAO_SET_NUM_BIT : ESAO_LABEL_NUM_IN_BIT_V + 1 + ESAO_SET_NUM_BIT;
        cur_cost = (double)(cur_dist)+RATE_TO_COST_LAMBDA(ctx->lambda_esao[comp_idx], cur_rate);

        if (cur_cost < min_cost)
        {
            min_cost = cur_cost;
            improved = TRUE;
            temp_cost[set_num - 1] = cur_cost;
        }
        else
        {
            improved = FALSE;
        }
        if (temp_cost[set_num - 1] < best_cost[set_num - 1])
        {
            best_cost[set_num - 1] = temp_cost[set_num - 1];
            SBAC_STORE((core->s_esao_cur_best), (*esao_sbac));
            copy_esao_param_for_one_component(best_esao_param, temp_esao_param, ctx->info.f_lcu);
        }
        iter++;
        optimizing = improved && iter < max_iter;
    }
}

void esao_rdcost_for_mode_new(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp)
{
    double best_cost[N_C][ESAO_SET_NUM];
    double temp_cost[N_C][ESAO_SET_NUM];
    memset(best_cost, 0, sizeof(double)*N_C*ESAO_SET_NUM);
    memset(temp_cost, 0, sizeof(double)*N_C*ESAO_SET_NUM);

    int type_num[3] = { 2,1,1 };
    int label_yuv[N_C] = { ESAO_LABEL_NUM_Y,ESAO_LABEL_NUM_U,ESAO_LABEL_NUM_V };

    ENC_SBAC * esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_new), (*esao_sbac));
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));

        for (int set_num = 1; set_num <= ESAO_SET_NUM; set_num++)
        {
            setup_init_esao_param(ctx, core, esao_bs_temp, comp_idx, set_num, &ctx->init_esao_param[comp_idx], &ctx->best_esao_param[comp_idx]);
            best_cost[comp_idx][set_num - 1] = set_num >= 2 ? best_cost[comp_idx][set_num - 2] : 0;
            for (int type = 0; type < type_num[comp_idx]; type++)
            {
                for (int mode = 0; mode < label_yuv[comp_idx]; mode++)
                {
                    setup_temp_esao_param(ctx, comp_idx, set_num, type, mode, &ctx->temp_esao_param[comp_idx], &ctx->init_esao_param[comp_idx]);
                    esao_rdcost_for_mode_new_yuv_each(ctx, core, esao_bs_temp, &ctx->best_esao_param[comp_idx], &ctx->temp_esao_param[comp_idx], mode, set_num, type, best_cost[comp_idx], temp_cost[comp_idx], comp_idx);
                }
            }
        }
        int lcu_cnt = 0;
        for (int lcu_idx = 0; lcu_idx < ctx->info.f_lcu; lcu_idx++)
        {
            if (ctx->best_esao_param[comp_idx].lcu_flag[lcu_idx]) lcu_cnt++;
        }
        ctx->info.pic_header.pic_esao_on[comp_idx] = lcu_cnt > 0 ? TRUE : FALSE;
        ctx->info.pic_header.esao_lcu_enable[comp_idx] = (lcu_cnt > 0 && ctx->info.f_lcu != lcu_cnt) || ctx->best_esao_param[comp_idx].set_num >= 2 ? TRUE : FALSE;

        if (ctx->info.pic_header.pic_esao_on[comp_idx])
        {
            copy_esao_param_for_one_component(&(ctx->info.pic_header.pic_esao_params[comp_idx]), &ctx->best_esao_param[comp_idx], ctx->info.f_lcu);;

            ctx->info.pic_header.esao_set_num[comp_idx] = ctx->best_esao_param[comp_idx].set_num;
            for (int set = 0; set < ctx->info.pic_header.esao_set_num[comp_idx]; set++)
            {
                ctx->info.pic_header.esao_adaptive_param[comp_idx][set] = ctx->best_esao_param[comp_idx].mode[set];
                if (comp_idx == Y_C)
                {
                    ctx->info.pic_header.esao_luma_type[set] = ctx->best_esao_param[comp_idx].luma_type[set];
                }
                if (comp_idx != Y_C)
                {
                    ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1][set] = ctx->best_esao_param[comp_idx].chroma_band_flag[set];
                    ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1][set] = ctx->best_esao_param[comp_idx].chroma_band_length[set];
                    ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1][set] = ctx->best_esao_param[comp_idx].chroma_start_band[set];
                }
            }
            SBAC_STORE((core->s_esao_cur_blk), (core->s_esao_cur_best));
        }
        SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));
    }
    SBAC_LOAD((*esao_sbac), (core->s_esao_cur_new));
}
#else
void get_statistics_for_esao_one_LCU(COM_PIC *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA *lcu_state_data, int bit_depth, int comp_idx, int lcu_height, int lcu_width,
    int label_index, int is_template_one, int pix_y, int pix_x, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
    int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int x, y;
    pel *rec_pic, *org_pic;
    rec_pic = NULL;
    int rec_stride, org_stride;
    int shift_count = (label_index + 1);
    switch (comp_idx)
    {
    case Y_C:
        rec_stride = pic_esao->stride_luma;
        org_stride = pic_org->stride_luma;
        rec_pic = pic_esao->y;
        org_pic = pic_org->y;
        break;
    case U_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->u;
        org_pic = pic_org->u;
        break;
    case V_C:
        rec_stride = pic_esao->stride_chroma;
        org_stride = pic_org->stride_chroma;
        rec_pic = pic_esao->v;
        org_pic = pic_org->v;
        break;
    default:
        rec_stride = 0;
        org_stride = 0;
        rec_pic = NULL;
        org_pic = NULL;
        assert(0);
    }
    int  start_x_r, end_x_r, start_y, end_y;
    rec_pic += pix_y * rec_stride + pix_x;
    org_pic += pix_y * org_stride + pix_x;
    if (comp_idx != 0)
    {
        start_x_r = 0;
        start_y = 0;
        end_y = lcu_height;
        end_x_r = lcu_width;
    }
    else 
    {
        start_x_r = (lcu_available_left) ? 0 : 1;
        start_y = (lcu_available_up) ? 0 : 1;
        end_y = (lcu_available_down) ? lcu_height : lcu_height - 1;
        end_x_r = (lcu_available_right) ? lcu_width : lcu_width - 1;
    }
    for (y = start_y; y < end_y; y++)
    {
        for (x = start_x_r; x < end_x_r; x++)
        {
            if (comp_idx == 0)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                int diff21 = 0, diff22 = 0, diff23 = 0, diff24 = 0;
                int diff25 = 0, diff26 = 0, diff27 = 0, diff28 = 0;
                //up
                if (rec_pic[(y - 1) * rec_stride + x] > rec_pic[y * rec_stride + x]) //
                {
                    diff2 = 1;
                    diff22 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x] < rec_pic[y * rec_stride + x])
                {
                    diff2 = -1;
                }
                //upleft
                if (rec_pic[(y - 1) * rec_stride + x - 1] > rec_pic[y * rec_stride + x])
                {
                    diff1 = 1;
                    diff21 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x - 1] < rec_pic[y * rec_stride + x])
                {
                    diff1 = -1;
                }
                //upright
                if (rec_pic[(y - 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff3 = 1;
                    diff23 = 1;
                }
                else if (rec_pic[(y - 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff3 = -1;
                }
                //left
                if (rec_pic[(y)* rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff4 = 1;
                    diff24 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff4 = -1;
                }
                //right
                if (rec_pic[(y)* rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff5 = 1;
                    diff25 = 1;
                }
                else if (rec_pic[(y)* rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff5 = -1;
                }
                //down
                if (rec_pic[(y + 1) * rec_stride + x] > rec_pic[(y)* rec_stride + x])
                {
                    diff7 = 1;
                    diff27 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x] < rec_pic[(y)* rec_stride + x])
                {
                    diff7 = -1;
                }
                //downleft
                if (rec_pic[(y + 1) * rec_stride + x - 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff6 = 1;
                    diff26 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x - 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff6 = -1;
                }
                //downright
                if (rec_pic[(y + 1) * rec_stride + x + 1] > rec_pic[(y)* rec_stride + x])
                {
                    diff8 = 1;
                    diff28 = 1;
                }
                else if (rec_pic[(y + 1) * rec_stride + x + 1] < rec_pic[(y)* rec_stride + x])
                {
                    diff8 = -1;
                }
                int diff_count, num;
                int band_type = (rec_pic[y  * rec_stride + x] * shift_count) >> bit_depth;
                if (is_template_one)
                {
                    diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8 + 8;
                    num = NUM_ESAO_LUMA_TYPE0;
                }
                else 
                {
                    diff_count = diff21 + diff22 + diff23 + diff24 + diff25 + diff26 + diff27 + diff28;
                    num = NUM_ESAO_LUMA_TYPE1;
                }
                int true_index = band_type * num + diff_count;
                lcu_state_data->diff[true_index] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                lcu_state_data->count[true_index] ++;
            }
            else
            {
                int index_shift = tab_esao_chroma_class[label_index];
                int band_type = (rec_pic[y  * rec_stride + x] * index_shift) >> bit_depth;
                lcu_state_data->diff[band_type] += org_pic[y  * org_stride + x] - rec_pic[y  * rec_stride + x];
                lcu_state_data->count[band_type] ++;
            }
        }
    }
}

void get_lcu_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA *lcu_state_data, int comp_idx, int lcu_pos, int label_index, int is_template_one)
{
    int bit_depth = info->bit_depth_internal;
    int lcu_pix_width;
    int lcu_pix_height;
    int lcu_pix_height_t, lcu_pix_width_t;
    int  lcu_height = 1 << info->log2_max_cuwh;
    int lcu_width = lcu_height;
    int  is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail,
        is_below_right_avail;
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_max_size_in_bit = info->log2_max_cuwh;
    int pix_y, pix_x, pix_x_t, pix_y_t;
    //1.calculate the position according to lcu_pos
    pix_y = (lcu_pos / info->pic_width_in_lcu)*lcu_height;
    pix_x = (lcu_pos %info->pic_width_in_lcu)*lcu_width;
    lcu_pix_width = min(1 << (input_max_size_in_bit), (pic_pix_width - pix_x));
    lcu_pix_height = min(1 << (input_max_size_in_bit), (pic_pix_height - pix_y));
    lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1)) : (lcu_pix_width);
    lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1)) : (lcu_pix_height);
    pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
    pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
    check_boundary_available_for_esao(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_avail,
        &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
    get_statistics_for_esao_one_LCU(pic_org, pic_esao, lcu_state_data, bit_depth, comp_idx, lcu_pix_height_t, lcu_pix_width_t, label_index, is_template_one, pix_y_t, pix_x_t,
        is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
}

int uv_not_equal_count(ESAO_BLK_PARAM *esao_cur_param,int *begins, int mode_index)
{
    int count = 0;
    int class_idc;
    int class_index = tab_esao_chroma_class[mode_index];
    for (class_idc = 0; class_idc < class_index; class_idc++)
    {
        if (esao_cur_param->offset[class_idc] != 0)
        {
                begins[0] = class_idc; //start the band not equal zero
                break;
        }
    }
    for (class_idc = class_index-1; class_idc >=0; class_idc--)
    {
        if (esao_cur_param->offset[class_idc] != 0)
        {
            begins[1] = class_idc; //end the band not equal zero
            break;
        }
    }
    count = begins[1] - begins[0] + 1;
    return count;
}

long long int get_distortion_esao(int comp_idx,ESAO_STAT_DATA esao_state_data, ESAO_BLK_PARAM *esao_cur_param, int mode_index,int types)
{
    int class_idc;
    volatile long long int dist = 0;
    int class_index;
    if (comp_idx == Y_C)
    {
        int num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
        class_index = (mode_index + 1) * num;
    }
    else
    {
        class_index = tab_esao_chroma_class[mode_index];
    }
    for (class_idc = 0; class_idc < class_index; class_idc++)
    {
        dist += distortion_cal_esao(esao_state_data.count[class_idc], esao_cur_param->offset[class_idc],
            esao_state_data.diff[class_idc]);
    }
    return dist;
}

void find_esao_offset(int comp_idx, ESAO_STAT_DATA esao_state_date, ESAO_BLK_PARAM *esao_blk_param, double lambda, int mode_index, int types)
{
    int class_i;
    double class_cost[ESAO_LABEL_CLASSES_MAX];
    double offth;
    int num_class ,num;
    if (comp_idx == Y_C)
    {
        num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
        num_class = (mode_index + 1) * num;
    }
    else
    {
        num_class = tab_esao_chroma_class[mode_index];
    }
    for (int i = 0; i < ESAO_LABEL_CLASSES_MAX; i++)
    {
        esao_blk_param->offset[i] = 0;
    }
    for (class_i = 0; class_i < num_class; class_i++)
    {
        if (esao_state_date.count[class_i] == 0)
        {
            esao_blk_param->offset[class_i] = 0;
            continue;
        }
        offth = esao_state_date.diff[class_i] > 0 ? 0.5 : (esao_state_date.diff[class_i] < 0 ?
            -0.5 : 0);
        esao_blk_param->offset[class_i] = (int)((double)esao_state_date.diff[class_i] /
            (double)esao_state_date.count[class_i] + offth);
    }
    for (class_i = 0; class_i < num_class; class_i++)
    {
        if (esao_state_date.count[class_i] == 0)
        {
            esao_blk_param->offset[class_i] = 0;
            continue;
        }
        esao_blk_param->offset[class_i] = offset_esao_estimation(lambda, esao_blk_param->offset[class_i],
            esao_state_date.count[class_i], esao_state_date.diff[class_i], &(class_cost[class_i]));
    }
}

void esao_rdcost_for_mode_new_yuv_each(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, ESAO_BLK_PARAM *rec_esao_cur_param, int mode_index,
    double *cost_count, int *types, int comp_idx, int uv_offset_count[2], int start_band[2], int esao_chroma_band_flag[2])
{
    ENC_SBAC *esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_type), (*esao_sbac));
    SBAC_STORE((core->s_esao_cur_best), (*esao_sbac));
    long long int cur_dist;
    int  cur_rate; 
    int remeFlag = 0;
    double cur_cost, min_cost;
    int class_count;
    int type_num, loops, num;
    ESAO_BLK_PARAM temp_esao_param[ESAO_LUMA_TYPES];
    type_num = (comp_idx == Y_C) ? ESAO_LUMA_TYPES : 1;
    min_cost = 0;
    for (loops = 0; loops < type_num; loops++)
    {
        SBAC_LOAD((*esao_sbac), (core->s_esao_cur_type));
        if (comp_idx == Y_C)
        {
            num = (loops == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
            class_count = (mode_index + 1) * num;
        }
        else 
        {
            class_count = tab_esao_chroma_class[mode_index];
        }
        if (comp_idx == Y_C)
        {
            find_esao_offset(comp_idx, ctx->esao_luma_data[mode_index][loops], &temp_esao_param[loops], esao_lambda[comp_idx], mode_index, loops);
            cur_dist = get_distortion_esao(comp_idx, ctx->esao_luma_data[mode_index][loops], &temp_esao_param[loops], mode_index, loops);
        }
        else
        {
            find_esao_offset(comp_idx, ctx->esao_chroma_data[mode_index][comp_idx - 1], &temp_esao_param[loops], esao_lambda[comp_idx], mode_index, loops);
            cur_dist = get_distortion_esao(comp_idx, ctx->esao_chroma_data[mode_index][comp_idx - 1], &temp_esao_param[loops], mode_index, loops);
        }
        int count_not_zero=0;
        int class_count_ori = class_count;
        int begins[2] = { 0,0 };
        int uvop = 1;
        if (comp_idx != Y_C)
        {
            count_not_zero = uv_not_equal_count(&temp_esao_param[loops], begins, mode_index);
            class_count = count_not_zero;
        }
#if ESAO_PH_SYNTAX
        if (comp_idx!=Y_C)
        {
            cur_rate = 1; 
            cur_rate += esao_uvlcBitrate_estimate(begins[0]);
            cur_rate += esao_uvlcBitrate_estimate(class_count);
            for (int i = 0; i < class_count; i++)
            {
                cur_rate += esao_svlc_bit_estimate(temp_esao_param[loops].offset[i + begins[0]]);
            }
            int off_rate = 1;
            for (int i = 0; i < class_count_ori; i++)
            {
                off_rate += esao_svlc_bit_estimate(temp_esao_param[loops].offset[i]);
            }
            if (off_rate<cur_rate)
            {
                cur_rate = off_rate;
                uvop = 0;
            }
            else
            {
                uvop = 1;
            }
        }
        else 
        {
            cur_rate = 0;
            for (int i = 0; i < class_count_ori; i++)
            {
                cur_rate += esao_svlc_bit_estimate(temp_esao_param[loops].offset[i]);
            }
        }
        cur_rate += (comp_idx == Y_C) ? ESAO_LABEL_NUM_IN_BIT_Y + 1 + 1 : ESAO_LABEL_NUM_IN_BIT_V + 1;

#else
        if (comp_idx != Y_C)
        {
            //try to optimize the chroma filter coeff
            cur_rate = enc_get_bit_number(esao_sbac);
            eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 1);
            eco_esao_chroma_len(class_count, begins[0], mode_index, esao_sbac, esao_bs_temp);
            for (int i = 0; i < class_count; i++)
            {
                eco_esao_offset_AEC(temp_esao_param[loops].offset[i + begins[0]], esao_sbac, esao_bs_temp);
            }
            cur_rate = enc_get_bit_number(esao_sbac) - cur_rate;
            SBAC_STORE((core->s_esao_uvop), (*esao_sbac));
            SBAC_LOAD((*esao_sbac), (core->s_esao_cur_type));
            eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 0);
            int off_rate = enc_get_bit_number(esao_sbac);
            for (int i = 0; i < class_count_ori; i++)
            {
                eco_esao_offset_AEC(temp_esao_param[loops].offset[i], esao_sbac, esao_bs_temp);
            }
            off_rate = enc_get_bit_number(esao_sbac) - off_rate;
            if (off_rate < cur_rate)
            {
                cur_rate = off_rate;
                uvop = 0;
            }
            else
            {
                uvop = 1;
                SBAC_LOAD((*esao_sbac), (core->s_esao_uvop));
            }
        }
        else
        {
            cur_rate = enc_get_bit_number(esao_sbac);
            for (int i = 0; i < class_count_ori; i++)
            {
                eco_esao_offset_AEC(temp_esao_param[loops].offset[i], esao_sbac, esao_bs_temp);
            }
            cur_rate = enc_get_bit_number(esao_sbac) - cur_rate;
        }
        cur_rate += esao_uvlcBitrate_estimate(mode_index);
#endif

        cur_cost = (double)(cur_dist)+RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], cur_rate);
        if (cur_cost < min_cost)
        {
            min_cost = cur_cost;
            cost_count[comp_idx] = cur_cost;
            remeFlag = 1;
            if (comp_idx == Y_C)
            {
                (*types) = loops;
            }
            SBAC_STORE((core->s_esao_cur_best), (*esao_sbac));
            copy_esao_param_for_one_component(rec_esao_cur_param, &(temp_esao_param[loops]));
            if (comp_idx != Y_C)
            {
                esao_chroma_band_flag[comp_idx - 1] = uvop;
                uv_offset_count[comp_idx - 1] = count_not_zero;
                start_band[comp_idx - 1] = begins[0];
            }
        }
    }
    SBAC_LOAD((*esao_sbac), (core->s_esao_cur_best));
}

void esao_rdcost_for_mode_new(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *min_cost)
{
    ESAO_BLK_PARAM rec_esao_cur_param;
    double cost_yuv[N_C] = { MAX_COST ,MAX_COST ,MAX_COST };
    int label_yuv[N_C] = { ESAO_LABEL_NUM_Y,ESAO_LABEL_NUM_U,ESAO_LABEL_NUM_V };
    ENC_SBAC * esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_new), (*esao_sbac));
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));
        SBAC_STORE((core->s_esao_next_type), (*esao_sbac));
        for (int mode = 0; mode < label_yuv[comp_idx]; mode++)
        {
            SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));
            cost_yuv[comp_idx] = MAX_COST;
            int types;
            int uv_offset_count[2];
            int start_band[2];
            int esao_chroma_band_flag[2];
            esao_rdcost_for_mode_new_yuv_each(ctx, core, esao_bs_temp, esao_lambda, &rec_esao_cur_param, mode, cost_yuv, &types, comp_idx , 
                uv_offset_count, start_band, esao_chroma_band_flag);
            if (cost_yuv[comp_idx] < min_cost[comp_idx])
            {
                min_cost[comp_idx] = cost_yuv[comp_idx];
                ctx->info.pic_header.esao_adaptive_param[comp_idx] = mode;
                ctx->info.pic_header.pic_esao_on[comp_idx] = 1;
                if (comp_idx == Y_C)
                {
                    ctx->info.pic_header.esao_luma_type = types;
                    assert(ctx->info.pic_header.esao_luma_type == 0 || ctx->info.pic_header.esao_luma_type == 1);
                }
                if (comp_idx != Y_C )
                {
                    if (esao_chroma_band_flag[comp_idx - 1])
                    {
                        ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1] = 1;
                        ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1] = uv_offset_count[comp_idx - 1];
                        ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1] = start_band[comp_idx - 1];
                    }
                    else
                    {
                        ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1] = 0;
                    }
                }
#if ESAO_PH_SYNTAX
                copy_esao_param_for_one_component(&(ctx->info.pic_header.pic_esao_params[comp_idx]), &rec_esao_cur_param);
#else
                copy_esao_param_for_one_component(&(ctx->pic_esao_params[comp_idx]), &rec_esao_cur_param);
#endif
                SBAC_STORE((core->s_esao_next_type), (*esao_sbac));
            }
        }
        SBAC_LOAD((*esao_sbac), (core->s_esao_next_type));
    }
    SBAC_LOAD((*esao_sbac), (core->s_esao_cur_new));
}

void esao_rdcost_for_mode_lcu(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *cost_best)
{
    ENC_SBAC * esao_sbac = GET_SBAC_ENC(esao_bs_temp);
    SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));
    int comp_idx;
    int  cur_rate;
    int  modex_index;
    int *lcu_open_control_flag;
    ESAO_STAT_DATA lcu_stata_data;
    int class_count;
    int types, num;
    int is_template_one=0;
    int lcu_count = ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0));
    lcu_open_control_flag = (int *)malloc(sizeof(int)*lcu_count);
    for (comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        if (ctx->info.pic_header.pic_esao_on[comp_idx])
        {
            types = (comp_idx == Y_C) ? ctx->info.pic_header.esao_luma_type : 0;
            modex_index = ctx->info.pic_header.esao_adaptive_param[comp_idx]; // class
            if (comp_idx == Y_C)
            {
                is_template_one = (types == 0) ? 1 : 0;
                num = (types == 0) ? NUM_ESAO_LUMA_TYPE0 : NUM_ESAO_LUMA_TYPE1;
                class_count = (modex_index + 1) * num;
            }
            else 
            {
                class_count = tab_esao_chroma_class[modex_index];
            }
            SBAC_LOAD((*esao_sbac), (core->s_esao_cur_blk));
#if ESAO_PH_SYNTAX
            cur_rate = (comp_idx != Y_C) ? 1 : 0;
            if (comp_idx != Y_C && ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1])
            {
                cur_rate += esao_uvlcBitrate_estimate(ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1]);
                cur_rate += esao_uvlcBitrate_estimate(ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1]);
                for (int i = 0; i < ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1]; i++)
                {
                    cur_rate += esao_svlc_bit_estimate(ctx->info.pic_header.pic_esao_params[comp_idx].offset[i + ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1]]);
                }
            }
            else
            {
                for (int i = 0; i < class_count; i++)
                {
                    cur_rate += esao_svlc_bit_estimate(ctx->info.pic_header.pic_esao_params[comp_idx].offset[i]);
                }
            }
            cur_rate += (comp_idx == Y_C) ? ESAO_LABEL_NUM_IN_BIT_Y + 1 + 1 : ESAO_LABEL_NUM_IN_BIT_V + 1;
#else
            cur_rate = enc_get_bit_number(esao_sbac);
            if (comp_idx != Y_C && ctx->info.pic_header.esao_chroma_band_flag[comp_idx - 1])
            {
                eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 1);
                class_count = ctx->info.pic_header.esao_chroma_band_length[comp_idx - 1];
                eco_esao_chroma_len(class_count, ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1], modex_index, esao_sbac, esao_bs_temp);
                for (int i = 0; i < class_count; i++)
                {
                    eco_esao_offset_AEC(ctx->pic_esao_params[comp_idx].offset[i + ctx->info.pic_header.esao_chroma_start_band[comp_idx - 1]], esao_sbac, esao_bs_temp);
                }
            }
            else
            {
                if (comp_idx != Y_C)
                {
                    eco_esao_chroma_band_flag(esao_sbac, esao_bs_temp, 0);
                }
                for (int i = 0; i < class_count; i++)
                {
                    eco_esao_offset_AEC(ctx->pic_esao_params[comp_idx].offset[i], esao_sbac, esao_bs_temp);
                }
            }
            cur_rate = enc_get_bit_number(esao_sbac) - cur_rate;
            cur_rate += esao_uvlcBitrate_estimate(modex_index);
#endif
            double one_lcu_open_cost, one_lcu_close_cost;
            int one_lcu_rate;
            double cost_lcu_total = 0;
            SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac));
            for (int lcu_index = 0; lcu_index < lcu_count; lcu_index++)
            {
                SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
                memset(lcu_stata_data.diff, 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
                memset(lcu_stata_data.count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
                get_lcu_statistics_for_esao(&ctx->info, &ctx->map, PIC_ORG(ctx), PIC_REC(ctx), &lcu_stata_data, comp_idx, lcu_index, modex_index, is_template_one);
#if ESAO_PH_SYNTAX
                one_lcu_open_cost = (double)get_distortion_esao(comp_idx, lcu_stata_data, &ctx->info.pic_header.pic_esao_params[comp_idx], modex_index, types);
#else
                one_lcu_open_cost = (double)get_distortion_esao(comp_idx, lcu_stata_data, &ctx->pic_esao_params[comp_idx], modex_index, types);
#endif
                //open
                one_lcu_rate = enc_get_bit_number(esao_sbac);
                enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, 1);
                one_lcu_rate = enc_get_bit_number(esao_sbac) - one_lcu_rate;
                one_lcu_open_cost += RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], one_lcu_rate);
                SBAC_STORE((core->s_esao_lcu_open), (*esao_sbac));
                //close
                SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
                one_lcu_rate = enc_get_bit_number(esao_sbac);
                enc_eco_esao_lcu_control_flag(esao_sbac, esao_bs_temp, 0);
                one_lcu_rate = enc_get_bit_number(esao_sbac) - one_lcu_rate;
                one_lcu_close_cost = RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], one_lcu_rate);
                SBAC_STORE((core->s_esao_lcu_close), (*esao_sbac)); 
                if (one_lcu_open_cost < one_lcu_close_cost) 
                {
                    cost_lcu_total += one_lcu_open_cost;
                    lcu_open_control_flag[lcu_index] = 1;
                    SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_open));
                    SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac)); 
                }
                else
                {
                    cost_lcu_total += one_lcu_close_cost;
                    lcu_open_control_flag[lcu_index] = 0;
                    SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_close));
                    SBAC_STORE((core->s_esao_lcu_loop), (*esao_sbac)); 
                }
            }
            cost_lcu_total += RATE_TO_COST_LAMBDA(esao_lambda[comp_idx], cur_rate);
            if (cost_lcu_total < cost_best[comp_idx])
            {
                ctx->info.pic_header.esao_lcu_enable[comp_idx] = 1;
                cost_best[comp_idx] = cost_lcu_total;
#if ESAO_PH_SYNTAX
                memcpy(ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag, lcu_open_control_flag, sizeof(int)*lcu_count);
#else
                memcpy(ctx->pic_esao_params[comp_idx].lcu_flag, lcu_open_control_flag, sizeof(int)*lcu_count);
#endif
                SBAC_LOAD((*esao_sbac), (core->s_esao_lcu_loop));
                SBAC_STORE((core->s_esao_cur_blk), (*esao_sbac));
            }
            else
            {
                ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
            }
        }
        else
        {
            ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
        }
    }
    free(lcu_open_control_flag);
}
#endif

void get_frame_param_for_esao(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp)
{
#if ESAO_ENH
    esao_rdcost_for_mode_new(ctx, core, esao_bs_temp);
#else
    double esao_lambda[N_C];
    double min_cost[N_C] = { 0,0,0 };
    int scale_lambda = (ctx->info.bit_depth_internal == 10) ? ctx->info.qp_offset_bit_depth : 1;
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        esao_lambda[comp_idx] = ctx->lambda[0] * scale_lambda;
    }

    esao_rdcost_for_mode_new(ctx, core, esao_bs_temp, esao_lambda, min_cost);
    
    esao_rdcost_for_mode_lcu(ctx, core, esao_bs_temp, esao_lambda, min_cost);
#endif
}

void enc_esao_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs)
{
    get_frame_statistics_for_esao(&ctx->info, &ctx->map, PIC_ORG(ctx), ctx->pic_esao, ctx->esao_luma_data, ctx->esao_chroma_data);
    get_frame_param_for_esao(ctx, core, esao_bs);
}

void enc_esao_init(ENC_CTX *ctx)
{
    copy_frame_for_esao(ctx->pic_esao, PIC_REC(ctx));
#if ESAO_ENH
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        ctx->info.pic_header.pic_esao_on[comp_idx]     = 0;

        ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
        ctx->info.pic_header.esao_set_num[comp_idx]    = 0;
        for (int set = 0; set < ESAO_SET_NUM; set++)
        {
            ctx->info.pic_header.esao_adaptive_param[comp_idx][set] = 0;
            ctx->info.pic_header.esao_luma_type[set]                = 0;
            ctx->info.pic_header.esao_chroma_band_flag[0][set]      = 0;
            ctx->info.pic_header.esao_chroma_band_flag[1][set]      = 0;
        }
#if ESAO_PH_SYNTAX
        memset(ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag, 0, sizeof(int)*ctx->info.f_lcu);
#else
        memset(ctx->pic_esao_params[comp_idx].lcu_flag, 0, sizeof(int)*ctx->info.f_lcu);
#endif
    }
    for (int lcu_idx = 0; lcu_idx < ctx->info.f_lcu; lcu_idx++)
    {
        for (int i = 0; i < ESAO_LABEL_NUM_Y; i++)
        {
            for (int j = 0; j < ESAO_LUMA_TYPES; j++)
            {
                memset(ctx->esao_luma_data[lcu_idx][i][j].count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
                memset(ctx->esao_luma_data[lcu_idx][i][j].diff,  0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
            }
        }
        for (int i = 0; i < ESAO_LABEL_NUM_MAX; i++)
        {
            for (int j = 0; j < N_C-1; j++)
            {
                memset(ctx->esao_chroma_data[lcu_idx][i][j].count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
                memset(ctx->esao_chroma_data[lcu_idx][i][j].diff,  0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
            }
        }
    }
    for (int comp = Y_C; comp < N_C; comp++)
    {
        memset(ctx->best_esao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
        memset(ctx->temp_esao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
        memset(ctx->init_esao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
        ctx->best_esao_param[comp].set_num = ctx->temp_esao_param[comp].set_num = ctx->init_esao_param[comp].set_num = 0;

        for (int set = 0; set < ESAO_SET_NUM; set++)
        {
            memset(ctx->best_esao_param[comp].offset[set], 0, sizeof(int) * ESAO_LABEL_CLASSES_MAX);
            memset(ctx->temp_esao_param[comp].offset[set], 0, sizeof(int) * ESAO_LABEL_CLASSES_MAX);
            memset(ctx->init_esao_param[comp].offset[set], 0, sizeof(int) * ESAO_LABEL_CLASSES_MAX);
            ctx->best_esao_param[comp].chroma_band_flag[set]   = ctx->temp_esao_param[comp].chroma_band_flag[set]   = ctx->init_esao_param[comp].chroma_band_flag[set] = 0;
            ctx->best_esao_param[comp].chroma_band_length[set] = ctx->temp_esao_param[comp].chroma_band_length[set] = ctx->init_esao_param[comp].chroma_band_length[set] = 0;
            ctx->best_esao_param[comp].chroma_start_band[set]  = ctx->temp_esao_param[comp].chroma_start_band[set]  = ctx->init_esao_param[comp].chroma_start_band[set] = 0;
            ctx->best_esao_param[comp].mode[set] = ctx->temp_esao_param[comp].mode[set] = ctx->init_esao_param[comp].mode[set] = 0;
        }
        int scale_lambda = (ctx->info.bit_depth_internal == 10) ? ctx->info.qp_offset_bit_depth : 1;
        ctx->lambda_esao[comp] = ctx->lambda[Y_C] * scale_lambda;
    }

#else
    ctx->info.pic_header.esao_luma_type = -1;
    int lcu_count = ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0));
    for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
    {
        ctx->info.pic_header.esao_adaptive_param[comp_idx] = -1;
        ctx->info.pic_header.pic_esao_on[comp_idx] = 0;
        ctx->info.pic_header.esao_lcu_enable[comp_idx] = 0;
#if ESAO_PH_SYNTAX
        memset(ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag, 0, sizeof(int)*lcu_count);
#else
        memset(ctx->pic_esao_params[comp_idx].lcu_flag, 0, sizeof(int)*lcu_count);
#endif
    }
    for (int i = 0; i < ESAO_LABEL_NUM_Y; i++)
    {
        for (int j = 0; j < ESAO_LUMA_TYPES; j++)
        {
            memset(ctx->esao_luma_data[i][j].count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
            memset(ctx->esao_luma_data[i][j].diff, 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
        }
    }
    for (int i = 0; i < ESAO_LABEL_NUM_MAX; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            memset(ctx->esao_chroma_data[i][j].count, 0, sizeof(int)*ESAO_LABEL_CLASSES_MAX);
            memset(ctx->esao_chroma_data[i][j].diff, 0, sizeof(long long)*ESAO_LABEL_CLASSES_MAX);
        }
    }
#endif
}

int enc_esao(ENC_CTX *ctx, ENC_CORE *core)
{
    enc_esao_init(ctx);
    enc_sbac_init(&(core->bs_temp));
    enc_esao_rdo(ctx, core, &(core->bs_temp));
    copy_frame_for_esao(ctx->pic_esao, PIC_REC(ctx));
#if ESAO_PH_SYNTAX
    esao_on_frame(&ctx->info, &ctx->map, PIC_REC(ctx), ctx->pic_esao, ctx->info.pic_header.pic_esao_params, &ctx->func_esao_block_filter);
#else
    esao_on_frame(&ctx->info, &ctx->map, PIC_REC(ctx), ctx->pic_esao, ctx->pic_esao_params, &ctx->func_esao_block_filter);
#endif
    return COM_OK;
}
#endif 

#if CCSAO
#if CCSAO_PH_SYNTAX
unsigned int uvlc_bitrate_estimate(int val)
{
    unsigned int length = 1;
    val++;
    assert(val);
    while (1 != val)
    {
        val >>= 1;
        length += 2;
    }
    return ((length >> 1) + ((length + 1) >> 1));
}

unsigned int svlc_bitrate_estimate(int val)
{
    return uvlc_bitrate_estimate((val <= 0) ? (-val << 1) : ((val << 1) - 1));
}
#endif

#if ECCSAO
void get_ccsao_class_stat_edge(COM_PIC *pic_org, COM_PIC *pic_ccsao[2],
    CCSAO_STAT_DATA ***ccsao_stat_data, int bit_depth, int comp, int lcu_width_c, int lcu_height_c, int mode, int lcu_pos, int x_c, int y_c,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{

    pel *p_src = pic_ccsao[0]->y;
    pel *p_src2 = comp == U_C - 1 ? pic_ccsao[0]->u : pic_ccsao[0]->v;
    pel *p_dst = comp == U_C - 1 ? pic_ccsao[1]->u : pic_ccsao[1]->v;
    pel *p_org = comp == U_C - 1 ? pic_org->u : pic_org->v;
    int  i_src = pic_ccsao[0]->stride_luma;
    int  i_src2 = pic_ccsao[0]->stride_chroma;
    int  i_dst = pic_ccsao[1]->stride_chroma;
    int  i_org = pic_org->stride_chroma;

    int  band_num = (mode <= 1) ? mode + 1 : mode - 2 + 1;


    int start_x_c = lcu_available_left ? 0 : 1;
    int start_y_c = lcu_available_up ? 0 : 1;
    int end_x_c = lcu_width_c;
    int end_y_c = lcu_height_c;

    p_src += (y_c << 1) * i_src + (x_c << 1) + (start_y_c << 1) * i_src;
    p_src2 += y_c * i_src2 + x_c + start_y_c * i_src2;

    p_dst += y_c * i_dst + x_c + start_y_c * i_dst;
    p_org += y_c * i_org + x_c + start_y_c * i_org;

    int signa, signb, band;

    for (int y_c = start_y_c; y_c < end_y_c; y_c++)
    {
#if CCSAO_LINE_BUFFER
        int line_idx = (lcu_available_down && y_c >= lcu_height_c - CCSAO_PAD_ROWS - 1) ? lcu_height_c - y_c - 1 : 0;
#endif
        for (int x_c = start_x_c; x_c < end_x_c; x_c++)
        {
            int  x = x_c << 1;

            for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
            {

#if CCSAO_LINE_BUFFER
                pel *col_Y = p_src + x + i_src * ccsao_edge_type_y[0][0][line_idx];
                pel *col_a = p_src + x + i_src * ccsao_edge_type_y[type][0][line_idx] + ccsao_edge_type_x[type][0];
                pel *col_b = p_src + x + i_src * ccsao_edge_type_y[type][1][line_idx] + ccsao_edge_type_x[type][1];
#else
                pel *col_Y = p_src + x;
                pel *col_a = p_src + x + i_src * ccsao_edge_type_y[type][0] + ccsao_edge_type_x[type][0];
                pel *col_b = p_src + x + i_src * ccsao_edge_type_y[type][1] + ccsao_edge_type_x[type][1];
#endif
                pel *col_C = p_src2 + x_c;

                signa = 0;
                signb = 0;

                for (int th = 0; th < CCSAO_QUAN_NUM; th++)
                {
                    signa = calc_diff_range(*col_Y, *col_a, th);

                    signb = calc_diff_range(*col_Y, *col_b, th);

                    signa = signa * 4 + signb;

                    if (mode <= 1)
                    {
                        band = (*col_Y * band_num) >> bit_depth;
                        band = band * CCSAO_EDGE_NUM + signa;

                    }
                    else
                    {
                        band = (*col_C * band_num) >> bit_depth;
                        band = band * CCSAO_EDGE_NUM + signa;
                    }
                    assert(band < CCSAO_CLASS_NUM);
                    ccsao_stat_data[mode][th][type].diff[band] += p_org[x_c] - p_dst[x_c];
                    ccsao_stat_data[mode][th][type].count[band]++;
                }
            }
        }
        p_src += i_src << 1;
        p_src2 += i_src2;
        p_dst += i_dst;
        p_org += i_org;
    }
}
#endif


void get_ccsao_class_stat(COM_PIC *pic_org,
#if CCSAO_ENHANCEMENT
    COM_PIC *pic_ccsao[2],
#else
    COM_PIC *pic_ccsao,
#endif
    CCSAO_STAT_DATA *ccsao_stat_data, int bit_depth, int comp, int lcu_width_c, int lcu_height_c, int mode,
#if CCSAO_ENHANCEMENT
    int mode_c,
#endif
    int lcu_pos, int x_c, int y_c,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
#if CCSAO_ENHANCEMENT
    pel *p_src = pic_ccsao[0]->y;
    pel *p_src2 = comp == U_C - 1 ? pic_ccsao[0]->u : pic_ccsao[0]->v;
    pel *p_dst = comp == U_C - 1 ? pic_ccsao[1]->u : pic_ccsao[1]->v;
    pel *p_org = comp == U_C - 1 ? pic_org->u : pic_org->v;
    int  i_src = pic_ccsao[0]->stride_luma;
    int  i_src2 = pic_ccsao[0]->stride_chroma;
    int  i_dst = pic_ccsao[1]->stride_chroma;
    int  i_org = pic_org->stride_chroma;
    int  band_num = mode + 1;
    int  band_num_c = mode_c + 1;
#else
    pel *p_src     = pic_ccsao->y;
    pel *p_dst     = comp == U_C-1 ? pic_ccsao->u : pic_ccsao->v;
    pel *p_org     = comp == U_C-1 ? pic_org  ->u : pic_org  ->v;
    int  i_src     = pic_ccsao->stride_luma;
    int  i_dst     = pic_ccsao->stride_chroma;
    int  i_org     = pic_org  ->stride_chroma;
    int  band_num  = mode + 1;
#endif

    int start_x_c = lcu_available_left ? 0 : 1;
    int start_y_c = lcu_available_up   ? 0 : 1;
    int end_x_c   = lcu_width_c;
    int end_y_c   = lcu_height_c;

    p_src += (y_c << 1) * i_src + (x_c << 1) + (start_y_c << 1) * i_src;
#if CCSAO_ENHANCEMENT
    p_src2 += y_c * i_src2 + x_c + start_y_c * i_src2;
#endif
    p_dst +=  y_c       * i_dst +  x_c       +  start_y_c       * i_dst;
    p_org +=  y_c       * i_org +  x_c       +  start_y_c       * i_org;

    for (int y_c = start_y_c; y_c < end_y_c; y_c++)
    {
#if CCSAO_LINE_BUFFER
        int line_idx = (lcu_available_down && y_c >= lcu_height_c - CCSAO_PAD_ROWS - 1) ? lcu_height_c - y_c - 1 : 0;
#endif
        for (int x_c = start_x_c; x_c < end_x_c; x_c++)
        {
            int  x = x_c << 1;

            for (int type = 0; type < CCSAO_TYPE_NUM; type++)
            {
#if CCSAO_LINE_BUFFER
                pel *col_Y = p_src + x + i_src * ccsao_type_y[type][line_idx] + ccsao_type_x[type];
#else
                pel *col_Y = p_src + x + i_src * ccsao_type_y[type] + ccsao_type_x[type];
#endif
                int  band  = (*col_Y * band_num) >> bit_depth;
#if CCSAO_ENHANCEMENT
                pel *col_C = p_src2 + x_c;
                int  band_c = (*col_C * band_num_c) >> bit_depth;
                band = band * band_num_c + band_c;
#endif
                ccsao_stat_data[type].diff [band] += p_org[x_c] - p_dst[x_c];
                ccsao_stat_data[type].count[band]++;
            }
        }
        p_src += i_src << 1;
#if CCSAO_ENHANCEMENT
        p_src2 += i_src2;
#endif
        p_dst += i_dst;
        p_org += i_org;
    }
}

void get_ccsao_frame_stat(COM_INFO *info, COM_MAP *map, COM_PIC *pic_org,
#if CCSAO_ENHANCEMENT
    COM_PIC *pic_ccsao[2], CCSAO_STAT_DATA *****ccsao_stat_data
#else
    COM_PIC *pic_ccsao, CCSAO_STAT_DATA ****ccsao_stat_data
#endif
#if ECCSAO
    , CCSAO_STAT_DATA *****ccsao_edge_stat_data
#endif
)
{
    int bit_depth     = info->bit_depth_internal;
    int pic_width     = info->pic_width;
    int pic_height    = info->pic_height;
    int log2_max_cuwh = info->log2_max_cuwh;
    int lcu_width, lcu_height;
    int is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;

    for (int y = 0; y < pic_height; y += lcu_height)
    {
        lcu_height = min(1 << log2_max_cuwh, pic_height - y);
        for (int x = 0; x < pic_width; x += lcu_width)
        {
            lcu_width = min(1 << log2_max_cuwh, pic_width - x);
            int x_in_lcu     = x >> log2_max_cuwh;
            int y_in_lcu     = y >> log2_max_cuwh;
            int lcu_pos      = x_in_lcu + y_in_lcu * info->pic_width_in_lcu;
            int lcu_width_c  = lcu_width  >> 1;
            int lcu_height_c = lcu_height >> 1;
            int x_c          = x >> 1;
            int y_c          = y >> 1;

            for (int comp = U_C-1; comp < N_C-1; comp++)
            {
                // reuse ESAO U V boundary check
                check_boundary_available_for_esao(info, map, y_c, x_c, lcu_height_c, lcu_width_c, comp+1, 
                    &is_left_avail, &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
                
                for (int mode = 0; mode < CCSAO_BAND_NUM; mode++)
                {
#if CCSAO_ENHANCEMENT
                    for (int mode_c = 0; mode_c < CCSAO_BAND_NUM_C; mode_c++)
                    {
                        get_ccsao_class_stat(pic_org, pic_ccsao, ccsao_stat_data[comp][lcu_pos][mode][mode_c], bit_depth, comp, lcu_width_c, lcu_height_c, mode, mode_c, lcu_pos, x_c, y_c,
                            is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
                    }
#else
                    get_ccsao_class_stat(pic_org, pic_ccsao, ccsao_stat_data[comp][lcu_pos][mode], bit_depth, comp, lcu_width_c, lcu_height_c, mode, lcu_pos, x_c, y_c,
                        is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
#endif
                }
#if ECCSAO
                for (int mode = 0; mode < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; mode++)
                {
                    get_ccsao_class_stat_edge(pic_org, pic_ccsao, ccsao_edge_stat_data[comp][lcu_pos], bit_depth, comp, lcu_width_c, lcu_height_c, mode, lcu_pos, x_c, y_c,
                        is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
                }
#endif
            }
        }
    }
}

long long int calc_ccsao_distorsion(long long int count, long long int offset, long long int diff)
{
    return count * offset * offset - 2 * offset * diff;
}

#if CCSAO_ENHANCEMENT
long long int get_ccsao_distorsion(CCSAO_STAT_DATA *ccsao_stat_data, CCSAO_BLK_PARAM *ccsao_param, int set)
{
    volatile long long int dist = 0;

#if ECCSAO
    int class_num;
    int class_type = ccsao_param->class_type[set];
    if (class_type == 0)
        class_num = (ccsao_param->mode[set] + 1) * (ccsao_param->mode_c[set] + 1);
    else
        class_num = (ccsao_param->mode[set] < CCSAO_EDGE_COMPARE_VALUE) ? (ccsao_param->mode[set] + 1)*CCSAO_EDGE_NUM : (ccsao_param->mode[set] - CCSAO_EDGE_COMPARE_VALUE + 1)*CCSAO_EDGE_NUM;
#else
    int class_num = (ccsao_param->mode[set] + 1) * (ccsao_param->mode_c[set] + 1);
#endif
    for (int i = 0; i < class_num; i++)
    {
        dist += calc_ccsao_distorsion(ccsao_stat_data->count[i], ccsao_param->offset[set][i], ccsao_stat_data->diff[i]);
    }
    return dist;
}
#else
long long int get_ccsao_distorsion(CCSAO_STAT_DATA *ccsao_stat_data, CCSAO_BLK_PARAM *ccsao_param)
{
    volatile long long int dist = 0;

    int class_num = ccsao_param->mode + 1;
    for (int i = 0; i < class_num; i++)
    {
        dist += calc_ccsao_distorsion(ccsao_stat_data->count[i], ccsao_param->offset[i], ccsao_stat_data->diff[i]);
    }
    return dist;
}
#endif

int ccsao_offset_estimation(double lambda, int org_offset, int count, long long int diff)
{
    int lower_bd     = ccsao_clip[0];
    int upper_bd     = ccsao_clip[1];
    int th           = ccsao_clip[2];
    int cur_offset   = COM_CLIP3(lower_bd, upper_bd, org_offset);
    int start_offset = cur_offset >= 0 ? 0 : cur_offset;
    int end_offset   = cur_offset >= 0 ? cur_offset : 0;
    int step         = 1;
    int best_offset  = 0;
    double best_cost = MAX_COST;
    
    for (int temp_offset = start_offset; temp_offset <= end_offset; temp_offset += step)
    {
#if CCSAO_PH_SYNTAX
        int temp_rate = svlc_bitrate_estimate(temp_offset);
#else
        int temp_rate = abs(temp_offset);
        temp_rate = temp_rate ? temp_rate + 1 : 0;
        temp_rate = temp_rate == th ? temp_rate : temp_rate + 1;
#endif
        long long int temp_dist = calc_ccsao_distorsion(count, temp_offset, diff);
        double temp_cost = (double)temp_dist + lambda * (double)temp_rate;
        if (temp_cost < best_cost)
        {
            best_cost   = temp_cost;
            best_offset = temp_offset;
        }
    }
    return best_offset;
}

#if CCSAO_ENHANCEMENT
void find_ccsao_offset(ENC_CTX *ctx, double lambda, CCSAO_STAT_DATA ****ccsao_stat_data, CCSAO_REFINE_DATA *ccsao_refine_data, CCSAO_BLK_PARAM *ccsao_param
#if ECCSAO
    , CCSAO_STAT_DATA ****ccsao_edge_stat_data
#endif
)
{
    int  set_num                  = ccsao_param->set_num;
    int *type                     = ccsao_param->type;
    int *mode                     = ccsao_param->mode;
    int *mode_c                   = ccsao_param->mode_c;
#if ECCSAO
    int *class_type = ccsao_param->class_type;
#endif
    int  class_num[CCSAO_SET_NUM] = { 0 };

    for (int set = 0; set < set_num; set++)
    {
#if ECCSAO
        if (class_type[set] == 0)
            class_num[set] = (mode[set] + 1) * (mode_c[set] + 1);
        else
            class_num[set] = (mode[set] < CCSAO_EDGE_COMPARE_VALUE) ? (mode[set] + 1)*CCSAO_EDGE_NUM : (mode[set] - CCSAO_EDGE_COMPARE_VALUE + 1)*CCSAO_EDGE_NUM;
#else
        class_num[set] = (mode[set] + 1) * (mode_c[set] + 1);
#endif
    }

    for (int set = 0; set < CCSAO_SET_NUM; set++)
    {
        memset(ccsao_param      ->offset[set], 0, sizeof(int)       * CCSAO_CLASS_NUM);
        memset(ccsao_refine_data->count [set], 0, sizeof(int)       * CCSAO_CLASS_NUM);
        memset(ccsao_refine_data->diff  [set], 0, sizeof(long long) * CCSAO_CLASS_NUM);
    }
    
    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        if (ccsao_param->lcu_flag[lcu_pos])
        {
            int set = ccsao_param->lcu_flag[lcu_pos] - 1;
#if ECCSAO
            if (class_type[set] == 0)
            {
                for (int i = 0; i < class_num[set]; i++)
                {
                    ccsao_refine_data->count[set][i] += ccsao_stat_data[lcu_pos][mode[set]][mode_c[set]][type[set]].count[i];
                    ccsao_refine_data->diff[set][i] += ccsao_stat_data[lcu_pos][mode[set]][mode_c[set]][type[set]].diff[i];
                }
            }
            else
            {
                for (int i = 0; i < class_num[set]; i++)
                {
                    ccsao_refine_data->count[set][i] += ccsao_edge_stat_data[lcu_pos][mode[set]][mode_c[set]][type[set]].count[i];
                    ccsao_refine_data->diff[set][i] += ccsao_edge_stat_data[lcu_pos][mode[set]][mode_c[set]][type[set]].diff[i];
                }
            }
#else
            for (int i = 0; i < class_num[set]; i++)
            {
                ccsao_refine_data->count[set][i] += ccsao_stat_data[lcu_pos][mode[set]][mode_c[set]][type[set]].count[i];
                ccsao_refine_data->diff[set][i] += ccsao_stat_data[lcu_pos][mode[set]][mode_c[set]][type[set]].diff[i];
            }
#endif
        }
    }

    for (int set = 0; set < set_num; set++)
    {
        for (int i = 0; i < class_num[set]; i++)
        {
            if (ccsao_refine_data->count[set][i] == 0)
            {
                ccsao_param->offset[set][i] = 0;
                continue;
            }

            double offth = ccsao_refine_data->diff[set][i] > 0 ?  0.5 :
                          (ccsao_refine_data->diff[set][i] < 0 ? -0.5 : 0);
            ccsao_param->offset[set][i] = (int)((double)ccsao_refine_data->diff[set][i] / (double)ccsao_refine_data->count[set][i] + offth);
        }

        for (int i = 0; i < class_num[set]; i++)
        {
            if (ccsao_refine_data->count[set][i] == 0)
            {
                continue;
            }
            ccsao_param->offset[set][i] = ccsao_offset_estimation(lambda, ccsao_param->offset[set][i], ccsao_refine_data->count[set][i], ccsao_refine_data->diff[set][i]);
        }
    }
}
#else
void find_ccsao_offset(ENC_CTX *ctx, double lambda, CCSAO_STAT_DATA ***ccsao_stat_data, CCSAO_STAT_DATA *ccsao_refine_data, CCSAO_BLK_PARAM *ccsao_param)
{
    int type      = ccsao_param->type;
    int mode      = ccsao_param->mode;
    int class_num = mode + 1;

    memset(ccsao_param      ->offset, 0, sizeof(int)       * CCSAO_CLASS_NUM);
    memset(ccsao_refine_data->count,  0, sizeof(int)       * CCSAO_CLASS_NUM);
    memset(ccsao_refine_data->diff,   0, sizeof(long long) * CCSAO_CLASS_NUM);

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        if (ccsao_param->lcu_flag[lcu_pos])
        {
            for (int i = 0; i < class_num; i++)
            {
                ccsao_refine_data->count[i] += ccsao_stat_data[lcu_pos][mode][type].count[i];
                ccsao_refine_data->diff [i] += ccsao_stat_data[lcu_pos][mode][type].diff [i];
            }
        }
    }

    for (int i = 0; i < class_num; i++)
    {
        if (ccsao_refine_data->count[i] == 0)
        {
            ccsao_param->offset[i] = 0;
            continue;
        }

        double offth = ccsao_refine_data->diff[i] > 0 ?  0.5 :
                      (ccsao_refine_data->diff[i] < 0 ? -0.5 : 0);
        ccsao_param->offset[i] = (int)((double)ccsao_refine_data->diff[i] / (double)ccsao_refine_data->count[i] + offth);
    }

    for (int i = 0; i < class_num; i++)
    {
        if (ccsao_refine_data->count[i] == 0)
        {
            continue;
        }
        ccsao_param->offset[i] = ccsao_offset_estimation(lambda, ccsao_param->offset[i], ccsao_refine_data->count[i], ccsao_refine_data->diff[i]);
    }
}
#endif

void copy_ccsao_param(ENC_CTX *ctx, CCSAO_BLK_PARAM *ccsao_para_dst, CCSAO_BLK_PARAM *ccsao_para_src)
{
#if CCSAO_ENHANCEMENT
    ccsao_para_dst->set_num = ccsao_para_src->set_num;
    memcpy(ccsao_para_dst->type,   ccsao_para_src->type,   sizeof(int) * CCSAO_SET_NUM);
    memcpy(ccsao_para_dst->mode,   ccsao_para_src->mode,   sizeof(int) * CCSAO_SET_NUM);
    memcpy(ccsao_para_dst->mode_c, ccsao_para_src->mode_c, sizeof(int) * CCSAO_SET_NUM);
#if ECCSAO
    memcpy(ccsao_para_dst->class_type, ccsao_para_src->class_type, sizeof(int) * CCSAO_SET_NUM);
#endif
    for (int set = 0; set < CCSAO_SET_NUM; set++)
    {
        memcpy(ccsao_para_dst->offset[set], ccsao_para_src->offset[set], sizeof(int) * CCSAO_CLASS_NUM);
    }
#else
    ccsao_para_dst->type = ccsao_para_src->type;
    ccsao_para_dst->mode = ccsao_para_src->mode;
    memcpy(ccsao_para_dst->offset,   ccsao_para_src->offset,   sizeof(int) * CCSAO_CLASS_NUM);
#endif
    memcpy(ccsao_para_dst->lcu_flag, ccsao_para_src->lcu_flag, sizeof(int) * ctx->info.f_lcu);
}

#if CCSAO_ENHANCEMENT
void setup_init_ccsao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int set_num, CCSAO_BLK_PARAM *init_ccsao_param, CCSAO_BLK_PARAM *best_ccsao_param)
{
    if (set_num == 1)
    {
        for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
        {
            init_ccsao_param->lcu_flag[lcu_pos] = 1;
        }
        return;
    }
    
    copy_ccsao_param(ctx, init_ccsao_param, best_ccsao_param);

    int lcu_on_cnt = 0;
    CCSAO_LCU_COST *lcu_cost = (CCSAO_LCU_COST *)malloc(ctx->info.f_lcu * sizeof(CCSAO_LCU_COST));
    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);
    SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_init));
    for (int lcu_idx = 0; lcu_idx < ctx->info.f_lcu; lcu_idx++)
    {
        int lcu_rate = enc_get_bit_number(ccsao_sbac);
        enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, init_ccsao_param->lcu_flag[lcu_idx], set_num);
        lcu_rate = enc_get_bit_number(ccsao_sbac) - lcu_rate;
        long long lcu_dist = 0;

        if (init_ccsao_param->lcu_flag[lcu_idx])
        {
            int set    = init_ccsao_param->lcu_flag[lcu_idx] - 1;
            int type   = init_ccsao_param->type[set];
            int mode   = init_ccsao_param->mode[set];
            int mode_c = init_ccsao_param->mode_c[set];
#if ECCSAO
            int class_type = init_ccsao_param->class_type[set];
            if (class_type == 0)
                lcu_dist = get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_idx][mode][mode_c][type], init_ccsao_param, set);
            else
                lcu_dist = get_ccsao_distorsion(&ctx->ccsao_edge_chroma_data[comp][lcu_idx][mode][mode_c][type], init_ccsao_param, set);

#else
            lcu_dist = get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_idx][mode][mode_c][type], init_ccsao_param, set);
#endif
            lcu_on_cnt++;
        }

        lcu_cost[lcu_idx].lcu_pos  = lcu_idx;
        lcu_cost[lcu_idx].lcu_cost = (double)lcu_dist + RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], lcu_rate);
    }
    
    qsort(lcu_cost, ctx->info.f_lcu, sizeof(CCSAO_LCU_COST), compare_ccsao_lcu_cost);

    for (int lcu_idx = 0; lcu_idx < ctx->info.f_lcu; lcu_idx++)
    {
        int lcu_pos = lcu_cost[lcu_idx].lcu_pos;
        if (lcu_idx < lcu_on_cnt)
        {
            if (lcu_idx * set_num > lcu_on_cnt * (set_num-1))
            {
                init_ccsao_param->lcu_flag[lcu_pos] = set_num;
            }
        }
        else
        {
            init_ccsao_param->lcu_flag[lcu_pos] = 0;
        }
    }
    com_mfree(lcu_cost);
}

void setup_temp_ccsao_param(ENC_CTX *ctx, int comp, int set_num, int type, int mode, int mode_c, CCSAO_BLK_PARAM *temp_ccsao_param, CCSAO_BLK_PARAM *init_ccsao_param
#if ECCSAO
    , int class_type
#endif
)
{
    copy_ccsao_param(ctx, temp_ccsao_param, init_ccsao_param);
    temp_ccsao_param->set_num            = set_num;
    temp_ccsao_param->type   [set_num-1] = type;
    temp_ccsao_param->mode   [set_num-1] = mode;
    temp_ccsao_param->mode_c [set_num-1] = mode_c;
#if ECCSAO
    temp_ccsao_param->class_type[set_num - 1] = class_type;
#endif
}

int compare_ccsao_lcu_cost(const void *a, const void *b)
{
    int    a_pos  = ((CCSAO_LCU_COST *)a)->lcu_pos,  b_pos  = ((CCSAO_LCU_COST *)b)->lcu_pos;
    double a_cost = ((CCSAO_LCU_COST *)a)->lcu_cost, b_cost = ((CCSAO_LCU_COST *)b)->lcu_cost;
    if (a_cost == b_cost) return (a_pos  < b_pos  ? -1 : 1);
    else                  return (a_cost < b_cost ? -1 : 1);
}

int compare_ccsao_set_cnt(const void *a, const void *b)
{
    int a_set = ((CCSAO_SET_CNT *)a)->set, b_set = ((CCSAO_SET_CNT *)b)->set;
    int a_cnt = ((CCSAO_SET_CNT *)a)->cnt, b_cnt = ((CCSAO_SET_CNT *)b)->cnt;
    if (a_cnt == b_cnt) return (a_set < b_set ? -1 : 1);
    else                return (a_cnt > b_cnt ? -1 : 1);
}
#else
void init_temp_ccsao_param(ENC_CTX *ctx, int comp, int type, int mode,
    CCSAO_BLK_PARAM *temp_ccsao_param)
{
    temp_ccsao_param[comp].type = type;
    temp_ccsao_param[comp].mode = mode;
    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        temp_ccsao_param[comp].lcu_flag[lcu_pos] = TRUE;
    }
}
#endif

#if CCSAO_ENHANCEMENT
void update_temp_ccsao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int iter, CCSAO_BLK_PARAM *temp_ccsao_param, int *curr_rate, long long int *curr_dist)
{
    int       rate = 0;
    long long dist = 0;
    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);

    if (temp_ccsao_param->set_num == 1 && iter == 0)
    {
        int mode   = temp_ccsao_param->mode  [0];
        int mode_c = temp_ccsao_param->mode_c[0];
        int type   = temp_ccsao_param->type  [0];
#if ECCSAO
        int class_type = temp_ccsao_param->class_type[0];
#endif

        for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
        {
#if ECCSAO
            if (class_type == 0)
                dist += get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_pos][mode][mode_c][type], temp_ccsao_param, 0);
            else
                dist += get_ccsao_distorsion(&ctx->ccsao_edge_chroma_data[comp][lcu_pos][mode][mode_c][type], temp_ccsao_param, 0);
#else
            dist += get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_pos][mode][mode_c][type], temp_ccsao_param, 0);
#endif
        }

        *curr_rate += rate;
        *curr_dist += dist;
        return;
    }

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        int       best_set     = 0;
        double    lcu_cost_set = 0;
        int       lcu_rate_on  = 0;
        long long lcu_dist_on  = 0;
        double    lcu_cost_on  = MAX_COST;

        for (int cand_set = 0; cand_set < temp_ccsao_param->set_num; cand_set++)
        {
            SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_lcu_loop));
            int lcu_rate_set = enc_get_bit_number(ccsao_sbac);
            enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, cand_set + 1, temp_ccsao_param->set_num);
            lcu_rate_set = enc_get_bit_number(ccsao_sbac) - lcu_rate_set;
            int cand_type   = temp_ccsao_param->type  [cand_set];
            int cand_mode   = temp_ccsao_param->mode  [cand_set];
            int cand_mode_c = temp_ccsao_param->mode_c[cand_set];

#if ECCSAO
            long long lcu_dist_set;
            if (temp_ccsao_param->class_type[cand_set] == 0)
                lcu_dist_set = get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_pos][cand_mode][cand_mode_c][cand_type], temp_ccsao_param, cand_set);
            else
                lcu_dist_set = get_ccsao_distorsion(&ctx->ccsao_edge_chroma_data[comp][lcu_pos][cand_mode][cand_mode_c][cand_type], temp_ccsao_param, cand_set);
#else
            long long lcu_dist_set = get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_pos][cand_mode][cand_mode_c][cand_type], temp_ccsao_param, cand_set);
#endif
            lcu_cost_set = (double)lcu_dist_set + RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], lcu_rate_set);
    
            if (lcu_cost_set < lcu_cost_on)
            {
                best_set    = cand_set;
                lcu_rate_on = lcu_rate_set;
                lcu_dist_on = lcu_dist_set;
                lcu_cost_on = lcu_cost_set;
                SBAC_STORE((core->s_ccsao_lcu_on), (*ccsao_sbac));
            }
        }

        SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_lcu_loop));
        int lcu_rate_off = enc_get_bit_number(ccsao_sbac);
        enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, FALSE, temp_ccsao_param->set_num);
        lcu_rate_off = enc_get_bit_number(ccsao_sbac) - lcu_rate_off;
        double lcu_cost_off = RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], lcu_rate_off);
        SBAC_STORE((core->s_ccsao_lcu_off), (*ccsao_sbac));

        if (lcu_cost_on < lcu_cost_off)
        {
            temp_ccsao_param->lcu_flag[lcu_pos] = best_set + 1;
            rate += lcu_rate_on;
            dist += lcu_dist_on;
            SBAC_STORE((core->s_ccsao_lcu_loop), (core->s_ccsao_lcu_on));
        }
        else
        {
            temp_ccsao_param->lcu_flag[lcu_pos] = FALSE;
            rate += lcu_rate_off;
            dist += 0;
            SBAC_STORE((core->s_ccsao_lcu_loop), (core->s_ccsao_lcu_off));
        }
    }

    CCSAO_SET_CNT set_cnt[CCSAO_SET_NUM];
    for (int idx = 0; idx < CCSAO_SET_NUM; idx++)
    {
        set_cnt[idx].en  = FALSE;
        set_cnt[idx].set = idx;
        set_cnt[idx].cnt = 0;
    }

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        if (temp_ccsao_param->lcu_flag[lcu_pos])
        {
            int set = temp_ccsao_param->lcu_flag[lcu_pos] - 1;
            set_cnt[set].en = TRUE;
            set_cnt[set].cnt++;
        }
    }

    qsort(set_cnt, CCSAO_SET_NUM, sizeof(CCSAO_SET_CNT), compare_ccsao_set_cnt);

    s8 set_map[CCSAO_SET_NUM];
    memset(set_map, -1, sizeof(s8) * CCSAO_SET_NUM);
    for (int set = 0; set < CCSAO_SET_NUM; set++)
    {
        for (int idx = 0; idx < CCSAO_SET_NUM; idx++)
        {
            if (set_cnt[idx].en == TRUE && set_cnt[idx].set == set)
            {
                set_map[set] = idx;
                break;
            }
        }
    }

    BOOL remap = FALSE;
    for (int set = 0; set < temp_ccsao_param->set_num; set++)
    {
        remap = set_map[set] != set ? TRUE : remap;
    }

    if (!remap)
    {
        *curr_rate += rate;
        *curr_dist += dist;
        return;
    }

    int temp_type  [CCSAO_SET_NUM];
    int temp_mode  [CCSAO_SET_NUM];
    int temp_mode_c[CCSAO_SET_NUM];
    int temp_offset[CCSAO_SET_NUM][CCSAO_CLASS_NUM];
#if ECCSAO
    int temp_class_type[CCSAO_SET_NUM];
    memcpy(temp_class_type, temp_ccsao_param->class_type, sizeof(int) * CCSAO_SET_NUM);
#endif
    memcpy(temp_type,   temp_ccsao_param->type,   sizeof(int) * CCSAO_SET_NUM);
    memcpy(temp_mode,   temp_ccsao_param->mode,   sizeof(int) * CCSAO_SET_NUM);
    memcpy(temp_mode_c, temp_ccsao_param->mode_c, sizeof(int) * CCSAO_SET_NUM);
    memcpy(temp_offset, temp_ccsao_param->offset, sizeof(int) * CCSAO_SET_NUM * CCSAO_CLASS_NUM);

    temp_ccsao_param->set_num = 0;
    for (int idx = 0; idx < CCSAO_SET_NUM; idx++)
    {
        temp_ccsao_param->set_num += set_cnt[idx].en ? 1 : 0;
        int set = set_cnt[idx].set;
        temp_ccsao_param->type  [idx] = temp_type  [set];
        temp_ccsao_param->mode  [idx] = temp_mode  [set];
        temp_ccsao_param->mode_c[idx] = temp_mode_c[set];
#if ECCSAO
        temp_ccsao_param->class_type[idx] = temp_class_type[set];
#endif
        memcpy(temp_ccsao_param->offset[idx], temp_offset[set], sizeof(int) * CCSAO_CLASS_NUM);
    }

    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        int lcu_flag = temp_ccsao_param->lcu_flag[lcu_pos];
        if (lcu_flag)
        {
            int set = lcu_flag - 1;
            temp_ccsao_param->lcu_flag[lcu_pos] = set_map[set] + 1;
        }
    }

    SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_init));
    rate = enc_get_bit_number(ccsao_sbac);
    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, temp_ccsao_param->lcu_flag[lcu_pos], temp_ccsao_param->set_num);
    }
    rate = enc_get_bit_number(ccsao_sbac) - rate;

    *curr_rate += rate;
    *curr_dist += dist;
}

int get_ccsao_offset_rate(ENC_CORE *core, COM_BSW *ccsao_bs_temp, CCSAO_BLK_PARAM *temp_ccsao_param)
{
    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);
    SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_init));

    int rate = enc_get_bit_number(ccsao_sbac);
    for (int set = 0; set < temp_ccsao_param->set_num; set++)
    {
        int class_num = (temp_ccsao_param->mode[set] + 1) * (temp_ccsao_param->mode_c[set] + 1);
#if ECCSAO
        if (temp_ccsao_param->class_type[set] == 1)
            class_num = (temp_ccsao_param->mode[set] < CCSAO_EDGE_COMPARE_VALUE) ? (temp_ccsao_param->mode[set] + 1)*CCSAO_EDGE_NUM : (temp_ccsao_param->mode[set] - CCSAO_EDGE_COMPARE_VALUE + 1)*CCSAO_EDGE_NUM;
#endif
        for (int i = 0; i < class_num; i++)
        {
#if CCSAO_PH_SYNTAX
            rate -= svlc_bitrate_estimate(temp_ccsao_param->offset[set][i]);
#else
            eco_ccsao_offset(temp_ccsao_param->offset[set][i], ccsao_sbac, ccsao_bs_temp);
#endif
        }
    }
    rate = enc_get_bit_number(ccsao_sbac) - rate;
    return rate;
}

void ccsao_rdo_core(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int set_num, int type, int mode, int mode_c, double *best_cost, double *temp_cost, CCSAO_BLK_PARAM *best_ccsao_param, CCSAO_BLK_PARAM *temp_ccsao_param)
{
    double    curr_cost  = 0;
    double    prev_cost  = 0;
    int       iter       = 0;
    int       max_iter   = 15;
    BOOL      improved   = FALSE;
    BOOL      optimizing = TRUE;
    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);
    
    while (optimizing)
    {
        SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_init));
#if ECCSAO
        find_ccsao_offset(ctx, ctx->lambda[comp], ctx->ccsao_chroma_data[comp], &ctx->ccsao_refine_data[comp], temp_ccsao_param, ctx->ccsao_edge_chroma_data[comp]);
#else
        find_ccsao_offset(ctx, ctx->lambda[comp], ctx->ccsao_chroma_data[comp], &ctx->ccsao_refine_data[comp], temp_ccsao_param);
#endif  
        
        int       curr_rate = 0;
        long long curr_dist = 0;
        SBAC_STORE((core->s_ccsao_lcu_loop), (*ccsao_sbac));
        update_temp_ccsao_param(ctx, core, ccsao_bs_temp, comp, iter, temp_ccsao_param, &curr_rate, &curr_dist);

        curr_rate += CCSAO_SET_NUM_BIT + CCSAO_TYPE_NUM_BIT + CCSAO_BAND_NUM_BIT + 1;
        curr_rate += get_ccsao_offset_rate(core, ccsao_bs_temp, temp_ccsao_param);
        double wrate;
        if      (ctx->param.i_period == 1) wrate = ctx->info.pic_width >= 3840 ? 1.2 : 1.5;
#if CCSAO_RATE
        else if (ctx->param.i_period >  1) wrate = ctx->info.pic_width >= 3840 ? 1.1 : 1.2;
#else
        else if (ctx->param.i_period >  1) wrate = ctx->info.pic_width >= 3840 ? 1.0 : 1.3;
#endif
        else                               wrate = 1.0;
        curr_rate  = (int)(curr_rate * wrate);
        curr_cost  = (double)curr_dist + RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], curr_rate);

        if (curr_cost < prev_cost)
        {
            improved             = TRUE;
            prev_cost            = curr_cost;
            temp_cost[set_num-1] = curr_cost;
        }
        else
        {
            improved = FALSE;
        }

        if (temp_cost[set_num-1] < best_cost[set_num-1])
        {
            best_cost[set_num-1] = temp_cost[set_num-1];
            copy_ccsao_param(ctx, best_ccsao_param, temp_ccsao_param);
        }

        iter++;
        optimizing = improved && iter < max_iter;
    }
}
#else
void ccsao_rdo_core(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int type, int mode,
    double *best_cost, double *temp_cost, CCSAO_BLK_PARAM *best_ccsao_param, CCSAO_BLK_PARAM *temp_ccsao_param)
{
    double    curr_cost  = 0;
    double    prev_cost  = 0;
    int       iter       = 0;
    int       max_iter   = 15;
    BOOL      improved   = FALSE;
    BOOL      optimizing = TRUE;
    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);

    init_temp_ccsao_param(ctx, comp, type, mode, temp_ccsao_param);

    while (optimizing)
    {
        SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_init));
        find_ccsao_offset(ctx, ctx->lambda[comp], ctx->ccsao_chroma_data[comp], &ctx->ccsao_refine_data[comp], &temp_ccsao_param[comp]);
        
        long long int curr_dist = get_ccsao_distorsion(&ctx->ccsao_refine_data[comp], &temp_ccsao_param[comp]);
        int           curr_rate = enc_get_bit_number(ccsao_sbac);

        int class_num = mode + 1;
        for (int i = 0; i < class_num; i++)
        {
#if CCSAO_PH_SYNTAX
            curr_rate -= svlc_bitrate_estimate(temp_ccsao_param[comp].offset[i]);
#else
            eco_ccsao_offset(temp_ccsao_param[comp].offset[i], ccsao_sbac, ccsao_bs_temp);
#endif
        }
        SBAC_STORE((core->s_ccsao_lcu_loop), (*ccsao_sbac));

        if (iter != 0)
        {
            for (int lcu_pos = 0; lcu_pos < ctx->lcu_cnt; lcu_pos++)
            {
                enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, temp_ccsao_param[comp].lcu_flag[lcu_pos]);
            }
        }

        curr_rate = enc_get_bit_number(ccsao_sbac) - curr_rate;
        curr_rate += CCSAO_TYPE_NUM_BIT + CCSAO_BAND_NUM_BIT + 1;
        curr_rate = (int)(curr_rate * (iter == 0 ? 1.0 : 3.9));

        curr_cost = (double)curr_dist + RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], curr_rate);

        if (curr_cost < prev_cost)
        {
            improved = TRUE;
            prev_cost       = curr_cost;
            temp_cost[comp] = curr_cost;
        }
        else
        {
            improved = FALSE;
        }

        if (temp_cost[comp] < best_cost[comp])
        {
            best_cost[comp] = temp_cost[comp];
            copy_ccsao_param(ctx, &best_ccsao_param[comp], &temp_ccsao_param[comp]);
        }

        iter++;
        optimizing = improved && iter < max_iter;

        if (optimizing)
        {
            for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
            {
                SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_lcu_loop));
                int lcu_rate = enc_get_bit_number(ccsao_sbac);
                enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, TRUE);
                int           lcu_rate_on = enc_get_bit_number(ccsao_sbac) - lcu_rate;
                long long int lcu_dist_on = get_ccsao_distorsion(&ctx->ccsao_chroma_data[comp][lcu_pos][mode][type], &temp_ccsao_param[comp]);
                double        lcu_cost_on = (double)lcu_dist_on + RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], lcu_rate_on);
                SBAC_STORE((core->s_ccsao_lcu_on), (*ccsao_sbac));

                SBAC_LOAD((*ccsao_sbac), (core->s_ccsao_lcu_loop));
                lcu_rate = enc_get_bit_number(ccsao_sbac);
                enc_eco_ccsao_lcu_flag(ccsao_sbac, ccsao_bs_temp, FALSE);
                int           lcu_rate_off = enc_get_bit_number(ccsao_sbac) - lcu_rate;
                double        lcu_cost_off = RATE_TO_COST_LAMBDA(ctx->lambda_ccsao[comp], lcu_rate_off);
                SBAC_STORE((core->s_ccsao_lcu_off), (*ccsao_sbac));

                if (lcu_cost_on < lcu_cost_off)
                {
                    temp_ccsao_param[comp].lcu_flag[lcu_pos] = TRUE;
                    SBAC_STORE((core->s_ccsao_lcu_loop), (core->s_ccsao_lcu_on));
                }
                else
                {
                    temp_ccsao_param[comp].lcu_flag[lcu_pos] = FALSE;
                    SBAC_STORE((core->s_ccsao_lcu_loop), (core->s_ccsao_lcu_off));
                }
            }
        }
    }
}
#endif

#if CCSAO_ENHANCEMENT
void get_ccsao_frame_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp)
{
    double best_cost[CCSAO_SET_NUM] = { 0 };
    double temp_cost[CCSAO_SET_NUM] = { 0 };

    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);
    SBAC_STORE((core->s_ccsao_init), (*ccsao_sbac));
    for (int set_num = 1; set_num <= CCSAO_SET_NUM; set_num++)
    {
        setup_init_ccsao_param(ctx, core, ccsao_bs_temp, comp, set_num, &ctx->init_ccsao_param[comp], &ctx->best_ccsao_param[comp]);
        best_cost[set_num-1] = set_num >= 2 ? best_cost[set_num-2] : 0;

        for (int type = 0; type < CCSAO_TYPE_NUM; type++)
        {
            for (int mode = 0; mode < CCSAO_BAND_NUM; mode++)
            {
                for (int mode_c = 0; mode_c < CCSAO_BAND_NUM_C; mode_c++)
                {
                    setup_temp_ccsao_param(ctx, comp, set_num, type, mode, mode_c, &ctx->temp_ccsao_param[comp], &ctx->init_ccsao_param[comp]
#if ECCSAO
                        , 0
#endif
                    );
                    ccsao_rdo_core(ctx, core, ccsao_bs_temp, comp, set_num, type, mode, mode_c, best_cost, temp_cost, &ctx->best_ccsao_param[comp], &ctx->temp_ccsao_param[comp]);
                }
            }
        }
#if ECCSAO
        temp_cost[set_num - 1] = 0;
        for (int type = 0; type < CCSAO_EDGE_TYPE; type++)
        {
            for (int mode = 0; mode < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; mode++)
            {
                for (int th = 0; th < CCSAO_QUAN_NUM; th++)
                {
                    setup_temp_ccsao_param(ctx, comp, set_num, type, mode, th, &ctx->temp_ccsao_param[comp], &ctx->init_ccsao_param[comp], 1);
                    ccsao_rdo_core(ctx, core, ccsao_bs_temp, comp, set_num, type, mode, th, best_cost, temp_cost, &ctx->best_ccsao_param[comp], &ctx->temp_ccsao_param[comp]);
                }
            }
        }
#endif
    }

    int lcu_on_cnt = 0;
    for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
    {
        lcu_on_cnt += ctx->best_ccsao_param[comp].lcu_flag[lcu_pos] ? 1 : 0;
    }
    ctx->info.pic_header.pic_ccsao_on  [comp] = lcu_on_cnt > 0 ? TRUE : FALSE;
    ctx->info.pic_header.ccsao_lcu_ctrl[comp] = (lcu_on_cnt > 0 && lcu_on_cnt != ctx->info.f_lcu) || ctx->best_ccsao_param[comp].set_num >= 2 ? TRUE : FALSE;
    
    if (ctx->info.pic_header.pic_ccsao_on[comp])
    {
#if CCSAO_PH_SYNTAX
        copy_ccsao_param(ctx, &ctx->info.pic_header.pic_ccsao_params[comp], &ctx->best_ccsao_param[comp]);
#endif
        copy_ccsao_param(ctx, &ctx->pic_ccsao_params[comp], &ctx->best_ccsao_param[comp]);
        ctx->info.pic_header.ccsao_set_num[comp] = ctx->pic_ccsao_params[comp].set_num;
        for (int set = 0; set < ctx->info.pic_header.ccsao_set_num[comp]; set++)
        {
            ctx->info.pic_header.ccsao_type      [comp][set] = ctx->pic_ccsao_params[comp].type[set];
            ctx->info.pic_header.ccsao_band_num  [comp][set] = ctx->pic_ccsao_params[comp].mode[set] + 1;
            ctx->info.pic_header.ccsao_band_num_c[comp][set] = ctx->pic_ccsao_params[comp].mode_c[set] + 1;
#if ECCSAO
            ctx->info.pic_header.ccsao_class_type[comp][set] = ctx->pic_ccsao_params[comp].class_type[set];
#endif
        }
    }
}
#else
void get_ccsao_frame_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp)
{
    double best_cost[N_C-1] = { 0 };
    double temp_cost[N_C-1] = { 0 };

    ENC_SBAC *ccsao_sbac = GET_SBAC_ENC(ccsao_bs_temp);
    SBAC_STORE((core->s_ccsao_init), (*ccsao_sbac));
    for (int comp = U_C-1; comp < N_C-1; comp++)
    {
        for (int type = 0; type < CCSAO_TYPE_NUM; type++)
        {
            for (int mode = 0; mode < CCSAO_BAND_NUM; mode++)
            {
                ccsao_rdo_core(ctx, core, ccsao_bs_temp, comp, type, mode, best_cost, temp_cost, ctx->best_ccsao_param, ctx->temp_ccsao_param);
            }
        }

        int lcu_on_cnt = 0;
        for (int lcu_pos = 0; lcu_pos < ctx->info.f_lcu; lcu_pos++)
        {
            lcu_on_cnt += ctx->best_ccsao_param[comp].lcu_flag[lcu_pos] ? 1 : 0;
        }
        ctx->info.pic_header.pic_ccsao_on  [comp] = lcu_on_cnt > 0 ? TRUE : FALSE;
        ctx->info.pic_header.ccsao_lcu_ctrl[comp] = lcu_on_cnt > 0 && lcu_on_cnt != ctx->info.f_lcu ? TRUE : FALSE;
        
        if (ctx->info.pic_header.pic_ccsao_on[comp])
        {
#if CCSAO_PH_SYNTAX
            copy_ccsao_param(ctx, &ctx->info.pic_header.pic_ccsao_params[comp], &ctx->best_ccsao_param[comp]);
#endif
            copy_ccsao_param(ctx, &ctx->pic_ccsao_params[comp], &ctx->best_ccsao_param[comp]);
            ctx->info.pic_header.ccsao_type    [comp] = ctx->pic_ccsao_params[comp].type;
            ctx->info.pic_header.ccsao_band_num[comp] = ctx->pic_ccsao_params[comp].mode + 1;
        }
    }
}
#endif

void enc_ccsao_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs)
{
    get_ccsao_frame_stat(&ctx->info, &ctx->map, PIC_ORG(ctx), ctx->pic_ccsao, ctx->ccsao_chroma_data
#if ECCSAO
        , ctx->ccsao_edge_chroma_data
#endif
    );
#if CCSAO_ENHANCEMENT
    get_ccsao_frame_param(ctx, core, ccsao_bs, U_C-1);
    get_ccsao_frame_param(ctx, core, ccsao_bs, V_C-1);
#else
    get_ccsao_frame_param(ctx, core, ccsao_bs);
#endif
}

void enc_ccsao_init(ENC_CTX *ctx)
{
#if CCSAO_ENHANCEMENT
    copy_frame_for_ccsao(ctx->pic_ccsao[1], PIC_REC(ctx), U_C);
    copy_frame_for_ccsao(ctx->pic_ccsao[1], PIC_REC(ctx), V_C);
#else
    copy_frame_for_ccsao(ctx->pic_ccsao, PIC_REC(ctx), U_C);
    copy_frame_for_ccsao(ctx->pic_ccsao, PIC_REC(ctx), V_C);
#endif

    for (int comp = U_C-1; comp < N_C-1; comp++)
    {
        ctx->info.pic_header.pic_ccsao_on  [comp] = 0;
        ctx->info.pic_header.ccsao_lcu_ctrl[comp] = 0;
#if CCSAO_ENHANCEMENT
        ctx->info.pic_header.ccsao_set_num [comp] = 0;
        for (int set = 0; set < CCSAO_SET_NUM; set++)
        {
            ctx->info.pic_header.ccsao_type      [comp][set] = 0;
            ctx->info.pic_header.ccsao_band_num  [comp][set] = 0;
            ctx->info.pic_header.ccsao_band_num_c[comp][set] = 0;
#if ECCSAO
            ctx->info.pic_header.ccsao_class_type[comp][set] = 0;
#endif
        }

        ctx->pic_ccsao_params[comp].set_num = ctx->best_ccsao_param[comp].set_num = ctx->temp_ccsao_param[comp].set_num = ctx->init_ccsao_param[comp].set_num = 0;
        for (int set = 0; set < CCSAO_SET_NUM; set++)
        {
            ctx->pic_ccsao_params[comp].type  [set] = ctx->best_ccsao_param[comp].type  [set] = ctx->temp_ccsao_param[comp].type  [set] = ctx->init_ccsao_param[comp].type  [set] = 0;
            ctx->pic_ccsao_params[comp].mode  [set] = ctx->best_ccsao_param[comp].mode  [set] = ctx->temp_ccsao_param[comp].mode  [set] = ctx->init_ccsao_param[comp].mode  [set] = 0;
            ctx->pic_ccsao_params[comp].mode_c[set] = ctx->best_ccsao_param[comp].mode_c[set] = ctx->temp_ccsao_param[comp].mode_c[set] = ctx->init_ccsao_param[comp].mode_c[set] = 0;
#if ECCSAO
            ctx->pic_ccsao_params[comp].class_type[set] = ctx->best_ccsao_param[comp].class_type[set] = ctx->temp_ccsao_param[comp].class_type[set] = ctx->init_ccsao_param[comp].class_type[set] = 0;
#endif
            memset(ctx->pic_ccsao_params[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
            memset(ctx->best_ccsao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
            memset(ctx->temp_ccsao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
            memset(ctx->init_ccsao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
        }
        memset(ctx->pic_ccsao_params[comp].offset, 0, sizeof(int) * CCSAO_SET_NUM * CCSAO_CLASS_NUM);
        memset(ctx->best_ccsao_param[comp].offset, 0, sizeof(int) * CCSAO_SET_NUM * CCSAO_CLASS_NUM);
        memset(ctx->temp_ccsao_param[comp].offset, 0, sizeof(int) * CCSAO_SET_NUM * CCSAO_CLASS_NUM);
        memset(ctx->init_ccsao_param[comp].offset, 0, sizeof(int) * CCSAO_SET_NUM * CCSAO_CLASS_NUM);
#else
        ctx->info.pic_header.ccsao_type    [comp] = 0;
        ctx->info.pic_header.ccsao_band_num[comp] = 0;

        memset(ctx->pic_ccsao_params[comp].offset,   0, sizeof(int) * CCSAO_CLASS_NUM);
        memset(ctx->best_ccsao_param[comp].offset,   0, sizeof(int) * CCSAO_CLASS_NUM);
        memset(ctx->temp_ccsao_param[comp].offset,   0, sizeof(int) * CCSAO_CLASS_NUM);
        memset(ctx->pic_ccsao_params[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
        memset(ctx->best_ccsao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
        memset(ctx->temp_ccsao_param[comp].lcu_flag, 0, sizeof(int) * ctx->info.f_lcu);
#endif

        for (int i = 0; i < ctx->info.f_lcu; i++)
        {
            for (int j = 0; j < CCSAO_BAND_NUM; j++)
            {
#if CCSAO_ENHANCEMENT
                for (int l = 0; l < CCSAO_BAND_NUM_C; l++)
                {
#endif
                    for (int k = 0; k < CCSAO_TYPE_NUM; k++)
                    {
#if CCSAO_ENHANCEMENT
                        memset(ctx->ccsao_chroma_data[comp][i][j][l][k].count, 0, sizeof(int)       * CCSAO_CLASS_NUM);
                        memset(ctx->ccsao_chroma_data[comp][i][j][l][k].diff, 0, sizeof(long long) * CCSAO_CLASS_NUM);
#else
                        memset(ctx->ccsao_chroma_data[comp][i][j][k].count, 0, sizeof(int)       * CCSAO_CLASS_NUM);
                        memset(ctx->ccsao_chroma_data[comp][i][j][k].diff, 0, sizeof(long long) * CCSAO_CLASS_NUM);
#endif
                    }
#if CCSAO_ENHANCEMENT
                }
#endif
            }
#if ECCSAO
            for (int j = 0; j < CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C; j++)
            {
                for (int l = 0; l < CCSAO_QUAN_NUM; l++)
                {
                    for (int k = 0; k < CCSAO_EDGE_TYPE; k++)
                    {
                        memset(ctx->ccsao_edge_chroma_data[comp][i][j][l][k].count, 0, sizeof(int)       * CCSAO_CLASS_NUM);
                        memset(ctx->ccsao_edge_chroma_data[comp][i][j][l][k].diff, 0, sizeof(long long) * CCSAO_CLASS_NUM);
                    }
                }
            }
#endif
        }
    }

    int scale_lambda = (ctx->info.bit_depth_internal == 10) ? ctx->info.qp_offset_bit_depth : 1;
    ctx->lambda_ccsao[U_C-1] = ctx->lambda[Y_C] * scale_lambda;
    ctx->lambda_ccsao[V_C-1] = ctx->lambda[Y_C] * scale_lambda;
}

int enc_ccsao(ENC_CTX *ctx, ENC_CORE *core)
{
    enc_ccsao_init(ctx);
    enc_sbac_init(&core->bs_temp);
    enc_ccsao_rdo(ctx, core, &core->bs_temp);

    if (ctx->info.pic_header.pic_ccsao_on[U_C-1] || ctx->info.pic_header.pic_ccsao_on[V_C-1])
    {
#if CCSAO_PH_SYNTAX
        ccsao_on_frame(&ctx->info, &ctx->map, PIC_REC(ctx), ctx->pic_ccsao, ctx->info.pic_header.pic_ccsao_params, &ctx->ccsao_func_ptr);
#else
        ccsao_on_frame(&ctx->info, &ctx->map, PIC_REC(ctx), ctx->pic_ccsao, ctx->pic_ccsao_params, &ctx->ccsao_func_ptr);
#endif
    }
    return COM_OK;
}
#endif