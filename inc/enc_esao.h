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

#ifndef ENC_ESAO_H
#define ENC_ESAO_H
#include "com_def.h"
#include  "enc_def.h"
#if ESAO
void get_multi_classes_statistics_for_esao(COM_PIC *pic_org,COM_PIC  *pic_lic, ESAO_STAT_DATA *esao_state_data, int bit_depth, int comp_idx, int smb_pix_height, int smb_pix_width,
    int label_index,int lcu_pos,int pix_y, int pix_x, int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down,
    int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon);

#if ESAO_ENH
void get_frame_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA ***esao_luma_state_data, ESAO_STAT_DATA ***esao_chroma_state_data);
#else
void get_frame_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA **esao_luma_state_data, ESAO_STAT_DATA **esao_chroma_state_data);

void get_lcu_statistics_for_esao(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_org, COM_PIC  *pic_esao, ESAO_STAT_DATA *lcu_sata_data, int comp_idx, int lcu_pos, int label_index, int is_template_one);
#endif

void eco_esao_chroma_band_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag);

void eco_esao_chroma_len(int length, int start_band, int mode_index, ENC_SBAC *sbac, COM_BSW *bs);

long long int  distortion_cal_esao(long long int count, int offset, long long int diff);

int offset_esao_estimation(double lambda, int offset_ori, int count, long long int diff, double *best_cost);

void eco_esao_offset_AEC(int value1, ENC_SBAC *sbac, COM_BSW *bs);

#if !ESAO_PH_SYNTAX
void enc_eco_esao_param(ENC_CTX *ctx, COM_BSW * bs, COM_PIC_HEADER *sh);
#endif

#if ESAO_ENH
void find_esao_offset(ENC_CTX *ctx, int comp_idx, ESAO_REFINE_DATA *esao_state_date, ESAO_BLK_PARAM *esao_blk_param, double lambda);

int compare_esao_lcu_cost(const void *a, const void *b);

int compare_esao_set_cnt(const void *a, const void *b);

void setup_temp_esao_param(ENC_CTX *ctx, int comp, int set_num, int type, int mode, ESAO_BLK_PARAM *temp_ccsao_param, ESAO_BLK_PARAM *init_ccsao_param);

void enc_eco_esao_lcu_control_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag, int set_num);

long long int get_distortion_esao(int comp_idx, ESAO_STAT_DATA esao_state_data, ESAO_BLK_PARAM *sao_cur_param, int set_num);

void update_temp_esao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int iter, ESAO_BLK_PARAM *temp_ccsao_param, int *curr_rate, long long int *curr_dist);

int uv_not_equal_count(ESAO_BLK_PARAM *esao_cur_param, int *begin, int mode_index, int set_num);

int get_esao_offset_rate(ENC_CORE *core, int comp, COM_BSW *esao_bs_temp, ESAO_BLK_PARAM *temp_esao_param);

void esao_rdcost_for_mode_new_yuv_each(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, ESAO_BLK_PARAM *best_esao_param, ESAO_BLK_PARAM *temp_esao_param, int mode_index, int set_num, int type,
    double *best_cost, double *temp_cost, int comp_idx);

void esao_rdcost_for_mode_new(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp);
#else
int uv_not_equal_count(ESAO_BLK_PARAM *esao_cur_param,int *begin, int mode_index);

long long int get_distortion_esao(int comp_idx, ESAO_STAT_DATA esao_state_data, ESAO_BLK_PARAM *sao_cur_param, int mode_index, int types);

void enc_eco_esao_lcu_control_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag);

void find_esao_offset(int comp_idx, ESAO_STAT_DATA esao_state_date, ESAO_BLK_PARAM *esao_blk_param, double lambda, int mode_index, int types);

void esao_rdcost_for_mode_new_yuv_each(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, ESAO_BLK_PARAM *rec_esao_cur_param, int mode_index,
    double *cost_count, int *types, int comp_idx, int uv_offset_count[2], int start_band[2], int esao_chroma_band_flag[2]);

void esao_rdcost_for_mode_new(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *min_cost);

void esao_rdcost_for_mode_lcu(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp, double* esao_lambda, double *cost_best);
#endif

void get_frame_param_for_esao(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *esao_bs_temp);

void enc_esao_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *sao_bs);

void enc_esao_init(ENC_CTX *ctx);

int enc_esao(ENC_CTX *ctx, ENC_CORE *core);
#endif 

#if CCSAO
void get_ccsao_class_stat(COM_PIC *pic_org,
#if CCSAO_ENHANCEMENT
    COM_PIC *pic_ccsao[2],
#else
    COM_PIC *pic_ccsao,
#endif
    CCSAO_STAT_DATA *ccsao_stat_data, int bit_depth, int comp_idx, int lcu_width_c, int lcu_height_c, int mode,
#if CCSAO_ENHANCEMENT
    int mode_c,
#endif
    int lcu_pos, int x_c, int y_c,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon);

#if ECCSAO
void get_ccsao_class_stat_edge(COM_PIC *pic_org, COM_PIC *pic_ccsao[2],
    CCSAO_STAT_DATA ***ccsao_stat_data, int bit_depth, int comp, int lcu_width_c, int lcu_height_c, int mode, int lcu_pos, int x_c, int y_c,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon);
#endif

void get_ccsao_frame_stat(COM_INFO *info, COM_MAP *map, COM_PIC *pic_org,
#if CCSAO_ENHANCEMENT
    COM_PIC *pic_ccsao[2], CCSAO_STAT_DATA *****ccsao_stat_data
#else
    COM_PIC *pic_ccsao, CCSAO_STAT_DATA ****ccsao_stat_data
#endif
#if ECCSAO
    , CCSAO_STAT_DATA *****ccsao_edge_stat_data
#endif
);

#if CCSAO_ENHANCEMENT
void setup_init_ccsao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int set_num, CCSAO_BLK_PARAM *init_ccsao_param, CCSAO_BLK_PARAM *best_ccsao_param);

void setup_temp_ccsao_param(ENC_CTX *ctx, int comp, int set_num, int type, int mode, int mode_c, CCSAO_BLK_PARAM *temp_ccsao_param, CCSAO_BLK_PARAM *init_ccsao_param
#if ECCSAO
    , int class_type
#endif
);

int compare_ccsao_lcu_cost(const void *a, const void *b);

int compare_ccsao_set_cnt(const void *a, const void *b);

void update_temp_ccsao_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int iter, CCSAO_BLK_PARAM *temp_ccsao_param, int *curr_rate, long long int *curr_dist);

int get_ccsao_offset_rate(ENC_CORE *core, COM_BSW *ccsao_bs_temp, CCSAO_BLK_PARAM *temp_ccsao_param);

void ccsao_rdo_core(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int set_num, int type, int mode, int mode_c, double *best_cost, double *temp_cost, CCSAO_BLK_PARAM *best_ccsao_param, CCSAO_BLK_PARAM *temp_ccsao_param);

void find_ccsao_offset(ENC_CTX *ctx, double lambda, CCSAO_STAT_DATA ****ccsao_stat_data, CCSAO_REFINE_DATA *ccsao_refine_data, CCSAO_BLK_PARAM *ccsao_param
#if ECCSAO
    , CCSAO_STAT_DATA ****ccsao_edge_stat_data
#endif
);

long long int get_ccsao_distorsion(CCSAO_STAT_DATA *ccsao_stat_data, CCSAO_BLK_PARAM *ccsao_param, int set);
#else
void init_temp_ccsao_param(ENC_CTX *ctx, int comp, int type, int mode,
    CCSAO_BLK_PARAM *temp_ccsao_param);

void ccsao_rdo_core(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp, int type, int mode, double *best_cost, double *temp_cost, CCSAO_BLK_PARAM *best_ccsao_param, CCSAO_BLK_PARAM *temp_ccsao_param);

void find_ccsao_offset(ENC_CTX *ctx, double lambda, CCSAO_STAT_DATA ***ccsao_stat_data, CCSAO_STAT_DATA *ccsao_refine_data, CCSAO_BLK_PARAM *ccsao_param);

long long int get_ccsao_distorsion(CCSAO_STAT_DATA *ccsao_stat_data, CCSAO_BLK_PARAM *ccsao_param);
#endif

long long int calc_ccsao_distorsion(long long int count, long long int offset, long long int diff);

int ccsao_offset_estimation(double lambda, int org_offset, int count, long long int diff);

#if !CCSAO_PH_SYNTAX
void eco_ccsao_offset(int offset, ENC_SBAC *sbac, COM_BSW *bs);
#endif

#if CCSAO_ENHANCEMENT
void enc_eco_ccsao_lcu_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag, int set_num);
#else
void enc_eco_ccsao_lcu_flag(ENC_SBAC *sbac, COM_BSW *bs, int flag);
#endif

#if !CCSAO_PH_SYNTAX
void enc_eco_ccsao_param(ENC_CTX *ctx, COM_BSW * bs, COM_PIC_HEADER *pic_header);
#endif

void copy_ccsao_param(ENC_CTX *ctx, CCSAO_BLK_PARAM *ccsao_para_dst, CCSAO_BLK_PARAM *ccsao_para_src);

#if CCSAO_ENHANCEMENT
void get_ccsao_frame_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp, int comp);
#else
void get_ccsao_frame_param(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs_temp);
#endif

void enc_ccsao_rdo(ENC_CTX *ctx, ENC_CORE *core, COM_BSW *ccsao_bs);

void enc_ccsao_init(ENC_CTX *ctx);

int enc_ccsao(ENC_CTX *ctx, ENC_CORE *core);
#endif
#endif
