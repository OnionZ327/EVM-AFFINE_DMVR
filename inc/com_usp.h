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

#ifndef COM_USP_H
#define COM_USP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "com_def.h"

#if USE_SP
#include <stdlib.h>
#include <malloc.h>
#include "enc_def.h"   //for SBAC in evs
typedef enum _SP_HASH_STATUS_IDX
{
    HS_CURR_BEST,
    HS_NEXT_BEST,
    HS_TEMP_BEST,
    HS_CTX_NUM,
} SP_HASH_STATUS_IDX;
typedef enum _SP_SRB_RUN_MODE
{
    SRB_RUN_LEFT = 0,
    SRB_RUN_ABOVE = 1,
    NUM_SRB_RUN = 2
}SP_SRB_RUN_MODE;
typedef enum _SP_CHANNEL_TYPE
{
    CHANNEL_TYPE_LUMA = 0,
    CHANNEL_TYPE_CHROMA = 1,
    MAX_NUM_CHANNEL_TYPE = 2
}SP_CHANNEL_TYPE;
typedef enum _SP_MATCH_TYPE
{
    MATCH_POS_WIDTH = 0,
    MATCH_POS_ONE = 1,
    MATCH_NONE = 2,
    NUMBER_OF_MATCH_TYPES = 3
}SP_MATCH_TYPE;

typedef struct _COM_SP_POS
{
    s16 x;
    s16 y;
} COM_SP_POS;

typedef struct _COM_SP_INPUT
{
    int sample_bit_depth;
    int chroma_format;
    int img_width;
    int img_height;
    int max_cu_width;
    int max_cu_height;
    int y_stride;
    int recy_stride;
    int c_stride;
    int recc_stride;
} COM_SP_INPUT;

#if FAST_SCC_PRECODING
typedef struct _ENC_SP_VISIT
{
    int    visit;
    int    cssvisit0mv[MV_D];
}ENC_SP_VISIT;
#endif

typedef struct _COM_SP_CODING_UNIT
{
    int ctu_log2size;
    int         cu_pix_x;
    int         cu_pix_y;
    int         cu_width_log2;
    int         cu_height_log2;
    int         pic_width_in_scu;
    int         pic_height_in_scu;
    int         qp;
    pel         rec[N_C][MAX_CU_DIM];
    u32         *map_scu;
    int         scup;
    u8          tree_status;
    double      lamda;
    double      chroma_weight[2];
    double      cur_bst_rdcost;
    u8          string_prediction_mode_flag;
    u8          string_copy_direction;  //TRUE:Horizontal FALSE:Vertical
    int         sub_string_no;
    COM_SP_INFO p_string_copy_info[SP_STRING_INFO_NO];
    int         max_str_cnt;            //max string num =width * height / 4
    int         p_x, p_y;               //x_pos,y_pos of parent cu
    int         p_width, p_height;      //width,height of parent cu
    COM_MOTION  *p_cand;
    u16         p_cand_num;
    COM_MOTION  *b_cand;
    u16         b_cand_num;
    COM_MOTION  *n_cand;
    s8          n_cand_num;
    u8          is_sp_pix_completed;
    u8          is_sp_skip_non_scc;
#if SCC_CROSSINFO_ULTILIZE
    int ibc_mv[2];
#endif

    COM_SP_EVS_INFO p_evs_copy_info[SP_STRING_INFO_NO];      //warning:mayby merge with p_string_copy_info later
    int             unpredict_pix_num;
    COM_SP_PIX      unpredict_pix_info[SP_STRING_INFO_NO];
    //SRB
    u8              m_cs2_mode_flag;                         //evs_sous_mode_flag
    u8              m_evs_present_flag;                      //equal_value_string_flag
    u8              m_unpredictable_pixel_present_flag;        
    pel             m_srb[N_C][MAX_SRB_SIZE];                ///< Cluster
    u8              m_pvbuf_size;
    pel             m_srb_prev[N_C][MAX_SRB_PRED_SIZE];
    u8              m_pvbuf_size_prev;
    u8              m_pvbuf_reused_flag[MAX_SRB_PRED_SIZE];
    u8              m_srb_sharing_flag;
    u8              m_all_comp_flag[MAX_SRB_SIZE];
    u8              m_all_comp_pre_flag[MAX_SRB_PRED_SIZE];
    u8              cu_ext;
    u8              m_cuS_flag[MAX_SRB_SIZE];
    u8              m_cuS_pre_flag[MAX_SRB_PRED_SIZE];
    s16             m_pv_x[MAX_SRB_SIZE];
    s16             m_pv_prev_x[MAX_SRB_PRED_SIZE];
    s16             m_pv_y[MAX_SRB_SIZE];
    s16             m_pv_prev_y[MAX_SRB_PRED_SIZE];
    u8              m_dpb_idx[MAX_SRB_SIZE];
    u8              m_dpb_idx_prev[MAX_SRB_PRED_SIZE];
    u8              m_dpb_reYonly[MAX_SRB_SIZE];
    u8              m_dpb_reYonly_prev[MAX_SRB_PRED_SIZE];
    u8              m_bit_depth;
    /* SBAC structure for SP RDO */
    ENC_SBAC        s_curr_best[MAX_CU_DEPTH][MAX_CU_DEPTH];
    ENC_SBAC        s_temp_run;
    ENC_SBAC        s_temp_best;
#if FAST_SCC_PRECODING
    int      spvisit;
    int      sp_transmit_x;
    int      sp_transmit_y;
#endif
    /* bitstream structure for RDO */
    COM_BSW         bs_temp;
} COM_SP_CODING_UNIT;
#if defined(WIN32) || defined(WIN64)
#define x_malloc( type, len )        _aligned_malloc( sizeof(type)*(len), 32 )
#define x_free( ptr )                _aligned_free  ( ptr ) 
#else
#define x_malloc( type, len )        memalign( 32, sizeof(type)*(len))
#define x_free( ptr )                free  ( ptr ) 
#endif
extern int*  g_run_golomb_groups;
extern u8    g_uc_run_top_lut[5];
extern u8    g_uc_run_left_lut[5];
extern pel   g_cluster[N_C][MAX_SRB_SIZE];
extern pel   g_cluster_pred[N_C][MAX_SRB_SIZE];
extern int   g_ui_cluster_size;
extern int   g_ui_cluster_size_pred;
extern u8    g_last_pvbuf_size;
typedef struct _CS2_MODE_INFO 
{
    u8              string_copy_direction;
    int             sub_string_no;
    COM_SP_EVS_INFO p_evs_copy_info[SP_STRING_INFO_NO];
    int             unpredict_pix_num;
    COM_SP_PIX      unpredict_pix_info[SP_STRING_INFO_NO];
    //SRB
    u8              m_cs2_mode_flag;
    u8              m_evs_present_flag;
    u8              m_unpredictable_pixel_present_flag;
    pel             m_srb[3][MAX_SRB_SIZE];   
    u8              m_pvbuf_size;
    u8              m_pvbuf_reused_flag[MAX_SRB_PRED_SIZE];
    pel             m_srb_prev[3][MAX_SRB_PRED_SIZE];
    u8              m_pvbuf_size_prev;
    u8              m_bit_depth;
    u8              m_all_comp_flag[MAX_SRB_SIZE];
    u8              m_all_comp_pre_flag[MAX_SRB_PRED_SIZE];
    u8              m_cuS_flag[MAX_SRB_SIZE];
    u8              m_cuS_pre_flag[MAX_SRB_PRED_SIZE];
    s16             m_pv_x[MAX_SRB_SIZE];
    s16             m_pv_prev_x[MAX_SRB_PRED_SIZE];
    s16             m_pv_y[MAX_SRB_SIZE];
    s16             m_pv_prev_y[MAX_SRB_PRED_SIZE];
    u8              m_dpb_idx[MAX_SRB_SIZE];
    u8              m_dpb_idx_prev[MAX_SRB_PRED_SIZE];
    u8              m_dpb_reYonly[MAX_SRB_SIZE];
    u8              m_dpb_reYonly_prev[MAX_SRB_PRED_SIZE];
} CS2_MODE_INFO;
#endif
#ifdef __cplusplus
}
#endif

#endif 