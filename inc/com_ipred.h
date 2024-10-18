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

#ifndef _COM_IPRED_H_
#define _COM_IPRED_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "com_def.h"

#define COM_IPRED_CHK_CONV(mode)\
    ((mode) == IPD_VER || (mode) == IPD_HOR || (mode) == IPD_DC || (mode) == IPD_BI)

#define COM_IPRED_CONV_L2C(mode)\
    ((mode) == IPD_VER) ? IPD_VER_C : \
    ((mode) == IPD_HOR ? IPD_HOR_C : ((mode) == IPD_DC ? IPD_DC_C : IPD_BI_C))

#define COM_IPRED_CONV_L2C_CHK(mode, chk) \
    if(COM_IPRED_CHK_CONV(mode)) \
    {\
        (mode) = ((mode) == IPD_VER) ? IPD_VER_C : ((mode) == IPD_HOR ? IPD_HOR_C:\
        ((mode) == IPD_DC ? IPD_DC_C : IPD_BI_C)); \
        (chk) = 1; \
    }\
    else \
        (chk) = 0;

void com_get_nbr(int x, int y, int width, int height, pel *src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 *map_scu, int pic_width_in_scu, int h_scu, int bit_depth, int ch_type);
#if ECCPM
void com_get_template_nbr(int x, int y, int width, int height, pel *src, int s_src, u16 avail_cu, pel nb_temp[N_C][N_REF][(MAX_CU_SIZE+ECCPM_TEMP_SIZE) * 3], int scup, u32 *map_scu, int pic_width_in_scu, int pic_height_in_scu, int bit_depth, int ch_type);
#endif

#if DSAWP
void com_dawp_get_nbr(int x, int y, int width, int height, pel* src, int s_src, u16 avail_cu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32* map_scu, int pic_width_in_scu, int h_scu, int bit_depth, int ch_type);
void com_tpl_ipred(pel* src_le, pel* src_up, pel* dst, int ipm, int blk_w, int blk_h, int bit_depth, u16 avail_cu, u8 ipf_flag
#if MIPF
    , int mipf_enable_flag
#endif
#if IIP
    , u8 iip_flag
#endif
    , int tpl_w, int tpl_h);
#endif

void com_ipred(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h, int bit_depth, u16 avail_cu, u8 ipf_flag
#if MIPF
    , int mipf_enable_flag
#endif
#if IIP
    , u8 iip_flag
#endif
);
void com_ipred_uv(pel *src_le, pel *src_up, pel *dst, int ipm_c, int ipm, int w, int h, int bit_depth, u16 avail_cu
#if TSCPM
                  , int comp_id, pel *reco_y, int stride_y, pel nb[N_C][N_REF][MAX_CU_SIZE * 3]
#endif
#if MIPF
                  , int mipf_enable_flag
#endif
#if PMC || EPMC
                  , pel* reco_u, int stride_u
#endif
#if IPF_CHROMA
                  , u8 ipfc_flag, CHANNEL_TYPE ch_type
#endif
#if CCNPM
                  , pel* reco_luma, const int stride_luma, pel* reco_chroma_u, pel* reco_chroma_v, const int stride_chroma, const int x, const int y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                  , const int max_cu_height
#endif
                 );
#if ECCPM
void com_ipred_uv_template(pel *src_le, pel *src_up, pel *dst,int ipm_c, int ipm, int w, int h, int bit_depth, u16 avail_cu
#if TSCPM
                  , int comp_id, pel *reco_y, int stride_y, pel nb_template[N_C][N_REF][(MAX_CU_SIZE+ECCPM_TEMP_SIZE) * 3]
#endif
#if MIPF
                  , int mipf_enable_flag
#endif
#if PMC || EPMC
                  , pel* reco_u, int stride_u
#endif
#if IPF_CHROMA
                  , u8 ipfc_flag, CHANNEL_TYPE ch_type
#endif
                 );
void calculate_eccpm_model(pel* chroma_u_le,pel* chroma_u_up,pel* luma_recon_le, pel* chroma_recon_le, pel* chroma_pred_le,int stride_luma,int stride_chroma, int w_le, int h_le, pel* luma_recon_up, pel* chroma_recon_up, pel* chroma_pred_up, int w_up, int h_up,  long long* a0,  long long* a1,  long long* a2,long long* a3,int bit_depth,int chroma_type,pel(*A)[MAX_CU_DIM], pel *YC,int param_num);
void pred_eccpm(pel* luma_recon, int stride_luma,pel* chroma_pred, pel* dst, int w, int h,  long long a0,  long long a1,  long long a2,long long a3,int bit_depth,pel* rec_u,int u_stride,int param_num);
#endif

#if INTERPF
void com_inter_filter(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h, int bit_depth, u16 avail_cu, int pfIdx);
#endif

#if IPC
void com_inter_pred_correction(pel *src_le, pel *src_up, pel *dst, int ipm, int w, int h, int bit_depth, u16 avail_cu, int pfIdx, int ch_type
#if IPC_SB8
    , int ipc_size
#endif
);
#endif
void com_get_mpm(int x_scu, int y_scu, u32 * map_scu, s8 * map_ipm, int scup, int pic_width_in_scu, u8 mpm[2]);
#if SAWP
void com_get_sawp_mpm(int x_scu, int y_scu, int cu_width_in_scu, int cu_height_in_scu, u32* map_scu, s8* map_ipm, int scup, int pic_width_in_scu, u8 sawp_mpm[SAWP_MPM_NUM], u8 awp_idx
#if DSAWP
    , int* is_mpm_from_sawp
#endif
);
#endif // SAWP
#if FIMC
void com_get_cntmpm(int x_scu, int y_scu, u32 * map_scu, s8 * map_ipm, int scup, int pic_width_in_scu, u8 mpm[2], COM_CNTMPM* cntmpm);
#endif

#if DT_PARTITION
void pred_recon_intra_luma_pb(COM_PIC *pic, u32 *map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, s16 resi[N_C][MAX_CU_DIM], pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE *mod_info_curr,
                              int pb_x, int pb_y, int pb_w, int pb_h, int pb_scup, int pb_idx, int cu_x, int cu_y, int cu_width, int cu_height, int bit_depth
#if MIPF
    , int mipf_enable_flag
#endif
#if DSAWP
    , COM_INFO* info, pel**** awp_weight_tpl
    , pel nb_tpl[N_C][N_REF][MAX_CU_SIZE * 3]
#endif
);

#endif

#if IBC_APF
void pred_ibc_filter(COM_PIC* pic, u32* map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE* mod_info_curr,
    int x, int y, int cu_width, int cu_height, int bit_depth, int component, int pfIdx);
#endif

#if INTERPF
void pred_inter_filter(COM_PIC *pic, u32 *map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE *mod_info_curr,
                       int x, int y, int cu_width, int cu_height, int bit_depth, int component, int pfIdx);
#endif

#if INTER_CCNPM
void ipred_ccnpm(int comp_id, pel* reco_luma, const int stride_luma, pel* reco_chroma_u, pel* reco_chroma_v, const int stride_chroma, pel* p_dst, const int x, const int y, int width_c, int height_c, int bit_depth, u16 avail_cu
#if CCNPM_TEMPLATE_OPT
    , int ipm_c
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
    , const int max_cu_height
#endif
);
#if INTER_CCNPM_OPT
void ccnpm_blending(pel* p_src, pel* p_dst, int width_c, int height_c, int bit_depth);
#endif
#endif

#if IPC
void pred_inter_pred_correction(COM_PIC *pic, u32 *map_scu, s8* map_ipm, int pic_width_in_scu, int pic_height_in_scu, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], COM_MODE *mod_info_curr,
                       int x, int y, int cu_width, int cu_height, int bit_depth, int component, int pfIdx
#if IPC_SB8
    , int ipc_size
#endif
);
#endif


#ifdef __cplusplus
}
#endif

#endif /* _COM_IPRED_H_ */
