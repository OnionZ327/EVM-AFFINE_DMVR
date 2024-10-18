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

#ifndef _ENC_ECO_H_
#define _ENC_ECO_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "enc_def.h"

#define GET_SBAC_ENC(bs)   ((ENC_SBAC *)(bs)->pdata[1])

int enc_eco_cnkh(COM_BSW * bs, COM_CNKH * cnkh);
int enc_eco_sqh(ENC_CTX *ctx, COM_BSW * bs, COM_SQH * sqh);
#if EXTENSION_USER_DATA
int enc_eco_extension_data(ENC_CTX *ctx, COM_BSW * bs);
#if TEMPORAL_SCALABILITY_EXTENSION
int enc_eco_temporal_scalability_extension(ENC_CTX *ctx, COM_BSW * bs);
#endif
#if HDR_METADATA_EXTENSION
int enc_hdr_dynamic_metadata_extension(ENC_CTX *ctx, COM_BSW * bs);
#endif
#endif
int enc_eco_pic_header(COM_BSW * bs, COM_PIC_HEADER * sh, COM_SQH * sqh);
int enc_eco_patch_header(COM_BSW * bs, COM_SQH *sqh, COM_PIC_HEADER *ph, COM_SH_EXT * sh, u8 patch_idx, PATCH_INFO* patch);
#if PATCH
int enc_eco_send(COM_BSW * bs);
#endif
int enc_eco_udata(ENC_CTX * ctx, COM_BSW * bs);
int encode_pred_mode(COM_BSW * bs, u8 pred_mode, ENC_CTX * ctx);
#if ECCPM
int encode_eccpm_flag(COM_BSW* bs, u8 eccpm_flag);
#endif
int encode_ipf_flag(COM_BSW *bs, u8 ipf_flag);
#if IIP
int encode_iip_flag(COM_BSW *bs, u8 iip_flag);
#endif
int encode_mvd(COM_BSW * bs, s16 mvd[MV_D]);
#if USE_IBC
int encode_bvd(COM_BSW * bs, s16 mvd[MV_D]);
#endif
void enc_sbac_init(COM_BSW * bs);

void enc_sbac_finish(COM_BSW *bs, int is_ipcm);
void enc_sbac_encode_bin(u32 bin, ENC_SBAC *sbac, SBAC_CTX_MODEL *ctx_model, COM_BSW *bs);
void enc_sbac_encode_bin_trm(u32 bin, ENC_SBAC *sbac, COM_BSW *bs);
int encode_coef(COM_BSW * bs, s16 coef[N_C][MAX_CU_DIM], int cu_width_log2, int cu_height_log2, u8 pred_mode, COM_MODE *mi, u8 tree_status, ENC_CTX * ctx
#if CUDQP
    , int qp_y
#endif
#if INTER_CCNPM
    , int x, int y
#endif
);

int enc_eco_unit(ENC_CTX * ctx, ENC_CORE * core, int x, int y, int cup, int cu_width, int cu_height);
#if ETS
void enc_eco_ets_flag(COM_BSW * bs, int flag);
#endif
#if IBC_TS
void enc_eco_ts_flag(COM_BSW * bs, int flag);
#endif
#if EST
void enc_eco_est_flag(COM_BSW * bs, int flag);
#endif
#if ST_CHROMA
void enc_eco_st_chroma_flag(COM_BSW * bs, int flag);
#endif
#if INTER_CCNPM
void enc_eco_inter_ccnpm_flag(COM_BSW* bs, int flag);
#endif
#if CHROMA_NOT_SPLIT
int enc_eco_unit_chroma(ENC_CTX * ctx, ENC_CORE * core, int x, int y, int cup, int cu_width, int cu_height);
#endif
int enc_eco_split_mode(COM_BSW *bs, ENC_CTX *c, ENC_CORE *core, int cud, int cup, int cu_width, int cu_height, int lcu_s
                       , const int parent_split, int qt_depth, int bet_depth, int x, int y);
#if MODE_CONS
void enc_eco_cons_pred_mode_child(COM_BSW * bs, u8 cons_pred_mode_child);
#endif
int enc_extension_and_user_data(ENC_CTX* ctx, COM_BSW * bs, int i, u8 isExtension, COM_SQH* sqh, COM_PIC_HEADER* pic_header);

void enc_eco_lcu_delta_qp(COM_BSW *bs, int val, int last_dqp);
void enc_eco_slice_end_flag(COM_BSW * bs, int flag);
int encode_mvd(COM_BSW *bs, s16 mvd[MV_D]);
#if USE_IBC
int encode_bvd(COM_BSW *bs, s16 mvd[MV_D]);
#endif
int encode_refidx(COM_BSW * bs, int num_refp, int refi);
void encode_inter_dir(COM_BSW * bs, s8 refi[REFP_NUM], int part_size, ENC_CTX * ctx);
void encode_skip_flag(COM_BSW * bs, ENC_SBAC *sbac, int flag, ENC_CTX * ctx);
#if INTERPF
void encode_inter_filter_flag(COM_BSW * bs, int flag);
#endif
#if IPC
void encode_ipc_flag(COM_BSW * bs,int flag);
#endif
#if BGC
void encode_bgc_flag(COM_BSW * bs, int flag, int idx);
#endif
#if AWP
void encode_umve_awp_flag(COM_BSW *bs, int flag);
#else
void encode_umve_flag(COM_BSW *bs, int flag);
#endif
void encode_umve_idx(COM_BSW *bs, int umve_idx);
#if UMVE_ENH 
void encode_umve_idx_sec_set(COM_BSW *bs, int umve_idx);
#endif
#if AFFINE_UMVE
void encode_affine_umve_flag(COM_BSW *bs, int flag, ENC_CTX * ctx);
void encode_affine_umve_idx(COM_BSW * bs, int affine_umve_idx);
#endif
void enc_eco_split_flag(ENC_CTX *c, int cu_width, int cu_height, int x, int y, COM_BSW * bs, ENC_SBAC *sbac, int flag);
void encode_skip_idx(COM_BSW *bs, int skip_idx, int num_hmvp_cands, 
#if MVAP
    int num_mvap_cands,
#endif
    ENC_CTX * ctx);
void encode_direct_flag(COM_BSW *bs, int t_direct_flag, ENC_CTX * ctx);
#if IBC_BVP
void encode_ibc_bvp_flag(COM_BSW * bs, ENC_SBAC *sbac, int flag, ENC_CTX * ctx);
#endif
//! \todo Change list of arguments
void enc_eco_xcoef(ENC_CTX *ctx, COM_BSW *bs, s16 *coef, int log2_w, int log2_h, int num_sig, int ch_type
#if PARITY_HIDING
    , COM_MODE * mod_info_curr
#endif
);
//! \todo Change list of arguments
int encode_intra_dir(COM_BSW *bs, u8 ipm,
#if EIPM
    u8 eipm_flag,
#endif
    u8 mpm[2]);
#if SAWP
int encode_sawp_dir(COM_BSW* bs, u8 ipm, u8 mpm[SAWP_MPM_NUM]);
int encode_sawp_dir1(COM_BSW* bs, u8 ipm, u8 mpm[SAWP_MPM_NUM], u8 dir0);
#endif // SAWP
int encode_intra_dir_c(COM_BSW *bs, u8 ipm, u8 ipm_l
#if TSCPM
    , u8 tscpm_enable_flag
#endif
#if ENHANCE_TSPCM
    , u8 enhance_tscpm_enable_flag
#endif
#if PMC || EPMC 
    , u8 pmc_enable_flag
#endif
#if SAWP
    ,u8 sawp_flag
#endif // SAWP
#if CCNPM
    , u8 ccnpm_flag, int x, int y
#endif
#if ECCPM
        , u8 eccpm_mode_flag, int check, int eccpm_mode_enable_flag, int eccpm_scc_flag
#endif
);
#if IPCM
void encode_ipcm(ENC_SBAC *sbac, COM_BSW *bs, s16 pcm[MAX_CU_DIM], int tb_width, int tb_height, int cu_width, int bit_depth, int ch_type);
#endif
int encode_mvr_idx(COM_BSW *bs, u8 mvr_idx, BOOL is_affine_mode);
#if IBC_ABVR
int encode_bvr_idx(COM_BSW * bs, u8 bvr_idx);
#endif
#if IBC_ENH
int encode_ibcpf_idx(COM_BSW* bs, u8 idx, ENC_CTX* ctx);
#endif

#if EXT_AMVR_HMVP
void encode_extend_amvr_flag(COM_BSW *bs, u8 mvp_from_hmvp_flag);
#endif

void encode_affine_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_affine_mrg_idx( COM_BSW * bs, s16 affine_mrg_idx, ENC_CTX * ctx);

#if ETMVP
void encode_etmvp_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_etmvp_idx(COM_BSW *bs, int etmvp_idx);
#endif

#if AWP_ENH
void encode_sawp_mode_idx(COM_BSW* bs, int awp_idx, ENC_CTX* ctx);
void encode_awp_mode_idx(COM_BSW* bs, int awp_idx, ENC_CTX* ctx);
#endif

#if AWP_ENH
void encode_awp_blend_idx(COM_BSW* bs, int idx, ENC_CTX* ctx);
void encode_sawp_blend_idx(COM_BSW* bs, int idx, ENC_CTX* ctx);
#endif

#if AWP
void encode_awp_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_awp_mode(COM_BSW * bs, int awp_idx, int awp_cand_idx0, int awp_cand_idx1, ENC_CTX * ctx
#if AWP_ENH
    , int awp_blend_idx
#endif
#if AWP_ENH
    , int dawp_idx
#endif
);
#endif
#if INTER_TM
void encode_tm_flag(COM_BSW* bs, int flag);
void encode_tm_idx(COM_BSW* bs, int tm_idx);
#endif
#if CIBC
void encode_cibc_flag(COM_BSW* bs, int flag, ENC_CTX* ctx);
#endif
#if SAWP
void encode_sawp_flag(COM_BSW* bs, int flag, ENC_CTX* ctx);
void encode_sawp_mode(COM_BSW* bs, int awp_idx, ENC_CTX* ctx
#if AWP_ENH
    , int awp_blend_idx
#endif
#if DSAWP
    , int dawp_idx
#endif
);
#endif // SAWP

#if AWP_MVR
void encode_awp_cand_idx0(COM_BSW * bs, int awp_cand_idx);
void encode_awp_cand_idx1(COM_BSW * bs, int awp_cand_idx);
void encode_awp_mode1(COM_BSW * bs, int awp_idx, int awp_cand_idx0, int awp_cand_idx1, ENC_CTX * ctx
#if AWP_ENH
    , int awp_blend_idx
#endif
#if AWP_ENH
    , int dawp_idx
#endif
);
void encode_awp_mvr_flag(COM_BSW * bs, int flag, ENC_CTX * ctx);
void encode_awp_mvr_idx(COM_BSW * bs, int awp_mvr_idx);
#endif

#if SMVD
void encode_smvd_flag( COM_BSW * bs, int flag );
#endif

#if USE_IBC
int enc_eco_ibc(COM_BSW * bs, u8 pred_mode_ibc_flag, ENC_CTX *ctx);
#endif
#if ISC_RSD
void enc_eco_rsd_flag(COM_BSW* bs, int flag, int dir);
int  enc_eco_sp_rsd_flag(COM_BSW* bs, u8 sp_rsd_flag);
int  enc_eco_intra_row_rsd_flag(COM_BSW* bs, u8 sp_rsd_flag);
#endif
#if USE_SP
void encode_sp_or_ibc_cu_flag(COM_BSW *bs, int cu_width, int cu_height, ENC_CU_DATA* cu_data, int cup, ENC_CTX *ctx);
void encode_sp_or_cs2_cu_flag(COM_BSW *bs, int cu_width, int cu_height, ENC_CU_DATA* cu_data, int cup, ENC_CTX *ctx);
extern  void write_bins_ep(ENC_SBAC *sbac, COM_BSW *bs, int binValues, int numBins);
void  encode_dup_count(COM_BSW *bs, u32 ui_run, u8 b_copy_top_mode, u32 ui_SRB_idx, u32 ui_max_run);
int enc_eco_cs2_flag(COM_BSW * bs, u8 cs2_flag);
void enc_eco_sp(ENC_CTX* ctx, ENC_CORE * core, COM_MODE* mod_info_curr, COM_BSW *bs, ENC_CU_DATA* cu_data, int x, int y, int cup);
void enc_eco_sp_pixel_group(ENC_CTX* ctx, ENC_CORE * core, COM_MODE* mod_info_curr, COM_BSW *bs, ENC_CU_DATA* cu_data, int x, int y, int cup);
int  enc_eco_sp_or_ibc_flag(COM_BSW * bs, u8 sp_or_ibc_flag);
int  enc_eco_sp_flag(COM_BSW * bs, u8 sp_flag);
int  enc_eco_sp_copy_dir_flag(COM_BSW * bs, u8 sp_copy_direct_flag);
int  enc_eco_above_offset(COM_BSW * bs, u8 sp_above_offset);
int  enc_eco_offset_zero(COM_BSW * bs, u8 offset_zero, u8 is_offset_x);
void enc_eco_sp_mvd_hor(COM_BSW * bs, ENC_CORE * core, COM_SP_INFO *p_sp_info, s16 mvd[MV_D], u8 offset_in_cu, u8 offset_sign_flag);
void enc_eco_sp_mvd_ver(COM_BSW * bs, ENC_CORE * core, COM_SP_INFO *p_sp_info, s16 mvd[MV_D], u8 offset_in_cu, u8 offset_sign_flag);
void enc_eco_sp_mvd(COM_BSW * bs, ENC_CORE * core, COM_SP_INFO *p_sp_info, u8 sp_copy_dir, int cu_width, int cu_height, int cur_pixel, s16 mvd[MV_D]);
int  enc_eco_sp_n_recent_flag(COM_BSW * bs, u8 sp_n_recent_flag);
void enc_eco_sp_n_recent_index(COM_BSW * bs, ENC_SBAC *sbac, s8 flag);
int  enc_eco_sp_is_matched_flag(COM_BSW * bs, u8 sp_is_matched_flag);
void enc_eco_len_in_suffix(COM_BSW * bs, unsigned int b, unsigned int temp, int n, unsigned int max_val_infix);
int  enc_eco_sp_string_length(COM_BSW * bs, u16 value, u16 max_value);
int  enc_eco_pixel_y(pel pixel[N_C], int bit_depth, COM_BSW *bs);
int  enc_eco_pixel_uv(pel pixel[N_C], int bit_depth, COM_BSW *bs);
int  enc_eco_sp_pixel_is_matched_flag(COM_BSW * bs, u8 pixel_is_matched_flag);
#endif

#if DT_SYNTAX
void encode_part_size(ENC_CTX *ctx, COM_BSW * bs, int part_size, int cu_w, int cu_h, int pred_mode);
#endif

#if SBT
int enc_eco_sbt_info( COM_BSW * bs, int log2_cuw, int log2_cuh, int sbt_info, u8 sbt_avail );
#endif

int enc_eco_DB_param(COM_BSW * bs, COM_PIC_HEADER *sh
#if DBK_SCC || DBR
    , COM_SQH *sqh
#endif
);

void enc_eco_sao_mrg_flag(ENC_SBAC *sbac, COM_BSW *bs, int merge_left_avail, int merge_up_avail, SAO_BLK_PARAM *sao_blk_param);
void enc_eco_sao_mode(ENC_SBAC *sbac, COM_BSW *bs, SAO_BLK_PARAM *sao_blk_param);
void enc_eco_sao_offset(ENC_SBAC *sbac, COM_BSW *bs, SAO_BLK_PARAM *sao_blk_param);
void enc_eco_sao_type(ENC_SBAC *sbac, COM_BSW *bs, SAO_BLK_PARAM *sao_blk_param);
#if ESAO
#if ESAO_PH_SYNTAX
void enc_eco_esao_param(COM_BSW * bs, COM_PIC_HEADER *sh);
#else
void enc_eco_esao_pic_header(COM_BSW * bs, COM_PIC_HEADER *sh);
#endif
#endif
#if CCSAO
void enc_eco_ccsao_pic_header(COM_BSW *bs, COM_PIC_HEADER *sh);
#if CCSAO_PH_SYNTAX
#if !SH_UNIFY
void enc_eco_ccsao_param(COM_BSW *bs, COM_PIC_HEADER *pic_header);
#endif
void enc_eco_ccsao_offset(int offset, COM_BSW *bs);
#endif
#endif
int enc_eco_alf_param(COM_BSW * bs, COM_PIC_HEADER *sh
#if ALF_SHAPE
                      , int num_coeff
#endif
);
void enc_eco_alf_lcu_ctrl(ENC_SBAC *sbac, COM_BSW *bs, int flag);
void enc_eco_alf_coeff(COM_BSW *bs, ALF_PARAM *alf_param);

#if NN_FILTER
void enc_eco_nnlf_lcu_enable_flag(ENC_SBAC* sbac, COM_BSW* bs, int enable_flag, int ch);
void enc_eco_nnlf_lcu_set_index  (ENC_SBAC* sbac, COM_BSW* bs, int model_idx,   int ch, int num_of_nnlf);
#endif

void demulate(COM_BSW * bs);

#ifdef __cplusplus
}
#endif
#endif /* _ENC_ECO_H_ */
