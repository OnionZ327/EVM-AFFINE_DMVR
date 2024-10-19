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

#ifndef _COM_MC_H_
#define _COM_MC_H_

#ifdef __cplusplus

extern "C"
{
#endif

#if DMVR
typedef void (*COM_MC_L) (pel* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel* pred, int w, int h, int bit_depth, int is_half_pel_filter, int is_dmvr);
#else
typedef void (*COM_MC_L) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int is_half_pel_filter);
#endif

typedef void(*COM_MC_C) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if USE_IBC
    , int is_ibc
#endif
#if DMVR
    , int is_dmvr
#endif
    );

extern COM_MC_L com_tbl_mc_l[2][2];
extern COM_MC_C com_tbl_mc_c[2][2];

#if DMVR
typedef void(*COM_DMVR_MC_L) (pel* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel* pred, int w, int h, int bit_depth, int is_half_pel_filter, int is_dmvr);
typedef void(*COM_DMVR_MC_C) (pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int is_half_pel_filter);
    
extern COM_DMVR_MC_L ifvc_tbl_dmvr_mc_l[2][2];
extern COM_DMVR_MC_C ifvc_tbl_dmvr_mc_c[2][2];
#endif

#if DMVR
#define com_mc_l_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0xF)?1:0][(ori_mv_y & 0xF)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0)
#define com_mc_l(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0x3)?1:0][(ori_mv_y & 0x3)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0)
#else
#define com_mc_l_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0xF)?1:0][(ori_mv_y & 0xF)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1)
#define com_mc_l(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[(ori_mv_x & 0x3)?1:0][(ori_mv_y & 0x3)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0)
#endif

#if USE_IBC
#if DMVR
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0, 0)

#define com_mc_c_ibc(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(gmv_x & 0x7)?1:0][(gmv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1, 0)

#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0, 0)
#else
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0)

#define com_mc_c_ibc(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(gmv_x & 0x7)?1:0][(gmv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1)

#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0)
#endif
#else
#if DMVR
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1, 0)
#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0)
#else
#define com_mc_c_hp(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x1F)?1:0][(ori_mv_y & 0x1F)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 1)
#define com_mc_c(ori_mv_x, ori_mv_y, ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[(ori_mv_x & 0x7)?1:0][(ori_mv_y & 0x7)?1:0])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0)
#endif
#endif

#if DMVR
#define com_dmvr_mc_l(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_l[((gmv_x) | ((gmv_x)>>1)) & 0x1])\
        [((gmv_y) | ((gmv_y)>>1)) & 0x1]\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1)

#if USE_IBC
#define com_dmvr_mc_c(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_mc_c[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2)) & 0x1]\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2)) & 0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 0, 1)
#else
#define com_dmvr_mc_c(ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth) \
    (com_tbl_dmvr_mc_c[((gmv_x) | ((gmv_x)>>1) | ((gmv_x)>>2)) & 0x1]\
        [((gmv_y) | ((gmv_y)>>1) | ((gmv_y)>>2)) & 0x1])\
        (ref, gmv_x, gmv_y, s_ref, s_pred, pred, w, h, bit_depth, 0, 1)
#endif
#endif

/*****************************************************************************
* mc DMVR structure
*****************************************************************************/
#if AFFINE_DMVR
void process_AFFINEDMVR(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16(*mv)[VER_NUM][MV_D], COM_REFP(*refp)[REFP_NUM]);
void affine_mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][1][MV_D], s16(*mv_t)[1][MV_D]);
#endif
#if DMVR
typedef struct _COM_DMVR
{
    int poc_c;
    pel *dmvr_current_template; 
    pel (*dmvr_ref_pred_interpolated)[(MAX_CU_SIZE + (2*(DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + (2*(DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))];
    BOOL apply_DMVR;
    pel (*dmvr_padding_buf)[N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE];
} COM_DMVR;
#endif

void com_mc(int x, int y, int w, int h, int pred_stride, pel pred_buf[N_C][MAX_CU_DIM], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], CHANNEL_TYPE channel, int bit_depth
#if DMVR
            , COM_DMVR * dmvr
#endif
#if BIO
            , int ptr, int enc_fast, u8 mvr_idx
#endif
#if MVAP
            , int mvap_flag
#endif
#if SUB_TMVP
            , int sbTmvp_flag
#endif
#if BGC
            , s8 bgc_flag, s8 bgc_idx
#endif
);

#if OBMC
void pred_obmc(COM_MODE *mod_info_curr, COM_INFO *info, COM_MAP* pic_map, COM_REFP(*refp)[REFP_NUM], BOOL luma, BOOL chroma, int bit_depth
    , int ob_blk_width, int ob_blk_height
    , int cur_ptr
#if BGC
    , s8 bgc_flag, s8 bgc_idx
#endif
#if OBMC_TEMP
    ,COM_PIC* pic
#endif
);
#if OBMC_TEMP
void com_get_obmc_temp_pred(int x, int y, int w, int h, pel pred_buf[MAX_CU_SIZE],pel temp_buf[MAX_CU_SIZE], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], BOOL luma, BOOL doChromaOBMC, int bit_depth,int dir);
int caculate_obmc_mode(pel* rec, int stride, pel pred_src1[MAX_CU_SIZE],  pel pred_src2[MAX_CU_SIZE],  int width, int height, int bit_depth,int dir,pel* weight);
void com_obmc_temp_blending(pel* yuvPredDst, int predDstStride, pel* yuvPredSrc, int predSrcStride, int width, int height, int dir, pel* weight, int bit_depth,int obmc_mode,int temp_size);
#endif

void com_subblk_obmc(int x, int y, int w, int h, int pred_stride, pel* pred_buf[N_C], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], BOOL luma, BOOL doChromaOBMC, int bit_depth
#if BGC
    , s8 bgc_flag, s8 bgc_idx
#endif
);
void com_obmc_blending(pel *yuvPredDst, int predDstStride, pel *yuvPredSrc, int predSrcStride, int width, int height, int dir, pel *weight, int bit_depth);
#endif

#if AWP || SAWP
#if BAWP
#if SAWP_WEIGHT_OPT || AWP_ENH
void com_calculate_awp_weight(pel awp_weight0[N_C][MAX_AWP_DIM], pel awp_weight1[N_C][MAX_AWP_DIM], int comp_idx, int cu_width, int cu_height, int blend_idx, int step_idx, int angle_idx, int angle_area, int refine_flag, int is_p_slice);
int get_awp_weight_by_blend_idx(int distance, int blend_idx);
#else
void com_calculate_awp_weight(pel awp_weight0[N_C][MAX_AWP_DIM], pel awp_weight1[N_C][MAX_AWP_DIM], int comp_idx, int cu_width, int cu_height, int step_idx, int angle_idx, int angle_area, int refine_flag, int is_p_slice);
#endif
#if AWP_ENH
void derive_awp_blend_lut(int* size_lut, int* shift_lut);
void init_awp_tpl(COM_MODE* mod_info_curr, pel**** awp_weight_tpl);
void com_get_tpl_cur(COM_PIC* pic, u32* map_scu, int pic_width_in_scu, int pic_height_in_scu, COM_MODE* mod_info_curr);
int com_tpl_reorder_awp_mode(COM_MODE* mod_info_curr, int* mode_list, int* inv_mode_list);
void com_get_tpl_ref(COM_MODE* mod_info_curr, pel* pred_ref0, pel* pred_ref1/*, pel* weight_ref0, pel* weight_ref1*/);
void com_free_4d_Buf(pel**** array4D, int wNum, int hNum, int Num);
void com_malloc_4d_Buf(pel***** array4D, int wNum, int hNum, int Num, int sizeNum);
void com_derive_awp_tpl_weights(pel**** awp_weight_tpl, int width_idx, int height_idx, int awp_idx);
void com_calculate_awp_para(int awp_idx, int* blend_idx, int* step_idx, int* angle_idx, int* angle_area, int cu_width, int cu_height, int is_p_slice);
#else
void com_calculate_awp_para  (int awp_idx, int *step_idx, int *angle_idx, int *angle_area, int cu_width, int cu_height, int is_p_slice);
#endif
#if DSAWP
int com_tpl_reorder_sawp_mode(COM_MODE* mod_info_curr, int* mode_list, int* inv_mode_list, COM_INFO* info, u32* map_scu, s8* map_ipm, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int sawp_mpm_idx0, int sawp_mpm_idx1 , u8(*sawp_mpm_cache)[SAWP_MPM_NUM], pel(*pred_tpl_cache)[MAX_CU_DIM]);
int get_dawp_idx_from_sawp_idx(int ipm, u8 mpm[SAWP_MPM_NUM]);
int get_sawp_idx_from_dawp_idx(int ipm, u8 mpm[SAWP_MPM_NUM]);
#endif
#if SAWP_SCC == 0
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM], int is_p_slice, int is_sawp);
#else
void com_derive_awp_weight   (COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM], int is_p_slice);
#endif
#else
void com_calculate_awp_weight(pel awp_weight0[N_C][MAX_AWP_DIM], pel awp_weight1[N_C][MAX_AWP_DIM], int comp_idx, int cu_width, int cu_height, int step_idx, int angle_idx, int subangle_idx, int refine_flag);
void com_calculate_awp_para  (int awp_idx, int *step_idx, int *angle_idx, int *angle_area);
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM]);
#endif
void com_derive_awp_pred(COM_MODE *mod_info_curr, int compIdx, pel pred_buf0[N_C][MAX_CU_DIM], pel pred_buf1[N_C][MAX_CU_DIM], pel weight0[MAX_AWP_DIM], pel weight1[MAX_AWP_DIM]);
#if SAWP
void com_derive_sawp_pred(COM_MODE* mod_info_curr, int compIdx, pel pred_buf0[MAX_CU_DIM], pel pred_buf1[MAX_CU_DIM], pel weight0[MAX_AWP_DIM], pel weight1[MAX_AWP_DIM]);
#endif // SAWP

#endif

#if MVAP
void com_mvap_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
);
#endif

#if SUB_TMVP 
void com_sbTmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, s32 sub_blk_width, s32 sub_blk_height, COM_MOTION* sbTmvp, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
);
#endif

#if ETMVP
void com_etmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
);
#endif

#if USE_IBC
void com_IBC_mc(int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D], COM_PIC *ref_pic, pel pred[N_C][MAX_CU_DIM], CHANNEL_TYPE channel, int bit_depth
    );
#endif

void mv_clip(int x, int y, int pic_w, int pic_h, int w, int h,
             s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], s16(*mv_t)[MV_D]);

#if INTER_TM
void single_mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[MV_D]);
#endif

#if AWP_ENH
void com_dawp_mc(COM_INFO* info, COM_MODE* mod_info_curr, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth, COM_PIC* pic, u32* map_scu, pel**** awp_weight_tpl);
#endif
#if AWP
void com_awp_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth);
#endif
#if INTER_TM
void com_tm_process(int x, int y, int pic_w, int pic_h, int w, int h, COM_REFP(*refp)[REFP_NUM], s8 refi[REFP_NUM], s16 init_mv[REFP_NUM][MV_D], s16 mv[REFP_NUM][MV_D], pel* reco_luma, const int stride_luma, const int bit_depth, BOOL is_simplified);
void remove_duplicate_cands(s16(*pmv_cands)[REFP_NUM][MV_D], s8(*refi_cands)[REFP_NUM], int* num_cands_all);
void cands_adjustment(s16(*pmv_cands)[REFP_NUM][MV_D], s8(*refi_cands)[REFP_NUM], int* num_cands_all);
void pre_evaluation_cands(int x, int y, int pic_w, int pic_h, int w, int h, pel* reco_luma, const int stride_luma, COM_REFP(*refp)[REFP_NUM], s16(*pmv_cands)[REFP_NUM][MV_D], s8(*refi_cands)[REFP_NUM], int* num_cands_all, const int bit_depth, BOOL is_simplified);
void tm_padding(pel* img_pad, pel* img_rec, int s_rec, int width, int height, int offset);
#endif
void com_affine_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int bit_depth
#if AFFINE_DMVR
    , COM_DMVR* dmvr
#endif
);
#if ASP
void com_affine_mc_l(COM_INFO* info, int x, int y, int pic_w, int pic_h, int cu_width, int cu_height, CPMV ac_mv[VER_NUM][MV_D], COM_PIC* ref_pic, pel pred[MAX_CU_DIM], int cp_num, int sub_w, int sub_h, int bit_depth);
#else
void com_affine_mc_l(int x, int y, int pic_w, int pic_h, int cu_width, int cu_height, CPMV ac_mv[VER_NUM][MV_D], COM_PIC* ref_pic, pel pred[MAX_CU_DIM], int cp_num, int sub_w, int sub_h, int bit_depth);
#endif
void com_affine_mc_lc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, pel pred[N_C][MAX_CU_DIM], int sub_w, int sub_h, int lidx, int bit_depth);

#if SIMD_MC
#if AWP
#if SAWP_WEIGHT_OPT || AWP_ENH
void weight_average_16b_no_clip_sse(s16* src, s16* ref, s16* dst, s16* weight0, s16* weight1, int s_src, int s_ref, int s_dst, int s_weight, int wd, int ht, int shift);
#else
void weight_average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, s16 *weight0, s16 *weight1, int s_src, int s_ref, int s_dst, int s_weight, int wd, int ht);
#endif
#endif
void average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, int s_src, int s_ref, int s_dst, int wd, int ht);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _COM_MC_H_ */
