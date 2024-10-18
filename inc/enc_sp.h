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

#ifndef ENC_SM_H
#define ENC_SM_H

#include "com_usp.h"
#if USE_SP
#undef max
#undef min
#include <vector>
#include <math.h>

using namespace std;

#define SP_MAX_COST                       1.7e+308
#define SP_CHROMA_REFINE_CANDS            8
#define MAX_SM_TEST_CASE                  1000
#define SP_HASH_SIZE                      (1<<12)
static const u32     MAX_UINT =  0xFFFFFFFFU; ///< max. value of unsigned 32-bit integer 
extern "C" void* get_sp_instance(COM_SP_INPUT* input,  u8 isEncoding);
extern "C" void  release_sp_instance(void* sp_encoder);
extern "C" void  sm_pic_reset(void* sp_encoder, pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec);
extern "C" u8    sm_mode_rdcost(void* sp_encoder, COM_SP_CODING_UNIT * cur_sp_info, double*  min_rdcost, double*  distorion
, ENC_CTX *ctx
);
bool operator == (const COM_SP_PIX& _left, const COM_SP_PIX& _right);
bool operator != (const COM_SP_PIX& _left, const COM_SP_PIX& _right);
extern "C" void  sp_save_last_srb(void* sp_encoder, ENC_CU_DATA *cu_data, int cup);
extern "C" int   get_SSrb_bits(COM_SP_CODING_UNIT* p_cur_sp_info, COM_SP_PIX pixel_cur, int srb_length, int ui_idx, int ui_total, int* ui_pvbuf_adr, int ui_pvbuf_adr_prev, SP_MATCH_TYPE match_type_prev, int ui_pvbuf_adr_above, SP_MATCH_TYPE match_type_above, SP_MATCH_TYPE match_type_cur, int  pos_in_SRB, int ui_pvbuf_size, int ui_width);
extern "C" void  enc_bit_est_cs2(int log2_width, int log2_height, COM_BSW *bs, COM_SP_CODING_UNIT* p_cur_sp_info
    , ENC_CTX *ctx
    , int x, int y
);
extern "C" void  sp_lcu_hash_save(void* sp_encoder, int x_lcu_start, int y_lcu_start);
extern "C" void  sp_lcu_hash_restore(void* sp_encoder, int x_lcu_start, int y_lcu_start
    , int ctu_log2Size
);
extern "C" void  sp_cu_hash_copy(void* sp_encoder, int dst_w, int dst_h, int dst_status, int src_w, int src_h, int src_status);
extern "C" u8    sp_cu_hash_update(void* sp_encoder, int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start);
extern "C" u8    sp_is_hash_updated(u8 *map_usp, int x, int y, int cu_width, int cu_height, int pic_width_in_scu);
extern "C" u8    get_adaptive_sp_flag(void* sp_encoder);

struct SP_HASH_STATUS
{
    COM_SP_POS    m_hash_table[SP_HASH_SIZE];
    COM_SP_POS    m_dict[MAX_CU_SIZE][MAX_CU_SIZE];
    void          save(SP_HASH_STATUS* p);
};

typedef struct SP_CU_SIZE_STRING
{
    COM_SP_POS sm;
}SP_CU_SIZE_STRING;

class string_prediction
{
public:
    string_prediction(COM_SP_INPUT* input, u8 isEncoding);
    ~string_prediction();

private:
    string_prediction( const string_prediction& com_dict );
public:
    u8     rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion
        , ENC_CTX *ctx
    );
    u8     check_rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion
        , ENC_CTX *ctx
    );
    void   reset(pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec);
    int    get_sp_search_area_width(int x_start_in_pic, int uiMaxSearchWidthToLeftInCTUs);
    void   sp_postion_cand_update(u32  uiSad, int x, int y, u32* uiSadBestCand, SP_CU_SIZE_STRING* cSMPositionCand);
    int    sp_postion_chroma_refine(int sp_roi_width, int sp_roi_height, int cu_pel_x, int cu_pel_y, u32* uiSadBestCand, SP_CU_SIZE_STRING* cSMPositionCand);
    void   add_to_hash_table(int x_start_in_pic, int y_start_in_pic, int width, int height, u8 scale_x, u8 scale_y);
    u16    get_sp_hash_value_lossy(COM_SP_PIX pixelCur);
    u16(string_prediction::*get_sp_hashvalue)(COM_SP_PIX);
    void   save(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start);
    void   restore(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start);
    u8     sp_hash_update(int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start);
    u8     sp_hash_sp_judge(int x_start_in_pic, int y_start_in_pic);
    void   cu_size_str_search(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, COM_SP_POS& sp);
    u8     encode_cu_size_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* curBestRDCost, double* distortion);
#if FAST_SCC_PRECODING
    void   cu_size_str_searchrevisit(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, COM_SP_POS& sp);
#endif
    void   add_pixel_to_hash_table(int count, u8 includeHash, int x_start_in_pic, int y_start_in_pic, int cu_width_log2, int cu_height_log2, int iStartIdx, u8 bHorizontal = TRUE);
    int    get_tb_bits(int ui_symbol, int uiMaxSymbol);
    int    get_len_bits_cnt(int len, int totalPixel);
    int    get_offset_bits(int ui_symbol, int uiCount);
    int    get_svd_component_bits(u32 svd);
    int    get_exp_golomb_part_bits(u32 ui_symbol, int uiCount);
    int    check_str_len_bit(int len, int w, int h, int totalLeftPixel, u8 dir);
    double get_hor_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
        , int match_length, int offset_sign, int leftPixelInCU, int *bit_cnt
        , COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag);
    double get_ver_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
        , int match_length, int offset_sign, int leftPixelInCU, int *bit_cnt
        , COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag);
    void   get_string_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int match_length, int processed_count, u8 scale_x, u8 scale_y, int uiShiftY, int uiShiftC, u32 *prbDistY, u32 *prbDistUV);
    void   set_org_block(int uiWidth, int uiHeight, int x_start_in_pic, int y_start_in_pic);
#if SCC_CROSSINFO_ULTILIZE
    void   init_sps_cands(COM_SP_CODING_UNIT* p_cur_sp_info, COM_MOTION *cand, int cand_num, COM_MOTION *b_cand, int b_cand_num, vector<COM_MOTION> &sps_cands, u8 dir, COM_MOTION *n_cand, s8 n_cand_num);
#else
    void   init_sps_cands(COM_MOTION *cand, int cand_num, COM_MOTION *b_cand, int b_cand_num, vector<COM_MOTION> &sps_cands, u8 dir, COM_MOTION *n_cand, s8 n_cand_num);
#endif
    void   refine_sps_cands(vector<COM_MOTION>& org_sps_cands, vector<COM_MOTION>& refined_sps_cands, int current_x, int current_y);
    u8     encode_general_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* curBestRDCost, double* distortion);
    double get_unmatched_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
        , int match_length, int offset_sign, int leftPixelInCU, int *bit_cnt
        , COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag, int* match_dict, int* p_trav_scan_order, int cur_pixel);
    void   get_unmatched_string_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int match_length, int processed_count, u8 scale_x, u8 scale_y, int uiShiftY, int uiShiftC, u32 *prbDistY, u32 *prbDistUV, int* match_dict);
    u8     encode_evs_or_ubvs(COM_SP_CODING_UNIT* p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_EVS_INFO>& vec_dict_info, vector<COM_SP_PIX>& vec_dict_pixel, COM_SP_PIX* p_CQ_buf, double *cur_best_rdcost, u8 b_SRB_sharing_flag
        , ENC_CTX *ctx
    );
    unsigned int  pre_calc_RD_merge(COM_SP_CODING_UNIT* p_cur_sp_info, int ui_width, int ui_height, COM_SP_PIX* p_CQ_buf, vector<COM_SP_EVS_INFO>& vec_dict_info, unsigned int& dist_new, unsigned int calc_err_bits);
    void   decode_evs_ubvs(COM_SP_CODING_UNIT* p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, u8 is_horizontal_scanning);
    void   get_cu_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, unsigned int*  distortion_y, unsigned int*  distortion_uv);
    void   set_bit_depth(int type, int u) { m_bit_depth[type] = u; };
    void   derive_cluster(COM_SP_CODING_UNIT* p_cur_sp_info, pel *p_cluster[3], int ui_cluster_size, int ui_width, int ui_height, int x_start_in_pic, int y_start_in_pic, u8 b_cs2_mode_flag, double d_lambda, unsigned int** ind_error, unsigned int calc_error_bits);
    void   regen_cluster_info(COM_SP_CODING_UNIT* p_cur_sp_info, pel *p_cluster[3], int ui_cluster_size, int ui_width, int ui_height, int x_start_in_pic, int y_start_in_pic, unsigned int** ind_error, unsigned int calc_error_bits);
    void   set_srb_errLimit(int i_srb_errLimit) { m_srb_errlimit = i_srb_errLimit; }
    int    get_srb_errLimit() { return m_srb_errlimit; }
    void   set_srb_match_cnt(int i_srb_match_cnt) { m_srb_match_cnt = i_srb_match_cnt; }
    int    get_srb_match_cnt() { return m_srb_match_cnt; }
    void   init_TBC_table(u8 bit_depth);
    unsigned int  get_trunc_bin_bits(unsigned int ui_symbol, unsigned int ui_max_symbol);
    double calc_pix_pred_RD(COM_SP_CODING_UNIT* p_cur_sp_info, pel p_org[3], unsigned int *error);
    unsigned int  find_candidate_SRB_predictors(int srb_ind_best[], COM_SP_CODING_UNIT* p_cur_sp_info, pel *Srb[3], pel* p_pred[3], unsigned int ui_pvbuf_size_temp, unsigned int max_no_pred_ind);
    void   derive_srblossy(COM_SP_CODING_UNIT* p_cur_sp_info, pel *Srb[3], pel* p_src[3], int ui_width, int ui_height, int &ui_pvbuf_size);
    void   derive_srblossy_force_prediction(COM_SP_CODING_UNIT *p_cur_sp_info, pel *Srb[3], pel *p_src[3], int ui_width, int ui_height, int &ui_pvbuf_size);
    void   save_SRB(COM_SP_CODING_UNIT* p_cur_sp_info);
    void   reorder_SRB(COM_SP_CODING_UNIT* p_cur_sp_info, pel *p_srb[3], int ui_pvbuf_size, u8 b_cs2_mode_flag);
    void   SRB_predition(COM_SP_CODING_UNIT* p_cur_sp_info, COM_SP_PIX *p_hi_ref_UmP, unsigned int ui_hi_ref_UmPSize, u8* p_reused_prev);
    int    get_srb_length(COM_SP_CODING_UNIT*p_cur_sp_info, int width, int height, int xCuStartInPic, int yCuStartInPic, int offset_in_cu, unsigned int& dist, unsigned int& dist_y, unsigned int& dist_uv, COM_SP_PIX& pixelCur, SP_MATCH_TYPE& match_type_cur);
    int    get_SSrb_merge_info(COM_SP_CODING_UNIT*p_cur_sp_info, pel*p_SRB[3], int width, int height, int* p_trav_scan_order, unsigned int ui_idx, unsigned int ui_length, unsigned int& dist, int ui_srb_index, u8* index_blk, unsigned int *merge, int srb_mode);
    int    get_SSrb_index_bits(int ui_srb_index, int ui_max_symbol);
    int    get_SSrb_dist(COM_SP_CODING_UNIT*p_cur_sp_info, pel*p_SRB[3], int width, int height, int* p_trav_scan_order, unsigned int ui_idx, unsigned int ui_length, unsigned int& dist, u8* index_blk);
    int    get_SSrb_above_length(COM_SP_CODING_UNIT*p_cur_sp_info, int width, int height, int offset_in_cu, unsigned int& dist, unsigned int& dist_y, unsigned int& dist_uv, COM_SP_PIX& pixel_cur);
    unsigned int  calc_dist(COM_SP_PIX org_pixel, COM_SP_PIX rec_pixel, unsigned int& dist_y, unsigned int& dist_uv);
    unsigned int  calc_dist_norm(COM_SP_PIX org_pixel, COM_SP_PIX rec_pixel, unsigned int& dist_y, unsigned int& dist_uv);
    void   save_last_srb(ENC_CU_DATA *src, int cup);
    void   set_hi_ref_UmPSize(int i_value) { m_ui_hi_ref_UmPSize = i_value; };
    int    get_hi_ref_UmPSize() { return m_ui_hi_ref_UmPSize; };
    int    get_cluster_size() { return m_ui_pvbuf_size; };
    COM_SP_PIX* get_CQ_buf() { return m_p_CQ_pixbuffer; };
    int find_pos_in_hi_ref_UmP(pel* pix, bool b_update_internal = 0)
    {
        int pos = -1;
        for (int i = 0; i < m_ui_hi_ref_UmPSize; i++) {
            if (pix[0] == m_hi_ref_UmPix[i].Y && pix[1] == m_hi_ref_UmPix[i].U && pix[2] == m_hi_ref_UmPix[i].V) {
                pos = i;
                break;
            }
        }

        if (b_update_internal) {
            if (pos == -1) {
                m_hi_ref_UmPix[m_ui_hi_ref_UmPSize].Y = pix[0];
                m_hi_ref_UmPix[m_ui_hi_ref_UmPSize].U = pix[1];
                m_hi_ref_UmPix[m_ui_hi_ref_UmPSize].V = pix[2];
                m_hi_ref_UmPix_cnt[m_ui_hi_ref_UmPSize++] = 1;
            }
            else
            {
                m_hi_ref_UmPix_cnt[pos] ++;
            }
        }
        return pos;
    };
    void   reorder_hi_ref_UmP(int ui_hi_ref_UmPSize);
    void   set_max_pred_size(int ui_value) { m_srb_max_pred_size = ui_value; };
    void   set_max_pvbuf_size(int ui_value) { m_max_pvbuf_size = ui_value; };
    void   set_cluster_pred(bool b_flag) { m_pred_flag = b_flag; };
    void   set_hi_ref_flag(bool b_flag) { m_hi_ref_flag = b_flag; };
    void   set_lo_ref_flag(bool b_flag) { m_lo_ref_flag = b_flag; };

    __inline void copy_cu_size_str_info(SP_CU_SIZE_STRING *dst, SP_CU_SIZE_STRING *src) 
    {
        memcpy(dst, src, sizeof(SP_CU_SIZE_STRING));
    }

    __inline int get_eg_bits(int act_sym, int exp_golomb_order)
    {
        int uiLength = 0;
        while (1)
        {
            if (act_sym >= (1 << exp_golomb_order))
            {
                uiLength++;
                act_sym = act_sym - (1 << exp_golomb_order);
                exp_golomb_order++;
            }
            else
            {
                uiLength++;
                while (exp_golomb_order--)
                {
                    uiLength++;
                }
                break;
            }
        }
        return uiLength;
    }

    __inline int get_sp_offset_bit_cnt(int offset_x, int offset_y)
    {
        int bits = 0;
        int uiHorAbs = 0 > offset_x ? -offset_x : offset_x;
        int uiVerAbs = 0 > offset_y ? -offset_y : offset_y;
        bits += get_eg_bits(uiHorAbs - 1, 0);
        bits += get_eg_bits(uiVerAbs - 1, 0);
        return bits;
    }

    __inline u8 is_sv_valid(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int width, int height, int x, int y)
    {
        int ctu_log2size = p_cur_sp_info->ctu_log2size;
        int ctu_size = 1 << ctu_log2size;
        int ctu_x = x >> ctu_log2size << ctu_log2size;
        int ctu_y = y >> ctu_log2size << ctu_log2size;
        int new_x_left = x + offset_x;
        int new_x_right = x + offset_x + width - 1;
        int new_y_top = y + offset_y;
        int new_bottom_y = y + offset_y + height - 1;
        int numLeftCTUs = (1 << ((7 - ctu_log2size) << 1)) - ((ctu_log2size < 7) ? 1 : 0);
        int max_width = ctu_size * numLeftCTUs;

        if (new_x_left < ctu_x - max_width)
            return 0;
        if (new_x_right > ctu_x + ctu_size - 1)
            return 0;
        if (new_y_top < ctu_y)
            return 0;
        if (new_bottom_y > ctu_y + ctu_size - 1)
            return 0;
        return 1;
    }
#if CSS_REF_RANGE_XLCUR!=0||CSS_REF_RANGE_XqLCU!=0
    __inline u8 is_sv_validXqLCU(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int width, int height, int x, int y, int ref_Y_LT, int m_pic_width)
    {
        int ctu_log2size = p_cur_sp_info->ctu_log2size;
        int ctu_size = 1 << ctu_log2size;
        int ctu_x = x >> ctu_log2size << ctu_log2size;
        int ctu_y = y >> ctu_log2size << ctu_log2size;
        int new_x_left = x + offset_x;
        int new_y_top = y + offset_y;
        int new_x_right = x + offset_x + width - 1;
        int new_bottom_y = y + offset_y + height - 1;
        int numLeftCTUs = (1 << ((7 - ctu_log2size) << 1)) - ((ctu_log2size < 7) ? 1 : 0);
        int max_width = ctu_size * numLeftCTUs;
#if CSS_REF_RANGE_XqLCU!=0
        int xqlcu_mv = CSS_REF_RANGE_XqLCU * ctu_size;
        if (new_x_left < ctu_x - xqlcu_mv)
            return 0;
#else
        if (new_x_left < ctu_x - max_width)
            return 0;
#endif
#if IBSC_RIGHT_BOUND==0
        if (new_x_right > ctu_x + ctu_size - 1)
            return 0;
#endif 

#if CSS_REF_RANGE_XLCUR!=0
        int XLCUR_mv = (CSS_REF_RANGE_XLCUR - 1) * ctu_size;
        if (new_y_top < ctu_y - XLCUR_mv)
            return 0;
#else
        if (new_y_top < ctu_y)
            return 0;
#endif
#if IBSC_RIGHT_BOUND!=0
        int ctu_xrightbound = 0;
        if ((new_y_top >> ctu_log2size) == (y >> ctu_log2size))
        {
            ctu_xrightbound = ctu_size;
            if (new_x_right > ctu_x + ctu_xrightbound - 1)
            {
                return 0;
            }
        }
        else
        {
            ctu_xrightbound = m_pic_width;
            if (new_x_right > ctu_xrightbound - 1)
            {
                return 0;
            }
        }
#endif 
        if (new_bottom_y > ctu_y + ctu_size - 1)
            return 0;
        return 1;
    }
#endif

    __inline u8 is_ref_region_coded(COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int sp_roi_width, int sp_roi_height, int sv_x, int sv_y)
    {
        int pic_width_in_scu = p_cur_sp_info->pic_width_in_scu;      
        u32 *map_scu = p_cur_sp_info->map_scu;
        int upLeftOffset = (ref_Y_LT >> MIN_CU_LOG2)*pic_width_in_scu + (ref_X_LT >> MIN_CU_LOG2);
        int bottomRightOffset = ((ref_Y_LT + sp_roi_height - 1) >> MIN_CU_LOG2)*pic_width_in_scu + ((ref_X_LT + sp_roi_width - 1) >> MIN_CU_LOG2);
        if (sv_x <= 0 && sv_y < 0)
        {
            if (MCU_GET_CODED_FLAG(map_scu[upLeftOffset]))
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        if (MCU_GET_CODED_FLAG(map_scu[bottomRightOffset]) && MCU_GET_CODED_FLAG(map_scu[upLeftOffset])) 
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }

    __inline u8 is_ref_pix_coded(COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT) 
    {
        int pic_width_in_scu = p_cur_sp_info->pic_width_in_scu;
        u32 *map_scu = p_cur_sp_info->map_scu;
        int upLeftOffset = (ref_Y_LT >> MIN_CU_LOG2) * pic_width_in_scu + (ref_X_LT >> MIN_CU_LOG2);
        if (MCU_GET_CODED_FLAG(map_scu[upLeftOffset]))
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }

    //One-CTU mem reuse
    __inline u8 is_ref_region_valid(COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int sp_roi_width, int sp_roi_height,int cu_pel_x, int cu_pel_y) 
    {
        if (ref_Y_LT < 0 || ref_Y_LT + sp_roi_height - 1 >= m_pic_height || ref_X_LT < 0 || ref_X_LT + sp_roi_width - 1 >= m_pic_width)
        {
            return 0;
        }
#if CSS_REF_RANGE_XqLCU==0 && CSS_REF_RANGE_XLCUR==0
        if (!is_sv_valid(p_cur_sp_info, ref_X_LT - cu_pel_x, ref_Y_LT - cu_pel_y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            return 0;
        }
#else
        if (!is_sv_validXqLCU(p_cur_sp_info, ref_X_LT - cu_pel_x, ref_Y_LT - cu_pel_y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y, ref_Y_LT, m_pic_width))
        {
            return 0;
        }
#endif 

        int ctu128 = p_cur_sp_info->ctu_log2size == 7 ? 1 : 0;
        int log2Size = p_cur_sp_info->ctu_log2size == 7 ? 6 : p_cur_sp_info->ctu_log2size;
#if CSS_REF_RANGE_XqLCU==0
#if CSS_REF_RANGE_XLCUR==0||CSS_REF_RANGE_XLCUR==1
        int offset64x = ((ref_X_LT + m_max_cu_width) >> log2Size) << log2Size;
        int offset64y = (ref_Y_LT >> log2Size) << log2Size;
        if (ctu128)
        {
            assert(p_cur_sp_info->ctu_log2size == 7 && m_max_cu_width == 128);
            if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1) && offset64x < m_pic_width) //in the left LCU but already updated
            {
                if (is_ref_pix_coded(p_cur_sp_info, offset64x, offset64y) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) // already coded or just current cu
                {
                    return 0;
                }
            }
        }
#endif 
#else
        int ctu_luclength = 0;
        int ctu_mvx = CSS_REF_RANGE_XqLCU - 1;
#if REF_RANGE_FIX
        ctu_luclength = (1 << SRAM_SIZE_128_LOG2) * (CSS_REF_RANGE_XqLCU - 1);
#else
        ctu_luclength = 128 * (CSS_REF_RANGE_XqLCU - 1);
#endif
        int offset64x = ((ref_X_LT + m_max_cu_width + ctu_luclength) >> log2Size) << log2Size;
        int offset64y = (ref_Y_LT >> log2Size) << log2Size;
        if (ctu128)
        {
            assert(p_cur_sp_info->ctu_log2size == 7 && m_max_cu_width == 128);
            if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1 - ctu_mvx) && offset64x < m_pic_width) //in the left LCU but already updated
            {
                if (is_ref_pix_coded(p_cur_sp_info, offset64x, offset64y) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) // already coded or just current cu
                {
                    return 0;
                }
            }
        }
#endif

        if (!is_ref_region_coded(p_cur_sp_info, ref_X_LT, ref_Y_LT, sp_roi_width, sp_roi_height, ref_X_LT - cu_pel_x, ref_Y_LT - cu_pel_y))
        { 
            return 0;
        }
#if !CSS_CROSS_64
        int offset64x_br = ((ref_X_LT + sp_roi_width - 1 + m_max_cu_width) >> log2Size) << log2Size;
        int offset64y_br = ((ref_Y_LT + sp_roi_height - 1) >> log2Size) << log2Size;
        if (offset64x != offset64x_br || offset64y != offset64y_br)
        {
            return 0;
        }
#endif 
        return 1;
    }

    __inline u8 is_ref_pix_valid(int* puiReverseOrder, COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int processed_count) 
    {
        int cu_width_log2 = p_cur_sp_info->cu_width_log2;
        int cu_height_log2 = p_cur_sp_info->cu_height_log2;
        if (is_ref_pix_coded(p_cur_sp_info,ref_X_LT,ref_Y_LT))  // in the coded cu
        {
            return 1;
        }
        else if ((ref_X_LT >= p_cur_sp_info->cu_pix_x &&ref_X_LT < p_cur_sp_info->cu_pix_x + (1 << cu_width_log2)) && (ref_Y_LT >= p_cur_sp_info->cu_pix_y &&ref_Y_LT < p_cur_sp_info->cu_pix_y + (1 << cu_height_log2))) //in the same cu
        {
            int trav_x = ref_X_LT - p_cur_sp_info->cu_pix_x;
            int trav_y = ref_Y_LT - p_cur_sp_info->cu_pix_y;
            int RefPointOrder = puiReverseOrder[trav_y * (1 << cu_width_log2) + trav_x];
            if (RefPointOrder < processed_count)
                return 1;
            else
                return 0;
        }
        else 
        {
            return 0;
        }
    }

    __inline u8 is_ref_pix_in_one_ctu(int* puiReverseOrder, COM_SP_CODING_UNIT* p_cur_sp_info, int ref_X_LT, int ref_Y_LT, int processed_count,int fir_refx, int fir_refy)
    {
        //judge if SP reference point is in One-CTU
        int cu_pel_x = p_cur_sp_info->cu_pix_x;
        int cu_pel_y = p_cur_sp_info->cu_pix_y;
        int sp_roi_width = 1 << p_cur_sp_info->cu_width_log2;
        int sp_roi_height = 1 << p_cur_sp_info->cu_height_log2;
        int ctu128 = p_cur_sp_info->ctu_log2size == 7 ? 1 : 0;
        int log2size = p_cur_sp_info->ctu_log2size == 7 ? 6 : p_cur_sp_info->ctu_log2size;
#if GS_REF_RANGE_XqLCU==0
#if GS_REF_RANGE_XLCUR==0||GS_REF_RANGE_XLCUR==1
        int offset64x = ((ref_X_LT + m_max_cu_width) >> log2size) << log2size;
        int offset64y = (ref_Y_LT >> log2size) << log2size;
        if (ctu128)
        {
            assert(p_cur_sp_info->ctu_log2size == 7 && m_max_cu_width == 128);
            if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1) && offset64x < m_pic_width) //in the left LCU but already updated
            {
                if (is_ref_pix_coded(p_cur_sp_info, offset64x, offset64y) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) //not in the mem reuse area
                {
                    return 0;
                }
            }
        }
#endif 
#else
        int ctu_mvx = 0;
        int ctu_luclength = 0;
        ctu_mvx = GS_REF_RANGE_XqLCU - 1;
#if REF_RANGE_FIX
        ctu_luclength = (1 << SRAM_SIZE_128_LOG2) * (GS_REF_RANGE_XqLCU - 1);
#else
        ctu_luclength = 128 * (GS_REF_RANGE_XqLCU - 1);
#endif
        int offset64x = ((ref_X_LT + m_max_cu_width + ctu_luclength) >> log2size) << log2size;
        int offset64y = (ref_Y_LT >> log2size) << log2size;
        if (ctu128)
        {
            assert(p_cur_sp_info->ctu_log2size == 7 && m_max_cu_width == 128);
            if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1 - ctu_mvx)) //in the left LCU but already updated
            {
                if (is_ref_pix_coded(p_cur_sp_info, offset64x, offset64y) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) //not in the mem reuse area
                {
                    return 0;
                }
            }
        }
#endif
        if (!is_ref_pix_valid(puiReverseOrder, p_cur_sp_info, ref_X_LT, ref_Y_LT, processed_count))
        {
            return 0;
        }
#if !GS_CROSS_64
        int fir_ref_64x = ((fir_refx + m_max_cu_width) >> log2size) << log2size;
        int fir_ref_64y = (fir_refy >> log2size) << log2size;
        if (offset64x != fir_ref_64x || offset64y != fir_ref_64y)
        {
            return 0;
        }
#endif 
        return 1;
    }

#if RBU_LIMIT_GS||RBU_LIMIT_CSS||ExtStrGS||ExtStrCSS
    __inline int is_ref_pix_in_raw_range(int x, int y, int width, int height, int ref_X_LT, int ref_Y_LT)
    {
        int cu_pel_x = x;//in pic
        int cu_pel_y = y;
#if SCC_ENC_OPT
        int x_in_lcu = cu_pel_x & ((1 << MAX_CU_LOG22) - 1);
        int y_in_lcu = cu_pel_y & ((1 << MAX_CU_LOG22) - 1);
#else
        int x_in_lcu = cu_pel_x & ((1 << MAX_CU_LOG2) - 1);
        int y_in_lcu = cu_pel_y & ((1 << MAX_CU_LOG2) - 1);
#endif
        int sp_roi_width = width;
        int sp_roi_height = height;
#if SCC_ENC_OPT
        int offset64x = ((ref_X_LT + MAX_CU_SIZE2) >> (MAX_CU_LOG22 - 1)) << (MAX_CU_LOG22 - 1);
        int offset64y = (ref_Y_LT >> (MAX_CU_LOG22 - 1)) << (MAX_CU_LOG22 - 1);
        if (ref_Y_LT >> MAX_CU_LOG22 == ((cu_pel_y >> MAX_CU_LOG22)))
        {
            if ((ref_X_LT >> MAX_CU_LOG22) == ((cu_pel_x >> MAX_CU_LOG22) - 1)) //in the left LCU but already updated
#else
        int offset64x = ((ref_X_LT + MAX_CU_SIZE) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);//ref_X_LT ref_x_in_pic_n;ref_Y_LT ref_y_in_pic_n
        int offset64y = (ref_Y_LT >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        if (ref_Y_LT >> MAX_CU_LOG2 == ((cu_pel_y >> MAX_CU_LOG2)))
        {
            if ((ref_X_LT >> MAX_CU_LOG2) == ((cu_pel_x >> MAX_CU_LOG2) - 1)) //in the left LCU but already updated
#endif
            {
#if REF_RANGE_FIX
                int cu_size64_log2 = SRAM_SIZE_128_LOG2 - 1;
                int cu_size64 = 1 << cu_size64_log2;
                if ((x_in_lcu / cu_size64) && (y_in_lcu / cu_size64))
                {
                    return 0;
                }
                else if ((!(x_in_lcu / cu_size64)) && (!(y_in_lcu / cu_size64)))
                {
                    if ((offset64x == (cu_pel_x >> cu_size64_log2 << cu_size64_log2)) && (offset64y == (cu_pel_y >> cu_size64_log2 << cu_size64_log2)))
                        return 0;
                    else
                        return 1;
                }
                else if ((!(x_in_lcu / cu_size64)) && ((y_in_lcu / cu_size64)))
                {
                    if ((offset64y == (cu_pel_y >> cu_size64_log2 << cu_size64_log2)) && (offset64x == ((cu_pel_x >> cu_size64_log2 << cu_size64_log2) + cu_size64)))
                        return  1;
                    else
                        return 0;
                }
                else
                {
                    if (offset64y == (cu_pel_y >> cu_size64_log2 << cu_size64_log2))
                        return 0;
                    else
                        return 1;
                }
#else
                if ((x_in_lcu / 64) && (y_in_lcu / 64))
                {
                    return 0;
                }
                else if ((!(x_in_lcu / 64)) && (!(y_in_lcu / 64)))
                {
                    if ((offset64x == (cu_pel_x >> 6 << 6)) && (offset64y == (cu_pel_y >> 6 << 6)))
                        return 0;
                    else
                        return 1;
                }
                else if ((!(x_in_lcu / 64)) && ((y_in_lcu / 64)))
                {
                    if ((offset64y == (cu_pel_y >> 6 << 6)) && (offset64x == ((cu_pel_x >> 6 << 6) + 64)))
                        return  1;
                    else
                        return 0;
                }
                else
                {
                    if (offset64y == (cu_pel_y >> 6 << 6))
                        return 0;
                    else
                        return 1;
                }
#endif
            }
#if SCC_ENC_OPT
            else if (ref_X_LT >> MAX_CU_LOG22 == ((cu_pel_x >> MAX_CU_LOG22)))
#else
            else if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2)))
#endif
            {
                return 1;
            }
            else {
                return 0;
            }
        }
        else
        {
            return 0;
        }
    }
#endif
#if RBU_LIMIT_GS||RBU_LIMIT_CSS
    __inline int calc_recString_ext_rect_coord(int CuWidth, int x_start_in_pic, int y_start_in_pic, int start_x, int start_y, int end_x, int end_y, int NumCodedPixel, int SL, int sv[2])
    {
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        int Ystart = NumCodedPixel / CuWidth;
        int Yend = (NumCodedPixel + SL - 1) / CuWidth;
        int NbuFirstRow = 0, NbuLastRow = 0, NbuMidRow = 0, Nbu = 0;
        if (Yend == Ystart)
        {
            x0 = COM_MIN(start_x, end_x) + sv[0];  y0 = start_y + sv[1];
            x1 = COM_MAX(start_x, end_x) + sv[0];  y1 = end_y + sv[1];
            Nbu = ((x1 >> RBU_W_LOG2) - (x0 >> RBU_W_LOG2) + 1) * ((y1 >> RBU_H_LOG2) - (y0 >> RBU_H_LOG2) + 1);
        }
        else if (Yend == (Ystart + 1))
        {

            if ((Ystart) % 2)
            {
                if (((start_y + sv[1]) >> RBU_H_LOG2) != ((start_y + 1 + sv[1]) >> RBU_H_LOG2))
                {
                    NbuFirstRow = ((start_x + sv[0]) >> RBU_W_LOG2) - ((x_start_in_pic + sv[0]) >> RBU_W_LOG2) + 1;
                    NbuLastRow = ((end_x + sv[0]) >> RBU_W_LOG2) - ((x_start_in_pic + sv[0]) >> RBU_W_LOG2) + 1;
                    Nbu = NbuFirstRow + NbuLastRow;
                }
                if (!(NbuFirstRow || NbuLastRow))
                {
                    x0 = x_start_in_pic + sv[0];  y0 = start_y + sv[1];
                    x1 = COM_MAX(start_x, end_x) + sv[0];  y1 = end_y + sv[1];
                    Nbu = ((x1 >> RBU_W_LOG2) - (x0 >> RBU_W_LOG2) + 1) * ((y1 >> RBU_H_LOG2) - (y0 >> RBU_H_LOG2) + 1);
                }
            }
            else
            {
                if (((start_y + sv[1]) >> RBU_H_LOG2) != ((start_y + 1 + sv[1]) >> RBU_H_LOG2))
                {
                    NbuFirstRow = ((x_start_in_pic + CuWidth - 1 + sv[0]) >> RBU_W_LOG2) - ((start_x + sv[0]) >> RBU_W_LOG2) + 1;
                    NbuLastRow = ((x_start_in_pic + CuWidth - 1 + sv[0]) >> RBU_W_LOG2) - ((end_x + sv[0]) >> RBU_W_LOG2) + 1;
                    Nbu = NbuFirstRow + NbuLastRow;
                }
                if (!(NbuFirstRow || NbuLastRow))
                {
                    x0 = COM_MIN(start_x, end_x) + sv[0];  y0 = start_y + sv[1];
                    x1 = x_start_in_pic + CuWidth - 1 + sv[0];  y1 = end_y + sv[1];
                    Nbu = ((x1 >> RBU_W_LOG2) - (x0 >> RBU_W_LOG2) + 1) * ((y1 >> RBU_H_LOG2) - (y0 >> RBU_H_LOG2) + 1);
                }
            }
        }
        else
        {
            y0 = start_y + sv[1];  y1 = end_y + sv[1];
            if (((start_y + sv[1]) >> RBU_H_LOG2) != ((start_y + 1 + sv[1]) >> RBU_H_LOG2))
            {
                if ((Ystart) % 2)
                    NbuFirstRow = ((start_x + sv[0]) >> RBU_W_LOG2) - ((x_start_in_pic + sv[0]) >> RBU_W_LOG2) + 1;
                else
                    NbuFirstRow = ((x_start_in_pic + CuWidth - 1 + sv[0]) >> RBU_W_LOG2) - ((start_x + sv[0]) >> RBU_W_LOG2) + 1;
                y0 = start_y + 1 + sv[1];
            }
            if (((end_y + sv[1]) >> RBU_H_LOG2) != ((end_y - 1 + sv[1]) >> RBU_H_LOG2))
            {
                if ((Yend) % 2)
                    NbuLastRow = ((x_start_in_pic + CuWidth - 1 + sv[0]) >> RBU_W_LOG2) - ((end_x + sv[0]) >> RBU_W_LOG2) + 1;
                else
                    NbuLastRow = ((end_x + sv[0]) >> RBU_W_LOG2) - ((x_start_in_pic + sv[0]) >> RBU_W_LOG2) + 1;
                y1 = end_y - 1 + sv[1];
            }
            if (!(NbuFirstRow || NbuLastRow))
            {
                x0 = x_start_in_pic + sv[0];
                x1 = x_start_in_pic + CuWidth - 1 + sv[0];
                Nbu = ((x1 >> RBU_W_LOG2) - (x0 >> RBU_W_LOG2) + 1) * ((y1 >> RBU_H_LOG2) - (y0 >> RBU_H_LOG2) + 1);
            }
            else
            {
                x0 = x_start_in_pic + sv[0];
                x1 = x_start_in_pic + CuWidth - 1 + sv[0];
                NbuMidRow = ((x1 >> RBU_W_LOG2) - (x0 >> RBU_W_LOG2) + 1) * ((y1 >> RBU_H_LOG2) - (y0 >> RBU_H_LOG2) + 1);
                Nbu = NbuLastRow + NbuMidRow + NbuFirstRow;
            }
        }
        return Nbu;
    }
#endif

    __inline u8 is_pix_in_errorlimit(int trav_order_index, int ref_pos, int ref_pos_c, int errorLimit, u8 tree_status)
    {
        if (tree_status == TREE_LC)
        {
            if (abs(m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_org[0][ref_pos]) < errorLimit &&
                abs(m_p_org_pixel_buffer[trav_order_index].U - m_p_pel_org[1][ref_pos_c]) < errorLimit &&
                abs(m_p_org_pixel_buffer[trav_order_index].V - m_p_pel_org[2][ref_pos_c]) < errorLimit)
            {
                return TRUE;
            }
            else
            {
                return FALSE;
            }
        }
        else 
        {
            if (abs(m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_org[0][ref_pos]) < errorLimit)
            {
                return TRUE;
            }
            else
            {
                return FALSE;
            }
        }
    }
public:
    COM_SP_INPUT       iptemp;
    COM_SP_INPUT      *input;
    SP_HASH_STATUS ****m_ppp_dict_status;
    int                m_pic_width;
    int                m_pic_height;
private:
    int                m_max_cu_width;
    int                m_max_cu_height;
    u8                 m_is_encoding;
    pel               *m_p_pel_org[3];
    pel               *m_p_pel_rec[3];
    int                m_stride_org[3];
    int                m_stride_rec[3];
    pel               *m_p_pel_org444[3];
    pel               *m_p_pel_rec444[3];
    u8                *m_all_comp_rec444;
    int                m_bit_depth[3]; 
    int                m_init_TBC_table;
    int                m_srb_max_pred_size;
    int                m_max_pvbuf_size;
    bool               m_pred_flag;
    //                   
    int                m_srb_match_cnt;
    int                m_srb_errlimit;
    u16              **m_trunc_bin_bits; // two dimensions related with input bitdepth
    int                m_max_symbol_size;
    int                m_symbol_size;
    //
    u8                *m_b_major;
    u8                *m_c_match_index;
    COM_SP_PIX        *m_p_CQ_pixbuffer;
    u8                 m_esc_flag;
    int                m_ui_pvbuf_size;
    int                m_ui_hi_ref_UmPSize;
    u8                 m_hi_ref_flag;
    u8                 m_lo_ref_flag;
    COM_SP_PIX        *m_hi_ref_UmPix;
    int               *m_hi_ref_UmPix_cnt;
    //Merge opt
    unsigned int     **m_ind_error;
    u8                *m_c_index_block;
    int                m_pv_flag[MAX_CU_DIM];
    COM_SP_PIX         m_p_org_pixel_buffer[MAX_CU_DIM];
    COM_SP_POS         m_hash_table[SP_HASH_SIZE];
    COM_SP_POS       **m_dict;
};

u8 sp_max_error_quant[60] = { 0, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 21, 22, 24, 23, 25, 26, 28, 29, 31, 32, 34, 36, 37, 39, 41, 42, 45, 46, 47, 48, 49, 50, 51, 52, 53 };

class SORTING_ELEMENT
{
public:
    int ui_cnt;
    int ui_data[3];
    int ui_shift, ui_last_cnt, ui_sum_data[3];

    inline bool operator<(const SORTING_ELEMENT &other) const
    {
        return (ui_cnt > other.ui_cnt) ? 1 : 0;
    }

    SORTING_ELEMENT() {
        ui_cnt = ui_shift = ui_last_cnt = 0;
        ui_data[0] = ui_data[1] = ui_data[2] = 0;
        ui_sum_data[0] = ui_sum_data[1] = ui_sum_data[2] = 0;
    }
    void set_all(int ui0, int ui1, int ui2) {
        if (!ui0 && !ui1 && !ui2)
        {
            ui_shift = ui_last_cnt = 0;
            ui_sum_data[0] = ui_sum_data[1] = ui_sum_data[2] = 0;
        }
        ui_data[0] = ui0; ui_data[1] = ui1; ui_data[2] = ui2;
    }
    bool equal_data(SORTING_ELEMENT s_element)
    {
        return (ui_data[0] == s_element.ui_data[0]) && (ui_data[1] == s_element.ui_data[1]) && (ui_data[2] == s_element.ui_data[2]) ? 1 : 0;
    }

    void reset_element()
    {
        ui_cnt = ui_shift = ui_last_cnt = 0;
        ui_data[0] = ui_data[1] = ui_data[2] = 0;
        ui_sum_data[0] = ui_sum_data[1] = ui_sum_data[2] = 0;
    }

    bool almost_equal_data(SORTING_ELEMENT s_element, int i_errorLimit, const int bit_depths[3])
    {
        return ((((abs(ui_data[0] - s_element.ui_data[0]) >> (bit_depths[0] - 8)) <= i_errorLimit)
            && (((abs(ui_data[1] - s_element.ui_data[1]) >> (bit_depths[1] - 8)) * UV_WEIGHT) <= i_errorLimit)
            && (((abs(ui_data[2] - s_element.ui_data[2]) >> (bit_depths[1] - 8)) * UV_WEIGHT) <= i_errorLimit))
            ? 1 : 0);
    }
    int get_SAD(SORTING_ELEMENT s_element, const int bit_depths[3])
    {
        return (int)((abs(ui_data[0] - s_element.ui_data[0]) >> (bit_depths[0] - 8))
            + (abs(ui_data[1] - s_element.ui_data[1]) >> (bit_depths[1] - 8)) * UV_WEIGHT
            + (abs(ui_data[2] - s_element.ui_data[2]) >> (bit_depths[1] - 8)) * UV_WEIGHT);
    }

    void copy_data_from(SORTING_ELEMENT s_element) {
        ui_data[0] = s_element.ui_data[0];
        ui_data[1] = s_element.ui_data[1];
        ui_data[2] = s_element.ui_data[2];
        ui_shift = 0; ui_last_cnt = 1; ui_sum_data[0] = ui_data[0]; ui_sum_data[1] = ui_data[1]; ui_sum_data[2] = ui_data[2];
    }
    void copy_all_from(SORTING_ELEMENT s_element) {
        copy_data_from(s_element); ui_cnt = s_element.ui_cnt;
        ui_sum_data[0] = s_element.ui_sum_data[0]; ui_sum_data[1] = s_element.ui_sum_data[1]; ui_sum_data[2] = s_element.ui_sum_data[2];
        ui_last_cnt = s_element.ui_last_cnt; ui_shift = s_element.ui_shift;
    }

    void add_element(const SORTING_ELEMENT& s_element)
    {
        ui_cnt++;
        for (int i = 0; i < 3; i++)
        {
            ui_sum_data[i] += s_element.ui_data[i];
        }
        if (ui_cnt > 1 && ui_cnt == 2 * ui_last_cnt)
        {
            int ui_rnd;
            if (ui_cnt == 2)
            {
                ui_shift = 0;
                ui_rnd = 1;
            }
            else
            {
                ui_rnd = 1 << ui_shift;
            }
            ui_shift++;
            for (int i = 0; i < 3; i++)
            {
                ui_data[i] = (ui_sum_data[i] + ui_rnd) >> ui_shift;
            }
            ui_last_cnt = ui_cnt;
        }
    }
};
#endif
#endif //_ENC_SM_H