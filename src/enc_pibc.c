/* ====================================================================================================================

The copyright in this software is being made available under the License included below.
This software may be subject to other third party and contributor rights, including patent rights, and no such
rights are granted under this license.

Copyright (c) 2018, HUAWEI TECHNOLOGIES CO., LTD. All rights reserved.
Copyright (c) 2018, SAMSUNG ELECTRONICS CO., LTD. All rights reserved.
Copyright (c) 2018, PEKING UNIVERSITY SHENZHEN GRADUATE SCHOOL. All rights reserved.
Copyright (c) 2018, PENGCHENG LABORATORY. All rights reserved.
Copyright (c) 2019, TENCENT CO., LTD. All rights reserved.
Copyright (c) 2019, WUHAN UNIVERSITY. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted only for
the purpose of developing standards within Audio and Video Coding Standard Workgroup of China (AVS) and for testing and
promoting such standards. The following conditions are required to be met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and
the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
the following disclaimer in the documentation and/or other materials provided with the distribution.
* The name of HUAWEI TECHNOLOGIES CO., LTD. or SAMSUNG ELECTRONICS CO., LTD. or TENCENT CO., LTD. may not be used to endorse or promote products derived from
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

#include <math.h>

#include "enc_def.h"
#include "enc_ibc_hash_wrapper.h"
#if RSD_OPT
#include "enc_rsd_hash_wrapper.h"
#endif
#if USE_IBC
#define SWAP(a, b, t) { (t) = (a); (a) = (b); (b) = (t); }

#define CHROMA_REFINEMENT_CANDIDATES         8  /* 8 candidates BV to choose from */
#define MV_FRACTIONAL_BITS_INTERNAL          0

static s16    resi_t[N_C][MAX_CU_DIM];

#if TB_SPLIT_EXT
static u32 calc_sad_16b(int pu_w, int pu_h, void *src1, void *src2, int s_src1, int s_src2, int bit_depth)
{
    u32 cost = 0;
    int num_seg_in_pu_w = 1, num_seg_in_pu_h = 1;
    int seg_w_log2 = com_tbl_log2[pu_w];
    int seg_h_log2 = com_tbl_log2[pu_h];
    s16 *src1_seg, *src2_seg;
    s16* s1 = (s16 *)src1;
    s16* s2 = (s16 *)src2;

    if (seg_w_log2 == -1)
    {
        num_seg_in_pu_w = 3;
        seg_w_log2 = (pu_w == 48) ? 4 : (pu_w == 24 ? 3 : 2);
    }

    if (seg_h_log2 == -1)
    {
        num_seg_in_pu_h = 3;
        seg_h_log2 = (pu_h == 48) ? 4 : (pu_h == 24 ? 3 : 2);
    }

    if (num_seg_in_pu_w == 1 && num_seg_in_pu_h == 1)
    {
        cost += enc_sad_16b(seg_w_log2, seg_h_log2, s1, s2, s_src1, s_src2, bit_depth);
        return cost;
    }

    for (int j = 0; j < num_seg_in_pu_h; j++)
    {
        for (int i = 0; i < num_seg_in_pu_w; i++)
        {
            src1_seg = s1 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src1;
            src2_seg = s2 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src2;
            cost += enc_sad_16b(seg_w_log2, seg_h_log2, src1_seg, src2_seg, s_src1, s_src2, bit_depth);
        }
    }
    return cost;
}

static u32 calc_satd_16b(int pu_w, int pu_h, void *src1, void *src2, int s_src1, int s_src2, int bit_depth)
{
    u32 cost = 0;
    int num_seg_in_pu_w = 1, num_seg_in_pu_h = 1;
    int seg_w_log2 = com_tbl_log2[pu_w];
    int seg_h_log2 = com_tbl_log2[pu_h];
    s16 *src1_seg, *src2_seg;
    s16* s1 = (s16 *)src1;
    s16* s2 = (s16 *)src2;

    if (seg_w_log2 == -1)
    {
        num_seg_in_pu_w = 3;
        seg_w_log2 = (pu_w == 48) ? 4 : (pu_w == 24 ? 3 : 2);
    }

    if (seg_h_log2 == -1)
    {
        num_seg_in_pu_h = 3;
        seg_h_log2 = (pu_h == 48) ? 4 : (pu_h == 24 ? 3 : 2);
    }

    if (num_seg_in_pu_w == 1 && num_seg_in_pu_h == 1)
    {
        cost += enc_satd_16b(seg_w_log2, seg_h_log2, s1, s2, s_src1, s_src2, bit_depth);
        return cost;
    }

    for (int j = 0; j < num_seg_in_pu_h; j++)
    {
        for (int i = 0; i < num_seg_in_pu_w; i++)
        {
            src1_seg = s1 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src1;
            src2_seg = s2 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src2;
            cost += enc_satd_16b(seg_w_log2, seg_h_log2, src1_seg, src2_seg, s_src1, s_src2, bit_depth);
        }
    }
    return cost;
}
#endif
#if RBU_LIMIT_IBC
int calc_recString_ext_rect_coord(int CuWidth, int x_start_in_pic, int y_start_in_pic, int start_x, int start_y, int end_x, int end_y, int NumCodedPixel, int SL, int sv[2])
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
#if ISC_RSD
static void rsd_match_16b(int pu_w, int pu_h, void* dst, void* src, int s_src, u8 intra_row_rsd_flag)
{
    s16* s1 = (s16*)dst;
    s16* s2 = (s16*)src;
    if (intra_row_rsd_flag) // ver
    {
        s1 = (s16*)dst + pu_w * (pu_h - 1);
        for (int y = 0; y < pu_h; y++)
        {
            for (int x = 0; x < pu_w; x++)
            {
                s1[x] = (s16)s2[x];
            }
            s1 -= pu_w;
            s2 += s_src;
        }
    }
    else // hor
    {
        for (int y = 0; y < pu_h; y++)
        {
            for (int x = 0; x < pu_w; x++)
            {
                s1[x] = (s16)s2[pu_w - x - 1];
            }
            s1 += pu_w;
            s2 += s_src;
        }
    }
}
#endif
void check_best_ibc_mode(ENC_CORE *core, ENC_PIBC *pi, const double cost_curr, double *cost_best)
{
    COM_MODE *bst_info = &core->mod_info_best;
    COM_MODE *cur_info = &core->mod_info_curr;

    if (cost_curr < *cost_best)
    {
        int j, lidx;
        int cu_width_log2 = cur_info->cu_width_log2;
        int cu_height_log2 = cur_info->cu_height_log2;
        int cu_width = 1 << cu_width_log2;
        int cu_height = 1 << cu_height_log2;
        bst_info->cu_mode = cur_info->cu_mode;
        bst_info->skip_idx = cur_info->skip_idx;
        bst_info->ibc_flag = 1;
#if IST
#if ISTS
        bst_info->ist_tu_flag = cur_info->ist_tu_flag;
#else
        bst_info->ist_tu_flag = 0;
#endif
#endif
#if INTERPF
        bst_info->inter_filter_flag = 0;
#endif
#if IPC
        bst_info->ipc_flag = 0;
#endif

#if TB_SPLIT_EXT
        check_tb_part(cur_info);
        bst_info->pb_part = cur_info->pb_part;
        bst_info->tb_part = cur_info->tb_part;
        memcpy(&bst_info->pb_info, &cur_info->pb_info, sizeof(COM_PART_INFO));
        memcpy(&bst_info->tb_info, &cur_info->tb_info, sizeof(COM_PART_INFO));
#endif
#if SBT
#if !IST_SBT_IBC
        assert( cur_info->sbt_info == 0 );
#endif
        bst_info->sbt_info = cur_info->sbt_info;
#endif
        bst_info->umve_flag = 0;

        *cost_best = cost_curr;
        core->cost_best = cost_curr;

        SBAC_STORE(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_best);
        
#if EXT_AMVR_HMVP
        bst_info->mvp_from_hmvp_flag = 0;
#endif
#if IBC_ABVR
        bst_info->bvr_idx = pi->curr_bvr;
#endif
        bst_info->mvr_idx = 0;
#if IBC_BVP
        bst_info->cbvp_idx = cur_info->cbvp_idx;
        bst_info->cnt_hbvp_cands = cur_info->cnt_hbvp_cands;
#endif
#if IBC_ENH
        bst_info->ibcpf_idx = cur_info->ibcpf_idx;
#endif
#if ISC_RSD && !RSD_OPT
        bst_info->sp_rsd_flag = cur_info->sp_rsd_flag;
        bst_info->intra_row_rsd_flag = cur_info->intra_row_rsd_flag;
        if (cur_info->sp_rsd_flag != 0)
        {
            bst_info->string_copy_info[0].offset_x = cur_info->string_copy_info[0].offset_x;
            bst_info->string_copy_info[0].offset_y = cur_info->string_copy_info[0].offset_y;
            bst_info->string_copy_info[0].n_recent_flag = cur_info->string_copy_info[0].n_recent_flag;
            bst_info->string_copy_info[0].n_recent_idx = cur_info->string_copy_info[0].n_recent_idx;
        }
#endif
        bst_info->refi[REFP_0] = cur_info->refi[REFP_0];
        bst_info->refi[REFP_1] = cur_info->refi[REFP_1];
        for (lidx = 0; lidx < REFP_NUM; lidx++)
        {
            bst_info->mv[lidx][MV_X] = cur_info->mv[lidx][MV_X];
            bst_info->mv[lidx][MV_Y] = cur_info->mv[lidx][MV_Y];
            bst_info->mvd[lidx][MV_X] = cur_info->mvd[lidx][MV_X];
            bst_info->mvd[lidx][MV_Y] = cur_info->mvd[lidx][MV_Y];
        }
#if SMVD
        bst_info->smvd_flag = 0;
#endif

        bst_info->affine_flag = 0;
#if AFFINE_UMVE
        bst_info->affine_umve_flag = 0;
#endif
#if BGC
        bst_info->bgc_flag = 0;
        bst_info->bgc_idx  = 0;
#endif
#if AWP
        bst_info->awp_flag = 0;
#endif

#if AWP_MVR
        bst_info->awp_mvr_flag0 = 0;
        bst_info->awp_mvr_flag1 = 0;
#endif

#if INTER_TM
        bst_info->tm_flag = 0;
#endif

        for (j = 0; j < N_C; j++)
        {
            int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
            cu_plane_nz_cpy(bst_info->num_nz, cur_info->num_nz, j);
            com_mcpy(bst_info->pred[j], cur_info->pred[j], size_tmp * sizeof(pel));
            com_mcpy(bst_info->coef[j], cur_info->coef[j], size_tmp * sizeof(s16));
        }
    }
}

#if IST_SBT_IBC
static u8 search_ibc_tr_info(ENC_CORE *core, u16 cu_ssd, u8* tb_part_size, u8* sbt_info, u8* ist_tu_flag)
{
	COM_MODE *mod_info_curr = &core->mod_info_curr;
	*tb_part_size = 255;
	*sbt_info = 255;
	*ist_tu_flag = 255;
	ENC_BEF_DATA* p_data = &core->bef_data[mod_info_curr->cu_width_log2 - 2][mod_info_curr->cu_height_log2 - 2][core->cup];

	for (int idx = 0; idx < p_data->num_inter_pred; idx++)
	{
		if (p_data->inter_pred_dist[idx] == cu_ssd)
		{
			*tb_part_size = p_data->inter_tb_part[idx];
			*sbt_info = p_data->sbt_info_save[idx];
			*ist_tu_flag = p_data->ist_save[idx];
			return 1;
		}
	}
	return 0;
}

static void save_ibc_tr_info(ENC_CORE *core, u16 cu_ssd, u8 tb_part_size, u8 sbt_info, u8 ist_tu_flag)
{
	COM_MODE *mod_info_curr = &core->mod_info_curr;
	ENC_BEF_DATA* p_data = &core->bef_data[mod_info_curr->cu_width_log2 - 2][mod_info_curr->cu_height_log2 - 2][core->cup];
	if (p_data->num_inter_pred == NUM_SL_INTER)
		return;

	p_data->inter_pred_dist[p_data->num_inter_pred] = cu_ssd;
	p_data->inter_tb_part[p_data->num_inter_pred] = tb_part_size;
	p_data->sbt_info_save[p_data->num_inter_pred] = sbt_info;
	p_data->ist_save[p_data->num_inter_pred] = ist_tu_flag;
	p_data->num_inter_pred++;
}
#endif

#if RSD_OPT
static double prsd_residue_rdo(ENC_CTX* ctx, ENC_CORE* core, int bForceAllZero)
{
    COM_MODE* mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    s16(*mv)[MV_D] = mod_info_curr->mv;
    s8* refi = mod_info_curr->refi;
    ENC_PIBC* pi = &ctx->pibc;
    pel(*pred)[MAX_CU_DIM] = mod_info_curr->pred;
    int bit_depth = ctx->info.bit_depth_internal;
    int(*num_nz_coef)[N_C], tnnz, width[N_C], height[N_C], log2_w[N_C], log2_h[N_C];
    pel(*rec)[MAX_CU_DIM];
    s64    dist[2][N_C], dist_pred[N_C];
    double cost, cost_best = MAX_COST;
    int    cbf_best[N_C], nnz_store[MAX_NUM_TB][N_C], tb_part_store;
    int    bit_cnt;
    int    i, j, cbf_y, cbf_u, cbf_v;
    pel* org[N_C];
    double cost_comp_best = MAX_COST;
    int    cbf_comps[N_C] = { 0, };
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if ST_CHROMA
    int use_secTrans[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 } }; // secondary transform is disabled for ibc coding
    int use_alt4x4Trans[MAX_NUM_CHANNEL] = { 0, 0 };
#else
    int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for ibc coding
    int use_alt4x4Trans = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif

#if RDO_DBK
    u8  is_from_mv_field = 0;
#endif
    rec = mod_info_curr->rec;
    num_nz_coef = mod_info_curr->num_nz;
    width[Y_C] = 1 << cu_width_log2;
    height[Y_C] = 1 << cu_height_log2;
    width[U_C] = width[V_C] = 1 << (cu_width_log2 - 1);
    height[U_C] = height[V_C] = 1 << (cu_height_log2 - 1);
    log2_w[Y_C] = cu_width_log2;
    log2_h[Y_C] = cu_height_log2;
    log2_w[U_C] = log2_w[V_C] = cu_width_log2 - 1;
    log2_h[U_C] = log2_h[V_C] = cu_height_log2 - 1;
    org[Y_C] = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
    org[U_C] = pi->Yuv_org[U_C] + ((y >> 1) * pi->stride_org[U_C]) + (x >> 1);
    org[V_C] = pi->Yuv_org[V_C] + ((y >> 1) * pi->stride_org[V_C]) + (x >> 1);

    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
    COM_SP_INFO derived_rsd_strinfo[2];
    derived_rsd_strinfo[0].length = 4;
    derived_rsd_strinfo[0].offset_x = mv[0][MV_X] >> 2;
    derived_rsd_strinfo[0].offset_y = mv[0][MV_Y] >> 2;
    derived_rsd_strinfo[0].sp_rsd_flag = 1;
    derived_rsd_strinfo[0].intra_row_rsd_flag = mod_info_curr->intra_row_rsd_flag;
    derived_rsd_strinfo[1].length = (1 << cu_width_log2) * (1 << cu_height_log2) - 4;
    derived_rsd_strinfo[1].offset_x = mv[0][MV_X] >> 2;
    derived_rsd_strinfo[1].offset_y = mv[0][MV_Y] >> 2;
    derived_rsd_strinfo[1].sp_rsd_flag = 1;
    derived_rsd_strinfo[1].intra_row_rsd_flag = mod_info_curr->intra_row_rsd_flag;
    rsd_recon_yuv(mod_info_curr, x, y, cu_width_log2, cu_height_log2, pi->pic_unfiltered_rec, ctx->tree_status
        , ctx->info.pic_width, ctx->info.pic_height, ctx->info.pic_width_in_scu, ctx->map.map_scu
        , ctx->info.log2_max_cuwh, derived_rsd_strinfo, 2);

    /* get residual */
    enc_diff_pred(x, y, cu_width_log2, cu_height_log2, pi->pic_org, pred, resi_t);

    for (i = 0; i < N_C; i++)
    {
        dist[0][i] = dist_pred[i] = enc_ssd_16b(log2_w[i], log2_h[i], pred[i], org[i], width[i], pi->stride_org[i], bit_depth);
    }
#if RDO_DBK
    calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pred, cu_width, x, y, 0, 0, refi, mv, is_from_mv_field);
    dist[0][Y_C] += ctx->delta_dist;
#endif

    /* test all zero case */
    memset(cbf_best, 0, sizeof(int) * N_C);
    memset(num_nz_coef, 0, sizeof(int) * MAX_NUM_TB * N_C);
    mod_info_curr->tb_part = SIZE_2Nx2N;
    if (ctx->tree_status == TREE_LC)
    {
        cost_best = (double)dist[0][Y_C] + ((dist[0][U_C] + dist[0][V_C]) * ctx->dist_chroma_weight[0]);
    }
    else
    {
        assert(ctx->tree_status == TREE_L);
        cost_best = (double)dist[0][Y_C];
    }

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_rsd(ctx, core);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);

    /* transform and quantization */
    bForceAllZero |= 0;

    if (!bForceAllZero)
    {
#if TR_EARLY_TERMINATE
        core->dist_pred_luma = dist_pred[Y_C];
#endif

#if USE_IBC //support N*N TB
        core->best_tb_part_hist = 255; //enable this line to bypass the save load mechanism
#endif

        tnnz = enc_tq_yuv_nnz(ctx, core, mod_info_curr, coef, resi_t, pi->slice_type, 0, use_secTrans, use_alt4x4Trans, refi, mv, is_from_mv_field);

        if (tnnz)
        {
            com_itdq_yuv(mod_info_curr, coef, resi_t, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans
#if IST_CHROMA
                , ctx->tree_status
#endif  
            );
            for (i = 0; i < N_C; i++)
            {
                com_recon(i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N, resi_t[i], pred[i], num_nz_coef, i, width[i], height[i], width[i], rec[i], bit_depth
#if SBT
                    , 0
#endif
                );

                if (is_cu_plane_nz(num_nz_coef, i))
                {
                    dist[1][i] = enc_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], width[i], pi->stride_org[i], bit_depth);
                }
                else
                {
                    dist[1][i] = dist_pred[i];
                }
            }

#if RDO_DBK
            //filter rec and calculate ssd
            calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, rec, cu_width, x, y, 0, 1, refi, mv, is_from_mv_field);
            dist[1][Y_C] += ctx->delta_dist;
#endif

            cbf_y = is_cu_plane_nz(num_nz_coef, Y_C) > 0 ? 1 : 0;
            cbf_u = is_cu_plane_nz(num_nz_coef, U_C) > 0 ? 1 : 0;
            cbf_v = is_cu_plane_nz(num_nz_coef, V_C) > 0 ? 1 : 0;

            if (ctx->tree_status == TREE_LC)
            {
                cost = (double)dist[cbf_y][Y_C] + ((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);
            }
            else
            {
                assert(ctx->tree_status == TREE_L);
                cost = (double)dist[cbf_y][Y_C];
            }

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            //enc_sbac_bit_reset(&core->s_temp_run);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_rsd(ctx, core);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
            if (cost < cost_best)
            {
                cost_best = cost;
                cbf_best[Y_C] = cbf_y;
                cbf_best[U_C] = cbf_u;
                cbf_best[V_C] = cbf_v;
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
            }
            SBAC_LOAD(core->s_temp_prev_comp_best, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);

            //    /* cbf test for each component */
            com_mcpy(nnz_store, num_nz_coef, sizeof(int) * MAX_NUM_TB * N_C);
            tb_part_store = mod_info_curr->tb_part;
            if (ctx->tree_status == TREE_LC) // do not need to test for luma only
            {
                for (i = 0; i < N_C; i++)
                {
                    if (is_cu_plane_nz(nnz_store, i) > 0)
                    {
                        cost_comp_best = MAX_COST;
                        SBAC_LOAD(core->s_temp_prev_comp_run, core->s_temp_prev_comp_best);
                        for (j = 0; j < 2; j++)
                        {
                            cost = dist[j][i] * (i == 0 ? 1 : ctx->dist_chroma_weight[i - 1]);
                            if (j)
                            {
                                cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                                if (i == 0)
                                    mod_info_curr->tb_part = tb_part_store;
                            }
                            else
                            {
                                cu_plane_nz_cln(num_nz_coef, i);
                                if (i == 0)
                                    mod_info_curr->tb_part = SIZE_2Nx2N;
                            }

                            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_run);
                            //enc_sbac_bit_reset(&core->s_temp_run);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run);
                            enc_bit_est_inter_comp(ctx, core, coef[i], i);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                            cost += RATE_TO_COST_LAMBDA(ctx->lambda[i], bit_cnt);
                            if (cost < cost_comp_best)
                            {
                                cost_comp_best = cost;
                                cbf_comps[i] = j;
                                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                            }
                        }
                    }
                    else
                    {
                        cbf_comps[i] = 0;
                    }
                }

                // do not set if all zero, because the case of all zero has been tested
                if (cbf_comps[Y_C] != 0 || cbf_comps[U_C] != 0 || cbf_comps[V_C] != 0)
                {
                    for (i = 0; i < N_C; i++)
                    {
                        if (cbf_comps[i])
                        {
                            cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                            if (i == 0)
                                mod_info_curr->tb_part = tb_part_store;
                        }
                        else
                        {
                            cu_plane_nz_cln(num_nz_coef, i);
                            if (i == 0)
                                mod_info_curr->tb_part = SIZE_2Nx2N;
                        }
                    }

                    // if the best num_nz_coef is changed
                    if (!is_cu_nz_equ(num_nz_coef, nnz_store))
                    {
                        cbf_y = cbf_comps[Y_C];
                        cbf_u = cbf_comps[U_C];
                        cbf_v = cbf_comps[V_C];
                        cost = dist[cbf_y][Y_C] + ((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);

                        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                        //enc_sbac_bit_reset(&core->s_temp_run);
                        bit_cnt = enc_get_bit_number(&core->s_temp_run);
                        enc_bit_est_rsd(ctx, core);
                        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                        if (cost < cost_best)
                        {
                            cost_best = cost;
                            cbf_best[Y_C] = cbf_y;
                            cbf_best[U_C] = cbf_u;
                            cbf_best[V_C] = cbf_v;
                            SBAC_STORE(core->s_temp_best, core->s_temp_run);
                        }
                    }
                }
            }
            for (i = 0; i < N_C; i++)
            {
                if (cbf_best[i])
                {
                    cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                    if (i == 0)
                        mod_info_curr->tb_part = tb_part_store;
                }
                else
                {
                    cu_plane_nz_cln(num_nz_coef, i);
                    if (i == 0)
                        mod_info_curr->tb_part = SIZE_2Nx2N;
                }

                if (is_cu_plane_nz(num_nz_coef, i) == 0 && is_cu_plane_nz(nnz_store, i) != 0)
                {
                    com_mset(coef[i], 0, sizeof(s16) * ((cu_width * cu_height) >> (i == 0 ? 0 : 2)));
                }
            }
        }
    }

    if (!is_cu_plane_nz(num_nz_coef, Y_C))
        mod_info_curr->tb_part = SIZE_2Nx2N; // reset best tb_part if no residual Y

    return cost_best;
}

void check_best_rsd_mode(ENC_CORE* core, ENC_PIBC* pi, const double cost_curr, double* cost_best)
{
    COM_MODE* bst_info = &core->mod_info_best;
    COM_MODE* cur_info = &core->mod_info_curr;
    int j, lidx;
    int cu_width_log2 = cur_info->cu_width_log2;
    int cu_height_log2 = cur_info->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    bst_info->ibc_flag = 0;
    bst_info->skip_idx = 0;
    bst_info->sp_flag = 1;
    bst_info->cs2_flag = 0;
    bst_info->sub_string_no = 1;
    bst_info->string_copy_info[0].is_matched = 0;
    bst_info->string_copy_info[0].sp_rsd_flag = bst_info->sp_rsd_flag = 1;
    bst_info->string_copy_info[0].intra_row_rsd_flag = bst_info->intra_row_rsd_flag = cur_info->intra_row_rsd_flag;
    bst_info->string_copy_info[0].offset_x = cur_info->string_copy_info[0].offset_x;
    bst_info->string_copy_info[0].offset_y = cur_info->string_copy_info[0].offset_y;
    bst_info->string_copy_info[0].n_recent_flag = cur_info->string_copy_info[0].n_recent_flag;
    bst_info->string_copy_info[0].n_recent_idx = cur_info->string_copy_info[0].n_recent_idx;
    bst_info->cu_mode = cur_info->cu_mode;
#if IST
#if ISTS
    bst_info->ist_tu_flag = cur_info->ist_tu_flag;
#else
    bst_info->ist_tu_flag = 0;
#endif
#endif
#if INTERPF
    bst_info->inter_filter_flag = 0;
#endif
#if IPC
    bst_info->ipc_flag = 0;
#endif
#if TB_SPLIT_EXT
    check_tb_part(cur_info);
    bst_info->pb_part = cur_info->pb_part;
    bst_info->tb_part = cur_info->tb_part;
    memcpy(&bst_info->pb_info, &cur_info->pb_info, sizeof(COM_PART_INFO));
    memcpy(&bst_info->tb_info, &cur_info->tb_info, sizeof(COM_PART_INFO));
#endif
#if SBT
    assert(cur_info->sbt_info == 0);
    bst_info->sbt_info = cur_info->sbt_info;
#endif
    bst_info->umve_flag = 0;
    *cost_best = cost_curr;
    core->cost_best = cost_curr;
    SBAC_STORE(core->s_next_best[cu_width_log2 - 2][cu_height_log2 - 2], core->s_temp_best);
#if EXT_AMVR_HMVP
    bst_info->mvp_from_hmvp_flag = 0;
#endif
#if IBC_ABVR
    bst_info->bvr_idx = 0;
#endif
    bst_info->mvr_idx = 0;
#if IBC_BVP
    bst_info->cbvp_idx = 0;
    bst_info->cnt_hbvp_cands = 0;
#endif
    bst_info->refi[REFP_0] = cur_info->refi[REFP_0];
    bst_info->refi[REFP_1] = cur_info->refi[REFP_1];
    for (lidx = 0; lidx < REFP_NUM; lidx++)
    {
        bst_info->mv[lidx][MV_X] = cur_info->mv[lidx][MV_X];
        bst_info->mv[lidx][MV_Y] = cur_info->mv[lidx][MV_Y];
        bst_info->mvd[lidx][MV_X] = cur_info->mvd[lidx][MV_X];
        bst_info->mvd[lidx][MV_Y] = cur_info->mvd[lidx][MV_Y];
    }
#if SMVD
    bst_info->smvd_flag = 0;
#endif
    bst_info->affine_flag = 0;
#if AFFINE_UMVE
    bst_info->affine_umve_flag = 0;
#endif
#if BGC
    bst_info->bgc_flag = 0;
    bst_info->bgc_idx = 0;
#endif
#if AWP
    bst_info->awp_flag = 0;
#endif
#if AWP_MVR
    bst_info->awp_mvr_flag0 = 0;
    bst_info->awp_mvr_flag1 = 0;
#endif
#if INTER_TM
    bst_info->tm_flag = 0;
#endif
    for (j = 0; j < N_C; j++)
    {
        int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
        cu_plane_nz_cpy(bst_info->num_nz, cur_info->num_nz, j);
        com_mcpy(bst_info->pred[j], cur_info->pred[j], size_tmp * sizeof(pel));
        com_mcpy(bst_info->coef[j], cur_info->coef[j], size_tmp * sizeof(s16));
    }
}
#endif
static double pibc_residue_rdo(ENC_CTX *ctx, ENC_CORE *core, int bForceAllZero)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    s16(*mv)[MV_D] = mod_info_curr->mv;
    s8 *refi = mod_info_curr->refi;
    ENC_PIBC *pi = &ctx->pibc;
    pel(*pred)[MAX_CU_DIM] = mod_info_curr->pred;
    int bit_depth = ctx->info.bit_depth_internal;
    int(*num_nz_coef)[N_C], tnnz, width[N_C], height[N_C], log2_w[N_C], log2_h[N_C];
    pel(*rec)[MAX_CU_DIM];
    s64    dist[2][N_C], dist_pred[N_C];
    double cost, cost_best = MAX_COST;
    int    cbf_best[N_C], nnz_store[MAX_NUM_TB][N_C], tb_part_store;
    int    bit_cnt;
    int    i, j, cbf_y, cbf_u, cbf_v;
    pel   *org[N_C];
    double cost_comp_best = MAX_COST;
    int    cbf_comps[N_C] = { 0, };
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if IST_SBT_IBC
	int coef_restrict = 0;
	int run_comp[3] = { 1, ctx->tree_status == TREE_LC, ctx->tree_status == TREE_LC };
#endif
#if ST_CHROMA
    int use_secTrans[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 } }; // secondary transform is disabled for ibc coding
    int use_alt4x4Trans[MAX_NUM_CHANNEL] = { 0, 0 };
#else
    int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for ibc coding
    int use_alt4x4Trans = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if DEST_PH
    mod_info_curr->ph_dest_enable_flag = ctx->info.pic_header.ph_dest_enable_flag;
#endif

#if RDO_DBK
    u8  is_from_mv_field = 0;
#endif

#if !BD_AFFINE_AMVR
    if (mod_info_curr->affine_flag)
    {
        pi->curr_mvr = 0;
    }
#endif

    rec = mod_info_curr->rec;
    num_nz_coef = mod_info_curr->num_nz;
    width[Y_C] = 1 << cu_width_log2;
    height[Y_C] = 1 << cu_height_log2;
    width[U_C] = width[V_C] = 1 << (cu_width_log2 - 1);
    height[U_C] = height[V_C] = 1 << (cu_height_log2 - 1);
    log2_w[Y_C] = cu_width_log2;
    log2_h[Y_C] = cu_height_log2;
    log2_w[U_C] = log2_w[V_C] = cu_width_log2 - 1;
    log2_h[U_C] = log2_h[V_C] = cu_height_log2 - 1;
    org[Y_C] = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
    org[U_C] = pi->Yuv_org[U_C] + ((y >> 1) * pi->stride_org[U_C]) + (x >> 1);
    org[V_C] = pi->Yuv_org[V_C] + ((y >> 1) * pi->stride_org[V_C]) + (x >> 1);

    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
#if ISC_RSD && !RSD_OPT
    if (mod_info_curr->sp_rsd_flag != 0)
    {
        COM_SP_INFO derived_rsd_str_info[2];
        derived_rsd_str_info[0].length = 4;
        derived_rsd_str_info[0].offset_x = mv[0][MV_X] >> 2;
        derived_rsd_str_info[0].offset_y = mv[0][MV_Y] >> 2;
        derived_rsd_str_info[0].sp_rsd_flag = 1;
        derived_rsd_str_info[0].intra_row_rsd_flag = mod_info_curr->intra_row_rsd_flag;
        derived_rsd_str_info[1].length = (1 << cu_width_log2) * (1 << cu_height_log2) - 4;
        derived_rsd_str_info[1].offset_x = mv[0][MV_X] >> 2;
        derived_rsd_str_info[1].offset_y = mv[0][MV_Y] >> 2;
        derived_rsd_str_info[1].sp_rsd_flag = 1;
        derived_rsd_str_info[1].intra_row_rsd_flag = mod_info_curr->intra_row_rsd_flag;

        rsd_recon_yuv(mod_info_curr, x, y, cu_width_log2, cu_height_log2, pi->pic_unfiltered_rec, ctx->tree_status

            , ctx->info.pic_width, ctx->info.pic_height, ctx->info.pic_width_in_scu, ctx->map.map_scu

            , ctx->info.log2_max_cuwh, derived_rsd_str_info, 2);
    }
    else
    {
        com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mv[0], pi->pic_unfiltered_rec, pred, ctx->tree_status, bit_depth);
    }
#else
    com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mv[0], pi->pic_unfiltered_rec, pred, ctx->tree_status, bit_depth);
#endif
#if IBC_ENH
    if (mod_info_curr->ibcpf_idx)
    {
#if IBC_APF
        if (ctx->info.pic_header.ibc_type == 0)
        {
            pred_ibc_filter(PIC_REC(ctx)/*, PIC_ORG(ctx)*/, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, mod_info_curr
                , x, y, cu_width, cu_height, bit_depth, ctx->tree_status, mod_info_curr->ibcpf_idx);
        }
        else
        {
#endif
            pred_inter_filter(PIC_REC(ctx), ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, mod_info_curr
                , x, y, cu_width, cu_height, bit_depth, ctx->tree_status, mod_info_curr->ibcpf_idx);
#if IBC_APF
        }
#endif
    }
#endif
    /* get residual */
    enc_diff_pred(x, y, cu_width_log2, cu_height_log2, pi->pic_org, pred, resi_t);

    for (i = 0; i < N_C; i++)
    {
#if IST_SBT_IBC
		if (!run_comp[i])
			dist[0][i] = dist_pred[i] = 0;
#endif
        dist[0][i] = dist_pred[i] = enc_ssd_16b(log2_w[i], log2_h[i], pred[i], org[i], width[i], pi->stride_org[i], bit_depth);
    }
#if IST_SBT_IBC
	if (ctx->info.pic_header.ibc_type == 1)
	{
		core->dist_no_resi[Y_C] = dist[0][Y_C];
		core->dist_no_resi[U_C] = dist[0][U_C];
		core->dist_no_resi[V_C] = dist[0][V_C];
	}
#endif
#if RDO_DBK
    calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pred, cu_width, x, y, 0, 0, refi, mv, is_from_mv_field);
    dist[0][Y_C] += ctx->delta_dist;
#endif

    /* test all zero case */
    memset(cbf_best, 0, sizeof(int) * N_C);
    memset(num_nz_coef, 0, sizeof(int) * MAX_NUM_TB * N_C);
    mod_info_curr->tb_part = SIZE_2Nx2N;
    if (ctx->tree_status == TREE_LC)
    {
        cost_best = (double)dist[0][Y_C] + ((dist[0][U_C] + dist[0][V_C]) * ctx->dist_chroma_weight[0]);
    }
    else
    {
        assert(ctx->tree_status == TREE_L);
        cost_best = (double)dist[0][Y_C];
    }

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
#if IBC_ENH
    enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1, 1);
#else
    enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1);
#endif
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
#if IST_SBT_IBC
	if (ctx->info.pic_header.ibc_type == 1)
	{
		core->cost_best = min(cost_best, core->cost_best);
	}
#endif

    /* transform and quantization */
    bForceAllZero |= 0;

    if (!bForceAllZero)
    {
#if TR_EARLY_TERMINATE
        core->dist_pred_luma = dist_pred[Y_C];
#endif
#if IST_SBT_IBC
		s64 cu_ssd_s64 = dist_pred[Y_C] + dist_pred[U_C] + dist_pred[V_C];
		u16 cu_ssd_u16 = 0;
		int try_mtr = 0;

		core->best_sbt_info_hist = 255;
		core->best_ist_hist = 255;
#endif

#if USE_IBC //support N*N TB
        core->best_tb_part_hist = 255; //enable this line to bypass the save load mechanism
#endif

#if IST_SBT_IBC
		if (com_sbt_allow(mod_info_curr, ctx->info.sqh.sbt_enable_flag, ctx->tree_status, &(ctx->info))
			|| (ctx->info.pic_header.ibc_type == 1 && ctx->info.sqh.ist_enable_flag && !mod_info_curr->ph_ists_enable_flag && ctx->tree_status != TREE_C && cu_width_log2 < 6 && cu_height_log2 < 6))
		{
			int shift_val = min(cu_width_log2 + cu_height_log2, 9);
			cu_ssd_u16 = (u16)(cu_ssd_s64 + (s64)(1 << (shift_val - 1))) >> shift_val;
			search_ibc_tr_info(core, cu_ssd_u16, &core->best_tb_part_hist, &core->best_sbt_info_hist, &core->best_ist_hist);
			try_mtr = 1;
			mod_info_curr->ist_tu_flag = 0;
		}
#endif

        tnnz = enc_tq_yuv_nnz(ctx, core, mod_info_curr, coef, resi_t, pi->slice_type, 0, use_secTrans, use_alt4x4Trans, refi, mv, is_from_mv_field);

#if IST_SBT_IBC
		if (mod_info_curr->tb_part == SIZE_2Nx2N && mod_info_curr->ist_tu_flag && ctx->info.pic_header.ibc_type == 1 && !ctx->info.pic_header.ph_ists_enable_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
		{
			for (int y = 0; y < cu_height; y++)
			{
				for (int x = 0; x < cu_width; x++)
				{
					if ((x >= IST_MAX_COEF_SIZE || y >= IST_MAX_COEF_SIZE) && coef[Y_C][y * cu_width + x])
					{
						coef_restrict = 1;
					}
				}
			}
		}
#endif

#if IST_SBT_IBC
		if ((tnnz && !mod_info_curr->sbt_info) || (mod_info_curr->num_nz[TB0][Y_C] && mod_info_curr->sbt_info))
#else
		if (tnnz)
#endif
        {
            com_itdq_yuv(mod_info_curr, coef, resi_t, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans
#if IST_CHROMA
                , ctx->tree_status
#endif            
            );
            for (i = 0; i < N_C; i++)
            {
                com_recon(i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N, resi_t[i], pred[i], num_nz_coef, i, width[i], height[i], width[i], rec[i], bit_depth
#if SBT
#if IST_SBT_IBC
					, mod_info_curr->sbt_info
#else
                    , 0
#endif
#endif
                );

                if (is_cu_plane_nz(num_nz_coef, i))
                {
                    dist[1][i] = enc_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], width[i], pi->stride_org[i], bit_depth);
                }
                else
                {
                    dist[1][i] = dist_pred[i];
                }
            }

#if RDO_DBK
            //filter rec and calculate ssd
            calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, rec, cu_width, x, y, 0, 1, refi, mv, is_from_mv_field);
            dist[1][Y_C] += ctx->delta_dist;
#endif

            cbf_y = is_cu_plane_nz(num_nz_coef, Y_C) > 0 ? 1 : 0;
            cbf_u = is_cu_plane_nz(num_nz_coef, U_C) > 0 ? 1 : 0;
            cbf_v = is_cu_plane_nz(num_nz_coef, V_C) > 0 ? 1 : 0;

            if (ctx->tree_status == TREE_LC)
            {
                cost = (double)dist[cbf_y][Y_C] + ((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);
            }
            else
            {
                assert(ctx->tree_status == TREE_L);
                cost = (double)dist[cbf_y][Y_C];
            }

            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            //enc_sbac_bit_reset(&core->s_temp_run);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
#if IBC_ENH
            enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1, 1);
#else
            enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1);
#endif
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
            if (cost < cost_best)
            {
                cost_best = cost;
                cbf_best[Y_C] = cbf_y;
                cbf_best[U_C] = cbf_u;
                cbf_best[V_C] = cbf_v;
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
            }
            SBAC_LOAD(core->s_temp_prev_comp_best, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);

            //    /* cbf test for each component */
            com_mcpy(nnz_store, num_nz_coef, sizeof(int) * MAX_NUM_TB * N_C);
            tb_part_store = mod_info_curr->tb_part;
            if (ctx->tree_status == TREE_LC) // do not need to test for luma only
            {
                for (i = 0; i < N_C; i++)
                {
                    if (is_cu_plane_nz(nnz_store, i) > 0)
                    {
#if IST_SBT_IBC
						if (mod_info_curr->sbt_info && i == 0)
						{
							cbf_comps[i] = 1;
							continue;
						}
#endif
                        cost_comp_best = MAX_COST;
                        SBAC_LOAD(core->s_temp_prev_comp_run, core->s_temp_prev_comp_best);
                        for (j = 0; j < 2; j++)
                        {
                            cost = dist[j][i] * (i == 0 ? 1 : ctx->dist_chroma_weight[i - 1]);
                            if (j)
                            {
                                cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                                if (i == 0)
                                    mod_info_curr->tb_part = tb_part_store;
                            }
                            else
                            {
                                cu_plane_nz_cln(num_nz_coef, i);
                                if (i == 0)
                                    mod_info_curr->tb_part = SIZE_2Nx2N;
                            }

                            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_run);
                            //enc_sbac_bit_reset(&core->s_temp_run);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run);
                            enc_bit_est_inter_comp(ctx, core, coef[i], i);
                            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                            cost += RATE_TO_COST_LAMBDA(ctx->lambda[i], bit_cnt);
                            if (cost < cost_comp_best)
                            {
                                cost_comp_best = cost;
                                cbf_comps[i] = j;
                                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                            }
                        }
                    }
                    else
                    {
                        cbf_comps[i] = 0;
                    }
                }

                // do not set if all zero, because the case of all zero has been tested
                if (cbf_comps[Y_C] != 0 || cbf_comps[U_C] != 0 || cbf_comps[V_C] != 0)
                {
                    for (i = 0; i < N_C; i++)
                    {
                        if (cbf_comps[i])
                        {
                            cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                            if (i == 0)
                                mod_info_curr->tb_part = tb_part_store;
                        }
                        else
                        {
                            cu_plane_nz_cln(num_nz_coef, i);
                            if (i == 0)
                                mod_info_curr->tb_part = SIZE_2Nx2N;
                        }
                    }

                    // if the best num_nz_coef is changed
                    if (!is_cu_nz_equ(num_nz_coef, nnz_store))
                    {
                        cbf_y = cbf_comps[Y_C];
                        cbf_u = cbf_comps[U_C];
                        cbf_v = cbf_comps[V_C];
                        cost = dist[cbf_y][Y_C] + ((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);

                        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                        //enc_sbac_bit_reset(&core->s_temp_run);
                        bit_cnt = enc_get_bit_number(&core->s_temp_run);
#if IBC_ENH
                        enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1, 1);
#else
                        enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1);
#endif
                        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                        if (cost < cost_best)
                        {
                            cost_best = cost;
                            cbf_best[Y_C] = cbf_y;
                            cbf_best[U_C] = cbf_u;
                            cbf_best[V_C] = cbf_v;
                            SBAC_STORE(core->s_temp_best, core->s_temp_run);
                        }
                    }
                }
            }
            for (i = 0; i < N_C; i++)
            {
                if (cbf_best[i])
                {
                    cu_plane_nz_cpy(num_nz_coef, nnz_store, i);
                    if (i == 0)
                        mod_info_curr->tb_part = tb_part_store;
                }
                else
                {
                    cu_plane_nz_cln(num_nz_coef, i);
                    if (i == 0)
                        mod_info_curr->tb_part = SIZE_2Nx2N;
                }

                if (is_cu_plane_nz(num_nz_coef, i) == 0 && is_cu_plane_nz(nnz_store, i) != 0)
                {
                    com_mset(coef[i], 0, sizeof(s16) * ((cu_width * cu_height) >> (i == 0 ? 0 : 2)));
                }
            }
#if IST_SBT_IBC
			if (!is_cu_plane_nz(num_nz_coef, Y_C))
				mod_info_curr->sbt_info = 0;
#endif
        }
#if IST_SBT_IBC
		if (core->best_tb_part_hist == 255 && mod_info_curr->pb_part == SIZE_2Nx2N && try_mtr)
		{
			save_ibc_tr_info(core, cu_ssd_u16, (u8)mod_info_curr->tb_part, mod_info_curr->sbt_info, mod_info_curr->ist_tu_flag);
		}
#endif
    }

    if (!is_cu_plane_nz(num_nz_coef, Y_C))
        mod_info_curr->tb_part = SIZE_2Nx2N; // reset best tb_part if no residual Y
#if IST_SBT_IBC
	if (coef_restrict)
		cost = MAX_COST;
	if (!is_cu_plane_nz(num_nz_coef, Y_C))
		assert(mod_info_curr->sbt_info == 0);
	if (ctx->info.pic_header.ibc_type == 1)
		core->cost_best = min(cost_best, core->cost_best);
#endif

    return cost_best;
}

static void clip_ibc_mv(int rc_mv[2], int pic_width, int pic_height, int lcu_width, int lcu_height, int cu_pos_x, int cu_pos_y)
{
    int mv_shift = MV_FRACTIONAL_BITS_INTERNAL;
    int offset = 8;
    int hor_max = (pic_width + offset - cu_pos_x - 1) << mv_shift;
    int hor_min = (-lcu_width - offset - cu_pos_x + 1) << mv_shift;

    int ver_max = (pic_height + offset - cu_pos_y - 1) << mv_shift;
    int ver_min = (-lcu_height - offset - cu_pos_y + 1) << mv_shift;

    rc_mv[0] = COM_MIN(hor_max, COM_MAX(hor_min, rc_mv[0]));
    rc_mv[1] = COM_MIN(ver_max, COM_MAX(ver_min, rc_mv[1]));
}

static void ibc_set_search_range(ENC_CTX *ctx, ENC_CORE *core, int cu_pel_x, int cu_pel_y, int log2_cuw, int log2_cuh,
    const int local_search_range_x, const int local_search_range_y, int mv_search_range_left[2], int mv_search_range_right[2]
#if IBC_ME_OPT
    , int use_local_range
#endif
)
{
    int search_left = 0;
    int search_right = 0;
    int search_top = 0;
    int search_bottom = 0;

    const int roi_width = (1 << log2_cuw);
    const int roi_height = (1 << log2_cuh);

    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;

#if IBC_ME_OPT
    if (use_local_range)
    {
#endif
        search_left = -COM_MIN(cu_pel_x, local_search_range_x);
        search_top = -COM_MIN(cu_pel_y, local_search_range_y);
        search_right = COM_MIN(pic_width - cu_pel_x - roi_width, local_search_range_x);
        search_bottom = COM_MIN(pic_height - cu_pel_y - roi_height, local_search_range_y);
#if IBC_ME_OPT
    }
    else
    {
        int max_lcu_num = IBC_REF_RANGE_XLCUR;
#if SCC_ENC_OPT
        int lcu_y = (core->mod_info_curr.y_pos / MAX_CU_SIZE2) * MAX_CU_SIZE2;
        int ref_top = COM_MAX(lcu_y - (max_lcu_num - 1) * MAX_CU_SIZE2, 0);
        int ref_bottom = COM_MIN((lcu_y + MAX_CU_SIZE2), pic_height);
#else
        int lcu_y = (core->mod_info_curr.y_pos / MAX_CU_SIZE) * MAX_CU_SIZE;
        int ref_top = COM_MAX(lcu_y - (max_lcu_num - 1) * MAX_CU_SIZE, 0);
        int ref_bottom = COM_MIN((lcu_y + MAX_CU_SIZE), pic_height);
#endif
        search_left = -cu_pel_x;
        search_top = ref_top - cu_pel_y;
        search_right = pic_width - cu_pel_x - roi_width;
        search_bottom = ref_bottom - cu_pel_y  - roi_height;
    }
#endif

    mv_search_range_left[0] = search_left;
    mv_search_range_left[1] = search_top;
    mv_search_range_right[0] = search_right;
    mv_search_range_right[1] = search_bottom;

    clip_ibc_mv(mv_search_range_left, pic_width, pic_height, ctx->info.max_cuwh, ctx->info.max_cuwh,
        cu_pel_x, cu_pel_y);
    clip_ibc_mv(mv_search_range_right, pic_width, pic_height, ctx->info.max_cuwh, ctx->info.max_cuwh,
        cu_pel_x, cu_pel_y);
}
#if RSD_OPT
static void rsd_set_search_range(ENC_CTX* ctx, ENC_CORE* core, int cu_pel_x, int cu_pel_y, int log2_cuw, int log2_cuh, int mv_search_range_left[2], int mv_search_range_right[2]
)
{
    int search_left = 0;
    int search_right = 0;
    int search_top = 0;
    int search_bottom = 0;
    const int roi_width = (1 << log2_cuw);
    const int roi_height = (1 << log2_cuh);
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    int max_lcu_num = IBC_REF_RANGE_XLCUR;
    int lcu_y = (core->mod_info_curr.y_pos / MAX_CU_SIZE2) * MAX_CU_SIZE2;
    int ref_top = COM_MAX(lcu_y - (max_lcu_num - 1) * MAX_CU_SIZE2, 0);
    int ref_bottom = COM_MIN((lcu_y + MAX_CU_SIZE2), pic_height);

    search_left = -cu_pel_x;
    search_top = ref_top - cu_pel_y;
    search_right = pic_width - cu_pel_x - roi_width;
    search_bottom = ref_bottom - cu_pel_y - roi_height;
    mv_search_range_left[0] = search_left;
    mv_search_range_left[1] = search_top;
    mv_search_range_right[0] = search_right;
    mv_search_range_right[1] = search_bottom;

    clip_ibc_mv(mv_search_range_left, pic_width, pic_height, ctx->info.max_cuwh, ctx->info.max_cuwh,
        cu_pel_x, cu_pel_y);
    clip_ibc_mv(mv_search_range_right, pic_width, pic_height, ctx->info.max_cuwh, ctx->info.max_cuwh,
        cu_pel_x, cu_pel_y);
}
#endif
static void init_ibc_data(ENC_PIBC *pi, ENC_CORE *core)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    mod_info_curr->skip_idx = 0;
#if SMVD
    mod_info_curr->smvd_flag = 0;
#endif
    get_part_info(pi->pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, mod_info_curr->pb_part, &mod_info_curr->pb_info);
    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);

    com_mset(mod_info_curr->mv, 0, sizeof(s16) * REFP_NUM * MV_D);
    com_mset(mod_info_curr->mvd, 0, sizeof(s16) * REFP_NUM * MV_D);
    com_mset(mod_info_curr->refi, 0, sizeof(s8)  * REFP_NUM);

    com_mset(mod_info_curr->num_nz, 0, sizeof(int)*N_C*MAX_NUM_TB);
    com_mset(mod_info_curr->coef, 0, sizeof(s16) * N_C * MAX_CU_DIM);

    com_mset(mod_info_curr->affine_mv, 0, sizeof(CPMV) * REFP_NUM * VER_NUM * MV_D);
    com_mset(mod_info_curr->affine_mvd, 0, sizeof(s16) * REFP_NUM * VER_NUM * MV_D);
}

static void init_LOG_LUT(ENC_PIBC *pi)
{
    int size = sizeof(s8) * (MAX_CU_SIZE + 1);
    memset(pi->ctu_log2_tbl, 0, size);
    // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
    int c = 0;
    for (int i = 0, n = 0; i <= MAX_CU_SIZE; i++)
    {
        if (i == (1 << n))
        {
            c = n;
            n++;
        }

        pi->ctu_log2_tbl[i] = c;
    }
}

static void update_ibc_mv_cand(u32 sad, int x, int y, u32 *sad_best_cand, s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D])
{
    int j = CHROMA_REFINEMENT_CANDIDATES - 1;

    if (sad < sad_best_cand[CHROMA_REFINEMENT_CANDIDATES - 1])
    {
        for (int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
        {
            if (sad < sad_best_cand[t])
                j = t;
        }

        for (int k = CHROMA_REFINEMENT_CANDIDATES - 1; k > j; k--)
        {
            sad_best_cand[k] = sad_best_cand[k - 1];

            mv_cand[k][0] = mv_cand[k - 1][0];
            mv_cand[k][1] = mv_cand[k - 1][1];
        }
        sad_best_cand[j] = sad;
        mv_cand[j][0] = x;
        mv_cand[j][1] = y;
    }
}

#if USE_IBC
static u32 getIComponentBits(int val)
{
    if (!val)
    {
        return 1;
    }

    u32 length = 1;
    u32 temp = (val <= 0) ? (-val << 1) + 1 : (val << 1);

    while (1 != temp)
    {
        temp >>= 1;
        length += 2;
    }

    return length;
}

u32 get_bv_cost_bits(int mv_x, int mv_y)
{
    return getIComponentBits(mv_x) + getIComponentBits(mv_y);
}
#else
__inline static u32 get_exp_golomb_bits(u32 abs_mvd)
{
  int bits = 0;
  int len_i, len_c, nn;
  /* abs(mvd) */
  nn = ((abs_mvd + 1) >> 1);
  for (len_i = 0; len_i < 16 && nn != 0; len_i++)
  {
    nn >>= 1;
  }
  len_c = (len_i << 1) + 1;
  bits += len_c;
  /* sign */
  if (abs_mvd)
  {
    bits++;
  }
  return bits;
}

int get_ibc_mv_bits(int mvd_x, int mvd_y)
{
    int bits = 0;
    bits = (mvd_x > 2048 || mvd_x <= -2048) ? get_exp_golomb_bits(COM_ABS(mvd_x)) : enc_tbl_mv_bits[mvd_x];
    bits += (mvd_y > 2048 || mvd_y <= -2048) ? get_exp_golomb_bits(COM_ABS(mvd_y)) : enc_tbl_mv_bits[mvd_y];
    return bits;
}
#endif

#if IBC_BVP
u32 get_bv_cost_bits_ibc_hash(ENC_CORE *core, int mv_x, int mv_y, u8 *bvp_idx)
{
    int idx;
    u32  curr_bits = COM_UINT32_MAX;
    u32  best_bits = COM_UINT32_MAX;

    if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
    {
        *bvp_idx = 0;
#if IBC_MV_BITS_EST
        best_bits = IBC_MV_BITS(abs(mv_x)) + IBC_MV_BITS(abs(mv_y));
#else
        best_bits = enc_bvd_bits[mv_x + 512] + enc_bvd_bits[mv_y + 512];
#endif
    }
    else
    {
        for (idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
        {
#if IBC_MV_BITS_EST
            curr_bits = IBC_MV_BITS(abs(mv_x - core->bvp_cands[idx][MV_X])) + IBC_MV_BITS(abs(mv_y - core->bvp_cands[idx][MV_Y])) + enc_bvp_idx_bits[idx];
#else
            curr_bits = enc_bvd_bits[mv_x - core->bvp_cands[idx][MV_X] + 512] + enc_bvd_bits[mv_y - core->bvp_cands[idx][MV_Y] + 512] + enc_bvp_idx_bits[idx];
#endif

            if (curr_bits <= best_bits)
            {
                *bvp_idx = idx;
                best_bits = curr_bits;

                if (best_bits < 5)
                {
                    goto end;
                }
            }
        }
    }
end:
    return best_bits;
}

static int ibc_hash_found_check_bvp(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 mv[MV_D], u8 *bvp_idx)
{
    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    u32 rui_cost = COM_UINT32_MAX;
    int mv_bits = 0, best_mv_bits = 0;
    COM_PIC *ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel *org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel *rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel *ref = rec;

    u32 sad_bvp = COM_UINT32_MAX, sad_hash = COM_UINT32_MAX, sad_temp = COM_UINT32_MAX;
    u8 idx_best = 0;
    int x, y;
    x = mv[MV_X];
    y = mv[MV_Y];
#if ISC_RSD && !RSD_OPT
    static pel org_tmp[MAX_CU_DIM];
    u8 sp_rsd_flag = core->mod_info_curr.sp_rsd_flag;
    u8 intra_row_rsd_flag = core->mod_info_curr.intra_row_rsd_flag;
    if (sp_rsd_flag)
        rsd_match_16b(roi_width, roi_height, org_tmp, org, pi->stride_org[Y_C], intra_row_rsd_flag);
#endif
#if IBC_MV_BITS_EST
    mv_bits = IBC_MV_BITS(abs(x - (core->bvp_cands[*bvp_idx][MV_X]))) + IBC_MV_BITS(abs(y - (core->bvp_cands[*bvp_idx][MV_Y]))) + enc_bvp_idx_bits[*bvp_idx];
#else
    mv_bits = enc_bvd_bits[x - (core->bvp_cands[*bvp_idx][MV_X]) + 512] + enc_bvd_bits[y - (core->bvp_cands[*bvp_idx][MV_Y]) + 512] + enc_bvp_idx_bits[*bvp_idx];
#endif

    sad_hash = GET_MV_COST(ctx, mv_bits);
    ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
    if (sp_rsd_flag)
        sad_hash += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
    else
#endif
    sad_hash += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
    rui_cost = sad_hash;

    for (int idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
    {
        x = core->bvp_cands[idx][MV_X];
        y = core->bvp_cands[idx][MV_Y];

        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
            continue;
        }
#if ExtStrIBC||RBU_LIMIT_IBC
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = cu_x + tmp[k][0];
                current_y1 = cu_y + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1 + x;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrIBC
            if (roi_width * roi_height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_IBC
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = cu_x;
                int end_y_in_pic = cu_y + roi_height - 1;
                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                {
                    continue;
                }
            }

#endif
        }
#endif
        mv_bits = 2 + enc_bvp_idx_bits[idx];
        sad_temp = GET_MV_COST(ctx, mv_bits);
        ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
        if (sp_rsd_flag)
            sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
        else
#endif
        sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
        if (sad_temp < sad_bvp)
        {
            sad_bvp = sad_temp;
            idx_best = idx;
        }
    }
    if (sad_bvp <= sad_hash)
    {
        *bvp_idx = idx_best;
        mv[MV_X] = core->bvp_cands[idx_best][MV_X];
        mv[MV_Y] = core->bvp_cands[idx_best][MV_Y];
        rui_cost = sad_bvp;
    }
    return rui_cost;
}

static void update_ibc_mv_cand_one(u32 sad, int x, int y, u32 sad_best_cand[1], s16 mv_cand[1][MV_D], u8 mvp_idx, u8  bvp_idx_cand[1], int mv_bits, int best_bv_bits[1])
{
    if (sad < sad_best_cand[0])
    {
        sad_best_cand[0] = sad;
        mv_cand[0][0] = x;
        mv_cand[0][1] = y;
        bvp_idx_cand[0] = mvp_idx;
        best_bv_bits[0] = mv_bits;
    }
    else if (sad == sad_best_cand[0])
    {
        if (mv_bits < best_bv_bits[0])
        {
            sad_best_cand[0] = sad;
            mv_cand[0][0] = x;
            mv_cand[0][1] = y;
            bvp_idx_cand[0] = mvp_idx;
            best_bv_bits[0] = mv_bits;
        }
    }
}
#if RSD_OPT
static void update_rsd_mv_cand_one(u32 sad, int x, int y, u32 sad_best_cand[1], s16 mv_cand[1][MV_D], int mv_bits, int best_bv_bits[1])
{
    if (sad < sad_best_cand[0])
    {
        sad_best_cand[0] = sad;
        mv_cand[0][0] = x;
        mv_cand[0][1] = y;
        best_bv_bits[0] = mv_bits;
    }
    else if (sad == sad_best_cand[0])
    {
        if (mv_bits < best_bv_bits[0])
        {
            sad_best_cand[0] = sad;
            mv_cand[0][0] = x;
            mv_cand[0][1] = y;
            best_bv_bits[0] = mv_bits;
        }
    }
}
#endif
void get_bvd_bits_one_direction(ENC_CORE *core, u32 bvd_bits_table[MAX_NUM_BVP], int dir, int dir_value)
{
    int idx;
    for (idx = 0; idx < core->cnt_class_cands; idx++)
    {
#if IBC_MV_BITS_EST
        bvd_bits_table[idx] = IBC_MV_BITS(abs(dir_value - core->bvp_cands[idx][dir]));
#else
        bvd_bits_table[idx] = enc_bvd_bits[dir_value - core->bvp_cands[idx][dir] + 512];
#endif
    }

}

u32 get_bv_cost_bits_ibc_local(ENC_CORE *core, int mv_x, int mv_y, u8 *bvp_idx, u32 bvd_bits_table[MAX_NUM_BVP], int dir)
{
    int  idx, mv_dir;
    u32  curr_bits = COM_UINT32_MAX;
    u32  best_bits = COM_UINT32_MAX;

    if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
    {
        *bvp_idx = 0;
#if IBC_MV_BITS_EST
        best_bits = IBC_MV_BITS(abs(mv_x)) + IBC_MV_BITS(abs(mv_y));
#else
        best_bits = enc_bvd_bits[mv_x + 512] + enc_bvd_bits[mv_y + 512];
#endif
    }
    else
    {
        if (dir == 1)
        {
            mv_dir = mv_x;
        }
        else
        {
            mv_dir = mv_y;
        }
        for (idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
        {
#if IBC_MV_BITS_EST
            curr_bits = IBC_MV_BITS(abs(mv_dir - core->bvp_cands[idx][1 - dir])) + bvd_bits_table[idx] + enc_bvp_idx_bits[idx];
#else
            curr_bits = enc_bvd_bits[mv_dir - core->bvp_cands[idx][1 - dir] + 512] + bvd_bits_table[idx] + enc_bvp_idx_bits[idx];
#endif

            if (curr_bits <= best_bits)
            {
                *bvp_idx = idx;
                best_bits = curr_bits;
                if (best_bits < 5)
                {
                    goto end;
                }
            }
        }
    }
end:
    return best_bits;
}

static void ibc_set_starting_point(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 starting_point[MV_D], u8* bvpasbv_idx)
{
    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    int mv_bits = 0, best_mv_bits = 0;
    COM_PIC *ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel *org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel *rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel *ref = rec;

    u32 sad_bvp = COM_UINT32_MAX, sad_temp = COM_UINT32_MAX;
    u8 idx_best = 0;
    int x, y;
    int idx;

#if ISC_RSD && !RSD_OPT
    static pel org_tmp[MAX_CU_DIM];
    u8 sp_rsd_flag = core->mod_info_curr.sp_rsd_flag;
    u8 intra_row_rsd_flag = core->mod_info_curr.intra_row_rsd_flag;
    if (sp_rsd_flag)
        rsd_match_16b(roi_width, roi_height, org_tmp, org, pi->stride_org[Y_C], intra_row_rsd_flag);
#endif
    for (idx = 0; idx < min(MAX_NUM_BVP, core->cnt_class_cands); idx++)
    {
        if (core->cnt_hbvp_cands < 2)
        {
            continue;
        }
        x = core->bvp_cands[idx][MV_X];
        y = core->bvp_cands[idx][MV_Y];

        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
            continue;
        }
#if RBU_LIMIT_IBC||ExtStrIBC
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = cu_x + tmp[k][0];
                current_y1 = cu_y + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1 + x;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrIBC
            if (roi_width * roi_height < ExtStrLenMin)
            {

                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif	
#if RBU_LIMIT_IBC
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = cu_x;
                int end_y_in_pic = cu_y + roi_height - 1;
                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                {
                    continue;
                }
            }

#endif	
        }
#endif
        mv_bits = 2 + enc_bvp_idx_bits[idx];
        sad_temp = GET_MV_COST(ctx, mv_bits);
        ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
        if (sp_rsd_flag)
            sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
        else
#endif
        sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
        if (sad_temp < sad_bvp)
        {
            sad_bvp = sad_temp;
            idx_best = idx;
        }
    }

    *bvpasbv_idx = idx_best;

    if (sad_bvp < 1000)
    {
        starting_point[MV_X] = core->bvp_cands[idx_best][MV_X];
        starting_point[MV_Y] = core->bvp_cands[idx_best][MV_Y];
    }
}
#endif

#if IBC_ENH
static void fill_enc_bvp_list(s16(*bv_list)[MV_D], int* num_cands, const s16 bv_new[MV_D])
{
    if (*num_cands < MAX_NUM_ENC_BVP)
    {
        for (int i = 0; i < *num_cands; i++)
        {
            if (SAME_MV(bv_list[i], bv_new))
            {
                return;
            }
        }
        copy_mv(bv_list[*num_cands], bv_new);
        (*num_cands)++;
    }
}
#endif

#if IBC_BVP
static int pibc_search_estimation(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D], u8 *bvp_idx, s16 starting_point[MV_D], u8 *bvpasbv_idx)
#else
static int pibc_search_estimation(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D])
#endif
{
#if IBC_ME_OPT
  int search_mode = ctx->param.ibc_search_mode;
#endif
    int mv_search_range_left[2] = { 0 };
    int mv_search_range_right[2] = { 0 };

    int srch_rng_hor_left = 0;
    int srch_rng_hor_right = 0;
    int srch_rng_ver_top = 0;
    int srch_rng_ver_bottom = 0;

    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int pu_pel_offset_x = 0;
    const int pu_pel_offset_y = 0;

    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;

#if IBC_BVP
#if IBC_ME_OPT
    const int org_bvp_x = starting_point[MV_X];
    const int org_bvp_y = starting_point[MV_Y];
    int bvp_x = starting_point[MV_X];
    int bvp_y = starting_point[MV_Y];
#else
    const int bvp_x = starting_point[MV_X];
    const int bvp_y = starting_point[MV_Y];
#endif
    const int bvp_as_bv_idx = *bvpasbv_idx;
#if IBC_ME_OPT
    int cu_pel_x_new = starting_point[MV_X] + cu_x;
    int cu_pel_y_new = starting_point[MV_Y] + cu_y;
#else
    const int cu_pel_x_new = starting_point[MV_X] + cu_x;
    const int cu_pel_y_new = starting_point[MV_Y] + cu_y;
#endif
#endif

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);

    u32 sad = 0;
    u32 sad_best = COM_UINT32_MAX;
    u32 rui_cost = COM_UINT32_MAX;
    int bestX = 0;
    int bestY = 0;
    int mv_bits = 0, best_mv_bits = 0;

    COM_PIC *ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel *org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel *rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel *ref = rec;

    int best_cand_idx = 0;

#if IBC_BVP
    u32 sad_best_cand[1];
    s16 mv_cand[1][MV_D];
    u8 bvp_idx_cand[1];
    int best_bv_bits[1];
    u8 bvp_idx_temp;

    u32 bvd_bits_table[MAX_NUM_BVP] = { 0 };
    u8 bestIDX;

#if IBC_ME_OPT
    ibc_set_search_range(ctx, core, cu_pel_x_new, cu_pel_y_new, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
        ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right, !search_mode);
#else
    ibc_set_search_range(ctx, core, cu_pel_x_new, cu_pel_y_new, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
        ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right);
#endif
#else
    u32 sad_best_cand[CHROMA_REFINEMENT_CANDIDATES];
    s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D];

    ibc_set_search_range(ctx, core, cu_x, cu_y, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
        ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right);
#endif

    srch_rng_hor_left = mv_search_range_left[0];
    srch_rng_hor_right = mv_search_range_right[0];
    srch_rng_ver_top = mv_search_range_left[1];
    srch_rng_ver_bottom = mv_search_range_right[1];

#if !IBC_BVP
    mvp[MV_X] = 0;
    mvp[MV_Y] = 0;
#endif

#if IBC_BVP
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;

    sad_best_cand[0] = COM_UINT32_MAX;
    mv_cand[0][0] = 0;
    mv_cand[0][1] = 0;
    bvp_idx_cand[0] = 0;
    best_bv_bits[0] = COM_INT16_MAX;

    bvp_idx_temp = 0;
    bestIDX = 0;
    int bv_x_temp, bv_y_temp;

#if ISC_RSD && !RSD_OPT
    static pel org_tmp[MAX_CU_DIM];
    u8 sp_rsd_flag = core->mod_info_curr.sp_rsd_flag;
    u8 intra_row_rsd_flag = core->mod_info_curr.intra_row_rsd_flag;
    if (sp_rsd_flag)
        rsd_match_16b(roi_width, roi_height, org_tmp, org, pi->stride_org[Y_C], intra_row_rsd_flag);
#endif

    if (bvp_as_bv_idx < core->cnt_class_cands)
    {
        u32 sad_temp = COM_UINT32_MAX;
        int x, y;
        x = core->bvp_cands[bvp_as_bv_idx][MV_X];
        y = core->bvp_cands[bvp_as_bv_idx][MV_Y];

        if (core->cnt_hbvp_cands < 2)
        {
            x = 0;
            y = 0;
        }

        if (is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
#if RBU_LIMIT_IBC||ExtStrIBC
            int breakflag = 0;
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + x;
                    int ref_start_y_in_pic1 = current_y1 + y;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        breakflag = 1;
                    }
                }
#endif
#if RBU_LIMIT_IBC 
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { x,y };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        breakflag = 1;
                    }
                }
            }
#endif
#endif
#if  ExtStrIBC||RBU_LIMIT_IBC
            if (!breakflag)
            {
#endif
                mv_bits = 2 + enc_bvp_idx_bits[bvp_as_bv_idx];
                sad_temp = GET_MV_COST(ctx, mv_bits);
                ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
                if (sp_rsd_flag)
                    sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                else
#endif
                sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

                update_ibc_mv_cand_one(sad_temp, x, y, sad_best_cand, mv_cand, bvp_as_bv_idx, bvp_idx_cand, mv_bits, best_bv_bits);
            
#if  ExtStrIBC||RBU_LIMIT_IBC
            }
#endif
        }
    }

#else
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
        sad_best_cand[cand] = COM_UINT32_MAX;
        mv_cand[cand][0] = 0;
        mv_cand[cand][1] = 0;
    }

    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
#endif

#if IBC_ENH
    if(ctx->info.pic_header.ibc_type)
    {
        s16 enc_bvp_list[MAX_NUM_ENC_BVP][2] = { {0, }, };
        int cnt_enc_bvp = 0;
        int pos_x = cu_pel_x, pos_y = cu_pel_y;
        int height = roi_height;
        int width = roi_width;
        int temp_x = 0;
        int temp_y = 0;
        s16 temp_bv[MV_D];
        int pic_width_in_scu = (pic_width + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
        int scup = 0;
        int temp_valid = 0;
        u32* map_scu = ctx->map.map_scu;
        s16(*map_mv)[REFP_NUM][MV_D] = ctx->map.map_mv;
        int temp_pos_list[7][2] = {
            {pos_x - 1, pos_y},
            {pos_x, pos_y - 1},
            {pos_x + width - 1, pos_y - 1},
            {pos_x + width, pos_y - 1},
            {pos_x - 1, pos_y + height - 1},
            {pos_x - 1, pos_y + height},
            {pos_x - 1, pos_y - 1},
        };
        for (int i = 0; i < 7; i++)
        {
            temp_x = temp_pos_list[i][MV_X];
            temp_y = temp_pos_list[i][MV_Y];
            scup = (pic_width_in_scu * (temp_y >> MIN_CU_LOG2)) + (temp_x >> MIN_CU_LOG2);
            if (!(temp_x < 0 || temp_y < 0 || temp_x >= pic_width || temp_y >= pic_height))
            {
                if (MCU_GET_CODED_FLAG(map_scu[scup]) && MCU_GET_IBC(map_scu[scup]))
                {
                    temp_bv[MV_X] = map_mv[scup][0][MV_X] >> 2;
                    temp_bv[MV_Y] = map_mv[scup][0][MV_Y] >> 2;
                    fill_enc_bvp_list(enc_bvp_list, &cnt_enc_bvp, temp_bv);
                }
            }
        }
        for (int i = 0; i < core->cnt_hbvp_cands; i++)
        {
            temp_bv[MV_X] = core->block_motion_cands[i].mv[MV_X];
            temp_bv[MV_Y] = core->block_motion_cands[i].mv[MV_Y];
            fill_enc_bvp_list(enc_bvp_list, &cnt_enc_bvp, temp_bv);
        }
        int len = cnt_enc_bvp;
        for (int i = 0; i < len; i++)
        {
            temp_bv[MV_X] = enc_bvp_list[i][MV_X];
            temp_bv[MV_Y] = enc_bvp_list[i][MV_Y];
            temp_x = pos_x + temp_bv[MV_X];
            temp_y = pos_y + temp_bv[MV_Y];
            scup = (pic_width_in_scu * (temp_y >> MIN_CU_LOG2)) + (temp_x >> MIN_CU_LOG2);

            if (!(temp_x < 0 || temp_y < 0 || temp_x >= pic_width || temp_y >= pic_height))
            {
                if (MCU_GET_CODED_FLAG(map_scu[scup]) && MCU_GET_IBC(map_scu[scup]))
                {
                    temp_bv[MV_X] = temp_bv[MV_X] + (map_mv[scup][0][MV_X] >> 2);
                    temp_bv[MV_Y] = temp_bv[MV_Y] + (map_mv[scup][0][MV_Y] >> 2);
                    fill_enc_bvp_list(enc_bvp_list, &cnt_enc_bvp, temp_bv);
                }
            }
        }

        u32 tempSadBest = 0;
        for (int i = 0; i < cnt_enc_bvp; i++)
        {
            u32 sad_temp = COM_UINT32_MAX;
            int x, y;
            x = enc_bvp_list[i][MV_X];
            y = enc_bvp_list[i][MV_Y];

            get_bvd_bits_one_direction(core, bvd_bits_table, 1, y);
            if (is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, x, y, lcu_width))
            {
#if  ExtStrIBC||RBU_LIMIT_IBC
                int breakflag = 0;
                {
                    int is_out_of_raw_range_flag = 0;
                    int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                    for (int k = 0; k < 4; k++) {
                        int current_x1;
                        int current_y1;
                        current_x1 = cu_x + tmp[k][0];
                        current_y1 = cu_y + tmp[k][1];
                        int ref_start_x_in_pic1 = current_x1 + x;
                        int ref_start_y_in_pic1 = current_y1 + y;
                        if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                        {
                            is_out_of_raw_range_flag = 1;
                            break;
                        }
                    }
#if ExtStrIBC
                    if (roi_width * roi_height < ExtStrLenMin)
                    {
                        if (is_out_of_raw_range_flag)
                        {
                            breakflag = 1;
                        }
                    }
#endif
#if	RBU_LIMIT_IBC
                    if (is_out_of_raw_range_flag)
                    {
                        int nbu = 0;
                        int sv[2] = { x,y };
                        int end_x_in_pic = cu_x;
                        int end_y_in_pic = cu_y + roi_height - 1;
                        nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                        if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                        {
                            breakflag = 1;
                        }
                    }
                }
#endif
#endif
#if  ExtStrIBC||RBU_LIMIT_IBC
                if (!breakflag)
                {
#endif
                    mv_bits = get_bv_cost_bits_ibc_local(core, x, y, &bvp_idx_temp, bvd_bits_table, 1);
                    sad_temp = GET_MV_COST(ctx, mv_bits);
                    ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
                    if (sp_rsd_flag)
                        sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                    else
#endif
                    sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

                    update_ibc_mv_cand_one(sad_temp, x, y, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);

                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 3)
                    {
                        bestX = mv_cand[0][0];
                        bestY = mv_cand[0][1];
                        sad_best = sad_best_cand[0];
                        best_mv_bits = mv_bits;
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
#if IBC_BVP
                        bestIDX = bvp_idx_cand[0];
                        *bvp_idx = bestIDX;
#endif
                        goto end;
                    }

#if  ExtStrIBC||RBU_LIMIT_IBC
                }
#endif
            }
        }
    }
#endif

#if SCC_CROSSINFO_ULTILIZE
    if (ctx->slice_type == SLICE_I)
    {
        u32 tempSadBest = 0;
        for (int i = 0; i < core->b_offset_num; i++)
        {
            u32 sad_temp = COM_UINT32_MAX;
            int x, y;
            x = core->brother_offset[i].mv[0][0];
            y = core->brother_offset[i].mv[0][1];

            get_bvd_bits_one_direction(core, bvd_bits_table, 1, y);
            if (is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, x, y, lcu_width))
            {
#if  ExtStrIBC||RBU_LIMIT_IBC
                int breakflag = 0;
                {
                    int is_out_of_raw_range_flag = 0;
                    int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                    for (int k = 0; k < 4; k++) {
                        int current_x1;
                        int current_y1;
                        current_x1 = cu_x + tmp[k][0];
                        current_y1 = cu_y + tmp[k][1];
                        int ref_start_x_in_pic1 = current_x1 + x;
                        int ref_start_y_in_pic1 = current_y1 + y;
                        if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                        {
                            is_out_of_raw_range_flag = 1;
                            break;
                        }
                    }
#if ExtStrIBC
                    if (roi_width * roi_height < ExtStrLenMin)
                    {
                        if (is_out_of_raw_range_flag)
                        {
                            breakflag = 1;
                        }
                    }

#endif
#if	RBU_LIMIT_IBC
                    if (is_out_of_raw_range_flag)
                    {
                        int nbu = 0;
                        int sv[2] = { x,y };
                        int end_x_in_pic = cu_x;
                        int end_y_in_pic = cu_y + roi_height - 1;
                        nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                        if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                        {
                            breakflag = 1;
                        }
                    }
                }
#endif
#endif
#if  ExtStrIBC||RBU_LIMIT_IBC
                if (!breakflag)
                {
#endif
                    mv_bits = get_bv_cost_bits_ibc_local(core, x, y, &bvp_idx_temp, bvd_bits_table, 1);
                    sad_temp = GET_MV_COST(ctx, mv_bits);       
                    ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
                    if (sp_rsd_flag)
                        sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                    else
#endif
                    sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

                    update_ibc_mv_cand_one(sad_temp, x, y, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);

                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 3)
                    {
                        bestX = mv_cand[0][0];
                        bestY = mv_cand[0][1];
                        sad_best = sad_best_cand[0];
                        best_mv_bits = mv_bits;
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
#if IBC_BVP
                        bestIDX = bvp_idx_cand[0];
                        *bvp_idx = bestIDX;
#endif
                        goto end;
                    }
                
#if  ExtStrIBC||RBU_LIMIT_IBC
                }
#endif
            }
        }

        for (int i = 0; i < core->p_offset_num; i++)
        {
            u32 sad_temp = COM_UINT32_MAX;
            int x, y;
            x = core->parent_offset[i].mv[0][0];
            y = core->parent_offset[i].mv[0][1];

            get_bvd_bits_one_direction(core, bvd_bits_table, 1, y);
            if (is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, x, y, lcu_width))
            {
#if RBU_LIMIT_IBC||ExtStrIBC
                int breakflag = 0;
                {
                    int is_out_of_raw_range_flag = 0;
                    int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                    for (int k = 0; k < 4; k++) {
                        int current_x1;
                        int current_y1;
                        current_x1 = cu_x + tmp[k][0];
                        current_y1 = cu_y + tmp[k][1];
                        int ref_start_x_in_pic1 = current_x1 + x;
                        int ref_start_y_in_pic1 = current_y1 + y;
                        if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                        {
                            is_out_of_raw_range_flag = 1;
                            break;
                        }
                    }
#if ExtStrIBC
                    if (roi_width * roi_height < ExtStrLenMin)
                    {
                        if (is_out_of_raw_range_flag)
                        {
                            breakflag = 1;
                        }
                    }

#endif
#if RBU_LIMIT_IBC
                    if (is_out_of_raw_range_flag)
                    {
                        int nbu = 0;
                        int sv[2] = { x,y };
                        int end_x_in_pic = cu_x;
                        int end_y_in_pic = cu_y + roi_height - 1;
                        nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                        if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                        {
                            breakflag = 1;
                        }
                    }
                }
#endif
#endif
#if  ExtStrIBC||RBU_LIMIT_IBC
                if (!breakflag)
                {
#endif
                    mv_bits = get_bv_cost_bits_ibc_local(core, x, y, &bvp_idx_temp, bvd_bits_table, 1);
                    sad_temp = GET_MV_COST(ctx, mv_bits);
                    ref = rec + ref_pic->stride_luma * y + x;
#if ISC_RSD && !RSD_OPT
                    if (sp_rsd_flag)
                        sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                    else
#endif
                    sad_temp += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

                    update_ibc_mv_cand_one(sad_temp, x, y, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);

                    tempSadBest = sad_best_cand[0];
                    if (sad_best_cand[0] <= 3)
                    {
                        bestX = mv_cand[0][0];
                        bestY = mv_cand[0][1];
                        sad_best = sad_best_cand[0];
                        best_mv_bits = mv_bits;
                        mv[0] = bestX;
                        mv[1] = bestY;
                        rui_cost = sad_best;
#if IBC_BVP
                        bestIDX = bvp_idx_cand[0];
                        *bvp_idx = bestIDX;
#endif
                        goto end;
                    }
                
#if  ExtStrIBC||RBU_LIMIT_IBC
                }
#endif
            }
        }
    }
#endif

    {
        u32 tempSadBest = 0;
        int srLeft = srch_rng_hor_left, srRight = srch_rng_hor_right, srTop = srch_rng_ver_top, srBottom = srch_rng_ver_bottom;
        int boundY = 0;
        if (starting_point[0] == 0 && starting_point[1] == 0)
        {
            boundY = (0 - roi_height - pu_pel_offset_y);
        }
        else
        {
            boundY = (srch_rng_ver_bottom - pu_pel_offset_y);
        }

#if IBC_BVP
        bv_x_temp = 0 + bvp_x;
        get_bvd_bits_one_direction(core, bvd_bits_table, 0, bv_x_temp);

        for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y_new); y <= boundY; ++y)
#else
        for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y); y <= boundY; ++y)
#endif
        {
#if IBC_ENH
            if (ctx->info.pic_header.ibc_type == 1 && (abs(y) > 64) && (y % 2 == 0))
                continue;
#endif
#if IBC_BVP
            bv_y_temp = y + bvp_y;

            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, 0, y, lcu_width))
#endif
            {
                continue;
            }
#if RBU_LIMIT_IBC||ExtStrIBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                    int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }

#endif
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { bv_x_temp,bv_y_temp };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        continue;
                    }
                }
            }
#endif
#endif
#if IBC_BVP
            mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 0);
#else
            mv_bits = get_bv_cost_bits(0, y);
#endif
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
#if IBC_BVP
            ref = rec + ref_pic->stride_luma * bv_y_temp + bv_x_temp;
#else
            ref = rec + ref_pic->stride_luma * y;
#endif
#if ISC_RSD && !RSD_OPT
            if (sp_rsd_flag)
                sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
            else
#endif
            sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

#if IBC_BVP
            update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
            update_ibc_mv_cand(sad, 0, y, sad_best_cand, mv_cand);
#endif
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                *bvp_idx = bestIDX;
#endif
                goto end;
            }
        }

#if IBC_BVP
        const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x_new);
        bv_y_temp = 0 + bvp_y;
        get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
#else
        const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x);
#endif


        int temp_x = 0;
        if (starting_point[0] == 0 && starting_point[1] == 0)
        {
            temp_x = 0 - roi_width - pu_pel_offset_x;
        }
        else
        {
            temp_x = srch_rng_hor_right - pu_pel_offset_x;
        }
        for (int x = temp_x; x >= boundX; --x)
        {
#if IBC_ENH
            if (ctx->info.pic_header.ibc_type == 1 && abs(x) > 64 && (x % 2 == 0))
                continue;
#endif
#if IBC_BVP
            bv_x_temp = x + bvp_x;
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, 0, lcu_width))
#endif
            {
                continue;
            }
#if RBU_LIMIT_IBC||ExtStrIBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                    int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }

#endif	
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { bv_x_temp,bv_y_temp };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        continue;
                    }
                }

            }
#endif
#endif
#if IBC_BVP
            mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
            mv_bits = get_bv_cost_bits(x, 0);
#endif
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
#if IBC_BVP
            ref = rec + ref_pic->stride_luma * bv_y_temp + bv_x_temp;
#else
            ref = rec + x;
#endif
#if ISC_RSD && !RSD_OPT
            if (sp_rsd_flag)
                sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
            else
#endif
            sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

#if IBC_BVP
            update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
            update_ibc_mv_cand(sad, x, 0, sad_best_cand, mv_cand);
#endif
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                *bvp_idx = bestIDX;
#endif
                goto end;
            }
        }

        bestX = mv_cand[0][0];
        bestY = mv_cand[0][1];
        sad_best = sad_best_cand[0];

#if IBC_BVP
        bestIDX = bvp_idx_cand[0];
        if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
        {
#if IBC_MV_BITS_EST
            mv_bits = IBC_MV_BITS(abs(bestX)) + IBC_MV_BITS(abs(bestY));
#else
            mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512];
#endif
        }
        else
        {
#if IBC_MV_BITS_EST
            mv_bits = IBC_MV_BITS(abs(bestX - (core->bvp_cands[bestIDX][MV_X]))) + IBC_MV_BITS(abs(bestY - (core->bvp_cands[bestIDX][MV_Y]))) + enc_bvp_idx_bits[bestIDX];
#else
            mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
#endif
        }
#endif

        sad = GET_MV_COST(ctx, mv_bits);
        if ((!bestX && !bestY) || (sad_best - sad <= 32))
        {
            best_cand_idx = 0;
            bestX = mv_cand[best_cand_idx][0];
            bestY = mv_cand[best_cand_idx][1];
            sad_best = sad_best_cand[best_cand_idx];
            mv[0] = bestX;
            mv[1] = bestY;
            rui_cost = sad_best;
#if IBC_BVP
            bestIDX = bvp_idx_cand[0];
            *bvp_idx = bestIDX;
#endif
            goto end;
        }

#if IBC_ME_OPT
        if (search_mode) {
            starting_point[MV_X] = bestX;
            starting_point[MV_Y] = bestY;
            bvp_x = starting_point[MV_X];
            bvp_y = starting_point[MV_Y];
            cu_pel_x_new = starting_point[MV_X] + cu_x;
            cu_pel_y_new = starting_point[MV_Y] + cu_y;
            ibc_set_search_range(ctx, core, cu_pel_x_new, cu_pel_y_new, log2_cuw, log2_cuh, ctx->pibc.search_range_x,
                ctx->pibc.search_range_y, mv_search_range_left, mv_search_range_right, 1);
            srch_rng_hor_left = mv_search_range_left[0];
            srch_rng_hor_right = mv_search_range_right[0];
            srch_rng_ver_top = mv_search_range_left[1];
            srch_rng_ver_bottom = mv_search_range_right[1];
#if IBC_ENH
            const int blk_th = ctx->info.pic_header.ibc_type == 1 ? 128 : 16;
            int is_small_blk = (1 << log2_cuw) < blk_th && (1 << log2_cuh) < blk_th;
#else
            int is_small_blk = (1 << log2_cuw) < 16 && (1 << log2_cuh) < 16;
#endif

            if (!is_small_blk)
            {
                if (abs(bestX - org_bvp_x) >= 4)
                {
                    if (starting_point[0] == 0 && starting_point[1] == 0)
                    {
                        boundY = (0 - roi_height - pu_pel_offset_y);
                    }
                    else
                    {
                        boundY = (srch_rng_ver_bottom - pu_pel_offset_y);
                    }

#if IBC_BVP
                    bv_x_temp = 0 + bvp_x;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 0, bv_x_temp);

                    for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y_new); y <= boundY; ++y)
#else
                    for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y); y <= boundY; ++y)
#endif
                    {
#if IBC_BVP
                        bv_y_temp = y + bvp_y;

                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                            pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                            pic_width, pic_height, 0, y, lcu_width))
#endif
                        {
                            continue;
                        }
#if RBU_LIMIT_IBC||ExtStrIBC
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }

#endif
#if RBU_LIMIT_IBC
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }
                        }
#endif
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 0);
#else
                        mv_bits = get_bv_cost_bits(0, y);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + ref_pic->stride_luma * bv_y_temp + bv_x_temp;
#else
                        ref = rec + ref_pic->stride_luma * y;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, 0, y, sad_best_cand, mv_cand);
#endif
                        tempSadBest = sad_best_cand[0];
                        if (sad_best_cand[0] <= 3)
                        {
                            bestX = mv_cand[0][0];
                            bestY = mv_cand[0][1];
                            sad_best = sad_best_cand[0];
                            best_mv_bits = mv_bits;
                            mv[0] = bestX;
                            mv[1] = bestY;
                            rui_cost = sad_best;
#if IBC_BVP
                            bestIDX = bvp_idx_cand[0];
                            *bvp_idx = bestIDX;
#endif
                            goto end;
                        }
                    }
                }
                if (abs(bestY - org_bvp_y) >= 4)
                {
#if IBC_BVP
                    const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x_new);
                    bv_y_temp = 0 + bvp_y;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
#else
                    const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x);
#endif

                    int temp_x = 0;
                    if (starting_point[0] == 0 && starting_point[1] == 0)
                    {
                        temp_x = 0 - roi_width - pu_pel_offset_x;
                    }
                    else
                    {
                        temp_x = srch_rng_hor_right - pu_pel_offset_x;
                    }
                    for (int x = temp_x; x >= boundX; --x)
                    {
#if IBC_BVP
                        bv_x_temp = x + bvp_x;
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                            pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, 0, lcu_width))
#endif
                        {
                            continue;
                        }
#if RBU_LIMIT_IBC||ExtStrIBC
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }

#endif	
#if RBU_LIMIT_IBC
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }

                        }
#endif
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                        mv_bits = get_bv_cost_bits(x, 0);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + ref_pic->stride_luma * bv_y_temp + bv_x_temp;
#else
                        ref = rec + x;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);

#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, x, 0, sad_best_cand, mv_cand);
#endif
                        tempSadBest = sad_best_cand[0];
                        if (sad_best_cand[0] <= 3)
                        {
                            bestX = mv_cand[0][0];
                            bestY = mv_cand[0][1];
                            sad_best = sad_best_cand[0];
                            best_mv_bits = mv_bits;
                            mv[0] = bestX;
                            mv[1] = bestY;
                            rui_cost = sad_best;
#if IBC_BVP
                            bestIDX = bvp_idx_cand[0];
                            *bvp_idx = bestIDX;
#endif
                            goto end;
                        }
                    }
                }
#if IBC_ME_OPT
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];

#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX)) + IBC_MV_BITS(abs(bestY));
#else
                    mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512];
#endif
                }
                else
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX - (core->bvp_cands[bestIDX][MV_X]))) + IBC_MV_BITS(abs(bestY - (core->bvp_cands[bestIDX][MV_Y]))) + enc_bvp_idx_bits[bestIDX];
#else
                    mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
#endif
                }
#endif

                sad = GET_MV_COST(ctx, mv_bits);
                if ((!bestX && !bestY) || (sad_best - sad <= 32))
                {
                    best_cand_idx = 0;
                    bestX = mv_cand[best_cand_idx][0];
                    bestY = mv_cand[best_cand_idx][1];
                    sad_best = sad_best_cand[best_cand_idx];
                    mv[0] = bestX;
                    mv[1] = bestY;
                    rui_cost = sad_best;
#if IBC_BVP
                    bestIDX = bvp_idx_cand[0];
                    *bvp_idx = bestIDX;
#endif
                    goto end;
                }
#endif
            }
#if IBC_ENH
            if (ctx->info.pic_header.ibc_type == 1)
            {
              int localX = cu_pel_x_new == cu_pel_x ? 16 : 32;
              int localY = cu_pel_y_new == cu_pel_y ? 16 : 32;
              ibc_set_search_range(ctx, core, cu_pel_x_new, cu_pel_y_new, log2_cuw, log2_cuh, localX,
                localY, mv_search_range_left, mv_search_range_right, 1);
#if IBC_ENH
              srch_rng_hor_left = mv_search_range_left[0];
              srch_rng_hor_right = mv_search_range_right[0];
              srch_rng_ver_top = mv_search_range_left[1];
              srch_rng_ver_bottom = mv_search_range_right[1];
#endif
            }
#endif
        }
#endif

#if IBC_ENH
        if (ctx->info.pic_header.ibc_type == 1)
        {
            if ((1 << log2_cuw) < 128 && (1 << log2_cuh) < 128
#if ISC_RSD && !RSD_OPT
                && !sp_rsd_flag
#endif
                )
            {
#if IBC_BVP
                for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y_new); y <= srch_rng_ver_bottom; y += 2)
#else
                for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y); y <= srch_rng_ver_bottom; y += 2)
#endif
                {
#if IBC_BVP
                    if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                    if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                        continue;
#if IBC_BVP
                    bv_y_temp = y + bvp_y;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x_new); x <= srch_rng_hor_right; x += 2)
#else
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x++)
#endif
                    {
#if IBC_BVP
                        if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                        if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                            continue;
#if IBC_BVP
                        bv_x_temp = x + bvp_x;
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                        {
                            continue;
                        }
#if RBU_LIMIT_IBC||ExtStrIBC
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }
#endif
#if RBU_LIMIT_IBC
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }
                        }
#endif
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                        mv_bits = get_bv_cost_bits(x, y);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                        ref = rec + y * ref_pic->stride_luma + x;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                    }
                }

                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];

#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX)) + IBC_MV_BITS(abs(bestY));
#else
                    mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512];
#endif
                }
                else
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX - (core->bvp_cands[bestIDX][MV_X]))) + IBC_MV_BITS(abs(bestY - (core->bvp_cands[bestIDX][MV_Y]))) + enc_bvp_idx_bits[bestIDX];
#else
                    mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
#endif
                }
#else
                mv_bits = get_bv_cost_bits(bestX, bestY);
#endif
                sad = GET_MV_COST(ctx, mv_bits);

                if (sad_best - sad <= 16)
                {
                    best_cand_idx = 0;
                    bestX = mv_cand[0][0];
                    bestY = mv_cand[0][1];
                    sad_best = sad_best_cand[best_cand_idx];
                    best_mv_bits = mv_bits;
                    mv[0] = bestX;
                    mv[1] = bestY;
                    rui_cost = sad_best;
#if IBC_BVP
                    bestIDX = bvp_idx_cand[0];
                    *bvp_idx = bestIDX;
#endif
                    goto end;
                }

                starting_point[MV_X] = bestX;
                starting_point[MV_Y] = bestY;
                bvp_x = starting_point[MV_X];
                bvp_y = starting_point[MV_Y];
                cu_pel_x_new = starting_point[MV_X] + cu_x;
                cu_pel_y_new = starting_point[MV_Y] + cu_y;
                ibc_set_search_range(ctx, core, cu_pel_x_new, cu_pel_y_new, log2_cuw, log2_cuh, 1,
                    1, mv_search_range_left, mv_search_range_right, 1);
                srch_rng_hor_left = mv_search_range_left[0];
                srch_rng_hor_right = mv_search_range_right[0];
                srch_rng_ver_top = mv_search_range_left[1];
                srch_rng_ver_bottom = mv_search_range_right[1];

#if IBC_BVP
                for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y_new); y <= srch_rng_ver_bottom; y += 1)
#else
                for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y); y <= srch_rng_ver_bottom; y += 1)
#endif
                {
#if IBC_BVP
                    if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                    if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                        continue;

#if IBC_BVP
                    bv_y_temp = y + bvp_y;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x_new); x <= srch_rng_hor_right; x += 1)
#else
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x++)
#endif
                    {
#if IBC_BVP
                        if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                        if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                            continue;

#if IBC_BVP
                        bv_x_temp = x + bvp_x;
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                        {
                            continue;
                        }
#if RBU_LIMIT_IBC||ExtStrIBC
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }

#endif
#if RBU_LIMIT_IBC
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }
                        }
#endif
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                        mv_bits = get_bv_cost_bits(x, y);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                        ref = rec + y * ref_pic->stride_luma + x;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                    }
                }

                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];

#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX)) + IBC_MV_BITS(abs(bestY));
#else
                    mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512];
#endif
                }
                else
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX - (core->bvp_cands[bestIDX][MV_X]))) + IBC_MV_BITS(abs(bestY - (core->bvp_cands[bestIDX][MV_Y]))) + enc_bvp_idx_bits[bestIDX];
#else
                    mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
#endif
                }
#else
                mv_bits = get_bv_cost_bits(bestX, bestY);
#endif
                sad = GET_MV_COST(ctx, mv_bits);

                if (sad_best - sad <= 16)
                {
                    best_cand_idx = 0;
                    bestX = mv_cand[0][0];
                    bestY = mv_cand[0][1];
                    sad_best = sad_best_cand[best_cand_idx];
                    best_mv_bits = mv_bits;
                    mv[0] = bestX;
                    mv[1] = bestY;
                    rui_cost = sad_best;
#if IBC_BVP
                    bestIDX = bvp_idx_cand[0];
                    *bvp_idx = bestIDX;
#endif
                    goto end;
                }

            }
        }
        else
        {
#endif
            if ((1 << log2_cuw) < 16 && (1 << log2_cuh) < 16
#if ISC_RSD && !RSD_OPT
                && !sp_rsd_flag
#endif
                )
            {
#if IBC_BVP
                for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y_new); y <= srch_rng_ver_bottom; y += 2)
#else
                for (int y = COM_MAX(srch_rng_ver_top, -cu_pel_y); y <= srch_rng_ver_bottom; y += 2)
#endif
                {
#if IBC_BVP
                    if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                    if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                        continue;

#if IBC_BVP
                    bv_y_temp = y + bvp_y;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x_new); x <= srch_rng_hor_right; x++)
#else
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x++)
#endif
                    {
#if IBC_BVP
                        if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                        if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                            continue;

#if IBC_BVP
                        bv_x_temp = x + bvp_x;
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                        {
                            continue;
                        }
#if RBU_LIMIT_IBC||ExtStrIBC
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 },{ roi_width - 1, 0 },{ roi_width - 1, roi_height - 1 },{ 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }

#endif
#if RBU_LIMIT_IBC
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }
                        }
#endif
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                        mv_bits = get_bv_cost_bits(x, y);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                        ref = rec + y * ref_pic->stride_luma + x;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                    }
                }

                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];

#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX)) + IBC_MV_BITS(abs(bestY));
#else
                    mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512];
#endif
                }
                else
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX - (core->bvp_cands[bestIDX][MV_X]))) + IBC_MV_BITS(abs(bestY - (core->bvp_cands[bestIDX][MV_Y]))) + enc_bvp_idx_bits[bestIDX];
#else
                    mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
#endif
                }
#else
                mv_bits = get_bv_cost_bits(bestX, bestY);
#endif
                sad = GET_MV_COST(ctx, mv_bits);

                if (sad_best - sad <= 16)
                {
                    best_cand_idx = 0;
                    bestX = mv_cand[0][0];
                    bestY = mv_cand[0][1];
                    sad_best = sad_best_cand[best_cand_idx];
                    best_mv_bits = mv_bits;
                    mv[0] = bestX;
                    mv[1] = bestY;
                    rui_cost = sad_best;
#if IBC_BVP
                    bestIDX = bvp_idx_cand[0];
                    *bvp_idx = bestIDX;
#endif
                    goto end;
                }

#if IBC_BVP
                for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y_new) + 1); y <= srch_rng_ver_bottom; y += 2)
#else
                for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y) + 1); y <= srch_rng_ver_bottom; y += 2)
#endif
                {
#if IBC_BVP
                    if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                    if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                        continue;

#if IBC_BVP
                    bv_y_temp = y + bvp_y;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x_new); x <= srch_rng_hor_right; x += 2)
#else
                    for (int x = COM_MAX(srch_rng_hor_left, -cu_pel_x); x <= srch_rng_hor_right; x += 2)
#endif
                    {
#if IBC_BVP
                        if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                        if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                            continue;

#if IBC_BVP
                        bv_x_temp = x + bvp_x;
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                            pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                        {
                            continue;
                        }
#if RBU_LIMIT_IBC||ExtStrIBC
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 },{ roi_width - 1, 0 },{ roi_width - 1, roi_height - 1 },{ 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }

#endif
#if RBU_LIMIT_IBC	
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }
                        }
#endif
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                        mv_bits = get_bv_cost_bits(x, y);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                        ref = rec + y * ref_pic->stride_luma + x;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                        tempSadBest = sad_best_cand[0];
                        if (sad_best_cand[0] <= 5)
                        {
                            best_cand_idx = 0;
                            bestX = mv_cand[best_cand_idx][0];
                            bestY = mv_cand[best_cand_idx][1];
                            sad_best = sad_best_cand[best_cand_idx];
                            mv[0] = bestX;
                            mv[1] = bestY;
                            rui_cost = sad_best;
#if IBC_BVP
                            bestIDX = bvp_idx_cand[0];
                            *bvp_idx = bestIDX;
#endif
                            goto end;
                        }
                    }
                }

                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];

#if IBC_BVP
                bestIDX = bvp_idx_cand[0];
                if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX)) + IBC_MV_BITS(abs(bestY));
#else
                    mv_bits = enc_bvd_bits[bestX + 512] + enc_bvd_bits[bestY + 512];
#endif
                }
                else
                {
#if IBC_MV_BITS_EST
                    mv_bits = IBC_MV_BITS(abs(bestX - (core->bvp_cands[bestIDX][MV_X]))) + IBC_MV_BITS(abs(bestY - (core->bvp_cands[bestIDX][MV_Y]))) + enc_bvp_idx_bits[bestIDX];
#else
                    mv_bits = enc_bvd_bits[bestX - (core->bvp_cands[bestIDX][MV_X]) + 512] + enc_bvd_bits[bestY - (core->bvp_cands[bestIDX][MV_Y]) + 512] + enc_bvp_idx_bits[bestIDX];
#endif
                }
#else
                mv_bits = get_bv_cost_bits(bestX, bestY);
#endif
                sad = GET_MV_COST(ctx, mv_bits);

                if ((sad_best >= tempSadBest) || ((sad_best - sad) <= 32))
                {
                    best_cand_idx = 0;
                    bestX = mv_cand[best_cand_idx][0];
                    bestY = mv_cand[best_cand_idx][1];
                    sad_best = sad_best_cand[best_cand_idx];
                    mv[0] = bestX;
                    mv[1] = bestY;
                    rui_cost = sad_best;
#if IBC_BVP
                    bestIDX = bvp_idx_cand[0];
                    *bvp_idx = bestIDX;
#endif
                    goto end;
                }

                tempSadBest = sad_best_cand[0];

#if IBC_BVP
                for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y_new) + 1); y <= srch_rng_ver_bottom; y += 2)
#else
                for (int y = (COM_MAX(srch_rng_ver_top, -cu_pel_y) + 1); y <= srch_rng_ver_bottom; y += 2)
#endif
                {
#if IBC_BVP
                    if ((y == 0) || ((int)(cu_pel_y_new + y + roi_height) >= pic_height))
#else
                    if ((y == 0) || ((int)(cu_pel_y + y + roi_height) >= pic_height))
#endif
                        continue;

#if IBC_BVP
                    bv_y_temp = y + bvp_y;
                    get_bvd_bits_one_direction(core, bvd_bits_table, 1, bv_y_temp);
                    for (int x = (COM_MAX(srch_rng_hor_left, -cu_pel_x_new) + 1); x <= srch_rng_hor_right; x += 2)
#else
                    for (int x = (COM_MAX(srch_rng_hor_left, -cu_pel_x) + 1); x <= srch_rng_hor_right; x += 2)
#endif
                    {

#if IBC_BVP
                        if ((x == 0) || ((int)(cu_pel_x_new + x + roi_width) >= pic_width))
#else
                        if ((x == 0) || ((int)(cu_pel_x + x + roi_width) >= pic_width))
#endif
                            continue;

#if IBC_BVP
                        bv_x_temp = x + bvp_x;
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                            pic_width, pic_height, bv_x_temp, bv_y_temp, lcu_width))
#else
                        if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, y, lcu_width))
#endif
                        {
                            continue;
                        }

#if RBU_LIMIT_IBC||ExtStrIBC						
                        {
                            int is_out_of_raw_range_flag = 0;
                            int tmp[4][2] = { { 0, 0 },{ roi_width - 1, 0 },{ roi_width - 1, roi_height - 1 },{ 0, roi_height - 1 } };
                            for (int k = 0; k < 4; k++) {
                                int current_x1;
                                int current_y1;
                                current_x1 = cu_x + tmp[k][0];
                                current_y1 = cu_y + tmp[k][1];
                                int ref_start_x_in_pic1 = current_x1 + bv_x_temp;
                                int ref_start_y_in_pic1 = current_y1 + bv_y_temp;
                                if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                                {
                                    is_out_of_raw_range_flag = 1;
                                    break;
                                }
                            }
#if ExtStrIBC
                            if (roi_width * roi_height < ExtStrLenMin)
                            {
                                if (is_out_of_raw_range_flag)
                                {
                                    continue;
                                }
                            }

#endif
#if RBU_LIMIT_IBC
                            if (is_out_of_raw_range_flag)
                            {
                                int nbu = 0;
                                int sv[2] = { bv_x_temp,bv_y_temp };
                                int end_x_in_pic = cu_x;
                                int end_y_in_pic = cu_y + roi_height - 1;
                                nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                                {
                                    continue;
                                }
                            }
#endif
                        }
#endif
#if IBC_BVP
                        mv_bits = get_bv_cost_bits_ibc_local(core, bv_x_temp, bv_y_temp, &bvp_idx_temp, bvd_bits_table, 1);
#else
                        mv_bits = get_bv_cost_bits(x, y);
#endif
                        sad = GET_MV_COST(ctx, mv_bits);

                        /* get sad */
#if IBC_BVP
                        ref = rec + bv_y_temp * ref_pic->stride_luma + bv_x_temp;
#else
                        ref = rec + y * ref_pic->stride_luma + x;
#endif
#if ISC_RSD && !RSD_OPT
                        if (sp_rsd_flag)
                            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                        else
#endif
                        sad += calc_sad_16b(roi_width, roi_height, org, ref, pi->stride_org[Y_C], ref_pic->stride_luma, pi->bit_depth);
#if IBC_BVP
                        update_ibc_mv_cand_one(sad, bv_x_temp, bv_y_temp, sad_best_cand, mv_cand, bvp_idx_temp, bvp_idx_cand, mv_bits, best_bv_bits);
#else
                        update_ibc_mv_cand(sad, x, y, sad_best_cand, mv_cand);
#endif
                        tempSadBest = sad_best_cand[0];
                        if (sad_best_cand[0] <= 5)
                        {
                            best_cand_idx = 0;
                            bestX = mv_cand[best_cand_idx][0];
                            bestY = mv_cand[best_cand_idx][1];
                            sad_best = sad_best_cand[best_cand_idx];
                            mv[0] = bestX;
                            mv[1] = bestY;
                            rui_cost = sad_best;
#if IBC_BVP
                            bestIDX = bvp_idx_cand[0];
                            *bvp_idx = bestIDX;
#endif
                            goto end;
                        }
                    }
                }
            }
#if IBC_ENH
        }
#endif
    }

    best_cand_idx = 0;
    bestX = mv_cand[best_cand_idx][0];
    bestY = mv_cand[best_cand_idx][1];
    sad_best = sad_best_cand[best_cand_idx];
    mv[0] = bestX;
    mv[1] = bestY;
    rui_cost = sad_best;
#if IBC_BVP
    bestIDX = bvp_idx_cand[0];
    *bvp_idx = bestIDX;
#endif

end:
    //if (roi_width + roi_height > 8)
    //{
    //    m_numBVs = xMergeCandLists(m_acBVs, m_numBVs, mv_cand, CHROMA_REFINEMENT_CANDIDATES);

    //    if (roi_width + roi_height == 32)
    //    {
    //        m_numBV16s = m_numBVs;
    //    }
    //}

    return rui_cost;
}

static u32 pibc_me_search(ENC_CTX *ctx, ENC_CORE *core, ENC_PIBC *pi, int x, int y, int log2_cuw, int log2_cuh,
    s16 mvp[MV_D], s16 mv[MV_D]
#if IBC_BVP
    , u8 *bvp_idx
#endif
)
{
    u32 cost = 0;
    s16 mv_temp[MV_D] = { 0, 0 };

#if IBC_BVP
    if (ctx->param.ibc_hash_search_flag
#if ISC_RSD && !RSD_OPT      
        && !core->mod_info_curr.sp_rsd_flag
#endif        
        )
    {
#if FAST_SCC_PRECODING
        if (core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].visit && ctx->slice_type == SLICE_I && ctx->info.pic_header.sp_pic_flag) {
            cost = search_ibc_hash_matchrevisit(core, ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx);
        }
        else {
            cost = search_ibc_hash_match(core, ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx);
            core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0mv[MV_X] = mv_temp[0];
            core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0mv[MV_Y] = mv_temp[1];
        }
#else
        cost = search_ibc_hash_match(core, ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx);
#endif
        if ((!(mv_temp[0] == 0 && mv_temp[1] == 0)) && core->cnt_class_cands > 0 && core->cnt_hbvp_cands >= 2)
        {
            cost = ibc_hash_found_check_bvp(ctx, core, pi, x, y, log2_cuw, log2_cuh, mv_temp, bvp_idx);
        }
    }
#if ISC_RSD && !RSD_OPT
    if (ctx->param.rsd_enable_flag && core->mod_info_curr.sp_rsd_flag)
    {
        cost = search_rsd_hash_match(core, ctx, ctx->rsd_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx);
        if ((!(mv_temp[0] == 0 && mv_temp[1] == 0)) && core->cnt_class_cands > 0 && core->cnt_hbvp_cands >= 2)
        {
            cost = ibc_hash_found_check_bvp(ctx, core, pi, x, y, log2_cuw, log2_cuh, mv_temp, bvp_idx);
        }
    }
#endif

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        // if hash search does not work or is not enabled
        s16 starting_point[MV_D] = { 0,0 };
        u8  bvpasbv_idx = 0;
        ibc_set_starting_point(ctx, core, pi, x, y, log2_cuw, log2_cuh, starting_point, &bvpasbv_idx);
        cost = pibc_search_estimation(ctx, core, pi, x, y, log2_cuw, log2_cuh, mvp, mv_temp, bvp_idx, starting_point, &bvpasbv_idx);
    }
#else
    if (ctx->param.ibc_hash_search_flag)
    {
        cost = search_ibc_hash_match(ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mvp, mv_temp);
    }

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        // if hash search does not work or is not enabled
        cost = pibc_search_estimation(ctx, core, pi, x, y, log2_cuw, log2_cuh, mvp, mv_temp);
    }
#endif

    mv[0] = mv_temp[0];
    mv[1] = mv_temp[1];

    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        return COM_UINT32_MAX;
    }  
    return cost;
}
#if RSD_OPT
static int prsd_search_estimation(ENC_CTX* ctx, ENC_CORE* core, ENC_PIBC* pi, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 mv[MV_D])
{
    int mv_search_range_left[2] = { 0 };
    int mv_search_range_right[2] = { 0 };
    int srch_rng_hor_left = 0;
    int srch_rng_hor_right = 0;
    int srch_rng_ver_top = 0;
    int srch_rng_ver_bottom = 0;
    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int pu_pel_offset_x = 0;
    const int pu_pel_offset_y = 0;
    const int cu_pel_x = cu_x;
    const int cu_pel_y = cu_y;
    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);
    u32 sad = 0;
    u32 sad_best = COM_UINT32_MAX;
    u32 rui_cost = COM_UINT32_MAX;
    int bestX = 0;
    int bestY = 0;
    int mv_bits = 0, best_mv_bits = 0;
    COM_PIC* ref_pic = ctx->pibc.pic_unfiltered_rec;
    pel* org = pi->Yuv_org[Y_C] + cu_y * pi->stride_org[Y_C] + cu_x;
    pel* rec = ref_pic->y + cu_y * ref_pic->stride_luma + cu_x;
    pel* ref = rec;
    int best_cand_idx = 0;
    u32 sad_best_cand[CHROMA_REFINEMENT_CANDIDATES];
    s16 mv_cand[CHROMA_REFINEMENT_CANDIDATES][MV_D];
    int best_bv_bits[1];
    best_bv_bits[0] = COM_INT16_MAX;
    rsd_set_search_range(ctx, core, cu_x, cu_y, log2_cuw, log2_cuh, mv_search_range_left, mv_search_range_right);
    srch_rng_hor_left = mv_search_range_left[0];
    srch_rng_hor_right = mv_search_range_right[0];
    srch_rng_ver_top = mv_search_range_left[1];
    srch_rng_ver_bottom = mv_search_range_right[1];
    static pel org_tmp[MAX_CU_DIM];
    u8 sp_rsd_flag = core->mod_info_curr.sp_rsd_flag;
    u8 intra_row_rsd_flag = core->mod_info_curr.intra_row_rsd_flag;
    rsd_match_16b(roi_width, roi_height, org_tmp, org, pi->stride_org[Y_C], intra_row_rsd_flag);
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
        sad_best_cand[cand] = COM_UINT32_MAX;
        mv_cand[cand][0] = 0;
        mv_cand[cand][1] = 0;
    }
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    //****************************** svp ********************************************
    u32 tempSadBest = 0;
    for (int i = 0; i < core->n_offset_num; i++)
    {
        u32 sad_temp = COM_UINT32_MAX;
        int x = core->n_recent_offset[core->n_offset_num - 1 - i].mv[0][MV_X];
        int y = core->n_recent_offset[core->n_offset_num - 1 - i].mv[0][MV_Y];
        if (is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
            pic_width, pic_height, x, y, lcu_width))
        {
#if RBU_LIMIT_IBC||ExtStrIBC
            int breakflag = 0;
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + x;
                    int ref_start_y_in_pic1 = current_y1 + y;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        breakflag = 1;
                    }
                }

#endif
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { x,y };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        breakflag = 1;
                    }
                }
            }
#endif
#endif
#if  ExtStrIBC||RBU_LIMIT_IBC
            if (!breakflag)
            {
#endif
                mv_bits = get_bv_cost_bits(x, y);
                //mv_bits = get_sp_offset_bit_cnt(x, y);
                sad_temp = GET_MV_COST(ctx, mv_bits);
                ref = rec + ref_pic->stride_luma * y + x;
                sad_temp += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
                update_rsd_mv_cand_one(sad_temp, x, y, sad_best_cand, mv_cand, mv_bits, best_bv_bits);
                tempSadBest = sad_best_cand[0];
                if (sad_best_cand[0] <= 3)
                {
                    bestX = mv_cand[0][0];
                    bestY = mv_cand[0][1];
                    sad_best = sad_best_cand[0];
                    best_mv_bits = mv_bits;
                    mv[0] = bestX;
                    mv[1] = bestY;
                    rui_cost = sad_best;
                    goto end;
                }

#if  ExtStrIBC||RBU_LIMIT_IBC
            }
#endif
        }
    }
    //****************************** y dir *******************************************
    {
        u32 tempSadBest = 0;
        int srLeft = srch_rng_hor_left, srRight = srch_rng_hor_right, srTop = srch_rng_ver_top, srBottom = srch_rng_ver_bottom;
        int boundY = (0 - roi_height - pu_pel_offset_y);
        for (int y = COM_MAX(srch_rng_ver_top, 0 - cu_pel_y); y <= boundY; ++y)
        {
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, 0, y, lcu_width))
            {
                continue;
            }
#if RBU_LIMIT_IBC||ExtStrIBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1;
                    int ref_start_y_in_pic1 = current_y1 + y;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }

#endif
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { 0,y };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        continue;
                    }
                }
            }
#endif
#endif
            //mv_bits = get_sp_offset_bit_cnt(0, y);
            mv_bits = get_bv_cost_bits(0, y);
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
            ref = rec + ref_pic->stride_luma * y;
            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
            update_rsd_mv_cand_one(sad, 0, y, sad_best_cand, mv_cand, mv_bits, best_bv_bits);
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
                goto end;
            }
        }
    }
    //****************************** x dir *******************************************
    {
        const int boundX = COM_MAX(srch_rng_hor_left, -cu_pel_x);
        int  temp_x = 0 - roi_width - pu_pel_offset_x;
        for (int x = temp_x; x >= boundX; --x)
        {
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, x, 0, lcu_width))
            {
                continue;
            }
#if RBU_LIMIT_IBC||ExtStrIBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + x;
                    int ref_start_y_in_pic1 = current_y1 + 0;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }

#endif	
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { x,0 };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        continue;
                    }
                }

            }
#endif
#endif
            //mv_bits = get_sp_offset_bit_cnt(x, 0);
            mv_bits = get_bv_cost_bits(x, 0);
            sad = GET_MV_COST(ctx, mv_bits);

            /* get sad */
            ref = rec + x;
            sad += calc_sad_16b(roi_width, roi_height, org_tmp, ref, roi_width, ref_pic->stride_luma, pi->bit_depth);
            update_rsd_mv_cand_one(sad, x, 0, sad_best_cand, mv_cand, mv_bits, best_bv_bits);
            tempSadBest = sad_best_cand[0];
            if (sad_best_cand[0] <= 3)
            {
                bestX = mv_cand[0][0];
                bestY = mv_cand[0][1];
                sad_best = sad_best_cand[0];
                best_mv_bits = mv_bits;
                mv[0] = bestX;
                mv[1] = bestY;
                rui_cost = sad_best;
                goto end;
            }
        }

        bestX = mv_cand[0][0];
        bestY = mv_cand[0][1];
        sad_best = sad_best_cand[0];
        sad = GET_MV_COST(ctx, best_bv_bits[0]);
        if ((!bestX && !bestY) || (sad_best - sad <= 32))
        {
            best_cand_idx = 0;
            bestX = mv_cand[best_cand_idx][0];
            bestY = mv_cand[best_cand_idx][1];
            sad_best = sad_best_cand[best_cand_idx];
            mv[0] = bestX;
            mv[1] = bestY;
            rui_cost = sad_best;
            goto end;
        }
    }
    //****************************** end serach *******************************************

    best_cand_idx = 0;
    bestX = mv_cand[best_cand_idx][0];
    bestY = mv_cand[best_cand_idx][1];
    sad_best = sad_best_cand[best_cand_idx];
    mv[0] = bestX;
    mv[1] = bestY;
    rui_cost = sad_best;
end:
    return rui_cost;
}

static u32 prsd_me_search(ENC_CTX* ctx, ENC_CORE* core, ENC_PIBC* pi, int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D])
{
    u32 cost = 0;
    s16 mv_temp[MV_D] = { 0, 0 };
#if FAST_SCC_PRECODING
    if (core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].visit
        && ((ctx->slice_type == SLICE_I || !core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibc_flag_history) && ctx->info.pic_header.sp_pic_flag)) {
        cost = search_rsd_hash_matchrevisit(core, ctx, ctx->ibc_hash_handle, x, y, log2_cuw, log2_cuh, mv_temp);
    }
    else {
        cost = search_rsd_hash_match(core, ctx, ctx->rsd_hash_handle, x, y, log2_cuw, log2_cuh, mv_temp);
        core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0rsdmv[MV_X] = mv_temp[0];
        core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0rsdmv[MV_Y] = mv_temp[1];
    }
#else
    cost = search_rsd_hash_match(core, ctx, ctx->rsd_hash_handle, x, y, log2_cuw, log2_cuh, mv_temp);
#endif
    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        cost = prsd_search_estimation(ctx, core, pi, x, y, log2_cuw, log2_cuh, mv_temp);
    }
    mv[0] = mv_temp[0];
    mv[1] = mv_temp[1];
    if (mv_temp[0] == 0 && mv_temp[1] == 0)
    {
        return COM_UINT32_MAX;
    }
    return cost;
}
#endif
#if CIBC
double analyze_ibc_cu_chroma(ENC_CTX* ctx, ENC_CORE* core)
{
    ENC_PIBC* pi = &ctx->pibc;
    COM_MODE* cur_info = &core->mod_info_curr;
    COM_MODE* bst_info = &core->mod_info_best;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j;

    static s16  coef_blk[N_C][MAX_CU_DIM];
    static s16  resi[N_C][MAX_CU_DIM];

    int cu_width_log2 = cur_info->cu_width_log2;
    int cu_height_log2 = cur_info->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if SBT
    cur_info->sbt_info = 0;
#endif
#if ISTS
    int istsflag = 0;
    cur_info->ist_tu_flag = 0;
    cur_info->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if IST
    core->mod_info_curr.ist_tu_flag = 0;
#endif
#if INTERPF
    cur_info->inter_filter_flag = 0;
#endif
#if IBC_ENH
    cur_info->ibcpf_idx = 0;
#endif
#if IPC
    cur_info->ipc_flag = 0;
#endif
#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
#if IBC_ENH
    get_part_info(ctx->info.pic_width_in_scu, core->mod_info_curr.x_scu << 2, core->mod_info_curr.y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->mod_info_curr.x_scu << 2, core->mod_info_curr.y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr.x_scu << 2, mod_info_curr.y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif
#endif

    int found_available_ibc = 0;
    double cost_best = core->cost_best;
    double cost;
    pi->curr_bvr = 0;

    s16* mv;
    cur_info->cbvp_idx = 0;
    cur_info->cu_mode = MODE_IBC;
    cur_info->ibc_flag = 1;
    init_ibc_data(pi, core);

    mv = cur_info->mv[0];
    int valid = 0;
    if (has_cibc_mv(cur_info, &ctx->info, &ctx->map, ctx->tree_status, ctx->info.pic_header.ph_cibc_flag))
    {
        found_available_ibc = derive_cibc_mv(mv, cur_info, &ctx->info, &ctx->map);
    }

    if (found_available_ibc)
    {
        cur_info->cibc_flag = 1;
        found_available_ibc = 1;
        cost = pibc_residue_rdo_chroma(ctx, core);
        check_best_ibc_mode(core, pi, cost, &cost_best);

        /* reconstruct */
        int start_comp = (ctx->tree_status == TREE_L || ctx->tree_status == TREE_LC) ? Y_C : U_C;
        int num_comp = ctx->tree_status == TREE_LC ? 3 : (ctx->tree_status == TREE_L ? 1 : 2);
        for (j = start_comp; j < start_comp + num_comp; j++)
        {
            int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
            com_mcpy(coef_blk[j], bst_info->coef[j], sizeof(s16) * size_tmp);
        }
#if ST_CHROMA
        int use_secTrans[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 } };
        int use_alt4x4Trans[MAX_NUM_CHANNEL] = { 0, 0 };
#else
        int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for ibc coding
#endif
#if IST
        core->mod_info_best.slice_type = ctx->slice_type;
#endif
#if ISTS
        core->mod_info_best.ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if AWP
        core->mod_info_best.ph_awp_refine_flag = ctx->info.pic_header.ph_awp_refine_flag;
#endif
#if ST_CHROMA
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans
#if IST_CHROMA
            , ctx->tree_status
#endif        
        );
#else
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, 0);
#endif
        for (i = start_comp; i < start_comp + num_comp; i++)
        {
            int stride = (i == 0 ? cu_width : cu_width >> 1);
            com_recon(i == Y_C ? core->mod_info_best.tb_part : SIZE_2Nx2N, resi[i], bst_info->pred[i], core->mod_info_best.num_nz, i, stride, (i == 0 ? cu_height : cu_height >> 1), stride, bst_info->rec[i], bit_depth
#if SBT
                , bst_info->sbt_info
#endif
            );
        }
        return cost_best;
    }
    else
    {
        return MAX_COST;
    }
}
#endif

#if IBC_ENH
void check_ibc_pred_filter(ENC_CTX* ctx, ENC_CORE* core, double* cost_best)
{
    ENC_PIBC* pi = &ctx->pibc;
    COM_MODE* cur_info = &core->mod_info_curr;
    COM_MODE* bst_info = &core->mod_info_best;
    int cu_width_log2 = cur_info->cu_width_log2;
    int cu_height_log2 = cur_info->cu_height_log2;
    int bit_depth = ctx->info.bit_depth_internal;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    u8 bvp_idx = 0;
    u32 cy/*, cu, cv*/;
    double cost_y;
    double best_cost = MAX_COST;
    int best_bvp_idx = 0;
    int best_pf_idx = 0;
    int bit_cnt, shift = 0;
    s16 best_mv[MV_D] = { 0, 0 };
    s16* mv, * mvd, * mvp;
    int best_pf_bvr = 0;
    int best_pf_ist = 0;
    mv = cur_info->mv[0];
    mvd = cur_info->mvd[0];
    mvp = pi->mvp;

    const unsigned int lcu_width = ctx->info.max_cuwh;
    const int cu_pel_x = cur_info->x_pos;
    const int cu_pel_y = cur_info->y_pos;
    const int cu_x = cur_info->x_pos;
    const int cu_y = cur_info->y_pos;
    int log2_cuw = cu_width_log2;
    int log2_cuh = cu_height_log2;
    int roi_width = (1 << cu_width_log2);
    int roi_height = (1 << cu_height_log2);
    const int pic_width = ctx->info.pic_width;
    const int pic_height = ctx->info.pic_height;
    pel            predBuf[3][MAX_CU_DIM];
    pel(*pred)[MAX_CU_DIM] = predBuf;

    for (int bvp_idx = 0; bvp_idx < 1; bvp_idx++)
    {
        mv[MV_X] = bst_info->mv[0][MV_X] >> 2;
        mv[MV_Y] = bst_info->mv[0][MV_Y] >> 2;
        {
            int x = mv[MV_X];
            int y = mv[MV_Y];
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, x, y, lcu_width))
            {
                continue;
            }
#if ExtStrIBC||RBU_LIMIT_IBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + x;
                    int ref_start_y_in_pic1 = current_y1 + y;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }
#endif
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { x,y };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        continue;
                    }
                }
#endif
            }
#endif
        }
        cur_info->cbvp_idx = bst_info->cbvp_idx;
        cur_info->ist_tu_flag = bst_info->ist_tu_flag;
        cur_info->bvr_idx = bst_info->bvr_idx;
        pi->curr_bvr = bst_info->bvr_idx;
        {
            mv[MV_X] = mv[MV_X] << 2;
            mv[MV_Y] = mv[MV_Y] << 2;
            mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
            mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);
            SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
#if IBC_ABVR
            mv[MV_X] = (mv[MV_X] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
            mv[MV_Y] = (mv[MV_Y] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
#endif
#if IBC_BVP
            if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
            {
                cur_info->cbvp_idx = 0;
                mvp[MV_X] = 0;
                mvp[MV_Y] = 0;
                mvd[MV_X] = mv[MV_X];
                mvd[MV_Y] = mv[MV_Y];
            }
            else
            {
                mvp[MV_X] = core->bvp_cands[cur_info->cbvp_idx][MV_X] << 2;
                mvp[MV_Y] = core->bvp_cands[cur_info->cbvp_idx][MV_Y] << 2;
#if IBC_ABVR
                com_mv_rounding_s16(mvp[MV_X], mvp[MV_Y], &mvp[MV_X], &mvp[MV_Y], pi->curr_bvr + 2, pi->curr_bvr + 2);
#endif
                mvd[MV_X] = mv[MV_X] - mvp[MV_X];
                mvd[MV_Y] = mv[MV_Y] - mvp[MV_Y];
            }
        }
#endif
        com_IBC_mc(cu_x, cu_y, cu_width_log2, cu_height_log2, mv, pi->pic_unfiltered_rec, pred, CHANNEL_L, bit_depth);
#if IBC_APF
        int num_pf = MAX_NUM_IBC_PF;
#else
        const int num_pf = 3;
#endif
#if IBC_APF
        if (ctx->info.pic_header.ibc_type == 0)
        {
            num_pf = MAX_NUM_IBC_APF;
            if (!com_is_apf_valid(cu_x, cu_y, cu_width, cu_height, mv[MV_X], mv[MV_Y], ctx->tree_status, 2, pic_width, pic_height, ctx->info.log2_max_cuwh, ctx->map.map_scu))
            {
                cur_info->ibcpf_idx = 0;
                num_pf = 1;
            }
        }
#endif
        for (int pfIdx = 1; pfIdx <= num_pf - 1; pfIdx++)
        {
            cur_info->ibcpf_idx = pfIdx;
            memcpy(cur_info->pred[Y_C], predBuf[0], (cu_width * cu_height) * sizeof(pel));
            pel* y_org = pi->Yuv_org[Y_C] + cu_x + cu_y * pi->stride_org[Y_C];
#if IBC_APF
            if (ctx->info.pic_header.ibc_type == 0)
            {
                pred_ibc_filter(PIC_REC(ctx)/*, PIC_ORG(ctx)*/, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, cur_info
                    , cu_x, cu_y, cu_width, cu_height, bit_depth, CHANNEL_L, pfIdx);
            } else {
#endif
                pred_inter_filter(PIC_REC(ctx), ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, cur_info
                    , cu_x, cu_y, cu_width, cu_height, bit_depth, CHANNEL_L, pfIdx);
#if IBC_APF 
            }
#endif
            cy = calc_satd_16b(cu_width, cu_height, y_org, cur_info->pred[Y_C], pi->stride_org[Y_C], cu_width, bit_depth);
            cost_y = (double)cy;
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1, 0);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cost_y += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);
            if (cost_y < best_cost)
            {
                best_cost = cost_y;
                best_bvp_idx = cur_info->cbvp_idx;
                best_pf_idx = pfIdx;
                best_mv[MV_X] = mv[MV_X];
                best_mv[MV_Y] = mv[MV_Y];
                best_pf_ist = cur_info->ist_tu_flag;
                best_pf_bvr = cur_info->bvr_idx;
            }
        }
    }

    for (int bvp_idx = 0; bvp_idx < min(MAX_NUM_BVP, core->cnt_class_cands); bvp_idx++)
    {
        mv[MV_X] = core->bvp_cands[bvp_idx][MV_X];
        mv[MV_Y] = core->bvp_cands[bvp_idx][MV_Y];
        {
            int x = mv[MV_X];
            int y = mv[MV_Y];
            if (!is_bv_valid(ctx, cu_pel_x, cu_pel_y, roi_width, roi_height, log2_cuw, log2_cuh,
                pic_width, pic_height, x, y, lcu_width))
            {
                continue;
            }
#if ExtStrIBC||RBU_LIMIT_IBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { { 0, 0 }, { roi_width - 1, 0 }, { roi_width - 1, roi_height - 1 }, { 0, roi_height - 1 } };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + x;
                    int ref_start_y_in_pic1 = current_y1 + y;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }
#endif
#if RBU_LIMIT_IBC
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { x,y };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        continue;
                    }
                }
#endif
            }
#endif
        }
        cur_info->cbvp_idx = bvp_idx;
        cur_info->ist_tu_flag = 0;
        cur_info->bvr_idx = 0;
        pi->curr_bvr = 0;
        {
            mv[MV_X] = mv[MV_X] << 2;
            mv[MV_Y] = mv[MV_Y] << 2;
            mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
            mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);
            SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
#if IBC_ABVR
            mv[MV_X] = (mv[MV_X] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
            mv[MV_Y] = (mv[MV_Y] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
#endif
#if IBC_BVP
            if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
            {
                bvp_idx = 0;
                mvp[MV_X] = 0;
                mvp[MV_Y] = 0;
                mvd[MV_X] = mv[MV_X];
                mvd[MV_Y] = mv[MV_Y];
            }
            else
            {
                mvp[MV_X] = core->bvp_cands[bvp_idx][MV_X] << 2;
                mvp[MV_Y] = core->bvp_cands[bvp_idx][MV_Y] << 2;
#if IBC_ABVR
                com_mv_rounding_s16(mvp[MV_X], mvp[MV_Y], &mvp[MV_X], &mvp[MV_Y], pi->curr_bvr + 2, pi->curr_bvr + 2);
#endif
                mvd[MV_X] = mv[MV_X] - mvp[MV_X];
                mvd[MV_Y] = mv[MV_Y] - mvp[MV_Y];
            }
        }
#endif
        com_IBC_mc(cu_x, cu_y, cu_width_log2, cu_height_log2, mv, pi->pic_unfiltered_rec, pred, CHANNEL_L, bit_depth);
#if IBC_APF
        int num_pf = MAX_NUM_IBC_PF;
#else
        int num_pf = 3;
#endif
#if IBC_APF
        if (ctx->info.pic_header.ibc_type == 0)
        {
            num_pf = MAX_NUM_IBC_APF;
            if (!com_is_apf_valid(cu_x, cu_y, cu_width, cu_height, mv[MV_X], mv[MV_Y], ctx->tree_status, 2, pic_width, pic_height, ctx->info.log2_max_cuwh, ctx->map.map_scu))
            {
                cur_info->ibcpf_idx = 0;
                num_pf = 1;
            }
        }
#endif
        for (int pfIdx = 1; pfIdx <= num_pf - 1; pfIdx++)
        {
            cur_info->ibcpf_idx = pfIdx;
            memcpy(cur_info->pred[Y_C], predBuf[0], (cu_width * cu_height) * sizeof(pel));
            pel* y_org = pi->Yuv_org[Y_C] + cu_x + cu_y * pi->stride_org[Y_C];
#if IBC_APF
            if (ctx->info.pic_header.ibc_type == 0)
            {
                pred_ibc_filter(PIC_REC(ctx), ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, cur_info
                    , cu_x, cu_y, cu_width, cu_height, bit_depth, CHANNEL_L, pfIdx);
            } else {
#endif
                pred_inter_filter(PIC_REC(ctx), ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, cur_info
                    , cu_x, cu_y, cu_width, cu_height, bit_depth, CHANNEL_L, pfIdx);
#if IBC_APF
            }
#endif
            cy = calc_satd_16b(cu_width, cu_height, y_org, cur_info->pred[Y_C], pi->stride_org[Y_C], cu_width, bit_depth);
            cost_y = (double)cy;
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_ibc(ctx, core, ctx->info.pic_header.slice_type, 1, 0);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cost_y += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);
            if (cost_y < best_cost)
            {
                best_cost = cost_y;
                best_bvp_idx = bvp_idx;
                best_pf_idx = pfIdx;
                best_mv[MV_X] = mv[MV_X];
                best_mv[MV_Y] = mv[MV_Y];
                best_pf_ist = cur_info->ist_tu_flag;
                best_pf_bvr = cur_info->bvr_idx;
            }
        }
    }
    if (best_mv[MV_X] != 0 || best_mv[MV_Y] != 0)
    {
        bvp_idx = best_bvp_idx;
        mv[MV_X] = best_mv[MV_X] >> 2;
        mv[MV_Y] = best_mv[MV_Y] >> 2;
        pi->mv[MV_X] = mv[MV_X];
        pi->mv[MV_Y] = mv[MV_Y];
        mv[MV_X] = mv[MV_X] << 2;
        mv[MV_Y] = mv[MV_Y] << 2;
        mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
        mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);
        SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
#if IBC_BVP
        pi->curr_bvr = best_pf_bvr;
#else
        if (ctx->info.sqh.abvr_enable_flag && (mv[MV_X] % 16 == 0) && (mv[MV_Y] % 16 == 0))
        {
            pi->curr_bvr = 2;
        }
#endif
#if IBC_ABVR
        mv[MV_X] = (mv[MV_X] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
        mv[MV_Y] = (mv[MV_Y] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
#endif
#if IBC_BVP
        if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
        {
            bvp_idx = 0;
            mvp[MV_X] = 0;
            mvp[MV_Y] = 0;
            mvd[MV_X] = mv[MV_X];
            mvd[MV_Y] = mv[MV_Y];
        }
        else
        {
            mvp[MV_X] = core->bvp_cands[bvp_idx][MV_X] << 2;
            mvp[MV_Y] = core->bvp_cands[bvp_idx][MV_Y] << 2;
#if IBC_ABVR
            com_mv_rounding_s16(mvp[MV_X], mvp[MV_Y], &mvp[MV_X], &mvp[MV_Y], pi->curr_bvr + 2, pi->curr_bvr + 2);
#endif
            mvd[MV_X] = mv[MV_X] - mvp[MV_X];
            mvd[MV_Y] = mv[MV_Y] - mvp[MV_Y];
        }
        cur_info->cbvp_idx = bvp_idx;
        cur_info->ibcpf_idx = best_pf_idx;
        cur_info->cnt_hbvp_cands = core->mod_info_curr.cnt_hbvp_cands;
        cur_info->bvr_idx = best_pf_bvr;
#else
        mvd[MV_X] = mv[MV_X];
        mvd[MV_Y] = mv[MV_Y];
#endif
#if IBC_ABVR
        pi->mot_bits = get_bv_cost_bits(mvd[MV_X] >> (pi->curr_bvr + 2), mvd[MV_Y] >> (pi->curr_bvr + 2));
#else
        pi->mot_bits = get_bv_cost_bits(mvd[MV_X] >> 2, mvd[MV_Y] >> 2);
#endif
#if IBC_BVP
        pi->mvp_idx = bvp_idx;
#else
        pi->mvp_idx = mvp_idx;
#endif
#if IST && !ISTS
        core->mod_info_curr.ist_tu_flag = 0;
#endif
#if INTERPF
        core->mod_info_curr.inter_filter_flag = 0;
#endif
#if IPC
        core->mod_info_curr.ipc_flag = 0;
#endif
#if ISTS
        cur_info->ist_tu_flag = best_pf_ist;
#endif
        double cost = pibc_residue_rdo(ctx, core, 0);
        check_best_ibc_mode(core, pi, cost, cost_best);
    }
}
#endif

double analyze_ibc_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PIBC *pi = &ctx->pibc;
    COM_MODE *cur_info = &core->mod_info_curr;
    COM_MODE *bst_info = &core->mod_info_best;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j;

    static s16  coef_blk[N_C][MAX_CU_DIM];
    static s16  resi[N_C][MAX_CU_DIM];

    int cu_width_log2 = cur_info->cu_width_log2;
    int cu_height_log2 = cur_info->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if SBT
    cur_info->sbt_info = 0;
#endif
#if ISTS
    int istsflag = 0;
    cur_info->ist_tu_flag = 0;
    cur_info->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
#if IBC_ENH
    get_part_info(ctx->info.pic_width_in_scu, core->mod_info_curr.x_scu << 2, core->mod_info_curr.y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->mod_info_curr.x_scu << 2, core->mod_info_curr.y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, core->mod_info_curr.x_scu << 2, core->mod_info_curr.y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif
#endif

    int num_amvr = MAX_NUM_MVR;

    {
        num_amvr = 1; /* only allow 1/4 pel of resolution */
    }
    //pi->curr_mvr = 0;

#if IBC_ABVR
    int num_abvr = MAX_NUM_BVR;

    if (ctx->info.sqh.abvr_enable_flag)
    {
        num_abvr = MAX_NUM_BVR;
    }
    else
    {
        num_abvr = 1;
    }
#endif

    int allow_affine = 0;// disable affine for IBC

#if IBC_ABVR
    u8 found_available_ibc = 0;
    double cost_best = core->cost_best;
    u8 skip_me = 0;
    u32 mecost_fast;
    s16 mv_fast[MV_D];
#else
    u32 mecost, best_mecost;
    s16*mvp, *mv, *mvd;
    u8 mvp_idx = 0;
    double cost, cost_best = core->cost_best;
    u8 found_available_ibc = 0;
#endif
#if ISC_RSD && !RSD_OPT
    mv_fast[0] = 0;
    mv_fast[1] = 0;
#endif
    u8 bvp_idx = 0;
    u8 bvp_idx_fast = 0;
#if IBC_ABVR
    mecost_fast = COM_UINT32_MAX;
#if IBC_ENH
    if (ctx->info.pic_header.ibc_type == 1)
    {
        num_abvr = 1;
    }
#endif
#if IBC_BVP
    int bvr_idx = 0;
    for (bvr_idx = 0; bvr_idx < num_abvr; bvr_idx++)
#else
    pi->curr_bvr = 0;
#endif
    {
            u32 mecost, best_mecost;
            s16*mvp, *mv, *mvd;
            u8 mvp_idx = 0;
            double cost;
            found_available_ibc = 0;
#endif
            cur_info->cu_mode = MODE_IBC;
            cur_info->ibc_flag = 1;
            init_ibc_data(pi, core);

            mv = cur_info->mv[0];
            mvd = cur_info->mvd[0];
            best_mecost = COM_UINT32_MAX;

            mvp = pi->mvp;

            mvp[MV_X] = 0;
            mvp[MV_Y] = 0;

#if IBC_BVP
            // initialize pmv and refi
            for (i = 0; i < MAX_NUM_BVP; i++)
            {
                core->bvp_cands[i][MV_X] = 0;
                core->bvp_cands[i][MV_Y] = 0;
            }
            com_derive_bvp_list(ctx->slice_type, ctx->info.sqh.num_of_hbvp_cand, &core->mod_info_curr, core->block_motion_cands, core->cnt_hbvp_cands, core->bvp_cands, &core->cnt_class_cands, ctx->info.pic_width, ctx->info.pic_height, ctx->map.map_scu
#if USE_SP
                , ctx->map.map_usp
#endif
                , ctx->info.log2_max_cuwh, ctx->map.map_mv);
#endif

            /* motion search ********************/
#if !IBC_ABVR
            u8 skip_me = 0;
#endif
            //WYQ: Add the code to check if this block is already encoded, if yes set skip_me to be true
#if ISC_RSD && !RSD_OPT
            int rsd_idx = (ctx->info.sqh.rsd_enable_flag && ctx->info.pic_header.sp_pic_flag && !bvr_idx) ? 2 : 1;

#if IBC_SIZE_EXT
            if (cu_width > 32 || cu_height > 32)
            {
                rsd_idx = 1;
            }
#endif
            for (int sp_rsd_flag = 0; sp_rsd_flag < rsd_idx; sp_rsd_flag++)
            {
                core->mod_info_curr.sp_rsd_flag = sp_rsd_flag;
                if (sp_rsd_flag)
                {
                    for (int intra_row_rsd_flag = 0; intra_row_rsd_flag < 2; intra_row_rsd_flag++)
                    {
                        if (!is_cu_nz(core->mod_info_best.num_nz))
                        {
                            break;
                        }
                        core->mod_info_curr.intra_row_rsd_flag = intra_row_rsd_flag;
#if IBC_BVP
                        mecost = pibc_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mvp, mv, &bvp_idx);
#else
                        mecost = pibc_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mvp, mv);
#endif
                        if (mv[MV_X] != 0 || mv[MV_Y] != 0)
                        {
                            pi->mv[MV_X] = mv[MV_X];
                            pi->mv[MV_Y] = mv[MV_Y];

#if SCC_CROSSINFO_ULTILIZE
                            core->ibc_mv[0] = mv[0];
                            core->ibc_mv[1] = mv[1];
#endif
                            mv[MV_X] = mv[MV_X] << 2;
                            mv[MV_Y] = mv[MV_Y] << 2;
                            mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
                            mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);

                            SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
                            derive_rsd_sv_info(cur_info, mv[MV_X] >> 2, mv[MV_Y] >> 2, core->n_offset_num, core->n_recent_offset);
#if ISTS
                            int ists_all = (ctx->info.sqh.ists_enable_flag && cur_info->ph_ists_enable_flag && core->mod_info_curr.tb_part == 0 && cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
                            for (istsflag = 0; istsflag < ists_all; istsflag++)
                            {
                                cur_info->ist_tu_flag = istsflag;
                                cost = pibc_residue_rdo(ctx, core, 0);
                                check_best_ibc_mode(core, pi, cost, &cost_best);
                            }
#else
                            //cost = cost_ibc = pibc_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred, pi->coef, mvp_idx, pi->mv[1]);
                            cost = pibc_residue_rdo(ctx, core, 0);

                            check_best_ibc_mode(core, pi, cost, &cost_best);
#endif
                        }
                    }

                }
                else
                {
#endif
                    if (skip_me)
                    {
#if IBC_ABVR
                        mecost = mecost_fast;
                        mv[MV_X] = mv_fast[MV_X];
                        mv[MV_Y] = mv_fast[MV_Y];
#endif
#if IBC_BVP
                        bvp_idx = bvp_idx_fast;
#endif
                    }
                    else
                    {
#if IBC_BVP
                        mecost = pibc_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mvp, mv, &bvp_idx);
#else
                        mecost = pibc_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mvp, mv);
#endif
#if IBC_ABVR
                        mecost_fast = mecost;
                        mv_fast[MV_X] = mv[MV_X];
                        mv_fast[MV_Y] = mv[MV_Y];
#endif
                    }
                    if (mv[MV_X] != 0 || mv[MV_Y] != 0)
                    {
                        found_available_ibc = 1;
#if IBC_ABVR
                        skip_me = 1;
#endif
                        pi->mv[MV_X] = mv[MV_X];
                        pi->mv[MV_Y] = mv[MV_Y];
                        if (mecost < best_mecost)
                        {
                            best_mecost = mecost;
                        }

#if SCC_CROSSINFO_ULTILIZE
                        core->ibc_mv[0] = mv[0];
                        core->ibc_mv[1] = mv[1];
#endif

                        mv[MV_X] = mv[MV_X] << 2;
                        mv[MV_Y] = mv[MV_Y] << 2;
                        mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
                        mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);

                        SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
#if IBC_BVP
                        pi->curr_bvr = bvr_idx == 0 ? 0 : 2;
#else
                        if (ctx->info.sqh.abvr_enable_flag && (mv[MV_X] % 16 == 0) && (mv[MV_Y] % 16 == 0))
                        {
                            pi->curr_bvr = 2;
                        }
#endif

#if IBC_ABVR
                        mv[MV_X] = (mv[MV_X] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
                        mv[MV_Y] = (mv[MV_Y] >> (pi->curr_bvr + 2)) << (pi->curr_bvr + 2);
#endif
#if IBC_BVP
                        if (core->cnt_class_cands == 0 || core->cnt_hbvp_cands < 2)
                        {
                            bvp_idx = 0;
                            mvp[MV_X] = 0;
                            mvp[MV_Y] = 0;
                            mvd[MV_X] = mv[MV_X];
                            mvd[MV_Y] = mv[MV_Y];
                        }
                        else
                        {
                            mvp[MV_X] = core->bvp_cands[bvp_idx][MV_X] << 2;
                            mvp[MV_Y] = core->bvp_cands[bvp_idx][MV_Y] << 2;
#if IBC_ABVR
                            com_mv_rounding_s16(mvp[MV_X], mvp[MV_Y], &mvp[MV_X], &mvp[MV_Y], pi->curr_bvr + 2, pi->curr_bvr + 2);
#endif
                            mvd[MV_X] = mv[MV_X] - mvp[MV_X];
                            mvd[MV_Y] = mv[MV_Y] - mvp[MV_Y];
                        }

                        cur_info->cbvp_idx = bvp_idx;
                        cur_info->cnt_hbvp_cands = core->mod_info_curr.cnt_hbvp_cands;
#else
                        mvd[MV_X] = mv[MV_X];
                        mvd[MV_Y] = mv[MV_Y];
#endif
#if IBC_ABVR
                        pi->mot_bits = get_bv_cost_bits(mvd[MV_X] >> (pi->curr_bvr + 2), mvd[MV_Y] >> (pi->curr_bvr + 2));
#else
                        pi->mot_bits = get_bv_cost_bits(mvd[MV_X] >> 2, mvd[MV_Y] >> 2);
#endif

#if IBC_BVP
                        pi->mvp_idx = bvp_idx;
#else
                        pi->mvp_idx = mvp_idx;
#endif

#if IST && !ISTS
                        core->mod_info_curr.ist_tu_flag = 0;
#endif
#if INTERPF
                        core->mod_info_curr.inter_filter_flag = 0;
#endif
#if IPC
                        core->mod_info_curr.ipc_flag = 0;
#endif
#if ISTS
                        int ists_all = (ctx->info.sqh.ists_enable_flag && cur_info->ph_ists_enable_flag && core->mod_info_curr.tb_part == 0 && cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
                        for (istsflag = 0; istsflag < ists_all; istsflag++)
                        {
#if IBC_ENH
                            cur_info->ibcpf_idx = 0;
#endif
                            cur_info->ist_tu_flag = istsflag;
                            cost = pibc_residue_rdo(ctx, core, 0);
                            check_best_ibc_mode(core, pi, cost, &cost_best);
                        }
#else
                        //cost = cost_ibc = pibc_residue_rdo(ctx, core, x, y, log2_cuw, log2_cuh, pi->pred, pi->coef, mvp_idx, pi->mv[1]);
                        cost = pibc_residue_rdo(ctx, core, 0);

                        check_best_ibc_mode(core, pi, cost, &cost_best);
#endif
                    }
#if ISC_RSD && !RSD_OPT
                }
            }
#endif
#if IBC_ABVR
    }
#endif
#if IBC_ENH
#if IBC_APF
    if (ctx->info.pic_header.ph_ibc_pf_flag && found_available_ibc
#if ISC_RSD && !RSD_OPT
        && core->mod_info_best.sp_rsd_flag == 0
#endif
        )
#else
    if (ctx->info.pic_header.ibc_type == 1 && found_available_ibc)
#endif
    {
        check_ibc_pred_filter(ctx, core, &cost_best);
    }
#endif
    if (found_available_ibc)
    {
        /* reconstruct */
        int start_comp = (ctx->tree_status == TREE_L || ctx->tree_status == TREE_LC) ? Y_C : U_C;
        int num_comp = ctx->tree_status == TREE_LC ? 3 : (ctx->tree_status == TREE_L ? 1 : 2);
        for (j = start_comp; j < start_comp + num_comp; j++)
        {
            int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
            com_mcpy(coef_blk[j], bst_info->coef[j], sizeof(s16) * size_tmp);
        }
#if ST_CHROMA
        int use_secTrans[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 } }; // secondary transform is disabled for ibc coding
        int use_alt4x4Trans[MAX_NUM_CHANNEL] = { 0, 0 };
#else
        int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for ibc coding
#endif
#if IST
        core->mod_info_best.slice_type = ctx->slice_type;
#endif
#if ISTS
        core->mod_info_best.ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if AWP
        core->mod_info_best.ph_awp_refine_flag = ctx->info.pic_header.ph_awp_refine_flag;
#endif
#if ST_CHROMA
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans
#if IST_CHROMA
            , ctx->tree_status
#endif
        );
#else
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, 0);
#endif
        for (i = start_comp; i < start_comp + num_comp; i++)
        {
            int stride = (i == 0 ? cu_width : cu_width >> 1);
            com_recon(i == Y_C ? core->mod_info_best.tb_part : SIZE_2Nx2N, resi[i], bst_info->pred[i], core->mod_info_best.num_nz, i, stride, (i == 0 ? cu_height : cu_height >> 1), stride, bst_info->rec[i], bit_depth
#if SBT
                , bst_info->sbt_info
#endif
            );
        }

        return cost_best;
    }
    else
    {
        return MAX_COST;
    }
}
#if RSD_OPT
double analyze_rsd_cu(ENC_CTX* ctx, ENC_CORE* core)
{
    ENC_PIBC* pi = &ctx->pibc;
    COM_MODE* cur_info = &core->mod_info_curr;
    COM_MODE* bst_info = &core->mod_info_best;
    int bit_depth = ctx->info.bit_depth_internal;
    static s16  coef_blk[N_C][MAX_CU_DIM];
    static s16  resi[N_C][MAX_CU_DIM];
    int cu_width_log2 = cur_info->cu_width_log2;
    int cu_height_log2 = cur_info->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int j, i;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if SBT
    cur_info->sbt_info = 0;
#endif
#if ISTS
    int istsflag = 0;
    cur_info->ist_tu_flag = 0;
    cur_info->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif
#if INTERPF
    core->mod_info_curr.inter_filter_flag = 0;
#endif
#if IPC
    core->mod_info_curr.ipc_flag = 0;
#endif
    double cost_best = core->cost_best;
    double cost, mecost;
    int found_available_cu = 0;
    s16* mv;
    cur_info->cu_mode = MODE_IBC;
    init_ibc_data(pi, core);
    mv = cur_info->mv[0];
    cur_info->sp_rsd_flag = 1;
    for (int intra_row_rsd_flag = 0; intra_row_rsd_flag < 2; intra_row_rsd_flag++)
    {
        cur_info->intra_row_rsd_flag = intra_row_rsd_flag;
        mecost = prsd_me_search(ctx, core, pi, cur_info->x_pos, cur_info->y_pos, cu_width_log2, cu_height_log2, mv);
        if (mv[MV_X] != 0 || mv[MV_Y] != 0)
        {
            found_available_cu = 1;
            mv[MV_X] = mv[MV_X] << 2;
            mv[MV_Y] = mv[MV_Y] << 2;
            mv[MV_X] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_X]);
            mv[MV_Y] = COM_CLIP3(COM_INT16_MIN, COM_INT16_MAX, mv[MV_Y]);
            SET_REFI(cur_info->refi, REFI_INVALID, REFI_INVALID);
            derive_rsd_sv_info(cur_info, mv[MV_X] >> 2, mv[MV_Y] >> 2, core->n_offset_num, core->n_recent_offset);
            int ists_all = (ctx->info.sqh.ists_enable_flag && cur_info->ph_ists_enable_flag && core->mod_info_curr.tb_part == 0 && cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
            for (istsflag = 0; istsflag < ists_all; istsflag++)
            {
                cur_info->ist_tu_flag = istsflag;
                cost = prsd_residue_rdo(ctx, core, 0);
                if (cost < cost_best)
                    check_best_rsd_mode(core, pi, cost, &cost_best);
            }
        }
    }
    if (found_available_cu)
    {
        /* reconstruct */
        int start_comp = (ctx->tree_status == TREE_L || ctx->tree_status == TREE_LC) ? Y_C : U_C;
        int num_comp = ctx->tree_status == TREE_LC ? 3 : (ctx->tree_status == TREE_L ? 1 : 2);
        for (j = start_comp; j < start_comp + num_comp; j++)
        {
            int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
            com_mcpy(coef_blk[j], bst_info->coef[j], sizeof(s16) * size_tmp);
        }
#if ST_CHROMA
        int use_secTrans[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 } }; // secondary transform is disabled for ibc coding
        int use_alt4x4Trans[MAX_NUM_CHANNEL] = { 0, 0 };
#else
        int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for ibc coding
#endif
#if IST
        core->mod_info_best.slice_type = ctx->slice_type;
#endif
#if ISTS
        core->mod_info_best.ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if AWP
        core->mod_info_best.ph_awp_refine_flag = ctx->info.pic_header.ph_awp_refine_flag;
#endif
#if ST_CHROMA
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans
#if IST_CHROMA
            , ctx->tree_status
#endif  
        );
#else
        com_itdq_yuv(&core->mod_info_best, coef_blk, resi, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, 0);
#endif
        for (i = start_comp; i < start_comp + num_comp; i++)
        {
            int stride = (i == 0 ? cu_width : cu_width >> 1);
            com_recon(i == Y_C ? core->mod_info_best.tb_part : SIZE_2Nx2N, resi[i], bst_info->pred[i], core->mod_info_best.num_nz, i, stride, (i == 0 ? cu_height : cu_height >> 1), stride, bst_info->rec[i], bit_depth
#if SBT
                , bst_info->sbt_info
#endif
            );
        }

        return cost_best;
    }
    else
    {
        return MAX_COST;
    }
}
#endif
int pibc_init_frame(ENC_CTX *ctx)
{
    ENC_PIBC *pi;
    COM_PIC     *pic;
    pi = &ctx->pibc;
    pic = pi->pic_org = PIC_ORG(ctx);
    pi->Yuv_org[Y_C] = pic->y;
    pi->Yuv_org[U_C] = pic->u;
    pi->Yuv_org[V_C] = pic->v;
    pi->stride_org[Y_C] = pic->stride_luma;
    pi->stride_org[U_C] = pic->stride_chroma;
    pi->stride_org[V_C] = pic->stride_chroma;

    pic = pi->pic_unfiltered_rec = ctx->ibc_unfiltered_rec_pic;
    pi->unfiltered_rec[Y_C] = pic->y;
    pi->unfiltered_rec[U_C] = pic->u;
    pi->unfiltered_rec[V_C] = pic->v;

    pi->s_unfiltered_rec[Y_C] = pic->stride_luma;
    pi->s_unfiltered_rec[U_C] = pic->stride_chroma;
    pi->s_unfiltered_rec[V_C] = pic->stride_chroma;

    pi->slice_type = ctx->slice_type;

    pi->refp = ctx->refp;

    pi->bit_depth = ctx->info.bit_depth_internal;
    //pi->map_mv = ctx->map.map_mv;
    pi->pic_width_in_scu = ctx->info.pic_width_in_scu;
    //size = sizeof(pel) * MAX_CU_DIM;
    //com_mset(pi->pred_buf, 0, size);
    //size = sizeof(pel) * N_C * MAX_CU_DIM;
    //com_mset(pi->unfiltered_rec_buf, 0, size);
    //com_mset(pi->filtered_rec_buf, 0, size);

    init_LOG_LUT(pi);
    return COM_OK;
}

void reset_ibc_search_range(ENC_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh)
{
    int hashHitRatio = 0;
    ctx->pibc.search_range_x = ctx->param.ibc_search_range_x;
    ctx->pibc.search_range_y = ctx->param.ibc_search_range_y;

    hashHitRatio = get_hash_hit_ratio(ctx, ctx->ibc_hash_handle, cu_x, cu_y, log2_cuw, log2_cuh); // in percent

    if (hashHitRatio < 5) // 5%
    {
        ctx->pibc.search_range_x >>= 1;
        ctx->pibc.search_range_y >>= 1;
    }
}

int pibc_init_lcu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PIBC *pi;

    pi = &ctx->pibc;
    pi->lambda_mv = (u32)floor(65536.0 * ctx->sqrt_lambda[0]);
    pi->qp_y = core->qp_y;
    pi->qp_u = core->qp_u;
    pi->qp_v = core->qp_v;

    return COM_OK;
}

int pibc_set_complexity(ENC_CTX *ctx, int complexity)
{
    ENC_PIBC *pi;

    pi = &ctx->pibc;

    /* default values *************************************************/
    pi->search_range_x = ctx->param.ibc_search_range_x;
    pi->search_range_y = ctx->param.ibc_search_range_y;

    pi->complexity = complexity;
    return COM_OK;
}

double pibc_residue_rdo_chroma(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PIBC *pi = &ctx->pibc;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    s16(*coef)[MAX_CU_DIM] = mod_info_curr->coef;
    s16(*mv)[MV_D] = mod_info_curr->mv;
    s8 *refi = mod_info_curr->refi;
    pel(*pred)[MAX_CU_DIM] = mod_info_curr->pred;
    int bit_depth = ctx->info.bit_depth_internal;
    int(*num_nz_coef)[N_C], tnnz, width[N_C], height[N_C], log2_w[N_C], log2_h[N_C];;
    pel(*rec)[MAX_CU_DIM];
    s64    dist[2][N_C];
    double cost, cost_best = MAX_COST;
    int    cbf_best[N_C];
    int    bit_cnt;
    int    i, cbf_y, cbf_u, cbf_v;
    pel   *org[N_C];
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if ST_CHROMA
    int use_secTrans[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0, 0, 0, 0 },{ 0, 0, 0, 0 } }; // secondary transform is disabled for inter coding
    int use_alt4x4Trans[MAX_NUM_CHANNEL] = { 0,0 };
#else
    int use_secTrans[MAX_NUM_TB] = { 0, 0, 0, 0 }; // secondary transform is disabled for inter coding
    int use_alt4x4Trans = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if DEST_PH
    mod_info_curr->ph_dest_enable_flag = ctx->info.pic_header.ph_dest_enable_flag;
#endif

    rec = mod_info_curr->rec;
    num_nz_coef = mod_info_curr->num_nz;
    width[Y_C] = 1 << cu_width_log2;
    height[Y_C] = 1 << cu_height_log2;
    width[U_C] = width[V_C] = 1 << (cu_width_log2 - 1);
    height[U_C] = height[V_C] = 1 << (cu_height_log2 - 1);
    log2_w[Y_C] = cu_width_log2;
    log2_h[Y_C] = cu_height_log2;
    log2_w[U_C] = log2_w[V_C] = cu_width_log2 - 1;
    log2_h[U_C] = log2_h[V_C] = cu_height_log2 - 1;
    org[Y_C] = pi->Yuv_org[Y_C] + (y * pi->stride_org[Y_C]) + x;
    org[U_C] = pi->Yuv_org[U_C] + ((y >> 1) * pi->stride_org[U_C]) + (x >> 1);
    org[V_C] = pi->Yuv_org[V_C] + ((y >> 1) * pi->stride_org[V_C]) + (x >> 1);

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
#if CIBC
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->pb_part, &mod_info_curr->pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &mod_info_curr->tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, mod_info_curr->tb_part, &mod_info_curr->tb_info);
#endif
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif
#endif

#if !CIBC
    /* prepare MV */
    int luma_scup = mod_info_curr->scup + PEL2SCU(mod_info_curr->cu_width - 1) + PEL2SCU(mod_info_curr->cu_height - 1) * ctx->info.pic_width_in_scu;
    if (MCU_GET_IBC(ctx->map.map_scu[luma_scup]) == 0)
    {
        return MAX_COST;
    }
    for (i = 0; i < 1; i++)
    {
        refi[i] = ctx->map.map_refi[luma_scup][i];
        mv[i][MV_X] = ctx->map.map_mv[luma_scup][i][MV_X];
        mv[i][MV_Y] = ctx->map.map_mv[luma_scup][i][MV_Y];
    }
    if (mv[0][MV_X] == 0 && mv[0][MV_Y] == 0)
    {
        return MAX_COST;
    }
#endif

    /* chroma MC */
    assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
    com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mv[0], pi->pic_unfiltered_rec, pred, ctx->tree_status, bit_depth);

    /* get residual */
    enc_diff_pred(x, y, cu_width_log2, cu_height_log2, pi->pic_org, pred, resi_t);
    memset(dist, 0, sizeof(s64) * 2 * N_C);
    for (i = 1; i < N_C; i++)
    {
        dist[0][i] = enc_ssd_16b(log2_w[i], log2_h[i], pred[i], org[i], width[i], pi->stride_org[i], bit_depth);
    }

    /* test all zero case */
    memset(cbf_best, 0, sizeof(int) * N_C);
    memset(num_nz_coef, 0, sizeof(int) * MAX_NUM_TB * N_C);
    assert(mod_info_curr->tb_part == SIZE_2Nx2N);
    cost_best = (double)((dist[0][U_C] + dist[0][V_C]) * ctx->dist_chroma_weight[0]);

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
#if CIBC
    enc_bit_est_ibc_chroma(ctx, core, coef);
#else
    encode_coef(&core->bs_temp, coef, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, mod_info_curr->cu_mode, mod_info_curr, ctx->tree_status, ctx
#if CUDQP
        , core->qp_y
#endif
    ); // only count coeff bits for chroma tree
#endif
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);

    /* transform and quantization */
    tnnz = enc_tq_yuv_nnz(ctx, core, mod_info_curr, coef, resi_t, pi->slice_type, 0, use_secTrans, use_alt4x4Trans, refi, mv, 0);

    if (tnnz)
    {
        com_itdq_yuv(mod_info_curr, coef, resi_t, ctx->wq, cu_width_log2, cu_height_log2, pi->qp_y, pi->qp_u, pi->qp_v, bit_depth, use_secTrans, use_alt4x4Trans
#if IST_CHROMA
            , ctx->tree_status
#endif        
        );
        for (i = 1; i < N_C; i++)
        {
            com_recon(i == Y_C ? mod_info_curr->tb_part : SIZE_2Nx2N, resi_t[i], pred[i], num_nz_coef, i, width[i], height[i], width[i], rec[i], bit_depth
#if SBT
#if IST_SBT_IBC
				, mod_info_curr->sbt_info
#else
                , 0
#endif
#endif
            );

            if (is_cu_plane_nz(num_nz_coef, i))
            {
                dist[1][i] = enc_ssd_16b(log2_w[i], log2_h[i], rec[i], org[i], width[i], pi->stride_org[i], bit_depth);
            }
            else
            {
                dist[1][i] = dist[0][i];
            }
        }

        cbf_y = is_cu_plane_nz(num_nz_coef, Y_C) > 0 ? 1 : 0;
        cbf_u = is_cu_plane_nz(num_nz_coef, U_C) > 0 ? 1 : 0;
        cbf_v = is_cu_plane_nz(num_nz_coef, V_C) > 0 ? 1 : 0;

        cost = (double)((dist[cbf_u][U_C] + dist[cbf_v][V_C]) * ctx->dist_chroma_weight[0]);

        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
#if CIBC
        enc_bit_est_ibc_chroma(ctx, core, coef);
#else
        encode_coef(&core->bs_temp, coef, mod_info_curr->cu_width_log2, mod_info_curr->cu_height_log2, mod_info_curr->cu_mode, mod_info_curr, ctx->tree_status, ctx
#if CUDQP
            , core->qp_y
#endif
        ); // only count coeff bits for chroma tree
#endif
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
        if (cost < cost_best)
        {
            cost_best = cost;
            cbf_best[Y_C] = cbf_y;
            cbf_best[U_C] = cbf_u;
            cbf_best[V_C] = cbf_v;
            SBAC_STORE(core->s_temp_best, core->s_temp_run);
        }

    }

    /* save */
    for (i = 0; i < N_C; i++)
    {
        int size_tmp = (cu_width * cu_height) >> (i == 0 ? 0 : 2);
        if (cbf_best[i] == 0)
        {
            cu_plane_nz_cln(core->mod_info_best.num_nz, i);
            com_mset(core->mod_info_best.coef[i], 0, sizeof(s16) * size_tmp);
            com_mcpy(core->mod_info_best.rec[i], pred[i], sizeof(s16) * size_tmp);
        }
        else
        {
            cu_plane_nz_cpy(core->mod_info_best.num_nz, num_nz_coef, i);
            com_mcpy(core->mod_info_best.coef[i], coef[i], sizeof(s16) * size_tmp);
            com_mcpy(core->mod_info_best.rec[i], rec[i], sizeof(s16) * size_tmp);
        }
    }

    return cost_best;
}
#endif