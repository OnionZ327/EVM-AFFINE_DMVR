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
#if USE_SP    
#include "com_usp.h"
extern void* get_sp_instance(COM_SP_INPUT* input, u8 isEncoding);
extern void  sm_pic_reset(void* sp_encoder, pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec);
extern u8    sm_mode_rdcost(void* sp_encoder, COM_SP_CODING_UNIT * cur_sp_info, double*  min_rdcost, double*  distorion
    , ENC_CTX *ctx
);
extern void  enc_bit_est_cs2(int log2_width, int log2_height, COM_BSW *bs, COM_SP_CODING_UNIT* cur_sp_info
    , ENC_CTX *ctx
    , int x, int y
);
#endif
#include <math.h>

int pintra_init_frame(ENC_CTX * ctx)
{
    ENC_PINTRA * pi;
    COM_PIC     * pic;
    pi = &ctx->pintra;
    pic = PIC_ORG(ctx);
    pi->addr_org[Y_C] = pic->y;
    pi->addr_org[U_C] = pic->u;
    pi->addr_org[V_C] = pic->v;
    pi->stride_org[Y_C] = pic->stride_luma;
    pi->stride_org[U_C] = pic->stride_chroma;
    pi->stride_org[V_C] = pic->stride_chroma;

    pic = PIC_REC(ctx);
    pi->addr_rec_pic[Y_C] = pic->y;
    pi->addr_rec_pic[U_C] = pic->u;
    pi->addr_rec_pic[V_C] = pic->v;
    pi->stride_rec[Y_C] = pic->stride_luma;
    pi->stride_rec[U_C] = pic->stride_chroma;
    pi->stride_rec[V_C] = pic->stride_chroma;
    pi->slice_type = ctx->slice_type;
    pi->bit_depth = ctx->info.bit_depth_internal;

#if USE_SP
    if (ctx->param.sp_enable_flag|| ctx->param.evs_ubvs_enable_flag)
    {
        COM_SP_INPUT sp_input_pmt;
        sp_input_pmt.chroma_format = (&ctx->param)->chroma_format;
        sp_input_pmt.img_height    = (&ctx->param)->pic_height;
        sp_input_pmt.img_width     = (&ctx->param)->pic_width;
        sp_input_pmt.sample_bit_depth = ctx->info.bit_depth_internal;
        sp_input_pmt.max_cu_height = ctx->param.ctu_size;
        sp_input_pmt.max_cu_width = ctx->param.ctu_size;
        sp_input_pmt.y_stride      = pi->stride_org[Y_C];
        sp_input_pmt.recy_stride   = pi->stride_rec[Y_C];
        sp_input_pmt.c_stride      = pi->stride_org[U_C];
        sp_input_pmt.recc_stride   = pi->stride_rec[U_C];
        if (ctx->sp_encoder == NULL)
        {
            ctx->sp_encoder = get_sp_instance(&sp_input_pmt, TRUE);
        }
        sm_pic_reset(ctx->sp_encoder, pi->addr_org[Y_C], pi->addr_org[U_C], pi->addr_org[V_C], pi->addr_rec_pic[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C]);

    }
#endif
    return COM_OK;
}

int pintra_init_lcu(ENC_CTX * ctx, ENC_CORE * core)
{
    return COM_OK;
}
#if ECCPM
void caculate_eccpm_param(ENC_CTX *ctx, int x, int y,int cu_width, int cu_height,pel* pred,pel* pred_template,int avail_cu,int chroma_type,pel* rec_u,int u_stride,long long* a0,long long* a1,long long* a2,long long* a3,pel(*A)[MAX_CU_DIM], pel *YC,int param_num)
{
    ENC_PINTRA *pi = &ctx->pintra;
    int stride_y = pi->stride_rec[Y_C];
    pel *reco_y  = pi->addr_rec_pic[Y_C] + (y * stride_y) + x;
    int stride_uv = pi->stride_rec[chroma_type];
    int chroma_x = x >> 1;
    int chroma_y = y >> 1;
    int chroma_cu_width = cu_width >> 1;
    int chroma_cu_height = cu_height >> 1;
    pel* reco_chroma = pi->addr_rec_pic[chroma_type] + (chroma_y * stride_uv) + chroma_x;
    pel* reco_chroma_u = pi->addr_rec_pic[U_C] + (chroma_y * stride_uv) + chroma_x;
    int bit_depth = ctx->info.bit_depth_internal;
    int is_above = IS_AVAIL(avail_cu, AVAIL_UP2);
    int is_left = IS_AVAIL(avail_cu, AVAIL_LE2);
    if (is_above && is_left)
    {
        pel* luma_recon_le = reco_y - (ECCPM_TEMP_SIZE * 2);
        pel* chroma_recon_le = reco_chroma - ECCPM_TEMP_SIZE;
        pel* chroma_u_le = reco_chroma_u - ECCPM_TEMP_SIZE;
        pel* chroma_pred_le = pred_template;
        int w_le = ECCPM_TEMP_SIZE;
        int h_le = chroma_cu_height;
        pel* luma_recon_up = reco_y - (ECCPM_TEMP_SIZE * 2) * stride_y;
        pel* chroma_recon_up = reco_chroma - ECCPM_TEMP_SIZE * stride_uv;
        pel* chroma_u_up = reco_chroma_u - ECCPM_TEMP_SIZE * stride_uv;
        pel* chroma_pred_up = pred_template;
        int w_up = chroma_cu_width;
        int h_up = ECCPM_TEMP_SIZE;
        calculate_eccpm_model(chroma_u_le, chroma_u_up, luma_recon_le, chroma_recon_le, chroma_pred_le, stride_y, stride_uv, w_le, h_le, luma_recon_up, chroma_recon_up, chroma_pred_up, w_up, h_up, a0, a1, a2, a3, bit_depth, chroma_type, A, YC, param_num);
    }
    else
    {
        pel* luma_recon = NULL;
        pel* chroma_recon = NULL;
        pel* chroma_pred = NULL;
        pel* chroma_u_recon = NULL;
        int w = 0;
        int h = 0;
        if (is_left)
        {
            luma_recon = reco_y - (ECCPM_TEMP_SIZE * 2);
            chroma_recon = reco_chroma - ECCPM_TEMP_SIZE;
            chroma_u_recon = reco_chroma_u - ECCPM_TEMP_SIZE;
            chroma_pred = pred_template;
            w = ECCPM_TEMP_SIZE;
            h = chroma_cu_height;
        }
        else if (is_above)
        {
            luma_recon = reco_y - (ECCPM_TEMP_SIZE * 2)  *stride_y;
            chroma_recon = reco_chroma - ECCPM_TEMP_SIZE * stride_uv;
            chroma_u_recon = reco_chroma_u - ECCPM_TEMP_SIZE *stride_uv;
            chroma_pred = pred_template;
            w = chroma_cu_width;
            h = ECCPM_TEMP_SIZE;
        }
        if ((is_left) || (is_above))
        {
            calculate_eccpm_model(chroma_u_recon, NULL, luma_recon, chroma_recon, chroma_pred, stride_y, stride_uv, w, h, NULL, NULL, NULL, 0, 0, a0, a1, a2, a3, bit_depth, chroma_type, A, YC, param_num);
        }
    }
}
#endif
//Note: this is PB-based RDO
static double pintra_residue_rdo(ENC_CTX *ctx, ENC_CORE *core, pel *org_luma, pel *org_cb, pel *org_cr, int s_org, int s_org_c, int cu_width_log2, int cu_height_log2,
                                 s32 *dist, int bChroma, int pb_idx, int x, int y)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    ENC_PINTRA *pi = &ctx->pintra;
    int bit_depth = ctx->info.bit_depth_internal;
    static s16 coef_tmp[N_C][MAX_CU_DIM];
    static s16 resi[MAX_CU_DIM];
    int cu_width, cu_height, bit_cnt;
    double cost = 0;
    u16 avail_tb;
    int s_mod = pi->stride_rec[Y_C];
    pel* mod;
    int num_nz_temp[MAX_NUM_TB][N_C];
#if IST
    int coef_restrict = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif

    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
#if IST
    mod_info_curr->slice_type = ctx->slice_type;
#endif
    if (!bChroma)
    {
        int pb_part_size = mod_info_curr->pb_part;
        int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_idx);
        int pb_w = mod_info_curr->pb_info.sub_w[pb_idx];
        int pb_h = mod_info_curr->pb_info.sub_h[pb_idx];
        int pb_x = mod_info_curr->pb_info.sub_x[pb_idx];
        int pb_y = mod_info_curr->pb_info.sub_y[pb_idx];
        int tb_w, tb_h, tb_x, tb_y, tb_scup, tb_x_scu, tb_y_scu, coef_offset_tb;
        pel* pred_tb;
        cu_plane_nz_cln(mod_info_curr->num_nz, Y_C);

#if !DT_TRANSFORM
        get_tb_width_height_in_pb(pb_w, pb_h, pb_part_size, pb_idx, &tb_w, &tb_h);
#endif
        for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
        {
            //derive tu basic info
#if DT_TRANSFORM
            get_tb_width_height_in_pb(pb_w, pb_h, pb_part_size, pb_idx, &tb_w, &tb_h, tb_idx);
#endif
            get_tb_pos_in_pb(pb_x, pb_y, pb_part_size, tb_w, tb_h, tb_idx, &tb_x, &tb_y);
#if DT_TRANSFORM
            coef_offset_tb = get_coef_offset_tb(pb_x, pb_y, tb_x, tb_y, cu_width, cu_height, mod_info_curr->tb_part);
#else
            coef_offset_tb = tb_idx * tb_w * tb_h;
#endif
            tb_x_scu = PEL2SCU(tb_x);
            tb_y_scu = PEL2SCU(tb_y);
            tb_scup = tb_x_scu + (tb_y_scu * ctx->info.pic_width_in_scu);
            //get start of tb residual
            pel* resi_tb = resi + coef_offset_tb; //residual is stored sequentially, not in raster, like coef
            pel* rec_tb = pi->rec[Y_C] + (tb_y - mod_info_curr->y_pos) * cu_width + (tb_x - mod_info_curr->x_pos);

            avail_tb = com_get_avail_intra(tb_x_scu, tb_y_scu, ctx->info.pic_width_in_scu, tb_scup, ctx->map.map_scu
#if CCNPM
                , ctx->info.pic_height_in_scu, tb_w, tb_h
#endif            
            );

            s8 intra_mode = mod_info_curr->ipm[pb_idx][0];
            int tb_width_log2 = com_tbl_log2[tb_w];
            int tb_height_log2 = com_tbl_log2[tb_h];
            assert(tb_width_log2 > 0 && tb_height_log2 > 0);
#if EST
            int use_secTrans = (ctx->info.sqh.est_enable_flag ? mod_info_curr->est_flag : ctx->info.sqh.secondary_transform_enable_flag) &&
                               (tb_width_log2 > 2 || tb_height_log2 > 2);
            int use_alt4x4Trans = (ctx->info.sqh.est_enable_flag ? mod_info_curr->est_flag : ctx->info.sqh.secondary_transform_enable_flag);
#else
            int use_secTrans = ctx->info.sqh.secondary_transform_enable_flag && (tb_width_log2 > 2 || tb_height_log2 > 2);
            int use_alt4x4Trans = ctx->info.sqh.secondary_transform_enable_flag;
#endif
            int secT_Ver_Hor = 0;
            if (use_secTrans)
            {
                int vt, ht;
                int block_available_up = IS_AVAIL(avail_tb, AVAIL_UP);
                int block_available_left = IS_AVAIL(avail_tb, AVAIL_LE);
#if EIPM
                vt = (intra_mode < IPD_HOR) || (intra_mode >= IPD_DIA_L_EXT && intra_mode < IPD_HOR_EXT);
                ht = (intra_mode > IPD_VER && intra_mode < IPD_IPCM) || (intra_mode > IPD_VER_EXT && intra_mode < IPD_CNT) || (intra_mode <= IPD_BI);
#else
                vt = intra_mode < IPD_HOR;
                ht = (intra_mode > IPD_VER && intra_mode < IPD_CNT) || intra_mode <= IPD_BI;
#endif
                vt = vt && block_available_up;
                ht = ht && block_available_left;
                secT_Ver_Hor = (vt << 1) | ht;
#if EST
                if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0)
                {
                    secT_Ver_Hor = 3;
                }
#endif
            }

            //prediction according to the input ipm
#if IIP
#if SAWP
            if (mod_info_curr->sawp_flag)
            {
                pred_tb = mod_info_curr->pred[Y_C];
            }
            else
#endif // SAWP
            if (num_tb_in_pb == 1 && !mod_info_curr->iip_flag)
            {
                pred_tb = pi->pred_cache[intra_mode];
            }
#else
#if SAWP
            if (mod_info_curr->sawp_flag)
            {
                pred_tb = mod_info_curr->pred[Y_C];
            }
            else
#endif // SAWP
            if (num_tb_in_pb == 1)
                pred_tb = pi->pred_cache[intra_mode];
#endif
            else
            {
                mod = pi->addr_rec_pic[Y_C] + (tb_y * s_mod) + tb_x;
                pred_tb = mod_info_curr->pred[Y_C]; // pred is temp memory
                com_get_nbr(tb_x, tb_y, tb_w, tb_h, mod, s_mod, avail_tb, core->nb, tb_scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);
                com_ipred(core->nb[0][0] + STNUM, core->nb[0][1] + STNUM, pred_tb, intra_mode, tb_w, tb_h, bit_depth, avail_tb, mod_info_curr->ipf_flag
#if MIPF
                          , ctx->info.sqh.mipf_enable_flag
#endif
#if IIP
                          , mod_info_curr->iip_flag
#endif
                );
            }

            pel* org_luma_tb = org_luma + (tb_x - pb_x) + (tb_y - pb_y) * s_org;
            //trans & inverse trans
            enc_diff_16b(tb_width_log2, tb_height_log2, org_luma_tb, pred_tb, s_org, tb_w, tb_w, coef_tmp[Y_C]);
#if DEST
#if DEST_PH
            if (mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->est_flag > 1 && mod_info_curr->ph_dest_enable_flag)
#else
            if (mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->est_flag > 1)
#endif
            {
                s16 resi_tmp[MAX_TR_DIM];
                memcpy(resi_tmp, coef_tmp[Y_C], sizeof(s16) * cu_height * cu_width);
                if (mod_info_curr->est_flag == 2)
                {
                    for (int i = 0; i < cu_height; i++)
                    {
                        for (int j = 0; j < cu_width; j++)
                        {
                            coef_tmp[Y_C][i * cu_width + j] = (s16)resi_tmp[i * cu_width + (cu_width - 1 - j)];
                        }
                    }
                }
                else if (mod_info_curr->est_flag == 3)
                {
                    for (int i = 0; i < cu_height; i++)
                    {
                        for (int j = 0; j < cu_width; j++)
                        {
                            coef_tmp[Y_C][i * cu_width + j] = (s16)resi_tmp[(cu_height - 1 - i) * cu_width + j];
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < cu_height; i++)
                    {
                        for (int j = 0; j < cu_width; j++)
                        {
                            coef_tmp[Y_C][i * cu_width + j] = (s16)resi_tmp[(cu_height - 1 - i) * cu_width + (cu_width - 1 - j)];
                        }
                    }
                }
            }
#endif
            mod_info_curr->num_nz[tb_idx][Y_C] = enc_tq_nnz(ctx, mod_info_curr, Y_C, 0, core->qp_y, ctx->lambda[0], coef_tmp[Y_C], coef_tmp[Y_C], tb_width_log2, tb_height_log2, pi->slice_type, Y_C, 1, secT_Ver_Hor, use_alt4x4Trans);
#if IST
            if (ctx->info.sqh.ist_enable_flag && mod_info_curr->num_nz[tb_idx][Y_C] > 0)
            {
                int skip = 0;
                if (!mod_info_curr->ist_tu_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
                {
                    for (int y = 0; y < cu_height; y++)
                    {
                        for (int x = 0; x < cu_width; x++)
                        {
                            if ((x >= IST_MAX_COEF_SIZE || y >= IST_MAX_COEF_SIZE) && coef_tmp[Y_C][y * cu_width + x])
                            {
                                skip = 1;
                                break;
                            }
                        }
                        if (skip)
                            break;
                    }
                }
#if ISTS
                int even_num = 0;
                for (int ii = 0; ii < (1 << (tb_width_log2 + tb_height_log2)); ii++)
                {
                    even_num += ((coef_tmp[Y_C][ii] % 2) == 0 ? 1 : 0);
                }
#endif
#if ETS
                int refine_key = (!(even_num % 2) && !mod_info_curr->ist_tu_flag) || ((even_num % 2) && mod_info_curr->ist_tu_flag);
                if (ctx->info.pic_header.ph_ists_enable_flag && mod_info_curr->cu_mode == MODE_INTRA)
                {
                    if (!mod_info_curr->ist_tu_flag)
                        skip = 1;
                    refine_key = (!(even_num % 2) && mod_info_curr->ist_tu_flag == 1) || ((even_num % 2) && mod_info_curr->ist_tu_flag == 2);
                }
#endif
                //coefficients refine
                if (mod_info_curr->tb_part == 0 && cu_width_log2 < 6 && cu_height_log2 < 6 && !skip &&
#if ISTS
#if ETS
                    refine_key)
#else
                ((!(even_num % 2) && !mod_info_curr->ist_tu_flag) || ((even_num % 2) && mod_info_curr->ist_tu_flag)))
#endif
#else
                    ((mod_info_curr->num_nz[tb_idx][Y_C] % 2 && !mod_info_curr->ist_tu_flag) || (!(mod_info_curr->num_nz[tb_idx][Y_C] % 2) && mod_info_curr->ist_tu_flag)))
#endif
                {
                    int scan_pos, num_coeff, nz_cnt;
                    const u16     *scanp;
                    scanp = com_scan_tbl[COEF_SCAN_ZIGZAG][tb_width_log2 - 1][tb_height_log2 - 1];
                    num_coeff = 1 << (tb_width_log2 + tb_height_log2);
                    nz_cnt = 0;

                    for (scan_pos = 0; scan_pos <= num_coeff; scan_pos++)
                    {
                        if (nz_cnt < mod_info_curr->num_nz[tb_idx][Y_C])
                        {
                            nz_cnt = coef_tmp[Y_C][scanp[scan_pos]] ? nz_cnt + 1 : nz_cnt;
                        }
                        else
                        {
                            if (coef_tmp[Y_C][scanp[scan_pos - 1]])
                            {
                                if (scan_pos - 1)
                                {
#if ISTS
                                    if (ctx->info.pic_header.ph_ists_enable_flag)
                                    {
                                        coef_tmp[Y_C][scanp[scan_pos - 1]]--;
                                        if (coef_tmp[Y_C][scanp[scan_pos - 1]] == 0)
                                            mod_info_curr->num_nz[tb_idx][Y_C]--;
                                    }
                                    else
                                    {
                                        if (coef_tmp[Y_C][scanp[scan_pos - 1]] > 0)
                                        {
                                            coef_tmp[Y_C][scanp[scan_pos - 1]]--;
                                        }
                                        else
                                        {
                                            coef_tmp[Y_C][scanp[scan_pos - 1]]++;
                                        }
                                        if (coef_tmp[Y_C][scanp[scan_pos - 1]] == 0)
                                        {
                                            mod_info_curr->num_nz[tb_idx][Y_C]--;
                                        }
                                    }
#else
                                    mod_info_curr->num_nz[tb_idx][Y_C]--;
                                    coef_tmp[Y_C][scanp[scan_pos - 1]] = 0;
#endif
                                }
                                else
                                {
                                    mod_info_curr->num_nz[tb_idx][Y_C]++;
                                    coef_tmp[Y_C][scanp[scan_pos]] = 1;
                                }

                                break;
                            }
                        }
                    }
                }
#if ISTS
                if (mod_info_curr->tb_part == 0 && mod_info_curr->ist_tu_flag && !ctx->info.pic_header.ph_ists_enable_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
#else
                if (mod_info_curr->tb_part == 0 && mod_info_curr->ist_tu_flag && (cu_width_log2 >= 5 || cu_height_log2 >= 5))
#endif
                {
                    for (int y = 0; y < cu_height; y ++)
                    {
                        for (int x = 0; x < cu_width; x++)
                        {
                            if ((x >= IST_MAX_COEF_SIZE || y >= IST_MAX_COEF_SIZE) && coef_tmp[Y_C][y * cu_width + x])
                            {
                                coef_restrict = 1;
                            }
                        }
                    }
                }
            }
#endif
            s16* coef_tb = mod_info_curr->coef[Y_C] + coef_offset_tb;
            com_mcpy(coef_tb, coef_tmp[Y_C], sizeof(s16) * (tb_w * tb_h));
            if (mod_info_curr->num_nz[tb_idx][Y_C])
            {
                com_itdq(mod_info_curr, Y_C, 0, coef_tmp[Y_C], resi_tb, ctx->wq, tb_width_log2, tb_height_log2, core->qp_y, bit_depth, secT_Ver_Hor, use_alt4x4Trans
#if IST_CHROMA
                    , ctx->tree_status
#endif                
                );
            }

#if DEST
#if DEST_PH
            if (mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->ph_dest_enable_flag && mod_info_curr->est_flag && mod_info_curr->pb_info.num_sub_part == 1)
#else
            if (mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->est_flag && mod_info_curr->pb_info.num_sub_part == 1)
#endif
            {
                int reco_len = 4;
                int cu_rec_width = cu_width + 4;
                int cu_rec_height = cu_height + 4;

                pel    temp_pixel[(MAX_CU_SIZE + 4) * (MAX_CU_SIZE + 4)] = { 0 };
                pel    width_rec_pred_dist[MAX_CU_SIZE] = { 0 };
                pel    height_rec_pred_dist[MAX_CU_SIZE] = { 0 };
                pel* rec;
                int    s_rec;
                COM_PIC* pic = PIC_REC(ctx);

                for (int i = 0; i < cu_rec_height; i++)
                {
                    for (int j = 0; j < cu_rec_width; j++)
                    {
                        *(temp_pixel + i * cu_rec_width + j) = 512;
                    }
                }

                // 1st comp: REC+PRED
                if ((int)(x - reco_len) >= 0)
                {
                    s_rec = pic->stride_luma;
                    rec = pic->y + (y * s_rec) + x - reco_len;

                    for (int i = 0; i < cu_height; i++)
                    {
                        for (int j = 0; j < reco_len; j++)
                        {
                            *(temp_pixel + (i + reco_len) * cu_rec_width + j) = *(rec + i * s_rec + j);
                        }
                    }
                }
                if ((int)(y - reco_len) >= 0)
                {
                    s_rec = pic->stride_luma;
                    rec = pic->y + (y - reco_len) * s_rec + x;

                    for (int i = 0; i < reco_len; i++)
                    {
                        for (int j = 0; j < cu_width; j++)
                        {
                            *(temp_pixel + i * cu_rec_width + j + reco_len) = *(rec + i * s_rec + j);
                        }
                    }
                }
                if ((int)(x - reco_len) >= 0 && (int)(y - reco_len) >= 0)
                {
                    s_rec = pic->stride_luma;
                    rec = pic->y + (y - reco_len) * s_rec + x - reco_len;

                    for (int i = 0; i < reco_len; i++)
                    {
                        for (int j = 0; j < reco_len; j++)
                        {
                            *(temp_pixel + i * cu_rec_width + j) = *(rec + i * s_rec + j);
                        }
                    }
                }

                // 2nd comp: Pred
                for (int i = 0; i < cu_height; i++)
                {
                    for (int j = 0; j < cu_width; j++)
                    {
                        *(temp_pixel + (i + reco_len) * cu_rec_width + j + reco_len) = *(pred_tb + i * cu_width + j);
                    }
                }

                for (int i = 0; i < cu_width; i++)
                {
                    width_rec_pred_dist[i] = (*(temp_pixel + 3 * cu_rec_width + 4 + i)) * 2 - *(temp_pixel + 2 * cu_rec_width + 4 + i)
                        - *(temp_pixel + 4 * cu_rec_width + 4 + i);
                }

                for (int i = 0; i < cu_height; i++)
                {
                    height_rec_pred_dist[i] = (*(temp_pixel + (4 + i) * cu_rec_width + 3)) * 2 - *(temp_pixel + (4 + i) * cu_rec_width + 2)
                        - *(temp_pixel + (4 + i) * cu_rec_width + 4);
                }

                int boundary_dist[4][3] = { 0 };
#if ODEST
                for (int k = 0; k < 3; k++)
#else
                for (int k = 0; k < 4; k++)
#endif
                {
                    int tmp_dist = 0;
                    for (int i = 0; i < cu_width; i++)
                    {
                        switch (k)
                        {
                        case 0: tmp_dist = width_rec_pred_dist[i] - resi_tb[i]; break;
                        case 1: tmp_dist = width_rec_pred_dist[i] - resi_tb[cu_width - i - 1]; break;
                        case 2: tmp_dist = width_rec_pred_dist[i] - resi_tb[(cu_height - 1) * cu_width + i]; break;
                        case 3: tmp_dist = width_rec_pred_dist[i] - resi_tb[(cu_height - 1) * cu_width + cu_width - i - 1]; break;
                        }
                        boundary_dist[k][0] += abs(tmp_dist);
                    }
                    for (int i = 0; i < cu_height; i++)
                    {
                        switch (k)
                        {
                        case 0: tmp_dist = height_rec_pred_dist[i] - resi_tb[i * cu_width]; break;
                        case 1: tmp_dist = height_rec_pred_dist[i] - resi_tb[i * cu_width + cu_width - 1]; break;
                        case 2: tmp_dist = height_rec_pred_dist[i] - resi_tb[(cu_height - i - 1) * cu_width]; break;
                        case 3: tmp_dist = height_rec_pred_dist[i] - resi_tb[(cu_height - i - 1) * cu_width + cu_width - 1]; break;
                        }
                        boundary_dist[k][1] += abs(tmp_dist);
                    }

                    boundary_dist[k][2] = boundary_dist[k][0] + boundary_dist[k][1];
                    int tmp_pixel = 0;
                    for (int i = 0; i < cu_height; i++)
                    {
                        for (int j = 0; j < cu_width; j++)
                        {
                            switch (k)
                            {
                            case 0: tmp_pixel = *(pred_tb + i * cu_width + j) + resi_tb[i * cu_width + j]; break;
                            case 1: tmp_pixel = *(pred_tb + i * cu_width + j) + resi_tb[i * cu_width + cu_width - 1 - j]; break;
                            case 2: tmp_pixel = *(pred_tb + i * cu_width + j) + resi_tb[(cu_height - i - 1) * cu_width + j]; break;
                            case 3: tmp_pixel = *(pred_tb + i * cu_width + j) + resi_tb[(cu_height - i - 1) * cu_width + cu_width - 1 - j]; break;
                            }
                            if (tmp_pixel > ((1 << bit_depth) - 1) || tmp_pixel < 0)
                            {
                                boundary_dist[k][2] = 65472;
                                break;
                            }
                        }
                        if (boundary_dist[k][2] == 65472)
                            break;
                    }
                }

                int tmp_idx = 0;
                int tmp_cost = 0;
                tmp_idx = 1;
                tmp_cost = boundary_dist[0][2];
#if ODEST
                for (int i = 2; i < 4; i++)
#else
                for (int i = 2; i < 5; i++)
#endif
                {
                    if (boundary_dist[i - 1][2] < tmp_cost)
                    {
                        tmp_cost = boundary_dist[i - 1][2];
                        tmp_idx = i;
                    }
                }

                mod_info_curr->dest_flag = 1;
                if (tmp_idx > 1)
                {
                    s16 resi_tmp[MAX_TR_DIM] = { 0 };
                    memcpy(resi_tmp, resi_tb, sizeof(s16) * cu_height * cu_width);
                    if (tmp_idx == 2)
                    {
                        for (int i = 0; i < cu_height; i++)
                        {
                            for (int j = 0; j < cu_width; j++)
                            {
                                resi_tb[i * cu_width + j] = (s16)resi_tmp[i * cu_width + (cu_width - 1 - j)];
                            }
                        }
                    }
                    else if (tmp_idx == 3)
                    {
                        for (int i = 0; i < cu_height; i++)
                        {
                            for (int j = 0; j < cu_width; j++)
                            {
                                resi_tb[i * cu_width + j] = (s16)resi_tmp[(cu_height - 1 - i) * cu_width + j];
                            }
                        }
                    }
                    else
                    {
                        for (int i = 0; i < cu_height; i++)
                        {
                            for (int j = 0; j < cu_width; j++)
                            {
                                resi_tb[i * cu_width + j] = (s16)resi_tmp[(cu_height - 1 - i) * cu_width + (cu_width - 1 - j)];
                            }
                        }
                    }
                }
            }
#endif

            //to fit the interface of com_recon()
            for (int comp = 0; comp < N_C; comp++)
                num_nz_temp[TB0][comp] = mod_info_curr->num_nz[tb_idx][comp];
            com_recon(SIZE_2Nx2N, resi_tb, pred_tb, num_nz_temp, Y_C, tb_w, tb_h, cu_width, rec_tb, bit_depth
#if SBT
                , 0
#endif
            );

            //dist calc
            cost += enc_ssd_16b(tb_width_log2, tb_height_log2, rec_tb, org_luma_tb, cu_width, s_org, bit_depth); // stride for rec_tb is cu_width
            if (mod_info_curr->pb_part == SIZE_2Nx2N)
            {
                calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pi->rec, cu_width, x, y, 1, mod_info_curr->num_nz[TB0][Y_C] != 0, NULL, NULL, 0);
                cost += ctx->delta_dist;
            }

            //update map tb
#if SAWP
            if (mod_info_curr->sawp_flag)
            {
                update_sawp_info_map_scu(mod_info_curr, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu);
            }
            else {
#endif // SAWP
                update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_w, pb_h, ctx->info.pic_width_in_scu, intra_mode);
#if SAWP
            }
#endif // SAWP
            copy_rec_y_to_pic(rec_tb, tb_x, tb_y, tb_w, tb_h, cu_width, PIC_REC(ctx));
        }

        *dist = (s32)cost;
        if (pb_idx == 0)
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        else
            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_best);
        //enc_sbac_bit_reset(&core->s_temp_run);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        enc_bit_est_pb_intra_luma(ctx, core, ctx->info.pic_header.slice_type, mod_info_curr->coef, pb_idx);
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
#if IST
        if (coef_restrict)
        {
            cost = MAX_COST;
        }
#endif
    }
    else
    {
#if TSCPM
        int stride_y = pi->stride_rec[Y_C];
        pel *reco_y  = pi->addr_rec_pic[Y_C] + (y * stride_y) + x;  //x y 即亮度块在pic中的坐标
#endif
#if ST_CHROMA
        int use_alt4x4Trans = mod_info_curr->st_chroma_flag ? 1 : 0;
        int secT_Ver_Hor = mod_info_curr->st_chroma_flag ? 3 : 0;
#endif

        u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, ctx->info.pic_width_in_scu, mod_info_curr->scup, ctx->map.map_scu
#if CCNPM
            , ctx->info.pic_height_in_scu, cu_width, cu_height
#endif         
        );
#if SAWP
        if (mod_info_curr->sawp_flag && mod_info_curr->ipm[PB0][1] == IPD_DM_C)
        {
            pel pred0[MAX_CU_DIM] = { 0 };
            pel pred1[MAX_CU_DIM] = { 0 };

            com_ipred_uv(core->nb[1][0] + STNUM, core->nb[1][1] + STNUM, pred0, IPD_DM_C, mod_info_curr->sawp_idx0, cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                , U_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                , NULL, -1
#endif
#if IPF_CHROMA
                , mod_info_curr->ipf_flag&& ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C], pi->stride_rec[U_C], x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                , (1<<ctx->info.log2_max_cuwh)
#endif
            );

            com_ipred_uv(core->nb[1][0] + STNUM, core->nb[1][1] + STNUM, pred1, IPD_DM_C, mod_info_curr->sawp_idx1, cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                , U_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                , NULL, -1
#endif
#if IPF_CHROMA
                , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C], pi->stride_rec[U_C], x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                , (1<<ctx->info.log2_max_cuwh)
#endif
            );
            pel awp_weight0[N_C][MAX_AWP_DIM];
            pel awp_weight1[N_C][MAX_AWP_DIM];
            /* derive weights */
#if BAWP
#if SAWP_SCC == 0
            com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, 0, 1);
#else
            com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, 0);
#endif
#else
            com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1);
#endif
            /* combine two pred buf */
            com_derive_sawp_pred(mod_info_curr, U_C, pred0, pred1, awp_weight0[U_C], awp_weight1[U_C]);

            memcpy(pi->pred[U_C], mod_info_curr->pred[U_C], sizeof(pel)* (cu_width >> 1)* (cu_height >> 1));
        }
        else {
#endif // SAWP
#if ST_CHROMA
        if (!mod_info_curr->st_chroma_flag)
#endif
#if ECCPM
        {
#endif		
        com_ipred_uv(core->nb[1][0] + STNUM, core->nb[1][1] + STNUM, pi->pred[U_C], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                     , U_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                     , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                     , NULL, -1
#endif
#if IPF_CHROMA
                     , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                     , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C], pi->stride_rec[U_C], x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                     , (1<<ctx->info.log2_max_cuwh)
#endif
                    );
#if ECCPM
            if (!mod_info_curr->st_chroma_flag && mod_info_curr->eccpm_flag && (mod_info_curr->ipm[PB0][1]<IPD_TSCPM_C) && (IS_AVAIL(avail_cu, AVAIL_UP2) || IS_AVAIL(avail_cu, AVAIL_LE2)))
            {
                com_ipred_uv_template(core->nb_template[1][0] + STNUM, core->nb_template[1][1] + STNUM, pi->pred_template[0], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                    , U_C, reco_y, stride_y, core->nb_template
#endif
#if MIPF
                    , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                    , NULL, -1
#endif
#if IPF_CHROMA
                    , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
                );
                int param_num = 3;
                long long a0, a1, a2, a3 = 0;
                caculate_eccpm_param(ctx, x, y, cu_width, cu_height, pi->pred[U_C], pi->pred_template[0], avail_cu, U_C, pi->rec[U_C], cu_width >> 1, &a0, &a1, &a2, &a3, pi->A, pi->YC, param_num);
                pel* luma_recon = reco_y; 
                pel* chroma_pred = pi->pred[U_C];
                pel* dst = pi->pred[U_C];
                pred_eccpm(luma_recon, stride_y, chroma_pred, dst, cu_width >> 1, cu_height >> 1, a0, a1, a2, a3, bit_depth, pi->rec[U_C], cu_width >> 1, param_num);    
            }
          }
#endif  
#if SAWP
        }
#endif // SAWP
        enc_diff_16b(cu_width_log2 - 1, cu_height_log2 - 1, org_cb, pi->pred[U_C], s_org_c, cu_width >> 1, cu_width >> 1, coef_tmp[U_C]);
#if ST_CHROMA
        mod_info_curr->num_nz[TB0][U_C] = enc_tq_nnz(ctx, mod_info_curr, U_C, 0, core->qp_u, ctx->lambda[1], coef_tmp[U_C], coef_tmp[U_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, U_C, 1, secT_Ver_Hor, use_alt4x4Trans);
#else
        mod_info_curr->num_nz[TB0][U_C] = enc_tq_nnz(ctx, mod_info_curr, U_C, 0, core->qp_u, ctx->lambda[1], coef_tmp[U_C], coef_tmp[U_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, U_C, 1, 0, 0);
#endif
        com_mcpy(mod_info_curr->coef[U_C], coef_tmp[U_C], sizeof(u16) * (cu_width * cu_height));

#if PMC
        s8 ipm_c = mod_info_curr->ipm[PB0][1];
        int bMcpm = com_is_mcpm(ipm_c);
        if (bMcpm)
        {
            if (!IS_RIGHT_CBF_U(mod_info_curr->num_nz[TB0][U_C]))
            {
                return MAX_COST;
            }
        }
#endif
#if EPMC
        s8 ipm_c_t = mod_info_curr->ipm[PB0][1];
        int bEmcpm = com_is_emcpm(ipm_c_t);
        if (bEmcpm)
        {
            if (!IS_RIGHT_CBF_U(mod_info_curr->num_nz[TB0][U_C]))
            {
                return MAX_COST;
            }
        }
#endif

        if (mod_info_curr->num_nz[TB0][U_C])
        {
#if ST_CHROMA
            com_itdq(mod_info_curr, U_C, 0, coef_tmp[U_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, core->qp_u, bit_depth, secT_Ver_Hor, use_alt4x4Trans
#if IST_CHROMA
                , ctx->tree_status
#endif            
            );
#else
            com_itdq(mod_info_curr, U_C, 0, coef_tmp[U_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, core->qp_u, bit_depth, 0, 0);
#endif
        }

        com_recon(SIZE_2Nx2N, resi, pi->pred[U_C], mod_info_curr->num_nz, U_C, cu_width >> 1, cu_height >> 1, cu_width >> 1, pi->rec[U_C], bit_depth
#if SBT
            , 0
#endif
        );
#if SAWP
        if (mod_info_curr->sawp_flag && mod_info_curr->ipm[PB0][1] == IPD_DM_C)
        {
            pel pred0[MAX_CU_DIM] = { 0 };
            pel pred1[MAX_CU_DIM] = { 0 };

            com_ipred_uv(core->nb[2][0] + STNUM, core->nb[2][1] + STNUM, pred0, IPD_DM_C, mod_info_curr->sawp_idx0, cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                , V_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                , NULL, -1
#endif
#if IPF_CHROMA
                , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C],pi->addr_rec_pic[V_C], pi->stride_rec[V_C], x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                , (1<<ctx->info.log2_max_cuwh)
#endif
            );

            com_ipred_uv(core->nb[2][0] + STNUM, core->nb[2][1] + STNUM, pred1, IPD_DM_C, mod_info_curr->sawp_idx1, cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                , V_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                , NULL, -1
#endif
#if IPF_CHROMA
                , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C], pi->stride_rec[V_C], x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                , (1<<ctx->info.log2_max_cuwh)
#endif
            );
            pel awp_weight0[N_C][MAX_AWP_DIM];
            pel awp_weight1[N_C][MAX_AWP_DIM];
            /* derive weights */
#if BAWP
#if SAWP_SCC == 0
            com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, 0, 1);
#else
            com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, 0);
#endif
#else
            com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1);
#endif
            /* combine two pred buf */
            com_derive_sawp_pred(mod_info_curr, V_C, pred0, pred1, awp_weight0[V_C], awp_weight1[V_C]);
            
            memcpy(pi->pred[V_C], mod_info_curr->pred[V_C], sizeof(pel)* (cu_width >> 1)* (cu_height >> 1));
        }
        else {
#endif // SAWP
#if ECCPM
            int pred_v_flag = 0;
            if (((mod_info_curr->ipm[PB0][1] < IPD_TSCPM_C)) && mod_info_curr->eccpm_flag && (IS_AVAIL(avail_cu, AVAIL_UP2) || IS_AVAIL(avail_cu, AVAIL_LE2)))
            {
                pred_v_flag=1;
            }
#endif
#if ST_CHROMA
#if PMC
#if EPMC
        if (!mod_info_curr->st_chroma_flag || bEmcpm || bMcpm)
#else
        if (!mod_info_curr->st_chroma_flag || bMcpm)
#endif
#else
        if (!mod_info_curr->st_chroma_flag)
#endif
#endif
#if ECCPM
        com_ipred_uv(core->nb[2][0] + STNUM, core->nb[2][1] + STNUM, 
                    pred_v_flag ? pi->pred_v : pi->pred[V_C]
                     , mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                     , V_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                     , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                     , pi->rec[U_C], cu_width >> 1
#endif
#if IPF_CHROMA
                     , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                     , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C], pi->stride_rec[V_C], x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                     , (1<<ctx->info.log2_max_cuwh)
#endif
                    );
#else
        com_ipred_uv(core->nb[2][0] + STNUM, core->nb[2][1] + STNUM, pi->pred[V_C], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                     , V_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                     , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                     , pi->rec[U_C], cu_width >> 1
#endif
#if IPF_CHROMA
                     , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                     , pi->addr_rec_pic[Y_C], pi->stride_rec[Y_C], pi->addr_rec_pic[U_C], pi->addr_rec_pic[V_C], pi->stride_rec[V_C], x, y
#endif
                    );
#endif
#if ECCPM
                if(pred_v_flag)
                memcpy(pi->pred[V_C], pi->pred_v, (cu_width >> 1) * (cu_height >> 1) * sizeof(pel));
                static long long a0, a1, a2 ,a3= 0;
                static int param_num = 0;
                if (!mod_info_curr->st_chroma_flag && mod_info_curr->eccpm_flag && (mod_info_curr->ipm[PB0][1] < IPD_TSCPM_C) && (IS_AVAIL(avail_cu, AVAIL_UP2) || IS_AVAIL(avail_cu, AVAIL_LE2)))
                {
                    com_ipred_uv_template(core->nb_template[2][0] + STNUM, core->nb_template[2][1] + STNUM, pi->pred_template[1], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                        , V_C, reco_y, stride_y, core->nb_template
#endif
#if MIPF
                        , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                        , NULL, -1
#endif
#if IPF_CHROMA
                        , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
                    );
                    param_num = 4;
                    caculate_eccpm_param(ctx, x, y, cu_width, cu_height, pi->pred[V_C], pi->pred_template[1], avail_cu, V_C, pi->rec[U_C], cu_width >> 1, &a0, &a1, &a2, &a3, pi->A, pi->YC, param_num);
                }
                if (mod_info_curr->eccpm_flag && (mod_info_curr->ipm[PB0][1]<IPD_TSCPM_C) && (IS_AVAIL(avail_cu, AVAIL_UP2) || IS_AVAIL(avail_cu, AVAIL_LE2)))
                {
                       pel* luma_recon = reco_y; 
                       pel* chroma_pred = pi->pred[V_C];
                       pel* dst = pi->pred[V_C];
                       pred_eccpm(luma_recon, stride_y, chroma_pred, dst, cu_width >> 1, cu_height >> 1, a0, a1, a2, a3, bit_depth, pi->rec[U_C], cu_width >> 1, param_num);    
                }
#endif
#if SAWP
        }
#endif // SAWP
        enc_diff_16b(cu_width_log2 - 1, cu_height_log2 - 1, org_cr, pi->pred[V_C], s_org_c, cu_width >> 1, cu_width >> 1, coef_tmp[V_C]);
#if PMC || EPMC
        int qp_v = core->qp_v;
        double lambda_v = ctx->lambda[2];
#if EPMC && PMC
        if(com_is_mcpm(ipm_c)||com_is_emcpm(ipm_c_t))
#elif EPMC
        if(com_is_emcpm(ipm_c_t))
#else
        if (com_is_mcpm(ipm_c))
#endif
        {
            qp_v = core->qp_v_pmc;
            lambda_v = ctx->lambda_v_pmc;
        }
#if ST_CHROMA
        mod_info_curr->num_nz[TB0][V_C] = enc_tq_nnz(ctx, mod_info_curr, V_C, 0, qp_v, lambda_v, coef_tmp[V_C], coef_tmp[V_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, V_C, 1, secT_Ver_Hor, use_alt4x4Trans);
#else
        mod_info_curr->num_nz[TB0][V_C] = enc_tq_nnz(ctx, mod_info_curr, V_C, 0, qp_v, lambda_v, coef_tmp[V_C], coef_tmp[V_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, V_C, 1, 0, 0);
#endif
#else
#if ST_CHROMA
        mod_info_curr->num_nz[TB0][V_C] = enc_tq_nnz(ctx, mod_info_curr, V_C, 0, core->qp_v, ctx->lambda[2], coef_tmp[V_C], coef_tmp[V_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, V_C, 1, secT_Ver_Hor, use_alt4x4Trans);
#else
        mod_info_curr->num_nz[TB0][V_C] = enc_tq_nnz(ctx, mod_info_curr, V_C, 0, core->qp_v, ctx->lambda[2], coef_tmp[V_C], coef_tmp[V_C], cu_width_log2 - 1, cu_height_log2 - 1, pi->slice_type, V_C, 1, 0, 0);
#endif
#endif
        com_mcpy(mod_info_curr->coef[V_C], coef_tmp[V_C], sizeof(u16) * (cu_width * cu_height));
        if (pb_idx == 0)
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        else
            SBAC_LOAD(core->s_temp_run, core->s_temp_prev_comp_best);
        //enc_sbac_bit_reset(&core->s_temp_run);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        enc_bit_est_intra_chroma(ctx, core, mod_info_curr->coef);
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += ctx->dist_chroma_weight[0] * enc_ssd_16b(cu_width_log2 - 1, cu_height_log2 - 1, pi->rec[U_C], org_cb, cu_width >> 1, s_org_c, bit_depth);
        if (mod_info_curr->num_nz[TB0][V_C])
        {
#if PMC || EPMC
#if ST_CHROMA
            com_itdq(mod_info_curr, V_C, 0, coef_tmp[V_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, qp_v, bit_depth, secT_Ver_Hor, use_alt4x4Trans
#if IST_CHROMA
                , ctx->tree_status
#endif            
            );
#else
            com_itdq(mod_info_curr, V_C, 0, coef_tmp[V_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, qp_v, bit_depth, 0, 0);
#endif
#else
#if ST_CHROMA
            com_itdq(mod_info_curr, V_C, 0, coef_tmp[V_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, core->qp_v, bit_depth, secT_Ver_Hor, use_alt4x4Trans);
#else
            com_itdq(mod_info_curr, V_C, 0, coef_tmp[V_C], resi, ctx->wq, mod_info_curr->cu_width_log2 - 1, mod_info_curr->cu_height_log2 - 1, core->qp_v, bit_depth, 0, 0);
#endif
#endif
        }

        com_recon(SIZE_2Nx2N, resi, pi->pred[V_C], mod_info_curr->num_nz, V_C, cu_width >> 1, cu_height >> 1, cu_width >> 1, pi->rec[V_C], bit_depth
#if SBT
            , 0
#endif
        );

#if PMC || EPMC
        double dist_weight_v = ctx->dist_chroma_weight[1];
#if EPMC && PMC
        if(com_is_mcpm(ipm_c) || com_is_emcpm(ipm_c_t))
#elif EPMC
        if(com_is_emcpm(ipm_c_t))
#else
        if (com_is_mcpm(ipm_c))
#endif
        {
            dist_weight_v = ctx->dist_chroma_weight_v_pmc;
        }
        cost += dist_weight_v              * enc_ssd_16b(cu_width_log2 - 1, cu_height_log2 - 1, pi->rec[V_C], org_cr, cu_width >> 1, s_org_c, bit_depth);
#else
        cost += ctx->dist_chroma_weight[1] * enc_ssd_16b(cu_width_log2 - 1, cu_height_log2 - 1, pi->rec[V_C], org_cr, cu_width >> 1, s_org_c, bit_depth);
#endif
        *dist = (s32)cost;
        cost += enc_ssd_16b(cu_width_log2, cu_height_log2, pi->rec[Y_C], org_luma, cu_width, s_org, bit_depth);
        cost += RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    }
    return cost;
}

#define NUM_IPM_CAND 3
#define PUT_IPM2LIST(list, cnt, ipm)\
{\
    int idx_list, is_check = 0;\
    for(idx_list = 0; idx_list < (cnt); idx_list++)\
        if((ipm) == (list)[idx_list]) is_check = 1;\
    if(is_check == 0)\
    {\
        (list)[(cnt)] = (ipm); (cnt)++;\
    }\
}

#if SAWP
static u32 calc_sad_16b(int pu_w, int pu_h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    u32 cost = 0;
    int num_seg_in_pu_w = 1, num_seg_in_pu_h = 1;
    int seg_w_log2 = com_tbl_log2[pu_w];
    int seg_h_log2 = com_tbl_log2[pu_h];
    s16* src1_seg, * src2_seg;
    s16* s1 = (s16*)src1;
    s16* s2 = (s16*)src2;

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

static u32 calc_sad_mask_16b(int pu_w, int pu_h, void* src1, void* src2, void* hardmask, int s_src1, int s_src2, int s_mask, int bit_depth)
{
    u32 cost = 0;
    int num_seg_in_pu_w = 1, num_seg_in_pu_h = 1;
    int seg_w_log2 = com_tbl_log2[pu_w];
    int seg_h_log2 = com_tbl_log2[pu_h];
    s16* src1_seg, * src2_seg, * mask_seg;
    s16* s1 = (s16*)src1;
    s16* s2 = (s16*)src2;
    s16* m = (s16*)hardmask;

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
        cost += enc_sad_mask_16b(seg_w_log2, seg_h_log2, s1, s2, m, s_src1, s_src2, s_mask, bit_depth);
        return cost;
    }

    for (int j = 0; j < num_seg_in_pu_h; j++)
    {
        for (int i = 0; i < num_seg_in_pu_w; i++)
        {
            assert(0);
            src1_seg = s1 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src1;
            src2_seg = s2 + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_src2;
            mask_seg = m + (1 << seg_w_log2) * i + (1 << seg_h_log2) * j * s_mask;
            cost += enc_sad_mask_16b(seg_w_log2, seg_h_log2, src1_seg, src2_seg, mask_seg, s_src1, s_src2, s_mask, bit_depth);
        }
    }
    return cost;
}
#endif // SAWP


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

void com_update_cand_list(const u8 ipm, const double cost, const int ipd_num, int *ipred_list, double *cand_cost)
{
    int shift = 0;
    while (shift < ipd_num && cost < cand_cost[ipd_num - 1 - shift])
    {
        shift++;
    }
    if (shift != 0)
    {
        for (int j = 1; j < shift; j++)
        {
            ipred_list[ipd_num - j] = ipred_list[ipd_num - 1 - j];
            cand_cost[ipd_num - j] = cand_cost[ipd_num - 1 - j];
        }
        ipred_list[ipd_num - shift] = ipm;
        cand_cost[ipd_num - shift] = cost;
    }
}
#if EIPM
void check_one_intra_pred_mode(ENC_CTX *ctx, ENC_CORE *core, pel *org, int s_org, u8 ipm, int ipred_list_len, int *ipred_list, double *cand_cost, int part_idx, int pb_w, int pb_h, u16 avail_cu)
{
    ENC_PINTRA *pi = &ctx->pintra;
    int bit_cnt;
    double cost;
    int bit_depth = ctx->info.bit_depth_internal;
    int cu_width_log2 = (&core->mod_info_curr)->cu_width_log2;
    int cu_height_log2 = (&core->mod_info_curr)->cu_height_log2;
    pel *pred_buf = pi->pred_cache[ipm];
    com_ipred(core->nb[0][0] + STNUM, core->nb[0][1] + STNUM, pred_buf, ipm, pb_w, pb_h, bit_depth, avail_cu, core->mod_info_curr.ipf_flag
#if MIPF
              , ctx->info.sqh.mipf_enable_flag
#endif
#if IIP
              , core->mod_info_curr.iip_flag
#endif
    );
    cost = (double)calc_satd_16b(pb_w, pb_h, org, pred_buf, s_org, pb_w, bit_depth);
    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    encode_intra_dir(&core->bs_temp, ipm, ctx->info.sqh.eipm_enable_flag, core->mod_info_curr.mpm[part_idx]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);
    com_update_cand_list(ipm, cost, ipred_list_len, ipred_list, cand_cost);
}

static int make_ipred_list_fast(ENC_CTX *ctx, ENC_CORE *core, int width, int height, int cu_width_log2, int cu_height_log2, pel *org, int s_org, int *ipred_list, int part_idx, u16 avail_cu
    , int skip_ipd
    , u8 *mpm
)
{
    static  const int AngMap[IPD_CNT] =
    {
        0,1,2,3,34,4,35,5,36,6,37,7,
        38,8,39,9,40,10,41,11,42,43,
        12,44,45,13,46,14,47,15,48,
        16,49,17,50,18,51,19,52,20,
        53,21,54,22,55,23,56,57,24,
        58,59,25,60,26,61,27,62,28,
        63,29,64,30,65,31,32,33

    };

    static const int ReverseAng[IPD_CNT] =
    {
        0,1,2,3,5,7,9,11,13,15,
        17,19,22,25,27,29,31,33,
        35,37,39,41,43,45,48,51,
        53,55,57,59,61,63,64,65,
        4,6,8,10,12,14,16,18,20,
        21,23,24,26,28,30,32,34,
        36,38,40,42,44,46,47,49,
        50,52,54,56,58,60,62
    };

    static const u8 rmd_search_step_4[18] = { 0,  1,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32 };

    ENC_PINTRA *pi = &ctx->pintra;
    int cu_width, cu_height, i;
    int ipd_rdo_cnt = IPD_RDO_CNT;
#if FIMC
    ipd_rdo_cnt -= NUM_MPM;
    assert(ipd_rdo_cnt >= 0);
    int check_mpm_flag[NUM_MPM] = {0, 0};
#endif

    com_assert(ipd_rdo_cnt <= IPD_RDO_CNT);
    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
    for (i = 0; i < ipd_rdo_cnt; i++) 
    {
        ipred_list[i] = IPD_DC;
    }

    int num_cand_step_4_in = 18;
    int num_cand_step_4_out = 10;
    double rmd_cand_cost[18];
    int rmd_ipred_list[18];
    for (i = 0; i < num_cand_step_4_in; i++) 
    {
        rmd_ipred_list[i] = IPD_DC;
        rmd_cand_cost[i] = MAX_COST;
    }

#if FAST_SCC_PRECODING
    if (ctx->slice_type == SLICE_I && core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit
        && ctx->info.pic_header.sp_pic_flag)
    {
        int vist0_ipm = core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].Lipm_value;
        double rmd_cost[3];
        int rmd_ipred[3];
        for (int i = 0; i < 3; i++) {
            rmd_ipred[i] = IPD_DC;
            rmd_cost[i] = MAX_COST;
        }
        for (int skip = 0; skip < 1; skip++) {
            if (skip_ipd == 1 && (vist0_ipm == IPD_PLN || vist0_ipm == IPD_BI || vist0_ipm == IPD_DC))
            {
                continue;
            }
            if ((vist0_ipm == IPD_DC || vist0_ipm == IPD_PLN || vist0_ipm == IPD_BI) && (com_tbl_log2[height] < 2 || com_tbl_log2[width] < 2))
            {
                continue;
            }
            if (com_tbl_log2[height] >= 2 && com_tbl_log2[width] >= 2)
            {
                check_one_intra_pred_mode(ctx, core, org, s_org, 0, 3, rmd_ipred, rmd_cost, part_idx, width, height, avail_cu);
            }
            if ((vist0_ipm != IPD_DC) && (vist0_ipm != IPD_VER) && (vist0_ipm != IPD_HOR))
            {
                check_one_intra_pred_mode(ctx, core, org, s_org, vist0_ipm, 3, rmd_ipred, rmd_cost, part_idx, width, height, avail_cu);
            }
            check_one_intra_pred_mode(ctx, core, org, s_org, IPD_VER, 3, rmd_ipred, rmd_cost, part_idx, width, height, avail_cu);
            check_one_intra_pred_mode(ctx, core, org, s_org, IPD_HOR, 3, rmd_ipred, rmd_cost, part_idx, width, height, avail_cu);
#if FIMC
            for (i = 0; i < NUM_MPM; i++)
            {
                if (vist0_ipm == mpm[i])
                {
                    check_mpm_flag[i] = 1;
                }
            }
            if (0 == mpm[0] || 12 == mpm[0] || 24 == mpm[0])
            {
                check_mpm_flag[0] = 1;
            }
            if (0 == mpm[1] || 12 == mpm[1] || 24 == mpm[1])
            {
                check_mpm_flag[1] = 1;
            }
#endif
            //*history_skip_flag = 1;
            for (i = 0; i < NUM_MPM; i++)
            {
                if (check_mpm_flag[i] == 0 && mpm[i] != IPD_IPCM)
                {
                    if (mpm[i] == IPD_DC || mpm[i] == IPD_BI || mpm[i] == IPD_PLN)
                    {
                        if (com_tbl_log2[height] < 2 || com_tbl_log2[width] < 2)
                        {
                            continue;
                        }
                    }
                    check_one_intra_pred_mode(ctx, core, org, s_org, mpm[i], 3, rmd_ipred, rmd_cost, part_idx, width, height, avail_cu);
                }
            }
        }
        if ((rmd_ipred[0] == IPD_DC || rmd_ipred[0] == IPD_BI || rmd_ipred[0] == IPD_PLN) && (com_tbl_log2[height] >= 2 && com_tbl_log2[width] >= 2)) {
            ipred_list[0] = rmd_ipred[0];
            return 1;
        }
        else if (rmd_ipred[0] != IPD_DC || rmd_ipred[0] != IPD_BI || rmd_ipred[0] != IPD_PLN) {
            ipred_list[0] = rmd_ipred[0];
            return 1;
        }
    }
#endif

    u8 rmd_search_step_2[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // at most 2*num_cand_step_4_out
    int rmd_search_step_2_in = 0;
    int rmd_search_step_2_out = 6;

    u8 rmd_search_step_1[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // at most rmd_search_step_2_out * 2 + 4
    int rmd_search_step_1_in = 0;

    for (i = 0; i < num_cand_step_4_in; i++) 
    {
#if FIMC
        if (rmd_search_step_4[i] == mpm[0])
        {
            check_mpm_flag[0] = 1;
        }
        if (rmd_search_step_4[i] == mpm[1])
        {
            check_mpm_flag[1] = 1;
        }
#endif
        if (skip_ipd == 1 && (rmd_search_step_4[i] == IPD_PLN || rmd_search_step_4[i] == IPD_BI || rmd_search_step_4[i] == IPD_DC))
        {
            continue;
        }

        com_assert(rmd_search_step_4[i] != 33);
        check_one_intra_pred_mode(ctx, core, org, s_org, rmd_search_step_4[i], COM_MAX(num_cand_step_4_out, ipd_rdo_cnt), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu);
    }

    if (rmd_ipred_list[0] < 2 && rmd_ipred_list[1] < 2) 
    {
        for (i = 0; i < 3; i++) 
        {
            ipred_list[i] = rmd_ipred_list[i];
        }
#if FIMC
        for( i = 0; i < NUM_MPM; i++ )
        {
            if( check_mpm_flag[i] == 0 && mpm[i] != IPD_IPCM )
            {
                check_one_intra_pred_mode( ctx, core, org, s_org, mpm[i], COM_MIN( ipd_rdo_cnt, IPD_RDO_CNT ), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu );
            }
        }
#endif
        return 3;
    }

    for (int rmd_idx = 0; rmd_idx < num_cand_step_4_out; rmd_idx++) 
    {
        int imode4step = ReverseAng[rmd_ipred_list[rmd_idx]];
        int imode4step_sub_2 = AngMap[max(imode4step - 2, 0)];
        int imode4step_add_2 = AngMap[min(imode4step + 2, IPD_CNT - 1)];
        if (imode4step == 22 || imode4step == 48) 
        {
            imode4step_sub_2 = AngMap[imode4step - 3];
            imode4step_add_2 = AngMap[imode4step + 3];
        }       
        if (imode4step >= 5 && (imode4step != 64)) 
        {
            for (i = 0; i < rmd_search_step_2_in; i++) 
            {
                if (imode4step_sub_2 == rmd_search_step_2[i]) 
                {
                  break;
                }
            }
            if (i == rmd_search_step_2_in && imode4step_sub_2 >= 3) 
            {
                rmd_search_step_2[rmd_search_step_2_in++] = imode4step_sub_2;
            }
            for (i = 0; i < rmd_search_step_2_in; i++) 
            {
                if (imode4step_add_2 == rmd_search_step_2[i]) 
                {
                    break;
                }
            }
            if (i == rmd_search_step_2_in && imode4step_add_2 != 33) 
            {
                rmd_search_step_2[rmd_search_step_2_in++] = imode4step_add_2;
            }
        }
    }

    for (int rmd_idx = 0; rmd_idx < rmd_search_step_2_in; rmd_idx++) 
    {
        com_assert(rmd_search_step_2[rmd_idx] != 33);
#if FIMC
        if (rmd_search_step_2[rmd_idx] == mpm[0])
        {
            check_mpm_flag[0] = 1;
        }
        if (rmd_search_step_2[rmd_idx] == mpm[1])
        {
            check_mpm_flag[1] = 1;
        }
#endif
        check_one_intra_pred_mode(ctx, core, org, s_org, rmd_search_step_2[rmd_idx], COM_MAX(rmd_search_step_2_out, ipd_rdo_cnt), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu);
    }

    for (int rmd_idx = 0, j = 0; rmd_idx < rmd_search_step_2_out;)
    {
        int imode2step = ReverseAng[rmd_ipred_list[j++]];
        int imode2step_sub_1 = AngMap[max(imode2step - 1, 0)];
        int imode2step_add_1 = AngMap[min(imode2step + 1, IPD_CNT - 1)];

        if (j >= num_cand_step_4_in)
        {
            break;
        }
        if (imode2step >= 4 && imode2step != 65)
        {
            rmd_idx++;
            if (rmd_search_step_1_in == 0)
            {
                rmd_search_step_1[rmd_search_step_1_in++] = imode2step_sub_1;
                if (imode2step_add_1 != 33 && imode2step != 63)
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = imode2step_add_1;
                }
                if (imode2step == 22 || imode2step == 48)
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step - 2];
                    rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step + 2];
                }
            } 
            else
            {
                for (i = 0; i < rmd_search_step_1_in; i++)
                {
                    if (imode2step_sub_1 == rmd_search_step_1[i])
                    {
                        break;
                    }
                }
                if (i == rmd_search_step_1_in && (imode2step_sub_1 >= 3))
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = imode2step_sub_1;
                }
                for (i = 0; i < rmd_search_step_1_in; i++)
                {
                    if (imode2step_add_1 == rmd_search_step_1[i] || (imode2step == 63))
                    {
                        break;
                    }
                }
                if (i == rmd_search_step_1_in && (imode2step_add_1 != 33))
                {
                    rmd_search_step_1[rmd_search_step_1_in++] = imode2step_add_1;
                }
                if (imode2step == 22 || imode2step == 48)
                {
                    for (i = 0; i < rmd_search_step_1_in; i++)
                    {
                        if ((imode2step - 2) == ReverseAng[rmd_search_step_1[i]])
                        {
                            break;
                        }
                    }
                    if (i == rmd_search_step_1_in)
                    {
                        rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step - 2];
                    }
                    for (i = 0; i < rmd_search_step_1_in; i++)
                    {
                        if ((imode2step + 2) == ReverseAng[rmd_search_step_1[i]])
                        {
                            break;
                        }
                    }
                    if (i == rmd_search_step_1_in)
                    {
                        rmd_search_step_1[rmd_search_step_1_in++] = AngMap[imode2step + 2];
                    }
                }
            }
        }
    }
    for (int rmd_idx = 0; rmd_idx < rmd_search_step_1_in; rmd_idx++) 
    {
#if FIMC
        if (rmd_search_step_1[rmd_idx] == mpm[0])
        {
            check_mpm_flag[0] = 1;
        }
        if (rmd_search_step_1[rmd_idx] == mpm[1])
        {
            check_mpm_flag[1] = 1;
        }
#endif
        com_assert(rmd_search_step_1[rmd_idx] != 33);
        check_one_intra_pred_mode(ctx, core, org, s_org, rmd_search_step_1[rmd_idx], COM_MIN(ipd_rdo_cnt, IPD_RDO_CNT), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu);
    }
#if FIMC
    for( i = 0; i < NUM_MPM; i++ )
    {
        if( check_mpm_flag[i] == 0 && mpm[i] != IPD_IPCM )
        {
            check_one_intra_pred_mode( ctx, core, org, s_org, mpm[i], COM_MIN( ipd_rdo_cnt, IPD_RDO_CNT ), rmd_ipred_list, rmd_cand_cost, part_idx, width, height, avail_cu );
        }
    }
#endif
    int pred_cnt = COM_MIN(ipd_rdo_cnt, IPD_RDO_CNT);
    for (i = 0; i < pred_cnt; i++) 
    {
        ipred_list[i] = rmd_ipred_list[i];
    }

    for (i = pred_cnt - 1; i >= 0; i--)
    {
        if (rmd_cand_cost[i] > core->inter_satd * (1.2))
        {
            pred_cnt--;
        }
        else
        {
            break;
        }
    }
    return pred_cnt;
}
#endif

static int make_ipred_list(ENC_CTX * ctx, ENC_CORE * core, int width, int height, int cu_width_log2, int cu_height_log2, pel * org, int s_org, int * ipred_list, int part_idx, u16 avail_cu
                           , int skip_ipd
                          )
{
    ENC_PINTRA * pi = &ctx->pintra;
    int bit_depth = ctx->info.bit_depth_internal;
    int pred_cnt, i, j;
    double cost, cand_cost[IPD_RDO_CNT];
    u32 cand_satd_cost[IPD_RDO_CNT];
    u32 cost_satd;
    int ipd_rdo_cnt = (width >= height * 4 || height >= width * 4) ? IPD_RDO_CNT - 1 : IPD_RDO_CNT;
#if FIMC
    ipd_rdo_cnt -= NUM_MPM;
    assert(ipd_rdo_cnt >= 0);
#endif
    for (i = 0; i < ipd_rdo_cnt; i++)
    {
        ipred_list[i] = IPD_DC;
        cand_cost[i] = MAX_COST;
        cand_satd_cost[i] = COM_UINT32_MAX;
    }
    pred_cnt = IPD_CNT;
#if EIPM
    int num_intra;
    if (ctx->info.sqh.eipm_enable_flag) 
    {
        num_intra = IPD_CNT;
    } 
    else 
    {
        num_intra = IPD_IPCM;
    }
    for (i = 0; i < num_intra; i++)
#else
    for (i = 0; i < IPD_CNT; i++)
#endif
    {
        if (skip_ipd == 1 && (i == IPD_PLN || i == IPD_BI || i == IPD_DC))
        {
            continue;
        }

#if EIPM
        if (i == IPD_IPCM)
        {
            continue;
        }
#endif
        int bit_cnt, shift = 0;
        pel * pred_buf = NULL;
        pred_buf = pi->pred_cache[i];
        com_ipred(core->nb[0][0] + STNUM, core->nb[0][1] + STNUM, pred_buf, i, width, height, bit_depth, avail_cu, core->mod_info_curr.ipf_flag
#if MIPF
                  , ctx->info.sqh.mipf_enable_flag
#endif
#if IIP
                  , core->mod_info_curr.iip_flag
#endif
        );
        cost_satd = calc_satd_16b(width, height, org, pred_buf, s_org, width, bit_depth);
        cost = (double)cost_satd;
        SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
        //enc_sbac_bit_reset(&core->s_temp_run);
        bit_cnt = enc_get_bit_number(&core->s_temp_run);
        encode_intra_dir(&core->bs_temp, (u8)i,
#if EIPM
            ctx->info.sqh.eipm_enable_flag,
#endif
            core->mod_info_curr.mpm[part_idx]);
        bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
        cost += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt);
        while (shift < ipd_rdo_cnt && cost < cand_cost[ipd_rdo_cnt - 1 - shift])
        {
            shift++;
        }
        if (shift != 0)
        {
            for (j = 1; j < shift; j++)
            {
                ipred_list[ipd_rdo_cnt - j] = ipred_list[ipd_rdo_cnt - 1 - j];
                cand_cost[ipd_rdo_cnt - j] = cand_cost[ipd_rdo_cnt - 1 - j];
                cand_satd_cost[ipd_rdo_cnt - j] = cand_satd_cost[ipd_rdo_cnt - 1 - j];
            }
            ipred_list[ipd_rdo_cnt - shift] = i;
            cand_cost[ipd_rdo_cnt - shift] = cost;
            cand_satd_cost[ipd_rdo_cnt - shift] = cost_satd;
        }
    }
    pred_cnt = ipd_rdo_cnt;
    for (i = ipd_rdo_cnt - 1; i >= 0; i--)
    {
        if (cand_satd_cost[i] > core->inter_satd * (1.1))
        {
            pred_cnt--;
        }
        else
        {
            break;
        }
    }
    return COM_MIN(pred_cnt, ipd_rdo_cnt);
}

double analyze_intra_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PINTRA *pi = &ctx->pintra;
    COM_MODE *bst_info = &core->mod_info_best;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j, s_org, s_org_c, s_mod, s_mod_c;
    int best_ipd[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_pb_part[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_c = IPD_INVALID;
    s32 best_dist_y = 0, best_dist_c = 0;
    s32 best_dist_y_pb_part[MAX_NUM_TB] = { 0, 0, 0, 0 };
    u8  best_mpm_pb_part[MAX_NUM_TB][2];
    u8  best_mpm[MAX_NUM_TB][2];
    static s16  coef_y_pb_part[MAX_CU_DIM];
    static pel  rec_y_pb_part[MAX_CU_DIM];
    int  num_nz_y_pb_part[MAX_NUM_TB];
    int ipm_l2c = 0;
    int chk_bypass = 0;
    int bit_cnt = 0;
    int ipred_list[IPD_CNT];
    int pred_cnt = IPD_CNT;
    pel *org, *mod;
    pel *org_cb, *org_cr;
    pel *mod_cb, *mod_cr;
    double cost_temp, cost_best = MAX_COST;
    double cost_pb_temp, cost_pb_best;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int cu_x = mod_info_curr->x_pos;
    int cu_y = mod_info_curr->y_pos;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if EST
    double costdct2[5] = { MAX_COST, MAX_COST, MAX_COST, MAX_COST, MAX_COST };
    int cbfdct2[5] = { 1, 1, 1, 1, 1 };
    int cbfdst7[5] = { 1, 1, 1, 1, 1 };
    int bst_est_tu_flag = 0;
    bst_info->est_flag = 0;
    mod_info_curr->est_flag = 0;
#endif
#if DEST
	int bst_dest_tu_flag = 0;
	bst_info->dest_flag = 0;
	mod_info_curr->dest_flag = 0;
#endif
#if ST_CHROMA
    bst_info->st_chroma_flag = 0;
    mod_info_curr->st_chroma_flag = 0;
#endif
#if USE_IBC
    mod_info_curr->cu_mode = MODE_INTRA;
    mod_info_curr->ibc_flag = 0;
#endif
#if CIBC
    mod_info_curr->cibc_flag = 0;
#endif
#if USE_SP
    mod_info_curr->sp_flag = 0;
#if RSD_OPT
    mod_info_curr->sp_rsd_flag = 0;
#endif
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if DEST_PH
    mod_info_curr->ph_dest_enable_flag = ctx->info.pic_header.ph_dest_enable_flag;
#endif
    int ipd_buf[4] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int ipd_add[4][3] = { { IPD_INVALID, IPD_INVALID, IPD_INVALID },{ IPD_INVALID, IPD_INVALID, IPD_INVALID },
        { IPD_INVALID, IPD_INVALID, IPD_INVALID },{ IPD_INVALID, IPD_INVALID, IPD_INVALID }
    };
#if DT_SAVE_LOAD
    ENC_BEF_DATA* pData = &core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup];
#endif
#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif
    s_mod = pi->stride_rec[Y_C];
    s_org = pi->stride_org[Y_C];
    s_mod_c = pi->stride_rec[U_C];
    s_org_c = pi->stride_org[U_C];
    mod = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
    org = pi->addr_org[Y_C] + (y * s_org) + x;
    int part_size_idx, pb_part_idx;
    core->best_pb_part_intra = SIZE_2Nx2N;

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
    {
        int allowed_part_size[7] = { SIZE_2Nx2N };
        int num_allowed_part_size = 1;
        int dt_allow = ctx->info.sqh.dt_intra_enable_flag ? com_dt_allow(mod_info_curr->cu_width, mod_info_curr->cu_height, MODE_INTRA, ctx->info.sqh.max_dt_size) : 0;
#if DT_INTRA_BOUNDARY_FILTER_OFF
        if (core->mod_info_curr.ipf_flag)
            dt_allow = 0;
#endif
#if IIP
        int area = 1 << (cu_width_log2 + cu_height_log2);
        if (core->mod_info_curr.iip_flag)
        {
            dt_allow = 0;
        }
#endif
        //prepare allowed PU partition
        if (dt_allow & 0x1) // horizontal
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_2NxhN;
        }
        if (dt_allow & 0x2) // vertical
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_hNx2N;
        }
        if (dt_allow & 0x1) // horizontal
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_2NxnU;
            allowed_part_size[num_allowed_part_size++] = SIZE_2NxnD;
        }
        if (dt_allow & 0x2) // vertical
        {
            allowed_part_size[num_allowed_part_size++] = SIZE_nLx2N;
            allowed_part_size[num_allowed_part_size++] = SIZE_nRx2N;
        }

        cost_best = MAX_COST;
#if DT_INTRA_FAST_BY_RD
        u8 try_non_2NxhN = 1, try_non_hNx2N = 1;
        double cost_2Nx2N = MAX_COST, cost_hNx2N = MAX_COST, cost_2NxhN = MAX_COST;
#endif

#if FIMC
        // copy table for dt, last, best, core
        COM_CNTMPM cntmpm_cands_curr;
        COM_CNTMPM cntmpm_cands_last;
        if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
        {
            com_cntmpm_copy(&cntmpm_cands_last, &core->cntmpm_cands);
            com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
        }
#endif

        for (part_size_idx = 0; part_size_idx < num_allowed_part_size; part_size_idx++)
        {
            PART_SIZE pb_part_size = allowed_part_size[part_size_idx];
            PART_SIZE tb_part_size = get_tb_part_size_by_pb(pb_part_size, MODE_INTRA);
            set_pb_part(mod_info_curr, pb_part_size);
            set_tb_part(mod_info_curr, tb_part_size);
            get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, pb_part_size, &mod_info_curr->pb_info);
#if DT_TRANSFORM
            get_tb_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, tb_part_size, &mod_info_curr->tb_info);
#else
            get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, tb_part_size, &mod_info_curr->tb_info);
#endif
            assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
            cost_temp = 0;
            memset(num_nz_y_pb_part, 0, MAX_NUM_TB * sizeof(int));

            //DT fast algorithm here
#if DT_INTRA_FAST_BY_RD
            if (((pb_part_size == SIZE_2NxnU || pb_part_size == SIZE_2NxnD) && !try_non_2NxhN)
                    || ((pb_part_size == SIZE_nLx2N || pb_part_size == SIZE_nRx2N) && !try_non_hNx2N))
            {
                continue;
            }
#endif
#if DT_SAVE_LOAD
            if (pb_part_size != SIZE_2Nx2N && pData->num_intra_history > 1 && pData->best_part_size_intra[0] == SIZE_2Nx2N && pData->best_part_size_intra[1] == SIZE_2Nx2N)
                break;
#endif
#if FAST_SCC_PRECODING
            if (ctx->slice_type == SLICE_I && (pb_part_size != SIZE_2Nx2N && core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit0_part != pb_part_size) && core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit
                && ctx->info.pic_header.sp_pic_flag)
            {
                continue;
            }
#endif
            //end of DT fast algorithm

#if FIMC
            // copy table for dt
            if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
            {
                com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_last);
            }
#endif

            // pb-based intra mode candidate selection
            for (pb_part_idx = 0; pb_part_idx < mod_info_curr->pb_info.num_sub_part; pb_part_idx++)
            {
                int pb_x = mod_info_curr->pb_info.sub_x[pb_part_idx];
                int pb_y = mod_info_curr->pb_info.sub_y[pb_part_idx];
                int pb_w = mod_info_curr->pb_info.sub_w[pb_part_idx];
                int pb_h = mod_info_curr->pb_info.sub_h[pb_part_idx];
                int pb_scup = mod_info_curr->pb_info.sub_scup[pb_part_idx];
                int pb_x_scu = PEL2SCU(pb_x);
                int pb_y_scu = PEL2SCU(pb_y);
                int pb_coef_offset = get_coef_offset_tb(mod_info_curr->x_pos, mod_info_curr->y_pos, pb_x, pb_y, cu_width, cu_height, tb_part_size);
                int tb_idx_offset = get_tb_idx_offset(pb_part_size, pb_part_idx);
                int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_part_idx);
                int skip_ipd = 0;
                if (((pb_part_size == SIZE_2NxnU || pb_part_size == SIZE_nLx2N) && pb_part_idx == 1) ||
                    ((pb_part_size == SIZE_2NxnD || pb_part_size == SIZE_nRx2N) && pb_part_idx == 0))
                {
                    skip_ipd = 1;
                }

                cost_pb_best = MAX_COST;
                cu_nz_cln(mod_info_curr->num_nz);

                mod = pi->addr_rec_pic[Y_C] + (pb_y * s_mod) + pb_x;
                org = pi->addr_org[Y_C] + (pb_y * s_org) + pb_x;

                u16 avail_cu = com_get_avail_intra(pb_x_scu, pb_y_scu, ctx->info.pic_width_in_scu, pb_scup, ctx->map.map_scu
#if CCNPM
                    , ctx->info.pic_height_in_scu, pb_w, pb_h
#endif
                );
                com_get_nbr(pb_x, pb_y, pb_w, pb_h, mod, s_mod, avail_cu, core->nb, pb_scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);

#if FIMC
                if (ctx->info.sqh.fimc_enable_flag && ctx->info.pic_header.fimc_pic_flag)
                {
                    com_get_cntmpm(pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx], &core->cntmpm_cands);
                }
                else
                {
#endif
                    com_get_mpm( pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx] );
#if FIMC
                }
#endif

#if EIPM
                if (ctx->info.sqh.eipm_enable_flag)
                {
#if IIP
                    if (core->mod_info_curr.iip_flag)
                    {
                        pred_cnt = pi->iip_rdo_num;
                        memcpy(ipred_list, pi->iip_pred, pred_cnt * sizeof(int));
                    }
                    else
                    {
#endif
                        pred_cnt = make_ipred_list_fast(ctx, core, pb_w, pb_h, cu_width_log2, cu_height_log2, org, s_org, ipred_list, pb_part_idx, avail_cu
                            , skip_ipd, mod_info_curr->mpm[pb_part_idx]
                        );
#if IIP
                    }
                    
                    if (!core->mod_info_curr.ipf_flag && !core->mod_info_curr.iip_flag && part_size_idx == SIZE_2Nx2N && area >= MIN_IIP_BLK && area <= MAX_IIP_BLK)
                    {
                        memcpy(pi->iip_pred, ipred_list, pred_cnt * sizeof(int));
                        pi->iip_rdo_num = pred_cnt;
                    }
#endif
                }
                else
                {
#endif
#if IIP
                    if (core->mod_info_curr.iip_flag)
                    {
                        pred_cnt = pi->iip_rdo_num;
                        memcpy(ipred_list, pi->iip_pred, pred_cnt * sizeof(int));
                    }
                    else
                    {
#endif
                        pred_cnt = make_ipred_list(ctx, core, pb_w, pb_h, cu_width_log2, cu_height_log2, org, s_org, ipred_list, pb_part_idx, avail_cu
                            , skip_ipd
                        );
#if IIP
                    }

                    if (!core->mod_info_curr.ipf_flag && !core->mod_info_curr.iip_flag && part_size_idx == SIZE_2Nx2N && area >= MIN_IIP_BLK && area <= MAX_IIP_BLK)
                    {
                        memcpy(pi->iip_pred, ipred_list, pred_cnt * sizeof(int));
                        pi->iip_rdo_num = pred_cnt;
                    }
#endif

#if EIPM
                }
#endif
                if (skip_ipd == 1)
                {
                    if (pb_part_size == SIZE_2NxnU)
                    {
                        if (ipd_add[0][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[0][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[0][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                    else if (pb_part_size == SIZE_2NxnD)
                    {
                        if (ipd_add[1][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[1][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[1][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                    else if (pb_part_size == SIZE_nLx2N)
                    {
                        if (ipd_add[2][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[2][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[2][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                    else if (pb_part_size == SIZE_nRx2N)
                    {
                        if (ipd_add[3][0] == 1)
                            ipred_list[pred_cnt++] = IPD_PLN;
                        if (ipd_add[3][1] == 1)
                            ipred_list[pred_cnt++] = IPD_BI;
                        if (ipd_add[3][2] == 1)
                            ipred_list[pred_cnt++] = IPD_DC;
                    }
                }

#if FIMC
#if IBC_ENH
                if ((ctx->info.sqh.ibc_flag && pb_w * pb_h <= 32) || !ctx->info.sqh.ibc_flag || ctx->info.pic_header.ibc_type == 1)
#else
                if ((ctx->info.sqh.ibc_flag && pb_w * pb_h <= 32) || !ctx->info.sqh.ibc_flag)
#endif
                {
                    u8 mode_in_list[IPD_CNT];
                    memset(mode_in_list, 0, sizeof(u8)* IPD_CNT);
                    for (int mIdx = 0; mIdx < pred_cnt; mIdx++)
                    {
                        mode_in_list[ipred_list[mIdx]] = 1;
                    }
                    u8 *curr_mpm = mod_info_curr->mpm[pb_part_idx];
                    const u8 extra_rdo_num = NUM_MPM;
                    for (int mIdx = 0; mIdx < extra_rdo_num; mIdx++)
                    {
                        u8 curr_mode = curr_mpm[mIdx];
                        if (mode_in_list[curr_mode] == 0 && (curr_mode != IPD_IPCM))
                        {
                            assert(curr_mode >= 0 && curr_mode < IPD_CNT);
                            mode_in_list[curr_mode] = 1;
                            ipred_list[pred_cnt++] = curr_mode;
                        }
                    }
                }
#endif
                if (pred_cnt == 0)
                {
                    return MAX_COST;
                }
#if EST
                int st_all = (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0) ?
#if DEST
#if ODEST
#if DEST_PH
                    (mod_info_curr->ph_dest_enable_flag ? 4 : 2)
#else
                    4
#endif
#else
#if DEST_PH
                    (mod_info_curr->ph_dest_enable_flag ? 5 : 2)
#else
                    5
#endif
#endif
#else
                    2
#endif
                    : 1;
                for (int use_st = 0; use_st < st_all; use_st++)
                {
                    mod_info_curr->est_flag = use_st;
                    if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part != 0)
                        mod_info_curr->est_flag = 1;
#endif
                    for (j = 0; j < pred_cnt; j++) /* Y */
                    {
                        s32 dist_t = 0;
                        i = ipred_list[j];
                        mod_info_curr->ipm[pb_part_idx][0] = (s8)i;
                        mod_info_curr->ipm[pb_part_idx][1] = IPD_INVALID;
#if IST
                        int ist_all = (ctx->info.sqh.ist_enable_flag &&
#if EST
                            (ctx->info.sqh.est_enable_flag ? !mod_info_curr->est_flag : !mod_info_curr->tb_part) &&
#else
                            mod_info_curr->tb_part == 0 &&
#endif
                            cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
#if ETS
                        if (ctx->info.pic_header.ph_ists_enable_flag && mod_info_curr->cu_mode == MODE_INTRA && ist_all == 2)
                        {
                            ist_all = 3;
                        }
#endif
                        for (int use_ist = 0; use_ist < ist_all; use_ist++)
                        {
                            mod_info_curr->ist_tu_flag = use_ist;
#endif
#if EST
                            if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0 && mod_info_curr->est_flag)
                            {
                                if (cost_pb_best * 1.4 < costdct2[j])
                                    continue;
                                if (!cbfdct2[j] && !cbfdst7[j])
                                    continue;
                            }
#endif
                            cost_pb_temp = pintra_residue_rdo(ctx, core, org, NULL, NULL, s_org, s_org_c, cu_width_log2, cu_height_log2, &dist_t, 0, pb_part_idx, x, y);
#if EST
                            if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0 && !mod_info_curr->est_flag)
                            {
#if IST
                                if (mod_info_curr->ist_tu_flag)
                                {
                                    cbfdst7[j] = mod_info_curr->num_nz[0][0];
                                }
                                else
#endif
                                {
                                    costdct2[j] = cost_pb_temp;
                                    cbfdct2[j] = mod_info_curr->num_nz[0][0];
                                }
                            }
#endif
#if PRINT_CU_LEVEL_2
                            printf("\nluma pred mode %2d cost_pb_temp %10.1f", i, cost_pb_temp);
                            double val = 2815.9;
                            if (cost_pb_temp - val < 0.1 && cost_pb_temp - val > -0.1)
                            {
                                int a = 0;
                            }
#endif
                            if (cost_pb_temp < cost_pb_best)
                            {
                                cost_pb_best = cost_pb_temp;
                                best_dist_y_pb_part[pb_part_idx] = dist_t;
                                best_ipd_pb_part[pb_part_idx] = i;
                                best_mpm_pb_part[pb_part_idx][0] = mod_info_curr->mpm[pb_part_idx][0];
                                best_mpm_pb_part[pb_part_idx][1] = mod_info_curr->mpm[pb_part_idx][1];

#if IST
                                bst_ist_tu_flag = mod_info_curr->ist_tu_flag;
#endif
#if EST
                                bst_est_tu_flag = mod_info_curr->est_flag;
#endif
                                com_mcpy(coef_y_pb_part + pb_coef_offset, mod_info_curr->coef[Y_C], pb_w * pb_h * sizeof(s16));
                                for (int j = 0; j < pb_h; j++)
                                {
                                    int rec_offset = ((pb_y - cu_y) + j) * cu_width + (pb_x - cu_x);
                                    com_mcpy(rec_y_pb_part + rec_offset, pi->rec[Y_C] + rec_offset, pb_w * sizeof(pel));
                                }

                                for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
                                {
                                    num_nz_y_pb_part[tb_idx + tb_idx_offset] = mod_info_curr->num_nz[tb_idx][Y_C];
                                }
#if EST
                                if (ctx->info.sqh.est_enable_flag)
                                {
                                    SBAC_STORE(core->s_temp_prev_comp_run, core->s_temp_run);
                                }
                                else
                                {
                                    SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                                }
#else
                                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
#endif
                            }
#if IST
                        }
#endif
                    }
#if EST
                }
                if (ctx->info.sqh.est_enable_flag)
                {
                    SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_prev_comp_run);
                }
#endif
                cost_temp += cost_pb_best;
                if (pb_part_size == SIZE_2NxhN || pb_part_size == SIZE_hNx2N)
                {
                    ipd_buf[pb_part_idx] = best_ipd_pb_part[pb_part_idx];
                }

                //update map - pb
                update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_w, pb_h, ctx->info.pic_width_in_scu, best_ipd_pb_part[pb_part_idx]);
                copy_rec_y_to_pic(rec_y_pb_part + (pb_y - cu_y) * cu_width + (pb_x - cu_x), pb_x, pb_y, pb_w, pb_h, cu_width, PIC_REC(ctx));

#if FIMC
                if (ctx->info.sqh.fimc_enable_flag) //  && pb_part_idx == PB0
                {
                    com_cntmpm_update(&core->cntmpm_cands, best_ipd_pb_part[pb_part_idx]);
                }
#endif
            }

            if (pb_part_size == SIZE_2NxhN || pb_part_size == SIZE_hNx2N)
            {
                int mem_offset = pb_part_size == SIZE_hNx2N ? 2 : 0;
                if (ipd_buf[1] == IPD_PLN || ipd_buf[2] == IPD_PLN || ipd_buf[3] == IPD_PLN)
                {
                    ipd_add[mem_offset + 0][0] = 1;
                }
                if (ipd_buf[1] == IPD_BI || ipd_buf[2] == IPD_BI || ipd_buf[3] == IPD_BI)
                {
                    ipd_add[mem_offset + 0][1] = 1;
                }
                if (ipd_buf[1] == IPD_DC || ipd_buf[2] == IPD_DC || ipd_buf[3] == IPD_DC)
                {
                    ipd_add[mem_offset + 0][2] = 1;
                }

                if (ipd_buf[0] == IPD_PLN || ipd_buf[1] == IPD_PLN || ipd_buf[2] == IPD_PLN)
                {
                    ipd_add[mem_offset + 1][0] = 1;
                }
                if (ipd_buf[0] == IPD_BI || ipd_buf[1] == IPD_BI || ipd_buf[2] == IPD_BI)
                {
                    ipd_add[mem_offset + 1][1] = 1;
                }
                if (ipd_buf[0] == IPD_DC || ipd_buf[1] == IPD_DC || ipd_buf[2] == IPD_DC)
                {
                    ipd_add[mem_offset + 1][2] = 1;
                }
            }

            com_mcpy(pi->rec[Y_C], rec_y_pb_part, cu_width * cu_height * sizeof(pel));
#if RDO_DBK
            if (mod_info_curr->pb_part != SIZE_2Nx2N)
            {
                int cbf_y = num_nz_y_pb_part[0] + num_nz_y_pb_part[1] + num_nz_y_pb_part[2] + num_nz_y_pb_part[3];
                calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pi->rec, cu_width, x, y, 1, cbf_y, NULL, NULL, 0);
                cost_temp += ctx->delta_dist;
                best_dist_y_pb_part[PB0] += (s32)ctx->delta_dist; //add delta SSD to the first PB
            }
#endif
#if DT_INTRA_FAST_BY_RD
            if (pb_part_size == SIZE_2Nx2N)
                cost_2Nx2N = cost_temp;
            else if (pb_part_size == SIZE_2NxhN)
            {
                cost_2NxhN = cost_temp;
                assert(cost_2Nx2N != MAX_COST);
                if (cost_2NxhN > cost_2Nx2N * 1.05)
                    try_non_2NxhN = 0;
            }
            else if (pb_part_size == SIZE_hNx2N)
            {
                cost_hNx2N = cost_temp;
                assert(cost_2Nx2N != MAX_COST);
                if (cost_hNx2N > cost_2Nx2N * 1.05)
                    try_non_hNx2N = 0;
            }

            if (cost_hNx2N != MAX_COST && cost_2NxhN != MAX_COST)
            {
                if (cost_hNx2N > cost_2NxhN * 1.1)
                    try_non_hNx2N = 0;
                else if (cost_2NxhN > cost_hNx2N * 1.1)
                    try_non_2NxhN = 0;
            }
#endif
            //save luma cb decision for each pb_part_size
            if (cost_temp < cost_best)
            {
                cost_best = cost_temp;
                best_dist_y = 0;
                for (int pb_idx = 0; pb_idx < mod_info_curr->pb_info.num_sub_part; pb_idx++)
                {
                    best_dist_y += best_dist_y_pb_part[pb_idx];
                    best_ipd[pb_idx] = best_ipd_pb_part[pb_idx];
                    best_mpm[pb_idx][0] = best_mpm_pb_part[pb_idx][0];
                    best_mpm[pb_idx][1] = best_mpm_pb_part[pb_idx][1];
                }
#if IST
                bst_info->ist_tu_flag = bst_ist_tu_flag;
#endif
#if EST
                bst_info->est_flag = bst_est_tu_flag;
#endif
#if DEST
				bst_info->dest_flag = bst_dest_tu_flag;
#endif
                com_mcpy(bst_info->coef[Y_C], coef_y_pb_part, cu_width * cu_height * sizeof(s16));
                com_mcpy(bst_info->rec[Y_C], rec_y_pb_part, cu_width * cu_height * sizeof(pel));
                assert(mod_info_curr->pb_info.num_sub_part <= mod_info_curr->tb_info.num_sub_part);
                cu_plane_nz_cln(bst_info->num_nz, Y_C);
                for (int tb_idx = 0; tb_idx < mod_info_curr->tb_info.num_sub_part; tb_idx++)
                {
                    bst_info->num_nz[tb_idx][Y_C] = num_nz_y_pb_part[tb_idx];
                }
#if TB_SPLIT_EXT
                core->best_pb_part_intra = mod_info_curr->pb_part;
                core->best_tb_part_intra = mod_info_curr->tb_part;
#endif
                SBAC_STORE(core->s_temp_pb_part_best, core->s_temp_prev_comp_best);

#if FIMC
                // copy table for dt
                if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
                {
                    com_cntmpm_copy(&cntmpm_cands_curr, &core->cntmpm_cands);
                }
#endif
            }
        }
#if FIMC
        // copy table for dt
        if (ctx->info.sqh.fimc_enable_flag && num_allowed_part_size > 1)
        {
            com_cntmpm_copy(&core->cntmpm_cands, &cntmpm_cands_curr);
        }
#endif
        pel *pBestSrc = bst_info->rec[Y_C];
        pel *pModDst = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
        for (int h = 0; h < cu_height; h++)
        {
            com_mcpy(pModDst, pBestSrc, cu_width * sizeof(pel));
            pModDst += s_mod;
            pBestSrc += cu_width;
        }
    }

#if SAWP_COST_FAST
    pData->intra_luma_cost = cost_best;
#endif // SAWP_COST_FAST

#if IST_CHROMA
    if (!bst_info->num_nz[0][Y_C])
    {
        mod_info_curr->ist_tu_flag = 0;
        bst_info->ist_tu_flag = 0;
    }
    if (bst_info->ist_tu_flag)
    {
        bst_info->tb_part = SIZE_2Nx2N;
    }
    mod_info_curr->ist_tu_flag = bst_info->ist_tu_flag;
    mod_info_curr->tb_part = bst_info->tb_part;
#endif

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_C)
    {
        //chroma RDO
        SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_pb_part_best);
        org = pi->addr_org[Y_C] + (y * s_org) + x;
#if TSCPM
        mod = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
#endif
        mod_cb = pi->addr_rec_pic[U_C] + ((y >> 1) * s_mod_c) + (x >> 1);
        mod_cr = pi->addr_rec_pic[V_C] + ((y >> 1) * s_mod_c) + (x >> 1);
        org_cb = pi->addr_org[U_C] + ((y >> 1) * s_org_c) + (x >> 1);
        org_cr = pi->addr_org[V_C] + ((y >> 1) * s_org_c) + (x >> 1);

        u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, ctx->info.pic_width_in_scu, mod_info_curr->scup, ctx->map.map_scu
#if CCNPM
            , ctx->info.pic_height_in_scu, cu_width, cu_height
#endif        
        );
#if TSCPM
        com_get_nbr(x,      y,      cu_width,      cu_height,      mod,    s_mod,   avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);
#endif
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cb, s_mod_c, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, U_C);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cr, s_mod_c, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, V_C);
#if ECCPM
        /* U */
        com_get_template_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cb, s_mod_c, avail_cu, core->nb_template, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, U_C);
        /* V */
        com_get_template_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cr, s_mod_c, avail_cu, core->nb_template, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, V_C);
#endif
        cost_best = MAX_COST;

#if CHROMA_NOT_SPLIT
        //get luma pred mode
        if (ctx->tree_status == TREE_C)
        {
            assert(cu_width >= 8 && cu_height >= 8);
            int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;
            best_ipd[PB0] = ctx->map.map_ipm[luma_scup];
#if IPCM
            assert((best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT) || best_ipd[PB0] == IPD_IPCM);
#else
            assert(best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT);
#endif
#if USE_IBC
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]) || MCU_GET_IBC(ctx->map.map_scu[luma_scup]));
#else
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]));
#endif
        }
#endif
        ipm_l2c = best_ipd[PB0];
        mod_info_curr->ipm[PB0][0] = (s8)best_ipd[PB0];
        COM_IPRED_CONV_L2C_CHK(ipm_l2c, chk_bypass);
#if ECCPM
        mod_info_curr->eccpm_flag = 0;
        for (mod_info_curr->eccpm_flag = 0; mod_info_curr->eccpm_flag <= 1; mod_info_curr->eccpm_flag++)
        {
#endif        
#if TSCPM || PMC
        /* add tscpm, ehance_tscpm, pmc and epmc modes */
        for (i = 0; i < IPD_CHROMA_EXT_CNT; i++) /* UV */
#else
        for (i = 0; i < IPD_CHROMA_CNT; i++) /* UV */
#endif
        {
#if ECCPM
            if ((!ctx->info.sqh.eccpm_enable_flag) && (mod_info_curr->eccpm_flag == 1))
            {
                continue;
            }
            if (mod_info_curr->eccpm_flag == 1)
            {
                if ((i >= IPD_TSCPM_C))
                {
                    continue;
                }
                if(!ctx->info.pic_header.eccpm_scc_flag)
                {
                    if((i != IPD_DM_C) && (i != IPD_BI_C))
                    {
                        continue;
                    }
                }
                if(ctx->info.pic_header.eccpm_scc_flag)
                {
                    if((i != IPD_DM_C) && (i != IPD_VER_C) && (i != IPD_HOR_C) && (i != IPD_BI_C))
                    {
                        continue;
                    }
                }
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP2) && !IS_AVAIL(avail_cu, AVAIL_LE2) && (mod_info_curr->eccpm_flag == 1))
            {
                continue;
            }
#endif            
            s32 dist_t = 0;
            mod_info_curr->ipm[PB0][1] = (s8)i;
            if (i != IPD_DM_C && chk_bypass && i == ipm_l2c)
            {
                continue;
            }
#if IPCM
            if (i == IPD_DM_C && best_ipd[PB0] == IPD_IPCM)
            {
                continue;
            }
#endif
#if TSCPM
            if (!ctx->info.sqh.tscpm_enable_flag && i == IPD_TSCPM_C)
            {
                continue;
            }
#if ENHANCE_TSPCM
#if ENHANCE_LT_MODE
            if (!ctx->info.sqh.enhance_tscpm_enable_flag && (i == IPD_TSCPM_LT_C || i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C))
#else
            if (!ctx->info.sqh.enhance_tscpm_enable_flag && (i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)) 
#endif
            {
                continue;
            }
            if (IS_AVAIL(avail_cu, AVAIL_UP) && !IS_AVAIL(avail_cu, AVAIL_UP_LE) && i == IPD_TSCPM_T_C) 
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && IS_AVAIL(avail_cu, AVAIL_UP_LE) && i == IPD_TSCPM_L_C) 
            {
                continue;
            }
#else
#if ENHANCE_LT_MODE
            if (i == IPD_TSCPM_LT_C || i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)
#else
            if (i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)
#endif
            {
                continue;
            }
#endif
#endif
#if PMC
#if ENHANCE_LT_MODE
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_MCPM_LT_C || i == IPD_MCPM_C || i == IPD_MCPM_T_C || i == IPD_MCPM_L_C))
#else
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_MCPM_C || i == IPD_MCPM_T_C || i == IPD_MCPM_L_C))
#endif
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_LE) && i == IPD_MCPM_T_C)
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && i == IPD_MCPM_L_C)
            {
                continue;
            }
#endif
#if EPMC
#if ENHANCE_LT_MODE
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_EMCPM_C || i == IPD_EMCPM_LT_C || i == IPD_EMCPM_T_C || i == IPD_EMCPM_L_C || i == IPD_EMCPM2_C || i == IPD_EMCPM2_LT_C || i == IPD_EMCPM2_T_C || i == IPD_EMCPM2_L_C))
#else
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_EMCPM_C || i == IPD_EMCPM_T_C || i == IPD_EMCPM_L_C || i == IPD_EMCPM2_C || i == IPD_EMCPM2_T_C || i == IPD_EMCPM2_L_C))
#endif
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && (i == IPD_EMCPM_T_C|| i == IPD_EMCPM2_T_C))
            {
                continue;               
            }
            if (!IS_AVAIL(avail_cu, AVAIL_LE) && (i == IPD_EMCPM_L_C|| i == IPD_EMCPM2_L_C))
            {             
                 continue;
            }
#endif
#if EPMC
            if (!ctx->info.pic_header.ph_epmc_model_flag  && i >= IPD_EMCPM2_C && i <= IPD_EMCPM2_T_C)
            {
                continue;
            }
            if (ctx->info.pic_header.ph_epmc_model_flag  && i >= IPD_EMCPM_C && i <= IPD_EMCPM_T_C)
            {
                continue;
            }
#endif

#if CCNPM && !CCNPM_TEMPLATE_OPT
            if (!ctx->info.sqh.ccnpm_enable_flag && i == IPD_CCNPM)
            {
                continue;
            }
            if (i == IPD_CCNPM && x <= 0 && y <= 0)
            {
                continue;
            }
#endif

#if CCNPM_TEMPLATE_OPT
            if (!ctx->info.sqh.ccnpm_enable_flag && (i == IPD_CCNPM || i == IPD_CCNPM_L || i == IPD_CCNPM_T))
            {
                continue;
            }
            if (i == IPD_CCNPM && x <= 0 && y <= 0)
            {
                continue;
            }
            if ((i == IPD_CCNPM_L || i == IPD_CCNPM_T) && (x <= 0 || y <= 0))
            {
                continue;
            }
#endif

#if ST_CHROMA
            int st_chroma_all = com_st_chroma_allow(mod_info_curr, ctx->info.sqh.st_chroma_enable_flag, ctx->tree_status) ? 2 : 1;
            assert(cu_width_log2 >= 3 && cu_height_log2 >= 3);
            assert(i != IPD_DM_C || best_ipd[PB0] != IPD_IPCM);
            assert(ctx->tree_status != TREE_L);

            for (int use_st_chroma = 0; use_st_chroma < st_chroma_all; use_st_chroma++)
            {
                mod_info_curr->st_chroma_flag = use_st_chroma;
#endif
                cost_temp = pintra_residue_rdo(ctx, core, org, org_cb, org_cr, s_org, s_org_c, cu_width_log2, cu_height_log2, &dist_t, 1, 0, x, y);
#if PRINT_CU_LEVEL_2
                printf("\nchro pred mode %2d cost_temp %10.1f", i, cost_temp);
                double val = 2815.9;
                if (cost_temp - val < 0.1 && cost_temp - val > -0.1)
                {
                    int a = 0;
                }
#endif
                if (cost_temp < cost_best)
                {
                    cost_best = cost_temp;
                    best_dist_c = dist_t;
                    best_ipd_c = i;
#if ECCPM
                    bst_info->eccpm_flag = mod_info_curr->eccpm_flag;
#endif                    
#if ST_CHROMA
                    bst_info->st_chroma_flag = mod_info_curr->st_chroma_flag;
#endif
                    for (j = U_C; j < N_C; j++)
                    {
                        int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
                        com_mcpy(bst_info->coef[j], mod_info_curr->coef[j], size_tmp * sizeof(s16));
                        com_mcpy(bst_info->rec[j], pi->rec[j], size_tmp * sizeof(pel));
                        bst_info->num_nz[TB0][j] = mod_info_curr->num_nz[TB0][j];
                    }
                }
#if ST_CHROMA
            }
#endif
        }
#if ECCPM
    }
#endif        
    }

    int pb_part_size_best = core->best_pb_part_intra;
    int tb_part_size_best = get_tb_part_size_by_pb(pb_part_size_best, MODE_INTRA);
    int num_pb_best = get_part_num(pb_part_size_best);
    int num_tb_best = get_part_num(tb_part_size_best);
    //some check
    if (ctx->tree_status == TREE_LC)
    {
        assert(best_ipd_c != IPD_INVALID);
    }
    else if (ctx->tree_status == TREE_L)
    {
        assert(bst_info->num_nz[TBUV0][U_C] == 0);
        assert(bst_info->num_nz[TBUV0][V_C] == 0);
        assert(best_dist_c == 0);
    }
    else if (ctx->tree_status == TREE_C)
    {
        assert(best_ipd_c != IPD_INVALID);
        assert(bst_info->num_nz[TBUV0][Y_C] == 0);
        assert(num_pb_best == 1 && num_tb_best == 1);
        assert(best_dist_y == 0);
    }
    else
        assert(0);
    //end of checks
    cu_nz_cln(mod_info_curr->num_nz);
    for (pb_part_idx = 0; pb_part_idx < num_pb_best; pb_part_idx++)
    {
        core->mod_info_best.mpm[pb_part_idx][0] = best_mpm[pb_part_idx][0];
        core->mod_info_best.mpm[pb_part_idx][1] = best_mpm[pb_part_idx][1];
        core->mod_info_best.ipm[pb_part_idx][0] = (s8)best_ipd[pb_part_idx];
        core->mod_info_best.ipm[pb_part_idx][1] = (s8)best_ipd_c;
        mod_info_curr->ipm[pb_part_idx][0] = core->mod_info_best.ipm[pb_part_idx][0];
        mod_info_curr->ipm[pb_part_idx][1] = core->mod_info_best.ipm[pb_part_idx][1];
    }
    for (int tb_part_idx = 0; tb_part_idx < num_tb_best; tb_part_idx++)
    {
        for (j = 0; j < N_C; j++)
        {
            mod_info_curr->num_nz[tb_part_idx][j] = bst_info->num_nz[tb_part_idx][j];
        }
    }
#if ECCPM
    mod_info_curr->eccpm_flag = bst_info->eccpm_flag;
#endif
#if IST
    mod_info_curr->ist_tu_flag = bst_info->ist_tu_flag;
#endif
#if EST
    mod_info_curr->est_flag = bst_info->est_flag;
#endif
#if DEST
	mod_info_curr->dest_flag = bst_info->dest_flag;
#endif
#if ST_CHROMA
    mod_info_curr->st_chroma_flag = bst_info->st_chroma_flag;
#endif
#if TB_SPLIT_EXT
    mod_info_curr->pb_part = core->best_pb_part_intra;
    mod_info_curr->tb_part = core->best_tb_part_intra;
#endif
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    //enc_sbac_bit_reset(&core->s_temp_run);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_intra(ctx, core, ctx->info.pic_header.slice_type, bst_info->coef);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best = (best_dist_y + best_dist_c) + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    core->dist_cu = best_dist_y + best_dist_c;
#if DT_SAVE_LOAD
    if (pData->num_intra_history < 2 && core->mod_info_curr.ipf_flag == 0 && ctx->tree_status != TREE_C)
    {
        pData->best_part_size_intra[pData->num_intra_history++] = core->best_pb_part_intra;
    }
#endif
    return cost_best;
}

#if SAWP

void sort_sawp_partition_list(int awp_idx, int candidate_idx, double sad_cost, u8 bestCandX_idx[AWP_MODE_NUM][SAWP_CPP], double bestCandX_cost[AWP_MODE_NUM][SAWP_CPP])
{
    int shift = 0;
    while (shift < SAWP_CPP && sad_cost < bestCandX_cost[awp_idx][SAWP_CPP - 1 - shift])
    {
        shift++;
    }
    if (shift != 0)
    {
        for (int i = 1; i < shift; i++)
        {
            bestCandX_idx[awp_idx][SAWP_CPP - i] = bestCandX_idx[awp_idx][SAWP_CPP - 1 - i];
            bestCandX_cost[awp_idx][SAWP_CPP - i] = bestCandX_cost[awp_idx][SAWP_CPP - 1 - i];
        }
        bestCandX_idx[awp_idx][SAWP_CPP - shift] = candidate_idx;
        bestCandX_cost[awp_idx][SAWP_CPP - shift] = sad_cost;
    }
}

#if AWP_ENH
void sort_sawp_satd_list(int awp_idx, double satd_cost, u8 candidate0, u8 candidate1, int best_satd_cand_idx[SAWP_RDO_NUM], double best_satd_cost[SAWP_RDO_NUM], u8 best_cand0_idx[SAWP_RDO_NUM], u8 best_cand1_idx[SAWP_RDO_NUM])
#else
void sort_sawp_satd_list(int awp_idx, double satd_cost, u8 candidate0, u8 candidate1, u8 best_satd_cand_idx[SAWP_RDO_NUM], double best_satd_cost[SAWP_RDO_NUM], u8 best_cand0_idx[SAWP_RDO_NUM], u8 best_cand1_idx[SAWP_RDO_NUM])
#endif
{
    int shift = 0;
    while (shift < SAWP_RDO_NUM && satd_cost < best_satd_cost[SAWP_RDO_NUM - 1 - shift])
    {
        shift++;
    }
    if (shift != 0)
    {
        for (int i = 1; i < shift; i++)
        {
            best_satd_cand_idx[SAWP_RDO_NUM - i] = best_satd_cand_idx[SAWP_RDO_NUM - 1 - i];
            best_satd_cost[SAWP_RDO_NUM - i] = best_satd_cost[SAWP_RDO_NUM - 1 - i];
            best_cand0_idx[SAWP_RDO_NUM - i] = best_cand0_idx[SAWP_RDO_NUM - 1 - i];
            best_cand1_idx[SAWP_RDO_NUM - i] = best_cand1_idx[SAWP_RDO_NUM - 1 - i];
        }
        best_satd_cand_idx[SAWP_RDO_NUM - shift] = awp_idx;
        best_satd_cost[SAWP_RDO_NUM - shift] = satd_cost;
        best_cand0_idx[SAWP_RDO_NUM - shift] = candidate0;
        best_cand1_idx[SAWP_RDO_NUM - shift] = candidate1;
    }
}

#if AWP_ENH
int enc_bit_est_sawp_blend_idx(u8 idx)
{
    u8 bits = 0;
    if (idx == 0)
    {
        bits += 1;  // 0
    }
    else
    {
        bits += 1;  // 1
        if (idx == 1 || idx == 2)
        {
            bits += 1;  //10
            bits += 1;  // 100 / 101
        }
        else
        {
            bits += 1; // 11
            bits += 1;  // 110 / 111

        }
    }
    return bits;
}
#endif

#if SAWP_MPM_SIMP
void enc_bit_est_idx0_idx1(u8* bit_cnt0, u8* bit_cnt1, u8 sawp_idx0, u8 sawp_idx1, u8 sawp_mpm[SAWP_MPM_NUM])
{
    u8 ipm = sawp_idx0;
    u8* mpm = sawp_mpm;
    int dir0_mpm_idx = -1;
    int ipm_code = ipm == mpm[0] ? -2 : mpm[1] == ipm ? -1 : ipm < mpm[0] ? ipm : ipm < mpm[1] ? ipm - 1 : ipm - 2;
    if (ipm_code > 0)
    {
        ipm_code -= 5;
    }
    if (ipm_code < 0)
    {
        dir0_mpm_idx = ipm_code + 2;
        *bit_cnt0 = 2;
    }
    else
    {
        if (ipm_code % 3 == 2)
        {
            *bit_cnt0 = 5;
        }
        else
        {
            *bit_cnt0 = 6;
        }
    }

    ipm = sawp_idx1;
    ipm_code = ipm == mpm[0] ? -2 : mpm[1] == ipm ? -1 : ipm < mpm[0] ? ipm : ipm < mpm[1] ? ipm - 1 : ipm - 2;
    if (ipm_code > 0)
    {
        ipm_code -= 5;
    }
    if (ipm_code < 0)
    {
        if (dir0_mpm_idx == -1)
        {
            *bit_cnt1 = 2;
        }
        else
        {
            *bit_cnt1 = 1;
        }
    }
    else
    {
        if (ipm_code % 3 == 2)
        {
            *bit_cnt1 = 5;
        }
        else
        {
            *bit_cnt1 = 6;
        }
    }
}
#else // SAWP_MPM_SIMP
void enc_bit_est_idx0_idx1(u8 *bit_cnt0, u8 *bit_cnt1, u8 sawp_idx0, u8 sawp_idx1, u8 sawp_mpm[SAWP_MPM_NUM])
{
    u8 ipm = sawp_idx0;
    u8* mpm = sawp_mpm;
    int dir0_mpm_idx = -1;
    int ipm_code = ipm == mpm[0] ? -4 : mpm[1] == ipm ? -3 : mpm[2] == ipm ? -2 : mpm[3] == ipm ? -1 : ipm < mpm[0] ? ipm : ipm < mpm[1] ? ipm - 1 : ipm < mpm[2] ? ipm - 2 : ipm < mpm[3] ? ipm - 3 : ipm - 4;
    if (ipm_code > 0)
    {
        ipm_code -= 4;
    }
    if (ipm_code < 0)
    {
        dir0_mpm_idx = ipm_code + 4;
        *bit_cnt0 = 3;
    }
    else
    {
        if (ipm_code % 3 == 2)
        {
            *bit_cnt0 = 5;
        }
        else
        {
            *bit_cnt0 = 6;
        }
    }
    ipm = sawp_idx1;
    ipm_code = ipm == mpm[0] ? -4 : mpm[1] == ipm ? -3 : mpm[2] == ipm ? -2 : mpm[3] == ipm ? -1 : ipm < mpm[0] ? ipm : ipm < mpm[1] ? ipm - 1 : ipm < mpm[2] ? ipm - 2 : ipm < mpm[3] ? ipm - 3 : ipm - 4;
    if (ipm_code > 0)
    {
        ipm_code -= 4;
    }
    if (ipm_code < 0)
    {
        if (dir0_mpm_idx == -1)
        {
            *bit_cnt1 = 3;
        }
        else
        {
            int mpm_idx = ipm_code + 4;
            assert(mpm_idx != dir0_mpm_idx);
            if (mpm_idx > dir0_mpm_idx)
            {
                mpm_idx--;
            }
            if (mpm_idx == 0)
            {
                *bit_cnt1 = 2;
            }
            else
            {
                *bit_cnt1 = 3;
            }
        }
    }
    else
    {
        if (ipm_code % 3 == 2)
        {
            *bit_cnt1 = 5;
        }
        else
        {
            *bit_cnt1 = 6;
        }

    }
}
#endif // SAWP_MPM_SIMP

#if AWP_ENH
static inline u32 sort_sawp_mode(double* cost_list, int cost_len, int* idx_list, int idx_len)
{
    int num_valid = 1;
    double sorted_list[AWP_MODE_NUM];
    sorted_list[0] = cost_list[0];
    idx_list[0] = (int)0;

    for (int idx = 1; idx < cost_len; ++idx)
    {
        int insert_idx = 0;
        double* sub_list = sorted_list;
        u32 sub_list_size = num_valid;
        while (sub_list_size > 1)
        {
            int middle_idx = sub_list_size >> 1;
            if (cost_list[idx] < sub_list[middle_idx])
            {
                sub_list_size = middle_idx;
            }
            else
            {
                sub_list += middle_idx;
                sub_list_size -= middle_idx;
                insert_idx += middle_idx;
            }
        }
        insert_idx += (cost_list[idx] < sub_list[0] ? 0 : 1);
        if (insert_idx < idx_len)
        {
            int start_idx = idx_len - 1;
            if (num_valid < idx_len)
            {
                start_idx = num_valid;
                ++num_valid;
            }

            for (int i = start_idx; i > insert_idx; --i)
            {
                sorted_list[i] = sorted_list[i - 1];
                idx_list[i] = idx_list[i - 1];
            }
            sorted_list[insert_idx] = cost_list[idx];
            idx_list[insert_idx] = (int)idx;
        }
    }

    return num_valid;
}
#endif

double analyze_sawp_cu(ENC_CTX* ctx, ENC_CORE* core)
{
    ENC_PINTRA* pi = &ctx->pintra;
    COM_MODE* bst_info = &core->mod_info_best;
    COM_MODE* mod_info_curr = &core->mod_info_curr;
    int bit_depth = ctx->info.bit_depth_internal;
    int i, j, s_org, s_org_c, s_mod, s_mod_c;
    int best_ipd[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_pb_part[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    /* best sawp xx */
    int best_sawp_flag = 0;
    int best_sawp_idx = -1;
#if AWP_ENH
    int best_awp_blend_idx = -1;
#endif
    int best_sawp_cand0 = -1;
    int best_sawp_cand1 = -1;
#if DSAWP
    int best_dawp_idx = -1;
    int best_dawp_cand0 = -1;
    int best_dawp_cand1 = -1;
#endif
    /* best sawp xx */
    int best_ipd_c = IPD_INVALID;
    s32 best_dist_y = 0, best_dist_c = 0;
    s32 best_dist_y_pb_part[MAX_NUM_TB] = { 0, 0, 0, 0 };
    u8  best_mpm_pb_part[MAX_NUM_TB][2];
    u8  best_mpm[MAX_NUM_TB][2];
    best_mpm_pb_part[0][0] = 0;
    best_mpm_pb_part[0][1] = 0;
    best_mpm[0][0] = 0;
    best_mpm[0][1] = 0;

    u8 best_sawp_mpm[SAWP_MPM_NUM] = { 0 };

    s16  coef_y_pb_part[MAX_CU_DIM];
    pel  rec_y_pb_part[MAX_CU_DIM];
    int  num_nz_y_pb_part[MAX_NUM_TB];
    int ipm_l2c = 0;
    int chk_bypass = 0;
    int bit_cnt = 0;
    int pred_cnt = IPD_CNT;
    pel* org, * mod;
    pel* org_cb, * org_cr;
    pel* mod_cb, * mod_cr;
    double cost_temp, cost_best = MAX_COST;
    double cost_pb_temp, cost_pb_best;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;

    if (!(ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L))
    {
        return MAX_COST;
    }

    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int cu_x = mod_info_curr->x_pos;
    int cu_y = mod_info_curr->y_pos;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if EST
    double costdct2[SAWP_RDO_NUM] = { MAX_COST };
    int cbfdct2[SAWP_RDO_NUM] = { 1 };
    int cbfdst7[SAWP_RDO_NUM] = { 1 };
    int bst_est_tu_flag = 0;
    bst_info->est_flag = 0;
    mod_info_curr->est_flag = 0;
#endif
#if DEST
	int bst_dest_tu_flag = 0;
	bst_info->dest_flag = 0;
	mod_info_curr->dest_flag = 0;
#endif
#if ST_CHROMA
    bst_info->st_chroma_flag = 0;
    mod_info_curr->st_chroma_flag = 0;
#endif
#if USE_IBC
    mod_info_curr->cu_mode = MODE_INTRA;
    mod_info_curr->ibc_flag = 0;
#endif
#if CIBC
    mod_info_curr->cibc_flag = 0;
#endif
#if USE_SP
    mod_info_curr->sp_flag = 0;
#if RSD_OPT
    mod_info_curr->sp_rsd_flag = 0;
#endif
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if SAWP_SCC
    mod_info_curr->ph_awp_refine_flag = ctx->info.pic_header.ph_awp_refine_flag;
#endif
#if DEST_PH
    mod_info_curr->ph_dest_enable_flag = ctx->info.pic_header.ph_dest_enable_flag;
#endif
    int ipd_buf[4] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int ipd_add[4][3] = { { IPD_INVALID, IPD_INVALID, IPD_INVALID },{ IPD_INVALID, IPD_INVALID, IPD_INVALID },
        { IPD_INVALID, IPD_INVALID, IPD_INVALID },{ IPD_INVALID, IPD_INVALID, IPD_INVALID }
    };

#if SAWP_COST_FAST
    ENC_BEF_DATA* pData = &core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup];
#endif //SAWP_COST_FAST

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
    s_mod = pi->stride_rec[Y_C];
    s_org = pi->stride_org[Y_C];
    s_mod_c = pi->stride_rec[U_C];
    s_org_c = pi->stride_org[U_C];
    mod = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
    org = pi->addr_org[Y_C] + (y * s_org) + x;
    int pb_part_idx;
    core->best_pb_part_intra = SIZE_2Nx2N;

#if DSAWP
    int left_tpl_w[2] = { 0, 1 };
    int above_tpl_h[2] = { 1, 0 };
    int use_dawp = ctx->slice_type == SLICE_P ? 0 : 1;
    if (use_dawp)
    {
        init_awp_tpl(mod_info_curr, ctx->awp_weight_tpl);
        com_get_tpl_cur(PIC_REC(ctx), ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, mod_info_curr);
    }
#endif

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
    {
        int allowed_part_size[7] = { SIZE_2Nx2N };
        int num_allowed_part_size = 1;

        cost_best = MAX_COST;

        /*  only SIZE_2Nx2N is used in SAWP  i.e. no dt in SAWP  */
        PART_SIZE pb_part_size = SIZE_2Nx2N;
        PART_SIZE tb_part_size = SIZE_2Nx2N;
        set_pb_part(mod_info_curr, pb_part_size);
        set_tb_part(mod_info_curr, tb_part_size);
        get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, pb_part_size, &mod_info_curr->pb_info);
        get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, tb_part_size, &mod_info_curr->tb_info);
        assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
        cost_temp = 0;
        memset(num_nz_y_pb_part, 0, MAX_NUM_TB * sizeof(int));

        // pb-based intra mode candidate selection
        pb_part_idx = 0;
        {
            int pb_x = mod_info_curr->pb_info.sub_x[pb_part_idx];
            int pb_y = mod_info_curr->pb_info.sub_y[pb_part_idx];
            int pb_w = mod_info_curr->pb_info.sub_w[pb_part_idx];
            int pb_h = mod_info_curr->pb_info.sub_h[pb_part_idx];
            int pb_scup = mod_info_curr->pb_info.sub_scup[pb_part_idx];
            int pb_x_scu = PEL2SCU(pb_x);
            int pb_y_scu = PEL2SCU(pb_y);
            int pb_coef_offset = get_coef_offset_tb(mod_info_curr->x_pos, mod_info_curr->y_pos, pb_x, pb_y, cu_width, cu_height, tb_part_size);
            int tb_idx_offset = get_tb_idx_offset(pb_part_size, pb_part_idx);
            int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_part_idx);
            int skip_ipd = 0;

            cost_pb_best = MAX_COST;
            cu_nz_cln(mod_info_curr->num_nz);

            mod = pi->addr_rec_pic[Y_C] + (pb_y * s_mod) + pb_x;
            org = pi->addr_org[Y_C] + (pb_y * s_org) + pb_x;

            u16 avail_cu = com_get_avail_intra(pb_x_scu, pb_y_scu, ctx->info.pic_width_in_scu, pb_scup, ctx->map.map_scu
#if CCNPM
                , ctx->info.pic_height_in_scu, pb_w, pb_h
#endif
            );
            com_get_nbr(pb_x, pb_y, pb_w, pb_h, mod, s_mod, avail_cu, core->nb, pb_scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);

#if DSAWP
            const int tpl_ref_offset = 1 + DAWP_REF_OFFSET;
            const int tpl_ref_size = MIN_CU_SIZE;
            if (pb_x - tpl_ref_offset < 0 || pb_y - tpl_ref_offset < 0)
            {
                mod_info_curr->tpl_cur_avail[0] = 0;
                mod_info_curr->tpl_cur_avail[1] = 0;
            }
            com_dawp_get_nbr(pb_x, pb_y, pb_w, pb_h, mod, s_mod, avail_cu, core->nb_tpl, pb_scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);
#endif
            /*  make sawp mpm list for each awp mode  */
            u8 sawp_mpm[AWP_MODE_NUM][SAWP_MPM_NUM];
            for (int awp_idx = 0; awp_idx < AWP_MODE_NUM; awp_idx++)
            {
#if DSAWP
                com_get_sawp_mpm(PEL2SCU(pb_x), PEL2SCU(pb_y), PEL2SCU(pb_w), PEL2SCU(pb_h), ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, sawp_mpm[awp_idx], awp_idx, NULL);
#else
                com_get_sawp_mpm(PEL2SCU(pb_x), PEL2SCU(pb_y), PEL2SCU(pb_w), PEL2SCU(pb_h), ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, sawp_mpm[awp_idx], awp_idx);
#endif
            }

            enc_init_awp_template(ctx); // only once per sequence

            /*  get SAD for all candidates  */
            u64 sad_whole_blk[IPD_CNT] = { 0 };
            u64 best_whole_blk_sad = cu_width * cu_height * (1 << 16);
            double best_whole_blk_cost = 0;
            double cand_cost[IPD_CNT];
#if SAWP_MPM_SIMP
            for (int ipd_idx = 5; ipd_idx < 31; ipd_idx++)
#else // SAWP_MPM_SIMP
            for (int ipd_idx = 4; ipd_idx < 32; ipd_idx++)
#endif // SAWP_MPM_SIMP
            {
                if (ipd_idx == IPD_IPCM)
                {
                    continue;
                }
#if DSAWP
                if (use_dawp && mod_info_curr->tpl_cur_avail[0] == 1 && mod_info_curr->tpl_cur_avail[1] == 1)
                {
                    int dir;
                    const int tpl_size = 1;
                    int tpl_height[2], tpl_width[2];
                    tpl_width[0] = cu_width;
                    tpl_height[0] = tpl_size;
                    tpl_width[1] = tpl_size;
                    tpl_height[1] = cu_height;
                    int tpl_stride = 1;
                    int offset[2] = { 0, AWP_TPL_SIZE };
                    int pred_offset_x[2] = { 1 , 0 };
                    int pred_offset_y[2] = { 0 , 1 };
                    for (dir = 0; dir < 2; dir++)
                    {
                        com_tpl_ipred(core->nb_tpl[0][0] + STNUM, core->nb_tpl[0][1] + STNUM, pi->pred_cache[ipd_idx], ipd_idx, cu_width, cu_height, bit_depth, avail_cu, core->mod_info_curr.ipf_flag
#if MIPF
                            , ctx->info.sqh.mipf_enable_flag
#endif
#if IIP
                            , mod_info_curr->iip_flag
#endif
                            , left_tpl_w[dir], above_tpl_h[dir]
                        );
                        for (int i = 0; i < tpl_height[dir]; i++)
                        {
                            for (int j = 0; j < tpl_width[dir]; j++)
                            {
                                pi->pred_tpl_cache[ipd_idx][i * tpl_stride + j + offset[dir]] = pi->pred_cache[ipd_idx][(i + pred_offset_y[dir]) * (cu_width + 1) + (j + pred_offset_x[dir])];
                            }
                        }
                    }
                }
#endif
                com_ipred(core->nb[0][0] + STNUM, core->nb[0][1] + STNUM, pi->pred_cache[ipd_idx], ipd_idx, cu_width, cu_height, bit_depth, avail_cu, core->mod_info_curr.ipf_flag
#if MIPF
                    , ctx->info.sqh.mipf_enable_flag
#endif
#if IIP
                    , mod_info_curr->iip_flag
#endif
                );
                sad_whole_blk[ipd_idx] = calc_sad_16b(cu_width, cu_height, org, pi->pred_cache[ipd_idx], s_org, cu_width, bit_depth);

                cand_cost[ipd_idx] = (RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], 6)) / 2;
                if (sad_whole_blk[ipd_idx] < best_whole_blk_sad)
                {
                    best_whole_blk_sad = sad_whole_blk[ipd_idx];
                    best_whole_blk_cost = (double)best_whole_blk_sad + (double)cand_cost[ipd_idx];
                }
            }

            /*  get best combination for each awp mode  */
            u8 best_cand0_idx[AWP_MODE_NUM][SAWP_CPP];
            u8 best_cand1_idx[AWP_MODE_NUM][SAWP_CPP];
            double best_cand0_cost[AWP_MODE_NUM][SAWP_CPP];
            double best_cand1_cost[AWP_MODE_NUM][SAWP_CPP];


            //set largest sad
            for (int awp_idx = 0; awp_idx < AWP_MODE_NUM; awp_idx++)
            {
                for (int candidate_idx = 0; candidate_idx < SAWP_CPP; candidate_idx++)
                {
                    best_cand0_idx[awp_idx][candidate_idx] = -1;
                    best_cand1_idx[awp_idx][candidate_idx] = -1;
                    best_cand0_cost[awp_idx][candidate_idx] = MAX_COST;
                    best_cand1_cost[awp_idx][candidate_idx] = MAX_COST;
                }
            }

            for (int awp_idx = 0; awp_idx < AWP_MODE_NUM; awp_idx++)
            {
                u64 sad_small = 0;
                u64 sad_large = 0;
                double awp_cost0 = 0;
                double awp_cost1 = 0;
#if SAWP_MPM_SIMP
                for (int ipd_idx = 5; ipd_idx < 31; ipd_idx++)
#else // SAWP_MPM_SIMP
                for (int ipd_idx = 4; ipd_idx < 32; ipd_idx++)
#endif // SAWP_MPM_SIMP
                {
                    if (ipd_idx == IPD_IPCM)
                    {
                        continue;
                    }

                    BOOL bMPM = FALSE;
                    double bit_cost = 0;
                    for (int i = 0; i < SAWP_MPM_NUM; i++)
                    {
                        if (ipd_idx == sawp_mpm[awp_idx][i])
                        {
                            bMPM = TRUE;
                            break;
                        }
                    }
                    if (bMPM)
                    {
                        bit_cost = (RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], 3)) / 2;
                    }
                    else
                    {
                        bit_cost = (RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], 6)) / 2;
                    }

                    if (ctx->awp_larger_area[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx] == 0)
                    {
                        sad_small = calc_sad_mask_16b(cu_width, cu_height, org, pi->pred_cache[ipd_idx], ctx->awp_bin_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], s_org, cu_width, cu_width, bit_depth);

                        awp_cost1 = (double)sad_small + bit_cost;
                        sad_large = sad_whole_blk[ipd_idx] - sad_small;
                        awp_cost0 = (double)sad_large + bit_cost;

                    }
                    else
                    {
                        sad_small = calc_sad_mask_16b(cu_width, cu_height, org, pi->pred_cache[ipd_idx], ctx->awp_bin_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], s_org, cu_width, cu_width, bit_depth);

                        awp_cost0 = (double)sad_small + bit_cost;
                        sad_large = sad_whole_blk[ipd_idx] - sad_small;
                        awp_cost1 = (double)sad_large + bit_cost;
                    }
                    sort_sawp_partition_list(awp_idx, ipd_idx, awp_cost0, best_cand0_idx, best_cand0_cost);

                    sort_sawp_partition_list(awp_idx, ipd_idx, awp_cost1, best_cand1_idx, best_cand1_cost);
                }
            }

            /*  use SATD to get best candidates for RDO  */
            double satd_cost_y = 0;
            int bit_cnt_satd = 0;
#if AWP_ENH
            int best_satd_awp_idx[SAWP_RDO_NUM];
#else
            u8 best_satd_awp_idx[SAWP_RDO_NUM];
#endif
#if AWP_ENH
            double awp_best_cost[AWP_MODE_NUM];
            int awp_best_cost_idx[AWP_MODE_NUM];
            for (int i = 0; i < AWP_MODE_NUM;i++)
            {
                awp_best_cost[i] = MAX_COST;
                awp_best_cost_idx[i] = i;
            }
            int use_aawp = ctx->info.sqh.aawp_enable_flag && !mod_info_curr->ph_awp_refine_flag && ctx->info.pic_header.slice_type != SLICE_P ? 1 : 0;
#endif
            double best_satd_cost[SAWP_RDO_NUM];
            u8 best_cand0[SAWP_RDO_NUM];
            u8 best_cand1[SAWP_RDO_NUM];

            for (int i = 0; i < SAWP_RDO_NUM; i++)
            {
                best_satd_awp_idx[i] = 0;
                best_satd_cost[i] = MAX_COST;
                best_cand0[i] = -1;
                best_cand1[i] = -1;
            }

            int awp_stad_num = 0;
            u8 temp_cand0;
            u8 temp_cand1;

            for (int awp_idx = 0; awp_idx < AWP_MODE_NUM; awp_idx++)
            {
                for (int cand_per_mode0 = 0; cand_per_mode0 < SAWP_CPP; cand_per_mode0++)
                {
                    for (int cand_per_mode1 = 0; cand_per_mode1 < SAWP_CPP; cand_per_mode1++)
                    {
                        mod_info_curr->sawp_flag = 1;
                        mod_info_curr->skip_idx = awp_idx;
                        temp_cand0 = (u8)best_cand0_idx[awp_idx][cand_per_mode0];
                        temp_cand1 = (u8)best_cand1_idx[awp_idx][cand_per_mode1];
                        mod_info_curr->sawp_idx0 = temp_cand0;
                        mod_info_curr->sawp_idx1 = temp_cand1;

                        if (mod_info_curr->sawp_idx0 == mod_info_curr->sawp_idx1)
                            continue;

                        double temp_cost = best_cand0_cost[awp_idx][cand_per_mode0] + best_cand1_cost[awp_idx][cand_per_mode1];
                        if (temp_cost > best_whole_blk_cost)
                            continue;

                        awp_stad_num++;

                        /* combine two pred buf */
#if SAWP_SCC
                        if (mod_info_curr->ph_awp_refine_flag)
                        {
                            com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0_scc[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], ctx->awp_weight1_scc[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx]);
                        }
                        else
                        {
#if SAWP_WEIGHT_OPT && !AWP_ENH
                            int sawp_weight_idx = awp_idx + AWP_MODE_NUM;
                            com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][sawp_weight_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][sawp_weight_idx]);
#else
                            com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx]);
#endif

                        }
#else
                        com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx]);
#endif
                        satd_cost_y = (double)calc_satd_16b(cu_width, cu_height, org, mod_info_curr->pred[Y_C], s_org, cu_width, bit_depth);

                        int bit_cnt_awp = 5;
                        if (awp_idx > 7)
                        {
                            bit_cnt_awp++;
                        }

#if AWP_ENH
                        if (use_aawp)
                        {
                            bit_cnt_awp += enc_bit_est_sawp_blend_idx(0);
                        }
#endif

                        u8 bit_cnt0 = 6;
                        u8 bit_cnt1 = 6;

                        enc_bit_est_idx0_idx1(&bit_cnt0, &bit_cnt1, temp_cand0, temp_cand1, sawp_mpm[awp_idx]);
                        bit_cnt_satd = bit_cnt0 + bit_cnt1 + bit_cnt_awp;
                        satd_cost_y += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt_satd);

                        sort_sawp_satd_list(awp_idx, satd_cost_y, best_cand0_idx[awp_idx][cand_per_mode0], best_cand1_idx[awp_idx][cand_per_mode1], best_satd_awp_idx, best_satd_cost, best_cand0, best_cand1);
#if AWP_ENH
                        if (satd_cost_y < awp_best_cost[awp_idx])
                        {
                            awp_best_cost[awp_idx] = satd_cost_y;
                        }
#endif
                    }
                }
            }

#if AWP_ENH
            if (use_aawp)
            {
                const int total_awp_mode = TOTAL_BLEND_NUM;
                const int test_awp_mode_num = 8;
                sort_sawp_mode(awp_best_cost, AWP_MODE_NUM, awp_best_cost_idx, test_awp_mode_num);
                for (int blend_idx = 1; blend_idx < total_awp_mode; blend_idx++)
                {
                    for (int test_awp_idx = 0; test_awp_idx < test_awp_mode_num; test_awp_idx++)
                    {
                        int awp_idx = awp_best_cost_idx[test_awp_idx];
                        for (int cand_per_mode0 = 0; cand_per_mode0 < SAWP_CPP; cand_per_mode0++)
                        {
                            for (int cand_per_mode1 = 0; cand_per_mode1 < SAWP_CPP; cand_per_mode1++)
                            {
                                mod_info_curr->sawp_flag = 1;
                                mod_info_curr->skip_idx = awp_idx;
                                temp_cand0 = (u8)best_cand0_idx[awp_idx][cand_per_mode0];
                                temp_cand1 = (u8)best_cand1_idx[awp_idx][cand_per_mode1];
                                mod_info_curr->sawp_idx0 = temp_cand0;
                                mod_info_curr->sawp_idx1 = temp_cand1;
                                if (mod_info_curr->sawp_idx0 == mod_info_curr->sawp_idx1)
                                    continue;
                                double temp_cost = best_cand0_cost[awp_idx][cand_per_mode0] + best_cand1_cost[awp_idx][cand_per_mode1];
                                if (temp_cost > best_whole_blk_cost)
                                    continue;
                                awp_stad_num++;
#if AWP_ENH
                                int blend_awp_idx = awp_idx + blend_idx * AWP_MODE_NUM;
#endif
                                /* combine two pred buf */
#if SAWP_SCC
                                if (mod_info_curr->ph_awp_refine_flag)
                                {
                                    com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0_scc[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], ctx->awp_weight1_scc[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx]);
                                }
                                else
                                {
#if AWP_ENH
                                    com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][blend_awp_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][blend_awp_idx]);
#else
#if SAWP_WEIGHT_OPT
                                    int sawp_weight_idx = awp_idx + AWP_MODE_NUM;
                                    com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][sawp_weight_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][sawp_weight_idx]);
#else
                                    com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx]);
#endif
#endif
                                }
#else
                                com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[temp_cand0], pi->pred_cache[temp_cand1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][awp_idx]);
#endif
                                satd_cost_y = (double)calc_satd_16b(cu_width, cu_height, org, mod_info_curr->pred[Y_C], s_org, cu_width, bit_depth);
                                int bit_cnt_awp = 5;
                                if (awp_idx > 7)
                                {
                                    bit_cnt_awp++;
                                }
#if AWP_ENH
                                bit_cnt_awp += enc_bit_est_sawp_blend_idx(blend_idx);
#endif
                                u8 bit_cnt0 = 6;
                                u8 bit_cnt1 = 6;
                                enc_bit_est_idx0_idx1(&bit_cnt0, &bit_cnt1, temp_cand0, temp_cand1, sawp_mpm[awp_idx]);
                                bit_cnt_satd = bit_cnt0 + bit_cnt1 + bit_cnt_awp;
                                satd_cost_y += RATE_TO_COST_SQRT_LAMBDA(ctx->sqrt_lambda[0], bit_cnt_satd);
#if AWP_ENH
                                sort_sawp_satd_list(blend_awp_idx, satd_cost_y, best_cand0_idx[awp_idx][cand_per_mode0], best_cand1_idx[awp_idx][cand_per_mode1], best_satd_awp_idx, best_satd_cost, best_cand0, best_cand1);
#else
                                sort_sawp_satd_list(awp_idx, satd_cost_y, best_cand0_idx[awp_idx][cand_per_mode0], best_cand1_idx[awp_idx][cand_per_mode1], best_satd_awp_idx, best_satd_cost, best_cand0, best_cand1);
#endif
                            }
                        }
                    }
                }
            }
#endif

            if (pred_cnt == 0)
            {
                return MAX_COST;
            }
            if (awp_stad_num == 0)
            {
                return MAX_COST;
            }

#if EST
            int st_all = (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0) ?
#if DEST
#if ODEST
#if DEST_PH
                (mod_info_curr->ph_dest_enable_flag ? 4 : 2)
#else
                4
#endif
#else
#if DEST_PH
                (mod_info_curr->ph_dest_enable_flag ? 5 : 2)
#else
                5
#endif
#endif
#else
                2
#endif
                : 1;
#if DSAWP
            static u8 dawp_check[DAWP_IPM_CHECK_NUM][DAWP_IPM_CHECK_NUM] = { 0 };
            const int check_len = DAWP_IPM_CHECK_NUM * DAWP_IPM_CHECK_NUM;
            memset(dawp_check, 0, check_len * sizeof(u8));
            static int dawp_inv_mode_list[DAWP_IPM_CHECK_NUM][DAWP_IPM_CHECK_NUM][AWP_MODE_NUM] = { 0 };
#endif
            for (int use_st = 0; use_st < st_all; use_st++)
            {
                mod_info_curr->est_flag = use_st;
                if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part != 0)
                    mod_info_curr->est_flag = 1;
#endif

                /*  RDO process for selected candidates  */
                for (int awp_rdo_idx = 0; awp_rdo_idx < COM_MIN(SAWP_RDO_NUM, awp_stad_num); awp_rdo_idx++)
                {
                    //printf("\nRDO");
                    mod_info_curr->sawp_flag = 1;
#if AWP_ENH
                    int blend_awp_idx = best_satd_awp_idx[awp_rdo_idx];
                    mod_info_curr->skip_idx = blend_awp_idx % AWP_MODE_NUM;
                    mod_info_curr->awp_blend_idx = blend_awp_idx / AWP_MODE_NUM;
#else
                    mod_info_curr->skip_idx = best_satd_awp_idx[awp_rdo_idx];
#endif
                    mod_info_curr->sawp_idx0 = best_cand0[awp_rdo_idx];
                    mod_info_curr->sawp_idx1 = best_cand1[awp_rdo_idx];

#if DSAWP
                    int awp_idx = mod_info_curr->skip_idx;
                    mod_info_curr->dawp_ipm0 = get_dawp_idx_from_sawp_idx(mod_info_curr->sawp_idx0, sawp_mpm[awp_idx]);
                    mod_info_curr->dawp_ipm1 = get_dawp_idx_from_sawp_idx(mod_info_curr->sawp_idx1, sawp_mpm[awp_idx]);
                    const int mpm_offset = 2;
                    int temp_cand0 = mod_info_curr->dawp_ipm0 + mpm_offset;
                    int temp_cand1 = mod_info_curr->dawp_ipm1 + mpm_offset;
                    if (use_dawp)
                    {
                        assert(temp_cand0 < DAWP_IPM_CHECK_NUM&& temp_cand1 < DAWP_IPM_CHECK_NUM);
                        if (dawp_check[temp_cand0][temp_cand1] == 0)
                        {
                            int mode_list[AWP_MODE_NUM];
                            int* inv_mode_list = &dawp_inv_mode_list[temp_cand0][temp_cand1][0];
                            dawp_check[temp_cand0][temp_cand1] = 1;
                            com_tpl_reorder_sawp_mode(mod_info_curr, mode_list, inv_mode_list
                                , &ctx->info, ctx->map.map_scu, ctx->map.map_ipm, core->nb_tpl, mod_info_curr->dawp_ipm0, mod_info_curr->dawp_ipm1
                                , sawp_mpm, pi->pred_tpl_cache
                            );
                            int dawp_idx = inv_mode_list[awp_idx];
                            if (dawp_idx >= 56)
                            {
                                assert(0);
                                continue;
                            }
                            mod_info_curr->dawp_idx = dawp_idx;
                        }
                        else
                        {
                            int* inv_mode_list = &dawp_inv_mode_list[temp_cand0][temp_cand1][0];
                            int dawp_idx = inv_mode_list[awp_idx];
                            if (dawp_idx >= 56)
                            {
                                assert(0);
                                continue;
                            }
                            mod_info_curr->dawp_idx = dawp_idx;
                        }
                    }
                    else
                    {
                        mod_info_curr->dawp_idx = mod_info_curr->skip_idx;
                    }
#endif
                    for (int mpm_idx = 0; mpm_idx < SAWP_MPM_NUM; mpm_idx++)
                    {
                        mod_info_curr->sawp_mpm[mpm_idx] = sawp_mpm[mod_info_curr->skip_idx][mpm_idx];
                    }
#if SAWP_SCC
                    if (mod_info_curr->ph_awp_refine_flag)
                    {
                        com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[mod_info_curr->sawp_idx0], pi->pred_cache[mod_info_curr->sawp_idx1], ctx->awp_weight0_scc[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][mod_info_curr->skip_idx], ctx->awp_weight1_scc[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][mod_info_curr->skip_idx]);
                    }
                    else
                    {
#if AWP_ENH
                        com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[mod_info_curr->sawp_idx0], pi->pred_cache[mod_info_curr->sawp_idx1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][blend_awp_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][blend_awp_idx]);
#else
#if SAWP_WEIGHT_OPT
                        int sawp_weight_idx = mod_info_curr->skip_idx + AWP_MODE_NUM;
                        com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[mod_info_curr->sawp_idx0], pi->pred_cache[mod_info_curr->sawp_idx1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][sawp_weight_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][sawp_weight_idx]);
#else
                        com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[mod_info_curr->sawp_idx0], pi->pred_cache[mod_info_curr->sawp_idx1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][mod_info_curr->skip_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][mod_info_curr->skip_idx]);
#endif
#endif
                    }
#else
                    com_derive_sawp_pred(mod_info_curr, Y_C, pi->pred_cache[mod_info_curr->sawp_idx0], pi->pred_cache[mod_info_curr->sawp_idx1], ctx->awp_weight0[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][mod_info_curr->skip_idx], ctx->awp_weight1[cu_width_log2 - MIN_AWP_SIZE_LOG2][cu_height_log2 - MIN_AWP_SIZE_LOG2][mod_info_curr->skip_idx]);
#endif
                    s32 dist_t = 0;

                    update_sawp_info_map_scu(mod_info_curr, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu);
                    int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;
                    mod_info_curr->ipm[pb_part_idx][0] = ctx->map.map_ipm[luma_scup];

                    mod_info_curr->ipm[pb_part_idx][1] = IPD_INVALID;
#if IST
                    int ist_all = (ctx->info.sqh.ist_enable_flag &&
#if EST
                    (ctx->info.sqh.est_enable_flag ? !mod_info_curr->est_flag : !mod_info_curr->tb_part) &&
#else
                        mod_info_curr->tb_part == 0 &&
#endif
                        cu_width_log2 < 6 && cu_height_log2 < 6) ? 2 : 1;
#if ETS
                    if (ctx->info.pic_header.ph_ists_enable_flag && mod_info_curr->cu_mode == MODE_INTRA && ist_all == 2)
                    {
                        ist_all = 3;
                    }
#endif
                    for (int use_ist = 0; use_ist < ist_all; use_ist++)
                    {
                        mod_info_curr->ist_tu_flag = use_ist;
#endif
#if EST
                        if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0 && mod_info_curr->est_flag)
                        {
                            if (cost_pb_best * 1.4 < costdct2[awp_rdo_idx])
                                continue;
                            if (!cbfdct2[awp_rdo_idx] && !cbfdst7[awp_rdo_idx])
                                continue;
                        }
#endif
                        cost_pb_temp = pintra_residue_rdo(ctx, core, org, NULL, NULL, s_org, s_org_c, cu_width_log2, cu_height_log2, &dist_t, 0, pb_part_idx, x, y);
#if EST
                        if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0 && !mod_info_curr->est_flag)
                        {
#if IST
                            if (mod_info_curr->ist_tu_flag)
                            {
                                cbfdst7[awp_rdo_idx] = mod_info_curr->num_nz[0][0];
                            }
                            else
#endif
                            {
                                costdct2[awp_rdo_idx] = cost_pb_temp;
                                cbfdct2[awp_rdo_idx] = mod_info_curr->num_nz[0][0];
                            }
                        }
#endif

                        if (cost_pb_temp < cost_pb_best)
                        {
                            cost_pb_best = cost_pb_temp;
                            best_dist_y_pb_part[pb_part_idx] = dist_t;

                            best_ipd_pb_part[pb_part_idx] = mod_info_curr->ipm[pb_part_idx][0];

                            best_mpm_pb_part[pb_part_idx][0] = mod_info_curr->mpm[pb_part_idx][0];
                            best_mpm_pb_part[pb_part_idx][1] = mod_info_curr->mpm[pb_part_idx][1];

                            /* save sawp info */
                            best_sawp_flag = mod_info_curr->sawp_flag;
                            best_sawp_idx = mod_info_curr->skip_idx;
#if AWP_ENH
                            best_awp_blend_idx = mod_info_curr->awp_blend_idx;
#endif
                            best_sawp_cand0 = mod_info_curr->sawp_idx0;
                            best_sawp_cand1 = mod_info_curr->sawp_idx1;
#if DSAWP
                            best_dawp_idx = mod_info_curr->dawp_idx;
                            best_dawp_cand0 = mod_info_curr->dawp_ipm0;
                            best_dawp_cand1 = mod_info_curr->dawp_ipm1;
#endif

                            for (int mpm_idx = 0; mpm_idx < SAWP_MPM_NUM; mpm_idx++)
                            {
                                best_sawp_mpm[mpm_idx] = mod_info_curr->sawp_mpm[mpm_idx];
                            }

#if IST
                            bst_ist_tu_flag = mod_info_curr->ist_tu_flag;
#endif
#if EST
                            bst_est_tu_flag = mod_info_curr->est_flag;
#endif
#if DEST
							bst_dest_tu_flag = mod_info_curr->dest_flag;
#endif
                            com_mcpy(coef_y_pb_part + pb_coef_offset, mod_info_curr->coef[Y_C], pb_w * pb_h * sizeof(s16));
                            for (int j = 0; j < pb_h; j++)
                            {
                                int rec_offset = ((pb_y - cu_y) + j) * cu_width + (pb_x - cu_x);
                                com_mcpy(rec_y_pb_part + rec_offset, pi->rec[Y_C] + rec_offset, pb_w * sizeof(pel));
                            }

                            for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
                            {
                                num_nz_y_pb_part[tb_idx + tb_idx_offset] = mod_info_curr->num_nz[tb_idx][Y_C];
                            }
#if EST
                            if (ctx->info.sqh.est_enable_flag)
                            {
                                SBAC_STORE(core->s_temp_prev_comp_run, core->s_temp_run);
                            }
                            else
                            {
                                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
                            }
#else
                            SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_run);
#endif
                        }
#if IST
                    }
#endif
#if SAWP_COST_FAST
                    if (ctx->param.i_period == 1 && awp_rdo_idx >= 3 && cost_pb_best > pData->intra_luma_cost)
                    {
                        break;
                    }
#endif // SAWP_COST_FAST
                }


#if EST
            }
            if (ctx->info.sqh.est_enable_flag)
            {
                SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_prev_comp_run);
            }
#endif
            cost_temp += cost_pb_best;

            //update map - pb
            mod_info_curr->sawp_flag = best_sawp_flag;
            mod_info_curr->skip_idx = best_sawp_idx;
#if AWP_ENH
            mod_info_curr->awp_blend_idx = best_awp_blend_idx;
#endif
            mod_info_curr->sawp_idx0 = best_sawp_cand0;
            mod_info_curr->sawp_idx1 = best_sawp_cand1;
#if DSAWP
            mod_info_curr->dawp_idx = best_dawp_idx;
            mod_info_curr->dawp_ipm0 = best_dawp_cand0;
            mod_info_curr->dawp_ipm1 = best_dawp_cand1;
#endif
            update_sawp_info_map_scu(mod_info_curr, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu);
            copy_rec_y_to_pic(rec_y_pb_part + (pb_y - cu_y) * cu_width + (pb_x - cu_x), pb_x, pb_y, pb_w, pb_h, cu_width, PIC_REC(ctx));

        }

        com_mcpy(pi->rec[Y_C], rec_y_pb_part, cu_width * cu_height * sizeof(pel));

        //save luma cb decision for each pb_part_size
        if (cost_temp < cost_best)
        {
            cost_best = cost_temp;
            best_dist_y = 0;
            int pb_idx = 0; /*only 1 part*/
            {
                best_dist_y += best_dist_y_pb_part[pb_idx];
                int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;
                best_ipd[pb_idx] = ctx->map.map_ipm[luma_scup];
                bst_info->ipm[pb_idx][0] = best_ipd[pb_idx];
                best_mpm[pb_idx][0] = best_mpm_pb_part[pb_idx][0];
                best_mpm[pb_idx][1] = best_mpm_pb_part[pb_idx][1];
            }

            bst_info->sawp_flag = best_sawp_flag;
            bst_info->skip_idx = best_sawp_idx;
#if AWP_ENH
            bst_info->awp_blend_idx = best_awp_blend_idx;
#endif
            bst_info->sawp_idx0 = best_sawp_cand0;
            bst_info->sawp_idx1 = best_sawp_cand1;
#if DSAWP
            bst_info->dawp_idx = best_dawp_idx;
            bst_info->dawp_ipm0 = best_dawp_cand0;
            bst_info->dawp_ipm1 = best_dawp_cand1;
#endif
            for (int mpm_idx = 0; mpm_idx < SAWP_MPM_NUM; mpm_idx++)
            {
                bst_info->sawp_mpm[mpm_idx] = best_sawp_mpm[mpm_idx];
            }

#if IST
            bst_info->ist_tu_flag = bst_ist_tu_flag;
#endif
#if EST
            bst_info->est_flag = bst_est_tu_flag;
#endif
#if DEST
			bst_info->dest_flag = bst_dest_tu_flag;
#endif
            com_mcpy(bst_info->coef[Y_C], coef_y_pb_part, cu_width * cu_height * sizeof(s16));
            com_mcpy(bst_info->rec[Y_C], rec_y_pb_part, cu_width * cu_height * sizeof(pel));
            assert(mod_info_curr->pb_info.num_sub_part <= mod_info_curr->tb_info.num_sub_part);
            cu_plane_nz_cln(bst_info->num_nz, Y_C);
            int tb_idx = 0; /*only 1 transform block*/
            {
                bst_info->num_nz[tb_idx][Y_C] = num_nz_y_pb_part[tb_idx];
            }
#if TB_SPLIT_EXT
            core->best_pb_part_intra = mod_info_curr->pb_part;
            core->best_tb_part_intra = mod_info_curr->tb_part;
#endif
            SBAC_STORE(core->s_temp_pb_part_best, core->s_temp_prev_comp_best);

        }
        pel* pBestSrc = bst_info->rec[Y_C];
        pel* pModDst = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
        for (int h = 0; h < cu_height; h++)
        {
            com_mcpy(pModDst, pBestSrc, cu_width * sizeof(pel));
            pModDst += s_mod;
            pBestSrc += cu_width;
        }
    }

#if IST_CHROMA
    if (!bst_info->num_nz[0][Y_C])
    {
        mod_info_curr->ist_tu_flag = 0;
        bst_info->ist_tu_flag = 0;
    }
    if (bst_info->ist_tu_flag)
    {
        bst_info->tb_part = SIZE_2Nx2N;
    }
    mod_info_curr->ist_tu_flag = bst_info->ist_tu_flag;
    mod_info_curr->tb_part = bst_info->tb_part;
#endif

    /*  chroma currently is not changed, choose larger part ipd in luma as the luma mode
        later will try to use awp chroma
    */
    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_C)
    {
        //chroma RDO
        SBAC_STORE(core->s_temp_prev_comp_best, core->s_temp_pb_part_best);
        org = pi->addr_org[Y_C] + (y * s_org) + x;
#if TSCPM
        mod = pi->addr_rec_pic[Y_C] + (y * s_mod) + x;
#endif
        mod_cb = pi->addr_rec_pic[U_C] + ((y >> 1) * s_mod_c) + (x >> 1);
        mod_cr = pi->addr_rec_pic[V_C] + ((y >> 1) * s_mod_c) + (x >> 1);
        org_cb = pi->addr_org[U_C] + ((y >> 1) * s_org_c) + (x >> 1);
        org_cr = pi->addr_org[V_C] + ((y >> 1) * s_org_c) + (x >> 1);

        u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, ctx->info.pic_width_in_scu, mod_info_curr->scup, ctx->map.map_scu
#if CCNPM
            , ctx->info.pic_height_in_scu, cu_width, cu_height
#endif        
        );
#if TSCPM
        com_get_nbr(x, y, cu_width, cu_height, mod, s_mod, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, Y_C);
#endif
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cb, s_mod_c, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, U_C);
        com_get_nbr(x >> 1, y >> 1, cu_width >> 1, cu_height >> 1, mod_cr, s_mod_c, avail_cu, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, bit_depth, V_C);

        cost_best = MAX_COST;

#if CHROMA_NOT_SPLIT
        //get luma pred mode
        if (ctx->tree_status == TREE_C)
        {
            assert(cu_width >= 8 && cu_height >= 8);
            int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;
            best_ipd[PB0] = ctx->map.map_ipm[luma_scup];
#if IPCM
            assert((best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT) || best_ipd[PB0] == IPD_IPCM);
#else
            assert(best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT);
#endif
#if USE_IBC
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]) || MCU_GET_IBC(ctx->map.map_scu[luma_scup]));
#else
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]));
#endif
        }
#endif
        ipm_l2c = best_ipd[PB0];
        mod_info_curr->ipm[PB0][0] = (s8)best_ipd[PB0];
        COM_IPRED_CONV_L2C_CHK(ipm_l2c, chk_bypass);
#if ECCPM
        mod_info_curr->eccpm_flag = 0;
#endif
#if TSCPM || PMC
        /* add tscpm, ehance_tscpm, pmc and epmc modes */
        for (i = 0; i < IPD_CHROMA_EXT_CNT; i++) /* UV */
#else
        for (i = 0; i < IPD_CHROMA_CNT; i++) /* UV */
#endif
        {
            s32 dist_t = 0;
            mod_info_curr->ipm[PB0][1] = (s8)i;

            if (i != IPD_DM_C && chk_bypass && i == ipm_l2c
                && ctx->tree_status == TREE_C
                )
            {
                continue;
            }

#if IPCM
            if (i == IPD_DM_C && best_ipd[PB0] == IPD_IPCM)
            {
                continue;
            }
#endif
#if TSCPM
            if (!ctx->info.sqh.tscpm_enable_flag && i == IPD_TSCPM_C)
            {
                continue;
            }
#if ENHANCE_TSPCM
#if ENHANCE_LT_MODE
            if (!ctx->info.sqh.enhance_tscpm_enable_flag && (i == IPD_TSCPM_LT_C || i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C))
#else
            if (!ctx->info.sqh.enhance_tscpm_enable_flag && (i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C))
#endif
            {
                continue;
            }
            if (IS_AVAIL(avail_cu, AVAIL_UP) && !IS_AVAIL(avail_cu, AVAIL_UP_LE) && i == IPD_TSCPM_T_C)
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && IS_AVAIL(avail_cu, AVAIL_UP_LE) && i == IPD_TSCPM_L_C)
            {
                continue;
            }
#else
#if ENHANCE_LT_MODE
            if (i == IPD_TSCPM_LT_C || i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)
#else
            if (i == IPD_TSCPM_T_C || i == IPD_TSCPM_L_C)
#endif
            {
                continue;
            }
#endif
#endif
#if PMC
#if ENHANCE_LT_MODE
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_MCPM_LT_C || i == IPD_MCPM_C || i == IPD_MCPM_T_C || i == IPD_MCPM_L_C))
#else
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_MCPM_C || i == IPD_MCPM_T_C || i == IPD_MCPM_L_C))
#endif
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_LE) && i == IPD_MCPM_T_C)
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && i == IPD_MCPM_L_C)
            {
                continue;
            }
#endif
#if EPMC
#if ENHANCE_LT_MODE
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_EMCPM_C || i == IPD_EMCPM_LT_C || i == IPD_EMCPM_T_C || i == IPD_EMCPM_L_C || i == IPD_EMCPM2_C || i == IPD_EMCPM2_LT_C || i == IPD_EMCPM2_T_C || i == IPD_EMCPM2_L_C))
#else
            if (!ctx->info.sqh.pmc_enable_flag && (i == IPD_EMCPM_C || i == IPD_EMCPM_T_C || i == IPD_EMCPM_L_C || i == IPD_EMCPM2_C || i == IPD_EMCPM2_T_C || i == IPD_EMCPM2_L_C))
#endif
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_UP) && (i == IPD_EMCPM_T_C || i == IPD_EMCPM2_T_C))
            {
                continue;
            }
            if (!IS_AVAIL(avail_cu, AVAIL_LE) && (i == IPD_EMCPM_L_C || i == IPD_EMCPM2_L_C))
            {
                continue;
            }
#endif
#if EPMC
            if (!ctx->info.pic_header.ph_epmc_model_flag && i >= IPD_EMCPM2_C && i <= IPD_EMCPM2_T_C)
            {
                continue;
            }
            if (ctx->info.pic_header.ph_epmc_model_flag && i >= IPD_EMCPM_C && i <= IPD_EMCPM_T_C)
            {
                continue;
            }
#endif

#if CCNPM && !CCNPM_TEMPLATE_OPT
            if (!ctx->info.sqh.ccnpm_enable_flag && i == IPD_CCNPM)
            {
                continue;
            }
            if (i == IPD_CCNPM && x <= 0 && y <= 0)
            {
                continue;
            }
#endif

#if CCNPM_TEMPLATE_OPT
            if (!ctx->info.sqh.ccnpm_enable_flag && (i == IPD_CCNPM || i == IPD_CCNPM_L || i == IPD_CCNPM_T))
            {
                continue;
            }
            if (i == IPD_CCNPM && x <= 0 && y <= 0)
            {
                continue;
            }
            if ((i == IPD_CCNPM_L || i == IPD_CCNPM_T) && (x <= 0 || y <= 0))
            {
                continue;
            }
#endif

            cost_temp = pintra_residue_rdo(ctx, core, org, org_cb, org_cr, s_org, s_org_c, cu_width_log2, cu_height_log2, &dist_t, 1, 0, x, y);
#if PRINT_CU_LEVEL_2
            printf("\nchro pred mode %2d cost_temp %10.1f", i, cost_temp);
            double val = 2815.9;
            if (cost_temp - val < 0.1 && cost_temp - val > -0.1)
            {
                int a = 0;
            }
#endif
            if (cost_temp < cost_best)
            {
                cost_best = cost_temp;
                best_dist_c = dist_t;
                best_ipd_c = i;
#if ECCPM
                bst_info->eccpm_flag = mod_info_curr->eccpm_flag;
#endif                
                for (j = U_C; j < N_C; j++)
                {
                    int size_tmp = (cu_width * cu_height) >> (j == 0 ? 0 : 2);
                    com_mcpy(bst_info->coef[j], mod_info_curr->coef[j], size_tmp * sizeof(s16));
                    com_mcpy(bst_info->rec[j], pi->rec[j], size_tmp * sizeof(pel));
                    bst_info->num_nz[TB0][j] = mod_info_curr->num_nz[TB0][j];
                }
            }
        }
    }


    int pb_part_size_best = core->best_pb_part_intra;
    int tb_part_size_best = get_tb_part_size_by_pb(pb_part_size_best, MODE_INTRA);
    int num_pb_best = get_part_num(pb_part_size_best);
    int num_tb_best = get_part_num(tb_part_size_best);
    //some check
    if (ctx->tree_status == TREE_LC)
    {
        assert(best_ipd_c != IPD_INVALID);
    }
    else if (ctx->tree_status == TREE_L)
    {
        assert(bst_info->num_nz[TBUV0][U_C] == 0);
        assert(bst_info->num_nz[TBUV0][V_C] == 0);
        assert(best_dist_c == 0);
    }
    else if (ctx->tree_status == TREE_C)
    {
        assert(best_ipd_c != IPD_INVALID);
        assert(bst_info->num_nz[TBUV0][Y_C] == 0);
        assert(num_pb_best == 1 && num_tb_best == 1);
        assert(best_dist_y == 0);
    }
    else
        assert(0);
    //end of checks
    cu_nz_cln(mod_info_curr->num_nz);
    /*  for bit cnt  */
    mod_info_curr->sawp_flag = bst_info->sawp_flag;
    mod_info_curr->skip_idx = bst_info->skip_idx;
#if AWP_ENH
    mod_info_curr->awp_blend_idx = bst_info->awp_blend_idx;
#endif
    mod_info_curr->sawp_idx0 = bst_info->sawp_idx0;
    mod_info_curr->sawp_idx1 = bst_info->sawp_idx1;

#if DSAWP
    mod_info_curr->dawp_idx = bst_info->dawp_idx;
    mod_info_curr->dawp_ipm0 = bst_info->dawp_ipm0;
    mod_info_curr->dawp_ipm1 = bst_info->dawp_ipm1;
#endif
    for (int mpm_idx = 0; mpm_idx < SAWP_MPM_NUM; mpm_idx++)
    {
        mod_info_curr->sawp_mpm[mpm_idx] = bst_info->sawp_mpm[mpm_idx];
    }

    pb_part_idx = 0; /*only 1 part*/
    {
        core->mod_info_best.mpm[pb_part_idx][0] = best_mpm[pb_part_idx][0];
        core->mod_info_best.mpm[pb_part_idx][1] = best_mpm[pb_part_idx][1];
        core->mod_info_best.ipm[pb_part_idx][0] = (s8)best_ipd[pb_part_idx];
        core->mod_info_best.ipm[pb_part_idx][1] = (s8)best_ipd_c;
        mod_info_curr->ipm[pb_part_idx][0] = core->mod_info_best.ipm[pb_part_idx][0];
        mod_info_curr->ipm[pb_part_idx][1] = core->mod_info_best.ipm[pb_part_idx][1];
    }
    int tb_part_idx = 0; /*only 1 part*/
    {
        for (j = 0; j < N_C; j++)
        {
            mod_info_curr->num_nz[tb_part_idx][j] = bst_info->num_nz[tb_part_idx][j];
        }
    }
#if IST
    mod_info_curr->ist_tu_flag = bst_info->ist_tu_flag;
#endif
#if EST
    mod_info_curr->est_flag = bst_info->est_flag;
#endif
#if ECCPM
    mod_info_curr->eccpm_flag = bst_info->eccpm_flag;
#endif
#if DEST
	mod_info_curr->dest_flag = bst_info->dest_flag;
#endif
#if TB_SPLIT_EXT
    mod_info_curr->pb_part = core->best_pb_part_intra;
    mod_info_curr->tb_part = core->best_tb_part_intra;
#endif
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_intra(ctx, core, ctx->info.pic_header.slice_type, bst_info->coef);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best = (best_dist_y + best_dist_c) + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    core->dist_cu = best_dist_y + best_dist_c;
    return cost_best;
}
#endif // SAWP


#if USE_SP
static void init_sm_codingunit(ENC_CTX *ctx, ENC_CORE *core, COM_SP_CODING_UNIT* cur_sp_info)
{
    cur_sp_info->ctu_log2size = ctx->info.log2_max_cuwh;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    cur_sp_info->cu_pix_x          = mod_info_curr->x_pos;
    cur_sp_info->cu_pix_y          = mod_info_curr->y_pos;
    cur_sp_info->qp                = core->qp_y;
    cur_sp_info->lamda             = ctx->lambda[0];
    cur_sp_info->cu_width_log2     = mod_info_curr->cu_width_log2;
    cur_sp_info->cu_height_log2    = mod_info_curr->cu_height_log2;
    cur_sp_info->map_scu           = ctx->map.map_scu;
#if SCC_CROSSINFO_ULTILIZE
    cur_sp_info->ibc_mv[0] = core->ibc_mv[0];
    cur_sp_info->ibc_mv[1] = core->ibc_mv[1];
#endif
    cur_sp_info->pic_width_in_scu  = ctx->info.pic_width_in_scu;
    cur_sp_info->pic_height_in_scu = ctx->info.pic_height_in_scu;
    cur_sp_info->scup              = mod_info_curr->scup;
    cur_sp_info->tree_status       = ctx->tree_status;
    cur_sp_info->string_prediction_mode_flag = FALSE;
    cur_sp_info->max_str_cnt       = 1 << (cur_sp_info->cu_width_log2 + cur_sp_info->cu_height_log2 - 2);
    cur_sp_info->cur_bst_rdcost    = core->mod_info_best.cur_bst_rdcost;
    ENC_PARENT_INFO *parent_info = &core->cu_parent_info;
    cur_sp_info->p_x               = parent_info->p_x;
    cur_sp_info->p_y               = parent_info->p_y;
    cur_sp_info->p_width           = 1 << parent_info->p_width_log2;
    cur_sp_info->p_height          = 1 << parent_info->p_height_log2;
    cur_sp_info->is_sp_pix_completed  = FALSE;
#if ISC_RSD
    memset(cur_sp_info->p_string_copy_info, 0, SP_STRING_INFO_NO * sizeof(COM_SP_INFO));
    memset(cur_sp_info->p_evs_copy_info, 0, SP_STRING_INFO_NO * sizeof(COM_SP_EVS_INFO));
    memset(cur_sp_info->unpredict_pix_info, 0, SP_STRING_INFO_NO * sizeof(COM_SP_PIX));
#endif
    cur_sp_info->chroma_weight[0]  = ctx->dist_chroma_weight[0];
    cur_sp_info->chroma_weight[1]  = ctx->dist_chroma_weight[1];
    cur_sp_info->p_cand            = core->parent_offset;
    cur_sp_info->p_cand_num        = core->p_offset_num;
    cur_sp_info->b_cand            = core->brother_offset;
    cur_sp_info->b_cand_num        = core->b_offset_num;
    cur_sp_info->n_cand            = core->n_recent_offset;
    cur_sp_info->n_cand_num        = core->n_offset_num;
    cur_sp_info->is_sp_skip_non_scc    = core->sp_skip_non_scc;
    cur_sp_info->m_cs2_mode_flag = FALSE;
    cur_sp_info->m_bit_depth = ctx->info.bit_depth_internal;
    cur_sp_info->m_srb_sharing_flag = FALSE;
    //Load prev srb
    cur_sp_info->m_pvbuf_size_prev = core->n_pv_num;
    cur_sp_info->cu_ext = 0;
    if (core->n_pv_num > 0)
    {
        int ch;
        assert(core->n_pv_num <= MAX_SRB_PRED_SIZE);
        for (ch = 0; ch < 3; ch++)
        {
            memcpy(cur_sp_info->m_srb_prev[ch], core->n_recent_pv[ch], core->n_pv_num * sizeof(pel));
        }
        memcpy(cur_sp_info->m_dpb_reYonly_prev, core->n_recent_dpb_reYonly, core->n_pv_num * sizeof(u8));
        memcpy(cur_sp_info->m_dpb_idx_prev, core->n_recent_dpb_idx, core->n_pv_num * sizeof(u8));
        memcpy(cur_sp_info->m_all_comp_pre_flag, core->n_recent_all_comp_flag, core->n_pv_num * sizeof(u8));
        memcpy(cur_sp_info->m_cuS_pre_flag, core->n_recent_cuS_flag, core->n_pv_num * sizeof(u8));
        memcpy(cur_sp_info->m_pv_prev_x, core->n_recent_pv_x, core->n_pv_num * sizeof(s16));
        memcpy(cur_sp_info->m_pv_prev_y, core->n_recent_pv_y, core->n_pv_num * sizeof(s16));
    }
    SBAC_LOAD(cur_sp_info->s_curr_best[cur_sp_info->cu_width_log2 - 2][cur_sp_info->cu_height_log2 - 2], core->s_curr_best[cur_sp_info->cu_width_log2 - 2][cur_sp_info->cu_height_log2 - 2]);
    cur_sp_info->bs_temp.pdata[1] = &cur_sp_info->s_temp_run;
}
static u8 ver_skip(COM_MODE *bst_info, double distortion)
{
    int w = bst_info->cu_width;
    int h = bst_info->cu_height;

    if (bst_info->cs2_flag)
    {
        if (bst_info->evs_sub_string_no == 1 && bst_info->evs_str_copy_info[0].length == w * h)
        {
            return 1;
        }
    }
    return 0;
}

static u8 is_dir_skip(COM_MODE *bst_info, double distortion,u8 skipNonSCC)
{
    int w = bst_info->cu_width;
    int h = bst_info->cu_height;
    if (bst_info->sp_flag)
    {
        if (bst_info->sub_string_no == 1 && bst_info->string_copy_info[0].length == w * h)
        {
            return 1;
        }
    }
    if (skipNonSCC)
    {
        return 1;
    }
    return 0;
}

static void copy_to_bst_mode(COM_MODE *bst_info, COM_SP_CODING_UNIT* cur_sp_info)
{
    int width = 1 << cur_sp_info->cu_width_log2;
    int height = 1 << cur_sp_info->cu_height_log2;
    int size;
#if IST
    bst_info->ist_tu_flag = 0;
#endif
#if INTERPF
    bst_info->inter_filter_flag = 0;
#endif
#if IBC_ENH
    bst_info->ibcpf_idx = 0;
#endif
#if IPC
    bst_info->ipc_flag = 0;
#endif
    bst_info->skip_idx = 0;
    bst_info->ibc_flag = 0;
#if CIBC
    bst_info->cibc_flag = 0;
#endif
#if SBT
    bst_info->sbt_info = 0;
#endif
    bst_info->umve_flag = 0;
    bst_info->mvr_idx = 0;
#if IBC_ABVR
    bst_info->bvr_idx = 0;
#endif
#if SAWP
    bst_info->sawp_flag = 0;
#endif // SAWP

#if EXT_AMVR_HMVP
    bst_info->mvp_from_hmvp_flag = 0;
#endif
    for (int lidx = 0; lidx < REFP_NUM; lidx++)
    {
        bst_info->mv[lidx][MV_X] = 0;
        bst_info->mv[lidx][MV_Y] = 0;
        bst_info->mvd[lidx][MV_X] = 0;
        bst_info->mvd[lidx][MV_Y] = 0;
    }
#if SMVD
    bst_info->smvd_flag = 0;
#endif
    bst_info->affine_flag = 0;
    if (cur_sp_info->string_prediction_mode_flag) 
    {
        bst_info->sub_string_no = (u16)cur_sp_info->sub_string_no;
        bst_info->sp_copy_direction = cur_sp_info->string_copy_direction;
        COM_SP_INFO *p_str_info = cur_sp_info->p_string_copy_info;
        memcpy(bst_info->string_copy_info, p_str_info, bst_info->sub_string_no * sizeof(COM_SP_INFO));
        bst_info->sp_flag = TRUE;
        bst_info->cs2_flag = FALSE;
        cu_nz_cln(bst_info->num_nz);
        SET_REFI(bst_info->refi, REFI_INVALID, REFI_INVALID);
    }
    if (cur_sp_info->m_cs2_mode_flag)
    {
        bst_info->sp_flag = TRUE;
        bst_info->cs2_flag = TRUE;
        bst_info->evs_copy_direction = cur_sp_info->string_copy_direction;
        bst_info->evs_sub_string_no = cur_sp_info->sub_string_no;
        for (int i = 0; i < bst_info->evs_sub_string_no; i++)
        {
            bst_info->evs_str_copy_info[i] = cur_sp_info->p_evs_copy_info[i];
        }
        bst_info->unpred_pix_num = cur_sp_info->unpredict_pix_num;
        memcpy(bst_info->unpred_pix_info, cur_sp_info->unpredict_pix_info, bst_info->unpred_pix_num * sizeof(COM_SP_PIX));
        bst_info->equal_val_str_present_flag = cur_sp_info->m_evs_present_flag;
        bst_info->unpredictable_pix_present_flag = cur_sp_info->m_unpredictable_pixel_present_flag;
        bst_info->pvbuf_size = cur_sp_info->m_pvbuf_size;
        bst_info->pvbuf_size_prev = cur_sp_info->m_pvbuf_size_prev;
        memcpy(bst_info->pvbuf_reused_flag, cur_sp_info->m_pvbuf_reused_flag, bst_info->pvbuf_size_prev * sizeof(u8));
        memcpy(bst_info->m_dpb_reYonly, cur_sp_info->m_dpb_reYonly, bst_info->pvbuf_size * sizeof(u8));
        memcpy(bst_info->m_dpb_reYonly_prev, cur_sp_info->m_dpb_reYonly_prev, bst_info->pvbuf_size_prev * sizeof(u8));
        memcpy(bst_info->m_dpb_idx, cur_sp_info->m_dpb_idx, bst_info->pvbuf_size * sizeof(u8));
        memcpy(bst_info->m_dpb_idx_prev, cur_sp_info->m_dpb_idx_prev, bst_info->pvbuf_size_prev * sizeof(u8));
        memcpy(bst_info->all_comp_flag, cur_sp_info->m_all_comp_flag, bst_info->pvbuf_size * sizeof(u8));
        memcpy(bst_info->all_comp_pre_flag, cur_sp_info->m_all_comp_pre_flag, bst_info->pvbuf_size_prev * sizeof(u8));
        memcpy(bst_info->cuS_flag, cur_sp_info->m_cuS_flag, bst_info->pvbuf_size * sizeof(u8));
        memcpy(bst_info->cuS_pre_flag, cur_sp_info->m_cuS_pre_flag, bst_info->pvbuf_size_prev * sizeof(u8));
        memcpy(bst_info->pv_x, cur_sp_info->m_pv_x, bst_info->pvbuf_size * sizeof(s16));
        memcpy(bst_info->pv_x_prev, cur_sp_info->m_pv_prev_x, bst_info->pvbuf_size_prev * sizeof(s16));
        memcpy(bst_info->pv_y, cur_sp_info->m_pv_y, bst_info->pvbuf_size * sizeof(s16));
        memcpy(bst_info->pv_y_prev, cur_sp_info->m_pv_prev_y, bst_info->pvbuf_size_prev * sizeof(s16));
        for (int i = 0; i < 3; i++)
        {
            memcpy(bst_info->p_SRB[i], cur_sp_info->m_srb[i], bst_info->pvbuf_size * sizeof(pel));
            memcpy(bst_info->p_SRB_prev[i], cur_sp_info->m_srb_prev[i], bst_info->pvbuf_size_prev * sizeof(pel));
        }
        cu_nz_cln(bst_info->num_nz);
        SET_REFI(bst_info->refi, REFI_INVALID, REFI_INVALID);
    }
    bst_info->is_sp_pix_completed = cur_sp_info->is_sp_pix_completed;
    size = width * height * sizeof(pel);
    memcpy(bst_info->rec[Y_C], cur_sp_info->rec[Y_C], size);
    size = (width * height * sizeof(pel)) >> 2;
    memcpy(bst_info->rec[U_C], cur_sp_info->rec[U_C], size);
    memcpy(bst_info->rec[V_C], cur_sp_info->rec[V_C], size);
    com_mset(bst_info->coef[Y_C], 0, sizeof(s16) * width * height);
    com_mset(bst_info->coef[U_C], 0, sizeof(s16) * width * height >> 2);
    com_mset(bst_info->coef[V_C], 0, sizeof(s16) * width * height >> 2);
}

double analyze_sm_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PARENT_INFO *parent_info = &core->cu_parent_info;
    if (parent_info->c_sumRDCost >= parent_info->p_RDCost)
    {
        return MAX_COST;
    }
    int bit_cnt = 0;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    COM_MODE *bst_info = &core->mod_info_best;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if IST
    int bst_ist_tu_flag = 0;
#endif
#if SBT
    mod_info_curr->sbt_info = 0;
#endif
#if IST
    mod_info_curr->ist_tu_flag = 0;
#endif
#if INTERPF
    mod_info_curr->inter_filter_flag = 0;
#endif
#if IBC_ENH
    mod_info_curr->ibcpf_idx = 0;
#endif
#if IPC
    mod_info_curr->ipc_flag = 0;
#endif
#if SAWP
    mod_info_curr->sawp_flag = 0;
#endif // SAWP

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, 1 << cu_width_log2, 1 << cu_height_log2, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, 1 << cu_width_log2, 1 << cu_height_log2, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, core->x_scu << 2, core->y_scu << 2, 1 << cu_width_log2, 1 << cu_height_log2, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif
    mod_info_curr->cu_mode = MODE_IBC;
    int scu_x = 1 << (cu_width_log2 - 2);
    int scu_y = 1 << (cu_height_log2 - 2);
    int pic_width_in_scu = ctx->info.pic_width_in_scu;
    u32 *scu = ctx->map.map_scu + mod_info_curr->scup;
    for (int j = 0; j < scu_y; j++) 
    {
        for (int i = 0; i < scu_x; i++) 
        {
            MCU_CLR_CODED_FLAG(scu[i]);
        }
        scu += pic_width_in_scu;
    }
    /*send the parameters of the sm mode*/
    COM_SP_CODING_UNIT cu;
    COM_SP_CODING_UNIT* cur_sp_info = &cu;
    init_sm_codingunit(ctx, core, cur_sp_info);
    double min_sp_rdcost = core->cost_best;
    double cur_sp_rdcost = MAX_COST;
    double cur_sp_dist;
    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
    {
        cur_sp_rdcost = cur_sp_info->cur_bst_rdcost;
    }
    cur_sp_info->string_prediction_mode_flag = TRUE;
    if (0 == IS_VALID_SP_CU_SIZE((1 << cu_width_log2), (1 << cu_height_log2)) || !ctx->param.sp_enable_flag)
    {
        cur_sp_info->string_prediction_mode_flag = FALSE;
    }
    // doing SP search
    if (cur_sp_info->string_prediction_mode_flag == TRUE) 
    {
        u8 skip_non_scc = FALSE;
        u8 curr_skip_non_scc = cur_sp_info->is_sp_skip_non_scc;
        cur_sp_info->m_cs2_mode_flag = FALSE;
        //first direction
        cur_sp_info->string_copy_direction = 1;
#if FAST_SCC_PRECODING
        cur_sp_info->spvisit = core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit;
        if (ctx->slice_type != SLICE_I || !core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit)
        {
            cur_sp_info->sp_transmit_x = 0;
            cur_sp_info->sp_transmit_y = 0;
            cur_sp_info->spvisit = 0;
        }
        else
        {
            cur_sp_info->sp_transmit_x = core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].SPvisit0mv[MV_X];
            cur_sp_info->sp_transmit_y = core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].SPvisit0mv[MV_Y];
        }
#endif
        if (sm_mode_rdcost(ctx->sp_encoder, cur_sp_info, &cur_sp_rdcost, &cur_sp_dist
            , ctx
        ))
        {
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_sp(ctx, core, cur_sp_info->p_string_copy_info, cur_sp_info->string_copy_direction, 1);
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cur_sp_rdcost = (double)cur_sp_dist + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
            if (cur_sp_rdcost < min_sp_rdcost) 
            {
                min_sp_rdcost = cur_sp_rdcost;
                bst_info->slice_type = ctx->slice_type;
                copy_to_bst_mode(bst_info, cur_sp_info); 
                SBAC_STORE(core->s_temp_best, core->s_temp_run);

            }
        }
#if FAST_SCC_PRECODING
        if (ctx->slice_type == SLICE_I && !core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].visit)
        {
            core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].SPvisit0mv[MV_X] = cur_sp_info->sp_transmit_x;
            core->bef_data[cu_width_log2 - 2][cu_height_log2 - 2][core->cup].SPvisit0mv[MV_Y] = cur_sp_info->sp_transmit_y;
        }
#endif
        skip_non_scc = skip_non_scc || cur_sp_info->is_sp_skip_non_scc;
        cur_sp_info->is_sp_skip_non_scc = curr_skip_non_scc;
        cur_sp_info->string_prediction_mode_flag = FALSE;
    }
    assert(cur_sp_info->string_prediction_mode_flag == FALSE);
    u8  is_hor_cs2 = FALSE;
    cur_sp_info->m_cs2_mode_flag = TRUE;
    // limit cu size for CS2
    if (0 == IS_VALID_CS2_CU_SIZE((1 << cu_width_log2), (1 << cu_height_log2)) || cur_sp_info->is_sp_skip_non_scc || !ctx->param.evs_ubvs_enable_flag)
    {
        cur_sp_info->m_cs2_mode_flag = FALSE;
    }
    // doing CS2 search
    if (cur_sp_info->m_cs2_mode_flag)
    {
        if (ctx->tree_status == TREE_L)
        {
            cur_sp_info->cu_ext = 1;
        }
        //=========horizontal direction=================
        cur_sp_info->string_copy_direction = TRUE;
        cur_sp_info->m_srb_sharing_flag = FALSE;
        is_hor_cs2 = sm_mode_rdcost(ctx->sp_encoder, cur_sp_info, &cur_sp_rdcost, &cur_sp_dist
            , ctx
        );
        if (is_hor_cs2)
        {
            SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
            bit_cnt = enc_get_bit_number(&core->s_temp_run);
            enc_bit_est_cs2(cu_width_log2, cu_height_log2, &core->bs_temp, cur_sp_info
                , ctx
                , x, y
            );
            bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
            cur_sp_rdcost = (double)cur_sp_dist + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
            if (cur_sp_rdcost < min_sp_rdcost)
            {
                min_sp_rdcost = cur_sp_rdcost;
                bst_info->slice_type = ctx->slice_type;
                copy_to_bst_mode(bst_info, cur_sp_info);
                SBAC_STORE(core->s_temp_best, core->s_temp_run);
            }
        }

        if (cur_sp_info->m_pvbuf_size_prev > 0)
        {
            //=========derive_srblossy_force_prediction horizontal direction =================
            if (!ver_skip(bst_info, cur_sp_dist))
            {
                cur_sp_info->string_copy_direction = TRUE;//Horizontal
                cur_sp_info->m_cs2_mode_flag = TRUE;
                cur_sp_info->m_srb_sharing_flag = TRUE;
                is_hor_cs2 = FALSE;
                is_hor_cs2 = sm_mode_rdcost(ctx->sp_encoder, cur_sp_info, &cur_sp_rdcost, &cur_sp_dist
                    , ctx
                );
                if (is_hor_cs2) 
                {
                    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
                    bit_cnt = enc_get_bit_number(&core->s_temp_run);
                    enc_bit_est_cs2(cu_width_log2, cu_height_log2, &core->bs_temp, cur_sp_info
                        , ctx
                        , x, y
                    );
                    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
                    cur_sp_rdcost = (double)cur_sp_dist + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
                    if (cur_sp_rdcost < min_sp_rdcost)
                    {
                        min_sp_rdcost = cur_sp_rdcost;
                        bst_info->slice_type = ctx->slice_type;
                        copy_to_bst_mode(bst_info, cur_sp_info);
                        SBAC_STORE(core->s_temp_best, core->s_temp_run);
                    }
                }
            }
        }
    }
    return min_sp_rdcost;
}
#endif

#if IPCM
double analyze_ipcm_cu(ENC_CTX *ctx, ENC_CORE *core)
{
    ENC_PINTRA *pi = &ctx->pintra;
    COM_MODE *bst_info = &core->mod_info_best;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int i, j, s_org, s_org_c;

    int best_ipd[MAX_NUM_TB] = { IPD_INVALID, IPD_INVALID, IPD_INVALID, IPD_INVALID };
    int best_ipd_c = IPD_INVALID;
    s32 best_dist_y = 0, best_dist_c = 0;
    int bit_cnt = 0;
    pel *org;
    pel *org_cb, *org_cr;
    double cost_best = MAX_COST;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
#if SBT
    mod_info_curr->sbt_info = 0;
#endif

#if TB_SPLIT_EXT
    init_pb_part(&core->mod_info_curr);
    init_tb_part(&core->mod_info_curr);
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.pb_part, &core->mod_info_curr.pb_info);
#if DT_TRANSFORM
    get_tb_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#else
    get_part_info(ctx->info.pic_width_in_scu, mod_info_curr->x_scu << 2, mod_info_curr->y_scu << 2, cu_width, cu_height, core->mod_info_curr.tb_part, &core->mod_info_curr.tb_info);
#endif
#endif

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_L)
    {
        s_org = pi->stride_org[Y_C];
        org = pi->addr_org[Y_C] + (y * s_org) + x;
        int pb_part_idx = 0;
        core->best_pb_part_intra = SIZE_2Nx2N;

        for (i = 0; i < cu_height; i++)
        {
            for (j = 0; j < cu_width; j++)
            {
                bst_info->rec[Y_C][i * cu_width + j] = (org[i * s_org + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input)) << (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                bst_info->coef[Y_C][i * cu_width + j] = org[i * s_org + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
            }
        }

        int pb_x = mod_info_curr->pb_info.sub_x[pb_part_idx];
        int pb_y = mod_info_curr->pb_info.sub_y[pb_part_idx];
        int pb_w = mod_info_curr->pb_info.sub_w[pb_part_idx];
        int pb_h = mod_info_curr->pb_info.sub_h[pb_part_idx];
        int pb_scup = mod_info_curr->pb_info.sub_scup[pb_part_idx];
        int pb_x_scu = PEL2SCU(pb_x);
        int pb_y_scu = PEL2SCU(pb_y);

#if FIMC
        if (ctx->info.sqh.fimc_enable_flag && ctx->info.pic_header.fimc_pic_flag)
        {
            com_get_cntmpm(pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx], &core->cntmpm_cands);
        }
        else
        {
#endif
            com_get_mpm(pb_x_scu, pb_y_scu, ctx->map.map_scu, ctx->map.map_ipm, pb_scup, ctx->info.pic_width_in_scu, mod_info_curr->mpm[pb_part_idx]);
#if FIMC
        }
#endif
        update_intra_info_map_scu(ctx->map.map_scu, ctx->map.map_ipm, pb_x, pb_y, pb_w, pb_h, ctx->info.pic_width_in_scu, IPD_IPCM);

        best_ipd[PB0] = IPD_IPCM;
        cu_plane_nz_cln(mod_info_curr->num_nz, Y_C);
        cu_plane_nz_cln(bst_info->num_nz, Y_C);
        copy_rec_y_to_pic(bst_info->rec[Y_C] + (pb_y - y) * cu_width + (pb_x - x), pb_x, pb_y, pb_w, pb_h, cu_width, PIC_REC(ctx));
        com_mcpy(pi->rec[Y_C], bst_info->rec[Y_C], cu_width * cu_height * sizeof(pel));
#if RDO_DBK
        calc_delta_dist_filter_boundary(ctx, core, PIC_REC(ctx), PIC_ORG(ctx), cu_width, cu_height, pi->rec, cu_width, x, y, 1, 0, NULL, NULL, 0);
        best_dist_y += (s32)ctx->delta_dist; //add delta SSD to the first PB
#endif
    }

    if (ctx->tree_status == TREE_LC || ctx->tree_status == TREE_C)
    {
        s_org_c = pi->stride_org[U_C];
        org_cb = pi->addr_org[U_C] + ((y >> 1) * s_org_c) + (x >> 1);
        org_cr = pi->addr_org[V_C] + ((y >> 1) * s_org_c) + (x >> 1);
        core->best_pb_part_intra = SIZE_2Nx2N;

#if CHROMA_NOT_SPLIT
        //get luma pred mode
        if (ctx->tree_status == TREE_C)
        {
            assert(cu_width >= 8 && cu_height >= 8);
            int luma_scup = PEL2SCU(x + (cu_width - 1)) + PEL2SCU(y + (cu_height - 1)) * ctx->info.pic_width_in_scu;
            best_ipd[PB0] = ctx->map.map_ipm[luma_scup];
            assert((best_ipd[PB0] >= 0 && best_ipd[PB0] < IPD_CNT) || best_ipd[PB0] == IPD_IPCM);
#if USE_IBC
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]) || MCU_GET_IBC(ctx->map.map_scu[luma_scup]));
#else
            assert(MCU_GET_INTRA_FLAG(ctx->map.map_scu[luma_scup]));
#endif
        }
#endif
        if (best_ipd[PB0] == IPD_IPCM)
        {
            best_ipd_c = IPD_DM_C;

            for (i = 0; i < cu_height / 2; i++)
            {
                for (j = 0; j < cu_width / 2; j++)
                {
                    bst_info->rec[U_C][i * cu_width / 2 + j] = (org_cb[i * s_org_c + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input)) << (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                    bst_info->coef[U_C][i * cu_width / 2 + j] = org_cb[i * s_org_c + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                    bst_info->rec[V_C][i * cu_width / 2 + j] = (org_cr[i * s_org_c + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input)) << (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                    bst_info->coef[V_C][i * cu_width / 2 + j] = org_cr[i * s_org_c + j] >> (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                }
            }
        }
        else
        {
            return MAX_COST;
        }

        cu_plane_nz_cln(mod_info_curr->num_nz, U_C);
        cu_plane_nz_cln(bst_info->num_nz, U_C);
        cu_plane_nz_cln(mod_info_curr->num_nz, V_C);
        cu_plane_nz_cln(bst_info->num_nz, V_C);
    }

    int pb_part_idx = 0;

    core->mod_info_best.mpm[pb_part_idx][0] = mod_info_curr->mpm[pb_part_idx][0];
    core->mod_info_best.mpm[pb_part_idx][1] = mod_info_curr->mpm[pb_part_idx][1];
    core->mod_info_best.ipm[pb_part_idx][0] = best_ipd[PB0];
    core->mod_info_best.ipm[pb_part_idx][1] = best_ipd_c;
    mod_info_curr->ipm[pb_part_idx][0] = core->mod_info_best.ipm[pb_part_idx][0];
    mod_info_curr->ipm[pb_part_idx][1] = core->mod_info_best.ipm[pb_part_idx][1];

    SBAC_LOAD(core->s_temp_run, core->s_curr_best[cu_width_log2 - 2][cu_height_log2 - 2]);
    bit_cnt = enc_get_bit_number(&core->s_temp_run);
    enc_bit_est_intra(ctx, core, ctx->info.pic_header.slice_type, bst_info->coef);
    bit_cnt = enc_get_bit_number(&core->s_temp_run) - bit_cnt;
    cost_best = (best_dist_y + best_dist_c) + RATE_TO_COST_LAMBDA(ctx->lambda[0], bit_cnt);
    SBAC_STORE(core->s_temp_best, core->s_temp_run);
    core->dist_cu = best_dist_y + best_dist_c;

    return cost_best;
}
#endif

int pintra_set_complexity(ENC_CTX * ctx, int complexity)
{
    ENC_PINTRA * pi;
    pi = &ctx->pintra;
    pi->complexity = complexity;
    return COM_OK;
}

