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

#include "com_def.h"
#include "com_recon.h"
#if USE_SP
#include "com_usp.h"
#endif


void com_recon(PART_SIZE part, s16 *resi, pel *pred, int (*is_coef)[N_C], int plane, int cu_width, int cu_height, int s_rec, pel *rec, int bit_depth
#if SBT
    , u8 sbt_info
#endif
)
{
    int i, j;
    s16 t0;
    int k, part_num = get_part_num(part);
    int tb_height, tb_width;

    get_tb_width_height(cu_width, cu_height, part, &tb_width, &tb_height);

    for (k = 0; k < part_num; k++)
    {
        int tb_x, tb_y;
        pel *p, *r;
        s16 *c;

        get_tb_start_pos(cu_width, cu_height, part, k, &tb_x, &tb_y);

        p = pred + tb_y * cu_width + tb_x;
        r = rec  + tb_y * s_rec + tb_x;

        if (is_coef[k][plane] == 0) /* just copy pred to rec */
        {
            for (i = 0; i < tb_height; i++)
            {
                for (j = 0; j < tb_width; j++)
                {
                    r[i * s_rec + j] = COM_CLIP3(0, (1 << bit_depth) - 1, p[i * cu_width + j]);
                }
            }
        }
        else  /* add b/w pred and coef and copy it into rec */
        {
#if SBT
            if( sbt_info != 0 && plane == Y_C )
            {
                u8  sbt_idx = get_sbt_idx( sbt_info );
                u8  sbt_pos = get_sbt_pos( sbt_info );
                int tu0_w, tu0_h;
                int tu1_w, tu1_h;
#if SBT_EXT
                assert( sbt_idx >= 1 && sbt_idx <= 12 );
#else
                assert( sbt_idx >= 1 && sbt_idx <= 4 );
#endif
                assert( p == pred && part_num == 1 );
                assert( cu_width == tb_width && cu_height == tb_height );
#if SBT_EXT
                if( is_sbt_vertical( sbt_idx ) )
#else
                if( !is_sbt_horizontal( sbt_idx ) )
#endif
                {
                    tu0_w = is_sbt_quad_size( sbt_idx ) ? (cu_width / 4) : (cu_width / 2);
                    tu0_w = sbt_pos == 0 ? tu0_w : cu_width - tu0_w;
                    tu1_w = cu_width - tu0_w;
                    for( i = 0; i < cu_height; i++ )
                    {
                        for( j = 0; j < tu0_w; j++ )
                        {
                            t0 = (sbt_pos == 0 ? resi[i * tu0_w + j] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                        for( j = tu0_w; j < cu_width; j++ )
                        {
                            t0 = (sbt_pos == 1 ? resi[i * tu1_w + j - tu0_w] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                    }
                }
#if SBT_EXT
                else if( is_sbt_horizontal( sbt_idx ) )
#else
                else
#endif
                {
                    tu0_h = is_sbt_quad_size( sbt_idx ) ? (cu_height / 4) : (cu_height / 2);
                    tu0_h = sbt_pos == 0 ? tu0_h : cu_height - tu0_h;
                    tu1_h = cu_height - tu0_h;
                    for( j = 0; j < cu_width; j++ )
                    {
                        for( i = 0; i < tu0_h; i++ )
                        {
                            t0 = (sbt_pos == 0 ? resi[i * cu_width + j] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                        for( i = tu0_h; i < cu_height; i++ )
                        {
                            t0 = (sbt_pos == 1 ? resi[(i - tu0_h) * cu_width + j] : 0) + pred[i * cu_width + j];
                            rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                    }
                }
#if SBT_EXT
                else
                {
                    int tu_w = is_sbt_ext1( sbt_idx ) ? (cu_width / 2) : (cu_width / 4);
                    int tu_h = is_sbt_ext1( sbt_idx ) ? (cu_height / 2) : (cu_height / 4);
                    int x_start = is_sbt_ext_tl( sbt_idx ) || is_sbt_ext_bl( sbt_idx ) ? 0 : cu_width - tu_w;
                    int y_start = is_sbt_ext_tl( sbt_idx ) || is_sbt_ext_tr( sbt_idx ) ? 0 : cu_height - tu_h;
                    int x_end = x_start + tu_w;
                    int y_end = y_start + tu_h;
                    for (i = 0; i < cu_height; i++)
                    {
                        for (j = 0; j < cu_width; j++)
                        {
                          t0 = pred[i * cu_width + j];
                          if (i >= y_start && i < y_end && j >= x_start && j < x_end)
                          {
                            t0 += resi[(i - y_start) * tu_w + (j - x_start)];
                          }
                          rec[i * s_rec + j] = COM_CLIP3( 0, (1 << bit_depth) - 1, t0 );
                        }
                    }
                }
#endif
                return;
            }
#endif
            c = resi + k * tb_width * tb_height;
            for (i = 0; i < tb_height; i++)
            {
                for (j = 0; j < tb_width; j++)
                {
                    t0 = c[i * tb_width + j] + p[i * cu_width + j];
                    r[i * s_rec + j] = COM_CLIP3(0, (1 << bit_depth) - 1, t0);
                }
            }
        }
    }
}

void com_recon_yuv(PART_SIZE part_size, int x, int y, int cu_width, int cu_height, s16 resi[N_C][MAX_CU_DIM], pel pred[N_C][MAX_CU_DIM], int (*num_nz_coef)[N_C], COM_PIC *pic, CHANNEL_TYPE channel, int bit_depth 
#if SBT
    , u8 sbt_info
#endif
#if INTER_CCNPM
    , COM_MODE* mod_info_curr, int pic_width_in_scu, int pic_height_in_scu, u32* map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
    , const int max_cu_height
#endif
)
{
    pel * rec;
    int s_rec, off;

    /* Y */
    if (channel == CHANNEL_LC || channel == CHANNEL_L)
    {
        s_rec = pic->stride_luma;
        rec = pic->y + (y * s_rec) + x;
        com_recon(part_size, resi[Y_C], pred[Y_C], num_nz_coef, Y_C, cu_width, cu_height, s_rec, rec, bit_depth
#if SBT
            , sbt_info
#endif
        );
    }

#if INTER_CCNPM
    if (mod_info_curr->inter_ccnpm_flag)
    {
        u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, pic_width_in_scu, mod_info_curr->scup, map_scu, pic_height_in_scu, cu_width, cu_height);
#if INTER_CCNPM_OPT
        s16 org_pred[MAX_CU_DIM];
        int cu_size = 1 << (mod_info_curr->cu_width_log2 + mod_info_curr->cu_height_log2);
#endif
        for (int comp_id = U_C; comp_id < N_C; comp_id++)
        {
#if INTER_CCNPM_OPT
            memcpy(org_pred, pred[comp_id], sizeof(s16) * (cu_size >> 2));
#endif
            ipred_ccnpm(comp_id, pic->y, pic->stride_luma, pic->u, pic->v, pic->stride_chroma, pred[comp_id], x, y, cu_width / 2, cu_height / 2, bit_depth, avail_cu
#if CCNPM_TEMPLATE_OPT
                , IPD_CCNPM
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                , max_cu_height
#endif
            );
#if INTER_CCNPM_OPT
            ccnpm_blending(org_pred, pred[comp_id], cu_width / 2, cu_height / 2, bit_depth);
#endif
        }
    }
#endif

    /* chroma */
    if (channel == CHANNEL_LC || channel == CHANNEL_C)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        off = (x >> 1) + (y >> 1) * pic->stride_chroma;
        com_recon(SIZE_2Nx2N, resi[U_C], pred[U_C], num_nz_coef, U_C, cu_width, cu_height, pic->stride_chroma, pic->u + off, bit_depth
#if SBT
            , sbt_info
#endif
        );
        com_recon(SIZE_2Nx2N, resi[V_C], pred[V_C], num_nz_coef, V_C, cu_width, cu_height, pic->stride_chroma, pic->v + off, bit_depth
#if SBT
            , sbt_info

#endif
        );
    }

#if PMC || EPMC
    /* U only or V only */
    if (channel == CHANNEL_U)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        off = (x >> 1) + (y >> 1) * pic->stride_chroma;
        com_recon(SIZE_2Nx2N, resi[U_C], pred[U_C], num_nz_coef, U_C, cu_width, cu_height, pic->stride_chroma, pic->u + off, bit_depth
#if SBT
            , sbt_info
#endif
        );
    }

    if (channel == CHANNEL_V)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        off = (x >> 1) + (y >> 1) * pic->stride_chroma;
        com_recon(SIZE_2Nx2N, resi[V_C], pred[V_C], num_nz_coef, V_C, cu_width, cu_height, pic->stride_chroma, pic->v + off, bit_depth
#if SBT
            , sbt_info
#endif
        );
    }
#endif
}

#if USE_SP
#if !DEC_RANGE_EXTEND
__inline u8 is_ref_pix_in_2ctu(int ref_X_LT, int ref_Y_LT, int x, int y
    , int ctulog2size
)
{
    int ctu_x = x >> ctulog2size << ctulog2size;
    int ctu_y = y >> ctulog2size << ctulog2size;
    int ctusize = 1 << ctulog2size;
    int new_x_left = ref_X_LT;
    int new_y_top = ref_Y_LT;
    int numLeftCTUs = (1 << ((7 - ctulog2size) << 1)) - ((ctulog2size < 7) ? 1 : 0);
    int max_width = ctusize * numLeftCTUs;

    if (ctusize == 128)
    {
        assert(max_width == 128);
    }
    else if (ctusize == 64)
    {
        assert(max_width == 192);
    }
    else if (ctusize == 32)
    {
        assert(max_width == 480);
    }

    if (new_x_left < ctu_x - max_width)
    {
        return 0;
    }
    if (new_x_left > ctu_x + ctusize - 1)
    {
        return 0;
    }
    if (new_y_top < ctu_y)
    {
        return 0;
    }
    if (new_y_top > ctu_y + ctusize - 1)
    {
        return 0;
    }
    return 1;
}
__inline u8 is_ref_pix_coded(u32* map_scu, int cup, int ref_X_LT, int ref_Y_LT, int pic_width_in_scu)
{
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
__inline u8 is_ref_pix_valid(int* puiReverseOrder, COM_MODE *mod_info_curr, int ref_X_LT, int ref_Y_LT, int processed_count, int pic_width_in_scu, u32 *map)
{
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    if (is_ref_pix_coded(map, mod_info_curr->scup, ref_X_LT, ref_Y_LT, pic_width_in_scu))  // in the coded cu
    {
        return 1;
    }
    else if ((ref_X_LT >= mod_info_curr->x_pos &&ref_X_LT < mod_info_curr->x_pos + (1 << cu_width_log2)) && (ref_Y_LT >= mod_info_curr->y_pos &&ref_Y_LT < mod_info_curr->y_pos + (1 << cu_height_log2))) //in the same cu
    {
        int trav_x = ref_X_LT - mod_info_curr->x_pos;
        int trav_y = ref_Y_LT - mod_info_curr->y_pos;
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
__inline u8 is_ref_pix_in_one_ctu(int* puiReverseOrder, COM_MODE *mod_info_curr, int ref_X_LT, int ref_Y_LT, int processed_count, int fir_refx, int fir_refy, int pic_width_in_scu, u32*map
    , int ctulog2size, int m_pic_width
)
{
    //judge if SP reference pix is in One-CTU
    int cu_pel_x = mod_info_curr->x_pos;
    int cu_pel_y = mod_info_curr->y_pos;
    int sp_roi_width = 1 << mod_info_curr->cu_width_log2;
    int sp_roi_height = 1 << mod_info_curr->cu_height_log2;
    int ctu128 = ctulog2size == 7 ? 1 : 0;
    int log2size = ctulog2size == 7 ? 6 : ctulog2size;
    int ctusize = 1 << ctulog2size;
    int offset64x = ((ref_X_LT + ctusize) >> log2size) << log2size;
    int offset64y = (ref_Y_LT >> log2size) << log2size;

    if (ctu128)
    {
        if (ref_X_LT >> MAX_CU_LOG2 == ((cu_pel_x >> MAX_CU_LOG2) - 1) && offset64x < m_pic_width) //in the left LCU but already updated
        {
            if (is_ref_pix_coded(map, mod_info_curr->scup, offset64x, offset64y, pic_width_in_scu) || (offset64x == cu_pel_x && offset64y == cu_pel_y)) //not in virtual buffer of left CTU
            {
                return 0;
            }
        }
    }
    if (!is_ref_pix_valid(puiReverseOrder, mod_info_curr, ref_X_LT, ref_Y_LT, processed_count, pic_width_in_scu, map)) //if in coded area of current CTU
    {
        return 0;
    }
    int fir_ref_64x = ((fir_refx + ctusize) >> log2size) << log2size;
    int fir_ref_64y = (fir_refy >> log2size) << log2size;
    if (offset64x != fir_ref_64x || offset64y != fir_ref_64y) // if string cross two 1/4 ctu
    {
        return 0;
    }
    return 1;
}
__inline u8 dec_is_pix_ref_valid(int* puiReverseOrder, COM_MODE *mod_info_curr, int ref_X_LT, int ref_Y_LT, int sp_roi_width, int sp_roi_height, int cu_pel_x, int cu_pel_y, int processed_count, int m_pic_width, int m_pic_height, int fir_refx, int fir_refy, int pic_width_in_scu, u32*map
    , int ctulog2size
)
{
    // is pixel in reference string within picture boundry
    if (ref_Y_LT < 0 || ref_Y_LT >= m_pic_height || ref_X_LT < 0 || ref_X_LT >= m_pic_width)
    {
        assert(0);
        return 0;
    }
    // is pixel in reference string within curr & left CTU
    if (!is_ref_pix_in_2ctu(ref_X_LT, ref_Y_LT, cu_pel_x, cu_pel_y
        , ctulog2size
    ))
    {
        assert(0);
        return 0;
    }
    // is pixel in reference string within IBC virtual buffer
    if (!is_ref_pix_in_one_ctu(puiReverseOrder, mod_info_curr, ref_X_LT, ref_Y_LT, processed_count, fir_refx, fir_refy, pic_width_in_scu, map
        , ctulog2size, m_pic_width
    ))
    {
        assert(0);
        return 0;
    }
    return 1;
}
#endif 

#if ISC_RSD
void derive_rsd_string(COM_MODE* mod_info_curr, int cu_width_log2, int cu_height_log2, COM_SP_INFO* derived_rsd_str_info)
{
    COM_SP_INFO* p_sp_info = mod_info_curr->string_copy_info;
    derived_rsd_str_info[0].length = 4;
    derived_rsd_str_info[0].offset_x = p_sp_info->offset_x >> 2;
    derived_rsd_str_info[0].offset_y = p_sp_info->offset_y >> 2;
    derived_rsd_str_info[0].sp_rsd_flag = p_sp_info->sp_rsd_flag;
    derived_rsd_str_info[0].intra_row_rsd_flag = p_sp_info->intra_row_rsd_flag;
    derived_rsd_str_info[1].length = (1 << cu_width_log2) * (1 << cu_height_log2) - 4;
    derived_rsd_str_info[1].offset_x = p_sp_info->offset_x >> 2;
    derived_rsd_str_info[1].offset_y = p_sp_info->offset_y >> 2;
    derived_rsd_str_info[1].sp_rsd_flag = p_sp_info->sp_rsd_flag;
    derived_rsd_str_info[1].intra_row_rsd_flag = p_sp_info->intra_row_rsd_flag;
}

void rsd_recon_yuv(COM_MODE* mod_info_curr, int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC* pic, CHANNEL_TYPE channel
    , int pic_width, int pic_height, int pic_width_in_scu, u32* map
    , int ctu_log2_size, COM_SP_INFO* derived_rsd_str_info, u16 derived_rsd_str_no)
{
    int scu_x = 1 << (cu_width_log2 - 2);
    int scu_y = 1 << (cu_height_log2 - 2);
    u32* scu = map + mod_info_curr->scup;
    for (int j = 0; j < scu_y; j++)
    {
        for (int i = 0; i < scu_x; i++)
        {
            MCU_CLR_CODED_FLAG(scu[i]);
        }
        scu += pic_width_in_scu;
    }
    int total_pixel = 1 << (cu_width_log2 + cu_height_log2);
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int processed_count = 0;
    int s_src, s_src_c, s_dst, s_dst_c, i, j;
    int cur_pos, cur_pos_c, ref_pos, ref_pos_c;
    u8 scale_x = 1;
    u8 scale_y = 1;
    s_src = pic->stride_luma;
    s_dst = pic->stride_luma;
    s_src_c = pic->stride_chroma;
    s_dst_c = pic->stride_chroma;
    u8 is_hor_scan = mod_info_curr->sp_copy_direction;
    COM_SP_INFO* p_sp_info = derived_rsd_str_info;
    int str_length = 0;
    int offset_x, offset_y;
    int curr_x, curr_y;
    for (i = 0; i < derived_rsd_str_no; i++)
    {
        int is_ref_pix_valid = 1;
        int fst_ref_pix_x = x + p_sp_info[i].offset_x + com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
        int fst_ref_pix_y = y + p_sp_info[i].offset_y + com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
        if (p_sp_info[i].sp_rsd_flag != 0)
        {
            int sx_in_cu = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
            int sy_in_cu = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
            int last_x_in_cu = 0;
            int last_y_in_cu = 0;
            s16 n_sv[MV_D] = { 0,0 };
            s16 sv_bias[MV_D] = { 0,0 };
            n_sv[MV_X] = p_sp_info->offset_x;
            n_sv[MV_Y] = p_sp_info->offset_y;
            str_length = p_sp_info[i].length;
            offset_x = n_sv[MV_X] + sv_bias[MV_X];
            offset_y = n_sv[MV_Y] + sv_bias[MV_Y];
            if (p_sp_info->intra_row_rsd_flag == 0)
            {
                sv_bias[MV_X] = cu_width - 1 - 2 * sx_in_cu;
                sv_bias[MV_Y] = 0;
            }
            else
            {
                sv_bias[MV_X] = 0;
                sv_bias[MV_Y] = cu_height - 1 - 2 * sy_in_cu;
            }
            for (j = 0; j < str_length; j++)
            {
                curr_x = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
                curr_y = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];

                if (j > 0 && p_sp_info->intra_row_rsd_flag == 0 && ((curr_y - last_y_in_cu) & 0x1) == 0)
                {
                    if ((curr_y & 0x1) == 0)//even line
                    {
                        sv_bias[MV_X] -= 2;
                    }
                    else //odd line
                    {
                        sv_bias[MV_X] += 2;
                    }
                }
                if (j > 0 && p_sp_info->intra_row_rsd_flag == 1)
                {
                    if (((curr_y - last_y_in_cu) & 0x1) == 0)//same line
                    {
                        sv_bias[MV_Y] += 0;
                    }
                    else //different line
                    {
                        sv_bias[MV_Y] -= 2;
                    }
                }
                offset_x = n_sv[MV_X] + sv_bias[MV_X];
                offset_y = n_sv[MV_Y] + sv_bias[MV_Y];
#if !DEC_RANGE_EXTEND
                is_ref_pix_valid = dec_is_pix_ref_valid(com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2], mod_info_curr, x + curr_x + offset_x, y + curr_y + offset_y, cu_width, cu_height, x, y, processed_count, m_pic_width, m_pic_height, fst_ref_pix_x, fst_ref_pix_y, pic_width_in_scu, map , ctu_log2_size);
                assert(is_ref_pix_valid != 0);
#endif
                // luma
                if (channel == CHANNEL_LC || channel == CHANNEL_L)
                {
                    cur_pos = (y + curr_y) * s_dst + (x + curr_x);
                    ref_pos = (y + curr_y + offset_y) * s_src + (x + curr_x + offset_x);
                    pic->y[cur_pos] = pic->y[ref_pos];
                    mod_info_curr->pred[Y_C][curr_y * cu_width + curr_x] = pic->y[ref_pos];
                }
                // chroma
                if (channel == CHANNEL_LC || channel == CHANNEL_C)
                {
                    cur_pos_c = ((y + curr_y) >> scale_y) * s_dst_c + ((x + curr_x) >> scale_x);
                    ref_pos_c = ((y + curr_y + offset_y) >> scale_y) * s_src_c + ((x + curr_x + offset_x) >> scale_x);
                    if (!((curr_x & scale_x) || (curr_y & scale_y)))
                    {
                        pic->u[cur_pos_c] = pic->u[ref_pos_c];
                        pic->v[cur_pos_c] = pic->v[ref_pos_c];
                        mod_info_curr->pred[U_C][(curr_y >> scale_y) * (cu_width >> scale_x) + (curr_x >> scale_x)] = pic->u[ref_pos_c];
                        mod_info_curr->pred[V_C][(curr_y >> scale_y) * (cu_width >> scale_x) + (curr_x >> scale_x)] = pic->v[ref_pos_c];
                    }
                }
                processed_count++;
                last_x_in_cu = curr_x;
                last_y_in_cu = curr_y;
            }

        }
    }
    scu_x = 1 << (cu_width_log2 - 2);
    scu_y = 1 << (cu_height_log2 - 2);
    scu = map + mod_info_curr->scup;
    for (int j = 0; j < scu_y; j++)
    {
        for (int i = 0; i < scu_x; i++)
        {
            MCU_SET_CODED_FLAG(scu[i]);
        }
        scu += pic_width_in_scu;
    }
}
#endif

void sp_recon_yuv(COM_MODE *mod_info_curr, int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC *pic, CHANNEL_TYPE channel
    , int m_pic_width, int m_pic_height, int pic_width_in_scu, u32* map
    , int ctulog2size
)
{
    int scu_x = 1 << (cu_width_log2 - 2);
    int scu_y = 1 << (cu_height_log2 - 2);
    u32 *scu = map + mod_info_curr->scup;
    for (int j = 0; j < scu_y; j++)
    {
        for (int i = 0; i < scu_x; i++)
        {
            MCU_CLR_CODED_FLAG(scu[i]);
        }
        scu += pic_width_in_scu;
    }
    int total_pixel = 1 << (cu_width_log2 + cu_height_log2);
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int processed_count = 0;
    int s_src, s_src_c, s_dst, s_dst_c, i, j;
    int cur_pos, cur_pos_c, ref_pos, ref_pos_c;
    u8 scale_x = 1;
    u8 scale_y = 1;
    s_src = pic->stride_luma;
    s_dst = pic->stride_luma;
    s_src_c = pic->stride_chroma;
    s_dst_c = pic->stride_chroma;
    u8 is_hor_scan = mod_info_curr->sp_copy_direction;
    COM_SP_INFO *p_sp_info = mod_info_curr->string_copy_info;
    int str_length = 0;
    int offset_x, offset_y;
    int curr_x, curr_y;
    int max_str_cnt = cu_width * cu_height / 4;
    int additional_str_num = 0;
    for (i = 0; i < mod_info_curr->sub_string_no; i++) 
    {
        int is_ref_pix_valid = 1;
        int fst_ref_pix_x = x + p_sp_info[i].offset_x + com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
        int fst_ref_pix_y = y + p_sp_info[i].offset_y + com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
        if (p_sp_info[i].is_matched) 
        {
            str_length = p_sp_info[i].length;
            offset_x = p_sp_info[i].offset_x;
            offset_y = p_sp_info[i].offset_y;
            int no_overlap_str_num = 1;
            int no_overlap_str_len[256] = { 0 };
            int x_start_pos = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
            int y_start_pos = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
            int y_end_pos = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count + str_length - 1][1];
            no_overlap_str_len[0] = str_length;
            com_derive_no_overlap_str_len(x_start_pos, y_start_pos, y_end_pos, cu_width, &p_sp_info[i], no_overlap_str_len, &no_overlap_str_num);        
            additional_str_num += no_overlap_str_num - 1;

            for (int k = 0; k < no_overlap_str_num; k++)
            {
                str_length = no_overlap_str_len[k];
                fst_ref_pix_x = x + p_sp_info[i].offset_x + com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
                fst_ref_pix_y = y + p_sp_info[i].offset_y + com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
                for (j = 0; j < str_length; j++)
                {
                    curr_x = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
                    curr_y = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
#if !DEC_RANGE_EXTEND
                    is_ref_pix_valid = dec_is_pix_ref_valid(com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2], mod_info_curr, x + curr_x + offset_x, y + curr_y + offset_y, cu_width, cu_height, x, y, processed_count-j, m_pic_width, m_pic_height, fst_ref_pix_x, fst_ref_pix_y, pic_width_in_scu, map
                        , ctulog2size
                    );
                    assert(is_ref_pix_valid != 0);
#endif
                    // luma
                    if (channel == CHANNEL_LC || channel == CHANNEL_L)
                    {
                        cur_pos = (y + curr_y) * s_dst + (x + curr_x);
                        ref_pos = (y + curr_y + offset_y) * s_src + (x + curr_x + offset_x);
                        pic->y[cur_pos] = pic->y[ref_pos];
                    }
                    // chroma
                    if (channel == CHANNEL_LC || channel == CHANNEL_C)
                    {
                        cur_pos_c = ((y + curr_y) >> scale_y) * s_dst_c + ((x + curr_x) >> scale_x);
                        ref_pos_c = ((y + curr_y + offset_y) >> scale_y) * s_src_c + ((x + curr_x + offset_x) >> scale_x);
                        if (!((curr_x & scale_x) || (curr_y & scale_y)))
                        {
                            pic->u[cur_pos_c] = pic->u[ref_pos_c];
                            pic->v[cur_pos_c] = pic->v[ref_pos_c];
                        }
                    }
                    processed_count++;
                }
            }
        }
        else 
        {
            int unmatched_pixel_num = 0;
            str_length = p_sp_info[i].length;
            if (!p_sp_info[i].is_matched)
            {
                assert(str_length == 4);
            }

            offset_x = p_sp_info[i].offset_x;
            offset_y = p_sp_info[i].offset_y;
            for (int j = 0;j < 4;j++)
            {
                curr_x = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
                curr_y = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
#if !DEC_RANGE_EXTEND
                if (p_sp_info[i].match_dict[j])
                {
                    is_ref_pix_valid = dec_is_pix_ref_valid(com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2], mod_info_curr, x + curr_x + offset_x, y + curr_y + offset_y, cu_width, cu_height, x, y, processed_count-j, m_pic_width, m_pic_height, fst_ref_pix_x, fst_ref_pix_y, pic_width_in_scu, map
                        , ctulog2size
                    );
                    assert(is_ref_pix_valid != 0);
                }
                else
                {
                    unmatched_pixel_num++;
                }
#endif
                // luma
                if (channel == CHANNEL_LC || channel == CHANNEL_L)
                {
                    cur_pos = (y + curr_y) * s_dst + (x + curr_x);
                    if (p_sp_info[i].match_dict[j])
                    {
                        ref_pos = (y + curr_y + offset_y) * s_src + (x + curr_x + offset_x);
                        pic->y[cur_pos] = pic->y[ref_pos];
                    }
                    else
                    {
                        pic->y[cur_pos] = p_sp_info[i].pixel[j][Y_C];
                    }
                }
                // chroma
                if (channel == CHANNEL_LC || channel == CHANNEL_C)
                {
                    cur_pos_c = ((y + curr_y) >> scale_y) * s_dst_c + ((x + curr_x) >> scale_x);
                    if (!((curr_x & scale_x) || (curr_y & scale_y)))
                    {
                        if (p_sp_info[i].match_dict[j])
                        {
                            ref_pos_c = ((y + curr_y + offset_y) >> scale_y) * s_src_c + ((x + curr_x + offset_x) >> scale_x);
                            pic->u[cur_pos_c] = pic->u[ref_pos_c];
                            pic->v[cur_pos_c] = pic->v[ref_pos_c];
                        }
                        else
                        {
                            pic->u[cur_pos_c] = p_sp_info[i].pixel[j][U_C];
                            pic->v[cur_pos_c] = p_sp_info[i].pixel[j][V_C];
                        }
                    }
                }
                processed_count++;
            }
            additional_str_num += unmatched_pixel_num < 4 ? unmatched_pixel_num : 3;
        }
    }
    assert((mod_info_curr->sub_string_no + additional_str_num) <= max_str_cnt);
    scu_x = 1 << (cu_width_log2 - 2);
    scu_y = 1 << (cu_height_log2 - 2);
    scu = map + mod_info_curr->scup;
    for (int j = 0; j < scu_y; j++)
    {
        for (int i = 0; i < scu_x; i++)
        {
            MCU_SET_CODED_FLAG(scu[i]);
        }
        scu += pic_width_in_scu;
    }
}
void sp_cs2_recon_yuv(COM_MODE *mod_info_curr, int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC *pic, pel(*dpb_evs)[MAX_SRB_PRED_SIZE]
    , u8 tree_status
    , int ctu_size
)
{
    int total_pixel = 1 << (cu_width_log2 + cu_height_log2);
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int processed_count = 0;
    int trav_order_index = 0;
    int dict_pix_idx = 0;
    int evs_num = 0;
    int ubvs_num = 0;
    int unpixel_num = 0;

    int s_src, s_src_c, s_dst, s_dst_c, src_444, dst_444, i, j;
    int cur_pos, ref_pos;
    u8 scale_x = 0;
    u8 scale_y = 0;
    s_src = pic->stride_luma;
    s_dst = pic->stride_luma;
    s_src_c = pic->stride_chroma;
    s_dst_c = pic->stride_chroma;
    src_444 = pic->width_luma;
    dst_444 = pic->width_luma;
    u8 is_horizontal_scanning = mod_info_curr->evs_copy_direction;
    COM_SP_EVS_INFO* p_dict_info = mod_info_curr->evs_str_copy_info;

    int* p_trav_scan_order = com_tbl_raster2trav[is_horizontal_scanning][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
    int strlength = 0;
    int curr_x, curr_y;  

    int cu_ext = 0;
    if (tree_status == TREE_L)
    {
        cu_ext = 1;
    }

    for (i = 0; i < mod_info_curr->evs_sub_string_no; i++) {
        int length = p_dict_info[i].length;
        if (p_dict_info[i].match_type == MATCH_POS_WIDTH)
        {
            int offset_x = 0;
            int offset_y = 1;
            {
                int isc_part_ad_num = 1;
                int split_len[256] = { 0 };
                split_len[0] = length;

                int curr_x0 = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][0];
                int curr_y0 = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count][1];
                int curr_y1 = com_tbl_raster2trav_2d[cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2][processed_count + length - 1][1];
                int one_line_max_len = (curr_y0 & 0x1) ? curr_x0 + 1 : (1 << cu_width_log2) - curr_x0;
                int origin_len = length;

                isc_part_ad_num = (curr_y1 - curr_y0) / abs(offset_y) + 1;
                ubvs_num = ubvs_num + isc_part_ad_num;
                {
                    int split_str_cnt = 0;
                    if (origin_len >= one_line_max_len)
                    {
                        int i = 0;
                        while (origin_len > 0)
                        {
                            split_len[split_str_cnt] = one_line_max_len;
                            origin_len -= one_line_max_len;
                            one_line_max_len = (origin_len - (1 << cu_width_log2) >= 0) ? (1 << cu_width_log2) : origin_len;
                            split_str_cnt++;
                        }
                    }
                }
                assert(split_len[isc_part_ad_num - 1] > 0);

                for (int k = 0; k < isc_part_ad_num; k++)
                {
                    length = split_len[k];
            for (j = 0; j < length; j++)
            {
                trav_order_index = p_trav_scan_order[processed_count];
                curr_x = GET_TRAV_X(trav_order_index, cu_width);//trav_order_index % width;
                curr_y = GET_TRAV_Y(trav_order_index, cu_width_log2);//trav_order_index / width;
                cur_pos = (y + curr_y) * s_dst + (x + curr_x);
                ref_pos = (y + curr_y - offset_y) * s_src + (x + curr_x - offset_x);
                pic->y[cur_pos] = pic->y[ref_pos];
                processed_count++;
            }
                }
            }
        }
        else
        {
            if (p_dict_info[i].match_type == MATCH_NONE)
            {
                unpixel_num += length;
            }
            else
            {
                evs_num++;
            }
            for (j = 0; j < length; j++)
            {
                trav_order_index = p_trav_scan_order[processed_count];
                curr_x = GET_TRAV_X(trav_order_index, cu_width);//trav_order_index % width;
                curr_y = GET_TRAV_Y(trav_order_index, cu_width_log2);//trav_order_index / width;
                                                           
                cur_pos = (y + curr_y) * s_dst + (x + curr_x);
                if (p_dict_info[i].match_type == MATCH_NONE)
                {
                    COM_SP_PIX recPixel = mod_info_curr->unpred_pix_info[i + j];
                    int Y = recPixel.Y;
                    pic->y[cur_pos] = Y;
                }
                else
                {
                    int srb_index = p_dict_info[i].srb_index;
                    ref_pos = (mod_info_curr->m_pvbuf[1][srb_index] + mod_info_curr->LcuRy0) * s_src + mod_info_curr->m_pvbuf[0][srb_index] + mod_info_curr->LcuRx0;
                    {
                        pic->y[cur_pos] = pic->y[ref_pos];
                    }
                }
                processed_count++;
            }
            if (p_dict_info[i].match_type == MATCH_NONE)
            {
                i = i + length - 1;
            }
        }
    }
    if (!cu_ext)
    {
        int cur_pix_pos = 0;
        int ref_pos_c;
        scale_x = 1;
        scale_y = 1;
        while (cur_pix_pos < total_pixel)
        {
            int ui_pix_pos = cur_pix_pos;
            int trav_order_index = p_trav_scan_order[ui_pix_pos];
            int trav_x = GET_TRAV_X(trav_order_index, cu_width);
            int trav_y = GET_TRAV_Y(trav_order_index, cu_width_log2);
            if (!((trav_x & 1) || (trav_y & 1)))
            {
                if (mod_info_curr->m_pv_flag[ui_pix_pos] == 1)
                {
                    int U, V = 0;
                    ref_pos_c = ((mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRy0) >> scale_y) * s_src_c + ((mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRx0) >> scale_x);
                    {
                        U = pic->u[ref_pos_c];
                        V = pic->v[ref_pos_c];
                    }

                    pic->u[((y + trav_y) >> 1)*s_dst_c + ((x + trav_x) >> 1)] = U;
                    pic->v[((y + trav_y) >> 1)*s_dst_c + ((x + trav_x) >> 1)] = V;
                }
                else
                {

                    int uvCnt = 0;

                    int U00 = 0, U10 = 0, U01 = 0, U11 = 0;
                    int V00 = 0, V10 = 0, V01 = 0, V11 = 0;

                    if (mod_info_curr->PixelType[ui_pix_pos] != 2)
                    {
                        ref_pos_c = ((mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRy0) >> scale_y) * s_src_c + ((mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRx0) >> scale_x);
                        if (mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0 && mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0)
                        {

                            uvCnt++;

                            {
                                U00 = pic->u[ref_pos_c];
                                V00 = pic->v[ref_pos_c];
                            }
                        }
                    }
                    else
                    {
                        if (mod_info_curr->up_all_comp_flag[mod_info_curr->upIdx[ui_pix_pos]])
                        {

                            uvCnt++;

                            U00 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].U;
                            V00 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].V;
                        }
                    }

                    ui_pix_pos = cur_pix_pos + 1;
                    if (mod_info_curr->PixelType[ui_pix_pos] != 2)
                    {
                        ref_pos_c = ((mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRy0) >> scale_y) * s_src_c + ((mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRx0) >> scale_x);
                        if (mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0 && mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0)
                        {

                            uvCnt++;

                            {
                                U01 = pic->u[ref_pos_c];
                                V01 = pic->v[ref_pos_c];
                            }
                        }
                    }
                    else
                    {
                        if (mod_info_curr->up_all_comp_flag[mod_info_curr->upIdx[ui_pix_pos]])
                        {

                            uvCnt++;

                            U01 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].U;
                            V01 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].V;
                        }
                    }

                    int offsetAbove = 2 * (cu_width - trav_x) - 1;
                    ui_pix_pos = cur_pix_pos + offsetAbove;
                    if (mod_info_curr->PixelType[ui_pix_pos] != 2)
                    {
                        ref_pos_c = ((mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRy0) >> scale_y) * s_src_c + ((mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRx0) >> scale_x);
                        if (mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0 && mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0)
                        {

                            uvCnt++;

                            {
                                U10 = pic->u[ref_pos_c];
                                V10 = pic->v[ref_pos_c];
                            }
                        }
                    }
                    else
                    {
                        if (mod_info_curr->up_all_comp_flag[mod_info_curr->upIdx[ui_pix_pos]])
                        {

                            uvCnt++;

                            U10 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].U;
                            V10 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].V;
                        }
                    }
                    ui_pix_pos = cur_pix_pos + offsetAbove - 1;
                    if (mod_info_curr->PixelType[ui_pix_pos] != 2)
                    {
                        ref_pos_c = ((mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRy0) >> scale_y) * s_src_c + ((mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] + mod_info_curr->LcuRx0) >> scale_x);
                        if (mod_info_curr->m_pvbuf[0][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0 && mod_info_curr->m_pvbuf[1][mod_info_curr->PvAddr[ui_pix_pos]] % 2 == 0)
                        {
                            uvCnt++;

                            {
                                U11 = pic->u[ref_pos_c];
                                V11 = pic->v[ref_pos_c];
                            }
                        }

                    }
                    else
                    {
                        if (mod_info_curr->up_all_comp_flag[mod_info_curr->upIdx[ui_pix_pos]])
                        {
                            uvCnt++;

                            U11 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].U;
                            V11 = mod_info_curr->unpred_pix_info[mod_info_curr->upIdx[ui_pix_pos]].V;
                        }
                    }

                    pic->u[((trav_y + y) >> 1)*s_dst_c + ((x + trav_x) >> 1)] = (U00 + U01 + U10 + U11 + uvCnt / 2) / ((uvCnt == 0) ? 1 : uvCnt);
                    pic->v[((trav_y + y) >> 1)*s_dst_c + ((x + trav_x) >> 1)] = (V00 + V01 + V10 + V11 + uvCnt / 2) / ((uvCnt == 0) ? 1 : uvCnt);
              
 }
            }
            cur_pix_pos++;
        }
    }
    if ((evs_num + ubvs_num + unpixel_num) > cu_width*cu_height / 4)
    {
        assert(0);
    }
    if (mod_info_curr->PixelType)
    {
        free(mod_info_curr->PixelType);
    }
    mod_info_curr->PixelType = NULL;

    if (mod_info_curr->PvAddr)
    {
        free(mod_info_curr->PvAddr);
    }
    mod_info_curr->PvAddr = NULL;

    if (mod_info_curr->upIdx)
    {
        free(mod_info_curr->upIdx);
    }
    mod_info_curr->upIdx = NULL;

    if (mod_info_curr->up_all_comp_flag)
    {
        free(mod_info_curr->up_all_comp_flag);
    }
    mod_info_curr->up_all_comp_flag = NULL;

    if (mod_info_curr->evs_str_copy_info)
    {
        free(mod_info_curr->evs_str_copy_info);
    }
    mod_info_curr->evs_str_copy_info = NULL;
    if (mod_info_curr->unpred_pix_info)
    {
        free(mod_info_curr->unpred_pix_info);
    }
    mod_info_curr->unpred_pix_info = NULL;
}
#endif