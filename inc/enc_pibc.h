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

#ifndef _ENC_PIBC_H_
#define _ENC_PIBC_H_

#include "enc_def.h"

#if USE_IBC

#ifdef __cplusplus
extern "C"
{
#endif

    int pibc_init_frame(ENC_CTX *ctx);
    int pibc_init_lcu(ENC_CTX *ctx, ENC_CORE *core);
    double analyze_ibc_cu(ENC_CTX *ctx, ENC_CORE *core);
#if RSD_OPT
    double analyze_rsd_cu(ENC_CTX* ctx, ENC_CORE* core);
#endif
#if CIBC
    double analyze_ibc_cu_chroma(ENC_CTX* ctx, ENC_CORE* core);
#endif
#if IBC_ENH
    void check_ibc_pred_filter(ENC_CTX* ctx, ENC_CORE* core, double* cost_best);
#endif
    int pibc_set_complexity(ENC_CTX *ctx, int complexity);
#if IBC_BVP
    u32 get_bv_cost_bits_ibc_hash(ENC_CORE *core, int mv_x, int mv_y, u8 *bvp_idx);
    u32 get_bv_cost_bits_ibc_local(ENC_CORE *core, int mv_x, int mv_y, u8 *bvp_idx, u32 bvd_bits_table[MAX_NUM_BVP], int dir);
    void get_bvd_bits_one_direction(ENC_CORE *core, u32 bvd_bits_table[MAX_NUM_BVP], int dir, int dir_value);
#endif
    u32 get_bv_cost_bits(int mv_x, int mv_y);
#define GET_MV_COST(ctx, mv_bits)  ((u32)((pi->lambda_mv * mv_bits + (1 << 15)) >> 16))

    void reset_ibc_search_range(ENC_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh);

    double pibc_residue_rdo_chroma(ENC_CTX *ctx, ENC_CORE *core);

    void check_best_ibc_mode(ENC_CORE *core, ENC_PIBC *pi, const double cost_curr, double *cost_best);

    // for ibc pu validation
    __inline int is_bv_valid(ENC_CTX *ctx, int x, int y, int width, int height, int log2_cuw, int log2_cuh,
        int pic_width, int pic_height, int x_bv, int y_bv, int ctu_size)
    {
        int x_scu = PEL2SCU(x);
        int y_scu = PEL2SCU(y);
        const int ctu_size_log2 = ctx->pibc.ctu_log2_tbl[ctu_size];
        int ref_right_x = x + x_bv + width - 1;
        int ref_bottom_y = y + y_bv + height - 1;
        int ref_left_x = x + x_bv;
        int ref_top_y = y + y_bv;

        if (ref_left_x < 0)
        {
            return 0;
        }
        if (ref_right_x >= pic_width)
        {
            return 0;
        }
        if (ref_top_y < 0)
        {
            return 0;
        }
        if (ref_bottom_y >= pic_height)
        {
            return 0;
        }
        if ((x_bv + width) > 0 && (y_bv + height) > 0)
        {
            return 0;
        }
#if IBC_REF_RANGE_XLCUR
        int ctu_mvy = IBC_REF_RANGE_XLCUR - 1;
        int ctu_topy = 0;
        int ctu_y = y >> ctu_size_log2;
        if (ctu_y < ctu_mvy)
        {
            ctu_topy = 0;
        }
        else
        {
            ctu_topy = ctu_y - ctu_mvy;
        }
        if ((ref_top_y >> ctu_size_log2) < ctu_topy)
        {
            return 0;
        }
#else
        // cannot be in the above CTU row
        if ((ref_top_y >> ctu_size_log2) < (y >> ctu_size_log2))
        {
            return 0;
        }
#endif
        // cannot be in the below CTU row
        if ((ref_bottom_y >> ctu_size_log2) > (y >> ctu_size_log2))
        {
            return 0;
        }
#if IBC_REF_RANGE_XqLCU ==0
        // in the same CTU line
        int numLeftCTUs = (1 << ((7 - ctu_size_log2) << 1)) - ((ctu_size_log2 < 7) ? 1 : 0);
        //int numLeftCTUs = 0; //current CTU
        if (((ref_right_x >> ctu_size_log2) <= (x >> ctu_size_log2)) && ((ref_left_x >> ctu_size_log2) >= (x >> ctu_size_log2) - numLeftCTUs))
        {
            // in the same CTU, or left CTU
            // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
            if (((ref_left_x >> ctu_size_log2) == ((x >> ctu_size_log2) - 1)) && (ctu_size_log2 == 7))
            {
                // ref block's collocated block in current CTU
                int ref_pos_col_x = x + x_bv + ctu_size;
                int ref_pos_col_y = y + y_bv;
                int offset64x = (ref_pos_col_x >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                int offset64y = (ref_pos_col_y >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                u32 offset_x_scu = PEL2SCU(offset64x);
                u32 offset_y_scu = PEL2SCU(offset64y);
                u32 offset_scup = ((u32)offset_y_scu * ctx->info.pic_width_in_scu) + offset_x_scu;

                int avail_cu = offset64x < pic_width ? MCU_GET_CODED_FLAG(ctx->map.map_scu[offset_scup]) : 0;
                if (avail_cu)
                {
                    return 0;
                }
                if (offset64x == x && offset64y == y)
                {
                    return 0;
                }
            }
        }
        else
        {
            return 0;
        }
#else 
        int ctu_mvx = 0;
        int ctu_luclength = 0;
        int ctu_xrightbound = 0;
        ctu_mvx = IBC_REF_RANGE_XqLCU - 1;
#if REF_RANGE_FIX
        ctu_luclength = (1 << SRAM_SIZE_128_LOG2) * (IBC_REF_RANGE_XqLCU - 1);
#else
        ctu_luclength = 128 * (IBC_REF_RANGE_XqLCU - 1);
#endif
        // in the same CTU line
        int numLeftCTUs = (1 << ((7 - ctu_size_log2) << 1)) - ((ctu_size_log2 < 7) ? 1 : 0);//1
#if IBSC_RIGHT_BOUND==0
        if (((ref_right_x >> ctu_size_log2) <= (x >> ctu_size_log2)) && ((ref_left_x >> ctu_size_log2) >= (x >> ctu_size_log2) - numLeftCTUs - ctu_mvx))
#else
        if ((ref_top_y >> ctu_size_log2) == (y >> ctu_size_log2))
        {
            ctu_xrightbound = (x >> ctu_size_log2);
        }
        else
        {
            ctu_xrightbound = (pic_width >> ctu_size_log2);
        }
        if (((ref_right_x >> ctu_size_log2) <= ctu_xrightbound) && ((ref_left_x >> ctu_size_log2) >= (x >> ctu_size_log2) - numLeftCTUs - ctu_mvx))
#endif 
        {
            // in the same CTU, or left CTU
            // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
            if (((ref_left_x >> ctu_size_log2) == ((x >> ctu_size_log2) - numLeftCTUs - ctu_mvx)) && (ctu_size_log2 == 7))
            {
                // ref block's collocated block in current CTU
                int ref_pos_col_x = x + x_bv + ctu_size + ctu_luclength;
                int ref_pos_col_y = y + y_bv;
                int offset64x = (ref_pos_col_x >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                int offset64y = (ref_pos_col_y >> (ctu_size_log2 - 1)) << (ctu_size_log2 - 1);
                u32 offset_x_scu = PEL2SCU(offset64x);
                u32 offset_y_scu = PEL2SCU(offset64y);
                u32 offset_scup = ((u32)offset_y_scu * ctx->info.pic_width_in_scu) + offset_x_scu;
                int avail_cu = offset64x < pic_width ? MCU_GET_CODED_FLAG(ctx->map.map_scu[offset_scup]) : 0;

                if (avail_cu)
                {
                    return 0;
                }
                if (offset64x == x && offset64y == y)
                {
                    return 0;
                }
            }
        }
        else
        {
            return 0;
        }
#endif
        // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
        int ref_pos_LT_x = x + x_bv;
        int ref_pos_LT_y = y + y_bv;

        u32 ref_pos_LT_x_scu = PEL2SCU(ref_pos_LT_x);
        u32 ref_pos_LT_y_scu = PEL2SCU(ref_pos_LT_y);
        u32 ref_pos_LT_scup = ((u32)ref_pos_LT_y_scu * ctx->info.pic_width_in_scu) + ref_pos_LT_x_scu;

        int avail_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_LT_scup]);
        if (avail_cu == 0)
        {
            return 0;
        }

        int ref_pos_BR_x = x + width - 1 + x_bv;
        int ref_pos_BR_y = y + height - 1 + y_bv;

        u32 ref_pos_BR_x_scu = PEL2SCU(ref_pos_BR_x);
        u32 ref_pos_BR_y_scu = PEL2SCU(ref_pos_BR_y);
        u32 ref_pos_BR_scup = ((u32)ref_pos_BR_y_scu * ctx->info.pic_width_in_scu) + ref_pos_BR_x_scu;

        avail_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_BR_scup]);
        if (avail_cu == 0)
        {
            return 0;
        }
#if !IBC_CROSS_64
        int vSize = ctu_size >= 64 ? 64 : ctu_size;
        if (ref_pos_LT_x / vSize != ref_pos_BR_x / vSize || ref_pos_LT_y / vSize != ref_pos_BR_y / vSize)
        {
            return 0;
        }
#endif
        return 1;
    }
#if RBU_LIMIT_IBC||ExtStrIBC
    __inline int is_ref_pix_in_raw_range(int x, int y, int width, int height, int ref_X_LT, int ref_Y_LT)
    {
        int cu_pel_x = x;//in pic
        int cu_pel_y = y;
#if CTU_256
        int x_in_lcu = cu_pel_x & ((1 << MAX_CU_LOG22) - 1);
        int y_in_lcu = cu_pel_y & ((1 << MAX_CU_LOG22) - 1);
#else
        int x_in_lcu = cu_pel_x & ((1 << MAX_CU_LOG2) - 1);
        int y_in_lcu = cu_pel_y & ((1 << MAX_CU_LOG2) - 1);
#endif
        int sp_roi_width = width;
        int sp_roi_height = height;
#if CTU_256
        int offset64x = ((ref_X_LT + MAX_CU_SIZE2) >> (MAX_CU_LOG22 - 1)) << (MAX_CU_LOG22 - 1);//ref_X_LT ref_x_in_pic_n;ref_Y_LT ref_y_in_pic_n
        int offset64y = (ref_Y_LT >> (MAX_CU_LOG22 - 1)) << (MAX_CU_LOG22 - 1);
        if (ref_Y_LT >> MAX_CU_LOG22 == ((cu_pel_y >> MAX_CU_LOG22)))
        {
            if ((ref_X_LT >> MAX_CU_LOG22) == ((cu_pel_x >> MAX_CU_LOG22) - 1)) //in the left LCU but already updated
            {
#else
        int offset64x = ((ref_X_LT + MAX_CU_SIZE) >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);//ref_X_LT ref_x_in_pic_n;ref_Y_LT ref_y_in_pic_n
        int offset64y = (ref_Y_LT >> (MAX_CU_LOG2 - 1)) << (MAX_CU_LOG2 - 1);
        if (ref_Y_LT >> MAX_CU_LOG2 == ((cu_pel_y >> MAX_CU_LOG2)))
        {
            if ((ref_X_LT >> MAX_CU_LOG2) == ((cu_pel_x >> MAX_CU_LOG2) - 1)) //in the left LCU but already updated
            {
#endif
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
            }
#if CTU_256
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
#if RBU_LIMIT_IBC
    int calc_recString_ext_rect_coord(int CuWidth, int x_start_in_pic, int y_start_in_pic, int start_x, int start_y, int end_x, int end_y, int NumCodedPixel, int SL, int sv[2]);
    /*__inline int calc_recString_ext_rect_coord(int CuWidth, int x_start_in_pic, int y_start_in_pic, int start_x, int start_y, int end_x, int end_y, int NumCodedPixel, int SL, int sv[2])
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
    }*/
#endif
#ifdef __cplusplus
}
#endif

#endif

#endif /* _ENC_PIBC_H_ */
