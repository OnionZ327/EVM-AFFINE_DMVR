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

 /** \file     enc_ibc_hashmap.cpp
     \brief    IBC hash map encoder class
 */


#include "enc_ibc_hashmap.h"
#include "enc_pibc.h"
#include "enc_ibc_hash_wrapper.h"

#if FBHT
#include <algorithm>
#include <iterator>
#endif

#if USE_IBC

#define MIN_PU_SIZE 4

#ifdef __cplusplus
extern "C" {
#endif
    ibc_hash_handle* create_enc_IBC(int picWidth, int picHeight)
    {
        ENC_IBC_HashMap* p_hash = new ENC_IBC_HashMap;
        p_hash->init(picWidth, picHeight);

        return (ibc_hash_handle *)p_hash;
    }

    void destroy_enc_IBC(ibc_hash_handle* p)
    {
        ENC_IBC_HashMap* p_hash = (ENC_IBC_HashMap*)p;
        p_hash->destroy();
        delete p_hash;
    }

#if FBHT
    void rebuild_hashmap(ibc_hash_handle* p, const COM_PIC* pic, ENC_CTX* ctx)
#else
    void rebuild_hashmap(ibc_hash_handle* p, const COM_PIC* pic)
#endif
    {
        ENC_IBC_HashMap* p_hash = (ENC_IBC_HashMap*)p;

#if FBHT
        p_hash->rebuildPicHashMap(pic, ctx);
#else
        p_hash->rebuildPicHashMap(pic);
#endif
    }

#if IBC_BVP
    u32 search_ibc_hash_match(ENC_CORE *core, ENC_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y,
        int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D], u8 *bvp_idx)
#else
    u32 search_ibc_hash_match(ENC_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y,
        int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D])
#endif
    {
        u32 cost = 0;
        u32 min_cost = ((u32)0xFFFFFFFF);

        ENC_PIBC *pi = &ctx->pibc;

        ENC_IBC_HashMap* p_hash = (ENC_IBC_HashMap*)p;

#if IBC_BVP
        u8 bvp_idx_temp = 0;
#else
        mvp[MV_X] = 0;
        mvp[MV_Y] = 0;
#endif

        std::vector<Position> candPos;
        if (p_hash->ibcHashMatch(ctx, cu_x, cu_y, log2_cuw, log2_cuh, candPos,
            ctx->param.ibc_hash_search_max_cand, ctx->param.ibc_hash_search_range_4smallblk))
        {

            const unsigned int  lcuWidth = (1 << ctx->info.log2_max_cuwh);
            const int pic_width = ctx->info.pic_width;
            const int pic_height = ctx->info.pic_height;
            int roi_width = (1 << log2_cuw);
            int roi_height = (1 << log2_cuh);

            for (std::vector<Position>::iterator pos = candPos.begin(); pos != candPos.end(); pos++)
            {
                u32 ref_pos_LT_x_scu = PEL2SCU(pos->first);
                u32 ref_pos_LT_y_scu = PEL2SCU(pos->second);
                u32 ref_pos_LT_scup = ((u32)ref_pos_LT_y_scu * ctx->info.pic_width_in_scu) + ref_pos_LT_x_scu;

                int avail_LT_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_LT_scup]);

                int ref_bottom_right_x = pos->first + roi_width - 1;
                int ref_bottom_right_y = pos->second + roi_height - 1;

                u32 ref_pos_BR_x_scu = PEL2SCU(ref_bottom_right_x);
                u32 ref_pos_BR_y_scu = PEL2SCU(ref_bottom_right_y);
                u32 ref_pos_BR_scup = ((u32)ref_pos_BR_y_scu * ctx->info.pic_width_in_scu) + ref_pos_BR_x_scu;

                if (ref_bottom_right_x >= pic_width || ref_bottom_right_y >= pic_height)
                {
                    continue;
                }

                int avail_BR_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_BR_scup]);

                if (avail_LT_cu && avail_BR_cu)
                {
                    s16 cand_mv[MV_D];
                    cand_mv[MV_X] = pos->first - cu_x;
                    cand_mv[MV_Y] = pos->second - cu_y;

                    if (!is_bv_valid(ctx, cu_x, cu_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, cand_mv[0], cand_mv[1], lcuWidth))
                    {
                        continue;
                    }
#if ExtStrIBC||RBU_LIMIT_IBC
                    {

                        int is_out_of_raw_range_flag = 0;
                        int tmp[4][2] = { 0, 0, roi_width - 1, 0, roi_width - 1, roi_height - 1, 0, roi_height - 1 };
                        for (int k = 0; k < 4; k++) {
                            int current_x1;
                            int current_y1;
                            current_x1 = cu_x + tmp[k][0];
                            current_y1 = cu_y + tmp[k][1];
                            int ref_start_x_in_pic1 = current_x1 + cand_mv[MV_X];
                            int ref_start_y_in_pic1 = current_y1 + cand_mv[MV_Y];
                            if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                            {
                                is_out_of_raw_range_flag = 1;//in_raw_rangezai
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
#endif
#if RBU_LIMIT_IBC	
                        if (is_out_of_raw_range_flag)
                        {
                            int nbu = 0;
                            int sv[2] = { cand_mv[MV_X],cand_mv[MV_Y] };
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
#if IBC_BVP
                    int mv_bits = get_bv_cost_bits_ibc_hash(core, cand_mv[MV_X], cand_mv[MV_Y], &bvp_idx_temp);
#else
                    int mv_bits = get_bv_cost_bits(cand_mv[MV_X], cand_mv[MV_Y]);
#endif
                    cost = GET_MV_COST(ctx, mv_bits);

                    if (cost < min_cost)
                    {
                        mv[0] = cand_mv[0];
                        mv[1] = cand_mv[1];
                        min_cost = cost;
#if IBC_BVP
                        bvp_idx[0] = bvp_idx_temp;
#endif
                    }
                }
            }
        }

        return min_cost;
    }

#if FAST_SCC_PRECODING
#if IBC_BVP
    u32 search_ibc_hash_matchrevisit(ENC_CORE* core, ENC_CTX* ctx, ibc_hash_handle* p, int cu_x, int cu_y,
        int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D], u8* bvp_idx)
#else
    u32 search_ibc_hash_match(ENC_CTX* ctx, ibc_hash_handle* p, int cu_x, int cu_y,
        int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D])
#endif
    {
        u32 cost = 0;
        u32 min_cost = ((u32)0xFFFFFFFF);

        ENC_PIBC* pi = &ctx->pibc;

        ENC_IBC_HashMap* p_hash = (ENC_IBC_HashMap*)p;

#if IBC_BVP
        u8 bvp_idx_temp = 0;
#else
        mvp[MV_X] = 0;
        mvp[MV_Y] = 0;
#endif
        s16 mv_temp[MV_D] = { 0, 0 };
        mv_temp[0] = core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0mv[MV_X];
        mv_temp[1] = core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0mv[MV_Y];
        int refposX = mv_temp[0] + cu_x;
        int refposY = mv_temp[1] + cu_y;
        const unsigned int  lcuWidth = (1 << ctx->info.log2_max_cuwh);
        const int pic_width = ctx->info.pic_width;
        const int pic_height = ctx->info.pic_height;
        int roi_width = (1 << log2_cuw);
        int roi_height = (1 << log2_cuh);
        u32 ref_pos_LT_x_scu = PEL2SCU(refposX);
        u32 ref_pos_LT_y_scu = PEL2SCU(refposY);
        u32 ref_pos_LT_scup = ((u32)ref_pos_LT_y_scu * ctx->info.pic_width_in_scu) + ref_pos_LT_x_scu;

        int avail_LT_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_LT_scup]);

        int ref_bottom_right_x = refposX + roi_width - 1;
        int ref_bottom_right_y = refposY + roi_height - 1;

        u32 ref_pos_BR_x_scu = PEL2SCU(ref_bottom_right_x);
        u32 ref_pos_BR_y_scu = PEL2SCU(ref_bottom_right_y);
        u32 ref_pos_BR_scup = ((u32)ref_pos_BR_y_scu * ctx->info.pic_width_in_scu) + ref_pos_BR_x_scu;

        if (ref_bottom_right_x >= pic_width || ref_bottom_right_y >= pic_height)
        {
            return min_cost;
        }

        int avail_BR_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_BR_scup]);

        if (avail_LT_cu && avail_BR_cu)
        {
            s16 cand_mv[MV_D];
            cand_mv[MV_X] = refposX - cu_x;
            cand_mv[MV_Y] = refposY - cu_y;

            if (!is_bv_valid(ctx, cu_x, cu_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, cand_mv[0], cand_mv[1], lcuWidth))
            {
                return min_cost;
            }
#if ExtStrIBC||RBU_LIMIT_IBC
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { 0, 0, roi_width - 1, 0, roi_width - 1, roi_height - 1, 0, roi_height - 1 };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = cu_x + tmp[k][0];
                    current_y1 = cu_y + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + cand_mv[MV_X];
                    int ref_start_y_in_pic1 = current_y1 + cand_mv[MV_Y];
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, roi_width, roi_height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;//in_raw_rangezai
                        break;
                    }
                }
#if ExtStrIBC
                if (roi_width * roi_height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        return min_cost;
                    }

                }
#endif
#endif
#if RBU_LIMIT_IBC	
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { cand_mv[MV_X],cand_mv[MV_Y] };
                    int end_x_in_pic = cu_x;
                    int end_y_in_pic = cu_y + roi_height - 1;
                    nbu = calc_recString_ext_rect_coord(roi_width, cu_x, cu_y, cu_x, cu_y, end_x_in_pic, end_y_in_pic, 0, roi_width * roi_height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * roi_width * roi_height)
                    {
                        return min_cost;
                    }
                }
            }
#endif
#if IBC_BVP
            int mv_bits = get_bv_cost_bits_ibc_hash(core, cand_mv[MV_X], cand_mv[MV_Y], &bvp_idx_temp);
#else
            int mv_bits = get_bv_cost_bits(cand_mv[MV_X], cand_mv[MV_Y]);
#endif
            cost = GET_MV_COST(ctx, mv_bits);

            if (cost < min_cost)
            {
                mv[0] = cand_mv[0];
                mv[1] = cand_mv[1];
                min_cost = cost;
#if IBC_BVP
                bvp_idx[0] = bvp_idx_temp;
#endif
            }
        }

        return min_cost;
    }
#endif

    int get_hash_hit_ratio(ENC_CTX *ctx, ibc_hash_handle* p, int cu_x, int cu_y, int log2_cuw, int log2_cuh)
    {
        int pic_width = ctx->info.pic_width;
        int pic_height = ctx->info.pic_height;
        int roi_width = (1 << log2_cuw);
        int roi_height = (1 << log2_cuh);

        ENC_IBC_HashMap* p_hash = (ENC_IBC_HashMap*)p;

        int maxX = COM_MIN((int)(cu_x + roi_width), pic_width);
        int maxY = COM_MIN((int)(cu_y + roi_height), pic_height);
        int hit = 0, total = 0;
        for (int y = cu_y; y < maxY; y += MIN_PU_SIZE)
        {
            for (int x = cu_x; x < maxX; x += MIN_PU_SIZE)
            {
                const unsigned int hash = p_hash->m_pos2Hash[y][x];
                hit += (p_hash->m_hash2Pos[hash].size() > 1);
                total++;
            }
        }
        assert(total > 0);
        return 100 * hit / total;

        //return 0;
    }

#if ISC_RSD && !RSD_OPT
    rsd_hash_handle* create_enc_RSD(int picWidth, int picHeight)
    {
        ENC_RSD_HashMap* p_hash = new ENC_RSD_HashMap;
        p_hash->init(picWidth, picHeight);
        return (rsd_hash_handle*)p_hash;
    }

    void destroy_enc_RSD(rsd_hash_handle* p)
    {
        ENC_RSD_HashMap* p_hash = (ENC_RSD_HashMap*)p;
        p_hash->destroy();
        delete p_hash;
    }

    void rebuild_hashmap_rsd(rsd_hash_handle* p, const COM_PIC* pic)
    {
        ENC_RSD_HashMap* p_hash = (ENC_RSD_HashMap*)p;

        p_hash->rebuildPicHashMapRsd(pic);
    }

    u32 search_rsd_hash_match(ENC_CORE* core, ENC_CTX* ctx, rsd_hash_handle* p, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 mvp[MV_D], s16 mv[MV_D], u8* bvp_idx)
    {
        u32 cost = 0;
        u32 min_cost = ((u32)0xFFFFFFFF);
        ENC_PIBC* pi = &ctx->pibc;
        ENC_RSD_HashMap* p_hash = (ENC_RSD_HashMap*)p;

        u8 bvp_idx_temp = 0;
        int intra_row_rsd_flag = core->mod_info_curr.intra_row_rsd_flag;
        std::vector<PositionRsd> candPos;
        if (p_hash->rsdHashMatch(ctx, cu_x, cu_y, log2_cuw, log2_cuh, candPos, ctx->param.ibc_hash_search_max_cand, ctx->param.ibc_hash_search_range_4smallblk, intra_row_rsd_flag, PIC_ORG(ctx)))
        {
            const unsigned int  lcuWidth = (1 << ctx->info.log2_max_cuwh);
            const int pic_width = ctx->info.pic_width;
            const int pic_height = ctx->info.pic_height;
            int roi_width = (1 << log2_cuw);
            int roi_height = (1 << log2_cuh);
            for (std::vector<PositionRsd>::iterator pos = candPos.begin(); pos != candPos.end(); pos++)
            {
                u32 ref_pos_LT_x_scu = PEL2SCU(pos->first);
                u32 ref_pos_LT_y_scu = PEL2SCU(pos->second);
                u32 ref_pos_LT_scup = ((u32)ref_pos_LT_y_scu * ctx->info.pic_width_in_scu) + ref_pos_LT_x_scu;

                int avail_LT_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_LT_scup]);

                int ref_bottom_right_x = pos->first + roi_width - 1;
                int ref_bottom_right_y = pos->second + roi_height - 1;

                u32 ref_pos_BR_x_scu = PEL2SCU(ref_bottom_right_x);
                u32 ref_pos_BR_y_scu = PEL2SCU(ref_bottom_right_y);
                u32 ref_pos_BR_scup = ((u32)ref_pos_BR_y_scu * ctx->info.pic_width_in_scu) + ref_pos_BR_x_scu;

                if (ref_bottom_right_x >= pic_width || ref_bottom_right_y >= pic_height)
                {
                    continue;
                }

                int avail_BR_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[ref_pos_BR_scup]);

                if (avail_LT_cu && avail_BR_cu)
                {
                    s16 cand_mv[MV_D];
                    cand_mv[MV_X] = pos->first - cu_x;
                    cand_mv[MV_Y] = pos->second - cu_y;

                    if (!is_bv_valid(ctx, cu_x, cu_y, roi_width, roi_height, log2_cuw, log2_cuh, pic_width, pic_height, cand_mv[0], cand_mv[1], lcuWidth))
                    {
                        continue;
                    }
#if ExtStrIBC||RBU_LIMIT_IBC
                    {

                        int is_out_of_raw_range_flag = 0;
                        int tmp[4][2] = { 0, 0, roi_width - 1, 0, roi_width - 1, roi_height - 1, 0, roi_height - 1 };
                        for (int k = 0; k < 4; k++) {
                            int current_x1;
                            int current_y1;
                            current_x1 = cu_x + tmp[k][0];
                            current_y1 = cu_y + tmp[k][1];
                            int ref_start_x_in_pic1 = current_x1 + cand_mv[MV_X];
                            int ref_start_y_in_pic1 = current_y1 + cand_mv[MV_Y];
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
#endif
#if RBU_LIMIT_IBC	
                        if (is_out_of_raw_range_flag)
                        {
                            int nbu = 0;
                            int sv[2] = { cand_mv[MV_X],cand_mv[MV_Y] };
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
#if IBC_BVP
                    int mv_bits = get_bv_cost_bits_ibc_hash(core, cand_mv[MV_X], cand_mv[MV_Y], &bvp_idx_temp);
#else
                    int mv_bits = get_bv_cost_bits(cand_mv[MV_X], cand_mv[MV_Y]);
#endif
                    cost = GET_MV_COST(ctx, mv_bits);
                    if (cost < min_cost)
                    {
                        mv[0] = cand_mv[0];
                        mv[1] = cand_mv[1];
                        min_cost = cost;
#if IBC_BVP
                        bvp_idx[0] = bvp_idx_temp;
#endif
                    }
                }
            }
        }

        return min_cost;
    }
#endif
#ifdef __cplusplus
} //<-- extern "C"
#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

ENC_IBC_HashMap::ENC_IBC_HashMap()
{
    m_picWidth = 0;
    m_picHeight = 0;
    m_pos2Hash = NULL;
    m_computeCrc32c = xxComputeCrc32c16bit;
}

ENC_IBC_HashMap::~ENC_IBC_HashMap()
{
    destroy();
}

void ENC_IBC_HashMap::init(const int picWidth, const int picHeight)
{
    if (picWidth != m_picWidth || picHeight != m_picHeight)
    {
        destroy();
    }

    m_picWidth = picWidth;
    m_picHeight = picHeight;
    m_pos2Hash = new unsigned int*[m_picHeight];
    m_pos2Hash[0] = new unsigned int[m_picWidth * m_picHeight];
    for (int n = 1; n < m_picHeight; n++)
    {
        m_pos2Hash[n] = m_pos2Hash[n - 1] + m_picWidth;
    }
}

void ENC_IBC_HashMap::destroy()
{
    if (m_pos2Hash != NULL)
    {
        if (m_pos2Hash[0] != NULL)
        {
            delete[] m_pos2Hash[0];
        }
        delete[] m_pos2Hash;
    }
    m_pos2Hash = NULL;
}

#if ISC_RSD && !RSD_OPT
ENC_RSD_HashMap::ENC_RSD_HashMap()
{
    m_picWidth = 0;
    m_picHeight = 0;
    m_pos2HashRsd = NULL;
    m_computeCrc32cRsd = xxComputeCrc32c16bitRsd;
}

ENC_RSD_HashMap::~ENC_RSD_HashMap()
{
    destroy();
}

void ENC_RSD_HashMap::init(const int picWidth, const int picHeight)
{
    if (picWidth != m_picWidth || picHeight != m_picHeight)
    {
        destroy();
    }

    m_picWidth = picWidth;
    m_picHeight = picHeight;
    m_pos2HashRsd = new unsigned int* [m_picHeight];
    m_pos2HashRsd[0] = new unsigned int[m_picWidth * m_picHeight];
    for (int n = 1; n < m_picHeight; n++)
    {
        m_pos2HashRsd[n] = m_pos2HashRsd[n - 1] + m_picWidth;
    }
}

void ENC_RSD_HashMap::destroy()
{
    if (m_pos2HashRsd != NULL)
    {
        if (m_pos2HashRsd[0] != NULL)
        {
            delete[] m_pos2HashRsd[0];
        }
        delete[] m_pos2HashRsd;
    }
    m_pos2HashRsd = NULL;
}
#endif
////////////////////////////////////////////////////////
// CRC32C calculation in C code, same results as SSE 4.2's implementation
static const uint32_t crc32Table[256] = {
  0x00000000L, 0xF26B8303L, 0xE13B70F7L, 0x1350F3F4L,
  0xC79A971FL, 0x35F1141CL, 0x26A1E7E8L, 0xD4CA64EBL,
  0x8AD958CFL, 0x78B2DBCCL, 0x6BE22838L, 0x9989AB3BL,
  0x4D43CFD0L, 0xBF284CD3L, 0xAC78BF27L, 0x5E133C24L,
  0x105EC76FL, 0xE235446CL, 0xF165B798L, 0x030E349BL,
  0xD7C45070L, 0x25AFD373L, 0x36FF2087L, 0xC494A384L,
  0x9A879FA0L, 0x68EC1CA3L, 0x7BBCEF57L, 0x89D76C54L,
  0x5D1D08BFL, 0xAF768BBCL, 0xBC267848L, 0x4E4DFB4BL,
  0x20BD8EDEL, 0xD2D60DDDL, 0xC186FE29L, 0x33ED7D2AL,
  0xE72719C1L, 0x154C9AC2L, 0x061C6936L, 0xF477EA35L,
  0xAA64D611L, 0x580F5512L, 0x4B5FA6E6L, 0xB93425E5L,
  0x6DFE410EL, 0x9F95C20DL, 0x8CC531F9L, 0x7EAEB2FAL,
  0x30E349B1L, 0xC288CAB2L, 0xD1D83946L, 0x23B3BA45L,
  0xF779DEAEL, 0x05125DADL, 0x1642AE59L, 0xE4292D5AL,
  0xBA3A117EL, 0x4851927DL, 0x5B016189L, 0xA96AE28AL,
  0x7DA08661L, 0x8FCB0562L, 0x9C9BF696L, 0x6EF07595L,
  0x417B1DBCL, 0xB3109EBFL, 0xA0406D4BL, 0x522BEE48L,
  0x86E18AA3L, 0x748A09A0L, 0x67DAFA54L, 0x95B17957L,
  0xCBA24573L, 0x39C9C670L, 0x2A993584L, 0xD8F2B687L,
  0x0C38D26CL, 0xFE53516FL, 0xED03A29BL, 0x1F682198L,
  0x5125DAD3L, 0xA34E59D0L, 0xB01EAA24L, 0x42752927L,
  0x96BF4DCCL, 0x64D4CECFL, 0x77843D3BL, 0x85EFBE38L,
  0xDBFC821CL, 0x2997011FL, 0x3AC7F2EBL, 0xC8AC71E8L,
  0x1C661503L, 0xEE0D9600L, 0xFD5D65F4L, 0x0F36E6F7L,
  0x61C69362L, 0x93AD1061L, 0x80FDE395L, 0x72966096L,
  0xA65C047DL, 0x5437877EL, 0x4767748AL, 0xB50CF789L,
  0xEB1FCBADL, 0x197448AEL, 0x0A24BB5AL, 0xF84F3859L,
  0x2C855CB2L, 0xDEEEDFB1L, 0xCDBE2C45L, 0x3FD5AF46L,
  0x7198540DL, 0x83F3D70EL, 0x90A324FAL, 0x62C8A7F9L,
  0xB602C312L, 0x44694011L, 0x5739B3E5L, 0xA55230E6L,
  0xFB410CC2L, 0x092A8FC1L, 0x1A7A7C35L, 0xE811FF36L,
  0x3CDB9BDDL, 0xCEB018DEL, 0xDDE0EB2AL, 0x2F8B6829L,
  0x82F63B78L, 0x709DB87BL, 0x63CD4B8FL, 0x91A6C88CL,
  0x456CAC67L, 0xB7072F64L, 0xA457DC90L, 0x563C5F93L,
  0x082F63B7L, 0xFA44E0B4L, 0xE9141340L, 0x1B7F9043L,
  0xCFB5F4A8L, 0x3DDE77ABL, 0x2E8E845FL, 0xDCE5075CL,
  0x92A8FC17L, 0x60C37F14L, 0x73938CE0L, 0x81F80FE3L,
  0x55326B08L, 0xA759E80BL, 0xB4091BFFL, 0x466298FCL,
  0x1871A4D8L, 0xEA1A27DBL, 0xF94AD42FL, 0x0B21572CL,
  0xDFEB33C7L, 0x2D80B0C4L, 0x3ED04330L, 0xCCBBC033L,
  0xA24BB5A6L, 0x502036A5L, 0x4370C551L, 0xB11B4652L,
  0x65D122B9L, 0x97BAA1BAL, 0x84EA524EL, 0x7681D14DL,
  0x2892ED69L, 0xDAF96E6AL, 0xC9A99D9EL, 0x3BC21E9DL,
  0xEF087A76L, 0x1D63F975L, 0x0E330A81L, 0xFC588982L,
  0xB21572C9L, 0x407EF1CAL, 0x532E023EL, 0xA145813DL,
  0x758FE5D6L, 0x87E466D5L, 0x94B49521L, 0x66DF1622L,
  0x38CC2A06L, 0xCAA7A905L, 0xD9F75AF1L, 0x2B9CD9F2L,
  0xFF56BD19L, 0x0D3D3E1AL, 0x1E6DCDEEL, 0xEC064EEDL,
  0xC38D26C4L, 0x31E6A5C7L, 0x22B65633L, 0xD0DDD530L,
  0x0417B1DBL, 0xF67C32D8L, 0xE52CC12CL, 0x1747422FL,
  0x49547E0BL, 0xBB3FFD08L, 0xA86F0EFCL, 0x5A048DFFL,
  0x8ECEE914L, 0x7CA56A17L, 0x6FF599E3L, 0x9D9E1AE0L,
  0xD3D3E1ABL, 0x21B862A8L, 0x32E8915CL, 0xC083125FL,
  0x144976B4L, 0xE622F5B7L, 0xF5720643L, 0x07198540L,
  0x590AB964L, 0xAB613A67L, 0xB831C993L, 0x4A5A4A90L,
  0x9E902E7BL, 0x6CFBAD78L, 0x7FAB5E8CL, 0x8DC0DD8FL,
  0xE330A81AL, 0x115B2B19L, 0x020BD8EDL, 0xF0605BEEL,
  0x24AA3F05L, 0xD6C1BC06L, 0xC5914FF2L, 0x37FACCF1L,
  0x69E9F0D5L, 0x9B8273D6L, 0x88D28022L, 0x7AB90321L,
  0xAE7367CAL, 0x5C18E4C9L, 0x4F48173DL, 0xBD23943EL,
  0xF36E6F75L, 0x0105EC76L, 0x12551F82L, 0xE03E9C81L,
  0x34F4F86AL, 0xC69F7B69L, 0xD5CF889DL, 0x27A40B9EL,
  0x79B737BAL, 0x8BDCB4B9L, 0x988C474DL, 0x6AE7C44EL,
  0xBE2DA0A5L, 0x4C4623A6L, 0x5F16D052L, 0xAD7D5351L
};

#if FBHT
static uint16_t const crc16Table[256] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t ENC_IBC_HashMap::xxComputeCrc16(uint32_t crc, const pel pel)
{
    const void* buf = &pel;
    const uint8_t* p = (const uint8_t*)buf;
    size_t size = 2;

    while (size--)
    {
        crc = crc16Table[(crc ^ *p++) & 0xff] ^ (crc >> 8);
    }

    return crc;
}
#endif

uint32_t ENC_IBC_HashMap::xxComputeCrc32c16bit(uint32_t crc, const pel pel)
{
    const void *buf = &pel;
    const uint8_t *p = (const uint8_t *)buf;
    size_t size = 2;

    while (size--)
    {
        crc = crc32Table[(crc ^ *p++) & 0xff] ^ (crc >> 8);
    }

    return crc;
}
#if ISC_RSD && !RSD_OPT
uint32_t ENC_RSD_HashMap::xxComputeCrc32c16bitRsd(uint32_t crc, const pel pel)
{
    const void* buf = &pel;
    const uint16_t* p = (const uint16_t*)buf;
    crc = crc32Table[(crc ^ *p) & 0xff] ^ (crc >> 8);
    return crc;
}
#endif
// CRC calculation in C code
////////////////////////////////////////////////////////

unsigned int ENC_IBC_HashMap::xxCalcBlockHash(const pel* pel, const int stride, const int width, const int height, unsigned int crc)
{
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            crc = m_computeCrc32c(crc, pel[x]);
        }
        pel += stride;
    }
    return crc;
}

void ENC_IBC_HashMap::xxBuildPicHashMap(const COM_PIC* pic)
{
    int y_stride = 0;
    int c_stride = 0;
    int width = 0;
    int height = 0;
    const int chromaScalingX = 1;
    const int chromaScalingY = 1;
    const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
    const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
    const pel* pelY = NULL;
    const pel* pelCb = NULL;
    const pel* pelCr = NULL;

    width = pic->width_luma;
    height = pic->height_luma;

    y_stride = pic->stride_luma;
    c_stride = pic->stride_chroma;

    Position pos;
    for (pos.second = 0; pos.second + MIN_PU_SIZE <= height; pos.second++)
    {
        // row pointer
        pelY = pic->y + pos.second * y_stride;
        int chromaY = (pos.second + 1) >> chromaScalingY;
        pelCb = pic->u + chromaY * c_stride + ((pos.first + 1) >> chromaScalingY);
        pelCr = pic->v + chromaY * c_stride + ((pos.first + 1) >> chromaScalingY);
        for (pos.first = 0; pos.first + MIN_PU_SIZE <= width; pos.first++)
        {
            // 0x1FF is just an initial value
            unsigned int hashValue = 0x1FF;

            // luma part
            hashValue = xxCalcBlockHash(&pelY[pos.first], y_stride, MIN_PU_SIZE, MIN_PU_SIZE, hashValue);

            // chroma part
            hashValue = xxCalcBlockHash(&pelCb[0], c_stride, chromaMinBlkWidth, chromaMinBlkHeight, hashValue);
            hashValue = xxCalcBlockHash(&pelCr[0], c_stride, chromaMinBlkWidth, chromaMinBlkHeight, hashValue);
            // hash table
            m_hash2Pos[hashValue].push_back(pos);
            m_pos2Hash[pos.second][pos.first] = hashValue;
        }
    }
}

#if FBHT
void ENC_IBC_HashMap::xxBuildPicFbhtHashMap(const COM_PIC* pic)
{
    int y_stride = 0;
    int c_stride = 0;
    int width = 0;
    int height = 0;
    const int chromaScalingX = 1;
    const int chromaScalingY = 1;
    const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
    const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
    const pel* pelY = NULL;
    const pel* pelCb = NULL;

    width = pic->width_luma;
    height = pic->height_luma;

    y_stride = pic->stride_luma;
    c_stride = pic->stride_chroma;

    Position pos;
    for (pos.second = 0; pos.second + MIN_PU_SIZE <= height; pos.second++)
    {
        for (pos.first = 0; pos.first + MIN_PU_SIZE <= width; pos.first++)
        {
            // 0x1FF is just an initial value
            unsigned int hashValue = 0x1FF;

            hashValue = xxBuildPicFbhtHashMapByPos(pic, pos.first, pos.second);

            m_pos2Hash[pos.second][pos.first] = hashValue;

            auto x = m_hash2Pos.find(hashValue);
            if (x != m_hash2Pos.end())
            {
                auto v = x->second.end();
                v--;
                if (abs(pos.first - v->first) + abs(pos.second - v->second) < 4)
                    continue;
            }

            m_hash2Pos[hashValue].push_back(pos);
        }
    }
}

unsigned int ENC_IBC_HashMap::xxBuildPicFbhtHashMapByPos(const COM_PIC* pic, int posx, int posy) {
    int y_stride = 0;
    int c_stride = 0;
    int width = 0;
    int height = 0;
    const int chromaScalingX = 1;
    const int chromaScalingY = 1;
    const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
    const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
    const pel* pelY = NULL;
    const pel* pelCb = NULL;
    const pel* pelCr = NULL;
    const pel* plane = NULL;

    width = pic->width_luma;
    height = pic->height_luma;

    y_stride = pic->stride_luma;
    c_stride = pic->stride_chroma;

    const int iBitdepth = 10;

    Position pos;
    Position posB;
    pos.first = posx;
    pos.second = posy;
    assert(pos.second + MIN_PU_SIZE <= height && pos.first + MIN_PU_SIZE <= width);

    // row pointer
    pelY = pic->y + pos.second * y_stride + pos.first;

    int chromaY = (pos.second + 1) >> chromaScalingY;
    pelCb = pic->u + chromaY * c_stride + ((pos.first + 1) >> chromaScalingY);
    pelCr = pic->v + chromaY * c_stride + ((pos.first + 1) >> chromaScalingY);

    unsigned int grad = 0;
    unsigned int avgDC = 0;
    int maxDC = 0;
    int minDC = 255;
    unsigned int gradX = 0;
    unsigned int gradY = 0;
    unsigned int totalGradX = 0;
    unsigned int totalGradY = 0;
    unsigned int gradXsp = 0;
    unsigned int gradYsp = 0;
    unsigned int totalColor = 0;
    unsigned int evenness = 0;
    unsigned int uiHashIdx = 0x1FF;

    unsigned int iTotalSamples = MIN_PU_SIZE * MIN_PU_SIZE;

    //Y 
    {
        int cxstride = y_stride;

        int cxwidth = MIN_PU_SIZE;
        int cxheight = MIN_PU_SIZE;

        plane = pelY;
        std::unordered_map<unsigned int, std::vector<Position>> colorIndex;

        for (int y = 0; y < cxheight; y++) {
            for (int x = 0; x < cxwidth; x++) {
                avgDC += (plane[y * cxstride + x] >> (iBitdepth - 8));
                posB.first = x;
                posB.second = y;
                colorIndex[(plane[y * cxstride + x] >> (iBitdepth - 8))].push_back(posB);
                if ((plane[y * cxstride + x] >> (iBitdepth - 8)) > maxDC) {
                    maxDC = (plane[y * cxstride + x] >> (iBitdepth - 8));
                }
                if ((plane[y * cxstride + x] >> (iBitdepth - 8)) < minDC) {
                    minDC = (plane[y * cxstride + x] >> (iBitdepth - 8));
                }
            }
        }
        totalColor = (unsigned int)(colorIndex.size());

        for (int y = 0; y < cxheight - 1; y++) {
            for (int x = 0; x < cxwidth - 1; x++) {
                gradX = abs(plane[y * cxstride + x] - plane[y * cxstride + x + 1]) >> (iBitdepth - 8);
                if (gradX > 64) {
                    gradXsp++;
                }
                gradY = abs(plane[y * cxstride + x] - plane[(y + 1) * cxstride + x]) >> (iBitdepth - 8);
                if (gradY > 64) {
                    gradYsp++;
                }
                totalGradX += gradX;
                totalGradY += gradY;
            }
        }
    }

    // pelCb
    {
        int cxstride = c_stride;

        int cxwidth = chromaMinBlkWidth;
        int cxheight = chromaMinBlkHeight;

        plane = pelCb;

        for (int y = 0; y < cxheight; y++) {
            for (int x = 0; x < cxwidth; x++) {
                avgDC += (plane[y * cxstride + x] >> (iBitdepth - 8));
            }
        }
        for (int y = 0; y < cxheight - 1; y++) {
            for (int x = 0; x < cxwidth - 1; x++) {
                gradX = abs(plane[y * cxstride + x] - plane[y * cxstride + x + 1]) >> (iBitdepth - 8);
                if (gradX > 64) {
                    gradXsp++;
                }
                gradY = abs(plane[y * cxstride + x] - plane[(y + 1) * cxstride + x]) >> (iBitdepth - 8);
                if (gradY > 64) {
                    gradYsp++;
                }
                totalGradX += gradX;
                totalGradY += gradY;
            }
        }
    }

    // pelCr
    {
        int cxstride = c_stride;

        int cxwidth = chromaMinBlkWidth;
        int cxheight = chromaMinBlkHeight;

        plane = pelCr;

        for (int y = 0; y < cxheight; y++) {
            for (int x = 0; x < cxwidth; x++) {
                avgDC += (plane[y * cxstride + x] >> (iBitdepth - 8));
            }
        }
        for (int y = 0; y < cxheight - 1; y++) {
            for (int x = 0; x < cxwidth - 1; x++) {
                gradX = abs(plane[y * cxstride + x] - plane[y * cxstride + x + 1]) >> (iBitdepth - 8);
                if (gradX > 64) {
                    gradXsp++;
                }
                gradY = abs(plane[y * cxstride + x] - plane[(y + 1) * cxstride + x]) >> (iBitdepth - 8);
                if (gradY > 64) {
                    gradYsp++;
                }
                totalGradX += gradX;
                totalGradY += gradY;
            }
        }
    }

    avgDC = (avgDC) / (iTotalSamples);
    evenness = (((maxDC - minDC) << 3) / avgDC) & 0xFF;
    totalColor = (totalColor >> 1) & 0xFF;
    gradX = ((totalGradX / (iTotalSamples)) >> 3) & 0xFF;
    gradY = ((totalGradY / (iTotalSamples)) >> 3) & 0xFF;
    avgDC = (avgDC >> 5) & 0xFF;

    uiHashIdx = xxComputeCrc16(uiHashIdx, avgDC);
    uiHashIdx = xxComputeCrc16(uiHashIdx, gradY);
    uiHashIdx = xxComputeCrc16(uiHashIdx, gradX);
    uiHashIdx = xxComputeCrc16(uiHashIdx, totalColor);
    uiHashIdx = xxComputeCrc16(uiHashIdx, evenness);

    return uiHashIdx;
}

double ENC_IBC_HashMap::xxHashMapStat() {
    std::vector<std::pair<unsigned int, unsigned int>> tmp;
    unsigned int tt = 0;
    unsigned int total_1p = 0;
    unsigned int total_2p = 0;
    unsigned int total_3p = 0;
    auto x = m_hash2Pos.begin();
    unsigned int hashLinkNo = 0;
    while (x != m_hash2Pos.end()) {
        std::pair<unsigned int, unsigned int> p1(x->first, (unsigned int)x->second.size());
        tmp.push_back(p1);
        tt += (unsigned int)x->second.size();
        ++x;
        hashLinkNo++;
    }
    std::sort(tmp.begin(), tmp.end(),
        [=](std::pair<unsigned int, unsigned int>& a, std::pair<unsigned int, unsigned int>& b) { return a.second > b.second; });

    for (int i = 0; i < ((int)tmp.size() / 100); i++)
    {
        total_1p += tmp[i].second;
    }

    for (int i = 0; i < ((int)tmp.size() / 50); i++)
    {
        total_2p += tmp[i].second;
    }

    for (int i = 0; i < ((int)tmp.size() / 33); i++)
    {
        total_3p += tmp[i].second;
    }

    double ratio_1p = total_1p * 1.0 / tt;
    double ratio_2p = total_2p * 1.0 / tt;
    double ratio_3p = total_3p * 1.0 / tt;
    double ratio_aver = (ratio_1p + ratio_2p + ratio_3p) / 3;
    // hashEv represents the distribution evenness of the hash chains. It is a parameter for the auto-select algorithm.
    double hashEv = ((ratio_aver - ratio_1p) * (ratio_aver - ratio_1p) + (ratio_aver - ratio_2p) * (ratio_aver - ratio_2p) + (ratio_aver - ratio_3p) * (ratio_aver - ratio_3p)) / 2 * hashLinkNo;
    return hashEv;
}
#endif

#if FBHT
void ENC_IBC_HashMap::rebuildPicHashMap(const COM_PIC* pic, ENC_CTX* ctx)
#else
void ENC_IBC_HashMap::rebuildPicHashMap(const COM_PIC* pic)
#endif
{
    m_hash2Pos.clear();

#if FBHT
    if (ctx->param.ibc_feature_based_hash_table)
    {
        xxBuildPicFbhtHashMap(pic);
        double hashEV = xxHashMapStat();
        if (hashEV > 20 && hashEV < 40)
        {
            m_hash2Pos.clear();
            xxBuildPicHashMap(pic);
        }
    }
    else
    {
        xxBuildPicHashMap(pic);
    }
#else
    xxBuildPicHashMap(pic);
#endif
}

bool ENC_IBC_HashMap::ibcHashMatch(ENC_CTX *ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    std::vector<Position>& cand, const int maxCand, const int searchRange4SmallBlk)
{
    int cuw = (1 << log2_cuw);
    int cuh = (1 << log2_cuh);

    cand.clear();

    // find the block with least candidates
    size_t minSize = COM_UINT32_MAX;
    unsigned int targetHashOneBlock = 0;
    targetHashOneBlock = m_pos2Hash[cu_y][cu_x];

    if (m_hash2Pos[targetHashOneBlock].size() > 1)
    {
        std::vector<Position>& candOneBlock = m_hash2Pos[targetHashOneBlock];

        // check whether whole block match
        for (std::vector<Position>::iterator refBlockPos = candOneBlock.begin(); refBlockPos != candOneBlock.end(); refBlockPos++)
        {
            int offset_BR_x = refBlockPos->first + cuw - 1;
            int offset_BR_y = refBlockPos->second + cuh - 1;
            u32 offset_x_scu = PEL2SCU(offset_BR_x);
            u32 offset_y_scu = PEL2SCU(offset_BR_y);
            u32 offset_scup = ((u32)offset_y_scu * ctx->info.pic_width_in_scu) + offset_x_scu;


            int avail_cu = 0;
            if (offset_BR_x < m_picWidth && offset_BR_y < m_picHeight)
            {
                avail_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[offset_scup]);
            }


            bool wholeBlockMatch = true;
            if (cuw > MIN_PU_SIZE || cuh > MIN_PU_SIZE)
            {
                if (!avail_cu || offset_BR_x >= m_picWidth || offset_BR_y >= m_picHeight)
                {
                    continue;
                }
                for (int y = 0; y < cuh && wholeBlockMatch; y += MIN_PU_SIZE)
                {
                    for (int x = 0; x < cuw && wholeBlockMatch; x += MIN_PU_SIZE)
                    {
                        // whether the reference block and current block has the same hash
                        wholeBlockMatch &= (m_pos2Hash[cu_y + y][cu_x + x] == m_pos2Hash[refBlockPos->second + y][refBlockPos->first + x]);
                    }
                }
            }
            else
            {
                if (abs(refBlockPos->first - cu_x) > searchRange4SmallBlk || abs(refBlockPos->second - cu_y) > searchRange4SmallBlk || !avail_cu)
                {
                    continue;
                }
            }
            if (wholeBlockMatch)
            {
                cand.push_back(*refBlockPos);
                if (cand.size() > maxCand)
                {
                    break;
                }
            }
        }
    }

    return cand.size() > 0;
}

#if USE_IBC
u8 get_hash_hit_ratio_pic(ENC_CTX *ctx, ibc_hash_handle* p
#if TEMPORAL_FILTER_SCC
    , int thr
#endif
)
{
    int pic_width = ctx->info.pic_width;
    int pic_height = ctx->info.pic_height;
    ENC_IBC_HashMap* p_hash = (ENC_IBC_HashMap*)p;
    int hit = 0, total = 0;

    for (int y = 0; y < pic_height; y += MIN_PU_SIZE)
    {
        for (int x = 0; x < pic_width; x += MIN_PU_SIZE)
        {
            const unsigned int hash = p_hash->m_pos2Hash[y][x];
            hit += (p_hash->m_hash2Pos[hash].size() > 1);
            total++;
        }
    }
    assert(total > 0);
#if TEMPORAL_FILTER_SCC
    int hash_hit_ratio = 1000 * hit / total;
#else
    int hash_hit_ratio = 100 * hit / total;
#endif
#if FIMC || ISTS ||TS_INTER
#if TEMPORAL_FILTER_SCC
    if (hash_hit_ratio < thr)
#else
    if (hash_hit_ratio < 6) // 6%
#endif
#else
    if (hash_hit_ratio < 5) // 5%
#endif
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
#endif

#if ISC_RSD && !RSD_OPT
static void rsd_match_hash(int pu_w, int pu_h, void* dst, void* src, int s_src, u8 rsd_dir)
{
    s16* s1 = (s16*)dst;//org_temp
    s16* s2 = (s16*)src;//org
    if (rsd_dir) // ver
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

unsigned int ENC_RSD_HashMap::xxCalcBlockHashRsd(const pel* pel, const int stride, const int width, const int height, unsigned int crc)
{
    int matrix[4][4] = { 0 };
    int P2x2[12] = { 0 };
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            matrix[y][x] = pel[x];
        }
        pel += stride;
    }

    P2x2[0] = matrix[0][0] + matrix[0][1] + matrix[1][0] + matrix[1][1];
    P2x2[1] = matrix[0][2] + matrix[0][3] + matrix[1][2] + matrix[1][3];
    P2x2[2] = matrix[2][0] + matrix[2][1] + matrix[3][0] + matrix[3][1];
    P2x2[3] = matrix[2][2] + matrix[2][3] + matrix[3][2] + matrix[3][3];
    P2x2[4] = matrix[0][0] + matrix[0][1] + matrix[0][2] + matrix[0][3];
    P2x2[5] = matrix[1][0] + matrix[1][1] + matrix[1][2] + matrix[1][3];
    P2x2[6] = matrix[2][0] + matrix[2][1] + matrix[2][2] + matrix[2][3];
    P2x2[7] = matrix[3][0] + matrix[3][1] + matrix[3][2] + matrix[3][3];
    P2x2[8] = matrix[0][0] + matrix[1][0] + matrix[2][0] + matrix[3][0];
    P2x2[9] = matrix[0][1] + matrix[1][1] + matrix[2][1] + matrix[3][1];
    P2x2[10] = matrix[0][2] + matrix[1][2] + matrix[2][2] + matrix[3][2];
    P2x2[11] = matrix[0][3] + matrix[1][3] + matrix[2][3] + matrix[3][3];

    for (int i = 0; i < 12; i++)
    {
        crc = m_computeCrc32cRsd(crc, P2x2[i] >> 9);
    }
    return crc;
}

void ENC_RSD_HashMap::xxBuildPicHashMapRsd(const COM_PIC* pic)
{
    int y_stride = 0;
    int c_stride = 0;
    int width = 0;
    int height = 0;
    const int chromaScalingX = 1;
    const int chromaScalingY = 1;
    const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
    const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
    const pel* pelY = NULL;
    const pel* pelCb = NULL;
    const pel* pelCr = NULL;

    width = pic->width_luma;
    height = pic->height_luma;

    y_stride = pic->stride_luma;
    c_stride = pic->stride_chroma;

    PositionRsd pos;
    for (pos.second = 0; pos.second + MIN_PU_SIZE <= height; pos.second++)
    {
        // row pointer
        pelY = pic->y + pos.second * y_stride;
        for (pos.first = 0; pos.first + MIN_PU_SIZE <= width; pos.first++)
        {
            // 0x1FF is just an initial value
            unsigned int hashValue = 0x1FF;

            // luma part
            hashValue = xxCalcBlockHashRsd(&pelY[pos.first], y_stride, MIN_PU_SIZE, MIN_PU_SIZE, hashValue);
            // hash table
            m_hash2PosRsd[hashValue].push_back(pos);
            m_pos2HashRsd[pos.second][pos.first] = hashValue;
        }
    }
}

void ENC_RSD_HashMap::rebuildPicHashMapRsd(const COM_PIC* pic)
{
    m_hash2PosRsd.clear();

    xxBuildPicHashMapRsd(pic);
}

void ENC_RSD_HashMap::buildFlipHashTable(int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    unsigned int pos2HashCurPU[MAX_CU_SIZE][MAX_CU_SIZE], const int intra_row_rsd_flag, const COM_PIC* pic)
{

    int y_stride = pic->stride_luma;
    int c_stride = pic->stride_chroma;

    int roi_width = (1 << log2_cuw);
    int roi_height = (1 << log2_cuh);
    pel* org_y = pic->y + cu_y * y_stride + cu_x;

    for (int i = 0; i < MAX_CU_SIZE; i++)
    {
        for (int j = 0; j < MAX_CU_SIZE; j++)
        {
            pos2HashCurPU[i][j] = 0;
        }
    }
    static pel org_tmp_Y[MAX_CU_DIM];

    rsd_match_hash(roi_width, roi_height, org_tmp_Y, org_y, y_stride, intra_row_rsd_flag);

    const int chromaScalingX = 1;
    const int chromaScalingY = 1;// 420
    const int chromaMinBlkWidth = MIN_PU_SIZE >> chromaScalingX;
    const int chromaMinBlkHeight = MIN_PU_SIZE >> chromaScalingY;
    const pel* pelY = NULL;

    PositionRsd pos;
    int hei = roi_height;
    for (pos.second = 0; pos.second < roi_height; pos.second += MIN_PU_SIZE)
    {
        // row pointer
        pelY = org_tmp_Y + pos.second * roi_width;

        for (pos.first = 0; pos.first < roi_width; pos.first += MIN_PU_SIZE)
        {
            unsigned int hashValue = 0x1FF;

            // luma part
            hashValue = xxCalcBlockHashRsd(&pelY[pos.first], roi_width, MIN_PU_SIZE, MIN_PU_SIZE, hashValue);
            pos2HashCurPU[pos.second][pos.first] = hashValue;
        }
    }
}

bool ENC_RSD_HashMap::rsdHashMatch(ENC_CTX* ctx, int cu_x, int cu_y, int log2_cuw, int log2_cuh,
    std::vector<PositionRsd>& cand, const int maxCand, const int searchRange4SmallBlk, const int intra_row_rsd_flag, const COM_PIC* pi)
{
    unsigned int pos2HashCurPU[MAX_CU_SIZE][MAX_CU_SIZE];
    buildFlipHashTable(cu_x, cu_y, log2_cuw, log2_cuh, pos2HashCurPU, intra_row_rsd_flag, pi);
    int cuw = (1 << log2_cuw);
    int cuh = (1 << log2_cuh);

    cand.clear();

    // find the block with least candidates
    size_t minSize = COM_UINT32_MAX;
    unsigned int targetHashOneBlock = 0;
    unsigned int hash = 0;
    PositionRsd targetBlockOffsetInCu(0, 0);
    for (int y = 0; y < cuh && minSize > 1; y += MIN_PU_SIZE)
    {
        for (int x = 0; x < cuw && minSize > 1; x += MIN_PU_SIZE)
        {
            hash = pos2HashCurPU[y][x];
            if (m_hash2PosRsd[hash].size() < minSize)
            {
                minSize = m_hash2PosRsd[hash].size();
                targetHashOneBlock = hash;
                targetBlockOffsetInCu.first = x;
                targetBlockOffsetInCu.second = y;
            }
        }
    }

    if (m_hash2PosRsd[targetHashOneBlock].size() > 0)
    {
        std::vector<PositionRsd>& candOneBlock = m_hash2PosRsd[targetHashOneBlock];

        // check whether whole block match
        for (std::vector<PositionRsd>::iterator refBlockPos = candOneBlock.begin(); refBlockPos != candOneBlock.end(); refBlockPos++)
        {
            PositionRsd offset_TL;
            offset_TL.first = refBlockPos->first - targetBlockOffsetInCu.first;
            offset_TL.second = refBlockPos->second - targetBlockOffsetInCu.second;
            int offset_BR_x = offset_TL.first + cuw - 1;
            int offset_BR_y = offset_TL.second + cuh - 1;
            u32 offset_x_scu = PEL2SCU(offset_BR_x);
            u32 offset_y_scu = PEL2SCU(offset_BR_y);
            u32 offset_scup = ((u32)offset_y_scu * ctx->info.pic_width_in_scu) + offset_x_scu;


            int avail_cu = 0;
            if (offset_BR_x < m_picWidth && offset_BR_y < m_picHeight)
            {
                avail_cu = MCU_GET_CODED_FLAG(ctx->map.map_scu[offset_scup]);
            }

            bool wholeBlockMatch = true;
            if (cuw > MIN_PU_SIZE || cuh > MIN_PU_SIZE)
            {
                if (!avail_cu || offset_BR_x >= m_picWidth || offset_BR_y >= m_picHeight || offset_TL.first < 0 || offset_TL.second < 0)
                {
                    continue;
                }
                for (int y = 0; y < cuh && wholeBlockMatch; y += MIN_PU_SIZE)
                {
                    for (int x = 0; x < cuw && wholeBlockMatch; x += MIN_PU_SIZE)
                    {
                        // whether the reference block and current block has the same hash
                        wholeBlockMatch &= (pos2HashCurPU[y][x] == m_pos2HashRsd[offset_TL.second + y][offset_TL.first + x]);
                    }
                }
            }
            else
            {
                if (abs(offset_TL.first - cu_x) > searchRange4SmallBlk || abs(offset_TL.second - cu_y) > searchRange4SmallBlk || !avail_cu)
                {
                    continue;
                }
            }
            if (wholeBlockMatch)
            {
                cand.push_back(offset_TL);
                if (cand.size() > maxCand)
                {
                    break;
                }
            }
        }
    }

    return cand.size() > 0;
}
#endif
#endif