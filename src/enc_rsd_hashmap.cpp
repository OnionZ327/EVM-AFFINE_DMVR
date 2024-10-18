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


#include "enc_rsd_hashmap.h"
#include "enc_pibc.h"
#include "enc_rsd_hash_wrapper.h"

#define MIN_PU_SIZE 4
#if RSD_OPT
#ifdef __cplusplus
extern "C" {
#endif
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

    u32 search_rsd_hash_match(ENC_CORE* core, ENC_CTX* ctx, rsd_hash_handle* p, int cu_x, int cu_y,int log2_cuw, int log2_cuh, s16 mv[MV_D])
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
                    //int mv_bits = get_sp_offset_bit_cnt(cand_mv[MV_X], cand_mv[MV_Y]);
                    int mv_bits = get_bv_cost_bits(cand_mv[MV_X], cand_mv[MV_Y]);
                    cost = GET_MV_COST(ctx, mv_bits);
                    if (cost < min_cost)
                    {
                        mv[0] = cand_mv[0];
                        mv[1] = cand_mv[1];
                        min_cost = cost;
                    }
                }
            }
        }
        return min_cost;
    }

#if FAST_SCC_PRECODING
    u32 search_rsd_hash_matchrevisit(ENC_CORE* core, ENC_CTX* ctx, rsd_hash_handle* p, int cu_x, int cu_y, int log2_cuw, int log2_cuh, s16 mv[MV_D])
    {
        u32 cost = 0;
        u32 min_cost = ((u32)0xFFFFFFFF);
        ENC_PIBC* pi = &ctx->pibc;

        u8 bvp_idx_temp = 0;
        s16 mv_temp[MV_D] = { 0, 0 };
        mv_temp[0] = core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0rsdmv[MV_X];
        mv_temp[1] = core->bef_data[log2_cuw - 2][log2_cuh - 2][core->cup].ibcvisit0rsdmv[MV_Y];
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
            //int mv_bits = get_sp_offset_bit_cnt(cand_mv[MV_X], cand_mv[MV_Y]);
            int mv_bits = get_bv_cost_bits(cand_mv[MV_X], cand_mv[MV_Y]);
            cost = GET_MV_COST(ctx, mv_bits);
            if (cost < min_cost)
            {
                mv[0] = cand_mv[0];
                mv[1] = cand_mv[1];
                min_cost = cost;
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

uint32_t ENC_RSD_HashMap::xxComputeCrc32c16bitRsd(uint32_t crc, const pel pel)
{
    const void* buf = &pel;
    const uint16_t* p = (const uint16_t*)buf;
    crc = crc32Table[(crc ^ *p) & 0xff] ^ (crc >> 8);
    return crc;
}
// CRC calculation in C code
////////////////////////////////////////////////////////

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
    std::vector<PositionRsd>& cand, const int maxCand, const int searchRange4SmallBlk,const int intra_row_rsd_flag, const COM_PIC* pi)
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
