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

#include "com_tbl.h"
#include "com_mc.h"
#include "com_util.h"
#include <assert.h>
#include "math.h"

#if IF_LUMA12_CHROMA6
#define MAC_SFT_N0            (8)
#define MAC_ADD_N0            (1<<7)
#define MAC_SFT_0N            MAC_SFT_N0
#define MAC_ADD_0N            MAC_ADD_N0
#else
#define MAC_SFT_N0            (6)
#define MAC_ADD_N0            (1<<5)

#define MAC_SFT_0N            MAC_SFT_N0
#define MAC_ADD_0N            MAC_ADD_N0
#endif

#define BIO_IF_S1(BIT_DEPTH)  (2)
#define BIO_IF_S2(BIT_DEPTH)  (14)
#define BIO_IF_O1(BIT_DEPTH)  (1 << (BIO_IF_S1(BIT_DEPTH) - 1))
#define BIO_IF_O2(BIT_DEPTH)  (1 << (BIO_IF_S2(BIT_DEPTH) - 1))

#if IF_LUMA12_CHROMA6
#define MAC_12TAP(c, r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3)+(c)[4]*(r4)+\
    (c)[5]*(r5)+(c)[6]*(r6)+(c)[7]*(r7)+(c)[8]*(r8)+(c)[9]*(r9)+(c)[10]*(r10)+(c)[11]*(r11))
#define MAC_6TAP(c, r0, r1, r2, r3, r4, r5) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3)+(c)[4]*(r4)+(c)[5]*(r5))
#endif

#define MAC_8TAP(c, r0, r1, r2, r3, r4, r5, r6, r7) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3)+(c)[4]*(r4)+\
    (c)[5]*(r5)+(c)[6]*(r6)+(c)[7]*(r7))

#define MAC_4TAP(c, r0, r1, r2, r3) \
    ((c)[0]*(r0)+(c)[1]*(r1)+(c)[2]*(r2)+(c)[3]*(r3))

/* padding for store intermediate values, which should be larger than
1+ half of filter tap */
#if IF_LUMA12_CHROMA6
#define MC_IBUF_PAD_C          6
#define MC_IBUF_PAD_L          12
#else
#define MC_IBUF_PAD_C          4
#define MC_IBUF_PAD_L          8
#endif

#if SIMD_MC
static const s16 tbl_bl_mc_l_coeff[4][2] =
#else
static const s8 tbl_bl_mc_l_coeff[4][2] =
#endif
{
    { 64,  0 },
    { 48, 16 },
    { 32, 32 },
    { 16, 48 },
};


/****************************************************************************
 * motion compensation for luma
 ****************************************************************************/

#if SIMD_MC
static const s8 shuffle_2Tap[16] = {0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9};

void mc_filter_bilin_horz_sse
(
    s16 const *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const short *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h;
    int src_stride2, src_stride3;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i row1, row11, row2, row22, row3, row33, row4, row44;
    __m128i res0, res1, res2, res3;
    __m128i coeff0_1_8x16b, shuffle;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadl_epi64((__m128i*)coeff);      /*w0 w1 x x x x x x*/
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);  /*w0 w1 w0 w1 w0 w1 w0 w1*/
    shuffle = _mm_loadu_si128((__m128i*)shuffle_2Tap);
    rem_h = (height & 0x3);
    if(rem_w > 7)
    {
        for(row = height; row > 3; row -= 4)
        {
            int cnt = 0;
            for(col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                             /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));                        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));       /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row22 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 1));  /*b1 b2 b3 b4 b5 b6 b7 b8*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                row33 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 1));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                row44 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 1));
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);            /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);          /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
                row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
                row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
                row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row2 = _mm_add_epi32(row2, offset_4x32b);
                row22 = _mm_add_epi32(row22, offset_4x32b);
                row3 = _mm_add_epi32(row3, offset_4x32b);
                row33 = _mm_add_epi32(row33, offset_4x32b);
                row4 = _mm_add_epi32(row4, offset_4x32b);
                row44 = _mm_add_epi32(row44, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                row2 = _mm_srai_epi32(row2, shift);
                row22 = _mm_srai_epi32(row22, shift);
                row3 = _mm_srai_epi32(row3, shift);
                row33 = _mm_srai_epi32(row33, shift);
                row4 = _mm_srai_epi32(row4, shift);
                row44 = _mm_srai_epi32(row44, shift);
                row1 = _mm_packs_epi32(row1, row2);
                row11 = _mm_packs_epi32(row11, row22);
                row3 = _mm_packs_epi32(row3, row4);
                row33 = _mm_packs_epi32(row33, row44);
                res0 = _mm_unpacklo_epi16(row1, row11);
                res1 = _mm_unpackhi_epi16(row1, row11);
                res2 = _mm_unpacklo_epi16(row3, row33);
                res3 = _mm_unpackhi_epi16(row3, row33);
                if(is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res1 = _mm_min_epi16(res1, mm_max);
                    res2 = _mm_min_epi16(res2, mm_max);
                    res3 = _mm_min_epi16(res3, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                    res1 = _mm_max_epi16(res1, mm_min);
                    res2 = _mm_max_epi16(res2, mm_min);
                    res3 = _mm_max_epi16(res3, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 2 + cnt), res2);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 3 + cnt), res3);
                cnt += 8; /* To pointer updates*/
            }
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        /*extra height to be done --- one row at a time*/
        for(row = 0; row < rem_h; row++)
        {
            int cnt = 0;
            for(col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));       /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));  /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);              /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);            /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                row1 = _mm_packs_epi32(row1, row11);    /*a0 a2 a4 a6 a1 a3 a5 a7*/
                res0 = _mm_unpackhi_epi64(row1, row1);  /*a1 a3 a5 a7*/
                res1 = _mm_unpacklo_epi16(row1, res0);  /*a0 a1 a2 a3 a4 a5 a6 a7*/
                if(is_last)
                {
                    res1 = _mm_min_epi16(res1, mm_max);
                    res1 = _mm_max_epi16(res1, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res1);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if(rem_w > 3)
    {
        inp_copy = ref + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for(row = height; row > 3; row -= 4)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadu_si128((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
            row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));  /*a1 a2 a3 a4 a5 a6 a7 a8*/
            row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2));
            row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3));
            row1 = _mm_shuffle_epi8(row1, shuffle);  /*a0 a1 a1 a2 a2 a3 a3 a4 */
            row2 = _mm_shuffle_epi8(row2, shuffle);
            row3 = _mm_shuffle_epi8(row3, shuffle);
            row4 = _mm_shuffle_epi8(row4, shuffle);
            row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
            row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
            row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
            row1 = _mm_add_epi32(row1, offset_4x32b);
            row2 = _mm_add_epi32(row2, offset_4x32b);
            row3 = _mm_add_epi32(row3, offset_4x32b);
            row4 = _mm_add_epi32(row4, offset_4x32b);
            row1 = _mm_srai_epi32(row1, shift);
            row2 = _mm_srai_epi32(row2, shift);
            row3 = _mm_srai_epi32(row3, shift);
            row4 = _mm_srai_epi32(row4, shift);
            res0 = _mm_packs_epi32(row1, row2);
            res1 = _mm_packs_epi32(row3, row4);
            if(is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res1 = _mm_min_epi16(res1, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
                res1 = _mm_max_epi16(res1, mm_min);
            }
            /* to store the 8 pixels res. */
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 2), res1);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 3), _mm_unpackhi_epi64(res1, res1));
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        for(row = 0; row < rem_h; row++)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadu_si128((__m128i*)(inp_copy));  /*a0 a1 a2 a3 a4 a5 a6 a7*/
            res0 = _mm_shuffle_epi8(row1, shuffle);        /*a0 a1 a1 a2 a2 a3 a3 a4 */
            res0 = _mm_madd_epi16(res0, coeff0_1_8x16b);   /*a0+a1 a1+a2 a2+a3 a3+a4*/
            res0 = _mm_add_epi32(res0, offset_4x32b);
            res0 = _mm_srai_epi32(res0, shift);
            res0 = _mm_packs_epi32(res0, res0);
            if(is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if(rem_w)
    {
        int sum, sum1;
        inp_copy = ref + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for(row = height; row > 3; row -= 4)
        {
            for(col = 0; col < rem_w; col++)
            {
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + col));                        /*a0 a1 x x x x x x*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));  /*b0 b1 x x x x x x*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + col));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + col));
                row1 = _mm_unpacklo_epi32(row1, row2);  /*a0 a1 b0 b1*/
                row3 = _mm_unpacklo_epi32(row3, row4);  /*c0 c1 d0 d1*/
                row1 = _mm_unpacklo_epi64(row1, row3);  /*a0 a1 b0 b1 c0 c1 d0 d1*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);  /*a0+a1 b0+b1 c0+c1 d0+d1*/
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                res0 = _mm_packs_epi32(row1, row1);
                if(is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                }
                /*extract 32 bit integer form register and store it in dst_copy*/
                sum = _mm_extract_epi32(res0, 0);
                sum1 = _mm_extract_epi32(res0, 1);
                dst_copy[col] = (s16)(sum & 0xffff);
                dst_copy[col + dst_stride] = (s16)(sum >> 16);
                dst_copy[col + (dst_stride << 1)] = (s16)(sum1 & 0xffff);
                dst_copy[col + (dst_stride * 3)] = (s16)(sum1 >> 16);
            }
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        for(row = 0; row < rem_h; row++)
        {
            for(col = 0; col < rem_w; col++)
            {
                int val;
                sum = inp_copy[col + 0] * coeff[0];
                sum += inp_copy[col + 1] * coeff[1];
                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}

void mc_filter_bilin_vert_sse
(
    s16 const *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const short *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h;
    int src_stride2, src_stride3, src_stride4;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i row1, row11, row2, row22, row3, row33, row4, row44, row5;
    __m128i res0, res1, res2, res3;
    __m128i coeff0_1_8x16b;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    src_stride4 = (stored_alf_para_num << 2);
    coeff0_1_8x16b = _mm_loadl_epi64((__m128i*)coeff);      /*w0 w1 x x x x x x*/
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);  /*w0 w1 w0 w1 w0 w1 w0 w1*/
    rem_h = height & 0x3;
    if (rem_w > 7)
    {
        for (row = height; row > 3; row -= 4)
        {
            int cnt = 0;
            for (col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));  /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row3 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                row4 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                row5 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride4 + cnt));
                row11 = _mm_unpacklo_epi16(row1, row2);   /*a0 b0 a1 b1 a2 b2 a3 b3*/
                row1 = _mm_unpackhi_epi16(row1, row2);    /*a4 b4 a5 b5 a6 b6 a7 b7*/
                row22 = _mm_unpacklo_epi16(row2, row3);
                row2 = _mm_unpackhi_epi16(row2, row3);
                row33 = _mm_unpacklo_epi16(row3, row4);
                row3 = _mm_unpackhi_epi16(row3, row4);
                row44 = _mm_unpacklo_epi16(row4, row5);
                row4 = _mm_unpackhi_epi16(row4, row5);
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);    /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                row2 = _mm_madd_epi16(row2, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
                row3 = _mm_madd_epi16(row3, coeff0_1_8x16b);
                row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
                row4 = _mm_madd_epi16(row4, coeff0_1_8x16b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row22 = _mm_add_epi32(row22, offset_4x32b);
                row2 = _mm_add_epi32(row2, offset_4x32b);
                row33 = _mm_add_epi32(row33, offset_4x32b);
                row3 = _mm_add_epi32(row3, offset_4x32b);
                row44 = _mm_add_epi32(row44, offset_4x32b);
                row4 = _mm_add_epi32(row4, offset_4x32b);
                row11 = _mm_srai_epi32(row11, shift);
                row1 = _mm_srai_epi32(row1, shift);
                row22 = _mm_srai_epi32(row22, shift);
                row2 = _mm_srai_epi32(row2, shift);
                row33 = _mm_srai_epi32(row33, shift);
                row3 = _mm_srai_epi32(row3, shift);
                row44 = _mm_srai_epi32(row44, shift);
                row4 = _mm_srai_epi32(row4, shift);
                res0 = _mm_packs_epi32(row11, row1);
                res1 = _mm_packs_epi32(row22, row2);
                res2 = _mm_packs_epi32(row33, row3);
                res3 = _mm_packs_epi32(row44, row4);
                if (is_last)
                {
                    res0 = _mm_min_epi16(res0, mm_max);
                    res1 = _mm_min_epi16(res1, mm_max);
                    res2 = _mm_min_epi16(res2, mm_max);
                    res3 = _mm_min_epi16(res3, mm_max);
                    res0 = _mm_max_epi16(res0, mm_min);
                    res1 = _mm_max_epi16(res1, mm_min);
                    res2 = _mm_max_epi16(res2, mm_min);
                    res3 = _mm_max_epi16(res3, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 2 + cnt), res2);
                _mm_storeu_si128((__m128i *)(dst_copy + dst_stride * 3 + cnt), res3);
                cnt += 8;  /* To pointer updates*/
            }
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        /*extra height to be done --- one row at a time*/
        for (row = 0; row < rem_h; row++)
        {
            int cnt = 0;
            for (col = rem_w; col > 7; col -= 8)
            {
                /*load 8 pixel values from row 0*/
                row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));                        /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));  /*b0 b1 b2 b3 b4 b5 b6 b7*/
                row11 = _mm_unpacklo_epi16(row1, row2);  /*a0 b0 a1 b1 a2 b2 a3 b3*/
                row1 = _mm_unpackhi_epi16(row1, row2);   /*a4 b4 a5 b5 a6 b6 a7 b7*/
                row1 = _mm_madd_epi16(row1, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a1+a2 a3+a4 a5+a6 a7+a8*/
                row1 = _mm_add_epi32(row1, offset_4x32b);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                row11 = _mm_srai_epi32(row11, shift);
                res1 = _mm_packs_epi32(row11, row1);
                if (is_last)
                {
                    res1 = _mm_min_epi16(res1, mm_max);
                    res1 = _mm_max_epi16(res1, mm_min);
                }
                /* to store the 8 pixels res. */
                _mm_storeu_si128((__m128i *)(dst_copy + cnt), res1);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = ref + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = height; row > 3; row -= 4)
        {
            /*load 4 pixel values */
            row1 = _mm_loadl_epi64((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 x x x x*/
            row2 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num));  /*b0 b1 b2 b3 x x x x*/
            row3 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2));
            row4 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3));
            row5 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride4));
            row11 = _mm_unpacklo_epi16(row1, row2);  /*a0 b0 a1 b1 a2 b2 a3 b3*/
            row22 = _mm_unpacklo_epi16(row2, row3);
            row33 = _mm_unpacklo_epi16(row3, row4);
            row44 = _mm_unpacklo_epi16(row4, row5);
            row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
            row33 = _mm_madd_epi16(row33, coeff0_1_8x16b);
            row44 = _mm_madd_epi16(row44, coeff0_1_8x16b);
            row11 = _mm_add_epi32(row11, offset_4x32b);
            row22 = _mm_add_epi32(row22, offset_4x32b);
            row33 = _mm_add_epi32(row33, offset_4x32b);
            row44 = _mm_add_epi32(row44, offset_4x32b);
            row11 = _mm_srai_epi32(row11, shift);
            row22 = _mm_srai_epi32(row22, shift);
            row33 = _mm_srai_epi32(row33, shift);
            row44 = _mm_srai_epi32(row44, shift);
            res0 = _mm_packs_epi32(row11, row22);
            res1 = _mm_packs_epi32(row33, row44);
            if (is_last)
            {
                res0 = _mm_min_epi16(res0, mm_max);
                res1 = _mm_min_epi16(res1, mm_max);
                res0 = _mm_max_epi16(res0, mm_min);
                res1 = _mm_max_epi16(res1, mm_min);
            }
            /* to store the 8 pixels res. */
            _mm_storel_epi64((__m128i *)(dst_copy), res0);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 2), res1);
            _mm_storel_epi64((__m128i *)(dst_copy + dst_stride * 3), _mm_unpackhi_epi64(res1, res1));
            inp_copy += (stored_alf_para_num << 2);
            dst_copy += (dst_stride << 2);
        }
        for (row = 0; row < rem_h; row++)
        {
            /*load 8 pixel values from row 0*/
            row1 = _mm_loadl_epi64((__m128i*)(inp_copy));                        /*a0 a1 a2 a3 x x x x*/
            row2 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num));  /*b0 b1 b2 b3 x x x x*/
            row11 = _mm_unpacklo_epi16(row1, row2);         /*a0 b0 a1 b1 a2 b2 a3 b3*/
            row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);  /*a0+a1 a1+a2 a2+a3 a3+a4*/
            row11 = _mm_add_epi32(row11, offset_4x32b);
            row11 = _mm_srai_epi32(row11, shift);
            row11 = _mm_packs_epi32(row11, row11);
            if (is_last)
            {
                row11 = _mm_min_epi16(row11, mm_max);
                row11 = _mm_max_epi16(row11, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), row11);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = ref + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}
#endif

#if SIMD_MC
#if AWP || SAWP
#if SAWP_WEIGHT_OPT || AWP_ENH
void weight_average_16b_no_clip_sse(s16* src, s16* ref, s16* dst, s16* weight0, s16* weight1, int s_src, int s_ref, int s_dst, int s_weight, int wd, int ht, int shift)
#else
void weight_average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, s16 *weight0, s16 *weight1, int s_src, int s_ref, int s_dst, int s_weight, int wd, int ht)
#endif
{
    s16 *p0, *p1, *p2;
    s16 *w0, *w1;
    int rem_h = ht;
    int rem_w;
    int i, j;
    __m128i src_8x16b, src_8x16b_1, src_8x16b_2, src_8x16b_3;
    __m128i pred_8x16b, pred_8x16b_1, pred_8x16b_2, pred_8x16b_3;
    __m128i weight0_8x16b, weight0_8x16b_1, weight0_8x16b_2, weight0_8x16b_3;
    __m128i weight1_8x16b, weight1_8x16b_1, weight1_8x16b_2, weight1_8x16b_3;
    __m128i temp_0, temp_1, temp_2, temp_3;
    __m128i offset_8x16b;
    /* Can be changed for a generic avg fun. or taken as an argument! */
#if SAWP_WEIGHT_OPT || AWP_ENH
    short offset = 1 << (shift - 1);
#else
    short offset = 4;
    int shift = 3;
#endif
    p0 = src;
    p1 = ref;
    p2 = dst;
    w0 = weight0;
    w1 = weight1;

    offset_8x16b = _mm_set1_epi16(offset);

    /* Mult. of 4 Loop */
    if (rem_h >= 4)
    {
        for (i = 0; i < rem_h - 3; i += 4)
        {
            p0 = src + (i * s_src);
            p1 = ref + (i * s_ref);
            p2 = dst + (i * s_dst);
            w0 = weight0 + (i * s_weight);
            w1 = weight1 + (i * s_weight);

            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j > 7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    src_8x16b_2 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 2)));
                    src_8x16b_3 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 3)));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    pred_8x16b_2 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 2)));
                    pred_8x16b_3 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 3)));
                    weight0_8x16b = _mm_loadu_si128((__m128i *) (w0));
                    weight0_8x16b_1 = _mm_loadu_si128((__m128i *) (w0 + s_weight));
                    weight0_8x16b_2 = _mm_loadu_si128((__m128i *) (w0 + (s_weight * 2)));
                    weight0_8x16b_3 = _mm_loadu_si128((__m128i *) (w0 + (s_weight * 3)));
                    weight1_8x16b = _mm_loadu_si128((__m128i *) (w1));
                    weight1_8x16b_1 = _mm_loadu_si128((__m128i *) (w1 + s_weight));
                    weight1_8x16b_2 = _mm_loadu_si128((__m128i *) (w1 + (s_weight * 2)));
                    weight1_8x16b_3 = _mm_loadu_si128((__m128i *) (w1 + (s_weight * 3)));
                    //add mul
                    src_8x16b = _mm_mullo_epi16(src_8x16b, weight0_8x16b);
                    src_8x16b_1 = _mm_mullo_epi16(src_8x16b_1, weight0_8x16b_1);
                    src_8x16b_2 = _mm_mullo_epi16(src_8x16b_2, weight0_8x16b_2);
                    src_8x16b_3 = _mm_mullo_epi16(src_8x16b_3, weight0_8x16b_3);

                    pred_8x16b = _mm_mullo_epi16(pred_8x16b, weight1_8x16b);
                    pred_8x16b_1 = _mm_mullo_epi16(pred_8x16b_1, weight1_8x16b_1);
                    pred_8x16b_2 = _mm_mullo_epi16(pred_8x16b_2, weight1_8x16b_2);
                    pred_8x16b_3 = _mm_mullo_epi16(pred_8x16b_3, weight1_8x16b_3);

                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                    temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                    temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    temp_2 = _mm_srai_epi16(temp_2, shift);
                    temp_3 = _mm_srai_epi16(temp_3, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    _mm_storeu_si128((__m128i *)(p2 + 2 * s_dst), temp_2);
                    _mm_storeu_si128((__m128i *)(p2 + 3 * s_dst), temp_3);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                    w0 += 8;
                    w1 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                src_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 2)));
                src_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 3)));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                pred_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 2)));
                pred_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 3)));
                weight0_8x16b = _mm_loadl_epi64((__m128i *) (w0));
                weight0_8x16b_1 = _mm_loadl_epi64((__m128i *) (w0 + s_weight));
                weight0_8x16b_2 = _mm_loadl_epi64((__m128i *) (w0 + (s_weight * 2)));
                weight0_8x16b_3 = _mm_loadl_epi64((__m128i *) (w0 + (s_weight * 3)));
                weight1_8x16b = _mm_loadl_epi64((__m128i *) (w1));
                weight1_8x16b_1 = _mm_loadl_epi64((__m128i *) (w1 + s_weight));
                weight1_8x16b_2 = _mm_loadl_epi64((__m128i *) (w1 + (s_weight * 2)));
                weight1_8x16b_3 = _mm_loadl_epi64((__m128i *) (w1 + (s_weight * 3)));
                //add mul
                src_8x16b = _mm_mullo_epi16(src_8x16b, weight0_8x16b);
                src_8x16b_1 = _mm_mullo_epi16(src_8x16b_1, weight0_8x16b_1);
                src_8x16b_2 = _mm_mullo_epi16(src_8x16b_2, weight0_8x16b_2);
                src_8x16b_3 = _mm_mullo_epi16(src_8x16b_3, weight0_8x16b_3);

                pred_8x16b = _mm_mullo_epi16(pred_8x16b, weight1_8x16b);
                pred_8x16b_1 = _mm_mullo_epi16(pred_8x16b_1, weight1_8x16b_1);
                pred_8x16b_2 = _mm_mullo_epi16(pred_8x16b_2, weight1_8x16b_2);
                pred_8x16b_3 = _mm_mullo_epi16(pred_8x16b_3, weight1_8x16b_3);

                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                temp_2 = _mm_srai_epi16(temp_2, shift);
                temp_3 = _mm_srai_epi16(temp_3, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                _mm_storel_epi64((__m128i *)(p2 + 2 * s_dst), temp_2);
                _mm_storel_epi64((__m128i *)(p2 + 3 * s_dst), temp_3);
                p0 += 4;
                p1 += 4;
                p2 += 4;
                w0 += 4;
                w1 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] * weight0[j + 0 * s_weight] + p1[j + 0 * s_ref] * weight1[j + 0 * s_weight] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] * weight0[j + 1 * s_weight] + p1[j + 1 * s_ref] * weight1[j + 1 * s_weight] + offset) >> shift;
                    p2[j + 2 * s_dst] = (p0[j + 2 * s_src] * weight0[j + 2 * s_weight] + p1[j + 2 * s_ref] * weight1[j + 2 * s_weight] + offset) >> shift;
                    p2[j + 3 * s_dst] = (p0[j + 3 * s_src] * weight0[j + 3 * s_weight] + p1[j + 3 * s_ref] * weight1[j + 3 * s_weight] + offset) >> shift;
                }
            }
        }
    }
}
#endif
void average_16b_no_clip_sse(s16 *src, s16 *ref, s16 *dst, int s_src, int s_ref, int s_dst, int wd, int ht)
{
    s16 *p0, *p1, *p2;
    int rem_h = ht;
    int rem_w;
    int i, j;
    __m128i src_8x16b, src_8x16b_1, src_8x16b_2, src_8x16b_3;
    __m128i pred_8x16b, pred_8x16b_1, pred_8x16b_2, pred_8x16b_3;
    __m128i temp_0, temp_1, temp_2, temp_3;
    __m128i offset_8x16b;
    /* Can be changed for a generic avg fun. or taken as an argument! */
    short offset = 1;
    int shift = 1;
    p0 = src;
    p1 = ref;
    p2 = dst;
    offset_8x16b = _mm_set1_epi16(offset);
    /* Mult. of 4 Loop */
    if (rem_h >= 4)
    {
        for (i = 0; i < rem_h - 3; i += 4)
        {
            p0 = src + (i * s_src);
            p1 = ref + (i * s_ref);
            p2 = dst + (i * s_dst);
            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j >7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    src_8x16b_2 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 2)));
                    src_8x16b_3 = _mm_loadu_si128((__m128i *) (p0 + (s_src * 3)));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    pred_8x16b_2 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 2)));
                    pred_8x16b_3 = _mm_loadu_si128((__m128i *) (p1 + (s_ref * 3)));
                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                    temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                    temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    temp_2 = _mm_srai_epi16(temp_2, shift);
                    temp_3 = _mm_srai_epi16(temp_3, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    _mm_storeu_si128((__m128i *)(p2 + 2 * s_dst), temp_2);
                    _mm_storeu_si128((__m128i *)(p2 + 3 * s_dst), temp_3);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                src_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 2)));
                src_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0 + (s_src * 3)));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                pred_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 2)));
                pred_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1 + (s_ref * 3)));
                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_add_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_add_epi16(src_8x16b_3, pred_8x16b_3);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_2 = _mm_add_epi16(temp_2, offset_8x16b);
                temp_3 = _mm_add_epi16(temp_3, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                temp_2 = _mm_srai_epi16(temp_2, shift);
                temp_3 = _mm_srai_epi16(temp_3, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                _mm_storel_epi64((__m128i *)(p2 + 2 * s_dst), temp_2);
                _mm_storel_epi64((__m128i *)(p2 + 3 * s_dst), temp_3);
                p0 += 4;
                p1 += 4;
                p2 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] + p1[j + 0 * s_ref] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] + p1[j + 1 * s_ref] + offset) >> shift;
                    p2[j + 2 * s_dst] = (p0[j + 2 * s_src] + p1[j + 2 * s_ref] + offset) >> shift;
                    p2[j + 3 * s_dst] = (p0[j + 3 * s_src] + p1[j + 3 * s_ref] + offset) >> shift;
                }
            }
        }
    }
    /* Remaining rows */
    rem_h &= 0x3;
    /* One 2 row case */
    if (rem_h >= 2)
    {
        p0 = src + ((ht >> 2) << 2) * s_src;
        p1 = ref + ((ht >> 2) << 2) * s_ref;
        p2 = dst + ((ht >> 2) << 2) * s_dst;
        /* One 2 row case */
        {
            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j >7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    src_8x16b_1 = _mm_loadu_si128((__m128i *) (p0 + s_src));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    pred_8x16b_1 = _mm_loadu_si128((__m128i *) (p1 + s_ref));
                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    temp_1 = _mm_srai_epi16(temp_1, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    _mm_storeu_si128((__m128i *)(p2 + 1 * s_dst), temp_1);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                src_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0 + s_src));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                pred_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1 + s_ref));
                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_add_epi16(src_8x16b_1, pred_8x16b_1);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_1 = _mm_add_epi16(temp_1, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                temp_1 = _mm_srai_epi16(temp_1, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                _mm_storel_epi64((__m128i *)(p2 + 1 * s_dst), temp_1);
                p0 += 4;
                p1 += 4;
                p2 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j + 0 * s_dst] = (p0[j + 0 * s_src] + p1[j + 0 * s_ref] + offset) >> shift;
                    p2[j + 1 * s_dst] = (p0[j + 1 * s_src] + p1[j + 1 * s_ref] + offset) >> shift;
                }
            }
        }
    }
    /* Remaining 1 row */
    if (rem_h &= 0x1)
    {
        p0 = src + ((ht >> 1) << 1) * s_src;
        p1 = ref + ((ht >> 1) << 1) * s_ref;
        p2 = dst + ((ht >> 1) << 1) * s_dst;
        /* One 1 row case */
        {
            rem_w = wd;
            /* Mult. of 8 Loop */
            if (rem_w > 7)
            {
                for (j = rem_w; j >7; j -= 8)
                {
                    src_8x16b = _mm_loadu_si128((__m128i *) (p0));
                    pred_8x16b = _mm_loadu_si128((__m128i *) (p1));
                    temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                    temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                    temp_0 = _mm_srai_epi16(temp_0, shift);
                    _mm_storeu_si128((__m128i *)(p2 + 0 * s_dst), temp_0);
                    p0 += 8;
                    p1 += 8;
                    p2 += 8;
                }
            }
            rem_w &= 0x7;
            /* One 4 case */
            if (rem_w > 3)
            {
                src_8x16b = _mm_loadl_epi64((__m128i *) (p0));
                pred_8x16b = _mm_loadl_epi64((__m128i *) (p1));
                temp_0 = _mm_add_epi16(src_8x16b, pred_8x16b);
                temp_0 = _mm_add_epi16(temp_0, offset_8x16b);
                temp_0 = _mm_srai_epi16(temp_0, shift);
                _mm_storel_epi64((__m128i *)(p2 + 0 * s_dst), temp_0);
                p0 += 4;
                p1 += 4;
                p2 += 4;
            }
            /* Remaining */
            rem_w &= 0x3;
            if (rem_w)
            {
                for (j = 0; j < rem_w; j++)
                {
                    p2[j] = (p0[j] + p1[j] + offset) >> shift;
                }
            }
        }
    }
}

#if IF_LUMA12_CHROMA6_SIMD
static void mc_filter_l_12pel_horz_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8 is_last)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b, src_temp17_16x8b, src_temp18_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b, coeff8_9_8x16b, coeff10_11_8x16b;
    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;
    /* load 12 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    coeff8_9_8x16b = _mm_loadu_si128((__m128i*)(coeff + 8));
    coeff10_11_8x16b = _mm_shuffle_epi32(coeff8_9_8x16b, 0x55);
    coeff8_9_8x16b = _mm_shuffle_epi32(coeff8_9_8x16b, 0);

    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 12 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 8));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 9));
                    src_temp7_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp11_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp5_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff8_9_8x16b);
                    res_temp11_8x16b = _mm_madd_epi16(src_temp11_16x8b, coeff8_9_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 10));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 11));
                    src_temp8_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp12_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp6_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff10_11_8x16b);
                    res_temp12_8x16b = _mm_madd_epi16(src_temp12_16x8b, coeff10_11_8x16b);

                    res_temp1_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);  
                    res_temp3_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b); 
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp3_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp1_8x16b);

                    res_temp7_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp9_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                    res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp9_8x16b);
                    res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp7_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp11_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }
                    /* to store the 12 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 2)
            {
                /*load 12 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 8));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 9));
                src_temp7_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp5_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff8_9_8x16b);
                
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 10));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 11));
                src_temp8_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp6_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff10_11_8x16b);

                res_temp1_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp3_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp3_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp1_8x16b);

                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));    /* row = 1 */
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 8));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 9));
                src_temp17_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp15_8x16b = _mm_madd_epi16(src_temp17_16x8b, coeff8_9_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 10));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 11));
                src_temp18_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp16_8x16b = _mm_madd_epi16(src_temp18_16x8b, coeff10_11_8x16b);

                res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp13_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp13_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp11_8x16b);

                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    int val1, val2;
                    int sum1, sum2;

                    sum1 =  inp_copy[col + 0] * coeff[0];
                    sum1 += inp_copy[col + 1] * coeff[1];
                    sum1 += inp_copy[col + 2] * coeff[2];
                    sum1 += inp_copy[col + 3] * coeff[3];
                    sum1 += inp_copy[col + 4] * coeff[4];
                    sum1 += inp_copy[col + 5] * coeff[5];
                    sum1 += inp_copy[col + 6] * coeff[6];
                    sum1 += inp_copy[col + 7] * coeff[7];
                    sum1 += inp_copy[col + 8] * coeff[8];
                    sum1 += inp_copy[col + 9] * coeff[9];
                    sum1 += inp_copy[col + 10] * coeff[10];
                    sum1 += inp_copy[col + 11] * coeff[11];
                    val1 = (sum1 + offset) >> shift;

                    sum2 =  inp_copy[col + stored_alf_para_num + 0] * coeff[0];
                    sum2 += inp_copy[col + stored_alf_para_num + 1] * coeff[1];
                    sum2 += inp_copy[col + stored_alf_para_num + 2] * coeff[2];
                    sum2 += inp_copy[col + stored_alf_para_num + 3] * coeff[3];
                    sum2 += inp_copy[col + stored_alf_para_num + 4] * coeff[4];
                    sum2 += inp_copy[col + stored_alf_para_num + 5] * coeff[5];
                    sum2 += inp_copy[col + stored_alf_para_num + 6] * coeff[6];
                    sum2 += inp_copy[col + stored_alf_para_num + 7] * coeff[7];
                    sum2 += inp_copy[col + stored_alf_para_num + 8] * coeff[8];
                    sum2 += inp_copy[col + stored_alf_para_num + 9] * coeff[9];
                    sum2 += inp_copy[col + stored_alf_para_num + 10] * coeff[10];
                    sum2 += inp_copy[col + stored_alf_para_num + 11] * coeff[11];
                    val2 = (sum2 + offset) >> shift;

                    if (is_last)
                    {
                        val1 = COM_CLIP3(min_val, max_val, val1);
                        val2 = COM_CLIP3(min_val, max_val, val2);
                    }
                    dst_copy[col] = (s16)val1;
                    dst_copy[col + dst_stride] = (s16)val2;
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 12 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 8));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 9));
                    src_temp7_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp11_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp5_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff8_9_8x16b);
                    res_temp11_8x16b = _mm_madd_epi16(src_temp11_16x8b, coeff8_9_8x16b);

                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 10));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 11));
                    src_temp8_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp12_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp6_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff10_11_8x16b);
                    res_temp12_8x16b = _mm_madd_epi16(src_temp12_16x8b, coeff10_11_8x16b);

                    res_temp1_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);  
                    res_temp3_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b); 
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp3_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp1_8x16b);

                    res_temp7_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp9_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                    res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp9_8x16b);
                    res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp7_8x16b);

                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp11_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }
                    /* to store the 12 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* to pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < (height - 1); row += 2)
            {
                /*load 12 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 8));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 9));
                src_temp7_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp5_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff8_9_8x16b);

                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 10));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 11));
                src_temp8_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp6_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff10_11_8x16b);

                res_temp1_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp3_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp3_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp1_8x16b);

                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));    /* row = 1 */
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 8));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 9));
                src_temp17_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp15_8x16b = _mm_madd_epi16(src_temp17_16x8b, coeff8_9_8x16b);

                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 10));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 11));
                src_temp18_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp16_8x16b = _mm_madd_epi16(src_temp18_16x8b, coeff10_11_8x16b);

                res_temp11_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp13_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp13_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp11_8x16b);

                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* pointer update */
                dst_copy += (dst_stride << 1);  /* pointer update */
            }
            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 8));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 9));
            src_temp7_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp5_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff8_9_8x16b);

            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 10));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 11));
            src_temp8_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp6_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff10_11_8x16b);

            res_temp1_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp3_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp3_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp1_8x16b);

            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
            if (is_last)
            {
                res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            for (row = 0; row < height; row++)
            {
                for (col = 0; col < rem_w; col++)
                {
                    int val;
                    int sum;

                    sum =  inp_copy[col + 0] * coeff[0];
                    sum += inp_copy[col + 1] * coeff[1];
                    sum += inp_copy[col + 2] * coeff[2];
                    sum += inp_copy[col + 3] * coeff[3];
                    sum += inp_copy[col + 4] * coeff[4];
                    sum += inp_copy[col + 5] * coeff[5];
                    sum += inp_copy[col + 6] * coeff[6];
                    sum += inp_copy[col + 7] * coeff[7];
                    sum += inp_copy[col + 8] * coeff[8];
                    sum += inp_copy[col + 9] * coeff[9];
                    sum += inp_copy[col + 10] * coeff[10];
                    sum += inp_copy[col + 11] * coeff[11];
                    val = (sum + offset) >> shift;
                    if (is_last)
                    {
                        val = COM_CLIP3(min_val, max_val, val);
                    }
                    dst_copy[col] = val;    
                }
                inp_copy += stored_alf_para_num;
                dst_copy += dst_stride;
            }
        }
    }
}

static void mc_filter_l_12pel_vert_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8 is_last)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b, coeff8_9_8x16b, coeff10_11_8x16b;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s3_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b, s10_8x16b, s11_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b, s2_6_16x8b, s2_7_16x8b, s2_8_16x8b, s2_9_16x8b, s2_10_16x8b, s2_11_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_3_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b, s3_7_16x8b, s3_8_16x8b, s3_9_16x8b, s3_10_16x8b, s3_11_16x8b;
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    coeff8_9_8x16b = _mm_loadu_si128((__m128i*)(coeff + 8));
    coeff10_11_8x16b = _mm_shuffle_epi32(coeff8_9_8x16b, 0x55);
    coeff8_9_8x16b = _mm_shuffle_epi32(coeff8_9_8x16b, 0);
    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff0_1_8x16b);

                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 2) + cnt));
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_7_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s7_8x16b = _mm_madd_epi16(s3_7_16x8b, coeff2_3_8x16b);

                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 4) + cnt));
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 5) + cnt));
                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_8_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);
                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s8_8x16b = _mm_madd_epi16(s3_8_16x8b, coeff4_5_8x16b);

                s2_6_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 6) + cnt));
                s2_7_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 7) + cnt));
                s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_9_16x8b = _mm_unpackhi_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
                s9_8x16b = _mm_madd_epi16(s3_9_16x8b, coeff6_7_8x16b);

                s2_8_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 8) + cnt));
                s2_9_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 9) + cnt));
                s3_4_16x8b = _mm_unpacklo_epi16(s2_8_16x8b, s2_9_16x8b);
                s3_10_16x8b = _mm_unpackhi_epi16(s2_8_16x8b, s2_9_16x8b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff8_9_8x16b);
                s10_8x16b = _mm_madd_epi16(s3_10_16x8b, coeff8_9_8x16b);

                s2_10_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 10) + cnt));
                s2_11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 11) + cnt));
                s3_5_16x8b = _mm_unpacklo_epi16(s2_10_16x8b, s2_11_16x8b);
                s3_11_16x8b = _mm_unpackhi_epi16(s2_10_16x8b, s2_11_16x8b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff10_11_8x16b);
                s11_8x16b = _mm_madd_epi16(s3_11_16x8b, coeff10_11_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s7_8x16b);
                s8_8x16b = _mm_add_epi32(s8_8x16b, s9_8x16b);
                s10_8x16b = _mm_add_epi32(s10_8x16b, s11_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s4_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s8_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s10_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);
                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s6_8x16b, shift);

                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);
                if (is_last)
                {
                    s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
                }
                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = 0; row < height; row++)
        {
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));
            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);

            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));
            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);

            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * stored_alf_para_num)));
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * stored_alf_para_num)));
            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);

            s2_6_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (6 * stored_alf_para_num)));
            s2_7_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (7 * stored_alf_para_num)));
            s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
            s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);

            s2_8_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (8 * stored_alf_para_num)));
            s2_9_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (9 * stored_alf_para_num)));
            s3_4_16x8b = _mm_unpacklo_epi16(s2_8_16x8b, s2_9_16x8b);
            s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff8_9_8x16b);

            s2_10_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (10 * stored_alf_para_num)));
            s2_11_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (11 * stored_alf_para_num)));
            s3_5_16x8b = _mm_unpacklo_epi16(s2_10_16x8b, s2_11_16x8b);
            s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff10_11_8x16b);

            s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
            s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
            s6_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
            s6_8x16b = _mm_add_epi32(s6_8x16b, s4_8x16b);

            s7_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);
            if (is_last)
            {
                s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                sum += inp_copy[col + 4 * stored_alf_para_num] * coeff[4];
                sum += inp_copy[col + 5 * stored_alf_para_num] * coeff[5];
                sum += inp_copy[col + 6 * stored_alf_para_num] * coeff[6];
                sum += inp_copy[col + 7 * stored_alf_para_num] * coeff[7];
                sum += inp_copy[col + 8 * stored_alf_para_num] * coeff[8];
                sum += inp_copy[col + 9 * stored_alf_para_num] * coeff[9];
                sum += inp_copy[col + 10 * stored_alf_para_num] * coeff[10];
                sum += inp_copy[col + 11 * stored_alf_para_num] * coeff[11];
                val = (sum + offset) >> shift;
                if (is_last)
                {
                    val = COM_CLIP3(min_val, max_val, val);
                }
                dst_copy[col] = (s16)val;
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}
#endif

static void mc_filter_l_8pel_horz_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i    mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    //if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 2)
            {
                /*load 8 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));    /* row = 1 */
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                //if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    //if (is_last)
                    {
                        sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                        sum1 = (sum1 < min_val) ? min_val : (sum1>max_val ? max_val : sum1);
                    }
                    dst_copy[col] = sum;
                    dst_copy[col + dst_stride] = sum1;
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    /* row = 0 */
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    //if (is_last)
                    {
                        res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                        res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < (height - 1); row += 2)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                /* row =1 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                //if (is_last)
                {
                    res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                    res_temp15_8x16b = _mm_min_epi16(res_temp15_8x16b, mm_max);
                    res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
                    res_temp15_8x16b = _mm_max_epi16(res_temp15_8x16b, mm_min);
                }
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
            //if (is_last)
            {
                res_temp5_8x16b = _mm_min_epi16(res_temp5_8x16b, mm_max);
                res_temp5_8x16b = _mm_max_epi16(res_temp5_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < (height - 1); row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    //if (is_last)
                    {
                        sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                        sum1 = (sum1 < min_val) ? min_val : (sum1>max_val ? max_val : sum1);
                    }
                    dst_copy[col] = sum;
                    dst_copy[col + dst_stride] = sum1;
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
            for (col = 0; col < rem_w; col++)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                src_temp2_16x8b = _mm_srli_si128(src_temp1_16x8b, 4);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, src_temp2_16x8b);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                //if (is_last)
                {
                    sum = (sum < min_val) ? min_val : (sum>max_val ? max_val : sum);
                }
                    dst_copy[col] = sum;
            }
        }
    }
}

static void mc_filter_l_8pel_horz_no_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_8x16b = _mm_set1_epi32(offset);
    __m128i src_temp1_16x8b, src_temp2_16x8b, src_temp3_16x8b, src_temp4_16x8b, src_temp5_16x8b, src_temp6_16x8b;
    __m128i src_temp7_16x8b, src_temp8_16x8b, src_temp9_16x8b, src_temp0_16x8b;
    __m128i src_temp11_16x8b, src_temp12_16x8b, src_temp13_16x8b, src_temp14_16x8b, src_temp15_16x8b, src_temp16_16x8b;
    __m128i res_temp1_8x16b, res_temp2_8x16b, res_temp3_8x16b, res_temp4_8x16b, res_temp5_8x16b, res_temp6_8x16b, res_temp7_8x16b, res_temp8_8x16b;
    __m128i res_temp9_8x16b, res_temp0_8x16b;
    __m128i res_temp11_8x16b, res_temp12_8x16b, res_temp13_8x16b, res_temp14_8x16b, res_temp15_8x16b, res_temp16_8x16b;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    src_tmp = ref;
    rem_w = width;
    inp_copy = src_tmp;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (!(height & 1))    /*even height*/
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 2)
            {
                /*load 8 pixel values */
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));                /* row = 0 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));    /* row = 1 */
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < height; row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    dst_copy[col] = sum;
                    dst_copy[col + dst_stride] = sum1;
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
        }
    }
    else
    {
        if (rem_w > 7)
        {
            for (row = 0; row < height; row += 1)
            {
                int cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load 8 pixel values from row 0*/
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));
                    src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp7_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                    res_temp7_8x16b = _mm_madd_epi16(src_temp7_16x8b, coeff0_1_8x16b);
                    /* row = 0 */
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));
                    src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp8_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                    res_temp8_8x16b = _mm_madd_epi16(src_temp8_16x8b, coeff2_3_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp9_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                    res_temp9_8x16b = _mm_madd_epi16(src_temp9_16x8b, coeff4_5_8x16b);
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 6));
                    src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 7));
                    src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    src_temp0_16x8b = _mm_unpackhi_epi16(src_temp1_16x8b, src_temp2_16x8b);
                    res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                    res_temp0_8x16b = _mm_madd_epi16(src_temp0_16x8b, coeff6_7_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                    res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp7_8x16b, res_temp8_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp9_8x16b, res_temp0_8x16b);
                    res_temp8_8x16b = _mm_add_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                    res_temp7_8x16b = _mm_add_epi32(res_temp8_8x16b, offset_8x16b);
                    res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                    res_temp7_8x16b = _mm_srai_epi32(res_temp7_8x16b, shift);
                    res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp7_8x16b);
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res_temp5_8x16b);
                    cnt += 8; /* To pointer updates*/
                }
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        if (rem_w > 3)
        {
            inp_copy = src_tmp + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < (height - 1); row += 2)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
                src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
                src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
                src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
                src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
                src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
                res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
                res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
                res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
                res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
                res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 1));
                src_temp13_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp11_8x16b = _mm_madd_epi16(src_temp13_16x8b, coeff0_1_8x16b);
                /* row =1 */
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 2));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 3));
                src_temp14_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp12_8x16b = _mm_madd_epi16(src_temp14_16x8b, coeff2_3_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 4));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 5));
                src_temp15_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp13_8x16b = _mm_madd_epi16(src_temp15_16x8b, coeff4_5_8x16b);
                src_temp11_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 6));
                src_temp12_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + 7));
                src_temp16_16x8b = _mm_unpacklo_epi16(src_temp11_16x8b, src_temp12_16x8b);
                res_temp14_8x16b = _mm_madd_epi16(src_temp16_16x8b, coeff6_7_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp11_8x16b, res_temp12_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp13_8x16b, res_temp14_8x16b);
                res_temp15_8x16b = _mm_add_epi32(res_temp15_8x16b, res_temp16_8x16b);
                res_temp16_8x16b = _mm_add_epi32(res_temp15_8x16b, offset_8x16b);
                res_temp16_8x16b = _mm_srai_epi32(res_temp16_8x16b, shift);
                res_temp15_8x16b = _mm_packs_epi32(res_temp16_8x16b, res_temp16_8x16b);
                /* to store the 1st 4 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), res_temp15_8x16b);
                inp_copy += (stored_alf_para_num << 1);  /* Pointer update */
                dst_copy += (dst_stride << 1);  /* Pointer update */
            }
            /*extra one height to be done*/
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 1));
            src_temp3_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp1_8x16b = _mm_madd_epi16(src_temp3_16x8b, coeff0_1_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 2));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 3));
            src_temp4_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp2_8x16b = _mm_madd_epi16(src_temp4_16x8b, coeff2_3_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 4));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 5));
            src_temp5_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp3_8x16b = _mm_madd_epi16(src_temp5_16x8b, coeff4_5_8x16b);
            src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 6));
            src_temp2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + 7));
            src_temp6_16x8b = _mm_unpacklo_epi16(src_temp1_16x8b, src_temp2_16x8b);
            res_temp4_8x16b = _mm_madd_epi16(src_temp6_16x8b, coeff6_7_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp1_8x16b, res_temp2_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp3_8x16b, res_temp4_8x16b);
            res_temp5_8x16b = _mm_add_epi32(res_temp5_8x16b, res_temp6_8x16b);
            res_temp6_8x16b = _mm_add_epi32(res_temp5_8x16b, offset_8x16b);
            res_temp6_8x16b = _mm_srai_epi32(res_temp6_8x16b, shift);
            res_temp5_8x16b = _mm_packs_epi32(res_temp6_8x16b, res_temp6_8x16b);
            _mm_storel_epi64((__m128i *)(dst_copy), res_temp5_8x16b);
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            __m128i filt_coef;
            s16 sum, sum1;
            inp_copy = src_tmp + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            filt_coef = _mm_loadu_si128((__m128i*)coeff);   //w0 w1 w2 w3 w4 w5 w6 w7
            for (row = 0; row < (height - 1); row += 2)
            {
                for (col = 0; col < rem_w; col++)
                {
                    src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                    src_temp5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + col));
                    src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                    src_temp5_16x8b = _mm_madd_epi16(src_temp5_16x8b, filt_coef);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp5_16x8b);
                    src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                    src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                    src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                    sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                    sum1 = _mm_extract_epi16(src_temp1_16x8b, 1);
                    dst_copy[col] = sum;
                    dst_copy[col + dst_stride] = sum1;
                }
                inp_copy += (stored_alf_para_num << 1);
                dst_copy += (dst_stride << 1);
            }
            for (col = 0; col < rem_w; col++)
            {
                src_temp1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + col));
                src_temp1_16x8b = _mm_madd_epi16(src_temp1_16x8b, filt_coef);
                src_temp1_16x8b = _mm_hadd_epi32(src_temp1_16x8b, src_temp1_16x8b);
                src_temp2_16x8b = _mm_srli_si128(src_temp1_16x8b, 4);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, src_temp2_16x8b);
                src_temp1_16x8b = _mm_add_epi32(src_temp1_16x8b, offset_8x16b);
                src_temp1_16x8b = _mm_srai_epi32(src_temp1_16x8b, shift);
                src_temp1_16x8b = _mm_packs_epi32(src_temp1_16x8b, src_temp1_16x8b);
                sum = _mm_extract_epi16(src_temp1_16x8b, 0);
                dst_copy[col] = sum;
            }
        }
    }
}

static void mc_filter_l_8pel_vert_clip_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s3_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b, s2_6_16x8b, s2_7_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_3_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b, s3_7_16x8b;
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */
    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /*load 8 pixel values.*/
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                /*load 8 pixel values*/
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /*load 8 pixel values*/
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));
                /*load 8 pixel values*/
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);
                /*load 8 pixel values*/
                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 2) + cnt));
                /*load 8 pixel values*/
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 5) + cnt));
                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);
                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff4_5_8x16b);
                /*load 8 pixel values*/
                s2_6_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 6) + cnt));
                /*load 8 pixel values*/
                s2_7_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 7) + cnt));
                s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_7_16x8b = _mm_unpackhi_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
                s7_8x16b = _mm_madd_epi16(s3_7_16x8b, coeff6_7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s6_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);
                /* i2_tmp = CLIP_U8(i2_tmp);*/
                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);
                //if (is_last)
                {
                    s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
                }
                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = 0; row < height; row++)
        {
            /*load 8 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            /*load 8 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));
            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 8 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));
            /*load 8 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));
            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            /*load 8 pixel values*/
            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * stored_alf_para_num)));
            /*load 8 pixel values*/
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * stored_alf_para_num)));
            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
            /*load 8 pixel values*/
            s2_6_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (6 * stored_alf_para_num)));
            /*load 8 pixel values*/
            s2_7_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (7 * stored_alf_para_num)));
            s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
            s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s5_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
            s6_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
            s7_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);
            /*(i2_tmp + OFFSET_14_MINUS_BIT_DEPTH) >> SHIFT_14_MINUS_BIT_DEPTH */
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);
            /* i2_tmp = CLIP_U8(i2_tmp);*/
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);
            //if (is_last)
            {
                s9_8x16b = _mm_min_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_max_epi16(s9_8x16b, mm_min);
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                sum += inp_copy[col + 4 * stored_alf_para_num] * coeff[4];
                sum += inp_copy[col + 5 * stored_alf_para_num] * coeff[5];
                sum += inp_copy[col + 6 * stored_alf_para_num] * coeff[6];
                sum += inp_copy[col + 7 * stored_alf_para_num] * coeff[7];
                val = (sum + offset) >> shift;
                //if (is_last)
                {
                    val = COM_CLIP3(min_val, max_val, val);
                }
                dst_copy[col] = (s16)val;
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}

static void mc_filter_l_8pel_vert_no_clip_sse
(
    s16* ref,
    int stored_alf_para_num,
    s16* pred,
    int dst_stride,
    const s16* coeff,
    int width,
    int height,
    int offset,
    int shift)
{
    int row, col, rem_w;
    s16 const* src_tmp;
    s16 const* inp_copy;
    s16* dst_copy;

    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, coeff6_7_8x16b;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s3_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b, s2_6_16x8b, s2_7_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_3_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b, s3_7_16x8b;

    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */

    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;

    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);

    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff6_7_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xff);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);

    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /*load 8 pixel values.*/
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));

                /*load 8 pixel values*/
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));

                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);

                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /*load 8 pixel values*/
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));

                /*load 8 pixel values*/
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));

                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);

                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);

                /*load 8 pixel values*/
                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 2) + cnt));

                /*load 8 pixel values*/
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 5) + cnt));

                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);

                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff4_5_8x16b);

                /*load 8 pixel values*/
                s2_6_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 6) + cnt));

                /*load 8 pixel values*/
                s2_7_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 7) + cnt));

                s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);
                s3_7_16x8b = _mm_unpackhi_epi16(s2_6_16x8b, s2_7_16x8b);

                s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);
                s7_8x16b = _mm_madd_epi16(s3_7_16x8b, coeff6_7_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s2_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s6_8x16b = _mm_add_epi32(s6_8x16b, s7_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s6_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);

                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);

                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);

                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);

                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x7;

    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);

        for (row = 0; row < height; row++)
        {
            /*load 8 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));

            /*load 8 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));

            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);

            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 8 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));

            /*load 8 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));

            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);

            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            /*load 8 pixel values*/
            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * stored_alf_para_num)));

            /*load 8 pixel values*/
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * stored_alf_para_num)));

            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);

            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
            /*load 8 pixel values*/
            s2_6_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (6 * stored_alf_para_num)));

            /*load 8 pixel values*/
            s2_7_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (7 * stored_alf_para_num)));

            s3_3_16x8b = _mm_unpacklo_epi16(s2_6_16x8b, s2_7_16x8b);

            s3_8x16b = _mm_madd_epi16(s3_3_16x8b, coeff6_7_8x16b);

            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s5_8x16b = _mm_add_epi32(s2_8x16b, s3_8x16b);
            s6_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);

            s7_8x16b = _mm_add_epi32(s6_8x16b, offset_8x16b);

            /*(i2_tmp + OFFSET_14_MINUS_BIT_DEPTH) >> SHIFT_14_MINUS_BIT_DEPTH */
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);

            /* i2_tmp = CLIP_U8(i2_tmp);*/
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);

            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);

            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }

    rem_w &= 0x3;

    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);

        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;

                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                sum += inp_copy[col + 4 * stored_alf_para_num] * coeff[4];
                sum += inp_copy[col + 5 * stored_alf_para_num] * coeff[5];
                sum += inp_copy[col + 6 * stored_alf_para_num] * coeff[6];
                sum += inp_copy[col + 7 * stored_alf_para_num] * coeff[7];

                val = (sum + offset) >> shift;

                dst_copy[col] = (s16)val;
            }

            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}

#if IF_LUMA12_CHROMA6_SIMD
static void mc_filter_c_6pel_horz_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h, cnt;
    int src_stride2, src_stride3;
    s16 *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, mm_mask;
    __m128i res0;
    __m128i row1, row2, row3, row4, row5, row6;
    __m128i row12_lo, row12_hi, row34_lo, row34_hi, row56_lo, row56_hi;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    {
        rem_h = height & 0x3;
        rem_w = width;
        coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);  /*w2 w3 w2 w3 w2 w3 w2 w3*/
        coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
        coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);        /*w0 w1 w0 w1 w0 w1 w0 w1*/
        if (rem_w > 7)
        {
            cnt = 0;
            for (row = 0; row < height; row++)
            {
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row1 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row2 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row12_lo = _mm_unpacklo_epi16(row1, row2);
                    row12_hi = _mm_unpackhi_epi16(row1, row2);
                    row1 = _mm_madd_epi16(row12_lo, coeff0_1_8x16b);
                    row2 = _mm_madd_epi16(row12_hi, coeff0_1_8x16b);

                    row3 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row4 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                    row34_lo = _mm_unpacklo_epi16(row3, row4);
                    row34_hi = _mm_unpackhi_epi16(row3, row4);
                    row3 = _mm_madd_epi16(row34_lo, coeff2_3_8x16b);
                    row4 = _mm_madd_epi16(row34_hi, coeff2_3_8x16b);

                    row5 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 4));
                    row6 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 5));
                    row56_lo = _mm_unpacklo_epi16(row5, row6);
                    row56_hi = _mm_unpackhi_epi16(row5, row6);
                    row5 = _mm_madd_epi16(row56_lo, coeff4_5_8x16b);
                    row6 = _mm_madd_epi16(row56_hi, coeff4_5_8x16b);

                    row1 = _mm_add_epi32(row1, row3);
                    row1 = _mm_add_epi32(row1, row5);
                    row2 = _mm_add_epi32(row2, row4);
                    row2 = _mm_add_epi32(row2, row6);

                    row1 = _mm_add_epi32(row1, offset_4x32b);
                    row2 = _mm_add_epi32(row2, offset_4x32b);
                    row1 = _mm_srai_epi32(row1, shift);
                    row2 = _mm_srai_epi32(row2, shift);

                    res0 = _mm_packs_epi32(row1, row2);
                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                    cnt += 8;
                }
                cnt = 0;
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        /* one 4 pixel wd for multiple rows */
        if (rem_w > 3)
        {
            inp_copy = ref + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row++)
            {
                /*load pixel values from row 1*/
                row1 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row2 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row3 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row4 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                row5 = _mm_loadl_epi64((__m128i*)(inp_copy + 4));
                row6 = _mm_loadl_epi64((__m128i*)(inp_copy + 5));

                row12_lo = _mm_unpacklo_epi16(row1, row2);
                row1 = _mm_madd_epi16(row12_lo, coeff0_1_8x16b);
                row34_lo = _mm_unpacklo_epi16(row3, row4);
                row3 = _mm_madd_epi16(row34_lo, coeff2_3_8x16b);
                row56_lo = _mm_unpacklo_epi16(row5, row6);
                row5 = _mm_madd_epi16(row56_lo, coeff4_5_8x16b);

                row1 = _mm_add_epi32(row1, row3);
                row1 = _mm_add_epi32(row1, row5);

                row1 = _mm_add_epi32(row1, offset_4x32b);
                row1 = _mm_srai_epi32(row1, shift);
                res0 = _mm_packs_epi32(row1, row1);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res0, mm_max);
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res0);
                inp_copy += stored_alf_para_num; /* pointer updates*/
                dst_copy += dst_stride; /* pointer updates*/
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            inp_copy = ref + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            for (row = 0; row < height; row++)
            {
                for (col = 0; col < rem_w; col++)
                {
                    int val;
                    int sum;
                    sum = inp_copy[col + 0] * coeff[0];
                    sum += inp_copy[col + 1] * coeff[1];
                    sum += inp_copy[col + 2] * coeff[2];
                    sum += inp_copy[col + 3] * coeff[3];
                    sum += inp_copy[col + 4] * coeff[4];
                    sum += inp_copy[col + 5] * coeff[5];

                    val = (sum + offset) >> shift;
                    dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
                }
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
    }
}

static void mc_filter_c_6pel_vert_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, coeff4_5_8x16b, mm_mask;
    __m128i s0_8x16b, s1_8x16b, s2_8x16b, s4_8x16b, s5_8x16b, s6_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b, s2_4_16x8b, s2_5_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_2_16x8b, s3_4_16x8b, s3_5_16x8b, s3_6_16x8b;
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */
    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff4_5_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0xaa);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /* a0 a1 a2 a3 a4 a5 a6 a7 */
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                /* b0 b1 b2 b3 b4 b5 b6 b7 */
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                /* a0 b0 a1 b1 a2 b2 a3 b3 */
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a4 b4 ... a7 b7 */
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a0+b0 a1+b1 a2+b2 a3+b3*/
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /* c0 c1 c2 c3 c4 c5 c6 c7 */
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));
                /* d0 d1 d2 d3 d4 d5 d6 d7 */
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));
                /* c0 d0 c1 d1 c2 d2 c3 d3 */
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                /* c0+d0 c1+d1 c2+d2 c3+d3*/
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);
                s2_4_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 2) + cnt));
                s2_5_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 5) + cnt));
                s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
                s3_6_16x8b = _mm_unpackhi_epi16(s2_4_16x8b, s2_5_16x8b);
                s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);
                s6_8x16b = _mm_madd_epi16(s3_6_16x8b, coeff4_5_8x16b);

                /* a0+b0+c0+d0 ... a3+b3+c3+d3 */
                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, s2_8x16b);
                /* a4+b4+c4+d4 ... a7+b7+c7+d7 */
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, s6_8x16b);

                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);
                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
                }
                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = 0; row < height; row++)
        {
            /*load 6 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));
            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);

            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));
            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);

            s2_4_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (4 * stored_alf_para_num)));
            s2_5_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (5 * stored_alf_para_num)));
            s3_2_16x8b = _mm_unpacklo_epi16(s2_4_16x8b, s2_5_16x8b);
            s2_8x16b = _mm_madd_epi16(s3_2_16x8b, coeff4_5_8x16b);

            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s4_8x16b = _mm_add_epi32(s4_8x16b, s2_8x16b);
            s7_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);
            if (is_last)
            {
                mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                sum += inp_copy[col + 4 * stored_alf_para_num] * coeff[4];
                sum += inp_copy[col + 5 * stored_alf_para_num] * coeff[5];

                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}
#endif

static void mc_filter_c_4pel_horz_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w, rem_h, cnt;
    int src_stride2, src_stride3;
    s16 *inp_copy;
    s16 *dst_copy;
    /* all 128 bit registers are named with a suffix mxnb, where m is the */
    /* number of n bits packed in the register                            */
    __m128i offset_4x32b = _mm_set1_epi32(offset);
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, mm_mask;
    __m128i res0, res1, res2, res3;
    __m128i row11, row12, row13, row14, row21, row22, row23, row24;
    __m128i row31, row32, row33, row34, row41, row42, row43, row44;
    src_stride2 = (stored_alf_para_num << 1);
    src_stride3 = (stored_alf_para_num * 3);
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    {
        rem_h = height & 0x3;
        rem_w = width;
        coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);  /*w2 w3 w2 w3 w2 w3 w2 w3*/
        coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);        /*w0 w1 w0 w1 w0 w1 w0 w1*/
        /* 8 pixels at a time */
        if (rem_w > 7)
        {
            cnt = 0;
            for (row = 0; row < height; row += 4)
            {
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row12 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row13 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row14 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                    /*load pixel values from row 2*/
                    row21 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                    row22 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 1));
                    row23 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 2));
                    row24 = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt + 3));
                    /*load pixel values from row 3*/
                    row31 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt));
                    row32 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 1));
                    row33 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 2));
                    row34 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride2 + cnt + 3));
                    /*load pixel values from row 4*/
                    row41 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt));
                    row42 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 1));
                    row43 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 2));
                    row44 = _mm_loadu_si128((__m128i*)(inp_copy + src_stride3 + cnt + 3));
                    row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                    row12 = _mm_madd_epi16(row12, coeff0_1_8x16b);       /*a1+a2 a3+a4 a5+a6 a7+a8*/
                    row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a4+a5 a6+a7 a8+a9*/
                    row14 = _mm_madd_epi16(row14, coeff2_3_8x16b);       /*a3+a4 a5+a6 a7+a8 a9+a10*/
                    row21 = _mm_madd_epi16(row21, coeff0_1_8x16b);
                    row22 = _mm_madd_epi16(row22, coeff0_1_8x16b);
                    row23 = _mm_madd_epi16(row23, coeff2_3_8x16b);
                    row24 = _mm_madd_epi16(row24, coeff2_3_8x16b);
                    row31 = _mm_madd_epi16(row31, coeff0_1_8x16b);
                    row32 = _mm_madd_epi16(row32, coeff0_1_8x16b);
                    row33 = _mm_madd_epi16(row33, coeff2_3_8x16b);
                    row34 = _mm_madd_epi16(row34, coeff2_3_8x16b);
                    row41 = _mm_madd_epi16(row41, coeff0_1_8x16b);
                    row42 = _mm_madd_epi16(row42, coeff0_1_8x16b);
                    row43 = _mm_madd_epi16(row43, coeff2_3_8x16b);
                    row44 = _mm_madd_epi16(row44, coeff2_3_8x16b);
                    row11 = _mm_add_epi32(row11, row13);
                    row12 = _mm_add_epi32(row12, row14);
                    row21 = _mm_add_epi32(row21, row23);
                    row22 = _mm_add_epi32(row22, row24);
                    row31 = _mm_add_epi32(row31, row33);
                    row32 = _mm_add_epi32(row32, row34);
                    row41 = _mm_add_epi32(row41, row43);
                    row42 = _mm_add_epi32(row42, row44);
                    row11 = _mm_add_epi32(row11, offset_4x32b);
                    row12 = _mm_add_epi32(row12, offset_4x32b);
                    row21 = _mm_add_epi32(row21, offset_4x32b);
                    row22 = _mm_add_epi32(row22, offset_4x32b);
                    row31 = _mm_add_epi32(row31, offset_4x32b);
                    row32 = _mm_add_epi32(row32, offset_4x32b);
                    row41 = _mm_add_epi32(row41, offset_4x32b);
                    row42 = _mm_add_epi32(row42, offset_4x32b);
                    row11 = _mm_srai_epi32(row11, shift);
                    row12 = _mm_srai_epi32(row12, shift);
                    row21 = _mm_srai_epi32(row21, shift);
                    row22 = _mm_srai_epi32(row22, shift);
                    row31 = _mm_srai_epi32(row31, shift);
                    row32 = _mm_srai_epi32(row32, shift);
                    row41 = _mm_srai_epi32(row41, shift);
                    row42 = _mm_srai_epi32(row42, shift);
                    row11 = _mm_packs_epi32(row11, row21);
                    row12 = _mm_packs_epi32(row12, row22);
                    row31 = _mm_packs_epi32(row31, row41);
                    row32 = _mm_packs_epi32(row32, row42);
                    res0 = _mm_unpacklo_epi16(row11, row12);
                    res1 = _mm_unpackhi_epi16(row11, row12);
                    res2 = _mm_unpacklo_epi16(row31, row32);
                    res3 = _mm_unpackhi_epi16(row31, row32);
                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                        mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res1, mm_max);
                        res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                        mm_mask = _mm_cmpgt_epi16(res2, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res2 = _mm_or_si128(_mm_and_si128(mm_mask, res2), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res2, mm_max);
                        res2 = _mm_or_si128(_mm_and_si128(mm_mask, res2), _mm_andnot_si128(mm_mask, mm_max));
                        mm_mask = _mm_cmpgt_epi16(res3, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res3 = _mm_or_si128(_mm_and_si128(mm_mask, res3), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res3, mm_max);
                        res3 = _mm_or_si128(_mm_and_si128(mm_mask, res3), _mm_andnot_si128(mm_mask, mm_max));
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                    _mm_storeu_si128((__m128i *)(dst_copy + dst_stride + cnt), res1);
                    _mm_storeu_si128((__m128i *)(dst_copy + (dst_stride << 1) + cnt), res2);
                    _mm_storeu_si128((__m128i *)(dst_copy + (dst_stride * 3) + cnt), res3);
                    cnt += 8;
                }
                cnt = 0;
                inp_copy += (stored_alf_para_num << 2); /* pointer updates*/
                dst_copy += (dst_stride << 2); /* pointer updates*/
            }
            /*remaining ht */
            for (row = 0; row < rem_h; row++)
            {
                cnt = 0;
                for (col = width; col > 7; col -= 8)
                {
                    /*load pixel values from row 1*/
                    row11 = _mm_loadu_si128((__m128i*)(inp_copy + cnt));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                    row12 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                    row13 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                    row14 = _mm_loadu_si128((__m128i*)(inp_copy + cnt + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                    row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a2+a3 a4+a5 a6+a7*/
                    row12 = _mm_madd_epi16(row12, coeff0_1_8x16b);       /*a1+a2 a3+a4 a5+a6 a7+a8*/
                    row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a4+a5 a6+a7 a8+a9*/
                    row14 = _mm_madd_epi16(row14, coeff2_3_8x16b);       /*a3+a4 a5+a6 a7+a8 a9+a10*/
                    row11 = _mm_add_epi32(row11, row13); /*a0+a1+a2+a3 a2+a3+a4+a5 a4+a5+a6+a7 a6+a7+a8+a9*/
                    row12 = _mm_add_epi32(row12, row14); /*a1+a2+a3+a4 a3+a4+a5+a6 a5+a6+a7+a8 a7+a8+a9+a10*/
                    row11 = _mm_add_epi32(row11, offset_4x32b);
                    row12 = _mm_add_epi32(row12, offset_4x32b);
                    row11 = _mm_srai_epi32(row11, shift);
                    row12 = _mm_srai_epi32(row12, shift);
                    row11 = _mm_packs_epi32(row11, row12);
                    res3 = _mm_unpackhi_epi64(row11, row11);
                    res0 = _mm_unpacklo_epi16(row11, res3);
                    if (is_last)
                    {
                        mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                        mm_mask = _mm_cmplt_epi16(res0, mm_max);
                        res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                    }
                    /* to store the 8 pixels res. */
                    _mm_storeu_si128((__m128i *)(dst_copy + cnt), res0);
                    cnt += 8;
                }
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
        rem_w &= 0x7;
        /* one 4 pixel wd for multiple rows */
        if (rem_w > 3)
        {
            inp_copy = ref + ((width / 8) * 8);
            dst_copy = pred + ((width / 8) * 8);
            for (row = 0; row < height; row += 4)
            {
                /*load pixel values from row 1*/
                row11 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row12 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row13 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row14 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                /*load pixel values from row 2*/
                row21 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num));
                row22 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num + 1));
                row23 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num + 2));
                row24 = _mm_loadl_epi64((__m128i*)(inp_copy + stored_alf_para_num + 3));
                /*load pixel values from row 3*/
                row31 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2));
                row32 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 1));
                row33 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 2));
                row34 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride2 + 3));
                /*load pixel values from row 4*/
                row41 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3));
                row42 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 1));
                row43 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 2));
                row44 = _mm_loadl_epi64((__m128i*)(inp_copy + src_stride3 + 3));
                row11 = _mm_unpacklo_epi32(row11, row12);
                row13 = _mm_unpacklo_epi32(row13, row14);
                row21 = _mm_unpacklo_epi32(row21, row22);
                row23 = _mm_unpacklo_epi32(row23, row24);
                row31 = _mm_unpacklo_epi32(row31, row32);
                row33 = _mm_unpacklo_epi32(row33, row34);
                row41 = _mm_unpacklo_epi32(row41, row42);
                row43 = _mm_unpacklo_epi32(row43, row44);
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);
                row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);
                row21 = _mm_madd_epi16(row21, coeff0_1_8x16b);
                row23 = _mm_madd_epi16(row23, coeff2_3_8x16b);
                row31 = _mm_madd_epi16(row31, coeff0_1_8x16b);
                row33 = _mm_madd_epi16(row33, coeff2_3_8x16b);
                row41 = _mm_madd_epi16(row41, coeff0_1_8x16b);
                row43 = _mm_madd_epi16(row43, coeff2_3_8x16b);
                row11 = _mm_add_epi32(row11, row13);
                row21 = _mm_add_epi32(row21, row23);
                row31 = _mm_add_epi32(row31, row33);
                row41 = _mm_add_epi32(row41, row43);
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row21 = _mm_add_epi32(row21, offset_4x32b);
                row31 = _mm_add_epi32(row31, offset_4x32b);
                row41 = _mm_add_epi32(row41, offset_4x32b);
                row11 = _mm_srai_epi32(row11, shift);
                row21 = _mm_srai_epi32(row21, shift);
                row31 = _mm_srai_epi32(row31, shift);
                row41 = _mm_srai_epi32(row41, shift);
                res0 = _mm_packs_epi32(row11, row21);
                res1 = _mm_packs_epi32(row31, row41);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res0, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res0, mm_max);
                    res0 = _mm_or_si128(_mm_and_si128(mm_mask, res0), _mm_andnot_si128(mm_mask, mm_max));
                    mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res1, mm_max);
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res0);
                _mm_storel_epi64((__m128i *)(dst_copy + dst_stride), _mm_unpackhi_epi64(res0, res0));
                _mm_storel_epi64((__m128i *)(dst_copy + (dst_stride << 1)), res1);
                _mm_storel_epi64((__m128i *)(dst_copy + (dst_stride * 3)), _mm_unpackhi_epi64(res1, res1));
                inp_copy += (stored_alf_para_num << 2); /* pointer updates*/
                dst_copy += (dst_stride << 2); /* pointer updates*/
            }
            for (row = 0; row < rem_h; row++)
            {
                /*load pixel values from row 1*/
                row11 = _mm_loadl_epi64((__m128i*)(inp_copy));            /*a0 a1 a2 a3 a4 a5 a6 a7*/
                row12 = _mm_loadl_epi64((__m128i*)(inp_copy + 1));        /*a1 a2 a3 a4 a5 a6 a7 a8*/
                row13 = _mm_loadl_epi64((__m128i*)(inp_copy + 2));       /*a2 a3 a4 a5 a6 a7 a8 a9*/
                row14 = _mm_loadl_epi64((__m128i*)(inp_copy + 3));        /*a3 a4 a5 a6 a7 a8 a9 a10*/
                row11 = _mm_unpacklo_epi32(row11, row12);        /*a0 a1 a1 a2 a2 a3 a3 a4*/
                row13 = _mm_unpacklo_epi32(row13, row14);        /*a2 a3 a3 a4 a4 a5 a5 a6*/
                row11 = _mm_madd_epi16(row11, coeff0_1_8x16b);    /*a0+a1 a1+a2 a2+a3 a3+a4*/
                row13 = _mm_madd_epi16(row13, coeff2_3_8x16b);       /*a2+a3 a3+a4 a4+a5 a5+a6*/
                row11 = _mm_add_epi32(row11, row13);    /*r00 r01  r02  r03*/
                row11 = _mm_add_epi32(row11, offset_4x32b);
                row11 = _mm_srai_epi32(row11, shift);
                res1 = _mm_packs_epi32(row11, row11);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(res1, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(res1, mm_max);
                    res1 = _mm_or_si128(_mm_and_si128(mm_mask, res1), _mm_andnot_si128(mm_mask, mm_max));
                }
                /* to store the 8 pixels res. */
                _mm_storel_epi64((__m128i *)(dst_copy), res1);
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
        rem_w &= 0x3;
        if (rem_w)
        {
            inp_copy = ref + ((width / 4) * 4);
            dst_copy = pred + ((width / 4) * 4);
            for (row = 0; row < height; row++)
            {
                for (col = 0; col < rem_w; col++)
                {
                    int val;
                    int sum;
                    sum = inp_copy[col + 0] * coeff[0];
                    sum += inp_copy[col + 1] * coeff[1];
                    sum += inp_copy[col + 2] * coeff[2];
                    sum += inp_copy[col + 3] * coeff[3];
                    val = (sum + offset) >> shift;
                    dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
                }
                inp_copy += (stored_alf_para_num); /* pointer updates*/
                dst_copy += (dst_stride); /* pointer updates*/
            }
        }
    }
}

static void mc_filter_c_4pel_vert_sse
(
    s16 *ref,
    int stored_alf_para_num,
    s16 *pred,
    int dst_stride,
    const s16 *coeff,
    int width,
    int height,
    int min_val,
    int max_val,
    int offset,
    int shift,
    s8  is_last)
{
    int row, col, rem_w;
    s16 const *src_tmp;
    s16 const *inp_copy;
    s16 *dst_copy;
    __m128i coeff0_1_8x16b, coeff2_3_8x16b, mm_mask;
    __m128i s0_8x16b, s1_8x16b, s4_8x16b, s5_8x16b, s7_8x16b, s8_8x16b, s9_8x16b;
    __m128i s2_0_16x8b, s2_1_16x8b, s2_2_16x8b, s2_3_16x8b;
    __m128i s3_0_16x8b, s3_1_16x8b, s3_4_16x8b, s3_5_16x8b;
    __m128i mm_min = _mm_set1_epi16((short)min_val);
    __m128i mm_max = _mm_set1_epi16((short)max_val);
    __m128i offset_8x16b = _mm_set1_epi32(offset); /* for offset addition */
    src_tmp = ref;
    rem_w = width;
    inp_copy = ref;
    dst_copy = pred;
    /* load 8 8-bit coefficients and convert 8-bit into 16-bit  */
    coeff0_1_8x16b = _mm_loadu_si128((__m128i*)coeff);
    coeff2_3_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0x55);
    coeff0_1_8x16b = _mm_shuffle_epi32(coeff0_1_8x16b, 0);
    if (rem_w > 7)
    {
        for (row = 0; row < height; row++)
        {
            int cnt = 0;
            for (col = width; col > 7; col -= 8)
            {
                /* a0 a1 a2 a3 a4 a5 a6 a7 */
                s2_0_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + cnt));
                /* b0 b1 b2 b3 b4 b5 b6 b7 */
                s2_1_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + stored_alf_para_num + cnt));
                /* a0 b0 a1 b1 a2 b2 a3 b3 */
                s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a4 b4 ... a7 b7 */
                s3_4_16x8b = _mm_unpackhi_epi16(s2_0_16x8b, s2_1_16x8b);
                /* a0+b0 a1+b1 a2+b2 a3+b3*/
                s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
                s4_8x16b = _mm_madd_epi16(s3_4_16x8b, coeff0_1_8x16b);
                /* c0 c1 c2 c3 c4 c5 c6 c7 */
                s2_2_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num << 1) + cnt));
                /* d0 d1 d2 d3 d4 d5 d6 d7 */
                s2_3_16x8b = _mm_loadu_si128((__m128i*)(inp_copy + (stored_alf_para_num * 3) + cnt));
                /* c0 d0 c1 d1 c2 d2 c3 d3 */
                s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
                s3_5_16x8b = _mm_unpackhi_epi16(s2_2_16x8b, s2_3_16x8b);
                /* c0+d0 c1+d1 c2+d2 c3+d3*/
                s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
                s5_8x16b = _mm_madd_epi16(s3_5_16x8b, coeff2_3_8x16b);
                /* a0+b0+c0+d0 ... a3+b3+c3+d3 */
                s0_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
                /* a4+b4+c4+d4 ... a7+b7+c7+d7 */
                s4_8x16b = _mm_add_epi32(s4_8x16b, s5_8x16b);
                s0_8x16b = _mm_add_epi32(s0_8x16b, offset_8x16b);
                s4_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
                s7_8x16b = _mm_srai_epi32(s0_8x16b, shift);
                s8_8x16b = _mm_srai_epi32(s4_8x16b, shift);
                s9_8x16b = _mm_packs_epi32(s7_8x16b, s8_8x16b);
                if (is_last)
                {
                    mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                    mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                    s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
                }
                _mm_storeu_si128((__m128i*)(dst_copy + cnt), s9_8x16b);
                cnt += 8;
            }
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x7;
    if (rem_w > 3)
    {
        inp_copy = src_tmp + ((width / 8) * 8);
        dst_copy = pred + ((width / 8) * 8);
        for (row = 0; row < height; row++)
        {
            /*load 4 pixel values */
            s2_0_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy));
            /*load 4 pixel values */
            s2_1_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (stored_alf_para_num)));
            s3_0_16x8b = _mm_unpacklo_epi16(s2_0_16x8b, s2_1_16x8b);
            s0_8x16b = _mm_madd_epi16(s3_0_16x8b, coeff0_1_8x16b);
            /*load 4 pixel values*/
            s2_2_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (2 * stored_alf_para_num)));
            /*load 4 pixel values*/
            s2_3_16x8b = _mm_loadl_epi64((__m128i*)(inp_copy + (3 * stored_alf_para_num)));
            s3_1_16x8b = _mm_unpacklo_epi16(s2_2_16x8b, s2_3_16x8b);
            s1_8x16b = _mm_madd_epi16(s3_1_16x8b, coeff2_3_8x16b);
            s4_8x16b = _mm_add_epi32(s0_8x16b, s1_8x16b);
            s7_8x16b = _mm_add_epi32(s4_8x16b, offset_8x16b);
            s8_8x16b = _mm_srai_epi32(s7_8x16b, shift);
            s9_8x16b = _mm_packs_epi32(s8_8x16b, s8_8x16b);
            if (is_last)
            {
                mm_mask = _mm_cmpgt_epi16(s9_8x16b, mm_min);  /*if gt = -1...  -1 -1 0 0 -1 */
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_min));
                mm_mask = _mm_cmplt_epi16(s9_8x16b, mm_max);
                s9_8x16b = _mm_or_si128(_mm_and_si128(mm_mask, s9_8x16b), _mm_andnot_si128(mm_mask, mm_max));
            }
            _mm_storel_epi64((__m128i*)(dst_copy), s9_8x16b);
            inp_copy += (stored_alf_para_num);
            dst_copy += (dst_stride);
        }
    }
    rem_w &= 0x3;
    if (rem_w)
    {
        inp_copy = src_tmp + ((width / 4) * 4);
        dst_copy = pred + ((width / 4) * 4);
        for (row = 0; row < height; row++)
        {
            for (col = 0; col < rem_w; col++)
            {
                int val;
                int sum;
                sum = inp_copy[col + 0 * stored_alf_para_num] * coeff[0];
                sum += inp_copy[col + 1 * stored_alf_para_num] * coeff[1];
                sum += inp_copy[col + 2 * stored_alf_para_num] * coeff[2];
                sum += inp_copy[col + 3 * stored_alf_para_num] * coeff[3];
                val = (sum + offset) >> shift;
                dst_copy[col] = (s16)(is_last ? (COM_CLIP3(min_val, max_val, val)) : val);
            }
            inp_copy += stored_alf_para_num;
            dst_copy += dst_stride;
        }
    }
}
#endif
#if OBMC_TEMP
void com_mc_obmc_up(pel *ref, int gmv_x, int gmv_y, int s_ref, pel *pred, int w, int bit_depth)
{
    int  j;
    ref += gmv_y * s_ref + gmv_x;
    for(j=0; j<w; j++)
    {
        pred[j] = ref[j];
    }
}
void com_mc_obmc_left(pel *ref, int gmv_x, int gmv_y, int s_ref,pel *pred,int h, int bit_depth)
{
    int i=0;
    ref += gmv_y * s_ref + gmv_x;
    for(i=0; i<h; i++)
    {
        pred[i] = ref[i*s_ref];
    }    
}
#endif
void com_mc_l_00(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if DMVR
    , int is_dmvr
#endif
)
{
    int i, j;
    if (is_half_pel_filter)
    {
        gmv_x >>= 4;
        gmv_y >>= 4;
    }
    else
    {
        gmv_x >>= 2;
        gmv_y >>= 2;
    }
#if DMVR
    if (!is_dmvr)
    {
#endif
        ref += gmv_y * s_ref + gmv_x;
#if DMVR
    }
#endif
#if SIMD_MC
    if(((w & 0x7)==0) && ((h & 1)==0))
    {
        __m128i m00, m01;
        for(i=0; i<h; i+=2)
        {
            for(j=0; j<w; j+=8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));
                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref  += s_ref * 2;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i m00;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else
#endif
    {
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
}

void com_mc_l_n0(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if DMVR
    , int is_dmvr
#endif
)
{
#if IF_LUMA12_CHROMA6
    const int offset = 5;
#endif
    int dx;
    if (is_half_pel_filter)
    {
        dx = gmv_x & 15;
#if DMVR
        if (is_dmvr)
        {
#if IF_LUMA12_CHROMA6
            ref = ref - offset;
#else
            ref = ref - 3;
#endif
        }
        else
        {
#endif
#if IF_LUMA12_CHROMA6
            ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4) - offset;
#else
            ref += (gmv_y >> 4) * s_ref + (gmv_x >> 4) - 3;
#endif
#if DMVR
        }
#endif
    }
    else
    {
        dx = gmv_x & 0x3;
#if DMVR
        if (is_dmvr)
        {
#if IF_LUMA12_CHROMA6
            ref = ref - offset;
#else
            ref = ref - 3;
#endif
        }
        else
        {
#endif
#if IF_LUMA12_CHROMA6
            ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2) - offset;
#else
            ref += (gmv_y >> 2) * s_ref + (gmv_x >> 2) - 3;
#endif
#if DMVR
        }
#endif
    }
#if IF_LUMA12_CHROMA6
    const s16* coeff_hor = is_half_pel_filter ? tbl_mc_l_coeff_hp_12tap[dx] : tbl_mc_l_coeff_12tap[dx];
#if IF_LUMA12_CHROMA6_SIMD
    int max = ((1 << bit_depth) - 1);
    int min = 0;
    mc_filter_l_12pel_horz_clip_sse(ref, s_ref, pred, s_pred, coeff_hor, w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
#else
    {
        int i, j;
        s16 pt;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                pt = (MAC_12TAP(coeff_hor, (s32)ref[j], (s32)ref[j + 1], (s32)ref[j + 2], (s32)ref[j + 3], (s32)ref[j + 4], (s32)ref[j + 5], (s32)ref[j + 6], (s32)ref[j + 7], (s32)ref[j + 8], (s32)ref[j + 9], (s32)ref[j + 10], (s32)ref[j + 11]) + MAC_ADD_N0) >> MAC_SFT_N0;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref += s_ref;
            pred += s_pred;
        }
    }
#endif
#else
    const s16 *coeff_hor = is_half_pel_filter ? tbl_mc_l_coeff_hp[dx] : tbl_mc_l_coeff[dx];
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_l_8pel_horz_clip_sse(ref, s_ref, pred, s_pred, coeff_hor, w, h, min, max,
                                       MAC_ADD_N0, MAC_SFT_N0);
    }
#else
    {
        int i, j;
        s32 pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_8TAP(coeff_hor, ref[j], ref[j + 1], ref[j + 2], ref[j + 3],    ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + MAC_ADD_N0) >> MAC_SFT_N0;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref  += s_ref;
            pred += s_pred;
        }
    }
#endif
#endif
}

void com_mc_l_0n(pel *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, pel *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if DMVR
    , int is_dmvr
#endif
)
{
#if IF_LUMA12_CHROMA6
    const int offset = 5;
#endif
    int dy;

    if (is_half_pel_filter)
    {
        dy = gmv_y & 15;
#if DMVR
        if (is_dmvr)
        {
#if IF_LUMA12_CHROMA6
            ref = ref - (offset * s_ref);
#else
            ref = ref - (3 * s_ref);
#endif
        }
        else
        {
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 4) - offset) * s_ref + (gmv_x >> 4);
#else
            ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4);
#endif
#if DMVR
        }
#endif
    }
    else
    {
        dy = gmv_y & 0x3;
#if DMVR
        if (is_dmvr)
        {
#if IF_LUMA12_CHROMA6
            ref = ref - (offset * s_ref);
#else
            ref = ref - (3 * s_ref);
#endif
        }
        else
        {
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 2) - offset) * s_ref + (gmv_x >> 2);
#else
            ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2);
#endif
#if DMVR
        }
#endif
    }
#if IF_LUMA12_CHROMA6
    const s16* coeff_ver = is_half_pel_filter ? tbl_mc_l_coeff_hp_12tap[dy] : tbl_mc_l_coeff_12tap[dy];
#if IF_LUMA12_CHROMA6_SIMD
    int max = ((1 << bit_depth) - 1);
    int min = 0;
    mc_filter_l_12pel_vert_clip_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
#else
    {
        int i, j;
        s16 pt;

        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                pt = (MAC_12TAP(coeff_ver, (s32)ref[j], (s32)ref[s_ref + j], (s32)ref[s_ref * 2 + j], (s32)ref[s_ref * 3 + j], (s32)ref[s_ref * 4 + j], (s32)ref[s_ref * 5 + j], (s32)ref[s_ref * 6 + j], ref[s_ref * 7 + j], (s32)ref[s_ref * 8 + j], (s32)ref[s_ref * 9 + j], (s32)ref[s_ref * 10 + j], (s32)ref[s_ref * 11 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref += s_ref;
            pred += s_pred;
        }
    }
#endif
#else
    const s16 *coeff_ver = is_half_pel_filter ? tbl_mc_l_coeff_hp[dy] : tbl_mc_l_coeff[dy];
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_l_8pel_vert_clip_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max,
                                       MAC_ADD_0N, MAC_SFT_0N);
    }
#else
    {
        int i, j;
        s32 pt;

        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_8TAP(coeff_ver, ref[j], ref[s_ref + j], ref[s_ref*2 + j], ref[s_ref*3 + j], ref[s_ref*4 + j], ref[s_ref*5 + j], ref[s_ref*6 + j], ref[s_ref*7 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref  += s_ref;
            pred += s_pred;
        }
    }
#endif
#endif
}

void com_mc_l_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if DMVR
    , int is_dmvr
#endif
)
{
#if IF_LUMA12_CHROMA6
    const int offset = 5;
#endif
    static s16 buf[(MAX_CU_SIZE + MC_IBUF_PAD_L)*MAX_CU_SIZE];
    int        dx, dy;

    if (is_half_pel_filter)
    {
        dx = gmv_x & 15;
        dy = gmv_y & 15;
#if DMVR
        if (is_dmvr)
        {
#if IF_LUMA12_CHROMA6
            ref = ref - (offset * s_ref + offset);
#else
            ref = ref - (3 * s_ref + 3);
#endif
        }
        else
        {
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 4) - offset) * s_ref + (gmv_x >> 4) - offset;
#else
            ref += ((gmv_y >> 4) - 3) * s_ref + (gmv_x >> 4) - 3;
#endif
#if DMVR
        }
#endif
    }
    else
    {
        dx = gmv_x & 0x3;
        dy = gmv_y & 0x3;
#if DMVR
        if (is_dmvr)
        {
#if IF_LUMA12_CHROMA6
            ref = ref - (offset * s_ref + offset);
#else
            ref = ref - (3 * s_ref + 3);
#endif
        }
        else
        {
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 2) - offset) * s_ref + (gmv_x >> 2) - offset;
#else
            ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
#endif
#if DMVR
        }
#endif
    }
#if IF_LUMA12_CHROMA6
    const s16* coeff_hor = is_half_pel_filter ? tbl_mc_l_coeff_hp_12tap[dx] : tbl_mc_l_coeff_12tap[dx];
    const s16* coeff_ver = is_half_pel_filter ? tbl_mc_l_coeff_hp_12tap[dy] : tbl_mc_l_coeff_12tap[dy];

    const int shift1 = bit_depth - 6;
    const int shift2 = 22 - bit_depth; 
    const s32 add1 = (1 << shift1) >> 1;
    const s32 add2 = 1 << (shift2 - 1);
#if IF_LUMA12_CHROMA6_SIMD
    int max = ((1 << bit_depth) - 1);
    int min = 0;

    mc_filter_l_12pel_horz_clip_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 11), min, max, add1, shift1, 0);
    mc_filter_l_12pel_vert_clip_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2, 1);
#else
    {
        int i, j;
        s16* b;
        s16 pt;
        b = buf;
        s16* ref0 = ref;
        for (i = 0; i < h + 11; i++)
        {
            for (j = 0; j < w; j++)
            {
                b[j] = (MAC_12TAP(coeff_hor, (s32)ref[j], (s32)ref[j + 1], (s32)ref[j + 2], (s32)ref[j + 3], (s32)ref[j + 4], (s32)ref[j + 5], (s32)ref[j + 6], (s32)ref[j + 7], (s32)ref[j + 8], (s32)ref[j + 9], (s32)ref[j + 10], (s32)ref[j + 11]) + add1) >> shift1;
            }
            ref += s_ref;
            b += w;
        }
        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                pt = (MAC_12TAP(coeff_ver, (s32)b[j], (s32)b[j + w], (s32)b[j + w * 2], (s32)b[j + w * 3], (s32)b[j + w * 4], (s32)b[j + w * 5], (s32)b[j + w * 6], (s32)b[j + w * 7], (s32)b[j + w * 8], (s32)b[j + w * 9], (s32)b[j + w * 10], (s32)b[j + w * 11]) + add2) >> shift2;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            b += w;
        }
    }
#endif
#else
    const s16 *coeff_hor = is_half_pel_filter ? tbl_mc_l_coeff_hp[dx] : tbl_mc_l_coeff[dx];
    const s16 *coeff_ver = is_half_pel_filter ? tbl_mc_l_coeff_hp[dy] : tbl_mc_l_coeff[dy];

    const int shift1 = bit_depth - 8;
    const int shift2 = 20 - bit_depth;
    const int add1 = (1 << shift1) >> 1;
    const int add2 = 1 << (shift2 - 1);
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 7), add1, shift1);
        mc_filter_l_8pel_vert_clip_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2);
    }
#else
    {
        int i, j;
        s16 * b;
        s32 pt;
        b = buf;

        for(i=0; i<h+7; i++)
        {
            for(j=0; j<w; j++)
            {
                b[j] = (MAC_8TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3], ref[j+4], ref[j+5], ref[j+6], ref[j+7]) + add1) >> shift1;
            }
            ref  += s_ref;
            b     += w;
        }
        b = buf;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_8TAP(coeff_ver, b[j], b[j+w], b[j+w*2], b[j+w*3], b[j+w*4], b[j+w*5], b[j+w*6], b[j+w*7]) + add2) >> shift2;
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            b     += w;
        }
    }
#endif
#endif
}

#if INTER_TM
void com_mc_tm_l_00(pel *ref, pel *pred, int s_ref, int s_pred, int init_gmv_x, int init_gmv_y, int gmv_x, int gmv_y, int w, int h, int tm_size, int bit_depth)
{
    int i, j;

    gmv_x >>= 2;
    gmv_y >>= 2;
    ref += gmv_y * s_ref + gmv_x;

#if SIMD_MC
    if(((w & 0x7)==0) && ((h & 1)==0))
    {
        __m128i m00, m01;
        for(i=0; i<h; i+=2)
        {
            for(j=0; j<w; j+=8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));
                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref  += s_ref * 2;
        }
    }
    else if((w & 0x3)==0)
    {
        __m128i m00;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else
#endif
    {
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
}

void com_mc_tm_l_n0(pel *ref, pel *pred, int s_ref, int s_pred, int init_gmv_x, int init_gmv_y, int gmv_x, int gmv_y, int w, int h, BOOL is_search, int bit_depth)
{
    const int tm_size = (SIMPLIFIED_MAX_TM_ITERATIONS >> 1) + 1;
    u16 offset = is_search ? 2 : 5;
    int w_buf = w + tm_size * 2 + (2*offset + 1), h_buf = h + tm_size * 2 + (2*offset + 1);
    pel* img_pad = (pel*)malloc(w_buf * h_buf * sizeof(pel));
    ref += ((init_gmv_y >> 2) - tm_size) * s_ref + ((init_gmv_x >> 2) - tm_size);
    tm_padding(img_pad, ref, s_ref, w_buf, h_buf, offset);
    int gmv_y_offset = (gmv_y - init_gmv_y) >> 2;
    int gmv_x_offset = (gmv_x - init_gmv_x) >> 2;
    pel* ref_buf = img_pad;
    ref_buf += (gmv_y_offset + tm_size + offset) * w_buf + gmv_x_offset + tm_size;
    int dx = gmv_x & 0x3;
    const s16 *coeff_hor_12tap = tbl_mc_l_coeff_12tap[dx];
    const s16 *coeff_hor_6tap = tbl_mc_c_coeff_6tap[dx*2];
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        if (!is_search)
        {
            mc_filter_l_12pel_horz_clip_sse(ref_buf, w_buf, pred, s_pred, coeff_hor_12tap, w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
        }
        else
        {
            mc_filter_c_6pel_horz_sse(ref_buf, w_buf, pred, s_pred, coeff_hor_6tap, w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
        }
    }
#else
    {
        int i, j;
        s16 pt;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                if (!is_search)
                {
                    pt = (MAC_12TAP(coeff_hor_12tap, (s32)ref_buf[j], (s32)ref_buf[j + 1], (s32)ref_buf[j + 2], (s32)ref_buf[j + 3], (s32)ref_buf[j + 4], (s32)ref_buf[j + 5], (s32)ref_buf[j + 6], (s32)ref_buf[j + 7], (s32)ref_buf[j + 8], (s32)ref_buf[j + 9], (s32)ref_buf[j + 10], (s32)ref_buf[j + 11]) + MAC_ADD_N0) >> MAC_SFT_N0;
                }
                else
                {
                    pt = (MAC_6TAP(coeff_hor_6tap, (s32)ref_buf[j], (s32)ref_buf[j + 1], (s32)ref_buf[j + 2], (s32)ref_buf[j + 3], (s32)ref_buf[j + 4], (s32)ref_buf[j + 5]) + MAC_ADD_N0) >> MAC_SFT_N0;
                }
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref_buf += w_buf;
            pred += s_pred;
        }
    }
#endif
    free(img_pad);
}

void com_mc_tm_l_0n(pel *ref, pel *pred, int s_ref, int s_pred, int init_gmv_x, int init_gmv_y, int gmv_x, int gmv_y, int w, int h, BOOL is_search, int bit_depth)
{
    const int tm_size = (SIMPLIFIED_MAX_TM_ITERATIONS >> 1) + 1;
    u16 offset = is_search ? 2 : 5;
    int w_buf = w + tm_size * 2 + (2*offset + 1), h_buf = h + tm_size * 2 + (2*offset + 1);
    pel* img_pad = (pel*)malloc(w_buf * h_buf * sizeof(pel));
    ref += ((init_gmv_y >> 2) - tm_size) * s_ref + ((init_gmv_x >> 2) - tm_size);
    tm_padding(img_pad, ref, s_ref, w_buf, h_buf, offset);
    int gmv_y_offset = (gmv_y - init_gmv_y) >> 2;
    int gmv_x_offset = (gmv_x - init_gmv_x) >> 2;
    pel* ref_buf = img_pad;
    ref_buf += (gmv_y_offset + tm_size) * w_buf + gmv_x_offset + tm_size + offset;

    int dy = gmv_y & 0x3;
    const s16* coeff_ver_12tap = tbl_mc_l_coeff_12tap[dy];
    const s16* coeff_ver_6tap = tbl_mc_c_coeff_6tap[dy*2];
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        if (!is_search)
        {
            mc_filter_l_12pel_vert_clip_sse(ref_buf, w_buf, pred, s_pred, coeff_ver_12tap, w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
        }
        else
        {
            mc_filter_c_6pel_vert_sse(ref_buf, w_buf, pred, s_pred, coeff_ver_6tap, w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
        }
    }
#else
    {
        int i, j;
        s32 pt;

        for(i = 0; i < h; i++)
        {
            for(j = 0; j < w; j++)
            {
                if (!is_search)
                {
                    pt = (MAC_12TAP(coeff_ver_12tap, ref_buf[j], ref_buf[w_buf + j], ref_buf[w_buf*2 + j], ref_buf[w_buf*3 + j], ref_buf[w_buf*4 + j], ref_buf[w_buf*5 + j], ref_buf[w_buf*6 + j], ref_buf[w_buf*7 + j], ref_buf[w_buf*8 + j], ref_buf[w_buf*9 + j], ref_buf[w_buf*10 + j], ref_buf[w_buf*11 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                }
                else
                {
                    pt = (MAC_6TAP(coeff_ver_6tap, ref_buf[j], ref_buf[w_buf + j], ref_buf[w_buf*2 + j], ref_buf[w_buf*3 + j], ref_buf[w_buf*4 + j], ref_buf[w_buf*5 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                }
                pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            ref_buf += w_buf;
            pred += s_pred;
        }
    }
#endif
    free(img_pad);
}

void com_mc_tm_l_nn(pel *ref, pel *pred, int s_ref, int s_pred, int init_gmv_x, int init_gmv_y, int gmv_x, int gmv_y, int w, int h, BOOL is_search, int bit_depth)
{
    const int tm_size = (SIMPLIFIED_MAX_TM_ITERATIONS >> 1) + 1;
    u16 offset = is_search ? 2 : 5;
    int w_buf = w + tm_size * 2 + (2*offset + 1), h_buf = h + tm_size * 2 + (2*offset + 1);
    pel* img_pad = (pel*)malloc(w_buf * h_buf * sizeof(pel));
    ref += ((init_gmv_y >> 2) - tm_size) * s_ref + ((init_gmv_x >> 2) - tm_size);
    tm_padding(img_pad, ref, s_ref, w_buf, h_buf, offset);
    int gmv_y_offset = (gmv_y - init_gmv_y) >> 2;
    int gmv_x_offset = (gmv_x - init_gmv_x) >> 2;
    pel* ref_buf = img_pad;
    ref_buf += (gmv_y_offset + tm_size) * w_buf + gmv_x_offset + tm_size;

    static s16 buf[(MAX_CU_SIZE + MC_IBUF_PAD_L)*MAX_CU_SIZE];
    int dx = gmv_x & 0x3;
    int dy = gmv_y & 0x3;
    const s16 *coeff_hor_12tap = tbl_mc_l_coeff_12tap[dx];
    const s16 *coeff_ver_12tap = tbl_mc_l_coeff_12tap[dy];
    const s16 *coeff_hor_6tap = tbl_mc_c_coeff_6tap[dx*2];
    const s16 *coeff_ver_6tap = tbl_mc_c_coeff_6tap[dy*2];

    const int shift1 = bit_depth - 6;
    const int shift2 = 22 - bit_depth; 
    const s32 add1 = (1 << shift1) >> 1;
    const s32 add2 = 1 << (shift2 - 1);
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        if (!is_search)
        {
            mc_filter_l_12pel_horz_clip_sse(ref_buf, w_buf, buf, w, coeff_hor_12tap, w, (h + 11), min, max, add1, shift1, 0);
            mc_filter_l_12pel_vert_clip_sse(buf, w, pred, s_pred, coeff_ver_12tap, w, h, min, max, add2, shift2, 1);
        }
        else
        {
            mc_filter_c_6pel_horz_sse(ref_buf, w_buf, buf, w, coeff_hor_6tap, w, (h + 5), min, max, add1, shift1, 0);
            mc_filter_c_6pel_vert_sse(buf, w, pred, s_pred, coeff_ver_6tap, w, h, min, max, add2, shift2, 1);
        }
    }
#else
    {
        int i, j;
        s16* b;
        s16 pt;
        b = buf;
        if (!is_search)
        {
            for (i = 0; i < h + 11; i++)
            {
                for (j = 0; j < w; j++)
                {
                    b[j] = (MAC_12TAP(coeff_hor_12tap, (s32)ref_buf[j], (s32)ref_buf[j + 1], (s32)ref_buf[j + 2], (s32)ref_buf[j + 3], (s32)ref_buf[j + 4], (s32)ref_buf[j + 5], (s32)ref_buf[j + 6], (s32)ref_buf[j + 7], (s32)ref_buf[j + 8], (s32)ref_buf[j + 9], (s32)ref_buf[j + 10], (s32)ref_buf[j + 11]) + add1) >> shift1;
                }
                ref_buf += w_buf;
                b     += w;
            }
            b = buf;
            for (i = 0; i < h; i++)
            {
                for (j = 0; j < w; j++)
                {
                    pt = (MAC_12TAP(coeff_ver_12tap, (s32)b[j], (s32)b[j + w], (s32)b[j + w * 2], (s32)b[j + w * 3], (s32)b[j + w * 4], (s32)b[j + w * 5], (s32)b[j + w * 6], (s32)b[j + w * 7], (s32)b[j + w * 8], (s32)b[j + w * 9], (s32)b[j + w * 10], (s32)b[j + w * 11]) + add2) >> shift2;
                    pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
                }
                pred += s_pred;
                b += w;
            }
        }
        else
        {
            for (i = 0; i < h + 5; i++)
            {
                for (j = 0; j < w; j++)
                {
                    b[j] = (MAC_6TAP(coeff_hor_6tap, (s32)ref_buf[j], (s32)ref_buf[j + 1], (s32)ref_buf[j + 2], (s32)ref_buf[j + 3], (s32)ref_buf[j + 4], (s32)ref_buf[j + 5]) + add1) >> shift1;
                }
                ref_buf += w_buf;
                b     += w;
            }
            b = buf;
            for (i = 0; i < h; i++)
            {
                for (j = 0; j < w; j++)
                {
                    pt = (MAC_6TAP(coeff_ver_6tap, (s32)b[j], (s32)b[j + w], (s32)b[j + w * 2], (s32)b[j + w * 3], (s32)b[j + w * 4], (s32)b[j + w * 5]) + add2) >> shift2;
                    pred[j] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
                }
                pred += s_pred;
                b += w;
            }
        }

    }
#endif
    free(img_pad);
}
#endif

/****************************************************************************
 * motion compensation for chroma
 ****************************************************************************/

void com_mc_c_00(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if USE_IBC
    , int is_ibc
#endif
#if DMVR
    , int is_dmvr
#endif
)
{
    int i, j;
    if (is_half_pel_filter)
    {
        gmv_x >>= 5;
        gmv_y >>= 5;
    }
    else
    {
        gmv_x >>= 3;
        gmv_y >>= 3;
    }
#if DMVR
    if (!is_dmvr)
    {
#endif
        ref += gmv_y * s_ref + gmv_x;
#if DMVR
    }
#endif
#if SIMD_MC
    if(((w & 0x7)==0) && ((h & 1)==0))
    {
        __m128i m00, m01;
        for(i=0; i<h; i+=2)
        {
            for(j=0; j<w; j+=8)
            {
                m00 = _mm_loadu_si128((__m128i*)(ref + j));
                m01 = _mm_loadu_si128((__m128i*)(ref + j + s_ref));
                _mm_storeu_si128((__m128i*)(pred + j), m00);
                _mm_storeu_si128((__m128i*)(pred + j + s_pred), m01);
            }
            pred += s_pred * 2;
            ref  += s_ref * 2;
        }
    }
    else if(((w & 0x3)==0))
    {
        __m128i m00;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j+=4)
            {
                m00 = _mm_loadl_epi64((__m128i*)(ref + j));
                _mm_storel_epi64((__m128i*)(pred + j), m00);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
    else
#endif
    {
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pred[j] = ref[j];
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
}

void com_mc_c_n0(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if USE_IBC
    , int is_ibc
#endif
#if DMVR
    , int is_dmvr
#endif
)
{
#if IF_LUMA12_CHROMA6
    int offset = 2;
    if (is_ibc)
    {
        offset = 1;
    }
#endif
    int dx;
    if (is_half_pel_filter)
    {
        dx = gmv_x & 31;
#if DMVR
        if (is_dmvr)
#if IF_LUMA12_CHROMA6
            ref -= offset;
#else
            ref -= 1;
#endif
        else
#endif
#if IF_LUMA12_CHROMA6
            ref += (gmv_y >> 5) * s_ref + (gmv_x >> 5) - offset;
#else
            ref += (gmv_y >> 5) * s_ref + (gmv_x >> 5) - 1;
#endif
    }
    else
    {
        dx = gmv_x & 0x7;
#if DMVR
        if (is_dmvr)
#if IF_LUMA12_CHROMA6
            ref -= offset;
#else
            ref -= 1;
#endif
        else
#endif
#if IF_LUMA12_CHROMA6
            ref += (gmv_y >> 3) * s_ref + (gmv_x >> 3) - offset;
#else
            ref += (gmv_y >> 3) * s_ref + (gmv_x >> 3) - 1;
#endif
    }
#if USE_IBC
#if DMVR
#if IF_LUMA12_CHROMA6
    const s16 *coeff_hor = is_ibc? tbl_mc_c_coeff_ibc[dx] : ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp_6tap[dx] : tbl_mc_c_coeff_6tap[dx]);
#else
    const s16 *coeff_hor = is_ibc? tbl_mc_c_coeff_ibc[dx] : ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
#endif
#else
    const s16 *coeff_hor = is_ibc? tbl_mc_c_coeff_ibc[dx] : (is_half_pel_filter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
#endif
#else
#if DMVR
    const s16 *coeff_hor = (is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
#else
    const s16 *coeff_hor = is_half_pel_filter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
#endif
#endif

#if IF_LUMA12_CHROMA6
#if IF_LUMA12_CHROMA6_SIMD
    int max = ((1 << bit_depth) - 1);
    int min = 0;
    is_ibc ? mc_filter_c_4pel_horz_sse(ref, s_ref, pred, s_pred, coeff_hor, w, h, min, max, (1<<5), 6, 1) : mc_filter_c_6pel_horz_sse(ref, s_ref, pred, s_pred, coeff_hor, w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
#else
    {
        int       i, j;
        s16       pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                if (is_ibc)
                {
                    pt = (MAC_4TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3]) + (1<<5)) >> 6;
                }
                else
                {
                    pt = (MAC_6TAP(coeff_hor, (s32)ref[j], (s32)ref[j+1], (s32)ref[j+2], (s32)ref[j+3], (s32)ref[j+4], (s32)ref[j+5]) + MAC_ADD_N0) >> MAC_SFT_N0;
                }
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
#else
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_c_4pel_horz_sse(ref, s_ref, pred, s_pred, coeff_hor,
                                  w, h, min, max, MAC_ADD_N0, MAC_SFT_N0, 1);
    }
#else
    {
        int       i, j;
        s32       pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_4TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3]) + MAC_ADD_N0) >> MAC_SFT_N0;
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
#endif
}

void com_mc_c_0n(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if USE_IBC
    , int is_ibc
#endif
#if DMVR
    , int is_dmvr
#endif
)
{
#if IF_LUMA12_CHROMA6
    int offset = 2;
    if (is_ibc)
    {
        offset = 1;
    }
#endif
    int dy;
    if (is_half_pel_filter)
    {
        dy = gmv_y & 31;
#if DMVR
        if( is_dmvr )
#if IF_LUMA12_CHROMA6
            ref -= offset * s_ref;
#else
            ref -= 1 * s_ref;
#endif
        else
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 5) - offset) * s_ref + (gmv_x >> 5);
#else
            ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5);
#endif
    }
    else
    {
        dy = gmv_y & 0x7;
#if DMVR
        if (is_dmvr)
#if IF_LUMA12_CHROMA6
            ref -= offset * s_ref;
#else
            ref -= 1 * s_ref;
#endif
        else
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 3) - offset) * s_ref + (gmv_x >> 3);
#else
            ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3);
#endif
    }
#if USE_IBC
#if DMVR
#if IF_LUMA12_CHROMA6
    const s16 *coeff_ver = is_ibc? tbl_mc_c_coeff_ibc[dy]: ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp_6tap[dy] : tbl_mc_c_coeff_6tap[dy]);
#else
    const s16 *coeff_ver = is_ibc? tbl_mc_c_coeff_ibc[dy]: ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#endif
#else
    const s16 *coeff_ver = is_ibc? tbl_mc_c_coeff_ibc[dy]: (is_half_pel_filter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#endif
#else
#if DMVR
    const s16 *coeff_ver = (is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#else
    const s16 *coeff_ver = is_half_pel_filter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#endif
#endif

#if IF_LUMA12_CHROMA6
#if IF_LUMA12_CHROMA6_SIMD
    int max = ((1 << bit_depth) - 1);
    int min = 0;
    is_ibc ? mc_filter_c_4pel_vert_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max, (1<<5), 6, 1) : mc_filter_c_6pel_vert_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
#else
    {
        int i, j;
        s16 pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                if (is_ibc)
                {
                    pt = (MAC_4TAP(coeff_ver, ref[j], ref[s_ref + j], ref[s_ref*2 + j], ref[s_ref*3 + j]) + (1<<5)) >> 6;
                }
                else
                {
                    pt = (MAC_6TAP(coeff_ver, (s32)ref[j], (s32)ref[s_ref + j], (s32)ref[s_ref*2 + j], (s32)ref[s_ref*3 + j], (s32)ref[s_ref*4 + j], (s32)ref[s_ref*5 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                }
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
#else
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_c_4pel_vert_sse(ref, s_ref, pred, s_pred, coeff_ver, w, h, min, max, MAC_ADD_0N, MAC_SFT_0N, 1);
    }
#else
    {
        int i, j;
        s32 pt;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_4TAP(coeff_ver, ref[j], ref[s_ref + j], ref[s_ref*2 + j], ref[s_ref*3 + j]) + MAC_ADD_0N) >> MAC_SFT_0N;
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            ref  += s_ref;
        }
    }
#endif
#endif
}

void com_mc_c_nn(s16 *ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16 *pred, int w, int h, int bit_depth, int is_half_pel_filter
#if USE_IBC
    , int is_ibc
#endif
#if DMVR
    , int is_dmvr
#endif
)
{
#if DMVR
#if IF_LUMA12_CHROMA6
    int offset = 2;
    if (is_ibc)
    {
        offset = 1;
    }
#endif
    static s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_C + 16)*(MAX_CU_SIZE + MC_IBUF_PAD_C + 16)];
#else
    static s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_C)*MAX_CU_SIZE];
#endif
    int         dx, dy;

    if (is_half_pel_filter)
    {
        dx = gmv_x & 31;
        dy = gmv_y & 31;
#if DMVR
        if( is_dmvr )
#if IF_LUMA12_CHROMA6
            ref -= (offset * s_ref + offset);
#else
            ref -= (1 * s_ref + 1);
#endif
        else
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 5) - offset) * s_ref + (gmv_x >> 5) - offset;
#else
            ref += ((gmv_y >> 5) - 1) * s_ref + (gmv_x >> 5) - 1;
#endif
    }
    else
    {
        dx = gmv_x & 0x7;
        dy = gmv_y & 0x7;
#if DMVR
        if (is_dmvr)
#if IF_LUMA12_CHROMA6
            ref -= (offset * s_ref + offset);
#else
            ref -= (1 * s_ref + 1);
#endif
        else
#endif
#if IF_LUMA12_CHROMA6
            ref += ((gmv_y >> 3) - offset) * s_ref + (gmv_x >> 3) - offset; 
#else
            ref += ((gmv_y >> 3) - 1) * s_ref + (gmv_x >> 3) - 1;
#endif
    }
#if USE_IBC
#if DMVR
#if IF_LUMA12_CHROMA6
    const s16 *coeff_hor = is_ibc ? tbl_mc_c_coeff_ibc[dx] : ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp_6tap[dx] : tbl_mc_c_coeff_6tap[dx]);
    const s16 *coeff_ver = is_ibc ? tbl_mc_c_coeff_ibc[dy] : ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp_6tap[dy] : tbl_mc_c_coeff_6tap[dy]);
#else
    const s16 *coeff_hor = is_ibc ? tbl_mc_c_coeff_ibc[dx] : ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
    const s16 *coeff_ver = is_ibc ? tbl_mc_c_coeff_ibc[dy] : ((is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#endif
#else
    const s16 *coeff_hor = is_ibc ? tbl_mc_c_coeff_ibc[dx] : (is_half_pel_filter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx]);
    const s16 *coeff_ver = is_ibc ? tbl_mc_c_coeff_ibc[dy] : (is_half_pel_filter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy]);
#endif
#else
#if DMVR
    const s16 *coeff_hor = (is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
    const s16 *coeff_ver = (is_half_pel_filter && !is_dmvr) ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#else
    const s16 *coeff_hor = is_half_pel_filter ? tbl_mc_c_coeff_hp[dx] : tbl_mc_c_coeff[dx];
    const s16 *coeff_ver = is_half_pel_filter ? tbl_mc_c_coeff_hp[dy] : tbl_mc_c_coeff[dy];
#endif
#endif

#if IF_LUMA12_CHROMA6
    int shift1 = bit_depth - 6;
    int shift2 = 22 - bit_depth;
    if (is_ibc)
    {
        shift1 = bit_depth - 8;
        shift2 = 20 - bit_depth;
    }
    const int add1 = (1 << shift1) >> 1;
    const int add2 = 1 << (shift2 - 1);
#if IF_LUMA12_CHROMA6_SIMD
    int max = ((1 << bit_depth) - 1);
    int min = 0;
    if (is_ibc)
    {
        mc_filter_c_4pel_horz_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 3), min, max, add1, shift1, 0);
        mc_filter_c_4pel_vert_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2, 1);
    }
    else
    {
        mc_filter_c_6pel_horz_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 5), min, max, add1, shift1, 0);
        mc_filter_c_6pel_vert_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2, 1);
    }
#else
    {
        int i, j;
        s16* b;
        s16 pt;
        b = buf;
        if (is_ibc)
        {
            for(i=0; i<h+3; i++)
            {
                for(j=0; j<w; j++)
                {
                    b[j] = (MAC_4TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3]) + add1) >> shift1;
                }
                ref  += s_ref;
                b     += w;
            }
            b = buf;
            for(i=0; i<h; i++)
            {
                for(j=0; j<w; j++)
                {
                    pt = (MAC_4TAP(coeff_ver, b[j], b[j+w], b[j+2*w], b[j+3*w]) + add2) >> shift2;
                    pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
                }
                pred += s_pred;
                b     += w;
            }
        }
        else
        {
            for(i=0; i<h+5; i++)
            {
                for(j=0; j<w; j++)
                {
                    b[j] = (MAC_6TAP(coeff_hor, (s32)ref[j], (s32)ref[j+1], (s32)ref[j+2], (s32)ref[j+3], (s32)ref[j+4], (s32)ref[j+5]) + add1) >> shift1;
                }
                ref  += s_ref;
                b     += w;
            }
            b = buf;
            for(i=0; i<h; i++)
            {
                for(j=0; j<w; j++)
                {
                    pt = (MAC_6TAP(coeff_ver, (s32)b[j], (s32)b[j+w], (s32)b[j+2*w], (s32)b[j+3*w], (s32)b[j+4*w], (s32)b[j+5*w]) + add2) >> shift2;
                    pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
                }
                pred += s_pred;
                b     += w;
            }
        }
    }
#endif
#else
    const int shift1 = bit_depth - 8;
    const int shift2 = 20 - bit_depth;
    const int add1 = (1 << shift1) >> 1;
    const int add2 = 1 << (shift2 - 1);
#if SIMD_MC
    {
        int max = ((1 << bit_depth) - 1);
        int min = 0;
        mc_filter_c_4pel_horz_sse(ref, s_ref, buf, w, coeff_hor, w, (h + 3), min, max, add1, shift1, 0);
        mc_filter_c_4pel_vert_sse(buf, w, pred, s_pred, coeff_ver, w, h, min, max, add2, shift2, 1);
    }
#else
    {
        s16        *b;
        int         i, j;
        s32         pt;
        b = buf;
        for(i=0; i<h+3; i++)
        {
            for(j=0; j<w; j++)
            {
                b[j] = (MAC_4TAP(coeff_hor, ref[j], ref[j+1], ref[j+2], ref[j+3]) + add1) >> shift1;
            }
            ref  += s_ref;
            b     += w;
        }
        b = buf;
        for(i=0; i<h; i++)
        {
            for(j=0; j<w; j++)
            {
                pt = (MAC_4TAP(coeff_ver, b[j], b[j+w], b[j+2*w], b[j+3*w]) + add2) >> shift2;
                pred[j] = (s16)COM_CLIP3(0, (1 << bit_depth) - 1, pt);
            }
            pred += s_pred;
            b     += w;
        }
    }
#endif
#endif
}

COM_MC_L com_tbl_mc_l[2][2] =
{
    {
        com_mc_l_00, /* dx == 0 && dy == 0 */
        com_mc_l_0n  /* dx == 0 && dy != 0 */
    },
    {
        com_mc_l_n0, /* dx != 0 && dy == 0 */
        com_mc_l_nn  /* dx != 0 && dy != 0 */
    }
};

COM_MC_C com_tbl_mc_c[2][2] =
{
    {
        com_mc_c_00, /* dx == 0 && dy == 0 */
        com_mc_c_0n  /* dx == 0 && dy != 0 */
    },
    {
        com_mc_c_n0, /* dx != 0 && dy == 0 */
        com_mc_c_nn  /* dx != 0 && dy != 0 */
    }
};


#if BIO
/****************************************************************************
* BIO motion compensation for luma
****************************************************************************/

s16 grad_x[2][MAX_CU_SIZE*MAX_CU_SIZE];
s16 grad_y[2][MAX_CU_SIZE*MAX_CU_SIZE];
s32 sigma[5][(MAX_CU_SIZE + 2 * BIO_WINDOW_SIZE) *(MAX_CU_SIZE + 2 * BIO_WINDOW_SIZE)];

/****************************************************************************
* 8 taps gradient
****************************************************************************/

#if SIMD_MC
static const s16 tbl_bio_grad_l_coeff[4][8] =
#else
static const s8 tbl_bio_grad_l_coeff[4][8] =
#endif
{
    { -4, 11, -39, -1, 41, -14, 8, -2, },
    { -2, 6, -19, -31, 53, -12, 7, -2, },
    { 0, -1, 0, -50, 50, 0, 1, 0, },
    { 2, -7, 12, -53, 31, 19, -6, 2, }
};


static void com_grad_x_l_nn(s16* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16* pred, int w, int h, int bit_depth, int is_dmvr)
{
    static s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L) * MAX_CU_SIZE];
#if IF_LUMA12_CHROMA6_SIMD
    int dx, dy;
#else
    s16* b;
    int         i, j, dx, dy;
    s32         pt;
#endif

    dx = gmv_x & 0x3;
    dy = gmv_y & 0x3;

#if IF_LUMA12_CHROMA6
    const int offset = 5;
    if (is_dmvr)
    {
        ref = ref - (offset * s_ref + 3);  
    }
    else
    {
        ref += ((gmv_y >> 2) - offset) * s_ref + (gmv_x >> 2) - 3;
    }
#else
    if (is_dmvr)
    {
        ref = ref - (3 * s_ref + 3);
    }
    else
    {
        ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
    }
#endif

#if IF_LUMA12_CHROMA6
    const int s2 = BIO_IF_S2(bit_depth) - 6 + 8;
    const int o2 = 1 << (s2 - 1);
#if IF_LUMA12_CHROMA6_SIMD
    mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_bio_grad_l_coeff[dx], w, (h + 11), BIO_IF_O1(bit_depth), BIO_IF_S1(bit_depth));
    mc_filter_l_12pel_vert_clip_sse(buf, w, pred, s_pred, tbl_mc_l_coeff_12tap[dy], w, h, 0, ((1 << bit_depth) - 1), o2, s2, 0);
#else
    {
        b = buf;
        for (i = 0; i < h + 11; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* GRADIENT ON X */
                b[j] = ((MAC_8TAP(tbl_bio_grad_l_coeff[dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + BIO_IF_O1(bit_depth)) >> BIO_IF_S1(bit_depth));
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* MC ON Y */
                pt = ((MAC_12TAP(tbl_mc_l_coeff_12tap[dy], (s32)b[j], (s32)b[j + w], (s32)b[j + w * 2], (s32)b[j + w * 3], (s32)b[j + w * 4], (s32)b[j + w * 5], (s32)b[j + w * 6], (s32)b[j + w * 7], (s32)b[j + w * 8], (s32)b[j + w * 9], (s32)b[j + w * 10], (s32)b[j + w * 11]) + o2) >> s2);
                /* NO CLIP */
                pred[j] = (s16)pt;
            }
            pred += s_pred;
            b += w;
        }
    }
#endif
#else
#if SIMD_MC
    if (1)
    {
        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_bio_grad_l_coeff[dx], w, (h + 7),
            BIO_IF_O1(bit_depth), BIO_IF_S1(bit_depth));

        mc_filter_l_8pel_vert_no_clip_sse(buf, w, pred, s_pred, tbl_mc_l_coeff[dy], w, h,
            BIO_IF_O2(bit_depth), BIO_IF_S2(bit_depth));
    }
    else
#endif
    {
        b = buf;
        for (i = 0; i < h + 7; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* GRADIENT ON X */
                b[j] = ((MAC_8TAP(tbl_bio_grad_l_coeff[dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + BIO_IF_O1(bit_depth)) >> BIO_IF_S1(bit_depth));
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* MC ON Y */
                pt = ((MAC_8TAP(tbl_mc_l_coeff[dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]) + BIO_IF_O2(bit_depth)) >> BIO_IF_S2(bit_depth));
                /* NO CLIP */
                pred[j] = (s16)pt;
            }
            pred += s_pred;
            b += w;
        }
    }
#endif
}

static void com_grad_y_l_nn(s16* ref, int gmv_x, int gmv_y, int s_ref, int s_pred, s16* pred, int w, int h, int bit_depth, int is_dmvr)
{
    static s16         buf[(MAX_CU_SIZE + MC_IBUF_PAD_L) * MAX_CU_SIZE];
#if IF_LUMA12_CHROMA6_SIMD
    int dx, dy;
#else
    s16* b;
    int         i, j, dx, dy;
    s32         pt;
#endif

    dx = gmv_x & 0x3;
    dy = gmv_y & 0x3;

#if IF_LUMA12_CHROMA6
    const int offset = 5;
    if (is_dmvr)
    {
        ref = ref - (3 * s_ref + offset);
    }
    else
    {
        ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - offset;
    }
#else
    if (is_dmvr)
    {
        ref = ref - (3 * s_ref + 3);
    }
    else
    {
        ref += ((gmv_y >> 2) - 3) * s_ref + (gmv_x >> 2) - 3;
    }
#endif

#if IF_LUMA12_CHROMA6
    const int s1 = BIO_IF_S1(bit_depth) - 6 + 8;
    const int o1 = 1 << (s1 - 1);
#if IF_LUMA12_CHROMA6_SIMD
    mc_filter_l_12pel_horz_clip_sse(ref, s_ref, buf, w, tbl_mc_l_coeff_12tap[dx], w, (h + 7), 0, ((1 << bit_depth) - 1), o1, s1, 0);
    mc_filter_l_8pel_vert_no_clip_sse(buf, w, pred, s_pred, tbl_bio_grad_l_coeff[dy], w, h, BIO_IF_O2(bit_depth), BIO_IF_S2(bit_depth));
#else
    {
        b = buf;
        for (i = 0; i < h + 7; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* MC ON X */
                b[j] = ((MAC_12TAP(tbl_mc_l_coeff_12tap[dx], (s32)ref[j], (s32)ref[j + 1], (s32)ref[j + 2], (s32)ref[j + 3], (s32)ref[j + 4], (s32)ref[j + 5], (s32)ref[j + 6], (s32)ref[j + 7], (s32)ref[j + 8], (s32)ref[j + 9], (s32)ref[j + 10], (s32)ref[j + 11]) + o1) >> s1);
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* GRADIENT ON Y */
                pt = ((MAC_8TAP(tbl_bio_grad_l_coeff[dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]) + BIO_IF_O2(bit_depth)) >> BIO_IF_S2(bit_depth));
                pred[j] = (s16)pt;
            }
            pred += s_pred;
            b += w;
        }
    }
#endif
#else
#if SIMD_MC
    if (1)
    {
        mc_filter_l_8pel_horz_no_clip_sse(ref, s_ref, buf, w, tbl_mc_l_coeff[dx], w, (h + 7),
            BIO_IF_O1(bit_depth), BIO_IF_S1(bit_depth));

        mc_filter_l_8pel_vert_no_clip_sse(buf, w, pred, s_pred, tbl_bio_grad_l_coeff[dy], w, h,
            BIO_IF_O2(bit_depth), BIO_IF_S2(bit_depth));
    }
    else
#endif
    {
        b = buf;
        for (i = 0; i < h + 7; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* MC ON X */
                b[j] = ((MAC_8TAP(tbl_mc_l_coeff[dx], ref[j], ref[j + 1], ref[j + 2], ref[j + 3], ref[j + 4], ref[j + 5], ref[j + 6], ref[j + 7]) + BIO_IF_O1(bit_depth)) >> BIO_IF_S1(bit_depth));
            }
            ref += s_ref;
            b += w;
        }

        b = buf;
        for (i = 0; i < h; i++)
        {
            for (j = 0; j < w; j++)
            {
                /* GRADIENT ON Y */
                pt = ((MAC_8TAP(tbl_bio_grad_l_coeff[dy], b[j], b[j + w], b[j + w * 2], b[j + w * 3], b[j + w * 4], b[j + w * 5], b[j + w * 6], b[j + w * 7]) + BIO_IF_O2(bit_depth)) >> BIO_IF_S2(bit_depth));
                pred[j] = (s16)pt;
            }
            pred += s_pred;
            b += w;
        }
    }
#endif
}

void bio_vxvy(s16* vx, s16* vy, int wb, s32* s1, s32* s2, s32* s3, s32* s5, s32* s6, int avg_size)
{
    int ii, jj;
    s32 s1a = 0; s32* s1t = s1;
    s32 s2a = 0; s32* s2t = s2;
    s32 s3a = 0; s32* s3t = s3;
    s32 s5a = 0; s32* s5t = s5;
    s32 s6a = 0; s32* s6t = s6;

#if SIMD_MC
    __m128i vzero = _mm_setzero_si128();
    __m128i mmS1a = _mm_setzero_si128();
    __m128i mmS2a = _mm_setzero_si128();
    __m128i mmS3a = _mm_setzero_si128();
    __m128i mmS5a = _mm_setzero_si128();
    __m128i mmS6a = _mm_setzero_si128();

    __m128i mmS1;
    __m128i mmS2;
    __m128i mmS3;
    __m128i mmS5;
    __m128i mmS6;
    for (jj = 0; jj < avg_size; jj++)
    {
        ii = 0;
        for (; ii < ((avg_size >> 2) << 2); ii += 4)
        {
            mmS1 = _mm_loadu_si128((__m128i*)(s1t + ii));
            mmS2 = _mm_loadu_si128((__m128i*)(s2t + ii));
            mmS3 = _mm_loadu_si128((__m128i*)(s3t + ii));
            mmS5 = _mm_loadu_si128((__m128i*)(s5t + ii));
            mmS6 = _mm_loadu_si128((__m128i*)(s6t + ii));

            mmS1a = _mm_add_epi32(mmS1a, mmS1);
            mmS2a = _mm_add_epi32(mmS2a, mmS2);
            mmS3a = _mm_add_epi32(mmS3a, mmS3);
            mmS5a = _mm_add_epi32(mmS5a, mmS5);
            mmS6a = _mm_add_epi32(mmS6a, mmS6);
        }
        for (; ii < ((avg_size >> 1) << 1); ii += 2)
        {
            mmS1 = _mm_loadl_epi64((__m128i*)(s1t + ii));
            mmS2 = _mm_loadl_epi64((__m128i*)(s2t + ii));
            mmS3 = _mm_loadl_epi64((__m128i*)(s3t + ii));
            mmS5 = _mm_loadl_epi64((__m128i*)(s5t + ii));
            mmS6 = _mm_loadl_epi64((__m128i*)(s6t + ii));

            mmS1a = _mm_add_epi32(mmS1a, mmS1);
            mmS2a = _mm_add_epi32(mmS2a, mmS2);
            mmS3a = _mm_add_epi32(mmS3a, mmS3);
            mmS5a = _mm_add_epi32(mmS5a, mmS5);
            mmS6a = _mm_add_epi32(mmS6a, mmS6);
        }
        s1t += wb; s2t += wb; s3t += wb; s5t += wb; s6t += wb;
    }

    mmS1a = _mm_hadd_epi32(_mm_hadd_epi32(mmS1a, vzero), vzero);
    mmS2a = _mm_hadd_epi32(_mm_hadd_epi32(mmS2a, vzero), vzero);
    mmS3a = _mm_hadd_epi32(_mm_hadd_epi32(mmS3a, vzero), vzero);
    mmS5a = _mm_hadd_epi32(_mm_hadd_epi32(mmS5a, vzero), vzero);
    mmS6a = _mm_hadd_epi32(_mm_hadd_epi32(mmS6a, vzero), vzero);

    s1a = _mm_cvtsi128_si32(mmS1a);
    s2a = _mm_cvtsi128_si32(mmS2a);
    s3a = _mm_cvtsi128_si32(mmS3a);
    s5a = _mm_cvtsi128_si32(mmS5a);
    s6a = _mm_cvtsi128_si32(mmS6a);
#else
    for (jj = 0; jj < avg_size; jj++)
    {
        for (ii = 0; ii < avg_size; ii++)
        {
            s1a += s1t[ii];
            s2a += s2t[ii];
            s3a += s3t[ii];
            s5a += s5t[ii];
            s6a += s6t[ii];
        }
        s1t += wb; s2t += wb; s3t += wb; s5t += wb; s6t += wb;
    }
#endif

    *vx = 0; *vy = 0;
    s32 vx_t = 0, vy_t = 0;
    if (s1a > DENOMBIO)
    {
        vx_t = (s3a << 5) >> (s32)floor(log2(s1a + DENOMBIO));
        *vx = COM_CLIP3(-LIMITBIO, LIMITBIO, vx_t);
    }
    if (s5a > DENOMBIO)
    {
        vy_t = ((s6a << 6) - *vx*s2a) >> (s32)floor(log2((s5a + DENOMBIO) << 1));    
        *vy = COM_CLIP3(-LIMITBIO, LIMITBIO, vy_t);
    }
}

static void bio_block_average_16b_clip_sse(s16 *p0t, s16 *p1t,
    s16 *gx0t, s16 *gy0t, s16 *gx1t, s16 *gy1t,
    s16 vx, s16 vy,
    int bio_cluster_size, int s_gt, int bit_depth)
{
    int i, j, grad_on = 1;
    int s_gt_3 = s_gt * 3;

    __m128i gx0t_8x16b_0, gx0t_8x16b_1, gx0t_8x16b_2, gx0t_8x16b_3;
    __m128i gx1t_8x16b_0, gx1t_8x16b_1, gx1t_8x16b_2, gx1t_8x16b_3;
    __m128i gy0t_8x16b_0, gy0t_8x16b_1, gy0t_8x16b_2, gy0t_8x16b_3;
    __m128i gy1t_8x16b_0, gy1t_8x16b_1, gy1t_8x16b_2, gy1t_8x16b_3;

    __m128i p0t_8x16b_0, p0t_8x16b_1, p0t_8x16b_2, p0t_8x16b_3;
    __m128i p1t_8x16b_0, p1t_8x16b_1, p1t_8x16b_2, p1t_8x16b_3;

    __m128i vx_4x32b = _mm_set1_epi32(vx);
    __m128i vy_4x32b = _mm_set1_epi32(vy);

    __m128i offset0_4x32b = _mm_set1_epi32(32);
    __m128i offset1_4x32b = _mm_set1_epi32(1);
    __m128i max_4x32b = _mm_set1_epi32((1 << bit_depth) - 1);
    __m128i min_4x32b = _mm_setzero_si128();

    /* Should be multiple of 4 */
    assert(!(bio_cluster_size & 0x3));

    if ((vx == 0) && (vy == 0))
        grad_on = 0;

    for (i = 0; i < bio_cluster_size; i += 4)
    {
        for (j = 0; j < bio_cluster_size; j += 4)
        {
            if (grad_on)
            {
                gx0t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gx0t));
                gx0t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gx0t + s_gt));

                gx1t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gx1t));
                gx1t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gx1t + s_gt));

                gy0t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gy0t));
                gy0t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gy0t + s_gt));

                gy1t_8x16b_0 = _mm_loadl_epi64((__m128i *) (gy1t));
                gy1t_8x16b_1 = _mm_loadl_epi64((__m128i *) (gy1t + s_gt));

                gx0t_8x16b_0 = _mm_cvtepi16_epi32(gx0t_8x16b_0);
                gx0t_8x16b_1 = _mm_cvtepi16_epi32(gx0t_8x16b_1);

                gx1t_8x16b_0 = _mm_cvtepi16_epi32(gx1t_8x16b_0);
                gx1t_8x16b_1 = _mm_cvtepi16_epi32(gx1t_8x16b_1);

                gy0t_8x16b_0 = _mm_cvtepi16_epi32(gy0t_8x16b_0);
                gy0t_8x16b_1 = _mm_cvtepi16_epi32(gy0t_8x16b_1);

                gy1t_8x16b_0 = _mm_cvtepi16_epi32(gy1t_8x16b_0);
                gy1t_8x16b_1 = _mm_cvtepi16_epi32(gy1t_8x16b_1);
                /*gx0t[ii] - gx1t[ii]*/
                gx0t_8x16b_0 = _mm_sub_epi32(gx0t_8x16b_0, gx1t_8x16b_0);
                gx0t_8x16b_1 = _mm_sub_epi32(gx0t_8x16b_1, gx1t_8x16b_1);
                /*gy0t[ii] - gy1t[ii]*/
                gy0t_8x16b_0 = _mm_sub_epi32(gy0t_8x16b_0, gy1t_8x16b_0);
                gy0t_8x16b_1 = _mm_sub_epi32(gy0t_8x16b_1, gy1t_8x16b_1);
                /*vx * (gx0t[ii] - gx1t[ii])*/
                gx0t_8x16b_0 = _mm_mullo_epi32(gx0t_8x16b_0, vx_4x32b);
                gx0t_8x16b_1 = _mm_mullo_epi32(gx0t_8x16b_1, vx_4x32b);
                /*vy * (gy0t[ii] - gy1t[ii])*/
                gy0t_8x16b_0 = _mm_mullo_epi32(gy0t_8x16b_0, vy_4x32b);
                gy0t_8x16b_1 = _mm_mullo_epi32(gy0t_8x16b_1, vy_4x32b);

                gx0t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gx0t + (s_gt << 1)));
                gx0t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gx0t + s_gt_3));

                gx1t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gx1t + (s_gt << 1)));
                gx1t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gx1t + s_gt_3));

                gy0t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gy0t + (s_gt << 1)));
                gy0t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gy0t + s_gt_3));

                gy1t_8x16b_2 = _mm_loadl_epi64((__m128i *) (gy1t + (s_gt << 1)));
                gy1t_8x16b_3 = _mm_loadl_epi64((__m128i *) (gy1t + s_gt_3));

                gx0t_8x16b_2 = _mm_cvtepi16_epi32(gx0t_8x16b_2);
                gx0t_8x16b_3 = _mm_cvtepi16_epi32(gx0t_8x16b_3);

                gx1t_8x16b_2 = _mm_cvtepi16_epi32(gx1t_8x16b_2);
                gx1t_8x16b_3 = _mm_cvtepi16_epi32(gx1t_8x16b_3);

                gy0t_8x16b_2 = _mm_cvtepi16_epi32(gy0t_8x16b_2);
                gy0t_8x16b_3 = _mm_cvtepi16_epi32(gy0t_8x16b_3);

                gy1t_8x16b_2 = _mm_cvtepi16_epi32(gy1t_8x16b_2);
                gy1t_8x16b_3 = _mm_cvtepi16_epi32(gy1t_8x16b_3);

                gx0t_8x16b_2 = _mm_sub_epi32(gx0t_8x16b_2, gx1t_8x16b_2);
                gx0t_8x16b_3 = _mm_sub_epi32(gx0t_8x16b_3, gx1t_8x16b_3);

                gy0t_8x16b_2 = _mm_sub_epi32(gy0t_8x16b_2, gy1t_8x16b_2);
                gy0t_8x16b_3 = _mm_sub_epi32(gy0t_8x16b_3, gy1t_8x16b_3);

                gx0t_8x16b_2 = _mm_mullo_epi32(gx0t_8x16b_2, vx_4x32b);
                gx0t_8x16b_3 = _mm_mullo_epi32(gx0t_8x16b_3, vx_4x32b);

                gy0t_8x16b_2 = _mm_mullo_epi32(gy0t_8x16b_2, vy_4x32b);
                gy0t_8x16b_3 = _mm_mullo_epi32(gy0t_8x16b_3, vy_4x32b);
                /*b = vx * (gx0t[ii] - gx1t[ii]) + vy * (gy0t[ii] - gy1t[ii])*/
                gx0t_8x16b_0 = _mm_add_epi32(gx0t_8x16b_0, gy0t_8x16b_0);
                gx0t_8x16b_1 = _mm_add_epi32(gx0t_8x16b_1, gy0t_8x16b_1);
                gx0t_8x16b_2 = _mm_add_epi32(gx0t_8x16b_2, gy0t_8x16b_2);
                gx0t_8x16b_3 = _mm_add_epi32(gx0t_8x16b_3, gy0t_8x16b_3);
                /*b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));*/
                gy0t_8x16b_0 = _mm_abs_epi32(gx0t_8x16b_0);
                gy0t_8x16b_1 = _mm_abs_epi32(gx0t_8x16b_1);
                gy0t_8x16b_2 = _mm_abs_epi32(gx0t_8x16b_2);
                gy0t_8x16b_3 = _mm_abs_epi32(gx0t_8x16b_3);

                gy0t_8x16b_0 = _mm_add_epi32(gy0t_8x16b_0, offset0_4x32b);
                gy0t_8x16b_1 = _mm_add_epi32(gy0t_8x16b_1, offset0_4x32b);
                gy0t_8x16b_2 = _mm_add_epi32(gy0t_8x16b_2, offset0_4x32b);
                gy0t_8x16b_3 = _mm_add_epi32(gy0t_8x16b_3, offset0_4x32b);

                gy0t_8x16b_0 = _mm_srli_epi32(gy0t_8x16b_0, 6);
                gy0t_8x16b_1 = _mm_srli_epi32(gy0t_8x16b_1, 6);
                gy0t_8x16b_2 = _mm_srli_epi32(gy0t_8x16b_2, 6);
                gy0t_8x16b_3 = _mm_srli_epi32(gy0t_8x16b_3, 6);

                gx0t_8x16b_0 = _mm_sign_epi32(gy0t_8x16b_0, gx0t_8x16b_0);
                gx0t_8x16b_1 = _mm_sign_epi32(gy0t_8x16b_1, gx0t_8x16b_1);
                gx0t_8x16b_2 = _mm_sign_epi32(gy0t_8x16b_2, gx0t_8x16b_2);
                gx0t_8x16b_3 = _mm_sign_epi32(gy0t_8x16b_3, gx0t_8x16b_3);

                /*b + 1*/
                gx0t_8x16b_0 = _mm_add_epi32(gx0t_8x16b_0, offset1_4x32b);
                gx0t_8x16b_1 = _mm_add_epi32(gx0t_8x16b_1, offset1_4x32b);
                gx0t_8x16b_2 = _mm_add_epi32(gx0t_8x16b_2, offset1_4x32b);
                gx0t_8x16b_3 = _mm_add_epi32(gx0t_8x16b_3, offset1_4x32b);
            }
            else
            {
                /*b + 1*/
                gx0t_8x16b_0 = offset1_4x32b;
                gx0t_8x16b_1 = offset1_4x32b;
                gx0t_8x16b_2 = offset1_4x32b;
                gx0t_8x16b_3 = offset1_4x32b;
            }

            /*p0t[ii]*/
            p0t_8x16b_0 = _mm_loadl_epi64((__m128i *) (p0t));
            p0t_8x16b_1 = _mm_loadl_epi64((__m128i *) (p0t + s_gt));
            p0t_8x16b_2 = _mm_loadl_epi64((__m128i *) (p0t + (s_gt << 1)));
            p0t_8x16b_3 = _mm_loadl_epi64((__m128i *) (p0t + s_gt_3));
            /*p1t[ii]*/
            p1t_8x16b_0 = _mm_loadl_epi64((__m128i *) (p1t));
            p1t_8x16b_1 = _mm_loadl_epi64((__m128i *) (p1t + s_gt));
            p1t_8x16b_2 = _mm_loadl_epi64((__m128i *) (p1t + (s_gt << 1)));
            p1t_8x16b_3 = _mm_loadl_epi64((__m128i *) (p1t + s_gt_3));

            p0t_8x16b_0 = _mm_cvtepi16_epi32(p0t_8x16b_0);
            p0t_8x16b_1 = _mm_cvtepi16_epi32(p0t_8x16b_1);
            p0t_8x16b_2 = _mm_cvtepi16_epi32(p0t_8x16b_2);
            p0t_8x16b_3 = _mm_cvtepi16_epi32(p0t_8x16b_3);

            p1t_8x16b_0 = _mm_cvtepi16_epi32(p1t_8x16b_0);
            p1t_8x16b_1 = _mm_cvtepi16_epi32(p1t_8x16b_1);
            p1t_8x16b_2 = _mm_cvtepi16_epi32(p1t_8x16b_2);
            p1t_8x16b_3 = _mm_cvtepi16_epi32(p1t_8x16b_3);

            /*p0t[ii] + p1t[ii]*/
            p0t_8x16b_0 = _mm_add_epi32(p0t_8x16b_0, p1t_8x16b_0);
            p0t_8x16b_1 = _mm_add_epi32(p0t_8x16b_1, p1t_8x16b_1);
            p0t_8x16b_2 = _mm_add_epi32(p0t_8x16b_2, p1t_8x16b_2);
            p0t_8x16b_3 = _mm_add_epi32(p0t_8x16b_3, p1t_8x16b_3);
            /*(p0t[ii] + p1t[ii] + b + 1)*/
            p0t_8x16b_0 = _mm_add_epi32(p0t_8x16b_0, gx0t_8x16b_0);
            p0t_8x16b_1 = _mm_add_epi32(p0t_8x16b_1, gx0t_8x16b_1);
            p0t_8x16b_2 = _mm_add_epi32(p0t_8x16b_2, gx0t_8x16b_2);
            p0t_8x16b_3 = _mm_add_epi32(p0t_8x16b_3, gx0t_8x16b_3);
            /*(p0t[ii] + p1t[ii] + b + 1) >> 1*/
            p0t_8x16b_0 = _mm_srai_epi32(p0t_8x16b_0, 1);
            p0t_8x16b_1 = _mm_srai_epi32(p0t_8x16b_1, 1);
            p0t_8x16b_2 = _mm_srai_epi32(p0t_8x16b_2, 1);
            p0t_8x16b_3 = _mm_srai_epi32(p0t_8x16b_3, 1);
            /*COM_CLIP3(0, (1 << BIT_DEPTH) - 1, (s16)((p0t[ii] + p1t[ii] + b + 1) >> 1))*/
            p0t_8x16b_0 = _mm_min_epi32(p0t_8x16b_0, max_4x32b);
            p0t_8x16b_1 = _mm_min_epi32(p0t_8x16b_1, max_4x32b);
            p0t_8x16b_2 = _mm_min_epi32(p0t_8x16b_2, max_4x32b);
            p0t_8x16b_3 = _mm_min_epi32(p0t_8x16b_3, max_4x32b);

            p0t_8x16b_0 = _mm_max_epi32(p0t_8x16b_0, min_4x32b);
            p0t_8x16b_1 = _mm_max_epi32(p0t_8x16b_1, min_4x32b);
            p0t_8x16b_2 = _mm_max_epi32(p0t_8x16b_2, min_4x32b);
            p0t_8x16b_3 = _mm_max_epi32(p0t_8x16b_3, min_4x32b);

            p0t_8x16b_0 = _mm_packs_epi32(p0t_8x16b_0, p0t_8x16b_0);
            p0t_8x16b_1 = _mm_packs_epi32(p0t_8x16b_1, p0t_8x16b_1);
            p0t_8x16b_2 = _mm_packs_epi32(p0t_8x16b_2, p0t_8x16b_2);
            p0t_8x16b_3 = _mm_packs_epi32(p0t_8x16b_3, p0t_8x16b_3);

            _mm_storel_epi64((__m128i *) (p0t), p0t_8x16b_0);
            _mm_storel_epi64((__m128i *) (p0t + s_gt), p0t_8x16b_1);
            _mm_storel_epi64((__m128i *) (p0t + (s_gt << 1)), p0t_8x16b_2);
            _mm_storel_epi64((__m128i *) (p0t + s_gt_3), p0t_8x16b_3);

            p0t += 4; p1t += 4;  gx0t += 4; gy0t += 4; gx1t += 4; gy1t += 4;
        }

        p0t = p0t - bio_cluster_size + 4 * s_gt;
        p1t = p1t - bio_cluster_size + 4 * s_gt;
        gx0t = gx0t - bio_cluster_size + 4 * s_gt;
        gy0t = gy0t - bio_cluster_size + 4 * s_gt;
        gx1t = gx1t - bio_cluster_size + 4 * s_gt;
        gy1t = gy1t - bio_cluster_size + 4 * s_gt;
    }
}

void bio_sigma(const pel* p0, const pel* p1, const pel* gx0, const pel* gx1, const pel* gy0, const pel* gy1, int* s1, int* s2, int* s3, int* s5, int* s6, const int w, const int h, const int winSize, const int wb)
{
    int* s1t = s1 + wb * winSize;
    int* s2t = s2 + wb * winSize;
    int* s3t = s3 + wb * winSize;
    int* s5t = s5 + wb * winSize;
    int* s6t = s6 + wb * winSize;

    int y = 0;
    for (; y < h; y++)
    {
        int x = 0;

        for (; x < ((w >> 3) << 3); x += 8)
        {
            __m128i mm_p0 = _mm_loadu_si128((__m128i*)(p0 + x));
            __m128i mm_p1 = _mm_loadu_si128((__m128i*)(p1 + x));
            __m128i mm_gx0 = _mm_loadu_si128((__m128i*)(gx0 + x));
            __m128i mm_gx1 = _mm_loadu_si128((__m128i*)(gx1 + x));
            __m128i mm_gy0 = _mm_loadu_si128((__m128i*)(gy0 + x));
            __m128i mm_gy1 = _mm_loadu_si128((__m128i*)(gy1 + x));

            __m128i mm_t = _mm_sub_epi16(mm_p1, mm_p0);
            __m128i mm_tx = _mm_add_epi16(mm_gx0, mm_gx1);
            __m128i mm_ty = _mm_add_epi16(mm_gy0, mm_gy1);

            // s1t
            __m128i mm_b = _mm_mulhi_epi16(mm_tx, mm_tx);
            __m128i mm_a = _mm_mullo_epi16(mm_tx, mm_tx);

            __m128i mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            __m128i mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s1t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s1t + x + 4), mm_h);

            // s2t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_ty);
            mm_a = _mm_mullo_epi16(mm_tx, mm_ty);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s2t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s2t + x + 4), mm_h);

            // s3t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_t);
            mm_a = _mm_mullo_epi16(mm_tx, mm_t);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s3t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s3t + x + 4), mm_h);

            // s5t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_ty);
            mm_a = _mm_mullo_epi16(mm_ty, mm_ty);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s5t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s5t + x + 4), mm_h);

            // s6t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_t);
            mm_a = _mm_mullo_epi16(mm_ty, mm_t);

            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
            mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s6t + x), mm_l);
            _mm_storeu_si128((__m128i *)(s6t + x + 4), mm_h);
        }

        for (; x < ((w >> 2) << 2); x += 4)
        {
            __m128i mm_p0 = _mm_loadl_epi64((__m128i*)(p0 + x));
            __m128i mm_p1 = _mm_loadl_epi64((__m128i*)(p1 + x));
            __m128i mm_gx0 = _mm_loadl_epi64((__m128i*)(gx0 + x));
            __m128i mm_gx1 = _mm_loadl_epi64((__m128i*)(gx1 + x));
            __m128i mm_gy0 = _mm_loadl_epi64((__m128i*)(gy0 + x));
            __m128i mm_gy1 = _mm_loadl_epi64((__m128i*)(gy1 + x));

            __m128i mm_t = _mm_sub_epi16(mm_p1, mm_p0);
            __m128i mm_tx = _mm_add_epi16(mm_gx0, mm_gx1);
            __m128i mm_ty = _mm_add_epi16(mm_gy0, mm_gy1);

            // s1t
            __m128i mm_b = _mm_mulhi_epi16(mm_tx, mm_tx);
            __m128i mm_a = _mm_mullo_epi16(mm_tx, mm_tx);
            __m128i mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s1t + x), mm_l);

            // s2t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_ty);
            mm_a = _mm_mullo_epi16(mm_tx, mm_ty);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s2t + x), mm_l);

            // s3t
            mm_b = _mm_mulhi_epi16(mm_tx, mm_t);
            mm_a = _mm_mullo_epi16(mm_tx, mm_t);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s3t + x), mm_l);

            // s5t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_ty);
            mm_a = _mm_mullo_epi16(mm_ty, mm_ty);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s5t + x), mm_l);

            // s6t
            mm_b = _mm_mulhi_epi16(mm_ty, mm_t);
            mm_a = _mm_mullo_epi16(mm_ty, mm_t);
            mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

            _mm_storeu_si128((__m128i *)(s6t + x), mm_l);
        }

        for (; x < w; x++)
        {
            int t = (p0[x]) - (p1[x]);
            int tx = (gx0[x] + gx1[x]);
            int ty = (gy0[x] + gy1[x]);
            s1t[x] = tx * tx;
            s2t[x] = tx * ty;
            s3t[x] = -tx * t;
            s5t[x] = ty * ty;
            s6t[x] = -ty * t;
        }
        for (; x < w + winSize; x++)
        {
            s1t[x] = s1t[x - 1];
            s2t[x] = s2t[x - 1];
            s3t[x] = s3t[x - 1];
            s5t[x] = s5t[x - 1];
            s6t[x] = s6t[x - 1];
        }
        for (x = -1; x >= -winSize; x--)
        {
            s1t[x] = s1t[x + 1];
            s2t[x] = s2t[x + 1];
            s3t[x] = s3t[x + 1];
            s5t[x] = s5t[x + 1];
            s6t[x] = s6t[x + 1];
        }

        p0 += w;
        p1 += w;
        gx0 += w;
        gx1 += w;
        gy0 += w;
        gy1 += w;

        s1t += wb;
        s2t += wb;
        s3t += wb;
        s5t += wb;
        s6t += wb;
    }

    for (; y < h + winSize; y++)
    {
        memcpy(s1t - winSize, s1t - winSize - wb, sizeof(int)*(wb));
        memcpy(s2t - winSize, s2t - winSize - wb, sizeof(int)*(wb));
        memcpy(s3t - winSize, s3t - winSize - wb, sizeof(int)*(wb));
        memcpy(s5t - winSize, s5t - winSize - wb, sizeof(int)*(wb));
        memcpy(s6t - winSize, s6t - winSize - wb, sizeof(int)*(wb));
        s1t += wb;
        s2t += wb;
        s3t += wb;
        s5t += wb;
        s6t += wb;
    }
    s1t = s1 + wb * (winSize - 1);
    s2t = s2 + wb * (winSize - 1);
    s3t = s3 + wb * (winSize - 1);
    s5t = s5 + wb * (winSize - 1);
    s6t = s6 + wb * (winSize - 1);
    for (y = -1; y >= -winSize; y--)
    {
        memcpy(s1t - winSize, s1t - winSize + wb, sizeof(int)*(wb));
        memcpy(s2t - winSize, s2t - winSize + wb, sizeof(int)*(wb));
        memcpy(s3t - winSize, s3t - winSize + wb, sizeof(int)*(wb));
        memcpy(s5t - winSize, s5t - winSize + wb, sizeof(int)*(wb));
        memcpy(s6t - winSize, s6t - winSize + wb, sizeof(int)*(wb));
        s1t -= wb;
        s2t -= wb;
        s3t -= wb;
        s5t -= wb;
        s6t -= wb;
    }
}

void bio_grad(pel* p0, pel* p1, pel* gx0, pel* gx1, pel* gy0, pel* gy1, int w, int h)
{
    assert((w & 3) == 0);
    pel *src;
    pel *gx, *gy;
    for (int dir = 0; dir < 2; dir++)
    {
        if (dir == 0)
        {
            src = p0;
            gx = gx0;
            gy = gy0;
        }
        else
        {
            src = p1;
            gx = gx1;
            gy = gy1;
        }

        for (int y = 0; y < h; y++)
        {
            int x = 0;
            for (; x < ((w >> 3) << 3); x += 8)
            {
                pel tmp[8];
                __m128i mm_top, mm_bottom, mm_left, mm_right;
                mm_top = _mm_loadu_si128((__m128i*)(y == 0 ? src + x : src + x - w));
                mm_bottom = _mm_loadu_si128((__m128i*)(y == h - 1 ? src + x : src + x + w));
                if (x == 0)
                {
                    tmp[0] = src[0];
                    memcpy(tmp + 1, src, sizeof(pel)*(7));
                }
                mm_left = _mm_loadu_si128((__m128i*)(x == 0 ? tmp : src + x - 1));
                if (x == w - 8)
                {
                    memcpy(tmp, src + x + 1, sizeof(pel)*(7));
                    tmp[7] = tmp[6];
                }
                mm_right = _mm_loadu_si128((__m128i*)(x == w - 8 ? tmp : src + x + 1));

                __m128i mm_gx = _mm_srai_epi16(_mm_sub_epi16(mm_right, mm_left), 4);
                __m128i mm_gy = _mm_srai_epi16(_mm_sub_epi16(mm_bottom, mm_top), 4);

                _mm_storeu_si128((__m128i *)(gx + x), mm_gx);
                _mm_storeu_si128((__m128i *)(gy + x), mm_gy);
            }
            for (; x < w; x += 4)
            {
                pel tmp[4];
                __m128i mm_top, mm_bottom, mm_left, mm_right;
                mm_top = _mm_loadl_epi64((__m128i*)(y == 0 ? src + x : src + x - w));
                mm_bottom = _mm_loadl_epi64((__m128i*)(y == h - 1 ? src + x : src + x + w));
                if (x == 0)
                {
                    tmp[0] = src[x];
                    memcpy(tmp + 1, src + x, sizeof(pel)*(3));
                }
                mm_left = _mm_loadl_epi64((__m128i*)(x == 0 ? tmp : src + x - 1));
                if (x == w - 4)
                {
                    memcpy(tmp, src + x + 1, sizeof(pel)*(3));
                    tmp[3] = tmp[2];
                }
                mm_right = _mm_loadl_epi64((__m128i*)(x == w - 4 ? tmp : src + x + 1));

                __m128i mm_gx = _mm_srai_epi16(_mm_sub_epi16(mm_right, mm_left), 4);
                __m128i mm_gy = _mm_srai_epi16(_mm_sub_epi16(mm_bottom, mm_top), 4);

                _mm_storel_epi64((__m128i *)(gy + x), mm_gy);
                _mm_storel_epi64((__m128i *)(gx + x), mm_gx);
            }

            gx += w;
            gy += w;
            src += w;
        }
    }
}

void bio_opt(pel *pred_buf, pel *pred_snd, int w, int h, int bit_depth)
{
    s32 *s1, *s2, *s3, *s5, *s6;
    s16 *p0, *p1, *gx0, *gx1, *gy0, *gy1;
    pel *p0t, *p1t, *gx0t, *gy0t, *gx1t, *gy1t;
    int bio_win_size = BIO_WINDOW_SIZE, bio_cluster_size = BIO_CLUSTER_SIZE, bio_avg_size = BIO_AVG_WIN_SIZE;
#if !SIMD_MC
    int t, tx, ty;
    int jj, ii;
    s32 b;
#endif
    int wb = w, os, ob;
    s16 vx = 0, vy = 0;

    wb = w + (bio_win_size << 1);
    int i, j;

    p0 = pred_buf;
    p1 = pred_snd;
    s1 = sigma[0] + bio_win_size;
    s2 = sigma[1] + bio_win_size;
    s3 = sigma[2] + bio_win_size;
    s5 = sigma[3] + bio_win_size;
    s6 = sigma[4] + bio_win_size;

    gx0 = grad_x[0]; gy0 = grad_y[0]; gx1 = grad_x[1]; gy1 = grad_y[1];
#if SIMD_MC
    bio_sigma(p0, p1, gx0, gx1, gy0, gy1, s1, s2, s3, s5, s6, w, h, bio_win_size, wb);
#else
    for (j = -bio_win_size; j < h + bio_win_size; j++)
    {
        for (i = -bio_win_size; i < w + bio_win_size; i++)
        {
            ii = COM_CLIP3(0, w - 1, i);
            t = p0[ii] - p1[ii];
            tx = gx0[ii] + gx1[ii];
            ty = gy0[ii] + gy1[ii];

            s1[i] = tx * tx;
            s2[i] = tx * ty;
            s3[i] = -tx * t;
            s5[i] = ty * ty;
            s6[i] = -ty * t;
        }
        s1 += wb; s2 += wb; s3 += wb; s5 += wb; s6 += wb;
        if ((j >= 0) && (j < h - 1))
        {
            p0 += w;
            p1 += w;
            gx0 += w;
            gy0 += w;
            gx1 += w;
            gy1 += w;
        }
    }
#endif
    os = bio_win_size + bio_win_size*wb;
    s1 = sigma[0] + os;
    s2 = sigma[1] + os;
    s3 = sigma[2] + os;
    s5 = sigma[3] + os;
    s6 = sigma[4] + os;
    gx0 = grad_x[0];
    gy0 = grad_y[0];
    gx1 = grad_x[1];
    gy1 = grad_y[1];
    p0 = pred_buf;
    p1 = pred_snd;

    for (j = 0; j < h; j += bio_cluster_size)
    {
        for (i = 0; i < w; i += bio_cluster_size)
        {
            os = i - bio_win_size + wb*(j - bio_win_size);
            bio_vxvy(&vx, &vy, wb, s1 + os, s2 + os, s3 + os, s5 + os, s6 + os, bio_avg_size);

            ob = i + j * w;
            p0t = p0 + ob;
            p1t = p1 + ob;
            gx0t = gx0 + ob;
            gy0t = gy0 + ob;
            gx1t = gx1 + ob;
            gy1t = gy1 + ob;
#if SIMD_MC
            bio_block_average_16b_clip_sse(p0t, p1t, gx0t, gy0t, gx1t, gy1t, vx, vy, bio_cluster_size, w, bit_depth);
#else
            {
                for (jj = 0; jj < bio_cluster_size; jj++)
                {
                    for (ii = 0; ii < bio_cluster_size; ii++)
                    {
                        b = vx * (gx0t[ii] - gx1t[ii]) + vy * (gy0t[ii] - gy1t[ii]);
                        b = (b > 0) ? ((b + 32) >> 6) : (-((-b + 32) >> 6));
                        p0t[ii] = (s16)((p0t[ii] + p1t[ii] + b + 1) >> 1);
                        p0t[ii] = COM_CLIP3(0, (1 << bit_depth) - 1, p0t[ii]);
                    }
                    p0t += w;
                    p1t += w;
                    gx0t += w;
                    gy0t += w;
                    gx1t += w;
                    gy1t += w;
                }
            }
#endif
        }
    }
}
#endif

#if SIMD_ASP
static void asp_mv_rounding(s32* hor, s32* ver, s32* rounded_hor, s32* rounded_ver, int size, int shift, int dmvLimit, BOOL hor_on, BOOL ver_on)
{
    int offset = (shift > 0) ? (1 << (shift - 1)) : 0;

    __m128i mm_dmvx, mm_dmvy, mm_mask;
    __m128i mm_zeros = _mm_set1_epi32(0);
    __m128i mm_dIoffset = _mm_set1_epi32(offset);
    __m128i mm_dimax = _mm_set1_epi32(dmvLimit);
    __m128i mm_dimin = _mm_set1_epi32(-dmvLimit);

    if (hor_on)
    {
        for (int i = 0; i < size; i += 4)
        {
            mm_dmvx = _mm_loadu_si128((__m128i*)(hor + i));
            mm_mask = _mm_cmplt_epi32(mm_dmvx, mm_zeros);
            mm_dmvx = _mm_abs_epi32(mm_dmvx);
            mm_dmvx = _mm_srai_epi32(_mm_add_epi32(mm_dmvx, mm_dIoffset), shift);
            mm_dmvx = _mm_xor_si128(_mm_add_epi32(mm_dmvx, mm_mask), mm_mask);
            mm_dmvx = _mm_min_epi32(mm_dimax, _mm_max_epi32(mm_dimin, mm_dmvx));
            _mm_storeu_si128((__m128i*)(rounded_hor + i), mm_dmvx);
        }
    }

    if (ver_on)
    {
        for (int i = 0; i < size; i += 4)
        {
            mm_dmvy = _mm_loadu_si128((__m128i*)(ver + i));
            mm_mask = _mm_cmplt_epi32(mm_dmvy, mm_zeros);
            mm_dmvy = _mm_abs_epi32(mm_dmvy);
            mm_dmvy = _mm_srai_epi32(_mm_add_epi32(mm_dmvy, mm_dIoffset), shift);
            mm_dmvy = _mm_xor_si128(_mm_add_epi32(mm_dmvy, mm_mask), mm_mask);
            mm_dmvy = _mm_min_epi32(mm_dimax, _mm_max_epi32(mm_dimin, mm_dmvy));
            _mm_storeu_si128((__m128i*)(rounded_ver + i), mm_dmvy);
        }
    }
}
#endif // SIMD_ASP

void mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16 mv[REFP_NUM][MV_D], s16(*mv_t)[MV_D])
{
    // 定义最小和最大剪辑值数组
    int min_clip[MV_D], max_clip[MV_D];

    // 将x, y, w, h的值左移两位，相当于乘以4，因为运动矢量的精度是四分之一像素
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;

#if CTU_256
    // 如果定义了CTU_256，则使用CTU大小为256的剪辑值
    min_clip[MV_X] = (-MAX_CU_SIZE2 - 4) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE2 - 4) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE2 + 4) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE2 + 4) << 2;
#else
    // 如果没有定义CTU_256，则使用CTU大小为128的剪辑值
    min_clip[MV_X] = (-MAX_CU_SIZE - 4) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE - 4) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE + 4) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE + 4) << 2;
#endif

    // 将原始运动矢量赋值给目标数组
    mv_t[REFP_0][MV_X] = mv[REFP_0][MV_X];
    mv_t[REFP_0][MV_Y] = mv[REFP_0][MV_Y];
    mv_t[REFP_1][MV_X] = mv[REFP_1][MV_X];
    mv_t[REFP_1][MV_Y] = mv[REFP_1][MV_Y];

    // 对第一个参考图像的运动矢量进行边界检查和调整
    if (REFI_IS_VALID(refi[REFP_0]))
    {
        if (x + mv[REFP_0][MV_X] < min_clip[MV_X])
            mv_t[REFP_0][MV_X] = (s16)(min_clip[MV_X] - x);
        if (y + mv[REFP_0][MV_Y] < min_clip[MV_Y])
            mv_t[REFP_0][MV_Y] = (s16)(min_clip[MV_Y] - y);
        if (x + mv[REFP_0][MV_X] + w - 4 > max_clip[MV_X])
            mv_t[REFP_0][MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
        if (y + mv[REFP_0][MV_Y] + h - 4 > max_clip[MV_Y])
            mv_t[REFP_0][MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
    }

    // 对第二个参考图像的运动矢量进行边界检查和调整
    if (REFI_IS_VALID(refi[REFP_1]))
    {
        if (x + mv[REFP_1][MV_X] < min_clip[MV_X])
            mv_t[REFP_1][MV_X] = (s16)(min_clip[MV_X] - x);
        if (y + mv[REFP_1][MV_Y] < min_clip[MV_Y])
            mv_t[REFP_1][MV_Y] = (s16)(min_clip[MV_Y] - y);
        if (x + mv[REFP_1][MV_X] + w - 4 > max_clip[MV_X])
            mv_t[REFP_1][MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
        if (y + mv[REFP_1][MV_Y] + h - 4 > max_clip[MV_Y])
            mv_t[REFP_1][MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
    }
}

#if INTER_TM
void single_mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[MV_D])
{
    int min_clip[MV_D], max_clip[MV_D];
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;
    min_clip[MV_X] = (-MAX_CU_SIZE2 - 4) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE2 - 4) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE2 + 4) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE2 + 4) << 2;

    if (x + mv[MV_X] < min_clip[MV_X]) mv[MV_X] = (s16)(min_clip[MV_X] - x);
    if (y + mv[MV_Y] < min_clip[MV_Y]) mv[MV_Y] = (s16)(min_clip[MV_Y] - y);
    if (x + mv[MV_X] + w - 4 > max_clip[MV_X]) mv[MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
    if (y + mv[MV_Y] + h - 4 > max_clip[MV_Y]) mv[MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
}
#endif

#if DMVR
// 函数声明，用于裁剪单个参考的运动矢量
static BOOL mv_clip_only_one_ref_dmvr(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[MV_D], s16(*mv_t))
{
    // 定义一个标志，用于指示是否需要裁剪
    BOOL clip_flag = 0;

    // 定义最小和最大剪辑值数组
    int min_clip[MV_D], max_clip[MV_D];

    // 将x, y, w, h的值左移两位，相当于乘以4，因为运动矢量的精度是四分之一像素
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;

    // 根据CTU_256宏定义选择裁剪边界
#if CTU_256
    // 如果定义了CTU_256，则使用CTU大小为256的裁剪值
    min_clip[MV_X] = (-MAX_CU_SIZE2) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE2) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE2) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE2) << 2;
#else
    // 如果没有定义CTU_256，则使用CTU大小为128的裁剪值
    min_clip[MV_X] = (-MAX_CU_SIZE) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE) << 2;
#endif

    // 将原始运动矢量赋值给目标数组
    mv_t[MV_X] = mv[MV_X];
    mv_t[MV_Y] = mv[MV_Y];

    // 检查并裁剪水平运动矢量
    if (x + mv[MV_X] < min_clip[MV_X])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[MV_X] = min_clip[MV_X] - x;  // 裁剪运动矢量
    }
    if (y + mv[MV_Y] < min_clip[MV_Y])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[MV_Y] = min_clip[MV_Y] - y;  // 裁剪运动矢量
    }
    if (x + mv[MV_X] + w - 4 > max_clip[MV_X])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[MV_X] = max_clip[MV_X] - x - w + 4;  // 裁剪运动矢量
    }
    if (y + mv[MV_Y] + h - 4 > max_clip[MV_Y])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[MV_Y] = max_clip[MV_Y] - y - h + 4;  // 裁剪运动矢量
    }

    // 返回是否进行了裁剪的标志
    return clip_flag;
}

#ifdef X86_SSE
#define SSE_SAD_16B_4PEL(src1, src2, s00, s01, sac0) \
        s00 = _mm_loadl_epi64((__m128i*)(src1)); \
        s01 = _mm_loadl_epi64((__m128i*)(src2));\
        s00 = _mm_sub_epi16(s00, s01); \
        s00 = _mm_abs_epi16(s00); \
        s00 = _mm_cvtepi16_epi32(s00); \
        \
        sac0 = _mm_add_epi32(sac0, s00);
#endif

#define SSE_SAD_16B_8PEL(src1, src2, s00, s01, sac0) \
        s00 = _mm_loadu_si128((__m128i*)(src1)); \
        s01 = _mm_loadu_si128((__m128i*)(src2)); \
        s00 = _mm_sub_epi16(s00, s01); \
        s01 = _mm_abs_epi16(s00); \
        \
        s00 = _mm_hadd_epi16(s01, s01); \
        s00 = _mm_cvtepi16_epi32(s00); \
        \
        sac0 = _mm_add_epi32(sac0, s00);

s32 com_DMVR_cost(int w, int h, pel* src1, pel* src2, int s_src1, int s_src2)
{
    int sad;
    s16* s1;
    s16* s2;
    __m128i s00, s01, sac0, sac1;
    int i, j;
    s1 = (s16*)src1;  // 将输入的图像块指针转换为16位整数指针
    s2 = (s16*)src2;
    sac0 = _mm_setzero_si128();  // 初始化SSE累加器为0
    sac1 = _mm_setzero_si128();

    if (w & 0x07)  // 检查宽度是否是8的倍数
    {
        for (i = 0; i < (h >> 2); i++)  // 按四次一组处理高度
        {
            for (j = 0; j < w; j += 4)  // 按四像素一组处理宽度
            {
                SSE_SAD_16B_4PEL(s1 + j, s2 + j, s00, s01, sac0);  // 计算SAD并使用SSE指令
                SSE_SAD_16B_4PEL(s1 + j + s_src1, s2 + j + s_src2, s00, s01, sac0);
                SSE_SAD_16B_4PEL(s1 + j + s_src1 * 2, s2 + j + s_src2 * 2, s00, s01, sac0);
                SSE_SAD_16B_4PEL(s1 + j + s_src1 * 3, s2 + j + s_src2 * 3, s00, s01, sac0);
            }
            s1 += s_src1 << 2;  // 移动到下一组像素
            s2 += s_src2 << 2;
        }
        sad = _mm_extract_epi32(sac0, 0);
        sad += _mm_extract_epi32(sac0, 1);
        sad += _mm_extract_epi32(sac0, 2);
        sad += _mm_extract_epi32(sac0, 3);
    }
    else  // 如果宽度是8的倍数
    {
        for (i = 0; i < (h >> 2); i++)
        {
            for (j = 0; j < w; j += 8)  // 按八像素一组处理宽度
            {
                SSE_SAD_16B_8PEL(s1 + j, s2 + j, s00, s01, sac0);  // 计算SAD并使用SSE指令
                SSE_SAD_16B_8PEL(s1 + j + s_src1, s2 + j + s_src2, s00, s01, sac0);
                SSE_SAD_16B_8PEL(s1 + j + s_src1 * 2, s2 + j + s_src2 * 2, s00, s01, sac0);
                SSE_SAD_16B_8PEL(s1 + j + s_src1 * 3, s2 + j + s_src2 * 3, s00, s01, sac0);
            }
            s1 += s_src1 << 2;
            s2 += s_src2 << 2;
        }
        sad = _mm_extract_epi32(sac0, 0);
        sad += _mm_extract_epi32(sac0, 1);
        sad += _mm_extract_epi32(sac0, 2);
        sad += _mm_extract_epi32(sac0, 3);
    }

    return sad;  // 返回计算的SAD值
}


typedef enum _SAD_POINT_INDEX
{
    SAD_NOT_AVAILABLE = -1,
    SAD_BOTTOM = 0,
    SAD_TOP,
    SAD_RIGHT,
    SAD_LEFT,
    SAD_TOP_LEFT,
    SAD_TOP_RIGHT,
    SAD_BOTTOM_LEFT,
    SAD_BOTTOM_RIGHT,
    SAD_CENTER,
    SAD_COUNT
} SAD_POINT_INDEX;

#define NUM_SAD_POINTS 21
void com_dmvr_refine(
    int w, int h, pel* ref_l0, int s_ref_l0, pel* ref_l1, int s_ref_l1,
    s32* min_cost, s16* delta_mvx, s16* delta_mvy, s32* sad_array)
{
    SAD_POINT_INDEX idx; // 用于遍历SAD点的索引
    s32 search_offset_x_back[5] = { 0, 0, 1, -1, 0 }; // 用于回溯搜索的X偏移量
    s32 search_offset_y_back[5] = { 1, -1, 0, 0, 0 }; // 用于回溯搜索的Y偏移量
    s32 j = 0; // 迭代变量
    s32 start_x = 0; // X方向起始偏移量
    s32 start_y = 0; // Y方向起始偏移量
    s32 search_offset_x[NUM_SAD_POINTS] = { 0, 0, 0, 1, -1, 0, 1, 2, 1, 0, -1, -2, -1, 2, 1, 2, 1, -1, -2, -2, -1 };
    s32 search_offset_y[NUM_SAD_POINTS] = { 0, 1, -1, 0, 0, 2, 1, 0, -1, -2, -1, 0, 1, 1, 2, -1, -2, -2, -1, 1, 2 };
    s32 cost_temp[5][5] = { { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX } };

    s32 center_x = 2; // 中心点的X索引
    s32 center_y = 2; // 中心点的Y索引

    pel* ref_l0_Orig = ref_l0; // 原始参考图像L0的指针
    pel* ref_l1_Orig = ref_l1; // 原始参考图像L1的指针
    for (idx = 0; idx < NUM_SAD_POINTS; ++idx) // 遍历所有SAD点
    {
        int sum = 0;
        ref_l0 = ref_l0_Orig + search_offset_x[idx] + search_offset_y[idx] * s_ref_l0; // 计算新的参考图像L0的指针
        ref_l1 = ref_l1_Orig - search_offset_x[idx] - search_offset_y[idx] * s_ref_l1; // 计算新的参考图像L1的指针
        s32 cost = com_DMVR_cost(w, h, ref_l0, ref_l1, s_ref_l0, s_ref_l1); // 计算当前点的成本
        cost_temp[center_y + search_offset_y[idx]][center_x + search_offset_x[idx]] = cost; // 存储成本
    }

    *min_cost = cost_temp[center_y][center_x]; // 初始化最小成本为当前中心点的成本
    for (j = 0; j < 3; j++) // 进行三次迭代以细化运动矢量
    {
        s32 delta_x = 0; // X方向的增量
        s32 delta_y = 0; // Y方向的增量

        for (idx = SAD_BOTTOM; idx < SAD_TOP_LEFT; ++idx) // 遍历回溯搜索点
        {
            s32 y_offset = center_y + start_y + search_offset_y_back[idx]; // 计算Y偏移量
            s32 x_offset = center_x + start_x + search_offset_x_back[idx]; // 计算X偏移量
            if (x_offset >= 0 && x_offset < 5 && y_offset >= 0 && y_offset < 5) // 确保偏移量在有效范围内
            {
                s32 cost = cost_temp[center_y + start_y + search_offset_y_back[idx]][center_x + start_x + search_offset_x_back[idx]]; // 获取当前点的成本
                if (cost < *min_cost) // 如果找到更小的成本
                {
                    *min_cost = cost; // 更新最小成本
                    delta_x = search_offset_x_back[idx]; // 更新X方向增量
                    delta_y = search_offset_y_back[idx]; // 更新Y方向增量
                    if (cost == 0) // 如果成本为0，直接跳出循环
                    {
                        break;
                    }
                }
            }
        }
        start_x += delta_x; // 更新起始X偏移量
        start_y += delta_y; // 更新起始Y偏移量
        if (*min_cost == 0) // 如果最小成本为0，跳出循环
        {
            break;
        }

        if ((delta_x == 0) && (delta_y == 0)) // 如果没有更新，跳出循环
        {
            break;
        }
    }
    *delta_mvx = start_x; // 更新最小成本对应的X增量
    *delta_mvy = start_y; // 更新最小成本对应的Y增量

    if ((abs(*delta_mvx) < center_x) && (abs(*delta_mvy) < center_y)) // 如果增量在中心点附近
    {
        *(sad_array + SAD_CENTER) = *min_cost; // 更新SAD数组中的中心点成本
        for (idx = SAD_BOTTOM; idx < SAD_TOP_LEFT; ++idx) // 遍历所有SAD点
        {
            *(sad_array + idx) = cost_temp[center_y + (*delta_mvy) + search_offset_y_back[idx]][center_x + (*delta_mvx) + search_offset_x_back[idx]]; // 更新SAD数组
        }
    }
    ref_l0 = ref_l0_Orig; // 恢复原始的参考图像L0的指针
    ref_l1 = ref_l1_Orig; // 恢复原始的参考图像L1的指针
}

s32 div_for_maxq7( s64 N, s64 D )
{
    s32 sign, q;

    sign = 0;
    if (N < 0)
    {
        sign = 1;
        N = -N;
    }

    q = 0;
    D = (D << 3);
    if (N >= D)
    {
        N -= D;
        q++;
    }
    q = (q << 1);

    D = (D >> 1);
    if (N >= D)
    {
        N -= D;
        q++;
    }
    q = (q << 1);

    if (N >= (D >> 1))
    {
        q++;
    }

    if (sign)
    {
        return (-q);
    }
    return(q);
}

void com_sub_pel_error_surface( int *sad_buffer, int *delta_mv )
{
    s64 num, denorm;
    int mv_delta_subpel;
    int mv_subpel_level = 4; //1: half pel, 2: Qpel, 3:1/8, 4: 1/16
    
    /*horizontal*/
    num = (s64)((sad_buffer[1] - sad_buffer[3]) << mv_subpel_level);
    denorm = (s64)((sad_buffer[1] + sad_buffer[3] - (sad_buffer[0] << 1)));

    if( 0 != denorm )
    {
        if( (sad_buffer[1] != sad_buffer[0]) && (sad_buffer[3] != sad_buffer[0]) )
        {
            mv_delta_subpel = div_for_maxq7( num, denorm );
            delta_mv[0] = (mv_delta_subpel);
        }
        else
        {
            if( sad_buffer[1] == sad_buffer[0] )
            {
                delta_mv[0] = -8;// half pel
            }
            else
            {
                delta_mv[0] = 8;// half pel
            }
        }
    }
    /*vertical*/
    num = (s64)((sad_buffer[2] - sad_buffer[4]) << mv_subpel_level);
    denorm = (s64)((sad_buffer[2] + sad_buffer[4] - (sad_buffer[0] << 1)));
    if( 0 != denorm )
    {
        if( (sad_buffer[2] != sad_buffer[0]) && (sad_buffer[4] != sad_buffer[0]) )
        {
            mv_delta_subpel = div_for_maxq7( num, denorm );
            delta_mv[1] = (mv_delta_subpel);
        }
        else
        {
            if( sad_buffer[2] == sad_buffer[0] )
            {
                delta_mv[1] = -8;// half pel
            }
            else
            {
                delta_mv[1] = 8;// half pel
            }
        }
    }
    return;
}

#if DMVR
void copy_buffer(pel* src, int src_stride, pel* dst, int dst_stride, int width, int height)
{
    // 计算每行的字节数
    int num_bytes = width * sizeof(pel);
    // 逐行复制
    for (int i = 0; i < height; i++)
    {
        // 使用memcpy函数复制每行的数据
        memcpy(dst + i * dst_stride, src + i * src_stride, num_bytes);
    }
}

void padding(pel* ptr, int stride, int width, int height, int pad_left_size, int pad_right_size, int pad_top_size, int pad_bottom_size)
{
    // 左填充
    pel* ptr_temp = ptr;
    int offset = 0;
    for (int i = 0; i < height; i++)
    {
        offset = stride * i;
        for (int j = 1; j <= pad_left_size; j++)
        {
            *(ptr_temp - j + offset) = *(ptr_temp + offset);
        }
    }

    // 右填充
    ptr_temp = ptr + (width - 1);
    for (int i = 0; i < height; i++)
    {
        offset = stride * i;
        for (int j = 1; j <= pad_right_size; j++)
        {
            *(ptr_temp + j + offset) = *(ptr_temp + offset);
        }
    }

    // 上填充
    int num_bytes = (width + pad_left_size + pad_right_size) * sizeof(pel);
    ptr_temp = (ptr - pad_left_size);
    for (int i = 1; i <= pad_top_size; i++)
    {
        memcpy(ptr_temp - (i * stride), (ptr_temp), num_bytes);
    }

    // 下填充
    num_bytes = (width + pad_left_size + pad_right_size) * sizeof(pel);
    ptr_temp = (ptr + (stride * (height - 1)) - pad_left_size);
    for (int i = 1; i <= pad_bottom_size; i++)
    {
        memcpy(ptr_temp + (i * stride), (ptr_temp), num_bytes);
    }
}

static void prefetch_for_sub_pu_mc(
    int x, int y, int pic_w, int pic_h, int w, int h,
    int x_sub, int y_sub, int w_sub, int h_sub,
    s8 refi[REFP_NUM], s16(*mv)[MV_D],
    COM_REFP(*refp)[REFP_NUM],
    int iteration,
    pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
)
{
    s16 mv_temp[REFP_NUM][MV_D];  // 临时存储裁剪后的运动矢量
    int l_w = w, l_h = h;         // 原始块的宽度和高度
    int c_w = w >> 1, c_h = h >> 1; // 子块的宽度和高度（假设是原始块的一半）
    int num_extra_pixel_left_for_filter; // 滤波器额外需要的像素数
    for (int i = 0; i < REFP_NUM; ++i)
    {
        int filter_size = NTAPS_LUMA; // 亮度滤波器的系数数量
        num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1); // 计算左侧额外像素数
        int offset = ((DMVR_ITER_COUNT) * (PAD_BUFFER_STRIDE + 1)); // 计算偏移量
        int pad_size = DMVR_PAD_LENGTH; // 填充大小
        int qpel_gmv_x, qpel_gmv_y; // 四分之一像素精度的运动矢量
        COM_PIC* ref_pic; // 引用图像指针
        mv_clip_only_one_ref_dmvr(x, y, pic_w, pic_h, w, h, mv[i], mv_temp[i]); // 裁剪运动矢量

        // 计算四分之一像素精度的全局运动矢量
        qpel_gmv_x = (((x + x_sub) << 2) + mv_temp[i][MV_X]);
        qpel_gmv_y = (((y + y_sub) << 2) + mv_temp[i][MV_Y]);

        ref_pic = refp[refi[i]][i].pic; // 获取参考图像
        pel* ref = ref_pic->y + ((qpel_gmv_y >> 2) - num_extra_pixel_left_for_filter) * ref_pic->stride_luma +
            (qpel_gmv_x >> 2) - num_extra_pixel_left_for_filter; // 计算亮度分量的引用地址
        pel* dst = dmvr_padding_buf[i][0] + offset; // 计算目标缓冲区的地址
        copy_buffer(ref, ref_pic->stride_luma, dst, PAD_BUFFER_STRIDE, (w_sub + filter_size), (h_sub + filter_size)); // 复制亮度分量
        padding(dst, PAD_BUFFER_STRIDE, (w_sub + filter_size), (h_sub + filter_size), pad_size, pad_size, pad_size, pad_size); // 填充亮度分量

        // 对色度分量进行相同的处理
        filter_size = NTAPS_CHROMA; // 色度滤波器的系数数量
        num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1); // 重新计算左侧额外像素数
        offset = (DMVR_ITER_COUNT); // 重新计算偏移量
        offset = offset * (PAD_BUFFER_STRIDE + 1); // 重新计算偏移量
        pad_size = DMVR_PAD_LENGTH >> 1; // 色度填充大小（通常是亮度的一半）

        // 处理U分量
        ref = ref_pic->u + ((qpel_gmv_y >> 3) - num_extra_pixel_left_for_filter) * ref_pic->stride_chroma +
            (qpel_gmv_x >> 3) - num_extra_pixel_left_for_filter;
        dst = dmvr_padding_buf[i][1] + offset;
        copy_buffer(ref, ref_pic->stride_chroma, dst, PAD_BUFFER_STRIDE, ((w_sub >> 1) + filter_size), ((h_sub >> 1) + filter_size));
        padding(dst, PAD_BUFFER_STRIDE, ((w_sub >> 1) + filter_size), ((h_sub >> 1) + filter_size), pad_size, pad_size, pad_size, pad_size);

        // 处理V分量
        ref = ref_pic->v + ((qpel_gmv_y >> 3) - num_extra_pixel_left_for_filter) * ref_pic->stride_chroma +
            (qpel_gmv_x >> 3) - num_extra_pixel_left_for_filter;
        dst = dmvr_padding_buf[i][2] + offset;
        copy_buffer(ref, ref_pic->stride_chroma, dst, PAD_BUFFER_STRIDE, ((w_sub >> 1) + filter_size), ((h_sub >> 1) + filter_size));
        padding(dst, PAD_BUFFER_STRIDE, ((w_sub >> 1) + filter_size), ((h_sub >> 1) + filter_size), pad_size, pad_size, pad_size, pad_size);
    }
}
#endif

static void final_padded_MC_for_dmvr(
    int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM],
    s16(*inital_mv)[MV_D], s16(*refined_mv)[MV_D], COM_REFP(*refp)[REFP_NUM],
    pel pred[N_C][MAX_CU_DIM], pel pred1[N_C][MAX_CU_DIM],
    int sub_pred_offset_x, int sub_pred_offset_y, int cu_pred_stride, int bit_depth,
    pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
#if BIO
    , int apply_BIO
#endif
)
{
    int i;
    COM_PIC* ref_pic;
    s16 mv_temp[REFP_NUM][MV_D];
    pel(*pred_buf)[MAX_CU_DIM] = pred;

    for (i = 0; i < REFP_NUM; ++i) // 遍历所有参考图像
    {
        int qpel_gmv_x, qpel_gmv_y;
        if (i > 0)
        {
            pred_buf = pred1; // 如果有多个参考图像，使用第二个预测缓冲区
        }
        ref_pic = refp[refi[i]][i].pic; // 获取参考图像

        s16 temp_uncliped_mv[MV_D] = { refined_mv[i][MV_X], refined_mv[i][MV_Y] };
        BOOL clip_flag = mv_clip_only_one_ref_dmvr(x, y, pic_w, pic_h, w, h, temp_uncliped_mv, mv_temp[i]);

        if (clip_flag)
        {
            qpel_gmv_x = (x << 2) + (mv_temp[i][MV_X]);
            qpel_gmv_y = (y << 2) + (mv_temp[i][MV_Y]);
        }
        else
        {
            qpel_gmv_x = (x << 2) + (refined_mv[i][MV_X]);
            qpel_gmv_y = (y << 2) + (refined_mv[i][MV_Y]);
        }

        int delta_x_l = 0;
        int delta_y_l = 0;
        int delta_x_c = 0;
        int delta_y_c = 0;
        int offset = 0;
        int filter_size = NTAPS_LUMA;
        int num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);

        if (clip_flag == 0)
        {
            // 计算从初始运动矢量到细化运动矢量的整数像素偏移
            delta_x_l = (refined_mv[i][MV_X] >> 2) - (inital_mv[i][MV_X] >> 2);
            delta_y_l = (refined_mv[i][MV_Y] >> 2) - (inital_mv[i][MV_Y] >> 2);
            delta_x_c = (refined_mv[i][MV_X] >> 3) - (inital_mv[i][MV_X] >> 3);
            delta_y_c = (refined_mv[i][MV_Y] >> 3) - (inital_mv[i][MV_Y] >> 3);
        }
        else
        {
            // 计算从裁剪后的运动矢量到初始运动矢量的整数像素偏移
            delta_x_l = (mv_temp[i][MV_X] >> 2) - (inital_mv[i][MV_X] >> 2);
            delta_y_l = (mv_temp[i][MV_Y] >> 2) - (inital_mv[i][MV_Y] >> 2);
            delta_x_c = (mv_temp[i][MV_X] >> 3) - (inital_mv[i][MV_X] >> 3);
            delta_y_c = (mv_temp[i][MV_Y] >> 3) - (inital_mv[i][MV_Y] >> 3);

            int luma_pad = DMVR_ITER_COUNT;
            int chroma_pad = DMVR_ITER_COUNT >> 1;
            delta_x_l = COM_CLIP3(-luma_pad, luma_pad, delta_x_l);
            delta_y_l = COM_CLIP3(-luma_pad, luma_pad, delta_y_l);
            delta_x_c = COM_CLIP3(-chroma_pad, chroma_pad, delta_x_c);
            delta_y_c = COM_CLIP3(-chroma_pad, chroma_pad, delta_y_c);
        }

        assert(DMVR_ITER_COUNT == 2);
        assert(delta_x_l >= -DMVR_ITER_COUNT && delta_x_l <= DMVR_ITER_COUNT);
        assert(delta_y_l >= -DMVR_ITER_COUNT && delta_y_l <= DMVR_ITER_COUNT);
        assert(delta_x_c >= -(DMVR_ITER_COUNT >> 1) && delta_x_c <= (DMVR_ITER_COUNT >> 1));
        assert(delta_y_c >= -(DMVR_ITER_COUNT >> 1) && delta_y_c <= (DMVR_ITER_COUNT >> 1));

        offset = (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
        offset += (delta_y_l)*PAD_BUFFER_STRIDE;
        offset += (delta_x_l);

        pel* src = dmvr_padding_buf[i][0] + offset;
        pel* temp = pred_buf[Y_C] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
        com_dmvr_mc_l(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp, w, h, bit_depth);

#if BIO
        if (apply_BIO)
        {
            pel* temp_grad_x = grad_x[i] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
            pel* temp_grad_y = grad_y[i] + sub_pred_offset_x + sub_pred_offset_y * cu_pred_stride;
            com_grad_x_l_nn(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp_grad_x, w, h, bit_depth, 1);
            com_grad_y_l_nn(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride, temp_grad_y, w, h, bit_depth, 1);
        }
#endif

        filter_size = NTAPS_CHROMA;
        num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);
        offset = (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
        offset += (delta_y_c)*PAD_BUFFER_STRIDE;
        offset += (delta_x_c);

        src = dmvr_padding_buf[i][1] + offset;
        temp = pred_buf[U_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
        com_dmvr_mc_c(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride >> 1, temp, w >> 1, h >> 1, bit_depth);

        src = dmvr_padding_buf[i][2] + offset;
        temp = pred_buf[V_C] + (sub_pred_offset_x >> 1) + (sub_pred_offset_y >> 1) * (cu_pred_stride >> 1);
        com_dmvr_mc_c(src, qpel_gmv_x, qpel_gmv_y, PAD_BUFFER_STRIDE, cu_pred_stride >> 1, temp, w >> 1, h >> 1, bit_depth);
    }
}

// from int pos, so with full error surface
void process_DMVR( int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], s16( *mv )[MV_D], COM_REFP( *refp )[REFP_NUM], pel pred[N_C][MAX_CU_DIM], pel pred1[N_C][MAX_CU_DIM]
    , int poc_c, pel *dmvr_current_template, pel( *dmvr_ref_pred_interpolated )[(MAX_CU_SIZE + (2 * (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT)) * (MAX_CU_SIZE + (2 * (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT))]
    , int iteration, int bit_depth, pel( *dmvr_padding_buf )[N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
#if BIO
    , int apply_BIO
#endif
)
{
    // 定义两个数组，用于存储两个参考图像平面（L0和L1）的子块（sub-PU）运动矢量
    s16 sub_pu_L0[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];
    s16 sub_pu_L1[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];

    // 定义一个缩放步长，用于后续的矢量缩放计算
    s16 ref_pred_mv_scaled_step = 2;

    // 定义一个数组，用于存储经过初步处理的运动矢量
    s16 refined_mv[REFP_NUM][MV_D] = { { mv[REFP_0][MV_X], mv[REFP_0][MV_Y] },
                                       { mv[REFP_1][MV_X], mv[REFP_1][MV_Y] }
    };

    // 定义一个数组，用于存储裁剪后的起始运动矢量
    s16 starting_mv[REFP_NUM][MV_D];
    if (refined_mv[0][MV_X] > 2)
    {
        // 调用mv_clip函数，对运动矢量进行裁剪，确保它们不会超出图像边界
        mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, starting_mv);
    }

    // 形成整数部分的运动矢量
    s16 sign_x = starting_mv[REFP_0][MV_X] >= 0 ? 1 : (-1); // 确定x分量的符号
    s16 sign_y = starting_mv[REFP_0][MV_Y] >= 0 ? 1 : (-1); // 确定y分量的符号
    s16 abs_x = abs(starting_mv[REFP_0][MV_X]); // 计算x分量的绝对值
    s16 abs_y = abs(starting_mv[REFP_0][MV_Y]); // 计算y分量的绝对值
    s16 xFrac = abs_x & 3; // 计算x分量的小数部分
    s16 yFrac = abs_y & 3; // 计算y分量的小数部分

    // 裁剪绝对运动矢量，确保它们不会超过最大允许值
    short max_abs_mv = (1 << 15) - 1 - (1 << 2);
    abs_x = COM_MIN(abs_x, max_abs_mv); // 裁剪x分量
    abs_y = COM_MIN(abs_y, max_abs_mv); // 裁剪y分量

    // 将裁剪后的绝对运动矢量转换回带符号的整数运动矢量
    starting_mv[REFP_0][MV_X] = (((abs_x) >> 2) << 2); // 将x分量的小数部分舍去
    starting_mv[REFP_0][MV_Y] = (((abs_y) >> 2) << 2); // 将y分量的小数部分舍去
    starting_mv[REFP_0][MV_X] += (((xFrac > 2) ? 1 : 0) << 2); // 如果x分量的小数部分大于2，则增加4（相当于0.5像素）
    starting_mv[REFP_0][MV_Y] += (((yFrac > 2) ? 1 : 0) << 2); // 如果y分量的小数部分大于2，则增加4（相当于0.5像素）
    starting_mv[REFP_0][MV_X] *= sign_x; // 恢复x分量的符号
    starting_mv[REFP_0][MV_Y] *= sign_y; // 恢复y分量的符号

    sign_x = starting_mv[REFP_1][MV_X] >= 0 ? 1 : (-1);
    sign_y = starting_mv[REFP_1][MV_Y] >= 0 ? 1 : (-1);
    abs_x = abs( starting_mv[REFP_1][MV_X] );
    abs_y = abs( starting_mv[REFP_1][MV_Y] );

    // clip ABSmv
    abs_x = COM_MIN(abs_x, max_abs_mv);
    abs_y = COM_MIN(abs_y, max_abs_mv);

    xFrac = abs_x & 3;
    yFrac = abs_y & 3;
    starting_mv[REFP_1][MV_X] = (((abs_x) >> 2) << 2);
    starting_mv[REFP_1][MV_Y] = (((abs_y) >> 2) << 2);
    starting_mv[REFP_1][MV_X] += (((xFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_1][MV_Y] += (((yFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_1][MV_X] *= sign_x;
    starting_mv[REFP_1][MV_Y] *= sign_y;

    // clip ABSmv
    short max_dmvr_mv =  (1 << 15) - (3 << 2);
    short min_dmvr_mv = -(1 << 15) + (2 << 2);
    starting_mv[REFP_0][MV_X] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_0][MV_X]));
    starting_mv[REFP_0][MV_Y] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_0][MV_Y]));
    starting_mv[REFP_1][MV_X] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_1][MV_X]));
    starting_mv[REFP_1][MV_Y] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_1][MV_Y]));

    // centre address holder for pred
    // 定义两个数组，用于存储参考图像的预测块地址
    pel* preds_array[REFP_NUM];
    pel* preds_centre_array[REFP_NUM];

    // 定义步长，通常用于图像处理中的像素访问
    int stride = PAD_BUFFER_STRIDE;

    // 初始化preds_array数组，存储两个参考图像的预测块地址
    preds_array[REFP_0] = dmvr_padding_buf[REFP_0][Y_C];
    preds_array[REFP_1] = dmvr_padding_buf[REFP_1][Y_C];

    // 定义滤波器的大小，这里使用的是NTAPS_LUMA，通常用于滤波器设计
    int filter_size = NTAPS_LUMA;

    // 计算滤波器左侧额外像素的数量
    int num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);

    // 计算预测块的中心地址，以便进行滤波处理
    preds_centre_array[REFP_0] = preds_array[REFP_0] + (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
    preds_centre_array[REFP_1] = preds_array[REFP_1] + (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));

    // 初始化最小代价变量，用于后续的最优预测块搜索
    int min_cost = INT_MAX;

    // 初始化最后一个方向变量，用于记录搜索过程中的方向
    int last_direction = -1;

    // 定义一个数组，用于存储不同方向的代价
    int array_cost[SAD_COUNT];

    // 定义变量dx和dy，用于存储搜索区域的宽度和高度
    int dx, dy;
    dy = min(h, DMVR_IMPLIFICATION_SUBCU_SIZE); // 取高度和DMVR_IMPLIFICATION_SUBCU_SIZE中的较小值
    dx = min(w, DMVR_IMPLIFICATION_SUBCU_SIZE); // 取宽度和DMVR_IMPLIFICATION_SUBCU_SIZE中的较小值

    // 定义变量用于计数和跟踪子预测单元（sub-PU）的起始坐标
    int num = 0;
    int sub_pu_start_x, sub_pu_start_y, start_x, start_y;

    // 外层循环：按子预测单元的高度迭代
    for (start_y = 0, sub_pu_start_y = y; sub_pu_start_y < (y + h); sub_pu_start_y += dy, start_y += dy)
    {
        // 内层循环：按子预测单元的宽度迭代
        for (start_x = 0, sub_pu_start_x = x; sub_pu_start_x < (x + w); sub_pu_start_x += dx, start_x += dx)
        {
            // 初始化总的运动矢量变化量
            s16 total_delta_mv[MV_D] = { 0, 0 };
            BOOL not_zero_cost = 1;  // 标志，用于检测是否找到非零成本的预测

            // 为当前子预测单元生成填充缓冲区，以便进行精确的运动补偿
            prefetch_for_sub_pu_mc(x, y, pic_w, pic_h, w, h, start_x, start_y, dx, dy, refi, starting_mv, refp, iteration, dmvr_padding_buf);

            // 获取当前子预测单元的中心地址
            pel* addr_subpu_l0 = preds_centre_array[REFP_0];
            pel* addr_subpu_l1 = preds_centre_array[REFP_1];

            // 初始化成本数组，用于存储不同方向的成本
            for (int loop = 0; loop < SAD_COUNT; loop++)
            {
                array_cost[loop] = INT_MAX;
            }

            // 调用运动矢量细化函数，尝试找到最小成本
            min_cost = INT_MAX;
            com_dmvr_refine(dx, dy, addr_subpu_l0, stride, addr_subpu_l1, stride,
                &min_cost,
                &total_delta_mv[MV_X], &total_delta_mv[MV_Y],
                array_cost);

            // 如果找到的成本为0，说明找到了完美的匹配，不需要进一步调整
            if (min_cost == 0)
            {
                not_zero_cost = 0;
            }

            // 将总的运动矢量变化量左移两位，以恢复到原始的四分之一像素精度
            total_delta_mv[MV_X] = (total_delta_mv[MV_X] << 2);
            total_delta_mv[MV_Y] = (total_delta_mv[MV_Y] << 2);

            // 如果成本非零，并且所有方向的成本都已计算，则尝试进一步细化运动矢量
            if (not_zero_cost && (min_cost == array_cost[SAD_CENTER])
                && (array_cost[SAD_CENTER] != INT_MAX)
                && (array_cost[SAD_LEFT] != INT_MAX)
                && (array_cost[SAD_TOP] != INT_MAX)
                && (array_cost[SAD_RIGHT] != INT_MAX)
                && (array_cost[SAD_BOTTOM] != INT_MAX))
            {
                int sadbuffer[5];  // 存储不同方向的成本
                int deltaMv[MV_D] = { 0, 0 };  // 存储进一步细化后的运动矢量变化量

                // 将成本存储到缓冲区，用于进一步细化
                sadbuffer[0] = array_cost[SAD_CENTER];
                sadbuffer[1] = array_cost[SAD_LEFT];
                sadbuffer[2] = array_cost[SAD_TOP];
                sadbuffer[3] = array_cost[SAD_RIGHT];
                sadbuffer[4] = array_cost[SAD_BOTTOM];
                com_sub_pel_error_surface(sadbuffer, deltaMv);

                // 将细化后的变化量应用到总的运动矢量变化量
                total_delta_mv[MV_X] += deltaMv[MV_X] >> 2;
                total_delta_mv[MV_Y] += deltaMv[MV_Y] >> 2;
            }

            // 更新参考图像0和1的细化后运动矢量
            refined_mv[REFP_0][MV_X] = (starting_mv[REFP_0][MV_X]) + (total_delta_mv[MV_X]);
            refined_mv[REFP_0][MV_Y] = (starting_mv[REFP_0][MV_Y]) + (total_delta_mv[MV_Y]);

            refined_mv[REFP_1][MV_X] = (starting_mv[REFP_1][MV_X]) - (total_delta_mv[MV_X]);
            refined_mv[REFP_1][MV_Y] = (starting_mv[REFP_1][MV_Y]) - (total_delta_mv[MV_Y]);

            // 存储每个子预测单元的细化后运动矢量
            sub_pu_L0[num][MV_X] = refined_mv[REFP_0][MV_X];
            sub_pu_L0[num][MV_Y] = refined_mv[REFP_0][MV_Y];

            sub_pu_L1[num][MV_X] = refined_mv[REFP_1][MV_X];
            sub_pu_L1[num][MV_Y] = refined_mv[REFP_1][MV_Y];

            // 对当前子预测单元进行最终的运动补偿
            final_padded_MC_for_dmvr(sub_pu_start_x, sub_pu_start_y, pic_w, pic_h, dx, dy, refi, starting_mv, refined_mv, refp, pred, pred1,
                start_x, start_y, w, bit_depth, dmvr_padding_buf
    #if BIO
                , apply_BIO
    #endif
            );

            // 增加子预测单元的计数
            num++;
        }
    }
}
#endif

#if AFFINE_DMVR
static BOOL affine_mv_clip_only_one_ref_dmvr(int x, int y, int pic_w, int pic_h, int w, int h, s16 mv[1][MV_D], s16(*mv_t)[MV_D])
{
    // 定义一个标志，用于指示是否需要裁剪s
    BOOL clip_flag = 0;

    // 定义最小和最大剪辑值数组
    int min_clip[MV_D], max_clip[MV_D];

    // 将x, y, w, h的值左移两位，相当于乘以4，因为运动矢量的精度是四分之一像素
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;

    // 根据CTU_256宏定义选择裁剪边界
#if CTU_256
    // 如果定义了CTU_256，则使用CTU大小为256的裁剪值
    min_clip[MV_X] = (-MAX_CU_SIZE2) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE2) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE2) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE2) << 2;
#else
    // 如果没有定义CTU_256，则使用CTU大小为128的裁剪值
    min_clip[MV_X] = (-MAX_CU_SIZE) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE) << 2;
#endif

    // 将原始运动矢量赋值给目标数组
    mv_t[0][MV_X] = mv[0][MV_X];
    mv_t[0][MV_Y] = mv[0][MV_Y];

    // 检查并裁剪水平运动矢量
    if (x + mv[0][MV_X] < min_clip[MV_X])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[0][MV_X] = min_clip[MV_X] - x;  // 裁剪运动矢量
    }
    if (y + mv[0][MV_Y] < min_clip[MV_Y])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[0][MV_Y] = min_clip[MV_Y] - y;  // 裁剪运动矢量
    }
    if (x + mv[0][MV_X] + w - 4 > max_clip[MV_X])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[0][MV_X] = max_clip[MV_X] - x - w + 4;  // 裁剪运动矢量
    }
    if (y + mv[0][MV_Y] + h - 4 > max_clip[MV_Y])
    {
        clip_flag = 1;  // 设置裁剪标志
        mv_t[0][MV_Y] = max_clip[MV_Y] - y - h + 4;  // 裁剪运动矢量
    }

    // 返回是否进行了裁剪的标志
    return clip_flag;
}
static void prefetch_for_affine_mc(
    int x, int y, int pic_w, int pic_h, int w, int h,
    s8 refi[REFP_NUM], s16 (*mv)[1][MV_D],
    COM_REFP(*refp)[REFP_NUM],
    pel dmvr_padding_buf[REFP_NUM][N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE]
)
{
    s16 mv_temp[REFP_NUM][1][MV_D];  // 临时存储裁剪后的运动矢量
    int num_extra_pixel_left_for_filter; // 滤波器额外需要的像素数
    for (int i = 0; i < REFP_NUM; ++i)
    {
        int filter_size = NTAPS_LUMA; // 亮度滤波器的系数数量
        num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1); // 计算左侧额外像素数
        int offset = ((DMVR_ITER_COUNT) * (PAD_BUFFER_STRIDE + 1)); // 计算偏移量+		mv[i]	0x0000009efb361a4c {0x0000009efb361a4c {8, 4}}	short[1][2]

        int pad_size = DMVR_PAD_LENGTH; // 填充大小
        int qpel_gmv_x, qpel_gmv_y; // 四分之一像素精度的运动矢量
        COM_PIC* ref_pic; // 引用图像指针
        affine_mv_clip_only_one_ref_dmvr(x, y, pic_w, pic_h, w, h, mv[i], mv_temp[i]); // 裁剪运动矢量

        // 计算四分之一像素精度的全局运动矢量
        qpel_gmv_x = (x << 2) + mv_temp[i][0][MV_X];
        qpel_gmv_y = (y << 2) + mv_temp[i][0][MV_Y];

        ref_pic = refp[refi[i]][i].pic; // 获取参考图像
        pel* ref = ref_pic->y + ((qpel_gmv_y >> 2) - num_extra_pixel_left_for_filter) * ref_pic->stride_luma +
            (qpel_gmv_x >> 2) - num_extra_pixel_left_for_filter; // 计算亮度分量的引用地址
        pel* dst = dmvr_padding_buf[i][0] + offset; // 计算目标缓冲区的地址
        copy_buffer(ref, ref_pic->stride_luma, dst, PAD_BUFFER_STRIDE, filter_size, filter_size); // 复制亮度分量
        padding(dst, PAD_BUFFER_STRIDE, filter_size, filter_size, pad_size, pad_size, pad_size, pad_size); // 填充亮度分量

        // 对色度分量进行相同的处理
        filter_size = NTAPS_CHROMA; // 色度滤波器的系数数量
        num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1); // 重新计算左侧额外像素数
        offset = (DMVR_ITER_COUNT); // 重新计算偏移量
        offset = offset * (PAD_BUFFER_STRIDE + 1); // 重新计算偏移量
        pad_size = DMVR_PAD_LENGTH >> 1; // 色度填充大小（通常是亮度的一半）

        // 处理U分量
        ref = ref_pic->u + ((qpel_gmv_y >> 3) - num_extra_pixel_left_for_filter) * ref_pic->stride_chroma +
            (qpel_gmv_x >> 3) - num_extra_pixel_left_for_filter;
        dst = dmvr_padding_buf[i][1] + offset;
        copy_buffer(ref, ref_pic->stride_chroma, dst, PAD_BUFFER_STRIDE, filter_size, filter_size);
        padding(dst, PAD_BUFFER_STRIDE, filter_size, filter_size, pad_size, pad_size, pad_size, pad_size);

        // 处理V分量
        ref = ref_pic->v + ((qpel_gmv_y >> 3) - num_extra_pixel_left_for_filter) * ref_pic->stride_chroma +
            (qpel_gmv_x >> 3) - num_extra_pixel_left_for_filter;
        dst = dmvr_padding_buf[i][2] + offset;
        copy_buffer(ref, ref_pic->stride_chroma, dst, PAD_BUFFER_STRIDE, filter_size, filter_size);
        padding(dst, PAD_BUFFER_STRIDE, filter_size, filter_size, pad_size, pad_size, pad_size, pad_size);
    }
}
void affine_mv_clip(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], CPMV(*mv)[VER_NUM][MV_D], s16(*mv_t)[1][MV_D])
{
    // 定义最小和最大剪辑值数组
    int min_clip[MV_D], max_clip[MV_D];

    // 将x, y, w, h的值左移两位，相当于乘以4，因为运动矢量的精度是四分之一像素
    x <<= 2;
    y <<= 2;
    w <<= 2;
    h <<= 2;

#if CTU_256
    // 如果定义了CTU_256，则使用CTU大小为256的剪辑值
    min_clip[MV_X] = (-MAX_CU_SIZE2 - 4) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE2 - 4) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE2 + 4) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE2 + 4) << 2;
#else
    // 如果没有定义CTU_256，则使用CTU大小为128的剪辑值
    min_clip[MV_X] = (-MAX_CU_SIZE - 4) << 2;
    min_clip[MV_Y] = (-MAX_CU_SIZE - 4) << 2;
    max_clip[MV_X] = (pic_w - 1 + MAX_CU_SIZE + 4) << 2;
    max_clip[MV_Y] = (pic_h - 1 + MAX_CU_SIZE + 4) << 2;
#endif

    // 将原始运动矢量赋值给目标数组
    mv_t[REFP_0][0][MV_X] = mv[REFP_0][0][MV_X];
    mv_t[REFP_0][0][MV_Y] = mv[REFP_0][0][MV_Y];
    mv_t[REFP_1][0][MV_X] = mv[REFP_1][0][MV_X];
    mv_t[REFP_1][0][MV_Y] = mv[REFP_1][0][MV_Y];

    // 对第一个参考图像的运动矢量进行边界检查和调整
    if (REFI_IS_VALID(refi[REFP_0]))
    {
        if (x + mv[REFP_0][0][MV_X] < min_clip[MV_X])
            mv_t[REFP_0][0][MV_X] = (s16)(min_clip[MV_X] - x);
        if (y + mv[REFP_0][0][MV_Y] < min_clip[MV_Y])
            mv_t[REFP_0][0][MV_Y] = (s16)(min_clip[MV_Y] - y);
        if (x + mv[REFP_0][0][MV_X] + w - 4 > max_clip[MV_X])
            mv_t[REFP_0][0][MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
        if (y + mv[REFP_0][0][MV_Y] + h - 4 > max_clip[MV_Y])
            mv_t[REFP_0][0][MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
    }

    // 对第二个参考图像的运动矢量进行边界检查和调整
    if (REFI_IS_VALID(refi[REFP_1]))
    {
        if (x + mv[REFP_1][0][MV_X] < min_clip[MV_X])
            mv_t[REFP_1][0][MV_X] = (s16)(min_clip[MV_X] - x);
        if (y + mv[REFP_1][0][MV_Y] < min_clip[MV_Y])
            mv_t[REFP_1][0][MV_Y] = (s16)(min_clip[MV_Y] - y);
        if (x + mv[REFP_1][0][MV_X] + w - 4 > max_clip[MV_X])
            mv_t[REFP_1][0][MV_X] = (s16)(max_clip[MV_X] - x - w + 4);
        if (y + mv[REFP_1][0][MV_Y] + h - 4 > max_clip[MV_Y])
            mv_t[REFP_1][0][MV_Y] = (s16)(max_clip[MV_Y] - y - h + 4);
    }
}

void com_affine_dmvr_refine(
    int w, int h, pel* ref_l0, int s_ref_l0, pel* ref_l1, int s_ref_l1,
    s32* min_cost, s16* delta_mvx, s16* delta_mvy, s32* sad_array)
{
    SAD_POINT_INDEX idx; // 用于遍历SAD点的索引
    s32 search_offset_x_back[5] = { 0, 0, 1, -1, 0 }; // 用于回溯搜索的X偏移量
    s32 search_offset_y_back[5] = { 1, -1, 0, 0, 0 }; // 用于回溯搜索的Y偏移量
    s32 j = 0; // 迭代变量
    s32 start_x = 0; // X方向起始偏移量
    s32 start_y = 0; // Y方向起始偏移量
    s32 search_offset_x[NUM_SAD_POINTS] = { 0, 0, 0, 1, -1, 0, 1, 2, 1, 0, -1, -2, -1, 2, 1, 2, 1, -1, -2, -2, -1 };
    s32 search_offset_y[NUM_SAD_POINTS] = { 0, 1, -1, 0, 0, 2, 1, 0, -1, -2, -1, 0, 1, 1, 2, -1, -2, -2, -1, 1, 2 };
    s32 cost_temp[5][5] = { { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX },
    { INT_MAX, INT_MAX, INT_MAX, INT_MAX, INT_MAX } };

    s32 center_x = 2; // 中心点的X索引
    s32 center_y = 2; // 中心点的Y索引

    pel* ref_l0_Orig = ref_l0; // 原始参考图像L0的指针
    pel* ref_l1_Orig = ref_l1; // 原始参考图像L1的指针
    for (idx = 0; idx < NUM_SAD_POINTS; ++idx) // 遍历所有SAD点
    {
        int sum = 0;
        ref_l0 = ref_l0_Orig + search_offset_x[idx] + search_offset_y[idx] * s_ref_l0; // 计算新的参考图像L0的指针
        ref_l1 = ref_l1_Orig - search_offset_x[idx] - search_offset_y[idx] * s_ref_l1; // 计算新的参考图像L1的指针
        s32 cost = com_DMVR_cost(w, h, ref_l0, ref_l1, s_ref_l0, s_ref_l1); // 计算当前点的成本
        cost_temp[center_y + search_offset_y[idx]][center_x + search_offset_x[idx]] = cost; // 存储成本
    }

    *min_cost = cost_temp[center_y][center_x]; // 初始化最小成本为当前中心点的成本
    for (j = 0; j < 3; j++) // 进行三次迭代以细化运动矢量
    {
        s32 delta_x = 0; // X方向的增量
        s32 delta_y = 0; // Y方向的增量

        for (idx = SAD_BOTTOM; idx < SAD_TOP_LEFT; ++idx) // 遍历回溯搜索点
        {
            s32 y_offset = center_y + start_y + search_offset_y_back[idx]; // 计算Y偏移量
            s32 x_offset = center_x + start_x + search_offset_x_back[idx]; // 计算X偏移量
            if (x_offset >= 0 && x_offset < 5 && y_offset >= 0 && y_offset < 5) // 确保偏移量在有效范围内
            {
                s32 cost = cost_temp[center_y + start_y + search_offset_y_back[idx]][center_x + start_x + search_offset_x_back[idx]]; // 获取当前点的成本
                if (cost < *min_cost) // 如果找到更小的成本
                {
                    *min_cost = cost; // 更新最小成本
                    delta_x = search_offset_x_back[idx]; // 更新X方向增量
                    delta_y = search_offset_y_back[idx]; // 更新Y方向增量
                    if (cost == 0) // 如果成本为0，直接跳出循环
                    {
                        break;
                    }
                }
            }
        }
        start_x += delta_x; // 更新起始X偏移量
        start_y += delta_y; // 更新起始Y偏移量
        if (*min_cost == 0) // 如果最小成本为0，跳出循环
        {
            break;
        }

        if ((delta_x == 0) && (delta_y == 0)) // 如果没有更新，跳出循环
        {
            break;
        }
    }
    *delta_mvx = start_x; // 更新最小成本对应的X增量
    *delta_mvy = start_y; // 更新最小成本对应的Y增量

    if ((abs(*delta_mvx) < center_x) && (abs(*delta_mvy) < center_y)) // 如果增量在中心点附近
    {
        *(sad_array + SAD_CENTER) = *min_cost; // 更新SAD数组中的中心点成本
        for (idx = SAD_BOTTOM; idx < SAD_TOP_LEFT; ++idx) // 遍历所有SAD点
        {
            *(sad_array + idx) = cost_temp[center_y + (*delta_mvy) + search_offset_y_back[idx]][center_x + (*delta_mvx) + search_offset_x_back[idx]]; // 更新SAD数组
        }
    }
    ref_l0 = ref_l0_Orig; // 恢复原始的参考图像L0的指针
    ref_l1 = ref_l1_Orig; // 恢复原始的参考图像L1的指针
}

void process_AFFINEDMVR(int x, int y, int pic_w, int pic_h, int w, int h, s8 refi[REFP_NUM], CPMV(*mv)[VER_NUM][MV_D], COM_REFP(*refp)[REFP_NUM], pel(*dmvr_padding_buf)[N_C][PAD_BUFFER_STRIDE * PAD_BUFFER_STRIDE])
{
    s16 sub_pu_L0[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];
    s16 sub_pu_L1[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)][MV_D];

    // 定义一个数组，用于存储经过初步处理的运动矢量
    /*s16 refined_mv[REFP_NUM][1][MV_D] = {{mv[REFP_0][0][MV_X], mv[REFP_0][0][MV_Y]},
                                       { mv[REFP_1][0][MV_X], mv[REFP_1][0][MV_Y] }
    };*/

    // 定义一个数组，用于存储裁剪后的起始运动矢量
    s16 starting_mv[REFP_NUM][1][MV_D];
    // 调用affine_mv_clip函数，对运动矢量进行裁剪，确保它们不会超出图像边界
    affine_mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, starting_mv);

    // 形成整数部分的运动矢量
    s16 sign_x = starting_mv[REFP_0][0][MV_X] >= 0 ? 1 : (-1); // 确定x分量的符号
    s16 sign_y = starting_mv[REFP_0][0][MV_Y] >= 0 ? 1 : (-1); // 确定y分量的符号
    s16 abs_x = abs(starting_mv[REFP_0][0][MV_X]); // 计算x分量的绝对值
    s16 abs_y = abs(starting_mv[REFP_0][0][MV_Y]); // 计算y分量的绝对值
    s16 xFrac = abs_x & 3; // 计算x分量的小数部分
    s16 yFrac = abs_y & 3; // 计算y分量的小数部分

    // 裁剪绝对运动矢量，确保它们不会超过最大允许值
    short max_abs_mv = (1 << 15) - 1 - (1 << 2);
    abs_x = COM_MIN(abs_x, max_abs_mv); // 裁剪x分量
    abs_y = COM_MIN(abs_y, max_abs_mv); // 裁剪y分量

    // 将裁剪后的绝对运动矢量转换回带符号的整数运动矢量
    starting_mv[REFP_0][0][MV_X] = (((abs_x) >> 2) << 2); // 将x分量的小数部分舍去
    starting_mv[REFP_0][0][MV_Y] = (((abs_y) >> 2) << 2); // 将y分量的小数部分舍去
    starting_mv[REFP_0][0][MV_X] += (((xFrac > 2) ? 1 : 0) << 2); // 如果x分量的小数部分大于2，则增加4（相当于0.5像素）
    starting_mv[REFP_0][0][MV_Y] += (((yFrac > 2) ? 1 : 0) << 2); // 如果y分量的小数部分大于2，则增加4（相当于0.5像素）
    starting_mv[REFP_0][0][MV_X] *= sign_x; // 恢复x分量的符号
    starting_mv[REFP_0][0][MV_Y] *= sign_y; // 恢复y分量的符号

    sign_x = starting_mv[REFP_1][0][MV_X] >= 0 ? 1 : (-1);
    sign_y = starting_mv[REFP_1][0][MV_Y] >= 0 ? 1 : (-1);
    abs_x = abs(starting_mv[REFP_1][0][MV_X]);
    abs_y = abs(starting_mv[REFP_1][0][MV_Y]);

    // clip ABSmv
    abs_x = COM_MIN(abs_x, max_abs_mv);
    abs_y = COM_MIN(abs_y, max_abs_mv);

    xFrac = abs_x & 3;
    yFrac = abs_y & 3;
    starting_mv[REFP_1][0][MV_X] = (((abs_x) >> 2) << 2);
    starting_mv[REFP_1][0][MV_Y] = (((abs_y) >> 2) << 2);
    starting_mv[REFP_1][0][MV_X] += (((xFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_1][0][MV_Y] += (((yFrac > 2) ? 1 : 0) << 2);
    starting_mv[REFP_1][0][MV_X] *= sign_x;
    starting_mv[REFP_1][0][MV_Y] *= sign_y;

    // clip ABSmv
    short max_dmvr_mv = (1 << 15) - (3 << 2);
    short min_dmvr_mv = -(1 << 15) + (2 << 2);
    starting_mv[REFP_0][0][MV_X] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_0][0][MV_X]));
    starting_mv[REFP_0][0][MV_Y] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_0][0][MV_Y]));
    starting_mv[REFP_1][0][MV_X] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_1][0][MV_X]));
    starting_mv[REFP_1][0][MV_Y] = COM_MIN(max_dmvr_mv, COM_MAX(min_dmvr_mv, starting_mv[REFP_1][0][MV_Y]));

    // centre address holder for pred
    pel* preds_array[REFP_NUM];
    pel* preds_centre_array[REFP_NUM];

    int stride = PAD_BUFFER_STRIDE;
    preds_array[REFP_0] = dmvr_padding_buf[REFP_0][Y_C];
    preds_array[REFP_1] = dmvr_padding_buf[REFP_1][Y_C];

    int filter_size = NTAPS_LUMA;
    int num_extra_pixel_left_for_filter = ((filter_size >> 1) - 1);
    // go to the center point
    preds_centre_array[REFP_0] = preds_array[REFP_0] + (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));
    preds_centre_array[REFP_1] = preds_array[REFP_1] + (DMVR_ITER_COUNT + num_extra_pixel_left_for_filter) * ((PAD_BUFFER_STRIDE + 1));

    // 初始化最小代价变量，用于后续的最优预测块搜索
    int min_cost = INT_MAX;

    // 初始化最后一个方向变量，用于记录搜索过程中的方向
    int last_direction = -1;

    // 定义一个数组，用于存储不同方向的代价
    int array_cost[SAD_COUNT];

    /*int dx, dy;
    dy = min(h, DMVR_IMPLIFICATION_SUBCU_SIZE);
    dx = min(w, DMVR_IMPLIFICATION_SUBCU_SIZE);*/

    // 初始化总的运动矢量变化量
    s16 total_delta_mv[MV_D] = { 0, 0 };
    BOOL not_zero_cost = 1;  // 标志，用于检测是否找到非零成本的预测

    prefetch_for_affine_mc(x, y, pic_w, pic_h, h, w, refi, starting_mv, refp, dmvr_padding_buf);
    pel* addr_subpu_l0 = preds_centre_array[REFP_0];
    pel* addr_subpu_l1 = preds_centre_array[REFP_1];
    // 初始化成本数组，用于存储不同方向的成本
    for (int loop = 0; loop < SAD_COUNT; loop++)
    {
        array_cost[loop] = INT_MAX;
    }

    // 调用运动矢量细化函数，尝试找到最小成本
    min_cost = INT_MAX;
    com_affine_dmvr_refine(h, w, addr_subpu_l0, stride, addr_subpu_l1,
        stride,
        &min_cost,
        &total_delta_mv[MV_X], &total_delta_mv[MV_Y],
        array_cost);

    // 如果找到的成本为0，说明找到了完美的匹配，不需要进一步调整
    if (min_cost == 0)
    {
        not_zero_cost = 0;
    }

    // 将总的运动矢量变化量左移两位，以恢复到原始的四分之一像素精度
    total_delta_mv[MV_X] = (total_delta_mv[MV_X] << 2);
    total_delta_mv[MV_Y] = (total_delta_mv[MV_Y] << 2);

    // 如果成本非零，并且所有方向的成本都已计算，则尝试进一步细化运动矢量
    if (not_zero_cost && (min_cost == array_cost[SAD_CENTER])
        && (array_cost[SAD_CENTER] != INT_MAX)
        && (array_cost[SAD_LEFT] != INT_MAX)
        && (array_cost[SAD_TOP] != INT_MAX)
        && (array_cost[SAD_RIGHT] != INT_MAX)
        && (array_cost[SAD_BOTTOM] != INT_MAX))
    {
        int sadbuffer[5];  // 存储不同方向的成本
        int deltaMv[MV_D] = { 0, 0 };  // 存储进一步细化后的运动矢量变化量

        // 将成本存储到缓冲区，用于进一步细化
        sadbuffer[0] = array_cost[SAD_CENTER];
        sadbuffer[1] = array_cost[SAD_LEFT];
        sadbuffer[2] = array_cost[SAD_TOP];
        sadbuffer[3] = array_cost[SAD_RIGHT];
        sadbuffer[4] = array_cost[SAD_BOTTOM];
        com_sub_pel_error_surface(sadbuffer, deltaMv);

        // 将细化后的变化量应用到总的运动矢量变化量
        total_delta_mv[MV_X] += deltaMv[MV_X] >> 2;
        total_delta_mv[MV_Y] += deltaMv[MV_Y] >> 2;
    }

    // 更新参考图像0和1的细化后运动矢量
    mv[REFP_0][0][MV_X] = (starting_mv[REFP_0][0][MV_X]) + (total_delta_mv[MV_X]);
    mv[REFP_0][0][MV_Y] = (starting_mv[REFP_0][0][MV_Y]) + (total_delta_mv[MV_Y]);
    mv[REFP_1][0][MV_X] = (starting_mv[REFP_1][0][MV_X]) - (total_delta_mv[MV_X]);
    mv[REFP_1][0][MV_Y] = (starting_mv[REFP_1][0][MV_Y]) - (total_delta_mv[MV_Y]);
}
#endif



void com_mc(int x, int y, int w, int h, int pred_stride, pel pred_buf[N_C][MAX_CU_DIM], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], CHANNEL_TYPE channel, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
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
)
{
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    s8 *refi = mod_info_curr->refi;
    s16 (*mv)[MV_D] = mod_info_curr->mv;
    COM_PIC *ref_pic;
    static pel pred_snd[N_C][MAX_CU_DIM];
#if BGC
    static pel pred_uv[V_C][MAX_CU_DIM];
    pel *dst_u, *dst_v;
    pel *pred_fir = info->pred_tmp;
    pel *p0, *p1, *dest0;
#endif
    pel (*pred)[MAX_CU_DIM] = pred_buf;
    int qpel_gmv_x, qpel_gmv_y;
#if INTER_TM
    int init_qpel_gmv_x, init_qpel_gmv_y;
    BOOL enable_bio = (info->sqh.tm_enable_flag == 2 && mod_info_curr->tm_flag) ? 0 : 1;
#endif
    int bidx = 0;
    s16 mv_t[REFP_NUM][MV_D];

#if DMVR
    int poc0 = refp[refi[REFP_0]][REFP_0].ptr;
    int poc1 = refp[refi[REFP_1]][REFP_1].ptr;
    s16 mv_refine[REFP_NUM][MV_D] = {{mv[REFP_0][MV_X], mv[REFP_0][MV_Y]},
                                     {mv[REFP_1][MV_X], mv[REFP_1][MV_Y]}};
    s16 inital_mv[REFP_NUM][MV_D] = { { mv[REFP_0][MV_X], mv[REFP_0][MV_Y] },
                                      { mv[REFP_1][MV_X], mv[REFP_1][MV_Y] }};
    BOOL dmvr_poc_condition = ((BOOL)((dmvr->poc_c - poc0)*(dmvr->poc_c - poc1) < 0)) && (abs(dmvr->poc_c - poc0) == abs(dmvr->poc_c - poc1));
    s32 extend_width = (DMVR_NEW_VERSION_ITER_COUNT + 1) * REF_PRED_EXTENTION_PEL_COUNT;
    s32 extend_width_minus1 = DMVR_NEW_VERSION_ITER_COUNT * REF_PRED_EXTENTION_PEL_COUNT;
    int stride = w + (extend_width << 1);
    s16 mv_offsets[REFP_NUM][MV_D] = {{0,},};
    int iterations_count = DMVR_ITER_COUNT;
#endif
#if BIO
#if FIX_395
    int bio_poc0 = REFI_IS_VALID(refi[REFP_0]) ? refp[refi[REFP_0]][REFP_0].ptr : -1;
    int bio_poc1 = REFI_IS_VALID(refi[REFP_1]) ? refp[refi[REFP_1]][REFP_1].ptr : -1;
#else
    int bio_poc0 = REFI_IS_VALID(refi[REFP_0]) ? refp[refi[REFP_0]][REFP_0].pic->ptr : -1;
    int bio_poc1 = REFI_IS_VALID(refi[REFP_1]) ? refp[refi[REFP_1]][REFP_1].pic->ptr : -1;
#endif
    int bio_is_bi = (ptr >= 0 && REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]) && ((bio_poc0 - ptr) * (ptr - bio_poc1) > 0)) ? 1 : 0;
#endif

    mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, mv_t);
#if DMVR
    dmvr->apply_DMVR = dmvr->apply_DMVR && dmvr_poc_condition;
    dmvr->apply_DMVR = dmvr->apply_DMVR && (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]));
#if LIBVC_ON
    dmvr->apply_DMVR = dmvr->apply_DMVR && !(refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr &&  mv_t[REFP_0][MV_X] == mv_t[REFP_1][MV_X] && mv_t[REFP_0][MV_Y] == mv_t[REFP_1][MV_Y] && refp[refi[REFP_0]][REFP_0].is_library_picture == refp[refi[REFP_1]][REFP_1].is_library_picture);
#else
    dmvr->apply_DMVR = dmvr->apply_DMVR && !(refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr &&  mv_t[REFP_0][MV_X] == mv_t[REFP_1][MV_X] && mv_t[REFP_0][MV_Y] == mv_t[REFP_1][MV_Y]);
#endif
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!((w == 4 && h <= 8) || (w <= 8 && h == 4)));
    dmvr->apply_DMVR = dmvr->apply_DMVR && (w >= 8 && h >= 8);
#if AWP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->awp_flag);
#endif
#if ETMVP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->etmvp_flag);
#endif
#if IPC 
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->ipc_flag);
#endif
#if INTER_TM
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->tm_flag);
#endif
#endif
    if (REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_0][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_0][MV_Y];
#if INTER_TM
        init_qpel_gmv_x = (x << 2) + mod_info_curr->init_mv[REFP_0][MV_X];
        init_qpel_gmv_y = (y << 2) + mod_info_curr->init_mv[REFP_0][MV_Y];
#endif

        if (channel != CHANNEL_C)
        {
#if DMVR
            if (!dmvr->apply_DMVR)
            {
#endif
#if INTER_TM
                if (mod_info_curr->tm_flag && info->sqh.tm_enable_flag == 2)
                {
                    int dx = mv[REFP_0][0] & 0x3 ? 1 : 0;
                    int dy = mv[REFP_0][1] & 0x3 ? 1 : 0;
                    BOOL is_search = FALSE;
                    if (dx == 0 && dy == 0)
                    {
                        com_mc_tm_l_00(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                    else if (dx == 1 && dy == 0)
                    {
                        com_mc_tm_l_n0(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                    else if (dx == 0 && dy == 1)
                    {
                        com_mc_tm_l_0n(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                    else if (dx == 1 && dy == 1)
                    {
                        com_mc_tm_l_nn(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                }
                else
                {
                    com_mc_l(mv[REFP_0][0], mv[REFP_0][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
                }
#else
                com_mc_l(mv[REFP_0][0], mv[REFP_0][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
#endif
#if BIO
                if (info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE
#if INTER_TM
                    && enable_bio
#endif
                    )
                {
                    com_grad_x_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_x[0], w, h, bit_depth, 0);
                    com_grad_y_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_y[0], w, h, bit_depth, 0);
                }
#endif
#if DMVR
            }
#endif
        }
        if (channel != CHANNEL_L)
        {
#if CHROMA_NOT_SPLIT
            assert(w >= 8 && h >= 8);
#endif
#if DMVR
            if(!REFI_IS_VALID(refi[REFP_1]) || !dmvr->apply_DMVR || !dmvr_poc_condition)
#endif
            {
                com_mc_c( mv[REFP_0][0], mv[REFP_0][1], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth );
                com_mc_c( mv[REFP_0][0], mv[REFP_0][1], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth );
            }
        }
        bidx++;
#if BGC
#if DMVR
        if (bgc_flag && !dmvr->apply_DMVR && channel != CHANNEL_C)
#else
        if (bgc_flag && channel != CHANNEL_C)
#endif
        {
            p0 = pred_buf[Y_C];
            dest0 = pred_fir;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    dest0[i] = p0[i];
                }
                p0 += pred_stride;
                dest0 += pred_stride;
            }

            p0 = pred_buf[U_C];
            p1 = pred_buf[V_C];
            dst_u = pred_uv[0];
            dst_v = pred_uv[1];
            for (int j = 0; j < (h >> 1); j++)
            {
                for (int i = 0; i < (w >> 1); i++)
                {
                    dst_u[i] = p0[i];
                    dst_v[i] = p1[i];
                }
                p0 += (pred_stride >> 1);
                p1 += (pred_stride >> 1);
                dst_u += (pred_stride >> 1);
                dst_v += (pred_stride >> 1);
            }
        }
#endif
    }

    /* check identical motion */
    if(REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
    {
#if LIBVC_ON
        if( refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr && mv[REFP_0][MV_X] == mv[REFP_1][MV_X] && mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y] && refp[refi[REFP_0]][REFP_0].is_library_picture == refp[refi[REFP_1]][REFP_1].is_library_picture )
#else
        if( refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr && mv[REFP_0][MV_X] == mv[REFP_1][MV_X] && mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y] )
#endif
        {
            return;
        }
    }

    if (REFI_IS_VALID(refi[REFP_1]))
    {
        /* backward */
        if (bidx)
        {
            pred = pred_snd;
        }
        ref_pic = refp[refi[REFP_1]][REFP_1].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_1][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_1][MV_Y];
#if INTER_TM
        init_qpel_gmv_x = (x << 2) + mod_info_curr->init_mv[REFP_1][MV_X];
        init_qpel_gmv_y = (y << 2) + mod_info_curr->init_mv[REFP_1][MV_Y];
#endif

        if (channel != CHANNEL_C)
        {
#if DMVR
            if (!dmvr->apply_DMVR)
            {
#endif
#if INTER_TM
                if (mod_info_curr->tm_flag && info->sqh.tm_enable_flag == 2)
                {
                    int dx = mv[REFP_1][0] & 0x3 ? 1 : 0;
                    int dy = mv[REFP_1][1] & 0x3 ? 1 : 0;
                    BOOL is_search = FALSE;
                    if (dx == 0 && dy == 0)
                    {
                        com_mc_tm_l_00(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                    else if (dx == 1 && dy == 0)
                    {
                        com_mc_tm_l_n0(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                    else if (dx == 0 && dy == 1)
                    {
                        com_mc_tm_l_0n(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                    else if (dx == 1 && dy == 1)
                    {
                        com_mc_tm_l_nn(ref_pic->y, pred[Y_C], ref_pic->stride_luma, pred_stride, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, w, h, is_search, bit_depth);
                    }
                }
                else
                {
                    com_mc_l(mv[REFP_1][0], mv[REFP_1][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
                }
#else
                com_mc_l(mv[REFP_1][0], mv[REFP_1][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
#endif
#if BIO
                if (info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE
#if INTER_TM
                    && enable_bio
#endif
                    )
                {
                    com_grad_x_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_x[bidx], w, h, bit_depth, 0);
                    com_grad_y_l_nn(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, w, grad_y[bidx], w, h, bit_depth, 0);
                }
#endif
#if DMVR
            }
#endif
        }

        if (channel != CHANNEL_L)
        {
#if CHROMA_NOT_SPLIT
            assert(w >= 8 && h >= 8);
#endif
#if DMVR
            if(!REFI_IS_VALID(refi[REFP_0]) || !dmvr->apply_DMVR || !dmvr_poc_condition)
#endif
            {
                com_mc_c( mv[REFP_1][0], mv[REFP_1][1], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth );
                com_mc_c( mv[REFP_1][0], mv[REFP_1][1], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth );
            }
        }
        bidx++;
    }

    if (bidx == 2)
    {
#if DMVR
        //only if the references are located on oppisite sides of the current frame
        if( dmvr->apply_DMVR )
        {
            process_DMVR( x, y, pic_w, pic_h, w, h, refi, mv, refp, pred_buf, pred_snd, dmvr->poc_c, dmvr->dmvr_current_template, dmvr->dmvr_ref_pred_interpolated
                , iterations_count
                , bit_depth
                , dmvr->dmvr_padding_buf
#if BIO
                , (info->sqh.bio_enable_flag&& bio_is_bi&& mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE
#if INTER_TM
                    && enable_bio
#endif
                    )
#endif
            );
#if BGC
            if (bgc_flag)
            {
                p0 = pred_buf[Y_C];
                dest0 = pred_fir;
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        dest0[i] = p0[i];
                    }
                    p0 += pred_stride;
                    dest0 += pred_stride;
                }

                if (channel != CHANNEL_C)
                {
                    p0 = pred_buf[U_C];
                    p1 = pred_buf[V_C];
                    dst_u = pred_uv[0];
                    dst_v = pred_uv[1];
                    for (int j = 0; j < (h >> 1); j++)
                    {
                        for (int i = 0; i < (w >> 1); i++)
                        {
                            dst_u[i] = p0[i];
                            dst_v[i] = p1[i];
                        }
                        p0 += (pred_stride >> 1);
                        p1 += (pred_stride >> 1);
                        dst_u += (pred_stride >> 1);
                        dst_v += (pred_stride >> 1);
                    }
                }
            }
#endif

            mv[REFP_0][MV_X] = inital_mv[REFP_0][MV_X];
            mv[REFP_0][MV_Y] = inital_mv[REFP_0][MV_Y];

            mv[REFP_1][MV_X] = inital_mv[REFP_1][MV_X];
            mv[REFP_1][MV_Y] = inital_mv[REFP_1][MV_Y];
        }
#endif

        if (channel != CHANNEL_C)
        {
#if BIO
#if IPC 
            if (!mod_info_curr->ipc_flag && info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE
#if INTER_TM
                && enable_bio
#endif
                )
#else
            if (info->sqh.bio_enable_flag && bio_is_bi && mvr_idx < BIO_MAX_MVR && !enc_fast && w <= BIO_MAX_SIZE && h <= BIO_MAX_SIZE)
#endif            
            {
                bio_opt(pred_buf[Y_C], pred_snd[Y_C], w, h, bit_depth);
            }
            else
            {
#endif
#if SIMD_MC
                average_16b_no_clip_sse(pred_buf[Y_C], pred_snd[Y_C], pred_buf[Y_C], pred_stride, pred_stride, pred_stride, w, h);
#else
                pel *p0, *p1;
                p0 = pred_buf[Y_C];
                p1 = pred_snd[Y_C];
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        p0[i] = (p0[i] + p1[i] + 1) >> 1;
                    }
                    p0 += pred_stride;
                    p1 += pred_stride;
                }
#endif
#if BIO
            }
#endif
#if BGC
            if (bgc_flag && info->sqh.bgc_enable_flag)
            {
                p0 = pred_fir;
                p1 = pred_snd[Y_C];
                dest0 = pred_buf[Y_C];
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        if (bgc_idx)
                        {
                            dest0[i] += (p0[i] - p1[i]) >> 3;
                        }
                        else
                        {
                            dest0[i] += (p1[i] - p0[i]) >> 3;
                        }
                        dest0[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dest0[i]);
                    }
                    p0 += pred_stride;
                    p1 += pred_stride;
                    dest0 += pred_stride;
                }
            }
#endif
        }

        if (channel != CHANNEL_L)
        {
#if SIMD_MC
            w >>= 1;
            h >>= 1;
            pred_stride >>= 1;
            average_16b_no_clip_sse(pred_buf[U_C], pred_snd[U_C], pred_buf[U_C], pred_stride, pred_stride, pred_stride, w, h);
            average_16b_no_clip_sse(pred_buf[V_C], pred_snd[V_C], pred_buf[V_C], pred_stride, pred_stride, pred_stride, w, h);
#else
            pel *p0, *p1, *p2, *p3;
            p0 = pred_buf[U_C];
            p1 = pred_snd[U_C];
            p2 = pred_buf[V_C];
            p3 = pred_snd[V_C];
            w >>= 1;
            h >>= 1;
            pred_stride >>= 1;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    p0[i] = (p0[i] + p1[i] + 1) >> 1;
                    p2[i] = (p2[i] + p3[i] + 1) >> 1;
                }
                p0 += pred_stride;
                p1 += pred_stride;
                p2 += pred_stride;
                p3 += pred_stride;
            }
#endif

#if BGC
            if (bgc_flag && info->sqh.bgc_enable_flag  && channel != CHANNEL_C)
            {
                pel *p2, *p3;
                p0 = pred_uv[0];
                p1 = pred_snd[U_C];
                p2 = pred_uv[1];
                p3 = pred_snd[V_C];
                dst_u = pred_buf[U_C];
                dst_v = pred_buf[V_C];
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {

                        if (bgc_idx)
                        {
                            dst_u[i] += (p0[i] - p1[i]) >> 3;
                            dst_v[i] += (p2[i] - p3[i]) >> 3;
                        }
                        else
                        {
                            dst_u[i] += (p1[i] - p0[i]) >> 3;
                            dst_v[i] += (p3[i] - p2[i]) >> 3;
                        }
                        dst_u[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dst_u[i]);
                        dst_v[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dst_v[i]);
                    }
                    p0 += pred_stride;
                    p1 += pred_stride;
                    p2 += pred_stride;
                    p3 += pred_stride;
                    dst_u += pred_stride;
                    dst_v += pred_stride;
                }
            }
#endif
        }
    }
}
#if OBMC_TEMP
static s64 obmc_ssd_16b_up(int w, void *src1, void *src2, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    int      j, diff;
    s64   ssd;
    const int shift = (bit_depth - 8) << 1;
    s1 = (s16 *)src1;
    s2 = (s16 *)src2;
    ssd = 0;
    for(j = 0; j < w; j++)
    {
        diff = s1[j] - s2[j];
        ssd += (diff * diff) >> shift;
    }

    return ssd;
}
static s64 obmc_ssd_16b_left( int h, void *src1, void *src2, int s_src1, int bit_depth)
{
    s16 * s1;
    s16 * s2;
    int     i,  diff;
    s64   ssd;
    const int shift = (bit_depth - 8) << 1;
    s1 = (s16 *)src1;
    s2 = (s16 *)src2;
    ssd = 0;
    for(i = 0; i < h; i++)
    {
        diff = s1[i*s_src1] - s2[i];
        ssd += (diff * diff) >> shift;
    }
    return ssd;
}
int caculate_obmc_mode(pel *rec,int stride,pel pred_src1[MAX_CU_SIZE],pel pred_src2[MAX_CU_SIZE],int width,int height,int bit_depth,int dir,pel* weight)
{
    int obmc_mode = 0;
    int w = width;
    int h = height;
    s64 cost1 = 0; s64 cost2 = 0; s64 cost3 = 0;
    
    if (dir == 0)
    {
        cost1 = obmc_ssd_16b_up(w, rec, pred_src1,bit_depth);
        cost2 = obmc_ssd_16b_up(w, rec, pred_src2,bit_depth);

    }
    if (dir == 1)
    {
        cost1 +=  obmc_ssd_16b_left( h, rec, pred_src1, stride, bit_depth);
        cost2 +=  obmc_ssd_16b_left( h, rec, pred_src2, stride, bit_depth);
    }

    if (cost1 < cost2)
    {
        obmc_mode = 0;
    }
    else if(cost1 == cost2)
    {
        obmc_mode = 1;
    }else
    {
        obmc_mode = 2;
    }
    
    return obmc_mode;
}

#endif
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
)
{
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;

    if (info->pic_header.slice_type == SLICE_P)
    {
        return;
    }

    if ((cu_width * cu_height) < 64)
    {
        return;
    }

#if DISABLE_OBMC_AFFINE
    if (mod_info_curr->affine_flag)
    {
        return;
    }
#endif

#if OBMC_SCC
    if (info->pic_header.ph_obmc_scc_flag)
    {
        return;
    }
#endif

    int widthInBlock = (cu_width >> MIN_CU_LOG2);
    int heightInBlock = (cu_height >> MIN_CU_LOG2);

    s8  org_refi[REFP_NUM] = { mod_info_curr->refi[REFP_0], mod_info_curr->refi[REFP_1] };
    s16 org_mv[REFP_NUM][MV_D] = { {mod_info_curr->mv[REFP_0][MV_X], mod_info_curr->mv[REFP_0][MV_Y]}, {mod_info_curr->mv[REFP_1][MV_X], mod_info_curr->mv[REFP_1][MV_Y]} };
    assert(ob_blk_width <= cu_width && ob_blk_height <= cu_height);
    int half_ob_width = (ob_blk_width >> 1);
    int half_ob_width_c = (half_ob_width >> 1);
    int half_ob_height = (ob_blk_height >> 1);
    int half_ob_height_c = (half_ob_height >> 1);

    int log2_half_w = com_tbl_log2[half_ob_width];
    int log2_half_h = com_tbl_log2[half_ob_height];
    int log2_half_w_c = com_tbl_log2[half_ob_width_c];
    int log2_half_h_c = com_tbl_log2[half_ob_height_c];

    pel *weight_above, *weight_left, *weight_above_c, *weight_left_c;
    if (info->pic_header.obmc_blending_flag)
    {
        weight_above = info->obmc_weight[1][log2_half_h];
        weight_above_c = info->obmc_weight[1][log2_half_h_c];
        weight_left = info->obmc_weight[1][log2_half_w];
        weight_left_c = info->obmc_weight[1][log2_half_w_c];
    }
    else
    {
        weight_above = info->obmc_weight[0][log2_half_h];
        weight_above_c = info->obmc_weight[0][log2_half_h_c];
        weight_left = info->obmc_weight[0][log2_half_w];
        weight_left_c = info->obmc_weight[0][log2_half_w_c];
    }

    for (int blkBoundaryType = 0; blkBoundaryType < 2; blkBoundaryType++)  // 0 - top; 1 - left
    {
        BOOL doChromaOBMC = chroma;
        if ((blkBoundaryType == 0) && (cu_height < 16))
        {
            doChromaOBMC = FALSE;
        }
        if ((blkBoundaryType == 1) && (cu_width < 16))
        {
            doChromaOBMC = FALSE;
        }
        int lengthInBlock = ((blkBoundaryType == 0) ? widthInBlock : heightInBlock);
        int subIdx = 0;
        int offsetX_in_unit = 0, offsetY_in_unit = 0;
        MOTION_MERGE_TYPE state = INVALID_NEIGH;
        while (subIdx < lengthInBlock)
        {
            int length = 0;
            if (blkBoundaryType == 0)
            {
                offsetX_in_unit = subIdx;
                offsetY_in_unit = 0;
            }
            else
            {
                offsetX_in_unit = 0;
                offsetY_in_unit = subIdx;
            }

            s16 neigh_mv[REFP_NUM][MV_D];
            s8  neigh_refi[REFP_NUM];

            state = getSameNeigMotion(info, mod_info_curr, pic_map, neigh_mv, neigh_refi, offsetX_in_unit, offsetY_in_unit, blkBoundaryType, &length, lengthInBlock - subIdx, TRUE, cur_ptr, refp);
            if (state == CONSECUTIVE_INTER)
            {
                int sub_blk_x = x + ((blkBoundaryType == 0) ? (subIdx << MIN_CU_LOG2) : 0);
                int sub_blk_y = y + ((blkBoundaryType == 0) ? 0 : (subIdx << MIN_CU_LOG2));
                int sub_blk_w = ((blkBoundaryType == 0) ? (length << MIN_CU_LOG2) : half_ob_width);
                int sub_blk_h = ((blkBoundaryType == 0) ? half_ob_height : (length << MIN_CU_LOG2));

                mod_info_curr->mv[REFP_0][MV_X] = neigh_mv[REFP_0][MV_X];
                mod_info_curr->mv[REFP_0][MV_Y] = neigh_mv[REFP_0][MV_Y];
                mod_info_curr->refi[REFP_0] = neigh_refi[REFP_0];
                mod_info_curr->mv[REFP_1][MV_X] = neigh_mv[REFP_1][MV_X];
                mod_info_curr->mv[REFP_1][MV_Y] = neigh_mv[REFP_1][MV_Y];
                mod_info_curr->refi[REFP_1] = neigh_refi[REFP_1];


                pel *pc_org_buffer[N_C] = { &(mod_info_curr->pred[Y_C][0]) + (sub_blk_x - x) + (sub_blk_y - y)        * cu_width,        \
                                               &(mod_info_curr->pred[U_C][0]) + ((sub_blk_x - x) >> 1) + ((sub_blk_y - y) >> 1) * (cu_width >> 1), \
                                               &(mod_info_curr->pred[V_C][0]) + ((sub_blk_x - x) >> 1) + ((sub_blk_y - y) >> 1) * (cu_width >> 1) };
                pel *pc_subblk_buffer[N_C] = { &(info->subblk_obmc_buf[Y_C][0]) + (sub_blk_x - x) + (sub_blk_y - y)        * cu_width,        \
                                               &(info->subblk_obmc_buf[U_C][0]) + ((sub_blk_x - x) >> 1) + ((sub_blk_y - y) >> 1) * (cu_width >> 1), \
                                               &(info->subblk_obmc_buf[V_C][0]) + ((sub_blk_x - x) >> 1) + ((sub_blk_y - y) >> 1) * (cu_width >> 1) };
                com_subblk_obmc(sub_blk_x, sub_blk_y, sub_blk_w, sub_blk_h, cu_width, pc_subblk_buffer, info, mod_info_curr, refp, luma, doChromaOBMC, bit_depth
#if BGC
                    , bgc_flag, bgc_idx
#endif
                );

#if OBMC_TEMP
                if (info->sqh.obmc_template_enable_flag)
                {
                    com_get_obmc_temp_pred(sub_blk_x, sub_blk_y, sub_blk_w, sub_blk_h, mod_info_curr->obmc_temp_pred1, mod_info_curr->obmc_temp, info, mod_info_curr, refp, TRUE, FALSE, bit_depth, blkBoundaryType);
                    mod_info_curr->mv[REFP_0][MV_X] = org_mv[REFP_0][MV_X];
                    mod_info_curr->mv[REFP_0][MV_Y] = org_mv[REFP_0][MV_Y];
                    mod_info_curr->refi[REFP_0] = org_refi[REFP_0];
                    mod_info_curr->mv[REFP_1][MV_X] = org_mv[REFP_1][MV_X];
                    mod_info_curr->mv[REFP_1][MV_Y] = org_mv[REFP_1][MV_Y];
                    mod_info_curr->refi[REFP_1] = org_refi[REFP_1];
                    com_get_obmc_temp_pred(sub_blk_x, sub_blk_y, sub_blk_w, sub_blk_h, mod_info_curr->obmc_temp_pred2, mod_info_curr->obmc_temp, info, mod_info_curr, refp, TRUE, FALSE, bit_depth, blkBoundaryType);

                    int f_x = ((mod_info_curr->x_scu) << 2) + ((blkBoundaryType == 0) ? (subIdx << MIN_CU_LOG2) : 0);
                    int f_y = ((mod_info_curr->y_scu) << 2) + ((blkBoundaryType == 0) ? 0 : (subIdx << MIN_CU_LOG2));
                    int  stride_y = pic->stride_luma;
                    pel* reco_y = pic->y + (f_y * stride_y) + f_x;

                    if (blkBoundaryType == 0)
                    {
                        reco_y = reco_y - stride_y;
                    }
                    if (blkBoundaryType == 1)
                    {
                        reco_y = reco_y - 1;
                    }
                    int obmc_mode = caculate_obmc_mode(reco_y, stride_y, mod_info_curr->obmc_temp_pred2, mod_info_curr->obmc_temp_pred1, sub_blk_w, sub_blk_h, bit_depth, blkBoundaryType, ((blkBoundaryType == 0) ? weight_above : weight_left));

                    if (info->pic_header.obmc_blending_flag)
                    {
                        if (obmc_mode == 0)
                        {
                            weight_above = info->obmc_weight[3][log2_half_h];
                            weight_above_c = info->obmc_weight[3][log2_half_h_c];
                            weight_left = info->obmc_weight[3][log2_half_w];
                            weight_left_c = info->obmc_weight[3][log2_half_w_c];
                        }
                        if (obmc_mode == 1)
                        {
                            weight_above = info->obmc_weight[1][log2_half_h];
                            weight_above_c = info->obmc_weight[1][log2_half_h_c];
                            weight_left = info->obmc_weight[1][log2_half_w];
                            weight_left_c = info->obmc_weight[1][log2_half_w_c];
                        }
                        if (obmc_mode == 2)
                        {
                            weight_above = info->obmc_weight[4][log2_half_h];
                            weight_above_c = info->obmc_weight[4][log2_half_h_c];
                            weight_left = info->obmc_weight[4][log2_half_w];
                            weight_left_c = info->obmc_weight[4][log2_half_w_c];
                        }
                    }
                    else
                    {
                        if (obmc_mode == 0)
                        {
                            weight_above = info->obmc_weight[0][log2_half_h];
                            weight_above_c = info->obmc_weight[0][log2_half_h_c];
                            weight_left = info->obmc_weight[0][log2_half_w];
                            weight_left_c = info->obmc_weight[0][log2_half_w_c];
                        }
                        if (obmc_mode == 1)
                        {
                            weight_above = info->obmc_weight[2][log2_half_h];
                            weight_above_c = info->obmc_weight[2][log2_half_h_c];
                            weight_left = info->obmc_weight[2][log2_half_w];
                            weight_left_c = info->obmc_weight[2][log2_half_w_c];
                        }
                        if (obmc_mode == 2)
                        {
                            weight_above = info->obmc_weight[1][log2_half_h];
                            weight_above_c = info->obmc_weight[1][log2_half_h_c];
                            weight_left = info->obmc_weight[1][log2_half_w];
                            weight_left_c = info->obmc_weight[1][log2_half_w_c];
                        }
                    }


                    if (luma)
                    {
                        com_obmc_blending(pc_org_buffer[Y_C], cu_width, pc_subblk_buffer[Y_C], cu_width, sub_blk_w, sub_blk_h, blkBoundaryType, ((blkBoundaryType == 0) ? weight_above : weight_left), bit_depth);
                    }
                    if (doChromaOBMC)
                    {
                        com_obmc_blending(pc_org_buffer[U_C], (cu_width >> 1), pc_subblk_buffer[U_C], (cu_width >> 1), (sub_blk_w >> 1), (sub_blk_h >> 1), blkBoundaryType, ((blkBoundaryType == 0) ? weight_above_c : weight_left_c), bit_depth);
                        com_obmc_blending(pc_org_buffer[V_C], (cu_width >> 1), pc_subblk_buffer[V_C], (cu_width >> 1), (sub_blk_w >> 1), (sub_blk_h >> 1), blkBoundaryType, ((blkBoundaryType == 0) ? weight_above_c : weight_left_c), bit_depth);
                    }
                }else
                {
                    if (luma)
                    {
                        com_obmc_blending(pc_org_buffer[Y_C], cu_width, pc_subblk_buffer[Y_C], cu_width, sub_blk_w, sub_blk_h, blkBoundaryType, ((blkBoundaryType == 0) ? weight_above : weight_left), bit_depth);
                    }
                    if (doChromaOBMC)
                    {
                        com_obmc_blending(pc_org_buffer[U_C], (cu_width >> 1), pc_subblk_buffer[U_C], (cu_width >> 1), (sub_blk_w >> 1), (sub_blk_h >> 1), blkBoundaryType, ((blkBoundaryType == 0) ? weight_above_c : weight_left_c), bit_depth);
                        com_obmc_blending(pc_org_buffer[V_C], (cu_width >> 1), pc_subblk_buffer[V_C], (cu_width >> 1), (sub_blk_w >> 1), (sub_blk_h >> 1), blkBoundaryType, ((blkBoundaryType == 0) ? weight_above_c : weight_left_c), bit_depth);
                    }
                }
#else
                if (luma)
                {
                    com_obmc_blending(pc_org_buffer[Y_C], cu_width, pc_subblk_buffer[Y_C], cu_width, sub_blk_w, sub_blk_h, blkBoundaryType, ((blkBoundaryType == 0) ? weight_above : weight_left), bit_depth);
                }
                if (doChromaOBMC)
                {
                    com_obmc_blending(pc_org_buffer[U_C], (cu_width >> 1), pc_subblk_buffer[U_C], (cu_width >> 1), (sub_blk_w >> 1), (sub_blk_h >> 1), blkBoundaryType, ((blkBoundaryType == 0) ? weight_above_c : weight_left_c), bit_depth);
                    com_obmc_blending(pc_org_buffer[V_C], (cu_width >> 1), pc_subblk_buffer[V_C], (cu_width >> 1), (sub_blk_w >> 1), (sub_blk_h >> 1), blkBoundaryType, ((blkBoundaryType == 0) ? weight_above_c : weight_left_c), bit_depth);
                }
#endif
                mod_info_curr->mv[REFP_0][MV_X] = org_mv[REFP_0][MV_X];
                mod_info_curr->mv[REFP_0][MV_Y] = org_mv[REFP_0][MV_Y];
                mod_info_curr->refi[REFP_0] = org_refi[REFP_0];
                mod_info_curr->mv[REFP_1][MV_X] = org_mv[REFP_1][MV_X];
                mod_info_curr->mv[REFP_1][MV_Y] = org_mv[REFP_1][MV_Y];
                mod_info_curr->refi[REFP_1] = org_refi[REFP_1];

                subIdx += length;
            }
            else if (state == CUR_BI_PRED)
            {
                subIdx += length;
            }
            else if ((state == CONSECUTIVE_INTRA) || (state == CONSECUTIVE_IBC))
            {
                subIdx += length;
            }
            else
            {
                subIdx += lengthInBlock;
            }
        }
        assert(subIdx == lengthInBlock);
    }
}

void com_subblk_obmc(int x, int y, int w, int h, int pred_stride, pel* pred_buf[N_C], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], BOOL luma, BOOL doChromaOBMC, int bit_depth
#if BGC
    , s8 bgc_flag, s8 bgc_idx
#endif
)
{
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    s8 *refi = mod_info_curr->refi;
    s16(*mv)[MV_D] = mod_info_curr->mv;

    COM_PIC *ref_pic = NULL;
#if BGC
    pel *pred_fir = info->pred_tmp;
    pel *p0 = NULL, *p1 = NULL, *dest0 = NULL;
    pel *p2 = NULL, *p3 = NULL;
    pel *pred_fir_u = info->pred_tmp_c[0];
    pel *pred_fir_v = info->pred_tmp_c[1];
    pel *dst_u = NULL, *dst_v = NULL;
#endif
    pel* pred[N_C] = { pred_buf[Y_C], pred_buf[U_C], pred_buf[V_C] };
    int qpel_gmv_x = 0, qpel_gmv_y = 0, bidx = 0;

    s16 mv_t[REFP_NUM][MV_D];
    mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, mv_t);

    if (REFI_IS_VALID(refi[REFP_0]))
    {
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_0][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_0][MV_Y];

        if (luma)
        {
            com_mc_l(mv[REFP_0][0], mv[REFP_0][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
        }
        if (doChromaOBMC)
        {
            com_mc_c(mv[REFP_0][0], mv[REFP_0][1], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth);
            com_mc_c(mv[REFP_0][0], mv[REFP_0][1], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth);
        }

        if (REFI_IS_VALID(refi[REFP_1]))
        {
#if LIBVC_ON
            if ((refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr) && (mv[REFP_0][MV_X] == mv[REFP_1][MV_X]) && (mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y]) && (refp[refi[REFP_0]][REFP_0].is_library_picture == refp[refi[REFP_1]][REFP_1].is_library_picture))
#else
            if ((refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr) && (mv[REFP_0][MV_X] == mv[REFP_1][MV_X]) && (mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y]))
#endif
            {
                return;
            }
        }

#if BGC
        if (bgc_flag && luma)
        {
            p0 = pred[Y_C];
            dest0 = pred_fir;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    dest0[i] = p0[i];
                }
                p0 += pred_stride;
                dest0 += pred_stride;
            }
        }
        if (bgc_flag && doChromaOBMC)
        {
            p0 = pred[U_C];
            p1 = pred[V_C];
            dst_u = pred_fir_u;
            dst_v = pred_fir_v;
            for (int j = 0; j < (h >> 1); j++)
            {
                for (int i = 0; i < (w >> 1); i++)
                {
                    dst_u[i] = p0[i];
                    dst_v[i] = p1[i];
                }
                p0 += (pred_stride >> 1);
                p1 += (pred_stride >> 1);
                dst_u += (pred_stride >> 1);
                dst_v += (pred_stride >> 1);
            }
        }
#endif
        bidx++;
    }

    if (REFI_IS_VALID(refi[REFP_1]))
    {
        if (bidx)
        {
            pred[Y_C] = info->pred_buf_snd[Y_C];
            pred[U_C] = info->pred_buf_snd[U_C];
            pred[V_C] = info->pred_buf_snd[V_C];
        }
        ref_pic = refp[refi[REFP_1]][REFP_1].pic;
        qpel_gmv_x = (x << 2) + mv_t[REFP_1][MV_X];
        qpel_gmv_y = (y << 2) + mv_t[REFP_1][MV_Y];

        if (luma)
        {
            com_mc_l(mv[REFP_1][0], mv[REFP_1][1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
        }
        if (doChromaOBMC)
        {
            com_mc_c(mv[REFP_1][0], mv[REFP_1][1], ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth);
            com_mc_c(mv[REFP_1][0], mv[REFP_1][1], ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth);
        }
        bidx++;
    }

    if (bidx == 2)
    {
        if (luma)
        {
#if SIMD_MC
            average_16b_no_clip_sse(pred_buf[Y_C], info->pred_buf_snd[Y_C], pred_buf[Y_C], pred_stride, pred_stride, pred_stride, w, h);
#else
            pel *pTmp0 = pred_buf[Y_C];
            pel *pTmp1 = info->pred_buf_snd[Y_C];
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    pTmp0[i] = (pTmp0[i] + pTmp1[i] + 1) >> 1;
                }
                pTmp0 += pred_stride;
                pTmp1 += pred_stride;
            }
#endif
#if BGC
            if (bgc_flag)
            {
                p0 = pred_fir;
                p1 = info->pred_buf_snd[Y_C];
                dest0 = pred_buf[Y_C];
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        if (bgc_idx)
                        {
                            dest0[i] += (p0[i] - p1[i]) >> 3;
                        }
                        else
                        {
                            dest0[i] += (p1[i] - p0[i]) >> 3;
                        }
                        dest0[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dest0[i]);
                    }
                    p0 += pred_stride;
                    p1 += pred_stride;
                    dest0 += pred_stride;
                }
            }
#endif
        }

        if (doChromaOBMC)
        {
#if SIMD_MC
            average_16b_no_clip_sse(pred_buf[U_C], info->pred_buf_snd[U_C], pred_buf[U_C], (pred_stride >> 1), (pred_stride >> 1), (pred_stride >> 1), (w >> 1), (h >> 1));
            average_16b_no_clip_sse(pred_buf[V_C], info->pred_buf_snd[V_C], pred_buf[V_C], (pred_stride >> 1), (pred_stride >> 1), (pred_stride >> 1), (w >> 1), (h >> 1));
#else
            pel *pTmp0 = pred_buf[U_C];
            pel *pTmp1 = info->pred_buf_snd[U_C];
            pel *pTmp2 = pred_buf[V_C];
            pel *pTmp3 = info->pred_buf_snd[V_C];
            for (int j = 0; j < (h >> 1); j++)
            {
                for (int i = 0; i < (w >> 1); i++)
                {
                    p0[i] = (p0[i] + p1[i] + 1) >> 1;
                    p2[i] = (p2[i] + p3[i] + 1) >> 1;
                }
                pTmp0 += (pred_stride >> 1);
                pTmp1 += (pred_stride >> 1);
                pTmp2 += (pred_stride >> 1);
                pTmp3 += (pred_stride >> 1);
            }
#endif
            if (bgc_flag)
            {
                p0 = pred_fir_u;
                p1 = info->pred_buf_snd[U_C];
                p2 = pred_fir_v;
                p3 = info->pred_buf_snd[V_C];
                dst_u = pred_buf[U_C];
                dst_v = pred_buf[V_C];
                for (int j = 0; j < (h >> 1); j++)
                {
                    for (int i = 0; i < (w >> 1); i++)
                    {
                        if (bgc_idx)
                        {
                            dst_u[i] += (p0[i] - p1[i]) >> 3;
                            dst_v[i] += (p2[i] - p3[i]) >> 3;
                        }
                        else
                        {
                            dst_u[i] += (p1[i] - p0[i]) >> 3;
                            dst_v[i] += (p3[i] - p2[i]) >> 3;
                        }
                        dst_u[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dst_u[i]);
                        dst_v[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dst_v[i]);
                    }
                    p0 += (pred_stride >> 1);
                    p1 += (pred_stride >> 1);
                    p2 += (pred_stride >> 1);
                    p3 += (pred_stride >> 1);
                    dst_u += (pred_stride >> 1);
                    dst_v += (pred_stride >> 1);
                }
            }
        }
    }
}
#if OBMC_TEMP

void com_get_obmc_temp_pred(int x, int y, int w, int h,  pel pred_buf[MAX_CU_SIZE],pel temp_buf[MAX_CU_SIZE], COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], BOOL luma, BOOL doChromaOBMC, int bit_depth,int dir)
{
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    s8 *refi = mod_info_curr->refi;
    s16(*mv)[MV_D] = mod_info_curr->mv;

    COM_PIC *ref_pic = NULL;
    int qpel_gmv_x = 0, qpel_gmv_y = 0;

    s16 mv_t[REFP_NUM][MV_D];
    
    mv_t[REFP_0][MV_X] = mv[REFP_0][MV_X];
    mv_t[REFP_0][MV_Y] = mv[REFP_0][MV_Y];
    mv_t[REFP_1][MV_X] = mv[REFP_1][MV_X];
    mv_t[REFP_1][MV_Y] = mv[REFP_1][MV_Y];
    
    mv_clip(x, y, pic_w, pic_h, w, h, refi, mv, mv_t);//mv is origin_mv, mv_t is filter_mv

    int bidx = 0;
    pel* pred =  NULL;
    if (REFI_IS_VALID(refi[REFP_0]))
    {
        ref_pic = refp[refi[REFP_0]][REFP_0].pic;
        if (dir == 0)
        {
            pred = pred_buf;//for up_template
            qpel_gmv_x = x + (mv_t[REFP_0][MV_X]>>2);
            qpel_gmv_y = y -1 + (mv_t[REFP_0][MV_Y]>>2);
            if (qpel_gmv_y < 0)
                qpel_gmv_y = 0;
            int sw = w;
            int sh = 1;
            if (luma)
            {
                com_mc_obmc_up(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred, sw, bit_depth);
            }
        }
        if (dir == 1)
        {
            pred = pred_buf;//for left_template
            qpel_gmv_x = x -1 + (mv_t[REFP_0][MV_X]>>2);
            if (qpel_gmv_x < 0)
                qpel_gmv_x = 0;
            qpel_gmv_y = y + (mv_t[REFP_0][MV_Y]>>2);
            int sw = 1;
            int sh = h;
            if (luma)
                com_mc_obmc_left(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred, sh, bit_depth);
        }
        if (REFI_IS_VALID(refi[REFP_1]))
        {
#if LIBVC_ON
            if ((refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr) && (mv[REFP_0][MV_X] == mv[REFP_1][MV_X]) && (mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y]) && (refp[refi[REFP_0]][REFP_0].is_library_picture == refp[refi[REFP_1]][REFP_1].is_library_picture))
#else
            if ((refp[refi[REFP_0]][REFP_0].pic->ptr == refp[refi[REFP_1]][REFP_1].pic->ptr) && (mv[REFP_0][MV_X] == mv[REFP_1][MV_X]) && (mv[REFP_0][MV_Y] == mv[REFP_1][MV_Y]))
#endif
            {
                return;
            }
        }


        bidx++;
    }



    if (REFI_IS_VALID(refi[REFP_1]))
    {
        if (bidx)
        {
            pred = temp_buf;
        }
        else
        {
            pred = pred_buf;
        }
        ref_pic = refp[refi[REFP_1]][REFP_1].pic;//for up_template
        if (dir == 0)
        {
            qpel_gmv_x = x + (mv_t[REFP_1][MV_X]>>2);
            qpel_gmv_y = y -1 + (mv_t[REFP_1][MV_Y]>>2);
            if (qpel_gmv_y < 0)
                qpel_gmv_y = 0;
            int sw = w;
            int sh = 1;
            if (luma)
                com_mc_obmc_up(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred, sw, bit_depth);
        }
        if (dir == 1)
        {
            qpel_gmv_x = x -1 + (mv_t[REFP_1][MV_X]>>2);//for left_template
            if (qpel_gmv_x < 0)
                qpel_gmv_x = 0;
            qpel_gmv_y = y + (mv_t[REFP_1][MV_Y]>>2);
            int sw = 1;
            int sh = h;
            if (luma)
                com_mc_obmc_left(ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred, sh, bit_depth);
        }
        bidx++;
    }
    if (bidx == 2)
    {
        if (dir == 0)
        {
            for (int i = 0; i < w; i++)
            {
                pred_buf[i] = ((pred_buf[i] + temp_buf[i] + 1) >> 1);
            }
        }
        if (dir == 1)
        {
            for (int i = 0; i < h; i++)
            {
                pred_buf[i] = ((pred_buf[i] + temp_buf[i] + 1) >> 1);
            }
        }
    }
}


#endif
void com_obmc_blending(pel *yuvPredDst, int predDstStride, pel *yuvPredSrc, int predSrcStride, int width, int height, int dir, pel *weight, int bit_depth)
{
    pel *pDst = yuvPredDst;
    pel *pSrc = yuvPredSrc;

#if SIMD_MC
    int shift = 6;
    __m128i min = _mm_setzero_si128();
    __m128i max = _mm_set1_epi32((1 << bit_depth) - 1);
    __m128i offset = _mm_set1_epi32(1 << (shift - 1));
    __m128i c = _mm_set1_epi32(64);

    if (dir == 0) //above
    {
        for (int j = 0; j < height; j++)
        {
            __m128i wdst = _mm_set1_epi32(weight[j]);
            __m128i wsrc = _mm_sub_epi32(c, wdst);
            int i = 0;
            for (; i < ((width >> 2) << 2); i += 4)
            {
                __m128i vdst = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pDst[i])));
                __m128i vsrc = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pSrc[i])));
                vdst = _mm_mullo_epi32(wdst, vdst);
                vsrc = _mm_mullo_epi32(wsrc, vsrc);
                vdst = _mm_add_epi32(_mm_add_epi32(vdst, vsrc), offset);
                vdst = _mm_srai_epi32(vdst, shift);
                vdst = _mm_min_epi32(max, _mm_max_epi32(vdst, min));
                vdst = _mm_packs_epi32(vdst, vdst);
                _mm_storel_epi64((__m128i *)&(pDst[i]), vdst);
            }
            for (; i < ((width >> 1) << 1); i += 2)
            {
                __m128i vdst = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pDst[i])));
                __m128i vsrc = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pSrc[i])));
                vdst = _mm_mullo_epi32(wdst, vdst);
                vsrc = _mm_mullo_epi32(wsrc, vsrc);
                vdst = _mm_add_epi32(_mm_add_epi32(vdst, vsrc), offset);
                vdst = _mm_srai_epi32(vdst, shift);
                vdst = _mm_min_epi32(max, _mm_max_epi32(vdst, min));
                pDst[i] = (pel)_mm_extract_epi32(vdst, 0);
                pDst[i + 1] = (pel)_mm_extract_epi32(vdst, 1);
            }
            for (; i < width; i++)
            {
                int value = ((int)weight[j] * (int)pDst[i] + (int)(64 - weight[j]) * (int)pSrc[i] + 32) >> 6;
                pDst[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, value);
            }
            pDst += predDstStride;
            pSrc += predSrcStride;
        }
    }

    if (dir == 1) //left
    {
        for (int j = 0; j < height; j++)
        {
            int i = 0;
            for (; i < ((width >> 2) << 2); i += 4)
            {
                __m128i wdst = _mm_set_epi32(weight[i + 3], weight[i + 2], weight[i + 1], weight[i]);
                __m128i wsrc = _mm_sub_epi32(c, wdst);

                __m128i vdst = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pDst[i])));
                __m128i vsrc = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pSrc[i])));
                vdst = _mm_mullo_epi32(wdst, vdst);
                vsrc = _mm_mullo_epi32(wsrc, vsrc);
                vdst = _mm_add_epi32(_mm_add_epi32(vdst, vsrc), offset);
                vdst = _mm_srai_epi32(vdst, shift);
                vdst = _mm_min_epi32(max, _mm_max_epi32(vdst, min));
                vdst = _mm_packs_epi32(vdst, vdst);
                _mm_storel_epi64((__m128i *)&(pDst[i]), vdst);
            }
            for (; i < ((width >> 1) << 1); i += 2)
            {
                __m128i wdst = _mm_set_epi32(0, 0, weight[i + 1], weight[i]);
                __m128i wsrc = _mm_sub_epi32(c, wdst);

                __m128i vdst = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pDst[i])));
                __m128i vsrc = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)&(pSrc[i])));
                vdst = _mm_mullo_epi32(wdst, vdst);
                vsrc = _mm_mullo_epi32(wsrc, vsrc);
                vdst = _mm_add_epi32(_mm_add_epi32(vdst, vsrc), offset);
                vdst = _mm_srai_epi32(vdst, shift);
                vdst = _mm_min_epi32(max, _mm_max_epi32(vdst, min));
                pDst[i] = (pel)_mm_extract_epi32(vdst, 0);
                pDst[i + 1] = (pel)_mm_extract_epi32(vdst, 1);
            }
            for (; i < width; i++)
            {
                int value = ((int)weight[i] * (int)pDst[i] + (int)(64 - weight[i]) * (int)pSrc[i] + 32) >> 6;
                pDst[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, value);
            }
            pDst += predDstStride;
            pSrc += predSrcStride;
        }
    }
#else
    if (dir == 0) //above
    {
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                int value = ((int)weight[j] * (int)pDst[i] + (int)(64 - weight[j]) * (int)pSrc[i] + 32) >> 6;
                pDst[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, value);
            }
            pDst += predDstStride;
            pSrc += predSrcStride;
        }
    }

    if (dir == 1) //left
    {
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                int value = ((int)weight[i] * (int)pDst[i + j * predDstStride] + (int)(64 - weight[i]) * (int)pSrc[i + j * predSrcStride] + 32) >> 6;
                pDst[i + j * predDstStride] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, value);
            }
        }
    }
#endif
}
#endif

#if SAWP_WEIGHT_OPT || AWP_ENH
int get_awp_weight_by_blend_idx(int distance, int blend_idx)
{
#if AWP_ENH
    const int blend_size_lut[TOTAL_BLEND_NUM] = {
        4, 8, 16, 1, 2,
    };

    const int blend_shift_lut[TOTAL_BLEND_NUM] = {
        2, 1, 0, 4, 3,
    };

    const int offset = 4 - blend_size_lut[blend_idx];
    const int shift = blend_shift_lut[blend_idx];
    const int max_weight = 32;

    static const int lut[33] = { 1, 1, 1, 2, 2, 2, 3, 4, 5, 5, 7, 8, 9, 11,
                                  12, 14, 16, 18, 20, 21, 23, 24, 25, 27, 27,
                                  28, 29, 30, 30, 30, 31, 31, 31 };
    int weight = distance << shift;
    weight = COM_CLIP3(0, max_weight, weight);

    if (blend_idx >= 0 && blend_idx < TOTAL_BLEND_NUM)
    {
        int blend_size = blend_size_lut[blend_idx] << 1;
        if (distance >= 0 && distance <= blend_size)
        {
            weight = lut[distance << shift];
        }
    }
    return weight;
#else
    int weight = COM_CLIP3(0, 8, distance);
    if (blend_idx == 1)
    {
        static const int lut[9] = { 0, 1, 2, 5, 8, 11, 14, 15, 16 };
        return lut[weight];
    }
    else
    {
        return weight;
    }
#endif
}
#endif

#if AWP_ENH
void derive_awp_blend_lut(int* size_lut, int* shift_lut)
{
    const int blend_size_lut[TOTAL_BLEND_NUM] = {
        4, 8, 16, 1, 2,
    };

    for (int i = 0; i < TOTAL_BLEND_NUM; i++)
    {
        size_lut[i] = blend_size_lut[i];

        shift_lut[i] = 4 - CONV_LOG2(blend_size_lut[i]);
    }
}
#endif

#if AWP || SAWP
#if SAWP_WEIGHT_OPT || AWP_ENH
void com_calculate_awp_weight(pel awp_weight0[N_C][MAX_AWP_DIM], pel awp_weight1[N_C][MAX_AWP_DIM], int comp_idx, int cu_width, int cu_height, int blend_idx, int step_idx, int angle_idx, int angle_area, int refine_flag, int is_p_slice)
#else
#if BAWP
void com_calculate_awp_weight(pel awp_weight0[N_C][MAX_AWP_DIM], pel awp_weight1[N_C][MAX_AWP_DIM], int comp_idx, int cu_width, int cu_height, int step_idx, int angle_idx, int angle_area, int refine_flag, int is_p_slice)
#else
void com_calculate_awp_weight(pel awp_weight0[N_C][MAX_AWP_DIM], pel awp_weight1[N_C][MAX_AWP_DIM], int comp_idx, int cu_width, int cu_height, int step_idx, int angle_idx, int angle_area, int refine_flag)
#endif
#endif
{
#if BAWP
    if (is_p_slice)
    {
        refine_flag = 0;
    }
    int x_c = 0;
    int y_c = 0;
#endif
    int first_pos = 0;
    int delta_pos_w = 0;
    int delta_pos_h = 0;

    // Set half pixel length
    int valid_length_w = (cu_width + (cu_height >> angle_idx)) << 1;
    int valid_length_h = (cu_height + (cu_width >> angle_idx)) << 1;

    // Reference weight array
    int* final_reference_weights = NULL;

    pel *weight0 = awp_weight0[comp_idx];
    pel *weight1 = awp_weight1[comp_idx];
    const int weight_stride = cu_width;
    int temp_w = ((cu_height << 1) >> angle_idx);
    int temp_h = ((cu_width << 1) >> angle_idx);

    delta_pos_w = (valid_length_w >> 3) - 1;
    delta_pos_h = (valid_length_h >> 3) - 1;
    delta_pos_w = delta_pos_w == 0 ? 1 : delta_pos_w;
    delta_pos_h = delta_pos_h == 0 ? 1 : delta_pos_h;
    delta_pos_w = step_idx * delta_pos_w;
    delta_pos_h = step_idx * delta_pos_h;

    int reference_weights[MAX_AWP_SIZE << 2] = { 0 };

#if AWP_ENH
    int max_weight = 32;
    if (is_p_slice || refine_flag)   
    {
        max_weight = 8;
    }

    int blend_size_lut[TOTAL_BLEND_NUM] = { 0 };
    int blend_shift_lut[TOTAL_BLEND_NUM] = { 0 };
    derive_awp_blend_lut(blend_size_lut, blend_shift_lut);

    const int offset = 4 - blend_size_lut[blend_idx];
    const int shift = blend_shift_lut[blend_idx];

#else
#if SAWP_WEIGHT_OPT
    const int max_weight[2] = { 8, SAWP_WEIGHT_PREC };

    if (is_p_slice || refine_flag)
    {
        blend_idx = 0;
    }
#endif
#endif

    switch (angle_area)
    {
    case 0:
        //Calculate first_pos & reference weights [per block]
#if BAWP
        if (refine_flag || is_p_slice)
#else
        if (refine_flag)
#endif
        {
            first_pos = (valid_length_h >> 1) - 3 + delta_pos_h;
        }
        else
        {
            first_pos = (valid_length_h >> 1) - 6 + delta_pos_h;
#if AWP_ENH
            first_pos = first_pos + offset;
#endif
        }

        if (refine_flag)
        {
            for (int i = 0; i < valid_length_h; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 2);
            }
        }
#if BAWP
        else if (is_p_slice)
        {
            for (int i = 0; i < valid_length_h; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 3);
            }
        }
#endif
        else
        {
            for (int i = 0; i < valid_length_h; i++)
            {
#if AWP_ENH
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
#if SAWP_WEIGHT_OPT
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
                reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
#endif
#endif
            }
        }

        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights;

        // Calculate Weight [per pixel]
        for (int y = 0; y < cu_height; y++)
        {
            for (int x = 0; x < cu_width; x++)
            {
#if BAWP
                if (is_p_slice)
                {
                    x_c = ((x >> 2) << 2) + 2;
                    y_c = ((y >> 2) << 2) + 2;
                }
                else
                {
                    x_c = x;
                    y_c = y;
                }
                weight0[x] = final_reference_weights[(y_c << 1) + ((x_c << 1) >> angle_idx)];
#if AWP_ENH
                weight1[x] = max_weight - weight0[x];
#else
#if SAWP_WEIGHT_OPT
                weight1[x] = max_weight[blend_idx] - weight0[x];
#else
                weight1[x] = 8 - weight0[x];
#endif
#endif
#else
                weight0[x] = final_reference_weights[(y << 1) + ((x << 1) >> angle_idx)];
                weight1[x] = 8 - weight0[x];
#endif
            }
            weight0 += weight_stride;
            weight1 += weight_stride;
        }
        break;
    case 1:
        //Calculate first_pos & reference weights [per block]
#if BAWP
        if (refine_flag || is_p_slice)
#else
        if (refine_flag)
#endif
        {
            first_pos = (valid_length_h >> 1) - 1 + delta_pos_h;
        }
        else
        {
            first_pos = (valid_length_h >> 1) - 4 + delta_pos_h;
#if AWP_ENH
            first_pos = first_pos + offset;
#endif
        }

        if (refine_flag)
        {
            for (int i = 0; i < valid_length_h; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 2);
            }
        }
#if BAWP
        else if (is_p_slice)
        {
            for (int i = 0; i < valid_length_h; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 3);
            }
        }
#endif
        else
        {
            for (int i = 0; i < valid_length_h; i++)
            {
#if AWP_ENH
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
#if SAWP_WEIGHT_OPT
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
                reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
#endif
#endif
            }
        }

        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights + temp_h;

        // Calculate Weight [per pixel]
        for (int y = 0; y < cu_height; y++)
        {
            for (int x = 0; x < cu_width; x++)
            {
#if BAWP
                if (is_p_slice)
                {
                    x_c = ((x >> 2) << 2) + 2;
                    y_c = ((y >> 2) << 2) + 2;
                }
                else
                {
                    x_c = x;
                    y_c = y;
                }
                weight0[x] = final_reference_weights[(y_c << 1) - ((x_c << 1) >> angle_idx)];
#if AWP_ENH
                weight1[x] = max_weight - weight0[x];
#else
#if SAWP_WEIGHT_OPT
                weight1[x] = max_weight[blend_idx] - weight0[x];
#else
                weight1[x] = 8 - weight0[x];
#endif
#endif
#else
                weight0[x] = final_reference_weights[(y << 1) - ((x << 1) >> angle_idx)];
                weight1[x] = 8 - weight0[x];
#endif
            }
            weight0 += weight_stride;
            weight1 += weight_stride;
        }
        break;
    case 2:
        //Calculate first_pos & reference weights [per block]
#if BAWP
        if (refine_flag || is_p_slice)
#else
        if (refine_flag)
#endif
        {
            first_pos = (valid_length_w >> 1) - 1 + delta_pos_w;
        }
        else
        {
            first_pos = (valid_length_w >> 1) - 4 + delta_pos_w;
#if AWP_ENH
            first_pos = first_pos + offset;
#endif
        }

        if (refine_flag)
        {
            for (int i = 0; i < valid_length_w; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 2);
            }
        }
#if BAWP
        else if (is_p_slice)
        {
            for (int i = 0; i < valid_length_w; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 3);
            }
        }
#endif
        else
        {
            for (int i = 0; i < valid_length_w; i++)
            {
#if AWP_ENH
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
#if SAWP_WEIGHT_OPT
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
                reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
#endif
#endif
            }
        }

        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights + temp_w;

        // Calculate Weight [per pixel]
        for (int y = 0; y < cu_height; y++)
        {
            for (int x = 0; x < cu_width; x++)
            {
#if BAWP
                if (is_p_slice)
                {
                    x_c = ((x >> 2) << 2) + 2;
                    y_c = ((y >> 2) << 2) + 2;
                }
                else
                {
                    x_c = x;
                    y_c = y;
                }
                weight0[x] = final_reference_weights[(x_c << 1) - ((y_c << 1) >> angle_idx)];
#if AWP_ENH
                weight1[x] = max_weight - weight0[x];
#else
#if SAWP_WEIGHT_OPT
                weight1[x] = max_weight[blend_idx] - weight0[x];
#else
                weight1[x] = 8 - weight0[x];
#endif
#endif
#else
                weight0[x] = final_reference_weights[(x << 1) - ((y << 1) >> angle_idx)];
                weight1[x] = 8 - weight0[x];
#endif
            }
            weight0 += weight_stride;
            weight1 += weight_stride;
        }
        break;
    case 3:
        //Calculate first_pos & reference weights [per block]
#if BAWP
        if (refine_flag || is_p_slice)
#else
        if (refine_flag)
#endif
        {
            first_pos = (valid_length_w >> 1) - 3 + delta_pos_w;
        }
        else
        {
            first_pos = (valid_length_w >> 1) - 6 + delta_pos_w;
#if AWP_ENH
            first_pos = first_pos + offset;
#endif
        }

        if (refine_flag)
        {
            for (int i = 0; i < valid_length_w; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 2);
            }
        }
#if BAWP
        else if (is_p_slice)
        {
            for (int i = 0; i < valid_length_w; i++)
            {
                reference_weights[i] = COM_CLIP3(0, 8, (i - first_pos) << 3);
            }
        }
#endif
        else
        {
            for (int i = 0; i < valid_length_w; i++)
            {
#if AWP_ENH
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
#if SAWP_WEIGHT_OPT
                reference_weights[i] = get_awp_weight_by_blend_idx(i - first_pos, blend_idx);
#else
                reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
#endif
#endif
            }
        }

        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights;

        // Calculate Weight [per pixel]
        for (int y = 0; y < cu_height; y++)
        {
            for (int x = 0; x < cu_width; x++)
            {
#if BAWP
                if (is_p_slice)
                {
                    x_c = ((x >> 2) << 2) + 2;
                    y_c = ((y >> 2) << 2) + 2;
                }
                else
                {
                    x_c = x;
                    y_c = y;
                }
                weight0[x] = final_reference_weights[(x_c << 1) + ((y_c << 1) >> angle_idx)];
#if AWP_ENH
                weight1[x] = max_weight - weight0[x];
#else
#if SAWP_WEIGHT_OPT
                weight1[x] = max_weight[blend_idx] - weight0[x];
#else
                weight1[x] = 8 - weight0[x];
#endif
#endif
#else
                weight0[x] = final_reference_weights[(x << 1) + ((y << 1) >> angle_idx)];
                weight1[x] = 8 - weight0[x];
#endif
            }
            weight0 += weight_stride;
            weight1 += weight_stride;
        }
        break;
    default:
        printf("\nError: awp parameter not expected\n");
        assert(0);
    }
}

#if BAWP
#if AWP_ENH
void com_calculate_awp_para(int awp_idx, int* blend_idx, int* step_idx, int* angle_idx, int* angle_area, int cu_width, int cu_height, int is_p_slice)
#else
#if SAWP_WEIGHT_OPT
void com_calculate_awp_para(int awp_idx, int *step_idx, int *angle_idx, int *angle_area, int cu_width, int cu_height, int is_p_slice)
#else
void com_calculate_awp_para(int awp_idx, int *step_idx, int *angle_idx, int *angle_area, int cu_width, int cu_height, int is_p_slice)
#endif
#endif
#else
void com_calculate_awp_para(int awp_idx, int *step_idx, int *angle_idx, int *angle_area)
#endif
{
#if BAWP
    int real_idx = 0;
    if (is_p_slice)
    {
        real_idx = com_tbl_bawp_mode[CONV_LOG2(cu_width) - MIN_AWP_SIZE_LOG2][CONV_LOG2(cu_height) - MIN_AWP_SIZE_LOG2][awp_idx] << 1;
    }
    else
    {
        real_idx = awp_idx;
    }

    *step_idx = (real_idx / (AWP_RWFERENCE_SET_NUM + 1)) - (AWP_RWFERENCE_SET_NUM >> 1);
    u8 mod_angle_num = real_idx % AWP_ANGLE_NUM;
#else
    *step_idx = (awp_idx / (AWP_RWFERENCE_SET_NUM + 1)) - (AWP_RWFERENCE_SET_NUM >> 1);
    u8 mod_angle_num = awp_idx % AWP_ANGLE_NUM;
#endif
    *angle_idx = mod_angle_num % 2;
    *angle_idx = (mod_angle_num == 2) ? AWP_ANGLE_HOR : ((mod_angle_num == 6) ? AWP_ANGLE_VER : *angle_idx);
    *angle_area = mod_angle_num >> 1;
}

#if BAWP
#if SAWP_SCC == 0
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM], int is_p_slice, int is_sawp)
#else
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM], int is_p_slice)
#endif
#else
void com_derive_awp_weight(COM_MODE *mod_info_curr, int compIdx, pel weight0[N_C][MAX_AWP_DIM], pel weight1[N_C][MAX_AWP_DIM])
#endif
{
    int step_idx, angle_idx, angle_area;
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
#if AWP_ENH
    int blend_idx = mod_info_curr->awp_blend_idx;
    com_calculate_awp_para(mod_info_curr->skip_idx, &blend_idx, &step_idx, &angle_idx, &angle_area, cu_width, cu_height, is_p_slice);
    int max_weight = (mod_info_curr->ph_awp_refine_flag || is_p_slice) ? 8 : 32;
#else
#if SAWP_WEIGHT_OPT
    int max_weight[2] = { 8, SAWP_WEIGHT_PREC };
    int blend_idx = mod_info_curr->sawp_flag && !mod_info_curr->ph_awp_refine_flag  ? 1 : 0;
#endif
#if BAWP
    com_calculate_awp_para(mod_info_curr->skip_idx, &step_idx, &angle_idx, &angle_area, cu_width, cu_height, is_p_slice); // mod_info_curr->skip_idx is awp_idx
#else
    com_calculate_awp_para(mod_info_curr->skip_idx, &step_idx, &angle_idx, &angle_area); // mod_info_curr->skip_idx is awp_idx
#endif
#endif

    if (compIdx == N_C) //all comp
    {
        // derive weights for luma
#if SAWP_WEIGHT_OPT || AWP_ENH
      com_calculate_awp_weight(weight0, weight1, Y_C, cu_width, cu_height, blend_idx, step_idx, angle_idx, angle_area, mod_info_curr->ph_awp_refine_flag, is_p_slice);
#else
#if BAWP
#if SAWP_SCC == 0
        com_calculate_awp_weight(weight0, weight1, Y_C, cu_width, cu_height, step_idx, angle_idx, angle_area, mod_info_curr->ph_awp_refine_flag && (!is_sawp), is_p_slice);
#else
        com_calculate_awp_weight(weight0, weight1, Y_C, cu_width, cu_height, step_idx, angle_idx, angle_area, mod_info_curr->ph_awp_refine_flag, is_p_slice);
#endif
#else
        com_calculate_awp_weight(weight0, weight1, Y_C, cu_width, cu_height, step_idx, angle_idx, angle_area, 
#if AWP
            mod_info_curr->ph_awp_refine_flag
#else // AWP
            0
#endif // AWP
        );
#endif
#endif

        // derive weight for chroma actually only 1 chroma is enough(top left position for each 2x2 luma area)
        int weightStrideY = cu_width << 1; // each 2 lines
        cu_width >>= 1;
        cu_height >>= 1;
        int weightStrideUV = cu_width; // cu_width >> 1
        pel *pYweight0 = weight0[Y_C];
        pel *pUweight0 = weight0[U_C];
        pel *pVweight0 = weight0[V_C];
        pel *pYweight1 = weight1[Y_C];
        pel *pUweight1 = weight1[U_C];
        pel *pVweight1 = weight1[V_C];
#if BAWP
        int x_c = 0;
        if (is_p_slice)
        {
            weightStrideY = cu_width << 4; // each 8 lines
        }
#endif
        for (int y = 0; y < cu_height; y++)
        {
            for (int x = 0; x < cu_width; x++)
            {
                // U_C is sub
#if BAWP
                if (is_p_slice)
                {
                    x_c = ((x >> 2) << 2);
                }
                else
                {
                    x_c = x;
                }
                pUweight0[x] = pYweight0[x_c << 1];
#else
                pUweight0[x] = pYweight0[x << 1];
#endif
#if AWP_ENH
                pUweight1[x] = max_weight - pUweight0[x];
#else
#if SAWP_WEIGHT_OPT
                pUweight1[x] = max_weight[blend_idx] - pUweight0[x];
#else
                pUweight1[x] = 8 - pUweight0[x];
#endif
#endif

                // V_C is the same as U_C
                pVweight0[x] = pUweight0[x];
                pVweight1[x] = pUweight1[x];
            }
#if BAWP
            if (is_p_slice)
            {
                if ((y + 1) % 4 == 0)
                {
                    pYweight0 += weightStrideY;
                    pYweight1 += weightStrideY;
                }
            }
            else
            {
                pYweight0 += weightStrideY;
                pYweight1 += weightStrideY;
            }
#else
            pYweight0 += weightStrideY;
            pYweight1 += weightStrideY;
#endif
            pUweight0 += weightStrideUV;
            pUweight1 += weightStrideUV;
            pVweight0 += weightStrideUV;
            pVweight1 += weightStrideUV;
        }
    }
    else
    {
        assert(compIdx == Y_C);
#if SAWP_WEIGHT_OPT || AWP_ENH
        com_calculate_awp_weight(weight0, weight1, compIdx, cu_width, cu_height, blend_idx, step_idx, angle_idx, angle_area, mod_info_curr->ph_awp_refine_flag, is_p_slice);
#else
#if BAWP
#if SAWP_SCC == 0
        com_calculate_awp_weight(weight0, weight1, compIdx, cu_width, cu_height, step_idx, angle_idx, angle_area, mod_info_curr->ph_awp_refine_flag && (!is_sawp), is_p_slice);
#else
        com_calculate_awp_weight(weight0, weight1, compIdx, cu_width, cu_height, step_idx, angle_idx, angle_area, mod_info_curr->ph_awp_refine_flag, is_p_slice);
#endif
#else
        com_calculate_awp_weight(weight0, weight1, compIdx, cu_width, cu_height, step_idx, angle_idx, angle_area, 
#if AWP
            mod_info_curr->ph_awp_refine_flag
#else // AWP
            0
#endif // AWP
        );
#endif
#endif
    }
}

#if AWP_ENH
static inline u32 sort_awp_cost_list(u32* in, int inputValueArraySize, int* tbl, int outputIndexArraySize)
{
    int numValidInList = 1;
    static u32 sortedlist[56];
    sortedlist[0] = in[0];
    tbl[0] = (int)0;

    for (int inIdx = 1; inIdx < inputValueArraySize; ++inIdx)
    {
        // Find insertion index position using binary search
        int insertIdx = 0;
        u32* subList = sortedlist;
        u32 subListSize = numValidInList;
        while (subListSize > 1)
        {
            int middleIdx = subListSize >> 1;
            if (in[inIdx] < subList[middleIdx])
            {
                subListSize = middleIdx;
            }
            else
            {
                subList += middleIdx;
                subListSize -= middleIdx;
                insertIdx += middleIdx;
            }
        }
        insertIdx += (in[inIdx] < subList[0] ? 0 : 1);

        // Perform insertion at found index position
        if (insertIdx < outputIndexArraySize)
        {
            int startIdx = outputIndexArraySize - 1;
            if (numValidInList < outputIndexArraySize)
            {
                startIdx = numValidInList;
                ++numValidInList;
            }

            for (int i = startIdx; i > insertIdx; --i)
            {
                sortedlist[i] = sortedlist[i - 1];
                tbl[i] = tbl[i - 1];
            }
            sortedlist[insertIdx] = in[inIdx];
            tbl[insertIdx] = (int)inIdx;
        }
    }

    return numValidInList;
}

void init_awp_tpl(COM_MODE* mod_info_curr, pel**** awp_weight_tpl)
{
    for (int dir = 0; dir < 2; dir++)
    {
        mod_info_curr->tpl_cur_stride[dir] = 1;
        mod_info_curr->tpl_ref_stride[dir] = 1;
        mod_info_curr->tpl_pred_stride[dir] = 1;
        mod_info_curr->tpl_weight_stride[dir] = 1;
        mod_info_curr->tpl_cur_avail[dir] = 0;
    }
    mod_info_curr->tpl_weight = awp_weight_tpl;
}

void com_get_tpl_cur(COM_PIC* pic, u32* map_scu, int pic_width_in_scu, int pic_height_in_scu, COM_MODE* mod_info_curr)
{

    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;

    int i;
    int* tpl_cur_avail = mod_info_curr->tpl_cur_avail;

    /* Y */
    int  s_src = pic->stride_luma;
    pel* src = pic->y + (y * s_src) + x;

    pel* left = mod_info_curr->tpl_cur[1];
    pel* up = mod_info_curr->tpl_cur[0]; 

    tpl_cur_avail[0] = 0;
    tpl_cur_avail[1] = 0;

    u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, pic_width_in_scu, mod_info_curr->scup, map_scu
#if CCNPM
        , pic_height_in_scu, cu_width, cu_height
#endif      
    );
    
    if (IS_AVAIL(avail_cu, AVAIL_UP))
    {
        
        tpl_cur_avail[0] = 1;

        com_mcpy(up, src - s_src, cu_width * sizeof(pel));
    }

    if (IS_AVAIL(avail_cu, AVAIL_LE))
    {
        tpl_cur_avail[1] = 1;

        src--;
        for (i = 0; i < cu_height; ++i)
        {
            left[i] = *src;
            src += s_src;
        }
    }
}

#if AWP_ENH
void com_free_4d_Buf(pel**** array4D, int wNum, int hNum, int Num)
{
    int i, j, k;
    if (array4D)
    {
        for (i = 0; i < wNum; i++)
        {
            if (array4D[i])
            {
                for (j = 0; j < hNum; j++)
                {
                    if (array4D[i][j])
                    {
                        for (k = 0; k < Num; k++)
                        {
                            free(array4D[i][j][k]);
                            array4D[i][j][k] = NULL;
                        }
                        free(array4D[i][j]);
                        array4D[i][j] = NULL;
                    }
                }
                free(array4D[i]);
                array4D[i] = NULL;
            }
        }
        free(array4D);
        array4D = NULL;
    }
}
void com_malloc_4d_Buf(pel***** array4D, int wNum, int hNum, int Num, int sizeNum)
{
    int i, j, k;
    if (((*array4D) = (pel****)calloc(wNum, sizeof(pel***))) == NULL)
    {
        printf("MALLOC FAILED: get_mem4DawpWeight0: array4D");
        assert(0);
        exit(-1);
    }

    for (i = 0; i < wNum; i++)
    {
        if ((*(*array4D + i) = (pel***)calloc(hNum, sizeof(pel**))) == NULL)
        {
            printf("MALLOC FAILED: get_mem3DawpWeight0: array3D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < hNum; j++)
        {
            if ((*(*(*array4D + i) + j) = (pel**)calloc(Num, sizeof(pel*))) == NULL)
            {
                printf("MALLOC FAILED: get_mem2DawpWeight0: array2D");
                assert(0);
                exit(-1);
            }
            for (k = 0; k < Num; k++)
            {
                if ((*(*(*(*array4D + i) + j) + k) = (pel*)calloc(sizeNum, sizeof(pel))) == NULL)
                {
                    printf("MALLOC FAILED: get_mem1DawpWeight0: arrayD");
                    assert(0);
                    exit(-1);
                }
            }
        }
    }
    return;
}
#endif

#if AWP_ENH
void com_derive_awp_tpl_weights(pel**** awp_weight_tpl, int width_idx, int height_idx, int awp_idx)
{
    int   step_idx, angle_idx, angle_area;
    const int   cu_width = (1 << (width_idx + MIN_AWP_SIZE_LOG2));
    const int   cu_height = (1 << (height_idx + MIN_AWP_SIZE_LOG2));
    pel weight;
    pel* tpl_weight[2];
    tpl_weight[0] = &awp_weight_tpl[width_idx][height_idx][awp_idx][0];
    tpl_weight[1] = &awp_weight_tpl[width_idx][height_idx][awp_idx][AWP_TPL_SIZE];
    const int tpl_weight_stride = 1;

    const int bawp_flag = 0;
#if AWP_ENH
    int blend_idx = 0;
    com_calculate_awp_para(awp_idx, &blend_idx, &step_idx, &angle_idx, &angle_area, cu_width, cu_height, bawp_flag);
#else
    com_calculate_awp_para(awp_idx, &step_idx, &angle_idx, &angle_area, cu_width, cu_height, bawp_flag);
#endif

    // Derive weights for luma
    int first_pos = 0;
    //int first_pos_scc = 0;
    int delta_pos_w = 0;
    int delta_pos_h = 0;

    // Set half pixel length
    int valid_length_w = (cu_width + (cu_height >> angle_idx)) << 1;
    int valid_length_h = (cu_height + (cu_width >> angle_idx)) << 1;

    // Reference weight array
    int* final_reference_weights = NULL;

    //const int weight_stride = cu_width;
    int temp_w = ((cu_height << 1) >> angle_idx);
    int temp_h = ((cu_width << 1) >> angle_idx);
    delta_pos_w = (valid_length_w >> 3) - 1;
    delta_pos_h = (valid_length_h >> 3) - 1;
    delta_pos_w = delta_pos_w == 0 ? 1 : delta_pos_w;
    delta_pos_h = delta_pos_h == 0 ? 1 : delta_pos_h;
    delta_pos_w = step_idx * delta_pos_w;
    delta_pos_h = step_idx * delta_pos_h;

    int reference_weights[MAX_AWP_SIZE << 2] = { 0 };

    int dir;
    const int tpl_size = 1;
    int tpl_height[2], tpl_width[2];
    tpl_width[0] = cu_width;
    tpl_height[0] = tpl_size;
    tpl_width[1] = tpl_size;
    tpl_height[1] = cu_height;

    int awp_weight_th = 4;

    switch (angle_area)
    {
    case 0:
        //Calculate first_pos & reference weights [per block]
        first_pos = (valid_length_h >> 1) - 6 + delta_pos_h;

        for (int i = 0; i < valid_length_h; i++)
        {
            reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
        }
        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights;

        // Calculate Weight [per pixel]
        for (dir = 0; dir < 2; dir++)
        {
            for (int y = 0; y < tpl_height[dir]; y++)
            {
                for (int x = 0; x < tpl_width[dir]; x++)
                {
                    weight = final_reference_weights[(y << 1) + ((x << 1) >> angle_idx)];
                    tpl_weight[dir][y * tpl_weight_stride + x] = weight >= awp_weight_th ? 1 : 0;
#if AWP_ENH
                    int tmp_x = dir == 0 ? x : -1;
                    int tmp_y = dir == 0 ? -1 : y;

                    int tmp_pos = (tmp_y << 1) + ((tmp_x << 1) >> angle_idx);
                    int tmp_offset = 0;
                    int tmp_weight = 0;
                    if (tmp_pos + tmp_offset <= first_pos + 3)
                    {
                        tmp_weight = 0;
                    }
                    else
                    {
                        tmp_weight = 1;
                    }
                    tpl_weight[dir][y * tpl_weight_stride + x] = tmp_weight;
#endif
                }
            }
        }
        break;
    case 1:
        //Calculate first_pos & reference weights [per block]
        first_pos = (valid_length_h >> 1) - 4 + delta_pos_h;

        for (int i = 0; i < valid_length_h; i++)
        {
            reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
        }
        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights + temp_h;

        // Calculate Weight [per pixel]

        for (dir = 0; dir < 2; dir++)
        {
            for (int y = 0; y < tpl_height[dir]; y++)
            {
                for (int x = 0; x < tpl_width[dir]; x++)
                {
                    weight = final_reference_weights[(y << 1) - ((x << 1) >> angle_idx)];
                    tpl_weight[dir][y * tpl_weight_stride + x] = weight >= awp_weight_th ? 1 : 0;

#if AWP_ENH
                    int tmp_x = dir == 0 ? x : -1;
                    int tmp_y = dir == 0 ? -1 : y;
                    int tmp_pos = (tmp_y << 1) - ((tmp_x << 1) >> angle_idx);
                    int tmp_offset = temp_h;
                    int tmp_weight = 0;
                    if (tmp_pos + tmp_offset <= first_pos + 3)
                    {
                        tmp_weight = 0;
                    }
                    else
                    {
                        tmp_weight = 1;
                    }
                    tpl_weight[dir][y * tpl_weight_stride + x] = tmp_weight;
#endif

                }
            }
        }
        break;
    case 2:
        //Calculate first_pos & reference weights [per block]
        first_pos = (valid_length_w >> 1) - 4 + delta_pos_w;

        for (int i = 0; i < valid_length_w; i++)
        {
            reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
        }
        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights + temp_w;

        // Calculate Weight [per pixel]
        for (dir = 0; dir < 2; dir++)
        {
            for (int y = 0; y < tpl_height[dir]; y++)
            {
                for (int x = 0; x < tpl_width[dir]; x++)
                {
                    weight = final_reference_weights[(x << 1) - ((y << 1) >> angle_idx)];
                    tpl_weight[dir][y * tpl_weight_stride + x] = weight >= awp_weight_th ? 1 : 0;


#if AWP_ENH
                    int tmp_x = dir == 0 ? x : -1;
                    int tmp_y = dir == 0 ? -1 : y;
                    int tmp_pos = (tmp_x << 1) - ((tmp_y << 1) >> angle_idx);
                    int tmp_offset = temp_w;
                    int tmp_weight = 0;
                    if (tmp_pos + tmp_offset <= first_pos + 3)
                    {
                        tmp_weight = 0;
                    }
                    else
                    {
                        tmp_weight = 1;
                    }
                    tpl_weight[dir][y * tpl_weight_stride + x] = tmp_weight;
#endif

                }
            }
        }
        break;
    case 3:
        //Calculate first_pos & reference weights [per block]
        first_pos = (valid_length_w >> 1) - 6 + delta_pos_w;
        for (int i = 0; i < valid_length_w; i++)
        {
            reference_weights[i] = COM_CLIP3(0, 8, i - first_pos);
        }
        //set Delta to align calculate [per block]
        final_reference_weights = reference_weights;

        // Calculate Weight [per pixel]
        for (dir = 0; dir < 2; dir++)
        {
            for (int y = 0; y < tpl_height[dir]; y++)
            {
                for (int x = 0; x < tpl_width[dir]; x++)
                {
                    weight = final_reference_weights[(x << 1) + ((y << 1) >> angle_idx)];
                    tpl_weight[dir][y * tpl_weight_stride + x] = weight >= awp_weight_th ? 1 : 0;
#if AWP_ENH
                    int tmp_x = dir == 0 ? x : -1;
                    int tmp_y = dir == 0 ? -1 : y;
                    int tmp_pos = (tmp_x << 1) + ((tmp_y << 1) >> angle_idx);
                    int tmp_offset = 0;
                    int tmp_weight = 0;
                    if (tmp_pos + tmp_offset <= first_pos + 3)
                    {
                        tmp_weight = 0;
                    }
                    else
                    {
                        tmp_weight = 1;
                    }
                    tpl_weight[dir][y * tpl_weight_stride + x] = tmp_weight;
#endif

                }
            }
        }
        break;
    default:
        printf("\nError: awp parameter not expected\n");
        assert(0);
    }
}
#endif

#if AWP_ENH
void com_get_tpl_ref(COM_MODE* mod_info_curr, pel* pred_ref0, pel* pred_ref1/*, pel* weight_ref0, pel* weight_ref1*/)
{
    {
        s32 cu_width = mod_info_curr->cu_width;
        s32 cu_height = mod_info_curr->cu_height;
        int pred_stride = 1;

        if (mod_info_curr->tpl_cur_avail[0] == 0 || mod_info_curr->tpl_cur_avail[1] == 0)
        {
            return;
        }

        int dir;
        dir = 0;
        for (int i = 0; i < cu_width; i++)
        {
            mod_info_curr->tpl_ref0[dir][i] = pred_ref0[i];
            mod_info_curr->tpl_ref1[dir][i] = pred_ref1[i];
        }

        dir = 1;
        for (int i = 0; i < cu_height; i++)
        {
            mod_info_curr->tpl_ref0[dir][i] = pred_ref0[i + AWP_TPL_SIZE];
            mod_info_curr->tpl_ref1[dir][i] = pred_ref1[i + AWP_TPL_SIZE];
        }

    }
}
#endif

#if DSAWP
int get_dawp_idx_from_sawp_idx(int ipm, u8 mpm[SAWP_MPM_NUM])
{
    int ipm_code = ipm == mpm[0] ? -2 : mpm[1] == ipm ? -1 : ipm < mpm[0] ? ipm : ipm < mpm[1] ? ipm - 1 : ipm - 2;
    if (ipm_code > 0)
    {
        ipm_code -= 5;
    }
    return ipm_code;
}

int get_sawp_idx_from_dawp_idx(int ipm, u8 mpm[SAWP_MPM_NUM])
{
    if (ipm < 0)
    {
        ipm = mpm[ipm + 2];
    }
    else
    {
        ipm += 5;
        ipm = (ipm + (ipm >= mpm[0]) + ((ipm + 1) >= mpm[1]));
    }
    return ipm;
}

int com_tpl_reorder_sawp_mode(COM_MODE* mod_info_curr, int* mode_list, int* inv_mode_list, COM_INFO* info, u32* map_scu, s8* map_ipm
    , pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int dawp_idx0, int dawp_idx1, u8 (*sawp_mpm_cache)[SAWP_MPM_NUM], pel (*pred_tpl_cache)[MAX_CU_DIM])
{
    int pic_width_in_scu = info->pic_width_in_scu;
    int pic_height_in_scu = info->pic_height_in_scu;
    int bit_depth = info->bit_depth_internal;
    u8 sawp_mpm[SAWP_MPM_NUM];

    int is_mpm_idx0 = dawp_idx0 < 0 ? 1 : 0;
    int is_mpm_idx1 = dawp_idx1 < 0 ? 1 : 0;

    int ipd_idx0;
    int ipd_idx1;

    pel pred_tpl[IPD_CNT][MAX_CU_SIZE * 2];
    int is_check[IPD_CNT][2] = { 0 };
    pel *pred_tmp = mod_info_curr->pred[0];
    int is_p_slice = 0;
    int num_valid_mode = 0;
    u32 cost_list[AWP_MODE_NUM] = { 0, };
    int dir;
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int width_idx = cu_width_log2 - MIN_AWP_SIZE_LOG2;
    int height_idx = cu_height_log2 - MIN_AWP_SIZE_LOG2;
    pel(*tpl_cur)[AWP_TPL_SIZE] = mod_info_curr->tpl_cur;
    pel(*tpl_pred)[AWP_TPL_SIZE] = mod_info_curr->tpl_pred;
    pel* tpl_weight[2];
    int* tpl_cur_stride = mod_info_curr->tpl_cur_stride;
    int* tpl_pred_stride = mod_info_curr->tpl_pred_stride;
    int* tpl_ref_stride = mod_info_curr->tpl_ref_stride;
    int* tpl_weight_stride = mod_info_curr->tpl_weight_stride;
    int* tpl_cur_avail = mod_info_curr->tpl_cur_avail;

    const int tpl_size = 1;
    int tpl_height[2], tpl_width[2];
    tpl_width[0] = cu_width;
    tpl_height[0] = tpl_size;
    tpl_width[1] = tpl_size;
    tpl_height[1] = cu_height;
    int tpl_stride = 1;
    int offset[2] = { 0, AWP_TPL_SIZE };
    int left_tpl_w[2] = {0, 1};
    int above_tpl_h[2] = { 1, 0 };
    for (int i = 0; i < AWP_MODE_NUM; i++)
    {
        mode_list[i] = i;
        inv_mode_list[i] = i;
    }

    if (!tpl_cur_avail[0] || !tpl_cur_avail[1])
    {
        num_valid_mode = AWP_MODE_NUM;
        return AWP_MODE_NUM;
    }

    for (int i = 0; i < AWP_MODE_NUM; i++)
    {
        inv_mode_list[i] = AWP_MODE_NUM;
    }

    int pb_part_idx = 0;
    int pb_x = mod_info_curr->pb_info.sub_x[pb_part_idx];
    int pb_y = mod_info_curr->pb_info.sub_y[pb_part_idx];
    int pb_w = mod_info_curr->pb_info.sub_w[pb_part_idx];
    int pb_h = mod_info_curr->pb_info.sub_h[pb_part_idx];
    int pb_scup = mod_info_curr->pb_info.sub_scup[pb_part_idx];
    int pb_x_scu = PEL2SCU(pb_x);
    int pb_y_scu = PEL2SCU(pb_y);

    u16 avail_cu = com_get_avail_intra(pb_x_scu, pb_y_scu, pic_width_in_scu, pb_scup, map_scu
#if CCNPM
        , pic_height_in_scu, pb_w, pb_h
#endif
    );

    int is_mpm_from_sawp = 0;
    if (sawp_mpm_cache == NULL)
    {
        com_get_sawp_mpm(PEL2SCU(pb_x), PEL2SCU(pb_y), PEL2SCU(pb_w), PEL2SCU(pb_h), map_scu
            , map_ipm, pb_scup, pic_width_in_scu, sawp_mpm, 0 /*use_default*/, &is_mpm_from_sawp);
    }

    int cur_ipd0 = -1, cur_ipd1 = -1;
    int pred_offset_x[2] = { 1 , 0 };
    int pred_offset_y[2] = { 0 , 1 };

    for (int awp_idx = 0; awp_idx < AWP_MODE_NUM; awp_idx++)
    {
        if (sawp_mpm_cache != NULL)
        {
            sawp_mpm[0] = sawp_mpm_cache[awp_idx][0];
            sawp_mpm[1] = sawp_mpm_cache[awp_idx][1];
        }
        else if (is_mpm_from_sawp)
        {
            com_get_sawp_mpm(PEL2SCU(pb_x), PEL2SCU(pb_y), PEL2SCU(pb_w), PEL2SCU(pb_h), map_scu
                , map_ipm, pb_scup, pic_width_in_scu, sawp_mpm, awp_idx, NULL);
        }
        ipd_idx0 = get_sawp_idx_from_dawp_idx(dawp_idx0, sawp_mpm);
        ipd_idx1 = get_sawp_idx_from_dawp_idx(dawp_idx1, sawp_mpm);

        for (dir = 0; dir < 2; dir++)
        {
            if (tpl_cur_avail[dir])
            {
                if (!is_check[ipd_idx0][dir])
                {
                    is_check[ipd_idx0][dir] = 1;
                    if (pred_tpl_cache)
                    {

                        for (int i = 0; i < tpl_height[dir]; i++)
                        {
                            for (int j = 0; j < tpl_width[dir]; j++)
                            {
                                pred_tpl[ipd_idx0][i * tpl_stride + j + offset[dir]] = pred_tpl_cache[ipd_idx0][i * tpl_stride + j + offset[dir]];
                            }
                        }

                    }
                    else
                    {
                        com_tpl_ipred(nb[0][0] + STNUM, nb[0][1] + STNUM, pred_tmp, ipd_idx0, cu_width, cu_height, bit_depth, avail_cu, mod_info_curr->ipf_flag
#if MIPF
                            , info->sqh.mipf_enable_flag
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
                                pred_tpl[ipd_idx0][i * tpl_stride + j + offset[dir]] = pred_tmp[(i + pred_offset_y[dir]) * (cu_width + 1) + (j + pred_offset_x[dir])];
                            }
                        }
                    }
                }

                if (!is_check[ipd_idx1][dir])
                {
                    is_check[ipd_idx1][dir] = 1;
                    assert(ipd_idx1 >= 5 && ipd_idx1 <= 30);
                    if (pred_tpl_cache)
                    {

                        for (int i = 0; i < tpl_height[dir]; i++)
                        {
                            for (int j = 0; j < tpl_width[dir]; j++)
                            {
                                pred_tpl[ipd_idx1][i * tpl_stride + j + offset[dir]] = pred_tpl_cache[ipd_idx1][i * tpl_stride + j + offset[dir]];
                            }
                        }

                    }
                    else
                    {
                        com_tpl_ipred(nb[0][0] + STNUM, nb[0][1] + STNUM, pred_tmp, ipd_idx1, cu_width, cu_height, bit_depth, avail_cu, mod_info_curr->ipf_flag
#if MIPF
                            , info->sqh.mipf_enable_flag
#endif
#if IIP
                            , mod_info_curr->iip_flag
#endif
                            , left_tpl_w[dir], above_tpl_h[dir]);
                        for (int i = 0; i < tpl_height[dir]; i++)
                        {
                            for (int j = 0; j < tpl_width[dir]; j++)
                            {
                                pred_tpl[ipd_idx1][i * tpl_stride + j + offset[dir]] = pred_tmp[(i + pred_offset_y[dir]) * (cu_width + 1) + (j + pred_offset_x[dir])];
                            }
                        }
                    }
                }
                pel* tpl_ref0[2];
                pel* tpl_ref1[2];
                tpl_ref0[dir] = &pred_tpl[ipd_idx0][offset[dir]];
                tpl_ref1[dir] = &pred_tpl[ipd_idx1][offset[dir]];
                int weight_offset = dir == 0 ? 0 : AWP_TPL_SIZE;
                tpl_weight[dir] = &mod_info_curr->tpl_weight[width_idx][height_idx][awp_idx][weight_offset];
                for (int i = 0; i < tpl_height[dir]; i++) {
                    for (int j = 0; j < tpl_width[dir]; j++) {
                        pel w = -(tpl_weight[dir][i * tpl_weight_stride[dir] + j]);
                        pel val = ((w & tpl_ref0[dir][i * tpl_ref_stride[dir] + j]) | ((~w) & tpl_ref1[dir][i * tpl_ref_stride[dir] + j]));
                        tpl_pred[dir][i * tpl_pred_stride[dir] + j] = val;
                        cost_list[awp_idx] += abs(tpl_cur[dir][i * tpl_cur_stride[dir] + j] - tpl_pred[dir][i * tpl_pred_stride[dir] + j]);
                    }
                }
            }
        }
        cur_ipd0 = ipd_idx0;
        cur_ipd1 = ipd_idx1;
    }

    int output_num = 56;
    num_valid_mode = sort_awp_cost_list(cost_list, AWP_MODE_NUM, mode_list, output_num);
    for (int cost_idx = 0; cost_idx < output_num; cost_idx++)
    {
        int awp_idx = mode_list[cost_idx];
        inv_mode_list[awp_idx] = cost_idx;
    }
    return num_valid_mode;
}
#endif

int com_tpl_reorder_awp_mode(COM_MODE* mod_info_curr, int* mode_list, int* inv_mode_list)
{
    int is_p_slice = 0;
    pel(*tpl_cur)[AWP_TPL_SIZE] = mod_info_curr->tpl_cur;
    pel(*tpl_pred)[AWP_TPL_SIZE] = mod_info_curr->tpl_pred;
    pel (*tpl_ref0)[AWP_TPL_SIZE] = mod_info_curr->tpl_ref0;
    pel (*tpl_ref1)[AWP_TPL_SIZE] = mod_info_curr->tpl_ref1;
    pel* tpl_weight[2];
    int cu_width_log2 = mod_info_curr->cu_width_log2;
    int cu_height_log2 = mod_info_curr->cu_height_log2;
    int cu_width = 1 << cu_width_log2;
    int cu_height = 1 << cu_height_log2;
    int width_idx = cu_width_log2 - MIN_AWP_SIZE_LOG2;
    int height_idx = cu_height_log2 - MIN_AWP_SIZE_LOG2;

    int* tpl_cur_stride = mod_info_curr->tpl_cur_stride;
    int* tpl_pred_stride = mod_info_curr->tpl_pred_stride;
    int* tpl_ref_stride = mod_info_curr->tpl_ref_stride;
    int* tpl_weight_stride = mod_info_curr->tpl_weight_stride;
    int* tpl_cur_avail = mod_info_curr->tpl_cur_avail;
    int num_valid_mode = 0;
    u32 cost_list[AWP_MODE_NUM] = { 0, };
    int dir;
    const int tpl_size = 1;
    int tpl_height[2], tpl_width[2];
    tpl_width[0] = cu_width;
    tpl_height[0] = tpl_size;
    tpl_width[1] = tpl_size;
    tpl_height[1] = cu_height;
    int th = 0;
    for (int i = 0; i < AWP_MODE_NUM; i++)
    {
        mode_list[i] = i;
        inv_mode_list[i] = i;
    }

    if (!tpl_cur_avail[0] || !tpl_cur_avail[1])
    {
        num_valid_mode = AWP_MODE_NUM;
        return AWP_MODE_NUM;
    }

    for (int i = 0; i < AWP_MODE_NUM; i++)
    {
        inv_mode_list[i] = AWP_MODE_NUM;
    }

    for (dir = 0; dir < 2; dir++)
    {
        if (tpl_cur_avail[dir])
        {
            for (int awp_idx = 0; awp_idx < AWP_MODE_NUM; awp_idx++)
            {
                int weight_offset = dir == 0 ? 0 : AWP_TPL_SIZE;
                tpl_weight[dir] = &mod_info_curr->tpl_weight[width_idx][height_idx][awp_idx][weight_offset];
                for (int i = 0; i < tpl_height[dir]; i++) {
                    for (int j = 0; j < tpl_width[dir]; j++) {
                        pel w = -(tpl_weight[dir][i * tpl_weight_stride[dir] + j]);
                        pel val = ((w & tpl_ref0[dir][i * tpl_ref_stride[dir] + j]) | ((~w) & tpl_ref1[dir][i * tpl_ref_stride[dir] + j]));
                        tpl_pred[dir][i * tpl_pred_stride[dir] + j] = val;
                        cost_list[awp_idx] += abs(tpl_cur[dir][i * tpl_cur_stride[dir] + j] - tpl_pred[dir][i * tpl_pred_stride[dir] + j]);
                    }
                }
            }
        }
    }

    int output_num = 56;
    num_valid_mode = sort_awp_cost_list(cost_list, AWP_MODE_NUM, mode_list, output_num);
    for (int cost_idx = 0; cost_idx < output_num; cost_idx++)
    {
        int awp_idx = mode_list[cost_idx];
        inv_mode_list[awp_idx] = cost_idx;
    }
    return num_valid_mode;
}
#endif

void com_derive_awp_pred(COM_MODE *mod_info_curr, int compIdx, pel pred_buf0[N_C][MAX_CU_DIM], pel pred_buf1[N_C][MAX_CU_DIM], pel weight0[MAX_AWP_DIM], pel weight1[MAX_AWP_DIM])
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    int pred_stride = cu_width;
    if (compIdx != Y_C)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        pred_stride >>= 1;
    }
#if SIMD_MC
#if AWP_ENH
    int shift = mod_info_curr->slice_type == SLICE_P || mod_info_curr->ph_awp_refine_flag ? 3 : 5;
    const int offset = 1 << (shift - 1);
    weight_average_16b_no_clip_sse(pred_buf0[compIdx], pred_buf1[compIdx], mod_info_curr->pred[compIdx], weight0, weight1, pred_stride, pred_stride, pred_stride, pred_stride, cu_width, cu_height, shift);
#else
#if SAWP_WEIGHT_OPT
    weight_average_16b_no_clip_sse(pred_buf0[compIdx], pred_buf1[compIdx], mod_info_curr->pred[compIdx], weight0, weight1, pred_stride, pred_stride, pred_stride, pred_stride, cu_width, cu_height, 3);
#else
    weight_average_16b_no_clip_sse(pred_buf0[compIdx], pred_buf1[compIdx], mod_info_curr->pred[compIdx], weight0, weight1, pred_stride, pred_stride, pred_stride, pred_stride, cu_width, cu_height);
#endif
#endif
#else
    for (int j = 0; j < cu_height; j++)
    {
        for (int i = 0; i < cu_width; i++)
        {
            mod_info_curr->pred[compIdx][i + j * pred_stride] = (pred_buf0[compIdx][i + j * pred_stride] * weight0[i + j * pred_stride] + pred_buf1[compIdx][i + j * pred_stride] * weight1[i + j * pred_stride] + 4) >> 3;
        }
    }
#endif

}

#if SAWP
void com_derive_sawp_pred(COM_MODE* mod_info_curr, int compIdx, pel pred_buf0[MAX_CU_DIM], pel pred_buf1[MAX_CU_DIM], pel weight0[MAX_AWP_DIM], pel weight1[MAX_AWP_DIM])
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    int pred_stride = cu_width;
    if (compIdx != Y_C)
    {
        cu_width >>= 1;
        cu_height >>= 1;
        pred_stride >>= 1;
    }
#if SAWP_WEIGHT_OPT || AWP_ENH
#if AWP_ENH
    int shift = mod_info_curr->ph_awp_refine_flag ? 3 : 5;
#else
    int shift = mod_info_curr->ph_awp_refine_flag ? 3 : SAWP_WEIGHT_SHIFT;
#endif
#if SIMD_MC
    weight_average_16b_no_clip_sse(pred_buf0, pred_buf1, mod_info_curr->pred[compIdx], weight0, weight1, pred_stride, pred_stride, pred_stride, pred_stride, cu_width, cu_height, shift);
#else
    const int offset = 1 << (shift - 1);
    for (int j = 0; j < cu_height; j++)
    {
        for (int i = 0; i < cu_width; i++)
        {
            mod_info_curr->pred[compIdx][i + j * pred_stride] = ((int)pred_buf0[i + j * pred_stride] * weight0[i + j * pred_stride] + (int)pred_buf1[i + j * pred_stride] * weight1[i + j * pred_stride] + offset) >> shift;
        }
    }
#endif
#else
#if SIMD_MC
    weight_average_16b_no_clip_sse(pred_buf0, pred_buf1, mod_info_curr->pred[compIdx], weight0, weight1, pred_stride, pred_stride, pred_stride, pred_stride, cu_width, cu_height);
#else
    for (int j = 0; j < cu_height; j++)
    {
        for (int i = 0; i < cu_width; i++)
        {
            mod_info_curr->pred[compIdx][i + j * pred_stride] = (pred_buf0[i + j * pred_stride] * weight0[i + j * pred_stride] + pred_buf1[i + j * pred_stride] * weight1[i + j * pred_stride] + 4) >> 3;
        }
    }
#endif
#endif
}
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
)
{
    s32 cu_width  = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x         = mod_info_curr->x_pos;
    s32 y         = mod_info_curr->y_pos;
    s32 sub_w     = MIN_SUB_BLOCK_SIZE;
    s32 sub_h     = MIN_SUB_BLOCK_SIZE;
    s32 h         = 0;
    s32 w         = 0;
    s32 tmp_x     = x;
    s32 tmp_y     = y;
    static pel pred_tmp[N_C][MAX_CU_DIM];
    COM_MOTION *cu_mvfield = (COM_MOTION *)tmp_cu_mvfield;
    BOOL cur_apply_DMVR = dmvr->apply_DMVR;

    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            x = tmp_x + w;
            y = tmp_y + h;

            mod_info_curr->mv[REFP_0][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_Y];
            mod_info_curr->mv[REFP_1][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_X];
            mod_info_curr->mv[REFP_1][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_Y];
            mod_info_curr->refi[REFP_0] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_0];
            mod_info_curr->refi[REFP_1] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_1];
            dmvr->apply_DMVR = cur_apply_DMVR;
            com_mc(x, y, sub_w, sub_h, sub_w, pred_tmp, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
                , dmvr
#endif
#if BIO
                , ptr, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
                , 1
#endif
#if SUB_TMVP
                , 0
#endif
#if BGC
                , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
            );

            if (tree_status != CHANNEL_C)
            {
                for (int i = 0; i < MIN_SUB_BLOCK_SIZE; i++)
                {
                    for (int j = 0; j < MIN_SUB_BLOCK_SIZE; j++)
                    {
                        mod_info_curr->pred[Y_C][w + j + (i + h) * cu_width] = pred_tmp[Y_C][j + i * MIN_SUB_BLOCK_SIZE];
                    }
                }
            }

            if (tree_status != CHANNEL_L)
            {
                for (int i = 0; i < MIN_CU_SIZE; i++)
                {
                    for (int j = 0; j < MIN_CU_SIZE; j++)
                    {
                        mod_info_curr->pred[U_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[U_C][j + i * MIN_CU_SIZE];
                        mod_info_curr->pred[V_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[V_C][j + i * MIN_CU_SIZE];
                    }
                }
            }
        }
    }
}
#endif
#if SUB_TMVP
void com_sbTmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, s32 sub_blk_width, s32 sub_blk_height, COM_MOTION* sbTmvp, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
)
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;
    s32 sub_w = sub_blk_width;
    s32 sub_h = sub_blk_height;
    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    static pel pred_tmp[N_C][MAX_CU_DIM];
    BOOL cur_apply_DMVR = dmvr->apply_DMVR;
    for (int k = 0; k<SBTMVP_NUM; k++)
    {
        w = (k % 2)*sub_w;
        h = (k / 2)*sub_h;
        x = tmp_x + w;
        y = tmp_y + h;
        copy_mv(mod_info_curr->mv[REFP_0], sbTmvp[k].mv[REFP_0]);
        copy_mv(mod_info_curr->mv[REFP_1], sbTmvp[k].mv[REFP_1]);
        mod_info_curr->refi[REFP_0] = sbTmvp[k].ref_idx[REFP_0];
        mod_info_curr->refi[REFP_1] = sbTmvp[k].ref_idx[REFP_1];
        dmvr->apply_DMVR= cur_apply_DMVR ; 
        com_mc(x, y, sub_w, sub_h, sub_w, pred_tmp, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
            , dmvr
#endif
#if BIO
            , ptr, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
            , 0
#endif
#if SUB_TMVP
            , 1
#endif
#if BGC
            , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
        );

        if (tree_status != CHANNEL_C)
        {
            for (int i = 0; i < sub_w; i++)
            {
                for (int j = 0; j < sub_h; j++)
                {
                    mod_info_curr->pred[Y_C][w + i + (j + h) * cu_width] = pred_tmp[Y_C][i + j * sub_w];
                }
            }
        }
        
        if (tree_status != CHANNEL_L)
        {
            for (int i = 0; i < (sub_w >> 1); i++)
            {
                for (int j = 0; j < (sub_h >> 1); j++)
                {
                    mod_info_curr->pred[U_C][(w >> 1) + i + (j + (h >> 1)) * (cu_width >> 1)] = pred_tmp[U_C][i + j * (sub_w >> 1)];
                    mod_info_curr->pred[V_C][(w >> 1) + i + (j + (h >> 1)) * (cu_width >> 1)] = pred_tmp[V_C][i + j * (sub_w >> 1)];
                }
            }
        }
    }
}
#endif
#if ETMVP
void com_etmvp_mc(COM_INFO *info, COM_MODE *mod_info_curr, void *tmp_cu_mvfield, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth
#if DMVR
    , COM_DMVR* dmvr
#endif
#if BIO
    , int ptr, int enc_fast, u8 mvr_idx
#endif
)
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;
    s32 sub_w = MIN_ETMVP_MC_SIZE;
    s32 sub_h = MIN_ETMVP_MC_SIZE;
    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    static pel pred_tmp[N_C][MAX_CU_DIM];
    COM_MOTION *cu_mvfield = (COM_MOTION *)tmp_cu_mvfield;

    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            x = tmp_x + w;
            y = tmp_y + h;

            mod_info_curr->mv[REFP_0][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_0][MV_Y];
            mod_info_curr->mv[REFP_1][MV_X] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_X];
            mod_info_curr->mv[REFP_1][MV_Y] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].mv[REFP_1][MV_Y];
            mod_info_curr->refi[REFP_0] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_0];
            mod_info_curr->refi[REFP_1] = cu_mvfield[(w >> 2) + ((h * cu_width) >> 4)].ref_idx[REFP_1];

            com_mc(x, y, sub_w, sub_h, sub_w, pred_tmp, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
                , dmvr
#endif
#if BIO
                , ptr, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
                , 0
#endif
#if SUB_TMVP
                , 0
#endif
#if BGC
                , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
            );

            if (tree_status != CHANNEL_C)
            {
                for (int i = 0; i < MIN_ETMVP_MC_SIZE; i++)
                {
                    for (int j = 0; j < MIN_ETMVP_MC_SIZE; j++)
                    {
                        mod_info_curr->pred[Y_C][w + j + (i + h) * cu_width] = pred_tmp[Y_C][j + i * MIN_ETMVP_MC_SIZE];
                    }
                }
            }

            if (tree_status != CHANNEL_L)
            {
                for (int i = 0; i < MIN_CU_SIZE; i++)
                {
                    for (int j = 0; j < MIN_CU_SIZE; j++)
                    {
                        mod_info_curr->pred[U_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[U_C][j + i * MIN_CU_SIZE];
                        mod_info_curr->pred[V_C][(w >> 1) + j + (i + (h >> 1)) * (cu_width >> 1)] = pred_tmp[V_C][j + i * MIN_CU_SIZE];
                    }
                }
            }
        }
    }
}
#endif

#if USE_IBC
void com_IBC_mc(int x, int y, int log2_cuw, int log2_cuh, s16 mv[MV_D], COM_PIC *ref_pic, pel pred[N_C][MAX_CU_DIM], CHANNEL_TYPE channel, int bit_depth
    )
{
    int          qpel_gmv_x, qpel_gmv_y;

    int w = 1 << log2_cuw;
    int h = 1 << log2_cuh;
    int pred_stride = w;

    qpel_gmv_x = (x << 2) + mv[MV_X];
    qpel_gmv_y = (y << 2) + mv[MV_Y];

    if (channel != CHANNEL_C)
    {
        com_mc_l(mv[0], mv[1], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_stride, pred[Y_C], w, h, bit_depth);
    }
    if (channel != CHANNEL_L)
    {
#if CHROMA_NOT_SPLIT
        assert(w >= 8 && h >= 8);
#endif
        com_mc_c_ibc(ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[U_C], w >> 1, h >> 1, bit_depth);
        com_mc_c_ibc(ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, pred_stride >> 1, pred[V_C], w >> 1, h >> 1, bit_depth);
    }
}
#endif

#if ASP
#if SIMD_ASP
void com_sec_pred(int dmv_buf_x[DMV_BUF_SIZE], int dmv_buf_y[DMV_BUF_SIZE], pel* pred, pel* pred_temp, int cu_width, int sub_w, int sub_h, BOOL dMvH_on, BOOL dMvV_on, int bit_depth)
{
    __m128i dmv_x_sse, dmv_y_sse, coef_sse, pred_temp_sse, pred_sp_sse;
    __m128i vzero = _mm_setzero_si128();
    __m128i mm_min = _mm_set1_epi32(0);
    __m128i mm_max = _mm_set1_epi32((1 << bit_depth) - 1);
    
    int MvPrecNormShift = 5;
    int pred_temp_w = sub_w + 2;

    for (int yy = 0; yy < sub_h; yy++) 
    {
        for (int xx = 0; xx < sub_w; xx += 4) 
        {
            int pred_temp_xx = xx + 1;
            int pred_temp_yy = yy + 1;
            dmv_x_sse = _mm_loadu_si128((__m128i*)(dmv_buf_x + xx + sub_w * yy));
            dmv_y_sse = _mm_loadu_si128((__m128i*)(dmv_buf_y + xx + sub_w * yy));
            pred_sp_sse = _mm_setzero_si128();

            if (dMvH_on && dMvV_on)
            {
                int current_pos = pred_temp_xx - 1 + (pred_temp_yy - 1) * pred_temp_w;
                //left_up
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_sub_epi32(vzero, _mm_add_epi32(dmv_x_sse, dmv_y_sse));
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //up
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_slli_epi32(_mm_sub_epi32(vzero, dmv_y_sse), ASP_SHIFT);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //right_up
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_sub_epi32(dmv_x_sse, dmv_y_sse);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //left
                current_pos = pred_temp_xx - 1 + pred_temp_yy * pred_temp_w;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_slli_epi32(_mm_sub_epi32(vzero, dmv_x_sse), ASP_SHIFT);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //center
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_set1_epi32(1 << (ASP_SHIFT + 1 + MvPrecNormShift));
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //right
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_slli_epi32(dmv_x_sse, ASP_SHIFT);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //left_down
                current_pos = pred_temp_xx - 1 + (pred_temp_yy + 1) * pred_temp_w;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_sub_epi32(dmv_y_sse, dmv_x_sse);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //down
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_slli_epi32(dmv_y_sse, ASP_SHIFT);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //right_down
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_add_epi32(dmv_x_sse, dmv_y_sse);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));

                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_set1_epi32(1 << (MvPrecNormShift + ASP_SHIFT)));
                pred_sp_sse = _mm_srai_epi32(pred_sp_sse, (MvPrecNormShift + 1 + ASP_SHIFT));
            }
            else if (dMvH_on)
            {
                //left
                int current_pos = pred_temp_xx - 1 + pred_temp_yy * pred_temp_w;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_sub_epi32(vzero, dmv_x_sse);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //center
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_set1_epi32(1 << (1 + MvPrecNormShift));
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //right
                current_pos += 1;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = dmv_x_sse;
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));

                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_set1_epi32(1 << MvPrecNormShift));
                pred_sp_sse = _mm_srai_epi32(pred_sp_sse, (MvPrecNormShift + 1));
            }
            else if (dMvV_on)
            {
                int current_pos = pred_temp_xx + (pred_temp_yy - 1) * pred_temp_w;
                //up
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_sub_epi32(vzero, dmv_y_sse);
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //center
                current_pos += pred_temp_w;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = _mm_set1_epi32(1 << (1 + MvPrecNormShift));
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));
                //down
                current_pos += pred_temp_w;
                pred_temp_sse = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i*)(pred_temp + current_pos)), vzero);
                coef_sse = dmv_y_sse;
                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_mullo_epi32(pred_temp_sse, coef_sse));

                pred_sp_sse = _mm_add_epi32(pred_sp_sse, _mm_set1_epi32(1 << MvPrecNormShift));
                pred_sp_sse = _mm_srai_epi32(pred_sp_sse, (MvPrecNormShift + 1));
            }
            else
            {
                assert(0);
            }

            pred_sp_sse = _mm_min_epi32(pred_sp_sse, mm_max);
            pred_sp_sse = _mm_max_epi32(pred_sp_sse, mm_min);

            int pred_pos = xx + yy * cu_width;
            pred_sp_sse = _mm_packs_epi32(pred_sp_sse, vzero);
            _mm_storel_epi64((__m128i*)(pred + pred_pos), pred_sp_sse);
        }
    }
}

#else

void com_sec_pred(int dmv_buf_x[DMV_BUF_SIZE], int dmv_buf_y[DMV_BUF_SIZE], pel* pred, pel* pred_temp, int cu_width, int sub_w, int sub_h, BOOL dMvH_on, BOOL dMvV_on, int bit_depth)
{
    int MvPrecNormShift = 5;

    int pred_temp_w = sub_w + 2;
    s32 pred_secpred;
    for (int yy = 0; yy < sub_h; yy++) 
    {
        for (int xx = 0; xx < sub_w; xx++) 
        {
            int dmv_x = dmv_buf_x[xx + sub_w * yy];
            int dmv_y = dmv_buf_y[xx + sub_w * yy];

            int pred_temp_xx = xx + 1;
            int pred_temp_yy = yy + 1;

            if (dMvH_on && dMvV_on)
            {
                pred_secpred = (
                    /*current   */
                    ((s32)pred_temp[pred_temp_xx + pred_temp_yy * pred_temp_w]
                    << (MvPrecNormShift + 1 + ASP_SHIFT)) +
                    /*up        */
                    (s32)pred_temp[pred_temp_xx + (pred_temp_yy - 1) * pred_temp_w]
                    * ((-dmv_y) << ASP_SHIFT) +
                    /*down      */
                    (s32)pred_temp[pred_temp_xx + (pred_temp_yy + 1) * pred_temp_w]
                    * (dmv_y << ASP_SHIFT) +
                    /*left      */
                    (s32)pred_temp[(pred_temp_xx - 1) + pred_temp_yy * pred_temp_w]
                    * ((-dmv_x) << ASP_SHIFT) +
                    /*right     */
                    (s32)pred_temp[(pred_temp_xx + 1) + pred_temp_yy * pred_temp_w]
                    * (dmv_x << ASP_SHIFT) +
                    /*left up   */
                    (s32)pred_temp[(pred_temp_xx - 1) + (pred_temp_yy - 1) * pred_temp_w]
                    * (-dmv_x - dmv_y) +
                    /*right up  */
                    (s32)pred_temp[(pred_temp_xx + 1) + (pred_temp_yy - 1) * pred_temp_w]
                    * (dmv_x - dmv_y) +
                    /*left down */
                    (s32)pred_temp[(pred_temp_xx - 1) + (pred_temp_yy + 1) * pred_temp_w]
                    * (-dmv_x + dmv_y) +
                    /*right down*/
                    (s32)pred_temp[(pred_temp_xx + 1) + (pred_temp_yy + 1) * pred_temp_w]
                    * (dmv_x + dmv_y) +
                    /*counterweight*/
                    (1 << (MvPrecNormShift + ASP_SHIFT))
                    ) >> (MvPrecNormShift + 1 + ASP_SHIFT);
            }
            else if (dMvH_on)
            {
                pred_secpred = (
                    /*current   */
                    ((s32)pred_temp[pred_temp_xx + pred_temp_yy * pred_temp_w] << (MvPrecNormShift + 1)) +
                    /*left      */
                    (s32)pred_temp[(pred_temp_xx - 1) + pred_temp_yy * pred_temp_w] * (-dmv_x) +
                    /*right     */
                    (s32)pred_temp[(pred_temp_xx + 1) + pred_temp_yy * pred_temp_w] * dmv_x +
                    /*counterweight*/
                    (1 << MvPrecNormShift)) >> (MvPrecNormShift + 1);
            }
            else if (dMvV_on)
            {
                pred_secpred = (
                    /*current   */
                    ((s32)pred_temp[pred_temp_xx + pred_temp_yy * pred_temp_w] << (MvPrecNormShift + 1)) +
                    /*up        */
                    (s32)pred_temp[pred_temp_xx + (pred_temp_yy - 1) * pred_temp_w] * (-dmv_y) +
                    /*down      */
                    (s32)pred_temp[pred_temp_xx + (pred_temp_yy + 1) * pred_temp_w] * dmv_y +
                    /*counterweight*/
                    (1 << MvPrecNormShift)) >> (MvPrecNormShift + 1);
            }
            else
            {
                assert(0);
            }

            pred_secpred = COM_CLIP3(0, (1 << bit_depth) - 1, pred_secpred);
            pred[xx + yy * cu_width] = (pel)pred_secpred;
        }
    }
}
#endif // SIMD_ASP
#endif // ASP


#if ASP
BOOL asp_deltMV_calc(int dmv_hor_x, int dmv_hor_y, int dmv_ver_x, int dmv_ver_y, int dMvBuf[128], int dMvBufLT[128], int dMvBufTR[128], int dMvBufLB[128], int sub_w, int sub_h, BOOL* dMvH_on, BOOL* dMvV_on)
{
    int* dMvH = dMvBuf;
    int* dMvV = dMvBuf + DMV_BUF_SIZE;
    int* dMvHLT = dMvBufLT;
    int* dMvVLT = dMvBufLT + DMV_BUF_SIZE;
    int* dMvHTR = dMvBufTR;
    int* dMvVTR = dMvBufTR + DMV_BUF_SIZE;
    int* dMvHLB = dMvBufLB;
    int* dMvVLB = dMvBufLB + DMV_BUF_SIZE;

    int quadHorX = dmv_hor_x << 2;
    int quadHorY = dmv_hor_y << 2;
    int quadVerX = dmv_ver_x << 2;
    int quadVerY = dmv_ver_y << 2;

    dMvH[0] = ((dmv_hor_x + dmv_ver_x) << 1) - ((quadHorX + quadVerX) << (sub_w >> 2));
    dMvV[0] = ((dmv_hor_y + dmv_ver_y) << 1) - ((quadHorY + quadVerY) << (sub_w >> 2));
    dMvHLT[0] = 0;
    dMvVLT[0] = 0;
    dMvHTR[0] = -quadHorX * (sub_w - 1);
    dMvVTR[0] = -quadHorY * (sub_w - 1);
    dMvHLB[0] = -quadVerX * (sub_h - 1);
    dMvVLB[0] = -quadVerY * (sub_h - 1);

    for (int w = 1; w < sub_w; w++)
    {
        dMvH[w] = dMvH[w - 1] + quadHorX;
        dMvV[w] = dMvV[w - 1] + quadHorY;
        dMvHLT[w] = dMvHLT[w - 1] + quadHorX;
        dMvVLT[w] = dMvVLT[w - 1] + quadHorY;
        dMvHTR[w] = dMvHTR[w - 1] + quadHorX;
        dMvVTR[w] = dMvVTR[w - 1] + quadHorY;
        dMvHLB[w] = dMvHLB[w - 1] + quadHorX;
        dMvVLB[w] = dMvVLB[w - 1] + quadHorY;
    }

    dMvH += sub_w;
    dMvV += sub_w;
    dMvHLT += sub_w;
    dMvVLT += sub_w;
    dMvHTR += sub_w;
    dMvVTR += sub_w;
    dMvHLB += sub_w;
    dMvVLB += sub_w;

#if SIMD_ASP
    if(sub_w == 4)
    {
        __m128i delta_ver_x = _mm_set1_epi32(quadVerX);
        __m128i delta_ver_y = _mm_set1_epi32(quadVerY);
        __m128i mm_dMvH_line0 = _mm_loadu_si128((__m128i*)(dMvH - sub_w));
        __m128i mm_dMvV_line0 = _mm_loadu_si128((__m128i*)(dMvV - sub_w));
        __m128i mm_dMvHLT_line0 = _mm_loadu_si128((__m128i*)(dMvHLT - sub_w));
        __m128i mm_dMvVLT_line0 = _mm_loadu_si128((__m128i*)(dMvVLT - sub_w));
        __m128i mm_dMvHTR_line0 = _mm_loadu_si128((__m128i*)(dMvHTR - sub_w));
        __m128i mm_dMvVTR_line0 = _mm_loadu_si128((__m128i*)(dMvVTR - sub_w));
        __m128i mm_dMvHLB_line0 = _mm_loadu_si128((__m128i*)(dMvHLB - sub_w));
        __m128i mm_dMvVLB_line0 = _mm_loadu_si128((__m128i*)(dMvVLB - sub_w));

        for (int h = 1; h < sub_h; h++, dMvH += sub_w, dMvV += sub_w, dMvHLT += sub_w, dMvVLT += sub_w, dMvHTR += sub_w, dMvVTR += sub_w, dMvHLB += sub_w, dMvVLB += sub_w)
        {
            __m128i mm_dMvH_line1 = _mm_add_epi32(mm_dMvH_line0, delta_ver_x);
            __m128i mm_dMvV_line1 = _mm_add_epi32(mm_dMvV_line0, delta_ver_y);
            _mm_storeu_si128((__m128i*)dMvH, mm_dMvH_line1);
            _mm_storeu_si128((__m128i*)dMvV, mm_dMvV_line1);
            mm_dMvH_line0 = mm_dMvH_line1;
            mm_dMvV_line0 = mm_dMvV_line1;

            __m128i mm_dMvHLT_line1 = _mm_add_epi32(mm_dMvHLT_line0, delta_ver_x);
            __m128i mm_dMvVLT_line1 = _mm_add_epi32(mm_dMvVLT_line0, delta_ver_y);
            _mm_storeu_si128((__m128i*)dMvHLT, mm_dMvHLT_line1);
            _mm_storeu_si128((__m128i*)dMvVLT, mm_dMvVLT_line1);
            mm_dMvHLT_line0 = mm_dMvHLT_line1;
            mm_dMvVLT_line0 = mm_dMvVLT_line1;

            __m128i mm_dMvHTR_line1 = _mm_add_epi32(mm_dMvHTR_line0, delta_ver_x);
            __m128i mm_dMvVTR_line1 = _mm_add_epi32(mm_dMvVTR_line0, delta_ver_y);
            _mm_storeu_si128((__m128i*)dMvHTR, mm_dMvHTR_line1);
            _mm_storeu_si128((__m128i*)dMvVTR, mm_dMvVTR_line1);
            mm_dMvHTR_line0 = mm_dMvHTR_line1;
            mm_dMvVTR_line0 = mm_dMvVTR_line1;

            __m128i mm_dMvHLB_line1 = _mm_add_epi32(mm_dMvHLB_line0, delta_ver_x);
            __m128i mm_dMvVLB_line1 = _mm_add_epi32(mm_dMvVLB_line0, delta_ver_y);
            _mm_storeu_si128((__m128i*)dMvHLB, mm_dMvHLB_line1);
            _mm_storeu_si128((__m128i*)dMvVLB, mm_dMvVLB_line1);
            mm_dMvHLB_line0 = mm_dMvHLB_line1;
            mm_dMvVLB_line0 = mm_dMvVLB_line1;
        }
    }
    else
    {
        assert(sub_w == 8);
        __m128i delta_ver_x = _mm_set1_epi32(quadVerX);
        __m128i delta_ver_y = _mm_set1_epi32(quadVerY);
        __m128i mm_dMvH_line0[2];
        __m128i mm_dMvV_line0[2];
        __m128i mm_dMvHLT_line0[2];
        __m128i mm_dMvVLT_line0[2];
        __m128i mm_dMvHTR_line0[2];
        __m128i mm_dMvVTR_line0[2];
        __m128i mm_dMvHLB_line0[2];
        __m128i mm_dMvVLB_line0[2];
        mm_dMvH_line0[0] = _mm_loadu_si128((__m128i*)(dMvH - 8));     
        mm_dMvH_line0[1] = _mm_loadu_si128((__m128i*)(dMvH - 4));
        mm_dMvV_line0[0] = _mm_loadu_si128((__m128i*)(dMvV - 8));     
        mm_dMvV_line0[1] = _mm_loadu_si128((__m128i*)(dMvV - 4));
        mm_dMvHLT_line0[0] = _mm_loadu_si128((__m128i*)(dMvHLT - 8)); 
        mm_dMvHLT_line0[1] = _mm_loadu_si128((__m128i*)(dMvHLT - 4));
        mm_dMvVLT_line0[0] = _mm_loadu_si128((__m128i*)(dMvVLT - 8)); 
        mm_dMvVLT_line0[1] = _mm_loadu_si128((__m128i*)(dMvVLT - 4));
        mm_dMvHTR_line0[0] = _mm_loadu_si128((__m128i*)(dMvHTR - 8)); 
        mm_dMvHTR_line0[1] = _mm_loadu_si128((__m128i*)(dMvHTR - 4));
        mm_dMvVTR_line0[0] = _mm_loadu_si128((__m128i*)(dMvVTR - 8)); 
        mm_dMvVTR_line0[1] = _mm_loadu_si128((__m128i*)(dMvVTR - 4));
        mm_dMvHLB_line0[0] = _mm_loadu_si128((__m128i*)(dMvHLB - 8)); 
        mm_dMvHLB_line0[1] = _mm_loadu_si128((__m128i*)(dMvHLB - 4));
        mm_dMvVLB_line0[0] = _mm_loadu_si128((__m128i*)(dMvVLB - 8)); 
        mm_dMvVLB_line0[1] = _mm_loadu_si128((__m128i*)(dMvVLB - 4));


        for (int h = 1; h < sub_h; h++, dMvH += sub_w, dMvV += sub_w, dMvHLT += sub_w, dMvVLT += sub_w, dMvHTR += sub_w, dMvVTR += sub_w, dMvHLB += sub_w, dMvVLB += sub_w)
        {
            for (int w = 0; w < 2; w++) 
            {
                __m128i mm_dMvH_line1 = _mm_add_epi32(mm_dMvH_line0[w], delta_ver_x);
                __m128i mm_dMvV_line1 = _mm_add_epi32(mm_dMvV_line0[w], delta_ver_y);
                _mm_storeu_si128((__m128i*)(dMvH + 4 * w), mm_dMvH_line1);
                _mm_storeu_si128((__m128i*)(dMvV + 4 * w), mm_dMvV_line1);
                mm_dMvH_line0[w] = mm_dMvH_line1;
                mm_dMvV_line0[w] = mm_dMvV_line1;

                __m128i mm_dMvHLT_line1 = _mm_add_epi32(mm_dMvHLT_line0[w], delta_ver_x);
                __m128i mm_dMvVLT_line1 = _mm_add_epi32(mm_dMvVLT_line0[w], delta_ver_y);
                _mm_storeu_si128((__m128i*)(dMvHLT + 4 * w), mm_dMvHLT_line1);
                _mm_storeu_si128((__m128i*)(dMvVLT + 4 * w), mm_dMvVLT_line1);
                mm_dMvHLT_line0[w] = mm_dMvHLT_line1;
                mm_dMvVLT_line0[w] = mm_dMvVLT_line1;

                __m128i mm_dMvHTR_line1 = _mm_add_epi32(mm_dMvHTR_line0[w], delta_ver_x);
                __m128i mm_dMvVTR_line1 = _mm_add_epi32(mm_dMvVTR_line0[w], delta_ver_y);
                _mm_storeu_si128((__m128i*)(dMvHTR + 4 * w), mm_dMvHTR_line1);
                _mm_storeu_si128((__m128i*)(dMvVTR + 4 * w), mm_dMvVTR_line1);
                mm_dMvHTR_line0[w] = mm_dMvHTR_line1;
                mm_dMvVTR_line0[w] = mm_dMvVTR_line1;

                __m128i mm_dMvHLB_line1 = _mm_add_epi32(mm_dMvHLB_line0[w], delta_ver_x);
                __m128i mm_dMvVLB_line1 = _mm_add_epi32(mm_dMvVLB_line0[w], delta_ver_y);
                _mm_storeu_si128((__m128i*)(dMvHLB + 4 * w), mm_dMvHLB_line1);
                _mm_storeu_si128((__m128i*)(dMvVLB + 4 * w), mm_dMvVLB_line1);
                mm_dMvHLB_line0[w] = mm_dMvHLB_line1;
                mm_dMvVLB_line0[w] = mm_dMvVLB_line1;
            }
        }
    }
    
#else
    for (int h = 1; h < sub_h; h++)
    {
        for (int w = 0; w < sub_w; w++)
        {
            dMvH[w] = dMvH[w - sub_w] + quadVerX;
            dMvV[w] = dMvV[w - sub_w] + quadVerY;
            dMvHLT[w] = dMvHLT[w - sub_w] + quadVerX;
            dMvVLT[w] = dMvVLT[w - sub_w] + quadVerY;
            dMvHTR[w] = dMvHTR[w - sub_w] + quadVerX;
            dMvVTR[w] = dMvVTR[w - sub_w] + quadVerY;
            dMvHLB[w] = dMvHLB[w - sub_w] + quadVerX;
            dMvVLB[w] = dMvVLB[w - sub_w] + quadVerY;
        }
        dMvH += sub_w;
        dMvV += sub_w;
        dMvHLT += sub_w;
        dMvVLT += sub_w;
        dMvHTR += sub_w;
        dMvVTR += sub_w;
        dMvHLB += sub_w;
        dMvVLB += sub_w;
    }
#endif

    dMvH = dMvBuf;
    dMvV = dMvBuf + DMV_BUF_SIZE;
    dMvHLT = dMvBufLT;
    dMvVLT = dMvBufLT + DMV_BUF_SIZE;
    dMvHTR = dMvBufTR;
    dMvVTR = dMvBufTR + DMV_BUF_SIZE;
    dMvHLB = dMvBufLB;
    dMvVLB = dMvBufLB + DMV_BUF_SIZE;

    BOOL enable_asp = TRUE;    
    const int dMVThresh = ((1 << 5) * 10);
    int dMvH_max = max(abs(dMvH[sub_w * (sub_h - 1)]), max(abs(dMvH[sub_w - 1]), max(abs(dMvH[0]), abs(dMvH[sub_w * sub_h - 1]))));
    int dMvV_max = max(abs(dMvV[sub_w * (sub_h - 1)]), max(abs(dMvV[sub_w - 1]), max(abs(dMvV[0]), abs(dMvV[sub_w * sub_h - 1]))));
    *dMvH_on = (dMvH_max >= dMVThresh);
    *dMvV_on = (dMvV_max >= dMVThresh);
    if (!(*dMvH_on) && !(*dMvV_on))
    {
        enable_asp = FALSE;
    }

    if (enable_asp)
    {
        const int mvShift = 8;
        const int dmvLimit = (1 << 5) - 1;
#if SIMD_ASP
        asp_mv_rounding(dMvH,   dMvV,   dMvH,   dMvV,   sub_w * sub_h, mvShift, dmvLimit, *dMvH_on, *dMvV_on);
        asp_mv_rounding(dMvHLT, dMvVLT, dMvHLT, dMvVLT, sub_w * sub_h, mvShift, dmvLimit, *dMvH_on, *dMvV_on);
        asp_mv_rounding(dMvHTR, dMvVTR, dMvHTR, dMvVTR, sub_w * sub_h, mvShift, dmvLimit, *dMvH_on, *dMvV_on);
        asp_mv_rounding(dMvHLB, dMvVLB, dMvHLB, dMvVLB, sub_w * sub_h, mvShift, dmvLimit, *dMvH_on, *dMvV_on);
#else
        for (int idx = 0; idx < sub_w * sub_h; idx++)
        {
            com_mv_rounding_s32(dMvH[idx], dMvV[idx], &dMvH[idx], &dMvV[idx], mvShift, 0);
            dMvH[idx] = min(dmvLimit, max(-dmvLimit, dMvH[idx]));
            dMvV[idx] = min(dmvLimit, max(-dmvLimit, dMvV[idx]));
            com_mv_rounding_s32(dMvHLT[idx], dMvVLT[idx], &dMvHLT[idx], &dMvVLT[idx], mvShift, 0);
            dMvHLT[idx] = min(dmvLimit, max(-dmvLimit, dMvHLT[idx]));
            dMvVLT[idx] = min(dmvLimit, max(-dmvLimit, dMvVLT[idx]));
            com_mv_rounding_s32(dMvHTR[idx], dMvVTR[idx], &dMvHTR[idx], &dMvVTR[idx], mvShift, 0);
            dMvHTR[idx] = min(dmvLimit, max(-dmvLimit, dMvHTR[idx]));
            dMvVTR[idx] = min(dmvLimit, max(-dmvLimit, dMvVTR[idx]));
            com_mv_rounding_s32(dMvHLB[idx], dMvVLB[idx], &dMvHLB[idx], &dMvVLB[idx], mvShift, 0);
            dMvHLB[idx] = min(dmvLimit, max(-dmvLimit, dMvHLB[idx]));
            dMvVLB[idx] = min(dmvLimit, max(-dmvLimit, dMvVLB[idx]));
        }
#endif
    }

    return enable_asp;
}

void com_affine_mc_l(COM_INFO* info, int x, int y, int pic_w, int pic_h, int cu_width, int cu_height, CPMV ac_mv[VER_NUM][MV_D], COM_PIC* ref_pic, pel pred[MAX_CU_DIM], int cp_num, int sub_w, int sub_h, int bit_depth)
#else
void com_affine_mc_l(int x, int y, int pic_w, int pic_h, int cu_width, int cu_height, CPMV ac_mv[VER_NUM][MV_D], COM_PIC* ref_pic, pel pred[MAX_CU_DIM], int cp_num, int sub_w, int sub_h, int bit_depth)
#endif
{
    assert(com_tbl_log2[cu_width] >= 4);
    assert(com_tbl_log2[cu_height] >= 4);
    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred;
    int w, h;
    int half_w, half_h;
    s32 dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
    s32 mv_scale_hor = (s32)ac_mv[0][MV_X] << 7;
    s32 mv_scale_ver = (s32)ac_mv[0][MV_Y] << 7;
    s32 mv_scale_tmp_hor, mv_scale_tmp_ver;
    s32 hor_max, hor_min, ver_max, ver_min;
    s32 mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori;

#if CPMV_BIT_DEPTH == 18
    for (int i = 0; i < cp_num; i++)
    {
        assert(ac_mv[i][MV_X] >= COM_CPMV_MIN && ac_mv[i][MV_X] <= COM_CPMV_MAX);
        assert(ac_mv[i][MV_Y] >= COM_CPMV_MIN && ac_mv[i][MV_Y] <= COM_CPMV_MAX);
    }
#endif

    // get clip MV Range
#if CTU_256
    hor_max = (pic_w + MAX_CU_SIZE2 + 4 - x - cu_width + 1) << 4;
    ver_max = (pic_h + MAX_CU_SIZE2 + 4 - y - cu_height + 1) << 4;
    hor_min = (-MAX_CU_SIZE2 - 4 - x) << 4;
    ver_min = (-MAX_CU_SIZE2 - 4 - y) << 4;
#else
    hor_max = (pic_w + MAX_CU_SIZE + 4 - x - cu_width + 1) << 4;
    ver_max = (pic_h + MAX_CU_SIZE + 4 - y - cu_height + 1) << 4;
    hor_min = (-MAX_CU_SIZE - 4 - x) << 4;
    ver_min = (-MAX_CU_SIZE - 4 - y) << 4;
#endif

#if ENC_ME_IMP
    if ((cp_num == 3 && (ac_mv[0][MV_X] == ac_mv[1][MV_X] && ac_mv[0][MV_Y] == ac_mv[1][MV_Y]) && (ac_mv[0][MV_X] == ac_mv[2][MV_X] && ac_mv[0][MV_Y] == ac_mv[2][MV_Y])) || \
        (cp_num == 2 && (ac_mv[0][MV_X] == ac_mv[1][MV_X] && ac_mv[0][MV_Y] == ac_mv[1][MV_Y])))
    {
        mv_scale_tmp_hor_ori = ac_mv[0][MV_X];
        mv_scale_tmp_ver_ori = ac_mv[0][MV_Y];
        mv_scale_tmp_hor = min(hor_max, max(hor_min, ac_mv[0][MV_X]));
        mv_scale_tmp_ver = min(ver_max, max(ver_min, ac_mv[0][MV_Y]));

        qpel_gmv_x = (x << 4) + mv_scale_tmp_hor;
        qpel_gmv_y = (y << 4) + mv_scale_tmp_ver;
        com_mc_l_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, cu_width, pred_y, cu_width, cu_height, bit_depth);

        return;
    }
#endif
    // get sub block size
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = (((s32)ac_mv[1][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_width];      // deltaMvHor
    dmv_hor_y = (((s32)ac_mv[1][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_width];
    if (cp_num == 3)
    {
        dmv_ver_x = (((s32)ac_mv[2][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_height]; // deltaMvVer
        dmv_ver_y = (((s32)ac_mv[2][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_height];
    }
    else
    {
        dmv_ver_x = -dmv_hor_y;                                                                    // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

#if ASP
    BOOL enable_asp = info->sqh.asp_enable_flag;
    enable_asp &= !info->skip_me_asp;
    enable_asp &= !((cp_num == 3 && ac_mv[0] == ac_mv[1] && ac_mv[0] == ac_mv[2]) || (cp_num == 2 && ac_mv[0] == ac_mv[1]));
    int    dMvBuf[DMV_BUF_SIZE * 2];
    int    dMvBufLT[DMV_BUF_SIZE * 2];
    int    dMvBufTR[DMV_BUF_SIZE * 2];
    int    dMvBufLB[DMV_BUF_SIZE * 2];

    pel pred_temp[100];
    int pred_temp_w = sub_w + 2;
    int pred_temp_h = sub_h + 2;

    int* dMvBufHor = dMvBuf;
    int* dMvBufVer = dMvBuf + DMV_BUF_SIZE;

    BOOL   dMvH_on = TRUE;
    BOOL   dMvV_on = TRUE;

    if (enable_asp)
    {
        enable_asp = asp_deltMV_calc(dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, dMvBuf, dMvBufLT, dMvBufTR, dMvBufLB, sub_w, sub_h, &dMvH_on, &dMvV_on);
    }
#endif

    // get prediction block by block
    for (h = 0; h < cu_height; h += sub_h)
    {
        for(w = 0; w < cu_width; w += sub_w)
        {
            int pos_x = w + half_w;
            int pos_y = h + half_h;

#if ASP
            dMvBufHor = dMvBuf;
            dMvBufVer = dMvBuf + DMV_BUF_SIZE;
#endif //ASP
            if (w == 0 && h == 0)
            {
                pos_x = 0;
                pos_y = 0;
#if ASP
                dMvBufHor = dMvBufLT;
                dMvBufVer = dMvBufLT + DMV_BUF_SIZE;
#endif //ASP
            }
            else if (w + sub_w == cu_width && h == 0)
            {
                pos_x = cu_width;
                pos_y = 0;
#if ASP
                dMvBufHor = dMvBufTR;
                dMvBufVer = dMvBufTR + DMV_BUF_SIZE;
#endif //ASP
            }
            else if (w == 0 && h + sub_h == cu_height && cp_num == 3)
            {
                pos_x = 0;
                pos_y = cu_height;
#if ASP
                dMvBufHor = dMvBufLB;
                dMvBufVer = dMvBufLB + DMV_BUF_SIZE;
#endif //ASP
            }

            mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
            mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

            // 1/16 precision, 18 bits, for MC
#if BD_AFFINE_AMVR
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);
#else
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#endif
            mv_scale_tmp_hor = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_hor);
            mv_scale_tmp_ver = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_ver);

            // clip
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));
            qpel_gmv_x = ((x + w) << 4) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << 4) + mv_scale_tmp_ver;
#if ASP
            if (enable_asp) 
            {
                pel* padding_start_pel = ref_pic->y + ((qpel_gmv_x - 9) >> 4) + ((qpel_gmv_y - 9) >> 4) * ref_pic->stride_luma;
                memcpy(pred_temp, padding_start_pel, sizeof(pel)* pred_temp_w);
                memcpy(pred_temp + (sub_h + 1) * pred_temp_w, padding_start_pel + (sub_h + 1) * ref_pic->stride_luma, sizeof(pel)* pred_temp_w);
                for (int padding_row = 0; padding_row < sub_h; padding_row++) 
                {
                    pred_temp[(padding_row + 1) * pred_temp_w] = padding_start_pel[(padding_row + 1) * ref_pic->stride_luma];
                    pred_temp[pred_temp_w - 1 + (padding_row + 1) * pred_temp_w] = padding_start_pel[pred_temp_w - 1 + (padding_row + 1) * ref_pic->stride_luma];
                }
                com_mc_l_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_temp_w, pred_temp + 1 + pred_temp_w, sub_w, sub_h, bit_depth);
                com_sec_pred(dMvBufHor, dMvBufVer, (pred_y + w), pred_temp, cu_width, sub_w, sub_h, dMvH_on, dMvV_on, bit_depth);
            }
            else {
#endif //ASP
                com_mc_l_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, cu_width, (pred_y + w), sub_w, sub_h, bit_depth);
#if ASP
            }
#endif // ASP
        }
        pred_y += (cu_width * sub_h);
    }
}

void com_affine_mc_lc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, pel pred[N_C][MAX_CU_DIM], int sub_w, int sub_h, int lidx, int bit_depth)

{
    int scup = mod_info_curr->scup;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int cu_width = mod_info_curr->cu_width;
    int cu_height = mod_info_curr->cu_height;
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;
    int cp_num = mod_info_curr->affine_flag + 1;
    s8 *refi = mod_info_curr->refi;
    COM_PIC* ref_pic = ref_pic = refp[refi[lidx]][lidx].pic;
    CPMV(*ac_mv)[MV_D] = mod_info_curr->affine_mv[lidx];
    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;

    assert(com_tbl_log2[cu_width] >= 4);
    assert(com_tbl_log2[cu_height] >= 4);

    int qpel_gmv_x, qpel_gmv_y;
    pel *pred_y = pred[Y_C], *pred_u = pred[U_C], *pred_v = pred[V_C];
    int w, h;
    int half_w, half_h;
    int dmv_hor_x, dmv_ver_x, dmv_hor_y, dmv_ver_y;
    s32 mv_scale_hor = (s32)ac_mv[0][MV_X] << 7;
    s32 mv_scale_ver = (s32)ac_mv[0][MV_Y] << 7;
    s32 mv_scale_tmp_hor, mv_scale_tmp_ver;
    s32 hor_max, hor_min, ver_max, ver_min;
    s32 mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori;
    s32 mv_save[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2][MV_D];

#if CPMV_BIT_DEPTH == 18
    for (int i = 0; i < cp_num; i++)//判断各个mv是否在范围内
    {
        assert(ac_mv[i][MV_X] >= COM_CPMV_MIN && ac_mv[i][MV_X] <= COM_CPMV_MAX);
        assert(ac_mv[i][MV_Y] >= COM_CPMV_MIN && ac_mv[i][MV_Y] <= COM_CPMV_MAX);
    }
#endif

    // get clip MV Range
#if CTU_256
    hor_max = (pic_w + MAX_CU_SIZE2 + 4 - x - cu_width + 1) << 4;
    ver_max = (pic_h + MAX_CU_SIZE2 + 4 - y - cu_height + 1) << 4;
    hor_min = (-MAX_CU_SIZE2 - 4 - x) << 4;
    ver_min = (-MAX_CU_SIZE2 - 4 - y) << 4;
#else
    hor_max = (pic_w + MAX_CU_SIZE + 4 - x - cu_width + 1) << 4;
    ver_max = (pic_h + MAX_CU_SIZE + 4 - y - cu_height + 1) << 4;
    hor_min = (-MAX_CU_SIZE - 4 - x) << 4;
    ver_min = (-MAX_CU_SIZE - 4 - y) << 4;
#endif

#if ENC_ME_IMP
    if ((cp_num == 3 && (ac_mv[0][MV_X] == ac_mv[1][MV_X] && ac_mv[0][MV_Y] == ac_mv[1][MV_Y]) && (ac_mv[0][MV_X] == ac_mv[2][MV_X] && ac_mv[0][MV_Y] == ac_mv[2][MV_Y])) || \
        (cp_num == 2 && (ac_mv[0][MV_X] == ac_mv[1][MV_X] && ac_mv[0][MV_Y] == ac_mv[1][MV_Y])))
    {
        mv_scale_tmp_hor_ori = ac_mv[0][MV_X];
        mv_scale_tmp_ver_ori = ac_mv[0][MV_Y];
        mv_scale_tmp_hor = min(hor_max, max(hor_min, ac_mv[0][MV_X]));
        mv_scale_tmp_ver = min(ver_max, max(ver_min, ac_mv[0][MV_Y]));

        qpel_gmv_x = (x << 4) + mv_scale_tmp_hor;
        qpel_gmv_y = (y << 4) + mv_scale_tmp_ver;
        com_mc_l_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, cu_width, pred_y, cu_width, cu_height, bit_depth);
        com_mc_c_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, cu_width >> 1, pred_u, cu_width >> 1, cu_height >> 1, bit_depth);
        com_mc_c_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, cu_width >> 1, pred_v, cu_width >> 1, cu_height >> 1, bit_depth);

        return;
    }
#endif

    // get sub block size
    half_w = sub_w >> 1;
    half_h = sub_h >> 1;

    // convert to 2^(storeBit + bit) precision
    dmv_hor_x = (((s32)ac_mv[1][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_width];      // deltaMvHor
    dmv_hor_y = (((s32)ac_mv[1][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_width];
    if (cp_num == 3)//六参数
    {
        dmv_ver_x = (((s32)ac_mv[2][MV_X] - (s32)ac_mv[0][MV_X]) << 7) >> com_tbl_log2[cu_height]; // deltaMvVer
        dmv_ver_y = (((s32)ac_mv[2][MV_Y] - (s32)ac_mv[0][MV_Y]) << 7) >> com_tbl_log2[cu_height];
    }
    else//四参数
    {
        dmv_ver_x = -dmv_hor_y;                                                                    // deltaMvVer
        dmv_ver_y = dmv_hor_x;
    }

#if ASP
    BOOL enable_asp = info->sqh.asp_enable_flag;
    enable_asp &= !info->skip_umve_asp;
    enable_asp &= !((cp_num == 3 && ac_mv[0] == ac_mv[1] && ac_mv[0] == ac_mv[2]) || (cp_num == 2 && ac_mv[0] == ac_mv[1]));
    int    dMvBuf[DMV_BUF_SIZE * 2];
    int    dMvBufLT[DMV_BUF_SIZE * 2];
    int    dMvBufTR[DMV_BUF_SIZE * 2];
    int    dMvBufLB[DMV_BUF_SIZE * 2];

    pel pred_temp[100];
    int pred_temp_w = sub_w + 2;
    int pred_temp_h = sub_h + 2;

    int* dMvBufHor = dMvBuf;
    int* dMvBufVer = dMvBuf + DMV_BUF_SIZE;

    BOOL   dMvH_on = TRUE;
    BOOL   dMvV_on = TRUE;

    if (enable_asp)
    {
        enable_asp = asp_deltMV_calc(dmv_hor_x, dmv_hor_y, dmv_ver_x, dmv_ver_y, dMvBuf, dMvBufLT, dMvBufTR, dMvBufLB, sub_w, sub_h, &dMvH_on, &dMvV_on);
    }
#endif

    // get prediction block by block (luma)
    for (h = 0; h < cu_height; h += sub_h)//计算各个亮度子块的MV
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            int pos_x = w + half_w;
            int pos_y = h + half_h;

#if ASP
            dMvBufHor = dMvBuf;
            dMvBufVer = dMvBuf + DMV_BUF_SIZE;
#endif //ASP
            if (w == 0 && h == 0)
            {
                pos_x = 0;
                pos_y = 0;
#if ASP
                dMvBufHor = dMvBufLT;
                dMvBufVer = dMvBufLT + DMV_BUF_SIZE;
#endif //ASP
            }
            else if (w + sub_w == cu_width && h == 0)
            {
                pos_x = cu_width;
                pos_y = 0;
#if ASP
                dMvBufHor = dMvBufTR;
                dMvBufVer = dMvBufTR + DMV_BUF_SIZE;
#endif //ASP
            }
            else if (w == 0 && h + sub_h == cu_height && cp_num == 3)
            {
                pos_x = 0;
                pos_y = cu_height;
#if ASP
                dMvBufHor = dMvBufLB;
                dMvBufVer = dMvBufLB + DMV_BUF_SIZE;
#endif //ASP
            }

            mv_scale_tmp_hor = mv_scale_hor + dmv_hor_x * pos_x + dmv_ver_x * pos_y;
            mv_scale_tmp_ver = mv_scale_ver + dmv_hor_y * pos_x + dmv_ver_y * pos_y;

            // 1/16 precision, 18 bits, for MC
#if BD_AFFINE_AMVR
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 7, 0);//变为1/16精度
#else
            com_mv_rounding_s32(mv_scale_tmp_hor, mv_scale_tmp_ver, &mv_scale_tmp_hor, &mv_scale_tmp_ver, 5, 0);
#endif
            //保证运动矢量在范围内
            mv_scale_tmp_hor = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_hor);
            mv_scale_tmp_ver = COM_CLIP3(COM_INT18_MIN, COM_INT18_MAX, mv_scale_tmp_ver);

#if AFFINE_MVF_VERIFY
            {
                int addr_scu = scup + (w >> MIN_CU_LOG2) + (h >> MIN_CU_LOG2) * pic_width_in_scu;
                s16 rounded_hor, rounded_ver;
                com_mv_rounding_s16(mv_scale_tmp_hor, mv_scale_tmp_ver, &rounded_hor, &rounded_ver, 2, 0); // 1/4 precision
                assert(rounded_hor == map_mv[addr_scu][lidx][MV_X]);
                assert(rounded_ver == map_mv[addr_scu][lidx][MV_Y]);
            }
#endif

            // save MVF for chroma interpolation
            int w_scu = PEL2SCU(w);
            int h_scu = PEL2SCU(h);
            mv_save[w_scu][h_scu][MV_X] = mv_scale_tmp_hor;
            mv_save[w_scu][h_scu][MV_Y] = mv_scale_tmp_ver;
            if (sub_w == 8 && sub_h == 8)
            {
                mv_save[w_scu + 1][h_scu][MV_X] = mv_scale_tmp_hor;
                mv_save[w_scu + 1][h_scu][MV_Y] = mv_scale_tmp_ver;
                mv_save[w_scu][h_scu + 1][MV_X] = mv_scale_tmp_hor;
                mv_save[w_scu][h_scu + 1][MV_Y] = mv_scale_tmp_ver;
                mv_save[w_scu + 1][h_scu + 1][MV_X] = mv_scale_tmp_hor;
                mv_save[w_scu + 1][h_scu + 1][MV_Y] = mv_scale_tmp_ver;
            }

            // clip
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));
            qpel_gmv_x = ((x + w) << 4) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << 4) + mv_scale_tmp_ver;
#if ASP
            if(enable_asp)
            {
                pel* padding_start_pel = ref_pic->y + ((qpel_gmv_x - 9) >> 4) + ((qpel_gmv_y - 9) >> 4)* ref_pic->stride_luma;
                memcpy(pred_temp, padding_start_pel, sizeof(pel) * pred_temp_w);
                memcpy(pred_temp + (sub_h+1)* pred_temp_w, padding_start_pel + (sub_h + 1)* ref_pic->stride_luma, sizeof(pel) * pred_temp_w);
                for (int padding_row = 0; padding_row < sub_h; padding_row++) 
                {
                    pred_temp[(padding_row + 1) * pred_temp_w] = padding_start_pel[(padding_row + 1) * ref_pic->stride_luma];
                    pred_temp[pred_temp_w -1 + (padding_row + 1) * pred_temp_w] = padding_start_pel[pred_temp_w - 1 + (padding_row + 1) * ref_pic->stride_luma];
                }
                com_mc_l_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, pred_temp_w, pred_temp + 1 + pred_temp_w, sub_w, sub_h, bit_depth);
                com_sec_pred(dMvBufHor, dMvBufVer, (pred_y + w), pred_temp, cu_width, sub_w, sub_h, dMvH_on, dMvV_on, bit_depth);
            }
            else {
#endif //ASP
                com_mc_l_hp(mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, cu_width, (pred_y + w), sub_w, sub_h, bit_depth);
#if ASP
            }
#endif // ASP
        }
        pred_y += (cu_width * sub_h);
    }

#if ASP
    if (!info->skip_umve_asp)
    {
#endif
    // get prediction block by block (chroma)
    sub_w = 8;
    sub_h = 8;
    for (h = 0; h < cu_height; h += sub_h)
    {
        for (w = 0; w < cu_width; w += sub_w)
        {
            int w_scu = PEL2SCU(w);
            int h_scu = PEL2SCU(h);

            mv_scale_tmp_hor = (mv_save[w_scu][h_scu][MV_X] + mv_save[w_scu + 1][h_scu][MV_X] + mv_save[w_scu][h_scu + 1][MV_X] + mv_save[w_scu + 1][h_scu + 1][MV_X] + 2) >> 2;
            mv_scale_tmp_ver = (mv_save[w_scu][h_scu][MV_Y] + mv_save[w_scu + 1][h_scu][MV_Y] + mv_save[w_scu][h_scu + 1][MV_Y] + mv_save[w_scu + 1][h_scu + 1][MV_Y] + 2) >> 2;
            mv_scale_tmp_hor_ori = mv_scale_tmp_hor;
            mv_scale_tmp_ver_ori = mv_scale_tmp_ver;
            mv_scale_tmp_hor = min(hor_max, max(hor_min, mv_scale_tmp_hor));
            mv_scale_tmp_ver = min(ver_max, max(ver_min, mv_scale_tmp_ver));
            qpel_gmv_x = ((x + w) << 4) + mv_scale_tmp_hor;
            qpel_gmv_y = ((y + h) << 4) + mv_scale_tmp_ver;
            com_mc_c_hp( mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->u, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, cu_width >> 1, pred_u + (w >> 1), sub_w >> 1, sub_h >> 1, bit_depth );
            com_mc_c_hp( mv_scale_tmp_hor_ori, mv_scale_tmp_ver_ori, ref_pic->v, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_chroma, cu_width >> 1, pred_v + (w >> 1), sub_w >> 1, sub_h >> 1, bit_depth );
        }
        pred_u += (cu_width * sub_h) >> 2;
        pred_v += (cu_width * sub_h) >> 2;
    }
#if ASP
    }
#endif
}

void com_affine_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], COM_MAP *pic_map, int bit_depth
#if AFFINE_DMVR
    , COM_DMVR* dmvr
#endif
)
{
    int scup = mod_info_curr->scup;
    int x = mod_info_curr->x_pos;
    int y = mod_info_curr->y_pos;
    int w = mod_info_curr->cu_width; 
    int h = mod_info_curr->cu_height;
    
    int pic_w = info->pic_width;
    int pic_h = info->pic_height;
    int pic_width_in_scu = info->pic_width_in_scu;
    COM_PIC_HEADER * sh = &info->pic_header;
    
    int vertex_num = mod_info_curr->affine_flag + 1;
    s8 *refi = mod_info_curr->refi; 
    CPMV (*mv)[VER_NUM][MV_D] = mod_info_curr->affine_mv;
    pel (*pred_buf)[MAX_CU_DIM] = mod_info_curr->pred;

#if BGC
    static pel pred_uv[V_C][MAX_CU_DIM];
    pel *dst_u, *dst_v;
    u8  bgc_flag = mod_info_curr->bgc_flag;
    u8  bgc_idx = mod_info_curr->bgc_idx;
    pel *pred_fir = info->pred_tmp;
#endif 

    s16(*map_mv)[REFP_NUM][MV_D] = pic_map->map_mv;

    static pel pred_snd[N_C][MAX_CU_DIM];
    pel(*pred)[MAX_CU_DIM] = pred_buf;

    int bidx = 0;
    int sub_w = 4;
    int sub_h = 4;
    if (sh->affine_subblock_size_idx == 1)
    {
        sub_w = 8;
        sub_h = 8;
    }
    if (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]))
    {
        sub_w = 8;
        sub_h = 8;
    }
#if AFFINE_DMVR
    int poc0 = refp[refi[REFP_0]][REFP_0].ptr;
    int poc1 = refp[refi[REFP_1]][REFP_1].ptr;
    BOOL dmvr_poc_condition = ((BOOL)((dmvr->poc_c - poc0) * (dmvr->poc_c - poc1) < 0)) && (abs(dmvr->poc_c - poc0) == abs(dmvr->poc_c - poc1));//前后参考帧与当前帧时域上间隔一样
    //dmvr->apply_DMVR = dmvr->apply_DMVR && x > 2 && y > 2;
    dmvr->apply_DMVR = dmvr->apply_DMVR && dmvr_poc_condition;
    dmvr->apply_DMVR = dmvr->apply_DMVR && (REFI_IS_VALID(refi[REFP_0]) && REFI_IS_VALID(refi[REFP_1]));
#if AWP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->awp_flag);
#endif
#if ETMVP
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->etmvp_flag);
#endif
#if IPC 
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->ipc_flag);
#endif
#if INTER_TM
    dmvr->apply_DMVR = dmvr->apply_DMVR && (!mod_info_curr->tm_flag);
#endif
    if (dmvr->apply_DMVR)
    {
        process_AFFINEDMVR(x, y, pic_w, pic_h, w, h, refi, mv, refp, dmvr->dmvr_padding_buf);
    }
#endif
    if(REFI_IS_VALID(refi[REFP_0]))
    {
        /* forward */
        com_affine_mc_lc(info, mod_info_curr, refp, pic_map, pred, sub_w, sub_h, REFP_0, bit_depth);
        bidx++;
#if BGC
        if (bgc_flag)
        {
            pel *p0, *dest;
            p0 = pred_buf[Y_C];
            dest = pred_fir;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    dest[i] = p0[i];
                }
                p0 += w;
                dest += w;
            }

            pel* p1;
            p0 = pred_buf[U_C];
            p1 = pred_buf[V_C];
            dst_u = pred_uv[0];
            dst_v = pred_uv[1];
            for (int j = 0; j < (h >> 1); j++)
            {
                for (int i = 0; i < (w >> 1); i++)
                {
                    dst_u[i] = p0[i];
                    dst_v[i] = p1[i];
                }
                p0 += (w >> 1);
                p1 += (w >> 1);
                dst_u += (w >> 1);
                dst_v += (w >> 1);
            }

        }
#endif
    }

    if(REFI_IS_VALID(refi[REFP_1]))
    {
        /* backward */
        if (bidx)
        {
            pred = pred_snd;
        }
        com_affine_mc_lc(info, mod_info_curr, refp, pic_map, pred, sub_w, sub_h, REFP_1, bit_depth);
        bidx++;
    }

    if(bidx == 2)
    {
#if SIMD_MC
        average_16b_no_clip_sse( pred_buf[Y_C], pred_snd[Y_C], pred_buf[Y_C], w, w, w, w, h );
        average_16b_no_clip_sse( pred_buf[U_C], pred_snd[U_C], pred_buf[U_C], w >> 1, w >> 1, w >> 1, w >> 1, h >> 1 );
        average_16b_no_clip_sse( pred_buf[V_C], pred_snd[V_C], pred_buf[V_C], w >> 1, w >> 1, w >> 1, w >> 1, h >> 1 );
#else
        pel *p0, *p1, *p2, *p3;

        p0 = pred_buf[Y_C];
        p1 = pred_snd[Y_C];
        for(int j = 0; j < h; j++)
        {
            for(int i = 0; i < w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
            }
            p0 += w;
            p1 += w;
        }
        p0 = pred_buf[U_C];
        p1 = pred_snd[U_C];
        p2 = pred_buf[V_C];
        p3 = pred_snd[V_C];
        int half_w = w >> 1;
        int half_h = h >> 1;

        for(int j = 0; j < half_h; j++)
        {
            for(int i = 0; i < half_w; i++)
            {
                p0[i] = (p0[i] + p1[i] + 1) >> 1;
                p2[i] = (p2[i] + p3[i] + 1) >> 1;
            }
            p0 += half_w;
            p1 += half_w;
            p2 += half_w;
            p3 += half_w;
        }
#endif
#if BGC
        if (bgc_flag && info->sqh.bgc_enable_flag)
        {
            pel *p0, *p1, *dest;
            dest = pred_buf[Y_C];
            p0 = pred_fir;
            p1 = pred_snd[Y_C];
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    if (bgc_idx)
                    {
                        dest[i] += (p0[i] - p1[i]) >> 3;
                    }
                    else
                    {
                        dest[i] += (p1[i] - p0[i]) >> 3;
                    }
                    dest[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dest[i]);
                }
                p0 += w;
                p1 += w;
                dest += w;
            }

            pel *p2, *p3;
            p0 = pred_uv[0];
            p1 = pred_snd[U_C];
            p2 = pred_uv[1];
            p3 = pred_snd[V_C];
            dst_u = pred_buf[U_C];
            dst_v = pred_buf[V_C];
            w >>= 1;
            h >>= 1;
            for (int j = 0; j < h; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    if (bgc_idx)
                    {
                        dst_u[i] += (p0[i] - p1[i]) >> 3;
                        dst_v[i] += (p2[i] - p3[i]) >> 3;
                    }
                    else
                    {
                        dst_u[i] += (p1[i] - p0[i]) >> 3;
                        dst_v[i] += (p3[i] - p2[i]) >> 3;
                    }
                    dst_u[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dst_u[i]);
                    dst_v[i] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, dst_v[i]);
                }
                p0 += w;
                p1 += w;
                p2 += w;
                p3 += w;
                dst_u += w;
                dst_v += w;
            }
        }
#endif
    }
}

#if AWP_ENH
void com_dawp_mc(COM_INFO* info, COM_MODE* mod_info_curr, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth, COM_PIC* pic, u32* map_scu, pel**** awp_weight_tpl)
{
    s32 cu_width = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;
    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    static pel pred_awp_tmp0[N_C][MAX_CU_DIM];
    static pel pred_awp_tmp1[N_C][MAX_CU_DIM];
    static pel pred_tpl0[1][AWP_TPL_SIZE * 2];
    static pel pred_tpl1[1][AWP_TPL_SIZE * 2];
    static pel awp_weight0[N_C][MAX_AWP_DIM];
    static pel awp_weight1[N_C][MAX_AWP_DIM];

#if DMVR
    /* disable DMVR*/
    COM_DMVR dmvr;
    dmvr.poc_c = 0;
    dmvr.apply_DMVR = 0;
#endif

    init_awp_tpl(mod_info_curr, awp_weight_tpl);
    com_get_tpl_cur(pic, map_scu, info->pic_width_in_scu, info->pic_height_in_scu, mod_info_curr);

    if (mod_info_curr->tpl_cur_avail[0] == 1 && mod_info_curr->tpl_cur_avail[1] == 1)
    {
        s16 tpl_mv_offset[2][MV_D] = { 0 };
        tpl_mv_offset[0][MV_X] = 0;
        tpl_mv_offset[0][MV_Y] = -(1 << 2);
        tpl_mv_offset[1][MV_X] = -(1 << 2);
        tpl_mv_offset[1][MV_Y] = 0;
        int dir;
        const int tpl_size = 1;
        int tpl_height[2], tpl_width[2];
        tpl_width[0] = cu_width;
        tpl_height[0] = tpl_size;
        tpl_width[1] = tpl_size;
        tpl_height[1] = cu_height;
        int tpl_stride = 1;
        int offset[2] = { 0, AWP_TPL_SIZE };
        for (dir = 0; dir < 2; dir++)
        {
            mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X] + tpl_mv_offset[dir][MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y] + tpl_mv_offset[dir][MV_Y];
            mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X] + tpl_mv_offset[dir][MV_X];
            mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y] + tpl_mv_offset[dir][MV_Y];
            mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi0[REFP_0];
            mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi0[REFP_1];
            com_mc(x, y, (tpl_width[dir] == 1 ? 4 : tpl_width[dir]), (tpl_height[dir] == 1 ? 4 : tpl_height[dir]), cu_width, pred_awp_tmp0, info, mod_info_curr, refp, CHANNEL_L, bit_depth
#if DMVR
                , &dmvr
#endif
#if BIO
                , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
                , 0
#endif
#if SUB_TMVP
                , 0
#endif
#if BGC
                , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
            );
            for (int i = 0; i < tpl_height[dir]; i++)
            {
                for (int j = 0; j < tpl_width[dir]; j++)
                {
                    pred_tpl0[0][i * tpl_stride + j + offset[dir]] = pred_awp_tmp0[Y_C][i * cu_width + j];
                }
            }

        }
    }

    /* get two pred buf */
    mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
    mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
    mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
    mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
    mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi0[REFP_0];
    mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi0[REFP_1];

    com_mc(x, y, cu_width, cu_height, cu_width, pred_awp_tmp0, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
        , &dmvr
#endif
#if BIO
        , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
        , 0
#endif
#if SUB_TMVP
        , 0
#endif
#if BGC
        , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
    );

    if (mod_info_curr->tpl_cur_avail[0] == 1 && mod_info_curr->tpl_cur_avail[1] == 1)
    {
        s16 tpl_mv_offset[2][MV_D] = { 0 };
        tpl_mv_offset[0][MV_X] = 0;
        tpl_mv_offset[0][MV_Y] = -(1 << 2);
        tpl_mv_offset[1][MV_X] = -(1 << 2);
        tpl_mv_offset[1][MV_Y] = 0;
        int dir;
        const int tpl_size = 1;
        int tpl_height[2], tpl_width[2];
        tpl_width[0] = cu_width;
        tpl_height[0] = tpl_size;
        tpl_width[1] = tpl_size;
        tpl_height[1] = cu_height;
        int tpl_stride = 1;
        int offset[2] = { 0, AWP_TPL_SIZE };
        for (dir = 0; dir < 2; dir++)
        {
            mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X] + tpl_mv_offset[dir][MV_X];
            mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y] + tpl_mv_offset[dir][MV_Y];
            mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X] + tpl_mv_offset[dir][MV_X];
            mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y] + tpl_mv_offset[dir][MV_Y];
            mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi1[REFP_0];
            mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi1[REFP_1];
            com_mc(x, y, (tpl_width[dir] == 1 ? 4 : tpl_width[dir]), (tpl_height[dir] == 1 ? 4 : tpl_height[dir]), cu_width, pred_awp_tmp1, info, mod_info_curr, refp, CHANNEL_L, bit_depth
#if DMVR
                , & dmvr
#endif
#if BIO
                , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
                , 0
#endif
#if SUB_TMVP
                , 0
#endif
#if BGC
                , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
            );

            for (int i = 0; i < tpl_height[dir]; i++)
            {
                for (int j = 0; j < tpl_width[dir]; j++)
                {
                    pred_tpl1[0][i * tpl_stride + j + offset[dir]] = pred_awp_tmp1[Y_C][i * cu_width + j];
                }
            }

        }
    }

    mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
    mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
    mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
    mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
    mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi1[REFP_0];
    mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi1[REFP_1];

    com_mc(x, y, cu_width, cu_height, cu_width, pred_awp_tmp1, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
        , &dmvr
#endif
#if BIO
        , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
        , 0
#endif
#if SUB_TMVP
        , 0
#endif
#if BGC
        , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
    );

    mod_info_curr->mv[REFP_0][MV_X] = 0;
    mod_info_curr->mv[REFP_0][MV_Y] = 0;
    mod_info_curr->mv[REFP_1][MV_X] = 0;
    mod_info_curr->mv[REFP_1][MV_Y] = 0;
    mod_info_curr->refi[REFP_0] = REFI_INVALID;
    mod_info_curr->refi[REFP_1] = REFI_INVALID;

    static int mode_list[AWP_MODE_NUM];
    static int inv_mode_list[AWP_MODE_NUM];
    com_get_tpl_ref(mod_info_curr, pred_tpl0[0], pred_tpl1[0]);
    com_tpl_reorder_awp_mode(mod_info_curr, mode_list, inv_mode_list);
    int dawp_idx = mod_info_curr->dawp_idx;
    int awp_idx = mode_list[dawp_idx];
    mod_info_curr->skip_idx = awp_idx;

    if (tree_status == CHANNEL_LC)
    {
        /* derive weights */
#if BAWP
#if SAWP_SCC == 0
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P, 0);
#else
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P);
#endif
#else
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1);
#endif
        /* combine two pred buf */
        com_derive_awp_pred(mod_info_curr, Y_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[Y_C], awp_weight1[Y_C]);
        com_derive_awp_pred(mod_info_curr, U_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[U_C], awp_weight1[U_C]);
        com_derive_awp_pred(mod_info_curr, V_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[V_C], awp_weight1[V_C]);
    }
    else
    {
        /* derive weights */
#if BAWP
#if SAWP_SCC == 0
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P, 0);
#else
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P);
#endif
#else
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1);
#endif
        /* combine two pred buf */
        com_derive_awp_pred(mod_info_curr, Y_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[Y_C], awp_weight1[Y_C]);
    }
}
#endif

#if AWP
void com_awp_mc(COM_INFO *info, COM_MODE *mod_info_curr, COM_REFP(*refp)[REFP_NUM], u8 tree_status, int bit_depth)
{
    s32 cu_width  = mod_info_curr->cu_width;
    s32 cu_height = mod_info_curr->cu_height;
    s32 x = mod_info_curr->x_pos;
    s32 y = mod_info_curr->y_pos;

    s32 h = 0;
    s32 w = 0;
    s32 tmp_x = x;
    s32 tmp_y = y;
    static pel pred_awp_tmp0[N_C][MAX_CU_DIM];
    static pel pred_awp_tmp1[N_C][MAX_CU_DIM];
    static pel awp_weight0[N_C][MAX_AWP_DIM];
    static pel awp_weight1[N_C][MAX_AWP_DIM];

#if DMVR
    /* disable DMVR*/
    COM_DMVR dmvr;
    dmvr.poc_c = 0;
    dmvr.apply_DMVR = 0;
#endif

    /* get two pred buf */
    mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv0[REFP_0][MV_X];
    mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv0[REFP_0][MV_Y];
    mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv0[REFP_1][MV_X];
    mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv0[REFP_1][MV_Y];
    mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi0[REFP_0];
    mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi0[REFP_1];

    com_mc(x, y, cu_width, cu_height, cu_width, pred_awp_tmp0, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
        , &dmvr
#endif
#if BIO
        , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
        , 0
#endif
#if SUB_TMVP
        , 0
#endif
#if BGC
        , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
    );

    mod_info_curr->mv[REFP_0][MV_X] = mod_info_curr->awp_mv1[REFP_0][MV_X];
    mod_info_curr->mv[REFP_0][MV_Y] = mod_info_curr->awp_mv1[REFP_0][MV_Y];
    mod_info_curr->mv[REFP_1][MV_X] = mod_info_curr->awp_mv1[REFP_1][MV_X];
    mod_info_curr->mv[REFP_1][MV_Y] = mod_info_curr->awp_mv1[REFP_1][MV_Y];
    mod_info_curr->refi[REFP_0] = mod_info_curr->awp_refi1[REFP_0];
    mod_info_curr->refi[REFP_1] = mod_info_curr->awp_refi1[REFP_1];

    com_mc(x, y, cu_width, cu_height, cu_width, pred_awp_tmp1, info, mod_info_curr, refp, tree_status, bit_depth
#if DMVR
        , &dmvr
#endif
#if BIO
        , -1, 0, mod_info_curr->mvr_idx
#endif
#if MVAP
        , 0
#endif
#if SUB_TMVP
        , 0
#endif
#if BGC
        , mod_info_curr->bgc_flag, mod_info_curr->bgc_idx
#endif
    );

    mod_info_curr->mv[REFP_0][MV_X] = 0;
    mod_info_curr->mv[REFP_0][MV_Y] = 0;
    mod_info_curr->mv[REFP_1][MV_X] = 0;
    mod_info_curr->mv[REFP_1][MV_Y] = 0;
    mod_info_curr->refi[REFP_0] = REFI_INVALID;
    mod_info_curr->refi[REFP_1] = REFI_INVALID;

    if (tree_status == CHANNEL_LC)
    {
        /* derive weights */
#if BAWP
#if SAWP_SCC == 0
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P, 0);
#else
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P);
#endif
#else
        com_derive_awp_weight(mod_info_curr, N_C, awp_weight0, awp_weight1);
#endif

        /* combine two pred buf */
        com_derive_awp_pred(mod_info_curr, Y_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[Y_C], awp_weight1[Y_C]);
        com_derive_awp_pred(mod_info_curr, U_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[U_C], awp_weight1[U_C]);
        com_derive_awp_pred(mod_info_curr, V_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[V_C], awp_weight1[V_C]);
    }
    else // only for luma
    {
        /* derive weights */
#if BAWP
#if SAWP_SCC == 0
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P, 0);
#else
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1, info->pic_header.slice_type == SLICE_P);
#endif
#else
        com_derive_awp_weight(mod_info_curr, Y_C, awp_weight0, awp_weight1);
#endif

        /* combine two pred buf */
        com_derive_awp_pred(mod_info_curr, Y_C, pred_awp_tmp0, pred_awp_tmp1, awp_weight0[Y_C], awp_weight1[Y_C]);
    }
}
#endif

#if INTER_TM
static int sad_16b(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    s16* s1;
    s16* s2;
    int i, j, sad;
    s1 = (s16*)src1;
    s2 = (s16*)src2;
    sad = 0;
    for (i = 0; i < h; i++)
    {
        for (j = 0; j < w; j++)
        {
            sad += COM_ABS16((s16)s1[j] - (s16)s2[j]);
        }
        s1 += s_src1;
        s2 += s_src2;
    }
    return (sad >> (bit_depth - 8));
}

#if SIMD_SAD
#define SSE_SAD_16B_8PEL_2(src1, src2, s00, s01, sac0, sac1) \
    s00 = _mm_loadu_si128((__m128i*)(src1)); \
    s01 = _mm_loadu_si128((__m128i*)(src2)); \
    s00 = _mm_sub_epi16(s00, s01); \
    s01 = _mm_abs_epi16(s00); \
    \
    s00 = _mm_srli_si128(s01, 8); \
    s00 = _mm_cvtepi16_epi32(s00); \
    s01 = _mm_cvtepi16_epi32(s01); \
    \
    sac0 = _mm_add_epi32(sac0, s00); \
    sac1 = _mm_add_epi32(sac1, s01);

static int sad_16b_4x2_simd(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16* s1;
    s16* s2;
    __m128i s00, s01, sac0;
    s1 = (s16*)src1;
    s2 = (s16*)src2;
    sac0 = _mm_setzero_si128();
    SSE_SAD_16B_4PEL(s1, s2, s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0);
    sad = _mm_extract_epi32(sac0, 0);
    sad += _mm_extract_epi32(sac0, 1);
    sad += _mm_extract_epi32(sac0, 2);
    sad += _mm_extract_epi32(sac0, 3);
    return (sad >> (bit_depth - 8));
}

static int sad_16b_4x2n_simd(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16* s1;
    s16* s2;
    __m128i s00, s01, sac0;
    int i;
    s1 = (s16*)src1;
    s2 = (s16*)src2;
    sac0 = _mm_setzero_si128();
    for (i = 0; i < h >> 1; i++)
    {
        SSE_SAD_16B_4PEL(s1, s2, s00, s01, sac0);
        SSE_SAD_16B_4PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0);
        s1 += s_src1 << 1;
        s2 += s_src2 << 1;
    }
    sad = _mm_extract_epi32(sac0, 0);
    sad += _mm_extract_epi32(sac0, 1);
    sad += _mm_extract_epi32(sac0, 2);
    sad += _mm_extract_epi32(sac0, 3);
    return (sad >> (bit_depth - 8));
}

static int sad_16b_4x4_simd(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16* s1;
    s16* s2;
    __m128i s00, s01, sac0;
    s1 = (s16*)src1;
    s2 = (s16*)src2;
    sac0 = _mm_setzero_si128();
    SSE_SAD_16B_4PEL(s1, s2, s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + s_src1, s2 + s_src2, s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + (s_src1 * 2), s2 + (s_src2 * 2), s00, s01, sac0);
    SSE_SAD_16B_4PEL(s1 + (s_src1 * 3), s2 + (s_src2 * 3), s00, s01, sac0);
    sad = _mm_extract_epi32(sac0, 0);
    sad += _mm_extract_epi32(sac0, 1);
    sad += _mm_extract_epi32(sac0, 2);
    sad += _mm_extract_epi32(sac0, 3);
    return (sad >> (bit_depth - 8));
}

static int sad_16b_sse_16x2(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    int sad;
    s16* s1;
    s16* s2;
    __m128i s00, s01, sac0, sac1;
    s1 = (s16*)src1;
    s2 = (s16*)src2;
    sac0 = _mm_setzero_si128();
    sac1 = _mm_setzero_si128();
    SSE_SAD_16B_8PEL_2(s1, s2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL_2(s1 + 8, s2 + 8, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL_2(s1 + s_src1, s2 + s_src2, s00, s01, sac0, sac1);
    SSE_SAD_16B_8PEL_2(s1 + s_src1 + 8, s2 + s_src2 + 8, s00, s01, sac0, sac1);
    s00 = _mm_add_epi32(sac0, sac1);
    sad = _mm_extract_epi32(s00, 0);
    sad += _mm_extract_epi32(s00, 1);
    sad += _mm_extract_epi32(s00, 2);
    sad += _mm_extract_epi32(s00, 3);
    return (sad >> (bit_depth - 8));
}

static int sad_16b_sse_8x2(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    __m128i src_8x16b;
    __m128i src_8x16b_1;
    __m128i pred_8x16b;
    __m128i pred_8x16b_1;
    __m128i temp;
    __m128i temp_1;
    __m128i temp_3;
    __m128i temp_dummy;
    __m128i result;
    short* pu2_inp;
    short* pu2_ref;
    int sad = 0;
    assert(bit_depth <= 14);
    assert(w == 8); /* fun usage expects w ==8, but assumption is width has to be multiple of 8 */
    assert(h == 2); /* fun usage expects h ==2, but assumption is height has to be multiple of 2 */
    pu2_inp = src1;
    pu2_ref = src2;
    temp_dummy = _mm_setzero_si128();
    result = _mm_setzero_si128();
    {
        src_8x16b = _mm_loadu_si128((__m128i*) (pu2_inp));
        src_8x16b_1 = _mm_loadu_si128((__m128i*) (pu2_inp + s_src1));
        pred_8x16b = _mm_loadu_si128((__m128i*) (pu2_ref));
        pred_8x16b_1 = _mm_loadu_si128((__m128i*) (pu2_ref + s_src2));
        temp = _mm_sub_epi16(src_8x16b, pred_8x16b);
        temp_1 = _mm_sub_epi16(src_8x16b_1, pred_8x16b_1);
        temp = _mm_abs_epi16(temp);
        temp_1 = _mm_abs_epi16(temp_1);
        temp = _mm_adds_epu16(temp, temp_1);
        temp_1 = _mm_unpackhi_epi16(temp, temp_dummy);
        temp_3 = _mm_unpacklo_epi16(temp, temp_dummy);
        temp = _mm_add_epi32(temp_1, temp_3);
        result = _mm_add_epi32(result, temp);
    }
    {
        int* val = (int*)&result;
        sad = val[0] + val[1] + val[2] + val[3];
    }
    return (sad >> (bit_depth - 8));
}

static int sad_16b_sse_8x4n(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    __m128i src_8x16b;
    __m128i src_8x16b_1;
    __m128i src_8x16b_2;
    __m128i src_8x16b_3;
    __m128i pred_8x16b;
    __m128i pred_8x16b_1;
    __m128i pred_8x16b_2;
    __m128i pred_8x16b_3;
    __m128i temp;
    __m128i temp_1;
    __m128i temp_2;
    __m128i temp_3;
    __m128i temp_4;
    __m128i temp_5;
    __m128i temp_dummy;
    __m128i result, result_1;
    short* pu2_inp;
    short* pu2_ref;
    int  i;
    int sad = 0;
    assert(bit_depth <= 14);
    assert(w == 8); /* fun usage expects w ==8, but assumption is width has to be multiple of 8 */
    assert(!(h & 3)); /* height has to be multiple of 4 */
    pu2_inp = src1;
    pu2_ref = src2;
    temp_dummy = _mm_setzero_si128();
    result = _mm_setzero_si128();
    result_1 = _mm_setzero_si128();
    {
        for (i = 0; i < h / 4; i++)
        {
            src_8x16b = _mm_loadu_si128((__m128i*) (pu2_inp));
            src_8x16b_1 = _mm_loadu_si128((__m128i*) (pu2_inp + s_src1));
            src_8x16b_2 = _mm_loadu_si128((__m128i*) (pu2_inp + (s_src1 * 2)));
            src_8x16b_3 = _mm_loadu_si128((__m128i*) (pu2_inp + (s_src1 * 3)));
            pred_8x16b = _mm_loadu_si128((__m128i*) (pu2_ref));
            pred_8x16b_1 = _mm_loadu_si128((__m128i*) (pu2_ref + s_src2));
            pred_8x16b_2 = _mm_loadu_si128((__m128i*) (pu2_ref + (s_src2 * 2)));
            pred_8x16b_3 = _mm_loadu_si128((__m128i*) (pu2_ref + (s_src2 * 3)));
            temp = _mm_sub_epi16(src_8x16b, pred_8x16b);
            temp_1 = _mm_sub_epi16(src_8x16b_1, pred_8x16b_1);
            temp_2 = _mm_sub_epi16(src_8x16b_2, pred_8x16b_2);
            temp_3 = _mm_sub_epi16(src_8x16b_3, pred_8x16b_3);
            temp = _mm_abs_epi16(temp);
            temp_1 = _mm_abs_epi16(temp_1);
            temp_2 = _mm_abs_epi16(temp_2);
            temp_3 = _mm_abs_epi16(temp_3);
            temp = _mm_add_epi16(temp, temp_1);
            temp_2 = _mm_add_epi16(temp_2, temp_3);
            temp_1 = _mm_unpackhi_epi16(temp, temp_dummy);
            temp_3 = _mm_unpacklo_epi16(temp, temp_dummy);
            temp_4 = _mm_unpackhi_epi16(temp_2, temp_dummy);
            temp_5 = _mm_unpacklo_epi16(temp_2, temp_dummy);
            temp = _mm_add_epi32(temp_1, temp_3);
            temp_2 = _mm_add_epi32(temp_4, temp_5);
            result = _mm_add_epi32(result, temp);
            result_1 = _mm_add_epi32(result_1, temp_2);
            pu2_inp += (4 * s_src1);
            pu2_ref += (4 * s_src2);
        }
        result = _mm_add_epi32(result, result_1);
    }
    {
        int* val = (int*)&result;
        sad = val[0] + val[1] + val[2] + val[3];
    }
    return (sad >> (bit_depth - 8));
}

static int sad_16b_sse_16nx4n(int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth)
{
    __m128i src_8x16b;
    __m128i src_8x16b_1;
    __m128i src_8x16b_2;
    __m128i src_8x16b_3;
    __m128i pred_8x16b;
    __m128i pred_8x16b_1;
    __m128i pred_8x16b_2;
    __m128i pred_8x16b_3;
    __m128i temp;
    __m128i temp_1;
    __m128i temp_2;
    __m128i temp_3;
    __m128i temp_4;
    __m128i temp_5;
    __m128i temp_dummy;
    __m128i result, result_1;
    short* pu2_inp;
    short* pu2_ref;
    int  i, j;
    int sad = 0;
    assert(bit_depth <= 14);
    assert(!(w & 15)); /*fun used only for multiple of 16, but internal assumption is only 8 */
    assert(!(h & 3)); /* height has to be multiple of 4 */
    pu2_inp = src1;
    pu2_ref = src2;
    temp_dummy = _mm_setzero_si128();
    result = _mm_setzero_si128();
    result_1 = _mm_setzero_si128();
    {
        for (i = 0; i < h / 4; i++)
        {
            int count = 0;
            for (j = w; j > 7; j -= 8)
            {
                src_8x16b = _mm_loadu_si128((__m128i*) (pu2_inp + count));
                src_8x16b_1 = _mm_loadu_si128((__m128i*) (pu2_inp + count + s_src1));
                src_8x16b_2 = _mm_loadu_si128((__m128i*) (pu2_inp + count + (s_src1 * 2)));
                src_8x16b_3 = _mm_loadu_si128((__m128i*) (pu2_inp + count + (s_src1 * 3)));
                pred_8x16b = _mm_loadu_si128((__m128i*) (pu2_ref + count));
                pred_8x16b_1 = _mm_loadu_si128((__m128i*) (pu2_ref + count + s_src2));
                pred_8x16b_2 = _mm_loadu_si128((__m128i*) (pu2_ref + count + (s_src2 * 2)));
                pred_8x16b_3 = _mm_loadu_si128((__m128i*) (pu2_ref + count + (s_src2 * 3)));
                temp = _mm_sub_epi16(src_8x16b, pred_8x16b);
                temp_1 = _mm_sub_epi16(src_8x16b_1, pred_8x16b_1);
                temp_2 = _mm_sub_epi16(src_8x16b_2, pred_8x16b_2);
                temp_3 = _mm_sub_epi16(src_8x16b_3, pred_8x16b_3);
                temp = _mm_abs_epi16(temp);
                temp_1 = _mm_abs_epi16(temp_1);
                temp_2 = _mm_abs_epi16(temp_2);
                temp_3 = _mm_abs_epi16(temp_3);
                temp = _mm_add_epi16(temp, temp_1);
                temp_2 = _mm_add_epi16(temp_2, temp_3);
                temp_1 = _mm_unpackhi_epi16(temp, temp_dummy);
                temp_3 = _mm_unpacklo_epi16(temp, temp_dummy);
                temp_4 = _mm_unpackhi_epi16(temp_2, temp_dummy);
                temp_5 = _mm_unpacklo_epi16(temp_2, temp_dummy);
                temp = _mm_add_epi32(temp_1, temp_3);
                temp_2 = _mm_add_epi32(temp_4, temp_5);
                result = _mm_add_epi32(result, temp);
                result_1 = _mm_add_epi32(result_1, temp_2);
                count += 8;
            }
            pu2_inp += (4 * s_src1);
            pu2_ref += (4 * s_src2);
        }
        result = _mm_add_epi32(result, result_1);
    }
    {
        int* val = (int*)&result;
        sad = val[0] + val[1] + val[2] + val[3];
    }
    return (sad >> (bit_depth - 8));
}
#endif /* SIMD_SAD */

typedef int  (*COM_FN_SAD) (int w, int h, void* src1, void* src2, int s_src1, int s_src2, int bit_depth);
#define com_sad_16b(log2w, log2h, src1, src2, s_src1, s_src2, bit_depth)\
    com_tbl_sad_16b[log2w][log2h](1<<(log2w), 1<<(log2h), src1, src2, s_src1, s_src2, bit_depth)
const COM_FN_SAD com_tbl_sad_16b[9][9] =
{
#if SIMD_SAD
    /* width == 1 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
        sad_16b, /* height == 256 */
    },
    /* width == 2 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
        sad_16b, /* height == 256 */
    },
    /* width == 4 */
    {
        sad_16b, /* height == 1 */
        sad_16b_4x2_simd,  /* height == 2 */
        sad_16b_4x4_simd,  /* height == 4 */
        sad_16b_4x2n_simd, /* height == 8 */
        sad_16b_4x2n_simd, /* height == 16 */
        sad_16b_4x2n_simd, /* height == 32 */
        sad_16b_4x2n_simd, /* height == 64 */
        sad_16b_4x2n_simd, /* height == 128 */
        sad_16b_4x2n_simd, /* height == 256 */
    },
    /* width == 8 */
    {
        sad_16b, /* height == 1 */
        sad_16b_sse_8x2,  /* height == 2 */
        sad_16b_sse_8x4n,  /* height == 4 */
        sad_16b_sse_8x4n,  /* height == 8 */
        sad_16b_sse_8x4n, /* height == 16 */
        sad_16b_sse_8x4n, /* height == 32 */
        sad_16b_sse_8x4n, /* height == 64 */
        sad_16b_sse_8x4n, /* height == 128 */
        sad_16b_sse_8x4n, /* height == 256 */
    },
    /* width == 16 */
    {
        sad_16b,    /* height == 1 */
        sad_16b_sse_16x2,    /* height == 2 */
        sad_16b_sse_16nx4n,  /* height == 4 */
        sad_16b_sse_16nx4n,  /* height == 8 */
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
        sad_16b_sse_16nx4n,  /* height == 256 */
    },
    /* width == 32 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b_sse_16nx4n,  /* height == 4 */
        sad_16b_sse_16nx4n,  /* height == 8 */
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
        sad_16b_sse_16nx4n,  /* height == 256 */
    },
    /* width == 64 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b_sse_16nx4n,  /* height == 4 */
        sad_16b_sse_16nx4n,  /* height == 8 */
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
        sad_16b_sse_16nx4n,  /* height == 256 */
    },
    /* width == 128 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b,    /* height == 4 */
        sad_16b,    /* height == 8 */
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
        sad_16b_sse_16nx4n,  /* height == 256 */
    },
    /* width == 256 */
    {
        sad_16b,    /* height == 1 */
        sad_16b,    /* height == 2 */
        sad_16b,    /* height == 4 */
        sad_16b,    /* height == 8 */
        sad_16b_sse_16nx4n,  /* height == 16 */
        sad_16b_sse_16nx4n,  /* height == 32 */
        sad_16b_sse_16nx4n,  /* height == 64 */
        sad_16b_sse_16nx4n,  /* height == 128 */
        sad_16b_sse_16nx4n,  /* height == 256 */
    }
#else /* default */
    /* width == 1 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 2 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 4 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 8 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 16 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 32 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 64 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    },
    /* width == 128 */
    {
        sad_16b, /* height == 1 */
        sad_16b, /* height == 2 */
        sad_16b, /* height == 4 */
        sad_16b, /* height == 8 */
        sad_16b, /* height == 16 */
        sad_16b, /* height == 32 */
        sad_16b, /* height == 64 */
        sad_16b, /* height == 128 */
    }
#endif
};

u64 com_calc_tm_cost(int x, int y, u16 tm_size[3][2], COM_PIC* ref_pic, s16 init_mv[MV_D], s16 mv[MV_D], pel template_rec[3][MAX_CU_SIZE * TM_WIDTH], pel template_pred[3][MAX_CU_SIZE * TM_WIDTH], const int bit_depth, BOOL is_simplified)
{
    u64 tm_cost = 0;
    BOOL is_search = TRUE;

    for (u8 i = 0; i < 3; i++)
    {
        u16 tm_w = tm_size[i][0];
        u16 tm_h = tm_size[i][1];
        int x_offset = i == 0 ?         0 : (i == 1 ? -TM_WIDTH : -TM_WIDTH);
        int y_offset = i == 0 ? -TM_WIDTH : (i == 1 ?         0 : -TM_WIDTH);
        int qpel_gmv_x = ((x + x_offset) << 2) + mv[MV_X];
        int qpel_gmv_y = ((y + y_offset) << 2) + mv[MV_Y];
        int init_qpel_gmv_x = ((x + x_offset) << 2) + init_mv[MV_X];
        int init_qpel_gmv_y = ((y + y_offset) << 2) + init_mv[MV_Y];
        
        int dx = mv[MV_X] & 0x3 ? 1 : 0;
        int dy = mv[MV_Y] & 0x3 ? 1 : 0;

        if (is_simplified)
        {
            if (dx == 0 && dy == 0)
            {
                com_mc_tm_l_00(ref_pic->y, template_pred[i], ref_pic->stride_luma, tm_w, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
            }
            else if (dx == 1 && dy == 0)
            {
                com_mc_tm_l_n0(ref_pic->y, template_pred[i], ref_pic->stride_luma, tm_w, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
            }
            else if (dx == 0 && dy == 1)
            {
                com_mc_tm_l_0n(ref_pic->y, template_pred[i], ref_pic->stride_luma, tm_w, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
            }
            else if (dx == 1 && dy == 1)
            {
                com_mc_tm_l_nn(ref_pic->y, template_pred[i], ref_pic->stride_luma, tm_w, init_qpel_gmv_x, init_qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
            }
        }
        else
        {
            com_mc_l(mv[MV_X], mv[MV_Y], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, tm_w, template_pred[i], tm_w, tm_h, bit_depth);
        }

        tm_cost += com_sad_16b(com_tbl_log2[tm_w], com_tbl_log2[tm_h], template_rec[i], template_pred[i], tm_w, tm_w, bit_depth);
    }
    return tm_cost;
}

u64 com_tm_search(int x, int y, int pic_w, int pic_h, int w, int h, u16 tm_size[3][2], COM_PIC* ref_pic, s16 init_mv[MV_D], s16 mv[MV_D], 
    pel template_rec[3][MAX_CU_SIZE * TM_WIDTH], pel template_pred[3][MAX_CU_SIZE * TM_WIDTH], const int bit_depth, BOOL is_simplified)
{
    single_mv_clip(x-TM_WIDTH, y-TM_WIDTH, pic_w, pic_h, w+TM_WIDTH, h+TM_WIDTH, mv); 
    u64 best_cost = com_calc_tm_cost(x, y, tm_size, ref_pic, init_mv, mv, template_rec, template_pred, bit_depth, is_simplified);

    const static s8 search_point_hexagon[6][2] = {
        { 2,0},{ 1,-2},{-1,-2},
        {-2,0},{-1, 2},{ 1, 2}
    };

    const static s8 search_point_square[8][2] = {
        { 1,0},{ 1,-1},{0,-1},{-1,-1},
        {-1,0},{-1, 1},{0, 1},{ 1, 1}
    };

    const static u8 status_set_hexagon[6][6] = {
        {1, 1, 0, 0, 0, 1},
        {1, 1, 1, 0, 0, 0},
        {0, 1, 1, 1, 0, 0},
        {0, 0, 1, 1, 1, 0},
        {0, 0, 0, 1, 1, 1},
        {1, 0, 0, 0, 1, 1}
    };

    const static u8 status_set_square[8][8] = {
        {1, 1, 0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 0, 0, 0, 1},
        {0, 1, 1, 1, 0, 0, 0, 0},
        {0, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 1, 1, 1, 0, 0},
        {0, 0, 0, 1, 1, 1, 1, 1},
        {0, 0, 0, 0, 0, 1, 1, 1},
        {1, 1, 0, 0, 0, 1, 1, 1}
    };

    s16 best_mv[MV_D], start_mv[MV_D];
    u16 best_search_idx, better_mv_found;

    copy_mv(best_mv, mv);
    u64 cur_cost = best_cost;
    u8 status_hexagon[6] = {1, 1, 1, 1, 1, 1};
    const int max_cnt = is_simplified ? SIMPLIFIED_MAX_TM_ITERATIONS : MAX_TM_ITERATIONS;
    for (u16 cnt = 0; cnt < max_cnt; cnt++)
    {
        better_mv_found = 0;
        best_search_idx = 6;
        copy_mv(start_mv, best_mv);
        for (u16 search_idx = 0; search_idx < 6; search_idx++)
        {
            if (!status_hexagon[search_idx])
            {
                continue;
            }
            mv[MV_X] = start_mv[MV_X] + search_point_hexagon[search_idx][MV_X];
            mv[MV_Y] = start_mv[MV_Y] + search_point_hexagon[search_idx][MV_Y];
            single_mv_clip(x-TM_WIDTH, y-TM_WIDTH, pic_w, pic_h, w+TM_WIDTH, h+TM_WIDTH, mv);
            cur_cost = com_calc_tm_cost(x, y, tm_size, ref_pic, init_mv, mv, template_rec, template_pred, bit_depth, is_simplified);
            if (cur_cost < best_cost)
            {
                better_mv_found = 1;
                best_search_idx = search_idx;
                best_cost = cur_cost;
                copy_mv(best_mv, mv);
            }
        }
        if (better_mv_found)
        {
            memcpy(status_hexagon, status_set_hexagon[best_search_idx], sizeof(u8)*6);
        }
        else
        {
            break;
        }
    }

    u8 status_square[8] = { 1, 1, 1, 1, 1, 1, 1, 1};
    for (u16 cnt = 0; cnt < 1; cnt++)
    {
        better_mv_found = 0;
        best_search_idx = 8;
        copy_mv(start_mv, best_mv);
        for (u16 search_idx = 0; search_idx < 8; search_idx++)
        {
            if (!status_square[search_idx])
            {
                continue;
            }
            mv[MV_X] = start_mv[MV_X] + search_point_square[search_idx][MV_X];
            mv[MV_Y] = start_mv[MV_Y] + search_point_square[search_idx][MV_Y];

            single_mv_clip(x-TM_WIDTH, y-TM_WIDTH, pic_w, pic_h, w+TM_WIDTH, h+TM_WIDTH, mv);

            cur_cost = com_calc_tm_cost(x, y, tm_size, ref_pic, init_mv, mv, template_rec, template_pred, bit_depth, is_simplified);

            if (cur_cost < best_cost)
            {
                better_mv_found = 1;
                best_search_idx = search_idx;
                best_cost = cur_cost;
                copy_mv(best_mv, mv);
            }
        }
        if (better_mv_found)
        {
            memcpy(status_square, status_set_square[best_search_idx], sizeof(u8)*8);
        }
        else
        {
            break;
        }
    }
    copy_mv(mv, best_mv);
    return best_cost;
}

void com_tm_process(int x, int y, int pic_w, int pic_h, int w, int h, COM_REFP(*refp)[REFP_NUM], s8 refi[REFP_NUM], s16 init_mv[REFP_NUM][MV_D], s16 mv[REFP_NUM][MV_D], pel* reco_luma, const int stride_luma, const int bit_depth, BOOL is_simplified)
{
    assert(x>=TM_WIDTH && y>=TM_WIDTH);

    COM_PIC* ref_pic;
    static pel template_rec[3][MAX_CU_SIZE*TM_WIDTH];
    static pel template_pred[3][MAX_CU_SIZE*TM_WIDTH];
    u16 tm_size[3][2] = { {w, TM_WIDTH}, {TM_WIDTH, h}, {TM_WIDTH, TM_WIDTH}};

    for (u16 i = 0; i < TM_WIDTH; i++)
    {
        memcpy(template_rec[0] + i*w, reco_luma + (y-TM_WIDTH+i)*stride_luma + x, sizeof(pel)*w); // top template
    }

    for (u16 i = 0; i < h; i++)
    {
        memcpy(template_rec[1] + i*TM_WIDTH, reco_luma + (y+i)*stride_luma + (x-TM_WIDTH), sizeof(pel)*TM_WIDTH); // left template
    }

    for (u16 i = 0; i < TM_WIDTH; i++)
    {
        memcpy(template_rec[2] + i*TM_WIDTH, reco_luma + (y-TM_WIDTH+i)*stride_luma + (x-TM_WIDTH), sizeof(pel)*TM_WIDTH); // top_left template
    }

    u64 best_cost[REFP_NUM];
    for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
    {
        if (!REFI_IS_VALID(refi[refp_idx]))
        {
            continue;
        }
        ref_pic = refp[refi[refp_idx]][refp_idx].pic;
        best_cost[refp_idx] = com_tm_search(x, y, pic_w, pic_h, w, h, tm_size, ref_pic, 
            init_mv[refp_idx], mv[refp_idx], template_rec, template_pred, bit_depth, is_simplified);
    }
}

void remove_duplicate_cands(s16(*pmv_cands)[REFP_NUM][MV_D], s8(*refi_cands)[REFP_NUM], int* num_cands_all)
{
    assert(REFI_IS_VALID(refi_cands[0][REFP_0]) && REFI_IS_VALID(refi_cands[0][REFP_1]));
    assert(REFI_IS_VALID(refi_cands[1][REFP_0]) && REFI_IS_VALID(refi_cands[1][REFP_1]));

    if (pmv_cands[0][REFP_0][MV_X] == pmv_cands[1][REFP_0][MV_X] && pmv_cands[0][REFP_0][MV_Y] == pmv_cands[1][REFP_0][MV_Y] && 
        pmv_cands[0][REFP_1][MV_X] == pmv_cands[1][REFP_1][MV_X] && pmv_cands[0][REFP_1][MV_Y] == pmv_cands[1][REFP_1][MV_Y] && 
        refi_cands[0][REFP_0] == refi_cands[1][REFP_0] && refi_cands[0][REFP_1] == refi_cands[1][REFP_1])
    {
        for (u8 i = 1; i < TM_CANDS - 1; i++)
        {
            for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
            {
                refi_cands[i][refp_idx] = refi_cands[i + 1][refp_idx];
                copy_mv(pmv_cands[i][refp_idx], pmv_cands[i + 1][refp_idx]);
            }
        }
        *num_cands_all -= 1;
    }
}

void cands_adjustment(s16(*pmv_cands)[REFP_NUM][MV_D], s8(*refi_cands)[REFP_NUM], int* num_cands_all)
{
    for (u8 cand_idx = 0; cand_idx < *num_cands_all; cand_idx++)
    {
        for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
        {
            if (REFI_IS_VALID(refi_cands[cand_idx][refp_idx]))
            {
                for (u8 i = 0; i < MV_D; i++)
                {
                    pmv_cands[cand_idx][refp_idx][i] = (pmv_cands[cand_idx][refp_idx][i] >> 2) << 2;
                }
            }
        }
    }
}

void pre_evaluation_cands(int x, int y, int pic_w, int pic_h, int w, int h, pel* reco_luma, const int stride_luma, COM_REFP(*refp)[REFP_NUM], s16(*pmv_cands)[REFP_NUM][MV_D], s8(*refi_cands)[REFP_NUM], int* num_cands_all, const int bit_depth, BOOL is_simplified)
{
    //断言确保x和y大于或等于模板宽度
    assert(x>=TM_WIDTH && y>=TM_WIDTH);

    COM_PIC* ref_pic;
    static pel template_rec[3][MAX_CU_SIZE*TM_WIDTH];
    static pel template_pred[REFP_NUM][3][MAX_CU_SIZE*TM_WIDTH];
    u16 tm_size[3][2] = { {w, TM_WIDTH}, {TM_WIDTH, h}, {TM_WIDTH, TM_WIDTH}};
    
    //复制模板区域原始数据
    for (u16 i = 0; i < TM_WIDTH; i++)
    {
        memcpy(template_rec[0] + i*w, reco_luma + (y-TM_WIDTH+i)*stride_luma + x, sizeof(pel)*w);
    }

    for (u16 i = 0; i < h; i++)
    {
        memcpy(template_rec[1] + i*TM_WIDTH, reco_luma + (y+i)*stride_luma + (x-TM_WIDTH), sizeof(pel)*TM_WIDTH);
    }

    for (u16 i = 0; i < TM_WIDTH; i++)
    {
        memcpy(template_rec[2] + i*TM_WIDTH, reco_luma + (y-TM_WIDTH+i)*stride_luma + (x-TM_WIDTH), sizeof(pel)*TM_WIDTH);
    }

    //初始化成本列表和原始候选数组
    u64 cost_list[TM_CANDS];
    s16 org_pmv_cands[TM_CANDS][REFP_NUM][MV_D];
    s8  org_refi_cands[TM_CANDS][REFP_NUM];
    for (u8 i = 0; i < TM_CANDS; i++)
    {
        cost_list[i] = 0xFFFFFFFFFFFFFFFFU;
        for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
        {
            copy_mv(org_pmv_cands[i][refp_idx], pmv_cands[i][refp_idx]);
            org_refi_cands[i][refp_idx] = refi_cands[i][refp_idx];
        }
    }
    
    //遍历每个候选进行评估
    for (u8 cand_idx = 0; cand_idx < *num_cands_all; cand_idx++)
    {
        s8   cur_refi[REFP_NUM];
        s16  cur_mv[REFP_NUM][MV_D];

        //获取当前候选的参考索引和运动矢量
        cur_refi[REFP_0] = org_refi_cands[cand_idx][REFP_0];
        cur_refi[REFP_1] = org_refi_cands[cand_idx][REFP_1];
        cur_mv[REFP_0][MV_X] = org_pmv_cands[cand_idx][REFP_0][MV_X];
        cur_mv[REFP_0][MV_Y] = org_pmv_cands[cand_idx][REFP_0][MV_Y];
        cur_mv[REFP_1][MV_X] = org_pmv_cands[cand_idx][REFP_1][MV_X];
        cur_mv[REFP_1][MV_Y] = org_pmv_cands[cand_idx][REFP_1][MV_Y];

        u64 tm_cost = 0;
        for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
        {
            if (!REFI_IS_VALID(cur_refi[refp_idx]))
            {
                continue;
            }

            ref_pic = refp[cur_refi[refp_idx]][refp_idx].pic;
            single_mv_clip(x-TM_WIDTH, y-TM_WIDTH, pic_w, pic_h, w+TM_WIDTH, h+TM_WIDTH, cur_mv[refp_idx]);//运动矢量裁剪
            for (u8 i = 0; i < 3; i++)
            {
                u16 tm_w = tm_size[i][0];
                u16 tm_h = tm_size[i][1];
                //根据模板区域的选择来确定x，y的偏移量
                int x_offset = i == 0 ?         0 : (i == 1 ? -TM_WIDTH : -TM_WIDTH);
                int y_offset = i == 0 ? -TM_WIDTH : (i == 1 ?         0 : -TM_WIDTH);
                int qpel_gmv_x = ((x + x_offset) << 2) + cur_mv[refp_idx][MV_X];
                int qpel_gmv_y = ((y + y_offset) << 2) + cur_mv[refp_idx][MV_Y];

                int dx = cur_mv[refp_idx][MV_X] & 0x3 ? 1 : 0;
                int dy = cur_mv[refp_idx][MV_Y] & 0x3 ? 1 : 0;
                BOOL is_search = TRUE;

                if (is_simplified)
                {
                    if (dx == 0 && dy == 0)
                    {
                        com_mc_tm_l_00(ref_pic->y, template_pred[refp_idx][i], ref_pic->stride_luma, tm_w, qpel_gmv_x, qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
                    }
                    else if (dx == 1 && dy == 0)
                    {
                        com_mc_tm_l_n0(ref_pic->y, template_pred[refp_idx][i], ref_pic->stride_luma, tm_w, qpel_gmv_x, qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
                    }
                    else if (dx == 0 && dy == 1)
                    {
                        com_mc_tm_l_0n(ref_pic->y, template_pred[refp_idx][i], ref_pic->stride_luma, tm_w, qpel_gmv_x, qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
                    }
                    else if (dx == 1 && dy == 1)
                    {
                        com_mc_tm_l_nn(ref_pic->y, template_pred[refp_idx][i], ref_pic->stride_luma, tm_w, qpel_gmv_x, qpel_gmv_y, qpel_gmv_x, qpel_gmv_y, tm_w, tm_h, is_search, bit_depth);
                    }
                }
                else
                {
                    com_mc_l(cur_mv[refp_idx][MV_X], cur_mv[refp_idx][MV_Y], ref_pic->y, qpel_gmv_x, qpel_gmv_y, ref_pic->stride_luma, tm_w, template_pred[refp_idx][i], tm_w, tm_h, bit_depth);
                }
            }
        }

        if (REFI_IS_VALID(cur_refi[REFP_0]) && REFI_IS_VALID(cur_refi[REFP_1]))
        {
            for (u8 i = 0; i < 3; i++)
            {
                u16 tm_w = tm_size[i][0];
                u16 tm_h = tm_size[i][1];
                for (int m = 0; m < tm_h; m++)
                {
                    for (int n = 0; n < tm_w; n++)
                    {
                        int pred0 = template_pred[REFP_0][i][m * tm_w + n];
                        int pred1 = template_pred[REFP_1][i][m * tm_w + n];
                        template_pred[REFP_0][i][m * tm_w + n] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, ((pred0 + pred1 + 1)>>1));
                    }
                }
                tm_cost += com_sad_16b(com_tbl_log2[tm_w], com_tbl_log2[tm_h], template_rec[i], template_pred[REFP_0][i], tm_w, tm_w, bit_depth);
            }
        }
        else
        {
            u8 refp_idx = REFI_IS_VALID(cur_refi[REFP_0]) ? REFP_0 : REFP_1;
            for (u8 i = 0; i < 3; i++)
            {
                u16 tm_w = tm_size[i][0];
                u16 tm_h = tm_size[i][1];
                tm_cost += com_sad_16b(com_tbl_log2[tm_w], com_tbl_log2[tm_h], template_rec[i], template_pred[refp_idx][i], tm_w, tm_w, bit_depth);
            }
        }

        int shift = 0;
        while (shift < *num_cands_all && tm_cost < cost_list[*num_cands_all - 1 - shift])
        {
            shift++;
        }
        if (shift != 0)
        {
            for (u8 i = 1; i < shift; i++)
            {
                cost_list[*num_cands_all - i] = cost_list[*num_cands_all - 1 - i];
                for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
                {
                    copy_mv(pmv_cands[*num_cands_all - i][refp_idx], pmv_cands[*num_cands_all - 1 - i][refp_idx]);
                    refi_cands[*num_cands_all - i][refp_idx] = refi_cands[*num_cands_all - 1 - i][refp_idx];
                }
            }
            cost_list[*num_cands_all - shift] = tm_cost;
            for (u8 refp_idx = 0; refp_idx < REFP_NUM; refp_idx++)
            {
                copy_mv(pmv_cands[*num_cands_all - shift][refp_idx], org_pmv_cands[cand_idx][refp_idx]);
                refi_cands[*num_cands_all - shift][refp_idx] = org_refi_cands[cand_idx][refp_idx];
            }
        }
    }
}

void tm_padding(pel* img_pad, pel* img_rec, int s_rec, int width, int height, int offset)
{
    const int available_w = width - (2 * offset + 1);
    const int available_h = height - (2 * offset + 1);
    for (int i = 0; i < available_h; i++)
    {
        memcpy(img_pad + (i + offset) * width + offset, img_rec + i * s_rec, sizeof(pel) * available_w);
    }

    // up padding
    for (int i = 0; i < offset; i++)
    {
        memcpy(img_pad + i * width + offset, img_pad + offset * width + offset, sizeof(pel) * available_w);
    }
    // down padding
    for (int i = height - 1; i > height - 1 - (offset + 1); i--)
    {
        memcpy(img_pad + i * width + offset, img_pad + (height - 1 - (offset + 1)) * width + offset, sizeof(pel) * available_w);
    }
    // left padding
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < offset; j++)
        {
            img_pad[i * width + j] = img_pad[i * width + offset];
        }
    }
    // right padding
    for (int i = 0; i < height; i++)
    {
        for (int j = width - (offset + 1); j < width; j++)
        {
            img_pad[i * width + j] = img_pad[i * width + width - 1 - (offset + 1)];
        }
    }
}
#endif
