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

#include "com_esao.h"
#include <mmintrin.h>
#include <emmintrin.h>
#include <tmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>
#if LINUX
#include <cpuid.h>
#else
#include <intrin.h>
#endif 
#if ESAO
void esao_on_block_for_luma_avx(pel* p_dst, int i_dst, pel * p_src, int  i_src, int i_block_w, int  i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type)
{
    int     x, y, z;
    int     shift_bo = bit_depth;
    int end_x_r0_16, start_x_r, end_x_r, start_y, end_y;
    __m256i s0, s1, s2, s4, s3, s5, s6, s7, s8;
    __m256i t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, etype;
    __m128i mask;
    start_x_r = (lcu_available_left) ? 0 : 1;
    start_y = (lcu_available_up) ? 0 : 1;
    end_y = (lcu_available_down) ? i_block_h : i_block_h - 1;
    end_x_r = (lcu_available_right) ? i_block_w : i_block_w - 1;
    end_x_r0_16 = end_x_r - ((end_x_r - start_x_r) & 0x0f);
    const short max_pixel = (1 << bit_depth) - 1;
    __m256i     min_val = _mm256_setzero_si256();
    __m256i     max_val = _mm256_set1_epi16(max_pixel);
    __m256i     c4 = _mm256_set1_epi16(bo_value);
    if (luma_type == 0)
    {
        __m256i     c3 = _mm256_set1_epi16(NUM_ESAO_LUMA_TYPE0);
        __m256i     c2 = _mm256_set1_epi16(8);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;

        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x_r; x < end_x_r; x += 16)
            {
                s0 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src - 1]);
                s1 = _mm256_loadu_si256((__m256i *)&p_src[x]);
                s2 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src + 1]);
                s3 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src]);
                s4 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src + 1]);
                s5 = _mm256_loadu_si256((__m256i *)&p_src[x - 1]);
                s6 = _mm256_loadu_si256((__m256i *)&p_src[x + 1]);
                s7 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src - 1]);
                s8 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src]);

                t3 = _mm256_min_epu16(s0, s1);
                t1 = _mm256_cmpeq_epi16(t3, s0);
                t2 = _mm256_cmpeq_epi16(t3, s1);
                t0 = _mm256_subs_epi16(t1, t2);

                t3 = _mm256_min_epu16(s2, s1);
                t1 = _mm256_cmpeq_epi16(t3, s1);
                t2 = _mm256_cmpeq_epi16(t3, s2);
                t3 = _mm256_subs_epi16(t2, t1);

                t4 = _mm256_min_epu16(s1, s3);
                t1 = _mm256_cmpeq_epi16(t4, s1);
                t2 = _mm256_cmpeq_epi16(t4, s3);
                t4 = _mm256_subs_epi16(t2, t1);

                t5 = _mm256_min_epu16(s1, s4);
                t1 = _mm256_cmpeq_epi16(t5, s1);
                t2 = _mm256_cmpeq_epi16(t5, s4);
                t5 = _mm256_subs_epi16(t2, t1);

                t6 = _mm256_min_epu16(s1, s5);
                t1 = _mm256_cmpeq_epi16(t6, s1);
                t2 = _mm256_cmpeq_epi16(t6, s5);
                t6 = _mm256_subs_epi16(t2, t1);

                t7 = _mm256_min_epu16(s1, s6);
                t1 = _mm256_cmpeq_epi16(t7, s1);
                t2 = _mm256_cmpeq_epi16(t7, s6);
                t7 = _mm256_subs_epi16(t2, t1);

                t8 = _mm256_min_epu16(s1, s7);
                t1 = _mm256_cmpeq_epi16(t8, s1);
                t2 = _mm256_cmpeq_epi16(t8, s7);
                t8 = _mm256_subs_epi16(t2, t1);

                t9 = _mm256_min_epu16(s1, s8);
                t1 = _mm256_cmpeq_epi16(t9, s1);
                t2 = _mm256_cmpeq_epi16(t9, s8);
                t9 = _mm256_subs_epi16(t2, t1);

                etype = _mm256_add_epi16(t0, t3);
                etype = _mm256_add_epi16(etype, t4);
                etype = _mm256_add_epi16(etype, t5);
                etype = _mm256_add_epi16(etype, t6);
                etype = _mm256_add_epi16(etype, t7);
                etype = _mm256_add_epi16(etype, t8);
                etype = _mm256_add_epi16(etype, t9);
                etype = _mm256_adds_epi16(etype, c2);

                t10 = _mm256_mullo_epi16(s1, c4);
                t10 = _mm256_srli_epi16(t10, shift_bo);
                t10 = _mm256_mullo_epi16(t10, c3);
                etype = _mm256_add_epi16(t10, etype);

                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 16; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm256_loadu_si256((__m256i *)&(t[0]));
                t0 = _mm256_add_epi16(t0, s1);
                t0 = _mm256_min_epi16(t0, max_val);
                t0 = _mm256_max_epi16(t0, min_val);

                if (x != end_x_r0_16)
                {
                    _mm256_storeu_si256((__m256i *)(p_dst + x), t0);
                }
                else
                {
                    if (end_x_r - x >= 8)
                    {
                        _mm_storeu_si128((__m128i *)(p_dst + x), _mm256_castsi256_si128(t0));

                        if (end_x_r - x > 8)
                        {
                            mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 9]));
                            _mm_maskmoveu_si128(_mm256_extracti128_si256(t0, 1), mask, (char *)(p_dst + x + 8));
                        }
                    }
                    else
                    {
                        mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                        _mm_maskmoveu_si128(_mm256_castsi256_si128(t0), mask, (char *)(p_dst + x));
                    }
                    break;
                }
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
    else
    {
        __m256i     c3 = _mm256_set1_epi16(NUM_ESAO_LUMA_TYPE1);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x_r; x < end_x_r; x += 16)
            {
                s0 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src - 1]);
                s1 = _mm256_loadu_si256((__m256i *)&p_src[x]);
                s2 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src + 1]);
                s3 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src]);
                s4 = _mm256_loadu_si256((__m256i *)&p_src[x - i_src + 1]);
                s5 = _mm256_loadu_si256((__m256i *)&p_src[x - 1]);
                s6 = _mm256_loadu_si256((__m256i *)&p_src[x + 1]);
                s7 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src - 1]);
                s8 = _mm256_loadu_si256((__m256i *)&p_src[x + i_src]);

                t0 = _mm256_cmpgt_epi16(s0, s1);
                t1 = _mm256_cmpgt_epi16(s2, s1);
                t2 = _mm256_cmpgt_epi16(s3, s1);
                t3 = _mm256_cmpgt_epi16(s4, s1);
                t4 = _mm256_cmpgt_epi16(s5, s1);
                t5 = _mm256_cmpgt_epi16(s6, s1);
                t6 = _mm256_cmpgt_epi16(s7, s1);
                t7 = _mm256_cmpgt_epi16(s8, s1);

                etype = _mm256_add_epi16(t0, t1);
                etype = _mm256_add_epi16(etype, t2);
                etype = _mm256_add_epi16(etype, t3);
                etype = _mm256_add_epi16(etype, t4);
                etype = _mm256_add_epi16(etype, t5);
                etype = _mm256_add_epi16(etype, t6);
                etype = _mm256_add_epi16(etype, t7);
                etype = _mm256_abs_epi16(etype);

                t10 = _mm256_mullo_epi16(s1, c4);
                t10 = _mm256_srli_epi16(t10, shift_bo);
                t10 = _mm256_mullo_epi16(t10, c3);
                etype = _mm256_add_epi16(t10, etype);

                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 16; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm256_loadu_si256((__m256i *)&(t[0]));
                t0 = _mm256_add_epi16(t0, s1);
                t0 = _mm256_min_epi16(t0, max_val);
                t0 = _mm256_max_epi16(t0, min_val);

                if (x != end_x_r0_16)
                {
                    _mm256_storeu_si256((__m256i *)(p_dst + x), t0);
                }
                else
                {
                    if (end_x_r - x >= 8)
                    {
                        _mm_storeu_si128((__m128i *)(p_dst + x), _mm256_castsi256_si128(t0));

                        if (end_x_r - x > 8)
                        {
                            mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 9]));
                            _mm_maskmoveu_si128(_mm256_extracti128_si256(t0, 1), mask, (char *)(p_dst + x + 8));
                        }
                    }
                    else
                    {
                        mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                        _mm_maskmoveu_si128(_mm256_castsi256_si128(t0), mask, (char *)(p_dst + x));
                    }
                    break;
                }
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
}

void esao_on_block_for_luma_sse(pel* p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type)
{
    int     x, y, z;
    int     shift_bo = bit_depth;
    int end_x_r0_16, start_x_r, end_x_r, start_y, end_y;
    __m128i s0, s1, s2, s4, s3, s5, s6, s7, s8;
    __m128i t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, etype;
    __m128i mask;

    start_x_r = (lcu_available_left) ? 0 : 1;
    start_y = (lcu_available_up) ? 0 : 1;
    end_y = (lcu_available_down) ? i_block_h : i_block_h - 1;
    end_x_r = (lcu_available_right) ? i_block_w : i_block_w - 1;
    end_x_r0_16 = end_x_r - ((end_x_r - start_x_r) & 0x7);
    const short max_pixel = (1 << bit_depth) - 1;
    __m128i     c4 = _mm_set1_epi16(bo_value);
    __m128i     min_val = _mm_setzero_si128();
    __m128i     max_val = _mm_set1_epi16(max_pixel);
    if (luma_type == 0)
    {
        __m128i     c3 = _mm_set1_epi16(NUM_ESAO_LUMA_TYPE0);
        __m128i     c2 = _mm_set1_epi16(8);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;
        for (y = start_y; y < end_y; y++) {
            for (x = start_x_r; x < end_x_r; x += 8) {
                s0 = _mm_loadu_si128((__m128i *)&p_src[x - i_src - 1]);
                s1 = _mm_loadu_si128((__m128i *)&p_src[x]);
                s2 = _mm_loadu_si128((__m128i *)&p_src[x + i_src + 1]);
                s3 = _mm_loadu_si128((__m128i *)&p_src[x - i_src]);
                s4 = _mm_loadu_si128((__m128i *)&p_src[x - i_src + 1]);
                s5 = _mm_loadu_si128((__m128i *)&p_src[x - 1]);
                s6 = _mm_loadu_si128((__m128i *)&p_src[x + 1]);
                s7 = _mm_loadu_si128((__m128i *)&p_src[x + i_src - 1]);
                s8 = _mm_loadu_si128((__m128i *)&p_src[x + i_src]);

                t3 = _mm_min_epu16(s0, s1);
                t1 = _mm_cmpeq_epi16(t3, s0);
                t2 = _mm_cmpeq_epi16(t3, s1);
                t0 = _mm_subs_epi16(t1, t2);

                t3 = _mm_min_epu16(s2, s1);
                t1 = _mm_cmpeq_epi16(t3, s1);
                t2 = _mm_cmpeq_epi16(t3, s2);
                t3 = _mm_subs_epi16(t2, t1);

                t4 = _mm_min_epu16(s1, s3);
                t1 = _mm_cmpeq_epi16(t4, s1);
                t2 = _mm_cmpeq_epi16(t4, s3);
                t4 = _mm_subs_epi16(t2, t1);

                t5 = _mm_min_epu16(s1, s4);
                t1 = _mm_cmpeq_epi16(t5, s1);
                t2 = _mm_cmpeq_epi16(t5, s4);
                t5 = _mm_subs_epi16(t2, t1);

                t6 = _mm_min_epu16(s1, s5);
                t1 = _mm_cmpeq_epi16(t6, s1);
                t2 = _mm_cmpeq_epi16(t6, s5);
                t6 = _mm_subs_epi16(t2, t1);

                t7 = _mm_min_epu16(s1, s6);
                t1 = _mm_cmpeq_epi16(t7, s1);
                t2 = _mm_cmpeq_epi16(t7, s6);
                t7 = _mm_subs_epi16(t2, t1);

                t8 = _mm_min_epu16(s1, s7);
                t1 = _mm_cmpeq_epi16(t8, s1);
                t2 = _mm_cmpeq_epi16(t8, s7);
                t8 = _mm_subs_epi16(t2, t1);

                t9 = _mm_min_epu16(s1, s8);
                t1 = _mm_cmpeq_epi16(t9, s1);
                t2 = _mm_cmpeq_epi16(t9, s8);
                t9 = _mm_subs_epi16(t2, t1);

                etype = _mm_add_epi16(t0, t3);
                etype = _mm_add_epi16(etype, t4);
                etype = _mm_add_epi16(etype, t5);
                etype = _mm_add_epi16(etype, t6);
                etype = _mm_add_epi16(etype, t7);
                etype = _mm_add_epi16(etype, t8);
                etype = _mm_add_epi16(etype, t9);
                etype = _mm_adds_epi16(etype, c2);

                t10 = _mm_mullo_epi16(s1, c4);
                t10 = _mm_srli_epi16(t10, shift_bo);
                t10 = _mm_mullo_epi16(t10, c3);
                etype = _mm_add_epi16(t10, etype);

                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 8; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm_loadu_si128((__m128i *)&t[0]);
                t0 = _mm_add_epi16(t0, s1);
                t0 = _mm_min_epi16(t0, max_val);
                t0 = _mm_max_epi16(t0, min_val);

                if (x != end_x_r0_16) {
                    _mm_storeu_si128((__m128i *)(p_dst + x), t0);
                }
                else {
                    mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                    _mm_maskmoveu_si128(t0, mask, (char *)(p_dst + x));
                    break;
                }

            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
    else
    {
        __m128i     c3 = _mm_set1_epi16(NUM_ESAO_LUMA_TYPE1);
        p_dst += start_y * i_dst;
        p_src += start_y * i_src;
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x_r; x < end_x_r; x += 8)
            {
                s0 = _mm_loadu_si128((__m128i *)&p_src[x - i_src - 1]);
                s1 = _mm_loadu_si128((__m128i *)&p_src[x]);
                s2 = _mm_loadu_si128((__m128i *)&p_src[x + i_src + 1]);
                s3 = _mm_loadu_si128((__m128i *)&p_src[x - i_src]);
                s4 = _mm_loadu_si128((__m128i *)&p_src[x - i_src + 1]);
                s5 = _mm_loadu_si128((__m128i *)&p_src[x - 1]);
                s6 = _mm_loadu_si128((__m128i *)&p_src[x + 1]);
                s7 = _mm_loadu_si128((__m128i *)&p_src[x + i_src - 1]);
                s8 = _mm_loadu_si128((__m128i *)&p_src[x + i_src]);

                t0 = _mm_cmpgt_epi16(s0, s1);
                t1 = _mm_cmpgt_epi16(s2, s1);
                t2 = _mm_cmpgt_epi16(s3, s1);
                t3 = _mm_cmpgt_epi16(s4, s1);
                t4 = _mm_cmpgt_epi16(s5, s1);
                t5 = _mm_cmpgt_epi16(s6, s1);
                t6 = _mm_cmpgt_epi16(s7, s1);
                t7 = _mm_cmpgt_epi16(s8, s1);

                etype = _mm_add_epi16(t0, t1);
                etype = _mm_add_epi16(etype, t2);
                etype = _mm_add_epi16(etype, t3);
                etype = _mm_add_epi16(etype, t4);
                etype = _mm_add_epi16(etype, t5);
                etype = _mm_add_epi16(etype, t6);
                etype = _mm_add_epi16(etype, t7);
                etype = _mm_abs_epi16(etype);

                t10 = _mm_mullo_epi16(s1, c4);
                t10 = _mm_srli_epi16(t10, shift_bo);
                t10 = _mm_mullo_epi16(t10, c3);
                etype = _mm_add_epi16(t10, etype);
                s16 *p_type = (s16*)&etype;
                s16 *t = (s16*)&t0;
                for (z = 0; z < 8; z++)
                {
                    t[z] = esao_offset[p_type[z]];
                }
                t0 = _mm_loadu_si128((__m128i *)&(t[0]));
                t0 = _mm_add_epi16(t0, s1);
                t0 = _mm_min_epi16(t0, max_val);
                t0 = _mm_max_epi16(t0, min_val);

                if (x != end_x_r0_16) {
                    _mm_storeu_si128((__m128i *)(p_dst + x), t0);
                }
                else {
                    mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x_r - end_x_r0_16 - 1]));
                    _mm_maskmoveu_si128(t0, mask, (char *)(p_dst + x));
                    break;
                }
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
}

void esao_on_block_for_luma_without_simd(pel* p_dst, int  i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bit_depth, int bo_value, const int *esao_offset, int lcu_available_left, int lcu_available_right,
    int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon, int luma_type)
{
    int start_x, end_x, start_y, end_y, x, y;
    start_x = (lcu_available_left) ? 0 : 1;
    start_y = (lcu_available_up) ? 0 : 1;
    end_y = (lcu_available_down) ? i_block_h : i_block_h - 1;
    end_x = (lcu_available_right) ? i_block_w : i_block_w - 1;
    p_dst += start_y * i_dst;
    p_src += start_y * i_src;
    if (luma_type == 0)
    {
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x; x < end_x; x++)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                //leftup
                if (p_src[x - 1 - i_src] > p_src[x])
                {
                    diff1 = 1;
                }
                else if (p_src[x - 1 - i_src] < p_src[x])
                {
                    diff1 = -1;
                }
                //up
                if (p_src[x - i_src] > p_src[x])
                {
                    diff2 = 1;
                }
                else if (p_src[x - i_src] < p_src[x])
                {
                    diff2 = -1;
                }
                //rightUp
                if (p_src[x + 1 - i_src] > p_src[x])
                {
                    diff3 = 1;
                }
                else if (p_src[x + 1 - i_src] < p_src[x])
                {
                    diff3 = -1;
                }
                //left
                if (p_src[x - 1] > p_src[x])
                {
                    diff4 = 1;
                }
                else if (p_src[x - 1] < p_src[x])
                {
                    diff4 = -1;
                }
                //right
                if (p_src[x + 1] > p_src[x])
                {
                    diff5 = 1;
                }
                else if (p_src[x + 1] < p_src[x])
                {
                    diff5 = -1;
                }
                //leftdown
                if (p_src[x - 1 + i_src] > p_src[x])
                {
                    diff6 = 1;
                }
                else if (p_src[x - 1 + i_src] < p_src[x])
                {
                    diff6 = -1;
                }
                //down
                if (p_src[x + i_src] > p_src[x])
                {
                    diff7 = 1;
                }
                else if (p_src[x + i_src] < p_src[x])
                {
                    diff7 = -1;
                }
                //rightDown
                if (p_src[x + 1 + i_src] > p_src[x])
                {
                    diff8 = 1;
                }
                else if (p_src[x + 1 + i_src] < p_src[x])
                {
                    diff8 = -1;
                }
                int diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8 + 8;
                int band_type = (p_src[x] * bo_value) >> bit_depth;
                int true_index = band_type * NUM_ESAO_LUMA_TYPE0 + diff_count;
                p_dst[x] = (pel)COM_CLIP3(0, ((1 << bit_depth) - 1), p_src[x] + esao_offset[true_index]);
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
    else
    {
        for (y = start_y; y < end_y; y++)
        {
            for (x = start_x; x < end_x; x++)
            {
                int diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;
                int diff5 = 0, diff6 = 0, diff7 = 0, diff8 = 0;
                if (p_src[x - 1 - i_src] > p_src[x])
                {
                    diff1 = 1;
                }
                if (p_src[x - i_src] > p_src[x])
                {
                    diff2 = 1;
                }
                if (p_src[x + 1 - i_src] > p_src[x])
                {
                    diff3 = 1;
                }
                if (p_src[x - 1] > p_src[x])
                {
                    diff4 = 1;
                }
                if (p_src[x + 1] > p_src[x])
                {
                    diff5 = 1;
                }
                if (p_src[x - 1 + i_src] > p_src[x])
                {
                    diff6 = 1;
                }
                if (p_src[x + i_src] > p_src[x])
                {
                    diff7 = 1;
                }
                if (p_src[x + 1 + i_src] > p_src[x])
                {
                    diff8 = 1;
                }
                int diff_count = diff1 + diff2 + diff3 + diff4 + diff5 + diff6 + diff7 + diff8;
                int band_type = (p_src[x] * bo_value) >> bit_depth;
                int true_index = band_type * NUM_ESAO_LUMA_TYPE1 + diff_count;
                p_dst[x] = (pel)COM_CLIP3(0, ((1 << bit_depth) - 1), p_src[x] + esao_offset[true_index]);
            }
            p_dst += i_dst;
            p_src += i_src;
        }
    }
}

void esao_on_block_for_chroma_avx(pel * p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset)
{
    __m256i t0, src0, src1;
    __m256i     c1 = _mm256_set1_epi16(bo_value);
    __m128i mask = _mm_setzero_si128();
    __m256i s1;
    int     x, y, z;
    int     shift_bo = shift_value;
    shift_bo -= (bit_depth == 8) ? 2 : 0;
    const short   max_pixel = (1 << bit_depth) - 1;
    const __m256i min_val = _mm256_setzero_si256();
    __m256i max_val = _mm256_set1_epi16(max_pixel);
    int     end_x = i_block_w;
    int     end_x_16 = end_x - ((end_x - 0) & 0x0f);

    for (y = 0; y < i_block_h; y++)
    {
        for (x = 0; x < i_block_w; x += 16)
        {
            s1 = _mm256_loadu_si256((__m256i *)&p_src[x]);
            src0 = _mm256_mullo_epi16(s1, c1);
            src1 = _mm256_srli_epi16(src0, shift_bo);
            s16 *p_type = (s16*)&src1;
            s16 *t = (s16*)&t0;
            for (z = 0; z < 16; z++)
            {
                t[z] = esao_offset[p_type[z]];
            }
            t0 = _mm256_loadu_si256((__m256i *)&(t[0]));
            t0 = _mm256_add_epi16(t0, s1);
            t0 = _mm256_min_epi16(t0, max_val);
            t0 = _mm256_max_epi16(t0, min_val);
            if (x < end_x_16)
            {
                _mm256_storeu_si256((__m256i *)(p_dst + x), t0);
            }
            else
            {
                if (end_x - x >= 8)
                {
                    _mm_storeu_si128((__m128i *)(p_dst + x), _mm256_castsi256_si128(t0));
                    if (end_x - x > 8)
                    {
                        mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x - end_x_16 - 9]));
                        _mm_maskmoveu_si128(_mm256_extracti128_si256(t0, 1), mask, (char *)(p_dst + x + 8));
                    }
                }
                else
                {
                    mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x - end_x_16 - 1]));
                    _mm_maskmoveu_si128(_mm256_castsi256_si128(t0), mask, (char *)(p_dst + x));
                }
                break;
            }
        }
        p_dst += i_dst;
        p_src += i_src;
    }
}

void esao_on_block_for_chroma_sse(pel * p_dst, int i_dst, pel * p_src, int i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset)
{
    __m128i t0, src0, src1;
    __m128i     c1 = _mm_set1_epi16(bo_value);
    __m128i mask = _mm_setzero_si128();
    __m128i s1;
    int     x, y, z;
    int     shift_bo = shift_value;
    shift_bo -= (bit_depth == 8) ? 2 : 0;
    const short   max_pixel = (1 << bit_depth) - 1;
    const __m128i min_val = _mm_setzero_si128();
    __m128i max_val = _mm_set1_epi16(max_pixel);
    int     end_x = i_block_w;
    int     end_x_16 = end_x - ((end_x - 0) & 0x7);

    for (y = 0; y < i_block_h; y++) {
        for (x = 0; x < i_block_w; x += 8) {
            s1 = _mm_loadu_si128((__m128i *)&p_src[x]);
            src0 = _mm_mullo_epi16(s1, c1);
            src1 = _mm_srli_epi16(src0, shift_bo);
            s16 *p_type = (s16*)&src1;
            s16 *t = (s16*)&t0;
            for (z = 0; z < 8; z++)
            {
                t[z] = esao_offset[p_type[z]];
            }

            t0 = _mm_add_epi16(t0, s1);
            t0 = _mm_min_epi16(t0, max_val);
            t0 = _mm_max_epi16(t0, min_val);

            if (x < end_x_16) {
                _mm_storeu_si128((__m128i *)(p_dst + x), t0);
            }
            else {
                mask = _mm_load_si128((__m128i *)(intrinsic_mask_10bit[end_x - end_x_16 - 1]));
                _mm_maskmoveu_si128(t0, mask, (char *)(p_dst + x));
                break;
            }
        }
        p_dst += i_dst;
        p_src += i_src;
    }
}

void esao_on_block_for_chroma_without_simd(pel * p_dst, int i_dst, pel * p_src, int  i_src, int i_block_w, int i_block_h, int bo_value, int shift_value, int bit_depth, const int *esao_offset)
{
    int x, y, z;
    int shift_bo = shift_value;
    shift_bo -= (bit_depth == 8) ? 2 : 0;
    int value_bo = bo_value;
    const int   max_pixel = (1 << bit_depth) - 1;
    for (y = 0; y < i_block_h; y++)
    {
        for (x = 0; x < i_block_w; x++)
        {
            z = (p_src[x] * value_bo) >> shift_bo;
            p_dst[x] = (pel)COM_CLIP3(0, max_pixel, p_src[x] + esao_offset[z]);
        }
        p_dst += i_dst;
        p_src += i_src;
    }
}

int is_support_sse_avx()
{
#if LINUX
    int avail = 0;
    __builtin_cpu_init();
    if (__builtin_cpu_supports("sse4.2"))
    {
        avail = 1;
    }
    if (__builtin_cpu_supports("avx2"))
    {
        avail = 2;
    }
    return avail;
#else
    int cpu_info[4] = { 0,0,0,0 };
    int avail = 0;
    //check sse4.2 available
    __cpuid(cpu_info, 1);
    if ((cpu_info[2] & 0x00100000) != 0)
    {
        avail = 1;
    }
    //check AVX2 available
    if ((cpu_info[2] & 0x18000000) == 0x18000000)
    {
        unsigned long long xcr_feature_mask = _xgetbv(_XCR_XFEATURE_ENABLED_MASK);
        if ((xcr_feature_mask & 0x6) == 0x6) // Check for OS support 
        {
            memset(cpu_info, 0, sizeof(int) * 4);
            __cpuid(cpu_info, 7);
            if ((cpu_info[1] & 0x00000020) != 0)
            {
                avail = 2;
            }
        }
    }
    return avail;  // 0:C  1:SSE  2:AVX
#endif
}

void decide_esao_filter_func_pointer(ESAO_FUNC_POINTER *func_esao_filter)
{
    int avail = is_support_sse_avx();
    if (avail == 2)
    {
        func_esao_filter->esao_on_block_for_luma = esao_on_block_for_luma_avx;
        func_esao_filter->esao_on_block_for_chroma = esao_on_block_for_chroma_avx;
    }
    else if (avail == 1)
    {
        func_esao_filter->esao_on_block_for_luma = esao_on_block_for_luma_sse;
        func_esao_filter->esao_on_block_for_chroma = esao_on_block_for_chroma_sse;
    }
    else
    {
        func_esao_filter->esao_on_block_for_luma = esao_on_block_for_luma_without_simd;
        func_esao_filter->esao_on_block_for_chroma = esao_on_block_for_chroma_without_simd;
    }
}

#if ESAO_ENH
int com_malloc_3d_esao_stat_data(ESAO_STAT_DATA **** array3D, int num_SMB, int num_comp, int num_class)
{
    int i, j;
    if (((*array3D) = (ESAO_STAT_DATA ** *)calloc(num_SMB, sizeof(ESAO_STAT_DATA **))) == NULL)
    {
        printf("MALLOC FAILED: get_mem3DESAOstatdate: array3D");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_SMB; i++)
    {
        if ((*(*array3D + i) = (ESAO_STAT_DATA **)calloc(num_comp, sizeof(ESAO_STAT_DATA *))) == NULL)
        {
            printf("MALLOC FAILED: get_mem2DESAOstatdate: array2D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < num_comp; j++)
        {
            if ((*(*(*array3D + i) + j) = (ESAO_STAT_DATA *)calloc(num_class, sizeof(ESAO_STAT_DATA))) == NULL)
            {
                printf("MALLOC FAILED: get_mem1DESAOstatdate: arrayD");
                assert(0);
                exit(-1);
            }
        }
    }
    return num_SMB * num_comp * num_class * sizeof(ESAO_STAT_DATA);
}

void com_free_3d_esao_stat_data(ESAO_STAT_DATA ****array3D, int num_SMB, int num_comp)
{
    int i, j;
    if (*array3D)
    {
        for (i = 0; i < num_SMB; i++)
        {
            if ((*array3D)[i])
            {
                for (j = 0; j < num_comp; j++)
                {
                    if ((*array3D)[i][j])
                    {
                        free((*array3D)[i][j]);
                        (*array3D)[i][j] = NULL;
                    }
                }
                free((*array3D)[i]);
                (*array3D)[i] = NULL;
            }
        }
        free(*array3D);
        *array3D = NULL;
    }
}

void copy_esao_param_for_one_component(ESAO_BLK_PARAM *esao_para_dst, ESAO_BLK_PARAM *esao_para_src, int lcu_count)
{
    int  j;
    esao_para_dst->set_num = esao_para_src->set_num;
    for (j = 0; j < ESAO_SET_NUM; j++)
    {
        memcpy(esao_para_dst->offset[j], esao_para_src->offset[j], sizeof(int) * ESAO_LABEL_CLASSES_MAX);
        esao_para_dst->mode[j]               = esao_para_src->mode[j];
        esao_para_dst->luma_type[j]          = esao_para_src->luma_type[j];
        esao_para_dst->chroma_band_flag[j]   = esao_para_src->chroma_band_flag[j];
        esao_para_dst->chroma_band_length[j] = esao_para_src->chroma_band_length[j];
        esao_para_dst->chroma_start_band[j]  = esao_para_src->chroma_start_band[j];
    }
    for (j = 0; j < lcu_count; j++)
    {
        esao_para_dst->lcu_flag[j] = esao_para_src->lcu_flag[j];
    }
}
#else
int com_malloc_2d_esao_stat_data(ESAO_STAT_DATA *** array2D, int num_SMB, int num_comp)
{
    int i;
    if (((*array2D) = (ESAO_STAT_DATA **)calloc(num_SMB, sizeof(ESAO_STAT_DATA *))) == NULL)
    {
        printf("MALLOC FAILED: com_malloc_2d_esao_stat_data 1");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_SMB; i++)
    {
        if ((*(*array2D + i) = (ESAO_STAT_DATA *)calloc(num_comp, sizeof(ESAO_STAT_DATA))) == NULL)
        {
            printf("MALLOC FAILED: com_malloc_2d_esao_stat_data 2");
            assert(0);
            exit(-1);
        }
    }
    return num_SMB * num_comp * sizeof(ESAO_STAT_DATA);
}

void com_free_2d_esao_stat_data(ESAO_STAT_DATA ***array2D, int num_SMB)
{
    int i;
    if (*array2D)
    {
        for (i = 0; i < num_SMB; i++)
        {
            if ((*array2D)[i])
            {
                free((*array2D)[i]);
                (*array2D)[i] = NULL;
            }
        }
        free(*array2D);
        *array2D = NULL;
    }
}

void copy_esao_param_for_one_component(ESAO_BLK_PARAM *esao_para_dst, ESAO_BLK_PARAM *esao_para_src)
{
    int  j;
    for (j = 0; j < ESAO_LABEL_CLASSES_MAX; j++)
    {
        esao_para_dst->offset[j] = esao_para_src->offset[j];
    }
}
#endif

void copy_frame_for_esao(COM_PIC * pic_dst, COM_PIC * pic_src)
{
    int i, j;
    int src_stride, dst_stride;
    pel* src;
    pel* dst;
    src_stride = pic_src->stride_luma;
    dst_stride = pic_dst->stride_luma;
    src = pic_src->y;
    dst = pic_dst->y;
    for (j = 0; j < pic_src->height_luma; j++)
    {
        for (i = 0; i < pic_src->width_luma; i++)
        {
            dst[i] = src[i];
        }
        memset(&dst[pic_src->width_luma], 0, sizeof(pel)*(dst_stride - pic_src->width_luma));
        dst += dst_stride;
        src += src_stride;
    }
    src_stride = pic_src->stride_chroma;
    dst_stride = pic_dst->stride_chroma;
    src = pic_src->u;
    dst = pic_dst->u;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        memset(&dst[pic_src->width_chroma], 0, sizeof(pel)*(dst_stride - pic_src->width_chroma));
        dst += dst_stride;
        src += src_stride;
    }
    src = pic_src->v;
    dst = pic_dst->v;
    for (j = 0; j < pic_src->height_chroma; j++)
    {
        for (i = 0; i < pic_src->width_chroma; i++)
        {
            dst[i] = src[i];
        }
        memset(&dst[pic_src->width_chroma], 0, sizeof(pel)*(dst_stride - pic_src->width_chroma));
        dst += dst_stride;
        src += src_stride;
    }
}

void esao_on_smb(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAO_FUNC_POINTER *func_esao_filter, int pix_y, int pix_x, int lcu_pix_width, int lcu_pix_height, ESAO_BLK_PARAM *esao_blk_param, int sample_bit_depth,int lcu_pos)
{
    int comp_idx;
    int lcu_pix_height_t, lcu_pix_width_t, pix_x_t, pix_y_t;
    int  is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;
    for (comp_idx = 0; comp_idx < N_C; comp_idx++)
    {
        if ( info->pic_header.pic_esao_on[comp_idx])
        {
            lcu_pix_width_t = comp_idx ? ((lcu_pix_width >> 1) ) : (lcu_pix_width );
            lcu_pix_height_t = comp_idx ? ((lcu_pix_height >> 1) ) : (lcu_pix_height );
            pix_x_t = comp_idx ? (pix_x >> 1) : pix_x;
            pix_y_t = comp_idx ? (pix_y >> 1) : pix_y;
            check_boundary_available_for_esao(info, map, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t, comp_idx, &is_left_avail,
                &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
            esao_on_block(info, map, pic_rec, pic_esao, esao_blk_param, func_esao_filter, comp_idx, pix_y_t, pix_x_t, lcu_pix_height_t, lcu_pix_width_t,sample_bit_depth, lcu_pos, esao_blk_param[comp_idx].lcu_flag[lcu_pos],
                is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail,is_above_right_avail, is_below_left_avail, is_below_right_avail);
        }
    }
}

void esao_on_block(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAO_BLK_PARAM *esao_blk_param, ESAO_FUNC_POINTER *func_esao_filter, int comp_idx, int pix_y, int pix_x, int lcu_pix_height,int lcu_pix_width, int sample_bit_depth, int lcu_pos,int lcu_open_flag,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
#if ESAO_ENH
    pel *src, *dst;
    int src_stride, dst_stride;
    src = dst = NULL;
    int lcu_flag = info->pic_header.esao_lcu_enable[comp_idx];
    int set = lcu_flag ? esao_blk_param[comp_idx].lcu_flag[lcu_pos] - 1 : 0;
    int mode = info->pic_header.esao_adaptive_param[comp_idx][set];
    int luma_type = info->pic_header.esao_luma_type[set];
    assert(mode >= 0 && mode < ESAO_LABEL_NUM_MAX);
    int shift_index = (mode + 1);
    switch (comp_idx)
    {
    case Y_C:
        src_stride = pic_esao->stride_luma;
        dst_stride = pic_rec->stride_luma;
        src = pic_esao->y;
        dst = pic_rec->y;
        break;
    case U_C:
        src_stride = pic_esao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_esao->u;
        dst = pic_rec->u;
        break;
    case V_C:
        src_stride = pic_esao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_esao->v;
        dst = pic_rec->v;
        break;
    default:
        src_stride = 0;
        dst_stride = 0;
        src = NULL;
        dst = NULL;
        assert(0);
    }
    if (comp_idx == Y_C)
    {
        if (lcu_flag)
        {
            if (lcu_open_flag)
            {
                func_esao_filter->esao_on_block_for_luma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height,
                    sample_bit_depth, shift_index, esao_blk_param[comp_idx].offset[set], lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down,
                    lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon, luma_type);
            }
        }
        else
        {
            func_esao_filter->esao_on_block_for_luma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height,
                sample_bit_depth, shift_index, esao_blk_param[comp_idx].offset[set], lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down,
                lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon, luma_type);
        }
    }
    else
    {
        if (lcu_flag)
        {
            if (lcu_open_flag)
            {
                func_esao_filter->esao_on_block_for_chroma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height, tab_esao_chroma_class_bit[mode][1], tab_esao_chroma_class_bit[mode][2],
                    sample_bit_depth, esao_blk_param[comp_idx].offset[set]);
            }
        }
        else
        {
            func_esao_filter->esao_on_block_for_chroma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height, tab_esao_chroma_class_bit[mode][1], tab_esao_chroma_class_bit[mode][2],
                sample_bit_depth, esao_blk_param[comp_idx].offset[set]);
        }
    }
#else
    pel *src, *dst;
    int src_stride, dst_stride;
    src = dst = NULL;
    int label_index = info->pic_header.esao_adaptive_param[comp_idx];
    assert(label_index >= 0 && label_index < ESAO_LABEL_NUM_MAX);
    int shift_index = (label_index + 1);
    switch (comp_idx)
    {
    case Y_C:
        src_stride = pic_esao->stride_luma;
        dst_stride = pic_rec->stride_luma;
        src = pic_esao->y;
        dst = pic_rec->y;
        break;
    case U_C:
        src_stride = pic_esao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_esao->u;
        dst = pic_rec->u;
        break;
    case V_C:
        src_stride = pic_esao->stride_chroma;
        dst_stride = pic_rec->stride_chroma;
        src = pic_esao->v;
        dst = pic_rec->v;
        break;
    default:
        src_stride = 0;
        dst_stride = 0;
        src = NULL;
        dst = NULL;
        assert(0);
    }
    if (comp_idx == Y_C)
    {
        if (info->pic_header.esao_lcu_enable[comp_idx])
        {
            if (lcu_open_flag)
            {
                func_esao_filter->esao_on_block_for_luma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height,
                    sample_bit_depth, shift_index, esao_blk_param[comp_idx].offset, lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down,
                    lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon,info->pic_header.esao_luma_type);
            }
        }
        else 
        {
            func_esao_filter->esao_on_block_for_luma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height,
                sample_bit_depth, shift_index, esao_blk_param[comp_idx].offset, lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down,
                lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon,info->pic_header.esao_luma_type);
        }
    }
    else
    {
        if (info->pic_header.esao_lcu_enable[comp_idx])
        {
            if (lcu_open_flag)
            {
                func_esao_filter->esao_on_block_for_chroma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height, tab_esao_chroma_class_bit[label_index][1], tab_esao_chroma_class_bit[label_index][2],
                    sample_bit_depth, esao_blk_param[comp_idx].offset);
            }
        }
        else
        {
            func_esao_filter->esao_on_block_for_chroma(dst + pix_y * dst_stride + pix_x, dst_stride, src + pix_y * src_stride + pix_x, src_stride, lcu_pix_width, lcu_pix_height, tab_esao_chroma_class_bit[label_index][1], tab_esao_chroma_class_bit[label_index][2],
                sample_bit_depth, esao_blk_param[comp_idx].offset);
        }
    }
#endif
}

void esao_on_frame(COM_INFO *info, COM_MAP *map, COM_PIC  *pic_rec, COM_PIC  *pic_esao, ESAO_BLK_PARAM *rec_esao_params, ESAO_FUNC_POINTER *func_esao_filter)
{
    if ((info->pic_header.pic_esao_on[Y_C] == 0) && (info->pic_header.pic_esao_on[U_C] == 0) && (info->pic_header.pic_esao_on[V_C] == 0))
    {
        return;
    }
    decide_esao_filter_func_pointer(func_esao_filter);
    int pic_pix_height = info->pic_height;
    int pic_pix_width = info->pic_width;
    int input_MaxSizeInBit = info->log2_max_cuwh;
    int bit_depth = info->bit_depth_internal;
    int pix_y, pix_x;
    int lcu_pix_height, lcu_pix_width;
    for (pix_y = 0; pix_y < pic_pix_height; pix_y += lcu_pix_height)
    {
        lcu_pix_height = min(1 << (input_MaxSizeInBit), (pic_pix_height - pix_y));
        for (pix_x = 0; pix_x < pic_pix_width; pix_x += lcu_pix_width)
        {
            int x_in_lcu = pix_x >> info->log2_max_cuwh;
            int y_in_lcu = pix_y >> info->log2_max_cuwh;
            int lcu_pos = x_in_lcu + y_in_lcu * info->pic_width_in_lcu;
            lcu_pix_width = min(1 << (input_MaxSizeInBit), (pic_pix_width - pix_x));
            esao_on_smb(info, map, pic_rec, pic_esao, func_esao_filter, pix_y, pix_x, lcu_pix_width, lcu_pix_height, rec_esao_params, bit_depth,lcu_pos);
        }
    }
}
#endif

#if ESAO || CCSAO
static BOOL is_same_patch(s8* map_patch_idx, int mb_nr1, int mb_nr2)
{
    assert(mb_nr1 >= 0);
    assert(mb_nr2 >= 0);
    return (map_patch_idx[mb_nr1] == map_patch_idx[mb_nr2]);
}

void check_boundary_available_for_esao(COM_INFO *info, COM_MAP *map, int pix_y, int pix_x, int lcu_pix_height, int lcu_pix_width, int comp, int *lcu_process_left, int *lcu_process_right,
    int *lcu_process_up, int *lcu_process_down, int *lcu_process_upleft, int *lcu_process_upright, int *lcu_process_leftdown, int *lcu_process_rightdwon, int filter_on)
{
    int pic_width = comp ? (info->pic_width >> 1) : info->pic_width;
    int pic_height = comp ? (info->pic_height >> 1) : info->pic_height;
    int pic_mb_width = info->pic_width_in_scu;
    int mb_size_in_bit = comp ? (MIN_CU_LOG2 - 1) : MIN_CU_LOG2;
    int mb_nr_cur, mb_nr_up, mb_nr_down, mb_nr_left, mb_nr_right, mb_nr_upleft, mb_nr_upright, mb_nr_leftdown, mb_nr_rightdown;
    s8 *map_patch_idx = map->map_patch_idx;
    int cross_patch_flag = info->sqh.cross_patch_loop_filter;

    mb_nr_cur = (pix_y >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_up = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_down = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + (pix_x >> mb_size_in_bit);
    mb_nr_left = (pix_y >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_right = (pix_y >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >> mb_size_in_bit);
    mb_nr_upleft = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_upright = ((pix_y - 1) >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >> mb_size_in_bit);
    mb_nr_leftdown = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + ((pix_x - 1) >> mb_size_in_bit);
    mb_nr_rightdown = ((pix_y + lcu_pix_height) >> mb_size_in_bit) * pic_mb_width + ((pix_x + lcu_pix_width) >>
        mb_size_in_bit);
    *lcu_process_up = (pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_up) ? 1 :
        cross_patch_flag;
    *lcu_process_down = (pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_down) ? 1 : cross_patch_flag;
    *lcu_process_left = (pix_x == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_left) ? 1 :
        cross_patch_flag;
    *lcu_process_right = (pix_x >= pic_width - lcu_pix_width) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_right) ? 1 : cross_patch_flag;
    *lcu_process_upleft = (pix_x == 0 ||
        pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_upleft) ? 1 : cross_patch_flag;
    *lcu_process_upright = (pix_x >= pic_width - lcu_pix_width ||
        pix_y == 0) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_upright) ? 1 : cross_patch_flag;
    *lcu_process_leftdown = (pix_x == 0 ||
        pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_leftdown)
        ? 1 : cross_patch_flag;
    *lcu_process_rightdwon = (pix_x >= pic_width - lcu_pix_width ||
        pix_y >= pic_height - lcu_pix_height) ? 0 : is_same_patch(map_patch_idx, mb_nr_cur, mb_nr_rightdown)
        ? 1 : cross_patch_flag;
    if (!filter_on)
    {
        *lcu_process_down = 0;
        *lcu_process_right = 0;
        *lcu_process_leftdown = 0;
        *lcu_process_rightdwon = 0;
    }
}
#endif

#if CCSAO
void ccsao_on_block_for_chroma(pel *p_dst, int i_dst, pel *p_src, int i_src,
#if CCSAO_ENHANCEMENT
    pel *p_src2, int i_src2,
#endif
    int lcu_width_c, int lcu_height_c, int bit_depth, int type, int band_num,
#if CCSAO_ENHANCEMENT
    int band_num_c,
#endif
    const int *ccsao_offset,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int start_x_c = lcu_available_left ? 0 : 1;
    int start_y_c = lcu_available_up   ? 0 : 1;
    int end_x_c   = lcu_width_c;
    int end_y_c   = lcu_height_c;

    p_src += (start_y_c << 1) * i_src;
    p_dst +=  start_y_c       * i_dst;
#if CCSAO_ENHANCEMENT
    p_src2 += start_y_c * i_src2;
#endif
    
    for (int y_c = start_y_c; y_c < end_y_c; y_c++)
    {
#if CCSAO_LINE_BUFFER
        int line_idx = (lcu_available_down && y_c >= lcu_height_c - CCSAO_PAD_ROWS - 1) ? lcu_height_c - y_c - 1 : 0;
#endif
        for (int x_c = start_x_c; x_c < end_x_c; x_c++)
        {
            int  x     = x_c << 1;
#if CCSAO_LINE_BUFFER
            pel *col_Y = p_src + x + i_src * ccsao_type_y[type][line_idx] + ccsao_type_x[type];
#else
            pel *col_Y = p_src + x + i_src * ccsao_type_y[type] + ccsao_type_x[type];
#endif
            int  band  = (*col_Y * band_num) >> bit_depth;
#if CCSAO_ENHANCEMENT
            pel *col_C = p_src2 + x_c;
            int  band_c = (*col_C * band_num_c) >> bit_depth;
            band = band * band_num_c + band_c;
#endif

            p_dst[x_c] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, p_dst[x_c] + ccsao_offset[band]);
        }
        p_src += i_src << 1;
        p_dst += i_dst;
#if CCSAO_ENHANCEMENT
        p_src2 += i_src2;
#endif
    }
}

#if ECCSAO
int calc_diff_range(pel a, pel b, int th)
{
    int diff = a - b;
    int value = 0;
    int thred = ccsao_quan_value[th];
    int neg_thred = (-1)*thred;
    if (diff < 0)
    {
        if (diff < neg_thred)
            value = 0;
        else
            value = 1;
    }
    else
    {
        if (diff < thred)
            value = 2;
        else
            value = 3;
    }
    return value;
}
void ccsao_on_block_for_chroma_edge(pel *p_dst, int i_dst, pel *p_src, int i_src, pel *p_src2, int i_src2,
    int lcu_width_c, int lcu_height_c, int bit_depth, int type, int band_num, int band_num_c, const int *ccsao_offset,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
    int start_x_c = lcu_available_left ? 0 : 1;
    int start_y_c = lcu_available_up ? 0 : 1;
    int end_x_c = lcu_width_c;
    int end_y_c = lcu_height_c;

    p_src += (start_y_c << 1) * i_src;
    p_dst += start_y_c * i_dst;
    p_src2 += start_y_c * i_src2;

    int signa, signb, band;
    int th = band_num_c - 1;
    if (band_num <= 2)
    {
        for (int y_c = start_y_c; y_c < end_y_c; y_c++)
        {
#if CCSAO_LINE_BUFFER
            int line_idx = (lcu_available_down && y_c >= lcu_height_c - CCSAO_PAD_ROWS - 1) ? lcu_height_c - y_c - 1 : 0;
#endif
            for (int x_c = start_x_c; x_c < end_x_c; x_c++)
            {
                int  x = x_c << 1;
#if CCSAO_LINE_BUFFER
                pel *col_Y = p_src + x + i_src * ccsao_edge_type_y[0][0][line_idx];
                pel *col_a = p_src + x + i_src * ccsao_edge_type_y[type][0][line_idx] + ccsao_edge_type_x[type][0];
                pel *col_b = p_src + x + i_src * ccsao_edge_type_y[type][1][line_idx] + ccsao_edge_type_x[type][1];
#else
                pel *col_Y = p_src + x;
                pel *col_a = p_src + x + i_src * ccsao_edge_type_y[type][0] + ccsao_edge_type_x[type][0];
                pel *col_b = p_src + x + i_src * ccsao_edge_type_y[type][1] + ccsao_edge_type_x[type][1];
#endif

                signa = calc_diff_range(*col_Y, *col_a, th);
                signb = calc_diff_range(*col_Y, *col_b, th);

                signa = signa * 4 + signb;
                band = (*col_Y * band_num) >> bit_depth;
                band = band * CCSAO_EDGE_NUM + signa;

                p_dst[x_c] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, p_dst[x_c] + ccsao_offset[band]);
            }
            p_src += i_src << 1;
            p_dst += i_dst;
        }
    }
    else
    {
        int bandc = band_num - 2;
        for (int y_c = start_y_c; y_c < end_y_c; y_c++)
        {
#if CCSAO_LINE_BUFFER
            int line_idx = (lcu_available_down && y_c >= lcu_height_c - CCSAO_PAD_ROWS - 1) ? lcu_height_c - y_c - 1 : 0;
#endif
            for (int x_c = start_x_c; x_c < end_x_c; x_c++)
            {
                int  x = x_c << 1;
#if CCSAO_LINE_BUFFER
                pel *col_Y = p_src + x + i_src * ccsao_edge_type_y[0][0][line_idx];
                pel *col_a = p_src + x + i_src * ccsao_edge_type_y[type][0][line_idx] + ccsao_edge_type_x[type][0];
                pel *col_b = p_src + x + i_src * ccsao_edge_type_y[type][1][line_idx] + ccsao_edge_type_x[type][1];
#else
                pel *col_Y = p_src + x;
                pel *col_a = p_src + x + i_src * ccsao_edge_type_y[type][0] + ccsao_edge_type_x[type][0];
                pel *col_b = p_src + x + i_src * ccsao_edge_type_y[type][1] + ccsao_edge_type_x[type][1];
#endif

                pel *col_C = p_src2 + x_c;

                signa = calc_diff_range(*col_Y, *col_a, th);
                signb = calc_diff_range(*col_Y, *col_b, th);

                signa = signa * 4 + signb;
                int  band = (*col_C * bandc) >> bit_depth;
                band = band * CCSAO_EDGE_NUM + signa;

                p_dst[x_c] = (pel)COM_CLIP3(0, (1 << bit_depth) - 1, p_dst[x_c] + ccsao_offset[band]);
            }
            p_src += i_src << 1;
            p_dst += i_dst;
            p_src2 += i_src2;
        }
    }
}

#endif

void decide_ccsao_func_pointer(CCSAO_FUNC_POINTER *ccsao_func_ptr)
{
    {
        ccsao_func_ptr->ccsao_on_block_for_chroma = ccsao_on_block_for_chroma;
    }
}

#if CCSAO_ENHANCEMENT
int com_malloc_4d_ccsao_statdata(CCSAO_STAT_DATA *****array4D, int num_LCU, int num_band, int num_band_c, int num_type)
{
    int i, j, k;
    if (((*array4D) = (CCSAO_STAT_DATA ****)calloc(num_LCU, sizeof(CCSAO_STAT_DATA ***))) == NULL)
    {
        printf("MALLOC FAILED: get_mem4DCCSAOstatdata: array4D");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_LCU; i++)
    {
        if ((*(*array4D + i) = (CCSAO_STAT_DATA ***)calloc(num_band, sizeof(CCSAO_STAT_DATA **))) == NULL)
        {
            printf("MALLOC FAILED: get_mem3DCCSAOstatdata: array3D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < num_band; j++)
        {
            if ((*(*(*array4D + i) + j) = (CCSAO_STAT_DATA **)calloc(num_band_c, sizeof(CCSAO_STAT_DATA *))) == NULL)
            {
                printf("MALLOC FAILED: get_mem2DCCSAOstatdata: array2D");
                assert(0);
                exit(-1);
            }
            for (k = 0; k < num_band_c; k++)
            {
                if ((*(*(*(*array4D + i) + j) + k) = (CCSAO_STAT_DATA *)calloc(num_type, sizeof(CCSAO_STAT_DATA))) == NULL)
                {
                    printf("MALLOC FAILED: get_mem1DCCSAOstatdata: array1D");
                    assert(0);
                    exit(-1);
                }
            }
        }
    }
    return num_LCU * num_band * num_band_c * num_type * sizeof(CCSAO_STAT_DATA);
}

void com_free_4d_ccsao_statdata(CCSAO_STAT_DATA *****array4D, int num_LCU, int num_band, int num_band_c)
{
    int i, j, k;
    if (*array4D)
    {
        for (i = 0; i < num_LCU; i++)
        {
            if ((*array4D)[i])
            {
                for (j = 0; j < num_band; j++)
                {
                    if ((*array4D)[i][j])
                    {
                        for (k = 0; k < num_band_c; k++)
                        {
                            if ((*array4D)[i][j][k])
                            {
                                free((*array4D)[i][j][k]);
                                (*array4D)[i][j][k] = NULL;
                            }
                        }
                        free((*array4D)[i][j]);
                        (*array4D)[i][j] = NULL;
                    }
                }
                free((*array4D)[i]);
                (*array4D)[i] = NULL;
            }
        }
        free(*array4D);
        *array4D = NULL;
    }
}
#else
int com_malloc_3d_ccsao_statdata(CCSAO_STAT_DATA ****array3D, int num_LCU, int num_band, int num_type)
{
    int i, j;
    if (((*array3D) = (CCSAO_STAT_DATA ** *)calloc(num_LCU, sizeof(CCSAO_STAT_DATA **))) == NULL)
    {
        printf("MALLOC FAILED: get_mem3DCCSAOstatdata: array3D");
        assert(0);
        exit(-1);
    }
    for (i = 0; i < num_LCU; i++)
    {
        if ((*(*array3D + i) = (CCSAO_STAT_DATA **)calloc(num_band, sizeof(CCSAO_STAT_DATA *))) == NULL)
        {
            printf("MALLOC FAILED: get_mem2DCCSAOstatdata: array2D");
            assert(0);
            exit(-1);
        }
        for (j = 0; j < num_band; j++)
        {
            if ((*(*(*array3D + i) + j) = (CCSAO_STAT_DATA *)calloc(num_type, sizeof(CCSAO_STAT_DATA))) == NULL)
            {
                printf("MALLOC FAILED: get_mem1DCCSAOstatdata: arrayD");
                assert(0);
                exit(-1);
            }
        }
    }
    return num_LCU * num_band * num_type * sizeof(CCSAO_STAT_DATA);
}

void com_free_3d_ccsao_statdata(CCSAO_STAT_DATA ****array3D, int num_LCU, int num_band)
{
    int i, j;
    if (*array3D)
    {
        for (i = 0; i < num_LCU; i++)
        {
            if ((*array3D)[i])
            {
                for (j = 0; j < num_band; j++)
                {
                    if ((*array3D)[i][j])
                    {
                        free((*array3D)[i][j]);
                        (*array3D)[i][j] = NULL;
                    }
                }
                free((*array3D)[i]);
                (*array3D)[i] = NULL;
            }
        }
        free(*array3D);
        *array3D = NULL;
    }
}
#endif

void copy_frame_for_ccsao(COM_PIC *pic_dst, COM_PIC *pic_src, int comp)
{
    pel *src        = comp == Y_C ? pic_src->y : comp == U_C ? pic_src->u : pic_src->v;
    pel *dst        = comp == Y_C ? pic_dst->y : comp == U_C ? pic_dst->u : pic_dst->v;
    int  src_stride = comp == Y_C ? pic_src->stride_luma : pic_src->stride_chroma; 
    int  dst_stride = comp == Y_C ? pic_dst->stride_luma : pic_dst->stride_chroma;
    int  pic_width  = comp == Y_C ? pic_src->width_luma  : pic_src->width_chroma;
    int  pic_height = comp == Y_C ? pic_src->height_luma : pic_src->height_chroma;

    for (int y = 0; y < pic_height; y++)
    {
        memcpy(dst, src, sizeof(pel) * pic_width);
        dst += dst_stride;
        src += src_stride;
    }
}

void ccsao_on_block(COM_INFO *info, COM_PIC *pic_rec,
#if CCSAO_ENHANCEMENT
    COM_PIC *pic_ccsao[2],
#else
    COM_PIC *pic_ccsao,
#endif
    CCSAO_BLK_PARAM *ccsao_param, CCSAO_FUNC_POINTER *ccsao_func_ptr, int comp, int x_c, int y_c, int lcu_width_c, int lcu_height_c, int bit_depth, int lcu_pos,
    int lcu_available_left, int lcu_available_right, int lcu_available_up, int lcu_available_down, int lcu_available_upleft, int lcu_available_upright, int lcu_available_leftdown, int lcu_available_rightdwon)
{
#if CCSAO_ENHANCEMENT
    pel *src = pic_ccsao[0]->y;
    pel *src2 = comp == U_C - 1 ? pic_ccsao[0]->u : pic_ccsao[0]->v;
    pel *dst = comp == U_C - 1 ? pic_rec->u : pic_rec->v;
    int  src_stride = pic_ccsao[0]->stride_luma;
    int  src_stride2 = pic_ccsao[0]->stride_chroma;
    int  dst_stride = pic_rec->stride_chroma;
    src += (y_c << 1) * src_stride + (x_c << 1);
    src2 += y_c * src_stride2 + x_c;
    dst += y_c * dst_stride + x_c;
#else
    pel *src        = pic_ccsao->y;
    pel *dst        = comp == U_C-1 ? pic_rec->u : pic_rec->v;
    int  src_stride = pic_ccsao->stride_luma;
    int  dst_stride = pic_rec->stride_chroma;
    src += (y_c << 1) * src_stride + (x_c << 1);
    dst +=  y_c       * dst_stride +  x_c;
#endif

#if CCSAO_ENHANCEMENT
    int  set        = info->pic_header.ccsao_lcu_ctrl[comp] ? ccsao_param[comp].lcu_flag[lcu_pos] - 1 : 0;
    int  type       = info->pic_header.ccsao_type      [comp][set];
    int  band_num   = info->pic_header.ccsao_band_num  [comp][set];
    int  band_num_c = info->pic_header.ccsao_band_num_c[comp][set];
    int *offset     = ccsao_param[comp].offset[set];

#if ECCSAO
    int class_type = info->pic_header.ccsao_class_type[comp][set];
    if (class_type == 0)
        ccsao_func_ptr->ccsao_on_block_for_chroma = ccsao_on_block_for_chroma;
    else
        ccsao_func_ptr->ccsao_on_block_for_chroma = ccsao_on_block_for_chroma_edge;
#endif

    assert(set      >= 0 && set      <  CCSAO_SET_NUM);
#else
    int  type       = info->pic_header.ccsao_type    [comp];
    int  band_num   = info->pic_header.ccsao_band_num[comp];
    int *offset     = ccsao_param[comp].offset;
#endif

#if ECCSAO
    if (class_type)
    {
        assert(type >= 0 && type < CCSAO_EDGE_TYPE);
        assert(band_num >= 1 && band_num <= CCSAO_EDGE_BAND_NUM_Y + CCSAO_EDGE_BAND_NUM_C);
#if CCSAO_ENHANCEMENT
        assert(band_num_c >= 1 && band_num_c <= CCSAO_QUAN_NUM);
#endif
    }
    else
    {
        assert(type >= 0 && type < CCSAO_TYPE_NUM);
        assert(band_num >= 1 && band_num <= CCSAO_BAND_NUM);
#if CCSAO_ENHANCEMENT
        assert(band_num_c >= 1 && band_num_c <= CCSAO_BAND_NUM_C);
#endif
    }
#else
    assert(type >= 0 && type < CCSAO_TYPE_NUM);
    assert(band_num >= 1 && band_num <= CCSAO_BAND_NUM);
#if CCSAO_ENHANCEMENT
    assert(band_num_c >= 1 && band_num_c <= CCSAO_BAND_NUM_C);
#endif
#endif

    ccsao_func_ptr->ccsao_on_block_for_chroma(dst, dst_stride, src, src_stride,
#if CCSAO_ENHANCEMENT
        src2, src_stride2,
#endif
        lcu_width_c, lcu_height_c, bit_depth, type, band_num,
#if CCSAO_ENHANCEMENT
        band_num_c,
#endif
        offset,
        lcu_available_left, lcu_available_right, lcu_available_up, lcu_available_down, lcu_available_upleft, lcu_available_upright, lcu_available_leftdown, lcu_available_rightdwon);
}

void ccsao_on_frame(COM_INFO *info, COM_MAP *map, COM_PIC *pic_rec,
#if CCSAO_ENHANCEMENT
    COM_PIC *pic_ccsao[2],
#else
    COM_PIC *pic_ccsao,
#endif
    CCSAO_BLK_PARAM *ccsao_param, CCSAO_FUNC_POINTER *ccsao_func_ptr)
{
    int pic_width_c     = info->pic_width  >> 1;
    int pic_height_c    = info->pic_height >> 1;
    int log2_max_cuwh_c = info->log2_max_cuwh - 1;
    int bit_depth     = info->bit_depth_internal;
    int lcu_width_c, lcu_height_c;
    
    decide_ccsao_func_pointer(ccsao_func_ptr);

    for (int comp = U_C-1; comp < N_C-1; comp++)
    {
        if (!info->pic_header.pic_ccsao_on[comp])
        {
            continue;
        }

        for (int y_c = 0; y_c < pic_height_c; y_c += lcu_height_c)
        {
            lcu_height_c = min(1 << log2_max_cuwh_c, pic_height_c - y_c);
            for (int x_c = 0; x_c < pic_width_c; x_c += lcu_width_c)
            {
                lcu_width_c = min(1 << log2_max_cuwh_c, pic_width_c - x_c);
                int x_c_in_lcu = x_c >> log2_max_cuwh_c;
                int y_c_in_lcu = y_c >> log2_max_cuwh_c;
                int lcu_pos = x_c_in_lcu + y_c_in_lcu * info->pic_width_in_lcu;
                int is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail;

                if (!ccsao_param[comp].lcu_flag[lcu_pos])
                {
                    continue;
                }

                // reuse ESAO U V boundary check
                check_boundary_available_for_esao(info, map, y_c, x_c, lcu_height_c, lcu_width_c, comp+1,
                    &is_left_avail, &is_right_avail, &is_above_avail, &is_below_avail, &is_above_left_avail, &is_above_right_avail, &is_below_left_avail, &is_below_right_avail, 1);
                ccsao_on_block(info, pic_rec, pic_ccsao, ccsao_param, ccsao_func_ptr, comp, x_c, y_c, lcu_width_c, lcu_height_c, bit_depth, lcu_pos,
                    is_left_avail, is_right_avail, is_above_avail, is_below_avail, is_above_left_avail, is_above_right_avail, is_below_left_avail, is_below_right_avail);
            }
        }
    }
}
#endif
