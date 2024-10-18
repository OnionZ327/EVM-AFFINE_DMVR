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

#ifndef _COM_RECON_H_
#define _COM_RECON_H_


void com_recon(PART_SIZE part_size, s16 *coef, pel *pred, int(*is_coef)[N_C], int plane, int cu_width, int cu_height, int s_rec, pel *rec, int bit_depth
#if SBT
    , u8 sbt_info
#endif
);

void com_recon_yuv(PART_SIZE part_size, int x, int y, int cu_width, int cu_height, s16 coef[N_C][MAX_CU_DIM], pel pred[N_C][MAX_CU_DIM], int (*num_nz_coef)[N_C], COM_PIC *pic, CHANNEL_TYPE channel, int bit_depth
#if SBT
    , u8 sbt_info
#endif
#if INTER_CCNPM
    , COM_MODE* mod_info_curr, int pic_width_in_scu, int pic_height_in_scu, u32* map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
    , const int max_cu_height
#endif
);

#if USE_SP
#if ISC_RSD
void derive_rsd_string(COM_MODE* mod_info_curr, int cu_width_log2, int cu_height_log2, COM_SP_INFO* derived_rsd_strinfo);
void rsd_recon_yuv(COM_MODE* mod_info_curr, int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC* pic, CHANNEL_TYPE channel
    , int pic_width, int pic_height, int pic_width_in_scu, u32* map
    , int ctu_log2_size, COM_SP_INFO* derived_rsd_str_info, u16 derived_rsd_str_no);
#endif

void sp_recon_yuv(COM_MODE *mod_info_curr, int x, int y, int cu_width, int cu_height, COM_PIC *pic, CHANNEL_TYPE channel
    , int m_pic_width, int m_pic_height, int pic_width_in_scu, u32* map
    , int ctulog2size
);
void sp_cs2_recon_yuv(COM_MODE *mod_info_curr, int x, int y, int cu_width_log2, int cu_height_log2, COM_PIC *pic, pel(*dpb_evs)[MAX_SRB_PRED_SIZE]
    , u8 tree_status
    , int ctu_size
);
#endif


#endif /* _COM_RECON_H_ */
