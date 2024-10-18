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

#include "enc_sp.h"
#if USE_SP
#include <string.h>
#include <math.h>
#include <vector>
#include <limits> 
using namespace std;
#include <cassert>
pel   g_cluster[N_C][MAX_SRB_SIZE];
pel   g_cluster_pred[N_C][MAX_SRB_SIZE];
int*  g_run_golomb_groups;
int   g_ui_cluster_size = 0;
int   g_ui_cluster_size_pred = 0;

bool operator == (const COM_SP_PIX& _left, const COM_SP_PIX& _right)
{
    return (bool)((_left.Y == _right.Y) && (_left.U == _right.U) && (_left.V == _right.V));
}

bool operator != (const COM_SP_PIX& _left, const COM_SP_PIX& _right)
{
    return (bool)!(_left == _right);
}

string_prediction::string_prediction(COM_SP_INPUT* input, u8 is_encoding)
{    
    m_max_cu_width            = input->max_cu_width;
    m_max_cu_height           = input->max_cu_height;
    m_pic_width               = input->img_width;
    m_pic_height              = input->img_height;
    m_is_encoding              = is_encoding;
    m_init_TBC_table          = -1;
    this->input               = &iptemp;
    this->input->max_cu_width = input->max_cu_width;
    this->input->max_cu_height = input->max_cu_height;
    this->input->img_width    = input->img_width;
    this->input->img_height   = input->img_height;
    this->input->sample_bit_depth = input->sample_bit_depth;
    this->input->chroma_format = input->chroma_format;    
    this->input->y_stride     = input->y_stride;
    this->input->c_stride     = input->c_stride;
    this->input->recy_stride  = input->recy_stride;    
    this->input->recc_stride  = input->recc_stride;
    if ( m_is_encoding )
    {
        //memory allocation
        m_dict = new COM_SP_POS*[m_pic_height];
        for (int i = 0; i < m_pic_height; i++)
        {
            m_dict[i] = new COM_SP_POS[m_pic_width];
        }
        m_ppp_dict_status = new SP_HASH_STATUS***[MAX_CU_DEPTH];
        for (int i = 0; i < MAX_CU_DEPTH; i++)
        {
            m_ppp_dict_status[i] = new SP_HASH_STATUS**[MAX_CU_DEPTH];
            for (int j = 0; j < MAX_CU_DEPTH; j++) 
            {
                m_ppp_dict_status[i][j] = new SP_HASH_STATUS*[HS_CTX_NUM];
                for (int k = 0; k < HS_CTX_NUM; k++)
                {
                    m_ppp_dict_status[i][j][k] = new SP_HASH_STATUS;
                }
            }
        }
        //initialize
        for (int i = 0; i < SP_HASH_SIZE; i++)
        {
            m_hash_table[i].x = -1;
            m_hash_table[i].y = -1;
        }
        for (int i = 0; i < m_pic_height; i++)
        {
            for (int j = 0; j < m_pic_width; j++)
            {
                m_dict[i][j].x = -1;
                m_dict[i][j].y = -1;
            }
        }
        for (int i = 0; i < MAX_CU_DEPTH; i++)
        {
            for (int j = 0; j < MAX_CU_DEPTH; j++) 
            {
                for (int k = 0; k < HS_CTX_NUM; k++)
                {
                    for (int l = 0; l < SP_HASH_SIZE; l++)
                    {
                        m_ppp_dict_status[i][j][k]->m_hash_table[l].x = -1;
                        m_ppp_dict_status[i][j][k]->m_hash_table[l].y = -1;
                    }
                    for (int h = 0; h < MAX_CU_SIZE; h++)
                    {
                        for (int w = 0; w < MAX_CU_SIZE; w++)
                        {
                            m_ppp_dict_status[i][j][k]->m_dict[h][w].x = -1;
                            m_ppp_dict_status[i][j][k]->m_dict[h][w].y = -1;
                        }
                    }
                }
            }
        }
        get_sp_hashvalue = &string_prediction::get_sp_hash_value_lossy;
        g_run_golomb_groups = new int[32 * 32];
        m_b_major = new u8[MAX_CU_SIZE*MAX_CU_SIZE];
        m_c_match_index = new u8[MAX_CU_SIZE*MAX_CU_SIZE];
        m_p_CQ_pixbuffer = new COM_SP_PIX[MAX_CU_SIZE*MAX_CU_SIZE];
        m_hi_ref_UmPix = new COM_SP_PIX[MAX_SRB_SIZE];
        m_hi_ref_UmPix_cnt = new int[MAX_SRB_SIZE];
        m_c_index_block = new u8[MAX_CU_SIZE*MAX_CU_SIZE];
        {
            //2D memory allocation
            m_ind_error = new unsigned int*[MAX_CU_SIZE * MAX_CU_SIZE];
            for (int i = 0; i < MAX_CU_SIZE * MAX_CU_SIZE; i++)
            {
                m_ind_error[i] = new unsigned int[MAX_SSRB_SIZE + 1];
            }
        }
        init_TBC_table(input->sample_bit_depth);
        for (int ch = 0; ch < 3; ch++)
        {
            set_bit_depth(ch, input->sample_bit_depth);
        }
        //
    }
    else
    {
        m_dict = NULL;
        m_ppp_dict_status = NULL;
    }
    
    for ( int i=0; i<3; i++ )
    {
        m_p_pel_org[i] = NULL;
        m_p_pel_rec[i] = NULL;
    }    
}

void* get_sp_instance(COM_SP_INPUT* input, u8 isEncoding)
{
    return new string_prediction(input, isEncoding);
}

void release_sp_instance(void* sp_encoder)
{
    if (sp_encoder)
    {
        delete ((string_prediction*)sp_encoder);
        sp_encoder = NULL;
    }
}

void sm_pic_reset(void* sp_encoder, pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec, pel *imgV_rec)
{
    if (sp_encoder)
    {
        ((string_prediction*)sp_encoder)->reset(imgY_org, imgU_org, imgV_org, imgY_rec, imgU_rec, imgV_rec);
    }
}

u8 sm_mode_rdcost(void* sp_encoder, COM_SP_CODING_UNIT * cur_sp_info, double* min_rdcost, double* distorion
    , ENC_CTX *ctx
)
{
    if (sp_encoder == NULL)
    {
        return 0;
    }
    return ((string_prediction*)sp_encoder)->rdcost_sp_mode(cur_sp_info, min_rdcost, distorion
        , ctx
    );
}

void sp_lcu_hash_save(void* sp_encoder, int x_lcu_start, int y_lcu_start)
{
    if (sp_encoder == NULL)
    {
        return;
    }
    for (int i = 0; i < MAX_CU_DEPTH; i++)
    {
        for (int j = 0; j < MAX_CU_DEPTH; j++) 
        {
            ((string_prediction*)sp_encoder)->save(((string_prediction*)sp_encoder)->m_ppp_dict_status[i][j][HS_CURR_BEST], x_lcu_start, y_lcu_start);
        }
    }
}
void sp_lcu_hash_restore(void* sp_encoder, int x_lcu_start, int y_lcu_start, int ctulog2size)
{
    ((string_prediction*)sp_encoder)->restore(((string_prediction*)sp_encoder)->m_ppp_dict_status[ctulog2size - 2][ctulog2size - 2][HS_NEXT_BEST], x_lcu_start, y_lcu_start);
}
void sp_cu_hash_copy(void* sp_encoder, int dst_w, int dst_h, int dst_status, int src_w, int src_h, int src_status)
{
    ((string_prediction*)sp_encoder)->m_ppp_dict_status[src_w][src_h][src_status]->save(((string_prediction*)sp_encoder)->m_ppp_dict_status[dst_w][dst_h][dst_status]);
}

u8 sp_cu_hash_update(void* sp_encoder, int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start)
{
    if (sp_encoder == NULL)
    {
        return FALSE;
    }
    return ((string_prediction*)sp_encoder)->sp_hash_update(cu_width_log2, cu_height_log2, x_cu_start, y_cu_start);
}

u8 sp_is_hash_updated(u8 *map_usp, int x, int y, int cu_width, int cu_height, int pic_width_in_scu)
{
    u8 is_sp_pix_completed = TRUE;
    u8 *map = map_usp;
    int idx = 0;
    for (int j = 0; j < cu_height >> MIN_CU_LOG2; j++)
    {
        for (int i = 0; i < cu_width >> MIN_CU_LOG2; i++)
        {
            is_sp_pix_completed &= map[idx + i];
            if (is_sp_pix_completed == FALSE)
            {
                return is_sp_pix_completed;
            }
        }
        idx += cu_width >> MIN_CU_LOG2;
    }
    return is_sp_pix_completed;
}

u8 get_adaptive_sp_flag(void* sp_encoder)
{
    if (sp_encoder == NULL)
    {
        return FALSE;
    }
    return (u8)((string_prediction*)sp_encoder)->sp_hash_sp_judge(0, 0);
}

void sp_save_last_srb(void* sp_encoder,ENC_CU_DATA *cu_data, int cup)
{
    ((string_prediction*)sp_encoder)->save_last_srb(cu_data, cup);
}
string_prediction::~string_prediction()
{
    if (m_is_encoding)
    {
        if (m_dict != NULL)
        {
            for (int i = 0; i < m_pic_height; i++)
            {
                if (m_dict[i] != NULL)
                {
                    delete[] m_dict[i];
                    m_dict[i] = NULL;
                }
            }
            delete[] m_dict;
            m_dict = NULL;
        }
        if (m_trunc_bin_bits != NULL)
        {
            for (int i = 0; i < m_symbol_size; i++)
            {
                if (m_trunc_bin_bits[i])
                {
                    x_free(m_trunc_bin_bits[i]);//delete[] m_trunc_bin_bits[i];
                    m_trunc_bin_bits[i] = NULL;
                }
            }
            x_free(m_trunc_bin_bits);//delete[] m_trunc_bin_bits;
            m_trunc_bin_bits = NULL;
        }
    } //m_is_encoding
    if (m_ppp_dict_status != NULL)
    {
        for (int i = 0; i < MAX_CU_DEPTH; i++)
        {
            if (m_ppp_dict_status[i] != NULL)
            {
                for (int j = 0; j < MAX_CU_DEPTH; j++)
                {
                    if (m_ppp_dict_status[i][j] != NULL)
                    {
                        for (int k = 0; k < HS_CTX_NUM; k++)
                        {
                            if (m_ppp_dict_status[i][j][k] != NULL)
                            {
                                delete m_ppp_dict_status[i][j][k];
                                m_ppp_dict_status[i][j][k] = NULL;
                            }
                        }
                        delete m_ppp_dict_status[i][j];
                        m_ppp_dict_status[i][j] = NULL;
                    }
                }
                delete[] m_ppp_dict_status[i];
                m_ppp_dict_status[i] = NULL;
            }
        }
        delete[] m_ppp_dict_status;
        m_ppp_dict_status = NULL;
    }
    //delete SRB status
    if (g_run_golomb_groups != NULL)
    {
        delete[] g_run_golomb_groups;
        g_run_golomb_groups = NULL;
    }
    if (m_b_major != NULL)
    {
        delete[] m_b_major;
        m_b_major = NULL;
    }
    if (m_c_match_index != NULL)
    {
        delete[] m_c_match_index;
        m_c_match_index = NULL;
    }
    if (m_p_CQ_pixbuffer != NULL)
    {
        delete[] m_p_CQ_pixbuffer;
        m_p_CQ_pixbuffer = NULL;
    }
    if (m_hi_ref_UmPix != NULL)
    {
        delete[] m_hi_ref_UmPix;
        m_hi_ref_UmPix = NULL;
    }
    if (m_hi_ref_UmPix_cnt != NULL)
    {
        delete[] m_hi_ref_UmPix_cnt;
        m_hi_ref_UmPix_cnt = NULL;
    }
    if (m_c_index_block != NULL)
    {
        delete[] m_c_index_block;
        m_c_index_block = NULL;
    }
    //2D memory free
    if (m_ind_error != NULL)
    {
        for (int i = 0; i<MAX_CU_SIZE * MAX_CU_SIZE; i++)
        {
            if (m_ind_error[i] != NULL)
            {
                m_ind_error[i] = NULL;
            }
        }
        delete[] m_ind_error;
        m_ind_error = NULL;
    }
}

void string_prediction::reset(pel *imgY_org, pel *imgU_org, pel *imgV_org, pel *imgY_rec, pel *imgU_rec,pel *imgV_rec)
{
    if ( imgY_org != NULL )
    {
        m_p_pel_org[0] = imgY_org;
        m_p_pel_org[1] = imgU_org;
        m_p_pel_org[2] = imgV_org;
        m_stride_org[0] = input->y_stride;
        m_stride_org[1] = input->c_stride;
        m_stride_org[2] = input->c_stride;
    }
    if( imgY_rec!=NULL )
    {
        m_p_pel_rec[0] = imgY_rec;
        m_p_pel_rec[1] = imgU_rec;
        m_p_pel_rec[2] = imgV_rec;
        m_stride_rec[0] = input->recy_stride;
        m_stride_rec[1] = input->recc_stride;
        m_stride_rec[2] = input->recc_stride;
    }
}

u16 string_prediction::get_sp_hash_value_lossy(COM_SP_PIX pixelCur)
{
    u8 CRCCalculationBuffer[3];
    u8* p = CRCCalculationBuffer;
    p[0] = static_cast<u8>(pixelCur.Y >> 2);
    p[1] = static_cast<u8>(pixelCur.U >> 2);
    p[2] = static_cast<u8>(pixelCur.V >> 2);
    u16 crc = ((p[0] & 0xff) << 4) | ((p[1] & 0xc0) >> 4) | ((p[2] & 0xc0) >> 6); //822
    return crc;
}

void string_prediction::add_to_hash_table(int x_start_in_pic, int y_start_in_pic, int width, int height, u8 scale_x, u8 scale_y)
{
    COM_SP_PIX cur_pixel;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int current_x = x_start_in_pic + j;
            int current_y = y_start_in_pic + i;
            int lengthMinus1 = 0;
            int startPositionX = current_x - lengthMinus1;
            int startPositionY = current_y;
            if (startPositionX < 0 || startPositionY < 0 || startPositionX >= m_pic_width || startPositionY >= m_pic_height)
            {
                continue;
            }
            int cur_pos = current_y * m_stride_org[0] + current_x;
            int cur_pos_c = (current_y >> scale_y) * m_stride_org[1] + (current_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos]; 
            cur_pixel.U = m_p_pel_org[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org[2][cur_pos_c];
            u16 hashValue = (this->*get_sp_hashvalue)(cur_pixel);
            m_dict[current_y][current_x].x = m_hash_table[hashValue].x;
            m_dict[current_y][current_x].y = m_hash_table[hashValue].y;
            m_hash_table[hashValue].x = startPositionX;
            m_hash_table[hashValue].y = startPositionY;
        }
    }
}

void string_prediction::save(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start)
{
    memcpy(pDictStatus->m_hash_table, m_hash_table, sizeof(COM_SP_POS)*SP_HASH_SIZE);
    int stride = (x_lcu_start + m_max_cu_width) > m_pic_width ? (m_pic_width - x_lcu_start) : m_max_cu_width;
    for (int j = 0; j < m_max_cu_width; j++)
    {
        if ((y_lcu_start + j) < m_pic_height)
        {
            memcpy(pDictStatus->m_dict[j], &m_dict[y_lcu_start + j][x_lcu_start], sizeof(COM_SP_POS)*stride);
        }
    }
}
void string_prediction::restore(SP_HASH_STATUS* pDictStatus, int x_lcu_start, int y_lcu_start)
{
    memcpy(m_hash_table, pDictStatus->m_hash_table, sizeof(COM_SP_POS)*SP_HASH_SIZE);
    int stride = (x_lcu_start + m_max_cu_width) > m_pic_width ? (m_pic_width - x_lcu_start) : m_max_cu_width;
    for (int j = 0; j < m_max_cu_height; j++)
    {
        if ((y_lcu_start + j) < m_pic_height)
        {
            memcpy(&m_dict[y_lcu_start + j][x_lcu_start], pDictStatus->m_dict[j], sizeof(COM_SP_POS)* stride);
        }
    }
}

u8 string_prediction::sp_hash_update(int cu_width_log2, int cu_height_log2, int x_cu_start, int y_cu_start)
{
    int width = 1 << cu_width_log2;
    int height = 1 << cu_height_log2;
    int x_lcu_start = x_cu_start / m_max_cu_width * m_max_cu_width;
    int y_lcu_start = y_cu_start / m_max_cu_width * m_max_cu_width;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    restore(m_ppp_dict_status[cu_width_log2 - 2][cu_height_log2 - 2][HS_CURR_BEST], x_lcu_start, y_lcu_start);
    add_to_hash_table(x_cu_start, y_cu_start, width, height, scale_x, scale_y);
    save(m_ppp_dict_status[cu_width_log2 - 2][cu_height_log2 - 2][HS_NEXT_BEST], x_lcu_start, y_lcu_start);
    return TRUE;
}

void SP_HASH_STATUS::save(SP_HASH_STATUS* p)
{
    memcpy(p->m_hash_table, m_hash_table, sizeof(COM_SP_POS)*SP_HASH_SIZE);
    for (int j = 0; j < MAX_CU_SIZE; j++)
    {
        memcpy(p->m_dict[j], m_dict[j], sizeof(COM_SP_POS)*MAX_CU_SIZE);//ZF
    }
}

u8 string_prediction::sp_hash_sp_judge(int x_start_in_pic, int y_start_in_pic)
{
    int hitratio = 0;
    int non_replic_pix = 0, all_pix = 0, diff_pix = 0;
    COM_SP_PIX cur_pixel;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    int hash_count[SP_HASH_SIZE] = { 0 };
    int long_chain = 0, short_chain = 0;
    for (int i = 0; i < m_pic_height; i ++)
    {
        for (int j = 0; j < m_pic_width; j ++)
        {
            int current_x = x_start_in_pic + j;
            int current_y = y_start_in_pic + i;
            int startPositionX = current_x;
            int startPositionY = current_y;
            if (startPositionX < 0 || startPositionY < 0 || startPositionX >= m_pic_width || startPositionY >= m_pic_height)
            {
                continue;
            }
            int cur_pos = current_y * m_stride_org[0] + current_x;
            int cur_pos_c = (current_y >> scale_y) * m_stride_org[1] + (current_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos];
            cur_pixel.U = m_p_pel_org[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org[2][cur_pos_c];
            u8 CRCCalculationBuffer[3];
            u8 * p = CRCCalculationBuffer;
            p[0] = static_cast<u8>(cur_pixel.Y);
            p[1] = static_cast<u8>(cur_pixel.U);
            p[2] = static_cast<u8>(cur_pixel.V);
            u16 hashValue = ((p[0] & 0xf0) << 4) | ((p[1] & 0xf0)) | ((p[2] & 0xf0) >> 4);
            hash_count[hashValue]++;
            if (hash_count[hashValue]==1)
            {
                non_replic_pix++;
            }
        }
    }
    for (int k = 0; k < SP_HASH_SIZE; k++)
    {
        if (hash_count[k] >= (m_pic_width*m_pic_width >> 11))
        {
            all_pix++;
        }
    }
    for (int k = 0; k < SP_HASH_SIZE-1; k++)
    {
        long_chain = (hash_count[k] >= hash_count[k + 1]) ? hash_count[k] : hash_count[k + 1];
        short_chain = (hash_count[k] < hash_count[k + 1]) ? hash_count[k] : hash_count[k + 1];
        if (((short_chain + 1) << 7) < long_chain)
        {
            diff_pix++;
        }
    }
    if (diff_pix * 1000 / (all_pix+1) > 200)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void  string_prediction::reorder_SRB(COM_SP_CODING_UNIT* p_cur_sp_info, pel *p_srb[3], int ui_pvbuf_size, u8 b_cs2_mode_flag)
{
    pel *p_srb_prev[3] = { p_cur_sp_info->m_srb_prev[0], p_cur_sp_info->m_srb_prev[1], p_cur_sp_info->m_srb_prev[2] };
    unsigned int ui_pvbuf_size_prev = p_cur_sp_info->m_pvbuf_size_prev;
    unsigned int ui_dict_max_size = ui_pvbuf_size;

    m_ui_pvbuf_size = ui_pvbuf_size;
    m_ui_hi_ref_UmPSize = 0;
    pel* p_srb_temp[3];
    for (unsigned int ch = 0; ch < 3; ch++)
    {
        p_srb_temp[ch] = (pel*)x_malloc(pel, MAX_SRB_SIZE);
        for (unsigned int i = 0; i < MAX_SRB_SIZE; i++)
        {
            p_srb_temp[ch][i] = p_srb[ch][i];
        }
    }

    unsigned int ui_idx_prev = 0, ui_idx_curr = 0;
    u8 b_reused = 0;
    u8 *b_predicted, *b_reused_prev;
    b_predicted = (u8*)x_malloc(u8, MAX_SRB_SIZE + 1);
    b_reused_prev = (u8*)x_malloc(u8, MAX_SRB_PRED_SIZE + 1);
    memset(b_predicted, 0, sizeof(u8)*(MAX_SRB_SIZE + 1));
    memset(b_reused_prev, 0, sizeof(u8)*(MAX_SRB_PRED_SIZE + 1));
    unsigned int ui_num_SRB_rceived = ui_dict_max_size, ui_num_SRB_predicted = 0;
    u8 p_used[MAX_SRB_SIZE] = { 0 };
    for (ui_idx_prev = 0; ui_idx_prev < ui_pvbuf_size_prev; ui_idx_prev++)
    {
        b_reused = 0;
        int i_counter = 0;
        for (ui_idx_curr = 0; ui_idx_curr < ui_dict_max_size; ui_idx_curr++)
        {
            i_counter = 0;
            if (p_cur_sp_info->cu_ext)
            {
                if (p_srb_prev[0][ui_idx_prev] == p_srb[0][ui_idx_curr])
                {
                    if (!p_used[ui_idx_curr])
                        i_counter = 1;
                }

                if (i_counter == 1)
                {
                    p_used[ui_idx_curr] = 1;
                    b_reused = 1;
                    break;
                }
            }
            else
            {
                for (unsigned int comp = 0; comp < 3; comp++)
                {
                    if (p_srb_prev[comp][ui_idx_prev] == p_srb[comp][ui_idx_curr])
                    {
                        i_counter++;
                    }
                }
                if (i_counter == 3)
                {
                    b_reused = 1;
                    break;
                }
            }
        }
        b_reused_prev[ui_idx_prev] = b_reused;
        b_predicted[ui_idx_curr] = b_reused;
        if (b_predicted[ui_idx_curr])
        {
            ui_num_SRB_rceived--;
            ui_num_SRB_predicted++;
        }
    }

    ui_idx_curr = 0;
    if (b_cs2_mode_flag)
    {
        m_ui_hi_ref_UmPSize = ui_num_SRB_predicted;
    }
    memset(p_cur_sp_info->m_all_comp_flag, 0, sizeof(u8) * MAX_SRB_SIZE);
    memset(p_cur_sp_info->m_cuS_flag, 0, sizeof(u8) * MAX_SRB_SIZE);
    memset(p_cur_sp_info->m_pv_x, -1, sizeof(s16) * MAX_SRB_SIZE);
    memset(p_cur_sp_info->m_pv_y, -1, sizeof(s16) * MAX_SRB_SIZE);
    memset(p_cur_sp_info->m_dpb_reYonly, 0, sizeof(u8) * MAX_SRB_SIZE);
    memset(p_cur_sp_info->m_dpb_idx, 0, sizeof(u8) * MAX_SRB_SIZE);
    for (unsigned int ui_prev_dx = 0; ui_prev_dx < ui_pvbuf_size_prev; ui_prev_dx++)
    {
        if (b_reused_prev[ui_prev_dx])
        {
            p_cur_sp_info->m_all_comp_flag[ui_idx_curr] = p_cur_sp_info->m_all_comp_pre_flag[ui_prev_dx];
            p_cur_sp_info->m_cuS_flag[ui_idx_curr] = p_cur_sp_info->m_cuS_pre_flag[ui_prev_dx];
            p_cur_sp_info->m_pv_x[ui_idx_curr] = p_cur_sp_info->m_pv_prev_x[ui_prev_dx];
            p_cur_sp_info->m_pv_y[ui_idx_curr] = p_cur_sp_info->m_pv_prev_y[ui_prev_dx];
            p_cur_sp_info->m_dpb_reYonly[ui_idx_curr] = p_cur_sp_info->m_dpb_reYonly_prev[ui_prev_dx];
            p_cur_sp_info->m_dpb_idx[ui_idx_curr] = p_cur_sp_info->m_dpb_idx_prev[ui_prev_dx];
            assert(p_cur_sp_info->m_dpb_reYonly[ui_idx_curr] == 0 || p_cur_sp_info->m_dpb_reYonly[ui_idx_curr] == 1);
            assert(p_cur_sp_info->m_dpb_idx[ui_idx_curr] == 0 || p_cur_sp_info->m_dpb_idx[ui_idx_curr] == 1);
            for (unsigned int comp = 0; comp < 3; comp++)
            {
                p_srb[comp][ui_idx_curr] = p_srb_prev[comp][ui_prev_dx];
            }
            if (b_cs2_mode_flag)
            {
                //initialize the hiRefPixel buffer
                m_hi_ref_UmPix[ui_idx_curr].Y = p_srb[0][ui_idx_curr];
                m_hi_ref_UmPix[ui_idx_curr].U = p_srb[1][ui_idx_curr];
                m_hi_ref_UmPix[ui_idx_curr].V = p_srb[2][ui_idx_curr];
                m_hi_ref_UmPix_cnt[ui_idx_curr] = 0;
            }
            ui_idx_curr++;
        }
    }

    for (unsigned int ui_idx = 0; ui_idx < ui_dict_max_size; ui_idx++)
    {
        if (b_predicted[ui_idx] == 0)
        {
            for (unsigned int comp = 0; comp < 3; comp++)
            {
                p_srb[comp][ui_idx_curr] = p_srb_temp[comp][ui_idx];
            }
            if (p_cur_sp_info->cu_ext)
            {
                p_cur_sp_info->m_cuS_flag[ui_idx_curr] = 1;
            }
            else
            {
                p_cur_sp_info->m_cuS_flag[ui_idx_curr] = 0;
            }
            ui_idx_curr++;
        }
    }
    //  
    p_cur_sp_info->m_pvbuf_size = (u8)ui_pvbuf_size;
    for (int ui_idx = 0; ui_idx < ui_pvbuf_size; ui_idx++)
    {
        p_cur_sp_info->m_srb[0][ui_idx] = p_srb[0][ui_idx];
        p_cur_sp_info->m_srb[1][ui_idx] = p_srb[1][ui_idx];
        p_cur_sp_info->m_srb[2][ui_idx] = p_srb[2][ui_idx];
    }
    //
    for (unsigned int ch = 0; ch < 3; ch++)
    {
        if (p_srb_temp[ch])
        {
            x_free(p_srb_temp[ch]);
            p_srb_temp[ch] = NULL;
        }
    }
    if (b_predicted)
    {
        x_free(b_predicted);
        b_predicted = NULL;
    }
    if (b_reused_prev)
    {
        x_free(b_reused_prev);
        b_reused_prev = NULL;
    }
}

void string_prediction::SRB_predition(COM_SP_CODING_UNIT* p_cur_sp_info, COM_SP_PIX *p_hi_ref_UmP, unsigned int ui_hi_ref_UmPSize, u8* p_reused_prev)
{
    pel *p_pred[3] = { p_cur_sp_info->m_srb_prev[0], p_cur_sp_info->m_srb_prev[1], p_cur_sp_info->m_srb_prev[2] };
    unsigned int ui_pvbuf_size_prev = p_cur_sp_info->m_pvbuf_size_prev;
    //set SRB Reused Flag
    pel p_SRB_temp[3][MAX_SRB_SIZE];
    unsigned int ui_idx_prev = 0, ui_idx_curr = 0;
    u8 b_reused;
    u8 p_predicted[MAX_SRB_SIZE + 1];
    memset(p_predicted, 0, sizeof(p_predicted));
    u8 p_used[MAX_SRB_SIZE] = { 0 };
    //save cur SRB into temp
    for (unsigned int i = 0; i < MAX_SRB_SIZE; i++)
    {
        p_SRB_temp[0][i] = p_hi_ref_UmP[i].Y;
        p_SRB_temp[1][i] = p_hi_ref_UmP[i].U;
        p_SRB_temp[2][i] = p_hi_ref_UmP[i].V;
    }
    //
    for (ui_idx_prev = 0; ui_idx_prev < ui_pvbuf_size_prev; ui_idx_prev++)
    {
        b_reused = 0;
        int i_counter = 0;
        for (ui_idx_curr = 0; ui_idx_curr < ui_hi_ref_UmPSize; ui_idx_curr++)
        {
            i_counter = 0;
            if (p_cur_sp_info->cu_ext)
            {
                if (p_pred[0][ui_idx_prev] == p_SRB_temp[0][ui_idx_curr])
                {
                    if (!p_used[ui_idx_curr])
                        i_counter = 1;
                }
                if (i_counter == 1)
                {
                    p_used[ui_idx_curr] = 1;
                    b_reused = 1;
                    break;
                }
            }
            else
            {
                for (unsigned int ch = 0; ch < 3; ch++)
                {
                    if (p_pred[ch][ui_idx_prev] == p_SRB_temp[ch][ui_idx_curr])
                    {
                        i_counter++;
                    }
                }
                if (i_counter == 3)
                {
                    b_reused = 1;
                    break;
                }
            }
        }
        p_reused_prev[ui_idx_prev] = b_reused;
        p_predicted[ui_idx_curr] = b_reused;
    }


    //reorder SRB
    ui_idx_curr = 0;
    for (ui_idx_prev = 0; ui_idx_prev < ui_pvbuf_size_prev; ui_idx_prev++)
    {
        if (p_reused_prev[ui_idx_prev])
        {
            p_hi_ref_UmP[ui_idx_curr].Y = p_pred[0][ui_idx_prev];
            p_hi_ref_UmP[ui_idx_curr].U = p_pred[1][ui_idx_prev];
            p_hi_ref_UmP[ui_idx_curr].V = p_pred[2][ui_idx_prev];
            ui_idx_curr++;
        }
    }

    for (unsigned int ui_idx = 0; ui_idx < ui_hi_ref_UmPSize; ui_idx++)
    {
        if (p_predicted[ui_idx] == 0)
        {
            p_hi_ref_UmP[ui_idx_curr].Y = p_SRB_temp[0][ui_idx];
            p_hi_ref_UmP[ui_idx_curr].U = p_SRB_temp[1][ui_idx];
            p_hi_ref_UmP[ui_idx_curr].V = p_SRB_temp[2][ui_idx];
            ui_idx_curr++;
        }
    }
}

void string_prediction::save_SRB(COM_SP_CODING_UNIT* p_cur_sp_info)
{
    p_cur_sp_info->m_evs_present_flag = m_hi_ref_flag;
    p_cur_sp_info->m_unpredictable_pixel_present_flag = m_lo_ref_flag;
    //set SRB
    assert(m_ui_hi_ref_UmPSize <= (int)m_max_pvbuf_size);
    p_cur_sp_info->m_pvbuf_size = m_ui_hi_ref_UmPSize;

    u8 *p_reused_prev = (u8 *)x_malloc(u8, MAX_SRB_PRED_SIZE + 1);//new u8[MAX_SRB_PRED_SIZE + 1];
    memset(p_reused_prev, 0, sizeof(u8)*(MAX_SRB_PRED_SIZE + 1));
    {
        //SRB prediction
        SRB_predition(p_cur_sp_info, m_hi_ref_UmPix, m_ui_hi_ref_UmPSize, p_reused_prev);
    }

    for (int ui_idx = 0; ui_idx < m_srb_max_pred_size; ui_idx++)
    {
        p_cur_sp_info->m_pvbuf_reused_flag[ui_idx] = p_reused_prev[ui_idx];
    }
    for (int ui_idx = 0; ui_idx < m_max_pvbuf_size; ui_idx++)
    {
        p_cur_sp_info->m_srb[0][ui_idx] = m_hi_ref_UmPix[ui_idx].Y;
        p_cur_sp_info->m_srb[1][ui_idx] = m_hi_ref_UmPix[ui_idx].U;
        p_cur_sp_info->m_srb[2][ui_idx] = m_hi_ref_UmPix[ui_idx].V;
    }
    if (p_reused_prev)
    {
        x_free(p_reused_prev);//delete[] p_reused_prev;
    }
    p_reused_prev = NULL;
}

void string_prediction::init_TBC_table(u8 bit_depth)
{
    if (m_init_TBC_table != -1)
    {
        return;
    }
    else
    {
        m_init_TBC_table = 1;
    }
    //
    m_max_symbol_size = (1 << bit_depth) + 1; //num of ui_symbol, max = 256, so size 257
    m_symbol_size = m_max_symbol_size - 1; // ui_symbol is in the range of [0, 255], size = 256

    m_trunc_bin_bits = (unsigned short**)x_malloc(unsigned short*, m_symbol_size);//new unsigned short*[m_symbol_size];
    for (int i = 0; i < m_symbol_size; i++)
    {
        m_trunc_bin_bits[i] = (unsigned short*)x_malloc(unsigned short, m_max_symbol_size);//new unsigned short[m_max_symbol_size];
        memset(m_trunc_bin_bits[i], 0, sizeof(unsigned short)*m_max_symbol_size);
    }

    for (int i = 0; i < m_max_symbol_size; i++)
    {
        for (int j = 0; j < i; j++)
        {
            m_trunc_bin_bits[j][i] = get_trunc_bin_bits(j, i);
        }
    }
    unsigned int group = 0, k = 0, j, ui_total = 32 * 32;
    while (k < ui_total)
    {
        if (group < 2)
        {
            g_run_golomb_groups[k] = group + 1;
            k++; group++;
            if (k >= ui_total)
            {
                break;
            }
        }
        else
        {
            for (j = 0; j < ((unsigned int)(1 << (group - 1))); j++)
            {
                g_run_golomb_groups[k + j] = 2 * group;
            }
            k += (1 << (group - 1));
            group++;
        }
        if (k >= ui_total)
        {
            break;
        }
    }
}

unsigned int  string_prediction::find_candidate_SRB_predictors(int srb_ind_best[], COM_SP_CODING_UNIT* p_cur_sp_info, pel *Srb[3], pel* p_pred[3], unsigned int ui_pvbuf_size_temp, unsigned int max_no_pred_ind)
{
    unsigned int ui_abs_error = 0, ui_min_error;
    unsigned int srb_pred_error[MAX_SRB_PRED_SIZE];

    for (int t = 0; t < p_cur_sp_info->m_pvbuf_size_prev; t++)
    {
        ui_abs_error = 0;
        int i_temp = p_pred[0][t] - Srb[0][ui_pvbuf_size_temp];
        ui_abs_error += (i_temp * i_temp) >> ((m_bit_depth[0] - 8) << 1);
        if (!p_cur_sp_info->cu_ext)
        {
        i_temp = p_pred[1][t] - Srb[1][ui_pvbuf_size_temp];
        ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[1] - 8) << 1)) * UV_WEIGHT);
        i_temp = p_pred[2][t] - Srb[2][ui_pvbuf_size_temp];
        ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[1] - 8) << 1)) * UV_WEIGHT);
        }
        srb_pred_error[t] = ui_abs_error;
        srb_ind_best[t] = t;
    }

    unsigned int best_ind;
    for (unsigned int t = 0; t < max_no_pred_ind; t++)
    {
        best_ind = t;
        ui_min_error = srb_pred_error[t];

        for (unsigned int l = t + 1; l < p_cur_sp_info->m_pvbuf_size_prev; l++)
        {
            if (srb_pred_error[l] < ui_min_error)
            {
                best_ind = l;
                ui_min_error = srb_pred_error[l];
            }
        }

        swap(srb_pred_error[best_ind], srb_pred_error[t]);
        swap(srb_ind_best[best_ind], srb_ind_best[t]);
    }
    unsigned int max_pred_check = ((unsigned int)p_cur_sp_info->m_pvbuf_size_prev < max_no_pred_ind) ? (unsigned int)p_cur_sp_info->m_pvbuf_size_prev : max_no_pred_ind;

    return(max_pred_check);
}

unsigned int string_prediction::get_trunc_bin_bits(unsigned int ui_symbol, unsigned int ui_max_symbol)
{
    unsigned int ui_idx_code_bit = 0;
    unsigned int ui_thresh;
    if (ui_max_symbol > 256)
    {
        unsigned int ui_thresh_val = 1 << 8;
        ui_thresh = 8;
        while (ui_thresh_val <= ui_max_symbol)
        {
            ui_thresh++;
            ui_thresh_val <<= 1;
        }
        ui_thresh--;
    }
    else
    {
        ui_thresh = g_sp_tb_tbl[ui_max_symbol];
    }

    unsigned int ui_val = 1 << ui_thresh;
    assert(ui_val <= ui_max_symbol);
    assert((ui_val << 1) > ui_max_symbol);
    assert(ui_symbol < ui_max_symbol);
    unsigned int b = ui_max_symbol - ui_val;
    assert(b < ui_val);
    if (ui_symbol < ui_val - b)
    {
        ui_idx_code_bit = ui_thresh;
    }
    else
    {
        ui_idx_code_bit = ui_thresh + 1;
    }
    return ui_idx_code_bit;
}

double string_prediction::calc_pix_pred_RD(COM_SP_CODING_UNIT* p_cur_sp_info, pel pOrg[3]/*, TComRdCost *pcCost*/, unsigned int *error)
{
    double rd_cost = 0;
    unsigned int rd_error = 0;
    for (unsigned int ch = 0; ch < 3; ch++)
    {
        int bitDepth = m_bit_depth[(ch > 0 ? CHANNEL_TYPE_CHROMA : CHANNEL_TYPE_LUMA)];
        rd_cost += p_cur_sp_info->lamda * bitDepth;
    }

    *error = rd_error;
    rd_cost += (*error);
    return (rd_cost);
}

void  string_prediction::derive_srblossy(COM_SP_CODING_UNIT* p_cur_sp_info, pel *Srb[3], pel* p_src[3], int ui_width, int ui_height, int &ui_pvbuf_size)
{
    int  i_errorlimit = get_srb_errLimit();//g_uhSRBQuant[int(p_cur_sp_info->getQP(0))];
    unsigned int ui_total_size = ui_height * ui_width;
    SORTING_ELEMENT *ps_list = new SORTING_ELEMENT[ui_total_size];
    SORTING_ELEMENT s_element;
    unsigned int ui_dict_max_size = MAX_SRB_SIZE;
  
    if (ui_width == 8 && ui_height == 8)
    {
        ui_dict_max_size = EVS_PV_MAX;
    }
    if ((ui_width == 4 && ui_height == 16) || (ui_width == 16 && ui_height == 4))
    {
        ui_dict_max_size = EVS_PV_MAX;
    }
    if (ui_width == 4 && ui_height == 4)
    {
        ui_dict_max_size = EVS_PV_4;
    }
    if ((ui_width == 4 && ui_height == 8) || (ui_width == 8 && ui_height == 4))
    {
        ui_dict_max_size = EVS_PV_8;
    }

    SORTING_ELEMENT *p_list_sort = new SORTING_ELEMENT[ui_dict_max_size + 1];
    unsigned int ui_idx = 0;
    unsigned int ui_pos;
    int last = -1;
    unsigned int scale_x = 0; //444, scale=0; 422 or 420 or 400, scaleX=1;
    unsigned int scale_y = 0; //422 or 444, scale=0; 420 or 400, scaleY=1;
    SORTING_ELEMENT *ps_list_histogram = new SORTING_ELEMENT[ui_total_size];
    SORTING_ELEMENT *ps_initial = new SORTING_ELEMENT[ui_total_size];
    int ui_his_idx = 0;

    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            ui_pos = ui_y * ui_width + ui_x;
            unsigned int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
            s_element.set_all(p_src[0][ui_pos], p_src[1][ui_pos_c], p_src[2][ui_pos_c]);

            int i = 0;
            for (i = ui_his_idx - 1; i >= 0; i--)
            {
                if (ps_list_histogram[i].equal_data(s_element))
                {
                    ps_list_histogram[i].add_element(s_element);
                    break;
                }
            }
            if (i == -1)
            {
                ps_list_histogram[ui_his_idx].copy_data_from(s_element);
                ps_list_histogram[ui_his_idx].ui_cnt = 1;
                ui_his_idx++;
            }
        }
    }

    int ui_his_cnt, ui_max_idx;
    int ui_limit = ((ui_height << 2)*i_errorlimit) >> 7;
    ui_limit = (ui_limit > (ui_height >> 1)) ? ui_limit : (ui_height >> 1);
    u8 b_other_peak_exist;
    while (1)
    {
        ui_his_cnt = ps_list_histogram[0].ui_cnt;
        ui_max_idx = 0;
        for (int i = 1; i < ui_his_idx; i++)
        {
            if ((ps_list_histogram[i].ui_cnt) > (ui_his_cnt - 1))
            {
                ui_his_cnt = ps_list_histogram[i].ui_cnt;
                ui_max_idx = i;
            }
        }
        if (ui_his_cnt >= ui_limit)
        {
            b_other_peak_exist = 0;
            for (int j = 0; j < ui_his_idx; j++)
            {
                if (ps_list_histogram[j].ui_cnt >= (ui_his_cnt >> 1) && j != ui_max_idx)
                {
                    if (ps_list_histogram[ui_max_idx].almost_equal_data(ps_list_histogram[j], i_errorlimit >> 2, m_bit_depth))
                    {
                        b_other_peak_exist = 1;
                    }
                }
            }

            if (!b_other_peak_exist)
            {
                ps_list[ui_idx].copy_all_from(ps_list_histogram[ui_max_idx]);
                ps_initial[ui_idx].copy_all_from(ps_list_histogram[ui_max_idx]);
                last = ui_idx;
                ui_idx++;

                for (int j = 0; j < ui_his_idx; j++)
                {
                    if (ps_list_histogram[ui_max_idx].almost_equal_data(ps_list_histogram[j], i_errorlimit >> 2, m_bit_depth) && j != ui_max_idx)
                    {
                        ps_list_histogram[j].reset_element();
                    }
                }
            }

            ps_list_histogram[ui_max_idx].reset_element();
        }
        else
        {
            break;
        }
    }

    int ui_initial_idx = ui_idx;
    u8 b_matched;

    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            ui_pos = ui_y * ui_width + ui_x;
            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
            s_element.set_all(p_src[0][ui_pos], p_src[1][ui_pos_c], p_src[2][ui_pos_c]);
            b_matched = 0;
            for (int i = 0; i < (int)ui_initial_idx; i++)
            {
                if (b_matched || ps_initial[i].equal_data(s_element))
                {
                    b_matched = 1;
                }
            }

            if (!b_matched)
            {
                int besti = last, best_SAD = (last == -1) ? MAX_UINT : ps_list[last].get_SAD(s_element, m_bit_depth);
                if (best_SAD)
                {
                    for (int i = ui_idx - 1; i >= 0; i--)
                    {
                        unsigned int sad = ps_list[i].get_SAD(s_element, m_bit_depth);

                        if ((int)sad < best_SAD)
                        {
                            best_SAD = sad;
                            besti = i;
                            if (!sad) break;
                        }
                    }
                }

                if (besti >= 0 && ps_list[besti].almost_equal_data(s_element, i_errorlimit, m_bit_depth))
                {
                    ps_list[besti].add_element(s_element);
                    last = besti;
                }
                else
                {
                    ps_list[ui_idx].copy_data_from(s_element);
                    ps_list[ui_idx].ui_cnt = 1;
                    last = ui_idx;
                    ui_idx++;
                }
            }
        }
    }


    for (int i = 0; i < (int)ui_dict_max_size; i++)
    {
        p_list_sort[i].ui_cnt = 0;
        p_list_sort[i].set_all(0, 0, 0);
    }

    //bubble sorting
    unsigned int ui_max_pvbuf_size = MAX_SRB_SIZE;
    if ((ui_width == 8) && (ui_height == 8))
    {
        ui_max_pvbuf_size = EVS_PV_MAX;
    }
    if (((ui_width == 16) && (ui_height == 4)) || ((ui_width == 4) && (ui_height == 16)))
    {
        ui_max_pvbuf_size = EVS_PV_MAX;
    }
    if ((ui_width == 4) && (ui_height == 4))
    {
        ui_max_pvbuf_size = EVS_PV_4;
    }
    if (((ui_width == 8) && (ui_height == 4)) || ((ui_width == 4) && (ui_height == 8)))
    {
        ui_max_pvbuf_size = EVS_PV_8;
    }

    ui_dict_max_size = 1;
    for (int i = 0; i < (int)ui_idx; i++)
    {
        if (ps_list[i].ui_cnt > p_list_sort[ui_dict_max_size - 1].ui_cnt)
        {
            int j;
            for (j = ui_dict_max_size; j > 0; j--)
            {
                if (ps_list[i].ui_cnt > p_list_sort[j - 1].ui_cnt)
                {
                    p_list_sort[j].copy_all_from(p_list_sort[j - 1]);
                    ui_dict_max_size = (ui_dict_max_size + 1 < ui_max_pvbuf_size) ? (ui_dict_max_size + 1) : ui_max_pvbuf_size;
                }
                else
                {
                    break;
                }
            }
            p_list_sort[j].copy_all_from(ps_list[i]);
        }
    }
    int srb_ind_pred[MAX_SSRB_SIZE];
    memset(srb_ind_pred, 0, MAX_SSRB_SIZE * sizeof(int));

    ui_pvbuf_size = 0;
    pel *p_pred[3] = { p_cur_sp_info->m_srb_prev[0], p_cur_sp_info->m_srb_prev[1], p_cur_sp_info->m_srb_prev[2] };
    double bit_cost = p_cur_sp_info->lamda * 24;//* (m_bit_depth[CHANNEL_TYPE_LUMA] + 2 * m_bit_depth[CHANNEL_TYPE_CHROMA]);
    if (p_cur_sp_info->cu_ext)
    {
        bit_cost = p_cur_sp_info->lamda * 8;
    }
    {
        for (unsigned int i = 0; i < ui_max_pvbuf_size; i++)
        {
            if (p_list_sort[i].ui_cnt)
            {
                int iHalf = p_list_sort[i].ui_cnt >> 1;
                Srb[0][ui_pvbuf_size] = (p_list_sort[i].ui_sum_data[0] + iHalf) / p_list_sort[i].ui_cnt;
                Srb[1][ui_pvbuf_size] = (p_list_sort[i].ui_sum_data[1] + iHalf) / p_list_sort[i].ui_cnt;
                Srb[2][ui_pvbuf_size] = (p_list_sort[i].ui_sum_data[2] + iHalf) / p_list_sort[i].ui_cnt;

                int best = -1;
                if (i_errorlimit)
                {
                    double pal[3] = { p_list_sort[i].ui_sum_data[0] / (double)p_list_sort[i].ui_cnt,
                        p_list_sort[i].ui_sum_data[1] / (double)p_list_sort[i].ui_cnt,
                        p_list_sort[i].ui_sum_data[2] / (double)p_list_sort[i].ui_cnt };

                    double err = pal[0] - Srb[0][ui_pvbuf_size];
                    double best_cost = (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_LUMA] - 8)));
                    if (!p_cur_sp_info->cu_ext)
                    {
                        err = pal[1] - Srb[1][ui_pvbuf_size]; best_cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                        err = pal[2] - Srb[2][ui_pvbuf_size]; best_cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                    }
                    best_cost = best_cost * p_list_sort[i].ui_cnt + bit_cost;

                    for (int t = 0; t < p_cur_sp_info->m_pvbuf_size_prev; t++)
                    {
                        err = pal[0] - p_pred[0][t];
                        double cost = (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_LUMA] - 8)));
                        if (!p_cur_sp_info->cu_ext)
                        {
                            err = pal[1] - p_pred[1][t]; cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                            err = pal[2] - p_pred[2][t]; cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                        }
                        cost *= p_list_sort[i].ui_cnt;
                        if (cost < best_cost)
                        {
                            best = t;
                            best_cost = cost;
                        }
                    }
                    if (best != -1)
                    {
                        Srb[0][ui_pvbuf_size] = p_pred[0][best];
                        Srb[1][ui_pvbuf_size] = p_pred[1][best];
                        Srb[2][ui_pvbuf_size] = p_pred[2][best];
                    }
                    srb_ind_pred[ui_pvbuf_size] = best;
                }

                u8 b_duplicate = 0;
                if (p_list_sort[i].ui_cnt == 1 && best == -1)
                {
                    b_duplicate = 1;
                }
                else
                {
                    for (int t = 0; t < ui_pvbuf_size; t++)
                    {
                        if (p_cur_sp_info->cu_ext)
                        {
                            if (Srb[0][ui_pvbuf_size] == Srb[0][t])
                            {
                                b_duplicate = 1;
                                break;
                            }
                        }
                        else if (Srb[0][ui_pvbuf_size] == Srb[0][t] && Srb[1][ui_pvbuf_size] == Srb[1][t] && Srb[2][ui_pvbuf_size] == Srb[2][t])
                        {
                            b_duplicate = 1;
                            break;
                        }
                    }
                }
                if (!b_duplicate) ui_pvbuf_size++;
            }
            else
            {
                break;
            }
        }
    }

    pel *p_index_block = new pel[MAX_CU_SIZE * MAX_CU_SIZE];
    unsigned int srb_pred_samples[MAX_SSRB_SIZE][4];
    memset(srb_pred_samples, 0, 4 * MAX_SSRB_SIZE * sizeof(unsigned int));
    unsigned int i_errorlimit_sqr = 3 * i_errorlimit*i_errorlimit;
    if (p_cur_sp_info->cu_ext)
    {
        i_errorlimit_sqr = i_errorlimit * i_errorlimit;
    }
    unsigned int ui_abs_error;
    unsigned int ui_min_error;


    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            ui_pos = ui_y * ui_width + ui_x;
            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
            int ui_best_idx = 0, ui_srb_idx = 0;
            ui_min_error = MAX_UINT;

            while (ui_srb_idx < ui_pvbuf_size)
            {
                int i_temp = Srb[0][ui_srb_idx] - p_src[0][ui_pos];
                ui_abs_error = ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1));
                if (!p_cur_sp_info->cu_ext)
                {
                
                    i_temp = Srb[1][ui_srb_idx] - p_src[1][ui_pos_c];
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT);
                    i_temp = Srb[2][ui_srb_idx] - p_src[2][ui_pos_c];
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT);
                }

                if (ui_abs_error < ui_min_error)
                {
                    ui_best_idx = ui_srb_idx;
                    ui_min_error = ui_abs_error;
                    if (ui_min_error == 0)
                    {
                        break;
                    }
                }
                ui_srb_idx++;
            }

            unsigned int escape = 0;
            if (ui_min_error > i_errorlimit_sqr)
            {
                pel p_org[3] = { p_src[0][ui_pos],  p_src[1][ui_pos_c],  p_src[2][ui_pos_c] };
                unsigned int error_temp;
                double rd_cost = calc_pix_pred_RD(p_cur_sp_info, p_org, /*pcCost,*/ &error_temp);
                if (p_cur_sp_info->cu_ext)
                {
                    rd_cost = p_cur_sp_info->lamda * m_bit_depth[CHANNEL_TYPE_LUMA];
                }
                if (rd_cost < ui_min_error)
                {
                    escape = 1;//up
                }
            }

            if (escape == 0)
            {
                srb_pred_samples[ui_best_idx][0]++;
                srb_pred_samples[ui_best_idx][1] += p_src[0][ui_pos];
                srb_pred_samples[ui_best_idx][2] += p_src[1][ui_pos_c];
                srb_pred_samples[ui_best_idx][3] += p_src[2][ui_pos_c];
                p_index_block[ui_pos] = ui_best_idx;
            }
            else
            {
                p_index_block[ui_pos] = -1;
            }
        }
    }


    int srb_ind_best[MAX_SRB_PRED_SIZE];

    int   ui_pvbuf_size_temp = 0;
    for (int i = 0; i < ui_pvbuf_size; i++)
    {
        if (srb_pred_samples[i][0] > 0)
        {
            int iHalf = srb_pred_samples[i][0] >> 1;
            Srb[0][ui_pvbuf_size_temp] = (srb_pred_samples[i][1] + iHalf) / srb_pred_samples[i][0];
            Srb[1][ui_pvbuf_size_temp] = (srb_pred_samples[i][2] + iHalf) / srb_pred_samples[i][0];
            Srb[2][ui_pvbuf_size_temp] = (srb_pred_samples[i][3] + iHalf) / srb_pred_samples[i][0];

            double d_min_error = p_cur_sp_info->lamda * (m_bit_depth[CHANNEL_TYPE_LUMA] + 2 * m_bit_depth[CHANNEL_TYPE_CHROMA]);
            if (p_cur_sp_info->cu_ext)
            {
                d_min_error = p_cur_sp_info->lamda * m_bit_depth[CHANNEL_TYPE_LUMA];
            }

            for (int ui_y = 0; ui_y < ui_height; ui_y++)
            {
                for (int ui_x = 0; ui_x < ui_width; ui_x++)
                {
                    ui_pos = ui_y * ui_width + ui_x;
                    if (p_index_block[ui_pos] == i)
                    {
                        int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);

                        int i_temp = Srb[0][ui_pvbuf_size_temp] - p_src[0][ui_pos];
                        d_min_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1));
                        if (!p_cur_sp_info->cu_ext)
                        {
                        
                            i_temp = Srb[1][ui_pvbuf_size_temp] - p_src[1][ui_pos_c];
                            d_min_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT;
                            i_temp = Srb[2][ui_pvbuf_size_temp] - p_src[2][ui_pos_c];
                            d_min_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT;
                        }
                    }
                }
            }

            unsigned int max_pred_check = find_candidate_SRB_predictors(srb_ind_best, p_cur_sp_info, Srb, p_pred, ui_pvbuf_size_temp, 4);//MAX_PRED_CHEK);

            int best = -1;
            if (srb_ind_pred[i] >= 0)
            {
                for (unsigned int t = 0; t < max_pred_check; t++)
                {
                    if (srb_ind_pred[i] == srb_ind_best[t])
                    {
                        best = 1;
                    }
                }
                if (best == -1)
                {
                    srb_ind_best[max_pred_check] = srb_ind_pred[i];
                    max_pred_check++;
                }
            }

            best = -1;
            int tested_srb_pred;

            for (unsigned int t = 0; t < max_pred_check; t++)
            {
                tested_srb_pred = srb_ind_best[t];

                ui_abs_error = 0;
                for (int ui_y = 0; ui_y < ui_height; ui_y++)
                {
                    for (int ui_x = 0; ui_x < ui_width; ui_x++)
                    {
                        ui_pos = ui_y * ui_width + ui_x;
                        if (p_index_block[ui_pos] == i)
                        {

                            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
                            int i_temp = p_pred[0][tested_srb_pred] - p_src[0][ui_pos];
                            ui_abs_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1));
                            if (!p_cur_sp_info->cu_ext)
                            {
                            
                                i_temp = p_pred[1][tested_srb_pred] - p_src[1][ui_pos_c];
                                ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT);
                                i_temp = p_pred[2][tested_srb_pred] - p_src[2][ui_pos_c];
                                ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT);
                            }
                        }
                    }
                    if (ui_abs_error > d_min_error)
                    {
                        break;
                    }
                }

                if (ui_abs_error < d_min_error || (ui_abs_error == d_min_error && best > (int)tested_srb_pred))
                {
                    best = tested_srb_pred;
                    d_min_error = ui_abs_error;
                }
            }


            if (best != -1)
            {
                Srb[0][ui_pvbuf_size_temp] = p_pred[0][best];
                Srb[1][ui_pvbuf_size_temp] = p_pred[1][best];
                Srb[2][ui_pvbuf_size_temp] = p_pred[2][best];
            }

            u8 b_duplicate = 0;
            if (srb_pred_samples[i][0] == 1 && best == -1)
            {
                b_duplicate = 1;
            }
            else
            {
                for (int t = 0; t < ui_pvbuf_size_temp; t++)
                {
                    if (p_cur_sp_info->cu_ext)
                    {
                        if (Srb[0][ui_pvbuf_size_temp] == Srb[0][t])
                        {
                            b_duplicate = 1;
                            break;
                        }
                    }
                    else
                    {        
                        if (Srb[0][ui_pvbuf_size_temp] == Srb[0][t] && Srb[1][ui_pvbuf_size_temp] == Srb[1][t] && Srb[2][ui_pvbuf_size_temp] == Srb[2][t])
                        {
                            b_duplicate = 1;
                            break;
                        }
                    }
                }
            }
            if (!b_duplicate) ui_pvbuf_size_temp++;
        }
    }

    ui_pvbuf_size = ui_pvbuf_size_temp;

    if (p_index_block)
    {
        delete[] p_index_block;
        p_index_block = NULL;
    }

    delete[] ps_list;
    delete[] p_list_sort;

    delete[] ps_list_histogram;
    delete[] ps_initial;

}
void string_prediction::derive_srblossy_force_prediction(COM_SP_CODING_UNIT *p_cur_sp_info, pel *Srb[3], pel *p_src[3], int ui_width, int ui_height, int &ui_pvbuf_size)
{
    const unsigned int i_errorlimit = (unsigned int)get_srb_errLimit();
    
    unsigned int max_pvbuf_size_sps = MAX_SRB_SIZE;
    if (ui_width == 4 && ui_height == 4)
    {
        max_pvbuf_size_sps = EVS_PV_4;
    }
    if ((ui_width == 4 && ui_height == 8) || (ui_width == 8 && ui_height == 4))
    {
        max_pvbuf_size_sps = EVS_PV_8;
    }
    if ((ui_width == 4 && ui_height == 16) || (ui_width == 16 && ui_height == 4))
    {
        max_pvbuf_size_sps = EVS_PV_MAX;
    }
    if ((ui_width == 8) && (ui_height == 8))
    {
        max_pvbuf_size_sps = EVS_PV_MAX;
    }

    const unsigned int ui_total_size = ui_height * ui_width;
    SORTING_ELEMENT *ps_list = new SORTING_ELEMENT[ui_total_size];
    SORTING_ELEMENT s_element;
    SORTING_ELEMENT *p_list_sort = new SORTING_ELEMENT[max_pvbuf_size_sps + 1];

    ui_pvbuf_size = 0;
    unsigned int ui_idx = 0, ui_pos, ui_best_idx = 0;
    int last = -1;

    unsigned int srb_pred_index_used[MAX_SRB_PRED_SIZE];
    memset(srb_pred_index_used, 0, sizeof(srb_pred_index_used));

    u8 srb_index_used[MAX_SRB_PRED_SIZE];
    memset(srb_index_used, 0, sizeof(srb_index_used));

    pel *p_pred[3] = { p_cur_sp_info->m_srb_prev[0], p_cur_sp_info->m_srb_prev[1], p_cur_sp_info->m_srb_prev[2] };

    int scale_x = 0; //444, scale=0; 422 or 420 or 400, scaleX=1;
    int scale_y = 0; //422 or 444, scale=0; 420 or 400, scaleY=1;

    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            ui_pos = ui_y * ui_width + ui_x;
            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
            int ui_srb_idx = 0;
            unsigned int ui_min_error = MAX_UINT;
            while (ui_srb_idx < p_cur_sp_info->m_pvbuf_size_prev)
            {
                unsigned int ui_abs_error = (unsigned int)((abs(p_pred[0][ui_srb_idx] - p_src[0][ui_pos]) >> (m_bit_depth[CHANNEL_TYPE_LUMA] - 8))
                    + (abs(p_pred[1][ui_srb_idx] - p_src[1][ui_pos_c]) >> (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8)) * UV_WEIGHT
                    + (abs(p_pred[2][ui_srb_idx] - p_src[2][ui_pos_c]) >> (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8)) * UV_WEIGHT);
                if (p_cur_sp_info->cu_ext)
                {
                    ui_abs_error = (unsigned int)(abs(p_pred[0][ui_srb_idx] - p_src[0][ui_pos]) >> (m_bit_depth[CHANNEL_TYPE_LUMA] - 8));
                }
                if (ui_abs_error < ui_min_error)
                {
                    ui_best_idx = ui_srb_idx;
                    ui_min_error = ui_abs_error;
                    if (ui_min_error == 0)
                    {
                        break;
                    }
                }
                ui_srb_idx++;
            }

            if (ui_min_error <= i_errorlimit)// (ui_min_error <= (i_errorlimit/3))
            {
                srb_pred_index_used[ui_best_idx]++;
            }
        }
    }
    while (ui_idx < max_pvbuf_size_sps)
    {
        unsigned int max_no_index_used = 0, best_index = 0;
        for (unsigned int i = 0; i < p_cur_sp_info->m_pvbuf_size_prev; i++)
        {
            if (srb_index_used[i] == 0 && srb_pred_index_used[i] > max_no_index_used)
            {
                max_no_index_used = srb_pred_index_used[i];
                best_index = i;
            }
        }
        if (max_no_index_used > 0)
        {
            srb_index_used[best_index] = 1;

            Srb[0][ui_pvbuf_size] = p_pred[0][best_index];
            Srb[1][ui_pvbuf_size] = p_pred[1][best_index];
            Srb[2][ui_pvbuf_size] = p_pred[2][best_index];
            ui_pvbuf_size++;
        }
        else
        {
            break;
        }
        ui_idx++;
    }
    ui_idx = 0;
    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            ui_pos = ui_y * ui_width + ui_x;
            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);

            int ui_srb_idx = 0;
            unsigned int ui_min_error = MAX_UINT;
            while (ui_srb_idx < ui_pvbuf_size)
            {
                unsigned int ui_abs_error = (unsigned int)((abs(Srb[0][ui_srb_idx] - p_src[0][ui_pos]) >> (m_bit_depth[CHANNEL_TYPE_LUMA] - 8))
                    + (abs(Srb[1][ui_srb_idx] - p_src[1][ui_pos_c]) >> (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8)) * UV_WEIGHT
                    + (abs(Srb[2][ui_srb_idx] - p_src[2][ui_pos_c]) >> (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8)) * UV_WEIGHT);
                if (p_cur_sp_info->cu_ext)
                {
                    ui_abs_error = (unsigned int)(abs(Srb[0][ui_srb_idx] - p_src[0][ui_pos]) >> (m_bit_depth[CHANNEL_TYPE_LUMA] - 8));
                }
                if (ui_abs_error < ui_min_error)
                {
                    ui_min_error = ui_abs_error;
                    if (ui_min_error == 0)
                    {
                        break;
                    }
                }
                ui_srb_idx++;
            }

            if (ui_min_error > i_errorlimit)
            {
                s_element.set_all(p_src[0][ui_pos], p_src[1][ui_pos_c], p_src[2][ui_pos_c]);

                int besti = last, best_SAD = (last == -1) ? MAX_UINT : ps_list[last].get_SAD(s_element, m_bit_depth);
                if (best_SAD)
                {
                    for (int i = ui_idx - 1; i >= 0; i--)
                    {
                        unsigned int sad = ps_list[i].get_SAD(s_element, m_bit_depth);

                        if ((int)sad < best_SAD)
                        {
                            best_SAD = sad;
                            besti = i;
                            if (!sad)
                            {
                                break;
                            }
                        }
                    }
                }

                if (besti >= 0 && ps_list[besti].almost_equal_data(s_element, i_errorlimit, m_bit_depth))
                {
                    ps_list[besti].add_element(s_element);
                    last = besti;
                }
                else
                {
                    ps_list[ui_idx].copy_data_from(s_element);
                    ps_list[ui_idx].ui_cnt = 1;
                    last = ui_idx;
                    ui_idx++;
                }
            }
        }
    }
    for (int i = 0; i < (int)max_pvbuf_size_sps; i++)
    {
        p_list_sort[i].ui_cnt = 0;
        p_list_sort[i].set_all(0, 0, 0);
    }

    //bubble sorting
    unsigned int ui_dict_max_size = 1;
    for (unsigned int i = 0; i < ui_idx; i++)
    {
        if (ps_list[i].ui_cnt > p_list_sort[ui_dict_max_size - 1].ui_cnt)
        {
            int j;
            for (j = ui_dict_max_size; j > 0; j--)
            {
                if (ps_list[i].ui_cnt > p_list_sort[j - 1].ui_cnt)
                {
                    p_list_sort[j].copy_all_from(p_list_sort[j - 1]);
                        ui_dict_max_size = (ui_dict_max_size + 1 < max_pvbuf_size_sps) ? (ui_dict_max_size + 1) : max_pvbuf_size_sps;
                }
                else
                {
                    break;
                }
            }
            p_list_sort[j].copy_all_from(ps_list[i]);
        }
    }

    double bit_cost = p_cur_sp_info->lamda * 24; // (m_bit_depth[CHANNEL_TYPE_LUMA] + 2 * m_bit_depth[CHANNEL_TYPE_CHROMA]);
    if (p_cur_sp_info->cu_ext)
    {
        bit_cost = p_cur_sp_info->lamda * 8;
    }
    {
        for (int i = 0; i < (int)max_pvbuf_size_sps && ui_pvbuf_size < (int)max_pvbuf_size_sps; i++)
        {
            if (p_list_sort[i].ui_cnt)
            {
                int iHalf = p_list_sort[i].ui_cnt >> 1;
                Srb[0][ui_pvbuf_size] = (p_list_sort[i].ui_sum_data[0] + iHalf) / p_list_sort[i].ui_cnt;
                Srb[1][ui_pvbuf_size] = (p_list_sort[i].ui_sum_data[1] + iHalf) / p_list_sort[i].ui_cnt;
                Srb[2][ui_pvbuf_size] = (p_list_sort[i].ui_sum_data[2] + iHalf) / p_list_sort[i].ui_cnt;

                u8 b_duplicate = 0;
                if (p_list_sort[i].ui_cnt == 1)
                {
                    b_duplicate = 1;
                }
                else
                {
                    int best = -1;
                    if (i_errorlimit)
                    {
                        double pal[3] = { p_list_sort[i].ui_sum_data[0] / (double)p_list_sort[i].ui_cnt,
                            p_list_sort[i].ui_sum_data[1] / (double)p_list_sort[i].ui_cnt,
                            p_list_sort[i].ui_sum_data[2] / (double)p_list_sort[i].ui_cnt };

                        double err = pal[0] - Srb[0][ui_pvbuf_size];
                        double best_cost = (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_LUMA] - 8)));
                        if (!p_cur_sp_info->cu_ext)
                        {             
                            err = pal[1] - Srb[1][ui_pvbuf_size]; best_cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                            err = pal[2] - Srb[2][ui_pvbuf_size]; best_cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                        }
                        best_cost = best_cost * p_list_sort[i].ui_cnt + bit_cost;

                        for (int t = 0; t < ui_pvbuf_size; t++)
                        {
                            if (p_cur_sp_info->cu_ext)
                            {
                                if (Srb[0][ui_pvbuf_size] == Srb[0][t])
                                {
                                    b_duplicate = 1;
                                    break;
                                }
                            }
                            else
                            {
                                if (Srb[0][ui_pvbuf_size] == Srb[0][t] && Srb[1][ui_pvbuf_size] == Srb[1][t] && Srb[2][ui_pvbuf_size] == Srb[2][t])
                                {
                                    b_duplicate = 1;
                                    break;
                                }
                            }                            
                            err = pal[0] - Srb[0][t];
                            double cost = (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_LUMA] - 8)));
                            if (!p_cur_sp_info->cu_ext)
                            {     
                                err = pal[1] - Srb[1][t]; cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                                err = pal[2] - Srb[2][t]; cost += (err * err) / (1 << (2 * (m_bit_depth[CHANNEL_TYPE_CHROMA] - 8))) * UV_WEIGHT;
                            }
                            cost *= p_list_sort[i].ui_cnt;
                            if (cost < best_cost)
                            {
                                best = t;
                                best_cost = cost;
                            }
                        }
                        if (best != -1)
                        {
                            b_duplicate = 1;
                        }
                    }
                }

                if (!b_duplicate)
                {
                    ui_pvbuf_size++;
                }
            }
            else
            {
                break;
            }
        }
    }
    pel *p_index_block = new pel[MAX_CU_SIZE * MAX_CU_SIZE];
    unsigned int srb_pred_samples[MAX_SSRB_SIZE][4];
    memset(srb_pred_samples, 0, 4 * MAX_SSRB_SIZE * sizeof(unsigned int));
    unsigned int i_errorlimit_sqr = 3 * i_errorlimit*i_errorlimit;
    if (p_cur_sp_info->cu_ext)
    {
        i_errorlimit_sqr = i_errorlimit * i_errorlimit;
    }
    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            ui_pos = ui_y * ui_width + ui_x;

            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);

            int ui_srb_idx = 0;
            unsigned int ui_min_error = MAX_UINT;
            while (ui_srb_idx < ui_pvbuf_size)
            {
                int i_temp = Srb[0][ui_srb_idx] - p_src[0][ui_pos];
                unsigned int ui_abs_error = ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1));
                if (!p_cur_sp_info->cu_ext)
                {
                    i_temp = Srb[1][ui_srb_idx] - p_src[1][ui_pos_c];
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT);
                    i_temp = Srb[2][ui_srb_idx] - p_src[2][ui_pos_c];
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT);
                }
                if (ui_abs_error < ui_min_error)
                {
                    ui_best_idx = ui_srb_idx;
                    ui_min_error = ui_abs_error;
                    if (ui_min_error == 0)
                    {
                        break;
                    }
                }
                ui_srb_idx++;
            }

            unsigned int escape = 0;
            if (ui_min_error > i_errorlimit_sqr)
            {
                pel p_org[3] = { p_src[0][ui_pos],  p_src[1][ui_pos_c],  p_src[2][ui_pos_c] };
                unsigned int error_temp;
                double rd_cost = calc_pix_pred_RD(p_cur_sp_info, p_org, /*pcCost,*/ &error_temp);
                if (p_cur_sp_info->cu_ext)
                {
                    rd_cost = p_cur_sp_info->lamda * m_bit_depth[CHANNEL_TYPE_LUMA];
                }
                if (rd_cost < ui_min_error)
                {
                    escape = 1;
                }
            }

            if (escape == 0)
            {
                srb_pred_samples[ui_best_idx][0]++;
                srb_pred_samples[ui_best_idx][1] += p_src[0][ui_pos];
                srb_pred_samples[ui_best_idx][2] += p_src[1][ui_pos_c];
                srb_pred_samples[ui_best_idx][3] += p_src[2][ui_pos_c];
                p_index_block[ui_pos] = ui_best_idx;
            }
            else
            {
                p_index_block[ui_pos] = -1;
            }
        }
    }

    int srb_ind_best[MAX_SRB_PRED_SIZE];

    int ui_pvbuf_size_temp = 0;
    for (int i = 0; i < ui_pvbuf_size; i++)
    {
        if (srb_pred_samples[i][0] > 0)
        {
            int iHalf = srb_pred_samples[i][0] >> 1;

            Srb[0][ui_pvbuf_size_temp] = (srb_pred_samples[i][1] + iHalf) / srb_pred_samples[i][0];
            Srb[1][ui_pvbuf_size_temp] = (srb_pred_samples[i][2] + iHalf) / srb_pred_samples[i][0];
            Srb[2][ui_pvbuf_size_temp] = (srb_pred_samples[i][3] + iHalf) / srb_pred_samples[i][0];

            double ui_min_error = p_cur_sp_info->lamda * (m_bit_depth[CHANNEL_TYPE_LUMA] + 2 * m_bit_depth[CHANNEL_TYPE_CHROMA]),
                ui_abs_error;
            if (p_cur_sp_info->cu_ext)
            {
                ui_min_error = p_cur_sp_info->lamda * m_bit_depth[CHANNEL_TYPE_LUMA];
            }
            for (int ui_y = 0; ui_y < ui_height; ui_y++)
            {
                for (int ui_x = 0; ui_x < ui_width; ui_x++)
                {
                    ui_pos = ui_y * ui_width + ui_x;
                    if (p_index_block[ui_pos] == i)
                    {
                        int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
                        int i_temp = Srb[0][ui_pvbuf_size_temp] - p_src[0][ui_pos];
                        ui_min_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1));
                        if (!p_cur_sp_info->cu_ext)
                        {
                            i_temp = Srb[1][ui_pvbuf_size_temp] - p_src[1][ui_pos_c];
                            ui_min_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT;
                            i_temp = Srb[2][ui_pvbuf_size_temp] - p_src[2][ui_pos_c];
                            ui_min_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT;
                        }
                    }
                }
            }


            unsigned int max_pred_check = find_candidate_SRB_predictors(srb_ind_best, p_cur_sp_info, Srb, p_pred, ui_pvbuf_size_temp, 4);//MAX_PRED_CHEK);
            int best = -1;
            int tested_srb_pred;

            for (unsigned int t = 0; t < max_pred_check; t++)
            {
                tested_srb_pred = srb_ind_best[t];

                ui_abs_error = 0;
                for (int ui_y = 0; ui_y < ui_height; ui_y++)
                {
                    for (int ui_x = 0; ui_x < ui_width; ui_x++)
                    {
                        ui_pos = ui_y * ui_width + ui_x;
                        if (p_index_block[ui_pos] == i)
                        {

                            int ui_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);

                            int i_temp = p_pred[0][tested_srb_pred] - p_src[0][ui_pos];
                            ui_abs_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1));
                            if (!p_cur_sp_info->cu_ext)
                            {
                                i_temp = p_pred[1][tested_srb_pred] - p_src[1][ui_pos_c];
                                ui_abs_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT;
                                i_temp = p_pred[2][tested_srb_pred] - p_src[2][ui_pos_c];
                                ui_abs_error += ((i_temp * i_temp) >> ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1)) * UV_WEIGHT;
                            }
                        }
                    }
                    if (ui_abs_error > ui_min_error)
                    {
                        break;
                    }
                }

                if (ui_abs_error < ui_min_error || (ui_abs_error == ui_min_error && best > (int)tested_srb_pred))
                {
                    best = tested_srb_pred;
                    ui_min_error = ui_abs_error;
                }
            }

            if (best != -1)
            {
                Srb[0][ui_pvbuf_size_temp] = p_pred[0][best];
                Srb[1][ui_pvbuf_size_temp] = p_pred[1][best];
                Srb[2][ui_pvbuf_size_temp] = p_pred[2][best];
            }

            u8 b_duplicate = 0;
            if (srb_pred_samples[i][0] == 1 && best == -1)
            {
                b_duplicate = 1;
            }
            else
            {
                for (int t = 0; t < ui_pvbuf_size_temp; t++)
                {
                    if (p_cur_sp_info->cu_ext)
                    {
                        if (Srb[0][ui_pvbuf_size_temp] == Srb[0][t])
                        {
                            b_duplicate = 1;
                            break;
                        }
                    }
                    else
                    {
                    
                        if (Srb[0][ui_pvbuf_size_temp] == Srb[0][t] && Srb[1][ui_pvbuf_size_temp] == Srb[1][t] && Srb[2][ui_pvbuf_size_temp] == Srb[2][t])
                        {
                            b_duplicate = 1;
                            break;
                        }
                    }
                }
            }
            if (!b_duplicate)
            {
                ui_pvbuf_size_temp++;
            }
        }
    }

    ui_pvbuf_size = ui_pvbuf_size_temp;


    if (p_index_block)
    {
        delete[] p_index_block;
        p_index_block = NULL;
    }


    delete[] ps_list;
    delete[] p_list_sort;
}



void string_prediction::derive_cluster(COM_SP_CODING_UNIT* p_cur_sp_info, pel *p_cluster[3], int ui_cluster_size, int ui_width, int ui_height, int x_start_in_pic, int y_start_in_pic, u8 b_cs2_mode_flag, double d_lambda, unsigned int** ind_error, unsigned int calc_error_bits)
{
    COM_SP_PIX cur_pixel;
    unsigned int  i_errorlimit = get_srb_errLimit();
    i_errorlimit = 3 * get_srb_errLimit()*get_srb_errLimit();//
    if (p_cur_sp_info->cu_ext)
    {
        i_errorlimit = get_srb_errLimit()*get_srb_errLimit();
    }
    pel dist_adj_y = ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1);
    pel dist_adj_c = ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1);
    short i_temp;
    int ui_pos;
    int scale_x = 0;
    int scale_y = 0;
    //gen major flag;  
    int  ui_total = ui_width * ui_height;
    u8* match_flag = (u8*)x_malloc(u8, ui_total);//new u8[ui_total];  
    int cu_match_cnt = 0;
    int ui_best_idx = 0;
    ui_pos = 0;
    m_esc_flag = 0;
    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            int curr_x = x_start_in_pic + ui_x;
            int curr_y = y_start_in_pic + ui_y;
            int cur_pos = curr_y * m_stride_org[0] + curr_x;
            int cur_pos_c = (ui_y >> scale_y) * (ui_width >> scale_x) + (ui_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos];
            cur_pixel.U = m_p_pel_org444[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org444[2][cur_pos_c];
            //save org pixel
            m_p_org_pixel_buffer[ui_pos].Y = cur_pixel.Y;
            m_p_org_pixel_buffer[ui_pos].U = cur_pixel.U;
            m_p_org_pixel_buffer[ui_pos].V = cur_pixel.V;

            //
            int i = 0;
            unsigned int ui_min_error = MAX_UINT;
            ui_best_idx = 0;
            while (i < ui_cluster_size)
            {
                unsigned int ui_abs_error;// = abs(p_cluster[0][i] - cur_pixel.Y) + abs(p_cluster[1][i] - cur_pixel.U) + abs(p_cluster[2][i] - cur_pixel.V);
                i_temp = p_cluster[0][i] - cur_pixel.Y;
                ui_abs_error = (i_temp * i_temp) >> dist_adj_y;
                if (!p_cur_sp_info->cu_ext)
                {
                    i_temp = p_cluster[1][i] - cur_pixel.U;
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> dist_adj_c) * UV_WEIGHT);
                    i_temp = p_cluster[2][i] - cur_pixel.V;
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> dist_adj_c) * UV_WEIGHT);
                }
                ind_error[ui_pos][i] = ui_abs_error;
                if (ui_abs_error < ui_min_error)
                {
                    ui_best_idx = i;
                    ui_min_error = ui_abs_error;
                }
                i++;
            }
            u8 b_mismatch = 0;//ui_min_error > 3 * i_errorlimit;

            if (ui_min_error > i_errorlimit || calc_error_bits)
            {
                unsigned int error_temp = 0;
                double rd_cost = SP_MAX_COST;

                {
                    pel pOrg[3] = { cur_pixel.Y, cur_pixel.U, cur_pixel.V };
                    rd_cost = calc_pix_pred_RD(p_cur_sp_info, pOrg, /*m_pRdCostImp,*/ &error_temp);
                    if (p_cur_sp_info->cu_ext)
                    {
                        rd_cost = p_cur_sp_info->lamda *  m_bit_depth[CHANNEL_TYPE_LUMA];
                    }
                    if (rd_cost < ui_min_error)
                    {
                        b_mismatch = 1;
                    }
                }
                ind_error[ui_pos][MAX_SSRB_SIZE - 1] = (unsigned int)rd_cost;
                ind_error[ui_pos][MAX_SSRB_SIZE] = (unsigned int)error_temp;
            }

            match_flag[ui_pos] = b_mismatch;
            m_b_major[ui_pos] = 1;
            m_c_match_index[ui_pos] = ui_best_idx;
            if (b_mismatch)
            {
                m_b_major[ui_pos] = 0;
                m_c_match_index[ui_pos] = 0xff;
            }
            ui_pos++;
        }
    }
    //gen CQ Pixel buffer
    for (int i = 0; i < ui_total; i++)
    {
        if (!match_flag[i])
        {
            m_p_CQ_pixbuffer[i].Y = p_cluster[0][m_c_match_index[i]];
            m_p_CQ_pixbuffer[i].U = p_cluster[1][m_c_match_index[i]];
            m_p_CQ_pixbuffer[i].V = p_cluster[2][m_c_match_index[i]];
            cu_match_cnt++;
        }
        else
        {
            m_esc_flag = 1;
            m_p_CQ_pixbuffer[i].Y = m_p_org_pixel_buffer[i].Y;
            m_p_CQ_pixbuffer[i].U = m_p_org_pixel_buffer[i].U;
            m_p_CQ_pixbuffer[i].V = m_p_org_pixel_buffer[i].V;
        }
    }
    set_srb_match_cnt(cu_match_cnt);

    // group individual pixel into neighbouring pixels.
    if (match_flag)
    {
        x_free(match_flag);//delete[] match_flag;
        match_flag = NULL;
    }
}

void string_prediction::regen_cluster_info(COM_SP_CODING_UNIT* p_cur_sp_info, pel *p_cluster[3], int ui_cluster_size, int ui_width, int ui_height, int x_start_in_pic, int y_start_in_pic, /*u8 bLosslessMode,*/ unsigned int** ind_error, unsigned int calc_error_bits)
{
    COM_SP_PIX cur_pixel;
    unsigned int  i_errorlimit = get_srb_errLimit();
    i_errorlimit = 3 * get_srb_errLimit()*get_srb_errLimit();//
    if (p_cur_sp_info->cu_ext)
    {
        i_errorlimit = get_srb_errLimit()*get_srb_errLimit();
    }
    pel dist_adj_y = ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1);
    pel dist_adj_c = ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1);
    short i_temp;
    int ui_pos;

    int scale_x = 0;
    int scale_y = 0;
    //gen major flag;  
    unsigned int  ui_total = ui_width * ui_height;
    int ui_best_idx = 0;
    ui_pos = 0;
    for (int ui_y = 0; ui_y < ui_height; ui_y++)
    {
        for (int ui_x = 0; ui_x < ui_width; ui_x++)
        {
            int curr_x = x_start_in_pic + ui_x;
            int curr_y = y_start_in_pic + ui_y;
            int cur_pos = curr_y * m_stride_org[0] + curr_x;
            int cur_pos_c = ui_y * ui_width + ui_x;
            cur_pixel.Y = m_p_pel_org444[0][cur_pos];
            cur_pixel.U = m_p_pel_org444[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org444[2][cur_pos_c];
            //save org pixel
            m_p_org_pixel_buffer[ui_pos].Y = cur_pixel.Y;
            m_p_org_pixel_buffer[ui_pos].U = cur_pixel.U;
            m_p_org_pixel_buffer[ui_pos].V = cur_pixel.V;

            //
            int i = 0;
            unsigned int ui_min_error = MAX_UINT;
            ui_best_idx = 0;
            while (i < ui_cluster_size)
            {
                unsigned int ui_abs_error;// = abs(p_cluster[0][i] - cur_pixel.Y) + abs(p_cluster[1][i] - cur_pixel.U) + abs(p_cluster[2][i] - cur_pixel.V);
                i_temp = p_cluster[0][i] - cur_pixel.Y;
                ui_abs_error = (i_temp * i_temp) >> dist_adj_y;
                if (!p_cur_sp_info->cu_ext)
                {
                
                    i_temp = p_cluster[1][i] - cur_pixel.U;
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> dist_adj_c) * UV_WEIGHT);
                    i_temp = p_cluster[2][i] - cur_pixel.V;
                    ui_abs_error += (unsigned int)(((i_temp * i_temp) >> dist_adj_c) * UV_WEIGHT);
                }
                ind_error[ui_pos][i] = ui_abs_error;
                if (ui_abs_error < ui_min_error)
                {
                    ui_best_idx = i;
                    ui_min_error = ui_abs_error;
                }
                i++;
            }
            u8 b_mismatch = 0;//ui_min_error > 3 * i_errorlimit;

            if (ui_min_error > i_errorlimit || calc_error_bits)
            {
                unsigned int error_temp = 0;
                double rd_cost = SP_MAX_COST;
                {
                    pel p_org[3] = { cur_pixel.Y, cur_pixel.U, cur_pixel.V };
                    rd_cost = calc_pix_pred_RD(p_cur_sp_info, p_org, /*m_pRdCostImp,*/ &error_temp);
                    if (p_cur_sp_info->cu_ext)
                    {
                        rd_cost = p_cur_sp_info->lamda * m_bit_depth[CHANNEL_TYPE_LUMA];
                    }
                    if (rd_cost < ui_min_error)
                    {
                        b_mismatch = 1;
                    }
                }
                ind_error[ui_pos][MAX_SSRB_SIZE - 1] = (unsigned int)rd_cost;
                ind_error[ui_pos][MAX_SSRB_SIZE] = (unsigned int)error_temp;
            }
            ui_pos++;
        }
    }
}

int string_prediction::get_srb_length(COM_SP_CODING_UNIT*p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, int offset_in_cu, unsigned int& dist, unsigned int& dist_y, unsigned int& dist_uv, COM_SP_PIX& pixel_cur, SP_MATCH_TYPE& match_type_cur)
{
    int srb_length = 1;
    unsigned int ui_total = width * height;
    u8 is_horizontal_scanning = p_cur_sp_info->string_copy_direction;
    COM_SP_PIX pixel_org_cur;
    COM_SP_PIX pixel_org;
    COM_SP_PIX pixel_nxt;
    int* p_trav_scan_order = com_tbl_raster2trav[is_horizontal_scanning][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    unsigned int uiTraIdx = p_trav_scan_order[offset_in_cu];
    int  cnt_nxt = offset_in_cu + 1;
    u8 b_major;
    unsigned int cur_dist_y, cur_dist_uv;


    pixel_org_cur = pixel_org = m_p_org_pixel_buffer[uiTraIdx];
    pixel_cur = m_p_CQ_pixbuffer[uiTraIdx];
    b_major = m_b_major[uiTraIdx];
    dist = calc_dist_norm(pixel_org, pixel_cur, cur_dist_y, cur_dist_uv);
    dist_y = cur_dist_y;
    dist_uv = cur_dist_uv;
    if (!b_major) {
        match_type_cur = MATCH_NONE;
        for (srb_length = 1; cnt_nxt < (int)ui_total; srb_length++) 
        {
            uiTraIdx = p_trav_scan_order[cnt_nxt];
            b_major = m_b_major[uiTraIdx];
            if (!b_major) 
            {
                cnt_nxt++;
            }
            else
            {
                break;
            }
        }
    }
    else 
    {
        match_type_cur = MATCH_POS_ONE;
        for (srb_length = 1; cnt_nxt < (int)ui_total; srb_length++) 
        {
            uiTraIdx = p_trav_scan_order[cnt_nxt];
            pixel_org = m_p_org_pixel_buffer[uiTraIdx];
            pixel_nxt = m_p_CQ_pixbuffer[uiTraIdx];
            if (pixel_cur == pixel_nxt) 
            {
                dist += calc_dist_norm(pixel_org, pixel_cur, cur_dist_y, cur_dist_uv);
                dist_y += cur_dist_y;
                dist_uv += cur_dist_uv;
                cnt_nxt++;
            }
            else
            {
                break;
            }
        }
    }
    return srb_length;
}

int string_prediction::get_SSrb_above_length(COM_SP_CODING_UNIT*p_cur_sp_info, int width, int height, int offset_in_cu, unsigned int& dist, unsigned int& dist_y, unsigned int& dist_uv, COM_SP_PIX& pixel_cur)
{
    int above_length = 0;
    unsigned int ui_total = width * height;
    u8 is_horizontal_scanning = p_cur_sp_info->string_copy_direction;
    COM_SP_PIX pixel_abv;
    u8 b_major_cur;
    u8 b_major_abv;
    //
    unsigned int tra_idx;
    unsigned int tra_idx_abv;
    int* p_trav_scan_order = com_tbl_raster2trav[is_horizontal_scanning][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    unsigned int trav_order_index;
    int  cntNxt = offset_in_cu + 1;
    if (offset_in_cu < width)
    {
        return -1;
    }
    unsigned int cur_dist_y, cur_dist_uv;
    unsigned int scale_x = 0;
    unsigned int scale_y = 0;

    trav_order_index = p_trav_scan_order[offset_in_cu];
    tra_idx = trav_order_index;
    tra_idx_abv = (is_horizontal_scanning ? (trav_order_index - width) : (trav_order_index - 1));

    pixel_cur = m_p_CQ_pixbuffer[tra_idx];
    pixel_abv = m_p_CQ_pixbuffer[tra_idx_abv];
    b_major_cur = m_b_major[tra_idx];
    b_major_abv = m_b_major[tra_idx_abv];
    //search above
    if (pixel_cur == pixel_abv)
    {
        COM_SP_PIX pixel_org = m_p_org_pixel_buffer[tra_idx];
        COM_SP_PIX pixel_rec = pixel_abv;//m_p_CQ_pixbuffer[tra_idx];
        dist = calc_dist_norm(pixel_org, pixel_rec, cur_dist_y, cur_dist_uv);
        dist_y = cur_dist_y;
        dist_uv = cur_dist_uv;
        for (above_length = 1; cntNxt < (int)ui_total; above_length++)
        {
            trav_order_index = p_trav_scan_order[cntNxt];
            tra_idx = trav_order_index;
            tra_idx_abv = (is_horizontal_scanning ? (trav_order_index - width) : (trav_order_index - 1));

            pixel_cur = m_p_CQ_pixbuffer[tra_idx];
            pixel_abv = m_p_CQ_pixbuffer[tra_idx_abv];
            b_major_cur = m_b_major[tra_idx];
            b_major_abv = m_b_major[tra_idx_abv];

            if (pixel_cur == pixel_abv)
            {
                //
                pixel_org = m_p_org_pixel_buffer[tra_idx];
                pixel_rec = pixel_abv;//m_p_CQ_pixbuffer[tra_idx];
                dist += calc_dist_norm(pixel_org, pixel_rec, cur_dist_y, cur_dist_uv);
                dist_y += cur_dist_y;
                dist_uv += cur_dist_uv;
                cntNxt++;
            }
            else
            {
                break;
            }
        }
    }
    return above_length;
}

unsigned int string_prediction::calc_dist(COM_SP_PIX orgPixel, COM_SP_PIX recPixel, unsigned int& dist_y, unsigned int& dist_uv)
{
    unsigned int shift_y = ((m_bit_depth[CHANNEL_TYPE_LUMA] - 8) << 1);
    unsigned int shift_c = ((m_bit_depth[CHANNEL_TYPE_CHROMA] - 8) << 1);
    int  i_temp;
    unsigned int ui_sum = 0;
    i_temp = orgPixel.Y - recPixel.Y; dist_y = ((i_temp * i_temp) >> shift_y);
    i_temp = orgPixel.U - recPixel.U; dist_uv = ((i_temp * i_temp) >> shift_c);
    i_temp = orgPixel.V - recPixel.V; dist_uv += ((i_temp * i_temp) >> shift_c);
    ui_sum = (unsigned int)(dist_y + dist_uv);
    return ui_sum;
}

void string_prediction::save_last_srb(ENC_CU_DATA *src, int cup)
{
    struct
    {
        pel values[MAX_SRB_PRED_SIZE];
        u8  flag_value[MAX_SRB_PRED_SIZE];
        u8 size;
    } chinfo[3] = { { { 0, }, 0, } };
    u8  dpb_reonly_Y[MAX_SRB_PRED_SIZE] = { 0 };
    u8  dpb_idx[MAX_SRB_PRED_SIZE] = { 0 };
    u8  cuSflag_value[MAX_SRB_PRED_SIZE] = { 0 };
    s16  x_value[MAX_SRB_PRED_SIZE] = { 0 };
    s16  y_value[MAX_SRB_PRED_SIZE] = { 0 };
    for (int ch = 0; ch < 3; ch++)
    {
        int srcCh = ch;
        int size = src->pvbuf_size[cup];
        int numEl = (size<m_srb_max_pred_size) ? size : m_srb_max_pred_size;
        ::memcpy(chinfo[ch].values, src->p_SRB+(cup*N_C+ch)*MAX_SRB_SIZE, numEl * sizeof(pel));

        int ui_pvbuf_size_prev = src->pvbuf_size_prev[cup];
        int limit = m_srb_max_pred_size;
        // Padd with last SRB
        for (int i = 0; i<ui_pvbuf_size_prev && numEl<limit; i++)
        {
            int idx = i;
            if (!src->pvbuf_reused_flag[cup*MAX_SRB_PRED_SIZE +idx])
            {
                chinfo[ch].values[numEl] = src->p_SRB_prev[(cup*N_C + ch)*MAX_SRB_PRED_SIZE +idx];
                numEl++;
            }
        }
        chinfo[ch].size = numEl;
    }

    for (int ch = 0; ch < 3; ch++)
    {
        src->pvbuf_size_prev[cup] = chinfo[ch].size;
        ::memcpy(src->p_SRB_prev+ (cup*N_C + ch)*MAX_SRB_PRED_SIZE, chinfo[ch].values, m_srb_max_pred_size * sizeof(pel));
    }

    {
        int size = src->pvbuf_size[cup];
        int numEl = (size<m_srb_max_pred_size) ? size : m_srb_max_pred_size;
        ::memcpy(chinfo[0].flag_value, src->all_comp_flag+cup*MAX_SRB_SIZE, numEl * sizeof(u8));
        ::memcpy(cuSflag_value, src->cuS_flag + cup * MAX_SRB_SIZE, numEl * sizeof(u8));
        ::memcpy(x_value, src->pv_x + cup * MAX_SRB_SIZE, numEl * sizeof(s16));
        ::memcpy(y_value, src->pv_y + cup * MAX_SRB_SIZE, numEl * sizeof(s16));
        ::memcpy(dpb_reonly_Y, src->m_dpb_reYonly + cup * MAX_SRB_SIZE, numEl * sizeof(u8));
        ::memcpy(dpb_idx, src->m_dpb_idx + cup * MAX_SRB_SIZE, numEl * sizeof(u8));
        int ui_pvbuf_size_prev = src->pvbuf_size_prev[cup];
        int limit = m_srb_max_pred_size;
        for (int i = 0; i < ui_pvbuf_size_prev && numEl < limit; i++)
        {
            int idx = i;
            if (!src->pvbuf_reused_flag[cup*MAX_SRB_PRED_SIZE + idx])
            {
                chinfo[0].flag_value[numEl] = src->all_comp_pre_flag[cup*MAX_SRB_PRED_SIZE+idx];
                cuSflag_value[numEl] = src->cuS_pre_flag[cup*MAX_SRB_PRED_SIZE + idx];
                x_value[numEl] = src->pv_x_prev[cup*MAX_SRB_PRED_SIZE + idx];
                y_value[numEl] = src->pv_y_prev[cup*MAX_SRB_PRED_SIZE + idx];
                dpb_reonly_Y[numEl] = src->m_dpb_reYonly_prev[cup*MAX_SRB_PRED_SIZE + idx];
                dpb_idx[numEl] = src->m_dpb_idx_prev[cup*MAX_SRB_PRED_SIZE + idx];
                numEl++;
            }
        }
    }
    ::memcpy(src->all_comp_pre_flag+cup* MAX_SRB_PRED_SIZE, chinfo[0].flag_value, m_srb_max_pred_size * sizeof(u8));
    ::memcpy(src->cuS_pre_flag + cup * MAX_SRB_PRED_SIZE, cuSflag_value, m_srb_max_pred_size * sizeof(u8));
    ::memcpy(src->pv_x_prev + cup * MAX_SRB_PRED_SIZE, x_value, m_srb_max_pred_size * sizeof(s16));
    ::memcpy(src->pv_y_prev + cup * MAX_SRB_PRED_SIZE, y_value, m_srb_max_pred_size * sizeof(s16));
    ::memcpy(src->m_dpb_reYonly_prev + cup * MAX_SRB_PRED_SIZE, dpb_reonly_Y, m_srb_max_pred_size * sizeof(u8));
    ::memcpy(src->m_dpb_idx_prev + cup * MAX_SRB_PRED_SIZE, dpb_idx, m_srb_max_pred_size * sizeof(u8));
    if (chinfo[0].size > 0)
    {
        int maxSize = chinfo[0].size;
        assert(maxSize > 0);
        for (int i = 0; i < maxSize - 1; i++)
        {
            COM_SP_PIX pixelCur, pixelNxt;
            pixelCur.Y = chinfo[0].values[i];
            pixelCur.U = chinfo[1].values[i];
            pixelCur.V = chinfo[2].values[i];

            for (int j = i + 1; j < maxSize; j++)
            {
                pixelNxt.Y = chinfo[0].values[j];
                pixelNxt.U = chinfo[1].values[j];
                pixelNxt.V = chinfo[2].values[j];
                assert(pixelCur != pixelNxt);
            }
        }
    }
}
u8 string_prediction::encode_evs_or_ubvs(COM_SP_CODING_UNIT* p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_EVS_INFO>& vec_dict_info, vector<COM_SP_PIX>& vec_dict_pixel, COM_SP_PIX* p_CQ_buf, double *cur_best_rdcost,/* unsigned int* distortion,*/ u8 b_SRB_sharing_flag
    , ENC_CTX *ctx
)
{
    int count = width * height;
    int processed_count = 0;
    unsigned int ui_max_pvbuf_size = MAX_SRB_SIZE;
    u8 is_horizontal_scanning = p_cur_sp_info->string_copy_direction;
    int  trav_x;
    int  trav_y;
    double dict_RD_cost = 0;
    unsigned int   dict_bits = 0;
    double tmp_dict_RD_cost = 0;
    unsigned int   dict_dist = 0;
    unsigned int dict_dist_y = 0, dict_dist_uv = 0;
    int* p_trav_scan_order = com_tbl_raster2trav[is_horizontal_scanning][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    unsigned int trav_order_index;
    int ui_pvbuf_size = get_cluster_size();
    SP_MATCH_TYPE srb_match_mode;
    int ui_pvbuf_adr = 0;
    int ui_pvbuf_adr_prev = 0, ui_pvbuf_adr_above = 0;
    int  pos_in_SRB = -1;
    SP_MATCH_TYPE match_type_prev = MATCH_NONE, match_type_above = MATCH_NONE, match_type_cur = MATCH_NONE;
    int ui_max_size = 0;
    int cur_active_SRB_count = m_ui_hi_ref_UmPSize;
    if (cur_active_SRB_count >= ui_pvbuf_size)
    {
        cur_active_SRB_count = ui_pvbuf_size;
        ui_max_size = cur_active_SRB_count + m_esc_flag;
    }
    else
    {
        ui_max_size = cur_active_SRB_count + 1 + m_esc_flag;
    }

    u8 b_hi_ref_flag = 0;
    u8 b_lo_ref_flag = 0;
    SP_MATCH_TYPE match_mode_line_buf[MAX_CU_SIZE];
    unsigned int pvbuf_adr_line_buf[MAX_CU_SIZE];
    COM_SP_EVS_INFO str_info;
    u8  merge_all_comp_flag[MAX_SRB_SIZE];
    memcpy(merge_all_comp_flag, p_cur_sp_info->m_all_comp_flag, sizeof(u8)*MAX_SRB_SIZE);
    u8   merge_cuS_flag[MAX_SRB_SIZE];
    s16  merge_pv_x[MAX_SRB_SIZE];
    s16  merge_pv_y[MAX_SRB_SIZE];
    memcpy(merge_cuS_flag, p_cur_sp_info->m_cuS_flag, sizeof(u8)*MAX_SRB_SIZE);
    memcpy(merge_pv_x, p_cur_sp_info->m_pv_x, sizeof(s16)*MAX_SRB_SIZE);
    memcpy(merge_pv_y, p_cur_sp_info->m_pv_y, sizeof(s16)*MAX_SRB_SIZE);
    u8  merge_dpb_reYonly[MAX_SRB_SIZE];
    u8  merge_dpb_idx[MAX_SRB_SIZE];
    memcpy(merge_dpb_reYonly, p_cur_sp_info->m_dpb_reYonly, sizeof(u8)*MAX_SRB_SIZE);
    memcpy(merge_dpb_idx, p_cur_sp_info->m_dpb_idx, sizeof(u8)*MAX_SRB_SIZE);
    int evs_num = 0;
    int ubvs_num = 0;
    int unmatched_num = 0;
    int max_res_num = count / 4;

    for (int i = 0; i < width; i++)
    {
        match_mode_line_buf[i] = MATCH_NONE;
        pvbuf_adr_line_buf[i] = 0;
    }

    while (processed_count < count)
    {
        int curr_x;// = x_start_in_pic + processed_count%width;
        int curr_y;// = y_start_in_pic + processed_count/width;
        trav_order_index = p_trav_scan_order[processed_count];
        trav_x = GET_TRAV_X(trav_order_index, width); //assert(trav_x==(trav_order_index%width));
        trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2); //assert(trav_y==(trav_order_index/width));

        curr_x = x_start_in_pic + trav_x;
        curr_y = y_start_in_pic + trav_y;

        match_type_above = match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y];
        ui_pvbuf_adr_above = pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y];
        //do srb search
        COM_SP_PIX pixel_cur;
        pel pixelcur[N_C];
        double srb_rdcost = SP_MAX_COST;
        unsigned int   srb_dist = 0, srb_dist_y = 0, srb_dist_uv = 0;
        unsigned int   srb_bits = 0;
        int            srb_length = 0;
        int            srb_above_length = 0;
        unsigned int   srb_above_dist = 0, srb_above_dist_y = 0, srb_above_dist_uv = 0;
        unsigned int   srb_above_bits = 0;
        double         srb_above_rdcost = SP_MAX_COST;

        double d_avebits_per_pix[2] = { SP_MAX_COST,SP_MAX_COST };
        srb_length = get_srb_length(p_cur_sp_info, width, height, x_start_in_pic, y_start_in_pic, processed_count, srb_dist, srb_dist_y, srb_dist_uv, pixel_cur, match_type_cur);
        if (p_cur_sp_info->cu_ext)
        {
            srb_dist_uv = 0;
            srb_dist = srb_dist_y;
        }
        if (match_type_cur == MATCH_POS_ONE)
        {
            pixelcur[0] = pixel_cur.Y;
            pixelcur[1] = pixel_cur.U;
            pixelcur[2] = pixel_cur.V;
            pos_in_SRB = find_pos_in_hi_ref_UmP(pixelcur);
            if (pos_in_SRB == -1) { //new SRB pixel
                pos_in_SRB = get_hi_ref_UmPSize();
            }
        }
        if (match_type_cur == MATCH_NONE)
        {
            if (cur_active_SRB_count >= ui_pvbuf_size)
            {
                cur_active_SRB_count = ui_pvbuf_size;
                pos_in_SRB = ui_pvbuf_size;
                ui_max_size = cur_active_SRB_count + m_esc_flag;
            }
            else
            {
                pos_in_SRB = cur_active_SRB_count + 1;
                ui_max_size = cur_active_SRB_count + 1 + m_esc_flag;
            }

        }

        srb_bits = get_SSrb_bits(p_cur_sp_info, pixel_cur, srb_length, processed_count, count, &ui_pvbuf_adr, ui_pvbuf_adr_prev, match_type_prev, ui_pvbuf_adr_above, match_type_above, match_type_cur, pos_in_SRB, ui_max_size, width);

        if (pos_in_SRB == cur_active_SRB_count && match_type_cur == MATCH_POS_ONE)
        {
            cur_active_SRB_count++;
            if (cur_active_SRB_count >= ui_pvbuf_size)
            {
                cur_active_SRB_count = ui_pvbuf_size;
                ui_max_size = cur_active_SRB_count + m_esc_flag;
            }
            else
            {
                ui_max_size = cur_active_SRB_count + 1 + m_esc_flag;
            }
        }

        if (match_type_cur == MATCH_NONE)
        {
            int bit_depth = input->sample_bit_depth;
            if (p_cur_sp_info->cu_ext)
            {
                srb_bits += srb_length * bit_depth;
            }
            else
            {
                srb_bits += srb_length * 3 * bit_depth;
            }
        }
        srb_rdcost = srb_bits;//m_pcRdCost->calcRdCost(srb_bits,srb_dist);
        d_avebits_per_pix[0] = srb_bits * 1.0 / srb_length;
        str_info.pixel[0] = pixel_cur.Y;
        str_info.pixel[1] = pixel_cur.U;
        str_info.pixel[2] = pixel_cur.V;

        if (processed_count >= width && match_type_prev != MATCH_POS_WIDTH)
        {
            srb_above_length = get_SSrb_above_length(p_cur_sp_info, width, height, processed_count, srb_above_dist, srb_above_dist_y, srb_above_dist_uv, pixel_cur);
        }
        if (srb_above_length > 0)
        {
            srb_above_bits = get_SSrb_bits(p_cur_sp_info, pixel_cur, srb_above_length, processed_count, count, &ui_pvbuf_adr, ui_pvbuf_adr_prev, match_type_prev, ui_pvbuf_adr_above, match_type_above, MATCH_POS_WIDTH, pos_in_SRB, ui_pvbuf_size, width);
            srb_above_rdcost = srb_above_bits;
            d_avebits_per_pix[1] = srb_above_bits * 1.0 / srb_above_length;
        }

        srb_match_mode = MATCH_POS_ONE;
        if (srb_above_length > 0)
        {
            if (srb_length > 0)
            {
                if (srb_above_rdcost * srb_length <= srb_rdcost * srb_above_length)
                {
                    srb_match_mode = MATCH_POS_WIDTH;
                }
            }
            else
            {
                srb_match_mode = MATCH_POS_WIDTH;
            }

            if (srb_match_mode == MATCH_POS_WIDTH)
            {
                srb_length = srb_above_length;
                srb_bits = srb_above_bits;
                srb_dist = srb_above_dist;
                srb_dist_y = srb_above_dist_y;
                srb_dist_uv = srb_above_dist_uv;
                srb_rdcost = srb_above_rdcost;
                str_info.pixel[0] = pixel_cur.Y;
                str_info.pixel[1] = pixel_cur.U;
                str_info.pixel[2] = pixel_cur.V;
            }
        }

        tmp_dict_RD_cost = srb_dist + p_cur_sp_info->lamda * srb_bits;//m_pcRdCost->calcRdCost(srb_bits,srb_dist);//srb_rdcost; //save current best srb RDCost
        {
            str_info.is_matched = 0;
            str_info.length = srb_length;
            str_info.esc_flag = 0;
            str_info.pos = processed_count;
            if (str_info.length == -1)
            {
                str_info.length = 1;
            }
            str_info.pvflag = 0;
            if (srb_match_mode == MATCH_POS_WIDTH)
            {
                for (int pixel_count = 0; pixel_count < str_info.length; pixel_count++)
                {
                    int cur_pix_pos = p_trav_scan_order[processed_count + pixel_count];
                    trav_x = GET_TRAV_X(cur_pix_pos, width);
                    trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);

                    if (match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y] == MATCH_NONE)
                    {
                        str_info.esc_flag = 1;
                    }
                    else
                    {
                        u16 pvaddr = pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y];
                        if (!p_cur_sp_info->cu_ext && (trav_x % 2 == 0 && trav_y % 2 == 0) && p_cur_sp_info->m_dpb_reYonly[pvaddr])
                        {
                            str_info.pvflag = 1;
                            p_cur_sp_info->m_pv_x[pvaddr] = x_start_in_pic + trav_x;
                            p_cur_sp_info->m_pv_y[pvaddr] = y_start_in_pic + trav_y;
                            p_cur_sp_info->m_dpb_reYonly[pvaddr] = 0;
                            p_cur_sp_info->m_dpb_idx[pvaddr] = 0;
                        }
                    }
                }
                str_info.match_type = MATCH_POS_WIDTH; ui_pvbuf_adr_prev = 0; match_type_prev = MATCH_POS_WIDTH;
                int y0 = GET_TRAV_Y(p_trav_scan_order[processed_count], p_cur_sp_info->cu_width_log2);
                int y1 = GET_TRAV_Y(p_trav_scan_order[processed_count + str_info.length - 1], p_cur_sp_info->cu_width_log2);
                int str_height = y1 - y0 + 1;
                int above_merge_str_num = 0;
                int offset_x = 0;
                int offset_y = -1;
                if (offset_x == 0 && offset_y < 0 && -offset_y < str_height)
                {
                    above_merge_str_num = (str_height - 1) / (-offset_y) + 1; //round up for (str_height / ySv)
                }
                else
                {
                    above_merge_str_num = 1;
                }
                ubvs_num = ubvs_num + above_merge_str_num;
            }
            else
            {
                if (match_type_cur == MATCH_POS_ONE)
                {
                    b_hi_ref_flag = 1;
                    pos_in_SRB = find_pos_in_hi_ref_UmP(str_info.pixel, 1);
                    if (pos_in_SRB == -1)
                    {
                        //new unmatched pixel
                        str_info.srb_index = get_hi_ref_UmPSize() - 1;
                        p_cur_sp_info->m_all_comp_flag[str_info.srb_index] = p_cur_sp_info->cu_ext ? 1 : 0;
                        p_cur_sp_info->m_pv_x[str_info.srb_index] = curr_x;
                        p_cur_sp_info->m_pv_y[str_info.srb_index] = curr_y;
                        p_cur_sp_info->m_dpb_reYonly[str_info.srb_index] = 0;
                        p_cur_sp_info->m_dpb_idx[str_info.srb_index] = 0;
                    }
                    else
                    {
                        str_info.srb_index = pos_in_SRB;
                    }
                    if ((width == 4) && (height == 4))
                    {
                        assert(str_info.srb_index <= EVS_PV_4);
                    }
                    else if ((width == 4 && height == 8) || (width == 8 && height == 4))
                    {
                        assert(str_info.srb_index <= EVS_PV_8);
                    }
                    else if ((width == 4 && height == 16) || (width == 16 && height == 4))
                    {
                        assert(str_info.srb_index <= EVS_PV_MAX);
                    }
                    else if ((width == 4 && height == 32) || (width == 32 && height == 4))
                    {
                        assert(str_info.srb_index <= MAX_SRB_SIZE);
                    }
                    else
                    if ((width == 8) && (height == 8))
                    {
                        assert(str_info.srb_index <= EVS_PV_MAX);
                    }
                    if (!p_cur_sp_info->cu_ext && !p_cur_sp_info->m_all_comp_flag[str_info.srb_index])
                    {
                        int log2size = p_cur_sp_info->ctu_log2size < 7 ? p_cur_sp_info->ctu_log2size : 6;
                        if (((p_cur_sp_info->m_pv_x[str_info.srb_index] >> log2size) != (x_start_in_pic >> log2size)) || ((p_cur_sp_info->m_pv_y[str_info.srb_index] >> log2size) != (y_start_in_pic >> log2size)))
                        {
                            p_cur_sp_info->m_pv_x[str_info.srb_index] = curr_x;
                            p_cur_sp_info->m_pv_y[str_info.srb_index] = curr_y;
                            p_cur_sp_info->m_dpb_idx[str_info.srb_index] = 0;
                        }
                    }
                    else if (!p_cur_sp_info->cu_ext && p_cur_sp_info->m_all_comp_flag[str_info.srb_index] && p_cur_sp_info->m_dpb_idx[str_info.srb_index])
                    {
                        p_cur_sp_info->m_dpb_reYonly[str_info.srb_index] = 1;
                        p_cur_sp_info->m_pv_x[str_info.srb_index] = curr_x;
                        p_cur_sp_info->m_pv_y[str_info.srb_index] = curr_y;
                    }
                    for (int i = 0; i < str_info.length; i++)
                    {
                        int cur_pix_pos = p_trav_scan_order[processed_count + i];
                        int first_trav_x = GET_TRAV_X(cur_pix_pos, width);
                        int first_trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);
                        if (!((first_trav_x & 1) || (first_trav_y & 1)) && !p_cur_sp_info->m_all_comp_flag[str_info.srb_index] && !p_cur_sp_info->cu_ext)
                        {
                            p_cur_sp_info->m_all_comp_flag[str_info.srb_index] = 1;
                            p_cur_sp_info->m_pv_x[str_info.srb_index] = x_start_in_pic + first_trav_x;
                            p_cur_sp_info->m_pv_y[str_info.srb_index] = y_start_in_pic + first_trav_y;
                            str_info.pvflag = 1;
                        }
                        else if (!((first_trav_x & 1) || (first_trav_y & 1)) && p_cur_sp_info->m_all_comp_flag[str_info.srb_index] && !p_cur_sp_info->cu_ext)
                        {
                            int log2size = p_cur_sp_info->ctu_log2size < 7 ? p_cur_sp_info->ctu_log2size : 6;

                            if (((p_cur_sp_info->m_pv_x[str_info.srb_index] >> log2size) != (x_start_in_pic >> log2size)) || ((p_cur_sp_info->m_pv_y[str_info.srb_index] >> log2size) != (y_start_in_pic >> log2size)) || p_cur_sp_info->m_dpb_idx[str_info.srb_index])
                            {
                                p_cur_sp_info->m_pv_x[str_info.srb_index] = x_start_in_pic + first_trav_x;
                                p_cur_sp_info->m_pv_y[str_info.srb_index] = y_start_in_pic + first_trav_y;
                                p_cur_sp_info->m_dpb_reYonly[str_info.srb_index] = 0;
                                p_cur_sp_info->m_dpb_idx[str_info.srb_index] = 0;
                                str_info.pvflag = 1;
                            }
                        }
                    }
                    // reconstruction
                    for (int pixel_count = 0; pixel_count < str_info.length; pixel_count++)
                    {
                        int curr_x;
                        int curr_y;
                        int cur_pix_pos = p_trav_scan_order[processed_count + pixel_count];
                        trav_x = GET_TRAV_X(cur_pix_pos, width);
                        trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);

                        curr_x = x_start_in_pic + trav_x;//cur_pix_pos % width;
                        curr_y = y_start_in_pic + trav_y;//cur_pix_pos / width;

                        match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y] = MATCH_POS_ONE;
                        pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y] = str_info.srb_index;
                    }
                    //
                    str_info.match_type = MATCH_POS_ONE; ui_pvbuf_adr_prev = str_info.srb_index; match_type_prev = MATCH_POS_ONE;
                    evs_num++;
                }
                else
                {
                    str_info.esc_flag = 1;
                    str_info.srb_index = ui_max_pvbuf_size;
                    b_lo_ref_flag = 1;
                    for (int pixel_count = 0; pixel_count < str_info.length; pixel_count++)
                    {
                        int cur_pix_pos = p_trav_scan_order[processed_count + pixel_count];
                        trav_x = GET_TRAV_X(cur_pix_pos, width);
                        trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);
                        COM_SP_PIX unmatch_pixel = p_CQ_buf[cur_pix_pos];
                        COM_SP_PIX pixel_org = unmatch_pixel;
                        vec_dict_pixel.push_back(unmatch_pixel);
                        str_info.pixel[0] = unmatch_pixel.Y;
                        str_info.pixel[1] = unmatch_pixel.U;
                        str_info.pixel[2] = unmatch_pixel.V;
                        match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y] = MATCH_NONE;
                        pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y] = ui_pvbuf_size;
                    }
                    str_info.match_type = MATCH_NONE; 
                    ui_pvbuf_adr_prev = ui_pvbuf_size;
                    match_type_prev = MATCH_NONE;
                    unmatched_num += str_info.length;
                }
            }
            str_info.srb_dist = srb_dist;
            vec_dict_info.push_back(str_info);
            processed_count += str_info.length;
        }

        dict_dist += srb_dist;
        dict_dist_y += srb_dist_y;
        dict_dist_uv += srb_dist_uv;
        assert(srb_dist == (srb_dist_y + srb_dist_uv));
        dict_bits += srb_bits;
        dict_RD_cost = (dict_dist_y + dict_dist_uv) + p_cur_sp_info->lamda * dict_bits;
        if ((evs_num + ubvs_num + unmatched_num) > max_res_num)
        {
            return FALSE;
        }      
    }

    if (!b_hi_ref_flag)
    {
        return FALSE;
    }
    assert(m_esc_flag == b_lo_ref_flag);
    //*distortion = dict_dist;
    set_hi_ref_flag(b_hi_ref_flag ? 1 : 0);
    set_lo_ref_flag(b_lo_ref_flag ? 1 : 0);
    //
    save_SRB(p_cur_sp_info);
    p_cur_sp_info->sub_string_no = (int)vec_dict_info.size();
    for (int i = 0; i < p_cur_sp_info->sub_string_no; i++)
    {
        p_cur_sp_info->p_evs_copy_info[i] = vec_dict_info.at(i);
    }
    p_cur_sp_info->unpredict_pix_num = (int)vec_dict_pixel.size();
    for (int i = 0; i < p_cur_sp_info->unpredict_pix_num; i++)
    {
        p_cur_sp_info->unpredict_pix_info[i] = vec_dict_pixel.at(i);
    }
    //get org bits and rd cost
    SBAC_LOAD(p_cur_sp_info->s_temp_run, p_cur_sp_info->s_curr_best[p_cur_sp_info->cu_width_log2 - 2][p_cur_sp_info->cu_height_log2 - 2]);
    unsigned int ui_bits_org = enc_get_bit_number(&p_cur_sp_info->s_temp_run);
    enc_bit_est_cs2(p_cur_sp_info->cu_width_log2, p_cur_sp_info->cu_height_log2, &p_cur_sp_info->bs_temp, p_cur_sp_info
        , ctx
        , x_start_in_pic, y_start_in_pic
    );
    ui_bits_org = enc_get_bit_number(&p_cur_sp_info->s_temp_run) - ui_bits_org;

    decode_evs_ubvs(p_cur_sp_info, width, height, x_start_in_pic, y_start_in_pic, is_horizontal_scanning);
    unsigned int org_dist_y, org_dist_uv;
    get_cu_dist(p_cur_sp_info, width, height, x_start_in_pic, y_start_in_pic, &org_dist_y, &org_dist_uv);
    unsigned int org_dist = org_dist_y + (org_dist_uv /** ctx->dist_chroma_weight[0]*/);
    if (p_cur_sp_info->cu_ext)
    {
        assert(org_dist_uv == 0);
        org_dist = org_dist_y;
    }
    double d_rdcost_org = org_dist + p_cur_sp_info->lamda * ui_bits_org;
    unsigned int ui_sizelimit = 192;
    if (width > 16) ui_sizelimit = 768;
    if (ui_pvbuf_size > 2 && vec_dict_info.size() > ((unsigned int)width >> 1) && vec_dict_info.size() < ui_sizelimit)
    {
        //check merge
        unsigned int dict_dist_new = 0;
        unsigned int calc_erro_bits = 0;
        vector<COM_SP_EVS_INFO> vec_dict_info_new;
        u8  org_all_comp_flag[MAX_SRB_SIZE];
        memcpy(org_all_comp_flag, p_cur_sp_info->m_all_comp_flag, sizeof(u8)*MAX_SRB_SIZE);
        u8  org_dpb_reYonly[MAX_SRB_SIZE];
        u8  org_dpb_idx[MAX_SRB_SIZE];
        memcpy(org_dpb_reYonly, p_cur_sp_info->m_dpb_reYonly, sizeof(u8)*MAX_SRB_SIZE);
        memcpy(org_dpb_idx, p_cur_sp_info->m_dpb_idx, sizeof(u8)*MAX_SRB_SIZE);
        u8   org_cuS_flag[MAX_SRB_SIZE];
        s16  org_pv_x[MAX_SRB_SIZE];
        s16  org_pv_y[MAX_SRB_SIZE];
        memcpy(org_cuS_flag, p_cur_sp_info->m_cuS_flag, sizeof(u8)*MAX_SRB_SIZE);
        memcpy(org_pv_x, p_cur_sp_info->m_pv_x, sizeof(s16)*MAX_SRB_SIZE);
        memcpy(org_pv_y, p_cur_sp_info->m_pv_y, sizeof(s16)*MAX_SRB_SIZE);
        pel *p_cluster[3];
        p_cluster[0] = p_cur_sp_info->m_srb[0];
        p_cluster[1] = p_cur_sp_info->m_srb[1];
        p_cluster[2] = p_cur_sp_info->m_srb[2];
        regen_cluster_info(p_cur_sp_info, p_cluster, ui_pvbuf_size, width, height, x_start_in_pic, y_start_in_pic/*, bLossless*/, m_ind_error, calc_erro_bits);
        pre_calc_RD_merge(p_cur_sp_info, width, height, p_CQ_buf, vec_dict_info_new, dict_dist_new, calc_erro_bits);
        p_cur_sp_info->sub_string_no = (int)vec_dict_info_new.size();
        for (int i = 0; i < p_cur_sp_info->sub_string_no; i++)
        {
            p_cur_sp_info->p_evs_copy_info[i] = vec_dict_info_new.at(i);
        }

        SBAC_LOAD(p_cur_sp_info->s_temp_run, p_cur_sp_info->s_curr_best[p_cur_sp_info->cu_width_log2 - 2][p_cur_sp_info->cu_height_log2 - 2]);
        unsigned int ui_bits_merge = enc_get_bit_number(&p_cur_sp_info->s_temp_run);
        enc_bit_est_cs2(p_cur_sp_info->cu_width_log2, p_cur_sp_info->cu_height_log2, &p_cur_sp_info->bs_temp, p_cur_sp_info
            , ctx
            , x_start_in_pic, y_start_in_pic
        );
        ui_bits_merge = enc_get_bit_number(&p_cur_sp_info->s_temp_run) - ui_bits_merge;

        memcpy(p_cur_sp_info->m_all_comp_flag, merge_all_comp_flag, sizeof(u8)*MAX_SRB_SIZE);
        memcpy(p_cur_sp_info->m_cuS_flag, merge_cuS_flag, sizeof(u8)*MAX_SRB_SIZE);
        memcpy(p_cur_sp_info->m_pv_x, merge_pv_x, sizeof(s16)*MAX_SRB_SIZE);
        memcpy(p_cur_sp_info->m_pv_y, merge_pv_y, sizeof(s16)*MAX_SRB_SIZE);
        int evs_n_m4 = 0;
        int ubvs_n_m4 = 0;
        int unpixel_n_m4 = 0;
        memcpy(p_cur_sp_info->m_dpb_reYonly, merge_dpb_reYonly, sizeof(u8)*MAX_SRB_SIZE);
        memcpy(p_cur_sp_info->m_dpb_idx, merge_dpb_idx, sizeof(u8)*MAX_SRB_SIZE);
        SP_MATCH_TYPE m_match_mode_line_buf[MAX_CU_SIZE];
        unsigned int m_pvbuf_adr_line_buf[MAX_CU_SIZE];
        for (int i = 0; i < width; i++)
        {
            m_match_mode_line_buf[i] = MATCH_NONE;
            m_pvbuf_adr_line_buf[i] = 0;
        }
        COM_SP_EVS_INFO *string_info;
        for (int i = 0; i < p_cur_sp_info->sub_string_no; i++)
        {
            string_info = &p_cur_sp_info->p_evs_copy_info[i];
            string_info->pvflag = 0;
            if (string_info->match_type == MATCH_POS_ONE)
            {
                evs_n_m4++;
                int cur_pix_pos = p_trav_scan_order[string_info->pos];
                int first_trav_x = GET_TRAV_X(cur_pix_pos, width);
                int first_trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);
                if ((p_cur_sp_info->m_pv_x[string_info->srb_index] == -1) && (p_cur_sp_info->m_pv_y[string_info->srb_index] == -1))
                {
                    p_cur_sp_info->m_all_comp_flag[string_info->srb_index] = p_cur_sp_info->cu_ext ? 1 : 0;
                    p_cur_sp_info->m_pv_x[string_info->srb_index] = x_start_in_pic + first_trav_x;
                    p_cur_sp_info->m_pv_y[string_info->srb_index] = y_start_in_pic + first_trav_y;
                    p_cur_sp_info->m_dpb_idx[string_info->srb_index] = 0;
                    p_cur_sp_info->m_dpb_reYonly[string_info->srb_index] = 0;
                }

                if (!p_cur_sp_info->cu_ext && !p_cur_sp_info->m_all_comp_flag[string_info->srb_index])
                {
                    int log2size = p_cur_sp_info->ctu_log2size < 7 ? p_cur_sp_info->ctu_log2size : 6;
                    if (((p_cur_sp_info->m_pv_x[string_info->srb_index] >> log2size) != (x_start_in_pic >> log2size)) || ((p_cur_sp_info->m_pv_y[string_info->srb_index] >> log2size) != (y_start_in_pic >> log2size)))
                    {
                        p_cur_sp_info->m_pv_x[string_info->srb_index] = x_start_in_pic + first_trav_x;
                        p_cur_sp_info->m_pv_y[string_info->srb_index] = y_start_in_pic + first_trav_y;
                        p_cur_sp_info->m_dpb_idx[string_info->srb_index] = 0;
                    }
                }
                else if (!p_cur_sp_info->cu_ext && p_cur_sp_info->m_all_comp_flag[string_info->srb_index] && p_cur_sp_info->m_dpb_idx[string_info->srb_index])
                {
                    p_cur_sp_info->m_dpb_reYonly[string_info->srb_index] = 1;
                    p_cur_sp_info->m_pv_x[string_info->srb_index] = x_start_in_pic + first_trav_x;
                    p_cur_sp_info->m_pv_y[string_info->srb_index] = y_start_in_pic + first_trav_y;
                }

                for (int i = 0; i < string_info->length; i++)
                {
                    int cur_pix_pos = p_trav_scan_order[string_info->pos + i];
                    int trav_x = GET_TRAV_X(cur_pix_pos, width);
                    int trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);

                    m_match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y] = MATCH_POS_ONE;
                    m_pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y] = string_info->srb_index;

                    if (!((trav_x & 1) || (trav_y & 1)) && !p_cur_sp_info->m_all_comp_flag[string_info->srb_index] && !p_cur_sp_info->cu_ext)
                    {
                        p_cur_sp_info->m_all_comp_flag[string_info->srb_index] = 1;
                        p_cur_sp_info->m_pv_x[string_info->srb_index] = x_start_in_pic + trav_x;
                        p_cur_sp_info->m_pv_y[string_info->srb_index] = y_start_in_pic + trav_y;
                        string_info->pvflag = 1;
                    }
                    if (!((trav_x & 1) || (trav_y & 1)) && p_cur_sp_info->m_all_comp_flag[string_info->srb_index] && !p_cur_sp_info->cu_ext)
                    {
                        int log2size = p_cur_sp_info->ctu_log2size < 7 ? p_cur_sp_info->ctu_log2size : 6;
                        if ((p_cur_sp_info->m_pv_x[string_info->srb_index] >> log2size) != (x_start_in_pic >> log2size) || (p_cur_sp_info->m_pv_y[string_info->srb_index] >> log2size) != (y_start_in_pic >> log2size) || p_cur_sp_info->m_dpb_idx[string_info->srb_index])
                        {
                            string_info->pvflag = 1;
                            p_cur_sp_info->m_pv_x[string_info->srb_index] = x_start_in_pic + trav_x;
                            p_cur_sp_info->m_pv_y[string_info->srb_index] = y_start_in_pic + trav_y;
                            p_cur_sp_info->m_dpb_idx[string_info->srb_index] = 0;
                            p_cur_sp_info->m_dpb_reYonly[string_info->srb_index] = 0;
                        }
                    }
                }
            }
            else if (string_info->match_type == MATCH_POS_WIDTH)
            {
                int y0 = GET_TRAV_Y(p_trav_scan_order[string_info->pos], p_cur_sp_info->cu_width_log2);
                int y1 = GET_TRAV_Y(p_trav_scan_order[string_info->pos + string_info->length - 1], p_cur_sp_info->cu_width_log2);
                int str_height = y1 - y0 + 1;
                int above_merge_str_num = 0;
                int offset_x = 0;
                int offset_y = -1;
                if (offset_x == 0 && offset_y < 0 && -offset_y < str_height)
                {
                    above_merge_str_num = (str_height - 1) / (-offset_y) + 1; //round up for (str_height / ySv)
                }
                else
                {
                    above_merge_str_num = 1;
                }
                ubvs_n_m4 = ubvs_n_m4 + above_merge_str_num;
                for (int i = 0; i < string_info->length; i++)
                {
                    int cur_pix_pos = p_trav_scan_order[string_info->pos + i];
                    int trav_x = GET_TRAV_X(cur_pix_pos, width);
                    int trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);

                    if (m_match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y] == MATCH_POS_ONE)
                    {
                        u16 pvaddr = m_pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y];
                        if (!p_cur_sp_info->cu_ext && (trav_x % 2 == 0 && trav_y % 2 == 0) && p_cur_sp_info->m_dpb_reYonly[pvaddr])
                        {
                            string_info->pvflag = 1;
                            p_cur_sp_info->m_pv_x[pvaddr] = x_start_in_pic + trav_x;
                            p_cur_sp_info->m_pv_y[pvaddr] = y_start_in_pic + trav_y;
                            p_cur_sp_info->m_dpb_reYonly[pvaddr] = 0;
                            p_cur_sp_info->m_dpb_idx[pvaddr] = 0;
                        }
                    }
                }
            }
            else
            {
                assert(string_info->match_type == MATCH_NONE);
                unpixel_n_m4 += string_info->length;
                for (int i = 0; i < string_info->length; i++)
                {
                    int cur_pix_pos = p_trav_scan_order[string_info->pos + i];
                    int trav_x = GET_TRAV_X(cur_pix_pos, width);
                    int trav_y = GET_TRAV_Y(cur_pix_pos, p_cur_sp_info->cu_width_log2);
                    m_match_mode_line_buf[is_horizontal_scanning ? trav_x : trav_y] = MATCH_NONE;
                    m_pvbuf_adr_line_buf[is_horizontal_scanning ? trav_x : trav_y] = string_info->srb_index;
                }
            }
        }
        int M4_num = evs_n_m4 + ubvs_n_m4 + unpixel_n_m4;
        if (M4_num > max_res_num)
        {
            return FALSE;
        }
        decode_evs_ubvs(p_cur_sp_info, width, height, x_start_in_pic, y_start_in_pic, is_horizontal_scanning);
        unsigned int merge_dist_y, merge_dist_uv;
        get_cu_dist(p_cur_sp_info, width, height, x_start_in_pic, y_start_in_pic, &merge_dist_y, &merge_dist_uv);
        unsigned int merge_dist = merge_dist_y + (merge_dist_uv /** ctx->dist_chroma_weight[0]*/);
        if (p_cur_sp_info->cu_ext)
        {
            assert(merge_dist_uv == 0);
            merge_dist = merge_dist_y;
        }
        double d_rdcost_merge = merge_dist + p_cur_sp_info->lamda * ui_bits_merge;

        double CS2_rdcost = (d_rdcost_merge >= d_rdcost_org) ? d_rdcost_org : d_rdcost_merge;
        if (CS2_rdcost >= *cur_best_rdcost)
        {
            return    FALSE;
        }

        if (d_rdcost_merge >= d_rdcost_org)
        {
            p_cur_sp_info->sub_string_no = (int)vec_dict_info.size();
            for (int i = 0; i < p_cur_sp_info->sub_string_no; i++)
            {
                p_cur_sp_info->p_evs_copy_info[i] = vec_dict_info.at(i);
            }
            memcpy(p_cur_sp_info->m_all_comp_flag, org_all_comp_flag, sizeof(u8)*MAX_SRB_SIZE);
            memcpy(p_cur_sp_info->m_cuS_flag, org_cuS_flag, sizeof(u8)*MAX_SRB_SIZE);
            memcpy(p_cur_sp_info->m_pv_x, org_pv_x, sizeof(s16)*MAX_SRB_SIZE);
            memcpy(p_cur_sp_info->m_pv_y, org_pv_y, sizeof(s16)*MAX_SRB_SIZE);
            memcpy(p_cur_sp_info->m_dpb_reYonly, org_dpb_reYonly, sizeof(u8)*MAX_SRB_SIZE);
            memcpy(p_cur_sp_info->m_dpb_idx, org_dpb_idx, sizeof(u8)*MAX_SRB_SIZE);
            save_SRB(p_cur_sp_info);//restore the SRB info.
        }
    }
    else
    {
        if (d_rdcost_org >= *cur_best_rdcost)
        {
            return    FALSE;
        }
    }


    return 1;
}
void string_prediction::decode_evs_ubvs(COM_SP_CODING_UNIT* p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, /*Pixel* pPixels,*/ u8 is_horizontal_scanning)
{
    int processed_count = 0;
    int count = width * height;
    int* p_trav_scan_order = com_tbl_raster2trav[is_horizontal_scanning][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    unsigned int trav_order_index;
    unsigned int scale_x = 0;
    unsigned int scale_y = 0;
    u8 ui_lo_ref_flag = p_cur_sp_info->m_unpredictable_pixel_present_flag;
    unsigned int dict_pix_idx = 0;
    COM_SP_EVS_INFO *p_dict_info = p_cur_sp_info->p_evs_copy_info;
    COM_SP_PIX *p_dict_pixel = p_cur_sp_info->unpredict_pix_info;
    int dict_pix_size = p_cur_sp_info->unpredict_pix_num;
    int str_info_size = p_cur_sp_info->sub_string_no;
    SP_MATCH_TYPE m_match_mode_line_buf[MAX_CU_SIZE];
    unsigned int m_pvbuf_adr_line_buf[MAX_CU_SIZE];
    for (int i = 0; i < width; i++)
    {
        m_match_mode_line_buf[i] = MATCH_NONE;
        m_pvbuf_adr_line_buf[i] = 0;
    }
    for (int i = 0; i < str_info_size; i++)
    {
        int length = p_dict_info[i].length;
        if (p_dict_info[i].match_type == MATCH_POS_WIDTH)
        {
            int offset_x = is_horizontal_scanning ? 0 : 1;
            int offset_y = is_horizontal_scanning ? 1 : 0;
            for (int j = 0; j < length; j++)
            {
                trav_order_index = p_trav_scan_order[processed_count];
                int curr_x = GET_TRAV_X(trav_order_index, width);
                int curr_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                int cur_pos = curr_y * width + curr_x;
                int cur_pos_c = (curr_y >> scale_y)*width + (curr_x >> scale_x);
                int ref_pos = (curr_y - offset_y)*width + (curr_x - offset_x);
                int ref_pos_c = ((curr_y - offset_y) >> scale_y)*width + ((curr_x - offset_x) >> scale_x);
                {
                    m_p_pel_rec444[0][cur_pos] = m_p_pel_rec444[0][ref_pos];
                    if (!p_cur_sp_info->cu_ext)
                    {
                     
                        if (!((curr_x & scale_x) || (curr_y & scale_y)))
                        {
                            m_p_pel_rec444[1][cur_pos_c] = m_p_pel_rec444[1][ref_pos_c];
                            m_p_pel_rec444[2][cur_pos_c] = m_p_pel_rec444[2][ref_pos_c];
                            m_all_comp_rec444[cur_pos_c] = m_all_comp_rec444[ref_pos_c];
                        }
                    }
                 
                }
                int curr_offset_x = x_start_in_pic + curr_x;
                int curr_offset_y = y_start_in_pic + curr_y;
                if (m_match_mode_line_buf[is_horizontal_scanning ? curr_x : curr_y] == MATCH_POS_ONE)
                {
                    u16 pvaddr = m_pvbuf_adr_line_buf[is_horizontal_scanning ? curr_x : curr_y];
                    if ((curr_offset_x) == (p_cur_sp_info->m_pv_x[pvaddr]) && (curr_offset_y) == (p_cur_sp_info->m_pv_y[pvaddr]))
                    {
                        assert(p_cur_sp_info->m_pv_x[pvaddr] % 2 == 0 && p_cur_sp_info->m_pv_y[pvaddr] % 2 == 0);
                        if (!p_cur_sp_info->cu_ext)
                        {
                            if (!((curr_x & scale_x) || (curr_y & scale_y)))
                            {
                                m_p_pel_rec444[1][cur_pos_c] = p_cur_sp_info->m_srb[1][pvaddr];
                                m_p_pel_rec444[2][cur_pos_c] = p_cur_sp_info->m_srb[2][pvaddr];
                                m_all_comp_rec444[cur_pos_c] = 1;
                            }
                        }
                        m_pv_flag[cur_pos] = 1;
                    }
                    else
                    {
                        m_pv_flag[cur_pos] = 0;
                    }
                }
                processed_count++;
            }
        }
        else
        {
            int pv_flag_exist = 0;
            for (int j = 0; j < length; j++)
            {
                trav_order_index = p_trav_scan_order[processed_count];
                int curr_x = GET_TRAV_X(trav_order_index, width);
                int curr_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                int curr_offset_x = x_start_in_pic + curr_x;
                int curr_offset_y = y_start_in_pic + curr_y;
                int cur_pos = curr_y * width + curr_x;
                int cur_pos_c = (curr_y >> scale_y)*width + (curr_x >> scale_x);
                if (p_dict_info[i].match_type == MATCH_NONE)
                {
                    m_match_mode_line_buf[is_horizontal_scanning ? curr_x : curr_y] = MATCH_NONE;
                    m_pvbuf_adr_line_buf[is_horizontal_scanning ? curr_x : curr_y] = p_dict_info[i].srb_index;
                    COM_SP_PIX dict_pixel = p_dict_pixel[dict_pix_idx++];
                    {
                        unsigned int Y = dict_pixel.Y;
                        unsigned int U = dict_pixel.U;
                        unsigned int V = dict_pixel.V;
                        m_p_pel_rec444[0][cur_pos] = Y;
                        if (!p_cur_sp_info->cu_ext)
                        {
                        
                            if (!((curr_x & 1) || (curr_y & 1)))
                            {
                                m_p_pel_rec444[1][cur_pos_c] = U;
                                m_p_pel_rec444[2][cur_pos_c] = V;
                                m_all_comp_rec444[cur_pos_c] = 1;
                            }
                            else
                            {
                                m_p_pel_rec444[1][cur_pos_c] = 0;
                                m_p_pel_rec444[2][cur_pos_c] = 0;
                                m_all_comp_rec444[cur_pos_c] = 0;
                            }
                        }
                        m_pv_flag[cur_pos] = 0;
                    }
                }
                else
                {
                    m_match_mode_line_buf[is_horizontal_scanning ? curr_x : curr_y] = MATCH_POS_ONE;
                    m_pvbuf_adr_line_buf[is_horizontal_scanning ? curr_x : curr_y] = p_dict_info[i].srb_index;
                    if (pv_flag_exist)
                    {
                        m_pv_flag[cur_pos] = 0;
                    }
                    else
                    {
                        if (!p_cur_sp_info->cu_ext)
                        {
                            int log2size = p_cur_sp_info->ctu_log2size < 7 ? p_cur_sp_info->ctu_log2size : 6;
                            if ((curr_x & 1) == 0 && (curr_y & 1) == 0)
                            {
                                assert((curr_offset_x >> log2size) == (p_cur_sp_info->m_pv_x[p_dict_info[i].srb_index] >> log2size) && (curr_offset_y >> log2size) == (p_cur_sp_info->m_pv_y[p_dict_info[i].srb_index] >> log2size));
                                if (p_dict_info[i].pvflag)
                                {
                                    m_pv_flag[cur_pos] = 1;
                                    pv_flag_exist = 1;
                                }
                                else
                                {
                                    m_pv_flag[cur_pos] = 0;
                                }

                            }
                        }
                    }
                    m_p_pel_rec444[0][cur_pos] = p_dict_info[i].pixel[0];
                    if (!p_cur_sp_info->cu_ext)
                    {
                        if (p_cur_sp_info->m_pv_x[p_dict_info[i].srb_index] % 2 == 0 && p_cur_sp_info->m_pv_y[p_dict_info[i].srb_index] % 2 == 0)
                        {
                            m_p_pel_rec444[1][cur_pos_c] = p_dict_info[i].pixel[1];
                            m_p_pel_rec444[2][cur_pos_c] = p_dict_info[i].pixel[2];
                            m_all_comp_rec444[cur_pos_c] = 1;
                        }
                        else
                        {
                            m_p_pel_rec444[1][cur_pos_c] = 0;
                            m_p_pel_rec444[2][cur_pos_c] = 0;
                            m_all_comp_rec444[cur_pos_c] = 0;
                        }
                    }

                }
                processed_count++;
            }
        }
    }

    assert(processed_count == count);
    if (ui_lo_ref_flag)
    {
        assert(dict_pix_idx == dict_pix_size);//pDictPixel->size());
    }
}

void string_prediction::get_cu_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int width, int height, int x_start_in_pic, int y_start_in_pic, unsigned int*  distortion_y, unsigned int*  distortion_uv)
{
    unsigned int scale_x = 0;
    unsigned int scale_y = 0;
    unsigned int shift_y = (input->sample_bit_depth == 8) ? 0 : (input->sample_bit_depth - 8) << 1;
    unsigned int shift_c = (input->sample_bit_depth == 8) ? 0 : (input->sample_bit_depth - 8) << 1;
    unsigned int ui_sum = 0, processed_count = 0;
    u8 is_horizontal_scanning = p_cur_sp_info->string_copy_direction;
    u8 b_cs2_mode_flag = p_cur_sp_info->m_cs2_mode_flag;
    int* p_trav_scan_order = com_tbl_raster2trav[is_horizontal_scanning][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    unsigned int trav_order_index;
    int trav_x, trav_y;
    int curr_x, curr_y;
    int cur_pos, cur_pos_c, cur_rec_pos, cur_rec_pos_c;
    COM_SP_PIX pixel_org, pixel_rec;
    unsigned int cur_dist_y, cur_dist_uv;

    *distortion_y = 0;
    *distortion_uv = 0;
    //444 to 420
    int cu_start_x = p_cur_sp_info->cu_pix_x;
    int cu_start_y = p_cur_sp_info->cu_pix_y;
    {
        for (int ui_y = 0; ui_y < height; ui_y++)
        {
            for (int ui_x = 0; ui_x < width; ui_x++)
            {
                unsigned int ui_pos = (cu_start_y + ui_y) * m_stride_rec[0] + (ui_x + cu_start_x);
                unsigned int cur_pos = ui_y * width + ui_x;
                m_p_pel_rec[0][ui_pos] = m_p_pel_rec444[0][cur_pos];
            }
        }
        if (!p_cur_sp_info->cu_ext)
        {
            for (int ch = 1; ch < 3; ch++)
            {
                for (int h = 0; h < height / 2; h++)
                {
                    for (int w = 0; w < width / 2; w++)
                    {
                        if (m_pv_flag[h * 2 * width + 2 * w] == 1)
                        {
                            m_p_pel_rec[ch][((cu_start_y >> 1) + h) * m_stride_rec[ch] + (cu_start_x >> 1) + w] =
                                (pel)m_p_pel_rec444[ch][h * 2 * width + 2 * w];
                        }
                        else
                        {
                            double cnt = 0;
                            cnt = m_all_comp_rec444[h * 2 * width + 2 * w] +
                                m_all_comp_rec444[h * 2 * width + 2 * w + 1] +
                                m_all_comp_rec444[(h * 2 + 1) * width + 2 * w] +
                                m_all_comp_rec444[(h * 2 + 1) * width + 2 * w + 1];

                            m_p_pel_rec[ch][((cu_start_y >> 1) + h) * m_stride_rec[ch] + (cu_start_x >> 1) + w] = (pel)(
                                (m_p_pel_rec444[ch][h * 2 * width + 2 * w] +
                                    m_p_pel_rec444[ch][h * 2 * width + 2 * w + 1] +
                                    m_p_pel_rec444[ch][(h * 2 + 1) * width + 2 * w] +
                                    m_p_pel_rec444[ch][(h * 2 + 1) * width + 2 * w + 1] + cnt / 2) / cnt);
                        }
                    }
                }
            }
        }
    }

    //dist_y
    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            trav_order_index = p_trav_scan_order[processed_count];
            trav_x = GET_TRAV_X(trav_order_index, width);
            trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
            curr_x = x_start_in_pic + i;
            curr_y = y_start_in_pic + j;
            cur_pos = curr_y * m_stride_org[0] + curr_x;
            cur_rec_pos = curr_y * m_stride_rec[0] + curr_x;
            pixel_org.Y = m_p_pel_org[0][cur_pos];
            pixel_rec.Y = m_p_pel_rec[0][cur_rec_pos];
            pixel_org.U = 0; pixel_org.V = 0;
            pixel_rec.U = 0; pixel_rec.V = 0;
            *distortion_y += calc_dist(pixel_org, pixel_rec, cur_dist_y, cur_dist_uv);
            processed_count++;
        }
    }
    //dist_uv
    if (!p_cur_sp_info->cu_ext)
    {
        processed_count = 0;
        scale_x = 1;
        scale_y = 1;
        for (int j = 0; j < (height >> 1); j++)
        {
            for (int i = 0; i < (width >> 1); i++)
            {
                curr_x = (x_start_in_pic >> scale_x) + i;
                curr_y = (y_start_in_pic >> scale_y) + j;
                cur_pos_c = curr_y * m_stride_org[1] + curr_x;
                cur_rec_pos_c = curr_y * m_stride_rec[1] + curr_x;
                pixel_org.U = m_p_pel_org[1][cur_pos_c]; pixel_org.V = m_p_pel_org[2][cur_pos_c];
                pixel_rec.U = m_p_pel_rec[1][cur_rec_pos_c]; pixel_rec.V = m_p_pel_rec[2][cur_rec_pos_c];
                pixel_org.Y = 0; pixel_rec.Y = 0;
                *distortion_uv += calc_dist(pixel_org, pixel_rec, cur_dist_y, cur_dist_uv);
                processed_count++;
            }
        }
    }
    //return ui_sum;
    //return (dist_y + dist_uv);
}
int  string_prediction::get_SSrb_index_bits(int ui_srb_index, int ui_max_symbol)
{
    return m_trunc_bin_bits[ui_srb_index][ui_max_symbol];
}

int string_prediction::get_SSrb_merge_info(COM_SP_CODING_UNIT*p_cur_sp_info, pel*p_SRB[3], int width, int height, int* p_trav_scan_order, unsigned int ui_idx, unsigned int ui_length, unsigned int& dist, int ui_srb_index, u8* index_blk, unsigned int *merge, int srb_mode)
{
    int srb_length = 0;
    unsigned int ui_total = width * height;
    unsigned int cur_dist_y, cur_dist_uv;
    *merge = 1;
    for (srb_length = 0; srb_length < (int)ui_length; srb_length++)
    {
        unsigned int ui_tra_idx = p_trav_scan_order[ui_idx + srb_length];
        COM_SP_PIX pixel_cur;//= m_p_CQ_pixbuffer[tra_idx];
        if (srb_mode == MATCH_POS_WIDTH)
        {
            unsigned int ui_srb_idx = index_blk[ui_tra_idx - width];
            if (ui_srb_idx >= MAX_SRB_SIZE)
            {
                *merge = 0;
                break;
            }
            pixel_cur.Y = p_SRB[0][ui_srb_idx];
            pixel_cur.U = p_SRB[1][ui_srb_idx];
            pixel_cur.V = p_SRB[2][ui_srb_idx];
        }
        else
        {
            pixel_cur.Y = p_SRB[0][ui_srb_index];
            pixel_cur.U = p_SRB[1][ui_srb_index];
            pixel_cur.V = p_SRB[2][ui_srb_index];
        }
        COM_SP_PIX pixel_org = m_p_org_pixel_buffer[ui_tra_idx];
        //
        if (srb_mode == MATCH_POS_ONE)
        {
            assert(m_b_major[ui_tra_idx] == 1);
        }
        dist += calc_dist_norm(pixel_org, pixel_cur, cur_dist_y, cur_dist_uv);
    }
    return srb_length;
}

int string_prediction::get_SSrb_dist(COM_SP_CODING_UNIT*p_cur_sp_info, pel*p_SRB[3], int width, int height, int* p_trav_scan_order, unsigned int ui_idx, unsigned int ui_length, unsigned int& dist, u8* index_blk)
{
    int srb_length = 0;
    unsigned int ui_total = width * height;
    unsigned int cur_dist_y, cur_dist_uv;
    for (srb_length = 0; srb_length < (int)ui_length; srb_length++)
    {
        unsigned int ui_tra_idx = p_trav_scan_order[ui_idx + srb_length];
        unsigned int ui_srb_index = index_blk[ui_tra_idx];
        //
        COM_SP_PIX pixel_org = m_p_org_pixel_buffer[ui_tra_idx];
        COM_SP_PIX pixel_cur;//= m_p_CQ_pixbuffer[tra_idx];

        u8 b_major = m_b_major[ui_tra_idx];
        if (!b_major)
        {
            pixel_cur = pixel_org;
            assert(ui_srb_index == MAX_SRB_SIZE);
        }
        else
        {
            pixel_cur.Y = p_SRB[0][ui_srb_index];
            pixel_cur.U = p_SRB[1][ui_srb_index];
            pixel_cur.V = p_SRB[2][ui_srb_index];
        }
        dist += calc_dist_norm(pixel_org, pixel_cur, cur_dist_y, cur_dist_uv);
    }
    assert(srb_length == ui_length);
    return srb_length;
}

unsigned int  string_prediction::pre_calc_RD_merge(COM_SP_CODING_UNIT* p_cur_sp_info, int ui_width, int ui_height, COM_SP_PIX* p_CQ_buf, vector<COM_SP_EVS_INFO>& vec_dict_info, unsigned int& dist_new, unsigned int calc_err_bits)
{
    int ui_idx_start, ui_tra_idx, length, ui_total = ui_height * ui_width;
    int i_max_symbol;
    COM_SP_EVS_INFO* p_dict_info = p_cur_sp_info->p_evs_copy_info;
    double rdcost_mod_run, rdcost_mod_run_min;
    unsigned int mod_run_mode, mod_run_curr_best = 0, ui_mod_pos_next_best = 0, mod_run_next_best = 0, srb_mode_merge = MATCH_POS_ONE, srb_bits_merge = 0, srb_dist_merge = 0;

    int *p_trav_scan_order = com_tbl_raster2trav[1][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    pel  *p_SRB[3] = { 0 };
    unsigned int ui_pvbuf_size = p_cur_sp_info->m_pvbuf_size;
    unsigned int ui_lo_ref_color_flag = p_cur_sp_info->m_unpredictable_pixel_present_flag;
    unsigned int ui_index_max_size = ui_pvbuf_size + ui_lo_ref_color_flag;
    unsigned int ui_dict_info_size = p_cur_sp_info->sub_string_no;
    assert(p_dict_info != NULL && ui_dict_info_size > 0);


    u8* copy_flag = (u8*)x_malloc(u8, ui_total);//new u8[ui_total];
    s8* mode_flag = (s8*)x_malloc(s8, ui_total);//new Char[ui_total];
    COM_SP_PIX* pixel_YUV = (COM_SP_PIX*)x_malloc(COM_SP_PIX, ui_total);//new Pixel[ui_total];
    memset(copy_flag, 0, sizeof(u8) * ui_total);
    memset(mode_flag, 0, sizeof(s8) * ui_total);

    unsigned int cur_pix_pos = 0;
    u8 match_type_prev = MATCH_NONE;
    unsigned int pred_index = MAX_SRB_SIZE;

    p_SRB[0] = p_cur_sp_info->m_srb[0];//getSRB(0,0);
    p_SRB[1] = p_cur_sp_info->m_srb[1];//getSRB(1,0);
    p_SRB[2] = p_cur_sp_info->m_srb[2];//getSRB(2,0);
    for (unsigned int i = 0; i<ui_dict_info_size; i++)
    {
        COM_SP_EVS_INFO& str_info = p_dict_info[i];
        assert(cur_pix_pos == str_info.pos);
        if (str_info.match_type == MATCH_POS_WIDTH)
        {
            for (int j = 0; j < str_info.length; j++)
            {
                ui_tra_idx = p_trav_scan_order[cur_pix_pos + j];
                copy_flag[ui_tra_idx - ui_width] = 1;
                m_c_index_block[ui_tra_idx] = m_c_index_block[ui_tra_idx - ui_width];
                mode_flag[ui_tra_idx] = str_info.match_type;
            }
            str_info.srb_bits = 0;//only count on the index bits
            unsigned int ui_dist_cur = 0;
            get_SSrb_dist(p_cur_sp_info, p_SRB, ui_width, ui_height, p_trav_scan_order, cur_pix_pos, str_info.length, ui_dist_cur, m_c_index_block);
            str_info.srb_dist = ui_dist_cur;
        }
        else if (str_info.match_type == MATCH_POS_ONE)
        {
            unsigned int Y = str_info.pixel[0];
            unsigned int U = str_info.pixel[1];
            unsigned int V = str_info.pixel[2];
            unsigned int ui_pvbuf_adr = MAX_SRB_SIZE;
            for (unsigned int idx = 0; idx < ui_pvbuf_size; idx++)
            {
                if (Y == p_SRB[0][idx] && U == p_SRB[1][idx] && V == p_SRB[2][idx])
                {
                    ui_pvbuf_adr = idx;
                    break;
                }
            }
            assert(ui_pvbuf_adr < ui_pvbuf_size);
            if (ui_pvbuf_adr != str_info.srb_index)
            {
                str_info.srb_index = ui_pvbuf_adr;
            }
            for (int j = 0; j < str_info.length; j++)
            {
                ui_tra_idx = p_trav_scan_order[cur_pix_pos + j];
                m_c_index_block[ui_tra_idx] = (u8)str_info.srb_index;
                mode_flag[ui_tra_idx] = str_info.match_type;
            }

            /////////
            unsigned int curr_index = str_info.srb_index;
            if (cur_pix_pos > 0)
            {
                ui_tra_idx = p_trav_scan_order[cur_pix_pos];
                if (match_type_prev == MATCH_POS_WIDTH)
                {
                    pred_index = m_c_index_block[ui_tra_idx - ui_width];
                }
                else
                {
                    pred_index = m_c_index_block[ui_tra_idx - 1];
                }
                if (curr_index >= pred_index && curr_index > 0)
                {
                    curr_index--;
                }
                str_info.srb_bits = get_SSrb_index_bits(curr_index, ui_index_max_size - 1);
            }
            else
            {
                str_info.srb_bits = get_SSrb_index_bits(curr_index, ui_index_max_size);
            }
            unsigned int ui_dist_cur = 0;
            get_SSrb_dist(p_cur_sp_info, p_SRB, ui_width, ui_height, p_trav_scan_order, cur_pix_pos, str_info.length, ui_dist_cur, m_c_index_block);
            str_info.srb_dist = ui_dist_cur;
        }
        else if (str_info.match_type == MATCH_NONE)
        {
            for (int j = 0; j < str_info.length; j++)
            {
                ui_tra_idx = p_trav_scan_order[cur_pix_pos + j];
                m_c_index_block[ui_tra_idx] = MAX_SRB_SIZE;
                mode_flag[ui_tra_idx] = str_info.match_type;
                pixel_YUV[ui_tra_idx].Y = str_info.pixel[0];
                pixel_YUV[ui_tra_idx].U = str_info.pixel[1];
                pixel_YUV[ui_tra_idx].V = str_info.pixel[2];
            }
            str_info.srb_bits = 0;
        }
        //
        match_type_prev = str_info.match_type;
        //
        cur_pix_pos += str_info.length;
    }
    assert(cur_pix_pos == ui_total);

    unsigned int ui_SRB_idx = 0;
    unsigned int str_info_idx = 0;
    ui_idx_start = 0;
    while (ui_idx_start < ui_total)
    {
        COM_SP_EVS_INFO& str_info_cur = (p_dict_info[str_info_idx]);//(p_cur_sp_info->p_evs_copy_info[strInfoIdx]);
        COM_SP_EVS_INFO& str_info_nxt = (p_dict_info[str_info_idx + 1]);//(p_cur_sp_info->p_evs_copy_info[strInfoIdx+1]);
        u8 copy_flag_cur = 0, copy_flag_nxt = 0;
        for (int i = 0; i < str_info_cur.length; i++)
        {
            ui_tra_idx = p_trav_scan_order[str_info_cur.pos + i];
            if (copy_flag[ui_tra_idx])
            {
                copy_flag_cur = 1;
                break;
            }
        }
        for (int i = 0; i < str_info_nxt.length; i++)
        {
            ui_tra_idx = p_trav_scan_order[str_info_nxt.pos + i];
            if (copy_flag[ui_tra_idx])
            {
                copy_flag_nxt = 1;
                break;
            }
        }
        assert(ui_idx_start == str_info_cur.pos);
        length = str_info_cur.length + str_info_nxt.length;

        unsigned int mod_mode = 0, merge = 0, force_merge = 0;
        unsigned int bits_min = 0, dist_min = 0;
        unsigned int merge_SRB_idx = 0;
        if (str_info_cur.esc_flag == 0 && str_info_nxt.esc_flag == 0)
        {
            double rdcost_orig = p_cur_sp_info->lamda * (double)(str_info_cur.srb_bits + str_info_nxt.srb_bits) + (str_info_cur.srb_dist + str_info_nxt.srb_dist);
            double rdcost_best_mode = rdcost_orig;
            double rdcost_merge = SP_MAX_COST, error_min = SP_MAX_COST;

            if (str_info_cur.match_type == MATCH_POS_ONE && str_info_nxt.match_type == MATCH_POS_ONE && str_info_cur.srb_index == str_info_nxt.srb_index)
            {
                force_merge = 1;
                merge_SRB_idx = str_info_cur.srb_index;
            }
            if (str_info_cur.match_type == MATCH_POS_WIDTH && str_info_nxt.match_type == MATCH_POS_WIDTH)
            {
                force_merge = 1;
            }

            mod_run_mode = 0;
            if (str_info_cur.match_type == MATCH_POS_ONE && str_info_nxt.match_type == MATCH_POS_WIDTH && force_merge == 0)
            {
                int ui_mod_pos_next, length_current, length_next, group_init, group_new;

                rdcost_mod_run_min = SP_MAX_COST;
                int start_copy = str_info_nxt.pos;
                while (start_copy > (ui_idx_start + 1) && start_copy > ui_width)
                {
                    ui_tra_idx = p_trav_scan_order[start_copy - 1];
                    if (m_c_index_block[ui_tra_idx] == m_c_index_block[ui_tra_idx - ui_width])
                    {
                        start_copy--;
                    }
                    else
                    {
                        break;
                    }
                }

                length_current = start_copy - ui_idx_start;
                length_next = length - length_current;

                group_init = g_run_golomb_groups[length_next];

                for (ui_mod_pos_next = start_copy; ui_mod_pos_next<str_info_nxt.pos; ui_mod_pos_next++)
                {
                    length_current = ui_mod_pos_next - ui_idx_start;
                    length_next = length - length_current;

                    group_new = g_run_golomb_groups[length_next];

                    if (ui_mod_pos_next == start_copy || group_new < group_init)
                    {
                        group_init = (group_new<group_init) ? group_new : group_init;
                        mod_run_mode = 1;
                        unsigned int ui_merge_bits = str_info_cur.srb_bits;
                        rdcost_mod_run = p_cur_sp_info->lamda * (double)(ui_merge_bits)+(str_info_cur.srb_dist + str_info_nxt.srb_dist);
                        if (rdcost_mod_run < rdcost_mod_run_min)
                        {
                            rdcost_mod_run_min = rdcost_mod_run;
                            ui_mod_pos_next_best = ui_mod_pos_next;
                            mod_run_curr_best = length_current;
                            mod_run_next_best = length_next;
                        }
                    }
                }
            }

            if (mod_run_mode == 1 && rdcost_mod_run_min <= rdcost_best_mode)
            {
                rdcost_best_mode = rdcost_mod_run_min;
                mod_mode = 1;
            }

            {
                unsigned int ui_idx_start_merge = ui_idx_start;

                unsigned int test_level = 0;
                if (str_info_cur.match_type == MATCH_POS_ONE)
                {
                    if (str_info_nxt.match_type == MATCH_POS_ONE && (copy_flag_cur == 0 || copy_flag_nxt == 0))
                    {
                        test_level = 1;
                    }
                    if (str_info_nxt.match_type == MATCH_POS_WIDTH && copy_flag_nxt == 0)
                    {
                        test_level = 1;
                    }
                    if (force_merge == 1)
                    {
                        test_level = 1;
                    }
                }
                unsigned int test_copy = (str_info_cur.pos >= ui_width) ? 1 : 0;
                if ((str_info_cur.match_type == MATCH_POS_ONE && copy_flag_cur == 1) || str_info_cur.match_type == MATCH_NONE)
                {
                    test_copy = 0;
                }
                if ((str_info_nxt.match_type == MATCH_POS_ONE && copy_flag_nxt == 1) || str_info_nxt.match_type == MATCH_NONE)
                {
                    test_copy = 0;
                }

                if (test_level)
                {
                    i_max_symbol = (ui_idx_start_merge > 0) ? ui_index_max_size - 1 : ui_index_max_size;
                    unsigned int ui_SRB_idx_start = 0, ui_SRB_idx_end = ui_pvbuf_size - 1;

                    if (copy_flag_cur == 1)
                    {
                        ui_SRB_idx_start = str_info_cur.srb_index;
                        ui_SRB_idx_end = str_info_cur.srb_index;
                    }
                    else if (copy_flag_nxt == 1)
                    {
                        ui_SRB_idx_start = str_info_nxt.srb_index;
                        ui_SRB_idx_end = str_info_nxt.srb_index;
                    }

                    error_min = SP_MAX_COST;
                    for (ui_SRB_idx = ui_SRB_idx_start; ui_SRB_idx <= ui_SRB_idx_end; ui_SRB_idx++)
                    {
                        if (ui_SRB_idx != pred_index)
                        { // do not allow copy mode
                            unsigned int ui_dist = 0;
                            double d_RD_test;
                            merge = 1;
                            unsigned int cur_index = ui_SRB_idx > pred_index ? ui_SRB_idx - 1 : ui_SRB_idx;
                            unsigned int cur_mode = MATCH_POS_ONE;
                            unsigned int ui_idx_bits = get_SSrb_index_bits(cur_index, i_max_symbol);
                            get_SSrb_merge_info(p_cur_sp_info, p_SRB, ui_width, ui_height, p_trav_scan_order, ui_idx_start, length, ui_dist, ui_SRB_idx, m_c_index_block, &merge, cur_mode);

                            d_RD_test = p_cur_sp_info->lamda * (double)(ui_idx_bits)+ui_dist;

                            if (d_RD_test<error_min)
                            {
                                error_min = d_RD_test;
                                dist_min = ui_dist;
                                bits_min = ui_idx_bits;
                                merge_SRB_idx = ui_SRB_idx;
                            }
                        }
                    }

                    if (merge == 1)
                    {
                        rdcost_merge = error_min;

                    }

                    if ((merge == 1 && rdcost_merge <= rdcost_best_mode) || force_merge == 1)
                    {
                        rdcost_best_mode = rdcost_merge;
                        mod_mode = 2;
                        srb_mode_merge = MATCH_POS_ONE;
                        srb_bits_merge = bits_min;
                        srb_dist_merge = dist_min;
                    }
                }
                if (test_copy)
                {
                    unsigned int ui_dist = 0, ui_idx_bits = 0;
                    unsigned int cur_index = 0;
                    unsigned int cur_mode = MATCH_POS_WIDTH;
                    get_SSrb_merge_info(p_cur_sp_info, p_SRB, ui_width, ui_height, p_trav_scan_order, ui_idx_start, length, ui_dist, cur_index, m_c_index_block, &merge, cur_mode);
                    if (str_info_cur.match_type == MATCH_POS_WIDTH && str_info_nxt.match_type == MATCH_POS_WIDTH)
                    {
                        merge = 1;
                    }
                    if (merge == 1)
                    {
                        rdcost_merge = p_cur_sp_info->lamda * (double)(ui_idx_bits) + ui_dist;
                    }

                    if (merge == 1 && rdcost_merge <= rdcost_best_mode)
                    {
                        rdcost_best_mode = rdcost_merge;
                        mod_mode = 2;
                        srb_mode_merge = MATCH_POS_WIDTH;
                        srb_bits_merge = 0;
                        srb_dist_merge = ui_dist;
                    }
                }
            }
        }

        {
            //post processing
            if (mod_mode == 0)
            {
                if ((str_info_idx + 2) >= ui_dict_info_size)
                {
                    ui_idx_start += length;
                    assert(ui_idx_start == ui_total);
                }
                else
                {
                    str_info_idx += 1;
                    ui_idx_start += str_info_cur.length;
                    //
                    match_type_prev = str_info_cur.match_type;
                    if (match_type_prev == MATCH_POS_WIDTH)
                    {
                        ui_tra_idx = p_trav_scan_order[ui_idx_start];
                        pred_index = m_c_index_block[ui_tra_idx - ui_width];
                    }
                    else
                    {
                        pred_index = str_info_cur.srb_index;
                    }
                }
            }

            else
            {
                if (mod_mode == 1)
                {
                    //get distortion
                    unsigned int ui_dist_cur = 0, ui_dist_nxt = 0;
                    get_SSrb_dist(p_cur_sp_info, p_SRB, ui_width, ui_height, p_trav_scan_order, ui_idx_start, mod_run_curr_best, ui_dist_cur, m_c_index_block);
                    get_SSrb_dist(p_cur_sp_info, p_SRB, ui_width, ui_height, p_trav_scan_order, ui_idx_start + mod_run_curr_best, mod_run_next_best, ui_dist_nxt, m_c_index_block);
                    match_type_prev = str_info_cur.match_type;
                    pred_index = str_info_cur.srb_index;
                    assert(match_type_prev == MATCH_POS_ONE);

                    str_info_cur.length = mod_run_curr_best;
                    str_info_nxt.length = mod_run_next_best;
                    str_info_nxt.pos = str_info_cur.pos + mod_run_curr_best;//changed
                    assert((mod_run_curr_best + mod_run_next_best) == length);
                    str_info_cur.srb_dist = ui_dist_cur;
                    str_info_nxt.srb_dist = ui_dist_nxt;
                    for (unsigned int j = 0; j < mod_run_next_best; j++)
                    {
                        ui_tra_idx = p_trav_scan_order[str_info_nxt.pos + j];
                        mode_flag[ui_tra_idx] = MATCH_POS_WIDTH;
                    }
                    //
                    if ((str_info_idx + 2) >= ui_dict_info_size)
                    {
                        ui_idx_start += length;
                        assert(ui_idx_start == ui_total);
                    }
                    else
                    {
                        str_info_idx += 1;
                        ui_idx_start += str_info_cur.length;
                    }
                }
                if (mod_mode == 2)
                {
                    str_info_cur.length = 0;

                    str_info_nxt.length = length;
                    str_info_nxt.pos = str_info_cur.pos;//changed
                    str_info_nxt.srb_index = merge_SRB_idx;
                    str_info_nxt.srb_bits = srb_bits_merge;
                    str_info_nxt.srb_dist = srb_dist_merge;
                    str_info_nxt.match_type = (SP_MATCH_TYPE)srb_mode_merge;

                    if (str_info_nxt.match_type == MATCH_POS_ONE)
                    {
                        str_info_nxt.pixel[0] = p_SRB[0][merge_SRB_idx];
                        str_info_nxt.pixel[1] = p_SRB[1][merge_SRB_idx];
                        str_info_nxt.pixel[2] = p_SRB[2][merge_SRB_idx];
                    }

                    for (int j = 0; j < length; j++)
                    {
                        ui_tra_idx = p_trav_scan_order[ui_idx_start + j];
                        mode_flag[ui_tra_idx] = srb_mode_merge;
                        if (srb_mode_merge == MATCH_POS_WIDTH)
                        {
                            m_c_index_block[ui_tra_idx] = m_c_index_block[ui_tra_idx - ui_width];
                            copy_flag[ui_tra_idx - ui_width] = 1;
                        }
                        else
                        {
                            m_c_index_block[ui_tra_idx] = merge_SRB_idx;
                        }
                    }

                    if ((str_info_idx + 2) >= ui_dict_info_size)
                    {
                        ui_idx_start += length;
                        assert(ui_idx_start == ui_total);
                    }
                    else
                    {
                        str_info_idx += 1;
                        ui_idx_start += 0;
                    }
                }
            }
        }
    }
    //
    COM_SP_EVS_INFO str_info_merge;
    unsigned int str_info_idx_merge = 0;
    ui_idx_start = 0;
    pred_index = MAX_SRB_SIZE;
    while (ui_idx_start<ui_total)
    {
        ui_tra_idx = p_trav_scan_order[ui_idx_start];
        unsigned int start_index = m_c_index_block[ui_tra_idx];
        unsigned int start_mode = mode_flag[ui_tra_idx];
        str_info_merge.match_type = (SP_MATCH_TYPE)start_mode;
        str_info_merge.pos = ui_idx_start;
        str_info_merge.srb_index = start_index;
        if (start_mode == MATCH_POS_ONE || start_mode == MATCH_NONE)
        {
            unsigned int cur_leng = 0;
            unsigned int cur_dist = 0;
            for (int ui_idx = ui_idx_start; ui_idx<ui_total; ui_idx++)
            {
                ui_tra_idx = p_trav_scan_order[ui_idx];
                if (m_c_index_block[ui_tra_idx] == start_index && mode_flag[ui_tra_idx] == start_mode)
                {
                    cur_leng++;
                    ui_SRB_idx = m_c_index_block[ui_tra_idx];
                    if (ui_SRB_idx >= MAX_SRB_SIZE)
                    {
                        cur_dist += m_ind_error[ui_tra_idx][MAX_SSRB_SIZE];
                    }
                    else
                    {
                        cur_dist += m_ind_error[ui_tra_idx][ui_SRB_idx];
                    }
                }
                else
                {
                    break;
                }
            }
            str_info_merge.is_matched = 0;
            str_info_merge.length = cur_leng;
            str_info_merge.srb_dist = cur_dist;
            if (start_index<ui_pvbuf_size)
            {
                str_info_merge.pixel[0] = p_SRB[0][start_index];
                str_info_merge.pixel[1] = p_SRB[1][start_index];
                str_info_merge.pixel[2] = p_SRB[2][start_index];
            }
            i_max_symbol = (ui_idx_start > 0) ? ui_index_max_size - 1 : ui_index_max_size;
            unsigned int cur_index = start_index > pred_index ? start_index - 1 : start_index;
            str_info_merge.srb_bits = get_SSrb_index_bits(cur_index, i_max_symbol);
        }
        else
        {
            unsigned int cur_leng = 0;
            unsigned int cur_dist = 0;
            assert(m_c_index_block[ui_tra_idx] == m_c_index_block[ui_tra_idx - ui_width]);
            for (int ui_idx = ui_idx_start; ui_idx<ui_total; ui_idx++)
            {
                ui_tra_idx = p_trav_scan_order[ui_idx];
                if (m_c_index_block[ui_tra_idx] == m_c_index_block[ui_tra_idx - ui_width])
                {
                    cur_leng++;
                    ui_SRB_idx = m_c_index_block[ui_tra_idx];
                    if (ui_SRB_idx >= MAX_SRB_SIZE)
                    {
                        if (!m_b_major[ui_tra_idx])
                        {
                            unsigned int ui_tra_idx_abv = ui_tra_idx - ui_width;
                            COM_SP_PIX pixel_cur = m_p_CQ_pixbuffer[ui_tra_idx];
                            COM_SP_PIX pixel_abv = m_p_CQ_pixbuffer[ui_tra_idx_abv];
                            int errorlimit = 1;
                            if (abs(pixel_cur.Y - pixel_abv.Y) >= errorlimit || abs(pixel_cur.U - pixel_abv.U) >= errorlimit || abs(pixel_cur.V - pixel_abv.V) >= errorlimit)
                            {
                                cur_leng--;
                                assert(cur_leng>0);
                                break;
                            }
                        }
                        cur_dist += m_ind_error[ui_tra_idx][MAX_SSRB_SIZE];
                    }
                    else
                    {
                        if (!m_b_major[ui_tra_idx])
                        {
                            unsigned int ui_tra_idx_abv = ui_tra_idx - ui_width;
                            COM_SP_PIX pixel_cur = m_p_CQ_pixbuffer[ui_tra_idx];
                            COM_SP_PIX pixel_abv = m_p_CQ_pixbuffer[ui_tra_idx_abv];
                            int errorlimit = 1;
                            if (abs(pixel_cur.Y - pixel_abv.Y) >= errorlimit || abs(pixel_cur.U - pixel_abv.U) >= errorlimit || abs(pixel_cur.V - pixel_abv.V) >= errorlimit)
                            {
                                cur_leng--;
                                assert(cur_leng>0);
                                break;
                            }
                        }
                        cur_dist += m_ind_error[ui_tra_idx][ui_SRB_idx];
                    }
                }
                else
                {
                    break;
                }
            }
            str_info_merge.is_matched = 0;
            str_info_merge.length = cur_leng;
            str_info_merge.srb_dist = cur_dist;
        }
        ui_idx_start += str_info_merge.length;
        if (str_info_merge.match_type == MATCH_POS_WIDTH && ui_idx_start<ui_total)
        {
            ui_tra_idx = p_trav_scan_order[ui_idx_start];
            pred_index = m_c_index_block[ui_tra_idx - ui_width];
        }
        else
        {
            pred_index = str_info_merge.srb_index;
        }
        vec_dict_info.push_back(str_info_merge); str_info_idx_merge++; dist_new += str_info_merge.srb_dist;
    }

    if (copy_flag)
    {
        x_free(copy_flag);//delete[] copyFlag;
        copy_flag = NULL;
    }
    if (mode_flag)
    {
        x_free(mode_flag);//delete[] modeFlag;
        mode_flag = NULL;
    }

    if (pixel_YUV)
    {
        x_free(pixel_YUV);//delete[] pixelYUV;
        pixel_YUV = NULL;
    }
    return 0;
}
unsigned int string_prediction::calc_dist_norm(COM_SP_PIX org_pixel, COM_SP_PIX rec_pixel, unsigned int& dist_y, unsigned int& dist_uv)
{
    unsigned int shift_y = ((input->sample_bit_depth - 8) << 1);
    unsigned int shift_c = ((input->sample_bit_depth - 8) << 1);
    int  i_temp;
    double ui_sum = 0;
    i_temp = org_pixel.Y - rec_pixel.Y; dist_y = ((i_temp * i_temp) >> shift_y);
    i_temp = org_pixel.U - rec_pixel.U; dist_uv = ((i_temp * i_temp) >> shift_c);
    i_temp = org_pixel.V - rec_pixel.V; dist_uv += ((i_temp * i_temp) >> shift_c);
    ui_sum = dist_y + dist_uv;
    return (unsigned int)ui_sum;
}
int string_prediction::get_sp_search_area_width(int x_start_in_pic, int uiMaxSearchWidthToLeftInCTUs)
{
    const int lcu_width = m_max_cu_width;
    const int max_width = uiMaxSearchWidthToLeftInCTUs * lcu_width;
    int width = 0;
    while (width < max_width)
    {
        if (x_start_in_pic / lcu_width == 0)
        {
            break;
        }
        else
        {
            x_start_in_pic -= lcu_width;
            width += lcu_width;
        }
    }
    return (width < max_width) ? width : max_width;
}

u8 string_prediction::rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion
    , ENC_CTX *ctx
)
{
    {
        double quantiser_scale = quant_scale[p_cur_sp_info->qp - 16];
        int shift_16_bit = 1; 
        int quantiser_right_shift = 15 + shift_16_bit;
        double d_qp = ((double)(1 << quantiser_right_shift)) / quantiser_scale;
        unsigned int srb_qp = (unsigned int)(d_qp / 4.0 + 0.5);
        set_srb_errLimit(srb_qp);
    }
    u8 isFound = check_rdcost_sp_mode(p_cur_sp_info, min_rdcost, distorion
        , ctx
    );
    return isFound;
}
u8 string_prediction::check_rdcost_sp_mode(COM_SP_CODING_UNIT * p_cur_sp_info, double*  min_rdcost, double*  distorion
    , ENC_CTX *ctx
)
{
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int x_cu_start = p_cur_sp_info->cu_pix_x;
    int y_cu_start = p_cur_sp_info->cu_pix_y;
    u8 b_cs2_mode_flag = p_cur_sp_info->m_cs2_mode_flag;
    u8 b_srb_sharing_flag = p_cur_sp_info->m_srb_sharing_flag;
    u8 dict_status = p_cur_sp_info->string_copy_direction;
    p_cur_sp_info->m_evs_present_flag = p_cur_sp_info->m_cs2_mode_flag;
    p_cur_sp_info->unpredict_pix_num = 0;
    unsigned int distortion_y, distortion_uv;
    pel *pa_orig[3] = { NULL,NULL,NULL }, *pa_srb[3] = { NULL,NULL,NULL };
    for (int i = 0; i < 3; i++)
    {
        m_p_pel_org444[i] = NULL;
        m_p_pel_rec444[i] = NULL;
    }
    m_all_comp_rec444 = NULL;
    if (b_cs2_mode_flag)
    {
        int ui_max_pvbuf_size = MAX_SRB_SIZE;
        int ui_max_pred_size = MAX_SRB_PRED_SIZE;
        set_max_pred_size(ui_max_pred_size);
        set_max_pvbuf_size(ui_max_pvbuf_size);
        set_cluster_pred(b_srb_sharing_flag ? 1 : 0);
        int ui_pvbuf_size = 1;

        for (int ch = 0; ch < 3; ch++)
        {
            pa_orig[ch] = (pel *)x_malloc(pel, width*height);
            pa_srb[ch] = b_srb_sharing_flag ? (&g_cluster_pred[ch][0]) : (&g_cluster[ch][0]);
            ui_pvbuf_size = b_srb_sharing_flag ? g_ui_cluster_size_pred : g_ui_cluster_size;
        }

        {
            //copy from source Image into cur CU memory.
            int scale_x = 0;
            int scale_y = 0;
            //change 420 cu to 444
            m_p_pel_org444[1] = (pel*)malloc(width * height * sizeof(pel));
            m_p_pel_org444[2] = (pel*)malloc(width * height * sizeof(pel));
            m_p_pel_org444[0] = m_p_pel_org[0];
            for (int i = 0; i < 3; i++)
            {
                m_p_pel_rec444[i] = (pel*)malloc(width * height * sizeof(pel));
            }
            m_all_comp_rec444 = (u8*)malloc(width * height * sizeof(u8));
            for (int ch = 1; ch < 3; ch++)
            {
                for (int h = 0; h < height / 2; h++)
                {
                    for (int w = 0; w < width / 2; w++)
                    {
                        if (p_cur_sp_info->cu_ext)
                        {
                            m_p_pel_org444[ch][h*width * 2 + w * 2] = 512;
                            m_p_pel_org444[ch][h*width * 2 + w * 2 + 1] = 512;
                            m_p_pel_org444[ch][h*width * 2 + w * 2 + width] = 512;
                            m_p_pel_org444[ch][h*width * 2 + w * 2 + width + 1] = 512;
                        }
                        else
                        {
                            m_p_pel_org444[ch][h*width * 2 + w * 2] = m_p_pel_org[ch][((y_cu_start >> 1) + h)* m_stride_org[ch] + (x_cu_start >> 1) + w];
                            m_p_pel_org444[ch][h*width * 2 + w * 2 + 1] = m_p_pel_org[ch][((y_cu_start >> 1) + h)* m_stride_org[ch] + (x_cu_start >> 1) + w];
                            m_p_pel_org444[ch][h*width * 2 + w * 2 + width] = m_p_pel_org[ch][((y_cu_start >> 1) + h)* m_stride_org[ch] + (x_cu_start >> 1) + w];
                            m_p_pel_org444[ch][h*width * 2 + w * 2 + width + 1] = m_p_pel_org[ch][((y_cu_start >> 1) + h)* m_stride_org[ch] + (x_cu_start >> 1) + w];
                        }
                    }
                }
            }

            for (int ui_y = 0; ui_y < height; ui_y++)
            {
                for (int ui_x = 0; ui_x < width; ui_x++)
                {
                    int cur_pos = ui_y * width + ui_x;
                    int cur_pos_c = (ui_y >> scale_y) * (width >> scale_x) + (ui_x >> scale_x);
                    int org_pos = (y_cu_start + ui_y) * m_stride_org[0] + (ui_x + x_cu_start);
                    int org_pos_c = ((y_cu_start + ui_y) >> scale_y) * (m_stride_org[0] >> scale_x) + ((ui_x + x_cu_start) >> scale_x);

                    pa_orig[0][cur_pos] = m_p_pel_org444[0][org_pos];
                    pa_orig[1][cur_pos_c] = m_p_pel_org444[1][cur_pos_c];
                    pa_orig[2][cur_pos_c] = m_p_pel_org444[2][cur_pos_c];
                    //init m_p_org_pixel_buffer with orig YUV
                    m_p_org_pixel_buffer[cur_pos].Y = pa_orig[0][cur_pos];
                    m_p_org_pixel_buffer[cur_pos_c].U = pa_orig[1][cur_pos_c];
                    m_p_org_pixel_buffer[cur_pos_c].V = pa_orig[2][cur_pos_c];
                    //init the recon YUV with orig 444YUV
                    m_p_pel_rec444[0][cur_pos] = m_p_pel_org444[0][org_pos];
                    m_p_pel_rec444[1][cur_pos_c] = m_p_pel_org444[1][cur_pos_c];
                    m_p_pel_rec444[2][cur_pos_c] = m_p_pel_org444[2][cur_pos_c];
                }
            }
        }

        if (b_cs2_mode_flag && dict_status) //Horizontal,DICT_STATUS_REC_HORIZONTAL
        {
            if (b_srb_sharing_flag)
            {
                derive_srblossy_force_prediction(p_cur_sp_info, pa_srb, pa_orig, width, height, ui_pvbuf_size);
                g_ui_cluster_size_pred = ui_pvbuf_size;

                if (g_ui_cluster_size_pred == g_ui_cluster_size)
                {
                    u8 same_cluster = 1;
                    for (int ui_idx = 0; ui_idx < g_ui_cluster_size_pred; ui_idx++)
                    {
                        if (g_cluster[0][ui_idx] != g_cluster_pred[0][ui_idx] || g_cluster[1][ui_idx] != g_cluster_pred[1][ui_idx] || g_cluster[2][ui_idx] != g_cluster_pred[2][ui_idx])
                        {
                            same_cluster = 0;
                            break;
                        }
                    }
                    if (same_cluster)
                    {
                        for (int ch = 0; ch < 3; ch++)
                        {
                            if (pa_orig[ch])
                            {
                                x_free(pa_orig[ch]);
                            }
                            pa_orig[ch] = NULL;
                        }
                        return 0;
                    }
                }
            }
            else
            {
                derive_srblossy(p_cur_sp_info, pa_srb, pa_orig, width, height, ui_pvbuf_size);
                g_ui_cluster_size = ui_pvbuf_size;
            }
        }
        //
        if ((width == 4) && (height == 4))
        {
            assert(ui_pvbuf_size <= EVS_PV_4);
        }
        else if ((width == 4 && height == 8) || (width == 8 && height == 4))
        {
            assert(ui_pvbuf_size <= EVS_PV_8);
        }
        else if ((width == 4 && height == 16) || (width == 16 && height == 4))
        {
            assert(ui_pvbuf_size <= EVS_PV_MAX);
        }
        if ((width == 8) && (height == 8))
        {
            assert(ui_pvbuf_size <= EVS_PV_MAX);
        }
        else
        {
            assert(ui_pvbuf_size <= MAX_SRB_SIZE);
            
        }

        unsigned int calc_erro_bits = 0;

        reorder_SRB(p_cur_sp_info, pa_srb, ui_pvbuf_size, b_cs2_mode_flag);
        derive_cluster(p_cur_sp_info, pa_srb, ui_pvbuf_size, width, height, x_cu_start, y_cu_start, b_cs2_mode_flag, p_cur_sp_info->lamda, m_ind_error, calc_erro_bits);
    }
    u8 result = 0;    
    // do rdo here
    int x_lcu_start = x_cu_start / m_max_cu_width * m_max_cu_width;
    int y_lcu_start = y_cu_start / m_max_cu_width * m_max_cu_width;
    restore(m_ppp_dict_status[p_cur_sp_info->cu_width_log2 - 2][p_cur_sp_info->cu_height_log2 - 2][HS_CURR_BEST], x_lcu_start, y_lcu_start);
    if (p_cur_sp_info->string_prediction_mode_flag == TRUE) 
    {
        vector<COM_SP_INFO> vec_dict_info;
        set_org_block(width, height, x_cu_start, y_cu_start);
        int cu_size_str_found = FALSE;
        result = encode_cu_size_str(p_cur_sp_info, x_cu_start, y_cu_start, vec_dict_info, min_rdcost, distorion);
        if (result == TRUE)
        {
            cu_size_str_found = TRUE;
        }
        if (cu_size_str_found == FALSE)
        {
            result = encode_general_str(p_cur_sp_info, x_cu_start, y_cu_start, vec_dict_info, min_rdcost, distorion);
        }
        if (result == TRUE) 
        {
            p_cur_sp_info->sub_string_no = (int)vec_dict_info.size();
            assert(p_cur_sp_info->sub_string_no <= p_cur_sp_info->max_str_cnt);
            for (int i = 0; i < p_cur_sp_info->sub_string_no; i++)
            {
                p_cur_sp_info->p_string_copy_info[i] = vec_dict_info.at(i);
            }
            if (p_cur_sp_info->is_sp_pix_completed)
            {
                save(m_ppp_dict_status[p_cur_sp_info->cu_width_log2 - 2][p_cur_sp_info->cu_height_log2 - 2][HS_NEXT_BEST], x_lcu_start, y_lcu_start);
            }
        }
        return result;
    }
    else if (b_cs2_mode_flag)
    {
        vector<COM_SP_EVS_INFO> vec_dict_info;
        vector<COM_SP_PIX> vec_dict_pixel;
        COM_SP_PIX* p_CQbuf = m_p_CQ_pixbuffer;


        result = encode_evs_or_ubvs(p_cur_sp_info, width, height, x_cu_start, y_cu_start, vec_dict_info, vec_dict_pixel, p_CQbuf, min_rdcost,/* distortion,*/ b_srb_sharing_flag
            , ctx
        );

        if (result)
        {
            decode_evs_ubvs(p_cur_sp_info, width, height, x_cu_start, y_cu_start, /*pPixels,*/ dict_status);
            get_cu_dist(p_cur_sp_info, width, height, x_cu_start, y_cu_start, &distortion_y, &distortion_uv);
            *distorion = distortion_y + distortion_uv * p_cur_sp_info->chroma_weight[0];
            for (int ch = 0; ch < 3; ch++)
            {
                int scale_x = (ch == 0 ? 0 : 1);
                int scale_y = (ch == 0 ? 0 : 1);

                int rec_pos = (y_cu_start >> scale_y) * m_stride_rec[ch] + (x_cu_start >> scale_x);
                int rec_stride = 0;
                for (int row = 0; row < (height >> scale_y); row++)
                {
                    for (int col = 0; col < (width >> scale_x); col++)
                    {
                        int curRecPos = rec_pos + col;
                        p_cur_sp_info->rec[ch][rec_stride + col] = m_p_pel_rec[ch][curRecPos];
                    }
                    rec_pos += m_stride_rec[ch];
                    rec_stride += (width >> scale_x);
                }
            }
        }
        //free 444buffer
        for (int i = 1; i < 3; i++)
        {
            if (m_p_pel_org444[i])
            {
                free(m_p_pel_org444[i]);
            }
            m_p_pel_org444[i] = NULL;
        }
        for (int i = 0; i < 3; i++)
        {
            if (m_p_pel_rec444[i])
            {
                free(m_p_pel_rec444[i]);
            }
            m_p_pel_rec444[i] = NULL;
        }
        if (m_all_comp_rec444)
        {
            free(m_all_comp_rec444);
        }
        m_all_comp_rec444 = NULL;
        for (int ch = 0; ch < 3; ch++)
        {
            if (pa_orig[ch])
            {
                x_free(pa_orig[ch]);
            }
            pa_orig[ch] = NULL;
        }

        return result;
    }
    return 0;
}

void string_prediction::add_pixel_to_hash_table(int count, u8 includeHash, int x_start_in_pic, int y_start_in_pic, int cu_width_log2, int cu_height_log2, int iStartIdx, u8 bHorizontal)
{
    int width = 1 << cu_width_log2;
    int height = 1 << cu_height_log2;
    int* p_trav_scan_order = com_tbl_raster2trav[bHorizontal][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];
    if (includeHash)
    {
        for (int pixel_count = 0; pixel_count < count; pixel_count++)
        {
            int current_x;
            int current_y;
            int  uiIdx = iStartIdx + pixel_count;
            int  trav_order_index = p_trav_scan_order[uiIdx];
            current_x = x_start_in_pic + GET_TRAV_X(trav_order_index, width);
            current_y = y_start_in_pic + GET_TRAV_Y(trav_order_index, cu_width_log2);
            int lengthMinus1 = 0;
            int startPositionX = current_x - lengthMinus1;
            int startPositionY = current_y;
            if (startPositionX < 0 || startPositionY < 0 || startPositionX >= m_pic_width || startPositionY >= m_pic_height)
            {
                continue;
            }
            u16 hashValue = (this->*get_sp_hashvalue)(m_p_org_pixel_buffer[trav_order_index]);
            m_dict[current_y][current_x].x = m_hash_table[hashValue].x;
            m_dict[current_y][current_x].y = m_hash_table[hashValue].y;
            m_hash_table[hashValue].x = startPositionX;
            m_hash_table[hashValue].y = startPositionY;
        }
    }
}

int string_prediction::get_tb_bits(int ui_symbol, int uiMaxSymbol)
{
    int bins = 0;
    int uiThresh;
    if (uiMaxSymbol > 256)
    {
        int uiThreshVal = 1 << 8;
        uiThresh = 8;
        while (uiThreshVal <= uiMaxSymbol)
        {
            uiThresh++;
            uiThreshVal <<= 1;
        }
        uiThresh--;
    }
    else
    {
        uiThresh = g_sp_tb_tbl[uiMaxSymbol];
    }
    int uiVal = 1 << uiThresh;
    assert(uiVal <= uiMaxSymbol);
    assert((uiVal << 1) > uiMaxSymbol);
    assert(ui_symbol < (int)uiMaxSymbol);
    int b = uiMaxSymbol - uiVal;
    assert(b < uiVal);
    if (ui_symbol < (int)(uiVal - b))
    {
        bins = uiThresh;
    }
    else
    {
        bins = uiThresh + 1;
    }
    return bins;
}

int string_prediction::get_len_bits_cnt(int len, int totalLeftPixel)
{
    int bins = 0;
    assert(len <= totalLeftPixel);
    if (len < 4) // 2 bits
    {
        if (totalLeftPixel >= 4)
        {
            bins++;
        }
        if (totalLeftPixel < 3)
            bins += get_tb_bits(len, totalLeftPixel + 1);
        else
            bins += 2;
    }
    else if (len - 4 < 16) // 4 bits
    {
        bins += 1;
        if (totalLeftPixel - 4 >= 16)
        {
            bins += 1;
        }
        if (totalLeftPixel < 19)
            bins += get_tb_bits(len - 4, totalLeftPixel - 3);
        else
            bins += 4;
    }
    else if (len - 4 - 16 < 256) // 8 bits
    {
        bins += 1;
        bins += 1;
        if (totalLeftPixel - 4 - 16 >= 256)
        {
            bins += 1;
        }
        if (totalLeftPixel < 275)
            bins += get_tb_bits(len - 20, totalLeftPixel - 19);
        else
            bins += 8;
    }
    else if (len - 4 - 16 - 256 < 65536) // 12 bits, maximum number 4096
    {
        bins += 3;
        if (totalLeftPixel < 4096)
            bins += get_tb_bits(len - 276, totalLeftPixel - 275);
        else
            bins += 12;
    }
    return bins;
}

int string_prediction::get_offset_bits(int ui_symbol, int uiCount)
{
    int bins = 0;
    int numBins = 0;
    while (ui_symbol >= (int)(1 << uiCount))
    {
        bins = 2 * bins + 1;
        numBins++;
        ui_symbol -= 1 << uiCount;
        uiCount++;
    }
    bins = 2 * bins + 0;
    numBins++;
    bins = (bins << uiCount) | ui_symbol;
    numBins += uiCount;
    return numBins;
}

int string_prediction::get_exp_golomb_part_bits(u32 ui_symbol, int uiCount)
{
    int bit_cnt = 0;
    while (1)
    {
        if (ui_symbol >= (unsigned int)(1 << uiCount))
        {
            bit_cnt++;
            ui_symbol = ui_symbol - (1 << uiCount);
            uiCount++;
        }
        else
        {
            bit_cnt++;
            while (uiCount--) //next binary part
            {
                bit_cnt++;
            }
            break;
        }
    }
    return bit_cnt;
}

int string_prediction::get_svd_component_bits(u32 svd)
{
    int bit_cnt = 0;
    if (svd < 17)
    {
        if (svd == 0)
        {
            bit_cnt++;
        }
        else if (svd == 1)
        {
            bit_cnt += 2;
        }
        else if (svd == 2)
        {
            bit_cnt += 3;
        }
        else if (svd == 3)
        {
            bit_cnt += 4;
        }
        else if (svd == 4)
        {
            bit_cnt += 5;
        }
        else if (svd < 9)
        {
            bit_cnt += 6;
            //flc
            int len = 2;
            bit_cnt += len;
        }
        else if (svd < 17)
        {
            bit_cnt += 7;
            //flc
            int len = 3;
            bit_cnt += len;
        }
    }
    else
    {
        bit_cnt += 7;

        int offset;

        svd -= 17;
        offset = svd & 1;

        bit_cnt++;
        svd = (svd - offset) >> 1;

        // exp_golomb part
        bit_cnt += get_exp_golomb_part_bits(svd, BVD_EXG_ORDER);
    }
    return bit_cnt;
}

int string_prediction::check_str_len_bit(int len, int w, int h, int total_left_pixel_minus1, u8 dir)
{
    int cur_bitcount;
    cur_bitcount = 0;
    int next_remianing_pixel_in_cu = total_left_pixel_minus1 + 1 - len;
    if (total_left_pixel_minus1 == 0)
        return cur_bitcount;
    if (next_remianing_pixel_in_cu == 0)
    {
        cur_bitcount++;
    }
    else
    {
        next_remianing_pixel_in_cu = next_remianing_pixel_in_cu / 4;
        assert(next_remianing_pixel_in_cu > 0);
        cur_bitcount++;
        cur_bitcount += get_len_bits_cnt(next_remianing_pixel_in_cu - 1, (total_left_pixel_minus1 + 1) / 4 - 1);
    }
    return cur_bitcount;
}
double string_prediction::get_hor_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
    , int match_length, int offset_sign, int left_pix_in_cu_minus1, int *bit_cnt
    , COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag)
{
    u32 sad = distortion;
    int num_of_bits = 0;
    int mappedOffset = offset_in_cu;
    // a bit for sign
    if (offset_x == 0 && offset_y == -1)
    {
        num_of_bits++;
    }
    else
    {
        num_of_bits++;
        int index = check_sp_offset(offset_x, offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
        if (index != p_cur_sp_info->n_cand_num)
        {
            num_of_bits++;
            num_of_bits += get_tb_bits(p_cur_sp_info->n_cand_num - 1 - index, p_cur_sp_info->n_cand_num);
            *n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
            *n_recent_flag = 1;
        }
        else
        {
            *n_recent_flag = 0;
            num_of_bits++;
            num_of_bits += get_svd_component_bits(COM_ABS(offset_y));
            if (offset_y != 0)
            {
                num_of_bits++;
            }
            if (offset_y != 0)
            {
                if (offset_y > 0)
                {
                    num_of_bits += get_offset_bits(abs(offset_x) - 1, SP_EXG_ORDER);
                }
                else
                {
                    num_of_bits++;
                    if (offset_x != 0)
                    {
                        num_of_bits++;
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, SP_EXG_ORDER);
                    }
                }
            }
            else
            {
                num_of_bits++;
                num_of_bits += get_offset_bits(abs(offset_x) - 1, SP_EXG_ORDER);
            }
        }
    }
    if (match_length != 0)
    {
        num_of_bits += check_str_len_bit(match_length, 1 << p_cur_sp_info->cu_width_log2, 1 << p_cur_sp_info->cu_height_log2, left_pix_in_cu_minus1, TRUE);
    }
    *bit_cnt = num_of_bits;
    return sad + num_of_bits * p_cur_sp_info->lamda;
}

double string_prediction::get_unmatched_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
    , int match_length, int offset_sign, int left_pix_in_cu_minus1, int *bit_cnt
    , COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag, int* match_dict, int* p_trav_scan_order, int cur_pixel)
{
    u32 sad = distortion;
    int num_of_bits = 0;
    int mappedOffset = offset_in_cu;

    int match_count = 0;
    for (int i = 0;i < 4;i++)
    {
        num_of_bits++;
        match_count += match_dict[i] ? 1 : 0;
        if (!match_dict[i])
        {

            int  trav_order_index = p_trav_scan_order[cur_pixel + i];
            int  trav_x = GET_TRAV_X(trav_order_index, 1 << p_cur_sp_info->cu_width_log2);
            int  trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
            if (p_cur_sp_info->tree_status != TREE_C)
            {
                num_of_bits += 10;
            }
            if (p_cur_sp_info->tree_status != TREE_L)
            {
                if (!(trav_x & 0x1 || trav_y & 0x1))
                {
                    num_of_bits += 20;
                }
            }
        }
    }

    if (match_count > 0)
    {
        assert(!(offset_x == 0 && offset_y == 0));
        if (offset_x == 0 && offset_y == -1)
        {
            num_of_bits++;
        }
        else
        {
            num_of_bits++;
            int index = check_sp_offset(offset_x, offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
            if (index != p_cur_sp_info->n_cand_num)
            {
                num_of_bits++;
                num_of_bits += get_tb_bits(p_cur_sp_info->n_cand_num - 1 - index, p_cur_sp_info->n_cand_num);
                *n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
                *n_recent_flag = 1;
            }
            else
            {
                *n_recent_flag = 0;
                num_of_bits++;
                num_of_bits += get_svd_component_bits(COM_ABS(offset_y));
                if (offset_y != 0)
                {
                    num_of_bits++;
                }
                if (offset_y != 0)
                {
                    if (offset_y > 0)
                    {
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, SP_EXG_ORDER);
                    }
                    else
                    {
                        num_of_bits++;
                        if (offset_x != 0)
                        {
                            num_of_bits++;
                            num_of_bits += get_offset_bits(abs(offset_x) - 1, SP_EXG_ORDER);
                        }
                    }
                }
                else
                {
                    num_of_bits++;
                    num_of_bits += get_offset_bits(abs(offset_x) - 1, SP_EXG_ORDER);
                }
            }
        }
    }
    *bit_cnt = num_of_bits;
    return sad + num_of_bits * p_cur_sp_info->lamda;
}

double string_prediction::get_ver_sp_cost(int offset_x, int offset_y, u32 distortion, int offset_in_cu, u8 is_boundary_cu
    , int match_length, int offset_sign, int left_pix_in_cu_minus1, int *bit_cnt
    , COM_SP_CODING_UNIT* p_cur_sp_info, s8 *n_recent_idx, u8 *n_recent_flag)
{
    u32 sad = distortion;
    int num_of_bits = 0;
    // a bit for sign
    if (offset_x == -1 && offset_y == 0)
    {
        num_of_bits++;
    }
    else
    {
        int index = check_sp_offset(offset_x, offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
        if (index != p_cur_sp_info->n_cand_num)
        {
            num_of_bits++;
            num_of_bits += get_tb_bits(p_cur_sp_info->n_cand_num - 1 - index, p_cur_sp_info->n_cand_num);
            *n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
            *n_recent_flag = 1;
        }
        else
        {
            *n_recent_flag = 0;
            if (offset_y == 0 && offset_x < -1)
            {
                offset_x--;
            }
            num_of_bits++;
            //
            num_of_bits++;
            if (offset_y != 0)
            {
                num_of_bits++;
                num_of_bits += get_offset_bits(abs(offset_y) - 1, 2);
                if (offset_y > 0)
                {
                    if (offset_sign == 1)
                    {
                        num_of_bits += get_offset_bits(abs(offset_x) + 1, 2);
                    }
                    else
                    {
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
                    }
                }
                else
                {
                    num_of_bits++;
                    if (offset_x != 0)
                    {
                        num_of_bits++;
                        num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
                    }
                }
            }
            else
            {
                num_of_bits += get_offset_bits(abs(offset_x) - 1, 2);
            }
        }
    }
    if (match_length != 0)
    {
        num_of_bits += check_str_len_bit(match_length, 1 << p_cur_sp_info->cu_width_log2, 1 << p_cur_sp_info->cu_height_log2, left_pix_in_cu_minus1, FALSE);
    }
    *bit_cnt = num_of_bits;
    return sad + num_of_bits * p_cur_sp_info->lamda;
}

void string_prediction::get_string_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int match_length,int processed_count,u8 scale_x, u8 scale_y,int uiShiftY, int uiShiftC,u32 *str_dist_y, u32 *str_dist_uv)
{
    u8 tree_status = p_cur_sp_info->tree_status;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int* p_trav_scan_order = com_tbl_raster2trav[p_cur_sp_info->string_copy_direction][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    int trav_order_index = p_trav_scan_order[processed_count];
    int trav_x, trav_y;
    int cur_pos, cur_pos_c, ref_pos, ref_pos_c;
    int mvX = offset_x;
    int mvY = offset_y;
    u32 ui_sum;
    s32 i_temp;
    for (int pixel_count = 0; pixel_count < match_length; pixel_count++)
    {
        trav_order_index = p_trav_scan_order[processed_count + pixel_count];
        trav_x = GET_TRAV_X(trav_order_index, width);
        trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
        int cur_x_in_pic = p_cur_sp_info->cu_pix_x + trav_x;
        int cur_y_in_pic = p_cur_sp_info->cu_pix_y + trav_y;
        int ref_x_in_pic = p_cur_sp_info->cu_pix_x + trav_x + mvX;
        int ref_y_in_pic = p_cur_sp_info->cu_pix_y + trav_y + mvY;
        ui_sum = 0;
        if (p_cur_sp_info->tree_status != TREE_C)
        {
            cur_pos = cur_y_in_pic * m_stride_rec[0] + cur_x_in_pic;
            ref_pos = ref_y_in_pic * m_stride_rec[0] + ref_x_in_pic;
            i_temp = m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_rec[0][ref_pos]; 
            ui_sum += ((i_temp * i_temp) >> uiShiftY);
            *str_dist_y += ui_sum;            
            m_p_pel_rec[0][cur_pos] = m_p_pel_rec[0][ref_pos];
        }
        ui_sum = 0;
        if (p_cur_sp_info->tree_status != TREE_L)
        {
            if (!((cur_x_in_pic & scale_x) || (cur_y_in_pic & scale_y)))
            {
                cur_pos_c = (cur_y_in_pic >> scale_y) * m_stride_rec[1] + (cur_x_in_pic >> scale_x);
                ref_pos_c = (ref_y_in_pic >> scale_y) * m_stride_rec[1] + (ref_x_in_pic >> scale_x);
                i_temp = m_p_org_pixel_buffer[trav_order_index].U - m_p_pel_rec[1][ref_pos_c];
                ui_sum += ((i_temp * i_temp) >> uiShiftC);
                i_temp = m_p_org_pixel_buffer[trav_order_index].V - m_p_pel_rec[2][ref_pos_c];
                ui_sum += ((i_temp * i_temp) >> uiShiftC);
                *str_dist_uv += ui_sum;
                m_p_pel_rec[1][cur_pos_c] = m_p_pel_rec[1][ref_pos_c];
                m_p_pel_rec[2][cur_pos_c] = m_p_pel_rec[2][ref_pos_c];
            }
        }
    }
}

void string_prediction::get_unmatched_string_dist(COM_SP_CODING_UNIT* p_cur_sp_info, int offset_x, int offset_y, int match_length, int processed_count, u8 scale_x, u8 scale_y, int uiShiftY, int uiShiftC, u32 *str_dist_y, u32 *str_dist_uv, int* match_dict)
{
    u8 tree_status = p_cur_sp_info->tree_status;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int* p_trav_scan_order = com_tbl_raster2trav[p_cur_sp_info->string_copy_direction][p_cur_sp_info->cu_width_log2 - MIN_CU_LOG2][p_cur_sp_info->cu_height_log2 - MIN_CU_LOG2];
    int trav_order_index = p_trav_scan_order[processed_count];
    int trav_x, trav_y;
    int cur_pos, cur_pos_c, ref_pos, ref_pos_c;
    int mvX = offset_x;
    int mvY = offset_y;
    u32 ui_sum;
    s32 i_temp;
    assert(match_length == 4);
    for (int pixel_count = 0; pixel_count < match_length; pixel_count++)
    {
        if (!match_dict[pixel_count])
        {
            continue;
        }

        trav_order_index = p_trav_scan_order[processed_count + pixel_count];
        trav_x = GET_TRAV_X(trav_order_index, width);
        trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
        int cur_x_in_pic = p_cur_sp_info->cu_pix_x + trav_x;
        int cur_y_in_pic = p_cur_sp_info->cu_pix_y + trav_y;
        int ref_x_in_pic = p_cur_sp_info->cu_pix_x + trav_x + mvX;
        int ref_y_in_pic = p_cur_sp_info->cu_pix_y + trav_y + mvY;
        ui_sum = 0;
        if (p_cur_sp_info->tree_status != TREE_C)
        {
            cur_pos = cur_y_in_pic * m_stride_rec[0] + cur_x_in_pic;
            ref_pos = ref_y_in_pic * m_stride_rec[0] + ref_x_in_pic;
            i_temp = m_p_org_pixel_buffer[trav_order_index].Y - m_p_pel_rec[0][ref_pos];
            ui_sum += ((i_temp * i_temp) >> uiShiftY);
            *str_dist_y += ui_sum;
            m_p_pel_rec[0][cur_pos] = m_p_pel_rec[0][ref_pos];
        }
        ui_sum = 0;
        if (p_cur_sp_info->tree_status != TREE_L)
        {
            if (!((cur_x_in_pic & scale_x) || (cur_y_in_pic & scale_y)))
            {
                cur_pos_c = (cur_y_in_pic >> scale_y) * m_stride_rec[1] + (cur_x_in_pic >> scale_x);
                ref_pos_c = (ref_y_in_pic >> scale_y) * m_stride_rec[1] + (ref_x_in_pic >> scale_x);
                i_temp = m_p_org_pixel_buffer[trav_order_index].U - m_p_pel_rec[1][ref_pos_c];
                ui_sum += ((i_temp * i_temp) >> uiShiftC);
                i_temp = m_p_org_pixel_buffer[trav_order_index].V - m_p_pel_rec[2][ref_pos_c];
                ui_sum += ((i_temp * i_temp) >> uiShiftC);
                *str_dist_uv += ui_sum;
                m_p_pel_rec[1][cur_pos_c] = m_p_pel_rec[1][ref_pos_c];
                m_p_pel_rec[2][cur_pos_c] = m_p_pel_rec[2][ref_pos_c];
            }
        }
    }
}

void string_prediction::set_org_block(int uiWidth, int uiHeight, int x_start_in_pic, int y_start_in_pic)
{
    COM_SP_PIX cur_pixel;
    int ui_pos;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    //gen major flag;  
    ui_pos = 0;
    for (int uiY = 0; uiY < uiHeight; uiY++)
    {
        for (int uiX = 0; uiX < uiWidth; uiX++)
        {
            int current_x = x_start_in_pic + uiX;
            int current_y = y_start_in_pic + uiY;
            int cur_pos = current_y * m_stride_org[0] + current_x;
            int cur_pos_c = (current_y >> scale_y) * m_stride_org[1] + (current_x >> scale_x);
            cur_pixel.Y = m_p_pel_org[0][cur_pos];
            cur_pixel.U = m_p_pel_org[1][cur_pos_c];
            cur_pixel.V = m_p_pel_org[2][cur_pos_c];
            //save org pixel
            m_p_org_pixel_buffer[ui_pos].Y = m_p_pel_org[0][cur_pos];
            m_p_org_pixel_buffer[ui_pos].U = m_p_pel_org[1][cur_pos_c];
            m_p_org_pixel_buffer[ui_pos].V = m_p_pel_org[2][cur_pos_c];
            ui_pos++;
        }
    }
}

#if SCC_CROSSINFO_ULTILIZE
void string_prediction::init_sps_cands(COM_SP_CODING_UNIT* p_cur_sp_info, COM_MOTION *p_cand, int p_cand_num, COM_MOTION *b_cand, int b_cand_num, vector<COM_MOTION> &sps_cands, u8 dir, COM_MOTION *n_cand, s8 n_cand_num)
#else
void string_prediction::init_sps_cands(COM_MOTION *p_cand, int p_cand_num, COM_MOTION *b_cand, int b_cand_num, vector<COM_MOTION> &sps_cands, u8 dir, COM_MOTION *n_cand, s8 n_cand_num)
#endif
{
    COM_MOTION tmp;
    int i = 0, j = 0;
    if (dir == FALSE) //ver
    {
        tmp.mv[0][MV_X] = -1;
        tmp.mv[0][MV_Y] = 0;
        sps_cands.push_back(tmp);
        tmp.mv[0][MV_X] = 0;
        tmp.mv[0][MV_Y] = -1;
        sps_cands.push_back(tmp);
    }
    else //hor
    {
        tmp.mv[0][MV_X] = 0;
        tmp.mv[0][MV_Y] = -1;
        sps_cands.push_back(tmp);
        tmp.mv[0][MV_X] = -1;
        tmp.mv[0][MV_Y] = 0;
        sps_cands.push_back(tmp);
    }
#if SCC_CROSSINFO_ULTILIZE
    for (i = 0; i < 1; i++)
    {
        COM_MOTION temp;
        temp.mv[0][0] = (s16)p_cur_sp_info->ibc_mv[0]; temp.mv[0][1] = (s16)p_cur_sp_info->ibc_mv[1];
        temp.ref_idx[0] = -1; temp.ref_idx[1] = -1;

        for (j = 0; j < sps_cands.size(); j++)
        {
            if (SAME_MV(sps_cands[j].mv[0], temp.mv[0]))
            {
                break;
            }
        }
        if (j == sps_cands.size())
        {
            sps_cands.push_back(temp);
        }
    }
#endif
    for (i = 0; i < p_cand_num; i++)
    {
        for (j = 0; j < sps_cands.size(); j++)
        {
            if (SAME_MV(sps_cands[j].mv[0], p_cand[i].mv[0]))
            {
                break;
            }
        }
        if (j == sps_cands.size())
        {
            sps_cands.push_back(p_cand[i]);
        }
    }
    for (i = 0; i < b_cand_num; i++)
    {
        for (j = 0; j < sps_cands.size(); j++)
        {
            if (SAME_MV(sps_cands[j].mv[0], b_cand[i].mv[0]))
            {
                break;
            }
        }
        if (j == sps_cands.size())
        {
            sps_cands.push_back(b_cand[i]);
        }
    }
}

void string_prediction::refine_sps_cands(vector<COM_MOTION>& org_sps_cands, vector<COM_MOTION>& refined_sps_cands, int current_x, int current_y)
{
    refined_sps_cands.clear();
    s16 mv_x = 0, mv_y = 0;
    for (int i = 0; i < org_sps_cands.size(); i++)
    {
        mv_x = org_sps_cands.at(i).mv[0][MV_X];
        mv_y = org_sps_cands.at(i).mv[0][MV_Y];
        if (current_x + mv_x >= 0 && current_y + mv_y >= 0)
        {
            refined_sps_cands.push_back(org_sps_cands.at(i));
        }
    }
}

u8 string_prediction::encode_general_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* curBestRDCost, double* distortion)
{
#if RBU_LIMIT_GS
    int stripe_num = 0;
    int out_stripe_num = 0;
#endif
    if (p_cur_sp_info->cu_width_log2 == 5 && p_cur_sp_info->cu_height_log2 == 5)
    {
        p_cur_sp_info->is_sp_skip_non_scc = 0;
    }
    if (p_cur_sp_info->is_sp_skip_non_scc)
    {
        return FALSE;
    }
    int replicate_num = 0;
    int same_num = 0;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int count = width * height;
    int processed_count = 0;
    const int ctu_log2Size = p_cur_sp_info->ctu_log2size;
    int numLeftCTUs = (1 << ((7 - ctu_log2Size) << 1)) - ((ctu_log2Size < 7) ? 1 : 0);
    int left_search_range = get_sp_search_area_width(x_start_in_pic, numLeftCTUs);
    const int  lcu_width = m_max_cu_width;
    const int  lcu_height = m_max_cu_height;
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1;
    int bit_depth = input->sample_bit_depth;
    u8 is_hor_scan = p_cur_sp_info->string_copy_direction; //true=horizontal; false=vertical
    int cu_height_log2 = p_cur_sp_info->cu_height_log2;
    int cu_width_log2 = p_cur_sp_info->cu_width_log2;
    int  trav_x;
    int  trav_y;
    int  error_limit = 1 + (sp_max_error_quant[(int(p_cur_sp_info->qp) >> 1) + 10] << 2);
    double dict_rdcost = 0;
    u32   dict_dist_y = 0, dict_dist_uv = 0;
    double tmp_dict_rdcost = 0;
    u32 tmp_dict_dist_y = 0, tmp_dict_dist_uv = 0;
    int* p_trav_scan_order = com_tbl_raster2trav[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];  // processed_count to TravOrder
    int* p_raster_scan_order = com_tbl_trav2raster[is_hor_scan][cu_width_log2 - MIN_CU_LOG2][cu_height_log2 - MIN_CU_LOG2];  // TravOrder to processed_count
    int trav_order_index;
    int ui_shift_y = (bit_depth == 8) ? 0 : (bit_depth - 8) << 1;
    int ui_shift_c = (bit_depth == 8) ? 0 : (bit_depth - 8) << 1;
    int additional_str_num = 0;
    vector<COM_MOTION> sps_cand;
#if SCC_CROSSINFO_ULTILIZE
    init_sps_cands(p_cur_sp_info, p_cur_sp_info->p_cand, p_cur_sp_info->p_cand_num, p_cur_sp_info->b_cand, p_cur_sp_info->b_cand_num, sps_cand, p_cur_sp_info->string_copy_direction, p_cur_sp_info->n_cand, p_cur_sp_info->n_cand_num);
#else
    init_sps_cands(p_cur_sp_info->p_cand, p_cur_sp_info->p_cand_num, p_cur_sp_info->b_cand, p_cur_sp_info->b_cand_num, sps_cand, p_cur_sp_info->string_copy_direction, p_cur_sp_info->n_cand, p_cur_sp_info->n_cand_num);
#endif
    vector<COM_MOTION> refined_sps_cand, *p_sps_cand;
    p_sps_cand = &sps_cand;
#if ERR_BUGFIX
#if REF_RANGE_FIX
    left_search_range += (GS_REF_RANGE_XqLCU - 1) * (1 << SRAM_SIZE_128_LOG2);
    if (left_search_range >= (x_start_in_pic >> SRAM_SIZE_128_LOG2 << SRAM_SIZE_128_LOG2))
    {
        left_search_range = x_start_in_pic >> SRAM_SIZE_128_LOG2 << SRAM_SIZE_128_LOG2;
    }
#else 
    left_search_range += (GS_REF_RANGE_XqLCU - 1) * 128;
    if (left_search_range >= (x_start_in_pic >> 7 << 7))
    {
        left_search_range = x_start_in_pic >> 7 << 7;
    }
#endif 
#endif

    while (processed_count < count)
    {
        u8 string_found = FALSE;
        const int max_test_cases = MAX_SM_TEST_CASE;
        int attemp = 0;
        int current_x;
        int current_y;
        trav_order_index = p_trav_scan_order[processed_count];
        trav_x = GET_TRAV_X(trav_order_index, width);
        trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
        current_x = x_start_in_pic + trav_x;
        current_y = y_start_in_pic + trav_y;
        int cur_offset = current_y * m_stride_org[0] + current_x;
        int max_match_length = 0;
        s8  best_n_recent_idx = SP_RECENT_CANDS;
        u8  best_n_recent_flag = 0;
        int best_match_start_x = -1;
        int best_match_start_y = -1;
        int best_match_type = 0;
        int best_match_dict[4] = { 0 };
        u32 str_dist_y = 0, str_dist_uv = 0;
        int str_bits = 0; 
        int best_str_bits = 0;
        u32 best_str_dist_y = 0;
        u32 best_str_dist_uv = 0;
        double cur_rdcost = 0;
        double min_rdcost = SP_MAX_COST;
#if RBU_LIMIT_GS||ExtStrGS
        int is_out_of_raw_range_flag = 0;
#endif
        {
            u16 hash_value = (this->*get_sp_hashvalue)(m_p_org_pixel_buffer[trav_order_index]);
            int curr_match_start_x = m_hash_table[hash_value].x;
            int curr_match_start_y = m_hash_table[hash_value].y;
            int after_sps_start_x = curr_match_start_x;
            int after_sps_start_y = curr_match_start_y;
            int sps_attemp = 0;
            refine_sps_cands(sps_cand, refined_sps_cand, current_x, current_y);
            p_sps_cand = &refined_sps_cand;
            if ((*p_sps_cand).size() > 0)
            {
                curr_match_start_x = current_x + (*p_sps_cand).at(sps_attemp).mv[0][MV_X];
                curr_match_start_y = current_y + (*p_sps_cand).at(sps_attemp).mv[0][MV_Y];
            }
            for (; attemp < max_test_cases && curr_match_start_x >= 0 && curr_match_start_y >= 0; attemp++)
            {
                int curr_match_length = 0;
                int next_match_start_x;
                int next_match_start_y;
                if (sps_attemp < (*p_sps_cand).size())
                {
                    if (sps_attemp + 1 < (*p_sps_cand).size())
                    {
                        next_match_start_x = current_x + (*p_sps_cand).at(sps_attemp + 1).mv[0][MV_X];
                        next_match_start_y = current_y + (*p_sps_cand).at(sps_attemp + 1).mv[0][MV_Y];
                    }
                    else
                    {
                        assert(sps_attemp + 1 == (*p_sps_cand).size());
                        next_match_start_x = after_sps_start_x;
                        next_match_start_y = after_sps_start_y;
                    }
                    sps_attemp++;
                    
                    if (curr_match_start_y < 0 || curr_match_start_y >= m_pic_height || curr_match_start_x < 0 || curr_match_start_x >= m_pic_width)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                }
                else
                {
                    assert(!(curr_match_start_y < 0 || curr_match_start_y >= m_pic_height || curr_match_start_x < 0 || curr_match_start_x >= m_pic_width));
                    next_match_start_x = m_dict[curr_match_start_y][curr_match_start_x].x;
                    next_match_start_y = m_dict[curr_match_start_y][curr_match_start_x].y;
                }

                s8 curr_n_recent_idx = SP_RECENT_CANDS;
                u8 curr_n_recent_flag = 0;
                int ref_offset = 0;
                int ref_offset_c = 0;
                int mv_x = curr_match_start_x - current_x;
                int mv_y = curr_match_start_y - current_y;
                if (best_match_start_x == curr_match_start_x && best_match_start_y == curr_match_start_y)
                {
                    curr_match_start_x = next_match_start_x;
                    curr_match_start_y = next_match_start_y;
                    continue;
                }
                if ((mv_y > 0 && mv_x > 0) || (mv_x == 0 && mv_y == 0)) 
                {
                    curr_match_start_x = next_match_start_x;
                    curr_match_start_y = next_match_start_y;
                    continue;
                }
                int i_ref_x = (current_x & (lcu_width - 1)) + mv_x;
                int i_ref_y = (current_y & (lcu_height - 1)) + mv_y;
                //Test if the reference pixel is within Search Range

#if IBSC_RIGHT_BOUND && GS_REF_RANGE_XLCUR!=0
#if REF_RANGE_FIX

                int ctu_ytop = current_y >> ctu_log2Size << ctu_log2Size;
                int	top_search_range = (GS_REF_RANGE_XLCUR - 1) * (1 << ctu_log2Size);
                int ctu_xrightbound = 0;
                if (ctu_ytop < top_search_range)
                {
                    top_search_range = ctu_ytop;
                }
                if ((curr_match_start_y >> ctu_log2Size) == (current_y >> ctu_log2Size))
                {
                    ctu_xrightbound = lcu_width;
                }
                else
                {
                    ctu_xrightbound = m_pic_width;
                }
#else
                int ctu_ytop = current_y >> 7 << 7;
                int	top_search_range = (GS_REF_RANGE_XLCUR - 1) * 128;
                int ctu_xrightbound = 0;
                if (ctu_ytop < top_search_range)
                {
                    top_search_range = ctu_ytop;
            }
                if ((curr_match_start_y >> 7) == (current_y >> 7))
                {
                    ctu_xrightbound = lcu_width;
                }
                else
                {
                    ctu_xrightbound = m_pic_width;
                }
#endif
#endif
#if GS_REF_RANGE_XLCUR==0
                if (i_ref_x < -left_search_range || i_ref_x >= (int)lcu_width || i_ref_y >= (int)lcu_height || i_ref_y < 0)
                {
                    //out of the valid research range
                    break;
                }
#else
                if (i_ref_x < -left_search_range || i_ref_x >= ctu_xrightbound || i_ref_y >= (int)lcu_height || i_ref_y < -top_search_range)
                {
                    //out of the valid research range
                    break;
                }
#endif
                ref_offset = curr_match_start_y * m_stride_org[0] + curr_match_start_x;
                ref_offset_c = (curr_match_start_y >> scale_y) * m_stride_org[1] + (curr_match_start_x >> scale_x);
                if (abs(m_p_pel_org[0][cur_offset] - m_p_pel_org[0][ref_offset]) >= error_limit)
                {
                    curr_match_start_x = next_match_start_x;
                    curr_match_start_y = next_match_start_y;
                    continue;
                }
                int max_len_minus1_trav_idx = 0;
                int cur_minus1_x, cur_minus1_y;
                int ref_x_in_pic_n;
                int ref_y_in_pic_n;
                if (max_match_length > 2)
                {
                    max_len_minus1_trav_idx = p_trav_scan_order[processed_count + max_match_length - 2]; 
                    cur_minus1_x = GET_TRAV_X(max_len_minus1_trav_idx, width);
                    cur_minus1_y = GET_TRAV_Y(max_len_minus1_trav_idx, p_cur_sp_info->cu_width_log2);
                    ref_x_in_pic_n = x_start_in_pic + cur_minus1_x + mv_x;
                    ref_y_in_pic_n = y_start_in_pic + cur_minus1_y + mv_y;
                    if (ref_y_in_pic_n < 0 || ref_y_in_pic_n >= m_pic_height || ref_x_in_pic_n < 0 || ref_x_in_pic_n >= m_pic_width)
                        break;
                    int cur_n_offset = (y_start_in_pic + cur_minus1_y) * m_stride_org[0] + x_start_in_pic + cur_minus1_x;
                    int ref_n_offset = ref_y_in_pic_n * m_stride_org[0] + ref_x_in_pic_n;
                    if (abs(m_p_pel_org[0][cur_n_offset] - m_p_pel_org[0][ref_n_offset]) >= error_limit)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                    int i_ref_x_n = ((x_start_in_pic + cur_minus1_x) & (lcu_width - 1)) + mv_x;
                    int i_ref_y_n = ((y_start_in_pic + cur_minus1_y) & (lcu_height - 1)) + mv_y;
                    //Test if the reference pixel is within Search Range
#if GS_REF_RANGE_XLCUR == 0
                    if (i_ref_x_n < -left_search_range || ref_x_in_pic_n >= m_pic_width || i_ref_x_n >= (int)lcu_width || ref_y_in_pic_n >= m_pic_height || i_ref_y_n >= (int)lcu_height || i_ref_y_n < 0)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
#else
                    if (i_ref_x_n < -left_search_range || ref_x_in_pic_n >= m_pic_width || i_ref_x_n >= ctu_xrightbound || ref_y_in_pic_n >= m_pic_height || i_ref_y_n >= (int)lcu_height || i_ref_y_n < -top_search_range)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
#endif
#if GS_REF_RANGE_XqLCU == 0||GS_REF_RANGE_XqLCU == 1
                    if (ctu_log2Size == 7)
                    {
                        assert(i_ref_x >= -128 && i_ref_x < 128 && i_ref_y >= 0 && i_ref_y < 128);
                    }
#else
#if GS_REF_RANGE_XLCUR == 0
                    if (ctu_log2Size == 7)
                    {
                        assert(i_ref_x >= -128 && i_ref_x < 128 && i_ref_y >= 0 && i_ref_y < 128);
                    }
#else
                    int top_ybound = (GS_REF_RANGE_XLCUR - 1) * lcu_height;
                    int left_xbound = GS_REF_RANGE_XqLCU * lcu_width;
                    if (ctu_log2Size == 7)
                    {
                        assert(i_ref_x >= -left_xbound && i_ref_x < ctu_xrightbound&& i_ref_y >= -top_ybound && i_ref_y < 128);
                    }
#endif 
#endif
                    else if (ctu_log2Size == 6)
                    {
                        assert(i_ref_x >= -192 && i_ref_x < 64 && i_ref_y >= 0 && i_ref_y < 64);
                    }
                    else if (ctu_log2Size == 5)
                    {
                        assert(i_ref_x >= -480 && i_ref_x < 32 && i_ref_y >= 0 && i_ref_y < 32);
                    }
                    if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, ref_x_in_pic_n, ref_y_in_pic_n, processed_count + max_match_length - 2, curr_match_start_x, curr_match_start_y)) 
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                }
                int match_dict[4] = { 0,0,0,0 };
                int is_bv_valid = 1;
                int match_type = 1;
#if RBU_LIMIT_GS||ExtStrGS
                is_out_of_raw_range_flag = 0;
#endif
                //HASH search
                {
                    int pc = processed_count;
                    int c_match_length = 0;
                    int ref_pos = ref_offset;
                    int ref_pos_c = ref_offset_c;
                    trav_order_index = p_trav_scan_order[processed_count];
                    trav_x = GET_TRAV_X(trav_order_index, width);
                    trav_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                    if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, x_start_in_pic + trav_x + mv_x, y_start_in_pic + trav_y + mv_y, processed_count, curr_match_start_x, curr_match_start_y)) 
                    {
                        assert(curr_match_start_x == x_start_in_pic + trav_x + mv_x);
                        assert(curr_match_start_y == y_start_in_pic + trav_y + mv_y);
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                    if (is_pix_in_errorlimit(trav_order_index, ref_pos, ref_pos_c, error_limit, p_cur_sp_info->tree_status))
                    {
                        c_match_length++;
                        pc++;
                        while ((int)pc < count)
                        {
                            for (int idx = 0; idx < 4; idx++)
                            {
                                if (idx == 0 && c_match_length == 1)
                                {
                                    match_dict[0] = 1;
                                    continue;
                                }
                                int i_cur_x, i_cur_y;
                                trav_order_index = p_trav_scan_order[pc];
                                i_cur_x = GET_TRAV_X(trav_order_index, width);
                                i_cur_y = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                                i_ref_x = ((x_start_in_pic + i_cur_x) & (lcu_width - 1)) + mv_x;
                                i_ref_y = ((y_start_in_pic + i_cur_y) & (lcu_height - 1)) + mv_y;
                                int iRefXInPic = x_start_in_pic + i_cur_x + mv_x;
                                int iRefYInPic = y_start_in_pic + i_cur_y + mv_y;
                                //Test if the reference pixel is within Search Range
#if GS_REF_RANGE_XLCUR==0       
                                if (i_ref_x < -left_search_range || iRefXInPic >= m_pic_width || i_ref_x >= (int)lcu_width || iRefYInPic >= m_pic_height || i_ref_y >= (int)lcu_height || i_ref_y < 0)
                                {
                                    memset(match_dict, 0, 4 * sizeof(int));
                                    is_bv_valid = 0;
                                    break; //out of the valid research range
                                }
#else 
                                if (i_ref_x < -left_search_range || iRefXInPic >= m_pic_width || i_ref_x >= ctu_xrightbound || iRefYInPic >= m_pic_height || i_ref_y >= (int)lcu_height || i_ref_y < -top_search_range)
                                {
                                    memset(match_dict, 0, 4 * sizeof(int));
                                    is_bv_valid = 0;
                                    break; //out of the valid research range
                                }
#endif
                                int above_merge_allow = processed_count;
                                if (is_hor_scan)
                                {
                                    above_merge_allow = mv_y <= -1 ? pc : processed_count;
                                }
#if ERR_BUGFIX
                                if (iRefXInPic < 0 || iRefXInPic >= m_pic_width || iRefYInPic < 0 || iRefYInPic >= m_pic_height)
                                {
                                    memset(match_dict, 0, 4 * sizeof(int));
                                    is_bv_valid = 0;
                                    break; //out of the valid research range
                                }
#endif
                                if (!is_ref_pix_in_one_ctu(p_raster_scan_order, p_cur_sp_info, iRefXInPic, iRefYInPic, above_merge_allow, curr_match_start_x, curr_match_start_y))
                                {
                                    memset(match_dict, 0, 4 * sizeof(int));
                                    is_bv_valid = 0;
                                    break;
                                }
                                ref_pos = iRefYInPic * m_stride_org[0] + iRefXInPic;
                                ref_pos_c = (iRefYInPic >> scale_y) * m_stride_org[1] + (iRefXInPic >> scale_x);
                                if (is_pix_in_errorlimit(trav_order_index, ref_pos, ref_pos_c, error_limit, p_cur_sp_info->tree_status))
                                {
                                    match_dict[idx] = 1;
                                    c_match_length++;
                                    pc++;
                                }
                                else
                                {
                                    match_dict[idx] = 0;
                                    pc++;
                                }
                            }

                            if (!is_bv_valid)
                            {
                                c_match_length = (c_match_length >> 2) << 2;
                                break;
                            }
                            else if (c_match_length != pc - processed_count)
                            {
                                c_match_length = (c_match_length >> 2) << 2;
                                break;
                            }
                        }
                    }
                    curr_match_length = c_match_length;
                }
                if (curr_match_length)
                {
                    match_type = 1;
                }
                else
                {
                    match_type = 0;
                    curr_match_length = 4;
                }
#if ExtStrGS||RBU_LIMIT_GS
                for (int k = 0; k < curr_match_length; k++) {
                    int current_x1;
                    int current_y1;
                    int trav_order_index1 = p_trav_scan_order[processed_count + k];
                    int trav_x1 = GET_TRAV_X(trav_order_index1, width);
                    int trav_y1 = GET_TRAV_Y(trav_order_index1, p_cur_sp_info->cu_width_log2);
                    current_x1 = x_start_in_pic + trav_x1;
                    current_y1 = y_start_in_pic + trav_y1;
                    int ref_start_x_in_pic1 = current_x1 + mv_x;
                    int ref_start_y_in_pic1 = current_y1 + mv_y;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#endif
#if ExtStrGS
                if (is_out_of_raw_range_flag)
                {
                    if (curr_match_length < ExtStrLenMin)
                    {
                        curr_match_start_x = next_match_start_x;
                        curr_match_start_y = next_match_start_y;
                        continue;
                    }
                }
#endif
                if (match_type)
                {
                    u8 isBCU = FALSE;
                    str_dist_y = 0; str_dist_uv = 0;
                    if (is_hor_scan)
                    {
                        int offset_in_cu = 0;
#if STR_SELECT_OPT
                        int mxLen = (max_match_length == 0) ? 0 : max_match_length - 13;
#else
                        int mxLen = (max_match_length == 0) ? 0 : max_match_length - 1;
#endif
                        if (curr_match_length != 0 && curr_match_length >= mxLen)
                        {
                            trav_order_index = p_trav_scan_order[processed_count];
                            int offset_sign_flag = 0;
                            int offset_x_in_cu = GET_TRAV_X(trav_order_index, width);
                            int offset_y_in_cu = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                            if (1 == (offset_y_in_cu & 0x1))
                            {
                                offset_sign_flag = 1;
                            }
                            if (error_limit != 1)
                            {
                                get_string_dist(p_cur_sp_info, mv_x, mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv);
                            }
                            int offset_map_value = offset_x_in_cu;
                            if (1 == (offset_y_in_cu & 0x1))
                            {
                                offset_map_value++;
                            }
                            else
                            {
                                offset_map_value += curr_match_length;
                            }
                            offset_in_cu = COM_MIN(width, offset_map_value);
                            offset_in_cu--;
                            cur_rdcost = get_hor_sp_cost(curr_match_start_x - current_x, curr_match_start_y - current_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]),
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag);
                        }
                        else
                            cur_rdcost = SP_MAX_COST;
                    }
                    else
                    {
                        int offset_in_cu = 0;
                        int mxLen = (max_match_length == 0) ? 0 : max_match_length - 1;
                        if (curr_match_length != 0 && (int)curr_match_length >= mxLen)
                        {
                            trav_order_index = p_trav_scan_order[processed_count];
                            int offset_sign_flag = 0;
                            int offset_x_in_cu = GET_TRAV_X(trav_order_index, width);
                            int offset_y_in_cu = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                            if (1 == (offset_x_in_cu & 0x1))
                            {
                                offset_sign_flag = 1;
                            }
                            if (error_limit != 1)
                            {
                                get_string_dist(p_cur_sp_info, mv_x, mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv);
                            }
                            cur_rdcost = get_ver_sp_cost(curr_match_start_x - current_x, curr_match_start_y - current_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]),
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag);
                        }
                        else
                            cur_rdcost = SP_MAX_COST;
                    }
                }
                else
                {
                    u8 isBCU = FALSE;
                    str_dist_y = 0; str_dist_uv = 0;
                    if (is_hor_scan)
                    {
                        int offset_in_cu = 0;
                        int mxLen = (max_match_length == 0) ? 0 : max_match_length - 1;
                        if (curr_match_length != 0 && curr_match_length >= mxLen)
                        {
                            trav_order_index = p_trav_scan_order[processed_count];
                            int offset_sign_flag = 0;
                            int offset_x_in_cu = GET_TRAV_X(trav_order_index, width);
                            int offset_y_in_cu = GET_TRAV_Y(trav_order_index, p_cur_sp_info->cu_width_log2);
                            if (1 == (offset_y_in_cu & 0x1))
                            {
                                offset_sign_flag = 1;
                            }
                            if (error_limit != 1)
                            {
                                get_unmatched_string_dist(p_cur_sp_info, mv_x, mv_y, curr_match_length, processed_count, scale_x, scale_y, ui_shift_y, ui_shift_c, &str_dist_y, &str_dist_uv, match_dict);
                            }
                            int offset_map_value = offset_x_in_cu;
                            if (1 == (offset_y_in_cu & 0x1))
                            {
                                offset_map_value++;
                            }
                            else
                            {
                                offset_map_value += curr_match_length;
                            }
                            offset_in_cu = COM_MIN(width, offset_map_value);
                            offset_in_cu--;
                            cur_rdcost = get_unmatched_sp_cost(curr_match_start_x - current_x, curr_match_start_y - current_y, (u32)(str_dist_y + str_dist_uv * p_cur_sp_info->chroma_weight[0]),
                                offset_in_cu, isBCU, curr_match_length, offset_sign_flag, count - processed_count - 1, &str_bits, p_cur_sp_info, &curr_n_recent_idx, &curr_n_recent_flag, match_dict, p_trav_scan_order, processed_count);
                        }
                        else
                        {
                            cur_rdcost = SP_MAX_COST;
                        }
                    }
                }
#if STR_SELECT_OPT
                if (((curr_match_length == max_match_length) && curr_match_length != 0 && cur_rdcost < min_rdcost)
                    || ((curr_match_length >= max_match_length - 13) && curr_match_length != 0 && (cur_rdcost * max_match_length) < (min_rdcost * curr_match_length))
                    )
#else
                if (curr_match_length > max_match_length
                    || ((curr_match_length == max_match_length) && curr_match_length != 0 && cur_rdcost < min_rdcost)
                    || ((curr_match_length == max_match_length - 1) && curr_match_length != 0 && (cur_rdcost*max_match_length) < (min_rdcost*curr_match_length))
                    )
#endif
                {
#if STR_SELECT_OPT
                    assert((1 - cur_rdcost * max_match_length / (curr_match_length * min_rdcost)) >= 0);
#endif
#if RBU_LIMIT_GS	
                    if (is_out_of_raw_range_flag)
                    {
                        int nbu = 0;
                        int match_sum = 0;
                        match_sum = match_dict[0] + match_dict[1] + match_dict[2] + match_dict[3];
                        int sv[2] = { curr_match_start_x - current_x,curr_match_start_y - current_y };
                        if (match_type == 0 && match_sum == 0) {
                            nbu = 0;
                        }
                        else {
                            int x_end_pos = GET_TRAV_X(p_trav_scan_order[processed_count + curr_match_length - 1], width);
                            int y_end_pos = GET_TRAV_Y(p_trav_scan_order[processed_count + curr_match_length - 1], p_cur_sp_info->cu_width_log2);
                            int end_x_in_pic = x_start_in_pic + x_end_pos;
                            int end_y_in_pic = y_start_in_pic + y_end_pos;
                            nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, current_x, current_y, end_x_in_pic, end_y_in_pic, processed_count, curr_match_length, sv);
                        }
                        if ((stripe_num + nbu) * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height)
                        {
                            curr_match_start_x = next_match_start_x;
                            curr_match_start_y = next_match_start_y;
                            continue;
                        }  
                    }
#endif
                    string_found = match_type;
                    memcpy(best_match_dict, match_dict, 4 * sizeof(int));
                    max_match_length = curr_match_length;
                    best_n_recent_flag = curr_n_recent_flag;
                    best_n_recent_idx = curr_n_recent_idx;
                    best_match_start_x = curr_match_start_x;
                    best_match_start_y = curr_match_start_y;
                    min_rdcost = cur_rdcost;
                    best_str_bits = str_bits;
                    best_str_dist_y = str_dist_y;
                    best_str_dist_uv = str_dist_uv;
                }
                curr_match_start_x = next_match_start_x;
                curr_match_start_y = next_match_start_y;

                if (match_type && str_dist_y + str_dist_uv == 0 && (int)curr_match_length >= (width * height >> 2))
                {
                    break;
                }
            }
        } // end hash search
#if RBU_LIMIT_GS
        for (int k = 0; k < max_match_length; k++) {
            int current_x1;
            int current_y1;
            int trav_order_index1 = p_trav_scan_order[processed_count + k];
            int trav_x1 = GET_TRAV_X(trav_order_index1, width);
            int trav_y1 = GET_TRAV_Y(trav_order_index1, p_cur_sp_info->cu_width_log2);
            current_x1 = x_start_in_pic + trav_x1;
            current_y1 = y_start_in_pic + trav_y1;
            int ref_start_x_in_pic1 = current_x1 + best_match_start_x - current_x;
            int ref_start_y_in_pic1 = current_y1 + best_match_start_y - current_y;
            if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
            {
                is_out_of_raw_range_flag = 1;
                break;
            }
        }
#endif
        if (string_found)
        {
            tmp_dict_rdcost = min_rdcost; //choose Hash RDCost
            tmp_dict_dist_y = best_str_dist_y;
            tmp_dict_dist_uv = best_str_dist_uv;
            COM_SP_INFO dict_info;
#if ISC_RSD
            dict_info.sp_rsd_flag = 0;
            dict_info.intra_row_rsd_flag = 0;
#endif
            dict_info.is_matched = TRUE;
            dict_info.offset_x = best_match_start_x - current_x;
            dict_info.offset_y = best_match_start_y - current_y;
            dict_info.length = max_match_length;
            int y0 = GET_TRAV_Y(p_trav_scan_order[processed_count], p_cur_sp_info->cu_width_log2);
            int y1 = GET_TRAV_Y(p_trav_scan_order[processed_count + max_match_length - 1], p_cur_sp_info->cu_width_log2);
            int str_height = y1 - y0 + 1;
            int above_merge_str_num = 0;
            if (is_str_overlap(dict_info.offset_x, dict_info.offset_y, width, str_height))
            {
                above_merge_str_num = (str_height - 1) / (-dict_info.offset_y) + 1; //round up for (str_height / ySv)
            }
            else
            {
                above_merge_str_num = 1;
            }
            additional_str_num += above_merge_str_num - 1;
            dict_info.n_recent_flag = best_n_recent_flag;
            dict_info.n_recent_idx = best_n_recent_idx;
            pel former_pix[N_C] = { 0 };
            pel replic_pix[N_C] = { 0 };
            //reconstruction
#if RBU_LIMIT_GS
            out_stripe_num = 0;
            if (is_out_of_raw_range_flag)
            {
                int sv[2] = { dict_info.offset_x,dict_info.offset_y };
                int x_end_pos = GET_TRAV_X(p_trav_scan_order[processed_count + dict_info.length - 1], width);
                int y_end_pos = GET_TRAV_Y(p_trav_scan_order[processed_count + dict_info.length - 1], p_cur_sp_info->cu_width_log2);
                int end_x_in_pic = x_start_in_pic + x_end_pos;
                int end_y_in_pic = y_start_in_pic + y_end_pos;
                out_stripe_num = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, current_x, current_y, end_x_in_pic, end_y_in_pic, processed_count, dict_info.length, sv);
            }
#endif
            for (int pixelCount = 0; pixelCount < dict_info.length; pixelCount++)
            {
                int cur_pos_x;
                int cur_pos_y;
                int cur_pixel_pos = p_trav_scan_order[processed_count + pixelCount];
                trav_x = GET_TRAV_X(cur_pixel_pos, width);
                trav_y = GET_TRAV_Y(cur_pixel_pos, p_cur_sp_info->cu_width_log2);
                cur_pos_x = x_start_in_pic + trav_x;
                cur_pos_y = y_start_in_pic + trav_y;
                int cur_pos = cur_pos_y * m_stride_rec[0] + cur_pos_x;
                int cur_pos_c = (cur_pos_y >> scale_y) * m_stride_rec[1] + (cur_pos_x >> scale_x);
                int ref_pos = (cur_pos_y + dict_info.offset_y) * m_stride_rec[0] + (cur_pos_x + dict_info.offset_x);
                int ref_pos_c = ((cur_pos_y + dict_info.offset_y) >> scale_y) * m_stride_rec[1] + ((cur_pos_x + dict_info.offset_x) >> scale_x);
                int cur_blk_pos = trav_y * width + trav_x;
                int cur_blk_pos_c = (trav_y >> scale_y)*(width >> scale_x) + (trav_x >> scale_x);
                p_cur_sp_info->rec[0][cur_blk_pos] = m_p_pel_rec[0][ref_pos];
                m_p_pel_rec[0][cur_pos] = m_p_pel_rec[0][ref_pos];
                if (!((cur_pos_x & scale_x) || (cur_pos_y & scale_y)))
                {
                    m_p_pel_rec[1][cur_pos_c] = m_p_pel_rec[1][ref_pos_c];
                    m_p_pel_rec[2][cur_pos_c] = m_p_pel_rec[2][ref_pos_c];
                    p_cur_sp_info->rec[1][cur_blk_pos_c] = m_p_pel_rec[1][ref_pos_c];
                    p_cur_sp_info->rec[2][cur_blk_pos_c] = m_p_pel_rec[2][ref_pos_c];
                }
                int curPosOrg = cur_pos_y * m_stride_org[0] + cur_pos_x;
                int curPosCOrg = (cur_pos_y >> scale_y) * m_stride_org[1] + (cur_pos_x >> scale_x);
                int diff[3] = { 0 };
                diff[0] = abs(m_p_pel_org[0][curPosOrg] - former_pix[0]);
                diff[1] = abs(m_p_pel_org[1][curPosCOrg] - former_pix[1]);
                diff[2] = abs(m_p_pel_org[2][curPosCOrg] - former_pix[2]);
                replicate_num = (diff[0] + diff[1] + diff[2] <= 8) ? replicate_num + 1 : replicate_num;
                same_num = (m_p_pel_org[0][curPosOrg] == former_pix[0]) ? same_num + 1 : same_num;
                former_pix[0] = m_p_pel_org[0][curPosOrg];
                former_pix[1] = m_p_pel_org[1][curPosCOrg];
                former_pix[2] = m_p_pel_org[2][curPosCOrg];
            }
            //add to hash 
            add_pixel_to_hash_table(max_match_length, TRUE, x_start_in_pic, y_start_in_pic, cu_width_log2, cu_height_log2, processed_count, is_hor_scan);
            vecDictInfo.push_back(dict_info);
            processed_count += max_match_length;
        }
        else
        {
            COM_SP_INFO dict_info;
#if ISC_RSD
            dict_info.sp_rsd_flag = 0;
            dict_info.intra_row_rsd_flag = 0;
#endif
            dict_info.is_matched = FALSE;
            dict_info.offset_x = best_match_start_x - current_x;
            dict_info.offset_y = best_match_start_y - current_y;
            dict_info.length = 4;
            dict_info.n_recent_flag = best_n_recent_flag;
            dict_info.n_recent_idx = best_n_recent_idx;
            memcpy(dict_info.match_dict, best_match_dict, 4 * sizeof(int));

            tmp_dict_rdcost = min_rdcost;
            tmp_dict_dist_y = best_str_dist_y;
            tmp_dict_dist_uv = best_str_dist_uv;
#if RBU_LIMIT_GS
            out_stripe_num = 0;
            if (is_out_of_raw_range_flag)
            {
                int match_sum = 0;
                match_sum = dict_info.match_dict[0] + dict_info.match_dict[1] + dict_info.match_dict[2] + dict_info.match_dict[3];
                if (match_sum != 0)
                {
                    out_stripe_num = 0;
                    {
                        int sv[2] = { dict_info.offset_x,dict_info.offset_y };
                        int x_end_pos = GET_TRAV_X(p_trav_scan_order[processed_count + dict_info.length - 1], width);
                        int y_end_pos = GET_TRAV_Y(p_trav_scan_order[processed_count + dict_info.length - 1], p_cur_sp_info->cu_width_log2);
                        int end_x_in_pic = x_start_in_pic + x_end_pos;
                        int end_y_in_pic = y_start_in_pic + y_end_pos;
                        out_stripe_num = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, current_x, current_y, end_x_in_pic, end_y_in_pic, processed_count, dict_info.length, sv);
                    }
                }
                else
                {
                    out_stripe_num = 0;
                }
            }
#endif
            int unmatched_pixel_num = 0;
            for (int pixel_count = 0; pixel_count < dict_info.length; pixel_count++)
            {
                int cur_pos_x;
                int cur_pos_y;
                int cur_pixel_pos = p_trav_scan_order[processed_count + pixel_count];
                trav_x = GET_TRAV_X(cur_pixel_pos, width);
                trav_y = GET_TRAV_Y(cur_pixel_pos, p_cur_sp_info->cu_width_log2);
                cur_pos_x = x_start_in_pic + trav_x;
                cur_pos_y = y_start_in_pic + trav_y;

                dict_info.pixel[pixel_count][Y_C] = m_p_pel_org[0][cur_pos_y*m_stride_org[0] + cur_pos_x];
                dict_info.pixel[pixel_count][U_C] = m_p_pel_org[1][(cur_pos_y >> scale_y)*m_stride_org[1] + (cur_pos_x >> scale_x)];
                dict_info.pixel[pixel_count][V_C] = m_p_pel_org[2][(cur_pos_y >> scale_y)*m_stride_org[2] + (cur_pos_x >> scale_x)];

                pel ydata = dict_info.pixel[pixel_count][Y_C];
                pel udata = dict_info.pixel[pixel_count][U_C];
                pel vdata = dict_info.pixel[pixel_count][V_C];
                int cur_pos = cur_pos_y * m_stride_rec[0] + cur_pos_x;
                int cur_pos_c = (cur_pos_y >> scale_y) * m_stride_rec[1] + (cur_pos_x >> scale_x);
                int cur_blk_pos = trav_y * width + trav_x;
                int cur_blk_pos_c = (trav_y >> scale_y)*(width >> scale_x) + (trav_x >> scale_x);
                int ref_pos = (cur_pos_y + dict_info.offset_y) * m_stride_rec[0] + (cur_pos_x + dict_info.offset_x);
                int ref_pos_c = ((cur_pos_y + dict_info.offset_y) >> scale_y) * m_stride_rec[1] + ((cur_pos_x + dict_info.offset_x) >> scale_x);

                if (dict_info.match_dict[pixel_count])
                {
                    p_cur_sp_info->rec[0][cur_blk_pos] = m_p_pel_rec[0][ref_pos];
                    m_p_pel_rec[0][cur_pos] = m_p_pel_rec[0][ref_pos];
                }
                else
                {
                    m_p_pel_rec[0][cur_pos] = ydata;
                    p_cur_sp_info->rec[0][cur_blk_pos] = ydata;
                    unmatched_pixel_num++;
                }

                if (!((cur_pos_x & scale_x) || (cur_pos_y & scale_y)))
                {
                    if (dict_info.match_dict[pixel_count])
                    {
                        m_p_pel_rec[1][cur_pos_c] = m_p_pel_rec[1][ref_pos_c];
                        m_p_pel_rec[2][cur_pos_c] = m_p_pel_rec[2][ref_pos_c];
                        p_cur_sp_info->rec[1][cur_blk_pos_c] = m_p_pel_rec[1][ref_pos_c];
                        p_cur_sp_info->rec[2][cur_blk_pos_c] = m_p_pel_rec[2][ref_pos_c];
                    }
                    else
                    {
                        m_p_pel_rec[1][cur_pos_c] = udata;
                        m_p_pel_rec[2][cur_pos_c] = vdata;
                        p_cur_sp_info->rec[1][cur_blk_pos_c] = udata;
                        p_cur_sp_info->rec[2][cur_blk_pos_c] = vdata;
                    }
                }
            }
            additional_str_num += unmatched_pixel_num < 4 ? unmatched_pixel_num : 3;
            //add to hash
            add_pixel_to_hash_table(dict_info.length, TRUE, x_start_in_pic, y_start_in_pic, cu_width_log2, cu_height_log2, processed_count, is_hor_scan);
            vecDictInfo.push_back(dict_info);
            processed_count += dict_info.length;
        }
#if RBU_LIMIT_GS
        if (is_out_of_raw_range_flag)
        {
            stripe_num += out_stripe_num;
            if (stripe_num * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height)
            {
                return FALSE;
            }
        }
#endif
        dict_dist_uv += tmp_dict_dist_uv;
        dict_dist_y += tmp_dict_dist_y;
        dict_rdcost += tmp_dict_rdcost;
        if (replicate_num < processed_count >> 2 && processed_count > 256 && width >= 32 && height >= 32)
        {
            if (same_num < (processed_count >> 3))
            {
                p_cur_sp_info->is_sp_skip_non_scc = 1;
            }
            return FALSE;
        }
        if (dict_rdcost > *curBestRDCost)
        {
            return FALSE;
        }
        if (vecDictInfo.size() + additional_str_num > p_cur_sp_info->max_str_cnt)
            return FALSE;
    } //endwhile
    assert(processed_count == count);
    if (processed_count == count)
    {
        p_cur_sp_info->is_sp_pix_completed = TRUE;
    }
    if (p_cur_sp_info->tree_status == TREE_LC)
    {
        *distortion = dict_dist_y + dict_dist_uv * p_cur_sp_info->chroma_weight[0];
        *curBestRDCost = dict_rdcost;
    }
    else
    {
        assert(p_cur_sp_info->tree_status == TREE_L);
        *distortion = dict_dist_y;
        *curBestRDCost = dict_rdcost;
    }
    return TRUE;
}

u8 string_prediction::encode_cu_size_str(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, vector<COM_SP_INFO>& vecDictInfo, double* cur_best_rdcost, double* distortion)
{
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    COM_SP_POS best_sp[2];
    COM_SP_POS temp_sp;
    u32 ui_temp_distortion = 0;
    u32 ui_temp_distortion_uv = 0;
    int cu_pel_x, cu_pel_y;
    pel *org_pic;
    pel *ref_pic;
    int sp_roi_width, sp_roi_height;
    int bitDepth = input->sample_bit_depth;
    u8 shift = (bitDepth == 8) ? 0 : (bitDepth - 8) << 1;
    sp_roi_width = width; 
    sp_roi_height = height;     
    int numOfStrings = 1; //only one cu_size_string is searched

    for (int nos = 0; nos < numOfStrings; nos++)
    {
        cu_pel_x = x_start_in_pic;
        cu_pel_y = y_start_in_pic;
#if FAST_SCC_PRECODING
        if (!p_cur_sp_info->spvisit)
        {
            cu_size_str_search(p_cur_sp_info, x_start_in_pic, y_start_in_pic, temp_sp);
            p_cur_sp_info->sp_transmit_x = temp_sp.x;
            p_cur_sp_info->sp_transmit_y = temp_sp.y;

        }
        else
        {
            cu_size_str_searchrevisit(p_cur_sp_info, x_start_in_pic, y_start_in_pic, temp_sp);
        }
#else 
        cu_size_str_search(p_cur_sp_info, x_start_in_pic, y_start_in_pic, temp_sp);
#endif
        best_sp[nos].x = temp_sp.x;
        best_sp[nos].y = temp_sp.y;
        if (best_sp[nos].x == 0 && best_sp[nos].y == 0)
            return 0;
        else
        {
            for (u8 ch = 0; ch < 3; ch++)
            {
                org_pic = m_p_pel_org[ch];
                ref_pic = m_p_pel_rec[ch];
                u8 scale_x = (ch == 0 ? 0 : 1);
                u8 scale_y = (ch == 0 ? 0 : 1);
                int org_pos = (cu_pel_y >> scale_y) * m_stride_org[ch] + (cu_pel_x >> scale_x);
                int ref_pos = ((cu_pel_y + best_sp[nos].y) >> scale_y) * m_stride_rec[ch] + ((cu_pel_x + best_sp[nos].x) >> scale_x);
                int cur_pos = (cu_pel_y >> scale_y) * m_stride_rec[ch] + (cu_pel_x >> scale_x);
                int recStride = 0;
                for (int row = 0; row < (sp_roi_height >> scale_y); row++)
                {
                    for (int col = 0; col < (sp_roi_width >> scale_x); col++)
                    {
                        int cu_ref_pos = ref_pos + col;
                        int cu_org_pos = org_pos + col;
                        if (ch == 0)
                        {
                            ui_temp_distortion += ((ref_pic[cu_ref_pos] - org_pic[cu_org_pos])*(ref_pic[cu_ref_pos] - org_pic[cu_org_pos]));
                        }
                        else
                        {
                            ui_temp_distortion_uv += ((ref_pic[cu_ref_pos] - org_pic[cu_org_pos])*(ref_pic[cu_ref_pos] - org_pic[cu_org_pos]));
                        }
                        p_cur_sp_info->rec[ch][recStride + col] = m_p_pel_rec[ch][cu_ref_pos];
                        m_p_pel_rec[ch][cur_pos + col] = m_p_pel_rec[ch][cu_ref_pos];
                    }
                    org_pos += m_stride_org[ch];
                    ref_pos += m_stride_rec[ch];
                    cur_pos += m_stride_rec[ch];
                    recStride += (sp_roi_width >> scale_x);
                }
            }
        }
    } //endfor nos
    if ((best_sp[0].x == 0) && (best_sp[0].y == 0))
    {
        return 0;
    }
    double cu_size_str_rdcost, cu_size_str_distortion;
    if (p_cur_sp_info->tree_status == TREE_LC)
    {
        cu_size_str_distortion = (ui_temp_distortion >> shift) + (ui_temp_distortion_uv >> shift) * p_cur_sp_info->chroma_weight[0];
        cu_size_str_rdcost = get_sp_offset_bit_cnt(best_sp[0].x, best_sp[0].y)*p_cur_sp_info->lamda + cu_size_str_distortion;
    }
    else
    {
        assert(p_cur_sp_info->tree_status == TREE_L);
        cu_size_str_distortion = ui_temp_distortion >> shift;
        cu_size_str_rdcost = get_sp_offset_bit_cnt(best_sp[0].x, best_sp[0].y)*p_cur_sp_info->lamda + cu_size_str_distortion;
    }
    if (cu_size_str_rdcost > *cur_best_rdcost)
    {
        return FALSE;
    }
    else
    {
        *distortion = cu_size_str_distortion;
        *cur_best_rdcost = cu_size_str_rdcost;
    }
#if ExtStrCSS||RBU_LIMIT_CSS	
    {
        int is_out_of_raw_range_flag = 0;
        int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
        for (int k = 0; k < 4; k++) {
            int current_x1;
            int current_y1;
            current_x1 = x_start_in_pic + tmp[k][0];
            current_y1 = y_start_in_pic + tmp[k][1];
            int ref_start_x_in_pic1 = current_x1 + best_sp[0].x;
            int ref_start_y_in_pic1 = current_y1 + best_sp[0].y;
            if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
            {
                is_out_of_raw_range_flag = 1;
                break;
            }
        }
#if ExtStrCSS
        if (width * height < ExtStrLenMin)
        {
            if (is_out_of_raw_range_flag)
            {
                return FALSE;
            }
        }
#endif
#if RBU_LIMIT_CSS	

        if (is_out_of_raw_range_flag)
        {
            int nbu = 0;
            int sv[2] = { best_sp[0].x,best_sp[0].y };
            int end_x_in_pic = x_start_in_pic;
            int end_y_in_pic = y_start_in_pic + height - 1;
            nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);
            if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height)
            {
                return FALSE;
            }
        }
#endif
    }
#endif
    COM_SP_INFO dict_info;
#if ISC_RSD
    dict_info.sp_rsd_flag = 0;
    dict_info.intra_row_rsd_flag = 0;
#endif
    dict_info.is_matched = TRUE;
    dict_info.offset_x = best_sp[0].x;
    dict_info.offset_y = best_sp[0].y;
    dict_info.length = (u16)width * height;
    dict_info.n_recent_flag = 0;
    int index = check_sp_offset(dict_info.offset_x, dict_info.offset_y, p_cur_sp_info->n_cand_num, p_cur_sp_info->n_cand);
    if (index != p_cur_sp_info->n_cand_num)
    {
        dict_info.n_recent_idx = p_cur_sp_info->n_cand_num - 1 - (s8)index;
        dict_info.n_recent_flag = 1;
    }
    vecDictInfo.push_back(dict_info);
    p_cur_sp_info->is_sp_pix_completed = 1;
    return 1;
}

void string_prediction::cu_size_str_search(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, COM_SP_POS& sp)
{
    //set search range
    int srLeft, srRight, srTop, srBottom;
    int cu_pel_x, cu_pel_y;
    int sp_roi_width, sp_roi_height;
    SP_CU_SIZE_STRING  best_coding_info;
    best_coding_info.sm.x = 0;
    best_coding_info.sm.y = 0;
    COM_SP_POS best_sp;
    const int lcu_width = m_max_cu_width;
    const int lcu_height = m_max_cu_height;
    const int i_pic_width = m_pic_width;
    const int i_pic_height = m_pic_height;
    pel* ref_pic = m_p_pel_rec[0];
    pel* org_pic = m_p_pel_org[0];
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int i_best_cand_idx = 0;
    sp_roi_width = width;
    sp_roi_height = height;
    cu_pel_x = x_start_in_pic;
    cu_pel_y = y_start_in_pic;

    org_pic = m_p_pel_org[0];
    ref_pic = m_p_pel_rec[0];
    const int iRelCUPelX = cu_pel_x & (lcu_width - 1);
    const int iRelCUPelY = cu_pel_y & (lcu_height - 1);
    const int ctu_log2Size = p_cur_sp_info->ctu_log2size;
    int numLeftCTUs = (1 << ((7 - ctu_log2Size) << 1)) - ((ctu_log2Size < 7) ? 1 : 0);
    int maxXsr = iRelCUPelX + get_sp_search_area_width(x_start_in_pic, numLeftCTUs);
#if CSS_REF_RANGE_XqLCU!=0 && CSS_REF_RANGE_XqLCU!=1
#if REF_RANGE_FIX
    int ctu_xleft = cu_pel_x >> (SRAM_SIZE_128_LOG2 - 1) << (SRAM_SIZE_128_LOG2 - 1);
    int ctu_ytop = cu_pel_y >> (SRAM_SIZE_128_LOG2 - 1) << (SRAM_SIZE_128_LOG2 - 1);
#else
    int ctu_xleft = cu_pel_x >> 6 << 6;
    int ctu_ytop = cu_pel_y >> 6 << 6;
#endif
#if ERR_BUGFIX
    int residual_x = ctu_xleft % m_max_cu_width;
    int residual_y = ctu_ytop % m_max_cu_height;
    if (residual_x == 0 || residual_y == 0)
    {
        maxXsr += (CSS_REF_RANGE_XqLCU - 1) * m_max_cu_width;
    }
    else
    {
        maxXsr += (CSS_REF_RANGE_XqLCU - 2) * m_max_cu_width;
    }
#endif
#endif
    int maxYsr = iRelCUPelY;
#if CSS_REF_RANGE_XLCUR!=0
    maxYsr += (CSS_REF_RANGE_XLCUR - 1) * m_max_cu_height;
#endif
    if ((input->chroma_format == 0) || (input->chroma_format == 2)) maxXsr &= ~0x4;
    if ((input->chroma_format == 0)) maxYsr &= ~0x4;
    srLeft = -maxXsr;
    srTop = -maxYsr;
#if CSS_REF_RANGE_XqLCU!=0
    int boundary_xleft = srLeft + cu_pel_x;
    if (boundary_xleft < 0)
    {
        srLeft = -cu_pel_x;
    }
    else
    {
        srLeft = srLeft;
    }
#endif
#if CSS_REF_RANGE_XLCUR!=0
    int boundary_yleft = srTop + cu_pel_y;
    if (boundary_yleft < 0)
    {
        srTop = -cu_pel_y;
    }
    else
    {
        srTop = srTop;
    }
#endif
    srRight = lcu_width - iRelCUPelX - sp_roi_width;
    srBottom = lcu_height - iRelCUPelY - sp_roi_height;
    if ((int)cu_pel_x + srRight + sp_roi_width > i_pic_width)
    {
        srRight = i_pic_width % lcu_width - iRelCUPelX - sp_roi_width;
    }
    if ((int)cu_pel_y + srBottom + sp_roi_height > i_pic_height)
    {
        srBottom = i_pic_height % lcu_height - iRelCUPelY - sp_roi_height;
    }
    //end search range
    //Do integer search    
    u32  uiSad;
    u32  uiSadBestCand[SP_CHROMA_REFINE_CANDS];
    SP_CU_SIZE_STRING      cSMPositionCand[SP_CHROMA_REFINE_CANDS];
    u32  uiSadBest = 0xFFFFFFFFU;
    u32  uiTempSadBest = 0;
    const int boundY = -1;
    for (int iCand = 0; iCand < SP_CHROMA_REFINE_CANDS; iCand++)
    {
        uiSadBestCand[iCand] = 0xFFFFFFFFU;
        cSMPositionCand[iCand].sm.x = 0;
        cSMPositionCand[iCand].sm.y = 0;
    }

    int lowY;
#if SCC_CROSSINFO_ULTILIZE
    for (int i = 0; i < 1; i++)
    {
        int x = p_cur_sp_info->ibc_mv[0];
        int y = p_cur_sp_info->ibc_mv[1];

        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
#if ExtStrCSS||RBU_LIMIT_CSS
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = x_start_in_pic + tmp[k][0];
                current_y1 = y_start_in_pic + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1 + x;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrCSS
            if (width * height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_CSS	
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = x_start_in_pic;
                int end_y_in_pic = y_start_in_pic + height - 1;
                nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);

                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                    continue;
                }
            }
#endif
        }
#endif
        uiSad = 1;
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (cu_pel_y + y + r) * m_stride_rec[0] + cu_pel_x + x;
            int cu_org_pos = (cu_pel_y + r) * m_stride_org[0] + cu_pel_x;

            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
        sp_postion_cand_update(uiSad, x, y, uiSadBestCand, cSMPositionCand);
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }
#endif

    // svp search
    for (int i = 0; i < p_cur_sp_info->n_cand_num;i++)
    {
        int x = p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_X];
        int y = p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_Y];
        if (cu_pel_y + y < 0 || cu_pel_y + sp_roi_height + y >= m_pic_height || cu_pel_x + x < 0 || cu_pel_x + sp_roi_width + x >= m_pic_width)
        {
            continue;
        }
        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
#if ExtStrCSS||RBU_LIMIT_CSS
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = x_start_in_pic + tmp[k][0];
                current_y1 = y_start_in_pic + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1 + x;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrCSS
            if (width * height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_CSS	
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = x_start_in_pic;
                int end_y_in_pic = y_start_in_pic + height - 1;
                nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);
                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                    continue;
                }
            }
#endif
        }
#endif
        uiSad = (u32)get_tb_bits(p_cur_sp_info->n_cand_num - 1 - i, p_cur_sp_info->n_cand_num);
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (cu_pel_y + y + r) * m_stride_rec[0] + cu_pel_x + x;
            int cu_org_pos = (cu_pel_y + r) * m_stride_org[0] + cu_pel_x;
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
        sp_postion_cand_update(uiSad, x, y, uiSadBestCand, cSMPositionCand);
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }
    
    // ver 1D search
    lowY = (srTop >= (int)(0 - cu_pel_y)) ? srTop : (0 - cu_pel_y);
    for (int y = boundY; y >= lowY; y--)
    {
        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
#if ExtStrCSS||RBU_LIMIT_CSS
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = x_start_in_pic + tmp[k][0];
                current_y1 = y_start_in_pic + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrCSS
            if (width * height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_CSS	
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { 0,y };
                int end_x_in_pic = x_start_in_pic;
                int end_y_in_pic = y_start_in_pic + height - 1;
                nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);

                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                    continue;
                }
            }
#endif
        }
#endif
        uiSad = (u32)get_sp_offset_bit_cnt(0, -y);
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (y + r + cu_pel_y) * m_stride_rec[0] + cu_pel_x;
            int cu_org_pos = cu_pel_x + (r + cu_pel_y) * m_stride_org[0];
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
        sp_postion_cand_update(uiSad, 0, y, uiSadBestCand, cSMPositionCand);
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }
    // hor 1D search
    {
        const int boundX = (srLeft >= (int)(0 - cu_pel_x)) ? srLeft : (0 - cu_pel_x);
        for (int x = 0 - sp_roi_width; x >= boundX; --x)
        {
            if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
            {
                continue;
            }
#if ExtStrCSS||RBU_LIMIT_CSS	
            {
                int is_out_of_raw_range_flag = 0;
                int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
                for (int k = 0; k < 4; k++) {
                    int current_x1;
                    int current_y1;
                    current_x1 = x_start_in_pic + tmp[k][0];
                    current_y1 = y_start_in_pic + tmp[k][1];
                    int ref_start_x_in_pic1 = current_x1 + x;
                    int ref_start_y_in_pic1 = current_y1;
                    if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                    {
                        is_out_of_raw_range_flag = 1;
                        break;
                    }
                }
#if ExtStrCSS
                if (width * height < ExtStrLenMin)
                {
                    if (is_out_of_raw_range_flag)
                    {
                        continue;
                    }
                }
#endif
#if RBU_LIMIT_CSS	
                if (is_out_of_raw_range_flag)
                {
                    int nbu = 0;
                    int sv[2] = { x,0 };
                    int end_x_in_pic = x_start_in_pic;
                    int end_y_in_pic = y_start_in_pic + height - 1;
                    nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);
                    if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                        continue;
                    }
                }
#endif
            }
#endif
            uiSad = (u32)get_sp_offset_bit_cnt(-x, 0);
            for (int r = 0; r < sp_roi_height; )
            {
                int cu_ref_pos = (r + cu_pel_y)* m_stride_rec[0] + x + cu_pel_x;
                int cu_org_pos = (r + cu_pel_y)* m_stride_org[0] + cu_pel_x;
                for (int j = 0; j < 4; j++)
                {
                    for (int i = 0; i < sp_roi_width; i++)
                    {
                        uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                    }
                    cu_ref_pos += m_stride_rec[0];
                    cu_org_pos += m_stride_org[0];
                }
                if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
                {
                    break;
                }
                r += 4;
            }
            sp_postion_cand_update(uiSad, x, 0, uiSadBestCand, cSMPositionCand);
            uiTempSadBest = uiSadBestCand[0];
            if (uiSadBestCand[0] <= 3)
            {
                copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
                uiSadBest = uiSadBestCand[0];
                goto rec;
            }
        }
    }
    copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
    best_sp.x = best_coding_info.sm.x;
    best_sp.y = best_coding_info.sm.y;
    uiSadBest = uiSadBestCand[0];
    i_best_cand_idx = 0;
    if ((uiSadBest - (u32)get_sp_offset_bit_cnt(best_sp.x, best_sp.y) <= 32))
    {
        //chroma refine    
        if (p_cur_sp_info->tree_status != TREE_L)
        {
            i_best_cand_idx = sp_postion_chroma_refine(sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y, uiSadBestCand, cSMPositionCand);
        }
        else
        {
            assert(p_cur_sp_info->tree_status == TREE_L);
        }
        copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[i_best_cand_idx]));
        uiSadBest = uiSadBestCand[i_best_cand_idx];
        goto rec;
    }
rec:
    sp.x = best_coding_info.sm.x;
    sp.y = best_coding_info.sm.y;
}

#if FAST_SCC_PRECODING
void string_prediction::cu_size_str_searchrevisit(COM_SP_CODING_UNIT* p_cur_sp_info, int x_start_in_pic, int y_start_in_pic, COM_SP_POS& sp)
{
    //set search range
    int srLeft, srRight, srTop, srBottom;
    int cu_pel_x, cu_pel_y;
    int sp_roi_width, sp_roi_height;
    SP_CU_SIZE_STRING  best_coding_info;
    best_coding_info.sm.x = 0;
    best_coding_info.sm.y = 0;
    COM_SP_POS best_sp;
    const int lcu_width = m_max_cu_width;
    const int lcu_height = m_max_cu_height;
    const int i_pic_width = m_pic_width;
    const int i_pic_height = m_pic_height;
    pel* ref_pic = m_p_pel_rec[0];
    pel* org_pic = m_p_pel_org[0];
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1; //444, scale=0; 422 or 420 or 400, scaleX=1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1; //422 or 444, scale=0; 420 or 400, scaleY=1;
    int width = 1 << p_cur_sp_info->cu_width_log2;
    int height = 1 << p_cur_sp_info->cu_height_log2;
    int i_best_cand_idx = 0;
    sp_roi_width = width;
    sp_roi_height = height;
    cu_pel_x = x_start_in_pic;
    cu_pel_y = y_start_in_pic;

    org_pic = m_p_pel_org[0];
    ref_pic = m_p_pel_rec[0];
    const int iRelCUPelX = cu_pel_x & (lcu_width - 1);
    const int iRelCUPelY = cu_pel_y & (lcu_height - 1);
    const int ctu_log2Size = p_cur_sp_info->ctu_log2size;
    int numLeftCTUs = (1 << ((7 - ctu_log2Size) << 1)) - ((ctu_log2Size < 7) ? 1 : 0);
    int maxXsr = iRelCUPelX + get_sp_search_area_width(x_start_in_pic, numLeftCTUs);
#if CSS_REF_RANGE_XqLCU!=0 && CSS_REF_RANGE_XqLCU!=1
#if REF_RANGE_FIX
    int ctu_xleft = cu_pel_x >> (SRAM_SIZE_128_LOG2 - 1) << (SRAM_SIZE_128_LOG2 - 1);
    int ctu_ytop = cu_pel_y >> (SRAM_SIZE_128_LOG2 - 1) << (SRAM_SIZE_128_LOG2 - 1);
#else
    int ctu_xleft = cu_pel_x >> 6 << 6;
    int ctu_ytop = cu_pel_y >> 6 << 6;
#endif 
#if ERR_BUGFIX
    int residual_x = ctu_xleft % m_max_cu_width;
    int residual_y = ctu_ytop % m_max_cu_height;
    if (residual_x == 0 || residual_y == 0)
    {
        maxXsr += (CSS_REF_RANGE_XqLCU - 1) * m_max_cu_width;
    }
    else
    {
        maxXsr += (CSS_REF_RANGE_XqLCU - 2) * m_max_cu_width;
    }
#endif
#endif
    int maxYsr = iRelCUPelY;
#if CSS_REF_RANGE_XLCUR!=0
    maxYsr += (CSS_REF_RANGE_XLCUR - 1) * m_max_cu_height;
#endif
    if ((input->chroma_format == 0) || (input->chroma_format == 2)) maxXsr &= ~0x4;
    if ((input->chroma_format == 0)) maxYsr &= ~0x4;
    srLeft = -maxXsr;
    srTop = -maxYsr;
#if CSS_REF_RANGE_XqLCU!=0
    int boundary_xleft = srLeft + cu_pel_x;
    if (boundary_xleft < 0)
    {
        srLeft = -cu_pel_x;
    }
    else
    {
        srLeft = srLeft;
    }
#endif
#if CSS_REF_RANGE_XLCUR!=0
    int boundary_yleft = srTop + cu_pel_y;
    if (boundary_yleft < 0)
    {
        srTop = -cu_pel_y;
    }
    else
    {
        srTop = srTop;
    }
#endif
    srRight = lcu_width - iRelCUPelX - sp_roi_width;
    srBottom = lcu_height - iRelCUPelY - sp_roi_height;
    if ((int)cu_pel_x + srRight + sp_roi_width > i_pic_width)
    {
        srRight = i_pic_width % lcu_width - iRelCUPelX - sp_roi_width;
    }
    if ((int)cu_pel_y + srBottom + sp_roi_height > i_pic_height)
    {
        srBottom = i_pic_height % lcu_height - iRelCUPelY - sp_roi_height;
    }
    //end search range
    //Do integer search    
    u32  uiSad;
    u32  uiSadBestCand[SP_CHROMA_REFINE_CANDS];
    SP_CU_SIZE_STRING      cSMPositionCand[SP_CHROMA_REFINE_CANDS];
    u32  uiSadBest = 0xFFFFFFFFU;
    u32  uiTempSadBest = 0;
    const int boundY = -1;
    for (int iCand = 0; iCand < SP_CHROMA_REFINE_CANDS; iCand++)
    {
        uiSadBestCand[iCand] = 0xFFFFFFFFU;
        cSMPositionCand[iCand].sm.x = 0;
        cSMPositionCand[iCand].sm.y = 0;
    }

    //int lowY;
#if SCC_CROSSINFO_ULTILIZE
    for (int i = 0; i < 1; i++)
    {
        int x = p_cur_sp_info->ibc_mv[0];
        int y = p_cur_sp_info->ibc_mv[1];

        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
#if ExtStrCSS||RBU_LIMIT_CSS
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = x_start_in_pic + tmp[k][0];
                current_y1 = y_start_in_pic + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1 + x;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrCSS
            if (width * height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_CSS	
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = x_start_in_pic;
                int end_y_in_pic = y_start_in_pic + height - 1;
                nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);

                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                    continue;
                }
            }
#endif
        }
#endif
        uiSad = 1;
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (cu_pel_y + y + r) * m_stride_rec[0] + cu_pel_x + x;
            int cu_org_pos = (cu_pel_y + r) * m_stride_org[0] + cu_pel_x;

            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
        sp_postion_cand_update(uiSad, x, y, uiSadBestCand, cSMPositionCand);
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }
#endif

    // svp search
    for (int i = 0; i < p_cur_sp_info->n_cand_num; i++)
    {
        int x = p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_X];
        int y = p_cur_sp_info->n_cand[p_cur_sp_info->n_cand_num - 1 - i].mv[0][MV_Y];
        if (cu_pel_y + y < 0 || cu_pel_y + sp_roi_height + y >= m_pic_height || cu_pel_x + x < 0 || cu_pel_x + sp_roi_width + x >= m_pic_width)
        {
            continue;
        }
        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
#if ExtStrCSS||RBU_LIMIT_CSS
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = x_start_in_pic + tmp[k][0];
                current_y1 = y_start_in_pic + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1 + x;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrCSS
            if (width * height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_CSS	
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = x_start_in_pic;
                int end_y_in_pic = y_start_in_pic + height - 1;
                nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);
                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                    continue;
                }
            }
#endif
        }
#endif
        uiSad = (u32)get_tb_bits(p_cur_sp_info->n_cand_num - 1 - i, p_cur_sp_info->n_cand_num);
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (cu_pel_y + y + r) * m_stride_rec[0] + cu_pel_x + x;
            int cu_org_pos = (cu_pel_y + r) * m_stride_org[0] + cu_pel_x;
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
        sp_postion_cand_update(uiSad, x, y, uiSadBestCand, cSMPositionCand);
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }

    // visit mv
    for (int mn = 0; mn < 1; mn++)
    {
        uiSad = 0xFFFFFFFFU;
        int x = p_cur_sp_info->sp_transmit_x;
        int y = p_cur_sp_info->sp_transmit_y;
        if (cu_pel_y + y < 0 || cu_pel_y + sp_roi_height + y >= m_pic_height || cu_pel_x + x < 0 || cu_pel_x + sp_roi_width + x >= m_pic_width)
        {
            continue;
        }
        if (!is_ref_region_valid(p_cur_sp_info, cu_pel_x + x, cu_pel_y + y, sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y))
        {
            continue;
        }
#if ExtStrCSS||RBU_LIMIT_CSS
        {
            int is_out_of_raw_range_flag = 0;
            int tmp[4][2] = { 0,0,width - 1,0 , width - 1,height - 1 ,0,height - 1 };
            for (int k = 0; k < 4; k++) {
                int current_x1;
                int current_y1;
                current_x1 = x_start_in_pic + tmp[k][0];
                current_y1 = y_start_in_pic + tmp[k][1];
                int ref_start_x_in_pic1 = current_x1;
                int ref_start_y_in_pic1 = current_y1 + y;
                if (!is_ref_pix_in_raw_range(current_x1, current_y1, width, height, ref_start_x_in_pic1, ref_start_y_in_pic1))
                {
                    is_out_of_raw_range_flag = 1;
                    break;
                }
            }
#if ExtStrCSS
            if (width * height < ExtStrLenMin)
            {
                if (is_out_of_raw_range_flag)
                {
                    continue;
                }
            }
#endif
#if RBU_LIMIT_CSS	
            if (is_out_of_raw_range_flag)
            {
                int nbu = 0;
                int sv[2] = { x,y };
                int end_x_in_pic = x_start_in_pic;
                int end_y_in_pic = y_start_in_pic + height - 1;
                nbu = calc_recString_ext_rect_coord(width, x_start_in_pic, y_start_in_pic, x_start_in_pic, y_start_in_pic, end_x_in_pic, end_y_in_pic, 0, width * height, sv);

                if (nbu * RCNC_Parameter > LIMIT_ON_STRPE_NUM * width * height) {
                    continue;
                }
            }
#endif
        }
#endif
        uiSad = (u32)get_sp_offset_bit_cnt(x, y);
        for (int r = 0; r < sp_roi_height; )
        {
            int cu_ref_pos = (r + cu_pel_y) * m_stride_rec[0] + x + cu_pel_x;
            int cu_org_pos = (r + cu_pel_y) * m_stride_org[0] + cu_pel_x;
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < sp_roi_width; i++)
                {
                    uiSad += abs(ref_pic[cu_ref_pos + i] - org_pic[cu_org_pos + i]);
                }
                cu_ref_pos += m_stride_rec[0];
                cu_org_pos += m_stride_org[0];
            }
            if (uiSad > uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
            {
                break;
            }
            r += 4;
        }
        sp_postion_cand_update(uiSad, x, y, uiSadBestCand, cSMPositionCand);
        uiTempSadBest = uiSadBestCand[0];
        if (uiSadBestCand[0] <= 3)
        {
            copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
            uiSadBest = uiSadBestCand[0];
            goto rec;
        }
    }

    copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[0]));
    best_sp.x = best_coding_info.sm.x;
    best_sp.y = best_coding_info.sm.y;
    uiSadBest = uiSadBestCand[0];
    i_best_cand_idx = 0;
    if ((uiSadBest - (u32)get_sp_offset_bit_cnt(best_sp.x, best_sp.y) <= 32))
    {
        //chroma refine    
        if (p_cur_sp_info->tree_status != TREE_L)
        {
            i_best_cand_idx = sp_postion_chroma_refine(sp_roi_width, sp_roi_height, cu_pel_x, cu_pel_y, uiSadBestCand, cSMPositionCand);
        }
        else
        {
            assert(p_cur_sp_info->tree_status == TREE_L);
        }
        copy_cu_size_str_info(&best_coding_info, &(cSMPositionCand[i_best_cand_idx]));
        uiSadBest = uiSadBestCand[i_best_cand_idx];
        goto rec;
    }
rec:
    sp.x = best_coding_info.sm.x;
    sp.y = best_coding_info.sm.y;
}
#endif

void string_prediction::sp_postion_cand_update(u32  uiSad, int x, int y, u32* uiSadBestCand, SP_CU_SIZE_STRING* cSMPositionCand)
{
    int j = SP_CHROMA_REFINE_CANDS - 1;
    if (uiSad < uiSadBestCand[SP_CHROMA_REFINE_CANDS - 1])
    {
        for (int t = SP_CHROMA_REFINE_CANDS - 1; t >= 0; t--)
        {
            if (uiSad < uiSadBestCand[t])
                j = t;
        }
        for (int k = SP_CHROMA_REFINE_CANDS - 1; k > j; k--)
        {
            uiSadBestCand[k] = uiSadBestCand[k - 1];
            cSMPositionCand[k].sm.x = cSMPositionCand[k - 1].sm.x;
            cSMPositionCand[k].sm.y = cSMPositionCand[k - 1].sm.y;
        }
        uiSadBestCand[j] = uiSad;
        cSMPositionCand[j].sm.x = x;
        cSMPositionCand[j].sm.y = y;
    }
}

int string_prediction::sp_postion_chroma_refine(
    int         sp_roi_width,
    int         sp_roi_height,
    int         cu_pel_x,
    int         cu_pel_y,
    u32* uiSadBestCand,
    SP_CU_SIZE_STRING*     cSMPositionCand
)
{
    int iBestCandIdx = 0;
    u32 uiSadBest = 0xFFFFFFFFU;
    pel* ref_pic = m_p_pel_rec[0];
    pel* org_pic = m_p_pel_org[0];
    u8 scale_x = (input->chroma_format == 3) ? 0 : 1;
    u8 scale_y = (input->chroma_format >= 2) ? 0 : 1;
    for (int iCand = 0; iCand < SP_CHROMA_REFINE_CANDS; iCand++)
    {
        u32 uiTempSad = uiSadBestCand[iCand];
        SP_CU_SIZE_STRING uiTempSMPosition = cSMPositionCand[iCand];
        if (uiTempSad == 0xFFFFFFFFU)
        {
            break;
        }
        for (u8 ch = 1; ch < 2; ch++)
        {
            int orgPos = (cu_pel_y >> scale_y) * m_stride_org[ch] + (cu_pel_x >> scale_x);
            int ref_pos = ((cu_pel_y + (uiTempSMPosition.sm.y)) >> scale_y) * m_stride_rec[ch] + ((cu_pel_x + (uiTempSMPosition.sm.x)) >> scale_x);
            org_pic = m_p_pel_org[ch];
            ref_pic = m_p_pel_rec[ch];
            for (int row = 0; row < (sp_roi_height >> scale_y); row++)
            {
                for (int col = 0; col < (sp_roi_width >> scale_x); col++)
                {
                    uiTempSad += abs(ref_pic[ref_pos + col] - org_pic[orgPos + col]);
                }
                orgPos += m_stride_org[ch];
                ref_pos += m_stride_rec[ch];
            }
        }
        if (uiTempSad < uiSadBest)
        {
            uiSadBest = uiTempSad;
            iBestCandIdx = iCand;
        }
    }
    return iBestCandIdx;
}
#endif
