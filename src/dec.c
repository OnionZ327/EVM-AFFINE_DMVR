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

#include "dec_def.h"
#include "dec_eco.h"
#include "com_df.h"
#include "com_sao.h"
#include "dec_DecAdaptiveLoopFilter.h"
#if ESAO || CCSAO
#include "com_esao.h"
#endif 

/* convert DEC into DEC_CTX */
#define DEC_ID_TO_CTX_R(id, ctx) \
    com_assert_r((id)); \
    (ctx) = (DEC_CTX *)id; \
    com_assert_r((ctx)->magic == DEC_MAGIC_CODE);

/* convert DEC into DEC_CTX with return value if assert on */
#define DEC_ID_TO_CTX_RV(id, ctx, ret) \
    com_assert_rv((id), (ret)); \
    (ctx) = (DEC_CTX *)id; \
    com_assert_rv((ctx)->magic == DEC_MAGIC_CODE, (ret));

static DEC_CTX * ctx_alloc(void)
{
    DEC_CTX * ctx;
    ctx = (DEC_CTX*)com_malloc_fast(sizeof(DEC_CTX));
    com_assert_rv(ctx != NULL, NULL);
    com_mset_x64a(ctx, 0, sizeof(DEC_CTX));
    /* set default value */
    ctx->dtr = 0;
    ctx->pic_cnt = 0;
    return ctx;
}

static void ctx_free(DEC_CTX * ctx)
{
    com_mfree_fast(ctx);
}

static DEC_CORE * core_alloc(void)
{
    DEC_CORE * core;
    core = (DEC_CORE*)com_malloc_fast(sizeof(DEC_CORE));
    com_assert_rv(core, NULL);
    com_mset_x64a(core, 0, sizeof(DEC_CORE));
    return core;
}

static void core_free(DEC_CORE * core)
{
    com_mfree_fast(core);
}

#if AWP_ENH
void dec_delete_awp_bufs(DEC_CTX* ctx)
{
    const int wNum = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    const int hNum = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    const int awpNum = AWP_MODE_NUM;
    com_free_4d_Buf(ctx->awp_weight_tpl, wNum, hNum, awpNum);
    ctx->awp_init = FALSE;
    return;
}
static void dec_init_awp_bufs(DEC_CTX* ctx)
{
    const int w_num = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    const int h_num = MAX_AWP_SIZE_LOG2 - MIN_AWP_SIZE_LOG2 + 1;
    const int awp_num = AWP_MODE_NUM;
    const int size_num = AWP_TPL_SIZE * 2;
    com_malloc_4d_Buf(&ctx->awp_weight_tpl, w_num, h_num, awp_num, size_num);
    for (int w = 0; w < w_num; w++)
    {
        for (int h = 0; h < h_num; h++)
        {
            for (int idx = 0; idx < AWP_MODE_NUM; idx++)
            {
                com_derive_awp_tpl_weights(ctx->awp_weight_tpl, w, h, idx);
            }
        }
    }
    ctx->awp_init = TRUE;
    return;
}
#endif

static void sequence_deinit(DEC_CTX * ctx)
{
    if( ctx->map.map_scu )
    {
        com_mfree( ctx->map.map_scu );
        ctx->map.map_scu = NULL;
    }
    if( ctx->map.map_split )
    {
        com_mfree( ctx->map.map_split );
        ctx->map.map_split = NULL;
    }
    if( ctx->map.map_ipm )
    {
        com_mfree( ctx->map.map_ipm );
        ctx->map.map_ipm = NULL;
    }
    if( ctx->map.map_cu_mode )
    {
        com_mfree( ctx->map.map_cu_mode );
        ctx->map.map_cu_mode = NULL;
    }
#if TB_SPLIT_EXT
    if( ctx->map.map_pb_tb_part )
    {
        com_mfree( ctx->map.map_pb_tb_part );
        ctx->map.map_pb_tb_part = NULL;
    }
#endif
    if( ctx->map.map_depth )
    {
        com_mfree( ctx->map.map_depth );
        ctx->map.map_depth = NULL;
    }
    if( ctx->map.map_patch_idx )
    {
        com_mfree( ctx->map.map_patch_idx );
        ctx->map.map_patch_idx = NULL;
    }
#if USE_SP
    if (ctx->map.map_usp)
    {
        com_mfree(ctx->map.map_usp);
        ctx->map.map_usp = NULL;
    }
#endif
    delete_edge_filter_avs2(ctx->edge_filter, ctx->info.pic_height);

    if( ctx->pic_sao )
    {
        com_picbuf_free( ctx->pic_sao );
        ctx->pic_sao = NULL;
    }

    com_free_3d_sao_stat_data(&ctx->sao_stat_data,
                            ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                        ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_free_2d_sao_param(&ctx->sao_blk_params,
                             ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                         ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
    com_free_2d_sao_param(&ctx->rec_sao_blk_params,
                             ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                         ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
#if ESAO
    if (ctx->pic_esao)
    {
        com_picbuf_free(ctx->pic_esao);
        ctx->pic_esao = NULL;
    }
#if ESAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_esao_params)
    {
        com_mfree(ctx->info.pic_header.pic_esao_params[Y_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params[U_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params[V_C].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_esao_params);
        ctx->info.pic_header.pic_esao_params = NULL;
    }
#else
    com_mfree(ctx->pic_esao_params[Y_C].lcu_flag);
    com_mfree(ctx->pic_esao_params[U_C].lcu_flag);
    com_mfree(ctx->pic_esao_params[V_C].lcu_flag);
#endif
#endif
#if CCSAO
#if CCSAO_ENHANCEMENT
    if (ctx->pic_ccsao[0])
    {
        com_picbuf_free(ctx->pic_ccsao[0]);
        ctx->pic_ccsao[0] = NULL;
    }
    if (ctx->pic_ccsao[1])
    {
        com_picbuf_free(ctx->pic_ccsao[1]);
        ctx->pic_ccsao[1] = NULL;
    }
#else
    if (ctx->pic_ccsao)
    {
        com_picbuf_free(ctx->pic_ccsao);
        ctx->pic_ccsao = NULL;
    }
#endif
#if CCSAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_ccsao_params)
    {
        com_mfree(ctx->info.pic_header.pic_ccsao_params[U_C - 1].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_ccsao_params[V_C - 1].lcu_flag);
        com_mfree(ctx->info.pic_header.pic_ccsao_params);
        ctx->info.pic_header.pic_ccsao_params = NULL;
    }
#else
    com_mfree(ctx->pic_ccsao_params[U_C-1].lcu_flag);
    com_mfree(ctx->pic_ccsao_params[V_C-1].lcu_flag);
#endif
#endif
    if (ctx->pic_alf_Rec)
    {
        com_picbuf_free(ctx->pic_alf_Rec);
        ctx->pic_alf_Rec = NULL;
    }
    if (ctx->pic_alf_Dec)
    {
        com_picbuf_free(ctx->pic_alf_Dec);
        ctx->pic_alf_Dec = NULL;
    }
    release_alf_global_buffer(ctx);
    ctx->info.pic_header.pic_alf_on = NULL;
    ctx->info.pic_header.alf_picture_param = NULL;
    com_picman_deinit(&ctx->dpm);

#if NN_FILTER
    for (int comp = 0; comp < N_C; comp++)
    {
        com_mfree(ctx->info.pic_header.nnlf_lcu_enable_flag[comp]);
        com_mfree(ctx->info.pic_header.nnlf_lcu_set_index[comp]);
        ctx->info.pic_header.nnlf_lcu_enable_flag[comp] = NULL;
        ctx->info.pic_header.nnlf_lcu_set_index[comp] = NULL;
    }
#endif

#if PATCH
    if (ctx->patch)
    {
        com_mfree(ctx->patch);
        ctx->patch = NULL;
    }
    if (ctx->map.map_scu_temp)
    {
        com_mfree(ctx->map.map_scu_temp);
        ctx->map.map_scu_temp = NULL;
    }
    if (ctx->map.map_cu_mode_temp)
    {
        com_mfree(ctx->map.map_cu_mode_temp);
        ctx->map.map_cu_mode_temp = NULL;
    }
    if (ctx->map.map_mv_temp)
    {
        com_mfree(ctx->map.map_mv_temp);
        ctx->map.map_mv_temp = NULL;
    }
    if (ctx->map.map_refi_temp)
    {
        com_mfree(ctx->map.map_refi_temp);
        ctx->map.map_refi_temp = NULL;
    }
#if USE_SP
    if (ctx->map.map_usp_temp)
    {
        com_mfree(ctx->map.map_usp_temp);
        ctx->map.map_usp_temp = NULL;
    }
#endif
#endif
#if AWP_ENH
    if (ctx->awp_weight_tpl)
    {
        dec_delete_awp_bufs(ctx);
    }
#endif
}

static int sequence_init(DEC_CTX * ctx, COM_SQH * sqh)
{
    int size;
    int ret;

    ctx->info.bit_depth_internal = (sqh->encoding_precision == 2) ? 10 : 8;
    assert(sqh->sample_precision == 1 || sqh->sample_precision == 2);
    ctx->info.bit_depth_input = (sqh->sample_precision == 1) ? 8 : 10;
    ctx->info.qp_offset_bit_depth = (8 * (ctx->info.bit_depth_internal - 8));

    sequence_deinit(ctx);
    ctx->info.pic_width  = ((sqh->horizontal_size + MINI_SIZE - 1) / MINI_SIZE) * MINI_SIZE;
    ctx->info.pic_height = ((sqh->vertical_size   + MINI_SIZE - 1) / MINI_SIZE) * MINI_SIZE;
    ctx->info.max_cuwh = 1 << sqh->log2_max_cu_width_height;
    ctx->info.log2_max_cuwh = CONV_LOG2(ctx->info.max_cuwh);

    size = ctx->info.max_cuwh;
    ctx->info.pic_width_in_lcu = (ctx->info.pic_width + (size - 1)) / size;
    ctx->info.pic_height_in_lcu = (ctx->info.pic_height + (size - 1)) / size;
    ctx->info.f_lcu = ctx->info.pic_width_in_lcu * ctx->info.pic_height_in_lcu;
    ctx->info.pic_width_in_scu = (ctx->info.pic_width + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->info.pic_height_in_scu = (ctx->info.pic_height + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->info.f_scu = ctx->info.pic_width_in_scu * ctx->info.pic_height_in_scu;
#if ASP
    ctx->info.skip_me_asp = FALSE;
    ctx->info.skip_umve_asp = FALSE;
#endif
    /* alloc SCU map */
    if (ctx->map.map_scu == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_scu = (u32 *)com_malloc(size);
        com_assert_gv(ctx->map.map_scu, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_scu, 0, size);
    }
    /* alloc cu mode SCU map */
    if (ctx->map.map_cu_mode == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_cu_mode = (u32 *)com_malloc(size);
        com_assert_gv(ctx->map.map_cu_mode, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_cu_mode, 0, size);
    }
    /* alloc map for CU split flag */
    if (ctx->map.map_split == NULL)
    {
#if CTU_256
        size = sizeof(s8) * ctx->info.f_lcu * MAX_CU_DEPTH2 * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU;
#else
        size = sizeof(s8) * ctx->info.f_lcu * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU;
#endif
        ctx->map.map_split = com_malloc(size);
        com_assert_gv(ctx->map.map_split, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_split, 0, size);
    }
    /* alloc map for intra prediction mode */
    if (ctx->map.map_ipm == NULL)
    {
        size = sizeof(s8) * ctx->info.f_scu;
        ctx->map.map_ipm = (s8 *)com_malloc(size);
        com_assert_gv(ctx->map.map_ipm, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_ipm, -1, size);
    }
    if (ctx->map.map_patch_idx == NULL)
    {
        size = sizeof(s8)* ctx->info.f_scu;
        ctx->map.map_patch_idx = (s8 *)com_malloc(size);
        com_assert_gv(ctx->map.map_patch_idx, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_patch_idx, -1, size);
    }
#if TB_SPLIT_EXT
    /* alloc map for pb and tb partition size (for luma and chroma) */
    if (ctx->map.map_pb_tb_part == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_pb_tb_part = (u32 *)com_malloc(size);
        com_assert_gv(ctx->map.map_pb_tb_part, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_pb_tb_part, 0xffffffff, size);
    }
#endif
    /* alloc map for intra depth */
    if( ctx->map.map_depth == NULL )
    {
        size = sizeof( s8 ) * ctx->info.f_scu;
        ctx->map.map_depth = com_malloc_fast( size );
        com_assert_gv( ctx->map.map_depth, ret, COM_ERR_OUT_OF_MEMORY, ERR );
        com_mset( ctx->map.map_depth, -1, size );
    }
#if USE_SP
    if (ctx->map.map_usp == NULL)
    {
        size = sizeof(u8) * ctx->info.f_scu;
        ctx->map.map_usp = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_usp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->map.map_usp, 0, size);
    }
#endif

    ctx->pa.width = ctx->info.pic_width;
    ctx->pa.height = ctx->info.pic_height;
    ctx->pa.pad_l = PIC_PAD_SIZE_L;
    ctx->pa.pad_c = PIC_PAD_SIZE_C;
    ret = com_picman_init(&ctx->dpm, sqh->max_dpb_size, MAX_NUM_REF_PICS, &ctx->pa);
    com_assert_g(COM_SUCCEEDED(ret), ERR);

    create_edge_filter_avs2(ctx->info.pic_width, ctx->info.pic_height, ctx->edge_filter);

    if (ctx->pic_sao == NULL)
    {
        ctx->pic_sao = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_sao != NULL, ret);
    }

    com_malloc_3d_sao_stat_data(&(ctx->sao_stat_data),
                              ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                          ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C, NUM_SAO_NEW_TYPES);
    com_malloc_2d_sao_param(&(ctx->sao_blk_params),
                               ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                           ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_malloc_2d_sao_param(&ctx->rec_sao_blk_params,
                               ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
                                           ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
#if ESAO
    if (ctx->pic_esao == NULL)
    {
        ctx->pic_esao = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_esao != NULL, ret);
    }
#if ESAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_esao_params == NULL)
    {
        ctx->info.pic_header.pic_esao_params = (ESAO_BLK_PARAM*)malloc(N_C * sizeof(ESAO_BLK_PARAM));
    }
    ctx->info.pic_header.pic_esao_params[Y_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->info.pic_header.pic_esao_params[U_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->info.pic_header.pic_esao_params[V_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#else
    ctx->pic_esao_params[Y_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->pic_esao_params[U_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->pic_esao_params[V_C].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#endif
#endif
#if CCSAO
#if CCSAO_ENHANCEMENT
    if (ctx->pic_ccsao[0] == NULL)
    {
        ctx->pic_ccsao[0] = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_ccsao[0] != NULL, ret);
    }
    if (ctx->pic_ccsao[1] == NULL)
    {
        ctx->pic_ccsao[1] = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_ccsao[1] != NULL, ret);
    }
#else
    if (ctx->pic_ccsao == NULL)
    {
        ctx->pic_ccsao = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_ccsao != NULL, ret);
    }
#endif
#if CCSAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_ccsao_params==NULL)
    {
        ctx->info.pic_header.pic_ccsao_params = (CCSAO_BLK_PARAM*)malloc((N_C - 1) * sizeof(CCSAO_BLK_PARAM));
    }
    ctx->info.pic_header.pic_ccsao_params[U_C-1].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->info.pic_header.pic_ccsao_params[V_C-1].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#else
    ctx->pic_ccsao_params[U_C-1].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
    ctx->pic_ccsao_params[V_C-1].lcu_flag = (int *)malloc(ctx->info.f_lcu * sizeof(int));
#endif
#endif
    ctx->info.pic_header.tool_alf_on = ctx->info.sqh.adaptive_leveling_filter_enable_flag;
#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
    ctx->info.pic_header.tool_alf_enhance_on = ctx->info.sqh.adaptive_leveling_filter_enhance_flag;
#endif 
    if (ctx->pic_alf_Rec == NULL)
    {
        ctx->pic_alf_Rec = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_alf_Rec != NULL, ret);
    }
    if (ctx->pic_alf_Dec == NULL)
    {
        ctx->pic_alf_Dec = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_alf_Dec != NULL, ret);
    }
    create_alf_global_buffer(ctx);
    ctx->info.pic_header.pic_alf_on = ctx->pic_alf_on;
    ctx->info.pic_header.alf_picture_param = ctx->dec_alf->alf_picture_param;

#if NN_FILTER
    for (int comp = 0; comp < N_C; comp++)
    {
        if (ctx->info.pic_header.nnlf_lcu_enable_flag[comp] == NULL)
        {
            ctx->info.pic_header.nnlf_lcu_enable_flag[comp] = (u8*)com_malloc(ctx->info.f_scu * sizeof(u8));
        }
        if (ctx->info.pic_header.nnlf_lcu_set_index[comp] == NULL)
        {
            ctx->info.pic_header.nnlf_lcu_set_index[comp] = (u8*)com_malloc(ctx->info.f_scu * sizeof(u8));
        }
    }
#endif

    /*initialize patch info */
#if PATCH
    if (ctx->patch == NULL)
    {
        ctx->patch = (PATCH_INFO*)com_malloc_fast(sizeof(PATCH_INFO));
    }
    ctx->patch->stable = sqh->patch_stable;
    ctx->patch->cross_patch_loop_filter = sqh->cross_patch_loop_filter;
    ctx->patch->ref_colocated = sqh->patch_ref_colocated;
    if (ctx->patch->stable)
    {
        ctx->patch->uniform = sqh->patch_uniform;
        if (ctx->patch->uniform)
        {
            ctx->patch->width = sqh->patch_width_minus1 + 1;
            ctx->patch->height = sqh->patch_height_minus1 + 1;
            ctx->patch->columns = ctx->info.pic_width_in_lcu / ctx->patch->width;
            ctx->patch->rows = ctx->info.pic_height_in_lcu / ctx->patch->height;
            /*set column_width and row_height*/
            for (int i = 0; i < ctx->patch->columns; i++)
            {
                ctx->patch_column_width[i] = ctx->patch->width;
            }
            if (ctx->info.pic_width_in_lcu%ctx->patch->width != 0)
            {
                if (ctx->patch->columns == 0)
                {
                    ctx->patch_column_width[ctx->patch->columns] = ctx->info.pic_width_in_lcu - ctx->patch->width*ctx->patch->columns;
                    ctx->patch->columns++;
                }
                else
                {
                    ctx->patch_column_width[ctx->patch->columns - 1] = ctx->patch_column_width[ctx->patch->columns - 1] + ctx->info.pic_width_in_lcu - ctx->patch->width*ctx->patch->columns;
                }
            }
            for (int i = 0; i < ctx->patch->rows; i++)
            {
                ctx->patch_row_height[i] = ctx->patch->height;
            }
            if (ctx->info.pic_height_in_lcu%ctx->patch->height != 0)
            {
                if (ctx->patch->rows == 0)
                {
                    ctx->patch_row_height[ctx->patch->rows] = ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
                    ctx->patch->rows++;
                }
                else
                {
#if PATCH_M4839
                    ctx->patch_row_height[ctx->patch->rows] = ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
                    ctx->patch->rows++;
#else
                    ctx->patch_row_height[ctx->patch->rows - 1] = ctx->patch_row_height[ctx->patch->rows - 1] + ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
#endif
                }
            }
            /*pointer to the data*/
            ctx->patch->width_in_lcu = ctx->patch_column_width;
            ctx->patch->height_in_lcu = ctx->patch_row_height;
        }
    }
#if BUGFIX_386
    assert(ctx->patch->width >= min(2, ctx->info.pic_width_in_lcu));
#endif

    /* alloc SCU map temp */
    if (ctx->map.map_scu_temp == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_scu_temp = (u32*)com_malloc(size);
        com_assert_gv(ctx->map.map_scu_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_scu_temp, 0, size);
    }
    /* alloc cu mode SCU map temp */
    if (ctx->map.map_cu_mode_temp == NULL)
    {
        size = sizeof(u32) * ctx->info.f_scu;
        ctx->map.map_cu_mode_temp = (u32*)com_malloc(size);
        com_assert_gv(ctx->map.map_cu_mode_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_cu_mode_temp, 0, size);
    }

    /* alloc mv map temp */
    if (ctx->map.map_mv_temp == NULL)
    {
        size = sizeof(s16) * ctx->info.f_scu * REFP_NUM * MV_D;
        ctx->map.map_mv_temp = com_malloc(size);
        com_assert_gv(ctx->map.map_mv_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_mv_temp, 0, size);
    }
    /* alloc refi map temp */
    if (ctx->map.map_refi_temp == NULL)
    {
        size = sizeof(s8) * ctx->info.f_scu * REFP_NUM;
        ctx->map.map_refi_temp = com_malloc(size);
        com_assert_gv(ctx->map.map_refi_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset_x64a(ctx->map.map_refi_temp, -1, size);
    }

#if USE_SP
    /* alloc usp map temp */
    if (ctx->map.map_usp_temp == NULL)
    {
        size = sizeof(u8) * ctx->info.f_scu;
        ctx->map.map_usp_temp = com_malloc(size);
        com_assert_gv(ctx->map.map_usp_temp, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->map.map_usp_temp, 0, size);
    }
#endif
#endif

#if AWP_ENH
    if (ctx->awp_weight_tpl == NULL)
    {
        dec_init_awp_bufs(ctx);
    }
#endif

    return COM_OK;
ERR:
    sequence_deinit(ctx);
    ctx->init_flag = 0;
    return ret;
}

static int slice_init(DEC_CTX * ctx, DEC_CORE * core, COM_PIC_HEADER * pic_header)
{
    core->lcu_num = 0;
    core->x_lcu = 0;
    core->y_lcu = 0;
    core->x_pel = 0;
    core->y_pel = 0;

    ctx->dtr_prev_low = pic_header->dtr;
    ctx->dtr = pic_header->dtr;
    ctx->ptr = pic_header->dtr; /* PTR */

    core->cnt_hmvp_cands = 0;
    com_mset_x64a(core->motion_cands, 0, sizeof(COM_MOTION)*ALLOWED_HMVP_NUM);
#if BGC
    com_mset_x64a(core->bgc_flag_cands, 0, sizeof(s8)*ALLOWED_HMVP_NUM);
    com_mset_x64a(core->bgc_idx_cands, 0, sizeof(s8)*ALLOWED_HMVP_NUM);
#endif

#if IBC_BVP
    core->cnt_hbvp_cands = 0;
    com_mset_x64a(core->block_motion_cands, 0, sizeof(COM_BLOCK_MOTION)*ALLOWED_HBVP_NUM);
    core->hbvp_empty_flag = 0;
#endif
#if HACD
    com_mset_x64a(core->history_affine_mv, 0, sizeof(COM_HISTORY_AFFINE_MV*) * ALLOWED_HAP_NUM);
#endif
#if USE_SP
    core->n_offset_num = 0;
    com_mset_x64a(core->n_recent_offset, 0, sizeof(COM_MOTION)*SP_RECENT_CANDS);
#endif
    core->n_pv_num = 0;
    core->LcuRx0 = 0;
    core->LcuRy0 = 0;
    com_mset_x64a(core->n_recent_pv, -1, sizeof(COM_MOTION)*MAX_SRB_PRED_SIZE);
    /* clear maps */
    com_mset_x64a(ctx->map.map_scu, 0, sizeof(u32) * ctx->info.f_scu);
    com_mset_x64a(ctx->map.map_cu_mode, 0, sizeof(u32) * ctx->info.f_scu);
#if USE_SP
    com_mset_x64a(ctx->map.map_usp, 0, sizeof(u8) * ctx->info.f_scu);
#endif
#if LIBVC_ON
    if (ctx->info.pic_header.slice_type == SLICE_I || (ctx->info.sqh.library_picture_enable_flag && ctx->info.pic_header.is_RLpic_flag))
#else
    if (ctx->info.pic_header.slice_type == SLICE_I)
#endif
    {
        ctx->last_intra_ptr = ctx->ptr;
    }

    return COM_OK;
}

static void make_stat(DEC_CTX * ctx, int btype, DEC_STAT * stat)
{
    int i, j;
    stat->read = 0;
    stat->ctype = btype;
    stat->stype = 0;
    stat->fnum = -1;
#if LIBVC_ON
    stat->is_RLpic_flag = ctx->info.pic_header.is_RLpic_flag;
#endif
    if (ctx)
    {
        stat->read = COM_BSR_GET_READ_BYTE(&ctx->bs);
        if (btype == COM_CT_SLICE)
        {
            stat->fnum = ctx->pic_cnt;
            stat->stype = ctx->info.pic_header.slice_type;
            /* increase decoded picture count */
            ctx->pic_cnt++;
            stat->poc = ctx->ptr;
            for (i = 0; i < 2; i++)
            {
                stat->refpic_num[i] = ctx->dpm.num_refp[i];
                for (j = 0; j < stat->refpic_num[i]; j++)
                {
#if LIBVC_ON
                    stat->refpic[i][j] = ctx->refp[j][i].pic->ptr;
#else
                    stat->refpic[i][j] = ctx->refp[j][i].ptr;
#endif
                }
            }
        }
        else if (btype == COM_CT_PICTURE)
        {
            stat->fnum = -1;
            stat->stype = ctx->info.pic_header.slice_type;
            stat->poc = ctx->info.pic_header.dtr;
            for (i = 0; i < 2; i++)
            {
                stat->refpic_num[i] = ctx->dpm.num_refp[i];
                for (j = 0; j < stat->refpic_num[i]; j++)
                {
#if LIBVC_ON
                    stat->refpic[i][j] = ctx->refp[j][i].pic->ptr;
#else
                    stat->refpic[i][j] = ctx->refp[j][i].ptr;
#endif
                }
            }
        }
    }

#if PRINT_SQH_PARAM_DEC
#if PHASE_2_PROFILE
    stat->profile_id = ctx->info.sqh.profile_id;
#endif
#if CFG_LEVEL_ID
    stat->level_id = ctx->info.sqh.level_id; 
#endif
    stat->internal_bit_depth = ctx->info.sqh.encoding_precision == 2 ? 10 : 8;;
#if ENHANCE_TSPCM
    stat->intra_tools = (ctx->info.sqh.tscpm_enable_flag << 0) + (ctx->info.sqh.enhance_tscpm_enable_flag << 1) +
        (ctx->info.sqh.ipf_enable_flag << 2) + (ctx->info.sqh.dt_intra_enable_flag << 3) + (ctx->info.sqh.ipcm_enable_flag << 4);
#if MIPF
    stat->intra_tools += (ctx->info.sqh.mipf_enable_flag << 5);
#endif
#else
    stat->intra_tools = (ctx->info.sqh.tscpm_enable_flag << 0) + (ctx->info.sqh.ipf_enable_flag << 1) + (ctx->info.sqh.dt_intra_enable_flag << 2) + (ctx->info.sqh.ipcm_enable_flag << 3);
#if MIPF
    stat->intra_tools += (ctx->info.sqh.mipf_enable_flag << 4);
#endif
#endif

#if PMC
    stat->intra_tools += (ctx->info.sqh.pmc_enable_flag << 6);
#endif

#if IPF_CHROMA
    stat->intra_tools += (ctx->info.sqh.chroma_ipf_enable_flag << 7);
#endif
#if IIP
    stat->intra_tools += (ctx->info.sqh.iip_enable_flag << 8);
#endif
#if SAWP
    stat->intra_tools += (ctx->info.sqh.sawp_enable_flag << 9);
#endif // SAWP
#if CCNPM
    stat->intra_tools += (ctx->info.sqh.ccnpm_enable_flag << 10);
#endif
#if CIBC
    stat->intra_tools += (ctx->info.sqh.cibc_enable_flag << 11);
#endif
    stat->inter_tools = (ctx->info.sqh.affine_enable_flag << 0) + (ctx->info.sqh.amvr_enable_flag << 1) + (ctx->info.sqh.umve_enable_flag << 2) + (ctx->info.sqh.emvr_enable_flag << 3);
    stat->inter_tools+= (ctx->info.sqh.smvd_enable_flag << 4) + (ctx->info.sqh.num_of_hmvp_cand << 10);
#if AWP
    stat->inter_tools += ctx->info.sqh.awp_enable_flag << 5;
#endif
#if INTER_TM
    stat->inter_tools += (ctx->info.sqh.tm_enable_flag > 1 ? 1 : ctx->info.sqh.tm_enable_flag) << 6;
#endif
    stat->trans_tools = (ctx->info.sqh.secondary_transform_enable_flag << 0) + (ctx->info.sqh.position_based_transform_enable_flag << 1);

    stat->filte_tools = (ctx->info.sqh.sample_adaptive_offset_enable_flag << 0) + (ctx->info.sqh.adaptive_leveling_filter_enable_flag << 1);

    stat->scc_tools   = 0;
#if FIMC
    stat->scc_tools  += (ctx->info.sqh.fimc_enable_flag << 0);
#endif
#if IBC_BVP
    stat->scc_tools += (ctx->info.sqh.num_of_hbvp_cand << 1);
#endif
#if PARITY_HIDING
    stat->coef_coding_tools = 0;
    stat->coef_coding_tools += (ctx->info.sqh.ph_enable_flag << 0);
#endif
#endif
}
#if ECCPM
static void get_nbr_yuv(int x, int y, int cu_width, int cu_height, u16 avail_cu, COM_PIC *pic_rec, pel nb[N_C][N_REF][MAX_CU_SIZE * 3],pel nb_template[N_C][N_REF][(MAX_CU_SIZE+ECCPM_TEMP_SIZE) * 3], int scup, u32 *map_scu, int pic_width_in_scu, int h_scu, u8 include_y, int bit_depth)
#else
static void get_nbr_yuv(int x, int y, int cu_width, int cu_height, u16 avail_cu, COM_PIC *pic_rec, pel nb[N_C][N_REF][MAX_CU_SIZE * 3], int scup, u32 *map_scu, int pic_width_in_scu, int h_scu, u8 include_y, int bit_depth)
#endif
{
    int  s_rec;
    pel *rec;
    if (include_y)
    {
        /* Y */
        s_rec = pic_rec->stride_luma;
        rec = pic_rec->y + (y * s_rec) + x;
        com_get_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb, scup, map_scu, pic_width_in_scu, h_scu, bit_depth, Y_C);
    }

    cu_width >>= 1;
    cu_height >>= 1;
    x >>= 1;
    y >>= 1;
    s_rec = pic_rec->stride_chroma;
    /* U */
    rec = pic_rec->u + (y * s_rec) + x;
    com_get_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb, scup, map_scu, pic_width_in_scu, h_scu, bit_depth, U_C);
    /* V */
    rec = pic_rec->v + (y * s_rec) + x;
    com_get_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb, scup, map_scu, pic_width_in_scu, h_scu, bit_depth, V_C);
#if ECCPM
    /* U */
    rec = pic_rec->u + (y * s_rec) + x;
    com_get_template_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb_template, scup, map_scu, pic_width_in_scu, h_scu, bit_depth, U_C);
    /* V */
    rec = pic_rec->v + (y * s_rec) + x;
    com_get_template_nbr(x, y, cu_width, cu_height, rec, s_rec, avail_cu, nb_template, scup, map_scu, pic_width_in_scu, h_scu, bit_depth, V_C);
#endif
}
#if ECCPM
void caculate_eccpm_param_dec(DEC_CTX *ctx, int x, int y, int cu_width, int cu_height, pel* pred, pel* pred_template, int avail_cu, int chroma_type, pel* rec_u, int u_stride, pel(*A)[MAX_CU_DIM], pel *YC, int param_num)
{
    int bit_depth = ctx->info.bit_depth_internal;
    int stride_y = ctx->pic->stride_luma;
    pel *reco_y  = ctx->pic->y + (y * stride_y) + x;
    int stride_uv = ctx->pic->stride_chroma;
    int chroma_x = x >> 1;
    int chroma_y = y >> 1;
    int chroma_cu_width = cu_width >> 1;
    int chroma_cu_height = cu_height >> 1;
    pel* reco_chroma = NULL;
    if (chroma_type == U_C)
    {
        reco_chroma = ctx->pic->u + (y / 2 * stride_uv) + x / 2;
    }
    else
    {
        reco_chroma = ctx->pic->v + (y / 2 * stride_uv) + x / 2;
    }
    pel* reco_chroma_u = ctx->pic->u + (y / 2 * stride_uv) + x / 2;
    //downscale luma pixel
    int is_above = IS_AVAIL(avail_cu, AVAIL_UP2);
    int is_left = IS_AVAIL(avail_cu, AVAIL_LE2);
    long long a0, a1, a2, a3 = 0;
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
	    calculate_eccpm_model(chroma_u_le, chroma_u_up, luma_recon_le, chroma_recon_le, chroma_pred_le, stride_y, stride_uv, w_le, h_le, luma_recon_up, chroma_recon_up, chroma_pred_up, w_up, h_up, &a0, &a1, &a2, &a3, bit_depth, chroma_type, A, YC, param_num);
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
            luma_recon = reco_y - (ECCPM_TEMP_SIZE*2);
            chroma_recon = reco_chroma - ECCPM_TEMP_SIZE;
            chroma_u_recon = reco_chroma_u - ECCPM_TEMP_SIZE;
            chroma_pred = pred_template;
            w = ECCPM_TEMP_SIZE;
            h = chroma_cu_height;
        }
        else if (is_above)
        {
            luma_recon = reco_y - (ECCPM_TEMP_SIZE*2)*stride_y;
            chroma_recon = reco_chroma - ECCPM_TEMP_SIZE*stride_uv;
            chroma_u_recon = reco_chroma_u - ECCPM_TEMP_SIZE*stride_uv;
            chroma_pred = pred_template;
            w = chroma_cu_width;
            h = ECCPM_TEMP_SIZE;
        }
        if ((is_left) || (is_above))
        {
        calculate_eccpm_model(chroma_u_recon, NULL, luma_recon, chroma_recon, chroma_pred, stride_y, stride_uv, w, h, NULL, NULL, NULL, 0, 0, &a0, &a1, &a2, &a3, bit_depth, chroma_type, A, YC, param_num);
        }
    }
    pel* luma_recon = reco_y; 
    pel* chroma_pred = pred;
    pel* dst = pred;
    if((is_left)||(is_above))
    pred_eccpm(luma_recon, stride_y, chroma_pred, dst, chroma_cu_width, chroma_cu_height, a0, a1, a2, a3, bit_depth, rec_u, u_stride, param_num);
}
#endif
static int dec_eco_unit(DEC_CTX * ctx, DEC_CORE * core, int x, int y, int cu_width_log2, int cu_height_log2, int cud)
{
    int ret, cu_width, cu_height;
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int bit_depth = ctx->info.bit_depth_internal;
    static s16 resi[N_C][MAX_CU_DIM];
    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
    mod_info_curr->LcuRx0 = core->LcuRx0;
    mod_info_curr->LcuRy0 = core->LcuRy0;
    mod_info_curr->x_pos = x;
    mod_info_curr->y_pos = y;
    mod_info_curr->x_scu = PEL2SCU(x);
    mod_info_curr->y_scu = PEL2SCU(y);
    mod_info_curr->scup = mod_info_curr->x_scu + mod_info_curr->y_scu * ctx->info.pic_width_in_scu;
    mod_info_curr->cu_width = cu_width;
    mod_info_curr->cu_height= cu_height;
    mod_info_curr->cu_width_log2 = cu_width_log2;
    mod_info_curr->cu_height_log2 = cu_height_log2;
    mod_info_curr->cud = cud;
#if DMVR
    mod_info_curr->dmvr_enable = 0;
#endif
#if IST
    mod_info_curr->slice_type = ctx->info.pic_header.slice_type;
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
#if AWP
    mod_info_curr->awp_flag = 0;
#endif
#if INTER_TM
    mod_info_curr->tm_flag = 0;
#endif
#if SAWP
    mod_info_curr->sawp_flag = 0;
#endif // SAWP

#if SUB_TMVP
    core->sbTmvp_flag = 0;
#endif
#if MVAP
    core->mvap_flag      = 0;
    core->valid_mvap_num = 0;
#endif
#if UNIFIED_HMVP_1
    mod_info_curr->mvap_flag = 0;
    mod_info_curr->sub_tmvp_flag = 0;
#endif
#if ISTS
    mod_info_curr->ph_ists_enable_flag = ctx->info.pic_header.ph_ists_enable_flag;
#endif
#if DEST_PH
    mod_info_curr->ph_dest_enable_flag = ctx->info.pic_header.ph_dest_enable_flag;
#endif
#if TS_INTER
    mod_info_curr->ph_ts_inter_enable_flag = ctx->info.pic_header.ph_ts_inter_enable_flag;
#endif
#if AWP
    mod_info_curr->ph_awp_refine_flag = ctx->info.pic_header.ph_awp_refine_flag;
#endif
#if SMVD
    mod_info_curr->smvd_flag = 0;
#endif
#if ETMVP
    mod_info_curr->etmvp_flag = 0;
#endif
#if CIBC
    mod_info_curr->cibc_flag = 0;
#endif

    COM_TRACE_COUNTER;
    COM_TRACE_STR("ptr: ");
    COM_TRACE_INT(ctx->ptr);
    COM_TRACE_STR("x pos ");
    COM_TRACE_INT(x);
    COM_TRACE_STR("y pos ");
    COM_TRACE_INT(y);
    COM_TRACE_STR("width ");
    COM_TRACE_INT(cu_width);
    COM_TRACE_STR("height ");
    COM_TRACE_INT(cu_height);
    COM_TRACE_STR("cons mode ");
    COM_TRACE_INT(ctx->cons_pred_mode);
    COM_TRACE_STR("tree status ");
    COM_TRACE_INT(ctx->tree_status);
    COM_TRACE_STR("\n");

    /* parse CU info */
    if (ctx->tree_status != TREE_C)
    {
        ret = dec_decode_cu(ctx, core);
    }
    else
    {
        ret = dec_decode_cu_chroma(ctx, core);
    }
    com_assert_g(ret == COM_OK, ERR);

    /* inverse transform and dequantization */
    if (mod_info_curr->cu_mode != MODE_SKIP)
    {
#if IPCM
#if EST
        int use_secTrans = (ctx->info.sqh.est_enable_flag ?
                           (mod_info_curr->est_flag || mod_info_curr->pb_part)
                           : ctx->info.sqh.secondary_transform_enable_flag) &&
                           mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->ipm[PB0][0] != IPD_IPCM && ctx->tree_status != TREE_C;
        int use_alt4x4Trans = (ctx->info.sqh.est_enable_flag ?
                              (mod_info_curr->est_flag || mod_info_curr->pb_part)
                              : ctx->info.sqh.secondary_transform_enable_flag) &&
                              mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->ipm[PB0][0] != IPD_IPCM && ctx->tree_status != TREE_C;
#else
        int use_secTrans = ctx->info.sqh.secondary_transform_enable_flag && mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->ipm[PB0][0] != IPD_IPCM && ctx->tree_status != TREE_C;
        int use_alt4x4Trans = ctx->info.sqh.secondary_transform_enable_flag && mod_info_curr->cu_mode == MODE_INTRA && mod_info_curr->ipm[PB0][0] != IPD_IPCM && ctx->tree_status != TREE_C;
#endif
#else
        int use_secTrans = ctx->info.sqh.secondary_transform_enable_flag && mod_info_curr->cu_mode == MODE_INTRA && ctx->tree_status != TREE_C;
        int use_alt4x4Trans = ctx->info.sqh.secondary_transform_enable_flag && mod_info_curr->cu_mode == MODE_INTRA && ctx->tree_status != TREE_C;
#endif
#if ST_CHROMA
        int secT_Ver_Hor[MAX_NUM_CHANNEL][MAX_NUM_TB] = { { 0,0,0,0 },{ 0,0,0,0 } };
        int use_4x4SecTrans[MAX_NUM_CHANNEL] = { 0,0 };
        use_4x4SecTrans[CHANNEL_LUMA] = use_alt4x4Trans;
#else
        int secT_Ver_Hor[MAX_NUM_TB] = { 0, 0, 0, 0 };
#endif
        if (use_secTrans) // derive secT_Ver_Hor for each tb
        {
            int pb_part_size = mod_info_curr->pb_part;
            int tb_w, tb_h, tb_x, tb_y, tb_scup, tb_x_scu, tb_y_scu;
            for (int pb_idx = 0; pb_idx < core->mod_info_curr.pb_info.num_sub_part; pb_idx++)
            {
                int num_tb_in_pb = get_part_num_tb_in_pb(pb_part_size, pb_idx);
                int pb_w = mod_info_curr->pb_info.sub_w[pb_idx];
                int pb_h = mod_info_curr->pb_info.sub_h[pb_idx];
                int pb_x = mod_info_curr->pb_info.sub_x[pb_idx];
                int pb_y = mod_info_curr->pb_info.sub_y[pb_idx];

                for (int tb_idx = 0; tb_idx < num_tb_in_pb; tb_idx++)
                {
                    get_tb_width_height_in_pb(pb_w, pb_h, pb_part_size, pb_idx, &tb_w, &tb_h
#if DT_TRANSFORM
                        , tb_idx
#endif
                    );
                    //derive tu basic info
                    int tb_idx_in_cu = tb_idx + get_tb_idx_offset(pb_part_size, pb_idx);
                    get_tb_pos_in_pb(pb_x, pb_y, pb_part_size, tb_w, tb_h, tb_idx, &tb_x, &tb_y);
                    tb_x_scu = PEL2SCU(tb_x);
                    tb_y_scu = PEL2SCU(tb_y);
                    tb_scup = tb_x_scu + (tb_y_scu * ctx->info.pic_width_in_scu);

                    int avail_tb = com_get_avail_intra(tb_x_scu, tb_y_scu, ctx->info.pic_width_in_scu, tb_scup, ctx->map.map_scu
#if CCNPM
                        , ctx->info.pic_height_in_scu, tb_w, tb_h
#endif                    
                    );
                    s8 intra_mode = mod_info_curr->ipm[pb_idx][0];
                    int tb_width_log2 = com_tbl_log2[tb_w];
                    int tb_height_log2 = com_tbl_log2[tb_h];
                    assert(tb_width_log2 >= 2 && tb_height_log2 >= 2);

                    if (tb_width_log2 > 2 || tb_height_log2 > 2)
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
#if ST_CHROMA
                        secT_Ver_Hor[CHANNEL_LUMA][tb_idx_in_cu] = (vt << 1) | ht;
#else
                        secT_Ver_Hor[tb_idx_in_cu] = (vt << 1) | ht;
#endif
                    }
#if EST
                    if (ctx->info.sqh.est_enable_flag && mod_info_curr->tb_part == 0)
                    {
#if ST_CHROMA
                        secT_Ver_Hor[CHANNEL_LUMA][0] = 3;
#else
                        secT_Ver_Hor[0] = 3;
#endif
                    }
#endif
                }
            }
        }

#if ST_CHROMA
        int use_secTrans_chroma = com_st_chroma_allow(mod_info_curr, ctx->info.sqh.st_chroma_enable_flag, ctx->tree_status);
        if (use_secTrans_chroma)
        {
            use_4x4SecTrans[CHANNEL_CHROMA] = mod_info_curr->st_chroma_flag ? 1 : 0;
            secT_Ver_Hor[CHANNEL_CHROMA][0] = mod_info_curr->st_chroma_flag ? 3 : 0;
        }
#endif

#if PMC || EPMC
        int qp_v = core->qp_v;
        s8 ipm_c = mod_info_curr->ipm[PB0][1];
#if EPMC
        s8 ipm_c_t = mod_info_curr->ipm[PB0][1];
#endif
#if EPMC && PMC
        if ((com_is_mcpm(ipm_c)||com_is_emcpm(ipm_c_t)) && mod_info_curr->cu_mode == MODE_INTRA)
#elif EPMC
        if (com_is_emcpm(ipm_c_t) && mod_info_curr->cu_mode == MODE_INTRA)
#else
        if (com_is_mcpm(ipm_c) && mod_info_curr->cu_mode == MODE_INTRA)
#endif
        {
            qp_v = core->qp_v_pmc;
        }
#if ST_CHROMA
        com_itdq_yuv(mod_info_curr, mod_info_curr->coef, resi, ctx->wq, cu_width_log2, cu_height_log2, core->qp_y, core->qp_u, qp_v, bit_depth, secT_Ver_Hor, use_4x4SecTrans
#if IST_CHROMA
            , ctx->tree_status
#endif        
        );
#else
        com_itdq_yuv(mod_info_curr, mod_info_curr->coef, resi, ctx->wq, cu_width_log2, cu_height_log2, core->qp_y, core->qp_u, qp_v, bit_depth, secT_Ver_Hor, use_alt4x4Trans);
#endif
#else
#if ST_CHROMA
        com_itdq_yuv(mod_info_curr, mod_info_curr->coef, resi, ctx->wq, cu_width_log2, cu_height_log2, core->qp_y, core->qp_u, core->qp_v, bit_depth, secT_Ver_Hor, use_4x4SecTrans);
#else
        com_itdq_yuv(mod_info_curr, mod_info_curr->coef, resi, ctx->wq, cu_width_log2, cu_height_log2, core->qp_y, core->qp_u, core->qp_v, bit_depth, secT_Ver_Hor, use_alt4x4Trans);
#endif
#endif
    }
    if (ctx->tree_status != TREE_C)
    {
        dec_set_dec_info(ctx, core);
    }

#if USE_IBC
    /* prediction */
    if (mod_info_curr->cu_mode == MODE_IBC)
    {
#if CIBC
        if (ctx->tree_status == TREE_C)
        {
            if (mod_info_curr->cibc_flag)
            {
                s16 mv[MV_D];
                int valid_bv = derive_cibc_mv(mv, mod_info_curr, &ctx->info, &ctx->map);
                if (valid_bv)
                {
                    com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mv, ctx->pic, mod_info_curr->pred, ctx->tree_status, bit_depth);
                    com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, ctx->tree_status, bit_depth
#if SBT
                        , 0
#endif
#if INTER_CCNPM
                        , mod_info_curr, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                        , (1<<ctx->info.log2_max_cuwh)
#endif
                    );
                }
                else
                {
                    int cw = cu_width >> 1;
                    int ch = cu_height >> 1;
                    int val = 1 << (bit_depth - 1);
                    for (int x = 0; x < cw; x++)
                    {
                        for (int y = 0; y < ch; y++)
                        {
                            mod_info_curr->pred[U_C][x + y * cw] = val;
                            mod_info_curr->pred[V_C][x + y * cw] = val;
                        }
                    }
                }
            }
        }
        else
#endif
#if USE_SP
        if (mod_info_curr->ibc_flag)
        {
#endif
            com_IBC_mc(x, y, cu_width_log2, cu_height_log2, mod_info_curr->mv[0], ctx->pic, mod_info_curr->pred, ctx->tree_status, bit_depth);
#if IBC_ENH
            if (mod_info_curr->ibcpf_idx)
            {
#if IBC_APF
                if (ctx->info.pic_header.ibc_type == 0)
                {
                    pred_ibc_filter(ctx->pic, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, mod_info_curr
                        , x, y, cu_width, cu_height, bit_depth, ctx->tree_status, mod_info_curr->ibcpf_idx);
                }
                else
                {
#endif
                    pred_inter_filter(ctx->pic, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, mod_info_curr
                        , x, y, cu_width, cu_height, bit_depth, ctx->tree_status, mod_info_curr->ibcpf_idx);
#if IBC_APF
                }
#endif
            }
#endif
            com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, ctx->tree_status, bit_depth
#if SBT
#if IST_SBT_IBC
			, mod_info_curr->sbt_info
#else
			, 0
#endif
#endif
#if INTER_CCNPM
            , mod_info_curr, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                , (1<<ctx->info.log2_max_cuwh)
#endif
            );
#if USE_SP
        }
        else if (mod_info_curr->sp_flag)
        {
            assert(mod_info_curr->cu_mode == MODE_IBC);
            if (mod_info_curr->cs2_flag)
            {
                sp_cs2_recon_yuv(mod_info_curr, x, y, cu_width_log2, cu_height_log2, ctx->pic, ctx->dpb_evs
                    , ctx->tree_status
                    , 1 << ctx->info.log2_max_cuwh
                );
            }
            else
            {
#if ISC_RSD
                if (mod_info_curr->string_copy_info[0].is_matched == 0 && mod_info_curr->string_copy_info[0].sp_rsd_flag == 1)
                {
                    COM_SP_INFO derived_rsd_str_info[2];
                    u16 derived_rsd_str_no = 2;
                    derive_rsd_string(mod_info_curr, cu_width_log2, cu_height_log2, derived_rsd_str_info);
                    rsd_recon_yuv(mod_info_curr, x, y, cu_width_log2, cu_height_log2, ctx->pic, ctx->tree_status
                        , ctx->info.pic_width, ctx->info.pic_height, ctx->info.pic_width_in_scu, ctx->map.map_scu
                        , ctx->info.log2_max_cuwh, derived_rsd_str_info, derived_rsd_str_no);
#if !RSD_OPT
                    mod_info_curr->mv[0][MV_X] = mod_info_curr->string_copy_info[0].offset_x;
                    mod_info_curr->mv[0][MV_Y] = mod_info_curr->string_copy_info[0].offset_y;
#endif

                    com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, ctx->tree_status, bit_depth
#if SBT
                        , 0
#endif
#if INTER_CCNPM
                        , mod_info_curr, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                        , (1<<ctx->info.log2_max_cuwh)
#endif
                    );

                }
                else
                {
#endif
                sp_recon_yuv(mod_info_curr, x, y, cu_width_log2, cu_height_log2, ctx->pic, ctx->tree_status
                    , ctx->info.pic_width, ctx->info.pic_height, ctx->info.pic_width_in_scu, ctx->map.map_scu
                    , ctx->info.log2_max_cuwh
                );
#if ISC_RSD
                }
#endif
            }
        }
#endif
    }
    else
#endif
    /* prediction */
    if (mod_info_curr->cu_mode != MODE_INTRA)
    {
#if INTER_TM
        if (mod_info_curr->tm_flag)
        {
            BOOL is_simplified = ctx->info.sqh.tm_enable_flag == 1 ? 0 : 1;
            com_tm_process(x, y, ctx->info.pic_width, ctx->info.pic_height, cu_width, cu_height, ctx->refp, mod_info_curr->refi, mod_info_curr->init_mv, mod_info_curr->mv, ctx->pic->y, ctx->pic->stride_luma, bit_depth, is_simplified);
        }
#endif

#if IPC_SB8
        int  ipc_size = 16;
#endif
#if ETMVP
#if BAWP
        if (ctx->info.pic_header.slice_type == SLICE_P && mod_info_curr->etmvp_flag != 1 && mod_info_curr->awp_flag != 1)
#else
        if (ctx->info.pic_header.slice_type == SLICE_P && mod_info_curr->etmvp_flag != 1)
#endif
#else
        if (ctx->info.pic_header.slice_type == SLICE_P)
#endif
        {
            assert(REFI_IS_VALID(mod_info_curr->refi[REFP_0]) && !REFI_IS_VALID(mod_info_curr->refi[REFP_1]));
        }

        if (mod_info_curr->affine_flag)
        {
#if AFFINE_MEMORY_CONSTRAINT
            int mem = (cu_width + 7 + cu_width / 4) * (cu_height + 7 + cu_height / 4);
            for (int i = 0; i < REFP_NUM; i++)
            {
                if (REFI_IS_VALID(mod_info_curr->refi[i]))
                {
                    int memory_access = com_get_affine_memory_access(mod_info_curr->affine_mv[i], cu_width, cu_height, mod_info_curr->affine_flag + 1);
                    assert(memory_access <= mem);
                }
            }
#endif
#if AFFINE_DMVR
            COM_DMVR dmvr;
            dmvr.poc_c = ctx->ptr;
            dmvr.dmvr_current_template = mod_info_curr->dmvr_template;
            dmvr.dmvr_ref_pred_interpolated = mod_info_curr->dmvr_ref_pred_interpolated;
            dmvr.apply_DMVR = (mod_info_curr->dmvr_enable == 1) && ctx->info.sqh.dmvr_enable_flag;
            dmvr.dmvr_padding_buf = mod_info_curr->dmvr_padding_buf;
#endif
            com_affine_mc(&ctx->info, mod_info_curr, ctx->refp, &ctx->map, bit_depth
#if AFFINE_DMVR
                , &dmvr
#endif
            );
#if OBMC
            if (ctx->info.sqh.obmc_enable_flag)
            {
                int sub_w = 4, sub_h = 4;
                if (ctx->info.pic_header.affine_subblock_size_idx == 1)
                {
                    sub_w = 8;
                    sub_h = 8;
                }
                if (REFI_IS_VALID(mod_info_curr->refi[REFP_0]) && REFI_IS_VALID(mod_info_curr->refi[REFP_1]))
                {
                    sub_w = 8;
                    sub_h = 8;
                }
                pred_obmc(mod_info_curr, &ctx->info, &ctx->map, ctx->refp, ((ctx->tree_status == CHANNEL_LC || ctx->tree_status == CHANNEL_L) ? TRUE : FALSE), ((ctx->tree_status == CHANNEL_LC || ctx->tree_status == CHANNEL_C) ? TRUE : FALSE), bit_depth
                    , sub_w, sub_h
                    , ctx->ptr
#if BGC
                    , ((ctx->tree_status != CHANNEL_C) ? mod_info_curr->bgc_flag : 0), ((ctx->tree_status != CHANNEL_C) ? mod_info_curr->bgc_idx : 0)
#endif
#if OBMC_TEMP
                     ,ctx->pic
#endif
                );
            }
#endif
        }
#if AWP
        else if (mod_info_curr->awp_flag)
        {
#if AWP_ENH
            if (ctx->info.sqh.dawp_enable_flag && ctx->info.pic_header.slice_type != SLICE_P)
            {
                com_dawp_mc(&ctx->info, mod_info_curr, ctx->refp, ctx->tree_status, bit_depth, ctx->pic, ctx->map.map_scu, ctx->awp_weight_tpl);

                if (mod_info_curr->awp_flag)
                {
                    com_set_awp_mvf(&ctx->info, mod_info_curr, ctx->refp, &ctx->map);
                }
            }
            else
            {
                com_awp_mc(&ctx->info, mod_info_curr, ctx->refp, ctx->tree_status, bit_depth);
            }
#else
            com_awp_mc(&ctx->info, mod_info_curr, ctx->refp, ctx->tree_status, bit_depth);
#endif
        }
#endif
        else
        {
#if DMVR
            COM_DMVR dmvr;
            dmvr.poc_c = ctx->ptr; 
            dmvr.dmvr_current_template = mod_info_curr->dmvr_template;
            dmvr.dmvr_ref_pred_interpolated = mod_info_curr->dmvr_ref_pred_interpolated;
            dmvr.apply_DMVR = (mod_info_curr->dmvr_enable == 1) && ctx->info.sqh.dmvr_enable_flag;
            dmvr.dmvr_padding_buf = mod_info_curr->dmvr_padding_buf;
#endif
#if IPC
            mod_info_curr->bgc_flag &= !mod_info_curr->ipc_flag;
#endif
#if OBMC
            int obmc_blk_width = cu_width;
            int obmc_blk_height = cu_height;
#endif
#if SUB_TMVP
            if (core->sbTmvp_flag)
            {
#if OBMC
                obmc_blk_width = cu_width / SBTMVP_NUM_1D;
                obmc_blk_height = cu_height / SBTMVP_NUM_1D;
#endif
                com_sbTmvp_mc(&ctx->info, mod_info_curr, cu_width / SBTMVP_NUM_1D, cu_height / SBTMVP_NUM_1D, core->sbTmvp, ctx->refp, ctx->tree_status, bit_depth
#if DMVR
                    , &dmvr
#endif
#if BIO
                    , ctx->ptr, 0, mod_info_curr->mvr_idx
#endif
                );
#if IPC_SB8
                ipc_size = 8;
#endif
            }
            else
            {
#endif
#if MVAP
                if (core->mvap_flag)
                {
#if OBMC
                    obmc_blk_width = MIN_SUB_BLOCK_SIZE;
                    obmc_blk_height = MIN_SUB_BLOCK_SIZE;
#endif
                    com_mvap_mc(&ctx->info, mod_info_curr, (void*)core->best_cu_mvfield, ctx->refp, ctx->tree_status, bit_depth
#if DMVR
                        , &dmvr
#endif
#if BIO
                        , ctx->ptr, 0, mod_info_curr->mvr_idx
#endif
                    );
#if IPC_SB8
                    ipc_size = 8;
#endif
                }
                else
                {
#endif
#if ETMVP
                    if (mod_info_curr->etmvp_flag)
                    {
#if OBMC
                        obmc_blk_width = MIN_ETMVP_MC_SIZE;
                        obmc_blk_height = MIN_ETMVP_MC_SIZE;
#endif
                        com_etmvp_mc(&ctx->info, mod_info_curr, (void*)core->best_etmvp_mvfield, ctx->refp, ctx->tree_status, bit_depth
#if DMVR
                            , &dmvr
#endif
#if BIO
                            , ctx->ptr, 0, mod_info_curr->mvr_idx
#endif
                        );
                    }
                    else
                    {
#endif
                        assert(mod_info_curr->pb_info.sub_scup[0] == mod_info_curr->scup);
                        com_mc(mod_info_curr->x_pos, mod_info_curr->y_pos, mod_info_curr->cu_width, mod_info_curr->cu_height, mod_info_curr->cu_width, mod_info_curr->pred, &ctx->info, mod_info_curr, ctx->refp, ctx->tree_status, bit_depth
#if DMVR
                            , &dmvr
#endif
#if BIO
                            , ctx->ptr, 0, mod_info_curr->mvr_idx
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
#if ETMVP
                    }
#endif
#if MVAP
                }
#endif
#if SUB_TMVP
            }
#endif
#if OBMC
            if (ctx->info.sqh.obmc_enable_flag)
            {
#if DISABLE_OBMC_LOCAL_CHROMA_TREE
                if (ctx->tree_status != TREE_C)
                {
                pred_obmc(mod_info_curr, &ctx->info, &ctx->map, ctx->refp, TRUE, (ctx->tree_status == CHANNEL_LC ? TRUE : FALSE), bit_depth
#else
                if (ctx->tree_status == TREE_C)
                {
                    buffer_mvfield(mod_info_curr->tmp_mvfield, (cu_width >> MIN_CU_LOG2), &ctx->map.map_mv[mod_info_curr->scup], &ctx->map.map_refi[mod_info_curr->scup], ctx->info.pic_width_in_scu, (cu_width >> MIN_CU_LOG2), (cu_height >> MIN_CU_LOG2), TRUE);
                    set_col_mvfield(mod_info_curr, &ctx->map.map_mv[mod_info_curr->scup], &ctx->map.map_refi[mod_info_curr->scup], ctx->info.pic_width_in_scu, (cu_width >> MIN_CU_LOG2), (cu_height >> MIN_CU_LOG2));
                }
                pred_obmc(mod_info_curr, &ctx->info, &ctx->map, ctx->refp, ((ctx->tree_status == CHANNEL_LC || ctx->tree_status == CHANNEL_L) ? TRUE : FALSE), ((ctx->tree_status == CHANNEL_LC || ctx->tree_status == CHANNEL_C) ? TRUE : FALSE), bit_depth
#endif
                    , obmc_blk_width, obmc_blk_height
                    , ctx->ptr
#if BGC
                    , ((ctx->tree_status != CHANNEL_C) ? mod_info_curr->bgc_flag : 0), ((ctx->tree_status != CHANNEL_C) ? mod_info_curr->bgc_idx : 0)
#endif
#if OBMC_TEMP
                    ,ctx->pic
#endif
                );
#if DISABLE_OBMC_LOCAL_CHROMA_TREE
                }
#else
                if (ctx->tree_status == TREE_C)
                {
                    buffer_mvfield(mod_info_curr->tmp_mvfield, (cu_width >> MIN_CU_LOG2), &ctx->map.map_mv[mod_info_curr->scup], &ctx->map.map_refi[mod_info_curr->scup], ctx->info.pic_width_in_scu, (cu_width >> MIN_CU_LOG2), (cu_height >> MIN_CU_LOG2), FALSE);
                }
#endif
            }
#endif
#if INTERPF
            if(mod_info_curr->inter_filter_flag)
            {
                pred_inter_filter(ctx->pic, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, mod_info_curr
                                  , x, y, cu_width, cu_height, bit_depth, ctx->tree_status, mod_info_curr->inter_filter_flag);
            }
#endif
#if IPC
            if(mod_info_curr->ipc_flag)
            {
                pred_inter_pred_correction(ctx->pic, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, core->nb, mod_info_curr
                                  , x, y, cu_width, cu_height, bit_depth, ctx->tree_status, mod_info_curr->ipc_flag
#if IPC_SB8
                                  , ipc_size
#endif
                );
            }
#endif
#if DMVR
            if (ctx->tree_status != TREE_C)
            {
                dec_set_dec_info(ctx, core);
            }
#endif
        }
        /* reconstruction */
        com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, ctx->tree_status, bit_depth
#if SBT
            , mod_info_curr->sbt_info
#endif
#if INTER_CCNPM
            , mod_info_curr, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
            , (1<<ctx->info.log2_max_cuwh)
#endif
        );
    }
    else
    {
        //pred and recon for luma
        if (ctx->tree_status != TREE_C)
        {
#if IPCM
            if (mod_info_curr->ipm[PB0][0] == IPD_IPCM)
            {
                pel* rec_y;
                int i, j, rec_s;
                rec_s = ctx->pic->stride_luma;
                rec_y = ctx->pic->y + (y * rec_s) + x;

                for (i = 0; i < cu_height; i++)
                {
                    for (j = 0; j < cu_width; j++)
                    {
                        rec_y[i * rec_s + j] = mod_info_curr->coef[Y_C][i * cu_width + j] << (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                    }
                }
            }
            else
            {
#endif
                for (int pb_idx = 0; pb_idx < mod_info_curr->pb_info.num_sub_part; pb_idx++)
                {
                    int pb_x = mod_info_curr->pb_info.sub_x[pb_idx];
                    int pb_y = mod_info_curr->pb_info.sub_y[pb_idx];
                    int pb_w = mod_info_curr->pb_info.sub_w[pb_idx];
                    int pb_h = mod_info_curr->pb_info.sub_h[pb_idx];
                    int pb_sucp = mod_info_curr->pb_info.sub_scup[pb_idx];
                    pred_recon_intra_luma_pb(ctx->pic, ctx->map.map_scu, ctx->map.map_ipm, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, resi, core->nb, mod_info_curr,
                                             pb_x, pb_y, pb_w, pb_h, pb_sucp, pb_idx, x, y, cu_width, cu_height, bit_depth
#if MIPF
                        , ctx->info.sqh.mipf_enable_flag
#endif
#if DSAWP
                        , &ctx->info, ctx->awp_weight_tpl
                        , core->nb_tpl
#endif
                    );
                }
#if IPCM
            }
#endif
        }

        //pred and recon for uv
        if (ctx->tree_status != TREE_L)
        {
#if IPCM
            if (mod_info_curr->ipm[PB0][0] == IPD_IPCM && mod_info_curr->ipm[PB0][1] == IPD_DM_C)
            {
                pel* rec_u;
                pel* rec_v;
                int i, j, rec_s_c;
                rec_s_c = ctx->pic->stride_chroma;
                rec_u = ctx->pic->u + (y / 2 * rec_s_c) + x / 2;
                rec_v = ctx->pic->v + (y / 2 * rec_s_c) + x / 2;

                for (i = 0; i < cu_height / 2; i++)
                {
                    for (j = 0; j < cu_width / 2; j++)
                    {
                        rec_u[i * rec_s_c + j] = mod_info_curr->coef[U_C][i * cu_width / 2 + j] << (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                        rec_v[i * rec_s_c + j] = mod_info_curr->coef[V_C][i * cu_width / 2 + j] << (ctx->info.bit_depth_internal - ctx->info.bit_depth_input);
                    }
                }
            }
            else
            {
#endif
                u16 avail_cu = com_get_avail_intra(mod_info_curr->x_scu, mod_info_curr->y_scu, ctx->info.pic_width_in_scu, mod_info_curr->scup, ctx->map.map_scu
#if CCNPM
                    , ctx->info.pic_height_in_scu, cu_width, cu_height
#endif                    
                );
#if TSCPM
#if ECCPM
                get_nbr_yuv(x, y, cu_width, cu_height, avail_cu, ctx->pic, core->nb, core->nb_template, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, 1, bit_depth);
#else
                get_nbr_yuv(x, y, cu_width, cu_height, avail_cu, ctx->pic, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, 1, bit_depth);
#endif
#else
                get_nbr_yuv(x, y, cu_width, cu_height, avail_cu, ctx->pic, core->nb, mod_info_curr->scup, ctx->map.map_scu, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, 0, bit_depth);
#endif
#if TSCPM
                int stride_y = ctx->pic->stride_luma;
                pel *reco_y  = ctx->pic->y + (y * stride_y) + x;
#if PMC || EPMC
                int stride_uv = ctx->pic->stride_chroma;
                pel* reco_u   = ctx->pic->u + (y / 2 * stride_uv) + x / 2;
                pel* reco_v   = ctx->pic->v + (y / 2 * stride_uv) + x / 2;
#endif
#endif
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
                        , ctx->pic->y, ctx->pic->stride_luma, ctx->pic->u, ctx->pic->v, ctx->pic->stride_chroma, x, y
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
                        , mod_info_curr->ipf_flag&& ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                        , ctx->pic->y, ctx->pic->stride_luma, ctx->pic->u, ctx->pic->v, ctx->pic->stride_chroma, x, y
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
                }
                else {
#endif // SAWP
                com_ipred_uv(core->nb[1][0] + STNUM, core->nb[1][1] + STNUM, mod_info_curr->pred[U_C], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
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
                             , ctx->pic->y, ctx->pic->stride_luma, ctx->pic->u, ctx->pic->v, ctx->pic->stride_chroma, x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                             , (1<<ctx->info.log2_max_cuwh)
#endif
                            );
#if ECCPM
                    if(mod_info_curr->eccpm_flag && (mod_info_curr->ipm[PB0][1]<IPD_TSCPM_C) && (IS_AVAIL(avail_cu, AVAIL_UP2) || IS_AVAIL(avail_cu, AVAIL_LE2)))
                    {
                        com_ipred_uv_template(core->nb_template[1][0] + STNUM, core->nb_template[1][1] + STNUM, mod_info_curr->pred_template[0], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
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
                        caculate_eccpm_param_dec(ctx, x, y, cu_width, cu_height, mod_info_curr->pred[U_C], mod_info_curr->pred_template[0], avail_cu, U_C, reco_u, stride_uv, mod_info_curr->A, mod_info_curr->YC, param_num);
                    }
#endif                            
#if SAWP
                }
#endif // SAWP

#if PMC || EPMC
                /* reconstruction U only */
                com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, CHANNEL_U, bit_depth
#if SBT
                    , mod_info_curr->sbt_info
#endif
#if INTER_CCNPM
                    , mod_info_curr, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                    , (1<<ctx->info.log2_max_cuwh)
#endif
                );
#endif

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
                        , ctx->pic->y, ctx->pic->stride_luma, ctx->pic->u, ctx->pic->v, ctx->pic->stride_chroma, x, y
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
                        , ctx->pic->y, ctx->pic->stride_luma, ctx->pic->u, ctx->pic->v, ctx->pic->stride_chroma, x, y
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
                }
                else {
#endif // SAWP
                com_ipred_uv(core->nb[2][0] + STNUM, core->nb[2][1] + STNUM, mod_info_curr->pred[V_C], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                             , V_C, reco_y, stride_y, core->nb
#endif
#if MIPF
                             , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                             , reco_u, stride_uv
#endif
#if IPF_CHROMA
                             , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
#if CCNPM
                             , ctx->pic->y, ctx->pic->stride_luma, ctx->pic->u, ctx->pic->v, ctx->pic->stride_chroma, x, y
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                             , (1<<ctx->info.log2_max_cuwh)
#endif
                            );
#if ECCPM
                    if(mod_info_curr->eccpm_flag && (mod_info_curr->ipm[PB0][1]<IPD_TSCPM_C) && (IS_AVAIL(avail_cu, AVAIL_UP2) || IS_AVAIL(avail_cu, AVAIL_LE2)))
                    {
                        com_ipred_uv_template(core->nb_template[2][0] + STNUM, core->nb_template[2][1] + STNUM, mod_info_curr->pred_template[1], mod_info_curr->ipm[PB0][1], mod_info_curr->ipm[PB0][0], cu_width >> 1, cu_height >> 1, bit_depth, avail_cu
#if TSCPM
                            , V_C, reco_y, stride_y, core->nb_template
#endif
#if MIPF
                            , ctx->info.sqh.mipf_enable_flag
#endif
#if PMC || EPMC
                            , reco_u, stride_uv
#endif
#if IPF_CHROMA
                            , mod_info_curr->ipf_flag && ctx->info.sqh.chroma_ipf_enable_flag, ctx->tree_status
#endif
                        );
                        int param_num = 4;
                        caculate_eccpm_param_dec(ctx, x, y, cu_width, cu_height, mod_info_curr->pred[V_C], mod_info_curr->pred_template[1], avail_cu, V_C, reco_u, stride_uv, mod_info_curr->A, mod_info_curr->YC, param_num);
                	}
#endif
#if SAWP
                }
#endif // SAWP

#if PMC || EPMC
                /* reconstruction V only */
                com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, CHANNEL_V, bit_depth
#if SBT
                    , mod_info_curr->sbt_info
#endif
#if INTER_CCNPM
                    , mod_info_curr, ctx->info.pic_width_in_scu, ctx->info.pic_height_in_scu, ctx->map.map_scu
#endif
#if CCNPM_LINE_BUFFER_REDUCTION
                    , (1<<ctx->info.log2_max_cuwh)
#endif
                );
#else
                /* reconstruction */
                com_recon_yuv(mod_info_curr->tb_part, x, y, cu_width, cu_height, resi, mod_info_curr->pred, mod_info_curr->num_nz, ctx->pic, CHANNEL_C, bit_depth
#if SBT
                    , mod_info_curr->sbt_info
#endif
                );
#endif
#if IPCM
            }
#endif
        }
    }
#if TRACE_REC
    {
        int s_rec;
        pel* rec;
        if (ctx->tree_status != TREE_C)
        {
            s_rec = ctx->pic->stride_luma;
            rec = ctx->pic->y + (y * s_rec) + x;
#if CUDQP
            COM_TRACE_STR("qp_y ");
            COM_TRACE_INT(core->qp_y);
            COM_TRACE_STR("pred qp ");
            COM_TRACE_INT(ctx->cu_qp_group.pred_qp);
            COM_TRACE_STR("num delta qp ");
            COM_TRACE_INT(ctx->cu_qp_group.num_delta_qp);
            COM_TRACE_STR("qgx ");
            COM_TRACE_INT(ctx->cu_qp_group.cu_qp_group_x);
            COM_TRACE_STR("qgy ");
            COM_TRACE_INT(ctx->cu_qp_group.cu_qp_group_y);
            COM_TRACE_STR("\n");
#endif
            COM_TRACE_STR("rec y\n");
            for (int j = 0; j < cu_height; j++)
            {
                for (int i = 0; i < cu_width; i++)
                {
                    COM_TRACE_INT(rec[i]);
                    COM_TRACE_STR(" ");
                }
                COM_TRACE_STR("\n");
                rec += s_rec;
            }
        }

        if (ctx->tree_status != TREE_L)
        {
            cu_width = cu_width >> 1;
            cu_height = cu_height >> 1;
            s_rec = ctx->pic->stride_chroma;
            rec = ctx->pic->u + ((y >> 1) * s_rec) + (x >> 1);
#if CUDQP
            if (is_cu_nz(mod_info_curr->num_nz))
            {
                int qp_v = core->qp_v;
#if PMC || EPMC
                s8 ipm_c = mod_info_curr->ipm[PB0][1];
#if EPMC
                s8 ipm_c_t = mod_info_curr->ipm[PB0][1];
#endif
#if EPMC && PMC
                if ((com_is_mcpm(ipm_c) || com_is_emcpm(ipm_c_t)) && mod_info_curr->cu_mode == MODE_INTRA)
#elif EPMC
                if (com_is_emcpm(ipm_c_t) && mod_info_curr->cu_mode == MODE_INTRA)
#else
                if (com_is_mcpm(ipm_c) && mod_info_curr->cu_mode == MODE_INTRA)
#endif
                {
                    qp_v = core->qp_v_pmc;
                }
#endif
                COM_TRACE_STR("qp_u ");
                COM_TRACE_INT(core->qp_u);
                COM_TRACE_STR("qp_v ");
                COM_TRACE_INT(qp_v);
                COM_TRACE_STR("pred qp ");
                COM_TRACE_INT(ctx->cu_qp_group.pred_qp);
                COM_TRACE_STR("num delta qp ");
                COM_TRACE_INT(ctx->cu_qp_group.num_delta_qp);
                COM_TRACE_STR("qgx ");
                COM_TRACE_INT(ctx->cu_qp_group.cu_qp_group_x);
                COM_TRACE_STR("qgy ");
                COM_TRACE_INT(ctx->cu_qp_group.cu_qp_group_y);
                COM_TRACE_STR("\n");
            }
#endif
            COM_TRACE_STR("rec u\n");
            for (int j = 0; j < cu_height; j++)
            {
                for (int i = 0; i < cu_width; i++)
                {
                    COM_TRACE_INT(rec[i]);
                    COM_TRACE_STR(" ");
                }
                COM_TRACE_STR("\n");
                rec += s_rec;
            }

            rec = ctx->pic->v + ((y >> 1) * s_rec) + (x >> 1);
            COM_TRACE_STR("rec v\n");
            for (int j = 0; j < cu_height; j++)
            {
                for (int i = 0; i < cu_width; i++)
                {
                    COM_TRACE_INT(rec[i]);
                    COM_TRACE_STR(" ");
                }
                COM_TRACE_STR("\n");
                rec += s_rec;
            }
        }
    }
#endif
    return COM_OK;
ERR:
    return ret;
}

static int dec_eco_tree(DEC_CTX * ctx, DEC_CORE * core, int x0, int y0, int cu_width_log2, int cu_height_log2, int cup, int cud, COM_BSR * bs, DEC_SBAC * sbac
                        , const int parent_split, int qt_depth, int bet_depth, u8 cons_pred_mode, u8 tree_status)
{
    COM_MODE *mod_info_curr = &core->mod_info_curr;
    int ret;
    s8  split_mode;
    int cu_width, cu_height;
    int bound;
    cu_width = 1 << cu_width_log2;
    cu_height = 1 << cu_height_log2;
#if CUDQP
    if (com_is_cu_dqp(&ctx->info) && cu_width * cu_height >= ctx->info.pic_header.cu_qp_group_area_size)
    {
        assert(ctx->tree_status != TREE_C);
        ctx->cu_qp_group.num_delta_qp = 0;
        ctx->cu_qp_group.cu_qp_group_x = x0;
        ctx->cu_qp_group.cu_qp_group_y = y0;
        int luma_scup = PEL2SCU(x0) + PEL2SCU(y0) * ctx->info.pic_width_in_scu;
        ctx->cu_qp_group.pred_qp = (x0 > 0 && MCU_GET_CODED_FLAG(ctx->map.map_scu[luma_scup - 1])) ? MCU_GET_QP(ctx->map.map_scu[luma_scup - 1]) : ctx->info.shext.slice_qp;
        assert(ctx->cu_qp_group.pred_qp >= 0 && ctx->cu_qp_group.pred_qp <= MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth);
    }
#endif

#if USE_SP
    const int ctu_size = 1 << ctx->info.log2_max_cuwh;
    int log2Size = min(ctu_size, 64);
    if ((x0 % log2Size) == 0 && (y0 % log2Size) == 0 && cu_width == log2Size && cu_height == log2Size)
    {
        pel dpb_new[N_C][MAX_SRB_PRED_SIZE] = { { 0, }, };
        int dpb_cnt = 0;
        int i = 0;
        for (i = 0; i < core->n_pv_num; i++)
        {
            if (is_pv_valid(x0, y0, core->n_recent_pv[i].mv[0][0] + core->LcuRx0, core->n_recent_pv[i].mv[0][1], ctx->info.log2_max_cuwh, ctx->info.pic_width_in_scu, ctx->map.map_scu, ctx->info.pic_width)
                && !core->n_recent_dpb_re[i])
            {
                core->n_recent_dpb_idx[i] = MAX_SRB_PRED_SIZE;
            }
            else
            {
                int idx = core->n_recent_dpb_idx[i];
                if (idx < MAX_SRB_PRED_SIZE)
                {
                    dpb_new[Y_C][dpb_cnt] = ctx->dpb_evs[Y_C][idx];
                    if (core->n_recent_all_comp_flag[i])
                    {
                        dpb_new[U_C][dpb_cnt] = ctx->dpb_evs[U_C][idx];
                        dpb_new[V_C][dpb_cnt] = ctx->dpb_evs[V_C][idx];
                    }
                }
                else
                {
                    int ref_pos = 0, ref_pos_c = 0;
                    int scale_x = 0, scale_y = 0;
                    int s_src = ctx->pic->stride_luma;
                    ref_pos = (core->n_recent_pv[i].mv[0][1] + core->LcuRy0) * s_src + core->n_recent_pv[i].mv[0][0] + core->LcuRx0;
                    int s_src_c = ctx->pic->stride_chroma;
                    ref_pos_c = ((core->n_recent_pv[i].mv[0][1] + core->LcuRy0) >> 1) * s_src_c + ((core->n_recent_pv[i].mv[0][0] + core->LcuRx0) >> 1);
                    dpb_new[Y_C][dpb_cnt] = ctx->pic->y[ref_pos];
                    if (core->n_recent_all_comp_flag[i])
                    {
                        dpb_new[U_C][dpb_cnt] = ctx->pic->u[ref_pos_c];
                        dpb_new[V_C][dpb_cnt] = ctx->pic->v[ref_pos_c];
                    }
                }
                core->n_recent_dpb_idx[i] = dpb_cnt;
                dpb_cnt++;
            }
        }
        for (i = 0; i < dpb_cnt; i++)
        {
            ctx->dpb_evs[Y_C][i] = dpb_new[Y_C][i];
            ctx->dpb_evs[U_C][i] = dpb_new[U_C][i];
            ctx->dpb_evs[V_C][i] = dpb_new[V_C][i];
        }
    }
#endif

    if (cu_width > MIN_CU_SIZE || cu_height > MIN_CU_SIZE)
    {
        COM_TRACE_COUNTER;
        COM_TRACE_STR("x pos ");
        COM_TRACE_INT(core->x_pel + ((cup % (ctx->info.max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
        COM_TRACE_STR("y pos ");
        COM_TRACE_INT(core->y_pel + ((cup / (ctx->info.max_cuwh >> MIN_CU_LOG2) << MIN_CU_LOG2)));
        COM_TRACE_STR("width ");
        COM_TRACE_INT(cu_width);
        COM_TRACE_STR("height ");
        COM_TRACE_INT(cu_height);
        COM_TRACE_STR("depth ");
        COM_TRACE_INT(cud);
        COM_TRACE_STR("\n");
        split_mode = dec_eco_split_mode(ctx, bs, sbac, cu_width, cu_height, parent_split, qt_depth, bet_depth, x0, y0);
    }
    else
    {
        split_mode = NO_SPLIT;
    }
    com_set_split_mode(split_mode, cud, cup, cu_width, cu_height, ctx->info.max_cuwh, core->split_mode);
    bound = !(x0 + cu_width <= ctx->info.pic_width && y0 + cu_height <= ctx->info.pic_height);
    if (split_mode != NO_SPLIT)
    {
        COM_SPLIT_STRUCT split_struct;
        com_split_get_part_structure(split_mode, x0, y0, cu_width, cu_height, cup, cud, ctx->info.log2_max_cuwh - MIN_CU_LOG2, &split_struct);
        u8 tree_status_child = TREE_LC;
        u8 cons_pred_mode_child = NO_MODE_CONS;
#if CHROMA_NOT_SPLIT
        tree_status_child = (tree_status == TREE_LC && com_tree_split(cu_width, cu_height, split_mode, ctx->info.pic_header.slice_type)) ? TREE_L : tree_status;
#endif
#if MODE_CONS
        if (cons_pred_mode == NO_MODE_CONS && com_constrain_pred_mode(cu_width, cu_height, split_mode, ctx->info.pic_header.slice_type))
        {
            cons_pred_mode_child = dec_eco_cons_pred_mode_child(bs, sbac);
        }
        else
        {
            cons_pred_mode_child = cons_pred_mode;
        }
#endif
        for (int part_num = 0; part_num < split_struct.part_count; ++part_num)
        {
            int cur_part_num = part_num;
            int log2_sub_cuw = split_struct.log_cuw[cur_part_num];
            int log2_sub_cuh = split_struct.log_cuh[cur_part_num];
            int x_pos = split_struct.x_pos[cur_part_num];
            int y_pos = split_struct.y_pos[cur_part_num];

            if (x_pos < ctx->info.pic_width && y_pos < ctx->info.pic_height)
            {
                ret = dec_eco_tree(ctx, core, x_pos, y_pos, log2_sub_cuw, log2_sub_cuh, split_struct.cup[cur_part_num], split_struct.cud, bs, sbac
                                   , split_mode, INC_QT_DEPTH(qt_depth, split_mode), INC_BET_DEPTH(bet_depth, split_mode), cons_pred_mode_child, tree_status_child);
                com_assert_g(ret == COM_OK, ERR);
            }
        }
#if CHROMA_NOT_SPLIT
        if (tree_status_child == TREE_L && tree_status == TREE_LC)
        {
            ctx->tree_status = TREE_C;
            ctx->cons_pred_mode = NO_MODE_CONS;
            ret = dec_eco_unit(ctx, core, x0, y0, cu_width_log2, cu_height_log2, cud);
            com_assert_g(ret == COM_OK, ERR);
            ctx->tree_status = TREE_LC;
        }
#endif
    }
    else
    {
#if CHROMA_NOT_SPLIT
        ctx->tree_status = tree_status;
#endif
#if MODE_CONS
        ctx->cons_pred_mode = cons_pred_mode;
#endif
        ret = dec_eco_unit(ctx, core, x0, y0, cu_width_log2, cu_height_log2, cud);
        com_assert_g(ret == COM_OK, ERR);
#if CUDQP
        if (com_is_cu_dqp(&ctx->info) && ctx->tree_status != TREE_C)
        {
            if (is_cu_nz(core->mod_info_curr.num_nz))
            {
                ctx->cu_qp_group.num_delta_qp++;
                ctx->cu_qp_group.pred_qp = core->qp_y;
            }
        }
#endif
#if UNIFIED_HMVP_1
        int subBlockFlag = core->mod_info_curr.mvap_flag || core->mod_info_curr.sub_tmvp_flag || core->mod_info_curr.etmvp_flag;
#endif
#if AWP
        if (ctx->info.sqh.num_of_hmvp_cand && core->mod_info_curr.cu_mode != MODE_INTRA && !core->mod_info_curr.affine_flag && !core->mod_info_curr.awp_flag
#if UNIFIED_HMVP_1
            && !subBlockFlag
#endif
            )
#else
        if (ctx->info.sqh.num_of_hmvp_cand && core->mod_info_curr.cu_mode != MODE_INTRA && !core->mod_info_curr.affine_flag
#if UNIFIED_HMVP_1
            && !subBlockFlag
#endif
        )
#endif
        {
#if BGC
            s8 bgc_flag;
            s8 bgc_idx;
            if (REFI_IS_VALID(ctx->map.map_refi[mod_info_curr->scup][REFP_0]) && REFI_IS_VALID(ctx->map.map_refi[mod_info_curr->scup][REFP_1]))
            {
                bgc_flag = core->mod_info_curr.bgc_flag;
                bgc_idx = core->mod_info_curr.bgc_idx;  
            }
            else
            {
                bgc_flag = 0;
                bgc_idx = 0;
            }
            update_skip_candidates(core->motion_cands, core->bgc_flag_cands, core->bgc_idx_cands, &core->cnt_hmvp_cands, ctx->info.sqh.num_of_hmvp_cand, ctx->map.map_mv[mod_info_curr->scup], ctx->map.map_refi[mod_info_curr->scup], bgc_flag, bgc_idx); // core->mod_info_curr.mv, core->mod_info_curr.refi, bgc_flag, bgcidx);
#else
            update_skip_candidates(core->motion_cands, &core->cnt_hmvp_cands, ctx->info.sqh.num_of_hmvp_cand, ctx->map.map_mv[mod_info_curr->scup], ctx->map.map_refi[mod_info_curr->scup]); // core->mod_info_curr.mv, core->mod_info_curr.refi);
#endif
        }
#if HACD
        if (ctx->info.sqh.affine_enable_flag && ctx->info.sqh.num_of_hat_cand && core->mod_info_curr.cu_mode != MODE_INTRA && core->mod_info_curr.affine_flag)
        {
            update_history_affine_candidates(core->history_affine_mv, core->mod_info_curr.affine_flag, core->mod_info_curr.affine_mv, ctx->info.sqh.num_of_hat_cand, ctx->map.map_refi[mod_info_curr->scup], cu_width_log2, cu_height_log2);
        }
#endif

#if USE_SP
        if (ctx->info.sqh.num_of_hbvp_cand && !core->mod_info_curr.affine_flag
            && (core->mod_info_curr.ibc_flag || (core->mod_info_curr.sp_flag
                && !core->mod_info_curr.cs2_flag
#if RSD_OPT
                && !core->mod_info_curr.sp_rsd_flag
#endif
        )))
        {
            if (!core->mod_info_curr.sp_flag)
            {
                core->hbvp_empty_flag = 1;
                update_ibc_skip_candidates(core->block_motion_cands, &core->cnt_hbvp_cands, ctx->info.sqh.num_of_hbvp_cand, ctx->map.map_mv[mod_info_curr->scup][0], x0, y0, cu_width, cu_height, cu_width * cu_height);
                assert(ctx->tree_status != TREE_C);
            }
            else
            {
                s16 offset_curr[MV_D] = { 0 };
                int cur_pixel = 0;
                int trav_x, trav_y;

                for (int j = 0; j < mod_info_curr->sub_string_no; j++)
                {
                    if (mod_info_curr->string_copy_info[j].is_matched)
                    {
                        trav_x = x0 + com_tbl_raster2trav_2d[mod_info_curr->cu_width_log2 - MIN_CU_LOG2][mod_info_curr->cu_height_log2 - MIN_CU_LOG2][cur_pixel][0];
                        trav_y = y0 + com_tbl_raster2trav_2d[mod_info_curr->cu_width_log2 - MIN_CU_LOG2][mod_info_curr->cu_height_log2 - MIN_CU_LOG2][cur_pixel][1];

                        offset_curr[MV_X] = mod_info_curr->string_copy_info[j].offset_x;
                        offset_curr[MV_Y] = mod_info_curr->string_copy_info[j].offset_y;

                        if (!((mod_info_curr->string_copy_info[j].is_matched == 0)
                            || (offset_curr[MV_X] == 0 && offset_curr[MV_Y] == -1 && mod_info_curr->sp_copy_direction == TRUE)
                            || (offset_curr[MV_X] == -1 && offset_curr[MV_Y] == 0 && mod_info_curr->sp_copy_direction == FALSE)))
                        {
                            offset_curr[MV_X] = offset_curr[MV_X] << 2;
                            offset_curr[MV_Y] = offset_curr[MV_Y] << 2;
                            core->hbvp_empty_flag = 1;
                            update_ibc_skip_candidates(core->block_motion_cands, &core->cnt_hbvp_cands, ctx->info.sqh.num_of_hbvp_cand, offset_curr, trav_x, trav_y, cu_width, cu_height, mod_info_curr->string_copy_info[j].length);
                        }
                        cur_pixel += mod_info_curr->string_copy_info[j].length;
                    }
#if ISC_RSD && !RSD_OPT
                    else if (j == 0 && mod_info_curr->string_copy_info[j].sp_rsd_flag != 0)
                    {
                        offset_curr[MV_X] = mod_info_curr->string_copy_info[j].offset_x;
                        offset_curr[MV_Y] = mod_info_curr->string_copy_info[j].offset_y;
                        core->hbvp_empty_flag = 1;
                        update_ibc_skip_candidates(core->block_motion_cands, &core->cnt_hbvp_cands, ctx->info.sqh.num_of_hbvp_cand, offset_curr, x0, y0, cu_width, cu_height, cu_width * cu_height);
                    }
#endif
                    else
                    {
                        cur_pixel += 4;
                    }
                }
            }

            // copy to sp_recent_cand
            core->n_offset_num = core->cnt_hbvp_cands;
            for (int j = 0;j < core->cnt_hbvp_cands;j++)
            {
                core->n_recent_offset[j].mv[0][MV_X] = core->block_motion_cands[j].mv[MV_X];
                core->n_recent_offset[j].mv[0][MV_Y] = core->block_motion_cands[j].mv[MV_Y];
            }
        }
        if (core->mod_info_curr.sp_flag && core->mod_info_curr.cs2_flag && !core->mod_info_curr.affine_flag && !core->mod_info_curr.ibc_flag)
        {
            update_sp_recent_pvcands(core->n_recent_pv, &core->n_pv_num, mod_info_curr);
            {
                memcpy(core->n_recent_all_comp_flag, mod_info_curr->all_comp_pre_flag, core->n_pv_num * sizeof(u8));
                memcpy(core->n_recent_dpb_idx, mod_info_curr->m_dpb_idx_prev, core->n_pv_num * sizeof(u8));
                memcpy(core->n_recent_dpb_re, mod_info_curr->m_dpb_reYonly_prev, core->n_pv_num * sizeof(u8));
            }
        }
#endif
    }
    return COM_OK;
ERR:
    return ret;
}

int dec_deblock_avs2(DEC_CTX * ctx)
{
    clear_edge_filter_avs2(0, 0, ctx->pic->width_luma, ctx->pic->height_luma, ctx->edge_filter);
    set_edge_filter_avs2(&ctx->info, &ctx->map, ctx->pic, ctx->edge_filter);
    deblock_frame_avs2(&ctx->info, &ctx->map, ctx->pic, ctx->refp, ctx->edge_filter);
    return COM_OK;
}

int dec_sao_avs2(DEC_CTX * ctx)
{
    copy_frame_for_sao(ctx->pic_sao, ctx->pic);
    sao_frame(&ctx->info, &ctx->map, ctx->pic, ctx->pic_sao, ctx->rec_sao_blk_params);
    return COM_OK;
}

void read_param_sao_one_lcu(DEC_CTX* ctx, int y_pel, int x_pel,
                         SAO_BLK_PARAM *sao_blk_param, SAO_BLK_PARAM *rec_sao_blk_param)
{
    COM_SH_EXT *sh = &(ctx->info.shext);
    int lcuw = 1 << ctx->info.log2_max_cuwh;
    int lcuh = 1 << ctx->info.log2_max_cuwh;
    int lcu_pix_width = min(lcuw, ctx->info.pic_width - x_pel);
    int lcu_pix_height = min(lcuh, ctx->info.pic_height - y_pel);
    int x_in_lcu = x_pel >> ctx->info.log2_max_cuwh;
    int y_in_lcu = y_pel >> ctx->info.log2_max_cuwh;
    int lcu_pos = x_in_lcu + y_in_lcu * ctx->info.pic_width_in_lcu;
    if (!sh->slice_sao_enable[Y_C] && !sh->slice_sao_enable[U_C] && !sh->slice_sao_enable[V_C])
    {
        off_sao(rec_sao_blk_param);
        off_sao(sao_blk_param);
    }
    else
    {
        read_sao_lcu(ctx, lcu_pos, y_pel, x_pel, lcu_pix_width, lcu_pix_height, sao_blk_param, rec_sao_blk_param);
    }
}

void read_sao_lcu(DEC_CTX* ctx, int lcu_pos, int pix_y, int pix_x, int smb_pix_width, int smb_pix_height,
                  SAO_BLK_PARAM *sao_cur_param, SAO_BLK_PARAM *rec_sao_cur_param)
{
    COM_SH_EXT *sh = &(ctx->info.shext);
    int mb_x = pix_x >> MIN_CU_LOG2;
    int mb_y = pix_y >> MIN_CU_LOG2;
    SAO_BLK_PARAM merge_candidate[NUM_SAO_MERGE_TYPES][N_C];
    int merge_avail[NUM_SAO_MERGE_TYPES];
    int merge_left_avail, merge_up_avail;
    int merge_mode, sao_mode, sao_type;
    int offset[32];
    int comp_idx, i;
    int start_band[2];
    DEC_SBAC   *sbac;
    COM_BSR     *bs;
    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);

    get_sao_merge_neighbor(&ctx->info, ctx->map.map_patch_idx, ctx->info.pic_width_in_scu, ctx->info.pic_width_in_lcu, lcu_pos, mb_y, mb_x,
                        ctx->rec_sao_blk_params, merge_avail, merge_candidate);
    merge_left_avail = merge_avail[0];
    merge_up_avail = merge_avail[1];
    merge_mode = 0;
    if (merge_left_avail + merge_up_avail > 0)
    {
        merge_mode = dec_eco_sao_merge_flag(sbac, bs, merge_left_avail, merge_up_avail);
    }
    if (merge_mode)
    {

        if (merge_mode == 2)
        {
            copy_sao_param_for_blk(rec_sao_cur_param, merge_candidate[SAO_MERGE_LEFT]);
        }
        else
        {
            assert(merge_mode == 1);
            copy_sao_param_for_blk(rec_sao_cur_param, merge_candidate[SAO_MERGE_ABOVE]);
        }
        copy_sao_param_for_blk(sao_cur_param, rec_sao_cur_param);
    }
    else
    {
        for (comp_idx = Y_C; comp_idx < N_C; comp_idx++)
        {
            if (!sh->slice_sao_enable[comp_idx])
            {
                sao_cur_param[comp_idx].mode_idc = SAO_MODE_OFF;
            }
            else
            {
                sao_mode = dec_eco_sao_mode(sbac, bs);
                switch (sao_mode)
                {
                case 0:
                    sao_cur_param[comp_idx].mode_idc = SAO_MODE_OFF;
                    break;
                case 1:
                    sao_cur_param[comp_idx].mode_idc = SAO_MODE_NEW;
                    sao_cur_param[comp_idx].type_idc = SAO_TYPE_BO;
                    break;
                case 2:
                    sao_cur_param[comp_idx].mode_idc = SAO_MODE_NEW;
                    sao_cur_param[comp_idx].type_idc = SAO_TYPE_EO_0;
                    break;
                default:
                    assert(1);
                    break;
                }

                if (sao_mode)
                {
                    dec_eco_sao_offset(sbac, bs, &(sao_cur_param[comp_idx]), offset);

                    if (sao_cur_param[comp_idx].type_idc == SAO_TYPE_BO)
                    {
                        dec_eco_sao_BO_start(sbac, bs, start_band);
                        memset(sao_cur_param[comp_idx].offset, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);

                        for (i = 0; i < 2; i++)
                        {
                            sao_cur_param[comp_idx].offset[start_band[i]] = offset[i * 2];
                            sao_cur_param[comp_idx].offset[(start_band[i] + 1) % 32] = offset[i * 2 + 1];
                        }
                    }
                    else
                    {
                        sao_type = dec_eco_sao_EO_type(sbac, bs);
                        assert(sao_cur_param[comp_idx].type_idc == SAO_TYPE_EO_0);
                        sao_cur_param[comp_idx].type_idc = sao_type;
                        sao_cur_param[comp_idx].offset[SAO_CLASS_EO_FULL_VALLEY] = offset[0];
                        sao_cur_param[comp_idx].offset[SAO_CLASS_EO_HALF_VALLEY] = offset[1];
                        sao_cur_param[comp_idx].offset[SAO_CLASS_EO_PLAIN] = 0;
                        sao_cur_param[comp_idx].offset[SAO_CLASS_EO_HALF_PEAK] = offset[2];
                        sao_cur_param[comp_idx].offset[SAO_CLASS_EO_FULL_PEAK] = offset[3];
                    }
                }
            }
        }
        copy_sao_param_for_blk(rec_sao_cur_param, sao_cur_param);
    }
}

#if ESAO
int  dec_esao(DEC_CTX * ctx)
{
    if (ctx->info.pic_header.pic_esao_on[0] || ctx->info.pic_header.pic_esao_on[1] || ctx->info.pic_header.pic_esao_on[2])
    {
        copy_frame_for_esao(ctx->pic_esao, ctx->pic);
#if ESAO_PH_SYNTAX
        esao_on_frame(&ctx->info, &ctx->map, ctx->pic, ctx->pic_esao, ctx->info.pic_header.pic_esao_params,&ctx->func_esao_block_filter);
#else
        esao_on_frame(&ctx->info, &ctx->map, ctx->pic, ctx->pic_esao, ctx->pic_esao_params,&ctx->func_esao_block_filter);
#endif
    }
    return COM_OK;
}
#endif 

#if CCSAO
int dec_ccsao(DEC_CTX *ctx)
{
    if (ctx->info.pic_header.pic_ccsao_on[U_C-1] || ctx->info.pic_header.pic_ccsao_on[V_C-1])
    {
#if CCSAO_ENHANCEMENT
        copy_frame_for_ccsao(ctx->pic_ccsao[1], ctx->pic, U_C);
        copy_frame_for_ccsao(ctx->pic_ccsao[1], ctx->pic, V_C);
#else
        copy_frame_for_ccsao(ctx->pic_ccsao, ctx->pic, U_C);
        copy_frame_for_ccsao(ctx->pic_ccsao, ctx->pic, V_C);
#endif
#if CCSAO_PH_SYNTAX
        ccsao_on_frame(&ctx->info, &ctx->map, ctx->pic, ctx->pic_ccsao, ctx->info.pic_header.pic_ccsao_params, &ctx->ccsao_func_ptr);
#else
        ccsao_on_frame(&ctx->info, &ctx->map, ctx->pic, ctx->pic_ccsao, ctx->pic_ccsao_params, &ctx->ccsao_func_ptr);
#endif
    }
    return COM_OK;
}
#endif

int dec_alf_avs2(DEC_CTX * ctx, COM_PIC *pic_rec)
{
    BOOL curr_alf_enable = !(ctx->dec_alf->alf_picture_param[Y_C]->alf_flag == 0 &&
                           ctx->dec_alf->alf_picture_param[U_C]->alf_flag == 0 && ctx->dec_alf->alf_picture_param[V_C]->alf_flag == 0);
    if (curr_alf_enable)
    {
        ctx->pic_alf_Dec->stride_luma = pic_rec->stride_luma;
        ctx->pic_alf_Dec->stride_chroma = pic_rec->stride_chroma;
        copy_frame_for_alf(ctx->pic_alf_Dec, pic_rec);
        alf_process_dec(ctx, ctx->dec_alf->alf_picture_param, pic_rec, ctx->pic_alf_Dec);
    }
    return COM_OK;
}

int dec_pic(DEC_CTX * ctx, DEC_CORE * core, COM_SQH *sqh, COM_PIC_HEADER * ph, COM_SH_EXT * shext)
{
    COM_BSR   * bs;
    DEC_SBAC  * sbac;
    int         ret;
    int         size;
    COM_PIC_HEADER *sh = &ctx->info.pic_header;
    int last_lcu_qp;
    int last_lcu_delta_qp;
#if PATCH
    int patch_cur_index = -1;
    PATCH_INFO *patch = ctx->patch;
    int patch_cur_lcu_x;
    int patch_cur_lcu_y;
    u32 * map_scu_temp;
    s8(*map_refi_temp)[REFP_NUM];
    s16(*map_mv_temp)[REFP_NUM][MV_D];
    u32 *map_cu_mode_temp;
    map_scu_temp = ctx->map.map_scu_temp;
    map_refi_temp = ctx->map.map_refi_temp;
    map_mv_temp = ctx->map.map_mv_temp;
    map_cu_mode_temp = ctx->map.map_cu_mode_temp;
    com_mset_x64a(map_scu_temp, 0, sizeof(u32)* ctx->info.f_scu);
    com_mset_x64a(map_refi_temp, -1, sizeof(s8)* ctx->info.f_scu * REFP_NUM);
    com_mset_x64a(map_mv_temp, 0, sizeof(s16)* ctx->info.f_scu * REFP_NUM * MV_D);
    com_mset_x64a(map_cu_mode_temp, 0, sizeof(u32)* ctx->info.f_scu);
#if USE_SP
    u8 * map_usp_temp;
    map_usp_temp = ctx->map.map_usp_temp;
    com_mset_x64a(map_usp_temp, 0, sizeof(u8)*ctx->info.f_scu);
#endif
#endif
    size = sizeof(s8) * ctx->info.f_scu * REFP_NUM;
    com_mset_x64a(ctx->map.map_refi, -1, size);
    size = sizeof(s16) * ctx->info.f_scu * REFP_NUM * MV_D;
    com_mset_x64a(ctx->map.map_mv, 0, size);
    bs = &ctx->bs;
    sbac = GET_SBAC_DEC(bs);
    /* reset SBAC */
    dec_sbac_init(bs);
    com_sbac_ctx_init(&(sbac->ctx));

    last_lcu_qp = ctx->info.shext.slice_qp;
    last_lcu_delta_qp = 0;
#if PATCH
    /*initial patch info*/
    patch->x_pel = patch->x_pat = patch->y_pel = patch->y_pat = patch->idx = 0;
    patch_cur_lcu_x = 0;
    patch_cur_lcu_y = 0;
    patch->left_pel = patch->x_pel;
    patch->up_pel = patch->y_pel;
    patch->right_pel = patch->x_pel + (*(patch->width_in_lcu + patch->x_pat) << ctx->info.log2_max_cuwh);
    patch->down_pel = patch->y_pel + (*(patch->height_in_lcu + patch->y_pat) << ctx->info.log2_max_cuwh);
    ctx->lcu_cnt = ctx->info.f_lcu;
#endif
#if ESAO && !CCSAO_PH_SYNTAX
    if (ctx->info.sqh.esao_enable_flag)
    {
        dec_eco_esao_param(ctx, sbac, bs);
    }
#endif
#if CCSAO && !CCSAO_PH_SYNTAX
    if (ctx->info.sqh.ccsao_enable_flag)
    {
        dec_eco_ccsao_param(ctx, sbac, bs);
    }
#endif
    while (1)
    {
        int lcu_qp;
        int adj_qp_cb, adj_qp_cr;
#if PATCH
        /*set patch idx*/
        if (patch_cur_index != patch->idx)
            dec_set_patch_idx(ctx->map.map_patch_idx, patch, ctx->info.pic_width, ctx->info.pic_height);
        patch_cur_index = patch->idx;
#endif

        // reset HMVP list before each LCU line for parallel computation
#if PATCH
        if (core->x_lcu * ctx->info.max_cuwh == patch->left_pel)
        {
#else
        if (core->x_lcu == 0)
        {
#endif
            core->cnt_hmvp_cands = 0;
            com_mset_x64a(core->motion_cands, 0, sizeof(COM_MOTION)*ALLOWED_HMVP_NUM);
#if BGC
            com_mset_x64a(core->bgc_flag_cands, 0, sizeof(s8)*ALLOWED_HMVP_NUM);
            com_mset_x64a(core->bgc_idx_cands, 0, sizeof(s8)*ALLOWED_HMVP_NUM);
#endif
#if IBC_BVP
            core->cnt_hbvp_cands = 0;
            com_mset_x64a(core->block_motion_cands, 0, sizeof(COM_BLOCK_MOTION)*ALLOWED_HBVP_NUM);
            core->hbvp_empty_flag = 0;
#endif
#if HACD
            com_mset_x64a(core->history_affine_mv, 0, sizeof(COM_HISTORY_AFFINE_MV*) * ALLOWED_HAP_NUM);
#endif
#if USE_SP
            core->n_offset_num = 0;
            com_mset_x64a(core->n_recent_offset, 0, sizeof(COM_MOTION)*SP_RECENT_CANDS);
#endif
            core->n_pv_num = 0;
            core->LcuRx0 = patch->left_pel;
            core->LcuRy0 = core->y_pel;
            com_mset_x64a(core->n_recent_pv, -1, sizeof(COM_MOTION)*MAX_SRB_PRED_SIZE);
        }
        com_assert_rv(core->lcu_num < ctx->info.f_lcu, COM_ERR_UNEXPECTED);

        if (ctx->info.shext.fixed_slice_qp_flag)
        {
            lcu_qp = ctx->info.shext.slice_qp;
        }
        else
        {
#if CUDQP
            if (com_is_cu_dqp(&ctx->info))
            {
                last_lcu_delta_qp = 0;
            }
            else
            {
#endif
                last_lcu_delta_qp = dec_eco_lcu_delta_qp(bs, sbac, last_lcu_delta_qp);
#if CUDQP
            }
#endif
            lcu_qp = last_lcu_qp + last_lcu_delta_qp;
        }
#if CUDQP
        ctx->cu_qp_group.pred_qp = lcu_qp; //when cudqp is on, it will be updated at gp group level
#endif

        core->qp_y = lcu_qp;
        adj_qp_cb = core->qp_y + sh->chroma_quant_param_delta_cb - ctx->info.qp_offset_bit_depth;
        adj_qp_cr = core->qp_y + sh->chroma_quant_param_delta_cr - ctx->info.qp_offset_bit_depth;
#if PMC || EPMC
        int adj_qp_cr_pmc = adj_qp_cr + V_QP_OFFSET;
        adj_qp_cr_pmc = COM_CLIP(adj_qp_cr_pmc, MIN_QUANT - 16, MAX_QUANT_BASE);
        if (adj_qp_cr_pmc >= 0)
        {
            adj_qp_cr_pmc = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, adj_qp_cr_pmc)];
        }
        core->qp_v_pmc = COM_CLIP(adj_qp_cr_pmc + ctx->info.qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth);
#endif
        adj_qp_cb = COM_CLIP( adj_qp_cb, MIN_QUANT - 16, MAX_QUANT_BASE );
        adj_qp_cr = COM_CLIP( adj_qp_cr, MIN_QUANT - 16, MAX_QUANT_BASE );
        if (adj_qp_cb >= 0)
        {
            adj_qp_cb = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, adj_qp_cb)];
        }
        if (adj_qp_cr >= 0)
        {
            adj_qp_cr = com_tbl_qp_chroma_adjust[COM_MIN(MAX_QUANT_BASE, adj_qp_cr)];
        }
        core->qp_u = COM_CLIP( adj_qp_cb + ctx->info.qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth );
        core->qp_v = COM_CLIP( adj_qp_cr + ctx->info.qp_offset_bit_depth, MIN_QUANT, MAX_QUANT_BASE + ctx->info.qp_offset_bit_depth );
        last_lcu_qp = lcu_qp;

        if (ctx->info.sqh.sample_adaptive_offset_enable_flag)
        {
            int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
            read_param_sao_one_lcu(ctx, core->y_pel, core->x_pel, ctx->sao_blk_params[lcu_pos], ctx->rec_sao_blk_params[lcu_pos]);
        }
            
#if ESAO_PH_SYNTAX
        if (ctx->info.sqh.esao_enable_flag)
        {
            int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
            for (int comp_idx = Y_C; comp_idx < N_C; comp_idx++)
            {
                if (ctx->info.pic_header.pic_esao_on[comp_idx] && ctx->info.pic_header.esao_lcu_enable[comp_idx])
                {
#if ESAO_ENH
                    ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag[lcu_pos] = dec_eco_esao_lcu_control_flag(sbac, bs, ctx->info.pic_header.esao_set_num[comp_idx]);
#else
                    ctx->info.pic_header.pic_esao_params[comp_idx].lcu_flag[lcu_pos] = dec_eco_esao_lcu_control_flag(sbac, bs);
#endif
                }
            }
        }
#endif

#if CCSAO_PH_SYNTAX
        if (ctx->info.sqh.ccsao_enable_flag)
        {
            int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
            for (int comp = U_C-1; comp < N_C-1; comp++)
            {
                if (ctx->info.pic_header.pic_ccsao_on[comp])
                {
                    if (ctx->info.pic_header.ccsao_lcu_ctrl[comp])
                    {
#if CCSAO_ENHANCEMENT
                        ctx->info.pic_header.pic_ccsao_params[comp].lcu_flag[lcu_pos] = dec_eco_ccsao_lcu_flag(sbac, bs, ctx->info.pic_header.ccsao_set_num[comp]);
#else          
                        ctx->info.pic_header.pic_ccsao_params[comp].lcu_flag[lcu_pos] = dec_eco_ccsao_lcu_flag(sbac, bs);
#endif         
                    }
                    else
                    {
                        ctx->info.pic_header.pic_ccsao_params[comp].lcu_flag[lcu_pos] = TRUE;
                    }
                }
            }
        }
#endif

        for (int comp_idx = 0; comp_idx < N_C; comp_idx++)
        {
            int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
            if (ctx->pic_alf_on[comp_idx])
            {
                ctx->dec_alf->alf_lcu_enabled[lcu_pos][comp_idx] = dec_eco_alf_lcu_ctrl(bs, sbac);
            }
            else
            {
                ctx->dec_alf->alf_lcu_enabled[lcu_pos][comp_idx] = FALSE;
            }
        }

#if NN_FILTER
        if (ctx->info.sqh.nnlf_enable_flag)
        {
            int lcu_pos = core->x_lcu + core->y_lcu * ctx->info.pic_width_in_lcu;
            for (int comp = 0; comp < N_C; comp++)
            {
                if (ctx->info.pic_header.ph_nnlf_adaptive_flag[comp])
                {
                    ctx->info.pic_header.nnlf_lcu_enable_flag[comp][lcu_pos] = dec_eco_nnlf_lcu_enable_flag(bs, sbac, comp);
                    if (ctx->info.pic_header.nnlf_lcu_enable_flag[comp][lcu_pos])
                    {
                        ctx->info.pic_header.nnlf_lcu_set_index[comp][lcu_pos] = dec_eco_nnlf_lcu_set_index(bs, sbac, comp, ctx->info.sqh.num_of_nnlf);
                    }
                    else
                    {
                        ctx->info.pic_header.nnlf_lcu_set_index[comp][lcu_pos] = -1;
                    }
                }
                else
                {
                    ctx->info.pic_header.nnlf_lcu_set_index[comp][lcu_pos] = ctx->info.pic_header.ph_nnlf_set_index[comp];
                }
            }
        }
#endif

#if USE_SP 
        //initiate n_recent_pv
        if (core->x_lcu * ctx->info.max_cuwh == patch->left_pel)
        {
            core->n_pv_num = 0;
            core->LcuRx0 = patch->left_pel;
            core->LcuRy0 = core->y_pel;
            com_mset_x64a(core->n_recent_pv, -1, sizeof(COM_MOTION)*MAX_SRB_PRED_SIZE);
            com_mset_x64a(core->n_recent_all_comp_flag, 0, sizeof(u8)*MAX_SRB_PRED_SIZE);
        }
#endif
        /* invoke coding_tree() recursion */
#if CTU_256
        com_mset(core->split_mode, 0, sizeof(s8) * MAX_CU_DEPTH2 * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
#else
        com_mset(core->split_mode, 0, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
#endif
        ret = dec_eco_tree(ctx, core, core->x_pel, core->y_pel, ctx->info.log2_max_cuwh, ctx->info.log2_max_cuwh, 0, 0, bs, sbac
                           , NO_SPLIT, 0, 0, NO_MODE_CONS, TREE_LC);
        com_assert_g(COM_SUCCEEDED(ret), ERR);
        /* set split flags to map */
#if CTU_256
        com_mcpy(ctx->map.map_split[core->lcu_num], core->split_mode, sizeof(s8) * MAX_CU_DEPTH2 * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
#else
        com_mcpy(ctx->map.map_split[core->lcu_num], core->split_mode, sizeof(s8) * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU);
#endif
        /* read end_of_picture_flag */
#if PATCH
        /* aec_lcu_stuffing_bit and byte alignment for bit = 1 case inside the function */
        dec_sbac_decode_bin_trm(bs, sbac);
        ctx->lcu_cnt--;
        if (ctx->lcu_cnt <= 0)
        {
            /*update and store map_scu*/
            de_copy_lcu_scu(map_scu_temp, ctx->map.map_scu, map_refi_temp, ctx->map.map_refi, map_mv_temp, ctx->map.map_mv, map_cu_mode_temp, ctx->map.map_cu_mode, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP
                , map_usp_temp, ctx->map.map_usp
#endif    
            );
            break;
        }
#else
        if (dec_sbac_decode_bin_trm(bs, sbac))
        {
            break;
        }
#endif
        core->x_lcu++;
#if PATCH
        /*prepare next patch*/
        if (core->x_lcu >= *(patch->width_in_lcu + patch->x_pat) + patch_cur_lcu_x)
        {
            core->x_lcu = patch_cur_lcu_x;
            core->y_lcu++;
            if (core->y_lcu >= *(patch->height_in_lcu + patch->y_pat) + patch_cur_lcu_y)
            {
                /*decode patch end*/
                ret = dec_eco_send(bs);
                com_assert_rv(ret == COM_OK, ret);
                while (com_bsr_next(bs, 24) != 0x1)
                {
                    com_bsr_read(bs, 8);
                };
                /*decode patch head*/
                ret = dec_eco_patch_header(bs, sqh, ph, shext,patch);
                last_lcu_qp = shext->slice_qp;
                last_lcu_delta_qp = 0;
                com_assert_rv(ret == COM_OK, ret);
                /*update and store map_scu*/
                de_copy_lcu_scu(map_scu_temp, ctx->map.map_scu, map_refi_temp, ctx->map.map_refi, map_mv_temp, ctx->map.map_mv, map_cu_mode_temp, ctx->map.map_cu_mode, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP    
                    , map_usp_temp, ctx->map.map_usp
#endif    
                );
                com_mset_x64a(ctx->map.map_scu, 0, sizeof(u32)* ctx->info.f_scu);
                com_mset_x64a(ctx->map.map_refi, -1, sizeof(s8)* ctx->info.f_scu * REFP_NUM);
                com_mset_x64a(ctx->map.map_mv, 0, sizeof(s16)* ctx->info.f_scu * REFP_NUM * MV_D);
                com_mset_x64a(ctx->map.map_cu_mode, 0, sizeof(u32) * ctx->info.f_scu);
#if USE_SP
                com_mset_x64a(ctx->map.map_usp, 0, sizeof(u8)*ctx->info.f_scu);
#endif
                /* reset SBAC */
                dec_sbac_init(bs);
                com_sbac_ctx_init(&(sbac->ctx));
                /*set patch location*/
                patch->x_pat = patch->idx % patch->columns;
                patch->y_pat = patch->idx / patch->columns;
                core->x_lcu = core->y_lcu = 0;
                /*set location at above-left of patch*/
                for (int i = 0; i < patch->x_pat; i++)
                {
                    core->x_lcu += *(patch->width_in_lcu + i);
                }
                for (int i = 0; i < patch->y_pat; i++)
                {
                    core->y_lcu += *(patch->height_in_lcu + i);
                }
                patch_cur_lcu_x = core->x_lcu;
                patch_cur_lcu_y = core->y_lcu;
                core->x_pel = core->x_lcu << ctx->info.log2_max_cuwh;
                core->y_pel = core->y_lcu << ctx->info.log2_max_cuwh;
                patch->x_pel = core->x_pel;
                patch->y_pel = core->y_pel;
                /*reset the patch boundary*/
                patch->left_pel = patch->x_pel;
                patch->up_pel = patch->y_pel;
                patch->right_pel = patch->x_pel + (*(patch->width_in_lcu + patch->x_pat) << ctx->info.log2_max_cuwh);
                patch->down_pel = patch->y_pel + (*(patch->height_in_lcu + patch->y_pat) << ctx->info.log2_max_cuwh);
            }
        }
#else
        if (core->x_lcu == ctx->info.pic_width_in_lcu)
        {
            core->x_lcu = 0;
            core->y_lcu++;
        }
#endif
        core->x_pel = core->x_lcu << ctx->info.log2_max_cuwh;
        core->y_pel = core->y_lcu << ctx->info.log2_max_cuwh;
#if PATCH
        core->lcu_num = core->x_lcu + core->y_lcu*ctx->info.pic_width_in_lcu;
#else
        core->lcu_num++;
#endif
    }
#if PATCH
    /*get scu from storage*/
    patch->left_pel = 0;
    patch->up_pel = 0;
    patch->right_pel = ctx->info.pic_width;
    patch->down_pel = ctx->info.pic_height;
    de_copy_lcu_scu(ctx->map.map_scu, map_scu_temp, ctx->map.map_refi, map_refi_temp, ctx->map.map_mv, map_mv_temp, ctx->map.map_cu_mode, map_cu_mode_temp, ctx->patch, ctx->info.pic_width, ctx->info.pic_height
#if USE_SP
        , ctx->map.map_usp, map_usp_temp
#endif
    );
#endif

#if CHECK_ALL_CTX
    {
        static char check_point[sizeof(COM_SBAC_CTX) / sizeof(SBAC_CTX_MODEL)];
        static int fst_frm = 1;
        int i, num = sizeof(COM_SBAC_CTX) / sizeof(SBAC_CTX_MODEL);
        SBAC_CTX_MODEL *p = (SBAC_CTX_MODEL*)(&GET_SBAC_DEC(bs)->ctx);

        if (fst_frm == 1)
        {
            fst_frm = 0;
            memset(check_point, 0, num);
        }
        for (i = 0; i < num; i++)
        {
            unsigned long value;
            int check_val = (p[i] >> 16) & 0xFFFF;
            _BitScanReverse(&value, check_val);

            //if (i % 50 == 0) printf("\n");
            if (check_val && value > 0)
            {
                check_point[i] = 1;
            }
            //printf("%3d", value);
        }

#define CHECK_ONE_VAR(x) {                                              \
            int start_idx = offsetof(COM_SBAC_CTX, x) / 4;              \
            int i, length = sizeof(*(&GET_SBAC_DEC(bs)->ctx.x)) / 4;    \
            printf("\n%20s(%3d,%2d) : ", #x, start_idx, length);        \
            for (i = start_idx; i < start_idx + length; i++) {          \
                printf("%d ", check_point[i]);                          \
            }                                                           \
        }
        CHECK_ONE_VAR(skip_flag);
        CHECK_ONE_VAR(skip_idx_ctx);
        CHECK_ONE_VAR(direct_flag);
        CHECK_ONE_VAR(umve_flag);
        CHECK_ONE_VAR(umve_base_idx);
        CHECK_ONE_VAR(umve_step_idx);
        CHECK_ONE_VAR(umve_dir_idx);
        CHECK_ONE_VAR(inter_dir);
        CHECK_ONE_VAR(intra_dir);
        CHECK_ONE_VAR(pred_mode);
        CHECK_ONE_VAR(cons_mode);
        CHECK_ONE_VAR(ipf_flag);
        CHECK_ONE_VAR(refi);
        CHECK_ONE_VAR(mvr_idx);
        CHECK_ONE_VAR(affine_mvr_idx);
        CHECK_ONE_VAR(mvp_from_hmvp_flag);
        CHECK_ONE_VAR(mvd);
        CHECK_ONE_VAR(ctp_zero_flag);
        CHECK_ONE_VAR(cbf);
        CHECK_ONE_VAR(tb_split);
        CHECK_ONE_VAR(run);
        CHECK_ONE_VAR(last1);
        CHECK_ONE_VAR(last2);
        CHECK_ONE_VAR(level);
        CHECK_ONE_VAR(split_flag);
        CHECK_ONE_VAR(bt_split_flag);
        CHECK_ONE_VAR(split_dir);
        CHECK_ONE_VAR(split_mode);
        CHECK_ONE_VAR(affine_flag);
        CHECK_ONE_VAR(affine_mrg_idx);
        CHECK_ONE_VAR(smvd_flag);
        CHECK_ONE_VAR(part_size);
        CHECK_ONE_VAR(sao_merge_flag);
        CHECK_ONE_VAR(sao_mode);
        CHECK_ONE_VAR(sao_offset);
        CHECK_ONE_VAR(alf_lcu_enable);
        CHECK_ONE_VAR(delta_qp);

        //for (i = 0; i < num; i++) {
        //    if (i % 50 == 0) {
        //        printf("\n");
        //    }
        //    else if(i % 10 == 0) {
        //        printf(" (%3d) ",i);
        //    }
        //    printf("%2d", check_point[i]);
        //}
    }
#endif

    /*decode patch end*/
    ret = dec_eco_send(bs);
    while (com_bsr_next(bs, 24) != 0x1)
    {
        com_bsr_read(bs, 8);
    }
    com_assert_rv(ret == COM_OK, ret);
    return COM_OK;
ERR:
    return ret;
}
#if PATCH
void de_copy_lcu_scu(u32 * scu_temp, u32 * scu_best, s8(*refi_temp)[REFP_NUM], s8(*refi_best)[REFP_NUM], s16(*mv_temp)[REFP_NUM][MV_D], s16(*mv_best)[REFP_NUM][MV_D], u32 *cu_mode_temp, u32 *cu_mode_best, PATCH_INFO * patch, int pic_width, int pic_height
#if USE_SP
    , u8 * usp_temp, u8 * usp_best
#endif
)
{
    /*get the boundary*/
    int left_scu = patch->left_pel >> MIN_CU_LOG2;
    int right_scu = patch->right_pel >> MIN_CU_LOG2;
    int down_scu = patch->down_pel >> MIN_CU_LOG2;
    int up_scu = patch->up_pel >> MIN_CU_LOG2;
    int width_scu = pic_width >> MIN_CU_LOG2;
    int height_scu = pic_height >> MIN_CU_LOG2;
    /*copy*/
    int scu = up_scu*width_scu + left_scu;
    for (int j = up_scu; j < COM_MIN(height_scu, down_scu); j++)
    {
        int temp = scu;
        for (int i = left_scu; i < COM_MIN(width_scu, right_scu); i++)
        {
            //map_patch_idx[scu]=(s8)patch->idx;
            scu_temp[scu] = scu_best[scu];
            cu_mode_temp[scu] = cu_mode_best[scu];
#if USE_SP
            usp_temp[scu] = usp_best[scu];
#endif
            for (int m = 0; m < REFP_NUM; m++)
            {
                refi_temp[scu][m] = refi_best[scu][m];
            }
            for (int m = 0; m < REFP_NUM; m++)
            {
                for (int n = 0; n < MV_D; n++)
                {
                    mv_temp[scu][m][n] = mv_best[scu][m][n];
                }
            }
            scu++;
        }
        scu = temp;
        scu += width_scu;
    }
}

void dec_set_patch_idx(s8 *map_patch_idx, PATCH_INFO * patch, int pic_width, int pic_height)
{
    /*get the boundary*/
    int left_scu = patch->left_pel >> MIN_CU_LOG2;
    int right_scu = patch->right_pel >> MIN_CU_LOG2;
    int down_scu = patch->down_pel >> MIN_CU_LOG2;
    int up_scu = patch->up_pel >> MIN_CU_LOG2;
    int width_scu = pic_width >> MIN_CU_LOG2;
    int height_scu = pic_height >> MIN_CU_LOG2;
    /*set idx*/
    int scu = up_scu*width_scu + left_scu;
    for (int j = up_scu; j < COM_MIN(height_scu, down_scu); j++)
    {
        int temp = scu;
        for (int i = left_scu; i < COM_MIN(width_scu, right_scu); i++)
        {
            map_patch_idx[scu] = (s8)patch->idx;
            scu++;
        }
        scu = temp;
        scu += width_scu;
    }
}
#endif

int dec_ready(DEC_CTX *ctx)
{
    int ret = COM_OK;
    DEC_CORE *core = NULL;
    com_assert(ctx);
    core = core_alloc();
    com_assert_gv(core != NULL, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    ctx->core = core;
    return COM_OK;
ERR:
    if (core)
    {
        core_free(core);
    }
    return ret;
}

void dec_flush(DEC_CTX * ctx)
{
    if (ctx->core)
    {
        core_free(ctx->core);
        ctx->core = NULL;
    }
}

int dec_cnk(DEC_CTX * ctx, COM_BITB * bitb, DEC_STAT * stat)
{
    COM_BSR  *bs;
    COM_PIC_HEADER   *pic_header;
    COM_SQH * sqh;
    COM_SH_EXT *shext;
    COM_CNKH *cnkh;
    int        ret = COM_OK;

    if (stat)
    {
        com_mset(stat, 0, sizeof(DEC_STAT));
    }
    bs  = &ctx->bs;
    sqh = &ctx->info.sqh;
    pic_header = &ctx->info.pic_header;
    shext = &ctx->info.shext;
    cnkh = &ctx->info.cnkh;

    /* set error status */
    ctx->bs_err = (u8)bitb->err;
#if TRACE_RDO_EXCLUDE_I
    if (pic_header->slice_type != SLICE_I)
    {
#endif
        COM_TRACE_SET(1);
#if TRACE_RDO_EXCLUDE_I
    }
    else
    {
        COM_TRACE_SET(0);
    }
#endif
    /* bitstream reader initialization */
    com_bsr_init(bs, bitb->addr, bitb->ssize, NULL);
    SET_SBAC_DEC(bs, &ctx->sbac_dec);

    if (bs->cur[3] == 0xB0)
    {
        cnkh->ctype = COM_CT_SQH;
        ret = dec_eco_sqh(bs, sqh);
        com_assert_rv(COM_SUCCEEDED(ret), ret);
#if LIBVC_ON
        ctx->dpm.libvc_data->is_libpic_processing = sqh->library_stream_flag;
        ctx->dpm.libvc_data->library_picture_enable_flag = sqh->library_picture_enable_flag;
        /*
        if (ctx->dpm.libvc_data->library_picture_enable_flag && !ctx->dpm.libvc_data->is_libpic_prepared)
        {
            ret = COM_ERR_UNEXPECTED;
            printf("\nError: when decode seq.bin with library picture enable, you need to input libpic.bin at the same time by using param: --input_libpics.");
            com_assert_rv(ctx->dpm.libvc_data->library_picture_enable_flag == ctx->dpm.libvc_data->is_libpic_prepared, ret);
        }
        */
#endif
		if (sqh->field_coded_sequence)
		{
			ctx->field_coding = 1;
		}
#if EXTENSION_USER_DATA
        extension_and_user_data(ctx, bs, 0, sqh, pic_header);
#endif
        if( !ctx->init_flag )
        {
            ret = sequence_init(ctx, sqh);
            com_assert_rv(COM_SUCCEEDED(ret), ret);
            g_DOIPrev = g_CountDOICyCleTime = 0;
            ctx->init_flag = 1;
        }
    }
    else if( bs->cur[3] == 0xB1 )
    {
        ctx->init_flag = 0;
        cnkh->ctype = COM_CT_SEQ_END;
    }
    else if (bs->cur[3] == 0xB3 || bs->cur[3] == 0xB6)
    {
        cnkh->ctype = COM_CT_PICTURE;
        /* decode slice header */
        pic_header->low_delay = sqh->low_delay;
        int need_minus_256 = 0;
        ret = dec_eco_pic_header(bs, pic_header, sqh, &need_minus_256);
        if (need_minus_256)
        {
            com_picman_dpbpic_doi_minus_cycle_length( &ctx->dpm );
        }

        ctx->wq[0] = pic_header->wq_4x4_matrix;
        ctx->wq[1] = pic_header->wq_8x8_matrix;

        if (!sqh->library_stream_flag)
        {
            com_picman_check_repeat_doi(&ctx->dpm, pic_header);
        }

#if EXTENSION_USER_DATA && (WRITE_MD5_IN_USER_DATA)
        extension_and_user_data(ctx, bs, 1, sqh, pic_header);
#endif
        com_constrcut_ref_list_doi(pic_header);

        //add by Yuqun Fan, init rpl list at ph instead of sh
#if HLS_RPL
#if LIBVC_ON
        if (!sqh->library_stream_flag)
#endif
        {
            ret = com_picman_refpic_marking_decoder(&ctx->dpm, pic_header);
            com_assert_rv(ret == COM_OK, ret);
        }
        com_cleanup_useless_pic_buffer_in_pm(&ctx->dpm);

        /* reference picture lists construction */
        ret = com_picman_refp_rpl_based_init_decoder(&ctx->dpm, pic_header, ctx->refp);
#if AWP
        if (ctx->info.pic_header.slice_type == SLICE_P || ctx->info.pic_header.slice_type == SLICE_B)
        {
            for (int i = 0; i < ctx->dpm.num_refp[REFP_0]; i++)
            {
                ctx->info.pic_header.ph_poc[REFP_0][i] = ctx->refp[i][REFP_0].ptr;
            }
        }

        if (ctx->info.pic_header.slice_type == SLICE_B)
        {
            for (int i = 0; i < ctx->dpm.num_refp[REFP_1]; i++)
            {
                ctx->info.pic_header.ph_poc[REFP_1][i] = ctx->refp[i][REFP_1].ptr;
            }
        }
#endif
#else
        /* initialize reference pictures */
        //ret = com_picman_refp_init(&ctx->dpm, ctx->info.sqh.num_ref_pics_act, sh->slice_type, ctx->ptr, ctx->info.sh.temporal_id, ctx->last_intra_ptr, ctx->refp);
#endif
        com_assert_rv(COM_SUCCEEDED(ret), ret);

    }
    else if (bs->cur[3] >= 0x00 && bs->cur[3] <= 0x8E)
    {
        cnkh->ctype = COM_CT_SLICE;
        ret = dec_eco_patch_header(bs, sqh, pic_header, shext, ctx->patch);
        /* initialize slice */
        ret = slice_init(ctx, ctx->core, pic_header);
        com_assert_rv(COM_SUCCEEDED(ret), ret);
        /* get available frame buffer for decoded image */
        ctx->pic = com_picman_get_empty_pic(&ctx->dpm, &ret);
        com_assert_rv(ctx->pic, ret);
        /* get available frame buffer for decoded image */
        ctx->map.map_refi = ctx->pic->map_refi;
        ctx->map.map_mv = ctx->pic->map_mv;
        /* decode slice layer */
        ret = dec_pic(ctx, ctx->core, sqh, pic_header, shext);
        com_assert_rv(COM_SUCCEEDED(ret), ret);
        /* deblocking filter */
        if (ctx->info.pic_header.loop_filter_disable_flag == 0)
        {
            ret = dec_deblock_avs2(ctx);
            com_assert_rv(COM_SUCCEEDED(ret), ret);
        }
#if CCSAO
        if (ctx->info.pic_header.pic_ccsao_on[U_C-1] || ctx->info.pic_header.pic_ccsao_on[V_C-1])
        {
#if CCSAO_ENHANCEMENT
            copy_frame_for_ccsao(ctx->pic_ccsao[0], ctx->pic, Y_C);
            copy_frame_for_ccsao(ctx->pic_ccsao[0], ctx->pic, U_C);
            copy_frame_for_ccsao(ctx->pic_ccsao[0], ctx->pic, V_C);
#else
            copy_frame_for_ccsao(ctx->pic_ccsao, ctx->pic, Y_C);
#endif
        }
#endif
        /* sao filter */
        if (ctx->info.sqh.sample_adaptive_offset_enable_flag)
        {
            ret = dec_sao_avs2(ctx);
            com_assert_rv(ret == COM_OK, ret);
        }
        /* esao filter */
#if ESAO
        if (ctx->info.sqh.esao_enable_flag)
        {
            ret = dec_esao(ctx);
            com_assert_rv(ret == COM_OK, ret);
        }
#endif
#if CCSAO
        /* ccsao filter */
        if (ctx->info.sqh.ccsao_enable_flag)
        {
            ret = dec_ccsao(ctx);
            com_assert_rv(ret == COM_OK, ret);
        }
#endif
        /* ALF */
        if (ctx->info.sqh.adaptive_leveling_filter_enable_flag)
        {
            ret = dec_alf_avs2(ctx, ctx->pic);
            com_assert_rv(COM_SUCCEEDED(ret), ret);
        }
        /* MD5 check for testing encoder-decoder match*/
        if (ctx->use_pic_sign && ctx->pic_sign_exist)
        {
            ret = dec_picbuf_check_signature(ctx->pic, ctx->pic_sign);
            com_assert_rv(COM_SUCCEEDED(ret), ret);
            ctx->pic_sign_exist = 0; /* reset flag */
        }
#if PIC_PAD_SIZE_L > 0
        /* expand pixels to padding area */
        dec_picbuf_expand(ctx, ctx->pic);
#endif
        /* put decoded picture to DPB */
#if LIBVC_ON
        if (sqh->library_stream_flag)
        {
            ret = com_picman_put_libpic(&ctx->dpm, ctx->pic, ctx->info.pic_header.slice_type, ctx->ptr, pic_header->decode_order_index, ctx->info.pic_header.temporal_id, 1, ctx->refp, pic_header);
        }
        else
#endif
        {
            ret = com_picman_put_pic(&ctx->dpm, ctx->pic, ctx->info.pic_header.slice_type, ctx->ptr, pic_header->decode_order_index, 
#if OBMC
#if CUDQP
                                     pic_header->picture_output_delay, ctx->info.pic_header.temporal_id, 1, ctx->refp, pic_header);
#else
                                     pic_header->picture_output_delay, ctx->info.pic_header.temporal_id, 1, ctx->refp, pic_header->picture_qp);
#endif
#else
                                     pic_header->picture_output_delay, ctx->info.pic_header.temporal_id, 1, ctx->refp);
#endif
#if LIBVC_ON
            assert((&ctx->dpm)->cur_pb_size + (&ctx->dpm)->cur_libpb_size <= sqh->max_dpb_size);
#else
            assert((&ctx->dpm)->cur_pb_size <= sqh->max_dpb_size);
#endif
        }
        com_assert_rv(COM_SUCCEEDED(ret), ret);
    }
    else
    {
        return COM_ERR_MALFORMED_BITSTREAM;
    }
    make_stat(ctx, cnkh->ctype, stat);
    return ret;
}

int dec_pull_frm(DEC_CTX *ctx, COM_IMGB **imgb, int state)
{
    int ret;
    COM_PIC *pic;
    *imgb = NULL;
    int library_picture_index;
#if LIBVC_ON
    // output lib pic and corresponding library_picture_index
    if (ctx->info.sqh.library_stream_flag)
    {
        pic = ctx->pic;
        library_picture_index = ctx->info.pic_header.library_picture_index;

        //output to the buffer outside the decoder
        ret = com_picman_out_libpic(pic, library_picture_index, &ctx->dpm);
        if (pic && ret == COM_OK)
        {
            com_assert_rv(pic->imgb != NULL, COM_ERR);
            pic->imgb->addref(pic->imgb);
            *imgb = pic->imgb;
        }

        return ret;
    }
    else
#endif
    {
        pic = com_picman_out_pic( &ctx->dpm, &ret, ctx->info.pic_header.decode_order_index, state ); //MX: doi is not increase mono, but in the range of [0,255]
        if (pic)
        {
            com_assert_rv(pic->imgb != NULL, COM_ERR);
            /* increase reference count */
            pic->imgb->addref(pic->imgb);
            *imgb = pic->imgb;
        }
        return ret;
    }
}

DEC dec_create(DEC_CDSC * cdsc, int * err)
{
    DEC_CTX *ctx = NULL;
    int ret;
#if ENC_DEC_TRACE
    if( fp_trace == NULL )
    {
        fopen_s( &fp_trace, "dec_trace.txt", "w+" );
    }
#endif
    ctx = ctx_alloc();
    com_assert_gv(ctx != NULL, ret, COM_ERR_OUT_OF_MEMORY, ERR);
    com_mcpy(&ctx->cdsc, cdsc, sizeof(DEC_CDSC));
#if USE_SP
    ret = com_sp_init();
    com_assert_g(ret == COM_OK, ERR);
#endif
    ret = com_scan_tbl_init();
    com_assert_g(ret == COM_OK, ERR);
    ret = dec_ready(ctx);
    com_assert_g(ret == COM_OK, ERR);
    /* Set CTX variables to default value */
    ctx->magic = DEC_MAGIC_CODE;
    ctx->id = (DEC)ctx;
    ctx->init_flag = 0;

#if FIMC
    com_cntmpm_init(&g_cntMpmInitTable); // dec init 
#endif
#if BGC
    ctx->info.pred_tmp = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
#endif
#if OBMC
    ctx->info.pred_tmp_c[0] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_tmp_c[1] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.subblk_obmc_buf[Y_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.subblk_obmc_buf[U_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.subblk_obmc_buf[V_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_buf_snd[Y_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_buf_snd[U_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    ctx->info.pred_buf_snd[V_C] = (pel *)malloc(MAX_CU_DIM * sizeof(pel));
    const double PI = 3.14159265358979323846;
    double w = 0;
#if CTU_256
    int blk_num = 8;
#else
    int blk_num = 7;
#endif
#if OBMC_TEMP
    ctx->info.obmc_weight = (pel ***)calloc(5, sizeof(pel **));
    for (int i = 0; i < 5; i++)
    {
        *(ctx->info.obmc_weight + i) = (pel **)malloc(blk_num * sizeof(pel *));
        for (int j = 0; j < blk_num; j++)
        {
            int blkSize = (1 << j);
            *(*(ctx->info.obmc_weight + i) + j) = (pel *)malloc(blkSize * sizeof(pel));
            for (int s = 0; s < blkSize; s++)
            {
            if (i == 0)
                {

					w = 0.125 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.875;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
                if (i == 1)
                {

					w = 0.375 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.625;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
                if (i == 2)
                {

					 w = 0.25 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.75;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
                if (i == 3)
                {

					w = 0.3125 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.6875;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
                if (i == 4)
                {

					w = 0.4375 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.5625;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }

            }
        }
    }
#else
    ctx->info.obmc_weight = (pel ***)calloc(2, sizeof(pel **));
    for (int i = 0; i < 2; i++)
    {
        *(ctx->info.obmc_weight + i) = (pel **)malloc(blk_num * sizeof(pel *));
        for (int j = 0; j < blk_num; j++)
        {
            int blkSize = (1 << j);
            *(*(ctx->info.obmc_weight + i) + j) = (pel *)malloc(blkSize * sizeof(pel));
            for (int s = 0; s < blkSize; s++)
            {
                if (i == 0)
                {
                    w = 0.125 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.875;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
                else
                {
                    w = 0.375 * cos(PI / 2.0 / (double)blkSize * (blkSize - s - 0.5)) + 0.625;
                    *(*(*(ctx->info.obmc_weight + i) + j) + s) = (pel)(64.0 * w + 0.5);
                }
            }
        }
    }
#endif    
#endif
    init_dct_coef();
    return (ctx->id);
ERR:
    if (ctx)
    {
        dec_flush(ctx);
        ctx_free(ctx);
    }
    if (err) *err = ret;
    return NULL;
}

void dec_delete(DEC id)
{
    DEC_CTX *ctx;
    DEC_ID_TO_CTX_R(id, ctx);
    sequence_deinit(ctx);
#if LIBVC_ON
    ctx->dpm.libvc_data = NULL;
#endif
#if BGC
    if (ctx->info.pred_tmp)
    {
        free( ctx->info.pred_tmp );
        ctx->info.pred_tmp = NULL;
    }
#endif
#if OBMC
    for (int i = 0; i < 2; i++)
    {
        if (ctx->info.pred_tmp_c[1])
        {
            free(ctx->info.pred_tmp_c[i]);
            ctx->info.pred_tmp_c[i] = NULL;
        }
    }
    for (int i = 0; i < N_C; i++)
    {
        if (ctx->info.subblk_obmc_buf[i])
        {
            free(ctx->info.subblk_obmc_buf[i]);
            ctx->info.subblk_obmc_buf[i] = NULL;
        }
        if (ctx->info.pred_buf_snd[i])
        {
            free(ctx->info.pred_buf_snd[i]);
            ctx->info.pred_buf_snd[i] = NULL;
        }
    }
    if (ctx->info.obmc_weight)
    {
        int blk_num = 7;
#if OBMC_TEMP
        for (int i = 0; i < 5; i++)
#else
        for (int i = 0; i < 2; i++)
#endif
        {
            for (int j = 0; j < blk_num; j++)
            {
                free(ctx->info.obmc_weight[i][j]);
                ctx->info.obmc_weight[i][j] = NULL;
            }
            free(ctx->info.obmc_weight[i]);
            ctx->info.obmc_weight[i] = NULL;
        }
        free(ctx->info.obmc_weight);
        ctx->info.obmc_weight = NULL;
    }
#endif
    dec_flush(ctx);
    ctx_free(ctx);
    com_scan_tbl_delete();
#if USE_SP
    com_sp_delete();
#endif
}

int dec_config(DEC id, int cfg, void * buf, int * size)
{
    DEC_CTX *ctx;
    DEC_ID_TO_CTX_RV(id, ctx, COM_ERR_INVALID_ARGUMENT);
    switch (cfg)
    {
    /* set config ************************************************************/
    case DEC_CFG_SET_USE_PIC_SIGNATURE:
        ctx->use_pic_sign = (*((int *)buf)) ? 1 : 0;
        break;
    /* get config ************************************************************/
    default:
        com_assert_rv(0, COM_ERR_UNSUPPORTED);
    }
    return COM_OK;
}

int dec_decode(DEC id, COM_BITB * bitb, DEC_STAT * stat)
{
    return 0;
}