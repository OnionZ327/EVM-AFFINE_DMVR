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

#define _CRT_SECURE_NO_WARNINGS


#include <com_typedef.h>
#include "app_util.h"
#include "app_args.h"
#include "com_def.h"
#include "dec_def.h"
#include "dec_DecAdaptiveLoopFilter.h"

#include "dec_eco.h"
#include "enc_eco.h"
#include "stdio.h"
#include "com_df.h"
#include "com_sao.h"

int g_CountDOICyCleTime;                    // number to count the DOI cycle time.
int g_DOIPrev;                              // the doi of previous frm.

#define VERBOSE_FRAME              VERBOSE_1

#define MAX_BS_BUF                 (64*1024*1024) /* byte */

static char op_fname_inp[256] = "\0";
static char op_fname_out[256] = "\0";
static int  op_max_frm_num = 0;
static int  op_use_pic_signature = 0;
static int  op_clip_org_size = 0;
static int  op_bit_depth_output = 8;
#if LIBVC_ON
static int  op_merge_libandseq_bitstream_flag = 0;
static char op_fname_inp_lib[256] = "\0";
static char op_fname_inp_seq[256] = "\0";
#endif
#if BBV
static int  op_bbv_check_flag = 0;
#if BBV_LIBVC
static int  op_bbv_check_merge_steam = 0;
#endif
static int  op_target_fps = 0;
typedef enum _BBV_STATES
{
    BBV_OK = 0,
    BBV_OVERFLOW,
    BBV_DOWNFLOW,
    BBV_DELAY_WRONG,
    BBV_INPUT_SPEED_OVERFLOW
}BBV_STATES;
#endif

typedef enum _STATES
{
    STATE_DECODING,
    STATE_BUMPING
} STATES;

typedef enum _OP_FLAGS
{
    OP_FLAG_FNAME_INP,
    OP_FLAG_FNAME_OUT,
    OP_FLAG_MAX_FRM_NUM,
    OP_FLAG_USE_PIC_SIGN,
    OP_FLAG_CLIP_ORG_SIZE,
    OP_FLAG_OUT_BIT_DEPTH,
#if LIBVC_ON
    OP_FLAG_MERGE_LIBANDSEQ_BITSTREAM,
    OP_FLAG_FNAME_INP_LIB,
    OP_FLAG_FNAME_INP_SEQ,
#endif
#if BBV
    OP_FLAG_BBV_CHECK,
#if BBV_LIBVC
    OP_FLAG_BBV_CHECK_MERGE_STREAM,
#endif
#endif
    OP_FLAG_TARGET_FPS,
    OP_FLAG_VERBOSE,
    OP_FLAG_MAX
} OP_FLAGS;

static int op_flag[OP_FLAG_MAX] = { 0 };

static COM_ARGS_OPTION options[] = \
{
    {
#if LIBVC_ON
        'i', "input", ARGS_TYPE_STRING,
#else
        'i', "input", ARGS_TYPE_STRING | ARGS_TYPE_MANDATORY,
#endif
            &op_flag[OP_FLAG_FNAME_INP], op_fname_inp,
            "file name of input bitstream"
    },
    {
        'o', "output", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_OUT], op_fname_out,
        "file name of decoded output"
    },
    {
        't', "tfps", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_TARGET_FPS], &op_target_fps,
        "traget fps"
    },
    {
        'f', "frames", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_MAX_FRM_NUM], &op_max_frm_num,
        "maximum number of frames to be decoded"
    },
    {
        's', "signature", COM_ARGS_VAL_TYPE_NONE,
        &op_flag[OP_FLAG_USE_PIC_SIGN], &op_use_pic_signature,
        "conformance check using picture signature (HASH)"
    },
    {   'c', "clip_org_size", COM_ARGS_VAL_TYPE_NONE,         
        &op_flag[OP_FLAG_CLIP_ORG_SIZE], &op_clip_org_size,
        "clip the output size to the original size (the input size at the encoder side)" 
    },
    {
        'v', "verbose", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_VERBOSE], &op_verbose,
        "verbose level\n"
        "\t 0: no message\n"
        "\t 1: frame-level messages (default)\n"
        "\t 2: all messages\n"
    },
    {
        COM_ARGS_NO_KEY, "output_bit_depth", ARGS_TYPE_INTEGER,
        &op_flag[OP_FLAG_OUT_BIT_DEPTH], &op_bit_depth_output,
        "output bitdepth (8(default), 10) "
    },
#if LIBVC_ON
        {
            COM_ARGS_NO_KEY, "merge_libandseq_bitstream", ARGS_TYPE_INTEGER,
            &op_flag[OP_FLAG_MERGE_LIBANDSEQ_BITSTREAM], &op_merge_libandseq_bitstream_flag,
            "merge_lib and seq_bitstream flag (0(default), 1) "
        },
    {
        COM_ARGS_NO_KEY, "input_lib_bitstream", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_INP_LIB], &op_fname_inp_lib,
        "file name of input lib bitstream"
    },
    {
        COM_ARGS_NO_KEY, "input_seq_bitstream", ARGS_TYPE_STRING,
        &op_flag[OP_FLAG_FNAME_INP_SEQ], &op_fname_inp_seq,
        "file name of input seq bitstream"
    },
#endif
#if BBV
        {
            COM_ARGS_NO_KEY, "bbv_check", ARGS_TYPE_INTEGER,
            &op_flag[OP_FLAG_BBV_CHECK], &op_bbv_check_flag,
            "bbv check flag (0(default), 1) "
        },
#if BBV_LIBVC
        {
            COM_ARGS_NO_KEY, "bbv_check_merge_stream", ARGS_TYPE_INTEGER,
            &op_flag[OP_FLAG_BBV_CHECK_MERGE_STREAM], &op_bbv_check_merge_steam,
            "whether the stream to do bbv check is a merge stream (0(default), 1) "
        },
#endif
#endif
        { 0, "", COM_ARGS_VAL_TYPE_NONE, NULL, NULL, "" } /* termination */
};

static void sequence_deinit(DEC_CTX * ctx)
{
    if (ctx->map.map_scu)
    {
        com_mfree(ctx->map.map_scu);
        ctx->map.map_scu = NULL;
    }
    if (ctx->map.map_split)
    {
        com_mfree(ctx->map.map_split);
        ctx->map.map_split = NULL;
    }
    if (ctx->map.map_ipm)
    {
        com_mfree(ctx->map.map_ipm);
        ctx->map.map_ipm = NULL;
    }
    if (ctx->map.map_cu_mode)
    {
        com_mfree(ctx->map.map_cu_mode);
        ctx->map.map_cu_mode = NULL;
    }
#if TB_SPLIT_EXT
    if (ctx->map.map_pb_tb_part)
    {
        com_mfree(ctx->map.map_pb_tb_part);
        ctx->map.map_pb_tb_part = NULL;
    }
#endif
    if (ctx->map.map_depth)
    {
        com_mfree(ctx->map.map_depth);
        ctx->map.map_depth = NULL;
    }
    if (ctx->map.map_patch_idx)
    {
        com_mfree(ctx->map.map_patch_idx);
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

    if (ctx->pic_sao)
    {
        com_picbuf_free(ctx->pic_sao);
        ctx->pic_sao = NULL;
    }
#if ESAO
    if (ctx->pic_esao)
    {
        com_picbuf_free(ctx->pic_esao);
        ctx->pic_esao = NULL;
    }
#if ESAO_PH_SYNTAX
    com_mfree(ctx->info.pic_header.pic_esao_params[Y_C].lcu_flag);
    com_mfree(ctx->info.pic_header.pic_esao_params[U_C].lcu_flag);
    com_mfree(ctx->info.pic_header.pic_esao_params[V_C].lcu_flag);
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
    com_mfree(ctx->info.pic_header.pic_ccsao_params[U_C - 1].lcu_flag);
    com_mfree(ctx->info.pic_header.pic_ccsao_params[V_C - 1].lcu_flag);
#else
    com_mfree(ctx->pic_ccsao_params[U_C - 1].lcu_flag);
    com_mfree(ctx->pic_ccsao_params[V_C - 1].lcu_flag);
#endif
#endif
    com_free_3d_sao_stat_data(&ctx->sao_stat_data,
        ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
            ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_free_2d_sao_param(&ctx->sao_blk_params,
        ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
            ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));
    com_free_2d_sao_param(&ctx->rec_sao_blk_params,
        ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
            ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)));

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
}
static int sequence_init(DEC_CTX * ctx, COM_SQH * sqh)
{
    int size;
    int ret;

    ctx->info.bit_depth_internal = (sqh->encoding_precision == 2) ? 10 : 8;
    assert(sqh->sample_precision == 1 || sqh->sample_precision == 2);
    ctx->info.bit_depth_input = (sqh->sample_precision == 1) ? 8 : 10;
    ctx->info.qp_offset_bit_depth = (8 * (ctx->info.bit_depth_internal - 8));

    /* resolution was changed */
    sequence_deinit( ctx );
    ctx->info.pic_width  = (sqh->horizontal_size  + MINI_SIZE - 1) / MINI_SIZE * MINI_SIZE;
    ctx->info.pic_height = (sqh->vertical_size    + MINI_SIZE - 1) / MINI_SIZE * MINI_SIZE;
    ctx->info.max_cuwh = 1 << sqh->log2_max_cu_width_height;
    ctx->info.log2_max_cuwh = CONV_LOG2( ctx->info.max_cuwh );

    size = ctx->info.max_cuwh;
    ctx->info.pic_width_in_lcu = (ctx->info.pic_width + (size - 1)) / size;
    ctx->info.pic_height_in_lcu = (ctx->info.pic_height + (size - 1)) / size;
    ctx->info.f_lcu = ctx->info.pic_width_in_lcu * ctx->info.pic_height_in_lcu;
    ctx->info.pic_width_in_scu = (ctx->info.pic_width + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->info.pic_height_in_scu = (ctx->info.pic_height + ((1 << MIN_CU_LOG2) - 1)) >> MIN_CU_LOG2;
    ctx->info.f_scu = ctx->info.pic_width_in_scu * ctx->info.pic_height_in_scu;
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
        size = sizeof(s8) * ctx->info.f_lcu * MAX_CU_DEPTH * NUM_BLOCK_SHAPE * MAX_CU_CNT_IN_LCU;
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
    if (ctx->map.map_depth == NULL)
    {
        size = sizeof(s8) * ctx->info.f_scu;
        ctx->map.map_depth = com_malloc_fast(size);
        com_assert_gv(ctx->map.map_depth, ret, COM_ERR_OUT_OF_MEMORY, ERR);
        com_mset(ctx->map.map_depth, -1, size);
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
    ret = com_picman_init(&ctx->dpm, MAX_PB_SIZE, MAX_NUM_REF_PICS, &ctx->pa);
    com_assert_g(COM_SUCCEEDED(ret), ERR);

    create_edge_filter_avs2(ctx->info.pic_width, ctx->info.pic_height, ctx->edge_filter);

    if (ctx->pic_sao == NULL)
    {
        ctx->pic_sao = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_sao != NULL, ret);
    }
#if ESAO
    if (ctx->pic_esao == NULL)
    {
        ctx->pic_esao = com_pic_alloc(&ctx->pa, &ret);
        com_assert_rv(ctx->pic_esao != NULL, ret);
    }
#if ESAO_PH_SYNTAX
    if (ctx->info.pic_header.pic_esao_params==NULL)
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
    com_malloc_3d_sao_stat_data(&(ctx->sao_stat_data),
        ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
            ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C, NUM_SAO_NEW_TYPES);
    com_malloc_2d_sao_param(&(ctx->sao_blk_params),
        ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
            ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);
    com_malloc_2d_sao_param(&ctx->rec_sao_blk_params,
        ((ctx->info.pic_width >> ctx->info.log2_max_cuwh) + (ctx->info.pic_width % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)) * ((
            ctx->info.pic_height >> ctx->info.log2_max_cuwh) + (ctx->info.pic_height % (1 << ctx->info.log2_max_cuwh) ? 1 : 0)), N_C);

#if ALF_SHAPE || ALF_IMP || ALF_SHIFT
    ctx->info.pic_header.tool_alf_enhance_on = ctx->info.sqh.adaptive_leveling_filter_enhance_flag;
#endif
    ctx->info.pic_header.tool_alf_on = ctx->info.sqh.adaptive_leveling_filter_enable_flag;
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
    /*initialize patch info */
#if PATCH
    PATCH_INFO * patch = NULL;
    patch = (PATCH_INFO *)com_malloc_fast(sizeof(PATCH_INFO));
    ctx->patch = patch;
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
                    ctx->patch_row_height[ctx->patch->rows - 1] = ctx->patch_row_height[ctx->patch->rows - 1] + ctx->info.pic_height_in_lcu - ctx->patch->height*ctx->patch->rows;
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
#endif

    return COM_OK;
ERR:
    sequence_deinit(ctx);
    ctx->init_flag = 0;
    return ret;
}
#define NUM_ARG_OPTION   ((int)(sizeof(options)/sizeof(options[0]))-1)
static void print_usage(void)
{
    int i;
    char str[1024];
    v0print("< Usage >\n");
    for (i = 0; i < NUM_ARG_OPTION; i++)
    {
        if (com_args_get_help(options, i, str) < 0) return;
        v0print("%s\n", str);
    }
}

static int xFindNextStartCode(FILE * fp, int * ruiPacketSize, unsigned char *pucBuffer)
{
    unsigned int uiDummy = 0;
    char bEndOfStream = 0;
    size_t ret = 0;
    ret = fread(&uiDummy, 1, 2, fp);
    if (ret != 2)
    {
        return -1;
    }

    if (feof(fp))
    {
        return -1;
    }
    assert(0 == uiDummy);

    ret = fread(&uiDummy, 1, 1, fp);
    if (ret != 1)
    {
        return -1;
    }
    if (feof(fp))
    {
        return -1;
    }
    assert(1 == uiDummy);

    int iNextStartCodeBytes = 0;
    unsigned int iBytesRead = 0;
    unsigned int uiZeros = 0;

    unsigned char pucBuffer_Temp[16];
    int iBytesRead_Temp = 0;
    pucBuffer[iBytesRead++] = 0x00;
    pucBuffer[iBytesRead++] = 0x00;
    pucBuffer[iBytesRead++] = 0x01;
    while (1)
    {
        unsigned char ucByte = 0;
        ret = fread(&ucByte, 1, 1, fp);

        if (feof(fp))
        {
            iNextStartCodeBytes = 0;
            bEndOfStream = 1;
            break;
        }
        pucBuffer[iBytesRead++] = ucByte;
        if (1 < ucByte)
        {
            uiZeros = 0;
        }
        else if (0 == ucByte)
        {
            uiZeros++;
        }
        else if (uiZeros > 1)
        {
            iBytesRead_Temp = 0;
            pucBuffer_Temp[iBytesRead_Temp] = ucByte;

            iBytesRead_Temp++;
            ret = fread(&ucByte, 1, 1, fp);
            if (ret != 1)
            {
                return -1;
            }
            pucBuffer_Temp[iBytesRead_Temp] = ucByte;
            //           iBytesRead = iBytesRead + 1;
            pucBuffer[iBytesRead++] = ucByte;
            iBytesRead_Temp++;
#if LIBVC_ON
            if (pucBuffer_Temp[0] == 0x01 && (pucBuffer_Temp[1] == 0xb3 ||
                pucBuffer_Temp[1] == 0xb6 || pucBuffer_Temp[1] == 0xb0 || pucBuffer_Temp[1] == 0xb1 || pucBuffer_Temp[1] == 0xb5
                || pucBuffer_Temp[1] == 0x00))
            {
#else
            if (pucBuffer_Temp[0] == 0x01 && (pucBuffer_Temp[1] == 0xb3 ||
                pucBuffer_Temp[1] == 0xb6 || pucBuffer_Temp[1] == 0xb0
                || pucBuffer_Temp[1] == 0x00))
            {
#endif
                iNextStartCodeBytes = 2 + 1 + 1;

                uiZeros = 0;
                break;
            }
            else
            {
                uiZeros = 0;
                iNextStartCodeBytes = 0;// 2 + 1 + 1;

            }

        }
        else
        {
            uiZeros = 0;
        }
    }

    *ruiPacketSize = iBytesRead - iNextStartCodeBytes;

    if (bEndOfStream)
    {
        return 0;
    }
    if (fseek(fp, -1 * iNextStartCodeBytes, SEEK_CUR) != 0)
    {
        printf("file seek failed!\n");
    };

    return 0;
    }


static unsigned int  initParsingConvertPayloadToRBSP(const unsigned int uiBytesRead, unsigned char* pBuffer, unsigned char* pBuffer2)
{
    unsigned int uiZeroCount = 0;
    unsigned int uiBytesReadOffset = 0;
    unsigned int uiBitsReadOffset = 0;
    const unsigned char *pucRead = pBuffer;
    unsigned char *pucWrite = pBuffer2;

    unsigned char ucHeaderType;
    unsigned int uiWriteOffset = uiBytesReadOffset;
    ucHeaderType = pucRead[uiBytesReadOffset++]; //HeaderType



    switch (ucHeaderType)
    {
    case 0xb5:
        if ((pucRead[uiBytesReadOffset] >> 4) == 0x0d)
        {
            break;
        }
        else
        {
        }
    case 0xb0:
    case 0xb2:
        //       initParsing(uiBytesRead);
        memcpy(pBuffer2, pBuffer, uiBytesRead);

        return uiBytesRead;
    default:
        break;
    }
    pucWrite[uiWriteOffset] = ucHeaderType;

    uiWriteOffset++;

    unsigned char ucCurByte = pucRead[uiBytesReadOffset];

    for (; uiBytesReadOffset < uiBytesRead; uiBytesReadOffset++)
    {
        ucCurByte = pucRead[uiBytesReadOffset];

        if (2 <= uiZeroCount && 0x02 == pucRead[uiBytesReadOffset])
        {
            pucWrite[uiWriteOffset] = ((pucRead[uiBytesReadOffset] >> 2) << (uiBitsReadOffset + 2));
            uiBitsReadOffset += 2;
            uiZeroCount = 0;
            //printf("\n error uiBitsReadOffset \n");
            if (uiBitsReadOffset >= 8)
            {
                uiBitsReadOffset = 0;
                continue;
            }
            if (uiBytesReadOffset >= uiBytesRead)
            {
                break;
            }
        }
        else if (2 <= uiZeroCount && 0x01 == pucRead[uiBytesReadOffset])
        {
            uiBitsReadOffset = 0;
            pucWrite[uiWriteOffset] = pucRead[uiBytesReadOffset];
        }
        else
        {
            pucWrite[uiWriteOffset] = (pucRead[uiBytesReadOffset] << uiBitsReadOffset);
        }

        if (uiBytesReadOffset + 1 < uiBytesRead)
        {
            pucWrite[uiWriteOffset] |= (pucRead[uiBytesReadOffset + 1] >> (8 - uiBitsReadOffset));

        }
        uiWriteOffset++;

        if (0x00 == ucCurByte)
        {
            uiZeroCount++;
        }
        else
        {
            uiZeroCount = 0;
        }
    }

    // th just clear the remaining bits in the buffer
    for (unsigned int ui = uiWriteOffset; ui < uiBytesRead; ui++)
    {
        pucWrite[ui] = 0;
    }
    //    initParsing(uiWriteOffset);
    memcpy(pBuffer, pBuffer2, uiWriteOffset);


    return uiBytesRead;
}


static int read_a_bs(FILE * fp, int * pos, unsigned char * bs_buf)
{
    int read_size, bs_size;

    unsigned char b = 0;
    bs_size = 0;
    read_size = 0;
    if (!fseek(fp, *pos, SEEK_SET))
    {
        int ret = 0;
        ret = xFindNextStartCode(fp, &bs_size, bs_buf);
        if (ret == -1)
        {
            v2print("End of file\n");
            return -1;
        }
         read_size = bs_size;
    }
    else
    {
        v0print("Cannot seek bitstream!\n");
        return -1;
    }
    return read_size;
}

int print_stat(DEC_STAT * stat, int ret)
{
    char stype;
    int i, j;
    if (COM_SUCCEEDED(ret))
    {
        if (stat->ctype == COM_CT_PICTURE)
        {
            switch (stat->stype)
            {
            case COM_ST_I:
                stype = 'I';
                break;
            case COM_ST_P:
                stype = 'P';
                break;
            case COM_ST_B:
                stype = 'B';
                break;
            case COM_ST_UNKNOWN:
            default:
                stype = 'U';
                break;
            }
            v1print("%c-slice", stype);
        }
        else if (stat->ctype == COM_CT_SQH)
        {
            v1print("Sequence header");
        }
        else
        {
            v0print("Unknown bitstream");
        }
        v1print(" (read=%d, poc=%d) ", stat->read, (int)stat->poc);
        for (i = 0; i < 2; i++)
        {
            v1print("[L%d ", i);
            for (j = 0; j < stat->refpic_num[i]; j++) v1print("%d ", stat->refpic[i][j]);
            v1print("] ");
        }
        if (ret == COM_OK)
        {
            v1print("\n");
        }
        else if (ret == COM_OK_FRM_DELAYED)
        {
            v1print("->Frame delayed\n");
        }
        else if (ret == COM_OK_DIM_CHANGED)
        {
            v1print("->Resolution changed\n");
        }
        else
        {
            v1print("->Unknown OK code = %d\n", ret);
        }
    }
    else
    {
        v0print("Decoding error = %d\n", ret);
    }
    return 0;
}

static int set_extra_config(DEC id)
{
    int  ret, size, value;
    if (op_use_pic_signature)
    {
        value = 1;
        size = 4;
        ret = dec_config(id, DEC_CFG_SET_USE_PIC_SIGNATURE, &value, &size);
        if (COM_FAILED(ret))
        {
            v0print("failed to set config for picture signature\n");
            return -1;
        }
    }
    return 0;
}

/* convert DEC into DEC_CTX with return value if assert on */
#define DEC_ID_TO_CTX_RV(id, ctx, ret) \
    com_assert_rv((id), (ret)); \
    (ctx) = (DEC_CTX *)id; \
    com_assert_rv((ctx)->magic == DEC_MAGIC_CODE, (ret));

void write_tmp_bs(COM_PIC_HEADER * pic_header, int add_dtr, unsigned char *bs_buf, FILE * merge_fp)
{
    u8 tmp_bs;
    u16 dtr = pic_header->dtr + add_dtr;
    tmp_bs = bs_buf[4] & 0xE0;
    tmp_bs |= (dtr >> 5) & 0x1F;
    fwrite(&tmp_bs, 1, 1, merge_fp);

    tmp_bs = bs_buf[5] & 0x07;
    tmp_bs |= (dtr & 0x1F) << 3;
    fwrite(&tmp_bs, 1, 1, merge_fp);
}

int main(int argc, const char **argv)
{
    /*
      print useage
    */
    printf("*******************Merge bitstream, parameter \"-t\" must put in the last position!******************************\n");
    printf("usage: bitstream_downsample.exe -i str1.bin -o str.bin -t 30\n");
    printf("***************************************************************************************************************\n");

#if TEMPORAL_SCALABILITY_EXTENSION

    unsigned char    * bs_buf = NULL;
    DEC              id = NULL;
    DEC_CDSC         cdsc;
    COM_BITB          bitb;
    /*temporal buffer for video bit depth less than 10bit */
    COM_IMGB        * imgb_t = NULL;
    int                ret;
    int                bs_size, bs_read_pos = 0;
    FILE             * fp_bs = NULL;
    FILE             * fp_bs_write = NULL;
    int                bs_num, max_bs_num;
    int                bs_end_pos;
    int                intra_dist_idx = 0;
    COM_BSR         * bs;
    COM_SQH         * sqh;
    COM_PIC_HEADER  * pic_header;
    COM_CNKH        * cnkh;
    DEC_CTX        * ctx;

    unsigned char    * new_bs_buf = NULL;
    unsigned char    * new_bs_buf2 = NULL;
    COM_SH_EXT *shext;
    COM_BITB    new_bitb;
    COM_BSW    *new_bs;
    new_bs = (COM_BSW*)malloc(sizeof(COM_BSW));
    u8 startcodebuf[4];
    startcodebuf[0] = 0x00;
    startcodebuf[1] = 0x00;
    startcodebuf[2] = 0x01;
    startcodebuf[3] = 0xB1;

    int target_fps = -1;
    int target_fps_code = -1;


    /* parse options */
    ret = com_args_parse_all(argc, argv, options);
    if (ret != 0)
    {
        if (ret > 0) v0print("-%c argument should be set\n", ret);
        if (ret < 0) v0print("Configuration error, please refer to the usage.\n");
        print_usage();
        return -1;
    }
    if (argc < 6)
    {
        printf("Three parameters are desired...\n");
        return -1;
    }

    {
        static int fps_code_tbl[14] = { -1, -1, 24, 25, -1, 30, 50, -1, 60, 100, 120, 200, 240, 300 };
        target_fps = op_target_fps;
        for (int i = 0; i < 14; i++)
        {
            if (target_fps == fps_code_tbl[i])
            {
                target_fps_code = i;
                break;
            }
        }

        // set line buffering (_IOLBF) for stdout to prevent incomplete logs when app crashed.
        setvbuf(stdout, NULL, _IOLBF, 1024);
        max_bs_num = argc - 2 - 2;
        fp_bs_write = fopen(argv[max_bs_num + 1], "wb");

        int pictureslice = 0;
        int pic_cnt = 0;
        int i_period = 0;
        int i_periodflag = 0;
        int max_bframes = 0;
        int gopflag = 0;
        int max_temporal_id_remain = -1;
        int throw_this_pic = 0;
        for (bs_num = 1; bs_num < max_bs_num; bs_num++)
        {
            ++bs_num;
            fp_bs = fopen(argv[bs_num], "rb");

            if (fp_bs == NULL)
            {
                v0print("ERROR: cannot open bitstream file = %s\n", op_fname_inp);
                print_usage();
                return -1;
            }
            fseek(fp_bs, 0, SEEK_END);
            bs_end_pos = ftell(fp_bs);

            bs_buf = malloc(MAX_BS_BUF);
            if (bs_buf == NULL)
            {
                v0print("ERROR: cannot allocate bit buffer, size=%d\n", MAX_BS_BUF);
                return -1;
            }
            id = dec_create(&cdsc, NULL);
            if (id == NULL)
            {
                v0print("ERROR: cannot create decoder\n");
                return -1;
            }
            if (set_extra_config(id))
            {
                v0print("ERROR: cannot set extra configurations\n");
                return -1;
            }
            new_bs_buf = malloc(MAX_BS_BUF);
            if (new_bs_buf == NULL)
            {
                v0print("ERROR: cannot allocate bit buffer, size=%d\n", MAX_BS_BUF);
                return -1;
            }
            new_bs_buf2 = malloc(MAX_BS_BUF);
            if (new_bs_buf2 == NULL)
            {
                v0print("ERROR: cannot allocate bit buffer, size=%d\n", MAX_BS_BUF);
                return -1;
            }
            new_bitb.addr = new_bs_buf;
            new_bitb.addr2 = new_bs_buf2;
            new_bitb.bsize = MAX_BS_BUF;
            new_bitb.err = 0;
            bs_read_pos = 0;
            while (1)
            {
                bs_size = read_a_bs(fp_bs, &bs_read_pos, bs_buf);
                if (bs_size <= 0)
                {
                    v1print("bumping process starting...\n");
                    continue;
                }

                bs_read_pos += (bs_size);
                bitb.addr = bs_buf;
                bitb.ssize = bs_size;
                bitb.bsize = MAX_BS_BUF;
                ctx = (DEC_CTX *)id;
                DEC_ID_TO_CTX_RV(id, ctx, COM_ERR_INVALID_ARGUMENT);
                bs = &ctx->bs;
                sqh = &ctx->info.sqh;
                pic_header = &ctx->info.pic_header;
                shext = &ctx->info.shext;
                cnkh = &ctx->info.cnkh;
                /* set error status */
                ctx->bs_err = 0;
                bitb.err = 0;
                COM_TRACE_SET(1);
                /* bitstream reader initialization */
                com_bsr_init(bs, bitb.addr, bitb.ssize, NULL);
                SET_SBAC_DEC(bs, &ctx->sbac_dec);

                if (bs->cur[3] == 0xB0)
                {
                    cnkh->ctype = COM_CT_SQH;
                }
                else if (bs->cur[3] == 0xB3 || bs->cur[3] == 0xB6)
                {
                    cnkh->ctype = COM_CT_PICTURE;
                    if (bs->cur[3] == 0xB3) pictureslice = 0;
                    if (bs->cur[3] == 0xB6) pictureslice = 1;
                }
                else if (bs->cur[3] >= 0x00 && bs->cur[3] <= 0x8E)
                {
                    cnkh->ctype = COM_CT_SLICE;
                }
                else if (bs->cur[3] == 0xB5)
                {
                    cnkh->ctype = 32768;
                }
                else if (bs->cur[3] == 0xB1)
                {
                    cnkh->ctype = 65536;
                }

                if (cnkh->ctype == COM_CT_SQH)
                {
                    ret = dec_eco_sqh(bs, sqh);
                    com_assert_rv(COM_SUCCEEDED(ret), ret);
                    if (!ctx->init_flag)
                    {
                        ret = sequence_init(ctx, sqh);
                        com_assert_rv(COM_SUCCEEDED(ret), ret);
                        g_DOIPrev = g_CountDOICyCleTime = 0;
                        ctx->init_flag = 1;
                    }
                    fwrite(bs_buf, 1, bs_size, fp_bs_write);
                    
                }
                else if (cnkh->ctype == 32768)
                {
                    fwrite(bs_buf, 1, bs_size, fp_bs_write);
                    // dec extension data
                    ret = extension_and_user_data(ctx, bs, 0, sqh, pic_header);

                    COM_TEMPORAL_SCALABILITY *temporal_scalability = &(ctx->dec_ctx_extension_data.extension_temporal_scalability);
                    int num_of_temporal_level = temporal_scalability->num_of_temporal_level;
                    // printf("num_of_temporal_level: %d\n", num_of_temporal_level);
                    for (int i = 0; i < num_of_temporal_level; i++)
                    {
                        if (temporal_scalability->temporal_frame_rate[i] == target_fps_code)
                        {
                            max_temporal_id_remain = i;
                        }
                        // printf("temporal_frame_rate, bit_rate_lower, bit_rate_upper: %d %d %d %d\n", i, temporal_scalability->temporal_frame_rate[i],
                        //     temporal_scalability->temporal_bit_rate_lower[i], temporal_scalability->temporal_bit_rate_upper[i]);
                    }
                    printf("Max temporal id remain: %d\n", max_temporal_id_remain);
                    com_assert(max_temporal_id_remain != -1);

                }
                else if (cnkh->ctype == COM_CT_PICTURE)
                {
                    /* decode slice header */
                    int need_minus_256 = 0;
                    ret = dec_eco_pic_header(bs, pic_header, sqh, &need_minus_256);
                    com_assert_rv(COM_SUCCEEDED(ret), ret);
                    if (pic_header->temporal_id > max_temporal_id_remain)
                    {
                        throw_this_pic = 1;
                        // printf("throw poc: %d\n", pic_header->poc);
                    }
                    else
                    {
                        throw_this_pic = 0;
                    }
                    if (!throw_this_pic)
                    {
                        fwrite(bs_buf, 1, bs_size, fp_bs_write);
                    }
                    
                }
                else if (cnkh->ctype == COM_CT_SLICE)
                {
                    if (!throw_this_pic)
                    {
                        fwrite(bs_buf, 1, bs_size, fp_bs_write);
                    }
                    pic_cnt++;
                }
                else if (cnkh->ctype == 65536)
                {
                    fwrite(bs_buf, 1, bs_size, fp_bs_write);
                }

                if (bs_read_pos == bs_end_pos)
                {
                    pic_cnt--;
                    break;
                }
            }
            if (id) dec_delete(id);
            if (imgb_t) imgb_free(imgb_t);
            if (fp_bs) fclose(fp_bs);
            if (bs_buf) free(bs_buf);
        }

        if (fp_bs_write) fclose(fp_bs_write);

        printf("\033[1m\033[40;33m Processing complete! \033[0m\n");

        return 0;
    }

#endif

}