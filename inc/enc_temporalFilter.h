/* Partially based on the algorithm in JVET-U0056 and JVET-V0056 */

#ifndef _ENC_TEMPORAL_FILTER_H_
#define _ENC_TEMPORAL_FILTER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <com_typedef.h>
#include "enc_def.h"
#if TEMPORAL_FILTER_SCC
#include "enc_ibc_hash_wrapper.h"
#endif
#if RSD_OPT
#include "enc_rsd_hash_wrapper.h"
#endif
#define COMP_Y    0
#define COMP_CB   1
#define COMP_CR   2
#define COMP_NUM  3

#define CHAN_LUMA   0
#define CHAN_CHROMA 1
#define CHAN_NUM    2
#define COMP_TO_CHAN(c) ((c == COMP_Y) ? CHAN_LUMA : CHAN_CHROMA)

typedef uint16_t pel_t;

typedef struct TF_PIC_YUV
{
    // Only Support 420 Now
    pel_t * pic_buf[COMP_NUM];
    pel_t * pic_org[COMP_NUM];

    uint32_t width[CHAN_NUM];
    uint32_t height[CHAN_NUM];
    uint32_t stride[CHAN_NUM];
    uint8_t  depth[CHAN_NUM];
    uint8_t  margin_x;
    uint8_t  margin_y;  
} TF_PIC_YUV;

typedef struct TF_MV
{
    int16_t x;
    int16_t y;
    int32_t ssd;
    int32_t noise;
} TF_MV;

typedef struct TF_MV_MAT
{
    uint32_t width;
    uint32_t height;
    TF_MV * buf;
} TF_MV_MAT;

#define FILTER_TYPE 3
#define FILTER_RANGE 4
#define MAX_FILTER_LEN (2 * FILTER_RANGE + 1)
#define POC_TO_FILTER_IDX(poc) ((poc) % MAX_FILTER_LEN)

typedef struct TF_PIC_BUF
{
    TF_PIC_YUV org_pic;
    TF_PIC_YUV crt_pic;
    uint8_t is_used;
    int64_t poc;

    TF_MV_MAT mvs;
    int16_t poc_offset;
} TF_PIC_BUF;

typedef struct TF_STRENGTH
{
    int8_t period;
    double  strength;
} TF_STRENGTH;

typedef struct TF_PIC_PRMD
{
    TF_PIC_YUV pic[3];
} TF_PIC_PRMD;

typedef struct TF_MV_PRMD
{
    TF_MV_MAT mvs[3];
} TF_MV_PRMD;

typedef struct TF_CTX
{
    uint8_t input_depth[CHAN_NUM];
    uint8_t internal_depth[CHAN_NUM];
    uint32_t source_width;
    uint32_t source_height;
    int16_t qp;
    
    TF_STRENGTH tf_strenghts[FILTER_TYPE];
    uint8_t lookahead;
    
    TF_PIC_YUV * org_pic;
    TF_PIC_YUV * new_pic;
    TF_PIC_PRMD pic_pyramid_a;
    TF_PIC_PRMD pic_pyramid_b;
    TF_MV_PRMD mv_pyramid;

    TF_PIC_BUF pic_bufs[MAX_FILTER_LEN];
} TF_CTX;

void tf_init(TF_CTX * h, int qp, int width, int height, int input_depth, int internal_depth, double strength[FILTER_TYPE], int period[FILTER_TYPE], int gop_based);
int  tf_prepare_frames(TF_CTX * h, int poc, int skip_frame, char * input_name
#if DQP_OPT
    , ENC_CTX *ctx
#endif
);
int  tf_filter(TF_CTX * h, int frame_idx, int skip_frame
#if DQP_OPT
    , ENC_CTX *ctx
#endif
);
void tf_uninit(TF_CTX * h);

#endif
