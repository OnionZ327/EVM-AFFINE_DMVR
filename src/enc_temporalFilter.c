
/* Partially based on the algorithm in JVET-U0056 and JVET-V0056 */

#include "enc_temporalFilter.h"
#include "com_typedef.h"
#include "enc_def.h"

static const double g_tf_chroma_factor = 0.55;
static const double g_tf_sigma_multiplier = 9.0;
static const double g_tf_sigma_zero_point = 10.0;
static const int g_tf_motion_vector_factor = 16;
static const int g_tf_padding = 128;
static const int g_tf_interpolation_filter[16][8] =
{
    {   0,   0,   0,  64,   0,   0,   0,   0 },
    {   0,   1,  -3,  64,   4,  -2,   0,   0 },
    {   0,   1,  -6,  62,   9,  -3,   1,   0 },
    {   0,   2,  -8,  60,  14,  -5,   1,   0 },
    {   0,   2,  -9,  57,  19,  -7,   2,   0 },
    {   0,   3, -10,  53,  24,  -8,   2,   0 },
    {   0,   3, -11,  50,  29,  -9,   2,   0 },
    {   0,   3, -11,  44,  35, -10,   3,   0 },
    {   0,   1,  -7,  38,  38,  -7,   1,   0 },
    {   0,   3, -10,  35,  44, -11,   3,   0 },
    {   0,   2,  -9,  29,  50, -11,   3,   0 },
    {   0,   2,  -8,  24,  53, -10,   3,   0 },
    {   0,   2,  -7,  19,  57,  -9,   2,   0 },
    {   0,   1,  -5,  14,  60,  -8,   2,   0 },
    {   0,   1,  -3,   9,  62,  -6,   1,   0 },
    {   0,   0,  -2,   4,  64,  -3,   1,   0 } 
};

static const double g_tf_ref_strengths[3][4] =
{
    {0.85, 0.57, 0.41, 0.33},
    {1.13, 0.97, 0.81, 0.57},
    {0.30, 0.30, 0.30, 0.30} 
};
#if DQP_OPT
const int bim_threshold[4] = { 75, 60, 30, 15 };
#endif

#define tf_malloc(type, size) malloc(sizeof(type) * (size))
#define tf_free(ptr) if(ptr != NULL) { free(ptr); ptr = NULL; }

#define tf_max(a, b) (((a) > (b)) ? (a) : (b))
#define tf_min(a, b) (((a) < (b)) ? (a) : (b))
#define tf_abs(a) (((a) < 0) ? (-(a)) : (a))
#define tf_round(a) ((int)((a) + 0.5))
#define tf_clip3(x, a, b) (tf_min(tf_max(x, a), b))

#define tf_luma_block_size 8

static TF_PIC_YUV * tf_pic_create(TF_PIC_YUV * ptr, int width, int height, int margin_x, int margin_y, uint8_t bitdepth[CHAN_NUM])
{
    ptr->width[CHAN_LUMA] = width;
    ptr->height[CHAN_LUMA] = height;
    ptr->width[CHAN_CHROMA] = width >> 1;
    ptr->height[CHAN_CHROMA] = height >> 1;
    ptr->stride[CHAN_LUMA] = ptr->width[CHAN_LUMA] + 2 * margin_x;
    ptr->stride[CHAN_CHROMA] = ptr->width[CHAN_CHROMA] + 2 * margin_x;

    ptr->margin_x = margin_x;
    ptr->margin_y = margin_y;

    uint32_t real_height[CHAN_NUM];
    real_height[CHAN_LUMA] = ptr->height[CHAN_LUMA] + 2 * margin_y;
    real_height[CHAN_CHROMA] = ptr->height[CHAN_CHROMA] + 2 * margin_y;
    
    ptr->pic_buf[0] = (pel_t *)tf_malloc(pel_t, (real_height[0] * ptr->stride[0]));
    ptr->pic_buf[1] = (pel_t *)tf_malloc(pel_t, (real_height[1] * ptr->stride[1]));
    ptr->pic_buf[2] = (pel_t *)tf_malloc(pel_t, (real_height[1] * ptr->stride[1]));
    
    ptr->pic_org[0] = ptr->pic_buf[0] + margin_x + ptr->stride[0] * margin_y;
    ptr->pic_org[1] = ptr->pic_buf[1] + margin_x + ptr->stride[1] * margin_y;
    ptr->pic_org[2] = ptr->pic_buf[2] + margin_x + ptr->stride[1] * margin_y;

    ptr->depth[CHAN_LUMA] = bitdepth[CHAN_LUMA];
    ptr->depth[CHAN_CHROMA] = bitdepth[CHAN_CHROMA];

    return ptr;
}

static TF_PIC_YUV * tf_pic_copy(TF_PIC_YUV * dst, TF_PIC_YUV * src)
{
    dst->width[CHAN_LUMA] = src->width[CHAN_LUMA];
    dst->height[CHAN_LUMA] = src->height[CHAN_LUMA];
    dst->width[CHAN_CHROMA] = src->width[CHAN_CHROMA];
    dst->height[CHAN_CHROMA] = src->height[CHAN_CHROMA];
    dst->stride[CHAN_LUMA] = src->stride[CHAN_LUMA];
    dst->stride[CHAN_CHROMA] = src->stride[CHAN_CHROMA];

    dst->margin_x = src->margin_x;
    dst->margin_y = src->margin_y;

    pel_t * pSrc = src->pic_org[COMP_Y];
    pel_t * pDst = dst->pic_org[COMP_Y];
    for (unsigned int y = 0; y < dst->height[CHAN_LUMA]; y++ )
    {
        memcpy(pDst, pSrc, dst->width[CHAN_LUMA] * sizeof(pel_t));
        pSrc += src->stride[CHAN_LUMA];
        pDst += dst->stride[CHAN_LUMA];
    }

    pSrc = src->pic_org[COMP_CB];
    pDst = dst->pic_org[COMP_CB];
    for (unsigned int y = 0; y < dst->height[CHAN_CHROMA]; y++ )
    {
        memcpy(pDst, pSrc, dst->width[CHAN_CHROMA] * sizeof(pel_t));
        pSrc += src->stride[CHAN_CHROMA];
        pDst += dst->stride[CHAN_CHROMA];
    }

    pSrc = src->pic_org[COMP_CR];
    pDst = dst->pic_org[COMP_CR];
    for (unsigned int y = 0; y < dst->height[CHAN_CHROMA]; y++ )
    {
        memcpy(pDst, pSrc, dst->width[CHAN_CHROMA] * sizeof(pel_t));
        pSrc += src->stride[CHAN_CHROMA];
        pDst += dst->stride[CHAN_CHROMA];
    }

    dst->depth[CHAN_LUMA] = src->depth[CHAN_LUMA];
    dst->depth[CHAN_CHROMA] = src->depth[CHAN_CHROMA];

    return dst;
}

static TF_PIC_YUV * tf_pic_extend_border(TF_PIC_YUV * ptr)
{
    for(int comp = COMP_Y; comp < COMP_NUM; comp++)
    {
        const int channel_idx = COMP_TO_CHAN(comp);
        
        pel_t * pi_org = ptr->pic_org[comp];
        const int stride = ptr->stride[channel_idx];
        const int width = ptr->width[channel_idx];
        const int height = ptr->height[channel_idx];
        const int margin_x = ptr->margin_x;
        const int margin_y = ptr->margin_y;
        pel_t*  pi = pi_org;
        
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < margin_x; x++ )
            {
                pi[ -margin_x + x ] = pi[0];
                pi[    width + x  ] = pi[width-1];
            }
            pi += stride;
        }

        pi -= (stride + margin_x);
        for (int y = 0; y < margin_y; y++ )
        {
            memcpy( pi + (y + 1) * stride, pi, sizeof(pel_t) * (width + (margin_x << 1)) );
        }

        pi -= ((height-1) * stride);
        for (int y = 0; y < margin_y; y++ )
        {
            memcpy( pi - (y + 1) * stride, pi, sizeof(pel_t) * (width + (margin_x << 1)) );
        }
    }

    return ptr;
}

static TF_PIC_YUV * tf_pic_read(TF_PIC_YUV * ptr, FILE * fid, uint8_t intputDepth, uint8_t internal_depth)
{
    if (intputDepth == 8)
    {
        pel_t * dst;
        uint8_t tmp;
        for (int comp = COMP_Y; comp < COMP_NUM; comp ++) {
            dst = ptr->pic_org[comp];
            int channel_idx = COMP_TO_CHAN(comp);
            for (unsigned int height = 0; height < ptr->height[channel_idx]; height ++) {
                for (unsigned int width = 0; width < ptr->width[channel_idx]; width ++) {
                    if (fread((void *)(&tmp), sizeof(uint8_t), 1, fid) != 1)
                    {
                        fclose(fid);
                        assert(0);
                    }
                    dst[width] = ((pel_t)tmp) << (internal_depth - intputDepth);
                }
                dst += ptr->stride[channel_idx];
            }
        }
        return ptr;
    }

    if (intputDepth == 10)
    {
        pel_t * dst;
        uint8_t tmp[2];
        for (int comp = COMP_Y; comp < COMP_NUM; comp ++) {
            dst = ptr->pic_org[comp];
            int channel_idx = COMP_TO_CHAN(comp);
            for (unsigned int height = 0; height < ptr->height[channel_idx]; height ++) {
                for (unsigned int width = 0; width < ptr->width[channel_idx]; width ++) {
                    if (fread((void *)(&tmp), sizeof(uint8_t), 2, fid) != 2)
                    {
                        fclose(fid);
                        assert(0);
                    }
                    dst[width] = (((pel_t)tmp[1]) << 8) | ((pel_t)tmp[0]);
                    dst[width] = dst[width] >> (intputDepth - internal_depth);
                }
                dst += ptr->stride[channel_idx];
            }
        }
        return ptr;
    }

    printf("ssd: only supported 8 or 10 bit YUV420\n");
    return NULL;
}

static TF_PIC_YUV * tf_pic_destroy(TF_PIC_YUV * ptr)
{
    tf_free(ptr->pic_buf[0]);
    tf_free(ptr->pic_buf[1]);
    tf_free(ptr->pic_buf[2]);
    
    ptr->width[CHAN_LUMA] = 0;
    ptr->width[CHAN_CHROMA] = 0;
    
    ptr->height[CHAN_LUMA] = 0;
    ptr->height[CHAN_CHROMA] = 0;

    ptr->stride[CHAN_LUMA] = 0;
    ptr->stride[CHAN_CHROMA] = 0;

    ptr->margin_x = 0;
    ptr->margin_y = 0;
    
    return ptr;
}

static TF_MV * tf_mv_create(TF_MV * ptr)
{
    ptr->x = 0;
    ptr->y = 0;
    ptr->noise = 0;
    ptr->ssd = INT_LEAST32_MAX;
    return ptr;
}

static TF_MV * tf_mv_set(TF_MV * ptr, int x, int y, int e)
{
    ptr->x = x;
    ptr->y = y;
    ptr->ssd = e;
    return ptr;
}

static TF_MV tf_mv_mat_getmv(TF_MV_MAT * ptr, int x, int y)
{
    return ptr->buf[y * ptr->width + x];
}

static void tf_mv_mat_setmv(TF_MV_MAT * ptr, TF_MV * pMv, int x, int y)
{
    TF_MV * cur = &(ptr->buf[y * ptr->width + x]);
    cur->ssd = pMv->ssd;
    cur->noise = pMv->noise;
    cur->x = pMv->x;
    cur->y = pMv->y;
    return;
}

static TF_MV_MAT * tf_mv_mat_create(TF_MV_MAT * ptr, int width, int height) 
{
    ptr->width = width;
    ptr->height = height;
    ptr->buf = (TF_MV *)tf_malloc(TF_MV, width * height);

    TF_MV mve;
    tf_mv_create(&mve);
    for (int x = 0; x < width; x ++) {
        for (int y = 0; y < height; y ++) {
            tf_mv_mat_setmv(ptr, &mve, x, y);
        }
    }

    return ptr;
}

static TF_MV_MAT * tf_mv_mat_destroy(TF_MV_MAT * ptr)
{
    ptr->width = 0;
    ptr->height = 0;
    tf_free(ptr->buf);
    return ptr;
}

static void tf_subsample_luma(const TF_PIC_YUV * src, TF_PIC_YUV * dst)
{
    const int newWidth  = src->width[COMP_Y] >> 1;
    const int newHeight = src->height[COMP_Y] >> 1;

    const pel_t * src_line       = src->pic_org[COMP_Y];
    const uint32_t src_stride    = src->stride[COMP_Y];
          pel_t * dst_line       = dst->pic_org[COMP_Y];
    const uint32_t dst_stride    = dst->stride[COMP_Y];

    for (int y = 0; y < newHeight; y++)
    {
        const pel_t *src_line_next = src_line + src_stride;

        for (int x = 0; x < newWidth; x++)
        {
            dst_line[x] = (
                src_line[x << 1] + src_line_next[x << 1] + 
                src_line[(x << 1) + 1] + src_line_next[(x << 1) + 1] + 
                2) >> 2;
        }

        src_line += (src_stride << 1);
        dst_line += dst_stride;
    }
    
    tf_pic_extend_border(dst);
    return;
}

static void tf_gen_pyramid_luma(TF_PIC_PRMD * ptr)
{
    tf_subsample_luma(&(ptr->pic[0]), &(ptr->pic[1]));
    tf_subsample_luma(&(ptr->pic[1]), &(ptr->pic[2]));
    return;
}

static int tf_calc_ssd(const TF_PIC_PRMD * org_pyramid, const TF_PIC_PRMD * ref_pyramid, int * temp, int x, int y, int dx, int dy, const int block_size, const int max_value, const int level)
{
    const pel_t * org = org_pyramid->pic[level].pic_org[COMP_Y];
    const pel_t * ref = ref_pyramid->pic[level].pic_org[COMP_Y];
    const int org_stride = org_pyramid->pic[level].stride[COMP_Y];
    const int ref_stride = ref_pyramid->pic[level].stride[COMP_Y];

    const int subsample_ratio = 1 << level;
    x = x / subsample_ratio;
    y = y / subsample_ratio;
    dx = dx / subsample_ratio;
    dy = dy / subsample_ratio;

    int ssd = 0;
    if (((dx & 0xF) == 0) && ((dy & 0xF) == 0))
    {
        dx /= g_tf_motion_vector_factor;
        dy /= g_tf_motion_vector_factor;

        int diff = 0;
        const pel_t * org_line = org + y * org_stride + x;
        const pel_t * ref_line = ref + (y + dy) * ref_stride + (x + dx);

        for (int y1 = 0; y1 < block_size; y1++)
        {
            for (int x1 = 0; x1 < block_size; x1 ++)
            {
                diff = org_line[x1] - ref_line[x1];
                ssd += diff * diff;
            }
            org_line += org_stride;
            ref_line += ref_stride;
        }
    }
    else
    {
        const int *filter_x = g_tf_interpolation_filter[(dx & 0xF)];
        const int *filter_y = g_tf_interpolation_filter[(dy & 0xF)];
        
        int interp;
        int tmp_stride = 64;
        int * tmp_line = temp + tmp_stride;
 
        const pel_t * ref_line = ref + (x + (dx >> 4) - 3);
        ref_line += ((y + (dy >> 4) - 3) * ref_stride);
        ref_line += ref_stride;

        for (int y1 = 1; y1 < block_size + 7; y1++) 
        {
            for (int x1 = 0; x1 < block_size; x1++) 
            {                
                interp  = filter_x[1] * ref_line[x1 + 1]; 
                interp += filter_x[2] * ref_line[x1 + 2]; 
                interp += filter_x[3] * ref_line[x1 + 3]; 
                interp += filter_x[4] * ref_line[x1 + 4]; 
                interp += filter_x[5] * ref_line[x1 + 5]; 
                interp += filter_x[6] * ref_line[x1 + 6]; 
                tmp_line[x1] = interp;
            }
            ref_line += ref_stride;
            tmp_line += tmp_stride;
        } 

        tmp_line = temp;
        const pel_t max_sample_value = (max_value); 
        
        for (int y1 = 0; y1 < block_size; y1++) 
        { 
            for (int x1 = 0; x1 < block_size; x1++) 
            {
                interp  = filter_y[1] * tmp_line[1 * tmp_stride + x1]; 
                interp += filter_y[2] * tmp_line[2 * tmp_stride + x1]; 
                interp += filter_y[3] * tmp_line[3 * tmp_stride + x1]; 
                interp += filter_y[4] * tmp_line[4 * tmp_stride + x1]; 
                interp += filter_y[5] * tmp_line[5 * tmp_stride + x1]; 
                interp += filter_y[6] * tmp_line[6 * tmp_stride + x1]; 
                interp = (interp + (1 << 11)) >> 12; 
                tmp_line[x1] = tf_clip3(interp, 0, max_sample_value); 
            }
            tmp_line += tmp_stride;
        }
        
        tmp_line = temp;
        const pel_t * org_line = org;
        org_line += (y * org_stride);
        org_line += x;
        
        for (int y1 = 0; y1 < block_size; y1++)
        {
            for (int x1 = 0; x1 < block_size; x1++)
            {
                interp = tmp_line[x1];
                ssd += (interp - org_line[x1]) * (interp - org_line[x1]);
            }
            org_line += org_stride;
            tmp_line += tmp_stride;
        }
    }
    return ssd;
}

#define tf_me_search_point(pBest, block_size, cur, curX, curY, ref, ofsX, ofsY, temp, max_value, level) \
    int ssd = tf_calc_ssd(ref, cur, temp, block_x, block_y, ofsX, ofsY, block_size, max_value, level); \
    if (ssd < pBest->ssd) { tf_mv_set(pBest, ofsX, ofsY, ssd); }

#define tf_me_full_search(pBest, block_size, cur, curX, curY, ref, center_x, center_y, range, step, temp, max_value, level) \
    for (int y2 = center_y - (range); y2 <= center_y + (range); y2 += step) { \
        for (int x2 = center_x - (range); x2 <= center_x + (range); x2 += step) { \
            tf_me_search_point(pBest, block_size, cur, curX, curY, ref, x2, y2, temp, max_value, level); }}

static void tf_motion_estimate_init(TF_MV * best, TF_MV_MAT *previous, const TF_PIC_PRMD * ref, const TF_PIC_PRMD * cur, int * temp, int block_y, int block_x, int block_size, const int level)
{
    int org_width = ref->pic[0].width[COMP_Y];
    int org_height = cur->pic[0].height[COMP_Y];
    const int max_value = (1 << cur->pic[0].depth[CHAN_LUMA]) - 1;

    const int step = (block_size / tf_luma_block_size);
    const int search_step = step << (level + 1);

    const int mat_center_pos_y = block_y / tf_luma_block_size;
    const int mat_center_pos_x = block_x / tf_luma_block_size;

    for (int mat_pos_y = mat_center_pos_y - search_step; mat_pos_y <= mat_center_pos_y + search_step; mat_pos_y += search_step)
    {
        for (int mat_pos_x = mat_center_pos_x - search_step; mat_pos_x <= mat_center_pos_x + search_step; mat_pos_x += search_step)
        {            
            if ((mat_pos_x >= 0) && (mat_pos_x < org_width / tf_luma_block_size) &&
                (mat_pos_y >= 0) && (mat_pos_y < org_height / tf_luma_block_size))
            {
                TF_MV old = tf_mv_mat_getmv(previous, mat_pos_x, mat_pos_y);
                tf_me_search_point(best, block_size, cur, block_x, block_y, ref, old.x, old.y, temp, max_value, level);
            }
        }
    }

    tf_me_search_point(best, block_size, cur, block_x, block_y, ref, 0, 0, temp, max_value, level);

    return;
}

static void tf_motion_estimation_luma(TF_MV_MAT * mvs, const TF_PIC_PRMD * ref, const TF_PIC_PRMD * cur, TF_MV_PRMD * mv_pyramid, const int refine, const int level)
{
    int range = refine ? 0 : 5;
    const int block_size = tf_luma_block_size << (1 - refine);
    const int step_size = block_size << level;
    const int org_width = ref->pic[0].width[COMP_Y];
    const int org_height = cur->pic[0].height[COMP_Y];
    const int max_value = (1 << cur->pic[0].depth[CHAN_LUMA]) - 1;
    const int real_block_size = block_size << level;

    int interp_buf[(64 + 8) * 64];
    TF_MV_MAT * previous;

    if (refine > 0)
    {
        previous = &(mv_pyramid->mvs[level]);
    } 
    else
    { 
        previous = (level == 2) ? (TF_MV_MAT *)NULL : &(mv_pyramid->mvs[level + 1]);
        mvs = &(mv_pyramid->mvs[level]);
    }

    for (int block_y = 0; block_y + real_block_size <= org_height; block_y += step_size)
    {
        for (int block_x = 0; block_x + real_block_size <= org_width; block_x += step_size)
        {
            TF_MV best;
            tf_mv_create(&best);
            if (previous == NULL) { range = 8; }
            
            if (previous)
            {
                tf_motion_estimate_init(&best, previous, ref, cur, interp_buf, block_y, block_x, block_size, level);           
            }
            
            TF_MV previous_best = best;
            const int search_step = g_tf_motion_vector_factor << level;
            tf_me_full_search((&best), block_size, cur, block_x, block_y, ref, previous_best.x, previous_best.y, range * search_step, search_step, interp_buf, max_value, level);
            
            if (refine)
            {
                previous_best = best;
                tf_me_full_search((&best), block_size, cur, block_x, block_y, ref, previous_best.x, previous_best.y, 3 * 4, 4, interp_buf, max_value, level);

                previous_best = best;
                tf_me_full_search((&best), block_size, cur, block_x, block_y, ref, previous_best.x, previous_best.y, 3, 1, interp_buf, max_value, level);
            }

            if (block_y > 0)
            {
                TF_MV above_mv;
                above_mv = tf_mv_mat_getmv(mvs, block_x / tf_luma_block_size, (block_y - step_size) / tf_luma_block_size);
                tf_me_search_point((&best), block_size, cur, block_x, block_y, ref, above_mv.x, above_mv.y, interp_buf, max_value, level);
            }
            
            if (block_x > 0)
            {
                TF_MV left_mv;
                left_mv = tf_mv_mat_getmv(mvs, (block_x - step_size) / tf_luma_block_size, block_y / tf_luma_block_size);
                tf_me_search_point((&best), block_size, cur, block_x, block_y, ref, left_mv.x, left_mv.y, interp_buf, max_value, level);
            }

            if ((refine > 0) && (level == 0))
            {
                pel_t pix;
                double avg = 0.0;
                double variance = 0.0;
                int32_t pix_stride = ref->pic[level].stride[COMP_Y];
                pel_t * pix_line = ref->pic[level].pic_org[COMP_Y] + block_x + pix_stride * block_y;
                
                for (int y1 = 0; y1 < block_size; y1++)
                {
                    for (int x1 = 0; x1 < block_size; x1++)
                    {
                        pix = pix_line[x1];
                        variance += (pix * pix);
                        avg += pix;
                    }
                    pix_line += pix_stride;
                }
                variance -= (avg * avg / (block_size * block_size));
                avg = avg / (block_size * block_size);
                
                best.ssd = (int) (20 * ((best.ssd + 5) / (variance + 5)) + (best.ssd / (block_size * block_size)) / 50);
            }

            int block_mat_pox_x = block_x / tf_luma_block_size;
            int block_mat_pox_y = block_y / tf_luma_block_size;
            
            for (int idxX = 0; idxX < real_block_size / tf_luma_block_size; idxX ++)
            {
                for (int idxY = 0; idxY < real_block_size / tf_luma_block_size; idxY ++)
                {
                    tf_mv_mat_setmv(mvs, &best, block_mat_pox_x + idxX, block_mat_pox_y + idxY);                        
                }   
            }
        }
    }
}

static void tf_motion_compensation(const TF_MV_MAT * mvs, const TF_PIC_YUV * input, TF_PIC_YUV * output,
                                   const int uiComponentNum, const uint8_t depth[CHAN_NUM])
{
    for(int c = COMP_Y; c < COMP_NUM; c++)
    {
        const int csx = (c == COMP_Y) ? 0 : 1;
        const int csy = (c == COMP_Y) ? 0 : 1;
        const int block_size_x = tf_luma_block_size >> csx;
        const int block_size_y = tf_luma_block_size >> csy;
        const int height = input->height[COMP_TO_CHAN(c)];
        const int width  = input->width[COMP_TO_CHAN(c)];

        const pel_t max_value = (1 << depth[COMP_TO_CHAN(c)]) - 1;

        const pel_t * src_img = input->pic_org[c];
        const int src_stride = input->stride[COMP_TO_CHAN(c)];

        pel_t * dst_img = output->pic_org[c];
        int dst_stride=output->stride[COMP_TO_CHAN(c)];

        int interp_buf[(tf_luma_block_size + 8) * tf_luma_block_size];
        int tmp_stride = tf_luma_block_size; 

        for (int y = 0, blockNumY = 0; y + block_size_y <= height; y += block_size_y, blockNumY++)
        {
            for (int x = 0, blockNumX = 0; x + block_size_x <= width; x += block_size_x, blockNumX++)
            {
                const TF_MV mv = tf_mv_mat_getmv((TF_MV_MAT *)mvs, blockNumX,blockNumY);
                const int dx = mv.x >> csx ;
                const int dy = mv.y >> csy ;
                const int ix = mv.x >> (4 + csx) ;
                const int iy = mv.y >> (4 + csy) ;

                const int *filter_x = g_tf_interpolation_filter[((dx & 0xf))]; 
                const int *filter_y = g_tf_interpolation_filter[((dy & 0xf))]; 
                
                int interp; 
                const pel_t * src_line = src_img;
                src_line += (((y + iy) - 3) * src_stride);
                src_line += ((x + ix) - 3);
                src_line += src_stride;

                for (int y1 = 1; y1 < (block_size_x) + 7; y1++) 
                { 
                    for (int x1 = 0; x1 < (block_size_x); x1++) 
                    {
                        interp  = filter_x[1] * src_line[x1 + 1]; 
                        interp += filter_x[2] * src_line[x1 + 2]; 
                        interp += filter_x[3] * src_line[x1 + 3]; 
                        interp += filter_x[4] * src_line[x1 + 4]; 
                        interp += filter_x[5] * src_line[x1 + 5]; 
                        interp += filter_x[6] * src_line[x1 + 6]; 
                        interp_buf[y1 * tmp_stride + x1] = interp; 
                    }
                    src_line += src_stride;
                } 

                const pel_t max_sample_value = max_value; 
                for (int y1 = 0; y1 < (block_size_x); y1++) { 
                    for (int x1 = 0; x1 < (block_size_x); x1++) { 
                        interp  = filter_y[1] * interp_buf[(y1 + 1) * tmp_stride + x1]; 
                        interp += filter_y[2] * interp_buf[(y1 + 2) * tmp_stride + x1]; 
                        interp += filter_y[3] * interp_buf[(y1 + 3) * tmp_stride + x1]; 
                        interp += filter_y[4] * interp_buf[(y1 + 4) * tmp_stride + x1]; 
                        interp += filter_y[5] * interp_buf[(y1 + 5) * tmp_stride + x1]; 
                        interp += filter_y[6] * interp_buf[(y1 + 6) * tmp_stride + x1]; 
                        interp = (interp + (1 << 11)) >> 12; 
                        interp_buf[y1 * tmp_stride + x1] = tf_clip3(interp, 0, max_sample_value); 
                    } 
                }

                pel_t * dst_line = dst_img + y * dst_stride;
                for (int by = 0; by < block_size_y; by++)
                {
                    for (int bx = 0; bx < block_size_x; bx++)
                    {
                        dst_line[x + bx] = interp_buf[by * tmp_stride + bx];
                    }
                    dst_line += dst_stride;
                }
            }
        }
    }
}

static void tf_motion_estimation(TF_CTX * h, TF_PIC_BUF * pic_buf)
{
    TF_MV_MAT * mat_mve = &(pic_buf->mvs); 
    TF_PIC_YUV * buffer = &(pic_buf->org_pic); 

    tf_pic_copy(&(h->pic_pyramid_b.pic[0]), buffer);
    tf_pic_extend_border(&(h->pic_pyramid_b.pic[0]));
    tf_gen_pyramid_luma(&(h->pic_pyramid_b));

    tf_motion_estimation_luma(NULL, &(h->pic_pyramid_a), &(h->pic_pyramid_b), &(h->mv_pyramid), 0, 2);
    tf_motion_estimation_luma(NULL, &(h->pic_pyramid_a), &(h->pic_pyramid_b), &(h->mv_pyramid), 0, 1);
    tf_motion_estimation_luma(NULL, &(h->pic_pyramid_a), &(h->pic_pyramid_b), &(h->mv_pyramid), 0, 0);
    tf_motion_estimation_luma(mat_mve, &(h->pic_pyramid_a), &(h->pic_pyramid_b), &(h->mv_pyramid), 1, 0);

    return;
}

static void tf_bilateral_filter(TF_CTX * h, int8_t * src_idx, const uint8_t ref_num, double strength)
{
    const TF_PIC_YUV * org_pic = h->org_pic;
    TF_PIC_YUV * newOrgPic = h->new_pic;

    const uint32_t source_width = h->source_width;
    const uint32_t source_height = h->source_height;
    const int qp = h->qp;

    int ref_strength_idx = 
        (ref_num == FILTER_RANGE * 2) ? 0:
        (ref_num == FILTER_RANGE) ? 1 : 2;

    const double luma_sigma_square = (qp - g_tf_sigma_zero_point) * (qp - g_tf_sigma_zero_point) * g_tf_sigma_multiplier * 9.0 / 16.0;
    const double chroma_sigma_square = 30 * 30;
    
    for(int c = COMP_Y; c < COMP_NUM; c++)
    {
        const int height = org_pic->height[COMP_TO_CHAN(c)];
        const int width  = org_pic->width[COMP_TO_CHAN(c)];

        pel_t * src_line = org_pic->pic_org[c];
        pel_t * dst_line = newOrgPic->pic_org[c];
        
        const int src_stride = org_pic->stride[COMP_TO_CHAN(c)];      
        const int dst_stride = newOrgPic->stride[COMP_TO_CHAN(c)];

        const double sigma_square = COMP_TO_CHAN(c) ? chroma_sigma_square : luma_sigma_square;
        const double weight_scale = strength * (COMP_TO_CHAN(c) ? g_tf_chroma_factor : 0.4);
        
        const pel_t max_sample_value = (1 << h->internal_depth[COMP_TO_CHAN(c)]) - 1;
        const double depth_scale = (1 << (10 - h->internal_depth[COMP_TO_CHAN(c)]));
        const int block_size = tf_luma_block_size >> COMP_TO_CHAN(c);

        for (int y = 0; y < height; y++)
        {            
            for (int x = 0; x < width; x++)
            {
                const int org_value = (int)src_line[x];
                double filter_weigth_sum = 1.0;
                double new_value = (double) org_value;

                if ((y % block_size == 0) && (x % block_size == 0))
                {
                    for (int i = 0; i < ref_num; i++)
                    {
                        double variance = 0, diffsum = 0;
                        for (int y1 = 0; y1 < block_size - 1; y1++)
                        {
                            for (int x1 = 0; x1 < block_size - 1; x1++)
                            {
                                int pix   = src_line[x + x1];
                                int pix_r = src_line[x + x1 + 1];
                                int pix_b = src_line[x + x1 + src_stride];
                                int ref  = *(h->pic_bufs[src_idx[i]].crt_pic.pic_org[c] + ((y + y1) * h->pic_bufs[src_idx[i]].crt_pic.stride[COMP_TO_CHAN(c)] + x + x1));

                                int diff   = pix  - ref;
                                int diff_r = pix_r - ref;
                                int diff_b = pix_b - ref;

                                variance += diff * diff;
                                diffsum  += diff_r * diff_r;
                                diffsum  += diff_b * diff_b;
                            }
                        }
                        TF_MV tmp_mve = tf_mv_mat_getmv(&(h->pic_bufs[src_idx[i]].mvs), x / block_size, y / block_size);
                        tmp_mve.noise = (int)tf_round((30 * variance + 1) / (diffsum + 1));
                        tf_mv_mat_setmv(&(h->pic_bufs[src_idx[i]].mvs), &tmp_mve, x / block_size, y / block_size);
                    }
                }

                double min_error = (tf_luma_block_size * tf_luma_block_size) << (h->input_depth[COMP_Y] << 1);
                
                for (int i = 0; i < ref_num; i++)
                {
                    min_error = tf_min(min_error, (double)tf_mv_mat_getmv(&(h->pic_bufs[src_idx[i]].mvs), x / block_size, y / block_size).ssd);
                }

                for (int i = 0; i < ref_num; i++)
                {
                    const int ssd = tf_mv_mat_getmv(&(h->pic_bufs[src_idx[i]].mvs), x / block_size, y / block_size).ssd;
                    const int noise = tf_mv_mat_getmv(&(h->pic_bufs[src_idx[i]].mvs), x / block_size, y / block_size).noise;
                    
                    const int ref_value = (int)h->pic_bufs[src_idx[i]].crt_pic.pic_org[c][(y * h->pic_bufs[src_idx[i]].crt_pic.stride[COMP_TO_CHAN(c)] + x)];
                    double diff = (double)(ref_value - org_value);
                    diff *= depth_scale;
                    double diff_square = diff * diff;

                    const int index = tf_min(3, tf_abs(h->pic_bufs[src_idx[i]].poc_offset) - 1);
                    
                    double adaptive_weight = (noise < 50) ? 1 : 1.2;
                    adaptive_weight *= (ssd < 64) ? 1.2 : ((ssd > 128) ? 0.8 : 1);
                    adaptive_weight *= ((min_error + 1) / (ssd + 1));

                    double sigma_scale = (noise < 50) ? 1.3 : 0.8;
                    sigma_scale *= (ssd < 64) ? 1.3 : 1;
                    sigma_scale *= 2.0;
                    
                    const double weight = weight_scale * g_tf_ref_strengths[ref_strength_idx][index] * adaptive_weight * exp(-diff_square / (sigma_scale * sigma_square));
                    new_value += weight * ref_value;
                    filter_weigth_sum += weight;
                }
                
                new_value /= filter_weigth_sum;
                pel_t sample_new_value = (pel_t)tf_round(new_value);
                dst_line[x] = tf_clip3(sample_new_value, 0, max_sample_value);
            }

            src_line += src_stride;
            dst_line += dst_stride;
        }
    }
}

int tf_prepare_frames(TF_CTX * h, int poc, int skip_frame, char * input_name
#if DQP_OPT
    , ENC_CTX *ctx
#endif
)
{
    int skip_offset = skip_frame;
    int first_frame = tf_max(poc + skip_offset - FILTER_RANGE, 0);
    int last_frame = poc + skip_offset + FILTER_RANGE;

    FILE * yuv_frames = fopen(input_name, "rb");

#if DQP_OPT
    int width = h->org_pic->width[0];
    int height = h->org_pic->height[0];
    const int ctu_size = ctx->info.max_cuwh;
    const int ctu_in_width = ((width + ctu_size - 1) / ctu_size) * ((height + ctu_size - 1) / ctu_size);
    int curr_idx = (ctx->pic_icnt + 1) % ctx->pico_max_cnt;
    ctx->pico_buf[curr_idx]->use_st_dqp = 0;
    memset(ctx->pico_buf[curr_idx]->st_qpmap, 0, sizeof(int)*ctu_in_width);
#endif

    for (int idx = 0; idx < tf_max(first_frame, 0); idx ++)
    {
        tf_pic_read(h->new_pic, yuv_frames, h->input_depth[0], h->internal_depth[0]);
    }

    for (int frame_idx = first_frame; frame_idx <= last_frame; frame_idx++)
    {
        int poc = frame_idx;
        int filter_buf_idx = POC_TO_FILTER_IDX(poc);
        
        if (h->pic_bufs[filter_buf_idx].is_used && (h->pic_bufs[filter_buf_idx].poc == poc)) 
        {
            tf_pic_read(h->new_pic, yuv_frames, h->input_depth[0], h->internal_depth[0]);
            continue;
        } 
        else 
        {
            if (feof(yuv_frames))
            {
                last_frame = frame_idx - 1;
                return 0;
            } 
            else
            {
                tf_pic_read(&(h->pic_bufs[filter_buf_idx].org_pic), yuv_frames, h->input_depth[0], h->internal_depth[0]);
                tf_pic_extend_border(&(h->pic_bufs[filter_buf_idx].org_pic));
                h->pic_bufs[filter_buf_idx].is_used = 1;
                h->pic_bufs[filter_buf_idx].poc = poc;
            }
        }
    }
    
    fclose(yuv_frames);
    return 1;
}

void tf_init(TF_CTX * h, int qp, int width, int height, int input_depth, int internal_depth, double strength[FILTER_TYPE], int period[FILTER_TYPE], int lookahead)
{
    h->qp = qp;
    h->source_width = width;
    h->source_height = height;
    h->input_depth[CHAN_LUMA] = input_depth;
    h->input_depth[CHAN_CHROMA] = input_depth;
    h->internal_depth[CHAN_LUMA] = internal_depth;
    h->internal_depth[CHAN_CHROMA] = internal_depth;
    
    for (int type = 0; type < FILTER_TYPE; type++ )
    {
        h->tf_strenghts[type].strength = strength[type];
        h->tf_strenghts[type].period = period[type];
    }
    
    h->lookahead = lookahead;

    for (int idx = 0; idx < MAX_FILTER_LEN; idx ++) 
    {
        tf_pic_create(&(h->pic_bufs[idx].org_pic), h->source_width, h->source_height, g_tf_padding, g_tf_padding, h->internal_depth);
        h->pic_bufs[idx].is_used = 0;
        h->pic_bufs[idx].poc = -1;

        tf_mv_mat_create(&(h->pic_bufs[idx].mvs), h->source_width / 4, h->source_height / 4);
        h->pic_bufs[idx].poc_offset = -1;

        tf_pic_create(&(h->pic_bufs[idx].crt_pic), h->source_width, h->source_height, g_tf_padding, g_tf_padding, h->internal_depth);
    }

    h->org_pic = &(h->pic_pyramid_a.pic[0]);
    h->new_pic = &(h->pic_pyramid_b.pic[0]);

    tf_pic_create(&(h->pic_pyramid_a.pic[0]), h->source_width, h->source_height, g_tf_padding, g_tf_padding, h->internal_depth);
    tf_pic_create(&(h->pic_pyramid_b.pic[0]), h->source_width, h->source_height, g_tf_padding, g_tf_padding, h->internal_depth);
    tf_pic_create(&(h->pic_pyramid_a.pic[1]), h->source_width / 2, h->source_height / 2, g_tf_padding, g_tf_padding, h->internal_depth);
    tf_pic_create(&(h->pic_pyramid_a.pic[2]), h->source_width / 4, h->source_height / 4, g_tf_padding, g_tf_padding, h->internal_depth);
    tf_pic_create(&(h->pic_pyramid_b.pic[1]), h->source_width / 2, h->source_height / 2, g_tf_padding, g_tf_padding, h->internal_depth);
    tf_pic_create(&(h->pic_pyramid_b.pic[2]), h->source_width / 4, h->source_height / 4, g_tf_padding, g_tf_padding, h->internal_depth);

    tf_mv_mat_create(&(h->mv_pyramid.mvs[0]), h->source_width / tf_luma_block_size, h->source_height / tf_luma_block_size);
    tf_mv_mat_create(&(h->mv_pyramid.mvs[1]), h->source_width / tf_luma_block_size, h->source_height / tf_luma_block_size);
    tf_mv_mat_create(&(h->mv_pyramid.mvs[2]), h->source_width / tf_luma_block_size, h->source_height / tf_luma_block_size);
}

int tf_filter(TF_CTX * h, int frame_idx, int skip_frame
#if DQP_OPT   
    , ENC_CTX *ctx
#endif
)
{
    int do_filter = 0;
    if (h->qp >= 16)
    {
        for (int type = 0; type < FILTER_TYPE; type++)
        {
            int filteredFrame = h->tf_strenghts[type].period;
            
            if (filteredFrame < 0) { continue; }

            if ((frame_idx - skip_frame) % filteredFrame == 0)
            {
                do_filter = 1;
                break;
            }
        }
    }

#if TEMPORAL_FILTER_SCC
    int build_hashmap_flag = ctx->param.use_ibc_flag || ctx->param.fimc_enable_flag || ctx->param.ists_enable_flag
        || (ctx->param.ts_inter_enable_flag == 1 && ctx->info.pic_header.slice_type != SLICE_I) || ctx->param.awp_enable_flag == 2;
    if (do_filter && build_hashmap_flag)
    {
        COM_PIC pic_org;
        pic_org.width_luma = h->org_pic->width[CHAN_LUMA];
        pic_org.height_luma = h->org_pic->height[CHAN_LUMA];
        pic_org.stride_luma = h->org_pic->stride[CHAN_LUMA];
        pic_org.stride_chroma = h->org_pic->stride[CHAN_CHROMA];
        pic_org.y = (pel *) h->org_pic->pic_org[COMP_Y];
        pic_org.u = (pel *) h->org_pic->pic_org[COMP_CB];
        pic_org.v = (pel *) h->org_pic->pic_org[COMP_CR];
        const COM_PIC* p_pic_org = &pic_org;
#if FBHT
        rebuild_hashmap(ctx->ibc_hash_handle, p_pic_org, ctx);
#else
        rebuild_hashmap(ctx->ibc_hash_handle, p_pic_org);
#endif
        int is_screen_pic = get_hash_hit_ratio_pic(ctx, ctx->ibc_hash_handle, 863);
        if (is_screen_pic)
        {
            do_filter = 0;
        }
    }
#endif

    if (do_filter)
    {
        int offset = skip_frame;
        
        int first_frame = frame_idx - FILTER_RANGE;
        int last_frame = frame_idx + FILTER_RANGE;
        
        if (!h->lookahead) { last_frame = frame_idx - 1; }

        int poc_offset = -FILTER_RANGE;

        tf_pic_extend_border(&(h->pic_pyramid_a.pic[0]));
        tf_gen_pyramid_luma(&(h->pic_pyramid_a));

        uint8_t src_num = 0;
        int8_t src_pic_idx[MAX_FILTER_LEN];
        memset(src_pic_idx, -1, MAX_FILTER_LEN);

        for (int poc = first_frame; poc <= last_frame; poc++)
        {
            if ((poc < 0) || (poc == frame_idx))
            {
                poc_offset++;
                continue;
            }

            int filter_buf_idx = POC_TO_FILTER_IDX(poc);
            if (h->pic_bufs[filter_buf_idx].is_used && (h->pic_bufs[filter_buf_idx].poc == poc)) 
            {
                src_pic_idx[src_num] = filter_buf_idx;
                src_num++;
            }

            tf_motion_estimation(h, &(h->pic_bufs[filter_buf_idx]));
            h->pic_bufs[filter_buf_idx].poc_offset = poc_offset;
            poc_offset++;
                
            tf_motion_compensation(
                &(h->pic_bufs[filter_buf_idx].mvs), 
                &(h->pic_bufs[filter_buf_idx].org_pic), 
                &(h->pic_bufs[filter_buf_idx].crt_pic), 
                COMP_NUM, h->internal_depth);
        }

        double strength = -1.0;
        for (int type = 0; type < FILTER_TYPE; type++ )
        {
            int frame = h->tf_strenghts[type].period;

            if (frame < 0) { continue; }

            if ((frame_idx - skip_frame) % frame == 0)
            {
                strength = h->tf_strenghts[type].strength;
            }
        }

#if DQP_OPT
        if ((last_frame - first_frame) > 0)
        {
            int bim_start_frame = frame_idx - 3;
            int bim_end_frame = frame_idx + 3;

            if (!h->lookahead)
            {
                bim_end_frame = frame_idx - 1;
            }

            const int ctu_size = ctx->info.max_cuwh;
            const int ctu_tf_block_size = ctu_size / tf_luma_block_size;
            int width = h->org_pic->width[0];
            int height = h->org_pic->height[0];
            int ctu_in_width = (width + ctu_size - 1) / ctu_size;
            int ctu_in_height = (height + ctu_size - 1) / ctu_size;
            const int ctu_num = ctu_in_width * ctu_in_height;

            int*    tmp_qpmap = (int*)malloc(sizeof(int) * ctu_num);
            double* sum_err = (double*)malloc(sizeof(double) * 3 * ctu_num);
            int*    blk_cnt = (int*)malloc(sizeof(int) * 3 * ctu_num);
            double* sum_grad = (double*)malloc(sizeof(double) * 6 * ctu_num);
            double* max_min_grad = (double*)malloc(sizeof(double) * 2 * ctu_num);
            memset(tmp_qpmap, 0, sizeof(int) * ctu_num);
            memset(sum_err, 0, sizeof(double) * ctu_num * 3);
            memset(blk_cnt, 0, sizeof(int) * ctu_num * 3);
            memset(sum_grad, 0, sizeof(double) * ctu_num * 6);
            memset(max_min_grad, 0, sizeof(double) * ctu_num * 2);

            const int mv_w = h->source_width / (tf_luma_block_size / 2);
            const int mv_h = h->source_height / (tf_luma_block_size / 2);

            int dist_factor[3] = { 3, 3, 3 };
            int fidx = 0;
            for (int poc = bim_start_frame; poc <= bim_end_frame; poc++)
            {
                if ((poc < 0) || (poc == frame_idx))
                {
                    continue;
                }

                int dist = abs(poc - frame_idx) - 1;
                dist_factor[dist]--;
                int filter_idx = POC_TO_FILTER_IDX(poc);
                TF_MV_MAT * mvs = &(h->pic_bufs[filter_idx].mvs);
                int blk_in_width = (mv_w / 2 + (ctu_tf_block_size - 1)) / ctu_tf_block_size;
                for (int y = 0; y < mv_h / 2; y++)
                {
                    for (int x = 0; x < mv_w / 2; x++)
                    {
                        int ctu_x = x / ctu_tf_block_size;
                        int ctu_y = y / ctu_tf_block_size;
                        int ctu_idx = ctu_y * blk_in_width + ctu_x;
                        TF_MV sub_mv = tf_mv_mat_getmv(mvs, x, y);
                        sum_err[dist * ctu_num + ctu_idx] += sub_mv.ssd;
                        blk_cnt[dist * ctu_num + ctu_idx] += 1;
                    }
                }

                for (int i = 0; i < ctu_num; i++)
                {
                    int xi = i % ctu_in_width;
                    int yi = i / ctu_in_width;
                    // Calculate Sobel gradient of CTU
                    const TF_PIC_YUV *org_pic = &(h->pic_bufs[filter_idx].org_pic);
                    int stride = org_pic->stride[COMP_Y];
                    pel_t * src_addr = org_pic->pic_org[COMP_Y];
                    int startx = xi * ctu_size;
                    int starty = yi * ctu_size;
                    double Grad = 0;
                    double num_pixel = 0;
                    for (int i = 1; i < ctu_size - 1; ++i)
                    {
                        for (int j = 1; j < ctu_size - 1; ++j)
                        {
                            if (((i + starty) >= height) || ((j + startx) >= width))
                                continue;
                            const int center_idx = (i + starty) * stride + j + startx;
                            int mat[3][3];
                            for (int ii = -1; ii <= 1; ++ii)
                            {
                                for (int jj = -1; jj <= 1; ++jj)
                                {
                                    const int idx = center_idx + ii * stride + jj;
                                    mat[ii + 1][jj + 1] = src_addr[idx];
                                }
                            }
                            const int Gx = (mat[0][0] - mat[0][2]) + (mat[2][0] - mat[2][2]) +
                                2 * (mat[1][0] - mat[1][2]);
                            const int Gy = (mat[0][0] - mat[2][0]) + (mat[0][2] - mat[2][2]) +
                                2 * (mat[0][1] - mat[2][1]);
                            const int Gs = abs(Gx) + abs(Gy);
                            Grad += Gs;
                            num_pixel++;
                        }
                    }
                    sum_grad[fidx*ctu_num + i] = Grad / num_pixel;
                }
                fidx++;
            }

            for (int i = 0; i < ctu_num; i++)
            {
                double max_grad = 0;
                double min_grad = MAX_COST;
                for (int frm_i = 0; frm_i < fidx; frm_i++)
                {
                    double ctu_grad = sum_grad[frm_i * ctu_num + i];
                    max_grad = max(max_grad, ctu_grad);
                    min_grad = min(min_grad, ctu_grad);
                }
                max_min_grad[i] = min_grad;
                max_min_grad[i + ctu_num] = max_grad;
            }

            const double weight = (frame_idx % 16) ? 0.6 : 1;
            const double center = 45.0;
            for (int i = 0; i < ctu_num; i++)
            {
                int avg_err1 = (int)((sum_err[i] / blk_cnt[i]) * dist_factor[0]);
                int avg_err2 = (int)((sum_err[i + ctu_num] / blk_cnt[i + ctu_num]) * dist_factor[1]);
                int avg_err3 = (int)((sum_err[i + 2 * ctu_num] / blk_cnt[i + 2 * ctu_num]) * dist_factor[2]);
                if (((avg_err1 == 0) && (avg_err2 == 0)) || ((avg_err1 == 0) && (avg_err3 == 0)) || ((avg_err2 == 0) && (avg_err3 == 0)))
                {
                    tmp_qpmap[i] = -2;
                    continue;
                }
                else if (avg_err1 == 0)
                {
                    avg_err1 = avg_err2;
                    avg_err2 = avg_err3;
                }
                else if (avg_err2 == 0)
                {
                    avg_err2 = avg_err3;
                }
                int ratio = (avg_err2 >= avg_err1) ? (avg_err2 / avg_err1) : (avg_err1 / avg_err2);
                double spatial_ratio = (max_min_grad[i] != 0 && max_min_grad[i + ctu_num] != 0) ? (max_min_grad[i + ctu_num] / max_min_grad[i]) : 2;
                double thre = (log(ratio) / log(2)) * 0.2 + 1;
                if (avg_err2 >= avg_err1)
                {
                    if ((ratio >= 3) && (spatial_ratio <= thre) && (avg_err1 > 5))
                    {
                        avg_err2 = (int)(spatial_ratio * avg_err1);
                    }
                }
                else
                {
                    if ((ratio >= 3) && (spatial_ratio <= thre) && (avg_err2 > 5))
                    {
                        avg_err1 = (int)(spatial_ratio * avg_err2);
                    }
                }
                int weighted_err = max(avg_err1, avg_err2) + abs(avg_err2 - avg_err1) * 3;
                weighted_err = (int)(weighted_err * weight + (1 - weight) * center);
                if (weighted_err > bim_threshold[0])
                {
                    tmp_qpmap[i] = 2;
                }
                else if (weighted_err > bim_threshold[1])
                {
                    tmp_qpmap[i] = 1;
                }
                else if (weighted_err < bim_threshold[3])
                {
                    tmp_qpmap[i] = -2;
                }
                else if (weighted_err < bim_threshold[2])
                {
                    tmp_qpmap[i] = -1;
                }
                else
                {
                    tmp_qpmap[i] = 0;
                }
            }

            int curr_idx = (ctx->pic_icnt + 1) % ctx->pico_max_cnt;
            ctx->pico_buf[curr_idx]->use_st_dqp = 1;
            memcpy(ctx->pico_buf[curr_idx]->st_qpmap, tmp_qpmap, sizeof(int) * ctu_num);

            tf_free(tmp_qpmap);
            tf_free(sum_err);
            tf_free(blk_cnt);
            tf_free(sum_grad);
            tf_free(max_min_grad);
        }
#endif

        tf_bilateral_filter(h, src_pic_idx, src_num, strength);
        return 1;
    }
    return 0;
}

void tf_uninit(TF_CTX * h)
{
    h->qp = 0;
    h->source_width = 0;
    h->source_height = 0;
    h->input_depth[CHAN_LUMA] = 8;
    h->input_depth[CHAN_CHROMA] = 8;
    h->internal_depth[CHAN_LUMA] = 8;
    h->internal_depth[CHAN_CHROMA] = 8;
    
    for (int type = 0; type < FILTER_TYPE; type++)
    {
        h->tf_strenghts[type].strength = 1.00;
        h->tf_strenghts[type].period = 8;
    }

    h->lookahead = 1;

    for (int idx = 0; idx < MAX_FILTER_LEN; idx ++) 
    {
        tf_pic_destroy(&(h->pic_bufs[idx].org_pic));
        h->pic_bufs[idx].is_used = 0;
        h->pic_bufs[idx].poc = -1;

        tf_mv_mat_destroy(&(h->pic_bufs[idx].mvs));
        h->pic_bufs[idx].poc_offset = -1;

        tf_pic_destroy(&(h->pic_bufs[idx].crt_pic));
    }

    tf_pic_destroy(&(h->pic_pyramid_a.pic[0]));
    tf_pic_destroy(&(h->pic_pyramid_b.pic[0]));
    tf_pic_destroy(&(h->pic_pyramid_a.pic[1]));
    tf_pic_destroy(&(h->pic_pyramid_a.pic[2]));
    tf_pic_destroy(&(h->pic_pyramid_b.pic[1]));
    tf_pic_destroy(&(h->pic_pyramid_b.pic[2]));

    tf_mv_mat_destroy(&(h->mv_pyramid.mvs[0]));
    tf_mv_mat_destroy(&(h->mv_pyramid.mvs[1]));
    tf_mv_mat_destroy(&(h->mv_pyramid.mvs[2]));
}
