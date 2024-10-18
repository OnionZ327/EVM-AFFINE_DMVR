#pragma once

#ifndef _ENC_OPTIMIZATION_H_
#define _ENC_OPTIMIZATION_H_

#include "enc_def.h"
#include <stdint.h>
#include "time.h"

#if ALO

#undef max
#undef min

#define MAXBLOCKSIZE 128
#define WORKBLOCKSIZE 16
#define SEARCHRANGE 64

#define pow2(a)        ((a)*(a))

typedef int16_t Pixel;   ///< pixel type (for 10 bits)

typedef struct Block
{
    uint32_t     BlockWidth;
    uint32_t     BlockHeight;
    uint32_t     OriginX;
    uint32_t     OriginY;
    Pixel        luma[MAXBLOCKSIZE * MAXBLOCKSIZE];
    Pixel        cr[MAXBLOCKSIZE * MAXBLOCKSIZE >> 2];
    Pixel        cb[MAXBLOCKSIZE * MAXBLOCKSIZE >> 2];
} Block;

typedef struct Frame
{
    int      FramePoc;
    uint32_t FrameWidth;
    uint32_t FrameHeight;
    uint32_t nStrideY;
    uint32_t nStrideC;
    Pixel* base;
    Pixel* Y;
    Pixel* U;
    Pixel* V;
} Frame;

typedef struct BlockDistortion
{
    uint32_t       GlobalBlockNumber;
    uint16_t       BlockNumInHeight;
    uint16_t       BlockNumInWidth;
    uint16_t       BlockWidth;
    uint16_t       BlockHeight;
    uint16_t       OriginX;
    uint16_t       OriginY;
    uint16_t       SearchRange;
    int16_t        MVx;
    int16_t        MVy;
    double         SAD;
    double         MSE;
    double         MAD;
    double         MVL;
    int16_t        BlockQP;
    double         BlockLambda;
    int16_t        BlockType;
} BlockDistortion, BD;

typedef struct FrameDistortion
{
    int                     CurFramePoc;
    int                     RefFramePoc;
    uint32_t                BlockSize;
    uint32_t                CUSize;
    uint32_t                TotalNumOfBlocks;
    uint32_t                TotalBlockNumInHeight;
    uint32_t                TotalBlockNumInWidth;
    BD* BlockDistortionArray;
    // FD*                     subFrameDistortionArray;
} FrameDistortion, FD;

typedef struct DistortionList
{
    uint32_t         TotalFrameNumber;
    uint32_t         FrameWidth;
    uint32_t         FrameHeight;
    FD* FrameDistortionArray;
} DistortionList, DL;

typedef struct ReferencedValue
{
    int* referencedCount;
    int** referencedList;
    double* referencedValue;
    int* updateFlag;
} ReferenceValue, RV;

class EncALO
{
private:
    int m_totalFrameNum;
    int m_picWidth;
    int m_picHeight;

    int m_GopSize;
    int m_intraPeriod;

    int m_encblockSize;
    int m_workblockSize;
    int m_searchRange;

    int m_encblockNumW;
    int m_encblockNumH;
    int m_workblockNumW;
    int m_workblockNumH;

    double m_cosineSimilarity;
    double* m_workCtuFactor;

    Pixel* m_currOrgPicBuf;
    Pixel* m_lastOrgPicBuf;

public:
    EncALO();
    ~EncALO();

    void setTotalFrameNum(int n) { m_totalFrameNum = n; }
    int getTotalFrameNum() { return m_totalFrameNum; }

    void setPicWidth(int width) { m_picWidth = width; }
    int getPicWidth() { return m_picWidth; }

    void setPicHeight(int height) { m_picHeight = height; }
    int getPicHeight() { return m_picHeight; }

    void setGopSize(int gopSize) { m_GopSize = gopSize; }
    int getGopSize() { return m_GopSize; }

    void setIntraPeriod(int i) { m_intraPeriod = i; }
    int getIntraPeriod() { return m_intraPeriod; }

    void setSearchRange(int searchrange) { m_searchRange = searchrange; }
    int getSearchRange() { return m_searchRange; }

    int getEncblockNumW() { return m_encblockNumW; }
    int getEncblockNumH() { return m_encblockNumH; }
    int getEncBlockSize() { return m_encblockSize; }
    int getWorkblockNumW() { return m_workblockNumW; }
    int getWorkBlockNumH() { return m_workblockNumH; }
    int getWorkBlockSize() { return m_workblockSize; }

    void setCosineSimilarity(double d) { m_cosineSimilarity = d; }
    double getCosineSimilarity() { return m_cosineSimilarity; }

    double* getWorkCtuFactor() { return m_workCtuFactor; }
    Pixel* getCurrOrgPicBuf() { return m_currOrgPicBuf; }
    Pixel* getLastOrgPicBuf() { return m_lastOrgPicBuf; }

    void initial(ENC_CTX* ctx);

    void destroy();

    void copyPicture(COM_PIC* src, Pixel* dst);

    void copyBlock(Pixel* src, Pixel* dst, int srcStride, int dstStride, int blockWidth, int blockHeight);

    void getBlockFromFrame(Block* block, Frame* frame);

    void CalulateBlockDistortion(Block blockA, Block blockB, double& rMSE, double& rMAD, double& rSAD);

    void MotionEstimate(FrameDistortion* curFD, Frame* frameA, Frame* frameB, int searchRange);

    FrameDistortion MotionCompensation(Pixel* srcYUV, Pixel* tarYUV, int srcPoc, int tarPoc, int width, int height, int blockSize, int searchRange);

    void CalulateCtuFactor(FrameDistortion& rFD, double* workCtuFactor);

    double CalulateCtuLambda(int ctuIdx, double oldLambda);

    void clearCtuFactor();

    void CalulateCosineSimilarity(Pixel* srcYUV, Pixel* tarYUV);
};

#endif
#endif