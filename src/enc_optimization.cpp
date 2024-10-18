#include "enc_optimization.h"
#include "enc_optimization_wrapper.h"

#if ALO
#include <iostream>
#include <fstream>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef void* enc_ALO_pointer;

    enc_ALO_pointer* createEncALO(ENC_CTX* ctx)
    {
        EncALO* pEncALO = new EncALO;
        pEncALO->initial(ctx);
        return (enc_ALO_pointer*)pEncALO;
    }

    void destroyEncALO(enc_ALO_pointer* pEncALO)
    {
        EncALO* p = (EncALO*)pEncALO;
        p->destroy();
        delete p;
    }

    void updateOrgPic(ENC_CTX* ctx)
    {
        EncALO* p = (EncALO*)(ctx->pEncALO);
        p->copyPicture(PIC_ORG(ctx), p->getCurrOrgPicBuf());
    }

    void updateRefPic(ENC_CTX* ctx)
    {
        EncALO* p = (EncALO*)(ctx->pEncALO);
        p->copyPicture(PIC_ORG(ctx), p->getLastOrgPicBuf());
    }

    void updateCtuFactor(ENC_CTX* ctx)
    {
        EncALO* p = (EncALO*)(ctx->pEncALO);
        FrameDistortion curFD = p->MotionCompensation(p->getCurrOrgPicBuf(), p->getLastOrgPicBuf(), ctx->poc, ctx->poc - 1, p->getPicWidth(), p->getPicHeight(), p->getWorkBlockSize(), p->getSearchRange());
        p->CalulateCtuFactor(curFD, p->getWorkCtuFactor());
    }

    double adjustCtuLambda(ENC_CTX* ctx)
    {
        EncALO* p = (EncALO*)(ctx->pEncALO);
        return p->CalulateCtuLambda(ctx->core->lcu_num, ctx->lambda[0]);
    }

    void updateCosineSimilarity(ENC_CTX* ctx)
    {
        EncALO* p = (EncALO*)(ctx->pEncALO);
        p->CalulateCosineSimilarity(p->getCurrOrgPicBuf(), p->getLastOrgPicBuf());
    }

#ifdef __cplusplus
}//<-- extern "C"
#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncALO::EncALO()
{
    m_totalFrameNum = 0;
    m_picWidth = 0;
    m_picHeight = 0;

    m_GopSize = 0;
    m_intraPeriod = 0;

    m_encblockSize = 0;
    m_workblockSize = 0;
    m_searchRange = 0;

    m_encblockNumW = 0;
    m_encblockNumH = 0;
    m_workblockNumW = 0;
    m_workblockNumH = 0;

    m_cosineSimilarity = 0.0;
    m_workCtuFactor = nullptr;
    m_currOrgPicBuf = nullptr;
    m_lastOrgPicBuf = nullptr;
}

EncALO::~EncALO()
{
    destroy();
}

void EncALO::initial(ENC_CTX* ctx)
{
    m_totalFrameNum = ctx->param.frames_to_be_encoded;
    m_picWidth = ctx->param.pic_width;
    m_picHeight = ctx->param.pic_height;

    m_intraPeriod = ctx->param.i_period;
    if (m_intraPeriod <= 0 && ctx->param.max_b_frames == 0)
    {
        m_GopSize = GOP_P; // for LD
    }
    else
    {
        m_GopSize = ctx->param.gop_size;
    }

    m_encblockSize = ctx->param.ctu_size;
    m_workblockSize = WORKBLOCKSIZE;
    m_searchRange = ctx->pinter.max_search_range;

    m_encblockNumW = m_picWidth / m_encblockSize + !!(m_picWidth % m_encblockSize);
    m_encblockNumH = m_picHeight / m_encblockSize + !!(m_picHeight % m_encblockSize);
    m_workblockNumW = m_picWidth / m_workblockSize + !!(m_picWidth % m_workblockSize);
    m_workblockNumH = m_picHeight / m_workblockSize + !!(m_picHeight % m_workblockSize);

    m_workCtuFactor = new double[m_encblockNumW * m_encblockNumH];
    m_currOrgPicBuf = new Pixel[m_picWidth * m_picHeight * 3 / 2];
    m_lastOrgPicBuf = new Pixel[m_picWidth * m_picHeight * 3 / 2];
}

void EncALO::destroy()
{
    if (m_workCtuFactor != nullptr)
    {
        delete[] m_workCtuFactor;
        m_workCtuFactor = nullptr;
    }
    if (m_currOrgPicBuf != nullptr)
    {
        delete[] m_currOrgPicBuf;
        m_currOrgPicBuf = nullptr;
    }
    if (m_lastOrgPicBuf != nullptr)
    {
        delete[] m_lastOrgPicBuf;
        m_lastOrgPicBuf = nullptr;
    }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncALO::copyPicture(COM_PIC* src, Pixel* dst)
{
    Pixel* bufY = src->y;
    Pixel* bufCb = src->u;
    Pixel* bufCr = src->v;
    int orgWidthY = src->width_luma;
    int orgWidthCb = src->width_chroma;
    int orgWidthCr = src->width_chroma;
    int orgHeightY = src->height_luma;
    int orgHeightCb = src->height_chroma;
    int orgHeightCr = src->height_chroma;
    int StrideY = src->stride_luma;
    int StrideCb = src->stride_chroma;
    int StrideCr = src->stride_chroma;

    for (int y = 0; y < orgHeightY; y++)
    {
        for (int x = 0; x < orgWidthY; x++)
        {
            dst[x] = bufY[x];
        }
        bufY += StrideY;
        dst += orgWidthY;
    }

    for (int y = 0; y < orgHeightCb; y++)
    {
        for (int x = 0; x < orgWidthCb; x++)
        {
            dst[x] = bufCb[x];
        }
        bufCb += StrideCb;
        dst += orgWidthCb;
    }

    for (int y = 0; y < orgHeightCr; y++)
    {
        for (int x = 0; x < orgWidthCr; x++)
        {
            dst[x] = bufCr[x];
        }
        bufCr += StrideCr;
        dst += orgWidthCr;
    }
}

void EncALO::copyBlock(Pixel* src, Pixel* dst, int srcStride, int dstStride, int blockWidth, int blockHeight)
{
    for (int y = 0; y < blockHeight; y++)
    {
        for (int x = 0; x < blockWidth; x++)
        {
            dst[x] = src[x];
        }
        src += srcStride;
        dst += dstStride;
    }
}

void EncALO::getBlockFromFrame(Block* block, Frame* frame)
{
    Pixel* luma = block->luma;
    Pixel* cb = block->cb;
    Pixel* cr = block->cr;

    int offsetY = block->OriginY * frame->nStrideY + block->OriginX;
    int offsetC = (block->OriginY * frame->nStrideC + block->OriginX) >> 1;

    Pixel* Y = frame->Y + offsetY;
    Pixel* U = frame->U + offsetC;
    Pixel* V = frame->V + offsetC;

    copyBlock(Y, luma, frame->nStrideY, block->BlockWidth, block->BlockWidth, block->BlockHeight);
    copyBlock(U, cb, frame->nStrideC, block->BlockWidth >> 1, block->BlockWidth >> 1, block->BlockHeight >> 1);
    copyBlock(V, cr, frame->nStrideC, block->BlockWidth >> 1, block->BlockWidth >> 1, block->BlockHeight >> 1);
}

void EncALO::CalulateBlockDistortion(Block blockA, Block blockB, double& rMSE, double& rMAD, double& rSAD)
{
    int diff = 0;
    int totalBlockPixels = blockA.BlockWidth * blockA.BlockHeight;
    double dSAD = 0;
    double dSSE = 0;

    for (int i = 0; i < totalBlockPixels; i++)
    {
        diff = blockA.luma[i] - blockB.luma[i];
        dSAD += abs(diff);
        dSSE += double(diff) * double(diff);
    }

    rSAD = dSAD;
    rMSE = dSSE / totalBlockPixels;
    rMAD = dSAD / totalBlockPixels;
}

void EncALO::MotionEstimate(FrameDistortion* curFD, Frame* frameA, Frame* frameB, int searchRange)
{
    int curX = 0;
    int curY = 0;
    int startX = 0;
    int startY = 0;
    int nextX = 0;
    int nextY = 0;

    int top = 0;
    int bottom = 0;
    int left = 0;
    int right = 0;

    double curDistortion = 0;
    double bestDistortion = 1048576;
    double MSE = 0;
    double MAD = 0;
    double SAD = 0;

    Block curBlock, refBlock;
    Block* pCurBlock = &curBlock;
    Block* pRefBlock = &refBlock;

    BlockDistortion* curBD;

    static int dx9[9] = { 0,-2,-1, 0, 1, 2, 1, 0,-1 };
    static int dy9[9] = { 0, 0,-1,-2,-1, 0, 1, 2, 1 };
    static int dx5[5] = { 0,-1, 0, 1, 0 };
    static int dy5[5] = { 0, 0,-1, 0, 1 };

    int* searchPatternX = nullptr;
    int* searchPatternY = nullptr;
    int patternSize = 0;
    int startIdx = 0;
    int idx = 0;
    int flag5p = 0;
    int flag9p = 0;

    uint32_t blockSize = curFD->BlockSize;
    uint32_t TotalBlockNumInWidth = curFD->TotalBlockNumInWidth;
    uint32_t TotalBlockNumInHeight = curFD->TotalBlockNumInHeight;

    for (uint32_t nBH = 0; nBH < TotalBlockNumInHeight; nBH++)
    {
        for (uint32_t nBW = 0; nBW < TotalBlockNumInWidth; nBW++)
        {
            memset(pCurBlock, '\0', sizeof(curBlock));
            memset(pRefBlock, '\0', sizeof(refBlock));
            pCurBlock->OriginX = blockSize * nBW;
            pCurBlock->OriginY = blockSize * nBH;
            pCurBlock->BlockWidth = COM_MIN(blockSize, frameA->FrameWidth - pCurBlock->OriginX);
            pCurBlock->BlockHeight = COM_MIN(blockSize, frameA->FrameHeight - pCurBlock->OriginY);
            getBlockFromFrame(pCurBlock, frameA);

            left = 0 + pCurBlock->OriginX - searchRange;
            right = 0 + pCurBlock->OriginX + searchRange;
            top = 0 + pCurBlock->OriginY - searchRange;
            bottom = 0 + pCurBlock->OriginY + searchRange;
            left = COM_MAX(0, COM_MIN(left, (int)(frameB->FrameWidth - pCurBlock->BlockWidth)));
            right = COM_MAX(0, COM_MIN(right, (int)(frameB->FrameWidth - pCurBlock->BlockWidth)));
            top = COM_MAX(0, COM_MIN(top, (int)(frameB->FrameHeight - pCurBlock->BlockHeight)));
            bottom = COM_MAX(0, COM_MIN(bottom, (int)(frameB->FrameHeight - pCurBlock->BlockHeight)));

            curBD = &curFD->BlockDistortionArray[nBH * TotalBlockNumInWidth + nBW];
            curBD->GlobalBlockNumber = nBH * TotalBlockNumInWidth + nBW;
            curBD->BlockNumInWidth = nBW;
            curBD->BlockNumInHeight = nBH;
            curBD->BlockWidth = pCurBlock->BlockWidth;
            curBD->BlockHeight = pCurBlock->BlockHeight;
            curBD->OriginX = pCurBlock->OriginX;
            curBD->OriginY = pCurBlock->OriginY;
            curBD->SearchRange = searchRange;

            flag5p = 0;
            flag9p = 1;

            startX = pCurBlock->OriginX;
            startY = pCurBlock->OriginY;
            startIdx = 0;
            bestDistortion = 1048576;

            while (flag5p || flag9p)
            {
                if (flag5p)
                {
                    searchPatternX = dx5;
                    searchPatternY = dy5;
                    patternSize = 5;
                }
                else
                {
                    searchPatternX = dx9;
                    searchPatternY = dy9;
                    patternSize = 9;
                }

                for (idx = startIdx; idx < patternSize; idx++)
                {
                    curX = startX + searchPatternX[idx];
                    curY = startY + searchPatternY[idx];

                    if (curX >= left && curX <= right && curY >= top && curY <= bottom)
                    {
                        pRefBlock->OriginX = curX;
                        pRefBlock->OriginY = curY;
                        pRefBlock->BlockWidth = pCurBlock->BlockWidth;
                        pRefBlock->BlockHeight = pCurBlock->BlockHeight;
                        getBlockFromFrame(pRefBlock, frameB);

                        CalulateBlockDistortion(curBlock, refBlock, MSE, MAD, SAD);
                        curDistortion = MSE;

                        if (curDistortion < bestDistortion)
                        {
                            bestDistortion = curDistortion;
                            curBD->MSE = MSE;
                            curBD->MAD = MAD;
                            curBD->SAD = SAD;

                            curBD->MVx = pCurBlock->OriginX - pRefBlock->OriginX;
                            curBD->MVy = pCurBlock->OriginY - pRefBlock->OriginY;
                            curBD->MVL = sqrt(pow2(1.0 * curBD->MVx) + pow2(1.0 * curBD->MVy));

                            nextX = curX;
                            nextY = curY;
                        }
                    }
                }

                if (nextX == startX && nextY == startY)
                {
                    flag9p = 0;
                    flag5p = 1 - flag5p;
                }
                else
                {
                    startX = nextX;
                    startY = nextY;
                }
                startIdx = 1;
            }
        }
    }
}

FrameDistortion EncALO::MotionCompensation(Pixel* srcYUV, Pixel* tarYUV, int srcPoc, int tarPoc, int width, int height, int blockSize, int searchRange)
{
    Frame srcFrame, tarFrame;
    Frame* pSrcFrame = &srcFrame;
    Frame* pTarFrame = &tarFrame;
    FrameDistortion curFD;
    FrameDistortion* pCurFD = &curFD;

    srcFrame.FramePoc = srcPoc;
    srcFrame.FrameWidth = width;
    srcFrame.FrameHeight = height;
    srcFrame.nStrideY = width;
    srcFrame.nStrideC = width >> 1;

    tarFrame.FramePoc = tarPoc;
    tarFrame.FrameWidth = width;
    tarFrame.FrameHeight = height;
    tarFrame.nStrideY = width;
    tarFrame.nStrideC = width >> 1;

    srcFrame.base = srcYUV;
    srcFrame.Y = srcYUV;
    srcFrame.U = srcFrame.Y + width * height;
    srcFrame.V = srcFrame.U + (width * height >> 2);

    tarFrame.base = tarYUV;
    tarFrame.Y = tarYUV;
    tarFrame.U = tarFrame.Y + width * height;
    tarFrame.V = tarFrame.U + (width * height >> 2);

    int nBW = width / blockSize + !!(width % blockSize);
    int nBH = height / blockSize + !!(height % blockSize);
    int nTB = nBH * nBW;

    curFD.CurFramePoc = srcPoc;
    curFD.RefFramePoc = tarPoc;
    curFD.BlockSize = blockSize;
    curFD.CUSize = blockSize;
    curFD.TotalBlockNumInWidth = nBW;
    curFD.TotalBlockNumInHeight = nBH;
    curFD.TotalNumOfBlocks = nTB;
    curFD.BlockDistortionArray = new BlockDistortion[nTB];

    MotionEstimate(pCurFD, pSrcFrame, pTarFrame, searchRange);

    return curFD;
}

void EncALO::CalulateCtuFactor(FrameDistortion& rFD, double* workCtuFactor)
{
    int nBW = rFD.TotalBlockNumInWidth;
    int nBH = rFD.TotalBlockNumInHeight;
    if (workCtuFactor == nullptr)
    {
        return;
    }
    int sampleSize = nBW * nBH;
    double* sampleValue = new double[sampleSize];

    for (int y = 0; y < nBH; y++)
    {
        for (int x = 0; x < nBW; x++)
        {
            sampleValue[x + y * nBW] = log(abs(rFD.BlockDistortionArray[x + y * nBW].SAD) + 1) / log(2.0);
        }
    }

    for (int x = 1; x < nBW; x++)
    {
        int deltaX = rFD.BlockDistortionArray[x].MVx - rFD.BlockDistortionArray[x - 1].MVx;
        int deltaY = rFD.BlockDistortionArray[x].MVy - rFD.BlockDistortionArray[x - 1].MVy;
        double deltaH1 = log((abs(deltaX) + 1) * (abs(deltaY) + 1)) / log(2.0);
        double deltaH2 = log((abs(rFD.BlockDistortionArray[x].MVx) + 1) * (abs(rFD.BlockDistortionArray[x].MVy) + 1)) / log(2.0);
        sampleValue[x] += COM_MIN(deltaH1, deltaH2);
    }
    for (int y = 1; y < nBH; y++)
    {
        int deltaX = rFD.BlockDistortionArray[y * nBW].MVx - rFD.BlockDistortionArray[(y - 1) * nBW].MVx;
        int deltaY = rFD.BlockDistortionArray[y * nBW].MVy - rFD.BlockDistortionArray[(y - 1) * nBW].MVy;
        double deltaH1 = log((abs(deltaX) + 1) * (abs(deltaY) + 1)) / log(2.0);
        double deltaH2 = log((abs(rFD.BlockDistortionArray[y * nBW].MVx) + 1) * (abs(rFD.BlockDistortionArray[y * nBW].MVy) + 1)) / log(2.0);
        sampleValue[y * nBW] += COM_MIN(deltaH1, deltaH2);
    }
    for (int y = 1; y < nBH; y++)
    {
        for (int x = 1; x < nBW; x++)
        {
            int deltaX1 = rFD.BlockDistortionArray[x + y * nBW].MVx - rFD.BlockDistortionArray[x - 1 + y * nBW].MVx;
            int deltaY1 = rFD.BlockDistortionArray[x + y * nBW].MVy - rFD.BlockDistortionArray[x - 1 + y * nBW].MVy;
            int deltaX2 = rFD.BlockDistortionArray[x + y * nBW].MVx - rFD.BlockDistortionArray[x + (y - 1) * nBW].MVx;
            int deltaY2 = rFD.BlockDistortionArray[x + y * nBW].MVy - rFD.BlockDistortionArray[x + (y - 1) * nBW].MVy;
            int deltaX3 = rFD.BlockDistortionArray[x + y * nBW].MVx - rFD.BlockDistortionArray[x - 1 + (y - 1) * nBW].MVx;
            int deltaY3 = rFD.BlockDistortionArray[x + y * nBW].MVy - rFD.BlockDistortionArray[x - 1 + (y - 1) * nBW].MVy;

            double deltaH1 = log((abs(deltaX1) + 1) * (abs(deltaY1) + 1)) / log(2.0);
            double deltaH2 = log((abs(deltaX2) + 1) * (abs(deltaY2) + 1)) / log(2.0);
            double deltaH3 = log((abs(deltaX3) + 1) * (abs(deltaY3) + 1)) / log(2.0);
            double deltaH4 = log((abs(rFD.BlockDistortionArray[x + y * nBW].MVx) + 1) * (abs(rFD.BlockDistortionArray[x + y * nBW].MVy) + 1)) / log(2.0);

            sampleValue[x + y * nBW] += COM_MIN(deltaH1, COM_MIN(deltaH2, COM_MIN(deltaH3, deltaH4)));
        }
    }

    double meanValue = 0.0;
    double varValue = 0.0;
    double standardDeviation = 0.0;
    double minValue = 0.0;
    for (int y = 0; y < nBH; y++)
    {
        for (int x = 0; x < nBW; x++)
        {
            meanValue += sampleValue[x + y * nBW];
            minValue = COM_MIN(minValue, sampleValue[x + y * nBW]);
        }
    }
    meanValue /= sampleSize;
    for (int y = 0; y < nBH; y++)
    {
        for (int x = 0; x < nBW; x++)
        {
            varValue += (sampleValue[x + y * nBW] - meanValue) * (sampleValue[x + y * nBW] - meanValue);
        }
    }
    if (sampleSize == 1)
    {
        varValue = 0.0;
    }
    else
    {
        varValue /= (sampleSize - 1);
    }
    standardDeviation = sqrt(varValue);

    /******************************************************************************/
    /*Calulate Ctu Fractor*/
    Block curCtu;
    Block* pCurCtu = &curCtu;
    clearCtuFactor();

    double avgFactor = 0.0;
    double epslon = 1e-6;
    double adjustedRatio = 0.0;
    if (m_intraPeriod <= 0)
    {
        adjustedRatio = 0.42;
    }
    else if (m_intraPeriod > 1)
    {
        adjustedRatio = 0.3;
    }
    for (int y = 0; y < m_encblockNumH; y++)
    {
        for (int x = 0; x < m_encblockNumW; x++)
        {
            pCurCtu->OriginX = x * m_encblockSize;
            pCurCtu->OriginY = y * m_encblockSize;
            pCurCtu->BlockWidth = COM_MIN(m_encblockSize, m_picWidth - (int)pCurCtu->OriginX);
            pCurCtu->BlockHeight = COM_MIN(m_encblockSize, m_picHeight - (int)pCurCtu->OriginY);
            int numBW = pCurCtu->BlockWidth / m_workblockSize + !!(pCurCtu->BlockWidth % m_workblockSize);
            int numBH = pCurCtu->BlockHeight / m_workblockSize + !!(pCurCtu->BlockHeight % m_workblockSize);
            double deltaValue = 0.0;
            for (int j = 0; j < numBH; j++)
            {
                for (int i = 0; i < numBW; i++)
                {
                    int cuX = x * (m_encblockSize / m_workblockSize) + i;
                    int cuY = y * (m_encblockSize / m_workblockSize) + j;
                    deltaValue += (sampleValue[cuX + cuY * nBW] - meanValue) / (standardDeviation + epslon);
                }
            }
            if (m_cosineSimilarity < 0.99)
            {
                m_cosineSimilarity = 0.0;
            }
            else
            {
                m_cosineSimilarity = 1.0;
            }
            workCtuFactor[x + y * m_encblockNumW] = exp(adjustedRatio * m_cosineSimilarity * (deltaValue / (numBW * numBH)));
            if (meanValue > 13.5 && varValue > 6)
            {
                workCtuFactor[x + y * m_encblockNumW] = COM_CLIP3(exp(-0.01 * adjustedRatio * m_cosineSimilarity), exp(0.01 * adjustedRatio * m_cosineSimilarity), workCtuFactor[x + y * m_encblockNumW]);
            }
            else
            {
                workCtuFactor[x + y * m_encblockNumW] = COM_CLIP3(exp(-1 * adjustedRatio * m_cosineSimilarity), exp(1 * adjustedRatio * m_cosineSimilarity), workCtuFactor[x + y * m_encblockNumW]);
            }
            avgFactor += workCtuFactor[x + y * m_encblockNumW] / (m_encblockNumW * m_encblockNumH);
        }
    }

    delete[] sampleValue;
    delete[] rFD.BlockDistortionArray;
}

double EncALO::CalulateCtuLambda(int ctuIdx, double oldLambda)
{
    int totalCtuNum = m_encblockNumW * m_encblockNumH;
    double adjustedLambda = 0.0;
    if (ctuIdx < totalCtuNum && oldLambda != 0)
    {
        adjustedLambda = oldLambda * m_workCtuFactor[ctuIdx];
    }
    return adjustedLambda;
}

void EncALO::clearCtuFactor()
{
    for (int i = 0; i < m_encblockNumW * m_encblockNumH; i++)
    {
        m_workCtuFactor[i] = 1;
    }
}

void EncALO::CalulateCosineSimilarity(Pixel* srcYUV, Pixel* tarYUV)
{
    int* srcArr = new int[256];
    int* tarArr = new int[256];
    memset(srcArr, 0, sizeof(int) * 256);
    memset(tarArr, 0, sizeof(int) * 256);

    int totalPixels = m_picWidth * m_picHeight;
    double product = 0.0;
    double srcMold = 0.0;
    double tarMold = 0.0;
    int srcSum = 0;
    int tarSum = 0;

    for (int i = 0; i < totalPixels; i++)
    {
        srcArr[(srcYUV[i] >> 2)]++;
        tarArr[(tarYUV[i] >> 2)]++;
    }
    for (int j = 0; j < 256; j++)
    {
        product += (long long)srcArr[j] * (long long)tarArr[j];
        srcMold += (long long)srcArr[j] * (long long)srcArr[j];
        tarMold += (long long)tarArr[j] * (long long)tarArr[j];
        srcSum += srcArr[j];
        tarSum += tarArr[j];
    }
    if (srcSum == totalPixels && tarSum == totalPixels)
    {
        m_cosineSimilarity = product / (sqrt(srcMold) * sqrt(tarMold));
    }

    if (srcArr != nullptr)
    {
        delete[] srcArr;
        srcArr = nullptr;
    }
    if (tarArr != nullptr)
    {
        delete[] tarArr;
        tarArr = nullptr;
    }
}

#endif