#pragma once

#ifndef _ENC_OPTIMIZATION_WRAPPER_H_
#define _ENC_OPTIMIZATION_WRAPPER_H_

#include "enc_def.h"
#include <stdint.h>

#if ALO

#ifdef __cplusplus
extern "C" {
#endif

typedef void* enc_ALO_pointer;

enc_ALO_pointer* createEncALO(ENC_CTX* ctx);

void destroyEncALO(enc_ALO_pointer* pEncALO);

void updateOrgPic(ENC_CTX* ctx);

void updateRefPic(ENC_CTX* ctx);

void updateCtuFactor(ENC_CTX* ctx);

double adjustCtuLambda(ENC_CTX* ctx);

void updateCosineSimilarity(ENC_CTX* ctx);

#ifdef __cplusplus
} //<-- extern "C"
#endif

#endif
#endif