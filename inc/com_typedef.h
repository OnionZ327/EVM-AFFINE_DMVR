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

#ifndef _COM_H_
#define _COM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define EVM_VERSION "0.5"

#define FIELD_CODING                       1 // futl: field coding support

/* AVS4 macros*/
#define CCNPM                              1 // M7909: cross-component nonlinear intra prediction method
#define SAWP_WEIGHT_OPT                    1 // M7911: SAWP reference weight optimization
#define DT_TRANSFORM                       1 // M7908: the transformation optimization of derived modes
#define M7669                              3 // 1: hpm15.3, 2: hpm15.3+RefAreaExt, 3: hpm15.3+RefAreaExt+RefBurstUnitNumConstraint
#define CIBC                               1 // M7910: chroma IBC
#define IBC_ME_OPT                         1 // M7928: IBC ME optimization
#define TEMPORAL_FILTER_SCC                1 // M7906: Add frame-level control on the encoder side for temporal filter
#define DEST                               1 // M7926: Decoder-side Enhanced Secondary Transform
#define SBT_EXT                            1 // M8036: SBT extension mode
#define PARITY_HIDING                      1 // M8018: Parity hiding for coefficient coding
#define ISC_RSD                            1 // M8029: Add reverse scan direction string type of intra string copy mode
#define CCNPM_TEMPLATE_OPT                 1 // M8038: Multi-template CCNPM
#define INTER_CCNPM                        1 // M8039: cross-component nonlinear inter prediction method 
#define PARTITIONING_OPT                   1 // M8037: Block partitioning optimization
#define AWP_ENH                            1 // M8040: Enhanced AWP mode
#define IBC_ENH                            1 // M8041: Enhanced IBC mode for camera-captured contents
#define OBMC_SCC                           1 // M8045: frame-level control of OBMC in SCC cases
#define FBHT                               1 // M8031: Feature-based IBC hash table
#define HACD                               1 // M8200: History-based Affine Candidate Derivation
#define IF_LUMA12_CHROMA6                  1 // M8239: Extension of interpolation filter
#define CTU_256                            1 // M8248: Adjust the size of LCU
#define DSAWP                              1 // M8241: Decoder-side derivation of SAWP mode
#define INTER_CCNPM_OPT                    1 // M8247: Optimization of inter CCNPM
#define IST_CHROMA                         1 // M8249: apply the IST core to chroma transform
#define ALF_SHAPE_IMPROVEMENT              1 // M8240: Filter shape improvement for ALF
#define IBC_OPT_SCC                        1 // M8242: IBC optimization for SCC
#define ECCPM                              1 // M8204: Enhanced Chroma Component Prediction Mode
#define OBMC_TEMP                          1 // M8322: Template-based OBMC weight selection
#define INTER_TM                           1 // M8343: Template matching for inter prediction
#define CCNPM_LINE_BUFFER_REDUCTION        1 // M8344: Line buffer reduction for CCNPM
#define FAST_SCC_PRECODING                 1 // M8331: Fast SCC precoding
#define SCC_ENC_OPT                        1 // M8376: SCC encoder optimization
#define IST_SBT_IBC                        1 // M8354: IST and SBT for camera-captured IBC mode

#if ISC_RSD
#define RSD_OPT                            1 // M8330: RSD encoder side optimization
#endif
#if INTER_TM
#define TM_CANDS                           4
#define TM_WIDTH                           4
#define MAX_TM_ITERATIONS                  30
#define SIMPLIFIED_MAX_TM_ITERATIONS       10                  
#define MAX_TM_RDO                         2
#endif

#if DEST
#define ODEST                              1 // M8043: Optimization Method of Decoder-side Enhanced Secondary Transform
#define DEST_PH                            1 // M8035: Add picture level control for DEST
#endif

#if IF_LUMA12_CHROMA6
#define IF_LUMA12_CHROMA6_SIMD             1 // SIMD implementation of M8239
#endif

#if M7669 == 3
#define RBU_LIMIT_GS                       1 // limit the GS RBU number
#define RBU_LIMIT_CSS                      1 // limit the CSS RBU number
#define RBU_LIMIT_IBC                      1 // limit the IBC RBU number

#if RBU_LIMIT_GS||RBU_LIMIT_CSS||RBU_LIMIT_IBC
#define RBU_W_LOG2                         4 //log_w of RBU (reference burst unit)
#define RBU_H_LOG2                         3 //log_h of RBU (reference burst unit)
#define RCNC_Parameter                     16 //pixel number in a CU (or VPDU) / RCNC_Parameter is the upper limit of RBU number allowed for the CU (or VPDU)
#define LIMIT_ON_STRPE_NUM                 1//pixel number in a CU (or VPDU) / RCNC_Parameter is the upper limit of RBU number allowed for the CU (or VPDU)
#endif 
#define ExtStrGS                           1// 1 or 0: enable or disable string length constraint in Additional Reference Area (ARA) for general string
#define ExtStrCSS                          1// 1 or 0: enable or disable string length constraint in Additional Reference Area (ARA) for cu size string
#define ExtStrIBC                          1// 1 or 0: enable or disable string length constraint in Additional Reference Area (ARA) for IBC
#define ExtStrLenMin                       16//minimum string length in Additional Reference Area (ARA)
#define ERR_BUGFIX                         1//1:Extended reference range bugfix
#define REF_RANGE_FIX                      1 //1:Make code writing more standardized
#define SRAM_SIZE_128_LOG2                 7 //The size of the SRAM is 128x128
#endif  
#if M7669 == 2
#define STR_SELECT_OPT                     1
#endif 
#if M7669 == 3||M7669 == 2
#define IBC_REF_RANGE_XqLCU                128 //0: hpm153 code; 1: ref range is One LCU(Original reference range); X>=2: ref range is X LCU
#define CSS_REF_RANGE_XqLCU                128 //0: hpm153 code; 1: ref range is One LCU(Original reference range); X>=2: ref range is X LCU
#define GS_REF_RANGE_XqLCU                 128 //0: hpm153 code; 1: ref range is One LCU(Original reference range); X>=2: ref range is X LCU

#define IBC_REF_RANGE_XLCUR                8 //0: hpm153 code; 1: ref range is current LCU rows; Y>=2: ref range is Y LCU rows (current and above rows);
#define CSS_REF_RANGE_XLCUR                8 //0: hpm153 code; 1: ref range is current LCU rows; Y>=2: ref range is Y LCU rows (current and above rows);
#define GS_REF_RANGE_XLCUR                 8 //0: hpm153 code; 1: ref range is current LCU rows; Y>=2: ref range is Y LCU rows (current and above rows);

#define IBC_CROSS_64                       1 //1 or 0:allowed or no allowed to cross  64x64;
#define CSS_CROSS_64                       1 //1 or 0:allowed or no allowed to cross  64x64;
#define GS_CROSS_64                        1 //1 or 0:allowed or no allowed to cross  64x64;
#define IBSC_RIGHT_BOUND                   1 //0: keep the current code w/o any change;1:specify the right boundary of several lcu columns including the current lcu column as the right boundary of the reference range 
#define IBC_MV_BITS_EST                    1 //0: hpm153 code; 1: ibc reference range is extended
#define DEC_RANGE_EXTEND                   1 //0：hpm153 code  ;1：It needs to be set to 1 when the reference range is extended 
#endif

#if IBC_OPT_SCC
#define IBC_APF                            1 // IBC adaptive prediction filter
#define IBC_SIZE_EXT                       1 // IBC size extension
#endif

/*end of AVS4 macros*/

/* AVS3 phase-2 macros*/
//intra
#define EIPM                               1 // M4993: extended intra prediction mode
#define MIPF                               1 // M5079: multiple intra prediction filter
#define PMC                                1 // M5158: Prediction from Multiple Cross-components
#define IPF_CHROMA                         1 // M5385 M5418: enable intrapf for chroma components
#define IIP                                1 // M5826: Improved Intra Prediction
#define SAWP                               1 // M6099: (CE1 related) Spatial Angular Weighted Prediction
#define UNIFIED_INTRA_CLIP                 1 // M6519: Unified intra clip in intra prediction
//inter
#define AFFINE_DMVR                        0
#define DMVR                               1 // M4813: decoder-side motion vector refinement
#define BIO                                1 // M4762: Bi-directional Optical Flow
#define FIX_395                            1 // Fix issue 395
#define BGC                                1 // M5398: Bi-directional Gradient Correction 
#define INTERPF                            1 // M4812: inter prediction filtering
#define MVAP                               1 // M4926: motion vector angel prediction
#define AFFINE_UMVE                        1 // M5095: Enhanced Affine Skip/Direct Mode
#define AWP                                1 // M5142: angular weighted prediction & M5143: AWP list construction optimization 
#define ETMVP                              1 // M5330(CE2-1.1):Enhanced temporal motion vector prediction
#define UMVE_ENH                           1 // M5379: enable the combination of interPF and UMVE modes; picture-level UMVE offset adaptation
#define SUB_TMVP                           1 // M5396(CE2-2.1) + M5397: NxN subblock TMVP
#define ASP                                1 // M5801 (CE2-3.2): affine secondary prediction
#define OBMC                               1 // M6105: overlapped block motion compensation
#define IPC                                1 // M6129: inter predictor correction (linear model compensation for luma and one value correction for chroma)
#define UNIFIED_HMVP                       1 // M6471: two minor changes on HMVP.
//transform & quantization
#define IST                                1 // M4772: implicit selection of transforms for intra residual
#define SBT                                1 // M4876: sub-block transform for inter residual
#define EST                                1 // M5159: Enhanced Secondary Transform
#define ST_CHROMA                          1 // M5874: Secondary transform for intra chroma residual
#define CUDQP                              1 // M5877: cu-level qp adjustment
//residual coding & CABAC
#define SRCC                               1 // M4763: scan region based coefficient coding
#define CABAC_MULTI_PROB                   1 // M4867 M5139: counter-based multi-probability CABAC
//scc tools (off for class A/B/C in CTC, and enabled for scc seqs)
#define USE_IBC                            1 // M4859: intra block copy [cannot off now]
#define FIMC                               1 // M4972: frequency-based intra mode coding
#if IST
#define ISTS                               1 // M5160: Implicit Selection of Transform Skip
#endif
#define USE_SP                             1 // M5226: String Prediction 
#if ISTS
#define TS_INTER                           1 // M5601: Transform skip for inter-coded blocks
#define ETS                                1 // M5954: Extended Transform Skip
#define IBC_TS                             1 // M6260: Indicate DCT/TS in explicit way for IBC
#if SRCC
#define TS_GTX                             1 // M6137: SRCC modification when transform skip enabled
#endif
#endif
// in-loop filter tools
#define ESAO                               1 // M5374: Enhanced Sample Adaptive Offset
#define DBR                                1 // M5587: deblocking refinement
#define ALF_SHAPE                          1 // M5589: increased ALF coefficient
#if ALF_SHAPE
#define ALF_SHIFT                          1 // M6151: ALF adaptive shift
#endif
#define ALF_IMP                            1 // M5964: ALF improvements
#define CCSAO                              1 // M5800: Cross-component Sample Adaptive Offset
#define DBK_SCC                            1 // M5221 + M5869: improved deblock filter
// pre-out-loop filter tools
#define TEMPORAL_FILTER                    1 // M7255: Temporal Filter
#if TEMPORAL_FILTER
#define DQP_OPT                            1 // M7382: Spatial temporal optimized lcu delta qp
#endif
//detailed change on each new coding tool
#if DBR
#define EDBR                               1 // M6126：conduct DBR for BS equals 0
#define FIX_382                            1 // Fix issue #382
#endif

#if SBT
#define ISBT                               1 // M5952: Implicit signaling for SBT
#endif

#if SAWP
#define SAWP_MPM_SIMP                      1 // M6286: SAWP MPM siplification
#define SAWP_SCC                           1 // M6190: SAWP & AWP alignment in SCC cases
#endif

#if PMC
#define EPMC                               1 // M5659: extended PMC
#endif

#if AWP
#define AWP_MVR                            1 // M5380: angular weighted prediction with motion vector refinement
#define BAWP                               1 // M6092: 4x4 block based AWP mode for P picture
#define FIX_372                            1
#endif

#if IPC
#define IPC_FAST                           1  //fast alogorithm
#define IPC_SB8                            1 // M6262: restrict samples on 8x8 for MVAP and SBTMVP in IPC
#endif

#if UNIFIED_HMVP
#define UNIFIED_HMVP_1                     1 // (1) HMVP Table does not updated when cu mode is SbTmvp, MVAP and ETMVP.
#define UNIFIED_HMVP_2                     1 // (2) The candidate in HMVP Table should not check the redundancy with the traditional temporal MVP when profile Id is 0x32 or 0x30.
#endif

#if OBMC
#define DISABLE_OBMC_AFFINE                1 // M6341: disable OBMC for affine mode
#define DISABLE_OBMC_LOCAL_CHROMA_TREE     1 // M6435: disable OBMC for local chroma tree
#endif

#if CUDQP
#define CUDQP_QP_MAP                       1 // generate 16x16 QP map based on texture variance
#define ONLY_ONE_DQP                       1 // only one delta QP in one QG (useful as 0 for conformance test, since we support multiple QPs in one QG)
#endif

#if ESAO
#define ESAO_PH_SYNTAX                     1
#endif

#if ESAO_PH_SYNTAX
#define ESAO_ENH                           1 // M6112: Improvement on ESAO
#endif

#if CCSAO
#define CCSAO_PH_SYNTAX                    1
#define CCSAO_LINE_BUFFER                  1 // M6281: reduce CCSAO line buffe
#endif

#if CCSAO_PH_SYNTAX
#define CCSAO_ENHANCEMENT                  1 // M6051: alternative set and chroma BO for CCSAO
#endif

#if CCSAO_ENHANCEMENT
#define ECCSAO                             1 // M6110: CE3-1 related: Improvement on CCSAO
#define CCSAO_RATE                         1 // M6162: encoder modification of CCSAO enhancement
#endif

#if USE_IBC
#define IBC_ABVR                           1 // M5070: IBC adaptive block vector resolution
#define IBC_BVP                            1 // M5081: IBC block vector prediction
#endif

#if SBT
#define SBT_RESTRICT                       1 // M6132: the ratio restriction of sbt
#endif

//code optimization
#define HDR_CHROMA_QP_OFFSET               1 // M6621: Frame level adaptive Chroma QP Offset for HDR content
#define QP_ADJUST_RA                       1 // M6122: QP adaptive parameter Adjustment under RA configuration

#define INTER_ME_MVLIB                     1 // M5527: improved EPZS motion estimation based on motion vector library
#define ENC_ME_IMP                         1 // M5806: gradient-based motion estimation and affine CPMV library

#if USE_IBC && USE_SP
#define SCC_CROSSINFO_ULTILIZE             1 // M5605: SCC cross info ultilize  
#endif

#define REUSE_CU_SPLIT_OPT                 1 // M5900: Adjust threshold for reusing cu results

//high-level
#define PHASE_2_PROFILE                    1 // 
#define RPL_GOP32                          1 // M6161: Hierarchical GOP structure of 32 for random access

#if PHASE_2_PROFILE
#define BIT_DEPTH_EXT                      1 // M6266: support 8bit or 10bit internal encoding for 10bit profile
#define SH_UNIFY                           1 // M6298: modification of sequence header
#define NN_HOOK                            1 // M6363: Hook for Neural Network Coding Tools
#endif

#if BIT_DEPTH_EXT
#define BIT_DEPTH_LIMIT                    1 // M6472: input bitDepth is no more than encoding bitDepth
#endif

#if SH_UNIFY
#define SH_ETSCPM                          1 // M6352: sequence header for Pmc and enhanced Tscpm
#define PH_UNIFY                           1 // modification of picture header 
#define PH_UNIFY_ISTS                      1 // modification of ISTS picture header 
#endif

#if NN_HOOK
#define NN_FILTER                          1 // M6363: Hook for Neural Network Filter
#endif

#define ALO                                1 // M6385: Adaptive Lambda Optimization for Lcu

/*end of AVS phase-2 macros*/

/* ------------------------BELOW HERE is related to AVS3 phase 1 -----------------------*/
/*non-normative bugfix for AVS3 phase-1 after HPM4.0.1 (if any, add here) */

/*end of non-normative bugfix */

/*normative bugfix for AVS3 phase-1 after HPM4.0.1 (if any, add here) */
#define AFFINE_MEMORY_CONSTRAINT           1 // M6157: Affine CPMVs constraint for reducing memory access

/*end of normative bugfix */



//high level syntax
#define WRITE_MD5_IN_USER_DATA             1 // write per-frame MD5 into user_data after picture header
#define REPEAT_SEQ_HEADER                  1 // add sequence header before each I frame

#define HLS_RPL                            1
#if HLS_RPL
#define DOI_CYCLE_LENGTH                 256 // the length of the DOI cycle.
#endif

#define PATCH                              1
#if PATCH
#define PATCH_M4839                        1
#define PATCH_HEADER_PARAM_TEST            0 // different patches have different patch-header patch_sao_enable_flag parameters
#define BUGFIX_386                         1 // min_patch_width shouuld larger than 2*LCU_width
#endif

//partition
#define EQT                                1

/*  ---------------------------   conformance test related, start   -------------------------------------*/
#define PRINT_SQH_PARAM_DEC                1
#define FIXED_SPLIT                        0  // fixed split pattern [must be OFF in formal test] <<<<<<<<<----------NOTE
#if FIXED_SPLIT
#define FS_ALL_COMBINATION                 0  // test split combination of 6 depth
#if FS_ALL_COMBINATION
#define START_QT_DEPTH                     0
#define START_SPLIT_MODE                   0  // 0: not constrain; 1~5: constrain 1 split at start qt depth
#define VERIFY_SPLIT                       0  // print target split combination for each CTU
#define FS_SIMPLE_ORDER                    1  // simple order of split modes (from smallest mode value to the largest)
#endif
#define FS_SAME_SIZE_PER_X_CTU             1  // constrained size changed per X CTUs
#if FS_SAME_SIZE_PER_X_CTU
#define FS_SAME_SIZE_X_VAL                 1  // can be any value larger than 0
#endif
#endif
/*  ---------------------------   conformance test related, end     -------------------------------------*/

//coding mode constraint for region of 64 pixels
#define MODE_CONS                          1  // [this macro cannot be off]
//chroma no split for avoiding 2xN chroma blocks
#define CHROMA_NOT_SPLIT                   1  // [this macro cannot be off]

//for DT and PBT
#define TB_SPLIT_EXT                       1  // extend the framework to support multiple luma prediction & transform blocks (support PBT and Intra DT) [this macro cannot be off]
#if TB_SPLIT_EXT
//fast algorithm (common)
#define TR_SAVE_LOAD                       1  // fast algorithm for PBT
#define TR_EARLY_TERMINATE                 1  // fast algorithm for PBT
#endif
#define DT_PARTITION                       1  // [this macro cannot be off] (DT can be turn off ONLY by configure)
#if DT_PARTITION
#define DT_SYNTAX                          1  // syntax change to support DT (must be 1)
//DT_INTRA
#define DT_INTRA_FAST_BY_RD                1  // fast algorithm: early skip based on RD cost comparison
#define DT_SAVE_LOAD                       1  // fast algorithm: save & load best part_size
#endif
#define PRINT_CU                           0
#define PRINT_CU_LEVEL_2                   0
#define PRINT_HMVP_FIFO                    0
#define PRINT_TRANSFORM_TABLE              0
#define DEUBG_TEST_CHANGE_HORI_VERT_SIZE   0  //change horizontal_size and vertical_size to (8xN-1) in sqh coding
#define PSNR_1020                          0  //use 1020 as peak value for 10-bit picture in PSNR calculation (for comparing PSNR with HM and VTM)
//**************** This part needs clean-up later (End)

//intra
#define TSCPM                              1
#if TSCPM
#define ENHANCE_TSPCM                      1
#if ENHANCE_TSPCM || PMC || EPMC
#define ENHANCE_TSCPM_BUGFIX               1 // M6140: TSCPM bugfix
#define ENHANCE_LT_MODE                    1 // M6140: add an enhanced LT mode for ENHANCE_TSCPM, PMC and EPMC
#endif
#endif
#define IPCM                               1

//inter
#define BD_AFFINE_AMVR                     1  // M4565 combine Affine and AMVR

#define EXT_AMVR_HMVP                      1
#if EXT_AMVR_HMVP
#define FAST_EXT_AMVR_HMVP                 1  // M6104: fast mvp decision
#endif

#define SMVD                               1  // Symmetric MVD mode

#define INTER_CU_CONSTRAINT                1

#define SEP_CONTEXT                        1 // Separate context for useless bin
//transform


// filter
#define DEBLOCK_M4647                      1

// others
#define USE_RDOQ                           1 // Use RDOQ
#define RDO_DBK                            1 // include DBK changes to luma samples into distortion
#if RDO_DBK
#define RDO_DBK_LUMA_ONLY                  1 // M5315: DBK encoder speed up 
#endif
#define CFG_LEVEL_ID                       1 // config the level id from the cmd

#define AFFINE_MVF_VERIFY                  0 // for debugging, clean up later

#define CPMV_BIT_DEPTH                     18
#if CPMV_BIT_DEPTH == 18
typedef int                                CPMV;
#else
typedef short                              CPMV;
#endif
#define COM_CPMV_MAX                       ((s32)((1<<(CPMV_BIT_DEPTH - 1)) - 1))
#define COM_CPMV_MIN                       ((s32)(-(1<<(CPMV_BIT_DEPTH - 1))))

#define LIBVC_ON                           1 // use Library Picture
#if LIBVC_ON
#define IPPPCRR                            1
#if IPPPCRR
#define PB_REF_LIB                         1
#define LIB_PIC_UPDATE                     1 // M5422 adaptive library picture update
#endif
#define CRR_ENC_OPT_CFG                    1 // M5533 CRR encoding optimization about configuration
#endif
#define EXTENSION_USER_DATA                1
#if EXTENSION_USER_DATA
#define TEMPORAL_SCALABILITY_EXTENSION     1 // M6257
#if TEMPORAL_SCALABILITY_EXTENSION
#define SPS_TRANSMIT_TSE                   0 // control whether transmit temporal scalability extension data
#endif
#define CRR_EXTENSION_DATA                 1 // M4822: cross random-access-point referencing extension 
#define AIR                                1 // M6191: adaptive intra refresh
#define HDR_DISPLAY                        1
#define HDR_METADATA_EXTENSION             1 // hdr_dynamic_metadata_extension
#endif

#define WEIGHTED_SATD                      1 // M5382: Improved SATD based cost calculation

//fast algorithm
#define ENC_ECU_DEPTH                      4 // for early CU termination
#define ENC_ECU_ADAPTIVE                   1 // for early CU termination
#define MULTI_REF_ME_STEP                  1 // for ME speed-up
#define FAST_LD                            1 // M5557: fast me for lowdelay case
#define BET_SPLIT_DECSION                  1 // M5965: Test2, fast BET split decision based on QT split result

//bbv
#define BBV                                1 // for Bitstream Buffer Verifier
#if BBV
#define BBV_DELAY_MAX                      0xFFFFFFFF
#define BBV_CHECK_FRAMES_MAX               2000
#define BBV_LIBVC                          1 // whether the bbv is with libvc
#endif
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                              SIMD Optimizations                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
#define X86_SSE                            1
#if X86_SSE
#define SIMD_MC                            1
#define SIMD_SAD                           1
#define SIMD_SSD                           1
#define SIMD_DIFF                          1
#define SIMD_HAD_SAD                       1
#define SIMD_AFFINE                        1
#if ASP
#define SIMD_ASP                           1
#endif // ASP
#if ENC_ME_IMP
#define SIMD_GRAD_ME                       1
#endif
#else
#define SIMD_MC                            0
#define SIMD_SAD                           0
#define SIMD_SSD                           0
#define SIMD_DIFF                          0
#define SIMD_HAD_SAD                       0
#define SIMD_AFFINE                        0
#if ASP
#define SIMD_ASP                           0
#endif // ASP
#if ENC_ME_IMP
#define SIMD_GRAD_ME                       0
#endif
#endif

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                         Certain Tools Parameters                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#define INC_QT_DEPTH(qtd, smode)           (smode == SPLIT_QUAD? (qtd + 1) : qtd)
#define INC_BET_DEPTH(betd, smode)         (smode != SPLIT_QUAD? (betd + 1) : betd)

#if EQT
#define MAX_SPLIT_NUM                      6
#define SPLIT_CHECK_NUM                    6
#else
#define MAX_SPLIT_NUM                      4
#define SPLIT_CHECK_NUM                    4
#endif

#if CCNPM
#define CCNPM_PARAM_NUM                    7
#define CCNPM_EXTENSION_SIZE               6
#define CCNPM_PADDING_SIZE                 1
#if CCNPM_LINE_BUFFER_REDUCTION
#define CCNPM_LINE_BUFFER_SIZE             2
#endif
#define CCNPM_MATRIX_SHIFT                 28
#define CCNPM_DECIM_SHIFT                  22
#define CCNPM_DECIM_MID                    ( 1 << (CCNPM_DECIM_SHIFT - 1 ) )
#define CCNPM_MULT(x, y)                   (long long)(((long long)x*y + CCNPM_DECIM_MID) >> CCNPM_DECIM_SHIFT )
#define CCNPM_DIV(x, y)                    (long long)(((long long)x << CCNPM_DECIM_SHIFT ) / y )
#endif

/*****************************************************************************
 * return values and error code
 *****************************************************************************/
/* no more frames, but it is OK */
#define COM_OK_NO_MORE_FRM              (205)
/* progress success, but output is not available temporarily */
#define COM_OK_OUT_NOT_AVAILABLE        (204)
/* frame dimension (width or height) has been changed */
#define COM_OK_DIM_CHANGED              (203)
/* decoding success, but output frame has been delayed */
#define COM_OK_FRM_DELAYED              (202)
#if LIB_PIC_UPDATE
#define RL_UPDATE_TO_LIBPIC             (2)
#define COM_OK_SKIP                     (3)
#endif
#define COM_OK                          (0)
#define END_OF_VIDEO_SEQUENCE           (206)
#define NOT_END_OF_VIDEO_SEQUENCE       (207)

#define COM_ERR                         (-1) /* generic error */
#define COM_ERR_INVALID_ARGUMENT        (-101)
#define COM_ERR_OUT_OF_MEMORY           (-102)
#define COM_ERR_REACHED_MAX             (-103)
#define COM_ERR_UNSUPPORTED             (-104)
#define COM_ERR_UNEXPECTED              (-105)
#define COM_ERR_BAD_CRC                 (-130) /* not matched CRC value */

#define COM_ERR_UNSUPPORTED_COLORSPACE  (-201)
#define COM_ERR_MALFORMED_BITSTREAM     (-202)

#define COM_ERR_UNKNOWN                 (-32767) /* unknown error */


/* return value checking *****************************************************/
#define COM_SUCCEEDED(ret)              ((ret) >= 0)
#define COM_FAILED(ret)                 ((ret) < 0)

/* YUV planar ****************************************************************/
#define COM_COLORSPACE_YUV400          300 /* Y 8bit */
#define COM_COLORSPACE_YUV420          301 /* YUV420 8bit */
#define COM_COLORSPACE_YUV422          302 /* YUV422 8bit narrow chroma*/
#define COM_COLORSPACE_YUV444          303 /* YUV444 8bit */
#define COM_COLORSPACE_YUV422N         COM_COLORSPACE_YUV422
#define COM_COLORSPACE_YUV422W         310 /* YUV422 8bit wide chroma */

#define COM_COLORSPACE_YUV400A8        400 /* Y+alpha 8bit */
#define COM_COLORSPACE_YUV420A8        401 /* YUV420+alpha 8bit */
#define COM_COLORSPACE_YUV422A8        402 /* YUV422+alpha 8bit narrow chroma*/
#define COM_COLORSPACE_YUV444A8        403 /* YUV444+alpha 8bit */
#define COM_COLORSPACE_YUV422NA8       COM_COLORSPACE_YUV422A8
#define COM_COLORSPACE_YUV422WA8       414 /* YUV422+alpha 8bit wide chroma*/

/* RGB pack ******************************************************************/

/* RGB pack 8bit */
#define COM_COLORSPACE_RGB888          2200
#define COM_COLORSPACE_BGR888          2201

#define COM_COLORSPACE_RGBA8888        2220
#define COM_COLORSPACE_BGRA8888        2221
#define COM_COLORSPACE_ARGB8888        2222
#define COM_COLORSPACE_ABGR8888        2223

/*****************************************************************************
 * config types for decoder
 *****************************************************************************/
#define DEC_CFG_SET_USE_PIC_SIGNATURE  (301)

/*****************************************************************************
 * chunk type
 *****************************************************************************/
#define COM_CT_UNKNOWN                  (0)
#define COM_CT_PICTURE                  (1) /* picture header */
#define COM_CT_SQH                      (2) /* sequence header */
#define COM_CT_SLICE                    (3) /* slice header */
#define COM_CT_SIGN                     (6) /* picture signature */
#define COM_CT_SEQ_END                  (7)

/*****************************************************************************
 * slice type
 *****************************************************************************/
#define COM_ST_UNKNOWN                  (0)
#define COM_ST_I                        (1)
#define COM_ST_P                        (2)
#define COM_ST_B                        (3)

/*****************************************************************************
 * software version
 *****************************************************************************/
#define COM_VER_1                       (1)

/*****************************************************************************
 * type and macro for media time
 *****************************************************************************/
/* media time in 100-nanosec unit */
typedef long long                    COM_MTIME;

/*****************************************************************************
 * image buffer format
 *****************************************************************************
 baddr
    +---------------------------------------------------+ ---
    |                                                   |  ^
    |                                              |    |  |
    |    a                                         v    |  |
    |   --- +-----------------------------------+ ---   |  |
    |    ^  |  (x, y)                           |  y    |  |
    |    |  |   +---------------------------+   + ---   |  |
    |    |  |   |                           |   |  ^    |  |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  |    |  |
    |       |   |                           |   |       |
    |    ah |   |                           |   |  h    |  e
    |       |   |                           |   |       |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  |    |  |
    |    |  |   |                           |   |  v    |  |
    |    |  |   +---------------------------+   | ---   |  |
    |    v  |                                   |       |  |
    |   --- +---+-------------------------------+       |  |
    |     ->| x |<----------- w ----------->|           |  |
    |       |<--------------- aw -------------->|       |  |
    |                                                   |  v
    +---------------------------------------------------+ ---

    |<---------------------- stride-------------------->|

 *****************************************************************************/

#define COM_IMGB_MAX_PLANE              (4)

typedef struct _COM_IMGB COM_IMGB;
struct _COM_IMGB
{
    int                 cs; /* color space */
    int                 np; /* number of plane */
    int                 horizontal_size;
    int                 vertical_size;
    /* width (in unit of pixel) */
    int                 width[COM_IMGB_MAX_PLANE];
    /* height (in unit of pixel) */
    int                 height[COM_IMGB_MAX_PLANE];
    /* buffer stride (in unit of byte) */
    int                 stride[COM_IMGB_MAX_PLANE];
    /* address of each plane */
    void              * addr_plane[COM_IMGB_MAX_PLANE];

    /* time-stamps */
    COM_MTIME          ts[4];

    /* aligned width (in unit of pixel) */
    int                 width_aligned[COM_IMGB_MAX_PLANE];
    /* aligned height (in unit of pixel) */
    int                 height_aligned[COM_IMGB_MAX_PLANE];

    /* left padding size (in unit of pixel) */
    int                 pad_left[COM_IMGB_MAX_PLANE];
    /* right padding size (in unit of pixel) */
    int                 pad_right[COM_IMGB_MAX_PLANE];
    /* up padding size (in unit of pixel) */
    int                 pad_up[COM_IMGB_MAX_PLANE];
    /* bottom padding size (in unit of pixel) */
    int                 pad_down[COM_IMGB_MAX_PLANE];

    /* address of actual allocated buffer */
    void              * buf_addr[COM_IMGB_MAX_PLANE];
    /* actual allocated buffer size */
    int                 buf_size[COM_IMGB_MAX_PLANE];

    /* life cycle management */
    int                 refcnt;
    int                 (*addref)(COM_IMGB * imgb);
    int                 (*getref)(COM_IMGB * imgb);
    int                 (*release)(COM_IMGB * imgb);
};


/*****************************************************************************
 * Bitstream buffer
 *****************************************************************************/
typedef struct _COM_BITB
{
    /* user space address indicating buffer */
    void              * addr;
    void              * addr2;
    /* physical address indicating buffer, if any */
    void              * pddr;
    /* byte size of buffer memory */
    int                 bsize;
    /* byte size of bitstream in buffer */
    int                 ssize;
    /* bitstream has an error? */
    int                 err;
    /* arbitrary data, if needs */
    int                 ndata[4];
    /* arbitrary address, if needs */
    void              * pdata[4];
    /* time-stamps */
    COM_MTIME          ts[4];

} COM_BITB;

/*****************************************************************************
 * description for creating of decoder
 *****************************************************************************/
typedef struct _DEC_CDSC
{
    int            __na; /* nothing */
} DEC_CDSC;

/*****************************************************************************
 * status after decoder operation
 *****************************************************************************/
typedef struct _DEC_STAT
{
    /* byte size of decoded bitstream (read size of bitstream) */
    int            read;
    /* chunk type */
    int            ctype;
    /* slice type */
    int            stype;
    /* frame number monotonically increased whenever decoding a frame.
    note that it has negative value if the decoded data is not frame */
    int            fnum;
    /* picture order count */
    int            poc;
    /* number of reference pictures */
    int            refpic_num[2];
    /* list of reference pictures */
    int            refpic[2][16];
#if LIBVC_ON
    /* is only ref libpic */
    int            is_RLpic_flag;
#if IPPPCRR
    int            reflib[2][16];
#endif
#endif
    /* for printing sqh parameter */
#if PRINT_SQH_PARAM_DEC
    /* framework param */
#if PHASE_2_PROFILE
    int            profile_id;
#endif
#if CFG_LEVEL_ID
    int            level_id;
#endif
    int            internal_bit_depth;
    /* tools */
    int            intra_tools;
    int            inter_tools;
    int            trans_tools;
    int            filte_tools;
    int            scc_tools;
#if PARITY_HIDING
    int            coef_coding_tools;
#endif
#endif
} DEC_STAT;

/*****************************************************************************
 * status after encoder operation
 *****************************************************************************/
typedef struct _ENC_STAT
{
    /* encoded bitstream byte size */
    int            write;
    /* picture number increased whenever encoding a frame */
    unsigned long  fnum;
    /* chunk type */
    int            ctype;
    /* slice type */
    int            stype;
    /* quantization parameter used for encoding */
    int            qp;
    /* picture order count */
    int            poc;
    /* number of reference pictures */
    int            refpic_num[2];
    /* list of reference pictures */
    int            refpic[2][16];
#if LIBVC_ON
    /* is only ref libpic */
    int            is_RLpic_flag;
#if IPPPCRR
    int            reflib[2][16];
#endif
#endif
} ENC_STAT;

#if RPL_GOP32
#define MAX_NUM_REF_PICS                   33
#define MAX_NUM_ACTIVE_REF_FRAME           4
#define MAX_NUM_RPLS                       64
#else
#define MAX_NUM_REF_PICS                   17
#define MAX_NUM_ACTIVE_REF_FRAME           4
#define MAX_NUM_RPLS                       32
#endif

extern int g_CountDOICyCleTime;             // number to count the DOI cycle time.
extern int g_DOIPrev;                       // the doi of previous frm.

#if HLS_RPL
/* rpl structure */
typedef struct _COM_RPL
{
    int slice_type;
    int poc;
    int tid;
    int ref_pic_num;
    int ref_pic_active_num;
    int ref_pics[MAX_NUM_REF_PICS];
    int ref_pics_ddoi[MAX_NUM_REF_PICS];
    int ref_pics_doi[MAX_NUM_REF_PICS];//ref pic list, doi info
#if LIBVC_ON
    int reference_to_library_enable_flag;
    int library_index_flag[MAX_NUM_REF_PICS];
#endif
} COM_RPL;
#endif
/*****************************************************************************
 * API for decoder only
 *****************************************************************************/
/* instance identifier for decoder */
typedef void  * DEC;

DEC dec_create(DEC_CDSC * cdsc, int * err);
void dec_delete(DEC id);
int dec_decode(DEC id, COM_BITB * bitb, DEC_STAT * stat);
int dec_config(DEC id, int cfg, void * buf, int * size);

#ifdef __cplusplus
}
#endif

#endif /* _COM_H_ */
