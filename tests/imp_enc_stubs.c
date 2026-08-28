/*
 * imp_enc_stubs.c -- every IMP encoder entry point hal_encoder.c can reach,
 * defined to abort.
 *
 * The companion to imp_stubs.c and the same argument: a suite whose claims are
 * about what gets built and what gets sent wants any unplanned trip into the
 * vendor SDK to be loud, not to return a plausible 0. Compiled without the IMP
 * headers, so `int sym(void)` does not have to agree with the real prototype --
 * C has no mangling and the link is by name.
 *
 * Absent from this list, and faked for real in t_enc_imp.c: GetChnAttr,
 * SetDefaultParam, SetChnAttrRcMode, RequestIDR. GetChnAttrRcMode is
 * deliberately left here -- a rate-control switch that reads the channel's
 * current arm is the bug this suite exists to catch, so reaching it aborts.
 *
 * Copyright (c) 2024 OpenIPC
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>

static int reached(const char *name)
{
    fprintf(stderr, "FAIL: a test reached the vendor SDK (%s)\n", name);
    abort();
}

#define STUB(sym)                                                                                  \
    int sym(void);                                                                                 \
    int sym(void)                                                                                  \
    {                                                                                              \
        return reached(#sym);                                                                      \
    }

STUB(IMP_Encoder_CreateChn)
STUB(IMP_Encoder_CreateGroup)
STUB(IMP_Encoder_DestroyChn)
STUB(IMP_Encoder_DestroyGroup)
STUB(IMP_Encoder_FlushStream)
STUB(IMP_Encoder_GetChnAttrRcMode)
STUB(IMP_Encoder_GetChnAveBitrate)
STUB(IMP_Encoder_GetChnColor2Grey)
STUB(IMP_Encoder_GetChnCrop)
STUB(IMP_Encoder_GetChnDenoise)
STUB(IMP_Encoder_GetChnEncType)
STUB(IMP_Encoder_GetChnEvalInfo)
STUB(IMP_Encoder_GetChnFrmRate)
STUB(IMP_Encoder_GetChnGopAttr)
STUB(IMP_Encoder_GetChnROI)
STUB(IMP_Encoder_GetFd)
STUB(IMP_Encoder_GetGDRCfg)
STUB(IMP_Encoder_GetH264TransCfg)
STUB(IMP_Encoder_GetH264Vui)
STUB(IMP_Encoder_GetH265TransCfg)
STUB(IMP_Encoder_GetH265Vui)
STUB(IMP_Encoder_GetJpegeQl)
STUB(IMP_Encoder_GetJpegQp)
STUB(IMP_Encoder_GetMaxStreamCnt)
STUB(IMP_Encoder_GetMbRC)
STUB(IMP_Encoder_GetPool)
STUB(IMP_Encoder_GetPskipCfg)
STUB(IMP_Encoder_GetQpgMode)
STUB(IMP_Encoder_GetSrdCfg)
STUB(IMP_Encoder_GetStream)
STUB(IMP_Encoder_GetStreamBufSize)
STUB(IMP_Encoder_GetSuperFrameCfg)
STUB(IMP_Encoder_InsertUserData)
STUB(IMP_Encoder_PollingModuleStream)
STUB(IMP_Encoder_PollingStream)
STUB(IMP_Encoder_Query)
STUB(IMP_Encoder_RegisterChn)
STUB(IMP_Encoder_ReleaseStream)
STUB(IMP_Encoder_RequestGDR)
STUB(IMP_Encoder_RequestPskip)
STUB(IMP_Encoder_SetbufshareChn)
STUB(IMP_Encoder_SetChnBitRate)
STUB(IMP_Encoder_SetChnColor2Grey)
STUB(IMP_Encoder_SetChnCrop)
STUB(IMP_Encoder_SetChnDenoise)
STUB(IMP_Encoder_SetChnEntropyMode)
STUB(IMP_Encoder_SetChnFrmRate)
STUB(IMP_Encoder_SetChnGopAttr)
STUB(IMP_Encoder_SetChnGopLength)
STUB(IMP_Encoder_SetChnMapRoi)
STUB(IMP_Encoder_SetChnMaxPictureSize)
STUB(IMP_Encoder_SetChnQp)
STUB(IMP_Encoder_SetChnQpBounds)
STUB(IMP_Encoder_SetChnQpBoundsPerFrame)
STUB(IMP_Encoder_SetChnQpgAI)
STUB(IMP_Encoder_SetChnQpIPDelta)
STUB(IMP_Encoder_SetChnResizeMode)
STUB(IMP_Encoder_SetChnROI)
STUB(IMP_Encoder_SetGDRCfg)
STUB(IMP_Encoder_SetGOPSize)
STUB(IMP_Encoder_SetH264TransCfg)
STUB(IMP_Encoder_SetH264Vui)
STUB(IMP_Encoder_SetH265TransCfg)
STUB(IMP_Encoder_SetH265Vui)
STUB(IMP_Encoder_SetJpegeQl)
STUB(IMP_Encoder_SetJpegQp)
STUB(IMP_Encoder_SetMaxStreamCnt)
STUB(IMP_Encoder_SetMbRC)
STUB(IMP_Encoder_SetPool)
STUB(IMP_Encoder_SetPskipCfg)
STUB(IMP_Encoder_SetQpgMode)
STUB(IMP_Encoder_SetSrdCfg)
STUB(IMP_Encoder_SetStreamBufSize)
STUB(IMP_Encoder_SetSuperFrameCfg)
STUB(IMP_Encoder_StartRecvPic)
STUB(IMP_Encoder_StopRecvPic)
STUB(IMP_Encoder_UnRegisterChn)
