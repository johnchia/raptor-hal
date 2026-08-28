/*
 * Link-time stand-ins for the Ingenic vendor entry points.
 *
 * t_isp_imp.c builds the real src/hal_isp.c for T31 on the host, which
 * references 111 IMP_* symbols that live in libimp on the camera and nowhere
 * here. This file defines them so the suite links.
 *
 * Every one of them aborts. That is the point: nothing in t_isp_imp.c should
 * ever reach the SDK -- the tests drive the argument checks and the capability
 * table, both of which answer before any vendor call -- so a stub being entered
 * means a test is exercising something it cannot actually verify, and a silent
 * `return 0` would let it pass vacuously. When a test does need a real fake,
 * replace that one stub with it.
 *
 * Declared (void) deliberately: this translation unit does not include the
 * vendor headers, so the prototypes cannot be honoured and do not need to be.
 * C has no name mangling, the linker only matches the name, and the body never
 * returns to its caller.
 *
 * Regenerate after adding a vendor call:
 *
 *   gcc -c -DPLATFORM_T31 -DHAL_MODULE_VIDEO -Iinclude -Isrc \
 *       -I../raptor-common/include -Iingenic-headers/T31/1.1.6/en \
 *       -o /tmp/hi.o src/hal_isp.c
 *   nm -u /tmp/hi.o | awk '{print $2}' | grep ^IMP_ | sort
 *
 * A stale list fails the link by name, which is the loud failure it should be.
 *
 * One symbol is deliberately absent: IMP_ISP_Tuning_SetBrightness, which
 * t_isp_imp.c defines itself as a recording fake, because proving that a legal
 * value still reaches the SDK needs a call that arrives rather than aborts.
 * Anything else the suite grows a need for comes out of this list the same way.
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

STUB(IMP_ISP_GetDefaultBinPath)
STUB(IMP_ISP_GetSensorRegister)
STUB(IMP_ISP_SetDefaultBinPath)
STUB(IMP_ISP_SetSensorRegister)
STUB(IMP_ISP_Tuning_AE_GetROI)
STUB(IMP_ISP_Tuning_AE_SetROI)
STUB(IMP_ISP_Tuning_Awb_GetRgbCoefft)
STUB(IMP_ISP_Tuning_DisableMovestate)
STUB(IMP_ISP_Tuning_EnableDefog)
STUB(IMP_ISP_Tuning_EnableDRC)
STUB(IMP_ISP_Tuning_EnableMovestate)
STUB(IMP_ISP_Tuning_GetAeAttr)
STUB(IMP_ISP_Tuning_GetAeComp)
STUB(IMP_ISP_Tuning_GetAeHist)
STUB(IMP_ISP_Tuning_GetAeHist_Origin)
STUB(IMP_ISP_Tuning_GetAE_IT_MAX)
STUB(IMP_ISP_Tuning_GetAeLuma)
STUB(IMP_ISP_Tuning_GetAeMin)
STUB(IMP_ISP_Tuning_GetAeState)
STUB(IMP_ISP_Tuning_GetAeTargetList)
STUB(IMP_ISP_Tuning_GetAeWeight)
STUB(IMP_ISP_Tuning_GetAeZone)
STUB(IMP_ISP_Tuning_GetAfHist)
STUB(IMP_ISP_Tuning_GetAFMetrices)
STUB(IMP_ISP_Tuning_GetAfWeight)
STUB(IMP_ISP_Tuning_GetAfZone)
STUB(IMP_ISP_Tuning_GetAntiFlickerAttr)
STUB(IMP_ISP_Tuning_GetAwbClust)
STUB(IMP_ISP_Tuning_GetAWBCt)
STUB(IMP_ISP_Tuning_GetAwbCtTrend)
STUB(IMP_ISP_Tuning_GetAwbHist)
STUB(IMP_ISP_Tuning_GetAwbWeight)
STUB(IMP_ISP_Tuning_GetAwbZone)
STUB(IMP_ISP_Tuning_GetBacklightComp)
STUB(IMP_ISP_Tuning_GetBcshHue)
STUB(IMP_ISP_Tuning_GetBlcAttr)
STUB(IMP_ISP_Tuning_GetContrast)
STUB(IMP_ISP_Tuning_GetCsc_Attr)
STUB(IMP_ISP_Tuning_GetDefog_Strength)
STUB(IMP_ISP_Tuning_GetDPC_Strength)
STUB(IMP_ISP_Tuning_GetDRC_Strength)
STUB(IMP_ISP_Tuning_GetEVAttr)
STUB(IMP_ISP_Tuning_GetExpr)
STUB(IMP_ISP_Tuning_GetFrontCrop)
STUB(IMP_ISP_Tuning_GetGamma)
STUB(IMP_ISP_Tuning_GetHiLightDepress)
STUB(IMP_ISP_Tuning_GetHVFlip)
STUB(IMP_ISP_Tuning_GetISPCustomMode)
STUB(IMP_ISP_Tuning_GetISPRunningMode)
STUB(IMP_ISP_Tuning_GetMask)
STUB(IMP_ISP_Tuning_GetMaxAgain)
STUB(IMP_ISP_Tuning_GetMaxDgain)
STUB(IMP_ISP_Tuning_GetModuleControl)
STUB(IMP_ISP_Tuning_GetSaturation)
STUB(IMP_ISP_Tuning_GetSensorFPS)
STUB(IMP_ISP_Tuning_GetSharpness)
STUB(IMP_ISP_Tuning_GetTotalGain)
STUB(IMP_ISP_Tuning_GetWB)
STUB(IMP_ISP_Tuning_GetWB_GOL_Statis)
STUB(IMP_ISP_Tuning_GetWB_Statis)
STUB(IMP_ISP_Tuning_GetWdr_OutputMode)
STUB(IMP_ISP_Tuning_SetAeAttr)
STUB(IMP_ISP_Tuning_SetAeFreeze)
STUB(IMP_ISP_Tuning_SetAeHist)
STUB(IMP_ISP_Tuning_SetAe_IT_MAX)
STUB(IMP_ISP_Tuning_SetAeMin)
STUB(IMP_ISP_Tuning_SetAeTargetList)
STUB(IMP_ISP_Tuning_SetAeWeight)
STUB(IMP_ISP_Tuning_SetAfHist)
STUB(IMP_ISP_Tuning_SetAfWeight)
STUB(IMP_ISP_Tuning_SetAntiFlickerAttr)
STUB(IMP_ISP_Tuning_SetAutoZoom)
STUB(IMP_ISP_Tuning_SetAwbClust)
STUB(IMP_ISP_Tuning_SetAwbCt)
STUB(IMP_ISP_Tuning_SetAwbCtTrend)
STUB(IMP_ISP_Tuning_SetAwbHist)
STUB(IMP_ISP_Tuning_SetAwbWeight)
STUB(IMP_ISP_Tuning_SetBacklightComp)
STUB(IMP_ISP_Tuning_SetBcshHue)
STUB(IMP_ISP_Tuning_SetContrast)
STUB(IMP_ISP_Tuning_SetCsc_Attr)
STUB(IMP_ISP_Tuning_SetDefog_Strength)
STUB(IMP_ISP_Tuning_SetDPC_Strength)
STUB(IMP_ISP_Tuning_SetDRC_Strength)
STUB(IMP_ISP_Tuning_SetExpr)
STUB(IMP_ISP_Tuning_SetFrontCrop)
STUB(IMP_ISP_Tuning_SetGamma)
STUB(IMP_ISP_Tuning_SetHiLightDepress)
STUB(IMP_ISP_Tuning_SetHVFLIP)
STUB(IMP_ISP_Tuning_SetISPBypass)
STUB(IMP_ISP_Tuning_SetISPCustomMode)
STUB(IMP_ISP_Tuning_SetISPRunningMode)
STUB(IMP_ISP_Tuning_SetMask)
STUB(IMP_ISP_Tuning_SetMaxAgain)
STUB(IMP_ISP_Tuning_SetMaxDgain)
STUB(IMP_ISP_Tuning_SetModuleControl)
STUB(IMP_ISP_Tuning_SetSaturation)
STUB(IMP_ISP_Tuning_SetScalerLv)
STUB(IMP_ISP_Tuning_SetSensorFPS)
STUB(IMP_ISP_Tuning_SetSharpness)
STUB(IMP_ISP_Tuning_SetSinterStrength)
STUB(IMP_ISP_Tuning_SetTemperStrength)
STUB(IMP_ISP_Tuning_SetVideoDrop)
STUB(IMP_ISP_Tuning_SetWB)
STUB(IMP_ISP_Tuning_SetWdr_OutputMode)
STUB(IMP_ISP_Tuning_WaitFrame)
STUB(IMP_ISP_WDR_ENABLE)
STUB(IMP_ISP_WDR_ENABLE_Get)
