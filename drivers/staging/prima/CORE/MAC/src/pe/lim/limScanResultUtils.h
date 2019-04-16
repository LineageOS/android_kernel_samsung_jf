/*
 * Copyright (c) 2012, 2018 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Airgo Networks, Inc proprietary. All rights reserved.
 * This file limScanResultUtils.h contains the utility definitions
 * LIM uses for maintaining and accessing scan results on STA.
 * Author:        Chandra Modumudi
 * Date:          02/13/02
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 */
#ifndef __LIM_SCAN_UTILS_H
#define __LIM_SCAN_UTILS_H

#include "parserApi.h"
#include "limTypes.h"

// Scan result hash related functions
tANI_U8 limScanHashFunction(tSirMacAddr);
void    limInitHashTable(tpAniSirGlobal);
eHalStatus    
   limLookupNaddHashEntry(tpAniSirGlobal, tLimScanResultNode *, tANI_U8, tANI_U8, tANI_U32);
void    limDeleteHashEntry(tLimScanResultNode *);
void    limDeleteCachedScanResults(tpAniSirGlobal);
void    limRestorePreScanState(tpAniSirGlobal);
void    limCopyScanResult(tpAniSirGlobal, tANI_U8 *);
void    limReInitScanResults(tpAniSirGlobal);
tANI_U32 limDeactivateMinChannelTimerDuringScan(tpAniSirGlobal);
void    limCheckAndAddBssDescription(tpAniSirGlobal, tpSirProbeRespBeacon, tANI_U8 *, tANI_BOOLEAN, tANI_U8);
#if defined WLAN_FEATURE_VOWIFI
void    limCollectBssDescription(tpAniSirGlobal,
                                 tSirBssDescription *,
                                 tpSirProbeRespBeacon,
                                 tANI_U8 *,
                                 tANI_U8);
#else
void    limCollectBssDescription(tpAniSirGlobal,
                                 tSirBssDescription *,
                                 tpSirProbeRespBeacon,
                                 tANI_U8 *);
#endif

#endif /* __LIM_SCAN_UTILS_H */
