/*
 * Cypress Touchkey firmware list
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#if defined(CONFIG_BOARD_JF_REFRESH)
#define BIN_FW_VERSION		0x1C
#define BASE_FW_VERSION	0x01
#define CYPRESS_MENU_BACK_MULTI_REPORT
#define CYPRESS_RECENT_BACK_REPORT_FW_VER BIN_FW_VERSION
#ifdef _CYPRESS_TKEY_FW_H
#include "fusion3_jf_refresh_tkey_fw.h"
#endif
#elif defined(CONFIG_MACH_JF_VZW) || defined(CONFIG_MACH_JF_SPR) || defined(CONFIG_MACH_JF_USC) || defined(CONFIG_MACH_JF_CRI)
#define BIN_FW_VERSION		0x1C
#define BASE_FW_VERSION		0x01
#define CYPRESS_MENU_BACK_MULTI_REPORT
#define CYPRESS_RECENT_BACK_REPORT_FW_VER BIN_FW_VERSION
#ifdef _CYPRESS_TKEY_FW_H
#include "fusion3_cdma_tkey_fw.h"
#endif
#elif defined(CONFIG_SEC_PRODUCT_8960)
#define BIN_FW_VERSION		0x06
#ifdef _CYPRESS_TKEY_FW_H
#include "d2_tkey_fw.h"
#endif
#else
#define BIN_FW_VERSION		0x12
#define BASE_FW_VERSION		0x01
#define CYPRESS_MENU_BACK_MULTI_REPORT
#define CYPRESS_RECENT_BACK_REPORT_FW_VER BIN_FW_VERSION
#ifdef _CYPRESS_TKEY_FW_H
#include "fusion3_tkey_fw.h"
#endif
#endif
#define BIN_FW_VERSION_20055	0x0D
#ifdef _CYPRESS_TKEY_FW_H
#include "fusion3_tkey_fw_20055.h"
#endif
