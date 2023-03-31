/* SPDX-License-Identifier: ISC */
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#ifndef __MT7996_EEPROM_H
#define __MT7996_EEPROM_H

#include "mt7996.h"

enum mt7996_eeprom_field {
	MT_EE_CHIP_ID =		0x000,
	MT_EE_VERSION =		0x002,
	MT_EE_MAC_ADDR =	0x004,
	MT_EE_MAC_ADDR2 =	0x00a,
	MT_EE_WIFI_CONF =	0x190,
	MT_EE_DO_PRE_CAL =	0x1a5,
	MT_EE_TESTMODE_EN =	0x1af,
	MT_EE_MAC_ADDR3 =	0x2c0,
	MT_EE_RATE_DELTA_2G =	0x1400,
	MT_EE_RATE_DELTA_5G =	0x147d,
	MT_EE_RATE_DELTA_6G =	0x154a,
	MT_EE_TX0_POWER_2G =	0x1300,
	MT_EE_TX0_POWER_5G =	0x1301,
	MT_EE_TX0_POWER_6G =	0x1310,

	__MT_EE_MAX =	0x1dff,
};

#define MT_EE_WIFI_CONF0_TX_PATH		GENMASK(2, 0)
#define MT_EE_WIFI_CONF0_BAND_SEL		GENMASK(2, 0)
#define MT_EE_WIFI_CONF1_BAND_SEL		GENMASK(5, 3)
#define MT_EE_WIFI_CONF2_BAND_SEL		GENMASK(2, 0)
#define MT_EE_WIFI_PA_LNA_CONFIG		GENMASK(1, 0)

#define MT_EE_WIFI_CAL_GROUP_2G			BIT(0)
#define MT_EE_WIFI_CAL_GROUP_5G			BIT(1)
#define MT_EE_WIFI_CAL_GROUP_6G			BIT(2)
#define MT_EE_WIFI_CAL_GROUP			GENMASK(2, 0)
#define MT_EE_WIFI_CAL_DPD_2G			BIT(3)
#define MT_EE_WIFI_CAL_DPD_5G			BIT(4)
#define MT_EE_WIFI_CAL_DPD_6G			BIT(5)
#define MT_EE_WIFI_CAL_DPD			GENMASK(5, 3)

#define MT_EE_CAL_UNIT				1024
#define MT_EE_CAL_GROUP_SIZE_2G			(4 * MT_EE_CAL_UNIT)
#define MT_EE_CAL_GROUP_SIZE_5G			(45 * MT_EE_CAL_UNIT)
#define MT_EE_CAL_GROUP_SIZE_6G			(125 * MT_EE_CAL_UNIT)
#define MT_EE_CAL_ADCDCOC_SIZE_2G		(4 * 4)
#define MT_EE_CAL_ADCDCOC_SIZE_5G		(4 * 4)
#define MT_EE_CAL_ADCDCOC_SIZE_6G		(4 * 5)
#define MT_EE_CAL_GROUP_SIZE			(MT_EE_CAL_GROUP_SIZE_2G + \
						 MT_EE_CAL_GROUP_SIZE_5G + \
						 MT_EE_CAL_GROUP_SIZE_6G + \
						 MT_EE_CAL_ADCDCOC_SIZE_2G + \
						 MT_EE_CAL_ADCDCOC_SIZE_5G + \
						 MT_EE_CAL_ADCDCOC_SIZE_6G)

#define DPD_PER_CH_LEGACY_SIZE			(4 * MT_EE_CAL_UNIT)
#define DPD_PER_CH_MEM_SIZE			(13 * MT_EE_CAL_UNIT)
#define DPD_PER_CH_OTFG0_SIZE			(2 * MT_EE_CAL_UNIT)
#define DPD_PER_CH_BW20_SIZE			(DPD_PER_CH_LEGACY_SIZE + DPD_PER_CH_OTFG0_SIZE)
#define DPD_PER_CH_GT_BW20_SIZE			(DPD_PER_CH_MEM_SIZE + DPD_PER_CH_OTFG0_SIZE)
#define MT_EE_CAL_DPD_SIZE			(780 * MT_EE_CAL_UNIT)

extern const struct ieee80211_channel dpd_2g_ch_list_bw20[];
extern const u32 dpd_2g_bw20_ch_num;
extern const struct ieee80211_channel dpd_5g_ch_list_bw160[];
extern const u32 dpd_5g_bw160_ch_num;
extern const struct ieee80211_channel dpd_6g_ch_list_bw160[];
extern const u32 dpd_6g_bw160_ch_num;
extern const struct ieee80211_channel dpd_6g_ch_list_bw320[];
extern const u32 dpd_6g_bw320_ch_num;

#define RF_DPD_FLAT_CAL				BIT(28)
#define RF_PRE_CAL				BIT(29)
#define RF_DPD_FLAT_5G_CAL			GENMASK(29, 28)
#define RF_DPD_FLAT_5G_MEM_CAL			(BIT(30) | BIT(28))
#define RF_DPD_FLAT_6G_CAL			GENMASK(30, 28)
#define RF_DPD_FLAT_6G_MEM_CAL			(BIT(31) | BIT(28))

#define MT_EE_WIFI_CONF1_TX_PATH_BAND0		GENMASK(5, 3)
#define MT_EE_WIFI_CONF2_TX_PATH_BAND1		GENMASK(2, 0)
#define MT_EE_WIFI_CONF2_TX_PATH_BAND2		GENMASK(5, 3)
#define MT_EE_WIFI_CONF3_RX_PATH_BAND0		GENMASK(2, 0)
#define MT_EE_WIFI_CONF3_RX_PATH_BAND1		GENMASK(5, 3)
#define MT_EE_WIFI_CONF4_RX_PATH_BAND2		GENMASK(2, 0)
#define MT_EE_WIFI_CONF4_STREAM_NUM_BAND0	GENMASK(5, 3)
#define MT_EE_WIFI_CONF5_STREAM_NUM_BAND1	GENMASK(2, 0)
#define MT_EE_WIFI_CONF5_STREAM_NUM_BAND2	GENMASK(5, 3)

#define MT_EE_RATE_DELTA_MASK			GENMASK(5, 0)
#define MT_EE_RATE_DELTA_SIGN			BIT(6)
#define MT_EE_RATE_DELTA_EN			BIT(7)

enum mt7996_eeprom_band {
	MT_EE_BAND_SEL_DEFAULT,
	MT_EE_BAND_SEL_2GHZ,
	MT_EE_BAND_SEL_5GHZ,
	MT_EE_BAND_SEL_6GHZ,
};

static inline int
mt7996_get_channel_group_5g(int channel)
{
	if (channel <= 64)
		return 0;
	if (channel <= 96)
		return 1;
	if (channel <= 128)
		return 2;
	if (channel <= 144)
		return 3;
	return 4;
}

static inline int
mt7996_get_channel_group_6g(int channel)
{
	if (channel <= 29)
		return 0;

	return DIV_ROUND_UP(channel - 29, 32);
}

#endif
