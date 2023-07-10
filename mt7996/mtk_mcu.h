/* SPDX-License-Identifier: ISC */
/*
 * Copyright (C) 2023 MediaTek Inc.
 */

#ifndef __MT7996_MTK_MCU_H
#define __MT7996_MTK_MCU_H

#include "../mt76_connac_mcu.h"

#ifdef CONFIG_MTK_DEBUG

enum {
	UNI_CMD_MURU_DBG_INFO = 0x18,
};

struct txpower_basic_info {
	u8 category;
	u8 rsv1;

	/* basic info */
	u8 band_idx;
	u8 band;

	/* board type info */
	bool is_epa;
	bool is_elna;

	/* power percentage info */
	bool percentage_ctrl_enable;
	s8 power_drop_level;

	/* frond-end loss TX info */
	s8 front_end_loss_tx[4];

	/* frond-end loss RX info */
	s8 front_end_loss_rx[4];

	/* thermal info */
	bool thermal_compensate_enable;
	s8 thermal_compensate_value;
	u8 rsv2;

	/* TX power max/min limit info */
	s8 max_power_bound;
	s8 min_power_bound;

	/* power limit info */
	bool sku_enable;
	bool bf_backoff_enable;

	/* MU TX power info */
	bool mu_tx_power_manual_enable;
	s8 mu_tx_power_auto;
	s8 mu_tx_power_manual;
	u8 rsv3;
};

struct txpower_phy_rate_info {
	u8 category;
	u8 band_idx;
	u8 band;
	u8 epa_gain;

	/* rate power info [dBm] */
	s8 frame_power[MT7996_SKU_RATE_NUM][__MT_MAX_BAND];

	/* TX power max/min limit info */
	s8 max_power_bound;
	s8 min_power_bound;
	u8 rsv1;
};

struct txpower_backoff_table_info {
	u8 category;
	u8 band_idx;
	u8 band;
	u8 backoff_en;

	s8 frame_power[MT7996_SKU_PATH_NUM];
	u8 rsv[3];
};

struct mt7996_mcu_txpower_event {
	u8 _rsv[4];

	__le16 tag;
	__le16 len;

	union {
		struct txpower_basic_info basic_info;
		struct txpower_phy_rate_info phy_rate_info;
		struct txpower_backoff_table_info backoff_table_info;
	};
};

enum txpower_category {
	BASIC_INFO,
	BACKOFF_TABLE_INFO,
	PHY_RATE_INFO,
};

enum txpower_event {
	UNI_TXPOWER_BASIC_INFO = 0,
	UNI_TXPOWER_BACKOFF_TABLE_SHOW_INFO = 3,
	UNI_TXPOWER_PHY_RATE_INFO = 5,
};

enum {
	EDCCA_CTRL_SET_EN = 0,
	EDCCA_CTRL_SET_THRES,
	EDCCA_CTRL_GET_EN,
	EDCCA_CTRL_GET_THRES,
	EDCCA_CTRL_NUM,
};

enum {
	EDCCA_DEFAULT = 0,
	EDCCA_FCC = 1,
	EDCCA_ETSI = 2,
	EDCCA_JAPAN = 3
};

enum {
	UNI_EVENT_SR_CFG_SR_ENABLE = 0x1,
	UNI_EVENT_SR_SW_SD = 0x83,
	UNI_EVENT_SR_HW_IND = 0xC9,
	UNI_EVENT_SR_HW_ESR_ENABLE = 0xD8,
};
enum {
	UNI_CMD_SR_CFG_SR_ENABLE = 0x1,
	UNI_CMD_SR_SW_SD = 0x84,
	UNI_CMD_SR_HW_IND = 0xCB,
	UNI_CMD_SR_HW_ENHANCE_SR_ENABLE = 0xDA,
};

struct mt7996_mcu_sr_basic_event {
	struct mt7996_mcu_rxd rxd;

	u8 band_idx;
	u8 _rsv[3];

	__le16 tag;
	__le16 len;
};

struct sr_sd_tlv {
	u8 _rsv[16];
	__le32 sr_tx_airtime;
	__le32 obss_airtime;
	__le32 my_tx_airtime;
	__le32 my_rx_airtime;
	__le32 channel_busy_time;
	__le32 total_airtime;
	__le32 total_airtime_ratio;
	__le32 obss_airtime_ratio;
	u8 rule;
	u8 _rsv2[59];
} __packed;

struct mt7996_mcu_sr_swsd_event {
	struct mt7996_mcu_sr_basic_event basic;
	struct sr_sd_tlv tlv[3];
} __packed;

struct mt7996_mcu_sr_common_event {
	struct mt7996_mcu_sr_basic_event basic;
	__le32 value;
};

struct mt7996_mcu_sr_hw_ind_event {
	struct mt7996_mcu_sr_basic_event basic;
	__le16 non_srg_valid_cnt;
	u8 _rsv[4];
	__le16 inter_bss_ppdu_cnt;
	u8 _rsv2[4];
	__le32 sr_ampdu_mpdu_cnt;
	__le32 sr_ampdu_mpdu_acked_cnt;
};
#endif

#endif
