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

#endif

#endif
