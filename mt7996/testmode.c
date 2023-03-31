// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include "mt7996.h"
#include "mac.h"
#include "mcu.h"
#include "testmode.h"
#include "eeprom.h"
#include "mtk_mcu.h"

enum {
	TM_CHANGED_TXPOWER,
	TM_CHANGED_FREQ_OFFSET,
	TM_CHANGED_SKU_EN,
	TM_CHANGED_TX_LENGTH,
	TM_CHANGED_TX_TIME,
	TM_CHANGED_CFG,

	/* must be last */
	NUM_TM_CHANGED
};

static const u8 tm_change_map[] = {
	[TM_CHANGED_TXPOWER] = MT76_TM_ATTR_TX_POWER,
	[TM_CHANGED_FREQ_OFFSET] = MT76_TM_ATTR_FREQ_OFFSET,
	[TM_CHANGED_SKU_EN] = MT76_TM_ATTR_SKU_EN,
	[TM_CHANGED_TX_LENGTH] = MT76_TM_ATTR_TX_LENGTH,
	[TM_CHANGED_TX_TIME] = MT76_TM_ATTR_TX_TIME,
	[TM_CHANGED_CFG] = MT76_TM_ATTR_CFG,
};

static u8 mt7996_tm_bw_mapping(enum nl80211_chan_width width, enum bw_mapping_method method)
{
	static const u8 width_to_bw[][NUM_BW_MAP] = {
		[NL80211_CHAN_WIDTH_40] = {FW_CDBW_40MHZ, TM_CBW_40MHZ},
		[NL80211_CHAN_WIDTH_80] = {FW_CDBW_80MHZ, TM_CBW_80MHZ},
		[NL80211_CHAN_WIDTH_80P80] = {FW_CDBW_8080MHZ, TM_CBW_8080MHZ},
		[NL80211_CHAN_WIDTH_160] = {FW_CDBW_160MHZ, TM_CBW_160MHZ},
		[NL80211_CHAN_WIDTH_5] = {FW_CDBW_5MHZ, TM_CBW_5MHZ},
		[NL80211_CHAN_WIDTH_10] = {FW_CDBW_10MHZ, TM_CBW_10MHZ},
		[NL80211_CHAN_WIDTH_20] = {FW_CDBW_20MHZ, TM_CBW_20MHZ},
		[NL80211_CHAN_WIDTH_20_NOHT] = {FW_CDBW_20MHZ, TM_CBW_20MHZ},
		[NL80211_CHAN_WIDTH_320] = {FW_CDBW_320MHZ, TM_CBW_320MHZ},
	};

	if (width >= ARRAY_SIZE(width_to_bw))
		return 0;

	return width_to_bw[width][method];
}

static u8 mt7996_tm_rate_to_phy(u8 tx_rate_mode)
{
	static const u8 rate_to_phy[] = {
		[MT76_TM_TX_MODE_CCK] = MT_PHY_TYPE_CCK,
		[MT76_TM_TX_MODE_OFDM] = MT_PHY_TYPE_OFDM,
		[MT76_TM_TX_MODE_HT] = MT_PHY_TYPE_HT,
		[MT76_TM_TX_MODE_VHT] = MT_PHY_TYPE_VHT,
		[MT76_TM_TX_MODE_HE_SU] = MT_PHY_TYPE_HE_SU,
		[MT76_TM_TX_MODE_HE_EXT_SU] = MT_PHY_TYPE_HE_EXT_SU,
		[MT76_TM_TX_MODE_HE_TB] = MT_PHY_TYPE_HE_TB,
		[MT76_TM_TX_MODE_HE_MU] = MT_PHY_TYPE_HE_MU,
		[MT76_TM_TX_MODE_EHT_SU] = MT_PHY_TYPE_EHT_SU,
		[MT76_TM_TX_MODE_EHT_TRIG] = MT_PHY_TYPE_EHT_TRIG,
		[MT76_TM_TX_MODE_EHT_MU] = MT_PHY_TYPE_EHT_MU,
	};

	if (tx_rate_mode > MT76_TM_TX_MODE_MAX)
		return -EINVAL;

	return rate_to_phy[tx_rate_mode];
}

static int
mt7996_tm_check_antenna(struct mt7996_phy *phy)
{
	struct mt76_testmode_data *td = &phy->mt76->test;
	struct mt7996_dev *dev = phy->dev;
	u8 band_idx = phy->mt76->band_idx;
	u32 chainmask = phy->mt76->chainmask;
	u32 aux_rx_mask;

	chainmask = chainmask >> dev->chainshift[band_idx];
	aux_rx_mask = BIT(fls(chainmask)) * phy->has_aux_rx;
	if (td->tx_antenna_mask & ~(chainmask | aux_rx_mask)) {
		dev_err(dev->mt76.dev,
			"tx antenna mask 0x%x exceeds hw limit (chainmask 0x%x, has aux rx: %s)\n",
			td->tx_antenna_mask, chainmask, phy->has_aux_rx ? "yes" : "no");
		return -EINVAL;
	}

	return 0;
}

static int
mt7996_tm_set(struct mt7996_dev *dev, u32 func_idx, u32 data)
{
	struct mt7996_tm_req req = {
		.rf_test = {
			.tag = cpu_to_le16(UNI_RF_TEST_CTRL),
			.len = cpu_to_le16(sizeof(req.rf_test)),
			.action = RF_ACTION_SET,
			.op.rf.func_idx = func_idx,
			.op.rf.param.func_data = cpu_to_le32(data),
		},
	};
	bool wait = (data == RF_CMD(START_TX)) ? true : false;

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TESTMODE_CTRL), &req,
				 sizeof(req), wait);
}

static int
mt7996_tm_get(struct mt7996_dev *dev, u32 func_idx, u32 data, u32 *result)
{
	struct mt7996_tm_req req = {
		.rf_test = {
			.tag = cpu_to_le16(UNI_RF_TEST_CTRL),
			.len = cpu_to_le16(sizeof(req.rf_test)),
			.action = RF_ACTION_GET,
			.op.rf.func_idx = func_idx,
			.op.rf.param.func_data = cpu_to_le32(data),
		},
	};
	struct mt7996_tm_event *event;
	struct sk_buff *skb;
	int ret;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_WM_UNI_CMD_QUERY(TESTMODE_CTRL),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;

	event = (struct mt7996_tm_event *)skb->data;
	*result = event->result.payload_length;

	dev_kfree_skb(skb);

	return ret;
}

static void
mt7996_tm_set_antenna(struct mt7996_phy *phy, u32 func_idx)
{
#define SPE_INDEX_MASK		BIT(31)
#define TX_ANTENNA_MASK		GENMASK(3, 0)
#define RX_ANTENNA_MASK		GENMASK(20, 16)		/* RX antenna mask at most 5 bit */
	struct mt7996_dev *dev = phy->dev;
	struct mt76_testmode_data *td = &phy->mt76->test;
	u32 antenna_mask;

	if (!mt76_testmode_param_present(td, MT76_TM_ATTR_TX_ANTENNA))
		return;

	if (func_idx == SET_ID(TX_PATH))
		antenna_mask = td->tx_spe_idx ? (SPE_INDEX_MASK | td->tx_spe_idx) :
						td->tx_antenna_mask & TX_ANTENNA_MASK;
	else if (func_idx == SET_ID(RX_PATH))
		antenna_mask = u32_encode_bits(td->tx_antenna_mask, RX_ANTENNA_MASK);
	else
		return;

	mt7996_tm_set(dev, func_idx, antenna_mask);
}

static void
mt7996_tm_set_mac_addr(struct mt7996_dev *dev, u8 *addr, u32 func_idx)
{
#define REMAIN_PART_TAG		BIT(18)
	u32 own_mac_first = 0, own_mac_remain = 0;
	int len = sizeof(u32);

	memcpy(&own_mac_first, addr, len);
	mt7996_tm_set(dev, func_idx, own_mac_first);
	/* Set the remain part of mac address */
	memcpy(&own_mac_remain, addr + len, ETH_ALEN - len);
	mt7996_tm_set(dev, func_idx | REMAIN_PART_TAG, own_mac_remain);
}

static int
mt7996_tm_rf_switch_mode(struct mt7996_dev *dev, u32 op_mode)
{
	struct mt7996_tm_req req = {
		.rf_test = {
			.tag = cpu_to_le16(UNI_RF_TEST_CTRL),
			.len = cpu_to_le16(sizeof(req.rf_test)),
			.action = RF_ACTION_SWITCH_TO_RF_TEST,
			.op.op_mode = cpu_to_le32(op_mode),
		},
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TESTMODE_CTRL), &req,
				 sizeof(req), false);
}

static void
mt7996_tm_init(struct mt7996_phy *phy, bool en)
{
	struct mt7996_dev *dev = phy->dev;
	u8 rf_test_mode = en ? RF_OPER_RF_TEST : RF_OPER_NORMAL;

	if (!test_bit(MT76_STATE_RUNNING, &phy->mt76->state))
		return;

	mt7996_mcu_set_tx_power_ctrl(phy, POWER_CTRL(ATE_MODE), en);
	mt7996_mcu_set_tx_power_ctrl(phy, POWER_CTRL(SKU_POWER_LIMIT), !en);
	mt7996_mcu_set_tx_power_ctrl(phy, POWER_CTRL(BACKOFF_POWER_LIMIT), !en);

	mt7996_tm_rf_switch_mode(dev, rf_test_mode);

	mt7996_mcu_add_bss_info(phy, phy->monitor_vif, en);
	mt7996_mcu_add_sta(dev, phy->monitor_vif, NULL, en);

	mt7996_tm_set(dev, SET_ID(BAND_IDX), phy->mt76->band_idx);

	/* use firmware counter for RX stats */
	phy->mt76->test.flag |= MT_TM_FW_RX_COUNT;
}

static void
mt7996_tm_update_channel(struct mt7996_phy *phy)
{
#define CHAN_FREQ_BW_80P80_TAG		(SET_ID(CHAN_FREQ) | BIT(16))
	struct mt7996_dev *dev = phy->dev;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	struct ieee80211_channel *chan = chandef->chan;
	u8 width = chandef->width;
	static const u8 ch_band[] = {
		[NL80211_BAND_2GHZ] = 0,
		[NL80211_BAND_5GHZ] = 1,
		[NL80211_BAND_6GHZ] = 2,
	};

	if (!chan || !chandef) {
		dev_info(dev->mt76.dev, "chandef not found, channel update failed!\n");
		return;
	}

	/* system bw */
	mt7996_tm_set(dev, SET_ID(CBW), mt7996_tm_bw_mapping(width, BW_MAP_NL_TO_FW));

	if (width == NL80211_CHAN_WIDTH_80P80) {
		width = NL80211_CHAN_WIDTH_160;
		mt7996_tm_set(dev, CHAN_FREQ_BW_80P80_TAG, chandef->center_freq2 * 1000);
	}

	/* TODO: define per-packet bw */
	/* per-packet bw */
	mt7996_tm_set(dev, SET_ID(DBW), mt7996_tm_bw_mapping(width, BW_MAP_NL_TO_FW));

	/* control channel selection index */
	mt7996_tm_set(dev, SET_ID(PRIMARY_CH), 0);
	mt7996_tm_set(dev, SET_ID(BAND), ch_band[chan->band]);

	/* trigger switch channel calibration */
	mt7996_tm_set(dev, SET_ID(CHAN_FREQ), chandef->center_freq1 * 1000);

	// TODO: update power limit table
}

static void
mt7996_tm_tx_stop(struct mt76_phy *mphy)
{
	struct mt76_testmode_data *td = &mphy->test;
	struct mt7996_phy *phy = mphy->priv;
	struct mt7996_dev *dev = phy->dev;

	mt7996_tm_set(dev, SET_ID(COMMAND), RF_CMD(STOP_TEST));
	td->tx_pending = 0;
}

static void
mt7996_tm_set_tx_frames(struct mt7996_phy *phy, bool en)
{
#define FRAME_CONTROL		0x88
	struct mt76_testmode_data *td = &phy->mt76->test;
	struct mt7996_dev *dev = phy->dev;

	//TODO: RU operation, replace mcs, nss, and ldpc
	if (en) {
		mt7996_tm_set(dev, SET_ID(MAC_HEADER), FRAME_CONTROL);
		mt7996_tm_set(dev, SET_ID(SEQ_CTRL), 0);
		mt7996_tm_set(dev, SET_ID(TX_COUNT), td->tx_count);
		mt7996_tm_set(dev, SET_ID(TX_MODE), mt7996_tm_rate_to_phy(td->tx_rate_mode));
		mt7996_tm_set(dev, SET_ID(TX_RATE), td->tx_rate_idx);

		if (mt76_testmode_param_present(td, MT76_TM_ATTR_TX_POWER))
			mt7996_tm_set(dev, SET_ID(POWER), td->tx_power[0]);

		if (mt76_testmode_param_present(td, MT76_TM_ATTR_TX_TIME)) {
			mt7996_tm_set(dev, SET_ID(TX_LEN), 0);
			mt7996_tm_set(dev, SET_ID(TX_TIME), td->tx_time);
		} else {
			mt7996_tm_set(dev, SET_ID(TX_LEN), td->tx_mpdu_len);
			mt7996_tm_set(dev, SET_ID(TX_TIME), 0);
		}

		mt7996_tm_set_antenna(phy, SET_ID(TX_PATH));
		mt7996_tm_set_antenna(phy, SET_ID(RX_PATH));
		mt7996_tm_set(dev, SET_ID(STBC), td->tx_rate_stbc);
		mt7996_tm_set(dev, SET_ID(ENCODE_MODE), td->tx_rate_ldpc);
		mt7996_tm_set(dev, SET_ID(IBF_ENABLE), td->ibf);
		mt7996_tm_set(dev, SET_ID(EBF_ENABLE), td->ebf);
		mt7996_tm_set(dev, SET_ID(IPG), td->tx_ipg);
		mt7996_tm_set(dev, SET_ID(GI), td->tx_rate_sgi);
		mt7996_tm_set(dev, SET_ID(NSS), td->tx_rate_nss);
		mt7996_tm_set(dev, SET_ID(AID_OFFSET), 0);
		mt7996_tm_set(dev, SET_ID(PUNCTURE), td->tx_preamble_puncture);

		mt7996_tm_set(dev, SET_ID(MAX_PE), 2);
		mt7996_tm_set(dev, SET_ID(HW_TX_MODE), 0);
		mt7996_tm_update_channel(phy);

		/* trigger firmware to start TX */
		mt7996_tm_set(dev, SET_ID(COMMAND), RF_CMD(START_TX));
	} else {
		mt7996_tm_tx_stop(phy->mt76);
	}
}

static int
mt7996_tm_rx_stats_user_ctrl(struct mt7996_phy *phy, u16 user_idx)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt7996_tm_rx_req req = {
		.band = phy->mt76->band_idx,
		.user_ctrl = {
			.tag = cpu_to_le16(UNI_TM_RX_STAT_SET_USER_CTRL),
			.len = cpu_to_le16(sizeof(req.user_ctrl)),
			.band_idx = phy->mt76->band_idx,
			.user_idx = cpu_to_le16(user_idx),
		},
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TESTMODE_RX_STAT), &req,
				 sizeof(req), false);
}

static void
mt7996_tm_set_rx_frames(struct mt7996_phy *phy, bool en)
{
#define RX_MU_DISABLE	0xf800
	struct mt76_testmode_data *td = &phy->mt76->test;
	struct mt7996_dev *dev = phy->dev;
	int ret;

	if (en) {
		ret = mt7996_tm_rx_stats_user_ctrl(phy, td->aid);
		if (ret) {
			dev_info(dev->mt76.dev, "Set RX stats user control failed!\n");
			return;
		}

		mt7996_tm_update_channel(phy);

		if (td->tx_rate_mode >= MT76_TM_TX_MODE_HE_MU) {
			if (td->aid)
				ret = mt7996_tm_set(dev, SET_ID(RX_MU_AID), td->aid);
			else
				ret = mt7996_tm_set(dev, SET_ID(RX_MU_AID), RX_MU_DISABLE);
		}
		mt7996_tm_set(dev, SET_ID(TX_MODE), mt7996_tm_rate_to_phy(td->tx_rate_mode));
		mt7996_tm_set(dev, SET_ID(GI), td->tx_rate_sgi);
		mt7996_tm_set_antenna(phy, SET_ID(RX_PATH));
		mt7996_tm_set(dev, SET_ID(MAX_PE), 2);

		mt7996_tm_set_mac_addr(dev, td->addr[1], SET_ID(SA));

		/* trigger firmware to start RX */
		mt7996_tm_set(dev, SET_ID(COMMAND), RF_CMD(START_RX));
	} else {
		/* trigger firmware to stop RX */
		mt7996_tm_set(dev, SET_ID(COMMAND), RF_CMD(STOP_TEST));
	}
}

static void
mt7996_tm_set_tx_cont(struct mt7996_phy *phy, bool en)
{
#define CONT_WAVE_MODE_OFDM	3
	struct mt76_testmode_data *td = &phy->mt76->test;
	struct mt7996_dev *dev = phy->dev;

	if (en) {
		mt7996_tm_update_channel(phy);
		mt7996_tm_set(dev, SET_ID(TX_MODE), mt7996_tm_rate_to_phy(td->tx_rate_mode));
		mt7996_tm_set(dev, SET_ID(TX_RATE), td->tx_rate_idx);
		/* fix payload is OFDM */
		mt7996_tm_set(dev, SET_ID(CONT_WAVE_MODE), CONT_WAVE_MODE_OFDM);
		mt7996_tm_set(dev, SET_ID(ANT_MASK), td->tx_antenna_mask);

		/* trigger firmware to start CONT TX */
		mt7996_tm_set(dev, SET_ID(COMMAND), RF_CMD(CONT_WAVE));
	} else {
		/* trigger firmware to stop CONT TX  */
		mt7996_tm_set(dev, SET_ID(COMMAND), RF_CMD(STOP_TEST));
	}
}

static int
mt7996_tm_group_prek(struct mt7996_phy *phy, enum mt76_testmode_state state)
{
	u8 *eeprom;
	u32 i, group_size, dpd_size, size, offs, *pre_cal;
	int ret = 0;
	struct mt7996_dev *dev = phy->dev;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt7996_tm_req req = {
		.rf_test = {
			.tag = cpu_to_le16(UNI_RF_TEST_CTRL),
			.len = cpu_to_le16(sizeof(req.rf_test)),
			.action = RF_ACTION_IN_RF_TEST,
			.icap_len = RF_TEST_ICAP_LEN,
			.op.rf.func_idx = cpu_to_le32(RF_TEST_RE_CAL),
			.op.rf.param.cal_param.func_data = cpu_to_le32(RF_PRE_CAL),
			.op.rf.param.cal_param.band_idx = phy->mt76->band_idx,
		},
	};

	if (!dev->flash_mode) {
		dev_err(dev->mt76.dev, "Currently not in FLASH or BIN FILE mode, return!\n");
		return -EOPNOTSUPP;
	}

	eeprom = mdev->eeprom.data;
	dev->cur_prek_offset = 0;
	group_size = MT_EE_CAL_GROUP_SIZE;
	dpd_size = MT_EE_CAL_DPD_SIZE;
	size = group_size + dpd_size;
	offs = MT_EE_DO_PRE_CAL;

	switch (state) {
	case MT76_TM_STATE_GROUP_PREK:
		if (!dev->cal) {
			dev->cal = devm_kzalloc(mdev->dev, size, GFP_KERNEL);
			if (!dev->cal)
				return -ENOMEM;
		}

		ret = mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TESTMODE_CTRL), &req,
					sizeof(req), false);
		wait_event_timeout(mdev->mcu.wait, dev->cur_prek_offset == group_size,
				   30 * HZ);

		if (ret) {
			dev_err(dev->mt76.dev, "Group Pre-cal: mcu send msg failed!\n");
			return ret;
		}

		if (!ret)
			eeprom[offs] |= MT_EE_WIFI_CAL_GROUP;
		break;
	case MT76_TM_STATE_GROUP_PREK_DUMP:
		pre_cal = (u32 *)dev->cal;
		if (!pre_cal) {
			dev_info(dev->mt76.dev, "Not group pre-cal yet!\n");
			return ret;
		}
		dev_info(dev->mt76.dev, "Group Pre-Cal:\n");
		for (i = 0; i < (group_size / sizeof(u32)); i += 4) {
			dev_info(dev->mt76.dev, "[0x%08lx] 0x%8x 0x%8x 0x%8x 0x%8x\n",
				 i * sizeof(u32), pre_cal[i], pre_cal[i + 1],
				 pre_cal[i + 2], pre_cal[i + 3]);
		}
		break;
	case MT76_TM_STATE_GROUP_PREK_CLEAN:
		pre_cal = (u32 *)dev->cal;
		if (!pre_cal)
			return ret;
		memset(pre_cal, 0, group_size);
		eeprom[offs] &= ~MT_EE_WIFI_CAL_GROUP;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int
mt7996_tm_dpd_prek_send_req(struct mt7996_phy *phy, struct mt7996_tm_req *req,
			    const struct ieee80211_channel *chan_list, u32 channel_size,
			    enum nl80211_chan_width width, u32 func_data)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt76_phy *mphy = phy->mt76;
	struct cfg80211_chan_def chandef_backup, *chandef = &mphy->chandef;
	struct ieee80211_channel chan_backup;
	int i, ret;

	if (!chan_list)
		return -EOPNOTSUPP;

	req->rf_test.op.rf.param.cal_param.func_data = cpu_to_le32(func_data);

	memcpy(&chan_backup, chandef->chan, sizeof(struct ieee80211_channel));
	memcpy(&chandef_backup, chandef, sizeof(struct cfg80211_chan_def));

	for (i = 0; i < channel_size; i++) {
		memcpy(chandef->chan, &chan_list[i], sizeof(struct ieee80211_channel));
		chandef->width = width;

		/* set channel switch reason */
		mphy->hw->conf.flags |= IEEE80211_CONF_OFFCHANNEL;
		mt7996_mcu_set_chan_info(phy, UNI_CHANNEL_SWITCH);

		ret = mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(TESTMODE_CTRL), req,
					sizeof(*req), false);
		if (ret) {
			dev_err(dev->mt76.dev, "DPD Pre-cal: mcu send msg failed!\n");
			goto out;
		}
	}

out:
	mphy->hw->conf.flags &= ~IEEE80211_CONF_OFFCHANNEL;
	memcpy(chandef, &chandef_backup, sizeof(struct cfg80211_chan_def));
	memcpy(chandef->chan, &chan_backup, sizeof(struct ieee80211_channel));
	mt7996_mcu_set_chan_info(phy, UNI_CHANNEL_SWITCH);

	return ret;
}

static int
mt7996_tm_dpd_prek(struct mt7996_phy *phy, enum mt76_testmode_state state)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt76_dev *mdev = &dev->mt76;
	struct mt76_phy *mphy = phy->mt76;
	struct mt7996_tm_req req = {
		.rf_test = {
			.tag = cpu_to_le16(UNI_RF_TEST_CTRL),
			.len = cpu_to_le16(sizeof(req.rf_test)),
			.action = RF_ACTION_IN_RF_TEST,
			.icap_len = RF_TEST_ICAP_LEN,
			.op.rf.func_idx = cpu_to_le32(RF_TEST_RE_CAL),
			.op.rf.param.cal_param.band_idx = phy->mt76->band_idx,
		},
	};
	u32 i, j, group_size, dpd_size, size, offs, *pre_cal;
	u32 wait_on_prek_offset = 0;
	u8 do_precal, *eeprom;
	int ret = 0;

	if (!dev->flash_mode) {
		dev_err(dev->mt76.dev, "Currently not in FLASH or BIN FILE mode, return!\n");
		return -EOPNOTSUPP;
	}

	eeprom = mdev->eeprom.data;
	dev->cur_prek_offset = 0;
	group_size = MT_EE_CAL_GROUP_SIZE;
	dpd_size = MT_EE_CAL_DPD_SIZE;
	size = group_size + dpd_size;
	offs = MT_EE_DO_PRE_CAL;

	if (!dev->cal && state < MT76_TM_STATE_DPD_DUMP) {
		dev->cal = devm_kzalloc(mdev->dev, size, GFP_KERNEL);
		if (!dev->cal)
			return -ENOMEM;
	}

	switch (state) {
	case MT76_TM_STATE_DPD_2G:
		ret = mt7996_tm_dpd_prek_send_req(phy, &req, dpd_2g_ch_list_bw20,
						  dpd_2g_bw20_ch_num,
						  NL80211_CHAN_WIDTH_20, RF_DPD_FLAT_CAL);
		wait_on_prek_offset += dpd_2g_bw20_ch_num * DPD_PER_CH_BW20_SIZE;
		wait_event_timeout(mdev->mcu.wait,
				   dev->cur_prek_offset == wait_on_prek_offset, 30 * HZ);

		do_precal = MT_EE_WIFI_CAL_DPD_2G;
		break;
	case MT76_TM_STATE_DPD_5G:
		/* 5g channel bw20 calibration */
		ret = mt7996_tm_dpd_prek_send_req(phy, &req, mphy->sband_5g.sband.channels,
						  mphy->sband_5g.sband.n_channels,
						  NL80211_CHAN_WIDTH_20, RF_DPD_FLAT_5G_CAL);
		if (ret)
			return ret;
		wait_on_prek_offset += mphy->sband_5g.sband.n_channels * DPD_PER_CH_BW20_SIZE;
		wait_event_timeout(mdev->mcu.wait,
				   dev->cur_prek_offset == wait_on_prek_offset, 30 * HZ);

		/* 5g channel bw160 calibration */
		ret = mt7996_tm_dpd_prek_send_req(phy, &req, dpd_5g_ch_list_bw160,
						  dpd_5g_bw160_ch_num,
						  NL80211_CHAN_WIDTH_160, RF_DPD_FLAT_5G_MEM_CAL);
		wait_on_prek_offset += dpd_5g_bw160_ch_num * DPD_PER_CH_GT_BW20_SIZE;
		wait_event_timeout(mdev->mcu.wait,
				   dev->cur_prek_offset == wait_on_prek_offset, 30 * HZ);

		do_precal = MT_EE_WIFI_CAL_DPD_5G;
		break;
	case MT76_TM_STATE_DPD_6G:
		/* 6g channel bw20 calibration */
		ret = mt7996_tm_dpd_prek_send_req(phy, &req, mphy->sband_6g.sband.channels,
						  mphy->sband_6g.sband.n_channels,
						  NL80211_CHAN_WIDTH_20, RF_DPD_FLAT_6G_CAL);
		if (ret)
			return ret;
		wait_on_prek_offset += mphy->sband_6g.sband.n_channels * DPD_PER_CH_BW20_SIZE;
		wait_event_timeout(mdev->mcu.wait,
				   dev->cur_prek_offset == wait_on_prek_offset, 30 * HZ);

		/* 6g channel bw160 calibration */
		ret = mt7996_tm_dpd_prek_send_req(phy, &req, dpd_6g_ch_list_bw160,
						  dpd_6g_bw160_ch_num,
						  NL80211_CHAN_WIDTH_160, RF_DPD_FLAT_6G_MEM_CAL);
		if (ret)
			return ret;
		wait_on_prek_offset += dpd_6g_bw160_ch_num * DPD_PER_CH_GT_BW20_SIZE;
		wait_event_timeout(mdev->mcu.wait,
				   dev->cur_prek_offset == wait_on_prek_offset, 30 * HZ);

		/* 6g channel bw320 calibration */
		ret = mt7996_tm_dpd_prek_send_req(phy, &req, dpd_6g_ch_list_bw320,
						  dpd_6g_bw320_ch_num,
						  NL80211_CHAN_WIDTH_320, RF_DPD_FLAT_6G_MEM_CAL);
		wait_on_prek_offset += dpd_6g_bw320_ch_num * DPD_PER_CH_GT_BW20_SIZE;
		wait_event_timeout(mdev->mcu.wait,
				   dev->cur_prek_offset == wait_on_prek_offset, 30 * HZ);

		do_precal = MT_EE_WIFI_CAL_DPD_6G;
		break;
	case MT76_TM_STATE_DPD_DUMP:
		if (!dev->cal) {
			dev_info(dev->mt76.dev, "Not DPD pre-cal yet!\n");
			return ret;
		}
		pre_cal = (u32 *)dev->cal;
		dev_info(dev->mt76.dev, "DPD Pre-Cal:\n");
		for (i = 0; i < dpd_size / sizeof(u32); i += 4) {
			j = i + (group_size / sizeof(u32));
			dev_info(dev->mt76.dev, "[0x%08lx] 0x%8x 0x%8x 0x%8x 0x%8x\n",
				 j * sizeof(u32), pre_cal[j], pre_cal[j + 1],
				 pre_cal[j + 2], pre_cal[j + 3]);
		}
		return 0;
	case MT76_TM_STATE_DPD_CLEAN:
		pre_cal = (u32 *)dev->cal;
		if (!pre_cal)
			return ret;
		memset(pre_cal + (group_size / sizeof(u32)), 0, dpd_size);
		do_precal = MT_EE_WIFI_CAL_DPD;
		eeprom[offs] &= ~do_precal;
		return 0;
	default:
		return -EINVAL;
	}

	if (!ret)
		eeprom[offs] |= do_precal;

	return ret;
}

static int
mt7996_tm_dump_precal(struct mt76_phy *mphy, struct sk_buff *msg, int flag, int type)
{
#define DPD_PER_CHAN_SIZE_MASK		GENMASK(31, 30)
#define DPD_2G_RATIO_MASK		GENMASK(29, 20)
#define DPD_5G_RATIO_MASK		GENMASK(19, 10)
#define DPD_6G_RATIO_MASK		GENMASK(9, 0)
	struct mt7996_phy *phy = mphy->priv;
	struct mt7996_dev *dev = phy->dev;
	u32 i, group_size, dpd_size, total_size, size, dpd_info = 0;
	u32 dpd_size_2g, dpd_size_5g, dpd_size_6g;
	u32 base, offs, transmit_size = 1000;
	u8 *pre_cal, *eeprom;
	void *precal;
	enum prek_ops {
		PREK_GET_INFO,
		PREK_SYNC_ALL,
		PREK_SYNC_GROUP,
		PREK_SYNC_DPD_2G,
		PREK_SYNC_DPD_5G,
		PREK_SYNC_DPD_6G,
		PREK_CLEAN_GROUP,
		PREK_CLEAN_DPD,
	};

	if (!dev->cal) {
		dev_info(dev->mt76.dev, "Not pre-cal yet!\n");
		return 0;
	}

	group_size = MT_EE_CAL_GROUP_SIZE;
	dpd_size = MT_EE_CAL_DPD_SIZE;
	total_size = group_size + dpd_size;
	pre_cal = dev->cal;
	eeprom = dev->mt76.eeprom.data;
	offs = MT_EE_DO_PRE_CAL;

	dpd_size_2g = mt7996_get_dpd_per_band_size(dev, NL80211_BAND_2GHZ);
	dpd_size_5g = mt7996_get_dpd_per_band_size(dev, NL80211_BAND_5GHZ);
	dpd_size_6g = mt7996_get_dpd_per_band_size(dev, NL80211_BAND_6GHZ);

	switch (type) {
	case PREK_SYNC_ALL:
		base = 0;
		size = total_size;
		break;
	case PREK_SYNC_GROUP:
		base = 0;
		size = group_size;
		break;
	case PREK_SYNC_DPD_2G:
		base = group_size;
		size = dpd_size_2g;
		break;
	case PREK_SYNC_DPD_5G:
		base = group_size + dpd_size_2g;
		size = dpd_size_5g;
		break;
	case PREK_SYNC_DPD_6G:
		base = group_size + dpd_size_2g + dpd_size_5g;
		size = dpd_size_6g;
		break;
	case PREK_GET_INFO:
		break;
	default:
		return 0;
	}

	if (!flag) {
		if (eeprom[offs] & MT_EE_WIFI_CAL_DPD) {
			dpd_info |= u32_encode_bits(1, DPD_PER_CHAN_SIZE_MASK) |
				    u32_encode_bits(dpd_size_2g / MT_EE_CAL_UNIT,
						    DPD_2G_RATIO_MASK) |
				    u32_encode_bits(dpd_size_5g / MT_EE_CAL_UNIT,
						    DPD_5G_RATIO_MASK) |
				    u32_encode_bits(dpd_size_6g / MT_EE_CAL_UNIT,
						    DPD_6G_RATIO_MASK);
		}
		dev->cur_prek_offset = 0;
		precal = nla_nest_start(msg, MT76_TM_ATTR_PRECAL_INFO);
		if (!precal)
			return -ENOMEM;
		nla_put_u32(msg, 0, group_size);
		nla_put_u32(msg, 1, dpd_size);
		nla_put_u32(msg, 2, dpd_info);
		nla_put_u32(msg, 3, transmit_size);
		nla_put_u32(msg, 4, eeprom[offs]);
		nla_nest_end(msg, precal);
	} else {
		precal = nla_nest_start(msg, MT76_TM_ATTR_PRECAL);
		if (!precal)
			return -ENOMEM;

		transmit_size = (dev->cur_prek_offset + transmit_size < size) ?
				transmit_size : (size - dev->cur_prek_offset);
		for (i = 0; i < transmit_size; i++) {
			if (nla_put_u8(msg, i, pre_cal[base + dev->cur_prek_offset + i]))
				return -ENOMEM;
		}
		dev->cur_prek_offset += transmit_size;

		nla_nest_end(msg, precal);
	}

	return 0;
}

static void
mt7996_tm_re_cal_event(struct mt7996_dev *dev, struct mt7996_tm_rf_test_result *result,
		       struct mt7996_tm_rf_test_data *data)
{
	u32 base, dpd_size_2g, dpd_size_5g, dpd_size_6g, cal_idx, cal_type, len = 0;
	u8 *pre_cal;

	pre_cal = dev->cal;
	dpd_size_2g = mt7996_get_dpd_per_band_size(dev, NL80211_BAND_2GHZ);
	dpd_size_5g = mt7996_get_dpd_per_band_size(dev, NL80211_BAND_5GHZ);
	dpd_size_6g = mt7996_get_dpd_per_band_size(dev, NL80211_BAND_6GHZ);

	cal_idx = le32_to_cpu(data->cal_idx);
	cal_type = le32_to_cpu(data->cal_type);
	len = le32_to_cpu(result->payload_length);
	len = len - sizeof(struct mt7996_tm_rf_test_data);

	switch (cal_type) {
	case RF_PRE_CAL:
		base = 0;
		break;
	case RF_DPD_FLAT_CAL:
		base = MT_EE_CAL_GROUP_SIZE;
		break;
	case RF_DPD_FLAT_5G_CAL:
	case RF_DPD_FLAT_5G_MEM_CAL:
		base = MT_EE_CAL_GROUP_SIZE + dpd_size_2g;
		break;
	case RF_DPD_FLAT_6G_CAL:
	case RF_DPD_FLAT_6G_MEM_CAL:
		base = MT_EE_CAL_GROUP_SIZE + dpd_size_2g + dpd_size_5g;
		break;
	default:
		dev_info(dev->mt76.dev, "Unknown calibration type!\n");
		return;
	}
	pre_cal += (base + dev->cur_prek_offset);

	memcpy(pre_cal, data->cal_data, len);
	dev->cur_prek_offset += len;
}

void mt7996_tm_rf_test_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_tm_event *event;
	struct mt7996_tm_rf_test_result *result;
	struct mt7996_tm_rf_test_data *data;
	static u32 event_type;

	skb_pull(skb, sizeof(struct mt7996_mcu_rxd));
	event = (struct mt7996_tm_event *)skb->data;
	result = (struct mt7996_tm_rf_test_result *)&event->result;
	data = (struct mt7996_tm_rf_test_data *)result->data;

	event_type = le32_to_cpu(result->func_idx);

	switch (event_type) {
	case RF_TEST_RE_CAL:
		mt7996_tm_re_cal_event(dev, result, data);
		break;
	default:
		break;
	}
}

static void
mt7996_tm_update_params(struct mt7996_phy *phy, u32 changed)
{
	struct mt76_testmode_data *td = &phy->mt76->test;
	struct mt7996_dev *dev = phy->dev;

	if (changed & BIT(TM_CHANGED_FREQ_OFFSET)) {
		mt7996_tm_set(dev, SET_ID(FREQ_OFFSET), td->freq_offset);
		mt7996_tm_set(dev, SET_ID(FREQ_OFFSET_C2), td->freq_offset);
	}
	if (changed & BIT(TM_CHANGED_TXPOWER))
		mt7996_tm_set(dev, SET_ID(POWER), td->tx_power[0]);
	if (changed & BIT(TM_CHANGED_SKU_EN)) {
		mt7996_tm_update_channel(phy);
		mt7996_mcu_set_tx_power_ctrl(phy, POWER_CTRL(SKU_POWER_LIMIT), td->sku_en);
		mt7996_mcu_set_tx_power_ctrl(phy, POWER_CTRL(BACKOFF_POWER_LIMIT), td->sku_en);
		mt7996_mcu_set_txpower_sku(phy);
	}
	if (changed & BIT(TM_CHANGED_TX_LENGTH)) {
		mt7996_tm_set(dev, SET_ID(TX_LEN), td->tx_mpdu_len);
		mt7996_tm_set(dev, SET_ID(TX_TIME), 0);
	}
	if (changed & BIT(TM_CHANGED_TX_TIME)) {
		mt7996_tm_set(dev, SET_ID(TX_LEN), 0);
		mt7996_tm_set(dev, SET_ID(TX_TIME), td->tx_time);
	}
	if (changed & BIT(TM_CHANGED_CFG)) {
		u32 func_idx = td->cfg.enable ? SET_ID(CFG_ON) : SET_ID(CFG_OFF);

		mt7996_tm_set(dev, func_idx, td->cfg.type);
	}
}

static int
mt7996_tm_set_state(struct mt76_phy *mphy, enum mt76_testmode_state state)
{
	struct mt76_testmode_data *td = &mphy->test;
	struct mt7996_phy *phy = mphy->priv;
	enum mt76_testmode_state prev_state = td->state;

	mphy->test.state = state;

	if (prev_state != MT76_TM_STATE_OFF)
		mt7996_tm_set(phy->dev, SET_ID(BAND_IDX), mphy->band_idx);

	if (prev_state == MT76_TM_STATE_TX_FRAMES ||
	    state == MT76_TM_STATE_TX_FRAMES)
		mt7996_tm_set_tx_frames(phy, state == MT76_TM_STATE_TX_FRAMES);
	else if (prev_state == MT76_TM_STATE_RX_FRAMES ||
		 state == MT76_TM_STATE_RX_FRAMES)
		mt7996_tm_set_rx_frames(phy, state == MT76_TM_STATE_RX_FRAMES);
	else if (prev_state == MT76_TM_STATE_TX_CONT ||
		 state == MT76_TM_STATE_TX_CONT)
		mt7996_tm_set_tx_cont(phy, state == MT76_TM_STATE_TX_CONT);
	else if (prev_state == MT76_TM_STATE_OFF ||
		 state == MT76_TM_STATE_OFF)
		mt7996_tm_init(phy, !(state == MT76_TM_STATE_OFF));
	else if (state >= MT76_TM_STATE_GROUP_PREK && state <= MT76_TM_STATE_GROUP_PREK_CLEAN)
		return mt7996_tm_group_prek(phy, state);
	else if (state >= MT76_TM_STATE_DPD_2G && state <= MT76_TM_STATE_DPD_CLEAN)
		return mt7996_tm_dpd_prek(phy, state);

	if ((state == MT76_TM_STATE_IDLE &&
	     prev_state == MT76_TM_STATE_OFF) ||
	    (state == MT76_TM_STATE_OFF &&
	     prev_state == MT76_TM_STATE_IDLE)) {
		u32 changed = 0;
		int i, ret;

		for (i = 0; i < ARRAY_SIZE(tm_change_map); i++) {
			u16 cur = tm_change_map[i];

			if (mt76_testmode_param_present(td, cur))
				changed |= BIT(i);
		}

		ret = mt7996_tm_check_antenna(phy);
		if (ret)
			return ret;

		mt7996_tm_update_params(phy, changed);
	}

	return 0;
}

static int
mt7996_tm_set_params(struct mt76_phy *mphy, struct nlattr **tb,
		     enum mt76_testmode_state new_state)
{
	struct mt76_testmode_data *td = &mphy->test;
	struct mt7996_phy *phy = mphy->priv;
	struct mt7996_dev *dev = phy->dev;
	u32 changed = 0;
	int i, ret;

	BUILD_BUG_ON(NUM_TM_CHANGED >= 32);

	if (new_state == MT76_TM_STATE_OFF ||
	    td->state == MT76_TM_STATE_OFF)
		return 0;

	ret = mt7996_tm_check_antenna(phy);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(tm_change_map); i++) {
		if (tb[tm_change_map[i]])
			changed |= BIT(i);
	}

	mt7996_tm_set(dev, SET_ID(BAND_IDX), mphy->band_idx);
	mt7996_tm_update_params(phy, changed);

	return 0;
}

static int
mt7996_tm_get_rx_stats(struct mt7996_phy *phy)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt7996_tm_rx_req req = {
		.band = phy->mt76->band_idx,
		.rx_stat_all = {
			.tag = cpu_to_le16(UNI_TM_RX_STAT_GET_ALL_V2),
			.len = cpu_to_le16(sizeof(req.rx_stat_all)),
			.band_idx = phy->mt76->band_idx,
		},
	};
	struct mt76_testmode_data *td = &phy->mt76->test;
	struct mt7996_tm_rx_event *rx_stats;
	struct mt7996_tm_rx_event_stat_all *rx_stats_all;
	struct sk_buff *skb;
	enum mt76_rxq_id qid;
	int i, ret = 0;
	u32 mac_rx_mdrdy_cnt;
	u16 mac_rx_len_mismatch, fcs_err_count;

	if (td->state != MT76_TM_STATE_RX_FRAMES)
		return 0;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_WM_UNI_CMD_QUERY(TESTMODE_RX_STAT),
					&req, sizeof(req), true, &skb);

	if (ret)
		return ret;

	rx_stats = (struct mt7996_tm_rx_event *)skb->data;
	rx_stats_all = &rx_stats->rx_stat_all;

	phy->test.last_freq_offset = le32_to_cpu(rx_stats_all->user_info[0].freq_offset);
	phy->test.last_snr = le32_to_cpu(rx_stats_all->user_info[0].snr);
	for (i = 0; i < ARRAY_SIZE(phy->test.last_rcpi); i++) {
		phy->test.last_rcpi[i] = le16_to_cpu(rx_stats_all->rxv_info[i].rcpi);
		phy->test.last_rssi[i] = le16_to_cpu(rx_stats_all->rxv_info[i].rssi);
		phy->test.last_ib_rssi[i] = rx_stats_all->fagc[i].ib_rssi;
		phy->test.last_wb_rssi[i] = rx_stats_all->fagc[i].wb_rssi;
	}

	if (phy->mt76->band_idx == 2)
		qid = MT_RXQ_BAND2;
	else if (phy->mt76->band_idx == 1)
		qid = MT_RXQ_BAND1;
	else
		qid = MT_RXQ_MAIN;

	fcs_err_count = le16_to_cpu(rx_stats_all->band_info.mac_rx_fcs_err_cnt);
	mac_rx_len_mismatch = le16_to_cpu(rx_stats_all->band_info.mac_rx_len_mismatch);
	mac_rx_mdrdy_cnt = le32_to_cpu(rx_stats_all->band_info.mac_rx_mdrdy_cnt);
	td->rx_stats.packets[qid] += mac_rx_mdrdy_cnt;
	td->rx_stats.packets[qid] += fcs_err_count;
	td->rx_stats.fcs_error[qid] += fcs_err_count;
	td->rx_stats.len_mismatch += mac_rx_len_mismatch;

	dev_kfree_skb(skb);

	return ret;
}

static void
mt7996_tm_reset_trx_stats(struct mt76_phy *mphy)
{
	struct mt7996_phy *phy = mphy->priv;
	struct mt7996_dev *dev = phy->dev;

	memset(&mphy->test.rx_stats, 0, sizeof(mphy->test.rx_stats));
	mt7996_tm_set(dev, SET_ID(TRX_COUNTER_RESET), 0);
}

static int
mt7996_tm_get_tx_stats(struct mt7996_phy *phy)
{
	struct mt7996_dev *dev = phy->dev;
	struct mt76_testmode_data *td = &phy->mt76->test;
	int ret;

	if (td->state != MT76_TM_STATE_TX_FRAMES)
		return 0;

	ret = mt7996_tm_get(dev, GET_ID(TXED_COUNT), 0, &td->tx_done);
	if (ret)
		return ret;

	td->tx_pending = td->tx_count - td->tx_done;

	return ret;
}

static int
mt7996_tm_dump_stats(struct mt76_phy *mphy, struct sk_buff *msg)
{
	struct mt7996_phy *phy = mphy->priv;
	void *rx, *rssi;
	int i;

	mt7996_tm_set(phy->dev, SET_ID(BAND_IDX), mphy->band_idx);
	mt7996_tm_get_rx_stats(phy);
	mt7996_tm_get_tx_stats(phy);

	rx = nla_nest_start(msg, MT76_TM_STATS_ATTR_LAST_RX);
	if (!rx)
		return -ENOMEM;

	if (nla_put_s32(msg, MT76_TM_RX_ATTR_FREQ_OFFSET, phy->test.last_freq_offset))
		return -ENOMEM;

	rssi = nla_nest_start(msg, MT76_TM_RX_ATTR_RCPI);
	if (!rssi)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(phy->test.last_rcpi); i++)
		if (nla_put_u8(msg, i, phy->test.last_rcpi[i]))
			return -ENOMEM;

	nla_nest_end(msg, rssi);

	rssi = nla_nest_start(msg, MT76_TM_RX_ATTR_RSSI);
	if (!rssi)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(phy->test.last_rssi); i++)
		if (nla_put_s8(msg, i, phy->test.last_rssi[i]))
			return -ENOMEM;

	nla_nest_end(msg, rssi);

	rssi = nla_nest_start(msg, MT76_TM_RX_ATTR_IB_RSSI);
	if (!rssi)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(phy->test.last_ib_rssi); i++)
		if (nla_put_s8(msg, i, phy->test.last_ib_rssi[i]))
			return -ENOMEM;

	nla_nest_end(msg, rssi);

	rssi = nla_nest_start(msg, MT76_TM_RX_ATTR_WB_RSSI);
	if (!rssi)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(phy->test.last_wb_rssi); i++)
		if (nla_put_s8(msg, i, phy->test.last_wb_rssi[i]))
			return -ENOMEM;

	nla_nest_end(msg, rssi);

	if (nla_put_u8(msg, MT76_TM_RX_ATTR_SNR, phy->test.last_snr))
		return -ENOMEM;

	nla_nest_end(msg, rx);

	return 0;
}

static int
mt7996_tm_write_back_to_efuse(struct mt7996_dev *dev)
{
	struct mt7996_mcu_eeprom_info req = {
		.tag = cpu_to_le16(UNI_EFUSE_ACCESS),
		.len = cpu_to_le16(sizeof(req) - 4),
	};
	u8 read_buf[MT76_TM_EEPROM_BLOCK_SIZE], *eeprom = dev->mt76.eeprom.data;
	int i, ret = -EINVAL;

	/* prevent from damaging chip id in efuse */
	if (mt76_chip(&dev->mt76) != get_unaligned_le16(eeprom))
		goto out;

	for (i = 0; i < MT7996_EEPROM_SIZE; i += MT76_TM_EEPROM_BLOCK_SIZE) {
		req.addr = cpu_to_le32(i);
		memcpy(req.data, eeprom + i, MT76_TM_EEPROM_BLOCK_SIZE);

		ret = mt7996_mcu_get_eeprom(dev, i, read_buf);
		if (ret < 0)
			return ret;

		if (!memcmp(req.data, read_buf, MT76_TM_EEPROM_BLOCK_SIZE))
			continue;

		ret = mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(EFUSE_CTRL),
					&req, sizeof(req), true);
		if (ret)
			return ret;
	}

out:
	return ret;
}

static int
mt7996_tm_set_eeprom(struct mt76_phy *mphy, u32 offset, u8 *val, u8 action)
{
	struct mt7996_phy *phy = mphy->priv;
	struct mt7996_dev *dev = phy->dev;
	u8 *eeprom = dev->mt76.eeprom.data;
	int ret = 0;

	if (offset >= MT7996_EEPROM_SIZE)
		return -EINVAL;

	switch (action) {
	case MT76_TM_EEPROM_ACTION_UPDATE_DATA:
		memcpy(eeprom + offset, val, MT76_TM_EEPROM_BLOCK_SIZE);
		break;
	case MT76_TM_EEPROM_ACTION_UPDATE_BUFFER_MODE:
		ret = mt7996_mcu_set_eeprom(dev);
		break;
	case MT76_TM_EEPROM_ACTION_WRITE_TO_EFUSE:
		ret = mt7996_tm_write_back_to_efuse(dev);
		break;
	default:
		break;
	}

	return ret;
}

const struct mt76_testmode_ops mt7996_testmode_ops = {
	.set_state = mt7996_tm_set_state,
	.set_params = mt7996_tm_set_params,
	.dump_stats = mt7996_tm_dump_stats,
	.reset_rx_stats = mt7996_tm_reset_trx_stats,
	.tx_stop = mt7996_tm_tx_stop,
	.set_eeprom = mt7996_tm_set_eeprom,
	.dump_precal = mt7996_tm_dump_precal,
};
