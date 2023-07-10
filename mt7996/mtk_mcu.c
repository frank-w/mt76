// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2023 MediaTek Inc.
 */

#include <linux/firmware.h>
#include <linux/fs.h>
#include "mt7996.h"
#include "mcu.h"
#include "mac.h"
#include "mtk_mcu.h"

#ifdef CONFIG_MTK_DEBUG

int mt7996_mcu_get_tx_power_info(struct mt7996_phy *phy, u8 category, void *event)
{
	struct mt7996_dev *dev = phy->dev;
	struct tx_power_ctrl req = {
		.tag = cpu_to_le16(UNI_TXPOWER_SHOW_INFO),
		.len = cpu_to_le16(sizeof(req) - 4),
		.power_ctrl_id = UNI_TXPOWER_SHOW_INFO,
		.show_info_category = category,
		.band_idx = phy->mt76->band_idx,
	};
	struct sk_buff *skb;
	int ret;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76,
					MCU_WM_UNI_CMD_QUERY(TXPOWER),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;

	memcpy(event, skb->data, sizeof(struct mt7996_mcu_txpower_event));

	dev_kfree_skb(skb);

	return 0;
}

int mt7996_mcu_muru_dbg_info(struct mt7996_dev *dev, u16 item, u8 val)
{
	struct {
		u8 __rsv1[4];

		__le16 tag;
		__le16 len;

		__le16 item;
		u8 __rsv2[2];
		__le32 value;
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_MURU_DBG_INFO),
		.len = cpu_to_le16(sizeof(req) - 4),
		.item = cpu_to_le16(item),
		.value = cpu_to_le32(val),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(MURU), &req,
				 sizeof(req), true);
}

int mt7996_mcu_edcca_enable(struct mt7996_phy *phy, bool enable)
{
	struct mt7996_dev *dev = phy->dev;
	struct cfg80211_chan_def *chandef = &phy->mt76->chandef;
	enum nl80211_band band = chandef->chan->band;
	struct {
		u8 band_idx;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;
		u8 enable;
		u8 std;
		u8 _rsv2[2];
	} __packed req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_BAND_CONFIG_EDCCA_ENABLE),
		.len = cpu_to_le16(sizeof(req) - 4),
		.enable = enable,
		.std = EDCCA_DEFAULT,
	};

	switch (dev->mt76.region) {
	case NL80211_DFS_JP:
		req.std = EDCCA_JAPAN;
		break;
	case NL80211_DFS_FCC:
		if (band == NL80211_BAND_6GHZ)
			req.std = EDCCA_FCC;
		break;
	case NL80211_DFS_ETSI:
		if (band == NL80211_BAND_6GHZ)
			req.std = EDCCA_ETSI;
		break;
	default:
		break;
	}

	return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(BAND_CONFIG),
				 &req, sizeof(req), true);
}

int mt7996_mcu_edcca_threshold_ctrl(struct mt7996_phy *phy, u8 *value, bool set)
{
	struct {
		u8 band_idx;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;
		u8 threshold[4];
		bool init;
	} __packed *res, req = {
		.band_idx = phy->mt76->band_idx,
		.tag = cpu_to_le16(UNI_BAND_CONFIG_EDCCA_THRESHOLD),
		.len = cpu_to_le16(sizeof(req) - 4),
		.init = false,
	};
	struct sk_buff *skb;
	int ret;
	int i;

	for (i = 0; i < EDCCA_MAX_BW_NUM; i++)
		req.threshold[i] = value[i];

	if (set)
		return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(BAND_CONFIG),
					 &req, sizeof(req), true);

	ret = mt76_mcu_send_and_get_msg(&phy->dev->mt76,
					MCU_WM_UNI_CMD_QUERY(BAND_CONFIG),
					&req, sizeof(req), true, &skb);

	if (ret)
		return ret;

	res = (void *)skb->data;

	for (i = 0; i < EDCCA_MAX_BW_NUM; i++)
		value[i] = res->threshold[i];

	dev_kfree_skb(skb);

	return 0;
}

int mt7996_mcu_set_sr_enable(struct mt7996_phy *phy, u8 action, u64 val, bool set)
{
	struct {
		u8 band_idx;
		u8 _rsv[3];

		__le16 tag;
		__le16 len;

		__le32 val;

	} __packed req = {
		.band_idx = phy->mt76->band_idx,

		.tag = cpu_to_le16(action),
		.len = cpu_to_le16(sizeof(req) - 4),

		.val = cpu_to_le32((u32) val),
	};

	if (set)
		return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD(SR), &req,
					 sizeof(req), false);
	else
		return mt76_mcu_send_msg(&phy->dev->mt76, MCU_WM_UNI_CMD_QUERY(SR), &req,
					 sizeof(req), false);
}

void mt7996_mcu_rx_sr_swsd(struct mt7996_dev *dev, struct sk_buff *skb)
{
#define SR_SCENE_DETECTION_TIMER_PERIOD_MS 500
	struct mt7996_mcu_sr_swsd_event *event;
	static const char * const rules[] = {"1 - NO CONNECTED", "2 - NO CONGESTION",
					     "3 - NO INTERFERENCE", "4 - SR ON"};
	u8 idx;

	event = (struct mt7996_mcu_sr_swsd_event *)skb->data;
	idx = event->basic.band_idx;

	dev_info(dev->mt76.dev, "Band index = %u\n", le16_to_cpu(event->basic.band_idx));
	dev_info(dev->mt76.dev, "Hit Rule = %s\n", rules[event->tlv[idx].rule]);
	dev_info(dev->mt76.dev, "Timer Period = %d(us)\n"
		 "Congestion Ratio  = %d.%1d%%\n",
		 SR_SCENE_DETECTION_TIMER_PERIOD_MS * 1000,
		 le32_to_cpu(event->tlv[idx].total_airtime_ratio) / 10,
		 le32_to_cpu(event->tlv[idx].total_airtime_ratio) % 10);
	dev_info(dev->mt76.dev,
		 "Total Airtime = %d(us)\n"
		 "ChBusy = %d\n"
		 "SrTx = %d\n"
		 "OBSS = %d\n"
		 "MyTx = %d\n"
		 "MyRx = %d\n"
		 "Interference Ratio = %d.%1d%%\n",
		 le32_to_cpu(event->tlv[idx].total_airtime),
		 le32_to_cpu(event->tlv[idx].channel_busy_time),
		 le32_to_cpu(event->tlv[idx].sr_tx_airtime),
		 le32_to_cpu(event->tlv[idx].obss_airtime),
		 le32_to_cpu(event->tlv[idx].my_tx_airtime),
		 le32_to_cpu(event->tlv[idx].my_rx_airtime),
		 le32_to_cpu(event->tlv[idx].obss_airtime_ratio) / 10,
		 le32_to_cpu(event->tlv[idx].obss_airtime_ratio) % 10);
}

void mt7996_mcu_rx_sr_hw_indicator(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt7996_mcu_sr_hw_ind_event *event;

	event = (struct mt7996_mcu_sr_hw_ind_event *)skb->data;

	dev_info(dev->mt76.dev, "Inter PPDU Count = %u\n",
		 le16_to_cpu(event->inter_bss_ppdu_cnt));
	dev_info(dev->mt76.dev, "SR Valid Count = %u\n",
		 le16_to_cpu(event->non_srg_valid_cnt));
	dev_info(dev->mt76.dev, "SR Tx Count = %u\n",
		 le32_to_cpu(event->sr_ampdu_mpdu_cnt));
	dev_info(dev->mt76.dev, "SR Tx Acked Count = %u\n",
		 le32_to_cpu(event->sr_ampdu_mpdu_acked_cnt));
}

void mt7996_mcu_rx_sr_event(struct mt7996_dev *dev, struct sk_buff *skb)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt7996_phy *phy;
	struct mt7996_mcu_sr_common_event *event;

	event = (struct mt7996_mcu_sr_common_event *)skb->data;
	mphy = dev->mt76.phys[event->basic.band_idx];
	if (!mphy)
		return;

	phy = (struct mt7996_phy *)mphy->priv;

	switch (le16_to_cpu(event->basic.tag)) {
	case UNI_EVENT_SR_CFG_SR_ENABLE:
		phy->sr_enable = le32_to_cpu(event->value) ? true : false;
		break;
	case UNI_EVENT_SR_HW_ESR_ENABLE:
		phy->enhanced_sr_enable = le32_to_cpu(event->value) ? true : false;
		break;
	case UNI_EVENT_SR_SW_SD:
		mt7996_mcu_rx_sr_swsd(dev, skb);
		break;
	case UNI_EVENT_SR_HW_IND:
		mt7996_mcu_rx_sr_hw_indicator(dev, skb);
		break;
	default:
		dev_info(dev->mt76.dev, "Unknown SR event tag %d\n",
			 le16_to_cpu(event->basic.tag));
	}
}
#endif
