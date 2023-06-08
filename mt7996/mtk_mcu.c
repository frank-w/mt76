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

#endif
