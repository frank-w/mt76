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
#endif
