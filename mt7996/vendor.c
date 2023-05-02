// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2020, MediaTek Inc. All rights reserved.
 */

#include <net/netlink.h>

#include "mt7996.h"
#include "mcu.h"
#include "vendor.h"
#include "mtk_mcu.h"

static const struct nla_policy
mu_ctrl_policy[NUM_MTK_VENDOR_ATTRS_MU_CTRL] = {
	[MTK_VENDOR_ATTR_MU_CTRL_ONOFF] = {.type = NLA_U8 },
	[MTK_VENDOR_ATTR_MU_CTRL_DUMP] = {.type = NLA_U8 },
};

static const struct nla_policy
amnt_ctrl_policy[NUM_MTK_VENDOR_ATTRS_AMNT_CTRL] = {
	[MTK_VENDOR_ATTR_AMNT_CTRL_SET] = {.type = NLA_NESTED },
	[MTK_VENDOR_ATTR_AMNT_CTRL_DUMP] = { .type = NLA_NESTED },
};

static const struct nla_policy
amnt_set_policy[NUM_MTK_VENDOR_ATTRS_AMNT_SET] = {
	[MTK_VENDOR_ATTR_AMNT_SET_INDEX] = {.type = NLA_U8 },
	[MTK_VENDOR_ATTR_AMNT_SET_MACADDR] = NLA_POLICY_EXACT_LEN_WARN(ETH_ALEN),
};

static const struct nla_policy
amnt_dump_policy[NUM_MTK_VENDOR_ATTRS_AMNT_DUMP] = {
	[MTK_VENDOR_ATTR_AMNT_DUMP_INDEX] = {.type = NLA_U8 },
	[MTK_VENDOR_ATTR_AMNT_DUMP_LEN] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_AMNT_DUMP_RESULT] = { .type = NLA_NESTED },
};

static struct nla_policy
bss_color_ctrl_policy[NUM_MTK_VENDOR_ATTRS_BSS_COLOR_CTRL] = {
	[MTK_VENDOR_ATTR_AVAL_BSS_COLOR_BMP] = { .type = NLA_U64 },
};

struct mt7996_amnt_data {
	u8 idx;
	u8 addr[ETH_ALEN];
	s8 rssi[4];
	u32 last_seen;
};

static int mt7996_vendor_mu_ctrl(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 const void *data,
				 int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct nlattr *tb[NUM_MTK_VENDOR_ATTRS_MU_CTRL];
	int err;
	u8 val8;
	u32 val32 = 0;

	err = nla_parse(tb, MTK_VENDOR_ATTR_MU_CTRL_MAX, data, data_len,
			mu_ctrl_policy, NULL);
	if (err)
		return err;

	if (tb[MTK_VENDOR_ATTR_MU_CTRL_ONOFF]) {
		val8 = nla_get_u8(tb[MTK_VENDOR_ATTR_MU_CTRL_ONOFF]);
		val32 |= FIELD_PREP(RATE_CFG_MODE, RATE_PARAM_AUTO_MU) |
			 FIELD_PREP(RATE_CFG_VAL, val8);
		ieee80211_iterate_active_interfaces_atomic(hw, IEEE80211_IFACE_ITER_RESUME_ALL,
							   mt7996_set_wireless_vif, &val32);
	}

	return 0;
}

static int
mt7996_vendor_mu_ctrl_dump(struct wiphy *wiphy, struct wireless_dev *wdev,
			   struct sk_buff *skb, const void *data, int data_len,
			   unsigned long *storage)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	int len = 0;

	if (*storage == 1)
		return -ENOENT;
	*storage = 1;

	if (nla_put_u8(skb, MTK_VENDOR_ATTR_MU_CTRL_DUMP, phy->muru_onoff))
		return -ENOMEM;
	len += 1;

	return len;
}

void mt7996_vendor_amnt_fill_rx(struct mt7996_phy *phy, struct sk_buff *skb)
{
	struct mt76_rx_status *status = (struct mt76_rx_status *)skb->cb;
	struct mt7996_air_monitor_ctrl *ctrl = &phy->amnt_ctrl;
	struct ieee80211_hdr *hdr = mt76_skb_get_hdr(skb);
	__le16 fc = hdr->frame_control;
	u8 addr[ETH_ALEN];
	int i;

	if (!ieee80211_has_fromds(fc))
		ether_addr_copy(addr, hdr->addr2);
	else if (ieee80211_has_tods(fc))
		ether_addr_copy(addr, hdr->addr4);
	else
		ether_addr_copy(addr, hdr->addr3);

	spin_lock_bh(&phy->amnt_lock);
	for (i = 0; i < MT7996_AIR_MONITOR_MAX_ENTRY; i++) {
		struct mt7996_air_monitor_entry *entry;

		if (ether_addr_equal(addr, ctrl->entry[i].addr)) {
			entry = &ctrl->entry[i];
			entry->rssi[0] = status->chain_signal[0];
			entry->rssi[1] = status->chain_signal[1];
			entry->rssi[2] = status->chain_signal[2];
			entry->rssi[3] = status->chain_signal[3];
			entry->last_seen = jiffies;
			break;
		}
	}
	spin_unlock_bh(&phy->amnt_lock);
}

static int
mt7996_vendor_smesh_ctrl(struct mt7996_phy *phy, u8 write,
			 u8 enable, u8 *value)
{
#define UNI_CMD_SMESH_PARAM  0
	struct mt7996_dev *dev = phy->dev;
	struct smesh_param {
		__le16 tag;
		__le16 length;

		u8 enable;
		bool a2;
		bool a1;
		bool data;
		bool mgnt;
		bool ctrl;
		u8 padding[2];
	} req = {
		.tag = cpu_to_le16(UNI_CMD_SMESH_PARAM),
		.length = cpu_to_le16(sizeof(req) - 4),

		.enable = enable,
		.a2 = true,
		.a1 = true,
		.data = true,
		.mgnt = false,
		.ctrl = false,
	};
	struct smesh_param *res;
	struct sk_buff *skb;
	int ret = 0;

	if (!value)
		return -EINVAL;

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_WM_UNI_CMD(CFG_SMESH),
					&req, sizeof(req), !write, &skb);

	if (ret || write)
		return ret;

	res = (struct smesh_param *) skb->data;

	*value = res->enable;

	dev_kfree_skb(skb);

	return 0;
}

static int
mt7996_vendor_amnt_muar(struct mt7996_phy *phy, u8 muar_idx, u8 *addr)
{
#define UNI_CMD_MUAR_ENTRY  2
	struct mt7996_dev *dev = phy->dev;
	struct muar_entry {
		__le16 tag;
		__le16 length;

		bool smesh;
		u8 hw_bss_index;
		u8 muar_idx;
		u8 entry_add;
		u8 mac_addr[6];
		u8 padding[2];
	} __packed req = {
		.tag = cpu_to_le16(UNI_CMD_MUAR_ENTRY),
		.length = cpu_to_le16(sizeof(req) - 4),

		.smesh = true,
		.hw_bss_index = phy != &dev->phy,
		.muar_idx = muar_idx,
		.entry_add = 1,
	};

	ether_addr_copy(req.mac_addr, addr);
	return mt76_mcu_send_msg(&dev->mt76, MCU_WM_UNI_CMD(REPT_MUAR), &req,
				 sizeof(req), true);
}

static int
mt7996_vendor_amnt_set_en(struct mt7996_phy *phy, u8 enable)
{
	u8 status;
	int ret;

	ret = mt7996_vendor_smesh_ctrl(phy, 0, enable, &status);
	if (ret)
		return ret;

	if (status == enable)
		return 0;

	ret = mt7996_vendor_smesh_ctrl(phy, 1, enable, &status);
	if (ret)
		return ret;

	return 0;
}

static int
mt7996_vendor_amnt_set_addr(struct mt7996_phy *phy, u8 index, u8 *addr)
{
	struct mt7996_air_monitor_ctrl *amnt_ctrl = &phy->amnt_ctrl;
	struct mt7996_air_monitor_group *group;
	struct mt7996_air_monitor_entry *entry;
	int ret, i, j;

	if (index >= MT7996_AIR_MONITOR_MAX_ENTRY)
		return -1;

	spin_lock_bh(&phy->amnt_lock);
	entry = &amnt_ctrl->entry[index];
	if (!is_zero_ether_addr(addr)) {
		if (entry->enable == false) {
			for (i = 0; i < MT7996_AIR_MONITOR_MAX_GROUP; i++) {
				group = &(amnt_ctrl->group[i]);
				if (group->used[0] == false)
					j = 0;
				else if (group->used[1] == false)
					j = 1;
				else
					continue;

				group->enable = true;
				group->used[j] = true;
				entry->enable = true;
				entry->group_idx = i;
				entry->group_used_idx = j;
				entry->muar_idx = 32 + 4 * i + 2 * j;
				break;
			}
		}
	} else {
		group = &(amnt_ctrl->group[entry->group_idx]);

		group->used[entry->group_used_idx] = false;
		if (group->used[0] == false && group->used[1] == false)
			group->enable = false;

		entry->enable = false;
	}
	ether_addr_copy(entry->addr, addr);
	amnt_ctrl->enable &= ~(1 << entry->group_idx);
	amnt_ctrl->enable |= entry->enable << entry->group_idx;
	spin_unlock_bh(&phy->amnt_lock);

	ret = mt7996_vendor_amnt_muar(phy, entry->muar_idx, addr);
	if (ret)
		return ret;

	return mt7996_vendor_amnt_set_en(phy, amnt_ctrl->enable);
}

static int
mt7966_vendor_amnt_ctrl(struct wiphy *wiphy, struct wireless_dev *wdev,
			const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct nlattr *tb1[NUM_MTK_VENDOR_ATTRS_AMNT_CTRL];
	struct nlattr *tb2[NUM_MTK_VENDOR_ATTRS_AMNT_SET];
	u8 index = 0;
	u8 mac_addr[ETH_ALEN];
	int err;

	err = nla_parse(tb1, MTK_VENDOR_ATTR_AMNT_CTRL_MAX, data, data_len,
			amnt_ctrl_policy, NULL);
	if (err)
		return err;

	if (!tb1[MTK_VENDOR_ATTR_AMNT_CTRL_SET])
		return -EINVAL;

	err = nla_parse_nested(tb2, MTK_VENDOR_ATTR_AMNT_SET_MAX,
		tb1[MTK_VENDOR_ATTR_AMNT_CTRL_SET], amnt_set_policy, NULL);

	if (!tb2[MTK_VENDOR_ATTR_AMNT_SET_INDEX] ||
		!tb2[MTK_VENDOR_ATTR_AMNT_SET_MACADDR])
		return -EINVAL;

	index = nla_get_u8(tb2[MTK_VENDOR_ATTR_AMNT_SET_INDEX]);
	memcpy(mac_addr, nla_data(tb2[MTK_VENDOR_ATTR_AMNT_SET_MACADDR]), ETH_ALEN);

	return mt7996_vendor_amnt_set_addr(phy, index, mac_addr);
}

int mt7996_vendor_amnt_sta_remove(struct mt7996_phy *phy,
				  struct ieee80211_sta *sta)
{
	u8 zero[ETH_ALEN] = {};
	int i;

	if (!phy->amnt_ctrl.enable)
		return 0;

	for (i = 0; i < MT7996_AIR_MONITOR_MAX_ENTRY; i++)
		if (ether_addr_equal(sta->addr, phy->amnt_ctrl.entry[i].addr))
			return mt7996_vendor_amnt_set_addr(phy, i, zero);
	return 0;
}

static int
mt7996_amnt_dump(struct mt7996_phy *phy, struct sk_buff *skb,
		 u8 amnt_idx, int *attrtype)
{
	struct mt7996_air_monitor_entry *entry;
	struct mt7996_amnt_data data;
	u32 last_seen = 0;

	spin_lock_bh(&phy->amnt_lock);
	entry = &phy->amnt_ctrl.entry[amnt_idx];
	if (entry->enable == 0) {
		spin_unlock_bh(&phy->amnt_lock);
		return 0;
	}

	last_seen = jiffies_to_msecs(jiffies - entry->last_seen);
	ether_addr_copy(data.addr, entry->addr);
	data.rssi[0] = entry->rssi[0];
	data.rssi[1] = entry->rssi[1];
	data.rssi[2] = entry->rssi[2];
	data.rssi[3] = entry->rssi[3];
	spin_unlock_bh(&phy->amnt_lock);

	data.idx = amnt_idx;
	data.last_seen = last_seen;

	nla_put(skb, (*attrtype)++, sizeof(struct mt7996_amnt_data), &data);

	return 1;
}

static int
mt7966_vendor_amnt_ctrl_dump(struct wiphy *wiphy, struct wireless_dev *wdev,
			     struct sk_buff *skb, const void *data, int data_len,
			     unsigned long *storage)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mt7996_phy *phy = mt7996_hw_phy(hw);
	struct nlattr *tb1[NUM_MTK_VENDOR_ATTRS_AMNT_CTRL];
	struct nlattr *tb2[NUM_MTK_VENDOR_ATTRS_AMNT_DUMP];
	void *a, *b;
	int err = 0, attrtype = 0, i, len = 0;
	u8 amnt_idx;

	if (*storage == 1)
		return -ENOENT;
	*storage = 1;

	err = nla_parse(tb1, MTK_VENDOR_ATTR_AMNT_CTRL_MAX, data, data_len,
			amnt_ctrl_policy, NULL);
	if (err)
		return err;

	if (!tb1[MTK_VENDOR_ATTR_AMNT_CTRL_DUMP])
		return -EINVAL;

	err = nla_parse_nested(tb2, MTK_VENDOR_ATTR_AMNT_DUMP_MAX,
			       tb1[MTK_VENDOR_ATTR_AMNT_CTRL_DUMP],
			       amnt_dump_policy, NULL);
	if (err)
		return err;

	if (!tb2[MTK_VENDOR_ATTR_AMNT_DUMP_INDEX])
		return -EINVAL;

	amnt_idx = nla_get_u8(tb2[MTK_VENDOR_ATTR_AMNT_DUMP_INDEX]);

	a = nla_nest_start(skb, MTK_VENDOR_ATTR_AMNT_CTRL_DUMP);
	b = nla_nest_start(skb, MTK_VENDOR_ATTR_AMNT_DUMP_RESULT);

	if (amnt_idx != 0xff) {
		len += mt7996_amnt_dump(phy, skb, amnt_idx, &attrtype);
	} else {
		for (i = 0; i < MT7996_AIR_MONITOR_MAX_ENTRY; i++)
			len += mt7996_amnt_dump(phy, skb, i, &attrtype);
	}

	nla_nest_end(skb, b);

	nla_put_u8(skb, MTK_VENDOR_ATTR_AMNT_DUMP_LEN, len);

	nla_nest_end(skb, a);

	return len + 1;
}

static int
mt7996_vendor_bss_color_ctrl_dump(struct wiphy *wiphy, struct wireless_dev *wdev,
				  struct sk_buff *skb, const void *data, int data_len,
				  unsigned long *storage)
{
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);
	struct ieee80211_bss_conf *bss_conf = &vif->bss_conf;
	int len = 0;

	if (*storage == 1)
		return -ENOENT;
	*storage = 1;

	if (nla_put_u64_64bit(skb, MTK_VENDOR_ATTR_AVAL_BSS_COLOR_BMP,
			      ~bss_conf->used_color_bitmap, NL80211_ATTR_PAD))
		return -ENOMEM;
	len += 1;

	return len;
}

static const struct wiphy_vendor_command mt7996_vendor_commands[] = {
	{
		.info = {
			.vendor_id = MTK_NL80211_VENDOR_ID,
			.subcmd = MTK_NL80211_VENDOR_SUBCMD_MU_CTRL,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = mt7996_vendor_mu_ctrl,
		.dumpit = mt7996_vendor_mu_ctrl_dump,
		.policy = mu_ctrl_policy,
		.maxattr = MTK_VENDOR_ATTR_MU_CTRL_MAX,
	},
	{
		.info = {
			.vendor_id = MTK_NL80211_VENDOR_ID,
			.subcmd = MTK_NL80211_VENDOR_SUBCMD_AMNT_CTRL,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = mt7966_vendor_amnt_ctrl,
		.dumpit = mt7966_vendor_amnt_ctrl_dump,
		.policy = amnt_ctrl_policy,
		.maxattr = MTK_VENDOR_ATTR_AMNT_CTRL_MAX,
	},
	{
		.info = {
			.vendor_id = MTK_NL80211_VENDOR_ID,
			.subcmd = MTK_NL80211_VENDOR_SUBCMD_BSS_COLOR_CTRL,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			WIPHY_VENDOR_CMD_NEED_RUNNING,
		.dumpit = mt7996_vendor_bss_color_ctrl_dump,
		.policy = bss_color_ctrl_policy,
		.maxattr = MTK_VENDOR_ATTR_BSS_COLOR_CTRL_MAX,
	},
};

void mt7996_vendor_register(struct mt7996_phy *phy)
{
	phy->mt76->hw->wiphy->vendor_commands = mt7996_vendor_commands;
	phy->mt76->hw->wiphy->n_vendor_commands = ARRAY_SIZE(mt7996_vendor_commands);

	spin_lock_init(&phy->amnt_lock);
}
