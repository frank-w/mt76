// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include <linux/firmware.h>
#include "mt7996.h"
#include "eeprom.h"
#include <linux/moduleparam.h>

static bool testmode_enable;
module_param(testmode_enable, bool, 0644);
MODULE_PARM_DESC(testmode_enable, "Enable testmode");

const struct ieee80211_channel dpd_2g_ch_list_bw20[] = {
	CHAN2G(3, 2422),
	CHAN2G(7, 2442),
	CHAN2G(11, 2462)
};

const struct ieee80211_channel dpd_5g_ch_list_bw160[] = {
	CHAN5G(50, 5250),
	CHAN5G(114, 5570),
	CHAN5G(163, 5815)
};

const struct ieee80211_channel dpd_6g_ch_list_bw160[] = {
	CHAN6G(15, 6025),
	CHAN6G(47, 6185),
	CHAN6G(79, 6345),
	CHAN6G(111, 6505),
	CHAN6G(143, 6665),
	CHAN6G(175, 6825),
	CHAN6G(207, 6985)
};

const struct ieee80211_channel dpd_6g_ch_list_bw320[] = {
	CHAN6G(31, 6105),
	CHAN6G(63, 6265),
	CHAN6G(95, 6425),
	CHAN6G(127, 6585),
	CHAN6G(159, 6745),
	CHAN6G(191, 6905)
};

const u32 dpd_2g_bw20_ch_num = ARRAY_SIZE(dpd_2g_ch_list_bw20);
const u32 dpd_5g_bw160_ch_num = ARRAY_SIZE(dpd_5g_ch_list_bw160);
const u32 dpd_6g_bw160_ch_num = ARRAY_SIZE(dpd_6g_ch_list_bw160);
const u32 dpd_6g_bw320_ch_num = ARRAY_SIZE(dpd_6g_ch_list_bw320);

static int mt7996_check_eeprom(struct mt7996_dev *dev)
{
#define FEM_INT				0
#define FEM_EXT				3
	u8 *eeprom = dev->mt76.eeprom.data;
	u8 i, fem[__MT_MAX_BAND], fem_type;
	u16 val = get_unaligned_le16(eeprom);

	for (i = 0; i < __MT_MAX_BAND; i++)
		fem[i] = eeprom[MT_EE_WIFI_CONF + 6 + i] & MT_EE_WIFI_PA_LNA_CONFIG;

	switch (val) {
	case 0x7990:
		return is_mt7996(&dev->mt76) ? 0 : -EINVAL;
	case 0x7992:
		if (dev->fem_type == MT7996_FEM_UNSET)
			return is_mt7992(&dev->mt76) ? 0 : -EINVAL;

		if (fem[0] == FEM_EXT && fem[1] == FEM_EXT)
			fem_type = MT7996_FEM_EXT;
		else if (fem[0] == FEM_INT && fem[1] == FEM_INT)
			fem_type = MT7996_FEM_INT;
		else if (fem[0] == FEM_INT && fem[1] == FEM_EXT)
			fem_type = MT7996_FEM_MIX;
		else
			return -EINVAL;

		return (is_mt7992(&dev->mt76) ? 0 : -EINVAL) |
		       (dev->fem_type == fem_type ? 0 : -EINVAL);
	default:
		return -EINVAL;
	}
}

static char *mt7996_eeprom_name(struct mt7996_dev *dev)
{
	if (dev->testmode_enable)
		return MT7996_EEPROM_DEFAULT_TM;

	switch (mt76_chip(&dev->mt76)) {
	case 0x7990:
		if (dev->chip_sku == MT7996_SKU_404)
			return MT7996_EEPROM_DEFAULT_404;
		return MT7996_EEPROM_DEFAULT;
	case 0x7992:
		if (dev->chip_sku == MT7992_SKU_23) {
			if (dev->fem_type == MT7996_FEM_INT)
				return MT7992_EEPROM_DEFAULT_23;
			return MT7992_EEPROM_DEFAULT_23_EXT;
		} else if (dev->chip_sku == MT7992_SKU_44) {
			if (dev->fem_type == MT7996_FEM_INT)
				return MT7992_EEPROM_DEFAULT;
			else if (dev->fem_type == MT7996_FEM_MIX)
				return MT7992_EEPROM_DEFAULT_MIX;
			return MT7992_EEPROM_DEFAULT_EXT;
		}
		return MT7992_EEPROM_DEFAULT_24;
	default:
		return MT7996_EEPROM_DEFAULT;
	}
}

int
mt7996_get_dpd_per_band_size(struct mt7996_dev *dev, enum nl80211_band band)
{
	/* handle different sku */
	static const u8 band_to_idx[] = {
		[NL80211_BAND_2GHZ] = MT_BAND0,
		[NL80211_BAND_5GHZ] = MT_BAND1,
		[NL80211_BAND_6GHZ] = MT_BAND2,
	};
	struct mt7996_phy *phy = __mt7996_phy(dev, band_to_idx[band]);
	struct mt76_phy *mphy;
	int dpd_size;

	if (!phy)
		return 0;

	mphy = phy->mt76;

	if (band == NL80211_BAND_2GHZ)
		dpd_size = dpd_2g_bw20_ch_num * DPD_PER_CH_BW20_SIZE;
	else if (band == NL80211_BAND_5GHZ)
		dpd_size = mphy->sband_5g.sband.n_channels * DPD_PER_CH_BW20_SIZE +
			   dpd_5g_bw160_ch_num * DPD_PER_CH_GT_BW20_SIZE;
	else
		dpd_size = mphy->sband_6g.sband.n_channels * DPD_PER_CH_BW20_SIZE +
			   (dpd_6g_bw160_ch_num + dpd_6g_bw320_ch_num) * DPD_PER_CH_GT_BW20_SIZE;

	return dpd_size;
}

static int
mt7996_eeprom_load_default(struct mt7996_dev *dev)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	const struct firmware *fw = NULL;
	int ret;

	ret = request_firmware(&fw, mt7996_eeprom_name(dev), dev->mt76.dev);
	if (ret)
		return ret;

	if (!fw || !fw->data) {
		dev_err(dev->mt76.dev, "Invalid default bin\n");
		ret = -EINVAL;
		goto out;
	}

	memcpy(eeprom, fw->data, MT7996_EEPROM_SIZE);
	dev->flash_mode = true;

out:
	release_firmware(fw);

	return ret;
}

int mt7996_eeprom_check_fw_mode(struct mt7996_dev *dev)
{
	u8 *eeprom;
	int ret;

	/* load eeprom in flash or bin file mode to determine fw mode */
	ret = mt76_eeprom_init(&dev->mt76, MT7996_EEPROM_SIZE);
	if (ret < 0)
		return ret;

	if (ret) {
		dev->flash_mode = true;
		eeprom = dev->mt76.eeprom.data;
		/* testmode enable priority: eeprom field > module parameter */
		dev->testmode_enable = !mt7996_check_eeprom(dev) ? eeprom[MT_EE_TESTMODE_EN] :
								   testmode_enable;
	}

	return ret;
}

static int mt7996_eeprom_load(struct mt7996_dev *dev)
{
	int ret;
	u8 free_block_num;
	u32 block_num, i;
	u32 eeprom_blk_size = MT7996_EEPROM_BLOCK_SIZE;

	/* flash or bin file mode eeprom is loaded before mcu init */
	if (!dev->flash_mode) {
		ret = mt7996_mcu_get_eeprom_free_block(dev, &free_block_num);
		if (ret < 0)
			return ret;

		/* efuse info isn't enough */
		if (free_block_num >= 59)
			return -EINVAL;

		/* read eeprom data from efuse */
		block_num = DIV_ROUND_UP(MT7996_EEPROM_SIZE, eeprom_blk_size);
		for (i = 0; i < block_num; i++) {
			ret = mt7996_mcu_get_eeprom(dev, i * eeprom_blk_size, NULL);
			if (ret < 0)
				return ret;
		}
	}

	return mt7996_check_eeprom(dev);
}

static int mt7996_eeprom_parse_efuse_hw_cap(struct mt7996_dev *dev)
{
#define MODE_HE_ONLY		BIT(0)
#define WTBL_SIZE_GROUP		GENMASK(31, 28)
	u32 cap = 0;
	int ret;

	ret = mt7996_mcu_get_chip_config(dev, &cap);
	if (ret)
		return ret;

	cap = 0x4b249248;	/* internal hardcode */
	if (cap) {
		dev->has_eht = !(cap & MODE_HE_ONLY);
		dev->wtbl_size_group = u32_get_bits(cap, WTBL_SIZE_GROUP);
	}

	if (dev->wtbl_size_group < 2 || dev->wtbl_size_group > 4 ||
	    is_mt7992(&dev->mt76))
		dev->wtbl_size_group = 2; /* set default */

	return 0;
}

static int mt7996_eeprom_parse_band_config(struct mt7996_phy *phy)
{
	u8 *eeprom = phy->dev->mt76.eeprom.data;
	u32 val = eeprom[MT_EE_WIFI_CONF];
	int ret = 0;

	switch (phy->mt76->band_idx) {
	case MT_BAND1:
		val = FIELD_GET(MT_EE_WIFI_CONF1_BAND_SEL, val);
		break;
	case MT_BAND2:
		val = eeprom[MT_EE_WIFI_CONF + 1];
		val = FIELD_GET(MT_EE_WIFI_CONF2_BAND_SEL, val);
		break;
	default:
		val = FIELD_GET(MT_EE_WIFI_CONF0_BAND_SEL, val);
		break;
	}

	switch (val) {
	case MT_EE_BAND_SEL_2GHZ:
		phy->mt76->cap.has_2ghz = true;
		break;
	case MT_EE_BAND_SEL_5GHZ:
		phy->mt76->cap.has_5ghz = true;
		break;
	case MT_EE_BAND_SEL_6GHZ:
		phy->mt76->cap.has_6ghz = true;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int mt7996_eeprom_parse_hw_cap(struct mt7996_dev *dev, struct mt7996_phy *phy)
{
	u8 path, rx_path, nss, band_idx = phy->mt76->band_idx;
	u8 *eeprom = dev->mt76.eeprom.data;
	struct mt76_phy *mphy = phy->mt76;
	int max_path = 5, max_nss = 4;
	int ret;

	switch (band_idx) {
	case MT_BAND1:
		path = FIELD_GET(MT_EE_WIFI_CONF2_TX_PATH_BAND1,
				 eeprom[MT_EE_WIFI_CONF + 2]);
		rx_path = FIELD_GET(MT_EE_WIFI_CONF3_RX_PATH_BAND1,
				    eeprom[MT_EE_WIFI_CONF + 3]);
		nss = FIELD_GET(MT_EE_WIFI_CONF5_STREAM_NUM_BAND1,
				eeprom[MT_EE_WIFI_CONF + 5]);
		break;
	case MT_BAND2:
		path = FIELD_GET(MT_EE_WIFI_CONF2_TX_PATH_BAND2,
				 eeprom[MT_EE_WIFI_CONF + 2]);
		rx_path = FIELD_GET(MT_EE_WIFI_CONF4_RX_PATH_BAND2,
				    eeprom[MT_EE_WIFI_CONF + 4]);
		nss = FIELD_GET(MT_EE_WIFI_CONF5_STREAM_NUM_BAND2,
				eeprom[MT_EE_WIFI_CONF + 5]);
		break;
	default:
		path = FIELD_GET(MT_EE_WIFI_CONF1_TX_PATH_BAND0,
				 eeprom[MT_EE_WIFI_CONF + 1]);
		rx_path = FIELD_GET(MT_EE_WIFI_CONF3_RX_PATH_BAND0,
				    eeprom[MT_EE_WIFI_CONF + 3]);
		nss = FIELD_GET(MT_EE_WIFI_CONF4_STREAM_NUM_BAND0,
				eeprom[MT_EE_WIFI_CONF + 4]);
		break;
	}

	if (!path || path > max_path)
		path = max_path;

	if (!nss || nss > max_nss)
		nss = max_nss;

	nss = min_t(u8, nss, path);

	if (path != rx_path)
		phy->has_aux_rx = true;

	mphy->antenna_mask = BIT(nss) - 1;
	mphy->chainmask = (BIT(path) - 1) << dev->chainshift[band_idx];
	dev->chainmask |= mphy->chainmask;
	if (band_idx < MT_BAND2)
		dev->chainshift[band_idx + 1] = dev->chainshift[band_idx] +
						hweight16(mphy->chainmask);

	ret = mt7996_eeprom_parse_efuse_hw_cap(dev);
	if (ret)
		return ret;

	return mt7996_eeprom_parse_band_config(phy);
}

int mt7996_eeprom_init(struct mt7996_dev *dev)
{
	int ret;

	ret = mt7996_get_chip_sku(dev);
	if (ret)
		return ret;

	ret = mt7996_eeprom_load(dev);
	if (ret < 0) {
		if (ret != -EINVAL)
			return ret;

		dev_warn(dev->mt76.dev, "eeprom load fail, use default bin\n");
		ret = mt7996_eeprom_load_default(dev);
		if (ret)
			return ret;
	}

	ret = mt7996_eeprom_parse_hw_cap(dev, &dev->phy);
	if (ret < 0)
		return ret;

	memcpy(dev->mphy.macaddr, dev->mt76.eeprom.data + MT_EE_MAC_ADDR, ETH_ALEN);
	mt76_eeprom_override(&dev->mphy);

	return 0;
}

int mt7996_eeprom_get_target_power(struct mt7996_dev *dev,
				   struct ieee80211_channel *chan)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	int target_power;

	if (chan->band == NL80211_BAND_5GHZ)
		target_power = eeprom[MT_EE_TX0_POWER_5G +
				      mt7996_get_channel_group_5g(chan->hw_value)];
	else if (chan->band == NL80211_BAND_6GHZ)
		target_power = eeprom[MT_EE_TX0_POWER_6G +
				      mt7996_get_channel_group_6g(chan->hw_value)];
	else
		target_power = eeprom[MT_EE_TX0_POWER_2G];

	return target_power;
}

s8 mt7996_eeprom_get_power_delta(struct mt7996_dev *dev, int band)
{
	u8 *eeprom = dev->mt76.eeprom.data;
	u32 val;
	s8 delta;

	if (band == NL80211_BAND_5GHZ)
		val = eeprom[MT_EE_RATE_DELTA_5G];
	else if (band == NL80211_BAND_6GHZ)
		val = eeprom[MT_EE_RATE_DELTA_6G];
	else
		val = eeprom[MT_EE_RATE_DELTA_2G];

	if (!(val & MT_EE_RATE_DELTA_EN))
		return 0;

	delta = FIELD_GET(MT_EE_RATE_DELTA_MASK, val);

	return val & MT_EE_RATE_DELTA_SIGN ? delta : -delta;
}
