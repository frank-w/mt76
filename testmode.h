/* SPDX-License-Identifier: ISC */
/*
 * Copyright (C) 2020 Felix Fietkau <nbd@nbd.name>
 */
#ifndef __MT76_TESTMODE_H
#define __MT76_TESTMODE_H

#define MT76_TM_TIMEOUT			10
#define MT76_TM_EEPROM_BLOCK_SIZE	16

/**
 * enum mt76_testmode_attr - testmode attributes inside NL80211_ATTR_TESTDATA
 *
 * @MT76_TM_ATTR_UNSPEC: (invalid attribute)
 *
 * @MT76_TM_ATTR_RESET: reset parameters to default (flag)
 * @MT76_TM_ATTR_STATE: test state (u32), see &enum mt76_testmode_state
 *
 * @MT76_TM_ATTR_MTD_PART: mtd partition used for eeprom data (string)
 * @MT76_TM_ATTR_MTD_OFFSET: offset of eeprom data within the partition (u32)
 * @MT76_TM_ATTR_BAND_IDX: band idx of the chip (u8)
 *
 * @MT76_TM_ATTR_SKU_EN: config txpower sku is enabled or disabled in testmode (u8)
 * @MT76_TM_ATTR_TX_COUNT: configured number of frames to send when setting
 *	state to MT76_TM_STATE_TX_FRAMES (u32)
 * @MT76_TM_ATTR_TX_PENDING: pending frames during MT76_TM_STATE_TX_FRAMES (u32)
 * @MT76_TM_ATTR_TX_LENGTH: packet tx mpdu length (u32)
 * @MT76_TM_ATTR_TX_RATE_MODE: packet tx mode (u8, see &enum mt76_testmode_tx_mode)
 * @MT76_TM_ATTR_TX_RATE_NSS: packet tx number of spatial streams (u8)
 * @MT76_TM_ATTR_TX_RATE_IDX: packet tx rate/MCS index (u8)
 * @MT76_TM_ATTR_TX_RATE_SGI: packet tx use short guard interval (u8)
 * @MT76_TM_ATTR_TX_RATE_LDPC: packet tx enable LDPC (u8)
 * @MT76_TM_ATTR_TX_RATE_STBC: packet tx enable STBC (u8)
 * @MT76_TM_ATTR_TX_LTF: packet tx LTF, set 0 to 2 for 1x, 2x, and 4x LTF (u8)
 *
 * @MT76_TM_ATTR_TX_ANTENNA: tx antenna mask (u8)
 * @MT76_TM_ATTR_TX_POWER_CONTROL: enable tx power control (u8)
 * @MT76_TM_ATTR_TX_POWER: per-antenna tx power array (nested, u8 attrs)
 *
 * @MT76_TM_ATTR_FREQ_OFFSET: RF frequency offset (u32)
 *
 * @MT76_TM_ATTR_STATS: statistics (nested, see &enum mt76_testmode_stats_attr)
 *
 * @MT76_TM_ATTR_PRECAL: Pre-cal data (u8)
 * @MT76_TM_ATTR_PRECAL_INFO: group size, dpd size, dpd_info, transmit size,
 *                            eeprom cal indicator (u32),
 *                            dpd_info = [dpd_per_chan_size, chan_num_2g,
 *                                        chan_num_5g, chan_num_6g]
 * @MT76_TM_ATTR_TX_SPE_IDX: tx spatial extension index (u8)
 *
 * @MT76_TM_ATTR_TX_DUTY_CYCLE: packet tx duty cycle (u8)
 * @MT76_TM_ATTR_TX_IPG: tx inter-packet gap, in unit of us (u32)
 * @MT76_TM_ATTR_TX_TIME: packet transmission time, in unit of us (u32)
 *
 * @MT76_TM_ATTR_DRV_DATA: driver specific netlink attrs (nested)
 *
 * @MT76_TM_ATTR_MAC_ADDRS: array of nested MAC addresses (nested)
 *
 * @MT76_TM_ATTR_EEPROM_ACTION: eeprom setting actions
 *	(u8, see &enum mt76_testmode_eeprom_action)
 * @MT76_TM_ATTR_EEPROM_OFFSET: offset of eeprom data block for writing (u32)
 * @MT76_TM_ATTR_EEPROM_VAL: values for writing into a 16-byte data block
 *	(nested, u8 attrs)
 *
 * @MT76_TM_ATTR_CFG: config testmode rf feature (nested, see &mt76_testmode_cfg)
 * @MT76_TM_ATTR_TXBF_ACT: txbf setting actions (u8)
 * @MT76_TM_ATTR_TXBF_PARAM: txbf parameters (nested)
 *
 * @MT76_TM_ATTR_OFF_CH_SCAN_CH: config the channel of background chain (ZWDFS) (u8)
 * @MT76_TM_ATTR_OFF_CH_SCAN_CENTER_CH: config the center channel of background chain (ZWDFS) (u8)
 * @MT76_TM_ATTR_OFF_CH_SCAN_BW: config the bandwidth of background chain (ZWDFS) (u8)
 * @MT76_TM_ATTR_OFF_CH_SCAN_PATH: config the tx path of background chain (ZWDFS) (u8)
 *
 * @MT76_TM_ATTR_IPI_THRESHOLD: config the IPI index you want to read (u8)
 * @MT76_TM_ATTR_IPI_PERIOD: config the time period for reading
 *			     the histogram of specific IPI index (u8)
 * @MT76_TM_ATTR_IPI_ANTENNA_INDEX: config the antenna index for reading
 *				    the histogram of specific IPI index (u8)
 * @MT76_TM_ATTR_IPI_RESET: Reset the IPI counter
 *
 */
enum mt76_testmode_attr {
	MT76_TM_ATTR_UNSPEC,

	MT76_TM_ATTR_RESET,
	MT76_TM_ATTR_STATE,

	MT76_TM_ATTR_MTD_PART,
	MT76_TM_ATTR_MTD_OFFSET,
	MT76_TM_ATTR_BAND_IDX,

	MT76_TM_ATTR_SKU_EN,
	MT76_TM_ATTR_TX_COUNT,
	MT76_TM_ATTR_TX_LENGTH,
	MT76_TM_ATTR_TX_RATE_MODE,
	MT76_TM_ATTR_TX_RATE_NSS,
	MT76_TM_ATTR_TX_RATE_IDX,
	MT76_TM_ATTR_TX_RATE_SGI,
	MT76_TM_ATTR_TX_RATE_LDPC,
	MT76_TM_ATTR_TX_RATE_STBC,
	MT76_TM_ATTR_TX_LTF,

	MT76_TM_ATTR_TX_ANTENNA,
	MT76_TM_ATTR_TX_POWER_CONTROL,
	MT76_TM_ATTR_TX_POWER,

	MT76_TM_ATTR_FREQ_OFFSET,

	MT76_TM_ATTR_STATS,
	MT76_TM_ATTR_PRECAL,
	MT76_TM_ATTR_PRECAL_INFO,

	MT76_TM_ATTR_TX_SPE_IDX,

	MT76_TM_ATTR_TX_DUTY_CYCLE,
	MT76_TM_ATTR_TX_IPG,
	MT76_TM_ATTR_TX_TIME,

	MT76_TM_ATTR_DRV_DATA,

	MT76_TM_ATTR_MAC_ADDRS,
	MT76_TM_ATTR_AID,
	MT76_TM_ATTR_RU_ALLOC,
	MT76_TM_ATTR_RU_IDX,

	MT76_TM_ATTR_EEPROM_ACTION,
	MT76_TM_ATTR_EEPROM_OFFSET,
	MT76_TM_ATTR_EEPROM_VAL,

	MT76_TM_ATTR_CFG,
	MT76_TM_ATTR_TXBF_ACT,
	MT76_TM_ATTR_TXBF_PARAM,

	MT76_TM_ATTR_OFF_CH_SCAN_CH,
	MT76_TM_ATTR_OFF_CH_SCAN_CENTER_CH,
	MT76_TM_ATTR_OFF_CH_SCAN_BW,
	MT76_TM_ATTR_OFF_CH_SCAN_PATH,

	MT76_TM_ATTR_IPI_THRESHOLD,
	MT76_TM_ATTR_IPI_PERIOD,
	MT76_TM_ATTR_IPI_ANTENNA_INDEX,
	MT76_TM_ATTR_IPI_RESET,

	/* keep last */
	NUM_MT76_TM_ATTRS,
	MT76_TM_ATTR_MAX = NUM_MT76_TM_ATTRS - 1,
};

/**
 * enum mt76_testmode_state - statistics attributes
 *
 * @MT76_TM_STATS_ATTR_TX_PENDING: pending tx frames (u32)
 * @MT76_TM_STATS_ATTR_TX_QUEUED: queued tx frames (u32)
 * @MT76_TM_STATS_ATTR_TX_QUEUED: completed tx frames (u32)
 *
 * @MT76_TM_STATS_ATTR_RX_PACKETS: number of rx packets (u64)
 * @MT76_TM_STATS_ATTR_RX_FCS_ERROR: number of rx packets with FCS error (u64)
 * @MT76_TM_STATS_ATTR_LAST_RX: information about the last received packet
 *	see &enum mt76_testmode_rx_attr
 * @MT76_TM_STATS_ATTR_RX_LEN_MISMATCH: number of rx packets with length
 *	mismatch error (u64)
 */
enum mt76_testmode_stats_attr {
	MT76_TM_STATS_ATTR_UNSPEC,
	MT76_TM_STATS_ATTR_PAD,

	MT76_TM_STATS_ATTR_TX_PENDING,
	MT76_TM_STATS_ATTR_TX_QUEUED,
	MT76_TM_STATS_ATTR_TX_DONE,

	MT76_TM_STATS_ATTR_RX_PACKETS,
	MT76_TM_STATS_ATTR_RX_FCS_ERROR,
	MT76_TM_STATS_ATTR_LAST_RX,
	MT76_TM_STATS_ATTR_RX_LEN_MISMATCH,

	/* keep last */
	NUM_MT76_TM_STATS_ATTRS,
	MT76_TM_STATS_ATTR_MAX = NUM_MT76_TM_STATS_ATTRS - 1,
};


/**
 * enum mt76_testmode_rx_attr - packet rx information
 *
 * @MT76_TM_RX_ATTR_FREQ_OFFSET: frequency offset (s32)
 * @MT76_TM_RX_ATTR_RCPI: received channel power indicator (array, u8)
 * @MT76_TM_RX_ATTR_RSSI: received signal strength indicator (array, s8)
 * @MT76_TM_RX_ATTR_IB_RSSI: internal inband RSSI (array, s8)
 * @MT76_TM_RX_ATTR_WB_RSSI: internal wideband RSSI (array, s8)
 * @MT76_TM_RX_ATTR_SNR: signal-to-noise ratio (u8)
 */
enum mt76_testmode_rx_attr {
	MT76_TM_RX_ATTR_UNSPEC,

	MT76_TM_RX_ATTR_FREQ_OFFSET,
	MT76_TM_RX_ATTR_RCPI,
	MT76_TM_RX_ATTR_RSSI,
	MT76_TM_RX_ATTR_IB_RSSI,
	MT76_TM_RX_ATTR_WB_RSSI,
	MT76_TM_RX_ATTR_SNR,

	/* keep last */
	NUM_MT76_TM_RX_ATTRS,
	MT76_TM_RX_ATTR_MAX = NUM_MT76_TM_RX_ATTRS - 1,
};

/**
 * enum mt76_testmode_state - phy test state
 *
 * @MT76_TM_STATE_OFF: test mode disabled (normal operation)
 * @MT76_TM_STATE_IDLE: test mode enabled, but idle
 * @MT76_TM_STATE_TX_FRAMES: send a fixed number of test frames
 * @MT76_TM_STATE_RX_FRAMES: receive packets and keep statistics
 * @MT76_TM_STATE_TX_CONT: waveform tx without time gap
 * @MT76_TM_STATE_ON: test mode enabled used in offload firmware
 */
enum mt76_testmode_state {
	MT76_TM_STATE_OFF,
	MT76_TM_STATE_IDLE,
	MT76_TM_STATE_TX_FRAMES,
	MT76_TM_STATE_RX_FRAMES,
	MT76_TM_STATE_TX_CONT,
	MT76_TM_STATE_ON,

	/* keep last */
	NUM_MT76_TM_STATES,
	MT76_TM_STATE_MAX = NUM_MT76_TM_STATES - 1,
};

/**
 * enum mt76_testmode_tx_mode - packet tx phy mode
 *
 * @MT76_TM_TX_MODE_CCK: legacy CCK mode
 * @MT76_TM_TX_MODE_OFDM: legacy OFDM mode
 * @MT76_TM_TX_MODE_HT: 802.11n MCS
 * @MT76_TM_TX_MODE_VHT: 802.11ac MCS
 * @MT76_TM_TX_MODE_HE_SU: 802.11ax single-user MIMO
 * @MT76_TM_TX_MODE_HE_EXT_SU: 802.11ax extended-range SU
 * @MT76_TM_TX_MODE_HE_TB: 802.11ax trigger-based
 * @MT76_TM_TX_MODE_HE_MU: 802.11ax multi-user MIMO
 * @MT76_TM_TX_MODE_EHT_SU: 802.11be single-user MIMO
 * @MT76_TM_TX_MODE_EHT_TRIG: 802.11be trigger-based
 * @MT76_TM_TX_MODE_EHT_MU: 802.11be multi-user MIMO
 */
enum mt76_testmode_tx_mode {
	MT76_TM_TX_MODE_CCK,
	MT76_TM_TX_MODE_OFDM,
	MT76_TM_TX_MODE_HT,
	MT76_TM_TX_MODE_VHT,
	MT76_TM_TX_MODE_HE_SU,
	MT76_TM_TX_MODE_HE_EXT_SU,
	MT76_TM_TX_MODE_HE_TB,
	MT76_TM_TX_MODE_HE_MU,
	MT76_TM_TX_MODE_EHT_SU,
	MT76_TM_TX_MODE_EHT_TRIG,
	MT76_TM_TX_MODE_EHT_MU,

	/* keep last */
	NUM_MT76_TM_TX_MODES,
	MT76_TM_TX_MODE_MAX = NUM_MT76_TM_TX_MODES - 1,
};

/**
 * enum mt76_testmode_eeprom_action - eeprom setting actions
 *
 * @MT76_TM_EEPROM_ACTION_UPDATE_DATA: update rf values to specific
 *	eeprom data block
 * @MT76_TM_EEPROM_ACTION_UPDATE_BUFFER_MODE: send updated eeprom data to fw
 * @MT76_TM_EEPROM_ACTION_WRITE_TO_EFUSE: write eeprom data back to efuse
 */
enum mt76_testmode_eeprom_action {
	MT76_TM_EEPROM_ACTION_UPDATE_DATA,
	MT76_TM_EEPROM_ACTION_UPDATE_BUFFER_MODE,
	MT76_TM_EEPROM_ACTION_WRITE_TO_EFUSE,

	/* keep last */
	NUM_MT76_TM_EEPROM_ACTION,
	MT76_TM_EEPROM_ACTION_MAX = NUM_MT76_TM_EEPROM_ACTION - 1,
};

extern const struct nla_policy mt76_tm_policy[NUM_MT76_TM_ATTRS];

#endif
