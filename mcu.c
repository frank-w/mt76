// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2019 Lorenzo Bianconi <lorenzo.bianconi83@gmail.com>
 */

#include "mt76.h"
#include "mt76_connac.h"
#include <linux/moduleparam.h>

struct sk_buff *
__mt76_mcu_msg_alloc(struct mt76_dev *dev, const void *data,
		     int len, int data_len, gfp_t gfp)
{
	const struct mt76_mcu_ops *ops = dev->mcu_ops;
	struct sk_buff *skb;

	len = max_t(int, len, data_len);
	len = ops->headroom + len + ops->tailroom;

	skb = alloc_skb(len, gfp);
	if (!skb)
		return NULL;

	memset(skb->head, 0, len);
	skb_reserve(skb, ops->headroom);

	if (data && data_len)
		skb_put_data(skb, data, data_len);

	return skb;
}
EXPORT_SYMBOL_GPL(__mt76_mcu_msg_alloc);

struct sk_buff *mt76_mcu_get_response(struct mt76_dev *dev,
				      unsigned long expires)
{
	unsigned long timeout;

	if (!time_is_after_jiffies(expires))
		return NULL;

	timeout = expires - jiffies;
	wait_event_timeout(dev->mcu.wait,
			   (!skb_queue_empty(&dev->mcu.res_q) ||
			    test_bit(MT76_MCU_RESET, &dev->phy.state)),
			   timeout);
	return skb_dequeue(&dev->mcu.res_q);
}
EXPORT_SYMBOL_GPL(mt76_mcu_get_response);

void mt76_mcu_rx_event(struct mt76_dev *dev, struct sk_buff *skb)
{
	skb_queue_tail(&dev->mcu.res_q, skb);
	wake_up(&dev->mcu.wait);
}
EXPORT_SYMBOL_GPL(mt76_mcu_rx_event);

int mt76_mcu_send_and_get_msg(struct mt76_dev *dev, int cmd, const void *data,
			      int len, bool wait_resp, struct sk_buff **ret_skb)
{
	struct sk_buff *skb;

	if (dev->mcu_ops->mcu_send_msg)
		return dev->mcu_ops->mcu_send_msg(dev, cmd, data, len, wait_resp);

	skb = mt76_mcu_msg_alloc(dev, data, len);
	if (!skb)
		return -ENOMEM;

	return mt76_mcu_skb_send_and_get_msg(dev, skb, cmd, wait_resp, ret_skb);
}
EXPORT_SYMBOL_GPL(mt76_mcu_send_and_get_msg);

int mt76_mcu_skb_send_and_get_msg(struct mt76_dev *dev, struct sk_buff *skb,
				  int cmd, bool wait_resp,
				  struct sk_buff **ret_skb)
{
#define MT76_MSG_MAX_RETRY_CNT 3
	unsigned long expires;
	int ret, seq, retry_cnt;
	struct sk_buff *skb_tmp;
	bool retry = wait_resp && is_mt7996(dev);

	if (ret_skb)
		*ret_skb = NULL;

	mutex_lock(&dev->mcu.mutex);
	retry_cnt = retry ? MT76_MSG_MAX_RETRY_CNT : 1;
	while (retry_cnt) {
		skb_tmp = mt76_mcu_msg_alloc(dev, skb->data, skb->len);
		if (!skb_tmp)
			goto out;

		if (retry && retry_cnt < MT76_MSG_MAX_RETRY_CNT) {
			if (test_bit(MT76_MCU_RESET, &dev->phy.state))
				usleep_range(200000, 500000);
			dev_err(dev->dev, "send message %08x timeout, try again.\n", cmd);
		}

		ret = dev->mcu_ops->mcu_skb_send_msg(dev, skb_tmp, cmd, &seq);
		if (ret < 0 && ret != -EAGAIN)
			goto out;

		if (!wait_resp) {
			ret = 0;
			goto out;
		}

		expires = jiffies + dev->mcu.timeout;

		do {
			skb_tmp = mt76_mcu_get_response(dev, expires);
			ret = dev->mcu_ops->mcu_parse_response(dev, cmd, skb_tmp, seq);
			if (ret == -ETIMEDOUT)
				break;

			if (!ret && ret_skb)
				*ret_skb = skb_tmp;
			else
				dev_kfree_skb(skb_tmp);

			if (ret != -EAGAIN)
				goto out;
		} while (ret == -EAGAIN);

		retry_cnt--;
	}

out:
	dev_kfree_skb(skb);
	mutex_unlock(&dev->mcu.mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(mt76_mcu_skb_send_and_get_msg);

int __mt76_mcu_send_firmware(struct mt76_dev *dev, int cmd, const void *data,
			     int len, int max_len)
{
	int err, cur_len;

	while (len > 0) {
		cur_len = min_t(int, max_len, len);

		err = mt76_mcu_send_msg(dev, cmd, data, cur_len, false);
		if (err)
			return err;

		data += cur_len;
		len -= cur_len;

		if (dev->queue_ops->tx_cleanup)
			dev->queue_ops->tx_cleanup(dev,
						   dev->q_mcu[MT_MCUQ_FWDL],
						   false);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__mt76_mcu_send_firmware);
