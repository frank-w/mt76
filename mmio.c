// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
 */

#include "mt76.h"
#include "dma.h"
#include "trace.h"

static u32 mt76_mmio_rr(struct mt76_dev *dev, u32 offset)
{
	u32 val;

	val = readl(dev->mmio.regs + offset);
	trace_reg_rr(dev, offset, val);

	return val;
}

static void mt76_mmio_wr(struct mt76_dev *dev, u32 offset, u32 val)
{
	trace_reg_wr(dev, offset, val);
	writel(val, dev->mmio.regs + offset);
}

static u32 mt76_mmio_rmw(struct mt76_dev *dev, u32 offset, u32 mask, u32 val)
{
	val |= mt76_mmio_rr(dev, offset) & ~mask;
	mt76_mmio_wr(dev, offset, val);
	return val;
}

static void mt76_mmio_write_copy(struct mt76_dev *dev, u32 offset,
				 const void *data, int len)
{
	__iowrite32_copy(dev->mmio.regs + offset, data, DIV_ROUND_UP(len, 4));
}

static void mt76_mmio_read_copy(struct mt76_dev *dev, u32 offset,
				void *data, int len)
{
	__ioread32_copy(data, dev->mmio.regs + offset, DIV_ROUND_UP(len, 4));
}

static int mt76_mmio_wr_rp(struct mt76_dev *dev, u32 base,
			   const struct mt76_reg_pair *data, int len)
{
	while (len > 0) {
		mt76_mmio_wr(dev, data->reg, data->value);
		data++;
		len--;
	}

	return 0;
}

static int mt76_mmio_rd_rp(struct mt76_dev *dev, u32 base,
			   struct mt76_reg_pair *data, int len)
{
	while (len > 0) {
		data->value = mt76_mmio_rr(dev, data->reg);
		data++;
		len--;
	}

	return 0;
}

void mt76_set_irq_mask(struct mt76_dev *dev, u32 addr,
		       u32 clear, u32 set)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->mmio.irq_lock, flags);
	dev->mmio.irqmask &= ~clear;
	dev->mmio.irqmask |= set;
	if (addr) {
		if (mtk_wed_device_active(&dev->mmio.wed))
			mtk_wed_device_irq_set_mask(&dev->mmio.wed,
						    dev->mmio.irqmask);
		else
			mt76_mmio_wr(dev, addr, dev->mmio.irqmask);
	}
	spin_unlock_irqrestore(&dev->mmio.irq_lock, flags);
}
EXPORT_SYMBOL_GPL(mt76_set_irq_mask);

#ifdef CONFIG_NET_MEDIATEK_SOC_WED
void mt76_mmio_wed_release_rx_buf(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);
	struct page *page;
	int i;

	for (i = 0; i < dev->rx_token_size; i++) {
		struct mt76_rxwi_cache *r;

		r = mt76_rx_token_release(dev, i);
		if (!r || !r->ptr)
			continue;

		dma_unmap_single(dev->dma_dev, r->dma_addr,
				 wed->wlan.rx_size, DMA_FROM_DEVICE);
		skb_free_frag(r->ptr);
		r->ptr = NULL;

		mt76_put_rxwi(dev, r);
	}

	mt76_free_pending_rxwi(dev);

	mt76_for_each_q_rx(dev, i) {
		struct mt76_queue *q = &dev->q_rx[i];

		if (mt76_queue_is_wed_rx(q)) {
			if (!q->rx_page.va)
				continue;

			page = virt_to_page(q->rx_page.va);
			__page_frag_cache_drain(page, q->rx_page.pagecnt_bias);
			memset(&q->rx_page, 0, sizeof(q->rx_page));
		}
	}

	if (!wed->rx_buf_ring.rx_page.va)
		return;

	page = virt_to_page(wed->rx_buf_ring.rx_page.va);
	__page_frag_cache_drain(page, wed->rx_buf_ring.rx_page.pagecnt_bias);
	memset(&wed->rx_buf_ring.rx_page, 0, sizeof(wed->rx_buf_ring.rx_page));
}
EXPORT_SYMBOL_GPL(mt76_mmio_wed_release_rx_buf);

u32 mt76_mmio_wed_init_rx_buf(struct mtk_wed_device *wed, int size)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);
	struct mtk_rxbm_desc *desc = wed->rx_buf_ring.desc;
	u32 length;
	int i;

	length = SKB_DATA_ALIGN(NET_SKB_PAD + wed->wlan.rx_size +
				sizeof(struct skb_shared_info));

	for (i = 0; i < size; i++) {
		struct mt76_rxwi_cache *r = mt76_get_rxwi(dev);
		dma_addr_t addr;
		int token;
		void *ptr;

		if (!r)
			goto unmap;

		ptr = page_frag_alloc(&wed->rx_buf_ring.rx_page, length,
				      GFP_ATOMIC | GFP_DMA32);
		if (!ptr) {
			mt76_put_rxwi(dev, r);
 			goto unmap;
		}

		addr = dma_map_single(dev->dma_dev, ptr,
					  wed->wlan.rx_size,
					  DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(dev->dev, addr))) {
			skb_free_frag(ptr);
			mt76_put_rxwi(dev, r);
			goto unmap;
		}

		desc->buf0 = cpu_to_le32(addr);
		token = mt76_rx_token_consume(dev, ptr, r, addr);
		if (token < 0) {
			dma_unmap_single(dev->dma_dev, addr,
					 wed->wlan.rx_size, DMA_TO_DEVICE);
			skb_free_frag(ptr);
			mt76_put_rxwi(dev, r);
			goto unmap;
		}

		token = FIELD_PREP(MT_DMA_CTL_TOKEN, token);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		token |= FIELD_PREP(MT_DMA_CTL_SDP0_H, addr >> 32);
#endif
		desc->token |= cpu_to_le32(token);
		desc++;
	}

	return 0;

unmap:
	mt76_mmio_wed_release_rx_buf(wed);

	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(mt76_mmio_wed_init_rx_buf);

int mt76_mmio_wed_offload_enable(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);

	spin_lock_bh(&dev->token_lock);
	dev->token_size = wed->wlan.token_start;
	spin_unlock_bh(&dev->token_lock);

	return !wait_event_timeout(dev->tx_wait, !dev->wed_token_count, HZ);
}
EXPORT_SYMBOL_GPL(mt76_mmio_wed_offload_enable);

void mt76_mmio_wed_offload_disable(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);

	spin_lock_bh(&dev->token_lock);
	dev->token_size = dev->drv->token_size;
	spin_unlock_bh(&dev->token_lock);
}
EXPORT_SYMBOL_GPL(mt76_mmio_wed_offload_disable);

void mt76_mmio_wed_reset_complete(struct mtk_wed_device *wed)
{
	struct mt76_dev *dev = container_of(wed, struct mt76_dev, mmio.wed);

	complete(&dev->mmio.wed_reset_complete);
}
EXPORT_SYMBOL_GPL(mt76_mmio_wed_reset_complete);
#endif /*CONFIG_NET_MEDIATEK_SOC_WED */

void mt76_mmio_init(struct mt76_dev *dev, void __iomem *regs)
{
	static const struct mt76_bus_ops mt76_mmio_ops = {
		.rr = mt76_mmio_rr,
		.rmw = mt76_mmio_rmw,
		.wr = mt76_mmio_wr,
		.write_copy = mt76_mmio_write_copy,
		.read_copy = mt76_mmio_read_copy,
		.wr_rp = mt76_mmio_wr_rp,
		.rd_rp = mt76_mmio_rd_rp,
		.type = MT76_BUS_MMIO,
	};

	dev->bus = &mt76_mmio_ops;
	dev->mmio.regs = regs;

	spin_lock_init(&dev->mmio.irq_lock);
}
EXPORT_SYMBOL_GPL(mt76_mmio_init);
