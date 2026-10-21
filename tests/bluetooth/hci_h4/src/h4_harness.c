/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vnd_h4_test_uart

#include <string.h>

#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq_offload.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "h4_harness.h"

#define ACL_BUFS      4U
#define EVT_BUFS      4U
#define GUARD_LEN     16U
#define GUARD_BYTE    0xa5U
#define RX_QUEUE_LEN  1024U
#define TX_SINK_LEN   64U
/* uart_irq_is_pending() calls with only RX pending and no byte read */
#define LIVELOCK_POLL 1000U

/* Each buffer has a guard behind its nominal size that shows an overrun */
NET_BUF_POOL_FIXED_DEFINE(acl_pool, ACL_BUFS, BT_BUF_ACL_RX_SIZE + GUARD_LEN, 0, NULL);
NET_BUF_POOL_FIXED_DEFINE(evt_pool, EVT_BUFS, BT_BUF_EVT_RX_SIZE + GUARD_LEN, 0, NULL);
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 1, BT_BUF_CMD_SIZE(0), 0, NULL);

static const struct device *const h4_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_hci));

/* HCI_Reset */
static const uint8_t tx_cmd[] = {BT_HCI_H4_CMD, 0x03, 0x0c, 0x00};

static struct {
	/* Test UART */
	const struct device *uart;
	uart_irq_callback_user_data_t cb;
	void *cb_data;
	bool rx_enabled;
	bool tx_enabled;
	uint8_t rxq[RX_QUEUE_LEN];
	size_t rx_head;
	size_t rx_tail;
	uint32_t rx_reads;
	uint32_t rx_reads_seen;
	uint32_t stall;
	bool livelock;
	size_t tx_len;

	/* Script */
	const struct h4_test_step *steps;
	size_t n_steps;
	size_t step;

	/* Receive buffers */
	bool blocked;
	struct k_sem unblock;

	/* Delivered packets */
	uint32_t delivered;
	struct k_sem delivered_sem;
	struct h4_test_pkt pkts[H4_TEST_MAX_PKTS];
} h;

static size_t rx_avail(void)
{
	return h.rx_tail - h.rx_head;
}

static bool irq_pending(void)
{
	bool rx = h.rx_enabled && rx_avail() > 0U;

	if (rx && !h.tx_enabled && h.rx_reads == h.rx_reads_seen) {
		/* Report the livelock and let the ISR return */
		if (++h.stall > LIVELOCK_POLL) {
			h.livelock = true;
			return false;
		}
	} else {
		h.stall = 0U;
	}
	h.rx_reads_seen = h.rx_reads;

	return rx || h.tx_enabled;
}

static void uart_isr(const void *param)
{
	ARG_UNUSED(param);

	h.cb(h.uart, h.cb_data);
}

/* Raise the interrupt if its condition holds. In the ISR, the driver's
 * loop checks uart_irq_is_pending() itself.
 */
static void fire(void)
{
	if (k_is_in_isr() || h.cb == NULL) {
		return;
	}

	h.stall = 0U;
	if (irq_pending()) {
		irq_offload(uart_isr, NULL);
	}
}

static void rx_put(const uint8_t *data, size_t len)
{
	zassert_true(h.rx_tail + len <= RX_QUEUE_LEN);
	memcpy(&h.rxq[h.rx_tail], data, len);
	h.rx_tail += len;
}

static void send_cmd(void)
{
	struct net_buf *buf = net_buf_alloc(&tx_pool, K_NO_WAIT);

	zassert_not_null(buf);
	net_buf_add_mem(buf, tx_cmd, sizeof(tx_cmd));
	zassert_ok(bt_hci_send(h4_dev, buf));
}

static void script_event(enum h4_test_event ev)
{
	const struct h4_test_step *s;

	if (k_is_in_isr() || h.step >= h.n_steps) {
		return;
	}

	s = &h.steps[h.step];
	if (s->ev != ev || h.delivered < s->min_delivered) {
		return;
	}

	h.step++;
	if (s->block) {
		h.blocked = true;
	}
	if (s->unblock) {
		h.blocked = false;
		k_sem_give(&h.unblock);
	}
	if (s->rx_len > 0U) {
		rx_put(s->rx, s->rx_len);
	}
	if (s->send_tx) {
		send_cmd();
	}
	fire();
}

static int test_fifo_fill(const struct device *dev, const uint8_t *data, int len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	h.tx_len += len;

	return len;
}

static int test_fifo_read(const struct device *dev, uint8_t *data, const int size)
{
	ARG_UNUSED(dev);

	if (size <= 0 || rx_avail() == 0U) {
		return 0;
	}

	data[0] = h.rxq[h.rx_head++];
	h.rx_reads++;

	return 1;
}

static void test_irq_tx_enable(const struct device *dev)
{
	ARG_UNUSED(dev);

	h.tx_enabled = true;
	fire();
}

static void test_irq_tx_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	h.tx_enabled = false;
}

static int test_irq_tx_ready(const struct device *dev)
{
	ARG_UNUSED(dev);

	return h.tx_enabled ? 1 : 0;
}

static void test_irq_rx_enable(const struct device *dev)
{
	ARG_UNUSED(dev);

	script_event(H4_TEST_EV_RX_ENABLE_PRE);
	h.rx_enabled = true;
	script_event(H4_TEST_EV_RX_ENABLE);
	fire();
}

static void test_irq_rx_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

	h.rx_enabled = false;
}

static int test_irq_rx_ready(const struct device *dev)
{
	ARG_UNUSED(dev);

	return rx_avail() > 0U ? 1 : 0;
}

static int test_irq_is_pending(const struct device *dev)
{
	ARG_UNUSED(dev);

	return irq_pending() ? 1 : 0;
}

static void test_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static void test_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				  void *user_data)
{
	h.uart = dev;
	h.cb = cb;
	h.cb_data = user_data;
}

static DEVICE_API(uart, test_uart_api) = {
	.fifo_fill = test_fifo_fill,
	.fifo_read = test_fifo_read,
	.irq_tx_enable = test_irq_tx_enable,
	.irq_tx_disable = test_irq_tx_disable,
	.irq_tx_ready = test_irq_tx_ready,
	.irq_rx_enable = test_irq_rx_enable,
	.irq_rx_disable = test_irq_rx_disable,
	.irq_rx_ready = test_irq_rx_ready,
	.irq_is_pending = test_irq_is_pending,
	.irq_update = test_irq_update,
	.irq_callback_set = test_irq_callback_set,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, PRE_KERNEL_1, 0, &test_uart_api);

/* Receive buffers as the Bluetooth host provides them: the H:4 packet type
 * first, nominal size of the host's pools.
 */
static struct net_buf *alloc_rx(struct net_buf_pool *pool, uint16_t size, uint8_t h4_type,
				k_timeout_t timeout)
{
	struct net_buf *buf;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		script_event(H4_TEST_EV_ALLOC);
	}

	if (h.blocked) {
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			return NULL;
		}
		(void)k_sem_take(&h.unblock, timeout);
	}

	buf = net_buf_alloc(pool, timeout);
	if (buf != NULL) {
		buf->size = size;
		memset(buf->__buf + size, GUARD_BYTE, GUARD_LEN);
		net_buf_add_u8(buf, h4_type);
	}

	return buf;
}

struct net_buf *bt_buf_get_rx(enum bt_buf_type type, k_timeout_t timeout)
{
	if (type == BT_BUF_EVT) {
		return alloc_rx(&evt_pool, BT_BUF_EVT_RX_SIZE, BT_HCI_H4_EVT, timeout);
	}

	return alloc_rx(&acl_pool, BT_BUF_ACL_RX_SIZE, BT_HCI_H4_ACL, timeout);
}

struct net_buf *bt_buf_get_evt(uint8_t evt, bool discardable, k_timeout_t timeout)
{
	ARG_UNUSED(evt);
	ARG_UNUSED(discardable);

	return bt_buf_get_rx(BT_BUF_EVT, timeout);
}

/* Used by the driver's debug logging */
const char *bt_hex(const void *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);

	return "";
}

static bool overrun(const struct net_buf *buf)
{
	if (buf->len > buf->size) {
		return true;
	}

	for (size_t i = 0U; i < GUARD_LEN; i++) {
		if (buf->__buf[buf->size + i] != GUARD_BYTE) {
			return true;
		}
	}

	return false;
}

static int recv(const struct device *dev, struct net_buf *buf)
{
	ARG_UNUSED(dev);

	if (h.delivered < H4_TEST_MAX_PKTS) {
		struct h4_test_pkt *p = &h.pkts[h.delivered];

		p->len = MIN(buf->len, sizeof(p->data));
		memcpy(p->data, buf->data, p->len);
		p->overrun = overrun(buf);
	}

	net_buf_unref(buf);
	h.delivered++;
	k_sem_give(&h.delivered_sem);

	return 0;
}

void h4_test_start(const struct h4_test_step *steps, size_t n_steps)
{
	const struct device *uart = h.uart;
	uart_irq_callback_user_data_t cb = h.cb;
	void *cb_data = h.cb_data;

	memset(&h, 0, sizeof(h));
	h.uart = uart;
	h.cb = cb;
	h.cb_data = cb_data;
	h.steps = steps;
	h.n_steps = n_steps;
	k_sem_init(&h.unblock, 0, 1);
	k_sem_init(&h.delivered_sem, 0, K_SEM_MAX_LIMIT);

	zassert_ok(bt_hci_open(h4_dev, recv));
}

void h4_test_stop(void)
{
	(void)bt_hci_close(h4_dev);
}

void h4_test_feed(const uint8_t *data, size_t len)
{
	rx_put(data, len);
	fire();
}

void h4_test_block_buffers(bool blocked)
{
	h.blocked = blocked;
}

uint32_t h4_test_wait_delivered(uint32_t n)
{
	int64_t end = k_uptime_get() + 500;

	while (h.delivered < n && k_uptime_get() < end) {
		(void)k_sem_take(&h.delivered_sem, K_MSEC(10));
	}
	k_msleep(20);

	return h.delivered;
}

const struct h4_test_pkt *h4_test_pkt(uint32_t idx)
{
	return &h.pkts[idx];
}

size_t h4_test_steps_left(void)
{
	return h.n_steps - h.step;
}

uint32_t h4_test_buffers_held(void)
{
	return (acl_pool.buf_count - atomic_get(&acl_pool.avail_count)) +
	       (evt_pool.buf_count - atomic_get(&evt_pool.avail_count));
}

size_t h4_test_tx_len(void)
{
	return h.tx_len;
}

bool h4_test_livelock(void)
{
	return h.livelock;
}
