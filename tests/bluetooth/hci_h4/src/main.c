/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Receive path of the H:4 driver (drivers/bluetooth/hci/h4.c). The script
 * of each case injects received bytes, and host transmissions, at driver
 * events (h4_harness.h), which places them deterministically inside the
 * windows where the driver's RX thread and its ISR interleave. Each case
 * checks the delivered packets byte for byte and that no buffer is
 * overrun or leaked, no RX interrupt is left pending that the ISR ignores,
 * and the whole script ran, so the window was reached.
 */

#include <string.h>

#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "h4_harness.h"

/* HCI_Command_Complete for HCI_Reset */
static const uint8_t evt_cc[] = {BT_HCI_H4_EVT, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};

/* ACL data packet on handle 1 with len payload bytes; returns its length */
static size_t make_acl(uint8_t *out, uint16_t len, uint8_t seed)
{
	out[0] = BT_HCI_H4_ACL;
	sys_put_le16(0x0001, &out[1]);
	sys_put_le16(len, &out[3]);
	for (uint16_t i = 0U; i < len; i++) {
		out[5 + i] = seed + i;
	}

	return 5U + len;
}

static void expect_delivered(uint32_t n)
{
	uint32_t got = h4_test_wait_delivered(n);

	zassert_equal(got, n, "%u packets delivered, %u expected", got, n);
}

static void expect_pkt(uint32_t idx, const uint8_t *data, size_t len)
{
	const struct h4_test_pkt *p = h4_test_pkt(idx);

	zassert_false(p->overrun, "packet %u: written past the buffer", idx);
	zassert_equal(p->len, len, "packet %u: %zu bytes delivered, %zu sent", idx, p->len, len);
	zassert_mem_equal(p->data, data, len, "packet %u: content differs", idx);
}

static void expect_clean(void)
{
	zassert_equal(h4_test_steps_left(), 0U, "script not run to its end");
	zassert_equal(h4_test_buffers_held(), 0U, "receive buffer leaked");
	zassert_false(h4_test_livelock(), "RX interrupt pending while the ISR reads nothing");
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* The cases share the buffer pools */
	zassume_equal(h4_test_buffers_held(), 0U, "buffer leaked by an earlier case");
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	h4_test_stop();
}

ZTEST_SUITE(hci_h4, NULL, NULL, before, after, NULL);

/* An event and ACL packets of different lengths, as a burst and byte by
 * byte.
 */
ZTEST(hci_h4, test_rx_baseline)
{
	static uint8_t burst[300];
	static uint8_t acl8[16];
	static uint8_t acl251[260];
	static uint8_t acl6[16];
	size_t n8 = make_acl(acl8, 8U, 0x10U);
	size_t n251 = make_acl(acl251, 251U, 0x20U);
	size_t n6 = make_acl(acl6, 6U, 0x30U);
	size_t n = 0U;

	memcpy(&burst[n], evt_cc, sizeof(evt_cc));
	n += sizeof(evt_cc);
	memcpy(&burst[n], acl8, n8);
	n += n8;
	memcpy(&burst[n], acl251, n251);
	n += n251;

	h4_test_start(NULL, 0U);
	h4_test_feed(burst, n);
	expect_delivered(3U);
	for (size_t i = 0U; i < n6; i++) {
		h4_test_feed(&acl6[i], 1U);
	}
	expect_delivered(4U);

	expect_pkt(0U, evt_cc, sizeof(evt_cc));
	expect_pkt(1U, acl8, n8);
	expect_pkt(2U, acl251, n251);
	expect_pkt(3U, acl6, n6);
	expect_clean();
}

/*
 * Stale wake-up: P2 completes while the RX thread delivers P1, so the
 * thread delivers P2 in the same pass and the semaphore stays given. The
 * header of P3 arrives after that pass; the thread wakes on the stale
 * semaphore with the RX interrupt enabled and allocates the buffer for P3
 * while the next byte of P3 arrives.
 */
ZTEST(hci_h4, test_rx_stale_wakeup)
{
	static uint8_t p1[16];
	static uint8_t p2[16];
	static uint8_t p3[32];
	size_t n1 = make_acl(p1, 8U, 0x10U);
	size_t n2 = make_acl(p2, 6U, 0x20U);
	size_t n3 = make_acl(p3, 20U, 0x30U);
	const struct h4_test_step steps[] = {
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = p1, .rx_len = n1},
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = p2, .rx_len = n2},
		{.ev = H4_TEST_EV_RX_ENABLE, .min_delivered = 2U, .rx = p3, .rx_len = 5U},
		{.ev = H4_TEST_EV_ALLOC, .min_delivered = 2U, .rx = &p3[5], .rx_len = 1U},
		{.ev = H4_TEST_EV_RX_ENABLE, .min_delivered = 2U, .rx = &p3[6], .rx_len = n3 - 6U},
	};

	h4_test_start(steps, ARRAY_SIZE(steps));
	expect_delivered(3U);

	expect_pkt(0U, p1, n1);
	expect_pkt(1U, p2, n2);
	expect_pkt(2U, p3, n3);
	expect_clean();
}

/*
 * TX interrupt while the RX thread allocates: the header of P2 completes
 * while the thread delivers P1. The thread then allocates the buffer for
 * P2 with the RX interrupt disabled, and meanwhile the next byte of P2
 * arrives and the host sends a command.
 */
ZTEST(hci_h4, test_rx_tx_irq_during_alloc)
{
	static uint8_t p1[16];
	static uint8_t p2[32];
	size_t n1 = make_acl(p1, 8U, 0x10U);
	size_t n2 = make_acl(p2, 20U, 0x30U);
	const struct h4_test_step steps[] = {
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = p1, .rx_len = n1},
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = p2, .rx_len = 5U},
		{.ev = H4_TEST_EV_ALLOC,
		 .min_delivered = 1U,
		 .rx = &p2[5],
		 .rx_len = 1U,
		 .send_tx = true},
		{.ev = H4_TEST_EV_RX_ENABLE, .min_delivered = 1U, .rx = &p2[6], .rx_len = n2 - 6U},
	};

	h4_test_start(steps, ARRAY_SIZE(steps));
	expect_delivered(2U);

	expect_pkt(0U, p1, n1);
	expect_pkt(1U, p2, n2);
	zassert_equal(h4_test_tx_len(), 4U, "command not sent");
	expect_clean();
}

/*
 * TX interrupt during the allocation that the ISR deferred to the RX
 * thread: no buffer is free when the header of P is complete. While the
 * thread waits, a buffer becomes free and the host sends a command.
 */
ZTEST(hci_h4, test_rx_tx_irq_during_deferred_alloc)
{
	static uint8_t p[32];
	size_t n = make_acl(p, 20U, 0x40U);
	const struct h4_test_step steps[] = {
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = p, .rx_len = n},
		{.ev = H4_TEST_EV_ALLOC, .unblock = true, .send_tx = true},
	};

	h4_test_start(steps, ARRAY_SIZE(steps));
	h4_test_block_buffers(true);
	expect_delivered(1U);

	expect_pkt(0U, p, n);
	zassert_equal(h4_test_tx_len(), 4U, "command not sent");
	expect_clean();
}

/*
 * TX interrupt while the RX thread hands the RX state back to the ISR: no
 * buffer is free, the header of P2 is complete and its next byte is
 * waiting when the thread re-enables RX after delivering P1, and the host
 * sends a command at that moment. The ISR defers the allocation for P2,
 * which must not leave an RX interrupt pending that the ISR ignores.
 */
ZTEST(hci_h4, test_rx_tx_irq_during_handover)
{
	static uint8_t p1[16];
	static uint8_t p2[32];
	static uint8_t first[32];
	size_t n1 = make_acl(p1, 8U, 0x10U);
	size_t n2 = make_acl(p2, 20U, 0x30U);
	const struct h4_test_step steps[] = {
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = first, .rx_len = n1 + 5U},
		{.ev = H4_TEST_EV_RX_ENABLE_PRE,
		 .rx = &p2[5],
		 .rx_len = 1U,
		 .block = true,
		 .send_tx = true},
		{.ev = H4_TEST_EV_ALLOC, .unblock = true},
		{.ev = H4_TEST_EV_RX_ENABLE, .min_delivered = 1U, .rx = &p2[6], .rx_len = n2 - 6U},
	};

	memcpy(first, p1, n1);
	memcpy(&first[n1], p2, 5U);
	h4_test_start(steps, ARRAY_SIZE(steps));
	expect_delivered(2U);

	expect_pkt(0U, p1, n1);
	expect_pkt(1U, p2, n2);
	zassert_equal(h4_test_tx_len(), 4U, "command not sent");
	expect_clean();
}

/*
 * Buffer size on the RX thread's allocation path: an ACL packet that fits
 * the buffer only without its header is discarded, and the next packet is
 * received intact.
 */
ZTEST(hci_h4, test_rx_size_check_with_header)
{
	static uint8_t rx[300];
	static uint8_t next[16];
	/* Payload length that fits the tailroom, but not with the header */
	size_t n_big = make_acl(rx, BT_BUF_ACL_RX_SIZE - 1U - 2U, 0x60U);
	size_t n_next = make_acl(next, 8U, 0x70U);
	const struct h4_test_step steps[] = {
		{.ev = H4_TEST_EV_RX_ENABLE, .rx = rx, .rx_len = n_big + n_next},
		{.ev = H4_TEST_EV_ALLOC, .unblock = true},
	};

	memcpy(&rx[n_big], next, n_next);
	h4_test_start(steps, ARRAY_SIZE(steps));
	h4_test_block_buffers(true);
	expect_delivered(1U);

	expect_pkt(0U, next, n_next);
	expect_clean();
}
