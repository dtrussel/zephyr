/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Harness of the H:4 driver test: a test UART, the receive buffers, the
 * receive callback, and a script of received bytes injected at driver
 * events.
 *
 * Test UART (vnd,h4-test-uart), with the interrupt semantics of the STM32
 * driver:
 * - uart_irq_rx_ready() reports waiting bytes whether or not the RX
 *   interrupt is enabled; uart_irq_is_pending() is true for RX enabled
 *   with bytes waiting, or for TX enabled; uart_irq_tx_ready() is true
 *   while TX is enabled.
 * - uart_fifo_read() and uart_fifo_fill() move at most one byte per call,
 *   so a command sent runs the ISR once per byte.
 * - The interrupt is raised (irq_offload) as soon as its condition holds.
 *   With interrupts locked it stays pending until they are unlocked, as
 *   a hardware interrupt does. This holds for irq_offload() on native_sim
 *   only, so the test runs there.
 *
 * A script step runs at the first occurrence of its event once at least
 * min_delivered packets were delivered; steps run in order, one per event.
 * Events occur in thread context only:
 * - H4_TEST_EV_RX_ENABLE_PRE: in uart_irq_rx_enable(), before the RX
 *   interrupt is enabled;
 * - H4_TEST_EV_RX_ENABLE: in uart_irq_rx_enable(), after it is enabled;
 * - H4_TEST_EV_ALLOC: in a K_FOREVER receive buffer allocation (the
 *   driver's RX thread);
 * - H4_TEST_EV_DELIVERED: in the receive callback, once the packet is
 *   recorded.
 * An optional step aims at a window that a fixed driver no longer opens;
 * h4_test_finish_script() then runs it from the test thread.
 */

#ifndef H4_HARNESS_H_
#define H4_HARNESS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define H4_TEST_MAX_PKTS    8U
#define H4_TEST_MAX_PKT_LEN 300U
/* Length of the HCI command the script sends */
#define H4_TEST_CMD_LEN     12U

enum h4_test_event {
	H4_TEST_EV_RX_ENABLE_PRE,
	H4_TEST_EV_RX_ENABLE,
	H4_TEST_EV_ALLOC,
	H4_TEST_EV_DELIVERED,
};

struct h4_test_step {
	enum h4_test_event ev;
	uint32_t min_delivered;
	/* Bytes that arrive at the UART */
	const uint8_t *rx;
	size_t rx_len;
	/* Make buffer allocation fail (K_NO_WAIT) or wait (K_FOREVER), or
	 * end that, before the bytes arrive.
	 */
	bool block;
	bool unblock;
	/* Send an HCI command, which raises the TX interrupt */
	bool send_tx;
	/* Run by h4_test_finish_script() if its event does not occur */
	bool optional;
};

struct h4_test_pkt {
	size_t len;
	uint8_t data[H4_TEST_MAX_PKT_LEN];
	/* Written past the end of the buffer */
	bool overrun;
};

/* Reset the harness, set the script and open the driver */
void h4_test_start(const struct h4_test_step *steps, size_t n_steps);
/* Close the driver */
void h4_test_stop(void);
/* Bytes arrive at the UART */
void h4_test_feed(const uint8_t *data, size_t len);
/* As the block and unblock script steps. Right after h4_test_start() it
 * takes effect before the driver receives anything.
 */
void h4_test_block_buffers(bool blocked);

/* Wait until n packets are delivered or a timeout expires, then a little
 * longer so a surplus delivery shows too. Returns the packets delivered.
 */
uint32_t h4_test_wait_delivered(uint32_t n);
const struct h4_test_pkt *h4_test_pkt(uint32_t idx);

/* Let the driver settle, then run the optional steps left, in order and
 * regardless of min_delivered; stops at a required step.
 */
void h4_test_finish_script(void);
size_t h4_test_steps_left(void);
/* Receive buffers not returned to their pools */
uint32_t h4_test_buffers_held(void);
/* Bytes the driver sent */
size_t h4_test_tx_len(void);
/* An RX interrupt was pending while the ISR read nothing: on hardware the
 * ISR would never return.
 */
bool h4_test_livelock(void);
/* Receive buffer allocations with K_FOREVER (the driver's RX thread), and
 * those with K_NO_WAIT that failed (the ISR)
 */
uint32_t h4_test_thread_allocs(void);
uint32_t h4_test_isr_alloc_failures(void);

#endif /* H4_HARNESS_H_ */
