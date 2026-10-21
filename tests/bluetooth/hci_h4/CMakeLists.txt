# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_hci_h4)

# The H:4 driver is built without the Bluetooth subsystem (CONFIG_BT=n): the
# test provides the receive buffers and receives the packets itself. The
# options the driver and <zephyr/bluetooth/buf.h> need are set here.
target_sources(app PRIVATE
  src/main.c
  src/h4_harness.c
  ${ZEPHYR_BASE}/drivers/bluetooth/hci/h4.c
)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/bluetooth)
target_compile_definitions(app PRIVATE
  CONFIG_BT_HCI_DRIVER_LOG_LEVEL=LOG_LEVEL_WRN
  CONFIG_BT_DRV_RX_STACK_SIZE=2048
  CONFIG_BT_RX_PRIO=8
  CONFIG_BT_HCI_INIT_PRIORITY=50
  CONFIG_BT_BUF_ACL_RX_SIZE=251
  CONFIG_BT_BUF_EVT_RX_SIZE=68
  CONFIG_BT_BUF_EVT_RX_COUNT=4
  CONFIG_BT_BUF_ACL_TX_COUNT=3
)
