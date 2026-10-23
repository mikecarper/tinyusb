#!/usr/bin/env python3
"""Run the production nRF5x DMA, SETUP and ISR paths against register mocks.

Controller functions and ISR dispatch execute from the driver source. Mock
register windows model task pulses and inject DMA completion; a uint32 software
word wrapper injects power interrupts between pending-word loads and stores.
These seams retain the production control flow and descriptor accounting.
"""

import os
import argparse
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / "src/portable/nordic/nrf5x/dcd_nrf5x.c"
CONFIG = r'''
#define CFG_TUSB_MCU OPT_MCU_NRF5X
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_DEBUG 0
#ifndef CFG_TUD_NRF5X_CONTROL_STALL_QUIET
#define CFG_TUD_NRF5X_CONTROL_STALL_QUIET 0
#endif
#ifndef NRFX_USBD_ERRATA_ENABLE
#define NRFX_USBD_ERRATA_ENABLE 1
#endif
'''

HARNESS = r'''
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <vector>
#include "tusb_option.h"
#include "device/dcd.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

enum RegKind { PLAIN, SETUP, IRQ_SET, IRQ_CLEAR, W1C, ERRATA, EPSTALL_WRITE, DTOGGLE_WRITE };
static std::function<void()> on_task_trigger;
struct Register {
  RegKind kind = PLAIN;
  uint32_t value = 0;
  operator uint32_t() const;
  void operator=(uint32_t value);
  void operator|=(uint32_t value) { *this = uint32_t(*this) | value; }
};
struct Endpoint { uintptr_t PTR = 0; uint32_t MAXCNT = 0, AMOUNT = 0; };
struct TaskRegister {
  unsigned triggers = 0;
  void operator=(uint32_t value) {
    if (value == 1) { if (on_task_trigger) on_task_trigger(); ++triggers; }
  }
  operator uint32_t() const { return 0; } // TASKS are write-only; zero is a hardware no-op.
};
struct MockUSBD {
  // The real ISR scans contiguous EVENTS registers using the hardware offsets.
  uint32_t EVENTS_USBRESET = 0, EVENTS_STARTED = 0, EVENTS_ENDEPIN[8] = {};
  uint32_t EVENTS_EP0DATADONE = 0, EVENTS_ENDISOIN = 0, EVENTS_ENDEPOUT[8] = {};
  uint32_t EVENTS_ENDISOOUT = 0, EVENTS_SOF = 0, EVENTS_USBEVENT = 0;
  uint32_t EVENTS_EP0SETUP = 0, EVENTS_EPDATA = 0;
  TaskRegister TASKS_STARTEPIN[8], TASKS_STARTEPOUT[8];
  TaskRegister TASKS_STARTISOIN, TASKS_STARTISOOUT;
  TaskRegister TASKS_EP0RCVOUT, TASKS_EP0STATUS, TASKS_EP0STALL, TASKS_DPDMDRIVE;
  uint32_t INTEN = 0, EPOUTEN = 0, EPINEN = 0, ISOSPLIT = 0;
  uint32_t ENABLE = 0, USBPULLUP = 0, LOWPOWER = 0, DPDMVALUE = 0;
  Register INTENSET, INTENCLR, EVENTCAUSE, EPDATASTATUS, EPSTALL, DTOGGLE;
  Register BMREQUESTTYPE, BREQUEST, WVALUEL, WVALUEH, WINDEXL, WINDEXH, WLENGTHL, WLENGTHH;
  Endpoint EPIN[9], EPOUT[9], ISOIN, ISOOUT;
  struct { uint32_t EPOUT[9] = {}, ISOOUT = 0; } SIZE;
  struct {
    uint32_t EPIN[8] = {}, EPOUT[8] = {};
  } HALTED;
} usbd;
static_assert(offsetof(MockUSBD, EVENTS_EPDATA) - offsetof(MockUSBD, EVENTS_USBRESET) == 24 * 4,
              "mock event layout must match Nordic interrupt positions");
static Register errata_register;
static bool anomaly = true, hardware_dma_busy = false, irq_enabled = true;
static uint32_t primask, barriers, setup_reads_while_busy;
static std::vector<unsigned> setup_read_order;
struct SetupScan {
  uint32_t operator[](unsigned index) const {
    Register *const registers[] = {&usbd.BMREQUESTTYPE, &usbd.BREQUEST, &usbd.WVALUEL,  &usbd.WVALUEH,
                                   &usbd.WINDEXL,       &usbd.WINDEXH,  &usbd.WLENGTHL, &usbd.WLENGTHH};
    assert(index < 8);
    setup_read_order.push_back(index);
    return uint32_t(*registers[index]);
  }
};
struct MmioOperation {
  bool     write;
  uint32_t address, value;
};
static std::vector<MmioOperation> mmio_operations;
struct MmioRegister {
  uint32_t address = 0, value = 0;
  operator uint32_t() const {
    mmio_operations.push_back(MmioOperation{false, address, value});
    return value;
  }
  void operator=(uint32_t data) {
    mmio_operations.push_back(MmioOperation{true, address, data});
    value = data;
  }
};
static MmioRegister errata_unlock, errata_171, errata_187;
static bool         anomaly_171, anomaly_187;
static unsigned setup_events, transfer_events, unplugged_events, sof_events;
static std::vector<int> delivered_events;
static uint8_t last_setup[8], last_endpoint;
static uint8_t                             last_setup_tag;
static bool                                auto_admit_setup;
static uint32_t last_length;
static bool                                last_completion_in_isr;
static std::function<void()> on_barrier;
static std::function<void(uint32_t, bool)> on_latch_write;
static std::function<void(uint32_t)> on_pending_read;
static std::function<void(uint32_t)> on_epstall_write;
static std::function<void(const Register *, uint32_t)> on_register_write;
static std::function<void()> on_primask_read;
static std::function<void()> on_irq_restore;
// This POD preserves the pending word's ordinary C read/modify/write order,
// while allowing a higher-priority power interrupt between its load/store.
struct PendingBits {
  uint32_t value;
  operator uint32_t() const {
    uint32_t const snapshot = value;
    if (on_pending_read) on_pending_read(snapshot);
    return snapshot;
  }
  void operator=(uint32_t bits) { value = bits; }
  template<typename Bits> void operator|=(Bits bits) { uint32_t const before = *this; value = before | uint32_t(bits); }
  template<typename Bits> void operator&=(Bits bits) { uint32_t const before = *this; value = before & uint32_t(bits); }
};
static std::function<void(uint32_t*)> on_event_read;
static uint32_t event_reads;
struct EventRef {
  uint32_t *value;
  operator uint32_t() const { ++event_reads; if (on_event_read) on_event_read(value); return *value; }
  void operator=(uint32_t v) { *value = v; }
};
struct EventScan {
  uint32_t *base;
  EventRef operator[](unsigned i) const { return EventRef{base + i}; }
};
static std::vector<uint32_t> latch_writes;
static std::vector<uint32_t> epstall_writes, dtoggle_writes;
struct Deferred { osal_task_func_t function; void *parameter; };
static std::vector<Deferred> deferred;
struct { uint32_t ICSR = 0; } scb;
#define NRF_USBD (&usbd)
#define NRF_USBD_BASE 0u
#define NRF52_SERIES 1
#define SCB (&scb)
#define SCB_ICSR_VECTACTIVE_Msk 0x1ffu
#define NVIC_GetEnableIRQ mock_get_enable_irq
static uint32_t mock_get_enable_irq(int) { return irq_enabled; }
static void NVIC_EnableIRQ(int) { irq_enabled = true; }
static void NVIC_DisableIRQ(int) { irq_enabled = false; }
static void NVIC_ClearPendingIRQ(int) {}
void NVIC_SetPendingIRQ(int) {}
uint32_t        __get_IPSR() {
  return scb.ICSR & SCB_ICSR_VECTACTIVE_Msk;
}
uint32_t __get_PRIMASK() { if (on_primask_read) on_primask_read(); return primask; }
void __disable_irq() { primask = 1; }
void __set_PRIMASK(uint32_t value) { primask = value; if (!value && on_irq_restore) on_irq_restore(); }
void __enable_irq() { __set_PRIMASK(0); }
unsigned mock_ctz(uint32_t bits) { assert(bits != 0); return __builtin_ctz(bits); }
static void __ISB() { ++barriers; if (on_barrier) on_barrier(); }
static void __DSB() { ++barriers; if (on_barrier) on_barrier(); }
bool nrfx_usbd_errata_199() { return NRFX_USBD_ERRATA_ENABLE && anomaly; }
static bool nrfx_usbd_errata_166() { return false; }
static bool nrfx_usbd_errata_171() { return NRFX_USBD_ERRATA_ENABLE && anomaly_171; }
static bool nrfx_usbd_errata_187() { return NRFX_USBD_ERRATA_ENABLE && anomaly_187; }
enum {
  USBD_IRQn = 39,
  USBD_INTEN_ENDEPIN0_Pos = 2, USBD_INTEN_ENDEPOUT0_Pos = 12, USBD_INTEN_EPDATA_Pos = 24,
  USBD_INTEN_ENDISOIN_Pos = 11,
  USBD_INTEN_USBRESET_Msk = 1u,
  USBD_INTEN_ENDEPIN0_Msk = 1u << 2, USBD_INTEN_ENDEPOUT0_Msk = 1u << 12,
  USBD_INTEN_EP0DATADONE_Msk = 1u << 10, USBD_INTEN_ENDISOIN_Msk = 1u << 11,
  USBD_INTEN_ENDISOOUT_Msk = 1u << 20, USBD_INTEN_SOF_Msk = 1u << 21,
  USBD_INTEN_USBEVENT_Msk = 1u << 22, USBD_INTEN_EP0SETUP_Msk = 1u << 23,
  USBD_INTEN_EPDATA_Msk = 1u << 24,
  USBD_INTENCLR_ENDISOIN_Msk = USBD_INTEN_ENDISOIN_Msk,
  USBD_INTENCLR_ENDISOOUT_Msk = USBD_INTEN_ENDISOOUT_Msk,
  USBD_INTENCLR_SOF_Msk = USBD_INTEN_SOF_Msk,
  USBD_INTENSET_ENDISOIN_Msk = USBD_INTEN_ENDISOIN_Msk,
  USBD_INTENSET_ENDISOOUT_Msk = USBD_INTEN_ENDISOOUT_Msk,
  USBD_INTENSET_SOF_Msk = USBD_INTEN_SOF_Msk,
  USBD_EPOUTEN_ISOOUT_Msk = 1u << 8, USBD_EPINEN_ISOIN_Msk = 1u << 8,
  USBD_SIZE_ISOOUT_ZERO_Msk = 1u << 16,
  USBD_EVENTCAUSE_READY_Msk = 1u << 11, USBD_EVENTCAUSE_SUSPEND_Msk = 1u << 8,
  USBD_EVENTCAUSE_RESUME_Msk = 1u << 9, USBD_EVENTCAUSE_USBWUALLOWED_Msk = 1u << 10,
  USBD_DPDMVALUE_STATE_Resume = 1,
  USBD_EPSTALL_STALL_UnStall = 0, USBD_EPSTALL_STALL_Stall = 1, USBD_EPSTALL_STALL_Pos = 8,
  USBD_DTOGGLE_VALUE_Data0 = 1, USBD_DTOGGLE_VALUE_Pos = 8,
  USBD_ISOSPLIT_SPLIT_OneDir = 0, USBD_ISOSPLIT_SPLIT_HalfIN = 0x80,
};

Register::operator uint32_t() const {
  if (kind == SETUP && hardware_dma_busy) { ++setup_reads_while_busy; return 0; }
  return value;
}
void Register::operator=(uint32_t v) {
  if (on_register_write) on_register_write(this, v);
  if (kind == ERRATA && on_latch_write) on_latch_write(v, false);
  if (kind == IRQ_SET) usbd.INTEN |= v;
  else if (kind == IRQ_CLEAR) usbd.INTEN &= ~v;
  else if (kind == W1C) value &= ~v;
  else {
    value = v;
    if (kind == EPSTALL_WRITE) {
      if (on_epstall_write) on_epstall_write(v);
      const unsigned epnum = v & 0xfu;
      assert(epnum <= 8);
      // ISO endpoint8 has no HALTED register; inherited open still writes UnStall.
      if (epnum < 8) {
        (v & 0x80u ? usbd.HALTED.EPIN[epnum] : usbd.HALTED.EPOUT[epnum]) = (v >> 8) & 1u;
      }
      epstall_writes.push_back(v);
    }
    if (kind == DTOGGLE_WRITE) {
      dtoggle_writes.push_back(v);
    }
    if (kind == ERRATA) {
      // Aborted DMA may still own the controller until it is disabled.
      if (!v && hardware_dma_busy) assert(!usbd.ENABLE);
      latch_writes.push_back(v);
    }
  }
  if (kind == ERRATA && on_latch_write) on_latch_write(v, true);
}
void dcd_event_handler(dcd_event_t const *event, bool) {
  if (event->event_id == DCD_EVENT_SETUP_RECEIVED) {
    ++setup_events; delivered_events.push_back(-1);
    memcpy(last_setup, &event->setup_received, 8);
#if DCD_TEST_HEADER_SETUP_TAGS
    last_setup_tag = event->setup_tagid;
#endif
  }
  if (event->event_id == DCD_EVENT_UNPLUGGED) ++unplugged_events;
}
void dcd_event_bus_signal(uint8_t, dcd_eventid_t event, bool) {
  if (event == DCD_EVENT_UNPLUGGED) ++unplugged_events;
  if (event == DCD_EVENT_SOF) ++sof_events;
}
void dcd_event_bus_reset(uint8_t, tusb_speed_t, bool) {}
void dcd_event_setup_received(uint8_t, uint8_t const *setup, bool) {
  ++setup_events;
  delivered_events.push_back(-1);
  memcpy(last_setup, setup, 8);
  last_setup_tag = 0;
}
void dcd_event_setup_received_tagid(uint8_t, const uint8_t *setup, uint8_t tag, bool) {
  ++setup_events;
  delivered_events.push_back(-1);
  memcpy(last_setup, setup, 8);
  last_setup_tag = tag;
}
void dcd_event_xfer_complete(uint8_t, uint8_t endpoint, uint32_t length, uint8_t result, bool in_isr) {
  assert(result == XFER_RESULT_SUCCESS);
  ++transfer_events; delivered_events.push_back(endpoint); last_endpoint = endpoint; last_length = length;
  last_completion_in_isr                                                                         = in_isr;
}
void usbd_defer_func(osal_task_func_t function, void *parameter, bool) {
  deferred.push_back(Deferred{function, parameter});
}
bool tud_inited(void) { return true; }
static bool hfclk_running() { return true; }
static void hfclk_enable() {}
static void hfclk_disable() {}

static unsigned quiet_delay_calls;
static std::function<void(uint32_t)> quiet_delay_hook;
static __attribute__((unused)) void nrf_delay_us(uint32_t us) {
  assert(!primask);
  ++quiet_delay_calls;
  if (quiet_delay_hook) quiet_delay_hook(us);
}

/* DRIVER_FUNCTIONS */

static bool admit_setup(uint8_t tag) {
#if DCD_TEST_DRIVER_SETUP_TAGS
  return dcd_edpt0_setup_begin(0, tag);
#else
  // A legacy DCD has no admission hook: the real core accepts its SETUPs.
  (void)tag;
  return true;
#endif
}
static void reset() {
  quiet_delay_calls = 0;
  quiet_delay_hook = {};
  on_barrier = {};
  on_task_trigger = {};
  on_latch_write = {};
  on_pending_read = {};
  on_epstall_write = {};
  on_register_write = {};
  on_primask_read = {};
  on_irq_restore = {};
  on_event_read = {};
  usbd = MockUSBD{};
  errata_register = Register{};
  errata_register.kind = ERRATA;
  usbd.INTENSET.kind = IRQ_SET; usbd.INTENCLR.kind = IRQ_CLEAR;
  usbd.EVENTCAUSE.kind = W1C; usbd.EPDATASTATUS.kind = W1C;
  usbd.EPSTALL.kind = EPSTALL_WRITE; usbd.DTOGGLE.kind = DTOGGLE_WRITE;
  for (Register *reg: {&usbd.BMREQUESTTYPE, &usbd.BREQUEST, &usbd.WVALUEL, &usbd.WVALUEH,
                       &usbd.WINDEXL, &usbd.WINDEXH, &usbd.WLENGTHL, &usbd.WLENGTHH}) reg->kind = SETUP;
  memset(&_dcd, 0, sizeof(_dcd));
#if DCD_TEST_DRIVER_SETUP_TAGS
  memset(&_ep0, 0, sizeof(_ep0));
#endif
  _dcd.xfer[0][TUSB_DIR_IN].mps = _dcd.xfer[0][TUSB_DIR_OUT].mps = MAX_PACKET_SIZE;
  scb.ICSR = 0;
  anomaly = irq_enabled = true; hardware_dma_busy = false;
  anomaly_171 = anomaly_187 = false;
  errata_unlock.address = 0x4006ec00;
  errata_171.address = 0x4006ec14;
  errata_187.address = 0x4006ed14;
  errata_unlock.value = errata_171.value = errata_187.value = 0;
  mmio_operations.clear();
  setup_read_order.clear();
  usbd.ENABLE = 1;
  primask = barriers = setup_reads_while_busy = 0;
  event_reads = 0;
  setup_events = transfer_events = unplugged_events = sof_events = 0;
  last_endpoint = 0; last_length = 0;
  last_completion_in_isr         = false;
  last_setup_tag                 = 0;
  auto_admit_setup               = true;
  memset(last_setup, 0, sizeof(last_setup));
  latch_writes.clear();
  epstall_writes.clear(); dtoggle_writes.clear();
  delivered_events.clear();
  deferred.clear();
}
static void setup_in() {
  usbd.BMREQUESTTYPE.value = 0x80;
  usbd.BREQUEST.value = TUSB_REQ_GET_DESCRIPTOR;
  usbd.WVALUEH.value = TUSB_DESC_DEVICE;
  usbd.WLENGTHL.value = 18;
  usbd.EVENTS_EP0SETUP = 1;
  usbd.INTEN |= USBD_INTEN_EP0SETUP_Msk;
}
static void setup_clear_out() {
  usbd.BMREQUESTTYPE.value = 0x02;
  usbd.BREQUEST.value      = TUSB_REQ_CLEAR_FEATURE;
  usbd.WVALUEL.value = usbd.WVALUEH.value = 0;
  usbd.WINDEXL.value                      = 3;
  usbd.WINDEXH.value                      = 0;
  usbd.WLENGTHL.value = usbd.WLENGTHH.value = 0;
  usbd.EVENTS_EP0SETUP                      = 1;
  usbd.INTEN |= USBD_INTEN_EP0SETUP_Msk;
}
static void interrupt() {
  const unsigned setups_before = setup_events;
  scb.ICSR = 16 + USBD_IRQn;
  dcd_int_handler(0);
  scb.ICSR = 0;
  // The ordinary fixture models usbd processing the delivered SETUP after
  // ISR return. Stale-work cases explicitly control admission themselves.
  if (auto_admit_setup && setup_events != setups_before) {
    assert(admit_setup(last_setup_tag));
  }
}
static unsigned control_tasks() {
  return usbd.TASKS_EP0STATUS.triggers + usbd.TASKS_EP0RCVOUT.triggers + usbd.TASKS_EP0STALL.triggers +
         usbd.TASKS_STARTEPIN[0].triggers + usbd.TASKS_STARTEPOUT[0].triggers;
}
static void fresh_control_status() {
  assert(admit_setup(last_setup_tag));
  const unsigned before    = usbd.TASKS_EP0STATUS.triggers;
  const unsigned completed = transfer_events;
  assert(dcd_edpt_xfer(0, 0x80, nullptr, 0));
  assert(usbd.TASKS_EP0STATUS.triggers == before + 1);
  assert(transfer_events == completed + 1 && last_endpoint == 0x80 && last_length == 0);
}
static void run_deferred_once() {
  size_t const pending = deferred.size();
  for (size_t i = 0; i < pending; ++i) {
    Deferred const work = deferred.front();
    deferred.erase(deferred.begin());
    work.function(work.parameter);
  }
}
static void begin_in(uint8_t endpoint) {
  static uint8_t buffer[64];
  _dcd.xfer[endpoint][TUSB_DIR_IN].mps = 64;
  assert(dcd_edpt_xfer(0, endpoint | 0x80, buffer, sizeof(buffer)));
}
static void begin_out(uint8_t endpoint) {
  static uint8_t buffer[64];
  _dcd.xfer[endpoint][TUSB_DIR_OUT].mps = 64;
  _dcd.xfer[endpoint][TUSB_DIR_OUT].data_received = true;
  usbd.SIZE.EPOUT[endpoint] = sizeof(buffer);
  assert(dcd_edpt_xfer(0, endpoint, buffer, sizeof(buffer)));
}
static void complete_in_dma(uint8_t endpoint) {
  usbd.EVENTS_ENDEPIN[endpoint] = 1;
  usbd.INTEN |= 1u << (USBD_INTEN_ENDEPIN0_Pos + endpoint);
  interrupt();
}
#if DCD_TEST_DRIVER_SETUP_TAGS
static void quiet_setup() {
  reset();
  _ep0.received = _ep0.served = 9;
  _ep0.status_ep              = 0;
  _dcd.setup_dir              = TUSB_DIR_IN;
}
static bool quiet_shape_test(const char *name) {
  if (!strcmp(name, "quiet-status-thread") || !strcmp(name, "quiet-status-isr")) {
    quiet_setup();
    const bool in_isr = !strcmp(name, "quiet-status-isr");
    if (in_isr) {
      scb.ICSR = 16 + USBD_IRQn;
    }
    on_task_trigger  = [&]() { assert(!transfer_events && primask); };
    quiet_delay_hook = [&](uint32_t us) {
      assert(us == 2000 && !primask && irq_enabled);
      assert(usbd.TASKS_EP0STATUS.triggers == 1 && transfer_events == 1);
      assert(last_completion_in_isr == in_isr);
    };
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    if (in_isr) {
      assert(!quiet_delay_calls);
      scb.ICSR = 0;
      interrupt();
    }
    assert(quiet_delay_calls == 1 && transfer_events == 1 && !primask && irq_enabled);
    return true;
  }
  if (!strcmp(name, "quiet-stall-cleanup")) {
    for (unsigned dir = 0; dir < 2; dir++) {
      quiet_setup();
      if (dir == 0) {
        begin_out(3);
        usbd.EVENTS_ENDEPOUT[3] = 1;
      } else {
        begin_in(1);
        usbd.EVENTS_ENDEPIN[1] = 1;
      }
      quiet_delay_hook = [&](uint32_t us) {
        assert(us == 2000 && !primask && !irq_enabled && epstall_writes.size() == 1);
        if (!dir) {
          assert(!_dcd.dma_running && !usbd.EVENTS_ENDEPOUT[3] && !_dcd.xfer[3][0].total_len &&
                 !_dcd.xfer[3][0].actual_len && _dcd.xfer[3][0].data_received == OUT_HALTED);
        } else {
          assert(_dcd.dma_running == TU_BIT(DMA_IN_BASE + 1) && usbd.EVENTS_ENDEPIN[1]);
        }
      };
      dcd_edpt_stall(0, dir ? 0x83 : 3);
      assert(quiet_delay_calls == 1 && !primask && irq_enabled);
      if (dir) {
        usbd.INTEN |= TU_BIT(DMA_IN_BASE + 1);
        interrupt();
        assert(!_dcd.dma_running);
      }
    }
    return true;
  }
  if (!strcmp(name, "quiet-masked-policy")) {
    for (unsigned action = 0; action < 3; action++) {
      quiet_setup();
      primask = 1;
      if (!action) {
        assert(dcd_edpt_xfer(0, 0, nullptr, 0));
      } else {
        dcd_edpt_stall(0, action == 1 ? 0x83 : 3);
      }
      assert(!quiet_delay_calls && primask == 1 && !usbd.ENABLE && !irq_enabled && unplugged_events == 1);
      assert(action || usbd.TASKS_EP0STATUS.triggers == 1); // TASK committed before failclosed boundary.
    }
    return true;
  }
  if (!strcmp(name, "quiet-lifecycle")) {
    for (unsigned action = 0; action < 3; action++) {
      quiet_setup();
      unsigned writes = 0, after = 0;
      on_register_write = [&](const Register *, uint32_t) { writes++; };
      quiet_delay_hook  = [&](uint32_t) {
        assert(!primask);
        tusb_hal_nrf_power_event(1);
        usbd.ENABLE = 1;
        after       = writes;
      };
      if (!action) {
        assert(dcd_edpt_xfer(0, 0, nullptr, 0));
      } else {
        dcd_edpt_stall(0, action == 1 ? 0x83 : 3);
      }
      assert(quiet_delay_calls == 1 && writes == after && usbd.ENABLE && unplugged_events == 1);
      assert(_ep0.received == 10 && !primask);
    }
    return true;
  }
  if (!strcmp(name, "quiet-setup-order")) {
    quiet_setup();
    quiet_delay_hook = [&](uint32_t) {
      assert(transfer_events == 1 && delivered_events.size() == 1 && delivered_events[0] == 0);
      setup_in();
      interrupt();
      assert(setup_events == 1 && _ep0.received == 10 && _ep0.served == 10);
    };
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(quiet_delay_calls == 1 && delivered_events.size() == 2);
    assert(delivered_events[0] == 0 && delivered_events[1] == -1);
    quiet_setup();
    quiet_delay_hook = [&](uint32_t) {
      assert(!irq_enabled);
      setup_in();
    };
    dcd_edpt_stall(0, 0x83);
    assert(quiet_delay_calls == 1 && irq_enabled && !setup_events);
    interrupt();
    assert(setup_events == 1 && _ep0.received == 10);
    return true;
  }
  if (!strcmp(name, "quiet-status-usb-progress")) {
    quiet_setup();
    static uint8_t buffer[64];
    _dcd.xfer[1][TUSB_DIR_OUT].mps = 64;
    assert(dcd_edpt_xfer(0, 1, buffer, 64));
    quiet_delay_hook = [&](uint32_t) {
      assert(!primask && irq_enabled && epstall_writes.empty());
      usbd.SIZE.EPOUT[1]      = 64;
      usbd.EPDATASTATUS.value = TU_BIT(17);
      usbd.EVENTS_EPDATA      = 1;
      usbd.INTEN |= USBD_INTEN_EPDATA_Msk;
      interrupt();
      assert(usbd.TASKS_STARTEPOUT[1].triggers == 1 && _dcd.dma_running == TU_BIT(DMA_OUT_BASE + 1));
    };
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(quiet_delay_calls == 1 && usbd.TASKS_STARTEPOUT[1].triggers == 1 && epstall_writes.empty());
    return true;
  }
  if (!strcmp(name, "quiet-unrelated")) {
    quiet_setup();
    assert(dcd_edpt_abort(0, 3));
    dcd_edpt_clear_stall(0, 3);
    dcd_edpt_stall(0, 0x80);
    assert(!quiet_delay_calls && usbd.TASKS_EP0STALL.triggers == 1);
    return true;
  }
  if (!strcmp(name, "quiet-optout")) {
    quiet_setup();
    dcd_edpt_stall(0, 0x83);
    assert(!quiet_delay_calls);
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(!quiet_delay_calls && usbd.TASKS_EP0STATUS.triggers == 1);
    return true;
  }
  return false;
}
#endif

int main(int argc, char **argv) {
#if DCD_TEST_DRIVER_SETUP_TAGS
  if (argc == 2 && quiet_shape_test(argv[1])) { return 0; }
#endif
  assert(argc == 2);
  if (!strcmp(argv[1], "factored-setup-order")) {
    const uint8_t expected[8] = {0x80, TUSB_REQ_GET_DESCRIPTOR, 0x53, 0x17, 0x29, 0x42, 0x12, 0};
    for (unsigned busy = 0; busy < 3; busy++) {
      reset();
      Register *const registers[] = {&usbd.BMREQUESTTYPE, &usbd.BREQUEST, &usbd.WVALUEL,  &usbd.WVALUEH,
                                     &usbd.WINDEXL,       &usbd.WINDEXH,  &usbd.WLENGTHL, &usbd.WLENGTHH};
      for (unsigned i = 0; i < 8; i++) {
        registers[i]->value = (0xe1a50000u + (i << 8)) | expected[i];
      }
      if (busy) {
        begin_in(1);
        hardware_dma_busy = true;
      }
      usbd.EVENTS_EP0SETUP = 1;
      usbd.INTEN |= USBD_INTEN_EP0SETUP_Msk;
      bool injected = false;
      if (busy == 2) {
        on_event_read = [&](uint32_t *) {
          if (!injected && !usbd.EVENTS_EP0SETUP) {
            injected               = true;
            usbd.EVENTS_ENDEPIN[1] = 1;
            hardware_dma_busy      = false;
          }
        };
      }
      interrupt();
      assert(setup_reads_while_busy == 0);
      if (busy == 1) {
        assert(setup_events == 0 && setup_read_order.empty());
      } else {
        assert(setup_events == 1 && setup_read_order.size() == 8);
        for (unsigned i = 0; i < 8; i++) {
          assert(setup_read_order[i] == i);
        }
        assert(!memcmp(last_setup, expected, sizeof(expected)));
        if (busy == 2) {
          assert(injected);
        }
      }
    }
    return 0;
  }
  if (!strcmp(argv[1], "factored-errata-write")) {
    for (unsigned target = 0; target < 2; target++) {
      for (unsigned locked = 0; locked < 2; locked++) {
        reset();
        errata_unlock.value       = locked ? 0x12345678u : 0;
        MmioRegister *const reg   = target ? &errata_187 : &errata_171;
        const uint32_t      value = target ? 3 : 0xc0;
        usbd_errata_write(reg, value);
        assert(mmio_operations.size() == (locked ? 2u : 4u));
        assert(!mmio_operations[0].write && mmio_operations[0].address == 0x4006ec00);
        assert(mmio_operations[0].value == (locked ? 0x12345678u : 0));
        const unsigned at = locked ? 1 : 2;
        assert(mmio_operations[at].write && mmio_operations[at].address == reg->address &&
               mmio_operations[at].value == value);
        if (!locked) {
          assert(mmio_operations[1].write && mmio_operations[1].address == 0x4006ec00 &&
                 mmio_operations[1].value == 0x9375);
          assert(mmio_operations[3].write && mmio_operations[3].address == 0x4006ec00 &&
                 mmio_operations[3].value == 0x9375);
        }
      }
    }
    reset();
    anomaly_171 = anomaly_187 = true;
    usbd.ENABLE               = 0;
    tusb_hal_nrf_power_event(0);
    assert(usbd.ENABLE && errata_187.value == 3 && errata_171.value == 0xc0);
    assert(mmio_operations.size() == 6);
    assert(mmio_operations[2].address == 0x4006ed14 && mmio_operations[2].value == 3);
    assert(mmio_operations[5].address == 0x4006ec14 && mmio_operations[5].value == 0xc0);
    mmio_operations.clear();
    errata_unlock.value   = 0;
    usbd.EVENTCAUSE.value = USBD_EVENTCAUSE_READY_Msk;
    tusb_hal_nrf_power_event(2);
    assert(usbd.USBPULLUP && !errata_171.value && !errata_187.value);
    assert(mmio_operations.size() == 6);
    assert(mmio_operations[2].address == 0x4006ec14 && mmio_operations[2].value == 0);
    assert(mmio_operations[5].address == 0x4006ed14 && mmio_operations[5].value == 0);
    unsigned reads = 0;
    for (const MmioOperation &operation: mmio_operations) {
      if (!operation.write) {
        reads++;
      }
    }
    assert(reads == 2);
    return 0;
  }
  if (!strcmp(argv[1], "factored-irq")) {
    for (unsigned enabled = 0; enabled <= 1; enabled++) {
      reset();
      irq_enabled       = enabled;
      const bool before = dcd_int_save();
      assert(before == bool(enabled) && !irq_enabled);
      dcd_int_restore(before);
      assert(irq_enabled == bool(enabled));
      (void)dcd_int_save();
      usbd.ENABLE = 0;
      dcd_int_restore(true);
      assert(!irq_enabled);
    }
    return 0;
  }
  if (!strcmp(argv[1], "identity")) {
    for (bool affected: {false, true}) {
      reset(); anomaly = affected;
      begin_in(1);
      bool const expected = NRFX_USBD_ERRATA_ENABLE && affected;
      assert(usbd.TASKS_STARTEPIN[1].triggers == 1 && _dcd.dma_running);
      assert(errata_register.value == (expected ? 0x82u : 0));
      edpt_dma_end();
      assert(!errata_register.value && !_dcd.dma_running);
      assert(latch_writes.size() == (expected ? 2u : 0u));
    }
    return 0;
  }
  if (!strcmp(argv[1], "latch")) {
    reset();
    on_task_trigger = []() { assert(_dcd.dma_running && errata_register.value == 0x82); };
    begin_in(1);
    assert(_dcd.dma_running && errata_register.value == 0x82);
    assert(usbd.TASKS_STARTEPIN[1].triggers == 1);
    edpt_dma_end();
    assert(!_dcd.dma_running && !errata_register.value);
    assert((latch_writes == std::vector<uint32_t>{0x82, 0}));
    for (bool status: {false, true}) {
      reset(); setup_in(); interrupt();
      uint8_t buffer[4] = {};
      assert(dcd_edpt_xfer(0, 0, buffer, status ? 0 : sizeof(buffer)));
      assert(!_dcd.dma_running && latch_writes.empty());
      assert((status ? usbd.TASKS_EP0STATUS.triggers : usbd.TASKS_EP0RCVOUT.triggers) == 1);
    }
    reset(); anomaly = false;
    begin_out(2); edpt_dma_end();
    assert(latch_writes.empty());
    reset(); begin_out(2);
    usbd.INTEN = USBD_INTEN_USBRESET_Msk | (1u << (USBD_INTEN_ENDEPOUT0_Pos + 2));
    usbd.EVENTS_USBRESET = usbd.EVENTS_ENDEPOUT[2] = 1;
    interrupt();
    assert(!_dcd.dma_running && !errata_register.value);
    assert(transfer_events == 0 && usbd.ENABLE == 1);
    begin_in(1);
    hardware_dma_busy = true;
    usbd.ENABLE = 1;
    tusb_hal_nrf_power_event(1);
    assert(!_dcd.dma_running && !errata_register.value && unplugged_events == 1);
    unsigned const starts = usbd.TASKS_STARTEPIN[1].triggers;
    xact_in_dma(1); // A queued DMA cannot restart a disabled peripheral.
    assert(usbd.TASKS_STARTEPIN[1].triggers == starts && !_dcd.dma_running);
    return 0;
  }
  if (!strcmp(argv[1], "reset-timeout")) {
    reset(); begin_out(2);
    hardware_dma_busy = true;
    usbd.INTEN = USBD_INTEN_USBRESET_Msk;
    usbd.EVENTS_USBRESET = 1;
    interrupt();
    assert(!usbd.ENABLE && !_dcd.dma_running && !errata_register.value);
    assert(unplugged_events == 1 && setup_events == 0 && transfer_events == 0);
    assert(event_reads <= 25u + 2u * 21u * 25u);
    return 0;
  }
  if (!strcmp(argv[1], "removal-preempts-start")) {
    for (bool after: {false, true}) {
      reset();
      bool injected = false, power_pending = false;
      on_irq_restore = [&]() {
        if (power_pending) { power_pending = false; tusb_hal_nrf_power_event(1); }
      };
      on_task_trigger = []() { assert(usbd.ENABLE); };
      on_latch_write = [&](uint32_t value, bool assigned) {
        if (!injected && value == 0x82 && assigned == after) {
          injected = true;
          if (primask) power_pending = true;
          else tusb_hal_nrf_power_event(1);
        }
      };
      begin_in(1);
      assert(injected && !power_pending && !usbd.ENABLE && !errata_register.value);
      assert(!_dcd.dma_running);
      unsigned const starts = usbd.TASKS_STARTEPIN[1].triggers;
      begin_in(1); // Requests made after removal cannot trigger hardware.
      assert(usbd.TASKS_STARTEPIN[1].triggers == starts);
    }
    return 0;
  }
  if (!strcmp(argv[1], "timeout")) {
    reset(); begin_out(2);
    hardware_dma_busy = true;
    setup_in(); interrupt();
    assert(setup_events == 0 && setup_reads_while_busy == 0);
    assert(_dcd.dma_running && errata_register.value == 0x82);
    assert(event_reads <= 25u + 21u * 25u);
    return 0;
  }
  if (!strcmp(argv[1], "captured-end")) {
    reset(); begin_in(1);
    setup_in();
    usbd.EVENTS_ENDEPIN[1] = 1;
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPIN0_Pos + 1);
    interrupt();
    assert(setup_events == 1 && !memcmp(last_setup, "\x80\x06\0\x01\0\0\x12\0", 8));
    assert(!_dcd.dma_running && !errata_register.value && usbd.EVENTS_ENDEPIN[1] == 0);
    return 0;
  }
  if (!strcmp(argv[1], "late-end")) {
    reset(); begin_in(1);
    hardware_dma_busy = true;
    setup_in();
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPIN0_Pos + 1);
    bool injected = false;
    unsigned late_reads = 0;
    on_event_read = [&](uint32_t*) {
      if (!injected && !usbd.EVENTS_EP0SETUP && ++late_reads == 4) {
        injected = true;
        usbd.EVENTS_ENDEPIN[1] = 1;
        hardware_dma_busy = false;
      }
    };
    interrupt();
    assert(injected && setup_events == 1 && setup_reads_while_busy == 0);
    assert(usbd.EVENTS_ENDEPIN[1] == 1 && _dcd.dma_running);
    on_event_read = {}; interrupt();
    assert(!_dcd.dma_running && !errata_register.value && usbd.EVENTS_ENDEPIN[1] == 0);
    return 0;
  }
  if (!strcmp(argv[1], "other-end")) {
    reset(); begin_in(2);
    hardware_dma_busy = true;
    setup_in();
    usbd.EVENTS_ENDEPIN[1] = 1; // A stale END belongs to a different DMA owner.
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPIN0_Pos + 1);
    interrupt();
    assert(setup_events == 0 && setup_reads_while_busy == 0);
    assert(_dcd.dma_running && errata_register.value == 0x82);
    return 0;
  }
  if (!strcmp(argv[1], "captured-end-next-dma")) {
    reset(); begin_in(1);
    uint8_t next_buffer[32] = {};
    _dcd.xfer[2][TUSB_DIR_OUT].mps = 64;
    assert(dcd_edpt_xfer(0, 2, next_buffer, sizeof(next_buffer)));
    setup_in();
    usbd.SIZE.EPOUT[2] = sizeof(next_buffer);
    usbd.EPDATASTATUS.value = 1u << 18;
    usbd.EVENTS_ENDEPIN[1] = usbd.EVENTS_EPDATA = 1;
    usbd.INTEN |= (1u << (USBD_INTEN_ENDEPIN0_Pos + 1)) | USBD_INTEN_EPDATA_Msk;
    interrupt(); run_deferred_once();
    assert(setup_events == 1 && usbd.TASKS_STARTEPOUT[2].triggers == 1);
    assert(_dcd.dma_running && errata_register.value == 0x82);
    complete_in_dma(1); // Another old END cannot release the new OUT owner.
    assert(_dcd.dma_running && errata_register.value == 0x82);
    usbd.EPOUT[2].AMOUNT = sizeof(next_buffer);
    usbd.EVENTS_ENDEPOUT[2] = 1;
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPOUT0_Pos + 2);
    interrupt();
    assert(!_dcd.dma_running && !errata_register.value);
    assert(transfer_events == 1 && last_endpoint == 2 && last_length == sizeof(next_buffer));
    return 0;
  }
  if (!strcmp(argv[1], "setup-old-ep0")) {
    reset();
    uint8_t old_buffer[32] = {};
    assert(dcd_edpt_xfer(0, 0, old_buffer, sizeof(old_buffer)));
    usbd.SIZE.EPOUT[0] = sizeof(old_buffer);
    usbd.EVENTS_EP0DATADONE = 1;
    usbd.INTEN = USBD_INTEN_EP0DATADONE_Msk;
    interrupt(); run_deferred_once();
    assert(usbd.TASKS_STARTEPOUT[0].triggers == 1);
    usbd.EPOUT[0].AMOUNT = sizeof(old_buffer);
    setup_in();
    usbd.EVENTS_ENDEPOUT[0] = usbd.EVENTS_EP0DATADONE = 1;
    usbd.INTEN |= USBD_INTEN_ENDEPOUT0_Msk;
    interrupt();
    // A co-captured old OUT completion belongs to the superseded request.
    // Delivering it after SETUP would account old data against the new one.
    assert((delivered_events == std::vector<int>{-1}));
    assert(!_dcd.dma_running && !errata_register.value);
    return 0;
  }
  if (!strcmp(argv[1], "setup-old-ep0-in")) {
    reset();
    setup_in();
    interrupt();
    uint8_t old_buffer[2] = {};
    assert(dcd_edpt_xfer(0, 0x80, old_buffer, sizeof(old_buffer)));
    assert(usbd.TASKS_STARTEPIN[0].triggers == 1);
    delivered_events.clear();
    usbd.EPIN[0].AMOUNT = sizeof(old_buffer);
    setup_clear_out();
    usbd.EVENTS_ENDEPIN[0] = usbd.EVENTS_EP0DATADONE = 1;
    usbd.INTEN |= USBD_INTEN_ENDEPIN0_Msk | USBD_INTEN_EP0DATADONE_Msk;
    interrupt();
    assert((delivered_events == std::vector<int>{-1}));
    assert(transfer_events == 0 && !_dcd.dma_running && !errata_register.value);
    assert(_dcd.xfer[0][TUSB_DIR_OUT].data_received == 0);
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-stale-admission")) {
    reset();
    auto_admit_setup = false;
    setup_in();
    interrupt();
    const uint8_t old_tag = last_setup_tag;
    setup_clear_out();
    interrupt();
    assert(!admit_setup(old_tag));
    assert(control_tasks() == 0 && transfer_events == 0);
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-stale-status") || !strcmp(argv[1], "setup-stale-data-in") ||
      !strcmp(argv[1], "setup-stale-data-out") || !strcmp(argv[1], "setup-stale-stall")) {
    for (uint8_t stale_stall_endpoint: {uint8_t(0), uint8_t(0x80)}) {
      reset();
      auto_admit_setup = false;
      setup_in();
      interrupt();
      assert(admit_setup(last_setup_tag));
      setup_clear_out();
      interrupt();
      uint8_t old_buffer[2] = {};
      if (!strcmp(argv[1], "setup-stale-stall")) {
        dcd_edpt_stall(0, stale_stall_endpoint);
      } else if (!strcmp(argv[1], "setup-stale-status")) {
        (void)dcd_edpt_xfer(0, 0, nullptr, 0);
      } else {
        (void)dcd_edpt_xfer(0, !strcmp(argv[1], "setup-stale-data-in") ? 0x80 : 0, old_buffer, sizeof(old_buffer));
      }
      // The old task callback runs after the ISR received a new SETUP, but
      // before usbd admits the new request. It must not touch the controller.
      assert(control_tasks() == 0 && transfer_events == 0 && !_dcd.dma_running);
      assert(uint32_t(_dcd.dma_pending) == 0);
      fresh_control_status();
    }
    return 0;
  }
  if (!strcmp(argv[1], "setup-status-deferred")) {
    reset();
    setup_in();
    interrupt();
    begin_in(1); // A bulk DMA owns the shared engine when status is requested.
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(usbd.TASKS_EP0STATUS.triggers == 0 && transfer_events == 0);
    on_task_trigger = []() { assert(transfer_events == 0); };
    complete_in_dma(1);
    assert(usbd.TASKS_EP0STATUS.triggers == 1);
    assert(transfer_events == 1 && last_endpoint == 0 && last_length == 0);
    assert(last_completion_in_isr);
    return 0;
  }
  if (!strcmp(argv[1], "setup-status-before-next-setup")) {
    reset();
    setup_in();
    interrupt();
    delivered_events.clear();
    bool injected  = false;
    on_irq_restore = [&]() {
      if (!injected && usbd.TASKS_EP0STATUS.triggers) {
        injected = true;
        // A SETUP interrupt may run as soon as the task-commit lock opens.
        setup_clear_out();
        interrupt();
      }
    };
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(injected);
    // Posting completion after unlocking reverses these events: usbd would
    // apply the old status completion to its newly admitted request.
    assert((delivered_events == std::vector<int>{0, -1}));
    assert(transfer_events == 1 && setup_events == 2);
    assert(!last_completion_in_isr);
    return 0;
  }
  if (!strcmp(argv[1], "setup-status-cancel")) {
    reset();
    setup_in();
    interrupt();
    begin_in(1);
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(usbd.TASKS_EP0STATUS.triggers == 0 && transfer_events == 0);
    setup_clear_out();
    usbd.EVENTS_ENDEPIN[1] = 1;
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPIN0_Pos + 1);
    interrupt();
    assert(usbd.TASKS_EP0STATUS.triggers == 0 && transfer_events == 0);
    assert(uint32_t(_dcd.dma_pending) == 0 && !_dcd.dma_running);
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-raw-pending")) {
    reset();
    setup_in();
    interrupt();
    const uint8_t old_tag = last_setup_tag;
    setup_clear_out(); // The hardware latched SETUP, but its ISR has not run.
    uint8_t old_buffer[2] = {};
    (void)dcd_edpt_xfer(0, 0x80, old_buffer, sizeof(old_buffer));
    dcd_edpt_stall(0, 0);
    assert(!admit_setup(old_tag));
    assert(control_tasks() == 0 && transfer_events == 0 && !_dcd.dma_running);
    interrupt();
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-timeout-generation")) {
    reset();
    auto_admit_setup = false;
    setup_in();
    interrupt();
    const uint8_t old_tag = last_setup_tag;
    assert(admit_setup(old_tag));
    begin_out(2);
    hardware_dma_busy = true;
    setup_clear_out();
    interrupt();
    // Unreadable SETUP registers still mean that the previous request was
    // superseded. Clearing the event before timing out must not revive it.
    assert(setup_events == 1 && setup_reads_while_busy == 0);
    assert(!usbd.EVENTS_EP0SETUP && !admit_setup(old_tag));
    uint8_t old_buffer[2] = {};
    (void)dcd_edpt_xfer(0, 0, nullptr, 0);
    (void)dcd_edpt_xfer(0, 0x80, old_buffer, sizeof(old_buffer));
    (void)dcd_edpt_xfer(0, 0, old_buffer, sizeof(old_buffer));
    dcd_edpt_stall(0, 0);
    dcd_edpt_stall(0, 0x80);
    assert(control_tasks() == 0 && transfer_events == 0);
    assert(_dcd.dma_running && errata_register.value == 0x82);
    hardware_dma_busy       = false;
    usbd.EPOUT[2].AMOUNT    = 64;
    usbd.EVENTS_ENDEPOUT[2] = 1;
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPOUT0_Pos + 2);
    interrupt();
    assert(!_dcd.dma_running && !errata_register.value);
    setup_clear_out();
    interrupt();
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-no-data-in-status")) {
    reset();
    setup_in();
    usbd.WLENGTHL.value = 0;
    interrupt();
    // A zero-length IN request has an IN status stage, rather than a data ZLP.
    assert(dcd_edpt_xfer(0, 0x80, nullptr, 0));
    assert(usbd.TASKS_EP0STATUS.triggers == 1 && usbd.TASKS_STARTEPIN[0].triggers == 0);
    assert(transfer_events == 1 && last_endpoint == 0x80 && last_length == 0);
    return 0;
  }
  if (!strcmp(argv[1], "setup-stall-during-retirement")) {
    reset();
    setup_in();
    interrupt();
    uint8_t old_buffer[2] = {};
    assert(dcd_edpt_xfer(0, 0x80, old_buffer, sizeof(old_buffer)));
    hardware_dma_busy = true;
    bool injected     = false;
    on_event_read     = [&](uint32_t *event) {
      if (!injected && event == &usbd.EVENTS_ENDEPIN[0]) {
        injected = true;
        // SETUP latches while USBD IRQ is masked for bounded retirement.
        setup_clear_out();
        hardware_dma_busy      = false;
        usbd.EVENTS_ENDEPIN[0] = 1;
      }
    };
    dcd_edpt_stall(0, 0);
    on_event_read = {};
    assert(injected && usbd.EVENTS_EP0SETUP);
    assert(usbd.TASKS_EP0STALL.triggers == 0 && transfer_events == 0);
    assert(!_dcd.dma_running && !errata_register.value);
    interrupt();
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-unrelated-pending")) {
    reset();
    setup_in();
    interrupt();
    begin_in(1);
    begin_out(2);
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    setup_clear_out();
    usbd.EVENTS_ENDEPIN[1] = 1;
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPIN0_Pos + 1);
    interrupt();
    assert(usbd.TASKS_EP0STATUS.triggers == 0 && transfer_events == 0);
    assert(usbd.TASKS_STARTEPOUT[2].triggers == 1 && _dcd.dma_running);
    usbd.EPOUT[2].AMOUNT    = 64;
    usbd.EVENTS_ENDEPOUT[2] = 1;
    usbd.INTEN |= 1u << (USBD_INTEN_ENDEPOUT0_Pos + 2);
    interrupt();
    assert(transfer_events == 1 && last_endpoint == 2 && last_length == 64);
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-address-generation")) {
    reset();
    auto_admit_setup = false;
    setup_in();
    interrupt();
    const uint8_t old_tag = last_setup_tag;
    assert(admit_setup(old_tag));
    setup_clear_out();
    usbd.BMREQUESTTYPE.value = 0;
    usbd.BREQUEST.value      = TUSB_REQ_SET_ADDRESS;
    usbd.WVALUEL.value       = 7;
    interrupt();
    assert(setup_events == 1); // Hardware handles SET_ADDRESS without a core event.
    assert(!admit_setup(old_tag));
    dcd_edpt_stall(0, 0);
    assert(control_tasks() == 0 && transfer_events == 0);
    setup_clear_out();
    interrupt();
    fresh_control_status();
    return 0;
  }
  if (!strcmp(argv[1], "setup-reset-generation")) {
    for (unsigned boundary: {0u, 1u, 2u}) {
      reset();
      auto_admit_setup = false;
      setup_in();
      interrupt();
      const uint8_t old_tag = last_setup_tag;
      assert(admit_setup(old_tag));
      if (boundary == 0) {
        usbd.EVENTS_USBRESET = 1;
        usbd.INTEN |= USBD_INTEN_USBRESET_Msk;
        interrupt();
      } else if (boundary == 1) {
        dcd_disconnect(0);
      } else {
        tusb_hal_nrf_power_event(1);
      }
      assert(!admit_setup(old_tag));
      (void)dcd_edpt_xfer(0, 0, nullptr, 0);
      dcd_edpt_stall(0, 0);
      assert(control_tasks() == 0 && transfer_events == 0);
      // A new transport epoch can subsequently admit and serve fresh SETUP.
      usbd.ENABLE = 1;
      setup_clear_out();
      interrupt();
      fresh_control_status();
    }
    return 0;
  }
  if (!strcmp(argv[1], "direction")) {
    reset(); setup_in(); interrupt();
    assert(setup_events == 1);
    usbd.BMREQUESTTYPE.value = 0; // Hardware registers may no longer reflect the captured SETUP.
    assert(dcd_edpt_xfer(0, 0, nullptr, 0));
    assert(usbd.TASKS_EP0STATUS.triggers == 1 && usbd.TASKS_EP0RCVOUT.triggers == 0);
    reset(); setup_in(); interrupt();
    usbd.BMREQUESTTYPE.value = 0;
    uint8_t buffer[4] = {};
    _dcd.xfer[0][TUSB_DIR_IN].buffer = buffer;
    _dcd.xfer[0][TUSB_DIR_IN].total_len = 4;
    usbd.EPIN[0].AMOUNT = 4;
    usbd.EVENTS_EP0DATADONE = 1;
    usbd.INTEN = USBD_INTEN_EP0DATADONE_Msk;
    interrupt();
    assert(transfer_events == 1 && last_endpoint == 0x80 && last_length == 4);
    return 0;
  }
  // Regression cases exercise packet, pending-operation and hardware ownership
  // across endpoint halt, rearm, reset and physical removal.
  if (!strcmp(argv[1], "abort-pending")) {
    reset();
    begin_in(2);
    uint8_t old[128] = {}, sibling[32] = {};
    auto   *out        = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps           = 64;
    out->data_received = true;
    usbd.SIZE.EPOUT[1] = 32;
    assert(dcd_edpt_xfer(0, 1, old, sizeof(old)));
    _dcd.xfer[3][TUSB_DIR_IN].mps = 64;
    assert(dcd_edpt_xfer(0, 0x83, sibling, sizeof(sibling)));
    const uint32_t owner  = _dcd.dma_running;
    const uint32_t queued = TU_BIT(DMA_OUT_BASE + 1) | TU_BIT(DMA_IN_BASE + 3);
    assert(uint32_t(_dcd.dma_pending) == queued);
    usbd.EPDATASTATUS.value = TU_BIT(17) | TU_BIT(19);
    usbd.SIZE.EPOUT[3]      = 19;
    usbd.EVENTS_ENDEPOUT[3] = 1;
    usbd.EVENTS_ENDEPIN[2] = 1;
    assert(dcd_edpt_abort(0, 1));
    assert(_dcd.dma_running == owner && uint32_t(_dcd.dma_pending) == TU_BIT(DMA_IN_BASE + 3));
    assert(out->buffer == old && out->mps == 64 && !out->total_len && !out->actual_len && !out->data_received);
    assert(usbd.SIZE.EPOUT[1] == 0 && usbd.SIZE.EPOUT[3] == 19);
    assert(usbd.EPDATASTATUS.value == TU_BIT(19) && usbd.EVENTS_ENDEPOUT[3] == 1);
    assert(epstall_writes.empty() && dtoggle_writes.empty() && transfer_events == 0);
    complete_in_dma(2);
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0 && usbd.TASKS_STARTEPIN[3].triggers == 1);
    assert(usbd.EPIN[3].PTR == (uintptr_t)sibling);
    return 0;
  }

  if (!strcmp(argv[1], "abort-sibling-dma")) {
    for (unsigned owner_ep: {0u, 2u}) {
      for (unsigned owner_dir = 0; owner_dir < 2; ++owner_dir) {
        for (const bool enabled: {false, true}) {
          reset();
          if (owner_dir == TUSB_DIR_IN) {
            _dcd.setup_dir = TUSB_DIR_IN;
            begin_in(owner_ep);
          } else {
            begin_out(owner_ep);
            if (!owner_ep) {
              usbd.EVENTS_EP0DATADONE = 1;
              usbd.INTEN |= USBD_INTEN_EP0DATADONE_Msk;
              interrupt();
            }
          }
          const uint32_t owner = TU_BIT((owner_dir == TUSB_DIR_IN ? DMA_IN_BASE : DMA_OUT_BASE) + owner_ep);
          assert(_dcd.dma_running == owner);
          auto      *active  = &_dcd.xfer[owner_ep][owner_dir];
          const auto before  = *active;
          uint8_t    old[64] = {}, sibling[32] = {};
          auto      *out     = &_dcd.xfer[1][TUSB_DIR_OUT];
          out->mps           = 64;
          out->data_received = true;
          usbd.SIZE.EPOUT[1] = sizeof(old);
          assert(dcd_edpt_xfer(0, 1, old, sizeof(old)));
          _dcd.xfer[3][TUSB_DIR_IN].mps = 64;
          assert(dcd_edpt_xfer(0, 0x83, sibling, sizeof(sibling)));
          assert(uint32_t(_dcd.dma_pending) == (TU_BIT(DMA_OUT_BASE + 1) | TU_BIT(DMA_IN_BASE + 3)));
          uint32_t *const raw_end =
            owner_dir == TUSB_DIR_IN ? &usbd.EVENTS_ENDEPIN[owner_ep] : &usbd.EVENTS_ENDEPOUT[owner_ep];
          usbd.EVENTS_ENDEPOUT[1] = 1; // An unrelated raw END cannot satisfy the global owner wait.
          usbd.EPDATASTATUS.value = TU_BIT(17) | TU_BIT(19);
          usbd.SIZE.EPOUT[3]      = 19;
          hardware_dma_busy       = true;
          irq_enabled             = enabled;
          unsigned polls = 0, flushes = 0;
          on_event_read = [&](uint32_t *) {
            assert(!primask && !irq_enabled && !flushes);
            if (++polls == 3) {
              hardware_dma_busy                                                              = false;
              *raw_end                                                                       = 1;
              (owner_dir == TUSB_DIR_IN ? usbd.EPIN[owner_ep] : usbd.EPOUT[owner_ep]).AMOUNT = 64;
            }
          };
          on_register_write = [&](const Register *reg, uint32_t value) {
            if (reg != &usbd.EPDATASTATUS) {
              return;
            }
            assert(!hardware_dma_busy && *raw_end == 1);
            assert(primask == 1 && !irq_enabled && usbd.ENABLE && _dcd.dma_running == owner);
            assert(value == TU_BIT(17) && !usbd.EVENTS_ENDEPOUT[1]);
            ++flushes;
          };
          assert(dcd_edpt_abort(0, 1));
          on_register_write = {};
          on_event_read     = {};
          assert(polls >= 3 && flushes == 1 && irq_enabled == enabled && !primask);
          assert(*raw_end == 1 && _dcd.dma_running == owner);
          assert(uint32_t(_dcd.dma_pending) == TU_BIT(DMA_IN_BASE + 3));
          assert(active->buffer == before.buffer && active->total_len == before.total_len &&
                 active->actual_len == before.actual_len && active->mps == before.mps &&
                 active->data_received == before.data_received &&
                 active->iso_in_transfer_ready == before.iso_in_transfer_ready);
          assert(out->buffer == old && out->mps == 64 && !out->total_len && !out->actual_len && !out->data_received);
          assert(!usbd.EVENTS_ENDEPOUT[1] && !usbd.SIZE.EPOUT[1]);
          assert(usbd.EPDATASTATUS.value == TU_BIT(19) && usbd.SIZE.EPOUT[3] == 19);
          assert(transfer_events == 0 && epstall_writes.empty() && dtoggle_writes.empty());
          usbd.INTEN |= owner | TU_BIT(DMA_OUT_BASE + 1);
          interrupt();
          assert(*raw_end == 0 && usbd.TASKS_STARTEPIN[3].triggers == 1);
          assert(usbd.EPIN[3].PTR == (uintptr_t)sibling && usbd.EPIN[3].MAXCNT == sizeof(sibling));
          if (owner_dir == TUSB_DIR_IN) {
            if (!owner_ep) {
              usbd.EVENTS_EP0DATADONE = 1;
              usbd.INTEN |= USBD_INTEN_EP0DATADONE_Msk;
            } else {
              usbd.EPDATASTATUS.value = TU_BIT(owner_ep);
              usbd.EVENTS_EPDATA      = 1;
              usbd.INTEN |= USBD_INTEN_EPDATA_Msk;
            }
            interrupt();
          }
          assert(transfer_events == 1 && last_endpoint == (owner_ep | (owner_dir << 7)) && last_length == 64);
          interrupt();
          assert(transfer_events == 1);
        }
      }
    }
    return 0;
  }
  if (!strcmp(argv[1], "abort-sibling-timeout")) {
    reset();
    begin_in(0);
    uint8_t old[64]    = {};
    auto   *out        = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps           = 64;
    out->data_received = true;
    usbd.SIZE.EPOUT[1] = sizeof(old);
    assert(dcd_edpt_xfer(0, 1, old, sizeof(old)));
    hardware_dma_busy = true;
    unsigned flushes  = 0;
    on_register_write = [&](const Register *reg, uint32_t) {
      if (reg == &usbd.EPDATASTATUS) {
        ++flushes;
      }
    };
    on_event_read = [&](uint32_t *) { assert(!primask && !flushes); };
    assert(!dcd_edpt_abort(0, 1));
    on_register_write = {};
    on_event_read     = {};
    assert(!usbd.ENABLE && !irq_enabled && !primask && unplugged_events == 1 && !flushes);
    assert(out->buffer == old && out->mps == 64 && out->total_len == sizeof(old) && !out->actual_len);
    assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending) && !errata_register.value);
    assert(epstall_writes.empty() && dtoggle_writes.empty() && transfer_events == 0);
    assert(event_reads > 0 && event_reads <= 21u * 21u * 2u);
    return 0;
  }
  if (!strcmp(argv[1], "abort-dma-power") || !strcmp(argv[1], "abort-power-before-commit")) {
    reset();
    begin_in(2);
    uint8_t old[64]    = {};
    auto   *out        = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps           = 64;
    out->data_received = true;
    usbd.SIZE.EPOUT[1] = sizeof(old);
    assert(dcd_edpt_xfer(0, 1, old, sizeof(old)));
    const uint8_t generation = _ep0.received;
    hardware_dma_busy        = true;
    bool       removed       = false;
    unsigned   mask_reads = 0, flushes = 0;
    const auto remove = [&]() {
      if (removed) {
        return;
      }
      removed = true;
      assert(!primask && !flushes);
      hardware_dma_busy      = false;
      usbd.EVENTS_ENDEPIN[2] = 1;
      tusb_hal_nrf_power_event(1);
      usbd.ENABLE = 1;
    };
    on_register_write = [&](const Register *reg, uint32_t) {
      if (reg == &usbd.EPDATASTATUS) {
        ++flushes;
      }
    };
    if (!strcmp(argv[1], "abort-dma-power")) {
      on_event_read = [&](uint32_t *) { remove(); };
    } else {
      hardware_dma_busy      = false;
      usbd.EVENTS_ENDEPIN[2] = 1;
      on_primask_read        = [&]() {
        if (++mask_reads == 2) {
          remove();
        }
      };
    }
    assert(!dcd_edpt_abort(0, 1));
    on_register_write = {};
    on_event_read     = {};
    on_primask_read   = {};
    assert(removed && _ep0.received == uint8_t(generation + 1) && usbd.ENABLE && !primask);
    assert(unplugged_events == 1 && !flushes && transfer_events == 0);
    assert(out->buffer == old && out->mps == 64 && out->total_len == sizeof(old) && !out->actual_len);
    assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending));
    assert(epstall_writes.empty() && dtoggle_writes.empty());
    return 0;
  }
  if (!strcmp(argv[1], "abort-active")) {
    for (const bool enabled: {false, true}) {
      reset();
      uint8_t old[128]   = {};
      auto   *out        = &_dcd.xfer[1][TUSB_DIR_OUT];
      out->mps           = 64;
      out->data_received = true;
      usbd.SIZE.EPOUT[1] = 32;
      assert(dcd_edpt_xfer(0, 1, old, sizeof(old)));
      hardware_dma_busy = true;
      irq_enabled       = enabled;
      bool     ended    = false;
      unsigned polls    = 0;
      on_event_read     = [&](uint32_t *) {
        if (!ended && ++polls == 3) {
          ended                   = true;
          hardware_dma_busy       = false;
          usbd.EPOUT[1].AMOUNT    = 32;
          usbd.EVENTS_ENDEPOUT[1] = 1;
        }
      };
      assert(dcd_edpt_abort(0, 1));
      on_event_read = {};
      assert(ended && !hardware_dma_busy && irq_enabled == enabled);
      assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending) && !errata_register.value);
      assert(out->buffer == old && out->mps == 64 && !out->total_len && !out->actual_len);
      assert(!usbd.EVENTS_ENDEPOUT[1] && !usbd.SIZE.EPOUT[1]);
      assert(epstall_writes.empty() && dtoggle_writes.empty() && transfer_events == 0);
      usbd.INTEN = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + 1);
      interrupt();
      assert(transfer_events == 0 && !out->actual_len);
    }
    return 0;
  }
  if (!strcmp(argv[1], "abort-cached")) {
    reset();
    auto *out               = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps                = 64;
    usbd.SIZE.EPOUT[1]      = 31;
    usbd.EPDATASTATUS.value = TU_BIT(17);
    usbd.EVENTS_EPDATA      = 1;
    usbd.INTEN              = USBD_INTEN_EPDATA_Msk;
    interrupt();
    assert(out->data_received == 1 && !_dcd.dma_running);
    assert(dcd_edpt_abort(0, 1));
    assert(!out->data_received && !out->total_len && !out->actual_len && !usbd.SIZE.EPOUT[1]);
    // During RESET_PENDING, more accepted data can only latch; it cannot DMA.
    usbd.SIZE.EPOUT[1]      = 17;
    usbd.EPDATASTATUS.value = TU_BIT(17);
    usbd.EVENTS_EPDATA      = 1;
    interrupt();
    assert(out->data_received == 1 && usbd.TASKS_STARTEPOUT[1].triggers == 0);
    assert(dcd_edpt_abort(0, 1));
    assert(!out->data_received && !usbd.SIZE.EPOUT[1] && !usbd.EPDATASTATUS.value);
    uint8_t fresh[64] = {};
    assert(dcd_edpt_xfer(0, 1, fresh, sizeof(fresh)));
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0);
    usbd.SIZE.EPOUT[1]      = 23;
    usbd.EPDATASTATUS.value = TU_BIT(17);
    usbd.EVENTS_EPDATA      = 1;
    interrupt();
    assert(usbd.EPOUT[1].PTR == (uintptr_t)fresh && usbd.EPOUT[1].MAXCNT == 23);
    memset((void *)usbd.EPOUT[1].PTR, 0x56, 23);
    usbd.EPOUT[1].AMOUNT    = 23;
    usbd.EVENTS_ENDEPOUT[1] = 1;
    usbd.INTEN |= TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + 1);
    interrupt();
    assert(transfer_events == 1 && last_endpoint == 1 && last_length == 23);
    for (unsigned i = 0; i < 23; i++) {
      assert(fresh[i] == 0x56);
    }
    assert(epstall_writes.empty() && dtoggle_writes.empty());
    return 0;
  }
  if (!strcmp(argv[1], "abort-stale-end")) {
    reset();
    uint8_t old[64] = {}, fresh[64] = {};
    auto   *out    = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->buffer    = old;
    out->mps       = 64;
    out->total_len = out->actual_len = 31;
    out->data_received               = true;
    usbd.EPOUT[1].AMOUNT             = 31;
    usbd.EVENTS_ENDEPOUT[1] = usbd.EVENTS_EPDATA = 1;
    usbd.EPDATASTATUS.value                      = TU_BIT(17);
    usbd.SIZE.EPOUT[1]                           = 31;
    assert(dcd_edpt_abort(0, 1));
    assert(dcd_edpt_xfer(0, 1, fresh, sizeof(fresh)));
    usbd.INTEN = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + 1) | USBD_INTEN_EPDATA_Msk;
    interrupt();
    // A raw END without an active owner must not complete the fresh request.
    assert(transfer_events == 0 && out->buffer == fresh && out->actual_len == 0);
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0 && !_dcd.dma_running);
    assert(epstall_writes.empty() && dtoggle_writes.empty());
    return 0;
  }
  if (!strcmp(argv[1], "abort-halted")) {
    reset();
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps  = 64;
    dcd_edpt_stall(0, 1);
    assert(out->data_received == OUT_HALTED && usbd.HALTED.EPOUT[1]);
    const auto stalls       = epstall_writes;
    usbd.SIZE.EPOUT[1]      = 32;
    usbd.EPDATASTATUS.value = TU_BIT(17) | TU_BIT(19);
    assert(dcd_edpt_abort(0, 1));
    assert(out->data_received == OUT_HALTED && usbd.HALTED.EPOUT[1]);
    assert(out->mps == 64 && !out->total_len && !out->actual_len && !usbd.SIZE.EPOUT[1]);
    assert(usbd.EPDATASTATUS.value == TU_BIT(19));
    assert(epstall_writes == stalls && dtoggle_writes.empty());
    return 0;
  }
  if (!strcmp(argv[1], "abort-bounds-irq")) {
    for (const bool enabled: {false, true}) {
      reset();
      irq_enabled             = enabled;
      usbd.EPDATASTATUS.value = TU_BIT(17);
      usbd.SIZE.EPOUT[1]      = 31;
      unsigned char before_dcd[sizeof(_dcd)], before_usbd[sizeof(usbd)];
      memcpy(before_dcd, &_dcd, sizeof(_dcd));
      memcpy(before_usbd, &usbd, sizeof(usbd));
      for (const uint8_t address: {0, 0x80, 0x81, 8, 0x88, 9, 0x89, 15, 0x8f, 0x11, 0x71, 0xff}) {
        assert(!dcd_edpt_abort(0, address));
        assert(!memcmp(before_dcd, &_dcd, sizeof(_dcd)) && !memcmp(before_usbd, &usbd, sizeof(usbd)));
        assert(irq_enabled == enabled && !barriers);
      }
      assert(dcd_edpt_abort(0, 1));
      assert(irq_enabled == enabled && !usbd.EPDATASTATUS.value && !usbd.SIZE.EPOUT[1]);
      assert(epstall_writes.empty() && dtoggle_writes.empty());
    }
    return 0;
  }
  if (!strcmp(argv[1], "abort-timeout")) {
    reset();
    begin_out(1);
    hardware_dma_busy = true;
    assert(!dcd_edpt_abort(0, 1));
    assert(!usbd.ENABLE && !irq_enabled && unplugged_events == 1);
    assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending) && !errata_register.value);
    assert(transfer_events == 0 && epstall_writes.empty() && dtoggle_writes.empty());
    return 0;
  }
  if (!strcmp(argv[1], "abort-disabled")) {
    for (const bool enabled: {false, true}) {
      reset();
      uint8_t old[64] = {};
      auto   *out     = &_dcd.xfer[1][TUSB_DIR_OUT];
      out->buffer     = old;
      out->mps = out->total_len = 64;
      out->actual_len           = 7;
      out->data_received        = true;
      _dcd.dma_running          = TU_BIT(DMA_OUT_BASE + 1);
      _dcd.dma_pending          = TU_BIT(DMA_OUT_BASE + 3);
      usbd.SIZE.EPOUT[1]        = 31;
      usbd.EPDATASTATUS.value   = TU_BIT(17);
      usbd.EVENTS_ENDEPOUT[1]   = 1;
      usbd.ENABLE               = 0;
      irq_enabled               = enabled;
      unsigned char before_dcd[sizeof(_dcd)], before_usbd[sizeof(usbd)];
      memcpy(before_dcd, &_dcd, sizeof(_dcd));
      memcpy(before_usbd, &usbd, sizeof(usbd));
      assert(!dcd_edpt_abort(0, 1));
      assert(!irq_enabled && !memcmp(before_dcd, &_dcd, sizeof(_dcd)));
      // Shared IRQ restoration orders prior accesses even when USB is off.
      assert(!memcmp(before_usbd, &usbd, sizeof(usbd)) && barriers == 2);
      assert(epstall_writes.empty() && dtoggle_writes.empty() && transfer_events == 0);
    }
    return 0;
  }
  if (!strcmp(argv[1], "clear-active-out") || !strcmp(argv[1], "clear-pending-out")) {
    reset();
    const bool pending = !strcmp(argv[1], "clear-pending-out");
    if (pending) {
      begin_in(2);
    }
    uint8_t buffer[128] = {};
    auto   *out         = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps            = 64;
    out->data_received  = true;
    usbd.SIZE.EPOUT[1]  = 64;
    assert(dcd_edpt_xfer(0, 1, buffer, sizeof(buffer)));
    const uint32_t owner  = _dcd.dma_running;
    const uint32_t queued = _dcd.dma_pending;
    assert(owner == TU_BIT(pending ? DMA_IN_BASE + 2 : DMA_OUT_BASE + 1));
    assert(queued == (pending ? TU_BIT(DMA_OUT_BASE + 1) : 0));
    const unsigned starts   = usbd.TASKS_STARTEPOUT[1].triggers;
    const uint32_t status   = TU_BIT(17) | TU_BIT(19);
    usbd.EPDATASTATUS.value = status;
    assert(!usbd.HALTED.EPOUT[1] && !out->data_received);
    dcd_edpt_clear_stall(0, 1);
    assert(out->buffer == buffer && out->total_len == sizeof(buffer) && !out->actual_len);
    assert(out->mps == 64 && !out->data_received);
    assert(_dcd.dma_running == owner && uint32_t(_dcd.dma_pending) == queued);
    assert(usbd.EPDATASTATUS.value == status && usbd.SIZE.EPOUT[1] == 64);
    assert(usbd.TASKS_STARTEPOUT[1].triggers == starts && transfer_events == 0);
    assert((dtoggle_writes == std::vector<uint32_t>{1, 0x101}));
    assert((epstall_writes == std::vector<uint32_t>{1}) && !usbd.HALTED.EPOUT[1]);
    // Complete the original two-packet request, not a replacement descriptor.
    if (pending) {
      complete_in_dma(2);
    }
    assert(usbd.EPOUT[1].PTR == (uintptr_t)buffer && usbd.EPOUT[1].MAXCNT == 64);
    usbd.INTEN = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + 1);
    memset((void *)usbd.EPOUT[1].PTR, 0x51, 64);
    usbd.EPOUT[1].AMOUNT    = 64;
    usbd.EVENTS_ENDEPOUT[1] = 1;
    interrupt();
    assert(out->buffer == buffer + 64 && out->actual_len == 64 && transfer_events == 0);
    usbd.EPDATASTATUS.value = TU_BIT(17);
    usbd.EVENTS_EPDATA      = 1;
    usbd.INTEN |= USBD_INTEN_EPDATA_Msk;
    interrupt();
    assert(usbd.EPOUT[1].PTR == (uintptr_t)(buffer + 64) && usbd.EPOUT[1].MAXCNT == 64);
    memset((void *)usbd.EPOUT[1].PTR, 0x52, 64);
    usbd.EPOUT[1].AMOUNT    = 64;
    usbd.EVENTS_ENDEPOUT[1] = 1;
    interrupt();
    assert(transfer_events == 1 && last_endpoint == 1 && last_length == sizeof(buffer));
    for (unsigned i = 0; i < sizeof(buffer); i++) {
      assert(buffer[i] == (i < 64 ? 0x51 : 0x52));
    }
    return 0;
  }
  if (!strcmp(argv[1], "clear-received-out")) {
    reset();
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps  = 64;
    // A real EPDATA interrupt caches a packet before a client supplies a buffer.
    usbd.SIZE.EPOUT[1]      = 17;
    usbd.EPDATASTATUS.value = TU_BIT(17);
    usbd.EVENTS_EPDATA      = 1;
    usbd.INTEN              = USBD_INTEN_EPDATA_Msk;
    interrupt();
    assert(out->data_received == 1 && !_dcd.dma_running && !uint32_t(_dcd.dma_pending));
    usbd.EPDATASTATUS.value = TU_BIT(17) | TU_BIT(19);
    dcd_edpt_clear_stall(0, 1);
    assert(out->data_received == 1 && !out->buffer && !out->total_len && !out->actual_len && out->mps == 64);
    assert(usbd.SIZE.EPOUT[1] == 17 && usbd.EPDATASTATUS.value == (TU_BIT(17) | TU_BIT(19)));
    assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending));
    assert((dtoggle_writes == std::vector<uint32_t>{1, 0x101}));
    uint8_t buffer[64] = {};
    assert(dcd_edpt_xfer(0, 1, buffer, sizeof(buffer)));
    assert(usbd.EPOUT[1].PTR == (uintptr_t)buffer && usbd.EPOUT[1].MAXCNT == 17);
    usbd.INTEN = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + 1);
    memset((void *)usbd.EPOUT[1].PTR, 0x53, 17);
    usbd.EPOUT[1].AMOUNT    = 17;
    usbd.EVENTS_ENDEPOUT[1] = 1;
    interrupt();
    assert(transfer_events == 1 && last_endpoint == 1 && last_length == 17);
    for (unsigned i = 0; i < 17; i++) {
      assert(buffer[i] == 0x53);
    }
    return 0;
  }
  if (!strcmp(argv[1], "clear-halted-out")) {
    reset();
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps  = 64;
    dcd_edpt_stall(0, 1);
    assert(usbd.HALTED.EPOUT[1] == 1 && out->data_received == OUT_HALTED);
    usbd.SIZE.EPOUT[1]      = 32;
    usbd.SIZE.EPOUT[3]      = 19;
    usbd.EPDATASTATUS.value = TU_BIT(17) | TU_BIT(19);
    dcd_edpt_clear_stall(0, 1);
    assert(!usbd.HALTED.EPOUT[1] && !out->data_received && usbd.SIZE.EPOUT[1] == 0);
    assert(usbd.EPDATASTATUS.value == TU_BIT(19) && usbd.SIZE.EPOUT[3] == 19);
    assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending) && transfer_events == 0);
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0);
    assert((dtoggle_writes == std::vector<uint32_t>{1, 0x101}));
    assert((epstall_writes == std::vector<uint32_t>{0x101, 1}));
    return 0;
  }

  if (!strcmp(argv[1], "stall-all-dma")) {
    for (unsigned target_dir = 0; target_dir < 2; ++target_dir) {
      for (unsigned owner_ep: {0u, 2u}) {
        for (unsigned owner_dir = 0; owner_dir < 2; ++owner_dir) {
          for (const bool enabled: {false, true}) {
            reset();
            if (owner_dir == TUSB_DIR_IN) {
              _dcd.setup_dir = TUSB_DIR_IN;
              begin_in(owner_ep);
            } else {
              begin_out(owner_ep);
              if (!owner_ep) {
                usbd.EVENTS_EP0DATADONE = 1;
                usbd.INTEN |= USBD_INTEN_EP0DATADONE_Msk;
                interrupt();
              }
            }
            const uint32_t owner = TU_BIT((owner_dir == TUSB_DIR_IN ? DMA_IN_BASE : DMA_OUT_BASE) + owner_ep);
            assert(_dcd.dma_running == owner);
            auto      *active             = &_dcd.xfer[owner_ep][owner_dir];
            const auto before             = *active;
            uint8_t    sibling[32]        = {};
            _dcd.xfer[3][TUSB_DIR_IN].mps = 64;
            assert(dcd_edpt_xfer(0, 0x83, sibling, sizeof(sibling)));
            const uint32_t pending = _dcd.dma_pending;
            assert(pending == TU_BIT(DMA_IN_BASE + 3));
            uint32_t *const raw_end =
              owner_dir == TUSB_DIR_IN ? &usbd.EVENTS_ENDEPIN[owner_ep] : &usbd.EVENTS_ENDEPOUT[owner_ep];
            hardware_dma_busy = true;
            irq_enabled       = enabled;
            unsigned polls    = 0;
            on_event_read     = [&](uint32_t *) {
              assert(!primask && !irq_enabled);
              // Own-OUT retirement can re-observe the same already-finished sibling.
              assert(epstall_writes.empty() || (!hardware_dma_busy && *raw_end == 1));
              if (++polls == 3) {
                hardware_dma_busy                                                              = false;
                *raw_end                                                                       = 1;
                (owner_dir == TUSB_DIR_IN ? usbd.EPIN[owner_ep] : usbd.EPOUT[owner_ep]).AMOUNT = 64;
              }
            };
            const uint8_t target = 1 | (target_dir << 7);
            on_epstall_write     = [&](uint32_t value) {
              assert(!hardware_dma_busy && *raw_end == 1);
              assert(primask == 1 && !irq_enabled && usbd.ENABLE && _dcd.dma_running == owner);
              assert(value == (0x100u | target));
            };
            dcd_edpt_stall(0, target);
            on_epstall_write = {};
            on_event_read    = {};
            assert(polls >= 3 && irq_enabled == enabled && !primask);
            assert(epstall_writes == std::vector<uint32_t>{0x100u | target});
            assert(*raw_end == 1 && _dcd.dma_running == owner && uint32_t(_dcd.dma_pending) == pending);
            assert(active->buffer == before.buffer && active->total_len == before.total_len &&
                   active->actual_len == before.actual_len && active->mps == before.mps &&
                   active->data_received == before.data_received &&
                   active->iso_in_transfer_ready == before.iso_in_transfer_ready);
            assert(transfer_events == 0 && dtoggle_writes.empty());
            assert((target_dir == TUSB_DIR_IN ? usbd.HALTED.EPIN[1] : usbd.HALTED.EPOUT[1]) == 1);
            if (target_dir == TUSB_DIR_OUT) {
              assert(_dcd.xfer[1][target_dir].data_received == OUT_HALTED);
            }
            usbd.INTEN |= owner;
            interrupt();
            assert(*raw_end == 0 && usbd.TASKS_STARTEPIN[3].triggers == 1);
            assert(usbd.EPIN[3].PTR == (uintptr_t)sibling && usbd.EPIN[3].MAXCNT == sizeof(sibling));
            if (owner_dir == TUSB_DIR_IN) {
              if (!owner_ep) {
                usbd.EVENTS_EP0DATADONE = 1;
                usbd.INTEN |= USBD_INTEN_EP0DATADONE_Msk;
              } else {
                usbd.EPDATASTATUS.value = TU_BIT(owner_ep);
                usbd.EVENTS_EPDATA      = 1;
                usbd.INTEN |= USBD_INTEN_EPDATA_Msk;
              }
              interrupt();
            }
            assert(transfer_events == 1 && last_endpoint == (owner_ep | (owner_dir << 7)) && last_length == 64);
            interrupt();
            assert(transfer_events == 1); // The observed END must reach normal accounting once.
          }
        }
      }
    }
    return 0;
  }
  if (!strcmp(argv[1], "stall-dma-timeout")) {
    for (unsigned dir = 0; dir < 2; ++dir) {
      reset();
      begin_in(0);
      hardware_dma_busy = true;
      on_event_read     = [&](uint32_t *) { assert(!primask && epstall_writes.empty()); };
      dcd_edpt_stall(0, 1 | (dir << 7));
      on_event_read = {};
      assert(!usbd.ENABLE && !irq_enabled && !primask && unplugged_events == 1);
      assert(epstall_writes.empty() && dtoggle_writes.empty() && transfer_events == 0);
      assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending) && !errata_register.value);
      assert(event_reads > 0 && event_reads <= 21u * 21u * 2u);
    }
    return 0;
  }
  if (!strcmp(argv[1], "stall-dma-power") || !strcmp(argv[1], "stall-power-before-commit")) {
    for (unsigned dir = 0; dir < 2; ++dir) {
      reset();
      begin_in(0);
      hardware_dma_busy           = true;
      const uint8_t generation    = _ep0.received;
      bool          removed       = false;
      unsigned      primask_reads = 0;
      const auto    remove        = [&]() {
        if (removed) {
          return;
        }
        removed = true;
        assert(!primask && epstall_writes.empty());
        hardware_dma_busy      = false;
        usbd.EVENTS_ENDEPIN[0] = 1;
        tusb_hal_nrf_power_event(1);
        usbd.ENABLE = 1; // Removal then a fresh power epoch before the stale call resumes.
      };
      if (!strcmp(argv[1], "stall-dma-power")) {
        on_event_read = [&](uint32_t *) { remove(); };
      } else {
        hardware_dma_busy      = false;
        usbd.EVENTS_ENDEPIN[0] = 1;
        on_primask_read        = [&]() {
          if (++primask_reads == 2) {
            remove(); // Skip dma_cancel's own short critical section.
          }
        };
      }
      dcd_edpt_stall(0, 1 | (dir << 7));
      on_event_read   = {};
      on_primask_read = {};
      assert(removed && _ep0.received == uint8_t(generation + 1));
      assert(usbd.ENABLE && !primask && unplugged_events == 1);
      assert(epstall_writes.empty() && dtoggle_writes.empty() && transfer_events == 0);
      assert(!_dcd.dma_running && !uint32_t(_dcd.dma_pending));
      assert(_dcd.xfer[1][TUSB_DIR_OUT].data_received != OUT_HALTED);
    }
    return 0;
  }
  if (!strcmp(argv[1], "stall-disabled")) {
    for (unsigned previous_mask = 0; previous_mask < 2; ++previous_mask) {
      for (const bool previous_irq: {false, true}) {
        for (const bool controller_enabled: {false, true}) {
          reset();
          primask          = previous_mask;
          irq_enabled      = previous_irq;
          usbd.ENABLE      = controller_enabled;
          on_epstall_write = [&](uint32_t) { assert(primask == 1 && !irq_enabled); };
          dcd_edpt_stall(0, 0x81);
          on_epstall_write = {};
          assert(primask == previous_mask && irq_enabled ==
                 (previous_irq && controller_enabled && !(CFG_TUD_NRF5X_CONTROL_STALL_QUIET && previous_mask)));
          assert(unplugged_events == unsigned(CFG_TUD_NRF5X_CONTROL_STALL_QUIET && previous_mask && controller_enabled));
          assert(dtoggle_writes.empty());
          assert(epstall_writes.size() == unsigned(controller_enabled));
          assert(!_dcd.xfer[1][TUSB_DIR_OUT].data_received);
        }
      }
    }
    return 0;
  }
  if (!strcmp(argv[1], "clear-active-in")) {
    for (const bool stalled: {false, true}) {
      reset();
      begin_in(1);
      auto          *in     = &_dcd.xfer[1][TUSB_DIR_IN];
      const auto     before = *in;
      const uint32_t owner  = _dcd.dma_running;
      if (stalled) {
        usbd.EVENTS_ENDEPIN[1] = 1; // END is observable but still owned by the ISR.
        dcd_edpt_stall(0, 0x81);
      }
      assert(usbd.HALTED.EPIN[1] == unsigned(stalled));
      usbd.EPDATASTATUS.value = TU_BIT(17);
      usbd.SIZE.EPOUT[1]      = 31;
      dcd_edpt_clear_stall(0, 0x81);
      assert(in->buffer == before.buffer && in->total_len == before.total_len && in->actual_len == before.actual_len);
      assert(in->mps == before.mps && in->data_received == before.data_received);
      assert(_dcd.dma_running == owner && !uint32_t(_dcd.dma_pending));
      assert(usbd.SIZE.EPOUT[1] == 31 && usbd.EPDATASTATUS.value == TU_BIT(17));
      assert(!usbd.HALTED.EPIN[1] && transfer_events == 0);
      assert((dtoggle_writes == std::vector<uint32_t>{0x81, 0x181}));
      assert(epstall_writes.back() == 0x81);
      complete_in_dma(1);
      usbd.EPIN[1].AMOUNT = 64;
      usbd.EPDATASTATUS.value = TU_BIT(1);
      usbd.EVENTS_EPDATA      = 1;
      usbd.INTEN |= USBD_INTEN_EPDATA_Msk;
      interrupt();
      assert(transfer_events == 1 && last_endpoint == 0x81 && last_length == 64);
    }
    return 0;
  }
  if (!strcmp(argv[1], "stale-out")) {
    reset();
    _dcd.xfer[1][TUSB_DIR_OUT].mps = 64;
    dcd_edpt_stall(0, 1);
    usbd.SIZE.EPOUT[1] = 32;
    usbd.EPDATASTATUS.value = 1u << 17;
    usbd.EVENTS_EPDATA = 1;
    usbd.INTEN = USBD_INTEN_EPDATA_Msk;
    interrupt(); // A packet ACKed before stall is reported after stall.
    assert(_dcd.xfer[1][TUSB_DIR_OUT].data_received);
    dcd_edpt_clear_stall(0, 1); // Releases that packet from the hardware FIFO.
    uint8_t next_buffer[64] = {};
    assert(dcd_edpt_xfer(0, 1, next_buffer, sizeof(next_buffer)));
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0); // Must wait for a genuinely new packet.
    return 0;
  }
  if (!strcmp(argv[1], "stall-deferred")) {
    reset();
    begin_in(2); // Another endpoint owns EasyDMA.
    uint8_t old_buffer[64] = {}, next_buffer[64] = {};
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps = 64; out->buffer = old_buffer; out->total_len = 64; out->data_received = true;
    usbd.SIZE.EPOUT[1] = 32;
    scb.ICSR = 16 + USBD_IRQn;
    usbd.EVENTS_ENDEPIN[2] = 1; // Hardware finishes; stall must leave its accounting queued.
    dcd_edpt_stall(0, 1);
    scb.ICSR = 0;
    dcd_edpt_clear_stall(0, 1);
    assert(dcd_edpt_xfer(0, 1, next_buffer, sizeof(next_buffer)));
    complete_in_dma(2);
    run_deferred_once();
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0); // Old drain must be cancelled before rearm.
    return 0;
  }
  if (!strcmp(argv[1], "stall-inflight")) {
    reset();
    uint8_t old_buffer[64] = {}, next_buffer[64] = {};
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps = 64; out->data_received = true;
    usbd.SIZE.EPOUT[1] = 32;
    assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    assert(_dcd.dma_running);
    hardware_dma_busy = true;
    bool ended = false;
    unsigned polls = 0;
    on_event_read = [&](uint32_t*) {
      if (!ended && ++polls == 3) {
        ended = true; hardware_dma_busy = false;
        usbd.EPOUT[1].AMOUNT = 32;
        usbd.EVENTS_ENDEPOUT[1] = 1;
      }
    };
    dcd_edpt_stall(0, 1);
    dcd_edpt_clear_stall(0, 1);
    assert(dcd_edpt_xfer(0, 1, next_buffer, sizeof(next_buffer)));
    // Inherited stall does not wait; its old END instead arrives after rearm.
    if (!ended) {
      ended = true; hardware_dma_busy = false;
      usbd.EPOUT[1].AMOUNT = 32;
      usbd.EVENTS_ENDEPOUT[1] = 1;
    }
    on_event_read = {};
    usbd.INTEN = 1u << (USBD_INTEN_ENDEPOUT0_Pos + 1);
    interrupt(); // The old DMA must never complete the newly armed transfer.
    assert(usbd.ENABLE && transfer_events == 0 && out->actual_len == 0);
    return 0;
  }
  if (!strcmp(argv[1], "reset-deferred")) {
    reset();
    begin_in(2);
    uint8_t old_buffer[64] = {};
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps = 64; out->data_received = true;
    usbd.SIZE.EPOUT[1] = 32;
    scb.ICSR = 16 + USBD_IRQn;
    assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    scb.ICSR = 0;
    usbd.EVENTS_USBRESET = usbd.EVENTS_ENDEPIN[2] = 1;
    usbd.INTEN = USBD_INTEN_USBRESET_Msk | (1u << (USBD_INTEN_ENDEPIN0_Pos + 2));
    interrupt();
    assert(!_dcd.dma_running && usbd.TASKS_STARTEPOUT[1].triggers == 0);
    run_deferred_once();
    assert(!_dcd.dma_running && usbd.TASKS_STARTEPOUT[1].triggers == 0);
    return 0;
  }
  if (!strcmp(argv[1], "fresh-out")) {
    reset();
    _dcd.xfer[1][TUSB_DIR_OUT].mps = 64;
    dcd_edpt_stall(0, 1);
    usbd.SIZE.EPOUT[1] = 32;
    usbd.EPDATASTATUS.value = 1u << 17;
    usbd.EVENTS_EPDATA = 1;
    usbd.INTEN = USBD_INTEN_EPDATA_Msk | (1u << (USBD_INTEN_ENDEPOUT0_Pos + 1));
    interrupt();
    dcd_edpt_clear_stall(0, 1);
    uint8_t next_buffer[32]; memset(next_buffer, 0xcc, sizeof(next_buffer));
    assert(dcd_edpt_xfer(0, 1, next_buffer, sizeof(next_buffer)));
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0);
    // A genuinely new packet arrives only after clear-halt released the FIFO.
    usbd.SIZE.EPOUT[1] = sizeof(next_buffer);
    usbd.EPDATASTATUS.value = 1u << 17;
    usbd.EVENTS_EPDATA = 1;
    interrupt(); run_deferred_once();
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 1);
    assert(usbd.EPOUT[1].PTR == (uintptr_t)next_buffer && usbd.EPOUT[1].MAXCNT == sizeof(next_buffer));
    memset((void*)usbd.EPOUT[1].PTR, 0x5a, usbd.EPOUT[1].MAXCNT); // Hardware DMA simulation.
    usbd.EPOUT[1].AMOUNT = sizeof(next_buffer);
    usbd.EVENTS_ENDEPOUT[1] = 1;
    interrupt();
    assert(transfer_events == 1 && last_endpoint == 1 && last_length == sizeof(next_buffer));
    for (uint8_t byte: next_buffer) assert(byte == 0x5a);
    return 0;
  }
  if (!strcmp(argv[1], "unrelated-pending")) {
    reset(); begin_in(2);
    uint8_t old_buffer[64] = {}, unrelated[32] = {};
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps = 64; out->data_received = true;
    usbd.SIZE.EPOUT[1] = 32;
    _dcd.xfer[3][TUSB_DIR_IN].mps = 64;
    scb.ICSR = 16 + USBD_IRQn;
    assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    assert(dcd_edpt_xfer(0, 0x83, unrelated, sizeof(unrelated)));
    usbd.EVENTS_ENDEPIN[2] = 1; // Observe this END without consuming its sibling completion.
    dcd_edpt_stall(0, 1);
    scb.ICSR = 0;
    dcd_edpt_clear_stall(0, 1);
    complete_in_dma(2); run_deferred_once();
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0 && usbd.TASKS_STARTEPIN[3].triggers == 1);
    assert(usbd.EPIN[3].PTR == (uintptr_t)unrelated && usbd.EPIN[3].MAXCNT == sizeof(unrelated));
    return 0;
  }
  if (!strcmp(argv[1], "removal-deferred")) {
    reset(); begin_in(2);
    uint8_t old_buffer[64] = {};
    auto *out = &_dcd.xfer[1][TUSB_DIR_OUT];
    out->mps = 64; out->data_received = true;
    usbd.SIZE.EPOUT[1] = 32;
    scb.ICSR = 16 + USBD_IRQn;
    assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    scb.ICSR = 0;
    tusb_hal_nrf_power_event(1);
    assert(!usbd.ENABLE);
    usbd.ENABLE = 1; // A new attachment must not revive the previous queue.
    dcd_init(0);
    run_deferred_once();
    assert(!_dcd.dma_running && usbd.TASKS_STARTEPOUT[1].triggers == 0);
    return 0;
  }
  if (!strcmp(argv[1], "endpoint-bounds")) {
    reset();
    tusb_desc_endpoint_t endpoint = {};
    endpoint.bLength = sizeof(endpoint); endpoint.bDescriptorType = TUSB_DESC_ENDPOINT;
    endpoint.bmAttributes.xfer = TUSB_XFER_BULK; endpoint.wMaxPacketSize = 64;
    for (unsigned number = 0; number < 8; ++number)
      for (unsigned direction = 0; direction < 2; ++direction) {
        endpoint.bEndpointAddress = number | (direction ? 0x80 : 0);
        assert(dcd_edpt_open(0, &endpoint));
        assert(_dcd.xfer[number][direction].mps == 64);
      }
    begin_in(7); complete_in_dma(7);
    assert(usbd.TASKS_STARTEPIN[7].triggers == 1 && !_dcd.dma_running);
    begin_out(7);
    usbd.EPOUT[7].AMOUNT = 32;
    usbd.EVENTS_ENDEPOUT[7] = 1;
    interrupt();
    assert(usbd.TASKS_STARTEPOUT[7].triggers == 1 && !_dcd.dma_running);
    assert(transfer_events == 1 && last_endpoint == 7 && last_length == 32);
#if !CFG_TUD_NRF5X_ISO
    uint8_t buffer[32] = {};
    unsigned char before[sizeof(_dcd)]; memcpy(before, &_dcd, sizeof(_dcd));
    for (unsigned direction = 0; direction < 2; ++direction) {
      uint8_t const address = 8 | (direction ? 0x80 : 0);
      endpoint.bEndpointAddress = address;
      for (unsigned type: {TUSB_XFER_BULK, TUSB_XFER_ISOCHRONOUS}) {
        endpoint.bmAttributes.xfer = type;
        assert(!dcd_edpt_open(0, &endpoint));
      }
      assert(!dcd_edpt_xfer(0, address, buffer, sizeof(buffer)));
      dcd_edpt_close(0, address); dcd_edpt_stall(0, address); dcd_edpt_clear_stall(0, address);
      assert(!memcmp(before, &_dcd, sizeof(_dcd)));
    }
    endpoint.bEndpointAddress = 1; endpoint.bmAttributes.xfer = TUSB_XFER_ISOCHRONOUS;
    assert(!dcd_edpt_open(0, &endpoint));
    assert(!memcmp(before, &_dcd, sizeof(_dcd)));
#else
    endpoint.bmAttributes.xfer = TUSB_XFER_ISOCHRONOUS;
    endpoint.bEndpointAddress = 8; assert(dcd_edpt_open(0, &endpoint));
    endpoint.bEndpointAddress = 0x88; assert(dcd_edpt_open(0, &endpoint));
    assert(usbd.EPOUTEN & USBD_EPOUTEN_ISOOUT_Msk);
    assert(usbd.EPINEN & USBD_EPINEN_ISOIN_Msk);
    uint8_t buffer[32] = {};
    assert(dcd_edpt_xfer(0, 0x88, buffer, sizeof(buffer)));
    assert(usbd.TASKS_STARTISOIN.triggers == 1 && usbd.ISOIN.PTR == (uintptr_t)buffer);
    usbd.SIZE.ISOOUT = USBD_SIZE_ISOOUT_ZERO_Msk;
    usbd.ISOIN.AMOUNT = sizeof(buffer);
    usbd.EVENTS_ENDISOIN = usbd.EVENTS_SOF = 1;
    interrupt();
    assert(!_dcd.dma_running && !errata_register.value);
    assert(transfer_events == 2 && last_endpoint == 0x88 && last_length == sizeof(buffer));
    dcd_edpt_close_all(0);
    assert(usbd.EPINEN == 1 && usbd.EPOUTEN == 1);
#endif
    return 0;
  }
  if (!strcmp(argv[1], "remote-wakeup-sof")) {
    reset(); usbd.LOWPOWER = 1;
    dcd_remote_wakeup(0); assert(usbd.LOWPOWER == 0);
    usbd.EVENTCAUSE.value = USBD_EVENTCAUSE_USBWUALLOWED_Msk;
    usbd.EVENTS_USBEVENT = 1; usbd.INTEN = USBD_INTEN_USBEVENT_Msk;
    interrupt();
    assert(usbd.TASKS_DPDMDRIVE.triggers == 1 && (usbd.INTEN & USBD_INTEN_SOF_Msk));
    usbd.EVENTS_SOF = 1; interrupt();
    assert(sof_events == 1 && !(usbd.INTEN & USBD_INTEN_SOF_Msk));
    assert(!usbd.TASKS_STARTISOIN.triggers && !usbd.TASKS_STARTISOOUT.triggers);
    return 0;
  }
  if (!strcmp(argv[1], "close-deferred")) {
    reset(); begin_in(2);
    uint8_t old_buffer[32] = {}, unrelated[32] = {};
    _dcd.xfer[1][TUSB_DIR_OUT].mps = 64;
    _dcd.xfer[1][TUSB_DIR_OUT].data_received = true;
    usbd.SIZE.EPOUT[1] = sizeof(old_buffer);
    _dcd.xfer[3][TUSB_DIR_IN].mps = 64;
    scb.ICSR = 16 + USBD_IRQn;
    assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    assert(dcd_edpt_xfer(0, 0x83, unrelated, sizeof(unrelated)));
    scb.ICSR = 0;
    usbd.EVENTS_ENDEPIN[2] = 1;
    dcd_edpt_close(0, 1);
    complete_in_dma(2); run_deferred_once();
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0 && usbd.TASKS_STARTEPIN[3].triggers == 1);
    assert(usbd.EPIN[3].PTR == (uintptr_t)unrelated);
    return 0;
  }
  if (!strcmp(argv[1], "close-all")) {
    reset(); begin_in(0);
    uint8_t old_buffer[32] = {}, unrelated[32] = {};
    _dcd.xfer[1][TUSB_DIR_OUT].mps = 64;
    _dcd.xfer[1][TUSB_DIR_OUT].data_received = true;
    usbd.SIZE.EPOUT[1] = sizeof(old_buffer);
    _dcd.xfer[3][TUSB_DIR_IN].mps = 64;
    scb.ICSR = 16 + USBD_IRQn;
    assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    assert(dcd_edpt_xfer(0, 0x83, unrelated, sizeof(unrelated)));
    scb.ICSR = 0;
    usbd.EVENTS_ENDEPIN[0] = 1;
    dcd_edpt_close_all(0);
    assert(_dcd.dma_running); // Closing data endpoints preserves EP0 completion ownership.
    complete_in_dma(0); run_deferred_once();
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0 && usbd.TASKS_STARTEPIN[3].triggers == 0);
    for (unsigned ep = 1; ep < sizeof(_dcd.xfer) / sizeof(_dcd.xfer[0]); ++ep)
      for (unsigned dir = 0; dir < 2; ++dir) {
        auto const *td = &_dcd.xfer[ep][dir];
        assert(!td->buffer && !td->total_len && !td->actual_len && !td->mps && !td->data_received);
      }
    assert(_dcd.xfer[0][TUSB_DIR_IN].mps == MAX_PACKET_SIZE);
    return 0;
  }
  if (!strcmp(argv[1], "usb-preempts-removal")) {
    reset(); begin_in(2);
    bool injected = false;
    on_barrier = [&]() {
      if (!injected && !usbd.USBPULLUP && irq_enabled) {
        injected = true;
        scb.ICSR = 16 + USBD_IRQn; // USBD priority2 preempts POWER priority7.
        begin_out(1); // Queued behind IN2 after removal has begun.
        scb.ICSR = 0;
        interrupt();
      }
    };
    tusb_hal_nrf_power_event(1);
    on_barrier = {};
    assert(injected && !usbd.ENABLE && !_dcd.dma_running);
    usbd.ENABLE = 1;
    interrupt(); run_deferred_once(); // Reattach cannot revive the old OUT request.
    assert(usbd.TASKS_STARTEPOUT[1].triggers == 0 && !_dcd.dma_running);
    return 0;
  }
#ifdef DCD_TEST_PENDING_RACES
  if (!strcmp(argv[1], "power-preempts-request") || !strcmp(argv[1], "power-preempts-dispatch")) {
    reset();
    bool queued_power = false, injected = false;
    on_irq_restore = [&]() {
      if (queued_power) { queued_power = false; tusb_hal_nrf_power_event(1); }
    };
    auto power_arrives = [&](uint32_t) {
      if (!injected) {
        injected = true;
        if (primask) queued_power = true;
        else tusb_hal_nrf_power_event(1);
      }
    };
    if (!strcmp(argv[1], "power-preempts-request")) {
      begin_in(2);
      uint8_t old_buffer[32] = {};
      _dcd.xfer[1][TUSB_DIR_OUT].mps = 64;
      _dcd.xfer[1][TUSB_DIR_OUT].data_received = true;
      usbd.SIZE.EPOUT[1] = sizeof(old_buffer);
      on_pending_read = power_arrives;
      assert(dcd_edpt_xfer(0, 1, old_buffer, sizeof(old_buffer)));
    } else {
      scb.ICSR = 16 + USBD_IRQn;
      begin_in(1); // Queue without starting until the ISR's final dispatch.
      scb.ICSR = 0;
      on_pending_read = power_arrives;
      interrupt();
    }
    on_pending_read = {};
    assert(injected && !queued_power && !usbd.ENABLE);
    assert(!_dcd.dma_running && uint32_t(_dcd.dma_pending) == 0);
    return 0;
  }
#endif
  return 2;
}
'''


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    while ";" in source[start:opening]:
        start = source.index(signature, opening)
        opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def driver_functions(source):
    beginning = re.search(r"\benum\s*\{", source).start()
    iso_default = source.find("#ifndef CFG_TUD_NRF5X_ISO", 0, beginning)
    if iso_default >= 0:
        beginning = iso_default
    end = source.index("// HFCLK helper", beginning)
    code = source[beginning:end]
    if iso_default < 0:
        # Legacy comparison drivers predate the option and always include ISO.
        code = "#ifndef CFG_TUD_NRF5X_ISO\n#define CFG_TUD_NRF5X_ISO 1\n#endif\n" + code
    code += function(source, "static __attribute__((noinline)) void usbd_errata_write")
    code += function(source, "void tusb_hal_nrf_power_event")
    # The hardware SETUP window is eight contiguous uint32_t registers. Mock
    # Register has two words; replace only its indexed address access, retaining
    # the production loop and byte truncation, and log every actual read.
    code, setup_seams = re.subn(
        r"(?:volatile\s+const|const\s+volatile)\s+uint32_t\s*\*\s*setup_regs\s*=\s*&NRF_USBD->BMREQUESTTYPE;",
        "SetupScan setup_regs;", code)
    assert setup_seams == 1, "expected one ordered hardware SETUP window"
    # Execute the actual errata helper through read/write-logging MMIO cells;
    # only register type/address geometry changes, not unlock control flow.
    code = code.replace("usbd_errata_write(volatile uint32_t *reg", "usbd_errata_write(MmioRegister *reg")
    code, latch_seams = re.subn(
        r"volatile\s+uint32_t\s*\*\s*const\s+latch\s*=\s*\(volatile\s+uint32_t\s*\*\)\s*0x4006EC00;",
        "MmioRegister *const latch = &errata_unlock;", code)
    assert latch_seams == 1, "expected one errata unlock register"
    code = re.sub(r"\(volatile\s+uint32_t\s*\*\)\s*0x4006EC14", "&errata_171", code)
    code = re.sub(r"\(volatile\s+uint32_t\s*\*\)\s*0x4006ED14", "&errata_187", code)
    code = re.sub(r"^[ \t]*#define NRF_USBD_ERRATA_199_REG.*$",
                  "#define NRF_USBD_ERRATA_199_REG errata_register", code, flags=re.M)
    # The mock EVENT window preserves the raw hardware layout. Its accessor
    # makes an END arriving during a bounded register scan deterministic.
    code = re.sub(r"volatile\s+uint32_t\s*\*\s*(\w+)\s*=\s*&NRF_USBD->EVENTS_USBRESET;",
                  r"EventScan \1{&NRF_USBD->EVENTS_USBRESET};", code)
    # Task writes are pulses: writing zero cannot cancel an EasyDMA transfer.
    code = re.sub(r"volatile\s+uint32_t\s*\*\s*reg_startep", "TaskRegister *reg_startep", code)
    # Deterministic preemption of a software word has the same uint32_t layout
    # and load/store semantics; production queue/dispatch expressions are kept.
    if re.search(r"(?:volatile\s+)?uint32_t\s+dma_pending;", code):
        code = "#define DCD_TEST_PENDING_RACES 1\n" + re.sub(
            r"(?:volatile\s+)?uint32_t\s+dma_pending;", "PendingBits dma_pending;", code)
        code = code.replace("__builtin_ctz(", "mock_ctz(")
    # Nordic PTR/MAXCNT are adjacent uint32 registers. The host PTR must hold
    # uintptr_t, so preserve these two hardware writes through the typed fixture.
    code = re.sub(r"volatile\s+uint32_t\s*\*\s*regs;", "Endpoint *regs;", code)
    code = re.sub(r"volatile\s+uint32_t\s*\*\s*task;", "TaskRegister *task;", code)
    code = re.sub(r"(&NRF_USBD->(?:EPIN\[epnum\]|EPOUT\[epnum\]|ISOIN|ISOOUT))\.PTR", r"\1", code)
    code = code.replace("regs[0]", "regs->PTR").replace("regs[1]", "regs->MAXCNT")
    # Endpoint pointer registers are uintptr_t in the host fixture.
    code = re.sub(r"\(uint32_t\)\s*xfer->buffer", "(uintptr_t)xfer->buffer", code)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("driver", nargs="?", type=Path, default=DRIVER)
    parser.add_argument("--source", type=Path, help="Override the production driver for artifact/negative checks")
    parser.add_argument("--errata-enabled", type=int, choices=[0, 1], default=1)
    parser.add_argument("--iso", type=int, choices=[0, 1], help="Override the generic default ISO configuration")
    parser.add_argument("--quiet-mode", type=int, choices=[0, 1],
                        help="Exercise the opt-in serialized-task control/STALL quiet contract")
    parser.add_argument("--negative-quiet-status", action="store_true",
                        help="Require omitted post-STATUS spacing to fail task/event ordering")
    parser.add_argument("--negative-quiet-stall", action="store_true",
                        help="Require omitted post-STALL spacing to fail the completed-cleanup regression")
    parser.add_argument("--negative-quiet-masked", action="store_true",
                        help="Require a masked caller to fail if it enters the millisecond delay")
    parser.add_argument("--negative-clear-active", action="store_true",
                        help="Require the inherited unconditional OUT cleanup to fail the active-transfer regression")
    parser.add_argument("--negative-abort-stale", action="store_true",
                        help="Require omission of raw-END retirement to complete the wrong request")
    parser.add_argument("--negative-stall-dma", action="store_true",
                        help="Require the unfenced EPSTALL write to fail while a sibling DMA owns hardware")
    parser.add_argument("--negative-abort-sibling", action="store_true",
                        help="Require owned-only retirement to flush OUT during another endpoint's DMA")
    parser.add_argument("--negative-abort-epoch", action="store_true",
                        help="Require an omitted lifecycle gate to flush after removal/reattachment")
    parser.add_argument("--scenarios", nargs="+", default=[
        "latch", "reset-timeout", "removal-preempts-start", "timeout",
        "captured-end", "late-end", "other-end", "captured-end-next-dma", "direction",
        "stale-out", "stall-deferred", "stall-inflight", "reset-deferred",
        "clear-active-out", "clear-pending-out", "clear-received-out", "clear-halted-out", "clear-active-in",
        "abort-pending", "abort-active", "abort-cached", "abort-stale-end", "abort-halted", "abort-bounds-irq", "abort-timeout",
        "abort-disabled", "abort-sibling-dma", "abort-sibling-timeout", "abort-dma-power", "abort-power-before-commit",
        "stall-all-dma", "stall-dma-timeout", "stall-dma-power", "stall-power-before-commit", "stall-disabled",
        "fresh-out", "unrelated-pending", "removal-deferred", "close-deferred",
        "close-all", "power-preempts-request", "power-preempts-dispatch", "usb-preempts-removal",
        "endpoint-bounds", "remote-wakeup-sof",
        "setup-status-deferred", "setup-status-before-next-setup", "setup-status-cancel",
        "setup-unrelated-pending",
        "setup-old-ep0", "setup-old-ep0-in", "setup-stale-admission",
        "setup-stale-status", "setup-stale-data-in", "setup-stale-data-out", "setup-stale-stall",
        "setup-raw-pending", "setup-timeout-generation", "setup-no-data-in-status",
        "setup-stall-during-retirement", "setup-address-generation", "setup-reset-generation",
        "factored-setup-order", "factored-errata-write", "factored-irq"])
    args = parser.parse_args()
    if args.source is not None:
        args.driver = args.source
    source = args.driver.read_text()
    if args.negative_clear_active:
        old = function(source, "void dcd_edpt_clear_stall")
        changed, matches = re.subn(r"dir == TUSB_DIR_OUT && NRF_USBD->HALTED\.EPOUT\[epnum\]", "dir == TUSB_DIR_OUT", old)
        assert matches == 1, "expected one guarded destructive OUT cleanup"
        source = source.replace(old, changed, 1)
        args.scenarios = ["clear-active-out"]
    if args.negative_abort_stale:
        assert not args.negative_clear_active, "run one negative control at a time"
        old = function(source, "bool dcd_edpt_abort")
        changed, matches = re.subn(r"NRF_USBD->EVENTS_ENDEPOUT\[epnum\]\s*=\s*0;", "(void)0;", old)
        assert matches == 1, "expected one raw-END retirement in abort"
        source = source.replace(old, changed, 1)
        args.scenarios = ["abort-stale-end"]
    if args.negative_stall_dma:
        assert not args.negative_clear_active and not args.negative_abort_stale, "run one negative control at a time"
        if "static uint32_t dma_commit_begin(" in source:
            old = function(source, "static uint32_t dma_commit_begin")
            changed, matches = re.subn(r"!dma_retire\(operations\)",
                                      "(operations && !dma_retire(operations))", old)
        else:
            old = function(source, "void dcd_edpt_stall")
            changed, matches = re.subn(r"if \(!dma_(?:wait_end|retire)\(0\)\)", "if (false)", old)
        assert matches == 1, "expected one global-DMA stall fence"
        source = source.replace(old, changed, 1)
        args.scenarios = ["stall-all-dma"]
    if args.negative_abort_sibling:
        assert not (args.negative_clear_active or args.negative_abort_stale or args.negative_stall_dma or
                    args.negative_abort_epoch), "run one negative control at a time"
        old = function(source, "static bool dma_retire")
        changed, matches = re.subn(r"if \(!dma_wait_end\(0\)\)",
                                  "if ((operations & _dcd.dma_active) && !dma_wait_end(0))", old)
        assert matches == 1, "expected one global-DMA retirement observer"
        source = source.replace(old, changed, 1)
        args.scenarios = ["abort-sibling-dma"]
    if args.negative_abort_epoch:
        assert not (args.negative_clear_active or args.negative_abort_stale or args.negative_stall_dma), \
            "run one negative control at a time"
        admission = "static uint32_t dma_commit_begin" if "static uint32_t dma_commit_begin(" in source else "bool dcd_edpt_abort"
        old = function(source, admission)
        changed, matches = re.subn(r"generation == _ep0\.received", "((void)generation, true)", old)
        assert matches == 1, "expected one abort lifecycle admission gate"
        source = source.replace(old, changed, 1)
        args.scenarios = ["abort-dma-power", "abort-power-before-commit"]
    quiet_negative = args.negative_quiet_status or args.negative_quiet_stall or args.negative_quiet_masked
    if quiet_negative:
        assert sum((args.negative_quiet_status, args.negative_quiet_stall, args.negative_quiet_masked,
                    args.negative_clear_active, args.negative_abort_stale, args.negative_stall_dma,
                    args.negative_abort_sibling, args.negative_abort_epoch)) == 1, "run one negative at a time"
        args.quiet_mode = 1
        if args.negative_quiet_status:
            source, count = re.subn(r"if \(operation == DMA_EP0_STATUS\)\s*\{\s*task_quiet_delay\(\);\s*\}",
                                    "(void)0;", source)
            assert count == 1, "expected one post-STATUS quiet call"
            args.scenarios, quiet_failure = ["quiet-status-thread"], "quiet_delay_calls == 1"
        elif args.negative_quiet_stall:
            old = function(source, "void dcd_edpt_stall")
            changed, count = re.subn(r"task_quiet_delay\(\);", "(void)0;", old)
            assert count == 1, "expected one post-STALL quiet call"
            source = source.replace(old, changed, 1)
            args.scenarios, quiet_failure = ["quiet-stall-cleanup"], "quiet_delay_calls==1"
        else:
            old = function(source, "static void task_quiet_delay")
            changed = old.replace("if (__get_PRIMASK())", "if (false)", 1)
            assert changed != old, "expected the masked-caller failclosed guard"
            source = source.replace(old, changed, 1)
            args.scenarios, quiet_failure = ["quiet-masked-policy"], "!primask"
    elif args.quiet_mode == 1:
        args.scenarios += ["quiet-status-thread", "quiet-status-isr", "quiet-stall-cleanup", "quiet-masked-policy",
                           "quiet-lifecycle", "quiet-setup-order", "quiet-status-usb-progress", "quiet-unrelated"]
    elif args.quiet_mode == 0 and "task_quiet_delay" in source:
        args.scenarios += ["quiet-optout"]
    with tempfile.TemporaryDirectory(prefix="nrf5x-dma-test-") as directory:
        temp = Path(directory)
        config = f"#define NRFX_USBD_ERRATA_ENABLE {args.errata_enabled}\n"
        if args.quiet_mode is not None:
            config += f"#define CFG_TUD_NRF5X_CONTROL_STALL_QUIET {args.quiet_mode}\n"
        config += f"#define DCD_TEST_DRIVER_SETUP_TAGS {int('dcd_edpt0_setup_begin' in source)}\n"
        header = (ROOT / "src/device/dcd.h").read_text()
        config += f"#define DCD_TEST_HEADER_SETUP_TAGS {int('setup_tagid' in header)}\n"
        if args.iso is not None:
            config += f"#define CFG_TUD_NRF5X_ISO {args.iso}\n"
        (temp / "tusb_config.h").write_text(config + CONFIG)
        cpp = temp / "dma.cpp"
        harness = HARNESS
        if "dma_active;" in source:
            harness = harness.replace("_dcd.dma_running", "_dcd.dma_active")
        cpp.write_text(harness.replace("/* DRIVER_FUNCTIONS */", driver_functions(source)))
        binary = temp / "dma"
        subprocess.run([
            *shlex.split(os.environ.get("CXX", "c++")), "-std=c++11", "-O2",
            "-Wall", "-Wextra", "-Werror", "-Wno-narrowing",
            *shlex.split(os.environ.get("USB_TEST_CXXFLAGS", "")),
            "-I", str(temp), "-I", str(ROOT / "src"), str(cpp), "-o", str(binary),
        ], check=True)
        for scenario in args.scenarios:
            if quiet_negative:
                result = subprocess.run([str(binary), scenario], cwd=temp, capture_output=True, text=True, timeout=2)
                assert result.returncode != 0 and re.sub(r"\s+", "", quiet_failure) in re.sub(r"\s+", "", result.stderr), (
                    "quiet negative must fail the actual ordering/context assertion", result.returncode, result.stderr)
                print("nRF5x quiet: omitted spacing/context guard reproduced (negative control)")
                continue
            if args.negative_abort_sibling or args.negative_abort_epoch:
                result = subprocess.run([str(binary), scenario], cwd=temp, capture_output=True, text=True, timeout=2)
                expected = "!hardware_dma_busy && *raw_end == 1" if args.negative_abort_sibling else "!dcd_edpt_abort(0, 1)"
                assert result.returncode != 0 and expected in result.stderr, (
                    "negative control must reproduce unsafe abort admission", result.returncode, result.stderr)
                print("nRF5x DMA: unsafe abort admission reproduced (negative control)")
                continue
            if args.negative_stall_dma:
                result = subprocess.run([str(binary), scenario], cwd=temp, capture_output=True, text=True, timeout=2)
                assert result.returncode != 0 and "!hardware_dma_busy && *raw_end == 1" in result.stderr, (
                    "negative control must write EPSTALL before the sibling DMA END", result.returncode, result.stderr)
                print("nRF5x DMA: unfenced EPSTALL overlaps sibling DMA (negative control)")
                continue
            if args.negative_abort_stale:
                result = subprocess.run([str(binary), scenario], cwd=temp, capture_output=True, text=True, timeout=2)
                assert result.returncode != 0 and "transfer_events == 0 && out->buffer == fresh" in result.stderr, (
                    "negative control must deliver the stale END to the fresh request", result.returncode, result.stderr)
                print("nRF5x DMA: omitted abort retirement completes fresh request with stale END (negative control)")
                continue
            if args.negative_clear_active:
                result = subprocess.run([str(binary), scenario], cwd=temp, capture_output=True, text=True, timeout=2)
                assert result.returncode != 0 and "EPDATASTATUS.value == status" in result.stderr, (
                    "negative control must fail the preserved active OUT packet assertion", result.returncode, result.stderr)
                print("nRF5x DMA: inherited already-clear OUT packet loss reproduced (negative control)")
                continue
            subprocess.run([str(binary), scenario], cwd=temp, check=True, timeout=2)
            print(f"nRF5x DMA: {scenario} PASS")


if __name__ == "__main__":
    main()
