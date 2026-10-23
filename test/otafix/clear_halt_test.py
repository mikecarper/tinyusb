#!/usr/bin/env python3
"""Execute real USBD CLEAR_FEATURE dispatch and endpoint recovery ownership."""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
CONFIG = r'''
#define CFG_TUSB_MCU OPT_MCU_NRF5X
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_VENDOR 1
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#define CFG_TUD_VENDOR_TX_BUFSIZE 64
'''

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include USB_TEST_USBD_SOURCE

#define CHECK(condition, reason)       \
  do {                                 \
    if (!(condition)) {                \
      fprintf(stderr, "%s\n", reason); \
      abort();                         \
    }                                  \
  } while (0)

static struct {
  unsigned clears, submits, aborts;
  bool     halted, data1;
  uint8_t *buffer;
  uint16_t length;
} endpoint[256];
static unsigned control_requests, status_submits, status_completions, bulk_completions;
static uint8_t  completed_endpoint;
static bool     irq_enabled;
static bool     abort_result;

void dcd_init(uint8_t port) {
  assert(port == 0);
}
void dcd_int_enable(uint8_t port) {
  assert(port == 0);
  irq_enabled = true;
}
void dcd_int_disable(uint8_t port) {
  assert(port == 0);
  irq_enabled = false;
}
void dcd_set_address(uint8_t port, uint8_t address) {
  (void)port;
  (void)address;
}
void dcd_remote_wakeup(uint8_t port) {
  (void)port;
}
bool dcd_edpt_open(uint8_t port, const tusb_desc_endpoint_t *descriptor) {
  (void)port;
  (void)descriptor;
  return true;
}
void dcd_edpt_close_all(uint8_t port) {
  (void)port;
}
void dcd_edpt_stall(uint8_t port, uint8_t ep) {
  assert(port == 0);
  endpoint[ep].halted = true;
}
void dcd_edpt_clear_stall(uint8_t port, uint8_t ep) {
  assert(port == 0);
  ++endpoint[ep].clears;
  endpoint[ep].halted = false;
  endpoint[ep].data1  = false;
}
#if WITH_ABORT_HOOK
bool dcd_edpt_abort(uint8_t port, uint8_t ep) {
  assert(port == 0);
  ++endpoint[ep].aborts;
  return abort_result;
}
#endif
bool dcd_edpt_xfer(uint8_t port, uint8_t ep, uint8_t *buffer, uint16_t length) {
  assert(port == 0);
  if (tu_edpt_number(ep) == 0) {
    assert(ep == 0x80 && length == 0);
    ++status_submits;
  } else {
    assert(buffer && length == 64);
    ++endpoint[ep].submits;
    endpoint[ep].buffer = buffer;
    endpoint[ep].length = length;
  }
  return true;
}
void dcd_edpt0_status_complete(uint8_t port, const tusb_control_request_t *request) {
  assert(port == 0 && request->bRequest == TUSB_REQ_CLEAR_FEATURE);
  assert(request->wValue == TUSB_REQ_FEATURE_EDPT_HALT && tu_edpt_number(request->wIndex) == 3);
  ++status_completions;
}

static void app_init(void) {
}
static void app_reset(uint8_t port) {
  (void)port;
}
static uint16_t reject_open(uint8_t port, const tusb_desc_interface_t *desc, uint16_t length) {
  (void)port;
  (void)desc;
  (void)length;
  return 0;
}
static uint16_t app_open(uint8_t port, const tusb_desc_interface_t *desc, uint16_t length) {
  assert(port == 0 && desc->bInterfaceNumber == 0 && desc->bNumEndpoints == 2 && length == 23);
  return length;
}
static bool app_control(uint8_t port, uint8_t stage, const tusb_control_request_t *request) {
  assert(port == 0 && stage == CONTROL_STAGE_SETUP && request->bRequest == TUSB_REQ_CLEAR_FEATURE);
  ++control_requests;
  return false; // Standard endpoint CLEAR_FEATURE must still be ACKed.
}
static bool app_xfer(uint8_t port, uint8_t ep, xfer_result_t result, uint32_t length) {
  assert(port == 0 && result == XFER_RESULT_SUCCESS && length == 64);
  ++bulk_completions;
  completed_endpoint = ep;
  return true;
}
const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *count) {
  static const usbd_class_driver_t drivers[] = {{.init            = app_init,
                                                 .reset           = app_reset,
                                                 .open            = reject_open,
                                                 .control_xfer_cb = app_control,
                                                 .xfer_cb         = app_xfer,
                                                 .sof             = NULL},
                                                {.init            = app_init,
                                                 .reset           = app_reset,
                                                 .open            = app_open,
                                                 .control_xfer_cb = app_control,
                                                 .xfer_cb         = app_xfer,
                                                 .sof             = NULL}};
  *count                                     = 2;
  return drivers;
}
bool tud_vendor_control_xfer_cb(uint8_t port, uint8_t stage, const tusb_control_request_t *request) {
  (void)port;
  (void)stage;
  (void)request;
  return false;
}
const uint8_t *tud_descriptor_device_cb(void) {
  static const tusb_desc_device_t descriptor = {.bLength            = sizeof(tusb_desc_device_t),
                                                .bDescriptorType    = TUSB_DESC_DEVICE,
                                                .bcdUSB             = 0x0200,
                                                .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
                                                .bNumConfigurations = 1};
  return (const uint8_t *)&descriptor;
}
const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  static const uint8_t descriptor[] = {9,
                                       TUSB_DESC_CONFIGURATION,
                                       32,
                                       0,
                                       1,
                                       1,
                                       0,
                                       0x80,
                                       50,
                                       9,
                                       TUSB_DESC_INTERFACE,
                                       0,
                                       0,
                                       2,
                                       TUSB_CLASS_VENDOR_SPECIFIC,
                                       0,
                                       0,
                                       0,
                                       7,
                                       TUSB_DESC_ENDPOINT,
                                       0x03,
                                       TUSB_XFER_BULK,
                                       64,
                                       0,
                                       0,
                                       7,
                                       TUSB_DESC_ENDPOINT,
                                       0x83,
                                       TUSB_XFER_BULK,
                                       64,
                                       0,
                                       0};
  (void)index;
  return descriptor;
}
const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t language) {
  static const uint16_t descriptor[] = {(TUSB_DESC_STRING << 8) | 4, 'T'};
  (void)index;
  (void)language;
  return descriptor;
}

static void reset(void) {
  assert(!tud_task_event_ready());
  dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
  tud_task();
  assert(irq_enabled && !tud_task_event_ready());
  memset(endpoint, 0, sizeof(endpoint));
  control_requests = status_submits = status_completions = bulk_completions = 0;
  for (unsigned ep = 0; ep < CFG_TUD_ENDPPOINT_MAX; ep++) {
    for (unsigned dir = 0; dir < 2; dir++) {
      CHECK(_usbd_dev.ep_status[ep][dir].driver == DRVID_INVALID, "reset retained an endpoint driver mapping");
      assert(!_usbd_dev.ep_status[ep][dir].busy && !_usbd_dev.ep_status[ep][dir].claimed &&
             !_usbd_dev.ep_status[ep][dir].stalled);
    }
  }
  assert(process_set_config(0, 1));
  assert(_usbd_dev.itf2drv[0] == 1);
  for (unsigned ep = 0; ep < CFG_TUD_ENDPPOINT_MAX; ep++) {
    for (unsigned dir = 0; dir < 2; dir++) {
      CHECK(_usbd_dev.ep_status[ep][dir].driver == (ep == 3 ? 1 : DRVID_INVALID),
            "configuration did not bind only its endpoints to the selected driver");
      assert(!_usbd_dev.ep_status[ep][dir].busy && !_usbd_dev.ep_status[ep][dir].claimed &&
             !_usbd_dev.ep_status[ep][dir].stalled);
    }
  }
}
static void clear_request(uint8_t ep) {
  const tusb_control_request_t request = {.bmRequestType = 0x02,
                                          .bRequest      = TUSB_REQ_CLEAR_FEATURE,
                                          .wValue        = TUSB_REQ_FEATURE_EDPT_HALT,
                                          .wIndex        = ep,
                                          .wLength       = 0};
  dcd_event_setup_received(0, (const uint8_t *)&request, true);
  tud_task();
  CHECK(endpoint[ep].data1 == false && endpoint[ep].clears == control_requests,
        "already-clear endpoint did not reset DATA0");
  assert(status_submits == control_requests && status_completions + 1 == control_requests);
  dcd_event_xfer_complete(0, 0x80, 0, XFER_RESULT_SUCCESS, true);
  tud_task();
  assert(status_completions == control_requests && !tud_task_event_ready());
}

static void test_abort_ownership(void) {
  for (unsigned direction = 0; direction < 2; direction++) {
    const uint8_t ep    = 3 | (direction << 7);
    const uint8_t other = ep ^ 0x80;
    for (unsigned halted = 0; halted < 2; halted++) {
      for (unsigned succeeds = 0; succeeds < 2; succeeds++) {
        reset();
        uint8_t buffer[64] = {}, unrelated[64] = {};
        assert(usbd_edpt_claim(0, ep) && usbd_edpt_xfer(0, ep, buffer, sizeof(buffer)));
        assert(usbd_edpt_claim(0, other) && usbd_edpt_xfer(0, other, unrelated, sizeof(unrelated)));
        if (halted) {
          usbd_edpt_stall(0, ep);
        }
        endpoint[ep].data1 = true;
        abort_result       = succeeds;
        const bool success = succeeds && WITH_ABORT_HOOK;
        CHECK(usbd_edpt_abort(0, ep) == success, "abort did not report the controller result");
        CHECK(usbd_edpt_busy(0, ep) == !success && _usbd_dev.ep_status[3][direction].claimed == !success,
              "failed or unsupported abort released endpoint ownership");
        CHECK(usbd_edpt_stalled(0, ep) == halted && endpoint[ep].halted == halted && endpoint[ep].data1,
              "abort changed halt or data toggle state");
        assert(endpoint[ep].aborts == WITH_ABORT_HOOK);
        assert(endpoint[ep].clears == 0 && endpoint[ep].submits == 1);
        assert(usbd_edpt_busy(0, other) && _usbd_dev.ep_status[3][!direction].claimed);
        assert(endpoint[other].buffer == unrelated && endpoint[other].length == 64);
        assert(endpoint[other].aborts == 0 && endpoint[other].clears == 0);
        assert(bulk_completions == 0 && status_submits == 0);
      }
    }
  }
}


static void test_endpoint_addressing(void) {
  for (unsigned number = 0; number < CFG_TUD_ENDPPOINT_MAX; ++number) {
    for (unsigned direction = 0; direction < 2; ++direction) {
      reset();
      const uint8_t ep = number | (direction << 7);
      for (unsigned n = 0; n < CFG_TUD_ENDPPOINT_MAX; n++) {
        for (unsigned d = 0; d < 2; d++) {
          _usbd_dev.ep_status[n][d].driver = (uint8_t)(n * 2 + d);
        }
      }
      assert(usbd_edpt_claim(0, ep));
      for (unsigned n = 0; n < CFG_TUD_ENDPPOINT_MAX; n++) {
        for (unsigned d = 0; d < 2; d++) {
          CHECK(_usbd_dev.ep_status[n][d].claimed == (n == number && d == direction),
                "endpoint address selected another state");
          assert(!usbd_edpt_busy(0, (uint8_t)(n | (d << 7))));
        }
      }
      assert(usbd_edpt_release(0, ep));
      usbd_edpt_stall(0, ep);
      for (unsigned n = 0; n < CFG_TUD_ENDPPOINT_MAX; n++) {
        for (unsigned d = 0; d < 2; d++) {
          const bool target = n == number && d == direction;
          CHECK(usbd_edpt_busy(0, (uint8_t)(n | (d << 7))) == target &&
                  usbd_edpt_stalled(0, (uint8_t)(n | (d << 7))) == target,
                "endpoint address selected another state");
          assert(_usbd_dev.ep_status[n][d].driver == n * 2 + d && !_usbd_dev.ep_status[n][d].claimed);
        }
      }
      usbd_edpt_clear_stall(0, ep);
      assert(!usbd_edpt_busy(0, ep) && !usbd_edpt_stalled(0, ep));
      if (number) {
        uint8_t buffer[64] = {};
        assert(usbd_edpt_claim(0, ep));
        assert(usbd_edpt_xfer(0, ep, buffer, 64));
        assert(usbd_edpt_busy(0, ep) && _usbd_dev.ep_status[number][direction].claimed);
        abort_result = true;
        assert(usbd_edpt_abort(0, ep) == (bool)WITH_ABORT_HOOK);
        assert(usbd_edpt_busy(0, ep) == !WITH_ABORT_HOOK &&
               _usbd_dev.ep_status[number][direction].claimed == !WITH_ABORT_HOOK);
      }
      for (unsigned n = 0; n < CFG_TUD_ENDPPOINT_MAX; n++) {
        for (unsigned d = 0; d < 2; d++) {
          if (n != number || d != direction) {
            assert(!_usbd_dev.ep_status[n][d].busy && !_usbd_dev.ep_status[n][d].claimed &&
                   !_usbd_dev.ep_status[n][d].stalled);
          }
          assert(_usbd_dev.ep_status[n][d].driver == n * 2 + d);
        }
      }
    }
  }
}

int main(void) {
  assert(tusb_init());
  test_endpoint_addressing();
  for (unsigned direction = 0; direction < 2; direction++) {
    const uint8_t ep    = 3 | (direction << 7);
    const uint8_t other = ep ^ 0x80;
    for (unsigned state = 0; state < 4; state++) {
      reset();
      uint8_t buffer[64], unrelated[64];
      memset(buffer, 0xa5, sizeof(buffer));
      memset(unrelated, 0x5a, sizeof(unrelated));
      assert(usbd_edpt_claim(0, other));
      assert(usbd_edpt_xfer(0, other, unrelated, sizeof(unrelated)));
      endpoint[other].data1 = true;
      if (state == 1 || state == 2) {
        assert(usbd_edpt_claim(0, ep));
      }
      if (state == 2) {
        assert(usbd_edpt_xfer(0, ep, buffer, sizeof(buffer)));
      }
      if (state == 3) {
        usbd_edpt_stall(0, ep);
      }
      endpoint[ep].data1 = true;
      for (unsigned repeat = 0; repeat < 2; repeat++) {
        clear_request(ep);
        CHECK(usbd_edpt_busy(0, ep) == (state == 2) &&
                _usbd_dev.ep_status[3][direction].claimed == (state == 1 || state == 2),
              "CLEAR_FEATURE released active endpoint ownership");
        assert(!usbd_edpt_stalled(0, ep) && !endpoint[ep].halted);
        assert(endpoint[ep].clears == repeat + 1 && endpoint[other].clears == 0);
        assert(usbd_edpt_busy(0, other) && _usbd_dev.ep_status[3][!direction].claimed);
        assert(endpoint[other].buffer == unrelated && endpoint[other].length == 64 && endpoint[other].data1);
        if (state == 2) {
          assert(endpoint[ep].buffer == buffer && endpoint[ep].length == 64 && endpoint[ep].submits == 1);
        }
        endpoint[ep].data1 = true; // A later CLEAR_FEATURE must reset it again.
      }
      if (state == 2) {
        dcd_event_xfer_complete(0, ep, sizeof(buffer), XFER_RESULT_SUCCESS, true);
        tud_task();
        assert(bulk_completions == 1 && completed_endpoint == ep);
        assert(!usbd_edpt_busy(0, ep) && !_usbd_dev.ep_status[3][direction].claimed);
        assert(endpoint[ep].submits == 1); // No duplicate transfer was armed.
      }
      for (unsigned i = 0; i < sizeof(buffer); i++) {
        assert(buffer[i] == 0xa5 && unrelated[i] == 0x5a);
      }
    }
  }
  test_abort_ownership();
  return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "src/device/usbd.c")
    args = parser.parse_args()
    source = args.source.read_text()
    begin = source.index("void usbd_edpt_clear_stall(")
    end = source.index("bool usbd_edpt_stalled(", begin)
    function = source[begin:end]
    old = function.replace("  dcd_edpt_clear_stall(rhport, ep_addr);\n", "", 1)
    old, count = re.subn(r"(if\s*\(\s*(?:_usbd_dev\.ep_status\[epnum\]\[dir\]\.|ep_state->)stalled\s*\)\s*\{)",
                         r"\1\n    dcd_edpt_clear_stall(rhport, ep_addr);", old, count=1)
    if count != 1 or old == function:
        raise AssertionError("cannot isolate inherited clear-stall behavior")
    released = function.replace("  dcd_edpt_clear_stall(rhport, ep_addr);",
                                "  dcd_edpt_clear_stall(rhport, ep_addr);\n  ep_state->busy = false;", 1)
    variants = (("fixed", function, 1, None), ("no-abort-hook", function, 0, None),
                ("inherited", old, 1, b"already-clear endpoint did not reset DATA0"),
                ("released", released, 1, b"CLEAR_FEATURE released active endpoint ownership"))
    if "get_ep_status(" in source:
        variants += (("aliased-direction", function, 1, b"endpoint address selected another state"),)
    with tempfile.TemporaryDirectory(prefix="clear-halt-test-") as directory:
        temp = Path(directory)
        (temp / "tusb_config.h").write_text(CONFIG)
        harness = temp / "clear_halt.c"
        harness.write_text(HARNESS)
        for label, replacement, abort_hook, expected_failure in variants:
            driver = temp / f"usbd-{label}.c"
            candidate = source[:begin] + replacement + source[end:]
            if label == "aliased-direction":
                getter = re.search(r"get_ep_status\(uint8_t ep_addr\)\s*\{[^}]+\}", candidate).group(0)
                changed = getter.replace("[tu_edpt_dir(ep_addr)]", "[0]", 1)
                assert changed != getter
                candidate = candidate.replace(getter, changed, 1)
            driver.write_text(candidate)
            binary = temp / label
            subprocess.run([
                *shlex.split(os.environ.get("CC", "cc")), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                "-ffunction-sections", "-fdata-sections", *shlex.split(os.environ.get("USB_TEST_CFLAGS", "")),
                "-I", str(temp), "-I", str(ROOT / "src"), "-I", str(ROOT / "src/device"),
                f'-DUSB_TEST_USBD_SOURCE="{driver}"', f"-DWITH_ABORT_HOOK={abort_hook}",
                str(harness), str(ROOT / "src/device/usbd_control.c"),
                str(ROOT / "src/tusb.c"), str(ROOT / "src/common/tusb_fifo.c"),
                str(ROOT / "src/class/vendor/vendor_device.c"), "-Wl,--gc-sections", "-o", str(binary)
            ], check=True)
            result = subprocess.run([str(binary)], cwd=temp, capture_output=True, timeout=5)
            if expected_failure is None:
                if result.returncode:
                    raise AssertionError(f"fixed failed:\n{result.stderr.decode()}")
            elif result.returncode == 0 or expected_failure not in result.stderr:
                raise AssertionError(f"{label} mutation did not reproduce its behavioral failure")
    print("USBD CLEAR_FEATURE: repeated DATA0 reset, active/claimed ownership, both directions PASS")
    print("USBD CLEAR_FEATURE: inherited skip and premature ownership-release negatives reproduced")
    print("USBD abort: success/failure/absent-hook ownership, halt and toggle preservation PASS")
    print("USBD endpoint addressing: all endpoint numbers/directions preserve driver and unrelated flags PASS")


if __name__ == "__main__":
    main()
