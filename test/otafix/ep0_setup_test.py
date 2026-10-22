#!/usr/bin/env python3
"""Run production USBD SETUP admission through its event queue and control engine.

The DCD stub models only SETUP generations and endpoint submissions. Request
dispatch, endpoint ownership, FIFO queueing, data and status handling all execute
from TinyUSB sources. The optional admission hook is also left undefined in a
separate binary to exercise controllers using the legacy event helper.
"""

import argparse
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/device/usbd.c"
LIVE_ROOT = ROOT
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

#define CHECK(condition, reason) do { \
  if (!(condition)) { fprintf(stderr, "%s\n", reason); abort(); } \
} while (0)

static unsigned descriptor_calls, vendor_setup_calls, vendor_data_calls, vendor_ack_calls;
static unsigned data_submissions, status_submissions, stalls, status_completions;
static unsigned hook_calls, rejected_setups;
static uint8_t last_hook_tag, last_ep;
#if WITH_SETUP_HOOK
static uint8_t current_tag;
#endif
static uint16_t last_size;
static uint8_t last_data[64];
static bool irq_enabled;
static tusb_control_request_t last_vendor_request, last_status_request;

void dcd_init(uint8_t rhport) { assert(rhport == 0); }
void dcd_int_enable(uint8_t rhport) { assert(rhport == 0); irq_enabled = true; }
void dcd_int_disable(uint8_t rhport) { assert(rhport == 0); irq_enabled = false; }
void dcd_set_address(uint8_t rhport, uint8_t address) { (void)rhport; (void)address; }
void dcd_remote_wakeup(uint8_t rhport) { (void)rhport; }
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *descriptor) {
  (void)rhport; (void)descriptor; return true;
}
void dcd_edpt_close_all(uint8_t rhport) { (void)rhport; }
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep) { (void)rhport; (void)ep; }
void dcd_edpt_stall(uint8_t rhport, uint8_t ep) {
  assert(rhport == 0 && tu_edpt_number(ep) == 0);
  ++stalls;
}
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep, uint8_t *buffer, uint16_t size) {
  assert(rhport == 0 && tu_edpt_number(ep) == 0);
  assert(size <= sizeof(last_data));
  last_ep = ep;
  last_size = size;
  if (size) {
    ++data_submissions;
    assert(buffer != NULL);
    if (tu_edpt_dir(ep) == TUSB_DIR_IN) memcpy(last_data, buffer, size);
  } else {
    ++status_submissions;
  }
  return true;
}
void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const *request) {
  assert(rhport == 0);
  assert(request->bRequest == TUSB_REQ_GET_STATUS || request->bRequest == TUSB_REQ_GET_DESCRIPTOR ||
         request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR);
  last_status_request = *request;
  ++status_completions;
}

#if WITH_SETUP_HOOK
bool dcd_edpt0_setup_begin(uint8_t rhport, uint8_t setup_tagid) {
  assert(rhport == 0);
  ++hook_calls;
  last_hook_tag = setup_tagid;
  if (setup_tagid != current_tag) {
    ++rejected_setups;
    return false;
  }
  return true;
}
#endif

uint8_t const *tud_descriptor_device_cb(void) {
  static tusb_desc_device_t const descriptor = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x1234, .idProduct = 0x5678, .bNumConfigurations = 1
  };
  ++descriptor_calls;
  return (uint8_t const *)&descriptor;
}
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  static uint8_t const descriptor[] = {9, TUSB_DESC_CONFIGURATION, 9, 0, 0, 1, 0, 0x80, 50};
  assert(index == 0);
  ++descriptor_calls;
  return descriptor;
}
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  static uint16_t const descriptor[] = {(TUSB_DESC_STRING << 8) | 4, 'T'};
  (void)index; (void)langid;
  ++descriptor_calls;
  return descriptor;
}
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  static uint8_t payload[] = {0x12, 0x34, 0x56, 0x78};
  last_vendor_request = *request;
  if (stage == CONTROL_STAGE_SETUP) {
    ++vendor_setup_calls;
    if (request->wLength) return tud_control_xfer(rhport, request, payload, sizeof(payload));
    return tud_control_status(rhport, request);
  }
  if (stage == CONTROL_STAGE_DATA) ++vendor_data_calls;
  if (stage == CONTROL_STAGE_ACK) ++vendor_ack_calls;
  return true;
}

static tusb_control_request_t const get_status = {
  .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_STATUS, .wLength = 2
};
static tusb_control_request_t const get_descriptor = {
  .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
  .wValue = TUSB_DESC_DEVICE << 8, .wLength = sizeof(tusb_desc_device_t)
};
static tusb_control_request_t const vendor_data = {
  .bmRequestType = 0xc0, .bRequest = 0x41, .wLength = 4
};
static tusb_control_request_t const vendor_status = {
  .bmRequestType = 0x40, .bRequest = 0x42
};
static tusb_control_request_t const unsupported = {
  .bmRequestType = 0x80, .bRequest = 0xff
};

static void clear_observations(void) {
  descriptor_calls = vendor_setup_calls = vendor_data_calls = vendor_ack_calls = 0;
  data_submissions = status_submissions = stalls = status_completions = 0;
  hook_calls = rejected_setups = 0;
  last_hook_tag = last_ep = 0xff;
  last_size = 0xffff;
  memset(last_data, 0xa5, sizeof(last_data));
  memset(&last_vendor_request, 0, sizeof(last_vendor_request));
  memset(&last_status_request, 0, sizeof(last_status_request));
}
static void reset(void) {
  assert(!tud_task_event_ready());
  dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
  tud_task();
  assert(!tud_task_event_ready() && irq_enabled);
  assert(!tud_connected());
  clear_observations();
}
static void queue_tagged(tusb_control_request_t const *request, uint8_t tag) {
  dcd_event_setup_received_tagid(0, (uint8_t const *)request, tag, true);
  assert(tud_task_event_ready());
}
static void complete(uint8_t ep, uint16_t size) {
  dcd_event_xfer_complete(0, ep, size, XFER_RESULT_SUCCESS, true);
  tud_task();
  assert(!tud_task_event_ready());
}
static void seed_ep0_ownership(void) {
  static uint8_t data[2];
  // Use the real endpoint APIs, retaining both claim and busy bits. These
  // sentinels reveal clearing either direction before admission succeeds.
  assert(usbd_edpt_claim(0, 0));
  assert(usbd_edpt_xfer(0, 0, data, sizeof(data)));
  assert(usbd_edpt_claim(0, 0x80));
  assert(usbd_edpt_xfer(0, 0x80, data, sizeof(data)));
  clear_observations();
}
#if WITH_SETUP_HOOK
static void check_no_rejected_effects(void) {
  // Check observable request effects before checking hook accounting, so the
  // old-handler negative control fails on behavior rather than a missing hook.
  CHECK(descriptor_calls == 0, "rejected SETUP invoked a descriptor callback");
  CHECK(vendor_setup_calls == 0 && vendor_data_calls == 0 && vendor_ack_calls == 0,
        "rejected SETUP invoked a vendor callback");
  CHECK(data_submissions == 0 && status_submissions == 0 && stalls == 0 && status_completions == 0,
        "rejected SETUP submitted data/status/stall");
  CHECK(_usbd_dev.ep_status[0][TUSB_DIR_OUT].busy && _usbd_dev.ep_status[0][TUSB_DIR_OUT].claimed &&
        _usbd_dev.ep_status[0][TUSB_DIR_IN].busy && _usbd_dev.ep_status[0][TUSB_DIR_IN].claimed,
        "rejected SETUP changed EP0 busy/claimed ownership");
  CHECK(!tud_connected(), "rejected SETUP changed the connection state");
}
#endif
static void check_get_status_data(void) {
  assert(data_submissions == 1 && status_submissions == 0 && stalls == 0);
  assert(descriptor_calls == 0 && vendor_setup_calls == 0);
  assert(last_ep == 0x80 && last_size == 2 && last_data[0] == 0 && last_data[1] == 0);
  assert(tud_connected());
  assert(_usbd_dev.ep_status[0][TUSB_DIR_IN].busy && !_usbd_dev.ep_status[0][TUSB_DIR_IN].claimed);
  assert(!_usbd_dev.ep_status[0][TUSB_DIR_OUT].busy && !_usbd_dev.ep_status[0][TUSB_DIR_OUT].claimed);
}
static void finish_get_status(void) {
  complete(0x80, 2);
  assert(data_submissions == 1 && status_submissions == 1 && stalls == 0);
  assert(last_ep == 0 && last_size == 0 && status_completions == 0);
  complete(0, 0);
  assert(status_completions == 1 && !usbd_edpt_busy(0, 0) && !usbd_edpt_busy(0, 0x80));
}

#if WITH_SETUP_HOOK
static void test_rejected_setup_preserves_control(void) {
  const tusb_control_request_t active_request = {.bmRequestType = 0xc0,
                                                 .bRequest      = 0x61,
                                                 .wValue        = 0x1234,
                                                 .wIndex        = 0x5678,
                                                 .wLength       = 4};
  reset();
  current_tag = 22;
  queue_tagged(&active_request, current_tag);
  tud_task();
  assert(vendor_setup_calls == 1 && vendor_data_calls == 0 && vendor_ack_calls == 0);
  assert(data_submissions == 1 && status_submissions == 0 && last_ep == 0x80 && last_size == 4);
  assert(last_data[0] == 0x12 && last_data[1] == 0x34 && last_data[2] == 0x56 && last_data[3] == 0x78);
  assert(usbd_edpt_busy(0, 0x80) && !usbd_edpt_busy(0, 0));

  // A stale notification must not disturb the accepted request's actual
  // control-engine state or its installed DATA/ACK completion callback.
  queue_tagged(&get_descriptor, 21);
  tud_task();
  assert(hook_calls == 2 && rejected_setups == 1 && last_hook_tag == 21);
  assert(descriptor_calls == 0 && vendor_setup_calls == 1 && vendor_data_calls == 0 && vendor_ack_calls == 0);
  assert(data_submissions == 1 && status_submissions == 0 && stalls == 0 && status_completions == 0);
  assert(usbd_edpt_busy(0, 0x80) && !usbd_edpt_busy(0, 0));
  assert(!_usbd_dev.ep_status[0][TUSB_DIR_IN].claimed && !_usbd_dev.ep_status[0][TUSB_DIR_OUT].claimed);

  complete(0x80, active_request.wLength);
  CHECK(vendor_data_calls == 1 && vendor_ack_calls == 0, "rejected SETUP lost the original DATA completion callback");
  CHECK(memcmp(&last_vendor_request, &active_request, sizeof(active_request)) == 0,
        "rejected SETUP replaced the original DATA request");
  assert(data_submissions == 1 && status_submissions == 1 && last_ep == 0 && last_size == 0);
  assert(status_completions == 0 && !usbd_edpt_busy(0, 0x80) && usbd_edpt_busy(0, 0));

  complete(0, 0);
  CHECK(vendor_data_calls == 1 && vendor_ack_calls == 1, "rejected SETUP lost the original ACK completion callback");
  CHECK(memcmp(&last_vendor_request, &active_request, sizeof(active_request)) == 0 &&
          memcmp(&last_status_request, &active_request, sizeof(active_request)) == 0,
        "rejected SETUP replaced the original STATUS request");
  assert(status_completions == 1 && stalls == 0);
  assert(!usbd_edpt_busy(0, 0) && !usbd_edpt_busy(0, 0x80));
  assert(!_usbd_dev.ep_status[0][TUSB_DIR_IN].claimed && !_usbd_dev.ep_status[0][TUSB_DIR_OUT].claimed);
}
#endif

int main(void) {
  assert(tusb_init());
#if WITH_SETUP_HOOK
  tusb_control_request_t const *rejected[] = {
    &get_descriptor, &vendor_data, &vendor_status, &unsupported, &get_status
  };
  for (unsigned i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
    reset();
    seed_ep0_ownership();
    queue_tagged(rejected[i], 10);
    current_tag = 11; // A newer SETUP superseded the queued event before tud_task.
    tud_task();
    check_no_rejected_effects();
    assert(hook_calls == 1 && rejected_setups == 1 && last_hook_tag == 10);
    queue_tagged(&get_status, current_tag);
    tud_task();
    check_get_status_data();
    assert(hook_calls == 2 && rejected_setups == 1 && last_hook_tag == current_tag);
    finish_get_status();
  }

  test_rejected_setup_preserves_control();

  // Several events can already be queued when the task finally runs. Only the
  // newest generation may reach request dispatch, including across byte wrap.
  reset();
  seed_ep0_ownership();
  queue_tagged(&get_descriptor, 0xfe);
  queue_tagged(&vendor_data, 0xff);
  queue_tagged(&get_status, 1);
  current_tag = 1;
  tud_task();
  check_get_status_data();
  assert(hook_calls == 3 && rejected_setups == 2 && last_hook_tag == 1);
  finish_get_status();
#else
  // No definition of the optional weak hook is linked into this executable.
  assert(!dcd_edpt0_setup_begin);
  reset();
  seed_ep0_ownership();
  dcd_event_setup_received(0, (uint8_t const *)&get_status, true);
  tud_task();
  check_get_status_data();
  assert(hook_calls == 0);
  finish_get_status();

  reset();
  dcd_event_setup_received(0, (uint8_t const *)&get_descriptor, true);
  tud_task();
  assert(descriptor_calls == 1 && data_submissions == 1 && last_ep == 0x80);
  assert(last_size == sizeof(tusb_desc_device_t) && last_data[1] == TUSB_DESC_DEVICE);
  complete(0x80, sizeof(tusb_desc_device_t));
  complete(0, 0);
  assert(status_completions == 1);

  reset();
  queue_tagged(&vendor_data, 37); // Tagged events are also harmless without a hook.
  tud_task();
  assert(vendor_setup_calls == 1 && data_submissions == 1 && last_size == 4);
  complete(0x80, 4);
  complete(0, 0);
  assert(vendor_data_calls == 1 && vendor_ack_calls == 1 && status_completions == 1);

  reset();
  dcd_event_setup_received(0, (uint8_t const *)&vendor_status, true);
  tud_task();
  assert(vendor_setup_calls == 1 && data_submissions == 0 && status_submissions == 1);
  complete(0x80, 0);
  assert(vendor_data_calls == 0 && vendor_ack_calls == 1 && status_completions == 1);

  reset();
  dcd_event_setup_received(0, (uint8_t const *)&unsupported, true);
  tud_task();
  assert(stalls == 2 && data_submissions == 0 && status_submissions == 0);
#endif
  return 0;
}
'''


def without_admission(source):
    """Restore only the old SETUP handler behavior in a temporary source copy."""
    pattern = (r"\s*if\s*\(\s*dcd_edpt0_setup_begin\s*&&\s*"
               r"!dcd_edpt0_setup_begin\(event\.rhport,\s*event\.setup_tagid\)\s*\)"
               r"\s*(?:\{\s*break;\s*\}|break;)")
    mutated, count = re.subn(pattern, "", source, count=1)
    if count != 1:
        raise AssertionError("cannot isolate the SETUP admission gate for the negative control")
    return mutated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE,
                        help="compile this usbd.c instead of the working-tree source")
    parser.add_argument("--tinyusb-root", type=Path, default=LIVE_ROOT,
                        help="live TinyUSB tree supplying unchanged source dependencies")
    parser.add_argument("--header-root", type=Path, default=ROOT / "src",
                        help="include root containing device/dcd.h")
    args = parser.parse_args()
    source = args.source.resolve()
    tinyusb_root = args.tinyusb_root.resolve()
    header_root = args.header_root.resolve()
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "tusb_config.h").write_text(CONFIG)
        # Overlay the chosen generation header while
        # taking unchanged stack dependencies from the shipping checkout. Copy
        # usbd_control.c beside it so its quoted dcd.h include cannot resolve to
        # the different shipping ABI in the original source directory.
        (temp / "device").mkdir()
        shutil.copy2(header_root / "device/dcd.h", temp / "device/dcd.h")
        shutil.copy2(tinyusb_root / "src/device/usbd_control.c", temp / "device/usbd_control.c")
        harness = temp / "ep0_setup.c"
        harness.write_text(HARNESS)
        negative = temp / "usbd_without_admission.c"
        for label, hook in (("hook", 1), ("no-hook", 0), ("inherited", 1)):
            driver = source
            if label == "inherited":
                negative.write_text(without_admission(source.read_text()))
                driver = negative
            binary = temp / label
            subprocess.run(
                [*shlex.split(os.environ.get("CC", "cc")), "-std=c11", "-O2", "-Wall", "-Wextra",
                 "-Werror", "-ffunction-sections", "-fdata-sections",
                 *shlex.split(os.environ.get("USB_TEST_CFLAGS", "")),
                 "-I", str(temp), "-I", str(tinyusb_root / "src"),
                 "-I", str(tinyusb_root / "src/device"),
                 f'-DUSB_TEST_USBD_SOURCE="{driver}"', f"-DWITH_SETUP_HOOK={hook}",
                 str(harness), str(temp / "device/usbd_control.c"),
                 str(tinyusb_root / "src/tusb.c"), str(tinyusb_root / "src/common/tusb_fifo.c"),
                 str(tinyusb_root / "src/class/vendor/vendor_device.c"),
                 "-Wl,--gc-sections", "-o", str(binary)], check=True,
            )
            result = subprocess.run([str(binary)], cwd=temp, capture_output=True, timeout=5)
            if label != "inherited":
                if result.returncode:
                    raise AssertionError(f"{label} failed:\n{result.stderr.decode()}")
            elif result.returncode == 0 or b"rejected SETUP invoked a descriptor callback" not in result.stderr:
                raise AssertionError("inherited handler did not reproduce superseded SETUP dispatch")
    print("EP0 SETUP: superseded descriptor/vendor/data/status/stall isolation and ownership PASS")
    print("EP0 SETUP: accepted GET_STATUS, queued generations and no-hook legacy path PASS")
    print("EP0 SETUP: active control request and DATA/ACK callbacks survive rejected SETUP PASS")
    print("EP0 SETUP: inherited superseded request dispatch reproduced (negative control)")


if __name__ == "__main__":
    main()
