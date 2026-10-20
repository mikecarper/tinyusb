#!/usr/bin/env python3
"""Execute the real MSC BOT driver across passed and failed status delivery."""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
CONFIG = r'''
#define CFG_TUSB_MCU OPT_MCU_NRF5X
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_MSC 1
#define CFG_TUD_MSC_EP_BUFSIZE 512
#define CFG_TUD_MSC_DEFERRED_WRITE_RETRY 1
'''

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "class/msc/msc_device.c"

static bool stalled[256], pending_eject, reject_eject;
static unsigned accepts, exits, completed, reads, writes;

bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep, uint8_t *buffer, uint16_t size) {
  (void)rhport; (void)ep; (void)buffer; (void)size;
  return true;
}
void usbd_edpt_stall(uint8_t rhport, uint8_t ep) { (void)rhport; stalled[ep] = true; }
bool usbd_edpt_stalled(uint8_t rhport, uint8_t ep) { (void)rhport; return stalled[ep]; }
bool usbd_edpt_busy(uint8_t rhport, uint8_t ep) { (void)rhport; (void)ep; return false; }
bool usbd_edpt_claim(uint8_t rhport, uint8_t ep) { (void)rhport; (void)ep; return true; }
bool usbd_edpt_release(uint8_t rhport, uint8_t ep) { (void)rhport; (void)ep; return true; }
void dcd_event_xfer_complete(uint8_t rhport, uint8_t ep, uint32_t n, uint8_t result, bool isr) {
  (void)rhport; (void)ep; (void)n; (void)result; (void)isr;
}
bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const *r, void *b, uint16_t n) {
  (void)rhport; (void)r; (void)b; (void)n; return true;
}
bool tud_control_status(uint8_t rhport, tusb_control_request_t const *r) {
  (void)rhport; (void)r; return true;
}
void tud_msc_inquiry_cb(uint8_t lun, uint8_t v[8], uint8_t p[16], uint8_t r[4]) {
  (void)lun; memset(v, ' ', 8); memset(p, ' ', 16); memset(r, ' ', 4);
}
bool tud_msc_test_unit_ready_cb(uint8_t lun) { (void)lun; return true; }
void tud_msc_capacity_cb(uint8_t lun, uint32_t *blocks, uint16_t *size) {
  (void)lun; *blocks = 16; *size = 512;
}
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t off, void *b, uint32_t n) {
  (void)lun; (void)lba; (void)off; memset(b, 0, n); return (int32_t)n;
}
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t off, uint8_t *b, uint32_t n) {
  (void)lun; (void)lba; (void)off; (void)b; return (int32_t)n;
}
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const cmd[16], void *b, uint16_t n) {
  (void)lun; (void)cmd; (void)b; (void)n; return -1;
}
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power, bool start, bool eject) {
  (void)lun; (void)power;
  accepts++;
  if (reject_eject) return false;
  if (!start && eject) pending_eject = true;
  return true;
}
void tud_msc_scsi_complete_cb(uint8_t lun, uint8_t const cmd[16]) {
  (void)lun;
  if (cmd[0] == SCSI_CMD_START_STOP_UNIT && pending_eject) exits++;
}
void tud_msc_command_complete_cb(uint8_t lun, uint8_t const cmd[16]) {
  (void)lun; (void)cmd;
  pending_eject = false;
  completed++;
}
void tud_msc_read10_complete_cb(uint8_t lun) { (void)lun; reads++; }
void tud_msc_write10_complete_cb(uint8_t lun) { (void)lun; writes++; }

static void reset(void) {
  mscd_init();
  _mscd_itf.ep_in = 0x81;
  _mscd_itf.ep_out = 1;
  memset(stalled, 0, sizeof(stalled));
  pending_eject = reject_eject = false;
  accepts = exits = completed = reads = writes = 0;
}
static void start_stop(uint8_t action, uint32_t data_bytes) {
  msc_cbw_t *cbw = &_mscd_itf.cbw;
  memset(cbw, 0, sizeof(*cbw));
  cbw->signature = MSC_CBW_SIGNATURE;
  cbw->cmd_len = 6;
  cbw->dir = 0x80;
  cbw->total_bytes = data_bytes;
  cbw->command[0] = SCSI_CMD_START_STOP_UNIT;
  cbw->command[4] = action;
  assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, sizeof(*cbw)));
}
static void accept_status(void) {
  if (stalled[0x81]) {
    // Host clears the rejected data-stage stall, then receives the failed CSW.
    stalled[0x81] = false;
    tusb_control_request_t const clear = {
      .bmRequestType = 0x02, .bRequest = TUSB_REQ_CLEAR_FEATURE,
      .wValue = TUSB_REQ_FEATURE_EDPT_HALT, .wIndex = 0x81
    };
    assert(mscd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &clear));
  }
  assert(_mscd_itf.stage == MSC_STAGE_STATUS_SENT);
  assert(mscd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, sizeof(msc_csw_t)));
}
int main(void) {
  reset();
  start_stop(2, 0);
  assert(accepts == 1 && exits == 0 && completed == 0);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_PASSED);
  accept_status();
  assert(exits == 1 && completed == 1);

  reset();
  start_stop(2, 1); // BOT case 4: unexpected IN data fails after acceptance.
  assert(accepts == 1 && pending_eject && exits == 0);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_FAILED);
  accept_status();
  assert(exits == 0 && completed == 1 && !pending_eject);
  start_stop(3, 0); // A later valid media load cannot reuse that failed eject.
  accept_status();
  assert(exits == 0 && completed == 2);

  reset();
  reject_eject = true;
  start_stop(2, 0);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_FAILED);
  accept_status();
  assert(exits == 0 && completed == 1);

  // READ/WRITE completion must remain unconditional so application-level
  // failed-transfer/abort handling still runs, followed by generic lifecycle.
  for (unsigned status = MSC_CSW_STATUS_PASSED; status <= MSC_CSW_STATUS_PHASE_ERROR; status++) {
    reset();
    _mscd_itf.cbw.command[0] = SCSI_CMD_READ_10;
    _mscd_itf.stage = MSC_STAGE_STATUS_SENT;
    _mscd_itf.csw.status = status;
    accept_status();
    assert(reads == 1 && writes == 0 && completed == 1);
    _mscd_itf.cbw.command[0] = SCSI_CMD_WRITE_10;
    _mscd_itf.stage = MSC_STAGE_STATUS_SENT;
    _mscd_itf.csw.status = status;
    accept_status();
    assert(reads == 1 && writes == 1 && completed == 2);
  }
  return 0;
}
'''


def main():
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "tusb_config.h").write_text(CONFIG)
        harness = temp / "msc_status.c"
        harness.write_text(HARNESS)
        driver = (ROOT / "src/class/msc/msc_device.c").read_text()
        success_gate = "p_csw->status == MSC_CSW_STATUS_PASSED && tud_msc_scsi_complete_cb"
        assert success_gate in driver
        for label in ("fixed", "inherited"):
            if label == "inherited":
                # Compile the complete driver again with only its inherited
                # unconditional specialized callback restored. It must fail
                # the unexpected-data eject assertion, not a source guard.
                copied = temp / "class/msc"
                copied.mkdir(parents=True)
                (copied / "msc_device.c").write_text(
                    driver.replace(success_gate, "tud_msc_scsi_complete_cb", 1))
                (copied / "msc_device.h").symlink_to(ROOT / "src/class/msc/msc_device.h")
            binary = temp / label
            subprocess.run(
                [*shlex.split(os.environ.get("CC", "cc")), "-std=c11", "-O2", "-Wall", "-Wextra",
                 "-Werror", "-ffunction-sections", "-fdata-sections",
                 *shlex.split(os.environ.get("USB_TEST_CFLAGS", "")),
                 "-I", str(temp), "-I", str(ROOT / "src"),
                 "-I", str(ROOT / "src/class/msc"), str(harness),
                 "-Wl,--gc-sections", "-o", str(binary)], check=True,
            )
            result = subprocess.run([str(binary)], cwd=temp, capture_output=True, timeout=5)
            if label == "fixed":
                result.check_returncode()
            elif result.returncode == 0 or b"exits == 0 && completed == 1" not in result.stderr:
                raise AssertionError("inherited driver did not reproduce failed-CSW eject")
    print("MSC status: passed eject, failed/aborted eject and READ/WRITE lifecycle PASS")
    print("MSC status: inherited failed-CSW eject reproduced (negative control)")


if __name__ == "__main__":
    main()
