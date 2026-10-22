#!/usr/bin/env python3
"""Run the actual MSC driver through bounded discard, reset and retry paths.

The temporary source overlay adds buffer sentinels without changing transfer
logic. Negative controls restore only the inherited large-OUT rejection or
remove reset invalidation, then require an observable behavioral failure.
"""
from __future__ import annotations

import argparse
import runpy
from types import SimpleNamespace
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

LIVE = Path(__file__).resolve().parents[2]
CANDIDATE = LIVE / 'src/class/msc/msc_device.c'
BASE = SimpleNamespace(**runpy.run_path(str(LIVE / 'test/otafix/msc_status_test.py')))

GUARDED_BUFFER = r'''
static struct {
  uint64_t                   before;
  CFG_TUSB_MEM_ALIGN uint8_t bytes[CFG_TUD_MSC_EP_BUFSIZE];
  uint64_t                   after;
} fixture_buffer;
#define _mscd_buf fixture_buffer.bytes
'''

EXTRA_DECLARATIONS = r'''
enum {
  EXPECT_DISCARD = 5
};
static unsigned begins, resets, generic_calls, read_calls, write_calls, builtin_calls, write_checks;
static bool     command_active, write_busy;
static struct {
  uint8_t  ep;
  uint8_t *buffer;
  uint16_t size;
} queued[128];
static unsigned queued_count;
static void     check_sentinels(void) {
  assert(_mscd_buf == (uint8_t *)&fixture_buffer.before + sizeof(fixture_buffer.before));
  assert((uint8_t *)&fixture_buffer.after == _mscd_buf + sizeof(_mscd_buf));
  assert(fixture_buffer.before == UINT64_C(0x53a9e18d72c406bf));
  assert(fixture_buffer.after == UINT64_C(0xb086e52a34f719cd));
}
'''

EXTRA_TESTS = r'''
void tud_msc_command_begin_cb(uint8_t lun, const uint8_t cmd[16]) {
  (void)lun;
  (void)cmd;
  assert(!command_active);
  command_active = true;
  begins++;
}
void tud_msc_reset_cb(void) {
  command_active = false;
  resets++;
}
bool tud_msc_is_writable_cb(uint8_t lun) {
  (void)lun;
  write_checks++;
  return true;
}

static void submit(uint8_t opcode, uint32_t length, bool in) {
  msc_cbw_t *cbw = &_mscd_itf.cbw;
  memset(cbw, 0, sizeof(*cbw));
  cbw->signature   = MSC_CBW_SIGNATURE;
  cbw->tag         = 0x12345678;
  cbw->cmd_len     = 16;
  cbw->dir         = in ? 0x80 : 0;
  cbw->total_bytes = length;
  cbw->command[0]  = opcode;
  cbw->command[4]  = 2;
  if (opcode == SCSI_CMD_READ_10 || opcode == SCSI_CMD_WRITE_10) {
    cbw->command[4] = 0;
    cbw->command[8] = (uint8_t)(length / 512);
  }
  assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, sizeof(*cbw)));
}
static void no_acceptance(void) {
  assert(generic_calls == 0 && read_calls == 0 && write_calls == 0);
  assert(builtin_calls == 0 && accepts == 0 && exits == 0 && write_checks == 0);
  assert(reads == 0 && writes == 0 && !pending_eject);
}
static void check_discard_start(uint32_t length) {
  assert(_mscd_itf.stage == EXPECT_DISCARD);
  assert(queued_count == 1 && queued[0].ep == 1 && queued[0].buffer == _mscd_buf && queued[0].size == 4096);
  assert(_mscd_itf.total_len == length && _mscd_itf.xferred_len == 0);
  assert(!stalled[1] && !stalled[0x81]);
  assert(begins == 1 && command_active && completed == 0);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_FAILED);
  assert(_mscd_itf.sense_key == 5 && _mscd_itf.add_sense_code == 0x20 && _mscd_itf.add_sense_qualifier == 0);
  no_acceptance();
  check_sentinels();
}
static void drain_command(uint8_t opcode, uint32_t length) {
  reset();
  submit(opcode, length, false);
  check_discard_start(length);
  uint32_t remaining = length;
  unsigned chunks    = 0;
  while (remaining) {
    uint32_t size = remaining < 4096 ? remaining : 4096;
    assert(queued_count == chunks + 1);
    assert(queued[chunks].ep == 1 && queued[chunks].buffer == _mscd_buf && queued[chunks].size == size);
    memset(queued[chunks].buffer, 0xa0 + chunks, size);
    check_sentinels();
    assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, size));
    remaining -= size;
    chunks++;
    assert(_mscd_itf.xferred_len == 0 && _mscd_itf.total_len == remaining);
    assert(command_active && completed == 0);
    assert(!stalled[1] && !stalled[0x81]);
    no_acceptance();
    check_sentinels();
  }
  assert(_mscd_itf.stage == MSC_STAGE_STATUS_SENT);
  assert(queued_count == chunks + 1 && queued[chunks].ep == 0x81 && queued[chunks].size == sizeof(msc_csw_t));
  assert(queued[chunks].buffer == (uint8_t *)&_mscd_itf.csw);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_FAILED && _mscd_itf.csw.data_residue == length);
  assert(_mscd_itf.csw.tag == 0x12345678);
  accept_status();
  assert(completed == 1 && begins == 1 && !command_active);
  assert(_mscd_itf.stage == MSC_STAGE_CMD && queued[queued_count - 1].size == sizeof(msc_cbw_t));
  no_acceptance();
  check_sentinels();
}
static void test_all_discard_paths(void) {
  const uint8_t commands[] = {0xff,
                              SCSI_CMD_START_STOP_UNIT,
                              SCSI_CMD_INQUIRY,
                              SCSI_CMD_TEST_UNIT_READY,
                              SCSI_CMD_READ_CAPACITY_10,
                              SCSI_CMD_MODE_SENSE_6};
  for (unsigned i = 0; i < sizeof(commands); i++) {
    drain_command(commands[i], 8192);
  }
  drain_command(0xff, 4097);
  drain_command(0xff, 8193);
  drain_command(0xff, 16385);
}
static void clear_endpoint(uint8_t ep) {
  stalled[ep]                        = false;
  const tusb_control_request_t clear = {.bmRequestType = 0x02,
                                        .bRequest      = TUSB_REQ_CLEAR_FEATURE,
                                        .wValue        = TUSB_REQ_FEATURE_EDPT_HALT,
                                        .wIndex        = ep};
  assert(mscd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &clear));
}
static void bot_reset(void) {
  const tusb_control_request_t request = {.bmRequestType = 0x21,
                                          .bRequest      = MSC_REQ_RESET,
                                          .wValue        = 0,
                                          .wIndex        = 0,
                                          .wLength       = 0};
  assert(mscd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &request));
}
static void fresh_read_after_recovery(void) {
  unsigned previous_begins     = begins;
  unsigned previous_completed  = completed;
  unsigned previous_read_calls = read_calls;
  submit(SCSI_CMD_READ_10, 512, true);
  assert(_mscd_itf.stage == MSC_STAGE_DATA && begins == previous_begins + 1);
  assert(read_calls == previous_read_calls + 1 && command_active);
  assert(queued[queued_count - 1].ep == 0x81 && queued[queued_count - 1].size == 512);
  assert(mscd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, 512));
  assert(_mscd_itf.stage == MSC_STAGE_STATUS_SENT);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_PASSED && _mscd_itf.csw.data_residue == 0);
  accept_status();
  assert(completed == previous_completed + 1 && !command_active && _mscd_itf.stage == MSC_STAGE_CMD);
  assert(!stalled[1] && !stalled[0x81]);
  check_sentinels();
}
static void test_malformed_discard(void) {
  const struct {
    uint8_t       ep;
    xfer_result_t result;
    uint32_t      bytes;
  } malformed[] = {{1, XFER_RESULT_SUCCESS, 0},       {1, XFER_RESULT_SUCCESS, 4095}, {1, XFER_RESULT_SUCCESS, 4097},
                   {1, XFER_RESULT_SUCCESS, 8192},    {1, XFER_RESULT_FAILED, 4096},  {1, XFER_RESULT_STALLED, 4096},
                   {0x81, XFER_RESULT_SUCCESS, 4096}, {2, XFER_RESULT_SUCCESS, 4096}};
  for (unsigned i = 0; i < sizeof(malformed) / sizeof(malformed[0]); i++) {
    reset();
    submit(0xff, 8192, false);
    check_discard_start(8192);
    assert(!mscd_xfer_cb(0, malformed[i].ep, malformed[i].result, malformed[i].bytes));
    assert(_mscd_itf.stage == MSC_STAGE_NEED_RESET && stalled[1] && stalled[0x81]);
    assert(_mscd_itf.total_len == 8192 && _mscd_itf.xferred_len == 0 && queued_count == 1);
    assert(completed == 0 && begins == 1 && command_active);
    no_acceptance();
    clear_endpoint(1);
    clear_endpoint(0x81);
    assert(_mscd_itf.stage == MSC_STAGE_NEED_RESET && stalled[1] && stalled[0x81]);
    assert(queued_count == 1);
    bot_reset();
    assert(resets == 1 && !command_active && _mscd_itf.stage == MSC_STAGE_CMD);
    clear_endpoint(0x81);
    clear_endpoint(1);
    assert(!stalled[1] && !stalled[0x81] && queued_count == 2);
    assert(queued[1].ep == 1 && queued[1].buffer == (uint8_t *)&_mscd_itf.cbw && queued[1].size == sizeof(msc_cbw_t));
    check_sentinels();
    fresh_read_after_recovery();
  }
}
static void test_clear_during_discard(void) {
  reset();
  submit(0xff, 8192, false);
  check_discard_start(8192);
  clear_endpoint(1);
  clear_endpoint(0x81);
  assert(_mscd_itf.stage == EXPECT_DISCARD && queued_count == 1);
  assert(begins == 1 && completed == 0 && command_active);
}
static void test_malformed_tail(void) {
  const struct {
    uint8_t       ep;
    xfer_result_t result;
    uint32_t      bytes;
  } malformed[] = {{1, XFER_RESULT_SUCCESS, 0},
                   {1, XFER_RESULT_SUCCESS, 2},
                   {1, XFER_RESULT_SUCCESS, 4096},
                   {1, XFER_RESULT_FAILED, 1},
                   {0x81, XFER_RESULT_SUCCESS, 1}};
  for (unsigned i = 0; i < sizeof(malformed) / sizeof(malformed[0]); i++) {
    reset();
    submit(0xff, 4097, false);
    assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 4096));
    assert(_mscd_itf.stage == EXPECT_DISCARD && _mscd_itf.total_len == 1 && queued[1].size == 1);
    assert(!mscd_xfer_cb(0, malformed[i].ep, malformed[i].result, malformed[i].bytes));
    assert(_mscd_itf.stage == MSC_STAGE_NEED_RESET && stalled[1] && stalled[0x81]);
    assert(_mscd_itf.total_len == 1 && _mscd_itf.xferred_len == 0 && queued_count == 2);
    no_acceptance();
    check_sentinels();
  }
}
static void test_read_write_and_deferred_retry(void) {
  reset();
  submit(SCSI_CMD_READ_10, 8192, true);
  assert(_mscd_itf.stage == MSC_STAGE_DATA && read_calls == 1 && queued[0].size == 4096);
  assert(mscd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, 4096));
  assert(read_calls == 2 && queued[1].size == 4096);
  assert(mscd_xfer_cb(0, 0x81, XFER_RESULT_SUCCESS, 4096));
  assert(_mscd_itf.stage == MSC_STAGE_STATUS_SENT && _mscd_itf.csw.status == MSC_CSW_STATUS_PASSED);
  accept_status();
  assert(reads == 1 && completed == 1 && !command_active);

  reset();
  submit(SCSI_CMD_WRITE_10, 8192, false);
  assert(_mscd_itf.stage == MSC_STAGE_DATA && queued[0].size == 4096 && write_calls == 0);
  write_busy = true;
  memset(_mscd_buf, 0x3c, 4096);
  assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 4096));
  uint32_t generation = 0;
  assert(tud_msc_write10_retry_snapshot(&generation) && generation != 0);
  assert(write_calls == 1 && queued_count == 1 && _mscd_itf.xferred_len == 0 && command_active);
  assert(!tud_msc_write10_retry(generation + 1));
  write_busy = false;
  assert(tud_msc_write10_retry(generation));
  assert(write_calls == 2 && queued_count == 2 && _mscd_itf.xferred_len == 4096);
  assert(!tud_msc_write10_retry(generation));
  assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 4096));
  assert(write_calls == 3 && _mscd_itf.stage == MSC_STAGE_STATUS_SENT);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_PASSED && _mscd_itf.csw.data_residue == 0);
  accept_status();
  assert(writes == 1 && completed == 1 && begins == 1 && !command_active);
  assert(generic_calls == 0);
  check_sentinels();

  reset();
  submit(SCSI_CMD_WRITE_10, 512, false);
  write_busy = true;
  assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 512));
  assert(tud_msc_write10_retry_snapshot(&generation));
  bot_reset();
  assert(!tud_msc_write10_retry(generation) && !tud_msc_write10_retry_snapshot(&generation));
  assert(resets == 1 && !command_active);
}
static void test_neighbor_paths(void) {
  reset();
  submit(0xff, 4096, false);
  assert(_mscd_itf.stage == MSC_STAGE_DATA && queued_count == 1 && queued[0].size == 4096);
  assert(generic_calls == 0);
  assert(mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, 4096));
  assert(generic_calls == 1 && _mscd_itf.stage == MSC_STAGE_STATUS_SENT);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_FAILED && _mscd_itf.csw.data_residue == 0);
  accept_status();
  assert(completed == 1 && !command_active);

  reset();
  submit(0xff, 0, false);
  assert(generic_calls == 1 && _mscd_itf.stage == MSC_STAGE_STATUS_SENT && queued_count == 1);
  assert(_mscd_itf.csw.status == MSC_CSW_STATUS_FAILED && _mscd_itf.csw.data_residue == 0);
  accept_status();
  assert(completed == 1 && !command_active);

  reset();
  submit(0xff, 8192, true);
  assert(generic_calls == 1 && _mscd_itf.stage == MSC_STAGE_STATUS && stalled[0x81]);
  assert(!stalled[1] && queued_count == 0 && _mscd_itf.csw.status == MSC_CSW_STATUS_FAILED);
  accept_status();
  assert(_mscd_itf.csw.data_residue == 8192 && completed == 1 && !command_active);
}
static void test_reset_stale_completion(uint32_t bytes) {
  reset();
  submit(0xff, 8192, false);
  check_discard_start(8192);
  bot_reset();
  assert(_mscd_itf.stage == MSC_STAGE_CMD && begins == 1 && resets == 1 && !command_active);
  /* The real old CBW is still present: no new host CBW has been copied. */
  assert(!mscd_xfer_cb(0, 1, XFER_RESULT_SUCCESS, bytes));
  assert(_mscd_itf.stage == MSC_STAGE_NEED_RESET && stalled[1] && stalled[0x81]);
  assert(begins == 1 && completed == 0 && !command_active);
  no_acceptance();
  bot_reset();
  clear_endpoint(0x81);
  clear_endpoint(1);
  fresh_read_after_recovery();
}
int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "reset-stale31") == 0) {
    test_reset_stale_completion(31);
    return 0;
  }
  if (argc > 1 && strcmp(argv[1], "reset-stale4096") == 0) {
    test_reset_stale_completion(4096);
    return 0;
  }
  assert(legacy_status_tests() == 0);
  test_all_discard_paths();
  test_malformed_discard();
  test_malformed_tail();
  test_clear_during_discard();
  test_read_write_and_deferred_retry();
  test_neighbor_paths();
  test_reset_stale_completion(31);
  test_reset_stale_completion(4096);
  return 0;
}
'''


def replace_once(text, before, after):
    assert text.count(before) == 1, before
    return text.replace(before, after, 1)


def harness():
    text = replace_once(BASE.HARNESS, '#include "class/msc/msc_device.c"',
                        '#include "class/msc/msc_device.c"\n' + EXTRA_DECLARATIONS)
    text = replace_once(text,
                        '  (void)rhport; (void)ep; (void)buffer; (void)size;\n  return true;',
                        '  (void)rhport;\n  check_sentinels();\n'
                        '  assert(queued_count < 128);\n'
                        '  if (buffer == _mscd_buf) assert(size <= sizeof(_mscd_buf));\n'
                        '  queued[queued_count].ep = ep; queued[queued_count].buffer = buffer;\n'
                        '  queued[queued_count++].size = size;\n  return true;')
    text = replace_once(text, "(void)lun; memset(v, ' ', 8);", "(void)lun; builtin_calls++; memset(v, ' ', 8);")
    text = replace_once(text, '(void)lun; return true; }', '(void)lun; builtin_calls++; return true; }')
    text = replace_once(text, '(void)lun; *blocks = 16;', '(void)lun; builtin_calls++; *blocks = 16;')
    text = replace_once(text, '(void)lun; (void)lba; (void)off; memset(b, 0, n);',
                        '(void)lun; (void)lba; (void)off; read_calls++; memset(b, 0, n);')
    text = replace_once(text, '(void)lun; (void)lba; (void)off; (void)b; return (int32_t)n;',
                        '(void)lun; (void)lba; (void)off; (void)b; write_calls++; return write_busy ? 0 : (int32_t)n;')
    text = replace_once(text, '(void)lun; (void)cmd; (void)b; (void)n; return -1;',
                        '(void)lun; (void)cmd; (void)b; (void)n; generic_calls++; return -1;')
    text = replace_once(text, '  pending_eject = false;\n  completed++;',
                        '  pending_eject = false;\n  command_active = false;\n  completed++;')
    text = replace_once(text, '  mscd_init();',
                        '  mscd_init();\n  begins = resets = generic_calls = read_calls = write_calls = builtin_calls = write_checks = 0;\n'
                        '  queued_count = 0; command_active = write_busy = false;\n'
                        '  fixture_buffer.before = UINT64_C(0x53a9e18d72c406bf);\n'
                        '  fixture_buffer.after = UINT64_C(0xb086e52a34f719cd);')
    return replace_once(text, 'int main(void)', 'static int legacy_status_tests(void)') + EXTRA_TESTS


def inherited_large_out_failure(driver):
    """Restore the inherited rejection at the same large-OUT decision."""
    marker = 'if (p_cbw->total_bytes > sizeof(_mscd_buf))'
    assert driver.count(marker) == 1, 'cannot isolate large-OUT admission'
    opening = driver.index('{', driver.index(marker))
    depth, end = 1, opening + 1
    while depth:
        depth += (driver[end] == '{') - (driver[end] == '}')
        end += 1
    assert 'MSC_STAGE_DISCARD' in driver[opening:end], 'discard admission missing'
    return driver[:opening + 1] + '\n            fail_scsi_op(rhport, p_msc, MSC_CSW_STATUS_FAILED);\n          ' + driver[end - 1:]


def run_source(source, scenario, negative=False, remove_reset_guard=False):
    with tempfile.TemporaryDirectory(prefix='msc-drain-') as directory:
        temp = Path(directory)
        (temp / 'tusb_config.h').write_text(BASE.CONFIG.replace('CFG_TUD_MSC_EP_BUFSIZE 512', 'CFG_TUD_MSC_EP_BUFSIZE 4096'))
        driver = source.read_text()
        if negative:
            driver = inherited_large_out_failure(driver)
        if remove_reset_guard:
            driver = replace_once(driver, '  p_msc->cbw.signature = 0;', '')
        declaration = 'CFG_TUSB_MEM_SECTION CFG_TUSB_MEM_ALIGN static uint8_t _mscd_buf[CFG_TUD_MSC_EP_BUFSIZE];'
        driver = replace_once(driver, declaration, GUARDED_BUFFER)
        overlay = temp / 'class/msc'
        overlay.mkdir(parents=True)
        (overlay / 'msc_device.c').write_text(driver)
        (overlay / 'msc_device.h').symlink_to(LIVE / 'src/class/msc/msc_device.h')
        test = temp / 'test.c'
        test.write_text(harness())
        binary = temp / 'test'
        subprocess.run([*shlex.split(os.environ.get('CC', 'cc')), '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                        '-ffunction-sections', '-fdata-sections', *shlex.split(os.environ.get('USB_TEST_CFLAGS', '')),
                        '-I', str(temp), '-I', str(LIVE / 'src'), '-I', str(LIVE / 'src/class/msc'),
                        str(test), '-Wl,--gc-sections', '-o', str(binary)], check=True)
        command = [str(binary)] + ([] if scenario == 'all' else [scenario])
        result = subprocess.run(command, cwd=temp, capture_output=True, text=True, timeout=10)
        if remove_reset_guard:
            if result.returncode == 0 or '!mscd_xfer_cb' not in result.stderr:
                raise AssertionError(f'reset guard removal did not reproduce stale admission: {result.returncode}\n{result.stderr}')
            print('NEGATIVE PASS: removing only reset signature invalidation re-admits old 31-byte completion')
        elif negative:
            if result.returncode == 0 or '_mscd_itf.stage == EXPECT_DISCARD' not in result.stderr:
                raise AssertionError(f'old driver did not fail real discard behavior: {result.returncode}\n{result.stderr}')
            print('NEGATIVE PASS: inherited large-OUT rejection reproduces failed DISCARD admission')
        elif result.returncode:
            raise AssertionError(f'{source} scenario={scenario} exited {result.returncode}\n{result.stderr}')
        else:
            print(f'PASS: actual MSC driver scenario={scenario}; bounded discard/callback/lifecycle/sentinel checks')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=CANDIDATE)
    parser.add_argument('--scenario', choices=('all', 'reset-stale31', 'reset-stale4096'), default='all')
    parser.add_argument('--negative-old', action='store_true')
    parser.add_argument('--negative-reset-guard', action='store_true')
    args = parser.parse_args()
    run_source(args.source, args.scenario)
    if args.negative_old:
        run_source(args.source, 'all', negative=True)
    if args.negative_reset_guard:
        run_source(args.source, 'reset-stale31', remove_reset_guard=True)


if __name__ == '__main__':
    main()
