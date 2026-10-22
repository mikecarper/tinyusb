#!/usr/bin/env python3
"""Actual factored TinyUSB breakpoint header with DHCSR/BKPT hardware seams."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


LIVE = Path(__file__).resolve().parents[2]
HEADER = LIVE / "src/common/tusb_verify.h"
CONFIG = r'''
#define CFG_TUSB_MCU OPT_MCU_NRF5X
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_DEBUG 0
'''
HARNESS = r'''
#include <cassert>
#include <cstdint>
static unsigned debug_reads, breakpoints, continued;
struct DebugRegister {
  uint32_t value;
  operator uint32_t() const {
    debug_reads++;
    return value;
  }
} mock_dhcsr;
static void mock_breakpoint() {
  breakpoints++;
}
#define __ARM_ARCH_7EM__ 1
#include "common/tusb_verify.h"
static bool one_argument(bool valid) {
  TU_ASSERT(valid);
  continued++;
  return true;
}
static int explicit_return(bool valid) {
  TU_ASSERT(valid, -7);
  continued++;
  return 17;
}
static void void_return(bool valid) {
  TU_ASSERT(valid, );
  continued++;
}
static int error_return(int error) {
  TU_ASSERT_ERR(error);
  continued++;
  return 17;
}
static int explicit_error_return(int error) {
  TU_ASSERT_ERR(error, -9);
  continued++;
  return 17;
}
static bool verify_only(bool valid) {
  TU_VERIFY(valid);
  continued++;
  return true;
}
int main() {
  const uint32_t values[] = {0, 0xfffffffeu, 1, 0xffffffffu};
  for (uint32_t value: values) {
    mock_dhcsr.value = value;
    debug_reads = breakpoints = continued = 0;
    assert(!one_argument(false));
    assert(explicit_return(false) == -7);
    void_return(false);
    assert(error_return(5) == 5);
    assert(explicit_error_return(5) == -9);
    assert(continued == 0 && debug_reads == 5);
    assert(breakpoints == ((value & 1) ? 5u : 0u));
    assert(!verify_only(false));
    assert(debug_reads == 5 && continued == 0);
    assert(one_argument(true));
    assert(explicit_return(true) == 17);
    void_return(true);
    assert(error_return(0) == 17);
    assert(explicit_error_return(0) == 17);
    assert(verify_only(true));
    assert(continued == 6 && debug_reads == 5);
    TU_BREAKPOINT();
    assert(debug_reads == 6 && breakpoints == ((value & 1) ? 6u : 0u));
  }
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=HEADER)
    args = parser.parse_args()
    original = args.source.read_text()
    header, registers = re.subn(
        r'volatile\s+uint32_t\s*\*\s*ARM_CM_DHCSR\s*=\s*\(volatile\s+uint32_t\s*\*\)\s*0xE000EDF0UL;',
        'DebugRegister *ARM_CM_DHCSR = &mock_dhcsr;', original)
    header, instructions = re.subn(r'__asm\("BKPT #0\\n"\);', 'mock_breakpoint();', header)
    assert registers == 1 and instructions == 1, 'expected actual one-read DHCSR/BKPT helper'
    condition = 'if ((*ARM_CM_DHCSR) & 1UL)'
    assert header.count(condition) == 1
    with tempfile.TemporaryDirectory(prefix='bkpt-test-') as directory:
        temp = Path(directory)
        (temp / 'common').mkdir()
        (temp / 'tusb_config.h').write_text(CONFIG)
        (temp / 'test.cpp').write_text(HARNESS)
        for label, source in (('candidate', header),
                              ('unguarded', header.replace(condition, 'if (((*ARM_CM_DHCSR) | 1UL) & 1UL)'))):
            (temp / 'common/tusb_verify.h').write_text(source)
            binary = temp / label
            subprocess.run([*shlex.split(os.environ.get('CXX', 'c++')), '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                            *shlex.split(os.environ.get('USB_TEST_CXXFLAGS', '')),
                            '-I', str(temp), '-I', str(LIVE / 'src'), '-I', str(LIVE / 'src/common'),
                            str(temp / 'test.cpp'), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], cwd=temp, capture_output=True, text=True, timeout=5)
            if label == 'candidate':
                if result.returncode:
                    raise AssertionError(result.stderr)
                print('BKPT PASS: actual helper debugger-bit guard, exactly one DHCSR read, assertion return values and VERIFY semantics')
            elif result.returncode == 0 or 'breakpoints ==' not in result.stderr:
                raise AssertionError('removing debugger guard did not reproduce failed behavior: ' + result.stderr)
            else:
                print('BKPT NEGATIVE PASS: removing debugger check fails actual helper behavior')


if __name__ == '__main__':
    main()
