# OTAFIX fork qualification

Before declaring a pushed commit ready, check every build/test workflow:

```sh
python3 tools/ci_status.py --repo mikecarper/tinyusb
```

This read-only command requires GitHub CLI (`gh`) and checks local HEAD by
default. Use `--commit FULL_SHA` to inspect a different commit. Exit status 0
means all seven required workflows, plus any additional workflows reported
for that commit, succeeded. Exit 1 means a completed suite has a failure;
exit 2 means the suite is incomplete or could not be verified. Missing,
queued, cancelled, and skipped build/test runs must not be treated as passes.
Run the command again while status is incomplete.

`Trigger Repos` is intentionally excluded: its two jobs are guarded to run
only in `hathach/tinyusb`, and write to that owner's other repositories.
Do not remove those guards or dispatch external repositories to test this fork.

The build workflows retain this TinyUSB 0.12 branch's compatibility toolchains
and SDKs. These are CI tools, not a recommendation to ship a new application
using an end-of-life SDK. OTAFIX selects its board-qualified Arm GNU release separately.

Local host regressions:

```sh
python3 -B test/otafix/ci_status_test.py
python3 -B test/otafix/nrf5x_power_test.py
python3 -B test/otafix/nrf5x_dma_test.py
python3 -B test/otafix/nrf5x_dma_test.py --iso 0
python3 -B test/otafix/nrf5x_dma_test.py --errata-enabled 0 --scenarios identity
python3 -B test/otafix/ep0_setup_test.py
python3 -B test/otafix/breakpoint_header_test.py
python3 -B test/otafix/msc_status_test.py
python3 -B test/otafix/msc_drain_test.py
USB_TEST_CXXFLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  python3 -B test/otafix/nrf5x_power_test.py
USB_TEST_CXXFLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  python3 -B test/otafix/nrf5x_dma_test.py
```

The DMA regression compiles the production Nordic controller and ISR with
mocked hardware registers. It covers the anomaly 199 latch from
[`4bbb23545`](https://github.com/hathach/tinyusb/commit/4bbb23545a91926b9372554f5a31294d0a279829),
bounded SETUP/DMA synchronization and captured direction from
[`8ec9bf6ab`](https://github.com/hathach/tinyusb/commit/8ec9bf6ab5acb7cfc4f9a4483828862ab044298f),
and stale OUT recovery from
[`e99616dd6`](https://github.com/hathach/tinyusb/commit/e99616dd618f836ccd9cd8030542417a18d658c5).
The legacy driver also needs pending-operation cancellation and bounded DMA
retirement so old queued or in-flight work cannot reuse a new descriptor.
Scenarios exercise fresh packets after clear-halt, unrelated endpoints, reset,
removal, and interrupt preemption in both priority directions. These are host
regressions rather than physical USB qualification. The 43 cases also check
ordered, bounded SETUP register reads, saved IRQ state, and the factored Nordic
171/187 unlock-write sequence against actual register operations. The separate
breakpoint-header regression preserves the debugger-enable guard, single
volatile DHCSR read, and assertion return values after outlining the helper;
a guard-bypass mutation must fail.

The EP0 status regressions require the synthetic completion to follow the
actual `EP0STATUS` task, including when bulk DMA delays that task. They check
that a newer SETUP cancels pending status without a phantom completion, that
completion reaches the queue before a subsequent SETUP, and that unrelated
bulk work and the real ISR/thread context remain intact.

The SETUP generation regressions exercise the admission contract from
[#4068](https://github.com/hathach/tinyusb/pull/4068). Superseded requests cannot
arm data/status or stall EP0, including before the SETUP ISR runs, after a
bounded SETUP-read timeout, and across reset/removal. Co-captured old EP0 events
cannot complete the newer request. The separate core regression compiles real
USBD request dispatch and control handling: rejected generations preserve EP0
ownership, fresh GET_STATUS completes, and controllers without the optional
admission hook retain the legacy path. Removing the admission gate reproduces
stale dispatch in its negative control. These host checks do not establish the
cause or resolution of the physical concurrent GET_STATUS failure.

ISO remains enabled by default in the generic driver. OTAFIX selects
`CFG_TUD_NRF5X_ISO=0`; both configurations run the DMA suite, including endpoint
bounds, ISO rejection when disabled, and retained remote-wakeup SOF behavior.

The MSC status regression compiles the complete BOT driver and checks that a
failed eject cannot complete media removal, including an eject rejected after
its acceptance callback because the host requested an unexpected data stage.
READ10/WRITE10 and generic lifecycle callbacks still run after failed status.
Its negative control restores the inherited callback and reproduces the exit.

The MSC drain regression exercises the complete driver with a 4096-byte buffer.
Large rejected non-READ10/WRITE10 OUT commands are discarded in bounded chunks
before a failed CSW reports the full unprocessed residue. Buffer sentinels,
callback exclusion, command lifecycle, malformed chunks, reset recovery, and
deferred WRITE retries are checked. The inherited stall branch and removal of
the reset signature invalidation each reproduce a behavioral failure. This is
the accept-all alternative permitted by BOT 1.0 section 6.7.3; it does not
establish the cause of arbitrary EP0/bulk-halt interoperability failures.
