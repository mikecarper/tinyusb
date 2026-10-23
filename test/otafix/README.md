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
python3 -B test/otafix/nrf5x_dma_test.py --quiet-mode 0
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --quiet-mode 0
python3 -B test/otafix/nrf5x_dma_test.py --negative-clear-active
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-clear-active
python3 -B test/otafix/nrf5x_dma_test.py --negative-abort-stale
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-abort-stale
python3 -B test/otafix/nrf5x_dma_test.py --negative-stall-dma
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-stall-dma
python3 -B test/otafix/nrf5x_dma_test.py --negative-abort-sibling
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-abort-sibling
python3 -B test/otafix/nrf5x_dma_test.py --negative-abort-epoch
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-abort-epoch
python3 -B test/otafix/nrf5x_dma_test.py --quiet-mode 1
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --quiet-mode 1
python3 -B test/otafix/nrf5x_dma_test.py --negative-quiet-status
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-quiet-status
python3 -B test/otafix/nrf5x_dma_test.py --negative-quiet-stall
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-quiet-stall
python3 -B test/otafix/nrf5x_dma_test.py --negative-quiet-masked
python3 -B test/otafix/nrf5x_dma_test.py --iso 0 --negative-quiet-masked
python3 -B test/otafix/nrf5x_dma_test.py --errata-enabled 0 --scenarios identity
python3 -B test/otafix/ep0_setup_test.py
python3 -B test/otafix/clear_halt_test.py
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
regressions rather than physical USB qualification. The 65 baseline cases also check
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

`CFG_TUD_NRF5X_CONTROL_STALL_QUIET` defaults to zero in the generic driver.
The bootloader opts in to a nominal 2 ms delay after actual EP0STATUS task and
completion enqueue, and after bulk STALL has finished its descriptor cleanup.
The global IRQ mask is restored before waiting; STALL retains its existing
USBD IRQ mask. This requires one serialized USB task, no reentrant or higher
priority endpoint STALL callers, and no caller-held global critical section.
An unexpected PRIMASK-masked caller disables/unplugs USB instead of introducing
a millisecond global wait. The check does not validate arbitrary BASEPRI or
FAULTMASK critical sections. The SDK delay is not a wire-handshake indicator.

Both ISO configurations run 73 cases with quiet spacing enabled and 66 with it
disabled. Additional cases check task/completion order, thread/ISR contexts,
finished OUT cleanup, continued USB/DMA progress from thread context, SETUP and
power boundaries, and the masked-caller policy. Three behavioral negatives
remove each post-task delay or the masked-caller guard. The core regression
also checks every endpoint number and direction with driver/flag sentinels;
an accessor mutation that aliases both directions must fail.

The CLEAR_FEATURE regression runs actual USBD request dispatch and control
completion. Endpoint-halt clear resets DATA0 even when already clear, as USB 2.0
section 9.4.5 requires, while preserving active transfer and claim ownership.
Both endpoint directions and repeated requests are covered; mutations restore
the inherited skip and prematurely release ownership to reproduce failures.
The Nordic regression also preserves an unhalted OUT packet/FIFO during this
toggle reset, while a genuinely halted OUT endpoint still drops stale data.

The optional OUT abort retires pending and active DMA before releasing buffer
ownership, flushes only that endpoint's cached data and END event, and preserves
halt and data toggle state. Disabled controllers and failed retirement fail
closed. The core regression checks successful, rejected, and unsupported aborts;
the Nordic tests cover stale END isolation, two reset-boundary flushes, IRQ
state, unrelated transfers, and both ISO configurations. Removing the END flush
reproduces an old completion reaching a newly armed buffer. Protocol callers
must also fence completions already queued before abort.

Retirement observes a sibling's active EasyDMA END before abort or close MMIO,
while leaving that sibling's raw event, descriptor and queued work intact.
Abort checks controller enable and removal generation again under a short
global IRQ mask before clearing its descriptor and FIFO. Tests cover sibling
EP0/CDC DMA, timeout, and removal/reattachment during and immediately after the
wait; owned-only waiting and a removed epoch guard each reproduce unsafe
cleanup. The clear-halt path is unchanged by this retirement fence.

Before writing a non-control endpoint's halt register, the Nordic driver waits
for the active EasyDMA END without consuming another endpoint's completion.
The bounded wait detaches on timeout; a short interrupt-masked commit checks
controller enable and removal generation before changing halt state. The tests
cover EP0 and CDC DMA in both directions, preserved sibling accounting and
pending transfers, removal/reattachment, and saved interrupt state. Removing
the fence reproduces an EPSTALL write before DMA finishes. This follows
[Nordic's register-access fence](https://github.com/NordicSemiconductor/nrfx/blob/e04a873799c0e7206166d62e2e4f185868acf8f9/drivers/src/nrfx_usbd.c#L1439-L1448);
these host checks do not establish resolution of the physical concurrent
GET_STATUS failure.

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
