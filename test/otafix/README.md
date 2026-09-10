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
using an end-of-life SDK. OTAFIX's separate bootloader build stays on GCC 14.2.

Local host regressions:

```sh
python3 -B test/otafix/ci_status_test.py
python3 -B test/otafix/nrf5x_power_test.py
USB_TEST_CXXFLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  python3 -B test/otafix/nrf5x_power_test.py
```
