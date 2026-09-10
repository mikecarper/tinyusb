#!/usr/bin/env python3
"""Read-only check of every build/test workflow for one exact TinyUSB commit."""

import argparse
import json
import subprocess
import sys


REQUIRED_WORKFLOWS = frozenset({
    "Build ARM", "Build AArch64", "Build ESP", "Build MSP430",
    "Build Renesas", "Build RISC-V", "nRF5x USB power regression",
})
# This dispatches writes to hathach's other repositories and is intentionally
# guarded off in this fork. Never trigger it to qualify a build.
UPSTREAM_ONLY_WORKFLOWS = frozenset({"Trigger Repos"})


def assess(runs, commit):
    """Return (exit code, status rows): 0 all pass, 1 failed, 2 incomplete."""
    latest = {}
    for run in runs:
        if run["headSha"] != commit:
            continue
        name = run["workflowName"]
        if name in UPSTREAM_ONLY_WORKFLOWS:
            continue
        if name not in latest or run["databaseId"] > latest[name]["databaseId"]:
            latest[name] = run

    rows = []
    failed = False
    incomplete = False
    # Also check new, unlisted workflows if they ran for this commit.
    for name in sorted(REQUIRED_WORKFLOWS | latest.keys()):
        run = latest.get(name)
        if run is None:
            status, url = "MISSING", ""
            incomplete = True
        elif run["status"] != "completed":
            status, url = run["status"].upper(), run["url"]
            incomplete = True
        else:
            status, url = (run["conclusion"] or "UNKNOWN").upper(), run["url"]
            if run["conclusion"] != "success":
                failed = True
        rows.append((name, status, url))
    # Wait for the whole suite, even if one job has already failed.
    return (2 if incomplete else 1 if failed else 0), rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, help="GitHub owner/repository")
    parser.add_argument("--commit", help="Full commit SHA (default: local HEAD)")
    args = parser.parse_args()
    try:
        commit = args.commit or subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, timeout=10).strip()
        output = subprocess.check_output([
            "gh", "run", "list", "--repo", args.repo, "--commit", commit,
            "--limit", "100", "--json",
            "databaseId,workflowName,headSha,status,conclusion,url",
        ], text=True, timeout=60)
        runs = json.loads(output)
        if len(runs) >= 100:
            print("INCOMPLETE: run listing may be truncated; inspect all runs.", file=sys.stderr)
            return 2
        result, rows = assess(runs, commit)
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError) as exc:
        print("Unable to verify CI: {}".format(exc), file=sys.stderr)
        return 2
    print("Commit: {}".format(commit))
    for name, status, url in rows:
        print("{:<28} {:<12} {}".format(name, status, url))
    print("PASS: all build/test workflows succeeded." if result == 0 else
          "NOT READY: inspect failures and wait for every required workflow.")
    return result


if __name__ == "__main__":
    sys.exit(main())
