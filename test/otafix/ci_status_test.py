#!/usr/bin/env python3
"""Regression tests for the complete-suite CI qualification check."""

import importlib.util
import contextlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "ci_status", Path(__file__).resolve().parents[2] / "tools" / "ci_status.py")
CI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CI)
COMMIT = "1" * 40


def run(name, number=1, status="completed", conclusion="success", commit=COMMIT):
    return dict(workflowName=name, databaseId=number, headSha=commit,
                status=status, conclusion=conclusion,
                url="https://github.com/example/tinyusb/actions/runs/{}".format(number))


class CompleteSuiteTest(unittest.TestCase):
    def setUp(self):
        self.runs = [run(name) for name in CI.REQUIRED_WORKFLOWS]

    def test_all_seven_workflows_pass(self):
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 0)

    def test_one_successful_workflow_is_not_enough(self):
        self.assertEqual(CI.assess([run("Build ESP")], COMMIT)[0], 2)

    def test_empty_listing_is_not_success(self):
        self.assertEqual(CI.assess([], COMMIT)[0], 2)

    def test_old_commit_success_does_not_count(self):
        self.runs[0]["headSha"] = "2" * 40
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 2)

    def test_new_failure_supersedes_old_success(self):
        self.runs.append(run("Build ARM", number=2, conclusion="failure"))
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 1)

    def test_new_success_supersedes_old_failure(self):
        self.runs.extend([run("Build ARM", number=2, conclusion="failure"),
                          run("Build ARM", number=3)])
        self.assertEqual(CI.assess(list(reversed(self.runs)), COMMIT)[0], 0)

    def test_queued_run_is_incomplete(self):
        self.runs.append(run("Build ARM", number=2, status="queued", conclusion=""))
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 2)

    def test_waits_for_rest_even_after_failure(self):
        self.runs.extend([run("Build ARM", number=2, conclusion="failure"),
                          run("Build ESP", number=3, status="in_progress", conclusion="")])
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 2)

    def test_cancelled_and_skipped_required_runs_are_not_success(self):
        for conclusion in ("cancelled", "skipped", "timed_out", "neutral", ""):
            with self.subTest(conclusion=conclusion):
                runs = self.runs + [run("Build ARM", number=2, conclusion=conclusion)]
                self.assertEqual(CI.assess(runs, COMMIT)[0], 1)

    def test_new_workflow_failure_is_not_ignored(self):
        self.runs.append(run("Additional test", conclusion="failure"))
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 1)

    def test_upstream_only_dispatch_is_not_required(self):
        self.runs.append(run("Trigger Repos", conclusion="skipped"))
        self.assertEqual(CI.assess(self.runs, COMMIT)[0], 0)

    def test_query_failure_is_incomplete(self):
        for commit_args in ([], ["--commit", COMMIT]):
            with self.subTest(commit_args=commit_args):
                with patch("sys.argv", ["ci_status.py", "--repo", "example/tinyusb"] + commit_args):
                    with patch.object(CI.subprocess, "check_output", side_effect=FileNotFoundError("tool unavailable")):
                        with contextlib.redirect_stderr(io.StringIO()):
                            self.assertEqual(CI.main(), 2)

    def test_truncated_listing_is_not_success(self):
        with patch("sys.argv", ["ci_status.py", "--repo", "example/tinyusb", "--commit", COMMIT]):
            with patch.object(CI.subprocess, "check_output", return_value=json.dumps(self.runs * 15)):
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(CI.main(), 2)

    def test_dispatch_guards_remain_in_place(self):
        text = (Path(__file__).resolve().parents[2] / ".github/workflows/trigger.yml").read_text(encoding="ascii")
        self.assertEqual(text.count("if: github.repository == 'hathach/tinyusb'"), 2)


if __name__ == "__main__":
    unittest.main()
