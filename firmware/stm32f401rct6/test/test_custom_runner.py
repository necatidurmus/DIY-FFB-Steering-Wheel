import click

from platformio.test.result import TestCase, TestStatus
from platformio.test.runners.base import TestRunnerBase


class CustomTestRunner(TestRunnerBase):
    """Use the native program's exit status; no downloadable test library."""

    NAME = "custom"
    EXTRA_LIB_DEPS = []

    def on_testing_line_output(self, line):
        click.echo(line, nl=False)
        stripped = line.strip()
        if stripped.startswith("FAIL"):
            self.test_suite.add_case(
                TestCase("wheel_logic", TestStatus.FAILED, stripped)
            )
        elif stripped.startswith("PASS"):
            self.test_suite.add_case(
                TestCase("wheel_logic", TestStatus.PASSED)
            )
            self.test_suite.on_finish()
