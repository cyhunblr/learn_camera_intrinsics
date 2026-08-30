"""pytest wrapper around the same checks check.py runs.

    uv run pytest -v

``test_solutions`` is the repo's own regression test: it proves the reference
answers are right, so a failure there means the repo is broken, not you.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import exercises                                             # noqa: E402
import solutions                                             # noqa: E402
from checks import CHECKS                                    # noqa: E402

IDS = [f"{n:02d}-{t.replace(' ', '-')}" for n, t, _ in CHECKS]


@pytest.mark.parametrize("num,title,check", CHECKS, ids=IDS)
def test_solutions(num, title, check):
    check(solutions)


@pytest.mark.parametrize("num,title,check", CHECKS, ids=IDS)
def test_exercises(num, title, check):
    try:
        check(exercises)
    except NotImplementedError:
        pytest.skip(f"exercise {num} ({title}) not implemented yet")
