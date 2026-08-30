#!/usr/bin/env python3
"""Score your work in exercises.py.

    uv run python/exercises/check.py           # everything
    uv run python/exercises/check.py 3 7       # only exercises 3 and 7
    uv run python/exercises/check.py --solutions   # verify the reference answers
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from checks import CHECKS                                   # noqa: E402

GREEN, RED, YELLOW, DIM, RESET = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"
) if sys.stdout.isatty() else ("", "", "", "", "")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("only", nargs="*", type=int, help="exercise numbers to run")
    ap.add_argument("--solutions", action="store_true",
                    help="check solutions.py instead of exercises.py")
    args = ap.parse_args()

    module = __import__("solutions" if args.solutions else "exercises")
    selected = [c for c in CHECKS if not args.only or c[0] in args.only]

    print(f"\nchecking {module.__name__}.py\n")
    passed = todo = 0
    for num, title, fn in selected:
        label = f"  {num:2d}. {title:30s}"
        try:
            fn(module)
        except NotImplementedError:
            print(f"{label}{YELLOW}not implemented yet{RESET}")
            todo += 1
        except AssertionError as exc:
            print(f"{label}{RED}FAIL{RESET}  {exc}")
        except Exception as exc:                       # noqa: BLE001
            print(f"{label}{RED}ERROR{RESET} {type(exc).__name__}: {exc}")
        else:
            print(f"{label}{GREEN}pass{RESET}")
            passed += 1

    total = len(selected)
    failed = total - passed - todo
    print(f"\n  {passed}/{total} passed"
          + (f", {failed} failed" if failed else "")
          + (f", {todo} still to do" if todo else ""))
    if passed == total:
        print(f"\n  {GREEN}All green.{RESET} Now open solutions.py and compare "
              "approaches -- \n  there is usually more than one reasonable way "
              "to write these.\n")
    else:
        print(f"\n  {DIM}Hints live in the docstrings; the theory is in docs/course/."
              f"{RESET}\n")
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
