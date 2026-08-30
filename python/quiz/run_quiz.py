#!/usr/bin/env python3
"""A terminal quiz over everything in this repo.

    uv run python/quiz/run_quiz.py                 # 10 random questions
    uv run python/quiz/run_quiz.py -n 28           # the whole bank
    uv run python/quiz/run_quiz.py --topic D       # only distortion
    uv run python/quiz/run_quiz.py --study         # print every Q and A, no quiz

Numeric answers are graded with a tolerance, so round sensibly rather than
worrying about the last decimal.  Every question explains itself afterwards --
getting one wrong is the useful part.
"""

import argparse
import json
import os
import random
import sys
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
BANK = os.path.join(HERE, "questions.json")

TTY = sys.stdout.isatty()
GREEN, RED, BOLD, DIM, CYAN, RESET = (
    ("\033[32m", "\033[31m", "\033[1m", "\033[2m", "\033[36m", "\033[0m")
    if TTY else ("", "", "", "", "", "")
)
WRAP = textwrap.TextWrapper(width=78, initial_indent="  ", subsequent_indent="  ")


def load(topic=None):
    with open(BANK, encoding="utf-8") as fh:
        data = json.load(fh)
    qs = data["questions"]
    if topic:
        qs = [q for q in qs if q["topic"].lower() == topic.lower()]
    return data["title"], qs


def ask(q, index, total):
    print(f"\n{BOLD}[{index}/{total}]{RESET} {DIM}({q['topic']}){RESET}")
    print(WRAP.fill(q["q"]))
    if q["type"] == "mc":
        for i, opt in enumerate(q["options"]):
            print(f"    {chr(ord('a') + i)}) {opt}")
        prompt = "  your answer (a-%s, or 's' to skip): " % chr(ord('a') + len(q["options"]) - 1)
    else:
        prompt = "  your answer (a number, or 's' to skip): "

    try:
        raw = input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\n  (stopping)")
        return None

    if raw in ("s", "skip", ""):
        correct = None
    elif q["type"] == "mc":
        idx = ord(raw[0]) - ord("a") if raw and raw[0].isalpha() else -1
        correct = idx == q["answer"]
    else:
        try:
            correct = abs(float(raw) - q["answer"]) <= q.get("tol", 1e-6)
        except ValueError:
            print(f"  {DIM}(not a number -- counting that as a miss){RESET}")
            correct = False

    if correct is True:
        print(f"  {GREEN}correct{RESET}")
    else:
        right = (f"{chr(ord('a') + q['answer'])}) {q['options'][q['answer']]}"
                 if q["type"] == "mc" else f"{q['answer']}")
        head = f"  {DIM}skipped{RESET}" if correct is None else f"  {RED}not quite{RESET}"
        print(f"{head} -- the answer is {CYAN}{right}{RESET}")
    print(WRAP.fill(q["explain"]))
    return correct


def study(title, qs):
    print(f"\n{BOLD}{title} - study sheet ({len(qs)} questions){RESET}")
    for q in qs:
        print(f"\n{BOLD}{q['id']:2d}.{RESET} {DIM}({q['topic']}){RESET}")
        print(WRAP.fill(q["q"]))
        if q["type"] == "mc":
            for i, opt in enumerate(q["options"]):
                mark = f"{GREEN} <-{RESET}" if i == q["answer"] else ""
                print(f"    {chr(ord('a') + i)}) {opt}{mark}")
        else:
            print(f"    {GREEN}answer: {q['answer']}{RESET}")
        print(WRAP.fill(q["explain"]))
    print()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-n", "--count", type=int, default=10)
    ap.add_argument("--topic", default=None,
                    help="pinhole | K | D | resize | undistort | calibration")
    ap.add_argument("--study", action="store_true", help="print questions with answers")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    title, qs = load(args.topic)
    if not qs:
        print(f"no questions for topic {args.topic!r}")
        return 1
    if args.study:
        study(title, qs)
        return 0

    random.seed(args.seed)
    picked = random.sample(qs, min(args.count, len(qs)))
    print(f"\n{BOLD}{title}{RESET}  -  {len(picked)} questions"
          + (f", topic '{args.topic}'" if args.topic else ""))
    print(f"{DIM}Enter to skip a question. Ctrl-C to stop early.{RESET}")

    score = asked = 0
    for i, q in enumerate(picked, 1):
        res = ask(q, i, len(picked))
        if res is None and not TTY:
            break
        if res is not None:
            asked += 1
            score += int(res)

    print(f"\n{BOLD}score: {score}/{asked or len(picked)}{RESET}")
    if asked and score == asked:
        print(f"{GREEN}Every one. Try --topic calibration for the harder set.{RESET}\n")
    elif asked:
        pct = 100 * score / asked
        weak = "docs/course/ has a chapter per topic; the ones you missed are tagged above."
        print(f"{pct:.0f}%. {weak}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
