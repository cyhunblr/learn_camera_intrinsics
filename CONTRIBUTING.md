# Contributing

## Commit messages

[Conventional Commits](https://www.conventionalcommits.org/), checked in CI by
[commitlint](https://commitlint.js.org/) on every push and pull request. The
rules live in [`commitlint.config.mjs`](commitlint.config.mjs).

```text
feat(web): lock the camera pane to the sensor aspect
fix(python): reject rays past the fold radius
docs: explain why the fixed-point solver diverges
```

* **Type** — `feat`, `fix`, `docs`, `refactor`, `perf`, `test`, `build`, `ci`,
  `chore`, `style`, `revert`.
* **Scope** (optional) — one of `python`, `cpp`, `web`, `docs`, `exercises`,
  `quiz`, `ci`, `deps`. An unlisted scope is rejected, so the list stays honest.
* **Subject** — lowercase, imperative ("add", not "Added"), header under 72
  characters.
* **Body** — wrapped at 80. This is where the reasoning goes, and this
  repository leans on it: several commits exist mainly to record a
  measurement, and that is the point. A one-line commit that changes behaviour
  is the thing to avoid, not a long one.

To get the same check locally, once per clone:

```bash
git config core.hooksPath .githooks
```

The hook skips silently if `npx` is unavailable, so it never blocks a commit
it cannot actually check.

The whole history conforms: the subjects predating this rule were rewritten in
one pass, leaving every commit's content byte-identical. CI checks the commits
a push or pull request introduces, not the whole history, so it stays fast.

## Before you push

```bash
uv run pytest                       # the exercise solutions
cmake --build cpp/build -j && ctest --test-dir cpp/build
./scripts/check_parity.sh           # Python == C++ == the viewer's JavaScript
./scripts/check_markdown.sh         # the docs, linted like the code
./scripts/generate_figures.sh       # if you changed anything the figures show
```

The whole repository is headless — nothing opens a window — so all of it runs
over SSH and in CI.

## Writing docs

Markdown is linted with [markdownlint](https://github.com/DavidAnson/markdownlint),
configured in [`.markdownlint-cli2.jsonc`](.markdownlint-cli2.jsonc). Two things
are worth knowing before you write:

* **Prose wraps at 80.** Tables and code blocks do not have to; a wrapped shell
  command stops being copy-pastable, and a wrapped table stops being a table.
* **A long KaTeX formula gets an inline exemption** rather than a rule change,
  so the exemption sits next to the formula it excuses:

  ```text
  <!-- markdownlint-disable-next-line MD013 -->
  $$ ... a formula that will not fit in 80 columns ... $$
  ```

  For a multi-line `$$` block, put `disable`/`enable` directives *outside* the
  block — inside, KaTeX renders them as part of the maths.

`./scripts/check_markdown.sh --fix` repairs whatever is mechanically fixable.
Note that `docs/practice/quiz.md` is generated: fix the wording in
`python/quiz/questions.json` and re-run `python/quiz/export_markdown.py`.
