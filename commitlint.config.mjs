// Conventional Commits, with the scopes this repository actually has.
//
// Enforced in CI on every push and pull request (.github/workflows/ci.yml).
// To get the same check before you commit, once per clone:
//
//   git config core.hooksPath .githooks
//
// Subjects stay lowercase and in the imperative ("add", not "Added"); the
// body is where the reasoning goes, and this repository leans on it heavily --
// several commits exist mainly to record a measurement.
export default {
  extends: ["@commitlint/config-conventional"],
  rules: {
    // Empty scope is allowed; when there is one it names a half of the repo.
    "scope-enum": [
      2,
      "always",
      ["python", "cpp", "web", "docs", "exercises", "quiz", "ci", "deps"],
    ],
    "header-max-length": [2, "always", 72],
    "body-max-line-length": [2, "always", 80],
  },
};
