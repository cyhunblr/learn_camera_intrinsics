// Score your work in exercises.cpp.
//
//   ./build/bin/check_exercises              everything
//   ./build/bin/check_exercises 3 7          only exercises 3 and 7
//   ./build/bin/check_exercises --solutions  verify the reference answers
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "checks.hpp"

namespace {

const bool kTty = isatty(fileno(stdout));
const char* g(const char* code) { return kTty ? code : ""; }

}  // namespace

int main(int argc, char** argv) {
  bool useSolutions = false;
  std::vector<int> only;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--solutions") == 0) {
      useSolutions = true;
    } else if (argv[i][0] == '-') {
      std::cout << "usage: check_exercises [--solutions] [numbers...]\n";
      return 2;
    } else {
      only.push_back(std::atoi(argv[i]));
    }
  }

  const exlab::Impl& impl =
      useSolutions ? exlab::solutions() : exlab::exercises();
  const char* GREEN = g("\033[32m");
  const char* RED = g("\033[31m");
  const char* YELLOW = g("\033[33m");
  const char* DIM = g("\033[2m");
  const char* RESET = g("\033[0m");

  std::cout << "\nchecking " << impl.name << ".cpp\n\n";
  int passed = 0, todo = 0, total = 0;
  for (const exlab::Check& c : exlab::checks()) {
    if (!only.empty() &&
        std::find(only.begin(), only.end(), c.number) == only.end())
      continue;
    ++total;
    char label[64];
    std::snprintf(label, sizeof(label), "  %2d. %-30s", c.number, c.title);
    std::cout << label;
    try {
      c.run(impl);
      std::cout << GREEN << "pass" << RESET << "\n";
      ++passed;
    } catch (const exlab::NotImplemented&) {
      std::cout << YELLOW << "not implemented yet" << RESET << "\n";
      ++todo;
    } catch (const exlab::CheckFailed& e) {
      std::cout << RED << "FAIL" << RESET << "  " << e.what() << "\n";
    } catch (const std::exception& e) {
      std::cout << RED << "ERROR" << RESET << " " << e.what() << "\n";
    }
  }

  const int failed = total - passed - todo;
  std::cout << "\n  " << passed << "/" << total << " passed";
  if (failed) std::cout << ", " << failed << " failed";
  if (todo) std::cout << ", " << todo << " still to do";
  std::cout << "\n";
  if (passed == total)
    std::cout << "\n  " << GREEN << "All green." << RESET
              << " Now open solutions.cpp and compare approaches --\n"
                 "  there is usually more than one reasonable way to write "
                 "these.\n\n";
  else
    std::cout << "\n  " << DIM
              << "Specifications are in exercises.hpp; the theory is in docs/course/."
              << RESET << "\n\n";
  return passed == total ? 0 : 1;
}
