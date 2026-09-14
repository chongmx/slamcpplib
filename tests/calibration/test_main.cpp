#include <string>

#include "TestHarness.hpp"

int main(int argc, char** argv) {
  // One optional argument, a substring filter on "Suite.Name".
  const std::string filter = argc > 1 ? argv[1] : "";
  return ::slamcpp::calib::test::run(filter);
}
