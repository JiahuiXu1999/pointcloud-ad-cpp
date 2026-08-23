#include <iostream>
#include <pointcloud_ad/version.hpp>
#include <string_view>

namespace {

void print_help() {
  std::cout << "PointCloudAD command-line interface\n\n"
               "Usage:\n"
               "  pcad --help       Show this help\n"
               "  pcad --version    Show the program version\n\n"
               "Inspection commands will be added in the vertical-slice milestones.\n";
}

} // namespace

int main(int argc, char* argv[]) {
  if (argc == 1) {
    print_help();
    return 0;
  }

  const std::string_view argument{argv[1]};
  if (argument == "--help" || argument == "-h") {
    print_help();
    return 0;
  }
  if (argument == "--version") {
    std::cout << "pcad " << pointcloud_ad::version_string() << '\n';
    return 0;
  }

  std::cerr << "Unknown argument: " << argument << "\nUse --help for usage.\n";
  return 2;
}
