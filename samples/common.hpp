#include <iostream>
#include <string>

#include <cepton_sdk_api.hpp>

void check_help(int argc, char **argv, const std::string &usage,
                const std::string &description = "") {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "-h") || (arg == "--help")) {
      std::printf("usage: %s\n", usage.c_str());
      if (!description.empty()) std::printf("\n%s\n", description.c_str());
      std::exit(0);
    }
  }
}