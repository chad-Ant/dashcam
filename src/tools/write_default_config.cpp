// write_default_config <out.xml>
//
// Emits a dashcam config file populated entirely with the built-in default
// values (a default-constructed AppConfig serialized via ConfigReader::save).
// The Makefile runs this at build time to seed each build's config/ directory;
// libconfig recreates the same defaults at runtime via ConfigReader::loadOrCreate.

#include "libconfig.h"
#include <cstdio>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <out.xml>\n", argv[0]);
        return 2;
    }
    dashcam::config::AppConfig cfg;  // built-in defaults
    if (!dashcam::config::ConfigReader::save(argv[1], cfg)) {
        std::fprintf(stderr, "write_default_config: failed to write '%s'\n", argv[1]);
        return 1;
    }
    return 0;
}
