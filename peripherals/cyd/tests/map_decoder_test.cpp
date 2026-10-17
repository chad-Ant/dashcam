#include "../lib/cydNavigation/MapDecoder.h"
#include <fstream>
#include <iterator>
#include <vector>
#include <cstdio>
static CydMapDecoder decoder;
static CydGraphics graphics;
int main(int argc, char** argv) {
    assert(argc == 2);
    for (const char* name : {"420.jpg", "422.jpg", "444.jpg", "gray.jpg"}) {
        std::ifstream file(std::string(argv[1])+"/"+name, std::ios::binary);
        std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), {});
        for (auto& pixel : graphics.pixels) pixel = 0xbeef;
        assert(decoder.draw(data.data(), data.size(), graphics));
        for (unsigned i = 320*224; i < 320*240; ++i) assert(graphics.pixels[i] == 0xbeef);
        if (std::string(name) != "gray.jpg") {
            assert((graphics.pixels[320*50+40] & 0xf800) > 0xe000);
            assert((graphics.pixels[320*50+160] & 0x07e0) > 0x0700);
            assert((graphics.pixels[320*50+280] & 0x001f) > 0x001c);
        }
        graphics.visible = false;
        assert(!decoder.draw(data.data(), data.size(), graphics));
        graphics.visible = true;
        assert(!decoder.draw(data.data(), data.size()-2, graphics));
        for (size_t i=2; i+9<data.size(); ++i) if (data[i]==0xff && data[i+1]==0xc0) {
            auto bad = data; bad[i+5] = 0xff; bad[i+6] = 0xff;
            assert(!decoder.draw(bad.data(), bad.size(), graphics));
            bad = data; bad[i+2] = 0xff; bad[i+3] = 0xff;
            assert(!decoder.draw(bad.data(), bad.size(), graphics));
            break;
        }
    }
    for (const char* name : {"large.jpg", "progressive.jpg", "broken.jpg"}) {
        std::ifstream file(std::string(argv[1])+"/"+name, std::ios::binary);
        std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)), {});
        assert(!decoder.draw(data.data(), data.size(), graphics));
    }
    puts("CYD MCU-to-display decode: colors, bounds, tabs, corrupt/progressive input passed");
}
