#include "../lib/es3Video/JpegDecoder.h"
#include <assert.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <stdio.h>
static Es3JpegDecoder decoder;
static uint16_t output[320*240 + 2];
int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string root=argv[1];
    for (const char* name : {"color420.jpg", "color422.jpg", "color444map.jpg", "odd.jpg", "gray.jpg"}) {
        std::ifstream file(root+"/"+name,std::ios::binary);
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),{});
        const bool odd=std::string(name)=="odd.jpg";
        const int width=odd?319:320, height=odd?179:(std::string(name)=="color444map.jpg"?224:180);
        output[0]=0xbeef; output[width*height+1]=0xcafe;
        assert(decoder.decode(bytes.data(),bytes.size(),output+1,width,height));
        assert(output[0]==0xbeef && output[width*height+1]==0xcafe);
        if (std::string(name).find("color")==0) {
            const uint16_t red=output[1+40*320+40], green=output[1+40*320+160], blue=output[1+40*320+280];
            assert((red & 0xf800)>0xe000 && (red & 0x07ff)<0x0100);
            assert((green & 0x07e0)>0x0700 && (green & 0xf81f)<0x1000);
            assert((blue & 0x001f)>0x001c && (blue & 0xffe0)<0x0800);
        }
        // Deliberately corrupt SOF dimensions and marker lengths. Neither
        // may reach RGB output or cause the decoder to walk outside input.
        for (size_t i=2;i+9<bytes.size();++i) if (bytes[i]==0xff && bytes[i+1]==0xc0) {
            auto corrupt=bytes; corrupt[i+5]=0xff; corrupt[i+6]=0xff;
            assert(!decoder.decode(corrupt.data(),corrupt.size(),output+1,width,height));
            corrupt=bytes; corrupt[i+2]=0xff; corrupt[i+3]=0xff;
            assert(!decoder.decode(corrupt.data(),corrupt.size(),output+1,width,height));
            break;
        }
        assert(!decoder.decode(bytes.data(),bytes.size()-2,output+1,width,height));
        assert(!decoder.decode(bytes.data(),bytes.size(),output+1,320,240));
    }
    for (const char* name : {"progressive.jpg", "large.jpg", "broken.jpg"}) {
        std::ifstream file(root+"/"+name,std::ios::binary);
        std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),{});
        assert(!decoder.decode(bytes.data(),bytes.size(),output+1,320,180));
    }
    puts("JPEGDEC fixtures, bounds, RGB565 colors, malformed/progressive tests passed");
}
