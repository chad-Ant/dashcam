#include "../lib/es3Video/PixelCopy.h"
#include <cassert>
#include <climits>
#include <algorithm>
#include <cstdio>
using namespace es3video;
int main() {
    uint16_t source[32], output[32];
    for (unsigned i=0;i<32;++i) source[i]=uint16_t(i+1);
    std::fill(output,output+32,0xbeef);
    DecodedPixels pixels;
    assert(!pixels.begin(output+1,3,9,26));
    assert(!pixels.copy(0,0,3,4,8,source));
    assert(pixels.begin(output+1,3,9,27));
    assert(!pixels.copy(INT_MAX,0,INT_MAX,INT_MAX,1,source));
    assert(!pixels.copy(-1,0,3,4,1,source));
    assert(!pixels.copy(0,0,3,2,1,source));
    assert(!pixels.copy(0,0,3,4,INT_MAX,source));
    assert(!pixels.copy(0,9,3,4,1,source));
    for (auto value : output) assert(value==0xbeef);
    assert(pixels.copy(0,0,3,4,8,source) && !pixels.complete());
    // Padded source rows must be clipped at the viewport's bottom.
    assert(pixels.copy(0,8,3,4,8,source) && pixels.complete());
    for (unsigned y=0;y<9;++y)
        for (unsigned x=0;x<3;++x) assert(output[1+y*3+x]==source[(y%8)*4+x]);
    assert(output[0]==0xbeef && output[28]==0xbeef);
    assert(!pixels.copy(0,0,3,4,1,source)); // repeated output cannot exceed the image budget
    assert(!pixels.begin(nullptr,3,9,27) && !pixels.complete());

    std::fill(output,output+32,0xbeef);
    assert(copyFrameStrip(output+1,24,source,3,9,3,9,0)==8);
    assert(std::equal(source,source+24,output+1));
    assert(copyFrameStrip(output+1,24,source,3,9,3,9,8)==1);
    assert(std::equal(source+24,source+27,output+1));
    std::fill(output,output+32,0xbeef);
    assert(!copyFrameStrip(output+1,23,source,3,9,3,9,0));
    assert(!copyFrameStrip(output+1,24,source,3,9,4,9,0));
    assert(!copyFrameStrip(output+1,24,source,3,9,3,8,0));
    assert(!copyFrameStrip(output+1,24,source,3,9,3,9,UINT_MAX));
    assert(!copyFrameStrip(output+1,24,nullptr,3,9,3,9,0));
    for (auto value : output) assert(value==0xbeef);
    puts("decoder block bounds and display strip checks passed");
}
