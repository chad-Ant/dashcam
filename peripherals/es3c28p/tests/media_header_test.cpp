#include "../lib/es3Graphics/BmpLayout.h"
#include "../src/DashboardConfig.h"
#include "../lib/es3Audio/WavHeader.h"
#include "../lib/es3Audio/AudioLevel.h"
#include "../lib/es3Graphics/SettingsLayout.h"
#include <vector>
#include <cstring>
#include <cassert>
#include <cstdio>

static void u32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i)); }
static void bmp() {
    uint8_t h[54]{}; h[0] = 'B'; h[1] = 'M'; u32(h + 10,54); u32(h + 14,40);
    u32(h + 18,319); u32(h + 22,224); h[26] = 1; h[28] = 24;
    es3graphics::BmpLayout layout;
    const uint64_t fileBytes = 54 + 960 * 224;
    assert(es3graphics::bmpLayout(h,sizeof(h),fileBytes,319,224,layout));
    assert(layout.bottomUp && layout.rowBytes == 960 && layout.offset == 54);
    u32(h + 22,uint32_t(-224));
    assert(es3graphics::bmpLayout(h,sizeof(h),fileBytes,319,224,layout) && !layout.bottomUp);
    u32(h + 22,0x80000000u);
    assert(!es3graphics::bmpLayout(h,sizeof(h),fileBytes,319,224,layout));
    u32(h + 22,224); u32(h + 10,UINT32_MAX - 1);
    assert(!es3graphics::bmpLayout(h,sizeof(h),fileBytes,319,224,layout));
    u32(h + 10,20);
    assert(!es3graphics::bmpLayout(h,sizeof(h),fileBytes,319,224,layout));
    u32(h + 10,54); u32(h + 14,UINT32_MAX);
    assert(!es3graphics::bmpLayout(h,sizeof(h),fileBytes,319,224,layout));
    u32(h + 14,40);
    assert(!es3graphics::bmpLayout(h,sizeof(h),fileBytes-1,319,224,layout));
    assert(!es3graphics::bmpLayout(h,53,fileBytes,319,224,layout));
    assert(!es3graphics::bmpLayout(h,sizeof(h),fileBytes,0,224,layout));
}
struct Reader {
    std::vector<uint8_t> data;
    size_t pos = 0;
    size_t position() const { return pos; }
    size_t size() const { return data.size(); }
    bool seek(size_t p) { if (p > data.size()) return false; pos = p; return true; }
    size_t read(uint8_t* out, size_t n) {
        if (n > data.size()-pos) return 0;
        memcpy(out,data.data()+pos,n); pos += n; return n;
    }
};
static Reader wav(unsigned junk = 0) {
    Reader r; r.data.resize(12); memcpy(r.data.data(),"RIFF",4); memcpy(r.data.data()+8,"WAVE",4);
    for (unsigned i = 0; i < junk; ++i) {
        size_t at = r.data.size(); r.data.resize(at+10);
        memcpy(r.data.data()+at,"JUNK",4); u32(r.data.data()+at+4,1);
    }
    size_t at = r.data.size(); r.data.resize(at+26+12);
    auto* p = r.data.data()+at; memcpy(p,"fmt ",4); u32(p+4,18);
    p[8] = 1; p[10] = 1; u32(p+12,8000); u32(p+16,8000); p[20] = 1; p[22] = 8;
    memcpy(p+26,"data",4); u32(p+30,4);
    u32(r.data.data()+4,uint32_t(r.data.size()-8)); return r;
}
static bool parse(Reader r) { es3audio::WavHeader<Reader> h(r); return h.parse(); }
static void wave() {
    auto r = wav(); es3audio::WavHeader<Reader> header(r);
    assert(header.parse() && header.sampleRate() == 8000 && header.dataBytes() == 4);
    assert(r.position() == r.size()-4);
    assert(parse(wav(3))); assert(!parse(wav(256)));
    auto bad = wav(); u32(bad.data.data()+4,UINT32_MAX); assert(!parse(bad));
    bad = wav(); u32(bad.data.data()+16,UINT32_MAX); assert(!parse(bad));
    bad = wav(); bad.data[22] = 2; assert(!parse(bad)); // stereo
    bad = wav(); bad.data[34] = 16; assert(!parse(bad)); // unsupported PCM depth
    bad = wav(); u32(bad.data.data()+24,48000); assert(!parse(bad));
    bad = wav(); u32(bad.data.data()+28,1); assert(!parse(bad)); // byte rate mismatch
    bad = wav(); u32(bad.data.data()+42,100); assert(!parse(bad)); // data outside RIFF
    bad = wav(); bad.data.resize(10); assert(!parse(bad));
}
static void levels() {
    using es3audio::scaleSample;
    assert(scaleSample(128, 100, 100) == 0);
    assert(scaleSample(255, 100, 100) == 127 * 256 && scaleSample(0, 100, 100) == -32768);
    assert(scaleSample(255, 100, 0) == 0 && scaleSample(0, 0, 100) == 0); // either level mutes
    assert(scaleSample(255, 100, 10) == 127 * 256 / 10);                  // master scales linearly
    assert(scaleSample(0, 80, 10) == -128 * 256 * 80 * 10 / 10000);       // boot tone at default master
    assert(scaleSample(0, 255, 255) == -32768);                           // out-of-range levels clamp
    assert(scaleSample(132, 50, 50) == 4 * 256 / 4);
    assert(kDefaultMasterVolume <= 100 && kDefaultSettings.masterVolume == kDefaultMasterVolume);
    assert(kDefaultSettings.brightness <= 100 && kDefaultSettings.micThreshold <= 100);
}
static void settingsLayout() {
    using namespace es3settings;
    // Slider value mapping: ends clamp, round trip through the knob position.
    assert(sliderValue(-5) == 0 && sliderValue(kTrackLeft) == 0);
    assert(sliderValue(kTrackRight) == 100 && sliderValue(400) == 100);
    for (int v = 0; v <= 100; ++v) assert(sliderValue(knobX(uint8_t(v))) == v);
    assert(knobX(200) == kTrackRight);
    // Hit targets: each control, gaps, and nothing in the title/footer/tabs.
    for (uint8_t s = 0; s < kSliderCount; ++s) {
        const Box z = sliderZone(s);
        assert(hitTest(z.x + z.w / 2, z.y + 1) == Target(VolumeTarget + s));
        assert(trackCenterY(s) - kKnobRadius >= z.y && trackCenterY(s) + kKnobRadius < z.y + z.h);
        assert(knobX(0) - kKnobRadius >= z.x && knobX(100) + kKnobRadius < z.x + z.w);
        assert(labelY(s) + 8 <= z.y && labelY(s) >= kTitleHeight);
    }
    const Box test = testButton(), mic = micIndicator(), recal = recalibrateButton();
    assert(hitTest(test.x + 2, test.y + 2) == TestTarget);
    assert(hitTest(recal.x + 2, recal.y + 2) == RecalibrateTarget);
    assert(hitTest(mic.x + 2, mic.y + 2) == NoTarget);   // indicator is display only
    assert(hitTest(160, 5) == NoTarget && hitTest(160, kFooterTop + 2) == NoTarget);
    // No two controls overlap, and all stay above the footer and the tabs.
    const Box all[] = {sliderZone(0), sliderZone(1), sliderZone(2), test, mic, recal};
    for (const Box& a : all) {
        assert(a.y >= kTitleHeight && a.y + a.h <= kFooterTop && a.x >= 0 && a.x + a.w <= 320);
        for (const Box& b : all) {
            if (&a == &b) continue;
            assert(a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y);
        }
    }
    // Brightness never turns the backlight fully off.
    assert(brightnessDuty(0) == 8 && brightnessDuty(100) == 255 && brightnessDuty(250) == 255);
    for (int p = 1; p <= 100; ++p) assert(brightnessDuty(uint8_t(p)) >= brightnessDuty(uint8_t(p - 1)));
    // Mic indicator: grey with no source, silence or a level under threshold.
    assert(!micDetected(kNoMicLevel, 0) && !micDetected(0, 0));
    assert(!micDetected(49, 50) && micDetected(50, 50) && micDetected(1, 0));
}
int main() {
    const uint8_t count = sizeof(kDemoPages) / sizeof(kDemoPages[0]);
    assert(count == 6 && es3ValidPages(kDemoPages, count, 6));
    auto page = kDemoPages[0]; page.line4 = "Tap SET tab to calibrate";
    assert(!es3ValidPages(&page, 1, 6)); // Previous shipped configuration blocked graphics startup.
    page.line4 = "123456789012345678901234";
    assert(!es3ValidPages(&page, 1, 6));
    page.line4 = "12345678901234567890123";
    assert(es3ValidPages(&page, 1, 6));
    page.line4 = nullptr; assert(!es3ValidPages(&page, 1, 6));
    bmp(); wave(); levels(); settingsLayout(); puts("default page configuration, BMP/WAV bounds, volume level and settings layout checks passed");
}
