#include "GraphicsInternal.h"
#include "BmpLayout.h"

bool Es3Graphics::drawBmp24FromSd(const char* path, int x, int y,
                                   uint16_t width, uint16_t height)
{
    if (!_started || path == nullptr || x < 0 || y < 0 || !width || !height ||
        x > kScreenWidth || y > kTabTop || width > kScreenWidth - x || height > kTabTop - y) {
        return false;
    }

    File file;
    if (!_storage.openRead(path, file)) {
        Serial0.printf("[es3] splash open failed: %s\n", path);
        return false;
    }
    const uint32_t startedUs = micros();

    uint8_t header[54];
    es3graphics::BmpLayout layout;
    if (file.read(header, sizeof(header)) != sizeof(header) ||
        !es3graphics::bmpLayout(header, sizeof(header), file.size(), width, height, layout)) {
        Serial0.printf("[es3] splash invalid or truncated BMP: %s\n", path);
        file.close();
        return false;
    }
    const uint32_t pixelOffset = layout.offset, rowBytes = layout.rowBytes;
    const uint32_t pixelEnd = pixelOffset + rowBytes * height;

    uint8_t bgr[3 * kScreenWidth]{};
    uint16_t rgb565[kScreenWidth]{};
    for (uint16_t sourceRow = 0; sourceRow < height; ++sourceRow) {
        const uint32_t offset = pixelOffset + static_cast<uint32_t>(sourceRow) * rowBytes;
        if (!file.seek(offset) || file.read(bgr, rowBytes) != rowBytes) {
            Serial0.printf("[es3] splash read error at row %u\n", sourceRow);
            file.close();
            return false;
        }
        for (uint16_t column = 0; column < width; ++column) {
            rgb565[column] = panel.color565(bgr[column * 3 + 2],
                                             bgr[column * 3 + 1],
                                             bgr[column * 3]);
        }
        const int destinationRow = layout.bottomUp
                                       ? y + height - 1 - sourceRow  // bottom-up BMP
                                       : y + sourceRow;              // top-down BMP
        panel.drawRGBBitmap(x, destinationRow, rgb565, width, 1);
    }
    file.close();
    const uint32_t elapsedUs = micros() - startedUs;
    Serial0.printf("[es3] BMP splash rendered: %s (%lu bytes, %lu ms, SD %lu MHz)\n",
                  path, static_cast<unsigned long>(pixelEnd),
                  static_cast<unsigned long>((elapsedUs + 500U) / 1000U),
                  static_cast<unsigned long>(_storage.busFrequencyHz() / 1000000U));
    return true;
}

bool Es3Graphics::drawRgb565FromSd(const char* path, int x, int y,
                                    uint16_t width, uint16_t height)
{
    if (!_started || path == nullptr || x < 0 || y < 0 || !width || !height ||
        x > kScreenWidth || y > kTabTop || width > kScreenWidth - x || height > kTabTop - y) {
        return false;
    }

    File file;
    if (!_storage.openRead(path, file)) {
        Serial0.printf("[es3] RGB565 splash open failed: %s\n", path);
        return false;
    }

    const uint32_t expectedBytes = static_cast<uint32_t>(width) * height * 2U;
    if (file.size() != expectedBytes) {
        Serial0.printf("[es3] RGB565 splash size error: %s expected=%lu actual=%lu\n",
                      path, static_cast<unsigned long>(expectedBytes),
                      static_cast<unsigned long>(file.size()));
        file.close();
        return false;
    }

    // The asset is explicitly big-endian (MSB first) RGB565, identical to
    // ILI9341 SPI wire order. Keeping it this way avoids a per-pixel byte swap.
    constexpr uint16_t kPixelsPerBlock = 320;
    uint16_t pixels[kPixelsPerBlock];
    const uint32_t startedUs = micros();
    uint32_t pixelsRemaining = static_cast<uint32_t>(width) * height;

    panel.startWrite();
    panel.setAddrWindow(x, y, width, height);
    while (pixelsRemaining > 0) {
        const uint16_t pixelsThisBlock = static_cast<uint16_t>(min<uint32_t>(
            pixelsRemaining, kPixelsPerBlock));
        const size_t byteCount = static_cast<size_t>(pixelsThisBlock) * 2U;
        if (file.read(reinterpret_cast<uint8_t*>(pixels), byteCount) != byteCount) {
            panel.endWrite();
            file.close();
            Serial0.printf("[es3] RGB565 splash read error: %s\n", path);
            return false;
        }
        panel.writePixels(pixels, pixelsThisBlock, true, true);
        pixelsRemaining -= pixelsThisBlock;
    }
    panel.endWrite();
    file.close();

    const uint32_t elapsedUs = micros() - startedUs;
    Serial0.printf("[es3] RGB565 splash rendered: %s (%lu bytes, %lu ms, SD %lu MHz)\n",
                  path, static_cast<unsigned long>(expectedBytes),
                  static_cast<unsigned long>((elapsedUs + 500U) / 1000U),
                  static_cast<unsigned long>(_storage.busFrequencyHz() / 1000000U));
    return true;
}
