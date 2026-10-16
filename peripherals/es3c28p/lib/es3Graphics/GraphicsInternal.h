#pragma once
// Private implementation header. Only the Arduino loop task may access panel.
// Splitting dashboard, assets, touch and streams does not create new owners.
#include "es3Graphics.h"
#include <Adafruit_ILI9341.h>
#include <cstring>
namespace es3graphics { extern Adafruit_ILI9341 panel; }
using es3graphics::panel;
