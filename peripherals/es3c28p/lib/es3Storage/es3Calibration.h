#pragma once

struct Es3TouchCalibrationRecord {
    bool xAxisIsRawX;
    int xAtLeft;
    int xAtLeftMiddle;
    int xAtRightMiddle;
    int xAtRight;
    int yAtTop;
    int yAtCenter;
    int yAtBottom;
};

// Native FT6336G coordinates. Strictly ordered interior knots prevent divide
// by zero and folds in the piecewise mapping, including after an SD edit.
inline bool es3CalibrationIsSane(const Es3TouchCalibrationRecord& r)
{
    const int xLimit = r.xAxisIsRawX ? 240 : 320;
    const int yLimit = r.xAxisIsRawX ? 320 : 240;
    const int xs[] = {r.xAtLeft, r.xAtLeftMiddle, r.xAtRightMiddle, r.xAtRight};
    const int ys[] = {r.yAtTop, r.yAtCenter, r.yAtBottom};
    for (const int x : xs) {
        if (x < 0 || x >= xLimit) return false;
    }
    for (const int y : ys) {
        if (y < 0 || y >= yLimit) return false;
    }
    const bool xIncreasing = xs[3] > xs[0];
    const bool yIncreasing = ys[2] > ys[0];
    for (int i = 1; i < 4; ++i) {
        if (xIncreasing ? xs[i] <= xs[i - 1] : xs[i] >= xs[i - 1]) return false;
    }
    for (int i = 1; i < 3; ++i) {
        if (yIncreasing ? ys[i] <= ys[i - 1] : ys[i] >= ys[i - 1]) return false;
    }
    const int xSpan = xIncreasing ? xs[3] - xs[0] : xs[0] - xs[3];
    const int ySpan = yIncreasing ? ys[2] - ys[0] : ys[0] - ys[2];
    return xSpan > 60 && ySpan > 60;
}
