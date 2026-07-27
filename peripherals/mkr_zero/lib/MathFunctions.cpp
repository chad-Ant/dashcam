#include <string.h>   // memcpy for the IEEE-754 bit tricks

#include "MathFunctions.h"

float fastReciprocal(float num){
    float sign = num < 0 ? -1 : 1;
    unsigned int numBit;
    num = divThreshold(num) * sign;
    memcpy(&numBit, &num, sizeof(numBit));
    
    int guessBit = (int)(0x7EF127EA - numBit); //0x7EF127EA: initial guess
    float guess;
    memcpy(&guess, &guessBit, sizeof(guess));
#ifdef MATHLIB_ROBUST
    guess *= 2 - num * guess;
#endif
    return guess * (2 - num * guess) * sign;
}

float interpolate(float num, float uBound, float lBound, float resultuBound, float resultlBound){
    if (uBound < lBound) swapCustom(uBound,lBound);
    if (resultuBound < resultlBound) swapCustom(resultlBound,resultuBound);
    if (num >= uBound) return resultuBound;
    if (num <= lBound) return resultlBound;
    float deltaSource = (uBound - lBound) < 0.001 ? 0.001 : uBound - lBound;
    float deltaSink = resultuBound - resultlBound;
    return (num - lBound) * deltaSink * fastReciprocal(deltaSource) + resultlBound;
}

/* save for later
void inverse2x2(float **array){
    float det = array[0][0] * array[1][1] - array[0][1] * array[1][0];
    if (det == 0){
        array[0][0] = NAN;
        array[0][1] = NAN;
        array[1][0] = NAN;
        array[1][1] = NAN;
        return;
    }
    if (det > -0.001 && det < 0.001) det = det > 0 ? 0.001 : -0.001;
    float detReciprocal = fastReciprocal(det);
    float tempValue = array[0][0];
    array[0][0] = array[1][1] * detReciprocal;
    array[0][1] = -array[0][1] * detReciprocal;
    array[1][0] = -array[1][0] * detReciprocal;
    array[1][1] = tempValue * detReciprocal;
}
    */