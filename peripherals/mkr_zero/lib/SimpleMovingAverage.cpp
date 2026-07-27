#include <string.h>   // memset
#include <math.h>     // isnan

#include "SignalProcessingFunctions.h"

SimpleMovingAverage::SimpleMovingAverage(const FilterWSize wSize):
    windowSize(wSize),
    outputBuffer(new float[wSize]),
    bufferIndex(0),
    reciprocalDivisor(NAN),
    sum(0),
    outputValid(false){
        memset(outputBuffer, 0, windowSize * sizeof(float)); //can get away with memset 0 because arduino uses IEEE 754
        switch (windowSize){
            case SIZE_8:
                reciprocalDivisor = 0.125f;
                break;
            case SIZE_16:
                reciprocalDivisor = 0.0625f;
                break;
            case SIZE_32:
                reciprocalDivisor = 0.03125f;
                break;
            default:
                reciprocalDivisor = NAN;
                break;
        }
    }

SimpleMovingAverage::~SimpleMovingAverage(){
    delete[] outputBuffer;
}

bool SimpleMovingAverage::calculate(float &input,float &output){
    if (isnan(reciprocalDivisor)){
        return false;
    }
    sum += input - outputBuffer[bufferIndex];
    outputBuffer[bufferIndex] = input;
    bufferIndex = (bufferIndex + 1) & (windowSize - 1); //wrap around index for window size power of 2
    output = sum * reciprocalDivisor;

    if (!outputValid && bufferIndex  == 0){
        outputValid = true;
    }
    return outputValid; //warm-up handling, set to true after first windowSize samples
}

void SimpleMovingAverage::reset(){
    memset(outputBuffer, 0, windowSize * sizeof(float));
    bufferIndex = 0;
    sum = 0;
    outputValid = false;
}