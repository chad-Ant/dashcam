#include "SignalProcessingFunctions.h"
#include "MathFunctions.h"
#define SEC_TO_MIN 0.0167f
//#define PID_BACK_CALC
//#define PID_ROBUST

// Initialiser order must match the member declaration order in the header
// (public sp/uBound/lBound first, then Kp..controlState) or GCC warns -Wreorder.
PIDControls::PIDControls(float gainP, float integralTime, float derivativeTime, float setpoint, float upperBound, float lowerBound):
    sp(setpoint),
    uBound(upperBound),
    lBound(lowerBound),
    Kp(gainP),
    Ti(integralTime),
    Td(derivativeTime),
    prevInput(0.0f),
    prevError(0.0f),
    integral(0.0f),
    derivative(0.0f),
    controlState(OFF){
        Ki = integralTime == 0.0f ? 0.0f : gainP / divThreshold(integralTime);
        Kd = gainP * derivativeTime;
    }

PIDControls::~PIDControls(){
    }

void PIDControls::reset(float input, float &output){
    controlState = ON;
    prevInput = input;
    prevError = input - sp;
    float tempGain = Kp * prevError;
    integral = output - tempGain;
    //integral = 0.0f; alternative way to reset integral
    derivative = 0.0f;
    output = tempGain;
    saturate(output, uBound, lBound);
}

bool PIDControls::calculate(float input, float &output, float dt_sec){
    if (dt_sec == 0.0f || controlState == OFF) return false; //no calculations take place

    float error = sp - input;
    integral += (error + prevError) *0.5f * dt_sec * SEC_TO_MIN; //change dt to minutes
    prevError = error;
#ifdef PID_ROBUST
    derivative = (prevInput - input) / divThreshold(dt_sec);
#else
    derivative = (prevInput - input) * fastReciprocal(dt_sec);
#endif
    prevInput = input;

    //back calculation
#ifdef PID_ROBUST
    float rawOutput = error * Kp + integral * Ki + derivative * Kd;
    output = rawOutput;
    saturate(output, uBound, lBound);
    if (Ki != 0){
        if (output == uBound || output == lBound) integral += (output - rawOutput) / divThreshold(Ki);
    }
#elif defined(PID_BACK_CALC)
    float rawOutput = error * Kp + integral * Ki + derivative * Kd;
    output = rawOutput;
    saturate(output, uBound, lBound);
    if (Ki != 0){
        if (output == uBound || output == lBound) integral += (output - rawOutput) * fastReciprocal(Ki);
    }
#else
    float tempGain = error * Kp;
    float tempIntegral = integral * Ki;
    output = tempGain + tempIntegral;
    saturate(output, uBound, lBound);
    if (output == uBound || output == lBound) integral = output - tempGain;
    output = tempGain + tempIntegral + derivative * Kd;
    saturate(output, uBound, lBound);
#endif

    return true;
}

void PIDControls::enable(float input, float output){
    reset(input, output);
}

void PIDControls::disable(){
    controlState = OFF;
}

bool PIDControls::getState(){
    return controlState == ON;
}

void PIDControls::setKp(float newKp, float &output){
    Kp = newKp;
    Ki = newKp / divThreshold(Ti);
    Kd = newKp * Td;
    integral = output - (Kp * prevError + derivative);
}

void PIDControls::setTi(float newTi, float &output){
    Ki = Kp / divThreshold(newTi);
    integral = output - (Kp * prevError + derivative);
}
void PIDControls::setTd(float newTd, float &output){
    Kd = Kp * newTd;
    integral = output - (Kp * prevError + derivative);
}