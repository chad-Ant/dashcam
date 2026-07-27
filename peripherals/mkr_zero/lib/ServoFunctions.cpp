#include "ServoFunctions.h"

ServoReturnStatus initializeServo(Servo &servo_x, const int pin, int min, int max){
    servo_x.attach(pin, min, max);
    return servo_x.attached() ? ServoReturnStatus::OK : ServoReturnStatus::NOK_INTERNAL_ERROR;
}

ServoReturnStatus readPosition(Servo &servo_x, int &position){
    if(!servo_x.attached()) return ServoReturnStatus::NOK_NOT_ATTACHED;
    position = servo_x.read();
    return ServoReturnStatus::OK;
}

ServoReturnStatus writePosition(Servo &servo_x, int position, int min, int max){
    if(!servo_x.attached()) return ServoReturnStatus::NOK_NOT_ATTACHED;
    position = position >= min ? (position <= max ? position : max) : min;
    servo_x.write(position);
    return ServoReturnStatus::OK;
}

ServoReturnStatus closeServo(Servo &servo_x){
    servo_x.detach();
    return !servo_x.attached() ? ServoReturnStatus::OK : ServoReturnStatus::NOK_INTERNAL_ERROR;
}