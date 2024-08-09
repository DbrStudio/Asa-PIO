#ifndef ENHANCEDMOTOR_H
#define ENHANCEDMOTOR_H

#include <LowPass.h>
#include <AutoPID.h>
#include <Adafruit_MotorShield.h>


class EnhancedMotor {
private:
    Adafruit_DCMotor* motor;
    AutoPID* pid;
    LowPass<2>* lpFilter; 
    float setpoint;
    int lastDirection;
    float output;
    float smoothedRPM;

public:
    EnhancedMotor(Adafruit_DCMotor* motorObject, AutoPID* pidObject, LowPass<2>* filterObject, float setpointValue);
    void controlMotor(float speed, int direction);
    void updateSpeed(float speed);
    void updateDirection(int direction);

private:
    float calculateRPM();
};

#endif // ENHANCEDMOTOR_H