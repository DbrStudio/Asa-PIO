#ifndef ENHANCEDMOTOR_H
#define ENHANCEDMOTOR_H

#include <LowPass.h>
#include <AutoPID.h>
#include <Adafruit_MotorShield.h>


class EnhancedMotor {
private:
    Adafruit_DCMotor* motor;
    AutoPID* pid;
    LowPass<2>* lpFilter;  // Example for first order filter
    float setpoint;
    int lastDirection;
    float output;
    float smoothedRPM;

public:
    EnhancedMotor(Motor* motorObject, AutoPID* pidObject, LowPassFilter<1>* filterObject, float setpointValue);
    void controlMotor(float speed, int direction);
    void updateSpeed(float speed);
    void updateDirection(int direction);

private:
    float calculateRPM();
};

#endif // ENHANCEDMOTOR_H