#ifndef LOWPASS_H
#define LOWPASS_H

#include <Arduino.h>


// Filter I found on youtube: https://www.youtube.com/watch?v=eM4VHtettGg
template <int order>
class LowPass {
private:
    float a[order];
    float b[order + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order + 1];
    float y[order + 1];

public:
    LowPass(float f0, float fs, bool adaptive);
    void setCoef();
    float filt(float xn);
};

template <int order>
LowPass<order>::LowPass(float f0, float fs, bool adaptive) {
    omega0 = 6.28318530718 * f0;
    dt = 1.0 / fs;
    adapt = adaptive;
    tn1 = -dt;
    for (int k = 0; k < order + 1; k++) {
        x[k] = 0;
        y[k] = 0;
    }
    setCoef();
}

template <int order>
void LowPass<order>::setCoef() {
    if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
    }

    float alpha = omega0 * dt;
    if (order == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
    }
    if (order == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
    }
}

template <int order>
float LowPass<order>::filt(float xn) {
    if (adapt) {
        setCoef();
    }
    y[0] = 0;
    x[0] = xn;
    for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
    }
    y[0] += b[order] * x[order];
    for (int k = order; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
    }
    return y[0];
}

#endif // LOWPASSFILTER_H
