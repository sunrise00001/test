#include "gravity_model.h"
#include <math.h>

/* 拟合系数 */
static const float coeff[2][7] = {
    {-1.462834116e+00f, 1.154834197e+00f, 1.102424651e+00f, -6.604008482e-01f, -3.490764621e-01f, 4.448315258e-01f, 1.015517003e+00f},
    {9.551727900e-01f, -9.150099305e-01f, -9.088935712e-01f, 6.283025466e-01f, 1.219419753e-01f, -6.367496295e-01f, 2.276800390e-01f}
};

void gravity_model_eval(const float q_rad[2], float torque_nm[2])
{
    float f[7];
    unsigned int i, j;
    f[0] = 1.0f;
    f[1] = sinf(q_rad[0]);
    f[2] = cosf(q_rad[0]);
    f[3] = sinf(q_rad[0] + q_rad[1]);
    f[4] = cosf(q_rad[0] + q_rad[1]);
    f[5] = sinf(q_rad[1]);
    f[6] = cosf(q_rad[1]);
    for (i = 0U; i < 2U; ++i) {
        float value = 0.0f;
        for (j = 0U; j < 7U; ++j) value += coeff[i][j] * f[j];
        torque_nm[i] = value;
    }
}
