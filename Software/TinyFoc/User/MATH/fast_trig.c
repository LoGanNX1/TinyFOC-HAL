#include "fast_trig.h"

// ���ȣ�4096 ��
#define LUT_SIZE         4096
#define TWO_PI           6.28318530718f
#define INV_TWO_PI       0.15915494309f
#define STEP_SIZE        (TWO_PI / LUT_SIZE)
#define STEP_SIZE_INV    (1.0f / STEP_SIZE)

static float sin_lut[LUT_SIZE + 1];  // +1 for interpolation safety

// ��ʼ������ֵ��
void fast_trig_init(void) {
    for (int i = 0; i <= LUT_SIZE; ++i) {
        float angle = (float)i * STEP_SIZE;
        sin_lut[i] = sinf(angle);
    }
}

// Wrap �� [0, TWO_PI)
static inline float wrap_angle(float x) {
    while (x >= TWO_PI) x -= TWO_PI;
    while (x < 0.0f) x += TWO_PI;
    return x;
}

// ���� Hermite ��ֵ
static inline float cubic_interp(float y0, float y1, float y2, float y3, float mu) {
    float a0 = -0.5f*y0 + 1.5f*y1 - 1.5f*y2 + 0.5f*y3;
    float a1 = y0 - 2.5f*y1 + 2.0f*y2 - 0.5f*y3;
    float a2 = -0.5f*y0 + 0.5f*y2;
    float a3 = y1;
    return ((a0 * mu + a1) * mu + a2) * mu + a3;
}

// �����������ֵ
float fast_sin_f32(float x) {
    x = wrap_angle(x);
    float idx_f = x * STEP_SIZE_INV;
    int idx = (int)idx_f;
    float mu = idx_f - idx;

    // �߽紦��
    int idx0 = (idx - 1 + LUT_SIZE) & (LUT_SIZE - 1);
    int idx1 = idx;
    int idx2 = (idx + 1) & (LUT_SIZE - 1);
    int idx3 = (idx + 2) & (LUT_SIZE - 1);

    return cubic_interp(sin_lut[idx0], sin_lut[idx1], sin_lut[idx2], sin_lut[idx3], mu);
}

// ���� cos(x) = sin(x + pi/2) ������
float fast_cos_f32(float x) {
    return fast_sin_f32(x + 1.57079632679f);  // cos(x) = sin(x + ��/2)
}
