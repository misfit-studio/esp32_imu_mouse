#include <Arduino.h>
#include <Adafruit_BNO08x.h>

typedef sh2_RotationVector_t sQuatRotationVector_t;
typedef struct sEulerRotationVector_t
{
    float yaw;
    float pitch;
    float roll;
} eulerRotationVector_t;

// Helper function to square a number
#ifndef sq
#define sq(x) ((x) * (x))
#endif

// Constant for converting radians to degrees
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

// Function to convert quaternion to Euler angles
sEulerRotationVector_t quat_to_euler(const sQuatRotationVector_t *q, bool degrees)
{
    sEulerRotationVector_t ypr;
    float sqr = sq(q->real);
    float sqi = sq(q->i);
    float sqj = sq(q->j);
    float sqk = sq(q->k);

    ypr.yaw = atan2(2.0f * (q->i * q->j + q->k * q->real), (sqi - sqj - sqk + sqr));
    ypr.pitch = asin(-2.0f * (q->i * q->k - q->j * q->real) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0f * (q->j * q->k + q->i * q->real), (-sqi - sqj + sqk + sqr));

    if (degrees)
    {
        ypr.yaw *= RAD_TO_DEG;
        ypr.pitch *= RAD_TO_DEG;
        ypr.roll *= RAD_TO_DEG;
    }

    return ypr;
}

// Function to calculate the inverse of a quaternion
sQuatRotationVector_t quat_inverse(const sQuatRotationVector_t q)
{
    sQuatRotationVector_t inverse = {-q.i, -q.j, -q.k, q.real}; // Assuming the quaternion is normalized
    return inverse;
}

// Function to multiply two quaternions
sQuatRotationVector_t quat_multiply(const sQuatRotationVector_t a, const sQuatRotationVector_t b)
{
    sQuatRotationVector_t result;
    result.real = a.real * b.real - a.i * b.i - a.j * b.j - a.k * b.k;
    result.i = a.real * b.i + a.i * b.real + a.j * b.k - a.k * b.j;
    result.j = a.real * b.j - a.i * b.k + a.j * b.real + a.k * b.i;
    result.k = a.real * b.k + a.i * b.j - a.j * b.i + a.k * b.real;
    return result;
}

sQuatRotationVector_t quat_normalize(const sQuatRotationVector_t q)
{
    float norm = sqrt(sq(q.real) + sq(q.i) + sq(q.j) + sq(q.k));
    sQuatRotationVector_t normalized_q = {q.i / norm, q.j / norm, q.k / norm, q.real / norm};
    return normalized_q;
}

// Function to calculate the conjugate of a quaternion
sQuatRotationVector_t quat_conjugate(const sQuatRotationVector_t q)
{
    sQuatRotationVector_t conjugate;
    conjugate.i = -q.i;
    conjugate.j = -q.j;
    conjugate.k = -q.k;
    conjugate.real = q.real;
    return conjugate;
}

// Function to calculate the rotational offset between two quaternions
sQuatRotationVector_t quat_delta(const sQuatRotationVector_t current, const sQuatRotationVector_t zero, const sQuatRotationVector_t reference = {0, 0, 0, 1})
{
    sQuatRotationVector_t current_normalized = quat_normalize(current);

    // Conjugate of the normalized reference quaternion represents the inverse rotation
    sQuatRotationVector_t reference_conjugate = quat_conjugate(quat_normalize(zero));

    // The resulting quaternion represents the rotation from the reference orientation to the current orientation
    return quat_multiply(reference_conjugate, current_normalized);
}
