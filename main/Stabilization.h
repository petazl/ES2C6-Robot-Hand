#pragma once

#include <math.h>

struct Vector3 {
    float x, y, z;
};

struct Rotation {
    float w, x, y, z;
};

constexpr float DEG_TO_RAD_F = 0.01745329252f;
constexpr float RAD_TO_DEG_F = 57.29577951f;

inline float dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vector3 cross(Vector3 a, Vector3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline Vector3 scale(Vector3 v, float n) {
    return {v.x * n, v.y * n, v.z * n};
}

inline Rotation multiply(Rotation a, Rotation b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

inline Rotation inverse(Rotation q) {
    return {q.w, -q.x, -q.y, -q.z};
}

inline Rotation normalized(Rotation q) {
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Vector3 rotate(Rotation q, Vector3 v) {
    Rotation r = multiply(multiply(q, {0, v.x, v.y, v.z}), inverse(q));
    return {r.x, r.y, r.z};
}

inline Rotation rotationAround(Vector3 axis, float degrees) {
    const float half = degrees * DEG_TO_RAD_F * 0.5f;
    const float s = sinf(half);
    return {cosf(half), axis.x * s, axis.y * s, axis.z * s};
}

inline Vector3 angleError(Rotation target, Rotation current) {
    const Rotation error = multiply(target, inverse(current));
    const float sine = sqrtf(error.x * error.x + error.y * error.y + error.z * error.z);
    const float sign = error.w < 0 ? -1.0f : 1.0f;
    const float factor = sine > 0.0001f ? 2.0f * atan2f(sine, fabsf(error.w)) / sine : 2.0f;
    return scale({error.x, error.y, error.z}, sign * factor * RAD_TO_DEG_F);
}

struct Attitude {
    Rotation estimate{1, 0, 0, 0};
    Vector3 rate{};
    bool started = false;

    bool update(Vector3 acceleration, Vector3 gyro, float seconds) {
        const Vector3 up{0, 0, 1};
        const float g = sqrtf(dot(acceleration, acceleration));
        const bool gravityValid = g > 0.85f && g < 1.15f;
        if (!started && !gravityValid) {
            return false;
        }

        if (!started) {
            const Vector3 a = scale(acceleration, 1.0f / g);
            // Initialize tilt from gravity. Heading starts at an arbitrary zero.
            estimate = a.z < -0.9999f ? Rotation{0, 1, 0, 0}
                                      : normalized({1.0f + a.z, a.y, -a.x, 0});
            started = true;
        } else {
            Vector3 omega = scale(gyro, DEG_TO_RAD_F);
            if (gravityValid) {
                const Vector3 measuredUp = scale(acceleration, 1.0f / g);
                const Vector3 predictedUp = rotate(inverse(estimate), up);
                const Vector3 correction = scale(cross(measuredUp, predictedUp), 2.0f);
                omega = {omega.x + correction.x, omega.y + correction.y, omega.z + correction.z};
            }
            const Rotation step = multiply(estimate, {0, omega.x, omega.y, omega.z});
            const float halfTime = 0.5f * seconds;
            estimate = normalized({
                estimate.w + step.w * halfTime, estimate.x + step.x * halfTime,
                estimate.y + step.y * halfTime, estimate.z + step.z * halfTime
            });
        }
        rate = rotate(estimate, gyro);
        return true;
    }
};

// Each value is the angular correction about that joint's axis.
struct JointCorrection {
    float heading, pitch, roll;
};

inline JointCorrection jointCorrection(Vector3 change, Vector3 forward, Vector3 pitchAxis) {
    // Base yaw is approximately vertical with the base upright. Near vertical claw
    // pitch, yaw and roll become the same axis, so let roll handle that direction.
    const float horizontalSquared = forward.x * forward.x + forward.y * forward.y;
    const float alongForward = dot(change, forward);
    const float pitch = dot(change, pitchAxis);
    if (horizontalSquared < 0.04f) {
        return {0, pitch, alongForward};
    }
    const float roll = (alongForward - forward.z * change.z) / horizontalSquared;
    return {change.z - forward.z * roll, pitch, roll};
}

struct Pid {
    float integral = 0;

    float update(float error, float measuredRate, float seconds, float maxRate) {
        const float candidate = fmaxf(-25.0f, fminf(25.0f, integral + error * seconds));
        const float output = 3.0f * error + 0.35f * candidate - 0.12f * measuredRate;
        const float limited = fmaxf(-maxRate, fminf(maxRate, output));
        if (output == limited || output * error < 0) {
            integral = candidate;
        }
        return limited;
    }
};
