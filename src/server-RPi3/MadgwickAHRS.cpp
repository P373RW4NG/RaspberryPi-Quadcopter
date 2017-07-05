//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	200.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float bbeta = betaDef;								// 2 * proportional gain (Kp)
volatile float qu0 = 1.0f, qu1 = 0.0f, qu2 = 0.0f, qu3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-qu1 * gx - qu2 * gy - qu3 * gz);
    qDot2 = 0.5f * (qu0 * gx + qu2 * gz - qu3 * gy);
    qDot3 = 0.5f * (qu0 * gy - qu1 * gz + qu3 * gx);
    qDot4 = 0.5f * (qu0 * gz + qu1 * gy - qu2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * qu0 * mx;
        _2q0my = 2.0f * qu0 * my;
        _2q0mz = 2.0f * qu0 * mz;
        _2q1mx = 2.0f * qu1 * mx;
        _2q0 = 2.0f * qu0;
        _2q1 = 2.0f * qu1;
        _2q2 = 2.0f * qu2;
        _2q3 = 2.0f * qu3;
        _2q0q2 = 2.0f * qu0 * qu2;
        _2q2q3 = 2.0f * qu2 * qu3;
        q0q0 = qu0 * qu0;
        q0q1 = qu0 * qu1;
        q0q2 = qu0 * qu2;
        q0q3 = qu0 * qu3;
        q1q1 = qu1 * qu1;
        q1q2 = qu1 * qu2;
        q1q3 = qu1 * qu3;
        q2q2 = qu2 * qu2;
        q2q3 = qu2 * qu3;
        q3q3 = qu3 * qu3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * qu3 + _2q0mz * qu2 + mx * q1q1 + _2q1 * my * qu2 + _2q1 * mz * qu3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * qu3 + my * q0q0 - _2q0mz * qu1 + _2q1mx * qu2 - my * q1q1 + my * q2q2 + _2q2 * mz * qu3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * qu2 + _2q0my * qu1 + mz * q0q0 + _2q1mx * qu3 - mz * q1q1 + _2q2 * my * qu3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * qu2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * qu3 + _2bz * qu1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * qu2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * qu1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * qu3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * qu2 + _2bz * qu0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * qu3 - _4bz * qu1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * qu2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * qu2 - _2bz * qu0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * qu1 + _2bz * qu3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * qu0 - _4bz * qu2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * qu3 + _2bz * qu1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * qu0 + _2bz * qu2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * qu1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= bbeta * s0;
        qDot2 -= bbeta * s1;
        qDot3 -= bbeta * s2;
        qDot4 -= bbeta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    qu0 += qDot1 * (1.0f / sampleFreq);
    qu1 += qDot2 * (1.0f / sampleFreq);
    qu2 += qDot3 * (1.0f / sampleFreq);
    qu3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(qu0 * qu0 + qu1 * qu1 + qu2 * qu2 + qu3 * qu3);
    qu0 *= recipNorm;
    qu1 *= recipNorm;
    qu2 *= recipNorm;
    qu3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-qu1 * gx - qu2 * gy - qu3 * gz);
    qDot2 = 0.5f * (qu0 * gx + qu2 * gz - qu3 * gy);
    qDot3 = 0.5f * (qu0 * gy - qu1 * gz + qu3 * gx);
    qDot4 = 0.5f * (qu0 * gz + qu1 * gy - qu2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * qu0;
        _2q1 = 2.0f * qu1;
        _2q2 = 2.0f * qu2;
        _2q3 = 2.0f * qu3;
        _4q0 = 4.0f * qu0;
        _4q1 = 4.0f * qu1;
        _4q2 = 4.0f * qu2;
        _8q1 = 8.0f * qu1;
        _8q2 = 8.0f * qu2;
        q0q0 = qu0 * qu0;
        q1q1 = qu1 * qu1;
        q2q2 = qu2 * qu2;
        q3q3 = qu3 * qu3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * qu1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * qu2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * qu3 - _2q1 * ax + 4.0f * q2q2 * qu3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= bbeta * s0;
        qDot2 -= bbeta * s1;
        qDot3 -= bbeta * s2;
        qDot4 -= bbeta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    qu0 += qDot1 * (1.0f / sampleFreq);
    qu1 += qDot2 * (1.0f / sampleFreq);
    qu2 += qDot3 * (1.0f / sampleFreq);
    qu3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(qu0 * qu0 + qu1 * qu1 + qu2 * qu2 + qu3 * qu3);
    qu0 *= recipNorm;
    qu1 *= recipNorm;
    qu2 *= recipNorm;
    qu3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

