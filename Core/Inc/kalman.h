/*
 * kelman.h
 *
 *  Created on: Jan 21, 2021
 *      Author: d2460
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct kfilter kfilter;

struct kfilter{
	float Q_angle; // Process noise variance for the accelerometer
	float Q_bias; // Process noise variance for the gyro bias
	float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};


    kfilter* new_Kalman();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(kfilter* self,float newAngle, float newRate, float dt);

    void setAngle(kfilter* self,float angle); // Used to set angle, this should be set as the starting angle
    float getRate(kfilter* self); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(kfilter* self,float Q_angle);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp.
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(kfilter* self,float Q_bias);
    void setRmeasure(kfilter* self,float R_measure);

    float getQangle(kfilter* self);
    float getQbias(kfilter* self);
    float getRmeasure(kfilter* self);




#endif /* INC_KALMAN_H_ */
