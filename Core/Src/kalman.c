/*
 * kelman.c
 *
 *  Created on: Jan 21, 2021
 *      Author: d2460
 */

#include "kalman.h"
#include <stdlib.h>

kfilter* new_Kalman() {
	kfilter* kfpt = (kfilter*)malloc(sizeof(kfilter));
    /* We will set the variables like so, these can also be tuned by the user */
    kfpt->Q_angle = 0.001f;
    kfpt->Q_bias = 0.003f;
    kfpt->R_measure = 0.03f;

    kfpt->angle = 0.0f; // Reset the angle
    kfpt->bias = 0.0f; // Reset bias

    kfpt->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    kfpt->P[0][1] = 0.0f;
    kfpt->P[1][0] = 0.0f;
    kfpt->P[1][1] = 0.0f;

    return kfpt;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(kfilter* self,float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    self->rate = newRate - self->bias;
    self->angle += dt * self->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    self->P[0][0] += dt * (dt*self->P[1][1] - self->P[0][1] - self->P[1][0] + self->Q_angle);
    self->P[0][1] -= dt * self->P[1][1];
    self->P[1][0] -= dt * self->P[1][1];
    self->P[1][1] += self->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = self->P[0][0] + self->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = self->P[0][0] / S;
    K[1] = self->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - self->angle; // Angle difference
    /* Step 6 */
    self->angle += K[0] * y;
    self->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = self->P[0][0];
    float P01_temp = self->P[0][1];

    self->P[0][0] -= K[0] * P00_temp;
    self->P[0][1] -= K[0] * P01_temp;
    self->P[1][0] -= K[1] * P00_temp;
    self->P[1][1] -= K[1] * P01_temp;

    return self->angle;
};

void setAngle(kfilter* self,float angle) { self->angle = angle; }; // Used to set angle, this should be set as the starting angle
float getRate(kfilter* self) { return self->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(kfilter* self,float Q_angle) { self->Q_angle = Q_angle; };
void setQbias(kfilter* self,float Q_bias) { self->Q_bias = Q_bias; };
void setRmeasure(kfilter* self,float R_measure) { self->R_measure = R_measure; };

float getQangle(kfilter* self) { return self->Q_angle; };
float getQbias(kfilter* self) { return self->Q_bias; };
float getRmeasure(kfilter* self) { return self->R_measure; };
