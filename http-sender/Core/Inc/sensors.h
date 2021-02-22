/*
 * sensors.h
 *
 *  Created on: 29 gen 2021
 *      Author: UTPM9
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
#include "main.h"

void init_accelerometer();

void getAxisAccelerometer(int16_t *accx, int16_t *accy, int16_t *accz);

int16_t getMicrophonedb(int analogValue);

// a trick to make the logbase2 of a number
int getfirstValidBit(int absoluteValue);

void init_magnetometer();

void getAxisMagnetometer(int16_t *magx, int16_t *magy, int16_t *magz);

void init_gyroscope();

void getAxisGyro(int16_t *gyrox, int16_t *gyroy, int16_t *gyroz);

int8_t getMicrophone();

void initHTS221();

void getTemperature(float *temperature);

void getHumidity(float *humidity);

void initLPS22hh();

void getPressure(float *pressure);

void startToF();

void getDistance(int *distance);

#endif /* INC_SENSORS_H_ */
