#ifndef FUNK_H
#define FUNK_H

#include <stdint.h>
#include <inttypes.h>

typedef struct{
	uint32_t time;
	float x;
	float y;
	float z;
} acc;

typedef struct {
	uint32_t time;
	float z;
} bar;

typedef struct {
	uint32_t time;
	float z;
	float vz;
} estimate;

typedef struct {
	float z;
	float vz;

	float P00;
	float P01;
	float P10;
	float P11;

	float az;

	uint32_t current_time;
} kalman;

double convert_to_rad(float deg);

void interpol(uint32_t time_ref, acc *manji, acc *veci, acc *rez);

acc mob_to_fix(acc akcel, acc rotacije);

void kalman_init(kalman *s, const bar *first_bar);

void kalman_predict(kalman *s, uint32_t next_time);

void kalman_correct(kalman *s, float mjer);

void kalman_filter(kalman *s, uint32_t t_next, float mjer, acc *akcel, uint8_t acc_or_bar, estimate *est);

#endif /*FUNK_H*/
