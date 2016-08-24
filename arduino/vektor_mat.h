#ifndef VEKTOR_MAT
#define VEKTOR_MAT

#include <Wire.h>

float * create_vec();
void normalize(float vi[3]);
void create_transform(float mat[9],float vx[3], float vy[3], float vz[3]);
void mat_mult_vec(float mat[9], float vi[3], float vo[3]);

#endif //VEKTOR_MAT
