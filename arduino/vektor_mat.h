#ifndef VEKTOR_MAT
#define VEKTOR_MAT

#include <Wire.h>

/** creates a new empty vector */
float * create_vec();

/** vi = vi/mod(vi) */
void normalize(float vi[3]);

/** makes  mat = (vx, vy, vz)
  wich represents a tranformation from coordinates relative to vx,y,z to
  the ones on wich they are defined
*/
void create_transform(float mat[9],float vx[3], float vy[3], float vz[3]);

/** makes vo = mat*vi */
void mat_mult_vec(float mat[9], float vi[3], float vo[3]);

#endif //VEKTOR_MAT
