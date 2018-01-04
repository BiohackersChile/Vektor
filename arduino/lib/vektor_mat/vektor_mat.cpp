#include <math.h>
//////////////////////////////////////////////////////////////////////
////////////Mat functions
//////////////////////////////////////////////////////////////////////
/** creates a new empty vector */
float * create_vec() {
  float vec[3] = {0.0f ,0.0f ,0.0f};
  return vec;
}

/** vi = vi/mod(vi) */
void normalize(float vi[3]) {
  float l =  sqrt( double(vi[0]*vi[0] + vi[1]*vi[1] + vi[2]*vi[2]) );
  vi[0] /= l;
  vi[1] /= l;
  vi[2] /= l;
}

/** makes  mat = (vx, vy, vz)
  wich represents a tranformation from coordinates relative to vx,y,z to
  the ones on wich they are defined
*/
void create_transform(float mat[9],float vx[3], float vy[3], float vz[3]) {
  mat[0] = vx[0];
  mat[1] = vx[1];
  mat[2] = vx[2];

  mat[3] = vy[0];
  mat[4] = vy[1];
  mat[5] = vy[2];

  mat[6] = vz[0];
  mat[7] = vz[1];
  mat[8] = vz[2];
}

/** makes vo = mat*vi */
void mat_mult_vec(float mat[9], float vi[3], float vo[3]) {
  vo[0] = vi[0]*mat[0] + vi[1]*mat[3] + vi[2]*mat[6];
  vo[1] = vi[0]*mat[1] + vi[1]*mat[4] + vi[2]*mat[7];
  vo[2] = vi[0]*mat[2] + vi[1]*mat[5] + vi[2]*mat[8];
}
