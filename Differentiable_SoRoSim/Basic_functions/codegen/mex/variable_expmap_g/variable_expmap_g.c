/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * variable_expmap_g.c
 *
 * Code generation for function 'variable_expmap_g'
 *
 */

/* Include files */
#include "variable_expmap_g.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void variable_expmap_g(const real_T Omega[6], real_T g[16])
{
  static const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  real_T Omegahatp2[16];
  real_T Omegahatp3[16];
  real_T absxk;
  real_T scale;
  real_T t;
  real_T theta;
  int32_T i;
  int32_T i1;
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(Omega[0]);
  if (absxk > 3.3121686421112381E-170) {
    theta = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    theta = t * t;
  }
  absxk = muDoubleScalarAbs(Omega[1]);
  if (absxk > scale) {
    t = scale / absxk;
    theta = theta * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    theta += t * t;
  }
  absxk = muDoubleScalarAbs(Omega[2]);
  if (absxk > scale) {
    t = scale / absxk;
    theta = theta * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    theta += t * t;
  }
  theta = scale * muDoubleScalarSqrt(theta);
  /*  optimized on 30.05.2022 */
  g[0] = 0.0;
  g[4] = -Omega[2];
  g[8] = Omega[1];
  g[12] = Omega[3];
  g[1] = Omega[2];
  g[5] = 0.0;
  g[9] = -Omega[0];
  g[13] = Omega[4];
  g[2] = -Omega[1];
  g[6] = Omega[0];
  g[10] = 0.0;
  g[14] = Omega[5];
  g[3] = 0.0;
  g[7] = 0.0;
  g[11] = 0.0;
  g[15] = 0.0;
  for (i = 0; i < 4; i++) {
    real_T d;
    int32_T i2;
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      Omegahatp2[i + i2] =
          ((g[i] * g[i2] + g[i + 4] * g[i2 + 1]) + g[i + 8] * g[i2 + 2]) +
          g[i + 12] * g[i2 + 3];
    }
    scale = Omegahatp2[i];
    absxk = Omegahatp2[i + 4];
    t = Omegahatp2[i + 8];
    d = Omegahatp2[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      Omegahatp3[i + i2] =
          ((scale * g[i2] + absxk * g[i2 + 1]) + t * g[i2 + 2]) + d * g[i2 + 3];
    }
  }
  if (theta <= 0.01) {
    for (i = 0; i < 16; i++) {
      g[i] =
          ((g[i] + (real_T)iv[i]) + Omegahatp2[i] / 2.0) + Omegahatp3[i] / 6.0;
    }
  } else {
    scale = theta * theta;
    absxk = (1.0 - muDoubleScalarCos(theta)) / scale;
    scale = (theta - muDoubleScalarSin(theta)) / (scale * theta);
    for (i = 0; i < 16; i++) {
      g[i] = ((g[i] + (real_T)iv[i]) + absxk * Omegahatp2[i]) +
             scale * Omegahatp3[i];
    }
  }
}

/* End of code generation (variable_expmap_g.c) */
