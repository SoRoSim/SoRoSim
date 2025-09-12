/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * variable_expmap_gTg.c
 *
 * Code generation for function 'variable_expmap_gTg'
 *
 */

/* Include files */
#include "variable_expmap_gTg.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void variable_expmap_gTg(const real_T Omega[6], real_T g[16], real_T Tg[36])
{
  static const int8_T iv1[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  real_T adjOmegap2[36];
  real_T adjOmegap3[36];
  real_T adjOmegap4[36];
  real_T Omegahatp2[16];
  real_T Omegahatp3[16];
  real_T absxk;
  real_T scale;
  real_T t;
  real_T t1;
  real_T theta;
  int32_T i;
  int32_T i1;
  int32_T i2;
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
  /*  optimized on 30.05.2022 */
  Tg[0] = 0.0;
  Tg[6] = -Omega[2];
  Tg[12] = Omega[1];
  Tg[18] = 0.0;
  Tg[24] = 0.0;
  Tg[30] = 0.0;
  Tg[1] = Omega[2];
  Tg[7] = 0.0;
  Tg[13] = -Omega[0];
  Tg[19] = 0.0;
  Tg[25] = 0.0;
  Tg[31] = 0.0;
  Tg[2] = -Omega[1];
  Tg[8] = Omega[0];
  Tg[14] = 0.0;
  Tg[20] = 0.0;
  Tg[26] = 0.0;
  Tg[32] = 0.0;
  Tg[3] = 0.0;
  Tg[9] = -Omega[5];
  Tg[15] = Omega[4];
  Tg[21] = 0.0;
  Tg[27] = -Omega[2];
  Tg[33] = Omega[1];
  Tg[4] = Omega[5];
  Tg[10] = 0.0;
  Tg[16] = -Omega[3];
  Tg[22] = Omega[2];
  Tg[28] = 0.0;
  Tg[34] = -Omega[0];
  Tg[5] = -Omega[4];
  Tg[11] = Omega[3];
  Tg[17] = 0.0;
  Tg[23] = -Omega[1];
  Tg[29] = Omega[0];
  Tg[35] = 0.0;
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      Omegahatp2[i + i2] =
          ((g[i] * g[i2] + g[i + 4] * g[i2 + 1]) + g[i + 8] * g[i2 + 2]) +
          g[i + 12] * g[i2 + 3];
    }
    scale = Omegahatp2[i];
    absxk = Omegahatp2[i + 4];
    t = Omegahatp2[i + 8];
    t1 = Omegahatp2[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      Omegahatp3[i + i2] =
          ((scale * g[i2] + absxk * g[i2 + 1]) + t * g[i2 + 2]) +
          t1 * g[i2 + 3];
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      scale = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        scale += Tg[i + 6 * i2] * Tg[i2 + 6 * i1];
      }
      adjOmegap2[i + 6 * i1] = scale;
    }
    for (i1 = 0; i1 < 6; i1++) {
      scale = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        scale += adjOmegap2[i + 6 * i2] * Tg[i2 + 6 * i1];
      }
      adjOmegap3[i + 6 * i1] = scale;
    }
    for (i1 = 0; i1 < 6; i1++) {
      scale = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        scale += adjOmegap3[i + 6 * i2] * Tg[i2 + 6 * i1];
      }
      adjOmegap4[i + 6 * i1] = scale;
    }
  }
  if (theta <= 0.01) {
    for (i = 0; i < 16; i++) {
      g[i] =
          ((g[i] + (real_T)iv[i]) + Omegahatp2[i] / 2.0) + Omegahatp3[i] / 6.0;
    }
    for (i = 0; i < 36; i++) {
      Tg[i] = (((0.5 * Tg[i] + (real_T)iv1[i]) +
                0.16666666666666666 * adjOmegap2[i]) +
               0.041666666666666664 * adjOmegap3[i]) +
              0.0083333333333333332 * adjOmegap4[i];
    }
  } else {
    real_T a;
    real_T b_a;
    real_T sintheta;
    real_T t2;
    real_T tp4;
    scale = theta * theta;
    absxk = scale * theta;
    tp4 = absxk * theta;
    sintheta = muDoubleScalarSin(theta);
    t = muDoubleScalarCos(theta);
    t1 = theta * sintheta;
    t2 = theta * t;
    a = (1.0 - t) / scale;
    b_a = (theta - sintheta) / absxk;
    for (i = 0; i < 16; i++) {
      g[i] = ((g[i] + (real_T)iv[i]) + a * Omegahatp2[i]) + b_a * Omegahatp3[i];
    }
    a = ((4.0 - 4.0 * t) - t1) / (2.0 * scale);
    b_a = ((4.0 * theta - 5.0 * sintheta) + t2) / (2.0 * absxk);
    absxk = ((2.0 - 2.0 * t) - t1) / (2.0 * tp4);
    scale = ((2.0 * theta - 3.0 * sintheta) + t2) / (2.0 * (tp4 * theta));
    for (i = 0; i < 36; i++) {
      Tg[i] = (((a * Tg[i] + (real_T)iv1[i]) + b_a * adjOmegap2[i]) +
               absxk * adjOmegap3[i]) +
              scale * adjOmegap4[i];
    }
  }
}

/* End of code generation (variable_expmap_gTg.c) */
