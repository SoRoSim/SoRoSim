/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * variable_expmap_gTgTgd.c
 *
 * Code generation for function 'variable_expmap_gTgTgd'
 *
 */

/* Include files */
#include "variable_expmap_gTgTgd.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Function Definitions */
void variable_expmap_gTgTgd(const real_T Omega[6], const real_T Omegad[6],
                            real_T g[16], real_T Tg[36], real_T Tgd[36])
{
  static const int8_T iv1[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const int8_T iv[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  __m128d r;
  __m128d r1;
  real_T adjOmegad[36];
  real_T adjOmegad2[36];
  real_T adjOmegad3[36];
  real_T adjOmegad4[36];
  real_T adjOmegap2[36];
  real_T adjOmegap3[36];
  real_T adjOmegap4[36];
  real_T b_Tgd[36];
  real_T Omegahatp2[16];
  real_T Omegahatp3[16];
  real_T absxk;
  real_T scale;
  real_T t;
  real_T theta;
  real_T tp2;
  real_T tp3;
  int32_T Tgd_tmp;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
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
  Tgd[0] = 0.0;
  Tgd[6] = -Omega[2];
  Tgd[12] = Omega[1];
  Tgd[18] = 0.0;
  Tgd[24] = 0.0;
  Tgd[30] = 0.0;
  Tgd[1] = Omega[2];
  Tgd[7] = 0.0;
  Tgd[13] = -Omega[0];
  Tgd[19] = 0.0;
  Tgd[25] = 0.0;
  Tgd[31] = 0.0;
  Tgd[2] = -Omega[1];
  Tgd[8] = Omega[0];
  Tgd[14] = 0.0;
  Tgd[20] = 0.0;
  Tgd[26] = 0.0;
  Tgd[32] = 0.0;
  Tgd[3] = 0.0;
  Tgd[9] = -Omega[5];
  Tgd[15] = Omega[4];
  Tgd[21] = 0.0;
  Tgd[27] = -Omega[2];
  Tgd[33] = Omega[1];
  Tgd[4] = Omega[5];
  Tgd[10] = 0.0;
  Tgd[16] = -Omega[3];
  Tgd[22] = Omega[2];
  Tgd[28] = 0.0;
  Tgd[34] = -Omega[0];
  Tgd[5] = -Omega[4];
  Tgd[11] = Omega[3];
  Tgd[17] = 0.0;
  Tgd[23] = -Omega[1];
  Tgd[29] = Omega[0];
  Tgd[35] = 0.0;
  /*  optimized on 30.05.2022 */
  adjOmegad[0] = 0.0;
  adjOmegad[6] = -Omegad[2];
  adjOmegad[12] = Omegad[1];
  adjOmegad[18] = 0.0;
  adjOmegad[24] = 0.0;
  adjOmegad[30] = 0.0;
  adjOmegad[1] = Omegad[2];
  adjOmegad[7] = 0.0;
  adjOmegad[13] = -Omegad[0];
  adjOmegad[19] = 0.0;
  adjOmegad[25] = 0.0;
  adjOmegad[31] = 0.0;
  adjOmegad[2] = -Omegad[1];
  adjOmegad[8] = Omegad[0];
  adjOmegad[14] = 0.0;
  adjOmegad[20] = 0.0;
  adjOmegad[26] = 0.0;
  adjOmegad[32] = 0.0;
  adjOmegad[3] = 0.0;
  adjOmegad[9] = -Omegad[5];
  adjOmegad[15] = Omegad[4];
  adjOmegad[21] = 0.0;
  adjOmegad[27] = -Omegad[2];
  adjOmegad[33] = Omegad[1];
  adjOmegad[4] = Omegad[5];
  adjOmegad[10] = 0.0;
  adjOmegad[16] = -Omegad[3];
  adjOmegad[22] = Omegad[2];
  adjOmegad[28] = 0.0;
  adjOmegad[34] = -Omegad[0];
  adjOmegad[5] = -Omegad[4];
  adjOmegad[11] = Omegad[3];
  adjOmegad[17] = 0.0;
  adjOmegad[23] = -Omegad[1];
  adjOmegad[29] = Omegad[0];
  adjOmegad[35] = 0.0;
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      Tgd_tmp = i1 << 2;
      Omegahatp2[i + Tgd_tmp] =
          ((g[i] * g[Tgd_tmp] + g[i + 4] * g[Tgd_tmp + 1]) +
           g[i + 8] * g[Tgd_tmp + 2]) +
          g[i + 12] * g[Tgd_tmp + 3];
    }
    absxk = Omegahatp2[i];
    t = Omegahatp2[i + 4];
    tp2 = Omegahatp2[i + 8];
    tp3 = Omegahatp2[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      Tgd_tmp = i1 << 2;
      Omegahatp3[i + Tgd_tmp] =
          ((absxk * g[Tgd_tmp] + t * g[Tgd_tmp + 1]) + tp2 * g[Tgd_tmp + 2]) +
          tp3 * g[Tgd_tmp + 3];
    }
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      absxk = 0.0;
      for (Tgd_tmp = 0; Tgd_tmp < 6; Tgd_tmp++) {
        absxk += Tgd[i + 6 * Tgd_tmp] * Tgd[Tgd_tmp + 6 * i1];
      }
      adjOmegap2[i + 6 * i1] = absxk;
    }
    for (i1 = 0; i1 < 6; i1++) {
      absxk = 0.0;
      for (Tgd_tmp = 0; Tgd_tmp < 6; Tgd_tmp++) {
        absxk += adjOmegap2[i + 6 * Tgd_tmp] * Tgd[Tgd_tmp + 6 * i1];
      }
      adjOmegap3[i + 6 * i1] = absxk;
    }
    for (i1 = 0; i1 < 6; i1++) {
      absxk = 0.0;
      t = 0.0;
      tp2 = 0.0;
      for (Tgd_tmp = 0; Tgd_tmp < 6; Tgd_tmp++) {
        i2 = Tgd_tmp + 6 * i1;
        tp3 = Tgd[i2];
        i3 = i + 6 * Tgd_tmp;
        absxk += adjOmegap3[i3] * tp3;
        t += adjOmegad[i3] * tp3;
        tp2 += Tgd[i3] * adjOmegad[i2];
      }
      Tgd_tmp = i + 6 * i1;
      b_Tgd[Tgd_tmp] = tp2;
      adjOmegad2[Tgd_tmp] = t;
      adjOmegap4[Tgd_tmp] = absxk;
    }
  }
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&adjOmegad2[i]);
    r1 = _mm_loadu_pd(&b_Tgd[i]);
    _mm_storeu_pd(&adjOmegad2[i], _mm_add_pd(r, r1));
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      absxk = 0.0;
      t = 0.0;
      for (Tgd_tmp = 0; Tgd_tmp < 6; Tgd_tmp++) {
        i2 = i + 6 * Tgd_tmp;
        i3 = Tgd_tmp + 6 * i1;
        absxk += adjOmegad2[i2] * Tgd[i3];
        t += adjOmegap2[i2] * adjOmegad[i3];
      }
      Tgd_tmp = i + 6 * i1;
      b_Tgd[Tgd_tmp] = t;
      adjOmegad3[Tgd_tmp] = absxk;
    }
  }
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&adjOmegad3[i]);
    r1 = _mm_loadu_pd(&b_Tgd[i]);
    _mm_storeu_pd(&adjOmegad3[i], _mm_add_pd(r, r1));
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      absxk = 0.0;
      t = 0.0;
      for (Tgd_tmp = 0; Tgd_tmp < 6; Tgd_tmp++) {
        i2 = i + 6 * Tgd_tmp;
        i3 = Tgd_tmp + 6 * i1;
        absxk += adjOmegad3[i2] * Tgd[i3];
        t += adjOmegap3[i2] * adjOmegad[i3];
      }
      Tgd_tmp = i + 6 * i1;
      b_Tgd[Tgd_tmp] = t;
      adjOmegad4[Tgd_tmp] = absxk;
    }
  }
  for (i = 0; i <= 34; i += 2) {
    r = _mm_loadu_pd(&adjOmegad4[i]);
    r1 = _mm_loadu_pd(&b_Tgd[i]);
    _mm_storeu_pd(&adjOmegad4[i], _mm_add_pd(r, r1));
  }
  if (theta <= 0.01) {
    for (i = 0; i < 16; i++) {
      g[i] =
          ((g[i] + (real_T)iv[i]) + Omegahatp2[i] / 2.0) + Omegahatp3[i] / 6.0;
    }
    for (i = 0; i < 36; i++) {
      Tg[i] = (((0.5 * Tgd[i] + (real_T)iv1[i]) +
                0.16666666666666666 * adjOmegap2[i]) +
               0.041666666666666664 * adjOmegap3[i]) +
              0.0083333333333333332 * adjOmegap4[i];
      Tgd[i] = ((0.5 * adjOmegad[i] + 0.16666666666666666 * adjOmegad2[i]) +
                0.041666666666666664 * adjOmegad3[i]) +
               0.0083333333333333332 * adjOmegad4[i];
    }
  } else {
    real_T a;
    real_T b_a;
    real_T sintheta;
    real_T t2;
    real_T t5;
    real_T t6;
    real_T t7;
    real_T t8;
    real_T tp4;
    real_T tp5;
    t = ((Omegad[0] * Omega[0] + Omegad[1] * Omega[1]) + Omegad[2] * Omega[2]) /
        theta;
    tp2 = theta * theta;
    tp3 = tp2 * theta;
    tp4 = tp3 * theta;
    tp5 = tp4 * theta;
    sintheta = muDoubleScalarSin(theta);
    scale = muDoubleScalarCos(theta);
    absxk = theta * sintheta;
    t2 = theta * scale;
    t5 = ((4.0 - 4.0 * scale) - absxk) / (2.0 * tp2);
    t6 = ((4.0 * theta - 5.0 * sintheta) + t2) / (2.0 * tp3);
    t7 = ((2.0 - 2.0 * scale) - absxk) / (2.0 * tp4);
    t8 = ((2.0 * theta - 3.0 * sintheta) + t2) / (2.0 * tp5);
    a = (1.0 - scale) / tp2;
    b_a = (theta - sintheta) / tp3;
    for (i = 0; i < 16; i++) {
      g[i] = ((g[i] + (real_T)iv[i]) + a * Omegahatp2[i]) + b_a * Omegahatp3[i];
    }
    absxk = t * (((8.0 - tp2) * scale - 8.0) + 5.0 * absxk);
    a = absxk / (2.0 * tp3);
    scale = t * ((-8.0 * theta + (15.0 - tp2) * sintheta) - 7.0 * t2);
    b_a = scale / (2.0 * tp4);
    sintheta = absxk / (2.0 * tp5);
    scale /= 2.0 * (tp5 * theta);
    for (i = 0; i < 36; i++) {
      absxk = Tgd[i];
      t = adjOmegap2[i];
      tp2 = adjOmegap3[i];
      tp3 = adjOmegap4[i];
      Tg[i] = (((t5 * absxk + (real_T)iv1[i]) + t6 * t) + t7 * tp2) + t8 * tp3;
      absxk = ((((((a * absxk + t5 * adjOmegad[i]) + b_a * t) +
                  t6 * adjOmegad2[i]) +
                 sintheta * tp2) +
                t7 * adjOmegad3[i]) +
               scale * tp3) +
              t8 * adjOmegad4[i];
      Tgd[i] = absxk;
    }
  }
}

/* End of code generation (variable_expmap_gTgTgd.c) */
