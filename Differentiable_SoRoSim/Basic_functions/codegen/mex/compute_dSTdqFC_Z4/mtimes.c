/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mtimes.c
 *
 * Code generation for function 'mtimes'
 *
 */

/* Include files */
#include "mtimes.h"
#include "compute_dSTdqFC_Z4_emxutil.h"
#include "compute_dSTdqFC_Z4_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo j_emlrtRSI = {
    142,      /* lineNo */
    "mtimes", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pathName */
};

static emlrtRSInfo k_emlrtRSI = {
    178,           /* lineNo */
    "mtimes_blas", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pathName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    218,      /* lineNo */
    20,       /* colNo */
    "mtimes", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pName */
};

static emlrtRTEInfo f_emlrtRTEI = {
    140,      /* lineNo */
    5,        /* colNo */
    "mtimes", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\mtimes.m" /* pName */
};

/* Function Definitions */
void b_mtimes(const emlrtStack *sp, const real_T A_data[],
              const int32_T A_size[2], const real_T B_data[],
              const int32_T B_size[2], emxArray_real_T *C)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T alpha1;
  real_T beta1;
  real_T *C_data;
  int32_T i;
  char_T TRANSA1;
  char_T TRANSB1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if ((A_size[0] == 0) || (B_size[1] == 0)) {
    int32_T loop_ub;
    i = C->size[0] * C->size[1];
    C->size[0] = A_size[0];
    C->size[1] = B_size[1];
    emxEnsureCapacity_real_T(sp, C, i, &f_emlrtRTEI);
    C_data = C->data;
    loop_ub = A_size[0] * B_size[1];
    for (i = 0; i < loop_ub; i++) {
      C_data[i] = 0.0;
    }
  } else {
    st.site = &j_emlrtRSI;
    b_st.site = &k_emlrtRSI;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)B_size[1];
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)A_size[0];
    i = C->size[0] * C->size[1];
    C->size[0] = A_size[0];
    C->size[1] = B_size[1];
    emxEnsureCapacity_real_T(&b_st, C, i, &d_emlrtRTEI);
    C_data = C->data;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B_data[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
}

void c_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B_data[], const int32_T B_size[2], real_T C_data[],
              int32_T C_size[2])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA1;
  char_T TRANSB1;
  if ((A_size[0] == 0) || (B_size[0] == 0)) {
    int32_T loop_ub;
    C_size[0] = A_size[0];
    C_size[1] = 6;
    loop_ub = A_size[0] * 6;
    if (loop_ub - 1 >= 0) {
      memset(&C_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    }
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)6;
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)B_size[0];
    ldc_t = (ptrdiff_t)A_size[0];
    C_size[0] = A_size[0];
    C_size[1] = 6;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B_data[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
}

int32_T d_mtimes(const real_T A_data[], const int32_T A_size[2],
                 const real_T B[6], real_T C_data[])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  int32_T C_size;
  char_T TRANSA1;
  char_T TRANSB1;
  if (A_size[0] == 0) {
    C_size = 0;
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)A_size[0];
    C_size = A_size[0];
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
  return C_size;
}

void e_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B[36], real_T C_data[], int32_T C_size[2])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA1;
  char_T TRANSB1;
  if (A_size[0] == 0) {
    C_size[0] = 0;
    C_size[1] = 6;
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)6;
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)A_size[0];
    C_size[0] = A_size[0];
    C_size[1] = 6;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
}

void f_mtimes(const real_T A_data[], const int32_T A_size[2],
              const real_T B_data[], const int32_T B_size[2], real_T C_data[],
              int32_T C_size[2])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA1;
  char_T TRANSB1;
  if ((A_size[0] == 0) || (B_size[0] == 0)) {
    int32_T loop_ub;
    C_size[0] = A_size[0];
    C_size[1] = B_size[0];
    loop_ub = A_size[0] * B_size[0];
    if (loop_ub - 1 >= 0) {
      memset(&C_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    }
  } else {
    TRANSB1 = 'T';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)B_size[0];
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)B_size[0];
    ldc_t = (ptrdiff_t)A_size[0];
    C_size[0] = A_size[0];
    C_size[1] = B_size[0];
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B_data[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
}

int32_T g_mtimes(const real_T A_data[], const int32_T A_size[2],
                 const real_T B[6], real_T C_data[])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  int32_T C_size;
  char_T TRANSA1;
  char_T TRANSB1;
  if ((A_size[0] == 0) || (A_size[1] == 0)) {
    int32_T loop_ub;
    loop_ub = A_size[0];
    C_size = A_size[0];
    if (loop_ub - 1 >= 0) {
      memset(&C_data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    }
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[0];
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)A_size[0];
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)A_size[0];
    C_size = A_size[0];
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
  return C_size;
}

void mtimes(const real_T A_data[], const int32_T A_size[2], const real_T B[36],
            real_T C_data[], int32_T C_size[2])
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA1;
  char_T TRANSB1;
  if (A_size[1] == 0) {
    C_size[0] = 0;
    C_size[1] = 6;
  } else {
    TRANSB1 = 'N';
    TRANSA1 = 'T';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)A_size[1];
    n_t = (ptrdiff_t)6;
    k_t = (ptrdiff_t)6;
    lda_t = (ptrdiff_t)6;
    ldb_t = (ptrdiff_t)6;
    ldc_t = (ptrdiff_t)A_size[1];
    C_size[0] = A_size[1];
    C_size[1] = 6;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B[0], &ldb_t, &beta1, &C_data[0], &ldc_t);
  }
}

/* End of code generation (mtimes.c) */
