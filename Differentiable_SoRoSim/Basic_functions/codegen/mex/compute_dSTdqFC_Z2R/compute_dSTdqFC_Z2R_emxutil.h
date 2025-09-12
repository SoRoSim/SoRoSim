/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_dSTdqFC_Z2R_emxutil.h
 *
 * Code generation for function 'compute_dSTdqFC_Z2R_emxutil'
 *
 */

#pragma once

/* Include files */
#include "compute_dSTdqFC_Z2R_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T *emxArray,
                              int32_T oldNumel,
                              const emlrtRTEInfo *srcLocation);

void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);

void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
                    const emlrtRTEInfo *srcLocation);

/* End of code generation (compute_dSTdqFC_Z2R_emxutil.h) */
