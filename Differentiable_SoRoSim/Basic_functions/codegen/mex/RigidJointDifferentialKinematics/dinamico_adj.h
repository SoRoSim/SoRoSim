/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dinamico_adj.h
 *
 * Code generation for function 'dinamico_adj'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void dinamico_adj(const emlrtStack *sp, const real_T screw_data[],
                  int32_T screw_size, real_T adj[36]);

/* End of code generation (dinamico_adj.h) */
